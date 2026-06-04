// =============================================================================
// stash.v - ORAM Stash with CAM Search  (v4 - hybrid: tags on-chip, data in DDR)
// =============================================================================
// Architecture change from v3:
//   - Metadata (valid, slot_addr, target_bucket) stays in on-chip registers
//     for 1-cycle CAM search. This is the "tag" portion.
//   - Data memory moves to external DDR/HBM accessed via stash_axi_master.
//   - A 1-entry line buffer (128 beats x 256-bit = 4 KB) provides local
//     staging for the active stash entry being read/written.
//
// Data access protocol (managed by flat_oram_gcm FSM):
//   1. FSM issues ext_rd_req with entry index -> stash_axi_master burst-reads
//      4KB from DDR into line buffer
//   2. FSM reads/writes line buffer beat-by-beat (1-cycle latency, same as
//      old URAM interface)
//   3. FSM issues ext_wr_req -> stash_axi_master burst-writes line buffer
//      back to DDR
//
// The drd/dwr ports now access the LINE BUFFER, not external memory directly.
// =============================================================================
`include "oram_params.vh"

module stash #(
    parameter DEPTH      = `ORAM_STASH_DEPTH,
    parameter PTR_W      = `STASH_PTR_W,
    parameter SLOT_AW    = `SLOT_ADDR_W,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter AXI_DATA_W = `AXI_DATA_W,
    parameter BEATS      = `BEATS_PER_BLOCK,
    parameter BEAT_W     = `BEAT_CNT_W,
    parameter BEAT_AW    = 7                    // log2(BEATS) = log2(128) = 7
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Insert Port
    input  wire [SLOT_AW-1:0]       ins_slot_addr,
    input  wire [BUCKET_W-1:0]      ins_target_bucket,
    input  wire                     ins_en,
    output wire [PTR_W-1:0]         ins_idx,
    output wire                     ins_full,

    // Remap Port
    input  wire [PTR_W-1:0]         remap_idx,
    input  wire [BUCKET_W-1:0]      remap_new_bucket,
    input  wire                     remap_en,

    // Evict Port
    input  wire [PTR_W-1:0]         evict_idx,
    input  wire                     evict_en,

    // Valid-check port: combinational read of valid_bm for a given index.
    // Used by the main FSM to skip eviction of already-freed entries.
    input  wire [PTR_W-1:0]         valid_check_idx,
    output wire                     valid_check_out,

    // =========================================================================
    // Line Buffer Data Ports (replace old URAM data ports)
    // Same interface as before: 1-cycle latency for beat-by-beat access.
    // The FSM must first load the entry from DDR into the line buffer
    // via ext_rd_req before reading, and flush via ext_wr_req after writing.
    // =========================================================================
    // Data Write Port (writes to line buffer)
    input  wire [PTR_W-1:0]         dwr_entry,      // ignored (line buf is single-entry)
    input  wire [BEAT_W-1:0]        dwr_beat,
    input  wire [AXI_DATA_W-1:0]    dwr_data,
    input  wire                     dwr_en,

    // Data Read Port (reads from line buffer)
    input  wire [PTR_W-1:0]         drd_entry,      // ignored (line buf is single-entry)
    input  wire [BEAT_W-1:0]        drd_beat,
    input  wire                     drd_en,
    output wire [AXI_DATA_W-1:0]    drd_data,
    output wire                     drd_valid,

    // =========================================================================
    // External Memory Interface (to stash_axi_master)
    // =========================================================================
    // Request: load/store one 4KB entry between DDR and line buffer
    output reg                      ext_rd_req,
    output reg                      ext_wr_req,
    input  wire [PTR_W-1:0]         ext_entry,      // driven by flat_oram_gcm FSM
    input  wire                     ext_rd_done,
    input  wire                     ext_wr_done,
    input  wire                     ext_busy,

    // Line buffer write port (driven by stash_axi_master during ext_rd)
    input  wire [AXI_DATA_W-1:0]    ext_lb_wr_data,
    input  wire [BEAT_AW-1:0]       ext_lb_wr_addr,
    input  wire                     ext_lb_wr_en,

    // Line buffer read port (driven by stash_axi_master during ext_wr)
    input  wire [BEAT_AW-1:0]       ext_lb_rd_addr,
    input  wire                     ext_lb_rd_en,
    output wire [AXI_DATA_W-1:0]    ext_lb_rd_data,
    output wire                     ext_lb_rd_valid,

    // Status
    output wire [PTR_W:0]           occupancy
);

    // =========================================================================
    // Metadata storage
    // =========================================================================

    // =========================================================================
    // Unpacked aliases - combinational mirrors for TB compatibility
    // =========================================================================
    // The testbench reads  dut.u_stash.valid_r[i], dut.u_stash.slot_r[i], etc.
    // and uses force/release on individual elements.  These wires provide the
    // same hierarchical names while drawing from the packed vectors.
    //
    // NOTE on force/release: the TB forces valid_r[i] / slot_r[i] / target_r[i].
    // Since these are now continuous-assign outputs from the packed vector,
    // forcing the unpacked element does NOT change the packed vector.
    // The TB must force the packed bit(s) instead, OR we keep the old unpacked
    // regs and copy FROM them INTO the packed vector.
    //
    // SIMPLEST APPROACH for TB compat: keep unpacked REG arrays as primary
    // storage for force/release, but write them with CONSTANT-indexed generate
    // blocks to avoid the the synth sim variable-index bug.
    // =========================================================================

    // -------------------------------------------------------------------------
    // Actually: use generate-for to create per-entry always blocks.
    // Each block uses a CONSTANT genvar index, which the synth sim handles correctly.
    // This preserves unpacked reg arrays for full TB force/release compatibility.
    // -------------------------------------------------------------------------
    // -------------------------------------------------------------------------
    // Occupancy bitmap (replaces per-entry valid_r/slot_r/target_r register
    // file). slot_r moved to HBM (slot_r_table master, read by the FSM at
    // eviction); target_r was dead (only the removed CAM bkt-search read it).
    // Only the 1-bit-per-entry occupancy survives on-chip, because the free-
    // slot finder must read all entries at once. A packed bit-select write
    // valid_bm[idx] <= x is handled by BOTH the sim tool and the synth sim, so no
    // synth/sim generate split is needed for the bitmap.
    // -------------------------------------------------------------------------
    reg [DEPTH-1:0]  valid_bm;

    // Combinational valid check for eviction skip
    assign valid_check_out = valid_bm[valid_check_idx];

    // -------------------------------------------------------------------------
    // Line buffer: full stash-depth staging for simulation correctness.
    // With LB_ENTRIES = STASH_DEPTH, every stash index maps to a unique LB
    // slot, eliminating aliasing that corrupts data when two entries with
    // the same (idx mod Z) are staged concurrently.
    // For FPGA synthesis, reduce back to Z and add scheduling interlocks.
    localparam LB_ENTRIES = (1 << PTR_W); // Full stash depth: no LB aliasing (sim)
    localparam LB_ENT_W   = $clog2(LB_ENTRIES);  // PTR_W bits
    localparam LB_ADDR_W  = LB_ENT_W + BEAT_AW;  // staging-slot index + beat
    reg [AXI_DATA_W-1:0] line_buf [0:LB_ENTRIES * BEATS - 1];

    // Address computation: {entry[LB_ENT_W-1:0], beat}.
    // CRITICAL: the buffer is only LB_ENTRIES (Z) deep. The full 14-bit stash
    // entry index must be reduced to a Z-wide staging slot, or entries >= Z
    // index OUT OF BOUNDS (entry 8 -> slot 1024 in a 1024-deep array). The FSM
    // stages exactly one block at a time (loads/flushes are strictly
    // sequential and every port — dwr/drd/ext — uses the same entry value for
    // the staged block), so reducing to entry[LB_ENT_W-1:0] is collision-free.
    wire [LB_ADDR_W-1:0] dwr_lb_addr = {dwr_entry[LB_ENT_W-1:0], dwr_beat[BEAT_AW-1:0]};
    wire [LB_ADDR_W-1:0] drd_lb_addr = {drd_entry[LB_ENT_W-1:0], drd_beat[BEAT_AW-1:0]};

    // Muxed write port: ext_lb_wr_en during DDR burst load, dwr_en during FSM phase
    wire        lb_wr_en_mux   = ext_lb_wr_en | dwr_en;
    wire [LB_ADDR_W-1:0] lb_wr_addr_mux = ext_lb_wr_en
        ? {ext_entry[LB_ENT_W-1:0], ext_lb_wr_addr}   // stash_axi_master uses ext_entry
        : dwr_lb_addr;
    wire [AXI_DATA_W-1:0] lb_wr_data_mux = ext_lb_wr_en ? ext_lb_wr_data : dwr_data;

    // -------------------------------------------------------------------------
    // NOTE: The associative CAM search (find-by-slot_addr, find-by-target) was
    // REMOVED. It was permanently disabled (cam_*_en tied to 0, all cam_*
    // outputs unconnected at the instantiation in flat_oram_gcm.v); the hash
    // table (lease_token_table, in HBM) is authoritative for slot_addr ->
    // stash_idx. The eviction-inhibit mask existed only to support the dead
    // bkt-search and is removed with it. What remains live: indexed reads of
    // slot_r[idx] (idx from the HT), the storage arrays, and the free finder.
    // -------------------------------------------------------------------------
    integer j;

    // -------------------------------------------------------------------------
    // Free slot finder
    // -------------------------------------------------------------------------
    reg  [DEPTH-1:0] free_mask;
    always @(*) begin
        free_mask = ~valid_bm;
    end

    wire [PTR_W-1:0] free_enc;
    wire             free_hit;
    priority_enc #(.W(DEPTH), .OUT_W(PTR_W)) u_free_penc (
        .in(free_mask), .out(free_enc), .hit(free_hit)
    );
    assign ins_idx  = free_enc;
    assign ins_full = !free_hit;

    // -------------------------------------------------------------------------
    // Occupancy counter
    // -------------------------------------------------------------------------
    reg [PTR_W:0] occ_r;
    assign occupancy = occ_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            occ_r <= {(PTR_W+1){1'b0}};
        end else begin
            if (evict_en && !(ins_en && !ins_full))
                occ_r <= occ_r - 1'b1;
            else if (!evict_en && (ins_en && !ins_full))
                occ_r <= occ_r + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Power-on initialisation (simulation only)
    // -------------------------------------------------------------------------
    integer init_i;
    initial begin
        occ_r    = {(PTR_W+1){1'b0}};
        valid_bm = {DEPTH{1'b0}};
        for (init_i = 0; init_i < LB_ENTRIES * BEATS; init_i = init_i + 1)
            line_buf[init_i] = {AXI_DATA_W{1'b0}};
        ext_rd_req = 1'b0;
        ext_wr_req = 1'b0;
        // ext_entry is now an input wire — no reset needed
    end

    // =========================================================================
    // Occupancy bitmap write.
    //   Only the valid (occupancy) bit is tracked on-chip now. slot_addr writes
    //   are handled by the slot_r_table HBM master (driven from the FSM on
    //   insert); the old target field is gone. A packed bit-select write is
    //   handled by both sim and synth tools, so no generate split is needed.
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin : occ_bitmap_write
        if (!rst_n) begin
            valid_bm <= {DEPTH{1'b0}};
        end else begin
            if (evict_en) begin
                valid_bm[evict_idx] <= 1'b0;
                `ifndef SYNTHESIS
                $display("[STASH_BM] t=%0t EVICT entry=%0d -> free (occ %0d->%0d)",
                         $time, evict_idx, occ_r,
                         (ins_en && !ins_full) ? occ_r : occ_r-1);
                // Double-free detector: evicting an entry whose valid bit is
                // already 0 means the same entry was evicted twice (the old
                // non-terminating-chain bug re-emitted it). After the HT unlink
                // fix this should never fire.
                if (!valid_bm[evict_idx])
                    $display("[STASH_BM] WARN t=%0t DOUBLE-FREE: evict entry=%0d but valid_bm bit already 0 (chain re-emitted a dead entry?)",
                             $time, evict_idx);
                // Occupancy-underflow detector.
                if (occ_r == 0 && !(ins_en && !ins_full))
                    $display("[STASH_BM] WARN t=%0t OCC UNDERFLOW: evict with occ=0 -> wraps negative (more evicts than inserts)",
                             $time);
                `endif
            end
            if (ins_en && !ins_full) begin
                valid_bm[free_enc] <= 1'b1;
                `ifndef SYNTHESIS
                $display("[STASH_BM] t=%0t INSERT entry=%0d -> occupied (occ %0d->%0d, ins_full=%0b)",
                         $time, free_enc, occ_r,
                         evict_en ? occ_r : occ_r+1, ins_full);
                `endif
            end
            `ifndef SYNTHESIS
            if (ins_en && ins_full)
                $display("[STASH_BM] t=%0t WARN insert attempted but STASH FULL (occ=%0d depth=%0d)",
                         $time, occ_r, DEPTH);
            `endif
        end
    end

    // -------------------------------------------------------------------------
    // Line buffer Port A: write (muxed) + FSM read
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (lb_wr_en_mux)
            line_buf[lb_wr_addr_mux] <= lb_wr_data_mux;
        `ifndef SYNTHESIS
        if (lb_wr_en_mux)
            $display("[LB_WR] t=%0t %s addr=%0d (ext_entry=%0d dwr_entry=%0d beat=%0d) data[31:0]=0x%08h",
                     $time, ext_lb_wr_en ? "EXT(load)" : "DWR(client)",
                     lb_wr_addr_mux, ext_entry, dwr_entry,
                     ext_lb_wr_en ? ext_lb_wr_addr : dwr_beat[6:0],
                     lb_wr_data_mux[31:0]);
        `endif
    end

    // -------------------------------------------------------------------------
    // Line buffer Port A: FSM data read (COMBINATIONAL — matches URAM timing)
    // -------------------------------------------------------------------------
    // In simulation, line_buf is a register array — combinational reads are
    // safe and give the same 1-NB-boundary latency as FPGA URAM. The
    // registered version added an extra NB boundary that caused 128 stash
    // read stalls per 4KB encryption/decryption (384 vs 256 cycles).
    assign drd_valid = drd_en;
    assign drd_data  = drd_en ? line_buf[drd_lb_addr] : {AXI_DATA_W{1'b0}};

    `ifndef SYNTHESIS
    always @(posedge clk) begin
        if (drd_en)
            $display("[LB_RD_FSM] t=%0t addr=%0d (drd_entry=%0d beat=%0d) data[31:0]=0x%08h",
                     $time, drd_lb_addr, drd_entry, drd_beat[6:0], line_buf[drd_lb_addr][31:0]);
    end
    `endif

    // -------------------------------------------------------------------------
    // Line buffer Port B: ext_lb_rd (stash_axi_master reads for DDR writeback)
    // -------------------------------------------------------------------------
    reg [AXI_DATA_W-1:0] ext_lb_rd_data_r;
    reg                   ext_lb_rd_valid_r;

    always @(posedge clk) begin
        if (ext_lb_rd_en)
            ext_lb_rd_data_r <= line_buf[{ext_entry[LB_ENT_W-1:0], ext_lb_rd_addr}];
        `ifndef SYNTHESIS
        if (ext_lb_rd_en)
            $display("[LB_RD_EXT] t=%0t addr=%0d (ext_entry=%0d beat=%0d) data[31:0]=0x%08h -> to HBM flush",
                     $time, {ext_entry[LB_ENT_W-1:0], ext_lb_rd_addr}, ext_entry, ext_lb_rd_addr,
                     line_buf[{ext_entry[LB_ENT_W-1:0], ext_lb_rd_addr}][31:0]);
        `endif
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ext_lb_rd_valid_r <= 1'b0;
        else
            ext_lb_rd_valid_r <= ext_lb_rd_en;
    end

    assign ext_lb_rd_data  = ext_lb_rd_data_r;
    assign ext_lb_rd_valid = ext_lb_rd_valid_r;

    // -------------------------------------------------------------------------
    // ext_rd_req / ext_wr_req / ext_entry are driven by the flat_oram_gcm FSM
    // and stash_axi_master consumes them. They're declared here for port
    // connectivity but their logic lives in flat_oram_gcm.v.
    // -------------------------------------------------------------------------

endmodule


// =============================================================================
// priority_enc - Parameterized Priority Encoder (LSB wins)
// =============================================================================
module priority_enc #(
    parameter W     = 128,
    parameter OUT_W = 7
)(
    input  wire [W-1:0]     in,
    output reg  [OUT_W-1:0] out,
    output wire             hit
);
    assign hit = |in;
    integer i;
    always @(*) begin
        out = {OUT_W{1'b0}};
        for (i = W-1; i >= 0; i = i - 1)
            if (in[i]) out = i[OUT_W-1:0];
    end
endmodule