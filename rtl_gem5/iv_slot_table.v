`timescale 1ns / 1ps
// =============================================================================
// iv_slot_table.v - v2 HBM-backed Per-Slot IV and TAG Storage
// =============================================================================
// Replaces the v1 on-chip BRAM table with an HBM-backed AXI4 master, mirroring
// the structure of pos_map.v v2.
//
// Stores the AES-GCM IV (96b) and authentication TAG (128b) for each encrypted
// slot. One 256-bit AXI beat per slot (no packing): full-beat read/write, so
// writes need NO WSTRB masking (the entire entry is rewritten every time).
//
//   beat layout : { pad[31:0], tag[127:0], iv[95:0] }   (256 bits)
//   beat_addr   : IVT_BASE + slot_idx * 32   (slot_idx = slot_addr[IDX_W+11:12])
//
// Functional interface is identical to v1 (rd_*/wr_*) so the FSM read path is
// unchanged EXCEPT it must tolerate multi-cycle latency (gate issue on !busy,
// wait for rd_valid). A new `busy` output exposes in-flight state.
//
// Access model (decided after FSM safety analysis):
//   READ  : blocking. FSM stalls in S_EXT_IV_RD on rd_valid. Result is on the
//           decrypt critical path with no overlappable work, so it is issued
//           through the FSM and waited on, exactly like the v1 read.
//   WRITE : latched / asynchronous. The main FSM pulses wr_en (fire-and-forget)
//           at the *_ENC_TAG states; this master drains the write to HBM on its
//           own schedule while the FSM proceeds into eviction / bucket write-
//           back. A 2-deep command latch absorbs back-to-back eviction-loop
//           writes so the FSM never stalls on a write.
//
// SAFETY (why async writes are bug-free here, by construction not by timing):
//   - No intra-op read-after-write: the only IVT read (S_EXT_IV_RD, req_slot)
//     precedes all writes within an op; eviction encrypts use a fresh IV seed
//     and never read back what was just written.
//   - busy stays asserted from acceptance of a write until its B-response
//     returns (state != S_IDLE while draining, and the queue is non-empty).
//   - The main FSM gates op completion (S_DDR_WRITE -> S_IDLE) on !busy, so no
//     operation can report done -- and therefore no subsequent op can begin and
//     read IV/TAG -- until every write of this op is durable in HBM.
//
// If both a read request and queued writes are pending, the READ is serviced
// first (it is on the critical path and the FSM is blocked waiting on it).
// =============================================================================
`include "oram_params.vh"

module iv_slot_table #(
    parameter N        = `ORAM_N,
    parameter BUCKET_W = `BUCKET_ID_W,
    parameter Z        = `ORAM_Z,
    parameter POS_W    = `POS_IN_BKT_W,
    parameter IV_W     = 96,
    parameter TAG_W    = 128,
    parameter AXI_DW   = `AXI_DATA_W,
    parameter AXI_AW   = `AXI_ADDR_W,
    parameter AXI_SW   = `AXI_STRB_W,
    parameter AXI_IDW  = `AXI_ID_W,
    parameter AXI_LENW = `AXI_LEN_W,
    parameter IVT_BASE   = `IVT_BASE,
    parameter PHYS_SLOTS = `IVT_PHYS_SLOTS,     // 65536 (>= numBuckets*Z)
    parameter PIDX_W     = $clog2(PHYS_SLOTS),  // 16-bit physical slot index
    parameter ENTRY_W    = IV_W + TAG_W,        // 224 bits
    parameter BEAT_BYTES = AXI_DW / 8,          // 32
    parameter QDEPTH     = 2                     // write command latch depth
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Read port: PHYSICAL addressing. Entry = bucket_id * Z + pos_in_bucket.
    // Result is multi-cycle: wait rd_valid.
    input  wire [BUCKET_W-1:0]  rd_bucket,
    input  wire [POS_W-1:0]     rd_pos,
    input  wire                 rd_en,
    output reg  [IV_W-1:0]      rd_iv,
    output reg  [TAG_W-1:0]     rd_tag,
    output reg                  rd_valid,

    // Write port: PHYSICAL addressing -- fire-and-forget; latched internally.
    input  wire [BUCKET_W-1:0]  wr_bucket,
    input  wire [POS_W-1:0]     wr_pos,
    input  wire [IV_W-1:0]      wr_iv,
    input  wire [TAG_W-1:0]     wr_tag,
    input  wire                 wr_en,

    // Busy: high whenever a read is in flight OR any write is queued/draining.
    // Main FSM folds this into the op-completion gate.
    output wire                 busy,
    // Debug
    output wire [2:0]           dbg_state,

    // Full AXI4 Master Interface -> HBM
    output reg  [AXI_IDW-1:0]   m_axi_arid,
    output reg  [AXI_AW-1:0]    m_axi_araddr,
    output reg  [AXI_LENW-1:0]  m_axi_arlen,
    output reg  [2:0]           m_axi_arsize,
    output reg  [1:0]           m_axi_arburst,
    output reg                  m_axi_arvalid,
    input  wire                 m_axi_arready,

    input  wire [AXI_IDW-1:0]   m_axi_rid,
    input  wire [AXI_DW-1:0]    m_axi_rdata,
    input  wire [1:0]           m_axi_rresp,
    input  wire                 m_axi_rlast,
    input  wire                 m_axi_rvalid,
    output reg                  m_axi_rready,

    output reg  [AXI_IDW-1:0]   m_axi_awid,
    output reg  [AXI_AW-1:0]    m_axi_awaddr,
    output reg  [AXI_LENW-1:0]  m_axi_awlen,
    output reg  [2:0]           m_axi_awsize,
    output reg  [1:0]           m_axi_awburst,
    output reg                  m_axi_awvalid,
    input  wire                 m_axi_awready,

    output reg  [AXI_DW-1:0]    m_axi_wdata,
    output reg  [AXI_SW-1:0]    m_axi_wstrb,
    output reg                  m_axi_wlast,
    output reg                  m_axi_wvalid,
    input  wire                 m_axi_wready,

    input  wire [AXI_IDW-1:0]   m_axi_bid,
    input  wire [1:0]           m_axi_bresp,
    input  wire                 m_axi_bvalid,
    output reg                  m_axi_bready
);

    // =========================================================================
    // Constants
    // =========================================================================
    localparam AXI_SIZE  = 3'd5;        // 32 bytes per beat
    localparam AXI_BURST = 2'b01;       // INCR

    // Physical slot index = bucket_id * Z + pos_in_bucket.
    // HBM byte address: IVT_BASE + phys_idx * 32 = IVT_BASE + (phys_idx << 5).
    // Z is a power of two (ORAM_Z), so bkt*Z == bkt << log2(Z). Using a shift
    // keeps the expression PIDX_W-wide with no 32-bit integer intermediate
    // (a bare "* Z" would promote to 32 bits and trigger WIDTHTRUNC). This is
    // width-exact because PIDX_W == BUCKET_W + log2(Z) by construction.
    localparam LOG2_Z = $clog2(Z);
    function [PIDX_W-1:0] phys_idx;
        input [BUCKET_W-1:0] bkt;
        input [POS_W-1:0]    pos;
        reg   [PIDX_W-1:0]   bkt_ext;
        begin
            bkt_ext  = {{(PIDX_W-BUCKET_W){1'b0}}, bkt};
            phys_idx = (bkt_ext << LOG2_Z) |
                       {{(PIDX_W-POS_W){1'b0}}, pos};
        end
    endfunction

    function [AXI_AW-1:0] beat_addr;
        input [BUCKET_W-1:0] bkt;
        input [POS_W-1:0]    pos;
        reg   [PIDX_W-1:0]   idx;
        begin
            idx = phys_idx(bkt, pos);
            beat_addr = IVT_BASE +
                ({{(AXI_AW-PIDX_W){1'b0}}, idx} << $clog2(BEAT_BYTES));
        end
    endfunction

    // =========================================================================
    // 2-deep write command latch (FIFO).
    // The main FSM pulses wr_en; entries drain to HBM asynchronously.
    // QDEPTH=2 covers back-to-back eviction-loop writes without ever stalling
    // the FSM. Overflow would only occur if a 3rd write arrived before the
    // 1st drained -- impossible at the FSM's write cadence (one per encrypt,
    // and each encrypt is many cycles > one HBM write round-trip).
    // Stores PHYSICAL coordinates (bucket, pos), not a slot address.
    // =========================================================================
    reg [BUCKET_W-1:0] wq_bkt [0:QDEPTH-1];
    reg [POS_W-1:0]    wq_pos [0:QDEPTH-1];
    reg [IV_W-1:0]     wq_iv  [0:QDEPTH-1];
    reg [TAG_W-1:0]    wq_tag [0:QDEPTH-1];
    reg [$clog2(QDEPTH+1)-1:0] wq_count;     // 0..QDEPTH
    reg [$clog2(QDEPTH)-1:0]   wq_head;      // next entry to drain
    reg [$clog2(QDEPTH)-1:0]   wq_tail;      // next free slot

    wire wq_empty = (wq_count == 0);
    wire wq_full  = (wq_count == QDEPTH);

    // =========================================================================
    // FSM
    // =========================================================================
    localparam S_IDLE   = 3'd0,
               S_RD_AR  = 3'd1,    // read: hold arvalid, deassert on arready
               S_RD_R   = 3'd2,    // read: wait R data
               S_WR_AW  = 3'd3,    // write: hold awvalid+wvalid, deassert on ready
               S_WR_B   = 3'd4;    // write: wait BRESP

    reg [2:0] ivt_state;

    // busy: a read in flight OR any write queued/draining OR a write arriving
    // this cycle. Held through BRESP because ivt_state stays out of S_IDLE
    // until bvalid, and wq stays non-empty until the draining write completes.
    assign busy      = (ivt_state != S_IDLE) || !wq_empty || wr_en;
    assign dbg_state = ivt_state;

    // Per-cycle enqueue/dequeue events (captured from state BEFORE the case
    // block updates ivt_state, so the count update is race-free).
    wire do_enq = wr_en && !wq_full;
    wire do_deq = (ivt_state == S_WR_B) && m_axi_bvalid;

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ivt_state     <= S_IDLE;
            rd_iv         <= {IV_W{1'b0}};
            rd_tag        <= {TAG_W{1'b0}};
            rd_valid      <= 1'b0;
            wq_count      <= 0;
            wq_head       <= 0;
            wq_tail       <= 0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_wlast   <= 1'b0;
            m_axi_bready  <= 1'b0;
            m_axi_wstrb   <= {AXI_SW{1'b0}};
            m_axi_arid    <= {AXI_IDW{1'b0}};
            m_axi_awid    <= {AXI_IDW{1'b0}};
            for (k = 0; k < QDEPTH; k = k + 1) begin
                wq_bkt[k] <= {BUCKET_W{1'b0}};
                wq_pos[k] <= {POS_W{1'b0}};
                wq_iv[k]  <= {IV_W{1'b0}};
                wq_tag[k] <= {TAG_W{1'b0}};
            end
        end else begin
            rd_valid <= 1'b0;

            // -----------------------------------------------------------------
            // Write-enqueue: independent of the AXI FSM so the FSM never has to
            // be in S_IDLE to accept a write. Fire-and-forget from the caller.
            // -----------------------------------------------------------------
            if (do_enq) begin
                wq_bkt[wq_tail] <= wr_bucket;
                wq_pos[wq_tail] <= wr_pos;
                wq_iv[wq_tail]  <= wr_iv;
                wq_tag[wq_tail] <= wr_tag;
                wq_tail         <= (wq_tail == QDEPTH-1) ? 0 : wq_tail + 1;
                `ifndef SYNTHESIS
                $display("[IVT] ENQ bkt=%0d pos=%0d (phys=%0d) iv=0x%024h tag=0x%032h tail=%0d count=%0d->%0d @%0t",
                         wr_bucket, wr_pos, phys_idx(wr_bucket, wr_pos),
                         wr_iv, wr_tag, wq_tail, wq_count,
                         (do_deq ? wq_count : wq_count+1), $time);
                `endif
            end
            `ifndef SYNTHESIS
            if (wr_en && wq_full)
                $display("[IVT] WARN write dropped: queue full @%0t (raise QDEPTH)", $time);
            `endif

            case (ivt_state)

            // =============================================================
            S_IDLE: begin
                if (rd_en) begin
                    // READ has priority -- FSM is blocked waiting on it.
                    m_axi_arid    <= {AXI_IDW{1'b0}};
                    m_axi_araddr  <= beat_addr(rd_bucket, rd_pos);
                    m_axi_arlen   <= {AXI_LENW{1'b0}};
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= AXI_BURST;
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b1;
                    ivt_state     <= S_RD_AR;
                    `ifndef SYNTHESIS
                    $display("[IVT] RD: bkt=%0d pos=%0d (phys=%0d) addr=0x%010h @%0t",
                             rd_bucket, rd_pos, phys_idx(rd_bucket, rd_pos),
                             beat_addr(rd_bucket, rd_pos), $time);
                    `endif
                end else if (!wq_empty) begin
                    // Drain head of write queue.
                    m_axi_awid    <= {AXI_IDW{1'b0}};
                    m_axi_awaddr  <= beat_addr(wq_bkt[wq_head], wq_pos[wq_head]);
                    m_axi_awlen   <= {AXI_LENW{1'b0}};
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= AXI_BURST;
                    m_axi_awvalid <= 1'b1;
                    // Full-beat write: { pad[31:0], tag[127:0], iv[95:0] }.
                    // Entire entry rewritten -> all strobes set, no masking.
                    m_axi_wdata   <= {{(AXI_DW-ENTRY_W){1'b0}},
                                      wq_tag[wq_head], wq_iv[wq_head]};
                    m_axi_wstrb   <= {AXI_SW{1'b1}};
                    m_axi_wlast   <= 1'b1;
                    m_axi_wvalid  <= 1'b1;
                    m_axi_bready  <= 1'b1;
                    ivt_state     <= S_WR_AW;
                    `ifndef SYNTHESIS
                    $display("[IVT] WR: bkt=%0d pos=%0d (phys=%0d) addr=0x%010h iv=0x%024h tag=0x%032h @%0t",
                             wq_bkt[wq_head], wq_pos[wq_head],
                             phys_idx(wq_bkt[wq_head], wq_pos[wq_head]),
                             beat_addr(wq_bkt[wq_head], wq_pos[wq_head]),
                             wq_iv[wq_head], wq_tag[wq_head], $time);
                    `endif
                end
            end

            // =============================================================
            // READ
            // =============================================================
            S_RD_AR: begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    ivt_state     <= S_RD_R;
                    `ifndef SYNTHESIS
                    $display("[IVT] AR accepted: addr=0x%010h -> await R @%0t",
                             m_axi_araddr, $time);
                    `endif
                end
            end

            S_RD_R: begin
                if (m_axi_rvalid) begin
                    m_axi_rready <= 1'b0;
                    rd_iv        <= m_axi_rdata[IV_W-1:0];
                    rd_tag       <= m_axi_rdata[ENTRY_W-1:IV_W];
                    rd_valid     <= 1'b1;
                    ivt_state    <= S_IDLE;
                    `ifndef SYNTHESIS
                    $display("[IVT] RD done: iv=0x%024h tag=0x%032h @%0t",
                             m_axi_rdata[IV_W-1:0],
                             m_axi_rdata[ENTRY_W-1:IV_W], $time);
                    `endif
                end
            end

            // =============================================================
            // WRITE
            // =============================================================
            S_WR_AW: begin
                if (m_axi_awready) m_axi_awvalid <= 1'b0;
                if (m_axi_wready)  m_axi_wvalid  <= 1'b0;
                if (m_axi_wready)  m_axi_wlast   <= 1'b0;
                `ifndef SYNTHESIS
                if (m_axi_awvalid && m_axi_awready)
                    $display("[IVT] AW accepted: addr=0x%010h @%0t", m_axi_awaddr, $time);
                if (m_axi_wvalid && m_axi_wready)
                    $display("[IVT] W accepted (wlast=%b) @%0t", m_axi_wlast, $time);
                `endif
                if ((!m_axi_awvalid || m_axi_awready) &&
                    (!m_axi_wvalid  || m_axi_wready)) begin
                    `ifndef SYNTHESIS
                    $display("[IVT] AW+W done -> S_WR_B (await BRESP) @%0t", $time);
                    `endif
                    ivt_state <= S_WR_B;
                end
            end

            S_WR_B: begin
                m_axi_awvalid <= 1'b0;
                m_axi_wvalid  <= 1'b0;
                if (m_axi_bvalid) begin
                    m_axi_bready <= 1'b0;
                    // Dequeue head now that the write is durable (BRESP in).
                    wq_head   <= (wq_head == QDEPTH-1) ? 0 : wq_head + 1;
                    ivt_state <= S_IDLE;
                    `ifndef SYNTHESIS
                    $display("[IVT] BRESP=%0d durable; deq head=%0d count=%0d->%0d @%0t",
                             m_axi_bresp, wq_head, wq_count, wq_count-1, $time);
                    if (m_axi_bresp != 2'b00)
                        $display("[IVT] WARN non-OKAY BRESP=%0d @%0t", m_axi_bresp, $time);
                    `endif
                end
            end

            default: ivt_state <= S_IDLE;
            endcase

            // -----------------------------------------------------------------
            // wq_count: single race-free update combining this cycle's enqueue
            // (do_enq) and dequeue (do_deq).
            // -----------------------------------------------------------------
            if (do_enq && !do_deq)      wq_count <= wq_count + 1;
            else if (!do_enq && do_deq) wq_count <= wq_count - 1;
            // both or neither -> unchanged
        end
    end

    initial begin
        ivt_state = S_IDLE;
        rd_valid  = 1'b0;
        wq_count  = 0;
        wq_head   = 0;
        wq_tail   = 0;
    end

endmodule