`timescale 1ns / 1ps
// =============================================================================
// slot_r_table.v - HBM-backed Per-Stash-Entry Slot-Address Storage
// =============================================================================
// Replaces the on-chip slot_r[] register array (formerly inside stash.v) with
// an HBM-backed AXI4 master, mirroring iv_slot_table.v / pos_map.v.
//
// Stores, for each stash ENTRY (0..DEPTH-1), the slot address of the block that
// currently occupies that entry. One 256-bit AXI beat per entry (no packing):
// full-beat read/write, so writes need NO WSTRB masking.
//
//   beat layout : { pad[223:0], slot_addr[31:0] }   (256 bits)
//   beat_addr   : SLOT_R_BASE + entry_idx * 32 = SLOT_R_BASE + (entry_idx << 5)
//
// Indexing is a DIRECT stash entry index (PTR_W bits), unlike the IVT table
// which derives a physical index from bucket*Z+pos. The free-slot finder and
// HT supply these indices (free_enc on insert, ht_evict_list[i] on eviction).
//
// Access model (same as IVT, decided by FSM safety analysis):
//   READ  : blocking. The FSM stalls in a wait-state on rd_valid. The evicted
//           slot address is needed before the encrypt path can proceed, so the
//           read is issued and waited on.
//   WRITE : latched / asynchronous. The FSM pulses wr_en (fire-and-forget) on
//           insert; this master drains the write to HBM on its own schedule.
//           A 2-deep command latch absorbs back-to-back inserts.
//
// SAFETY (async writes bug-free by construction):
//   - busy stays asserted from acceptance of a write until its B-response.
//   - The main FSM gates op completion (S_DDR_WRITE -> S_IDLE) on !busy of all
//     metadata masters (IVT, slot_r, bucket_meta), so no subsequent op begins
//     and reads slot_r until every write of this op is durable in HBM.
//   - If a read and queued writes are both pending, the READ is serviced first
//     (it is on the critical path and the FSM is blocked waiting on it).
// =============================================================================
`include "oram_params.vh"

module slot_r_table #(
    parameter DEPTH       = `ORAM_STASH_DEPTH,
    parameter PTR_W       = `STASH_PTR_W,
    parameter SLOT_AW     = `SLOT_ADDR_W,
    parameter BUCKET_W    = `BUCKET_ID_W,
    parameter AXI_DW      = `AXI_DATA_W,
    parameter AXI_AW      = `AXI_ADDR_W,
    parameter AXI_SW      = `AXI_STRB_W,
    parameter AXI_IDW     = `AXI_ID_W,
    parameter AXI_LENW    = `AXI_LEN_W,
    parameter SLOTR_BASE_P = `SLOT_R_BASE,
    parameter BEAT_BYTES  = AXI_DW / 8,          // 32
    parameter QDEPTH      = 2                     // write command latch depth
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Read port: direct stash-entry index. Result is multi-cycle: wait rd_valid.
    input  wire [PTR_W-1:0]     rd_idx,
    input  wire                 rd_en,
    output reg  [SLOT_AW-1:0]   rd_slot_addr,
    output reg  [BUCKET_W-1:0]  rd_bucket,       // bucket co-stored with slot addr
    output reg                  rd_valid,

    // Write port: direct stash-entry index -- fire-and-forget; latched.
    input  wire [PTR_W-1:0]     wr_idx,
    input  wire [SLOT_AW-1:0]   wr_slot_addr,
    input  wire [BUCKET_W-1:0]  wr_bucket,       // bucket co-stored with slot addr
    input  wire                 wr_en,

    // Busy: high whenever a read is in flight OR any write is queued/draining.
    output wire                 busy,
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

    // Byte address: SLOT_R_BASE + entry_idx * 32 = SLOT_R_BASE + (idx << 5).
    // idx is PTR_W bits; widen to AXI_AW before the shift (no 32-bit promote).
    function [AXI_AW-1:0] beat_addr;
        input [PTR_W-1:0] idx;
        begin
            beat_addr = SLOTR_BASE_P +
                ({{(AXI_AW-PTR_W){1'b0}}, idx} << $clog2(BEAT_BYTES));
        end
    endfunction

    // =========================================================================
    // 2-deep write command latch (FIFO). FSM pulses wr_en; entries drain async.
    // =========================================================================
    reg [PTR_W-1:0]   wq_idx  [0:QDEPTH-1];
    reg [SLOT_AW-1:0] wq_slot [0:QDEPTH-1];
    reg [BUCKET_W-1:0] wq_bkt [0:QDEPTH-1];
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

    reg [2:0] sr_state;

    assign busy      = (sr_state != S_IDLE) || !wq_empty || wr_en;
    assign dbg_state = sr_state;

    wire do_enq = wr_en && !wq_full;
    wire do_deq = (sr_state == S_WR_B) && m_axi_bvalid;

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sr_state      <= S_IDLE;
            rd_slot_addr  <= {SLOT_AW{1'b0}};
            rd_bucket     <= {BUCKET_W{1'b0}};
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
                wq_idx[k]  <= {PTR_W{1'b0}};
                wq_slot[k] <= {SLOT_AW{1'b0}};
                wq_bkt[k]  <= {BUCKET_W{1'b0}};
            end
        end else begin
            rd_valid <= 1'b0;

            // Write-enqueue: independent of the AXI FSM. Fire-and-forget.
            if (do_enq) begin
                wq_idx[wq_tail]  <= wr_idx;
                wq_slot[wq_tail] <= wr_slot_addr;
                wq_bkt[wq_tail]  <= wr_bucket;
                wq_tail          <= (wq_tail == QDEPTH-1) ? 0 : wq_tail + 1;
                `ifndef SYNTHESIS
                $display("[SLOTR] ENQ idx=%0d slot=0x%08h bkt=%0d tail=%0d count=%0d->%0d @%0t",
                         wr_idx, wr_slot_addr, wr_bucket, wq_tail, wq_count,
                         (do_deq ? wq_count : wq_count+1), $time);
                `endif
            end
            `ifndef SYNTHESIS
            if (wr_en && wq_full)
                $display("[SLOTR] WARN write dropped: queue full @%0t (raise QDEPTH)", $time);
            `endif

            case (sr_state)

            S_IDLE: begin
                if (rd_en) begin
                    // READ priority -- FSM is blocked waiting on it.
                    m_axi_arid    <= {AXI_IDW{1'b0}};
                    m_axi_araddr  <= beat_addr(rd_idx);
                    m_axi_arlen   <= {AXI_LENW{1'b0}};
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= AXI_BURST;
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b1;
                    sr_state      <= S_RD_AR;
                    `ifndef SYNTHESIS
                    $display("[SLOTR] RD: idx=%0d addr=0x%010h @%0t",
                             rd_idx, beat_addr(rd_idx), $time);
                    `endif
                end else if (!wq_empty) begin
                    // Drain head of write queue.
                    m_axi_awid    <= {AXI_IDW{1'b0}};
                    m_axi_awaddr  <= beat_addr(wq_idx[wq_head]);
                    m_axi_awlen   <= {AXI_LENW{1'b0}};
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= AXI_BURST;
                    m_axi_awvalid <= 1'b1;
                    // Full-beat write: { pad, bucket[BUCKET_W-1:0], slot_addr[31:0] }.
                    m_axi_wdata   <= {{(AXI_DW-SLOT_AW-BUCKET_W){1'b0}}, wq_bkt[wq_head], wq_slot[wq_head]};
                    m_axi_wstrb   <= {AXI_SW{1'b1}};
                    m_axi_wlast   <= 1'b1;
                    m_axi_wvalid  <= 1'b1;
                    m_axi_bready  <= 1'b1;
                    sr_state      <= S_WR_AW;
                    `ifndef SYNTHESIS
                    $display("[SLOTR] WR: idx=%0d addr=0x%010h slot=0x%08h @%0t",
                             wq_idx[wq_head], beat_addr(wq_idx[wq_head]),
                             wq_slot[wq_head], $time);
                    `endif
                end
            end

            // READ
            S_RD_AR: begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    sr_state      <= S_RD_R;
                    `ifndef SYNTHESIS
                    $display("[SLOTR] AR accepted: addr=0x%010h -> await R @%0t",
                             m_axi_araddr, $time);
                    `endif
                end
            end

            S_RD_R: begin
                if (m_axi_rvalid) begin
                    m_axi_rready <= 1'b0;
                    rd_slot_addr <= m_axi_rdata[SLOT_AW-1:0];
                    rd_bucket    <= m_axi_rdata[SLOT_AW +: BUCKET_W];
                    rd_valid     <= 1'b1;
                    sr_state     <= S_IDLE;
                    `ifndef SYNTHESIS
                    $display("[SLOTR] RD done: slot=0x%08h bkt=%0d @%0t",
                             m_axi_rdata[SLOT_AW-1:0], m_axi_rdata[SLOT_AW +: BUCKET_W], $time);
                    `endif
                end
            end

            // WRITE
            S_WR_AW: begin
                if (m_axi_awready) m_axi_awvalid <= 1'b0;
                if (m_axi_wready)  m_axi_wvalid  <= 1'b0;
                if (m_axi_wready)  m_axi_wlast   <= 1'b0;
                `ifndef SYNTHESIS
                if (m_axi_awvalid && m_axi_awready)
                    $display("[SLOTR] AW accepted: addr=0x%010h @%0t", m_axi_awaddr, $time);
                if (m_axi_wvalid && m_axi_wready)
                    $display("[SLOTR] W accepted (wlast=%b) @%0t", m_axi_wlast, $time);
                `endif
                if ((!m_axi_awvalid || m_axi_awready) &&
                    (!m_axi_wvalid  || m_axi_wready)) begin
                    `ifndef SYNTHESIS
                    $display("[SLOTR] AW+W done -> S_WR_B (await BRESP) @%0t", $time);
                    `endif
                    sr_state <= S_WR_B;
                end
            end

            S_WR_B: begin
                m_axi_awvalid <= 1'b0;
                m_axi_wvalid  <= 1'b0;
                if (m_axi_bvalid) begin
                    m_axi_bready <= 1'b0;
                    wq_head   <= (wq_head == QDEPTH-1) ? 0 : wq_head + 1;
                    sr_state  <= S_IDLE;
                    `ifndef SYNTHESIS
                    $display("[SLOTR] BRESP=%0d durable; deq head=%0d count=%0d->%0d @%0t",
                             m_axi_bresp, wq_head, wq_count, wq_count-1, $time);
                    if (m_axi_bresp != 2'b00)
                        $display("[SLOTR] WARN non-OKAY BRESP=%0d @%0t", m_axi_bresp, $time);
                    `endif
                end
            end

            default: sr_state <= S_IDLE;
            endcase

            if (do_enq && !do_deq)      wq_count <= wq_count + 1;
            else if (!do_enq && do_deq) wq_count <= wq_count - 1;
        end
    end

    initial begin
        sr_state = S_IDLE;
        rd_valid = 1'b0;
        wq_count = 0;
        wq_head  = 0;
        wq_tail  = 0;
    end

endmodule