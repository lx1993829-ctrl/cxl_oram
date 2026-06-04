`timescale 1ns / 1ps
// =============================================================================
// bucket_meta_hbm.v - HBM-backed Bucket Metadata
// =============================================================================
// Replaces the on-chip BRAM bucket_meta.v with an HBM-backed AXI4 master,
// mirroring iv_slot_table.v / slot_r_table.v. Required for scale: at millions
// of slots the bucket directory (B x 124b) is hundreds of MB and cannot live
// on-chip. 32768 slots is only the tractable gem5 stand-in.
//
// For each bucket stores { slot_list[Z*SLOT_W-1:0], fill_count[FILL_W-1:0] }.
// ENTRY_W = Z*SLOT_W + FILL_W = 8*15 + 4 = 124 bits, packed into one 256-bit
// AXI beat (no packing across buckets): full-beat read/write, no WSTRB masking.
//
//   beat layout : { pad, slot_list[Z*SLOT_W-1:0], fill_count[FILL_W-1:0] }
//   beat_addr   : BUCKET_META_BASE + bucket * 32 = BASE + (bucket << 5)
//
// Functional interface (rd_*/wr_*) is IDENTICAL to bucket_meta.v so the FSM
// wiring is unchanged EXCEPT it must tolerate multi-cycle read latency. The FSM
// already waits on rd_valid in S_DDR_READ, so the only FSM change needed is to
// also gate the S_DDR_READ exit on rd_valid (done in flat_oram_gcm.v).
//
// Access model:
//   READ  : issued at S_POS_LOOKUP, OVERLAPS the bucket data read; FSM waits on
//           rd_valid (and axim_read_done) in S_DDR_READ. Blocking-with-valid.
//   WRITE : latched / asynchronous (like IVT). FSM pulses wr_en at S_EVICT; the
//           write drains while the bucket data burst runs. busy gates op
//           completion at S_DDR_WRITE.
// =============================================================================
`include "oram_params.vh"

module bucket_meta_hbm #(
    parameter B          = `ORAM_B,
    parameter Z          = `ORAM_Z,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter SLOT_W     = `SLOT_ID_W,
    parameter FILL_W     = `FILL_CNT_W,
    parameter POS_W      = `POS_IN_BKT_W,
    parameter ENTRY_W    = Z * SLOT_W + FILL_W,   // 124
    parameter AXI_DW     = `AXI_DATA_W,
    parameter AXI_AW     = `AXI_ADDR_W,
    parameter AXI_SW     = `AXI_STRB_W,
    parameter AXI_IDW    = `AXI_ID_W,
    parameter AXI_LENW   = `AXI_LEN_W,
    parameter BM_BASE    = `BUCKET_META_BASE,
    parameter BEAT_BYTES = AXI_DW / 8,            // 32
    parameter QDEPTH     = 2
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Functional read port (same names/widths as bucket_meta.v).
    // Result is multi-cycle: wait rd_valid.
    input  wire [BUCKET_W-1:0]  rd_bucket,
    input  wire                 rd_en,
    output reg  [Z*SLOT_W-1:0]  rd_slot_list,
    output reg  [FILL_W-1:0]    rd_fill_count,
    output reg                  rd_valid,

    // Functional write port (same names/widths) -- fire-and-forget; latched.
    input  wire [BUCKET_W-1:0]  wr_bucket,
    input  wire [Z*SLOT_W-1:0]  wr_slot_list,
    input  wire [FILL_W-1:0]    wr_fill_count,
    input  wire                 wr_en,

    // Busy + debug
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

    localparam AXI_SIZE  = 3'd5;        // 32 bytes per beat
    localparam AXI_BURST = 2'b01;       // INCR

    // Byte address: BM_BASE + bucket * 32 = BM_BASE + (bucket << 5).
    function [AXI_AW-1:0] beat_addr;
        input [BUCKET_W-1:0] bkt;
        begin
            beat_addr = BM_BASE +
                ({{(AXI_AW-BUCKET_W){1'b0}}, bkt} << $clog2(BEAT_BYTES));
        end
    endfunction

    // 2-deep write command latch.
    reg [BUCKET_W-1:0]    wq_bkt  [0:QDEPTH-1];
    reg [Z*SLOT_W-1:0]    wq_sl   [0:QDEPTH-1];
    reg [FILL_W-1:0]      wq_fill [0:QDEPTH-1];
    reg [$clog2(QDEPTH+1)-1:0] wq_count;
    reg [$clog2(QDEPTH)-1:0]   wq_head;
    reg [$clog2(QDEPTH)-1:0]   wq_tail;

    wire wq_empty = (wq_count == 0);
    wire wq_full  = (wq_count == QDEPTH);

    localparam S_IDLE   = 3'd0,
               S_RD_AR  = 3'd1,
               S_RD_R   = 3'd2,
               S_WR_AW  = 3'd3,
               S_WR_B   = 3'd4;

    reg [2:0] bm_state;

    assign busy      = (bm_state != S_IDLE) || !wq_empty || wr_en;
    assign dbg_state = bm_state;

    wire do_enq = wr_en && !wq_full;
    wire do_deq = (bm_state == S_WR_B) && m_axi_bvalid;

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bm_state      <= S_IDLE;
            rd_slot_list  <= {(Z*SLOT_W){1'b0}};
            rd_fill_count <= {FILL_W{1'b0}};
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
                wq_bkt[k]  <= {BUCKET_W{1'b0}};
                wq_sl[k]   <= {(Z*SLOT_W){1'b0}};
                wq_fill[k] <= {FILL_W{1'b0}};
            end
        end else begin
            rd_valid <= 1'b0;

            if (do_enq) begin
                wq_bkt[wq_tail]  <= wr_bucket;
                wq_sl[wq_tail]   <= wr_slot_list;
                wq_fill[wq_tail] <= wr_fill_count;
                wq_tail          <= (wq_tail == QDEPTH-1) ? 0 : wq_tail + 1;
                `ifndef SYNTHESIS
                $display("[BMETA] ENQ bkt=%0d fill=%0d tail=%0d count=%0d->%0d @%0t",
                         wr_bucket, wr_fill_count, wq_tail, wq_count,
                         (do_deq ? wq_count : wq_count+1), $time);
                `endif
            end
            `ifndef SYNTHESIS
            if (wr_en && wq_full)
                $display("[BMETA] WARN write dropped: queue full @%0t (raise QDEPTH)", $time);
            `endif

            case (bm_state)

            S_IDLE: begin
                if (rd_en) begin
                    m_axi_arid    <= {AXI_IDW{1'b0}};
                    m_axi_araddr  <= beat_addr(rd_bucket);
                    m_axi_arlen   <= {AXI_LENW{1'b0}};
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= AXI_BURST;
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b1;
                    bm_state      <= S_RD_AR;
                    `ifndef SYNTHESIS
                    $display("[BMETA] RD: bkt=%0d addr=0x%010h @%0t",
                             rd_bucket, beat_addr(rd_bucket), $time);
                    `endif
                end else if (!wq_empty) begin
                    m_axi_awid    <= {AXI_IDW{1'b0}};
                    m_axi_awaddr  <= beat_addr(wq_bkt[wq_head]);
                    m_axi_awlen   <= {AXI_LENW{1'b0}};
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= AXI_BURST;
                    m_axi_awvalid <= 1'b1;
                    // Full-beat: { pad, slot_list, fill_count }. Entire entry
                    // rewritten -> all strobes set, no masking.
                    m_axi_wdata   <= {{(AXI_DW-ENTRY_W){1'b0}},
                                      wq_sl[wq_head], wq_fill[wq_head]};
                    m_axi_wstrb   <= {AXI_SW{1'b1}};
                    m_axi_wlast   <= 1'b1;
                    m_axi_wvalid  <= 1'b1;
                    m_axi_bready  <= 1'b1;
                    bm_state      <= S_WR_AW;
                    `ifndef SYNTHESIS
                    $display("[BMETA] WR: bkt=%0d addr=0x%010h fill=%0d @%0t",
                             wq_bkt[wq_head], beat_addr(wq_bkt[wq_head]),
                             wq_fill[wq_head], $time);
                    `endif
                end
            end

            S_RD_AR: begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    bm_state      <= S_RD_R;
                    `ifndef SYNTHESIS
                    $display("[BMETA] AR accepted: addr=0x%010h -> await R @%0t",
                             m_axi_araddr, $time);
                    `endif
                end
            end

            S_RD_R: begin
                if (m_axi_rvalid) begin
                    m_axi_rready  <= 1'b0;
                    // Unpack: fill in low FILL_W bits, slot_list above it.
                    rd_fill_count <= m_axi_rdata[FILL_W-1:0];
                    rd_slot_list  <= m_axi_rdata[ENTRY_W-1:FILL_W];
                    rd_valid      <= 1'b1;
                    bm_state      <= S_IDLE;
                    `ifndef SYNTHESIS
                    $display("[BMETA] RD done: fill=%0d @%0t",
                             m_axi_rdata[FILL_W-1:0], $time);
                    `endif
                end
            end

            S_WR_AW: begin
                if (m_axi_awready) m_axi_awvalid <= 1'b0;
                if (m_axi_wready)  m_axi_wvalid  <= 1'b0;
                if (m_axi_wready)  m_axi_wlast   <= 1'b0;
                `ifndef SYNTHESIS
                if (m_axi_awvalid && m_axi_awready)
                    $display("[BMETA] AW accepted: addr=0x%010h @%0t", m_axi_awaddr, $time);
                if (m_axi_wvalid && m_axi_wready)
                    $display("[BMETA] W accepted (wlast=%b) @%0t", m_axi_wlast, $time);
                `endif
                if ((!m_axi_awvalid || m_axi_awready) &&
                    (!m_axi_wvalid  || m_axi_wready)) begin
                    `ifndef SYNTHESIS
                    $display("[BMETA] AW+W done -> S_WR_B (await BRESP) @%0t", $time);
                    `endif
                    bm_state <= S_WR_B;
                end
            end

            S_WR_B: begin
                m_axi_awvalid <= 1'b0;
                m_axi_wvalid  <= 1'b0;
                if (m_axi_bvalid) begin
                    m_axi_bready <= 1'b0;
                    wq_head   <= (wq_head == QDEPTH-1) ? 0 : wq_head + 1;
                    bm_state  <= S_IDLE;
                    `ifndef SYNTHESIS
                    $display("[BMETA] BRESP=%0d durable; deq head=%0d count=%0d->%0d @%0t",
                             m_axi_bresp, wq_head, wq_count, wq_count-1, $time);
                    if (m_axi_bresp != 2'b00)
                        $display("[BMETA] WARN non-OKAY BRESP=%0d @%0t", m_axi_bresp, $time);
                    `endif
                end
            end

            default: bm_state <= S_IDLE;
            endcase

            if (do_enq && !do_deq)      wq_count <= wq_count + 1;
            else if (!do_enq && do_deq) wq_count <= wq_count - 1;
        end
    end

    initial begin
        bm_state = S_IDLE;
        rd_valid = 1'b0;
        wq_count = 0;
        wq_head  = 0;
        wq_tail  = 0;
    end

endmodule
