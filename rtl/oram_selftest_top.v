// =============================================================================
// oram_selftest_top.v - Board-level self-test wrapper
// =============================================================================
// Sequences:  1) ddr_pattern_init fills DDR with test data
//             2) oram_test_driver runs T1-T5
//             3) Results on LEDs / ILA
//
// Connect:
//   clk       <- DDR controller ui_clk (or your system clock)
//   rst_n     <- DDR controller init_calib_complete (active-high -> invert)
//   btn_start <- pushbutton (active-high, active for >=1 cycle)
//   busy/done/pass/fail -> status outputs (LEDs or ILA)
//   oram_fail_code[7:0] -> per-test-group failure bitmap
//   oram_test_num[3:0]  -> current test number
//   m_axi_*   -> DDR controller AXI slave port
// =============================================================================

`include "oram_params.vh"

module oram_selftest_top #(
    parameter AXI_DW   = `AXI_DATA_W,
    parameter AXI_AW   = `AXI_ADDR_W,
    parameter AXI_SW   = `AXI_STRB_W,
    parameter AXI_IDW  = `AXI_ID_W,
    parameter AXI_LENW = `AXI_LEN_W
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                btn_start,

    output wire                busy,
    output wire                done,
    output wire                pass,
    output wire                fail,
    output wire [7:0]          oram_fail_code,
    output wire [3:0]          oram_test_num,

    // Debug passthrough
    output wire [3:0]          dbg_state,
    output wire                dbg_client_done,
    output wire                dbg_client_req,
    output wire [`BUCKET_ID_W-1:0] dbg_req_b,
    output wire [`BUCKET_ID_W-1:0] dbg_req_b_new,
    output wire                dbg_found_in_bucket,
    output wire                dbg_found_in_stash,
    output wire                dbg_same_bucket,
    output wire                dbg_err_stash_ovf,
    output wire [`STASH_PTR_W:0] dbg_stash_occ,
    output wire                dbg_t6_start,
    // AXI debug (active during ORAM test phase, directly from m_axi)
    output wire                dbg_arvalid,
    output wire                dbg_arready,
    output wire [AXI_AW-1:0]  dbg_araddr,
    output wire [AXI_LENW-1:0] dbg_arlen,
    output wire                dbg_rvalid,
    output wire                dbg_rready,
    output wire                dbg_rlast,
    output wire                dbg_awvalid,
    output wire                dbg_awready,
    output wire [AXI_AW-1:0]  dbg_awaddr,
    output wire [AXI_LENW-1:0] dbg_awlen,
    output wire                dbg_wvalid,
    output wire                dbg_wready,
    output wire                dbg_wlast,
    output wire                dbg_bvalid,
    output wire                dbg_bready,

    // AXI4 Master ? DDR controller
    output wire [AXI_IDW-1:0]  m_axi_arid,
    output wire [AXI_AW-1:0]   m_axi_araddr,
    output wire [AXI_LENW-1:0] m_axi_arlen,
    output wire [2:0]           m_axi_arsize,
    output wire [1:0]           m_axi_arburst,
    output wire                 m_axi_arvalid,
    input  wire                 m_axi_arready,
    input  wire [AXI_IDW-1:0]  m_axi_rid,
    input  wire [AXI_DW-1:0]   m_axi_rdata,
    input  wire [1:0]           m_axi_rresp,
    input  wire                 m_axi_rlast,
    input  wire                 m_axi_rvalid,
    output wire                 m_axi_rready,
    output wire [AXI_IDW-1:0]  m_axi_awid,
    output wire [AXI_AW-1:0]   m_axi_awaddr,
    output wire [AXI_LENW-1:0] m_axi_awlen,
    output wire [2:0]           m_axi_awsize,
    output wire [1:0]           m_axi_awburst,
    output wire                 m_axi_awvalid,
    input  wire                 m_axi_awready,
    output wire [AXI_DW-1:0]   m_axi_wdata,
    output wire [AXI_SW-1:0]   m_axi_wstrb,
    output wire                 m_axi_wlast,
    output wire                 m_axi_wvalid,
    input  wire                 m_axi_wready,
    input  wire [AXI_IDW-1:0]  m_axi_bid,
    input  wire [1:0]           m_axi_bresp,
    input  wire                 m_axi_bvalid,
    output wire                 m_axi_bready
);

    // =========================================================================
    // Sequencer: IDLE ? DDR_INIT ? ORAM_TEST ? DONE
    // =========================================================================
    localparam SEQ_IDLE     = 2'd0;
    localparam SEQ_DDR_INIT = 2'd1;
    localparam SEQ_TEST     = 2'd2;
    localparam SEQ_DONE     = 2'd3;

    reg [1:0] seq_state;
    reg       ddr_init_start;
    reg       oram_test_start;

    wire      ddr_init_done, ddr_init_busy;
    wire      oram_test_done, oram_test_busy, oram_test_pass;
    wire [7:0] oram_fail_code_w;
    wire [3:0] oram_test_num_w;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            seq_state       <= SEQ_IDLE;
            ddr_init_start  <= 1'b0;
            oram_test_start <= 1'b0;
        end else begin
            ddr_init_start  <= 1'b0;
            oram_test_start <= 1'b0;

            case (seq_state)
                SEQ_IDLE: begin
                    if (btn_start) begin
                        ddr_init_start <= 1'b1;
                        seq_state      <= SEQ_DDR_INIT;
                    end
                end

                SEQ_DDR_INIT: begin
                    if (ddr_init_done) begin
                        oram_test_start <= 1'b1;
                        seq_state       <= SEQ_TEST;
                    end
                end

                SEQ_TEST: begin
                    if (oram_test_done)
                        seq_state <= SEQ_DONE;
                end

                SEQ_DONE: begin
                    // Stay here until reset
                end
            endcase
        end
    end

    // =========================================================================
    // AXI mux: DDR init phase uses write channel, ORAM test uses all channels
    // =========================================================================
    wire ddr_init_phase = (seq_state == SEQ_DDR_INIT);

    // DDR init AXI signals
    wire [AXI_IDW-1:0]  di_awid, di_arid;
    wire [AXI_AW-1:0]   di_awaddr, di_araddr;
    wire [AXI_LENW-1:0] di_awlen, di_arlen;
    wire [2:0]           di_awsize, di_arsize;
    wire [1:0]           di_awburst, di_arburst;
    wire                 di_awvalid, di_arvalid;
    wire [AXI_DW-1:0]   di_wdata;
    wire [AXI_SW-1:0]   di_wstrb;
    wire                 di_wlast, di_wvalid;
    wire                 di_bready, di_rready;

    // ORAM test AXI signals
    wire [AXI_IDW-1:0]  ot_awid, ot_arid;
    wire [AXI_AW-1:0]   ot_awaddr, ot_araddr;
    wire [AXI_LENW-1:0] ot_awlen, ot_arlen;
    wire [2:0]           ot_awsize, ot_arsize;
    wire [1:0]           ot_awburst, ot_arburst;
    wire                 ot_awvalid, ot_arvalid;
    wire [AXI_DW-1:0]   ot_wdata;
    wire [AXI_SW-1:0]   ot_wstrb;
    wire                 ot_wlast, ot_wvalid;
    wire                 ot_bready, ot_rready;

    // Mux to DDR
    assign m_axi_awid    = ddr_init_phase ? di_awid    : ot_awid;
    assign m_axi_awaddr  = ddr_init_phase ? di_awaddr  : ot_awaddr;
    assign m_axi_awlen   = ddr_init_phase ? di_awlen   : ot_awlen;
    assign m_axi_awsize  = ddr_init_phase ? di_awsize  : ot_awsize;
    assign m_axi_awburst = ddr_init_phase ? di_awburst : ot_awburst;
    assign m_axi_awvalid = ddr_init_phase ? di_awvalid : ot_awvalid;
    assign m_axi_wdata   = ddr_init_phase ? di_wdata   : ot_wdata;
    assign m_axi_wstrb   = ddr_init_phase ? di_wstrb   : ot_wstrb;
    assign m_axi_wlast   = ddr_init_phase ? di_wlast   : ot_wlast;
    assign m_axi_wvalid  = ddr_init_phase ? di_wvalid  : ot_wvalid;
    assign m_axi_bready  = ddr_init_phase ? di_bready  : ot_bready;
    assign m_axi_arid    = ddr_init_phase ? di_arid    : ot_arid;
    assign m_axi_araddr  = ddr_init_phase ? di_araddr  : ot_araddr;
    assign m_axi_arlen   = ddr_init_phase ? di_arlen   : ot_arlen;
    assign m_axi_arsize  = ddr_init_phase ? di_arsize  : ot_arsize;
    assign m_axi_arburst = ddr_init_phase ? di_arburst : ot_arburst;
    assign m_axi_arvalid = ddr_init_phase ? di_arvalid : ot_arvalid;
    assign m_axi_rready  = ddr_init_phase ? di_rready  : ot_rready;

    // =========================================================================
    // DDR pattern init
    // =========================================================================
    ddr_pattern_init u_ddr_init (
        .clk(clk), .rst_n(rst_n),
        .start(ddr_init_start),
        .done(ddr_init_done),
        .busy(ddr_init_busy),
        .m_axi_awid(di_awid),     .m_axi_awaddr(di_awaddr),
        .m_axi_awlen(di_awlen),   .m_axi_awsize(di_awsize),
        .m_axi_awburst(di_awburst), .m_axi_awvalid(di_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(di_wdata),   .m_axi_wstrb(di_wstrb),
        .m_axi_wlast(di_wlast),   .m_axi_wvalid(di_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid),    .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(di_bready),
        .m_axi_arid(di_arid),     .m_axi_araddr(di_araddr),
        .m_axi_arlen(di_arlen),   .m_axi_arsize(di_arsize),
        .m_axi_arburst(di_arburst), .m_axi_arvalid(di_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid),    .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(di_rready)
    );

    // =========================================================================
    // ORAM test driver
    // =========================================================================
    oram_test_driver u_test (
        .clk(clk), .rst_n(rst_n),
        .start(oram_test_start),
        .done(oram_test_done),
        .pass(oram_test_pass),
        .test_num(oram_test_num_w),
        .fail_code(oram_fail_code_w),
        .busy(oram_test_busy),
        .m_axi_arid(ot_arid),     .m_axi_araddr(ot_araddr),
        .m_axi_arlen(ot_arlen),   .m_axi_arsize(ot_arsize),
        .m_axi_arburst(ot_arburst), .m_axi_arvalid(ot_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid),    .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(ot_rready),
        .m_axi_awid(ot_awid),     .m_axi_awaddr(ot_awaddr),
        .m_axi_awlen(ot_awlen),   .m_axi_awsize(ot_awsize),
        .m_axi_awburst(ot_awburst), .m_axi_awvalid(ot_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(ot_wdata),   .m_axi_wstrb(ot_wstrb),
        .m_axi_wlast(ot_wlast),   .m_axi_wvalid(ot_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid),    .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(ot_bready),
        // Debug
        .dbg_state(dbg_state),
        .dbg_client_done(dbg_client_done),
        .dbg_client_req(dbg_client_req),
        .dbg_req_b(dbg_req_b),
        .dbg_req_b_new(dbg_req_b_new),
        .dbg_found_in_bucket(dbg_found_in_bucket),
        .dbg_found_in_stash(dbg_found_in_stash),
        .dbg_same_bucket(dbg_same_bucket),
        .dbg_err_stash_ovf(dbg_err_stash_ovf),
        .dbg_stash_occ(dbg_stash_occ),
        .dbg_t6_start(dbg_t6_start)
    );

    // =========================================================================
    // Status outputs
    // =========================================================================
    assign busy          = ddr_init_busy | oram_test_busy;
    assign done          = oram_test_done;
    assign pass          = oram_test_pass;
    assign fail          = |oram_fail_code;
    assign oram_fail_code = oram_fail_code_w;
    assign oram_test_num  = oram_test_num_w;

    // AXI debug - directly from top-level m_axi ports
    assign dbg_arvalid = m_axi_arvalid;
    assign dbg_arready = m_axi_arready;
    assign dbg_araddr  = m_axi_araddr;
    assign dbg_arlen   = m_axi_arlen;
    assign dbg_rvalid  = m_axi_rvalid;
    assign dbg_rready  = m_axi_rready;
    assign dbg_rlast   = m_axi_rlast;
    assign dbg_awvalid = m_axi_awvalid;
    assign dbg_awready = m_axi_awready;
    assign dbg_awaddr  = m_axi_awaddr;
    assign dbg_awlen   = m_axi_awlen;
    assign dbg_wvalid  = m_axi_wvalid;
    assign dbg_wready  = m_axi_wready;
    assign dbg_wlast   = m_axi_wlast;
    assign dbg_bvalid  = m_axi_bvalid;
    assign dbg_bready  = m_axi_bready;

endmodule