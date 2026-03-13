// =============================================================================
// oram_test_driver.v - On-board self-test for flat_oram_top (v46)
// =============================================================================
// Synthesisable RTL that pre-populates pos_map/bucket_meta via the init_mode
// mux, then runs tests T1-T5, reporting PASS/FAIL via output ports.
//
// Usage:
//   - Connect clk, rst_n, start (button/VIO pulse)
//   - Observe: done, pass, fail_code[7:0], test_num[3:0], busy
//   - Connect AXI master ports to your DDR controller
//   - DDR must be pre-initialised with pattern: beat[i] = {8{i[31:0]}}
//     (use MIG init or a small DMA fill before asserting start)
//
// Tests:
//   T1: Read slot#1 from bucket 0 (bucket hit, verify DDR data)
//   T2: Re-read slot#1 (stash hit - moved to stash after T1)
//   T3: Write slot#3 with pattern beat b ? {8{b}}
//   T4: Re-read slot#3 (verify written data survives)
//   T5a-d: Sequential reads of slots 2, 4, 5, 6
// =============================================================================

`include "oram_params.vh"

module oram_test_driver #(
    parameter AXI_DW   = `AXI_DATA_W,
    parameter AXI_AW   = `AXI_ADDR_W,
    parameter AXI_SW   = `AXI_STRB_W,
    parameter AXI_IDW  = `AXI_ID_W,
    parameter AXI_LENW = `AXI_LEN_W,
    parameter SLOT_AW  = `SLOT_ADDR_W,
    parameter BUCKET_W = `BUCKET_ID_W,
    parameter SLOT_W   = `SLOT_ID_W,
    parameter FILL_W   = `FILL_CNT_W,
    parameter BEAT_W   = `BEAT_CNT_W,
    parameter Z        = `ORAM_Z,
    parameter BEATS    = `BEATS_PER_BLOCK
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,       // pulse to begin

    output reg                 done,
    output reg                 pass,
    output reg  [3:0]          test_num,
    output reg  [7:0]          fail_code,   // bit per sub-test
    output reg                 busy,

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
    output wire                 m_axi_bready,

    // Debug passthrough from flat_oram_top
    output wire [3:0]           dbg_state,
    output wire                 dbg_client_done,
    output wire                 dbg_client_req,
    output wire [BUCKET_W-1:0] dbg_req_b,
    output wire [BUCKET_W-1:0] dbg_req_b_new,
    output wire                 dbg_found_in_bucket,
    output wire                 dbg_found_in_stash,
    output wire                 dbg_same_bucket,
    output wire                 dbg_err_stash_ovf,
    output wire [`STASH_PTR_W:0] dbg_stash_occ,
    output reg                  dbg_t6_start       // pulse: rises when first T6 access begins
);

    // =========================================================================
    // ORAM client signals
    // =========================================================================
    reg                 oram_req;
    reg                 oram_op;
    reg  [SLOT_AW-1:0]  oram_slot_addr;
    wire [AXI_DW-1:0]  oram_wdata;
    reg                 oram_wdata_valid;
    reg  [BEAT_W-1:0]   oram_wdata_beat;
    wire [BEAT_W-1:0]   oram_wdata_beat_req;
    wire [AXI_DW-1:0]  oram_rdata;
    wire                oram_rdata_valid;
    wire [BEAT_W-1:0]   oram_rdata_beat;
    wire                oram_done;
    wire                oram_stall;
    wire                oram_err_stash_ovf;
    wire                oram_err_bkt_ovf;

    // =========================================================================
    // Init mux signals
    // =========================================================================
    reg                          init_mode;
    reg  [SLOT_AW-1:0]           init_pm_wr_addr;
    reg  [BUCKET_W-1:0]          init_pm_wr_bucket;
    reg  [1:0]                   init_pm_wr_status;
    reg                          init_pm_wr_en;
    reg  [BUCKET_W-1:0]          init_bm_wr_bucket;
    reg  [Z*SLOT_W-1:0]         init_bm_wr_slot_list;
    reg  [FILL_W-1:0]            init_bm_wr_fill;
    reg                          init_bm_wr_en;

    // =========================================================================
    // DUT
    // =========================================================================
    flat_oram_top u_oram (
        .clk(clk), .rst_n(rst_n),
        .client_req(oram_req), .client_op(oram_op),
        .client_slot_addr(oram_slot_addr),
        .client_wdata(oram_wdata),
        .client_wdata_valid(oram_wdata_valid),
        .client_wdata_beat(oram_wdata_beat),
        .client_wdata_beat_req(oram_wdata_beat_req),
        .client_rdata(oram_rdata),
        .client_rdata_valid(oram_rdata_valid),
        .client_rdata_beat(oram_rdata_beat),
        .client_done(oram_done),
        .client_stall(oram_stall),
        .err_bucket_overflow(oram_err_bkt_ovf),
        .err_stash_overflow(oram_err_stash_ovf),
        // Init mux
        .init_mode(init_mode),
        .init_pm_wr_addr(init_pm_wr_addr),
        .init_pm_wr_bucket(init_pm_wr_bucket),
        .init_pm_wr_status(init_pm_wr_status),
        .init_pm_wr_en(init_pm_wr_en),
        .init_bm_wr_bucket(init_bm_wr_bucket),
        .init_bm_wr_slot_list(init_bm_wr_slot_list),
        .init_bm_wr_fill(init_bm_wr_fill),
        .init_bm_wr_en(init_bm_wr_en),
        // AXI passthrough
        .m_axi_arid(m_axi_arid),     .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen),   .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid),       .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp),   .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
        .m_axi_awid(m_axi_awid),     .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen),   .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata),   .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast),   .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid),       .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
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
        .dbg_stash_occ(dbg_stash_occ)
    );

    // =========================================================================
    // Write data: combinational from beat_req.  Pattern: beat b ? {8{b}}
    // =========================================================================
    assign oram_wdata = oram_wdata_valid
                        ? {8{{22'b0, oram_wdata_beat_req}}}
                        : {AXI_DW{1'b0}};

    // =========================================================================
    // Read data checker - incremental, no captured array needed
    // =========================================================================
    reg        rdata_err;
    reg        rdata_err_clr;   // pulse from FSM to clear error before each access
    reg        check_write_pat;
    reg        no_check;        // skip data verification for modified buckets
    reg [31:0] expect_gb0;   // global_beat for beat 0

    // Debug: expected value for comparison
    wire [31:0] dbg_exp_beat = check_write_pat
                               ? {{22'b0}, oram_rdata_beat}
                               : (expect_gb0 + {{22'b0}, oram_rdata_beat});
    wire [AXI_DW-1:0] dbg_expected = {8{dbg_exp_beat}};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rdata_err <= 1'b0;
        else if (rdata_err_clr)
            rdata_err <= 1'b0;
        else if (oram_rdata_valid) begin
            // synthesis translate_off
            if (oram_rdata_beat <= 3 || (oram_rdata !== dbg_expected && !rdata_err))
                $display("[CHK] t=%0t test=%0d beat=%0d got=%08X exp=%08X chk_wr=%b gb0=%0d %s",
                    $time, test_num, oram_rdata_beat,
                    oram_rdata[31:0], dbg_exp_beat,
                    check_write_pat, expect_gb0,
                    (oram_rdata !== dbg_expected) ? "MISMATCH" : "ok");
            // synthesis translate_on

            if (!no_check && oram_rdata !== dbg_expected)
                rdata_err <= 1'b1;
        end
    end

    // =========================================================================
    // Test table - synthesisable ROM (combinational from access_idx)
    // =========================================================================
    // Order: T1, T2, T3, T4 (proven), T6x10 (stress), T5a-d (last)
    // Total: 4 + 10 + 4 = 18 accesses
    localparam NUM_ACCESSES = 18;

    reg [SLOT_AW-1:0] tbl_addr;
    reg                tbl_op;
    reg                tbl_chk_wr;
    reg                tbl_no_check;
    reg [31:0]         tbl_gb0;
    reg [3:0]          tbl_id;

    // Slot address table for T6 stress: 6 slots, stride-5 rotation
    // i=0?slot0(0x1000), i=1?slot5(0x6000), i=2?slot4(0x5000),
    // i=3?slot3(0x4000), i=4?slot2(0x3000), i=5?slot1(0x2000), ...
    // Write on i%7==0 (i=0 and i=7)
    reg [SLOT_AW-1:0] t6_addr;
    reg                t6_op;
    always @(*) begin
        case (access_idx - 5'd4)  // T6 starts at index 4
            4'd0: begin t6_addr = 32'h0000_1000; t6_op = 1; end // i=0: slot#1 WR
            4'd1: begin t6_addr = 32'h0000_6000; t6_op = 0; end // i=1: slot#6 RD
            4'd2: begin t6_addr = 32'h0000_5000; t6_op = 0; end // i=2: slot#5 RD
            4'd3: begin t6_addr = 32'h0000_4000; t6_op = 0; end // i=3: slot#4 RD
            4'd4: begin t6_addr = 32'h0000_3000; t6_op = 0; end // i=4: slot#3 RD
            4'd5: begin t6_addr = 32'h0000_2000; t6_op = 0; end // i=5: slot#2 RD
            4'd6: begin t6_addr = 32'h0000_1000; t6_op = 0; end // i=6: slot#1 RD
            4'd7: begin t6_addr = 32'h0000_6000; t6_op = 1; end // i=7: slot#6 WR
            4'd8: begin t6_addr = 32'h0000_5000; t6_op = 0; end // i=8: slot#5 RD
            4'd9: begin t6_addr = 32'h0000_4000; t6_op = 0; end // i=9: slot#4 RD
         default: begin t6_addr = 32'h0000_1000; t6_op = 0; end
        endcase
    end

    always @(*) begin
        case (access_idx)
            //                addr             op  chk_wr no_chk gb0                id
            // --- T1-T4: proven core tests ---
            5'd0:  begin tbl_addr = 32'h0000_1000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 0; tbl_gb0 = 32'd0;    tbl_id = 4'd1; end // T1: fresh DDR read
            5'd1:  begin tbl_addr = 32'h0000_1000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0;    tbl_id = 4'd2; end // T2: stash re-read
            5'd2:  begin tbl_addr = 32'h0000_3000; tbl_op = 1; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd1024; tbl_id = 4'd3; end // T3: write
            5'd3:  begin tbl_addr = 32'h0000_3000; tbl_op = 0; tbl_chk_wr = 1; tbl_no_check = 0; tbl_gb0 = 32'd1024; tbl_id = 4'd4; end // T4: read-after-write
            // --- T6: stress test (10 mixed R/W, no data check) ---
            5'd4:  begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd5:  begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd6:  begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd7:  begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd8:  begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd9:  begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd10: begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd11: begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd12: begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            5'd13: begin tbl_addr = t6_addr; tbl_op = t6_op; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0; tbl_id = 4'd6; end
            // --- T5a-d: sequential slot reads (last) ---
            5'd14: begin tbl_addr = 32'h0000_2000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd128;  tbl_id = 4'd5; end // T5a
            5'd15: begin tbl_addr = 32'h0000_4000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd1152; tbl_id = 4'd5; end // T5b
            5'd16: begin tbl_addr = 32'h0000_5000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd2048; tbl_id = 4'd5; end // T5c
            5'd17: begin tbl_addr = 32'h0000_6000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd2176; tbl_id = 4'd5; end // T5d
         default: begin tbl_addr = 32'h0000_0000; tbl_op = 0; tbl_chk_wr = 0; tbl_no_check = 1; tbl_gb0 = 32'd0;    tbl_id = 4'd0; end
        endcase
    end

    // =========================================================================
    // Init tables - synthesisable ROM (combinational from init_ptr)
    // =========================================================================
    localparam PM_COUNT = 6;
    localparam BM_COUNT = 3;

    reg [SLOT_AW-1:0]  pm_tbl_addr;
    reg [BUCKET_W-1:0] pm_tbl_bucket;

    always @(*) begin
        case (init_ptr)
            3'd0: begin pm_tbl_addr = 32'h0000_1000; pm_tbl_bucket = {BUCKET_W{1'b0}};              end // slot#1 -> bkt 0
            3'd1: begin pm_tbl_addr = 32'h0000_2000; pm_tbl_bucket = {BUCKET_W{1'b0}};              end // slot#2 -> bkt 0
            3'd2: begin pm_tbl_addr = 32'h0000_3000; pm_tbl_bucket = {{(BUCKET_W-1){1'b0}}, 1'b1};  end // slot#3 -> bkt 1
            3'd3: begin pm_tbl_addr = 32'h0000_4000; pm_tbl_bucket = {{(BUCKET_W-1){1'b0}}, 1'b1};  end // slot#4 -> bkt 1
            3'd4: begin pm_tbl_addr = 32'h0000_5000; pm_tbl_bucket = {{(BUCKET_W-2){1'b0}}, 2'b10}; end // slot#5 -> bkt 2
            3'd5: begin pm_tbl_addr = 32'h0000_6000; pm_tbl_bucket = {{(BUCKET_W-2){1'b0}}, 2'b10}; end // slot#6 -> bkt 2
         default: begin pm_tbl_addr = 32'h0000_0000; pm_tbl_bucket = {BUCKET_W{1'b0}};              end
        endcase
    end

    reg [BUCKET_W-1:0]     bm_tbl_bucket;
    reg [Z*SLOT_W-1:0]    bm_tbl_slots;
    reg [FILL_W-1:0]       bm_tbl_fill;

    always @(*) begin
        case (init_ptr)
            3'd0: begin bm_tbl_bucket = {BUCKET_W{1'b0}};              bm_tbl_slots = {{((Z-2)*SLOT_W){1'b0}}, {{(SLOT_W-2){1'b0}},2'd2}, {{(SLOT_W-1){1'b0}},1'd1}}; bm_tbl_fill = 4'd2; end
            3'd1: begin bm_tbl_bucket = {{(BUCKET_W-1){1'b0}}, 1'b1};  bm_tbl_slots = {{((Z-2)*SLOT_W){1'b0}}, {{(SLOT_W-3){1'b0}},3'd4}, {{(SLOT_W-2){1'b0}},2'd3}}; bm_tbl_fill = 4'd2; end
            3'd2: begin bm_tbl_bucket = {{(BUCKET_W-2){1'b0}}, 2'b10}; bm_tbl_slots = {{((Z-2)*SLOT_W){1'b0}}, {{(SLOT_W-3){1'b0}},3'd6}, {{(SLOT_W-3){1'b0}},3'd5}}; bm_tbl_fill = 4'd2; end
         default: begin bm_tbl_bucket = {BUCKET_W{1'b0}};              bm_tbl_slots = {(Z*SLOT_W){1'b0}};                                                                bm_tbl_fill = 4'd0; end
        endcase
    end

    // =========================================================================
    // Test driver FSM
    // =========================================================================
    localparam TD_IDLE    = 3'd0;
    localparam TD_INIT_PM = 3'd1;
    localparam TD_INIT_BM = 3'd2;
    localparam TD_REQ     = 3'd3;
    localparam TD_WAIT    = 3'd4;
    localparam TD_CHECK   = 3'd5;
    localparam TD_DONE    = 3'd6;

    reg [2:0]   td_state;
    reg [2:0]   init_ptr;
    reg [4:0]   access_idx;
    reg [23:0]  timeout_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            td_state       <= TD_IDLE;
            init_ptr       <= 3'd0;
            access_idx <= 5'd0;
            timeout_cnt    <= 24'd0;
            done           <= 1'b0;
            pass           <= 1'b0;
            busy           <= 1'b0;
            test_num       <= 4'd0;
            fail_code      <= 8'd0;
            oram_req       <= 1'b0;
            oram_op        <= 1'b0;
            oram_slot_addr <= {SLOT_AW{1'b0}};
            oram_wdata_valid <= 1'b0;
            oram_wdata_beat  <= {BEAT_W{1'b0}};
            check_write_pat  <= 1'b0;
            no_check         <= 1'b1;
            expect_gb0       <= 32'd0;
            init_mode        <= 1'b0;
            init_pm_wr_en    <= 1'b0;
            init_bm_wr_en    <= 1'b0;
            rdata_err_clr    <= 1'b0;
            dbg_t6_start     <= 1'b0;
        end else begin
            // Defaults
            oram_req      <= 1'b0;
            init_pm_wr_en <= 1'b0;
            init_bm_wr_en <= 1'b0;
            rdata_err_clr <= 1'b0;

            case (td_state)

                TD_IDLE: begin
                    done <= 1'b0;
                    if (start && !busy) begin
                        busy       <= 1'b1;
                        fail_code  <= 8'd0;
                        access_idx <= 5'd0;
                        init_ptr   <= 3'd0;
                        init_mode  <= 1'b1;
                        td_state   <= TD_INIT_PM;
                    end
                end

                // Write pos_map entries: one per cycle
                TD_INIT_PM: begin
                    if (init_ptr < PM_COUNT) begin
                        init_pm_wr_addr   <= pm_tbl_addr;
                        init_pm_wr_bucket <= pm_tbl_bucket;
                        init_pm_wr_status <= `ST_VALID;
                        init_pm_wr_en     <= 1'b1;
                        init_ptr          <= init_ptr + 1'b1;
                    end else begin
                        init_ptr <= 3'd0;
                        td_state <= TD_INIT_BM;
                    end
                end

                // Write bucket_meta entries: one per cycle
                TD_INIT_BM: begin
                    if (init_ptr < BM_COUNT) begin
                        init_bm_wr_bucket    <= bm_tbl_bucket;
                        init_bm_wr_slot_list <= bm_tbl_slots;
                        init_bm_wr_fill      <= bm_tbl_fill;
                        init_bm_wr_en        <= 1'b1;
                        init_ptr             <= init_ptr + 1'b1;
                    end else begin
                        init_mode <= 1'b0;  // release BRAM write control to FSM
                        td_state  <= TD_REQ;
                    end
                end

                // Issue ORAM request
                TD_REQ: begin
                    test_num        <= tbl_id;
                    oram_req        <= 1'b1;
                    oram_op         <= tbl_op;
                    oram_slot_addr  <= tbl_addr;
                    check_write_pat <= tbl_chk_wr;
                    no_check        <= tbl_no_check;
                    expect_gb0      <= tbl_gb0;
                    rdata_err_clr   <= 1'b1;

                    // Debug trigger: goes high on first T6 access, stays high
                    if (tbl_id == 4'd6 && !dbg_t6_start)
                        dbg_t6_start <= 1'b1;

                    if (tbl_op)
                        oram_wdata_valid <= 1'b1;

                    timeout_cnt <= 24'd0;
                    td_state    <= TD_WAIT;
                end

                // Wait for done
                TD_WAIT: begin
                    timeout_cnt <= timeout_cnt + 1'b1;

                    if (oram_done) begin
                        oram_wdata_valid <= 1'b0;
                        td_state <= TD_CHECK;
                    end else if (timeout_cnt == 24'hFF_FFFF) begin
                        oram_wdata_valid <= 1'b0;
                        fail_code[test_num[2:0]] <= 1'b1;  // index by test group
                        td_state <= TD_CHECK;
                    end
                end

                // Check and advance
                TD_CHECK: begin
                    if (!tbl_op && !no_check && rdata_err)
                        fail_code[test_num[2:0]] <= 1'b1;  // index by test group

                    if (oram_err_stash_ovf)
                        fail_code[7] <= 1'b1;

                    if (access_idx == NUM_ACCESSES - 1)
                        td_state <= TD_DONE;
                    else begin
                        access_idx <= access_idx + 1'b1;
                        td_state   <= TD_REQ;
                    end
                end

                TD_DONE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    pass <= (fail_code == 8'd0);
                end

                default: td_state <= TD_IDLE;
            endcase
        end
    end

endmodule