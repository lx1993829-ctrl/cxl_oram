`timescale 1ns / 1ps
`include "oram_params.vh"

module secure_oram_top #(
    parameter N            = `ORAM_N,
    parameter Z            = `ORAM_Z,
    parameter B            = `ORAM_B,
    parameter STASH_DEPTH  = `ORAM_STASH_DEPTH,
    parameter AXI_DW       = `AXI_DATA_W,
    parameter AXI_AW       = `AXI_ADDR_W,
    parameter AXI_SW       = `AXI_STRB_W,
    parameter AXI_IDW      = `AXI_ID_W,
    parameter AXI_LENW     = `AXI_LEN_W,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8,
    parameter SLOT_AW        = `SLOT_ADDR_W
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Management Interface
    input  wire                                   mgmt_req,
    input  wire [2:0]                             mgmt_op,
    input  wire [LEASE_ID_WIDTH-1:0]              mgmt_lease_id,
    input  wire [$clog2(NUM_CLIENTS)-1:0]         mgmt_client_id,
    input  wire [AXI_AW-1:0]                      mgmt_base_addr,
    input  wire [AXI_AW-1:0]                      mgmt_size,
    input  wire [31:0]                            mgmt_duration,
    input  wire [TOKEN_WIDTH-1:0]                 mgmt_token_in,
    output wire                                   mgmt_ack,
    output wire                                   mgmt_error,
    output wire [TOKEN_WIDTH-1:0]                 mgmt_token_out,

    // Client Interface (flattened)
    input  wire [NUM_CLIENTS-1:0]                     client_req,
    input  wire [NUM_CLIENTS-1:0]                     client_op,
    input  wire [NUM_CLIENTS*SLOT_AW-1:0]             client_slot_addr,
    input  wire [NUM_CLIENTS*AXI_DW-1:0]              client_wdata,
    input  wire [NUM_CLIENTS-1:0]                     client_wdata_valid,
    input  wire [NUM_CLIENTS*TOKEN_WIDTH-1:0]         client_token,
    input  wire [NUM_CLIENTS*LEASE_ID_WIDTH-1:0]      client_lease_id,
    output wire [NUM_CLIENTS-1:0]                     client_done,
    output wire [NUM_CLIENTS*AXI_DW-1:0]              client_rdata,
    output wire [NUM_CLIENTS-1:0]                     client_rdata_valid,
    output wire [NUM_CLIENTS-1:0]                     access_violation,

    // BRAM Init Interface
    input  wire                                       init_mode,
    input  wire [SLOT_AW-1:0]                         init_pm_wr_addr,
    input  wire [`BUCKET_ID_W-1:0]                    init_pm_wr_bucket,
    input  wire [1:0]                                 init_pm_wr_status,
    input  wire                                       init_pm_wr_en,
    input  wire [`BUCKET_ID_W-1:0]                    init_bm_wr_bucket,
    input  wire [Z*`SLOT_ID_W-1:0]                    init_bm_wr_slot_list,
    input  wire [`FILL_CNT_W-1:0]                     init_bm_wr_fill,
    input  wire                                       init_bm_wr_en,

    // AES-GCM Configuration
    input  wire [127:0]                               aes_key,
    input  wire [95:0]                                aes_iv_seed,

    // AXI Master -> DDR
    output wire [AXI_IDW-1:0]   m_axi_arid,
    output wire [AXI_AW-1:0]    m_axi_araddr,
    output wire [AXI_LENW-1:0]  m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire                  m_axi_arvalid,
    input  wire                  m_axi_arready,
    input  wire [AXI_IDW-1:0]   m_axi_rid,
    input  wire [AXI_DW-1:0]    m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready,
    output wire [AXI_IDW-1:0]   m_axi_awid,
    output wire [AXI_AW-1:0]    m_axi_awaddr,
    output wire [AXI_LENW-1:0]  m_axi_awlen,
    output wire [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,
    output wire                  m_axi_awvalid,
    input  wire                  m_axi_awready,
    output wire [AXI_DW-1:0]    m_axi_wdata,
    output wire [AXI_SW-1:0]    m_axi_wstrb,
    output wire                  m_axi_wlast,
    output wire                  m_axi_wvalid,
    input  wire                  m_axi_wready,
    input  wire [AXI_IDW-1:0]   m_axi_bid,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output wire                  m_axi_bready,
    output wire [4:0] dbg_oram_state,
    // Status
    output wire                  oram_busy,
    output wire                  err_stash_overflow,
    output wire                  err_bucket_overflow,
    output wire                  err_tag_mismatch,
    output wire [`BEAT_CNT_W-1:0]  oram_beat_cnt,
    output wire dbg_gcm_tag_match,
    output wire dbg_gcm_tag_valid,

    // Test controls
    input  wire dbg_force_same_bucket,
    input  wire [`BUCKET_ID_W-1:0] dbg_prng_override,
    input  wire perf_reset,

    // Performance counters - 4-way: rb=read/bucket, rs=read/stash, wb=write/bucket, ws=write/stash
    output wire [31:0] perf_rb_ddr_rd, perf_rb_decrypt, perf_rb_encrypt,
    output wire [31:0] perf_rb_compact, perf_rb_evict, perf_rb_ddr_wr,
    output wire [31:0] perf_rb_scan, perf_rb_stash,
    output wire [31:0] perf_rb_total, perf_rb_ops,
    output wire [31:0] perf_rs_ddr_rd, perf_rs_decrypt, perf_rs_encrypt,
    output wire [31:0] perf_rs_compact, perf_rs_evict, perf_rs_ddr_wr,
    output wire [31:0] perf_rs_scan, perf_rs_stash,
    output wire [31:0] perf_rs_total, perf_rs_ops,
    output wire [31:0] perf_wb_ddr_rd, perf_wb_decrypt, perf_wb_encrypt,
    output wire [31:0] perf_wb_compact, perf_wb_evict, perf_wb_ddr_wr,
    output wire [31:0] perf_wb_scan, perf_wb_stash,
    output wire [31:0] perf_wb_total, perf_wb_ops,
    output wire [31:0] perf_ws_ddr_rd, perf_ws_decrypt, perf_ws_encrypt,
    output wire [31:0] perf_ws_compact, perf_ws_evict, perf_ws_ddr_wr,
    output wire [31:0] perf_ws_scan, perf_ws_stash,
    output wire [31:0] perf_ws_total, perf_ws_ops
);

    // =========================================================================
    // Gate Keeper: lease token validation
    // =========================================================================
    wire [NUM_CLIENTS-1:0] val_valid;

    lease_token_table #(
        .ADDR_WIDTH(AXI_AW),
        .NUM_CLIENTS(NUM_CLIENTS),
        .NUM_REGIONS(NUM_REGIONS),
        .LEASE_ID_WIDTH(LEASE_ID_WIDTH),
        .TOKEN_WIDTH(TOKEN_WIDTH)
    ) u_lease_table (
        .clk(clk), .rst_n(rst_n),
        .mgmt_req(mgmt_req), .mgmt_op(mgmt_op),
        .mgmt_lease_id(mgmt_lease_id), .mgmt_client_id(mgmt_client_id),
        .mgmt_base_addr(mgmt_base_addr), .mgmt_size(mgmt_size),
        .mgmt_duration(mgmt_duration), .mgmt_token_in(mgmt_token_in),
        .mgmt_ack(mgmt_ack), .mgmt_error(mgmt_error),
        .mgmt_token_out(mgmt_token_out),
        .client_addr(client_slot_addr),
        .client_token(client_token),
        .client_lease_id(client_lease_id),
        .val_valid(val_valid),
        .active_leases()
    );

    wire [NUM_CLIENTS-1:0] gated_req = client_req & val_valid;
    assign access_violation = client_req & ~val_valid;

    // =========================================================================
    // Simple Priority Arbiter
    // =========================================================================
    reg                            arb_grant_valid;
    reg [$clog2(NUM_CLIENTS)-1:0]  arb_grant_id;
    reg                            oram_req_pending;

    wire       oram_client_done;
    wire [AXI_DW-1:0] oram_client_rdata;
    wire       oram_client_rdata_valid;
    wire       oram_client_stall;

    wire oram_is_busy = oram_client_stall || oram_req_pending;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_grant_valid  <= 1'b0;
            arb_grant_id     <= {$clog2(NUM_CLIENTS){1'b0}};
            oram_req_pending <= 1'b0;
        end else begin
            if (oram_client_done) begin
                oram_req_pending <= 1'b0;
                arb_grant_valid  <= 1'b0;
            end else if (!oram_is_busy && |gated_req) begin
                if (gated_req[0]) begin
                    arb_grant_valid  <= 1'b1;
                    arb_grant_id     <= 1'b0;
                    oram_req_pending <= 1'b1;
                end else if (gated_req[1]) begin
                    arb_grant_valid  <= 1'b1;
                    arb_grant_id     <= 1'b1;
                    oram_req_pending <= 1'b1;
                end
            end
        end
    end

    // Mux to ORAM - gate with !done to prevent re-capture on completion cycle
    wire                oram_req   = arb_grant_valid && !oram_client_stall && !oram_client_done;
    wire                oram_op    = client_op[arb_grant_id];
    wire [SLOT_AW-1:0] oram_addr  = client_slot_addr[arb_grant_id*SLOT_AW +: SLOT_AW];
    wire [AXI_DW-1:0]  oram_wdata = client_wdata[arb_grant_id*AXI_DW +: AXI_DW];
    wire                oram_wdata_valid = client_wdata_valid[arb_grant_id];
    
    // =========================================================================
    // ORAM with AES-GCM
    // =========================================================================
    flat_oram_gcm #(
        .N(N), .Z(Z), .B(B), .STASH_DEPTH(STASH_DEPTH)
    ) u_oram (
        .clk(clk), .rst_n(rst_n),
        .client_req(oram_req), .client_op(oram_op),
        .client_slot_addr(oram_addr),
        .client_wdata(oram_wdata),
        .client_wdata_valid(oram_wdata_valid),
        .client_wdata_beat(), .client_wdata_beat_req(oram_beat_cnt),
        .client_rdata(oram_client_rdata),
        .client_rdata_valid(oram_client_rdata_valid),
        .client_rdata_beat(),
        .client_done(oram_client_done),
        .client_stall(oram_client_stall),
        .err_bucket_overflow(err_bucket_overflow),
        .err_stash_overflow(err_stash_overflow),
        .err_tag_mismatch(err_tag_mismatch),
        .init_mode(init_mode),
        .init_pm_wr_addr(init_pm_wr_addr),
        .init_pm_wr_bucket(init_pm_wr_bucket),
        .init_pm_wr_status(init_pm_wr_status),
        .init_pm_wr_en(init_pm_wr_en),
        .init_bm_wr_bucket(init_bm_wr_bucket),
        .init_bm_wr_slot_list(init_bm_wr_slot_list),
        .init_bm_wr_fill(init_bm_wr_fill),
        .init_bm_wr_en(init_bm_wr_en),
        .aes_key(aes_key), .aes_iv_seed(aes_iv_seed),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr),
        .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid),
        .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .dbg_state(dbg_oram_state), .dbg_client_done(), .dbg_client_req(),
        .dbg_req_b(), .dbg_req_b_new(),
        .dbg_found_in_bucket(), .dbg_found_in_stash(),
        .dbg_same_bucket(), .dbg_err_stash_ovf(), .dbg_stash_occ(),
        .dbg_gcm_tag_match(dbg_gcm_tag_match),.dbg_gcm_tag_valid(dbg_gcm_tag_valid),
        .dbg_force_same_bucket(dbg_force_same_bucket),
        .dbg_prng_override(dbg_prng_override),
        .perf_reset(perf_reset),
        .perf_rb_ddr_rd(perf_rb_ddr_rd), .perf_rb_decrypt(perf_rb_decrypt),
        .perf_rb_encrypt(perf_rb_encrypt), .perf_rb_compact(perf_rb_compact),
        .perf_rb_evict(perf_rb_evict), .perf_rb_ddr_wr(perf_rb_ddr_wr),
        .perf_rb_scan(perf_rb_scan), .perf_rb_stash(perf_rb_stash),
        .perf_rb_total(perf_rb_total), .perf_rb_ops(perf_rb_ops),
        .perf_rs_ddr_rd(perf_rs_ddr_rd), .perf_rs_decrypt(perf_rs_decrypt),
        .perf_rs_encrypt(perf_rs_encrypt), .perf_rs_compact(perf_rs_compact),
        .perf_rs_evict(perf_rs_evict), .perf_rs_ddr_wr(perf_rs_ddr_wr),
        .perf_rs_scan(perf_rs_scan), .perf_rs_stash(perf_rs_stash),
        .perf_rs_total(perf_rs_total), .perf_rs_ops(perf_rs_ops),
        .perf_wb_ddr_rd(perf_wb_ddr_rd), .perf_wb_decrypt(perf_wb_decrypt),
        .perf_wb_encrypt(perf_wb_encrypt), .perf_wb_compact(perf_wb_compact),
        .perf_wb_evict(perf_wb_evict), .perf_wb_ddr_wr(perf_wb_ddr_wr),
        .perf_wb_scan(perf_wb_scan), .perf_wb_stash(perf_wb_stash),
        .perf_wb_total(perf_wb_total), .perf_wb_ops(perf_wb_ops),
        .perf_ws_ddr_rd(perf_ws_ddr_rd), .perf_ws_decrypt(perf_ws_decrypt),
        .perf_ws_encrypt(perf_ws_encrypt), .perf_ws_compact(perf_ws_compact),
        .perf_ws_evict(perf_ws_evict), .perf_ws_ddr_wr(perf_ws_ddr_wr),
        .perf_ws_scan(perf_ws_scan), .perf_ws_stash(perf_ws_stash),
        .perf_ws_total(perf_ws_total), .perf_ws_ops(perf_ws_ops)
    );

    // =========================================================================
    // Route outputs to correct client
    // =========================================================================
    genvar ci;
    generate
        for (ci = 0; ci < NUM_CLIENTS; ci = ci + 1) begin : gen_out
            assign client_done[ci] = oram_client_done &&
                (arb_grant_id == ci[$clog2(NUM_CLIENTS)-1:0]);
            assign client_rdata[ci*AXI_DW +: AXI_DW] = oram_client_rdata;
            assign client_rdata_valid[ci] = oram_client_rdata_valid &&
                (arb_grant_id == ci[$clog2(NUM_CLIENTS)-1:0]);
        end
    endgenerate

    assign oram_busy = oram_is_busy;

endmodule