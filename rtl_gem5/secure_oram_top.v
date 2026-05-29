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
    output wire sel_stash_out,
    output wire sel_posmap_out,
    output wire st_burst_busy_out,
    output wire st_ext_busy_out,
    output wire pm_busy_out,

    // Debug: pos_map AXI internals (visible to SimObject)
    output wire        dbg_pm_awvalid,
    output wire        dbg_pm_wvalid,
    output wire        dbg_pm_arvalid,
    output wire        dbg_pm_bready,
    output wire [2:0]  dbg_pm_state,
    output wire        dbg_mux_awvalid,   // post-mux m_axi_awvalid
    output wire        dbg_found_in_bucket,
    output wire        dbg_found_in_stash,

    // HT FSM debug
    output wire [2:0]  dbg_ht_state,
    output wire [2:0]  dbg_ht_op,
    output wire        dbg_ht_done,
    output wire        dbg_ht_latch_lookup,
    output wire        dbg_ht_latch_insert,
    output wire        dbg_ht_latch_delete,
    output wire        dbg_ht_sng_rd_req,
    output wire        dbg_ht_sng_wr_req,
    output wire        dbg_ht_sng_done,
    // Test controls
    input  wire dbg_force_same_bucket,
    input  wire [`BUCKET_ID_W-1:0] dbg_prng_override,
    input  wire dbg_prng_override_en,
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
        .ADDR_WIDTH(SLOT_AW),
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
    // ORAM with AES-GCM (bucket master AXI signals go to internal bkt_* wires)
    // =========================================================================
    wire [AXI_IDW-1:0]  bkt_arid, bkt_awid;
    wire [AXI_AW-1:0]   bkt_araddr, bkt_awaddr;
    wire [AXI_LENW-1:0] bkt_arlen, bkt_awlen;
    wire [2:0]           bkt_arsize, bkt_awsize;
    wire [1:0]           bkt_arburst, bkt_awburst;
    wire                 bkt_arvalid, bkt_awvalid;
    wire                 bkt_arready, bkt_awready;
    wire                 bkt_rready;
    wire [AXI_DW-1:0]   bkt_wdata;
    wire [AXI_SW-1:0]   bkt_wstrb;
    wire                 bkt_wlast, bkt_wvalid, bkt_wready;
    wire                 bkt_bready;

    // =========================================================================
    // Internal AXI wires: stash master
    // =========================================================================
    wire [AXI_IDW-1:0]  st_arid, st_awid;
    wire [AXI_AW-1:0]   st_araddr, st_awaddr;
    wire [AXI_LENW-1:0] st_arlen, st_awlen;
    wire [2:0]           st_arsize, st_awsize;
    wire [1:0]           st_arburst, st_awburst;
    wire                 st_arvalid, st_awvalid;
    wire                 st_arready, st_awready;
    wire                 st_rready;
    wire [AXI_DW-1:0]   st_wdata;
    wire [AXI_SW-1:0]   st_wstrb;
    wire                 st_wlast, st_wvalid, st_wready;
    wire                 st_bready;

    // =========================================================================
    // Stash AXI Master
    // =========================================================================
    localparam PTR_W   = `STASH_PTR_W;
    localparam BEAT_AW = 7;

    wire                     st_ext_rd_req;
    wire                     st_ext_wr_req;
    wire [PTR_W-1:0]        st_ext_entry;
    wire                     st_ext_rd_done;
    wire                     st_ext_wr_done;
    wire                     st_ext_busy;
    wire                     st_burst_busy;

    wire [AXI_DW-1:0]       st_ext_lb_wr_data;
    wire [BEAT_AW-1:0]      st_ext_lb_wr_addr;
    wire                     st_ext_lb_wr_en;
    wire [BEAT_AW-1:0]      st_ext_lb_rd_addr;
    wire                     st_ext_lb_rd_en;
    wire [AXI_DW-1:0]       st_ext_lb_rd_data;
    wire                     st_ext_lb_rd_valid;

    // Single-beat command wires (hash table access)
    // Gate HT single-beat commands: WHITELIST of safe states where
    // NO other AXI master is active. Any state with bucket, stash, or
    // pos_map AXI activity is unsafe (mux cross-contamination).
    wire                            st_sng_rd_req_raw;
    wire                            st_sng_wr_req_raw;
    wire [4:0]                      oram_fsm_state = dbg_oram_state;
    // Safe: S_IDLE(0), S_SCAN(3), S_SCAN2(4), S_EXT_IV_RD(5),
    //       S_EXT_DEC_START(6), S_EXT_DEC_FEED(7), S_EXT_DEC_RECV(8),
    //       S_EXT_DEC_WAIT(9), S_EXTRACT_WR(10), S_COMPACT(13), S_EVICT(14),
    //       S_SB_WR_ENC_START(15), S_SB_WR_ENC_FEED(16),
    //       S_SB_WR_ENC_RECV(17), S_SB_WR_ENC_TAG(18),
    //       S_EV_ENC_START(20), S_EV_ENC_FEED(21),
    //       S_EV_ENC_RECV(22), S_EV_ENC_TAG(23),
    //       S_HT_LOOKUP_WAIT(31)
    // Step 4: S_COMPACT(13) and S_EVICT(14) moved to safe list —
    //   main FSM parks there waiting for HT BKT_FIND, no burst AXI.
    //   S_HT_LOOKUP_WAIT(31) is safe by default (not in blacklist).
    // Unsafe: S_POS_LOOKUP(1), S_DDR_READ(2), S_STASH_SEARCH(11),
    //         S_STASH_READ(12), S_DDR_WRITE(19), S_ST_LOAD(28),
    //         S_ST_FLUSH(29)
    // ht_safe: gate HT single-beat commands. Only block during states
    // where another AXI master has burst traffic in flight.
    // pm_busy_w removed: S_POS_LOOKUP(1) blacklist covers pos_map phase.
    // pm_busy lingers during pos_map write-back which overlaps into later
    // states, blocking HT ops and causing deadlock at S_HT_LOOKUP_WAIT.
    // st_ext_busy removed: sel_stash_fsm whitelist handles mux routing.
    // stash_axi_master ignores cmd_single_read while cmd_busy=1.
    wire ht_safe = (oram_fsm_state != 5'd1)   // S_POS_LOOKUP
                && (oram_fsm_state != 5'd2)   // S_DDR_READ
                && (oram_fsm_state != 5'd11)  // S_STASH_SEARCH
                && (oram_fsm_state != 5'd12)  // S_STASH_READ
                && (oram_fsm_state != 5'd19)  // S_DDR_WRITE
                && (oram_fsm_state != 5'd28)  // S_ST_LOAD
                && (oram_fsm_state != 5'd29); // S_ST_FLUSH
    wire                            st_sng_rd_req = st_sng_rd_req_raw && ht_safe;
    wire                            st_sng_wr_req = st_sng_wr_req_raw && ht_safe;
    wire [`AXI_ADDR_W-1:0]         st_sng_addr;
    wire [`AXI_DATA_W-1:0]         st_sng_wdata;
    wire [`AXI_DATA_W-1:0]         st_sng_rdata;
    wire                            st_sng_done;
    stash_axi_master u_stash_axi (
        .clk(clk), .rst_n(rst_n),
        .cmd_read(st_ext_rd_req),
        .cmd_write(st_ext_wr_req),
        .cmd_entry(st_ext_entry),
        .cmd_read_done(st_ext_rd_done),
        .cmd_write_done(st_ext_wr_done),
        .cmd_busy(st_ext_busy),
        .cmd_burst_busy(st_burst_busy),
        .cmd_single_read(st_sng_rd_req),
        .cmd_single_write(st_sng_wr_req),
        .cmd_single_addr(st_sng_addr),
        .cmd_single_wdata(st_sng_wdata),
        .cmd_single_rdata(st_sng_rdata),
        .cmd_single_done(st_sng_done),
        .lb_wr_data(st_ext_lb_wr_data),
        .lb_wr_addr(st_ext_lb_wr_addr),
        .lb_wr_en(st_ext_lb_wr_en),
        .lb_rd_addr(st_ext_lb_rd_addr),
        .lb_rd_en(st_ext_lb_rd_en),
        .lb_rd_data(st_ext_lb_rd_data),
        .lb_rd_valid(st_ext_lb_rd_valid),
        .m_axi_arid(st_arid), .m_axi_araddr(st_araddr),
        .m_axi_arlen(st_arlen), .m_axi_arsize(st_arsize),
        .m_axi_arburst(st_arburst), .m_axi_arvalid(st_arvalid),
        .m_axi_arready(st_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(st_rready),
        .m_axi_awid(st_awid), .m_axi_awaddr(st_awaddr),
        .m_axi_awlen(st_awlen), .m_axi_awsize(st_awsize),
        .m_axi_awburst(st_awburst), .m_axi_awvalid(st_awvalid),
        .m_axi_awready(st_awready),
        .m_axi_wdata(st_wdata), .m_axi_wstrb(st_wstrb),
        .m_axi_wlast(st_wlast), .m_axi_wvalid(st_wvalid),
        .m_axi_wready(st_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(st_bready)
    );

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
        // Bucket AXI -> internal bkt_* wires (muxed to output below)
        .m_axi_arid(bkt_arid), .m_axi_araddr(bkt_araddr),
        .m_axi_arlen(bkt_arlen), .m_axi_arsize(bkt_arsize),
        .m_axi_arburst(bkt_arburst), .m_axi_arvalid(bkt_arvalid),
        .m_axi_arready(bkt_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata),
        .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(bkt_rready),
        .m_axi_awid(bkt_awid), .m_axi_awaddr(bkt_awaddr),
        .m_axi_awlen(bkt_awlen), .m_axi_awsize(bkt_awsize),
        .m_axi_awburst(bkt_awburst), .m_axi_awvalid(bkt_awvalid),
        .m_axi_awready(bkt_awready),
        .m_axi_wdata(bkt_wdata), .m_axi_wstrb(bkt_wstrb),
        .m_axi_wlast(bkt_wlast), .m_axi_wvalid(bkt_wvalid),
        .m_axi_wready(bkt_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(bkt_bready),
        // Stash external memory interface -> stash_axi_master
        .st_ext_rd_req(st_ext_rd_req),
        .st_ext_wr_req(st_ext_wr_req),
        .st_ext_entry(st_ext_entry),
        .st_ext_rd_done(st_ext_rd_done),
        .st_ext_wr_done(st_ext_wr_done),
        .st_ext_busy(st_ext_busy),
        .st_burst_busy(st_burst_busy),
        // Single-beat hash table access
        .st_sng_rd_req(st_sng_rd_req_raw),
        .st_sng_wr_req(st_sng_wr_req_raw),
        .st_sng_addr(st_sng_addr),
        .st_sng_wdata(st_sng_wdata),
        .st_sng_rdata(st_sng_rdata),
        .st_sng_done(st_sng_done),
        .st_ext_lb_wr_data(st_ext_lb_wr_data),
        .st_ext_lb_wr_addr(st_ext_lb_wr_addr),
        .st_ext_lb_wr_en(st_ext_lb_wr_en),
        .st_ext_lb_rd_addr(st_ext_lb_rd_addr),
        .st_ext_lb_rd_en(st_ext_lb_rd_en),
        .st_ext_lb_rd_data(st_ext_lb_rd_data),
        .st_ext_lb_rd_valid(st_ext_lb_rd_valid),
        // Debug
        .dbg_state(dbg_oram_state), .dbg_client_done(), .dbg_client_req(),
        .dbg_req_b(), .dbg_req_b_new(),
        .dbg_found_in_bucket(dbg_found_in_bucket), .dbg_found_in_stash(dbg_found_in_stash),
        .dbg_same_bucket(), .dbg_err_stash_ovf(), .dbg_stash_occ(),
        .dbg_gcm_tag_match(dbg_gcm_tag_match),.dbg_gcm_tag_valid(dbg_gcm_tag_valid),
        .dbg_ht_state(dbg_ht_state), .dbg_ht_op(dbg_ht_op),
        .dbg_ht_done(dbg_ht_done),
        .dbg_ht_latch_lookup(dbg_ht_latch_lookup),
        .dbg_ht_latch_insert(dbg_ht_latch_insert),
        .dbg_ht_latch_delete(dbg_ht_latch_delete),
        .dbg_ht_sng_rd_req(dbg_ht_sng_rd_req),
        .dbg_ht_sng_wr_req(dbg_ht_sng_wr_req),
        .dbg_ht_sng_done(dbg_ht_sng_done),
        .dbg_force_same_bucket(dbg_force_same_bucket),
        .dbg_prng_override(dbg_prng_override),
        .dbg_prng_override_en(dbg_prng_override_en),
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
        .perf_ws_total(perf_ws_total), .perf_ws_ops(perf_ws_ops),
        // pos_map AXI master -> mux
        .pm_axi_arid(pm_arid), .pm_axi_araddr(pm_araddr),
        .pm_axi_arlen(pm_arlen), .pm_axi_arsize(pm_arsize),
        .pm_axi_arburst(pm_arburst), .pm_axi_arvalid(pm_arvalid),
        .pm_axi_arready(pm_arready),
        .pm_axi_rid(m_axi_rid), .pm_axi_rdata(m_axi_rdata),
        .pm_axi_rresp(m_axi_rresp), .pm_axi_rlast(m_axi_rlast),
        .pm_axi_rvalid(m_axi_rvalid), .pm_axi_rready(pm_rready),
        .pm_axi_awid(pm_awid), .pm_axi_awaddr(pm_awaddr),
        .pm_axi_awlen(pm_awlen), .pm_axi_awsize(pm_awsize),
        .pm_axi_awburst(pm_awburst), .pm_axi_awvalid(pm_awvalid),
        .pm_axi_awready(pm_awready),
        .pm_axi_wdata(pm_wdata), .pm_axi_wstrb(pm_wstrb),
        .pm_axi_wlast(pm_wlast), .pm_axi_wvalid(pm_wvalid),
        .pm_axi_wready(pm_wready),
        .pm_axi_bid(m_axi_bid), .pm_axi_bresp(m_axi_bresp),
        .pm_axi_bvalid(m_axi_bvalid), .pm_axi_bready(pm_bready),
        .pm_busy(pm_busy_w),
        .pm_dbg_state(dbg_pm_state)
    );

    // =========================================================================
    // Internal AXI wires: pos_map master
    // =========================================================================
    wire [AXI_IDW-1:0]  pm_arid, pm_awid;
    wire [AXI_AW-1:0]   pm_araddr, pm_awaddr;
    wire [AXI_LENW-1:0] pm_arlen, pm_awlen;
    wire [2:0]           pm_arsize, pm_awsize;
    wire [1:0]           pm_arburst, pm_awburst;
    wire                 pm_arvalid, pm_awvalid;
    wire                 pm_arready, pm_awready;
    wire                 pm_rready;
    wire [AXI_DW-1:0]   pm_wdata;
    wire [AXI_SW-1:0]   pm_wstrb;
    wire                 pm_wlast, pm_wvalid, pm_wready;
    wire                 pm_bready;
    wire                 pm_busy_w;

    // =========================================================================
    // 3-way AXI Mux: pos_map (priority) > stash > bucket (default)
    // =========================================================================
    wire sel_posmap = pm_busy_w;
    // sel_stash_fsm: route mux to stash for most states, EXCEPT:
    //   - S_POS_LOOKUP(1): posmap AXI
    //   - S_DDR_READ(2): bucket burst read
    //   - S_DDR_WRITE(19): bucket burst write
    //   - S_IDLE(0) when st_ext_busy=0: no stash op pending, bucket
    //     BRESPs need sel_stash=0 so bkt_bready routes through.
    //     When st_ext_busy=1 at S_IDLE, a stash write's BRESP is
    //     pending — sel_stash=1 to deliver it.
    wire sel_stash_fsm =
        (oram_fsm_state == 5'd0)  ? st_ext_busy :  // S_IDLE: conditional
        (oram_fsm_state != 5'd1)  &&               // S_POS_LOOKUP
        (oram_fsm_state != 5'd2)  &&               // S_DDR_READ
        (oram_fsm_state != 5'd19);                  // S_DDR_WRITE
    wire sel_stash  = !sel_posmap && sel_stash_fsm;
    assign sel_stash_out  = sel_stash;
    assign sel_posmap_out = sel_posmap;
    assign st_burst_busy_out = st_burst_busy;
    assign st_ext_busy_out = st_ext_busy;
    assign pm_busy_out = pm_busy_w;

    // Debug: expose pos_map AXI internals for SimObject probing
    assign dbg_pm_awvalid  = pm_awvalid;
    assign dbg_pm_wvalid   = pm_wvalid;
    assign dbg_pm_arvalid  = pm_arvalid;
    assign dbg_pm_bready   = pm_bready;
    assign dbg_mux_awvalid = m_axi_awvalid;  // final muxed output

    // AR channel
    assign m_axi_arid    = sel_posmap ? pm_arid    : sel_stash ? st_arid    : bkt_arid;
    assign m_axi_araddr  = sel_posmap ? pm_araddr  : sel_stash ? st_araddr  : bkt_araddr;
    assign m_axi_arlen   = sel_posmap ? pm_arlen   : sel_stash ? st_arlen   : bkt_arlen;
    assign m_axi_arsize  = sel_posmap ? pm_arsize  : sel_stash ? st_arsize  : bkt_arsize;
    assign m_axi_arburst = sel_posmap ? pm_arburst : sel_stash ? st_arburst : bkt_arburst;
    assign m_axi_arvalid = sel_posmap ? pm_arvalid : sel_stash ? st_arvalid : bkt_arvalid;
    assign pm_arready    = sel_posmap ? m_axi_arready : 1'b0;
    assign st_arready    = sel_stash  ? m_axi_arready : 1'b0;
    assign bkt_arready   = (!sel_posmap && !sel_stash) ? m_axi_arready : 1'b0;

    // R channel
    assign m_axi_rready  = sel_posmap ? pm_rready  : sel_stash ? st_rready  : bkt_rready;

    // AW channel
    assign m_axi_awid    = sel_posmap ? pm_awid    : sel_stash ? st_awid    : bkt_awid;
    assign m_axi_awaddr  = sel_posmap ? pm_awaddr  : sel_stash ? st_awaddr  : bkt_awaddr;
    assign m_axi_awlen   = sel_posmap ? pm_awlen   : sel_stash ? st_awlen   : bkt_awlen;
    assign m_axi_awsize  = sel_posmap ? pm_awsize  : sel_stash ? st_awsize  : bkt_awsize;
    assign m_axi_awburst = sel_posmap ? pm_awburst : sel_stash ? st_awburst : bkt_awburst;
    assign m_axi_awvalid = sel_posmap ? pm_awvalid : sel_stash ? st_awvalid : bkt_awvalid;
    assign pm_awready    = sel_posmap ? m_axi_awready : 1'b0;
    assign st_awready    = sel_stash  ? m_axi_awready : 1'b0;
    assign bkt_awready   = (!sel_posmap && !sel_stash) ? m_axi_awready : 1'b0;

    // W channel
    assign m_axi_wdata   = sel_posmap ? pm_wdata   : sel_stash ? st_wdata   : bkt_wdata;
    assign m_axi_wstrb   = sel_posmap ? pm_wstrb   : sel_stash ? st_wstrb   : bkt_wstrb;
    assign m_axi_wlast   = sel_posmap ? pm_wlast   : sel_stash ? st_wlast   : bkt_wlast;
    assign m_axi_wvalid  = sel_posmap ? pm_wvalid  : sel_stash ? st_wvalid  : bkt_wvalid;
    assign pm_wready     = sel_posmap ? m_axi_wready : 1'b0;
    assign st_wready     = sel_stash  ? m_axi_wready : 1'b0;
    assign bkt_wready    = (!sel_posmap && !sel_stash) ? m_axi_wready : 1'b0;

    // B channel
    assign m_axi_bready  = sel_posmap ? pm_bready  : sel_stash ? st_bready  : bkt_bready;

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