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
    output wire [5:0] dbg_oram_state,
    output wire       dbg_sel_slotr,
    output wire       dbg_sel_bmeta,
    output wire       dbg_sel_ivt,
    output wire       dbg_sel_posmap,
    output wire       dbg_sel_stash_fsm,
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
    output wire [`BUCKET_ID_W-1:0] dbg_req_b,
    output wire [`BUCKET_ID_W-1:0] dbg_req_b_new,
    output wire        dbg_same_bucket,
    output wire        dbg_err_stash_ovf,
    output wire [PTR_W:0] dbg_stash_occ,
    output wire        dbg_ht_ins_overwritten,
    output wire        dbg_ht_del_overwritten,
    output wire [7:0]  dbg_ht_ins_issued,
    output wire [7:0]  dbg_ht_ins_completed,
    output wire [7:0]  dbg_ht_del_issued,
    output wire [7:0]  dbg_ht_del_completed,
    output wire        dbg_ht_latch_ins_active,
    output wire        dbg_ht_latch_del_active,
    output wire [`AXI_ADDR_W-1:0] dbg_ht_lu_hbm_addr,
    output wire [3:0]  dbg_ht_lu_valid_bits,
    output wire        dbg_ht_lu_wb_hit,
    output wire [`SLOT_ADDR_W-1:0] dbg_ht_lu_slot_looked_up,

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
    wire [PTR_W-1:0]        st_ext_lb_entry;

    // Single-beat command wires (hash table access)
    // Gate HT single-beat commands: WHITELIST of safe states where
    // NO other AXI master is active. Any state with bucket, stash, or
    // pos_map AXI activity is unsafe (mux cross-contamination).
    wire                            st_sng_rd_req_raw;
    wire                            st_sng_wr_req_raw;
    wire [5:0]                      oram_fsm_state = dbg_oram_state;
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
    wire ht_safe = (oram_fsm_state != 6'd1)   // S_POS_LOOKUP
                && (oram_fsm_state != 6'd2)   // S_DDR_READ
                && (oram_fsm_state != 6'd11)  // S_STASH_SEARCH
                && (oram_fsm_state != 6'd12)  // S_STASH_READ
                && (oram_fsm_state != 6'd19)  // S_DDR_WRITE
                && (oram_fsm_state != 6'd28)  // S_ST_LOAD
                && (oram_fsm_state != 6'd29); // S_ST_FLUSH
    wire                            st_sng_rd_req = st_sng_rd_req_raw && ht_safe;
    wire                            st_sng_wr_req = st_sng_wr_req_raw && ht_safe;
    wire [`AXI_ADDR_W-1:0]         st_sng_addr;
    wire [`AXI_DATA_W-1:0]         st_sng_wdata;
    wire [`AXI_DATA_W-1:0]         st_sng_rdata;
    wire                            st_sng_done;
    wire                            st_sng_accepted;
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
        .cmd_single_accepted(st_sng_accepted),
        .lb_wr_data(st_ext_lb_wr_data),
        .lb_wr_addr(st_ext_lb_wr_addr),
        .lb_wr_en(st_ext_lb_wr_en),
        .lb_entry(st_ext_lb_entry),
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
        .m_axi_rvalid(st_rvalid_gated), .m_axi_rready(st_rready),
        .m_axi_awid(st_awid), .m_axi_awaddr(st_awaddr),
        .m_axi_awlen(st_awlen), .m_axi_awsize(st_awsize),
        .m_axi_awburst(st_awburst), .m_axi_awvalid(st_awvalid),
        .m_axi_awready(st_awready),
        .m_axi_wdata(st_wdata), .m_axi_wstrb(st_wstrb),
        .m_axi_wlast(st_wlast), .m_axi_wvalid(st_wvalid),
        .m_axi_wready(st_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(st_bvalid_gated), .m_axi_bready(st_bready)
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
        .m_axi_rvalid(bkt_rvalid_gated), .m_axi_rready(bkt_rready),
        .m_axi_awid(bkt_awid), .m_axi_awaddr(bkt_awaddr),
        .m_axi_awlen(bkt_awlen), .m_axi_awsize(bkt_awsize),
        .m_axi_awburst(bkt_awburst), .m_axi_awvalid(bkt_awvalid),
        .m_axi_awready(bkt_awready),
        .m_axi_wdata(bkt_wdata), .m_axi_wstrb(bkt_wstrb),
        .m_axi_wlast(bkt_wlast), .m_axi_wvalid(bkt_wvalid),
        .m_axi_wready(bkt_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp),
        .m_axi_bvalid(bkt_bvalid_gated), .m_axi_bready(bkt_bready),
        // Stash external memory interface -> stash_axi_master
        .st_ext_rd_req(st_ext_rd_req),
        .st_ext_wr_req(st_ext_wr_req),
        .st_ext_entry(st_ext_entry),
        .st_ext_rd_done(st_ext_rd_done),
        .st_ext_wr_done(st_ext_wr_done),
        .st_ext_busy(st_ext_busy),
        .metadata_idle(metadata_idle),
        .st_ext_lb_entry(st_ext_lb_entry),
        .st_burst_busy(st_burst_busy),
        // Single-beat hash table access
        .st_sng_rd_req(st_sng_rd_req_raw),
        .st_sng_wr_req(st_sng_wr_req_raw),
        .st_sng_addr(st_sng_addr),
        .st_sng_wdata(st_sng_wdata),
        .st_sng_rdata(st_sng_rdata),
        .st_sng_done(st_sng_done),
        .st_sng_accepted(st_sng_accepted),
        .st_ext_lb_wr_data(st_ext_lb_wr_data),
        .st_ext_lb_wr_addr(st_ext_lb_wr_addr),
        .st_ext_lb_wr_en(st_ext_lb_wr_en),
        .st_ext_lb_rd_addr(st_ext_lb_rd_addr),
        .st_ext_lb_rd_en(st_ext_lb_rd_en),
        .st_ext_lb_rd_data(st_ext_lb_rd_data),
        .st_ext_lb_rd_valid(st_ext_lb_rd_valid),
        // Debug
        .dbg_state(dbg_oram_state), .dbg_client_done(), .dbg_client_req(),
        .dbg_req_b(dbg_req_b), .dbg_req_b_new(dbg_req_b_new),
        .dbg_found_in_bucket(dbg_found_in_bucket), .dbg_found_in_stash(dbg_found_in_stash),
        .dbg_same_bucket(dbg_same_bucket), .dbg_err_stash_ovf(dbg_err_stash_ovf), .dbg_stash_occ(dbg_stash_occ),
        .dbg_ht_ins_overwritten(dbg_ht_ins_overwritten), .dbg_ht_del_overwritten(dbg_ht_del_overwritten),
        .dbg_ht_ins_issued(dbg_ht_ins_issued), .dbg_ht_ins_completed(dbg_ht_ins_completed),
        .dbg_ht_del_issued(dbg_ht_del_issued), .dbg_ht_del_completed(dbg_ht_del_completed),
        .dbg_ht_latch_ins_active(dbg_ht_latch_ins_active), .dbg_ht_latch_del_active(dbg_ht_latch_del_active),
        .dbg_ht_lu_hbm_addr(dbg_ht_lu_hbm_addr), .dbg_ht_lu_valid_bits(dbg_ht_lu_valid_bits),
        .dbg_ht_lu_wb_hit(dbg_ht_lu_wb_hit), .dbg_ht_lu_slot_looked_up(dbg_ht_lu_slot_looked_up),
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
        .pm_axi_rvalid(pm_rvalid_gated), .pm_axi_rready(pm_rready),
        .pm_axi_awid(pm_awid), .pm_axi_awaddr(pm_awaddr),
        .pm_axi_awlen(pm_awlen), .pm_axi_awsize(pm_awsize),
        .pm_axi_awburst(pm_awburst), .pm_axi_awvalid(pm_awvalid),
        .pm_axi_awready(pm_awready),
        .pm_axi_wdata(pm_wdata), .pm_axi_wstrb(pm_wstrb),
        .pm_axi_wlast(pm_wlast), .pm_axi_wvalid(pm_wvalid),
        .pm_axi_wready(pm_wready),
        .pm_axi_bid(m_axi_bid), .pm_axi_bresp(m_axi_bresp),
        .pm_axi_bvalid(pm_bvalid_gated), .pm_axi_bready(pm_bready),
        .pm_busy(pm_busy_w),
        .pm_dbg_state(dbg_pm_state),
        // IV/TAG master -> mux
        .ivt_axi_arid(ivt_arid), .ivt_axi_araddr(ivt_araddr),
        .ivt_axi_arlen(ivt_arlen), .ivt_axi_arsize(ivt_arsize),
        .ivt_axi_arburst(ivt_arburst), .ivt_axi_arvalid(ivt_arvalid),
        .ivt_axi_arready(ivt_arready),
        .ivt_axi_rid(m_axi_rid), .ivt_axi_rdata(m_axi_rdata),
        .ivt_axi_rresp(m_axi_rresp), .ivt_axi_rlast(m_axi_rlast),
        .ivt_axi_rvalid(ivt_rvalid_gated), .ivt_axi_rready(ivt_rready),
        .ivt_axi_awid(ivt_awid), .ivt_axi_awaddr(ivt_awaddr),
        .ivt_axi_awlen(ivt_awlen), .ivt_axi_awsize(ivt_awsize),
        .ivt_axi_awburst(ivt_awburst), .ivt_axi_awvalid(ivt_awvalid),
        .ivt_axi_awready(ivt_awready),
        .ivt_axi_wdata(ivt_wdata), .ivt_axi_wstrb(ivt_wstrb),
        .ivt_axi_wlast(ivt_wlast), .ivt_axi_wvalid(ivt_wvalid),
        .ivt_axi_wready(ivt_wready),
        .ivt_axi_bid(m_axi_bid), .ivt_axi_bresp(m_axi_bresp),
        .ivt_axi_bvalid(ivt_bvalid_gated), .ivt_axi_bready(ivt_bready),
        .ivt_busy(ivt_busy_w),
        .ivt_dbg_state(dbg_ivt_state),
        // slot_r master -> mux
        .sr_axi_arid(sr_arid), .sr_axi_araddr(sr_araddr),
        .sr_axi_arlen(sr_arlen), .sr_axi_arsize(sr_arsize),
        .sr_axi_arburst(sr_arburst), .sr_axi_arvalid(sr_arvalid),
        .sr_axi_arready(sr_arready),
        .sr_axi_rid(m_axi_rid), .sr_axi_rdata(m_axi_rdata),
        .sr_axi_rresp(m_axi_rresp), .sr_axi_rlast(m_axi_rlast),
        .sr_axi_rvalid(sr_rvalid_gated), .sr_axi_rready(sr_rready),
        .sr_axi_awid(sr_awid), .sr_axi_awaddr(sr_awaddr),
        .sr_axi_awlen(sr_awlen), .sr_axi_awsize(sr_awsize),
        .sr_axi_awburst(sr_awburst), .sr_axi_awvalid(sr_awvalid),
        .sr_axi_awready(sr_awready),
        .sr_axi_wdata(sr_wdata), .sr_axi_wstrb(sr_wstrb),
        .sr_axi_wlast(sr_wlast), .sr_axi_wvalid(sr_wvalid),
        .sr_axi_wready(sr_wready),
        .sr_axi_bid(m_axi_bid), .sr_axi_bresp(m_axi_bresp),
        .sr_axi_bvalid(sr_bvalid_gated), .sr_axi_bready(sr_bready),
        .slotr_busy(slotr_busy_w),
        .slotr_dbg_state(dbg_slotr_state),
        // bucket_meta master -> mux
        .bm_axi_arid(bm_arid), .bm_axi_araddr(bm_araddr),
        .bm_axi_arlen(bm_arlen), .bm_axi_arsize(bm_arsize),
        .bm_axi_arburst(bm_arburst), .bm_axi_arvalid(bm_arvalid),
        .bm_axi_arready(bm_arready),
        .bm_axi_rid(m_axi_rid), .bm_axi_rdata(m_axi_rdata),
        .bm_axi_rresp(m_axi_rresp), .bm_axi_rlast(m_axi_rlast),
        .bm_axi_rvalid(bm_rvalid_gated), .bm_axi_rready(bm_rready),
        .bm_axi_awid(bm_awid), .bm_axi_awaddr(bm_awaddr),
        .bm_axi_awlen(bm_awlen), .bm_axi_awsize(bm_awsize),
        .bm_axi_awburst(bm_awburst), .bm_axi_awvalid(bm_awvalid),
        .bm_axi_awready(bm_awready),
        .bm_axi_wdata(bm_wdata), .bm_axi_wstrb(bm_wstrb),
        .bm_axi_wlast(bm_wlast), .bm_axi_wvalid(bm_wvalid),
        .bm_axi_wready(bm_wready),
        .bm_axi_bid(m_axi_bid), .bm_axi_bresp(m_axi_bresp),
        .bm_axi_bvalid(bm_bvalid_gated), .bm_axi_bready(bm_bready),
        .bmeta_busy(bmeta_busy_w),
        .bmeta_dbg_state(dbg_bmeta_state)
    );

    // =========================================================================
    // Internal AXI wires: IV/TAG master
    // =========================================================================
    wire [AXI_IDW-1:0]  ivt_arid, ivt_awid;
    wire [AXI_AW-1:0]   ivt_araddr, ivt_awaddr;
    wire [AXI_LENW-1:0] ivt_arlen, ivt_awlen;
    wire [2:0]           ivt_arsize, ivt_awsize;
    wire [1:0]           ivt_arburst, ivt_awburst;
    wire                 ivt_arvalid, ivt_awvalid;
    wire                 ivt_arready, ivt_awready;
    wire                 ivt_rready;
    wire [AXI_DW-1:0]   ivt_wdata;
    wire [AXI_SW-1:0]   ivt_wstrb;
    wire                 ivt_wlast, ivt_wvalid, ivt_wready;
    wire                 ivt_bready;
    wire                 ivt_busy_w;
    wire [2:0]           dbg_ivt_state;

    // =========================================================================
    // Internal AXI wires: slot_r master
    // =========================================================================
    wire [AXI_IDW-1:0]  sr_arid, sr_awid;
    wire [AXI_AW-1:0]   sr_araddr, sr_awaddr;
    wire [AXI_LENW-1:0] sr_arlen, sr_awlen;
    wire [2:0]           sr_arsize, sr_awsize;
    wire [1:0]           sr_arburst, sr_awburst;
    wire                 sr_arvalid, sr_awvalid;
    wire                 sr_arready, sr_awready;
    wire                 sr_rready;
    wire [AXI_DW-1:0]   sr_wdata;
    wire [AXI_SW-1:0]   sr_wstrb;
    wire                 sr_wlast, sr_wvalid, sr_wready;
    wire                 sr_bready;
    wire                 slotr_busy_w;
    wire [2:0]           dbg_slotr_state;

    // =========================================================================
    // Internal AXI wires: bucket_meta master
    // =========================================================================
    wire [AXI_IDW-1:0]  bm_arid, bm_awid;
    wire [AXI_AW-1:0]   bm_araddr, bm_awaddr;
    wire [AXI_LENW-1:0] bm_arlen, bm_awlen;
    wire [2:0]           bm_arsize, bm_awsize;
    wire [1:0]           bm_arburst, bm_awburst;
    wire                 bm_arvalid, bm_awvalid;
    wire                 bm_arready, bm_awready;
    wire                 bm_rready;
    wire [AXI_DW-1:0]   bm_wdata;
    wire [AXI_SW-1:0]   bm_wstrb;
    wire                 bm_wlast, bm_wvalid, bm_wready;
    wire                 bm_bready;
    wire                 bmeta_busy_w;
    wire [2:0]           dbg_bmeta_state;

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
    // 4-way AXI Mux: IV/TAG (priority) > pos_map > stash > bucket (default)
    // =========================================================================
    // IVT is busy-keyed and sits at TOP priority, for two reasons:
    //   1. IVT reads (S_EXT_IV_RD) block the main FSM on the decrypt critical
    //      path — they must get the bus immediately.
    //   2. The op-completion interlock (S_DDR_WRITE waits on !ivt_busy) means a
    //      queued IVT write must be able to drain *while the FSM sits in state
    //      19*. At that point the bucket master has already finished its burst
    //      (axim_write_done is high before we wait on ivt_busy) and is back at
    //      ST_IDLE — not driving AR/AW/W — so handing the bus to IVT does not
    //      interrupt any in-flight bucket transfer. Bucket's BRESP is already
    //      consumed by then, so stealing bready is safe.
    //
    // IVT and pos_map are never both mid-burst: pos_map runs only at
    // S_POS_LOOKUP / pm writes, and the FSM serializes IVT access against those
    // by construction. Top-priority IVT therefore cannot starve a live pos_map
    // burst — if both ever asserted, pos_map (also busy-keyed) simply holds its
    // request until IVT's single-beat access completes.
    // 6-way priority arbitration. slot_r and bucket_meta (new HBM metadata
    // masters) are inserted at the TOP, above IVT, without changing the relative
    // order of the existing four (IVT > posmap > stash > bucket). All three
    // metadata masters are busy-keyed and single-beat; the FSM serializes their
    // access by construction (each is a distinct read/write step the FSM waits
    // on or launches async). Mid-burst grant to a metadata master only PAUSES
    // the bucket burst (AXI wready-gated advance), never corrupts it.
    //
    // Priority rationale per concurrency state (verified against the FSM):
    //   S_POS_LOOKUP/S_DDR_READ: bucket_meta read + bucket data read + posmap
    //     write coexist. bucket_meta (top) and posmap (busy-keyed) take the bus
    //     for their single beats, pausing the bucket burst; FSM waits for both
    //     bm_rd_done and axim_read_done.
    //   S_EVICT_SLOTR_WAIT: slot_r read; nothing else on the bus -> slot_r wins.
    //   S_DDR_WRITE: bucket_meta + slot_r writes (async) + bucket data burst.
    //     Metadata writes pause the burst; op completion is held until all
    //     metadata busy clears (interlock in flat_oram_gcm S_DDR_WRITE).
    wire sel_slotr  = slotr_busy_w;
    wire sel_bmeta  = !sel_slotr && bmeta_busy_w;
    wire sel_ivt    = !sel_slotr && !sel_bmeta && ivt_busy_w;
    wire sel_posmap = !sel_slotr && !sel_bmeta && !sel_ivt && pm_busy_w;
    // sel_stash_fsm: route mux to stash for most states, EXCEPT:
    //   - S_POS_LOOKUP(1): posmap AXI
    //   - S_DDR_READ(2): bucket burst read
    //   - S_DDR_WRITE(19): bucket burst write
    //   - S_IDLE(0) when st_ext_busy=0: no stash op pending, bucket
    //     BRESPs need sel_stash=0 so bkt_bready routes through.
    //     When st_ext_busy=1 at S_IDLE, a stash write's BRESP is
    //     pending — sel_stash=1 to deliver it.
    wire sel_stash_fsm =
        (oram_fsm_state == 6'd0)  ? st_ext_busy :  // S_IDLE: conditional
        (oram_fsm_state != 6'd1)  &&               // S_POS_LOOKUP
        (oram_fsm_state != 6'd2)  &&               // S_DDR_READ
        (oram_fsm_state != 6'd19);                  // S_DDR_WRITE
    wire sel_stash  = !sel_slotr && !sel_bmeta && !sel_ivt && !sel_posmap && sel_stash_fsm;
    wire sel_bucket = !sel_slotr && !sel_bmeta && !sel_ivt && !sel_posmap && !sel_stash;
    assign sel_stash_out  = sel_stash;
    assign dbg_sel_slotr     = sel_slotr;
    assign dbg_sel_bmeta     = sel_bmeta;
    assign dbg_sel_ivt       = sel_ivt;
    assign dbg_sel_posmap    = sel_posmap;
    assign dbg_sel_stash_fsm = sel_stash_fsm;

    // Metadata-idle flag: when all metadata masters are idle, the stash
    // can safely issue burst reads/writes without mux conflicts.
    wire metadata_idle = !sel_slotr && !sel_bmeta && !sel_ivt && !sel_posmap;
    assign sel_posmap_out = sel_posmap;
    assign st_burst_busy_out = st_burst_busy;
    assign st_ext_busy_out = st_ext_busy;
    assign pm_busy_out = pm_busy_w;

    // =========================================================================
    // Debug: mux grant tracing + contention watchdog (sim only)
    // Logs every change in which master owns the AXI bus, and flags the
    // (by-design impossible) case of IVT and pos_map both requesting at once,
    // so any future FSM change that breaks the serialization assumption is
    // caught immediately rather than as a silent BRESP-routing corruption.
    // =========================================================================
    `ifndef SYNTHESIS
    reg [2:0] dbg_prev_grant;   // 0=bucket 1=stash 2=posmap 3=ivt 4=bmeta 5=slotr
    wire [2:0] dbg_grant = sel_slotr  ? 3'd5 :
                           sel_bmeta  ? 3'd4 :
                           sel_ivt    ? 3'd3 :
                           sel_posmap ? 3'd2 :
                           sel_stash  ? 3'd1 : 3'd0;
    always @(posedge clk) begin
        if (!rst_n) begin
            dbg_prev_grant <= 3'd0;
        end else begin
            if (dbg_grant != dbg_prev_grant) begin
                $display("[MUX] t=%0t grant %0d -> %0d (slotr=%b bmeta=%b ivt=%b pm=%b st_ext=%b fsm=%0d)",
                         $time, dbg_prev_grant, dbg_grant,
                         slotr_busy_w, bmeta_busy_w, ivt_busy_w, pm_busy_w,
                         st_ext_busy, oram_fsm_state);
                dbg_prev_grant <= dbg_grant;
            end
            // Watchdog: IVT and pos_map should never both be busy.
            if (ivt_busy_w && pm_busy_w)
                $display("[MUX] WARN t=%0t IVT and PM both busy (fsm=%0d) — serialization assumption violated!",
                         $time, oram_fsm_state);
            // NOTE: slot_r/bucket_meta granted during a bucket burst window
            // (S_DDR_READ=2, S_DDR_WRITE=19) is EXPECTED — they pause the burst
            // for a single beat. Logged for inspection, not an error.
            if ((sel_slotr || sel_bmeta) && (oram_fsm_state == 6'd2 || oram_fsm_state == 6'd19))
                $display("[MUX] NOTE t=%0t metadata master granted during bucket burst (fsm=%0d) — burst paused, will resume",
                         $time, oram_fsm_state);
        end
    end
    `endif

    // Debug: expose pos_map AXI internals for SimObject probing
    assign dbg_pm_awvalid  = pm_awvalid;
    assign dbg_pm_wvalid   = pm_wvalid;
    assign dbg_pm_arvalid  = pm_arvalid;
    assign dbg_pm_bready   = pm_bready;
    assign dbg_mux_awvalid = m_axi_awvalid;  // final muxed output

    // =====================================================================
    // AR-issuer tracking FIFO: records which module issued each AR so the
    // R-channel response is delivered ONLY to that module.  Fixes the bug
    // where all modules see m_axi_rvalid simultaneously and a module that
    // didn't issue the read can capture (and consume) someone else's data.
    // =====================================================================
    localparam RFIFO_DEPTH = 128;       // 7-bit pointers — generous for production
    localparam RFIFO_AW    = 7;
    reg  [2:0] rfifo_data [0:RFIFO_DEPTH-1]; // issuer tag per AR
    reg  [RFIFO_AW-1:0] rfifo_wr, rfifo_rd;
    wire [RFIFO_AW-1:0] rfifo_count = rfifo_wr - rfifo_rd;
    wire rfifo_empty = (rfifo_wr == rfifo_rd);

    // Issuer encoding at AR-handshake time
    wire [2:0] ar_issuer = sel_slotr  ? 3'd5 :
                            sel_bmeta  ? 3'd4 :
                            sel_ivt    ? 3'd3 :
                            sel_posmap ? 3'd2 :
                            sel_stash  ? 3'd1 : 3'd0;

    wire ar_fire = m_axi_arvalid && m_axi_arready;
    wire r_fire  = m_axi_rvalid  && m_axi_rready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rfifo_wr <= 0;
            rfifo_rd <= 0;
        end else begin
            if (ar_fire) begin
                rfifo_data[rfifo_wr] <= ar_issuer;
                rfifo_wr <= rfifo_wr + 1;
                `ifndef SYNTHESIS
                if (rfifo_count == RFIFO_DEPTH-1)
                    $display("[RFIFO] ERROR t=%0t AR-issuer FIFO overflow! wr=%0d rd=%0d",
                             $time, rfifo_wr, rfifo_rd);
                `endif
            end
            if (r_fire && m_axi_rlast) begin
                rfifo_rd <= rfifo_rd + 1;
            end
        end
    end

    // Response target: who should receive the current R beat
    wire [2:0] resp_tgt  = rfifo_empty ? 3'd0 : rfifo_data[rfifo_rd];
    wire resp_for_bkt    = (resp_tgt == 3'd0);
    wire resp_for_stash  = (resp_tgt == 3'd1);
    wire resp_for_posmap = (resp_tgt == 3'd2);
    wire resp_for_ivt    = (resp_tgt == 3'd3);
    wire resp_for_bmeta  = (resp_tgt == 3'd4);
    wire resp_for_slotr  = (resp_tgt == 3'd5);

    // Gated rvalid per module
    wire st_rvalid_gated  = m_axi_rvalid && resp_for_stash;
    wire bkt_rvalid_gated = m_axi_rvalid && resp_for_bkt;
    wire pm_rvalid_gated  = m_axi_rvalid && resp_for_posmap;
    wire ivt_rvalid_gated = m_axi_rvalid && resp_for_ivt;
    wire bm_rvalid_gated  = m_axi_rvalid && resp_for_bmeta;
    wire sr_rvalid_gated  = m_axi_rvalid && resp_for_slotr;

    // =====================================================================
    // AW-issuer tracking FIFO: same pattern for B (write-response) channel
    // =====================================================================
    reg  [2:0] bfifo_data [0:RFIFO_DEPTH-1];
    reg  [RFIFO_AW-1:0] bfifo_wr, bfifo_rd;
    wire bfifo_empty = (bfifo_wr == bfifo_rd);

    wire [2:0] aw_issuer = sel_slotr  ? 3'd5 :
                            sel_bmeta  ? 3'd4 :
                            sel_ivt    ? 3'd3 :
                            sel_posmap ? 3'd2 :
                            sel_stash  ? 3'd1 : 3'd0;

    wire aw_fire = m_axi_awvalid && m_axi_awready;
    wire b_fire  = m_axi_bvalid  && m_axi_bready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bfifo_wr <= 0;
            bfifo_rd <= 0;
        end else begin
            if (aw_fire) begin
                bfifo_data[bfifo_wr] <= aw_issuer;
                bfifo_wr <= bfifo_wr + 1;
                `ifndef SYNTHESIS
                if ((bfifo_wr - bfifo_rd) == RFIFO_DEPTH-1)
                    $display("[BFIFO] ERROR t=%0t AW-issuer FIFO overflow! wr=%0d rd=%0d",
                             $time, bfifo_wr, bfifo_rd);
                `endif
            end
            if (b_fire) begin
                bfifo_rd <= bfifo_rd + 1;
            end
        end
    end

    wire [2:0] bresp_tgt    = bfifo_empty ? 3'd0 : bfifo_data[bfifo_rd];
    wire bresp_for_bkt    = (bresp_tgt == 3'd0);
    wire bresp_for_stash  = (bresp_tgt == 3'd1);
    wire bresp_for_posmap = (bresp_tgt == 3'd2);
    wire bresp_for_ivt    = (bresp_tgt == 3'd3);
    wire bresp_for_bmeta  = (bresp_tgt == 3'd4);
    wire bresp_for_slotr  = (bresp_tgt == 3'd5);

    wire st_bvalid_gated  = m_axi_bvalid && bresp_for_stash;
    wire bkt_bvalid_gated = m_axi_bvalid && bresp_for_bkt;
    wire pm_bvalid_gated  = m_axi_bvalid && bresp_for_posmap;
    wire ivt_bvalid_gated = m_axi_bvalid && bresp_for_ivt;
    wire bm_bvalid_gated  = m_axi_bvalid && bresp_for_bmeta;
    wire sr_bvalid_gated  = m_axi_bvalid && bresp_for_slotr;

    // AR channel
    assign m_axi_arid    = sel_slotr ? sr_arid    : sel_bmeta ? bm_arid    : sel_ivt ? ivt_arid    : sel_posmap ? pm_arid    : sel_stash ? st_arid    : bkt_arid;
    assign m_axi_araddr  = sel_slotr ? sr_araddr  : sel_bmeta ? bm_araddr  : sel_ivt ? ivt_araddr  : sel_posmap ? pm_araddr  : sel_stash ? st_araddr  : bkt_araddr;
    assign m_axi_arlen   = sel_slotr ? sr_arlen   : sel_bmeta ? bm_arlen   : sel_ivt ? ivt_arlen   : sel_posmap ? pm_arlen   : sel_stash ? st_arlen   : bkt_arlen;
    assign m_axi_arsize  = sel_slotr ? sr_arsize  : sel_bmeta ? bm_arsize  : sel_ivt ? ivt_arsize  : sel_posmap ? pm_arsize  : sel_stash ? st_arsize  : bkt_arsize;
    assign m_axi_arburst = sel_slotr ? sr_arburst : sel_bmeta ? bm_arburst : sel_ivt ? ivt_arburst : sel_posmap ? pm_arburst : sel_stash ? st_arburst : bkt_arburst;
    assign m_axi_arvalid = sel_slotr ? sr_arvalid : sel_bmeta ? bm_arvalid : sel_ivt ? ivt_arvalid : sel_posmap ? pm_arvalid : sel_stash ? st_arvalid : bkt_arvalid;
    assign sr_arready    = sel_slotr  ? m_axi_arready : 1'b0;
    assign bm_arready    = sel_bmeta  ? m_axi_arready : 1'b0;
    assign ivt_arready   = sel_ivt    ? m_axi_arready : 1'b0;
    assign pm_arready    = sel_posmap ? m_axi_arready : 1'b0;
    assign st_arready    = sel_stash  ? m_axi_arready : 1'b0;
    assign bkt_arready   = sel_bucket ? m_axi_arready : 1'b0;

    // R channel — rready routed by response-issuer FIFO, not current sel_*
    assign m_axi_rready  = resp_for_slotr  ? sr_rready :
                            resp_for_bmeta  ? bm_rready :
                            resp_for_ivt    ? ivt_rready :
                            resp_for_posmap ? pm_rready :
                            resp_for_stash  ? st_rready : bkt_rready;

    // AW channel
    assign m_axi_awid    = sel_slotr ? sr_awid    : sel_bmeta ? bm_awid    : sel_ivt ? ivt_awid    : sel_posmap ? pm_awid    : sel_stash ? st_awid    : bkt_awid;
    assign m_axi_awaddr  = sel_slotr ? sr_awaddr  : sel_bmeta ? bm_awaddr  : sel_ivt ? ivt_awaddr  : sel_posmap ? pm_awaddr  : sel_stash ? st_awaddr  : bkt_awaddr;
    assign m_axi_awlen   = sel_slotr ? sr_awlen   : sel_bmeta ? bm_awlen   : sel_ivt ? ivt_awlen   : sel_posmap ? pm_awlen   : sel_stash ? st_awlen   : bkt_awlen;
    assign m_axi_awsize  = sel_slotr ? sr_awsize  : sel_bmeta ? bm_awsize  : sel_ivt ? ivt_awsize  : sel_posmap ? pm_awsize  : sel_stash ? st_awsize  : bkt_awsize;
    assign m_axi_awburst = sel_slotr ? sr_awburst : sel_bmeta ? bm_awburst : sel_ivt ? ivt_awburst : sel_posmap ? pm_awburst : sel_stash ? st_awburst : bkt_awburst;
    assign m_axi_awvalid = sel_slotr ? sr_awvalid : sel_bmeta ? bm_awvalid : sel_ivt ? ivt_awvalid : sel_posmap ? pm_awvalid : sel_stash ? st_awvalid : bkt_awvalid;
    assign sr_awready    = sel_slotr  ? m_axi_awready : 1'b0;
    assign bm_awready    = sel_bmeta  ? m_axi_awready : 1'b0;
    assign ivt_awready   = sel_ivt    ? m_axi_awready : 1'b0;
    assign pm_awready    = sel_posmap ? m_axi_awready : 1'b0;
    assign st_awready    = sel_stash  ? m_axi_awready : 1'b0;
    assign bkt_awready   = sel_bucket ? m_axi_awready : 1'b0;

    // W channel
    assign m_axi_wdata   = sel_slotr ? sr_wdata   : sel_bmeta ? bm_wdata   : sel_ivt ? ivt_wdata   : sel_posmap ? pm_wdata   : sel_stash ? st_wdata   : bkt_wdata;
    assign m_axi_wstrb   = sel_slotr ? sr_wstrb   : sel_bmeta ? bm_wstrb   : sel_ivt ? ivt_wstrb   : sel_posmap ? pm_wstrb   : sel_stash ? st_wstrb   : bkt_wstrb;
    assign m_axi_wlast   = sel_slotr ? sr_wlast   : sel_bmeta ? bm_wlast   : sel_ivt ? ivt_wlast   : sel_posmap ? pm_wlast   : sel_stash ? st_wlast   : bkt_wlast;
    assign m_axi_wvalid  = sel_slotr ? sr_wvalid  : sel_bmeta ? bm_wvalid  : sel_ivt ? ivt_wvalid  : sel_posmap ? pm_wvalid  : sel_stash ? st_wvalid  : bkt_wvalid;
    assign sr_wready     = sel_slotr  ? m_axi_wready : 1'b0;
    assign bm_wready     = sel_bmeta  ? m_axi_wready : 1'b0;
    assign ivt_wready    = sel_ivt    ? m_axi_wready : 1'b0;
    assign pm_wready     = sel_posmap ? m_axi_wready : 1'b0;
    assign st_wready     = sel_stash  ? m_axi_wready : 1'b0;
    assign bkt_wready    = sel_bucket ? m_axi_wready : 1'b0;

    // B channel
    // B channel — bready routed by AW-issuer FIFO
    assign m_axi_bready  = bresp_for_slotr  ? sr_bready :
                            bresp_for_bmeta  ? bm_bready :
                            bresp_for_ivt    ? ivt_bready :
                            bresp_for_posmap ? pm_bready :
                            bresp_for_stash  ? st_bready : bkt_bready;

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