`timescale 1ns / 1ps
`include "oram_params.vh"

// =============================================================================
// flat_oram_gcm.v - ORAM Top with AES-GCM Encryption
// Step 5: BRAM CAM removed — HT is sole authoritative metadata store
//         Stash CAM ports disconnected (tied to 0). Shadow counters removed.
//         stash.v still provides slot_r[] for hierarchical eviction access.
// =============================================================================

module flat_oram_gcm #(
    parameter N            = `ORAM_N,
    parameter Z            = `ORAM_Z,
    parameter C            = `ORAM_C,
    parameter B            = `ORAM_B,
    parameter STASH_DEPTH  = `ORAM_STASH_DEPTH,
    parameter AXI_DW       = `AXI_DATA_W,
    parameter AXI_AW       = `AXI_ADDR_W,
    parameter AXI_SW       = `AXI_STRB_W,
    parameter AXI_IDW      = `AXI_ID_W,
    parameter AXI_LENW     = `AXI_LEN_W,
    parameter SLOT_AW      = `SLOT_ADDR_W,
    parameter BUCKET_W     = `BUCKET_ID_W,
    parameter SLOT_W       = `SLOT_ID_W,
    parameter FILL_W       = `FILL_CNT_W,
    parameter POS_W        = `POS_IN_BKT_W,
    parameter PTR_W        = `STASH_PTR_W,
    parameter BEATS        = `BEATS_PER_BLOCK,
    parameter BEAT_W       = `BEAT_CNT_W,
    parameter TOTAL_BEATS  = `BEATS_PER_BKT,
    parameter AES_BLK_W    = 128
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Client Interface
    input  wire                 client_req,
    input  wire                 client_op,
    input  wire [SLOT_AW-1:0]  client_slot_addr,
    input  wire [AXI_DW-1:0]   client_wdata,
    input  wire                 client_wdata_valid,
    input  wire [BEAT_W-1:0]   client_wdata_beat,
    output wire [BEAT_W-1:0]   client_wdata_beat_req,
    output reg  [AXI_DW-1:0]   client_rdata,
    output reg                  client_rdata_valid,
    output reg  [BEAT_W-1:0]   client_rdata_beat,
    output reg                  client_done,
    output reg                  client_stall,

    output reg                  err_bucket_overflow,
    output reg                  err_stash_overflow,
    output reg                  err_tag_mismatch,

    // BRAM Init Interface
    input  wire                     init_mode,
    input  wire [SLOT_AW-1:0]      init_pm_wr_addr,
    input  wire [BUCKET_W-1:0]     init_pm_wr_bucket,
    input  wire [1:0]              init_pm_wr_status,
    input  wire                     init_pm_wr_en,
    input  wire [BUCKET_W-1:0]     init_bm_wr_bucket,
    input  wire [Z*SLOT_W-1:0]    init_bm_wr_slot_list,
    input  wire [FILL_W-1:0]      init_bm_wr_fill,
    input  wire                     init_bm_wr_en,

    // AES-GCM Key
    input  wire [127:0]            aes_key,
    input  wire [95:0]             aes_iv_seed,

    // AXI4 Master -> DDR
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

    // Debug
    output wire dbg_gcm_tag_valid,

    // Test controls
    input  wire                  dbg_force_same_bucket,  // override req_b_new = req_b
    input  wire [BUCKET_W-1:0]  dbg_prng_override,      // when != 0, replaces prng_bucket
    input  wire                  dbg_prng_override_en,   // separate enable (allows bucket 0)
    input  wire                  perf_reset,              // clear all perf counters
    output wire dbg_gcm_tag_match,
    output wire [4:0]            dbg_state,
    output wire                  dbg_client_done,
    output wire                  dbg_client_req,
    output wire [BUCKET_W-1:0]  dbg_req_b,
    output wire [BUCKET_W-1:0]  dbg_req_b_new,
    output wire                  dbg_found_in_bucket,
    output wire                  dbg_found_in_stash,
    output wire                  dbg_same_bucket,
    output wire                  dbg_err_stash_ovf,
    output wire [PTR_W:0]       dbg_stash_occ,

    // HT FSM debug
    output wire [2:0]            dbg_ht_state,
    output wire [2:0]            dbg_ht_op,
    output wire                  dbg_ht_done,
    output wire                  dbg_ht_latch_lookup,
    output wire                  dbg_ht_latch_insert,
    output wire                  dbg_ht_latch_delete,
    output wire                  dbg_ht_sng_rd_req,
    output wire                  dbg_ht_sng_wr_req,
    output wire                  dbg_ht_sng_done,

    // Stash external memory interface (to stash_axi_master via secure_oram_top)
    output reg                      st_ext_rd_req,
    output reg                      st_ext_wr_req,
    output reg  [PTR_W-1:0]        st_ext_entry,
    input  wire                     st_ext_rd_done,
    input  wire                     st_ext_wr_done,
    input  wire                     st_ext_busy,
    input  wire                     st_burst_busy,

    // Single-beat hash table access (via stash_axi_master)
    output reg                      st_sng_rd_req,
    output reg                      st_sng_wr_req,
    output reg  [AXI_AW-1:0]       st_sng_addr,
    output reg  [AXI_DW-1:0]       st_sng_wdata,
    input  wire [AXI_DW-1:0]       st_sng_rdata,
    input  wire                     st_sng_done,

    // Stash line buffer external ports (wired through to stash_axi_master)
    input  wire [AXI_DW-1:0]       st_ext_lb_wr_data,
    input  wire [6:0]              st_ext_lb_wr_addr,
    input  wire                     st_ext_lb_wr_en,
    input  wire [6:0]              st_ext_lb_rd_addr,
    input  wire                     st_ext_lb_rd_en,
    output wire [AXI_DW-1:0]       st_ext_lb_rd_data,
    output wire                     st_ext_lb_rd_valid,

    // pos_map AXI4 master (to secure_oram_top 3-way mux -> HBM)
    output wire [`AXI_ID_W-1:0]    pm_axi_arid,
    output wire [`AXI_ADDR_W-1:0]  pm_axi_araddr,
    output wire [`AXI_LEN_W-1:0]   pm_axi_arlen,
    output wire [2:0]               pm_axi_arsize,
    output wire [1:0]               pm_axi_arburst,
    output wire                     pm_axi_arvalid,
    input  wire                     pm_axi_arready,
    input  wire [`AXI_ID_W-1:0]    pm_axi_rid,
    input  wire [`AXI_DATA_W-1:0]  pm_axi_rdata,
    input  wire [1:0]               pm_axi_rresp,
    input  wire                     pm_axi_rlast,
    input  wire                     pm_axi_rvalid,
    output wire                     pm_axi_rready,
    output wire [`AXI_ID_W-1:0]    pm_axi_awid,
    output wire [`AXI_ADDR_W-1:0]  pm_axi_awaddr,
    output wire [`AXI_LEN_W-1:0]   pm_axi_awlen,
    output wire [2:0]               pm_axi_awsize,
    output wire [1:0]               pm_axi_awburst,
    output wire                     pm_axi_awvalid,
    input  wire                     pm_axi_awready,
    output wire [`AXI_DATA_W-1:0]  pm_axi_wdata,
    output wire [`AXI_STRB_W-1:0]  pm_axi_wstrb,
    output wire                     pm_axi_wlast,
    output wire                     pm_axi_wvalid,
    input  wire                     pm_axi_wready,
    input  wire [`AXI_ID_W-1:0]    pm_axi_bid,
    input  wire [1:0]               pm_axi_bresp,
    input  wire                     pm_axi_bvalid,
    output wire                     pm_axi_bready,
    output wire                     pm_busy,
    output wire [2:0]               pm_dbg_state,

    // Performance counters
    output reg  [31:0]  perf_rb_ddr_rd, perf_rb_decrypt, perf_rb_encrypt,
    output reg  [31:0]  perf_rb_compact, perf_rb_evict, perf_rb_ddr_wr,
    output reg  [31:0]  perf_rb_scan, perf_rb_stash,
    output reg  [31:0]  perf_rb_total, perf_rb_ops,

    output reg  [31:0]  perf_rs_ddr_rd, perf_rs_decrypt, perf_rs_encrypt,
    output reg  [31:0]  perf_rs_compact, perf_rs_evict, perf_rs_ddr_wr,
    output reg  [31:0]  perf_rs_scan, perf_rs_stash,
    output reg  [31:0]  perf_rs_total, perf_rs_ops,

    output reg  [31:0]  perf_wb_ddr_rd, perf_wb_decrypt, perf_wb_encrypt,
    output reg  [31:0]  perf_wb_compact, perf_wb_evict, perf_wb_ddr_wr,
    output reg  [31:0]  perf_wb_scan, perf_wb_stash,
    output reg  [31:0]  perf_wb_total, perf_wb_ops,

    output reg  [31:0]  perf_ws_ddr_rd, perf_ws_decrypt, perf_ws_encrypt,
    output reg  [31:0]  perf_ws_compact, perf_ws_evict, perf_ws_ddr_wr,
    output reg  [31:0]  perf_ws_scan, perf_ws_stash,
    output reg  [31:0]  perf_ws_total, perf_ws_ops
);

    // =========================================================================
    // FSM States  (Step 4: added S_HT_LOOKUP_WAIT = 31)
    // =========================================================================
    localparam [4:0]
        S_IDLE          = 5'd0,
        S_POS_LOOKUP    = 5'd1,
        S_DDR_READ      = 5'd2,
        S_SCAN          = 5'd3,
        S_SCAN2         = 5'd4,
        S_EXT_IV_RD     = 5'd5,
        S_EXT_DEC_START = 5'd6,
        S_EXT_DEC_FEED  = 5'd7,
        S_EXT_DEC_RECV  = 5'd8,
        S_EXT_DEC_WAIT  = 5'd9,
        S_EXTRACT_WR    = 5'd10,
        S_STASH_SEARCH  = 5'd11,
        S_STASH_READ    = 5'd12,
        S_COMPACT       = 5'd13,
        S_EVICT         = 5'd14,
        S_EV_ENC_START  = 5'd15,
        S_EV_ENC_FEED   = 5'd16,
        S_EV_ENC_RECV   = 5'd17,
        S_EV_ENC_TAG    = 5'd18,
        S_DDR_WRITE     = 5'd19,
        S_SB_RD_ENC_START = 5'd20,
        S_SB_RD_ENC_FEED  = 5'd21,
        S_SB_RD_ENC_RECV  = 5'd22,
        S_SB_RD_ENC_TAG   = 5'd23,
        S_SB_WR_ENC_START = 5'd24,
        S_SB_WR_ENC_FEED  = 5'd25,
        S_SB_WR_ENC_RECV  = 5'd26,
        S_SB_WR_ENC_TAG   = 5'd27,
        S_ST_LOAD       = 5'd28,
        S_ST_FLUSH      = 5'd29,
        S_PM_WAIT       = 5'd30,
        S_HT_LOOKUP_WAIT = 5'd31;  // Step 4: wait for HT slot lookup result

    // Hash table FSM states
    localparam [2:0]
        HT_IDLE         = 3'd0,
        HT_WAIT_RD      = 3'd1,
        HT_WAIT_WR      = 3'd2,
        HT_BKT_CHAIN    = 3'd3,
        HT_SLOT_PROBE   = 3'd4;

    reg [4:0] state;
    reg [4:0] st_return_state;

    // =========================================================================
    // Internal registers  (Step 4: added evict_list_idx, evict_stash_idx)
    // =========================================================================
    reg [SLOT_AW-1:0]   req_slot_addr;
    reg                  req_op;
    reg [BUCKET_W-1:0]  req_b, req_b_new;
    reg [Z*SLOT_W-1:0]  bkt_slot_list;
    reg [FILL_W-1:0]    bkt_fill_count;
    reg                  same_bucket;
    reg                  found_in_bucket, found_in_stash;
    reg [POS_W-1:0]     found_pos;
    reg [PTR_W-1:0]     found_stash_idx;
    reg [PTR_W-1:0]     stash_alloc_idx;
    reg [SLOT_AW-1:0]   ev_slot_addr;
    wire [PTR_W:0] stash_occupancy;
    reg [BEAT_W-1:0]    beat_cnt;
    reg [POS_W-1:0]     scan_pos;
    reg [FILL_W-1:0]    evict_slots_remaining;
    reg [3:0]           evict_list_idx;    // Step 4: index into ht_evict_list
    reg [PTR_W-1:0]     evict_stash_idx;   // Step 4: current eviction stash index

    // AES-GCM feed control
    reg                  aes_half;
    reg [AXI_DW-1:0]    aes_beat_buf;
    reg [8:0]            aes_blk_cnt;
    reg [BEAT_W-1:0]    aes_out_beat;
    reg                  aes_out_half;
    reg [AES_BLK_W-1:0] aes_out_lo;

    // =========================================================================
    // AES-GCM Instance
    // =========================================================================
    reg          gcm_start;
    reg          gcm_encrypt;
    reg [127:0]  gcm_data_in;
    reg          gcm_data_valid;
    reg          gcm_data_last;
    wire [127:0] gcm_data_out;
    wire         gcm_data_out_valid;
    wire         gcm_data_out_last;
    reg  [127:0] gcm_tag_in;
    wire [127:0] gcm_tag_out;
    wire         gcm_tag_valid;
    wire         gcm_tag_match;
    wire         gcm_ready;
    wire         gcm_busy;
    wire         gcm_input_ready;
    wire [95:0]  gcm_iv_out;
    wire         gcm_iv_updated;
    wire         gcm_counter_overflow;

    (* max_fanout = 32 *) reg  [95:0]  gcm_iv_reg;

    aes_gcm_pipelined u_aes_gcm (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .start(gcm_start), .encrypt(gcm_encrypt),
        .key(aes_key), .iv(gcm_iv_reg),
        .data_in(gcm_data_in), .data_valid(gcm_data_valid),
        .data_last(gcm_data_last), .data_bytes_valid(4'd0),
        .data_out(gcm_data_out), .data_out_valid(gcm_data_out_valid),
        .data_out_last(gcm_data_out_last),
        .tag_in(gcm_tag_in), .tag_out(gcm_tag_out),
        .tag_valid(gcm_tag_valid), .tag_match(gcm_tag_match),
        .ready(gcm_ready), .busy(gcm_busy), .input_ready(gcm_input_ready),
        .iv_out(gcm_iv_out), .iv_updated(gcm_iv_updated),
        .counter_overflow(gcm_counter_overflow)
    );

    // =========================================================================
    // IV Slot Table
    // =========================================================================
    reg  [SLOT_AW-1:0]  ivt_rd_addr;
    reg                  ivt_rd_en;
    wire [95:0]          ivt_rd_iv;
    wire [127:0]         ivt_rd_tag;
    wire                 ivt_rd_valid;
    reg  [SLOT_AW-1:0]  ivt_wr_addr;
    reg  [95:0]          ivt_wr_iv;
    reg  [127:0]         ivt_wr_tag;
    reg                  ivt_wr_en;

    iv_slot_table u_iv_table (
        .clk(clk), .rst_n(rst_n),
        .rd_slot_addr(ivt_rd_addr), .rd_en(ivt_rd_en),
        .rd_iv(ivt_rd_iv), .rd_tag(ivt_rd_tag), .rd_valid(ivt_rd_valid),
        .wr_slot_addr(ivt_wr_addr), .wr_iv(ivt_wr_iv),
        .wr_tag(ivt_wr_tag), .wr_en(ivt_wr_en)
    );

    // =========================================================================
    // Submodules
    // =========================================================================

    // --- pos_map ---
    reg  [SLOT_AW-1:0]  pm_rd_addr;  reg pm_rd_en;
    wire [BUCKET_W-1:0]  pm_rd_bucket; wire [1:0] pm_rd_status; wire pm_rd_valid;
    reg  [SLOT_AW-1:0]  pm_wr_addr;  reg [BUCKET_W-1:0] pm_wr_bucket;
    reg  [1:0]           pm_wr_status; reg pm_wr_en;

    pos_map u_pos_map (
        .clk(clk), .rst_n(rst_n),
        .rd_slot_addr(pm_rd_addr), .rd_en(pm_rd_en),
        .rd_bucket(pm_rd_bucket), .rd_status(pm_rd_status), .rd_valid(pm_rd_valid),
        .wr_slot_addr(pm_wr_addr), .wr_bucket(pm_wr_bucket),
        .wr_status(pm_wr_status), .wr_en(pm_wr_en),
        .init_mode(init_mode),
        .init_pm_wr_addr(init_pm_wr_addr),
        .init_pm_wr_bucket(init_pm_wr_bucket),
        .init_pm_wr_status(init_pm_wr_status),
        .init_pm_wr_en(init_pm_wr_en),
        .busy(pm_busy),
        .dbg_state(pm_dbg_state),
        .m_axi_arid(pm_axi_arid), .m_axi_araddr(pm_axi_araddr),
        .m_axi_arlen(pm_axi_arlen), .m_axi_arsize(pm_axi_arsize),
        .m_axi_arburst(pm_axi_arburst), .m_axi_arvalid(pm_axi_arvalid),
        .m_axi_arready(pm_axi_arready),
        .m_axi_rid(pm_axi_rid), .m_axi_rdata(pm_axi_rdata),
        .m_axi_rresp(pm_axi_rresp), .m_axi_rlast(pm_axi_rlast),
        .m_axi_rvalid(pm_axi_rvalid), .m_axi_rready(pm_axi_rready),
        .m_axi_awid(pm_axi_awid), .m_axi_awaddr(pm_axi_awaddr),
        .m_axi_awlen(pm_axi_awlen), .m_axi_awsize(pm_axi_awsize),
        .m_axi_awburst(pm_axi_awburst), .m_axi_awvalid(pm_axi_awvalid),
        .m_axi_awready(pm_axi_awready),
        .m_axi_wdata(pm_axi_wdata), .m_axi_wstrb(pm_axi_wstrb),
        .m_axi_wlast(pm_axi_wlast), .m_axi_wvalid(pm_axi_wvalid),
        .m_axi_wready(pm_axi_wready),
        .m_axi_bid(pm_axi_bid), .m_axi_bresp(pm_axi_bresp),
        .m_axi_bvalid(pm_axi_bvalid), .m_axi_bready(pm_axi_bready)
    );

    // --- bucket_meta ---
    reg  [BUCKET_W-1:0]  bm_rd_bucket; reg bm_rd_en;
    wire [Z*SLOT_W-1:0]  bm_rd_slot_list; wire [FILL_W-1:0] bm_rd_fill; wire bm_rd_valid;
    reg  [BUCKET_W-1:0]  bm_wr_bucket; reg [Z*SLOT_W-1:0] bm_wr_slot_list;
    reg  [FILL_W-1:0]    bm_wr_fill; reg bm_wr_en;

    wire [BUCKET_W-1:0]  bm_wr_bucket_mux    = init_mode ? init_bm_wr_bucket    : bm_wr_bucket;
    wire [Z*SLOT_W-1:0] bm_wr_slot_list_mux = init_mode ? init_bm_wr_slot_list : bm_wr_slot_list;
    wire [FILL_W-1:0]    bm_wr_fill_mux      = init_mode ? init_bm_wr_fill      : bm_wr_fill;
    wire                  bm_wr_en_mux        = init_mode ? init_bm_wr_en        : bm_wr_en;

    bucket_meta u_bucket_meta (
        .clk(clk), .rst_n(rst_n),
        .rd_bucket(bm_rd_bucket), .rd_en(bm_rd_en),
        .rd_slot_list(bm_rd_slot_list), .rd_fill_count(bm_rd_fill), .rd_valid(bm_rd_valid),
        .wr_bucket(bm_wr_bucket_mux), .wr_slot_list(bm_wr_slot_list_mux),
        .wr_fill_count(bm_wr_fill_mux), .wr_en(bm_wr_en_mux)
    );

    // --- PRNG ---
    reg prng_en;
    wire [BUCKET_W-1:0] prng_bucket;
    prng_lfsr #(.OUT_W(BUCKET_W)) u_prng (
        .clk(clk), .rst_n(rst_n),
        .seed({BUCKET_W{1'b0}}), .seed_valid(1'b0),
        .en(prng_en), .rand_out(), .rand_bucket(prng_bucket)
    );

    // --- scratch_buf ---
    wire [AXI_DW-1:0] scr_stream_in_data; wire scr_stream_in_valid;
    wire [9:0] scr_stream_in_beat_addr;
    reg  [9:0] scr_fsm_rd_addr; reg scr_fsm_rd_en;
    wire [AXI_DW-1:0] scr_fsm_rd_data; wire scr_fsm_rd_valid;
    reg  [9:0] scr_fsm_wr_addr; reg [AXI_DW-1:0] scr_fsm_wr_data; reg scr_fsm_wr_en;
    wire [9:0] scr_stream_out_beat_addr; wire scr_stream_out_rd_en;
    wire [AXI_DW-1:0] scr_stream_out_data; wire scr_stream_out_valid;

    scratch_buf u_scratch (
        .clk(clk), .rst_n(rst_n),
        .stream_in_data(scr_stream_in_data), .stream_in_valid(scr_stream_in_valid),
        .stream_in_beat_addr(scr_stream_in_beat_addr), .stream_in_ready(),
        .fsm_rd_addr(scr_fsm_rd_addr), .fsm_rd_en(scr_fsm_rd_en),
        .fsm_rd_data(scr_fsm_rd_data), .fsm_rd_valid(scr_fsm_rd_valid),
        .fsm_wr_addr(scr_fsm_wr_addr), .fsm_wr_data(scr_fsm_wr_data),
        .fsm_wr_en(scr_fsm_wr_en),
        .stream_out_beat_addr(scr_stream_out_beat_addr),
        .stream_out_rd_en(scr_stream_out_rd_en),
        .stream_out_data(scr_stream_out_data), .stream_out_valid(scr_stream_out_valid)
    );

    // --- stash ---
    reg  [SLOT_AW-1:0] st_ins_slot; reg [BUCKET_W-1:0] st_ins_tgt; reg st_ins_en;
    wire [PTR_W-1:0] st_ins_idx; wire st_ins_full;
    reg  [PTR_W-1:0] st_remap_idx; reg [BUCKET_W-1:0] st_remap_bkt; reg st_remap_en;
    reg  [PTR_W-1:0] st_evict_idx; reg st_evict_en;
    reg  [PTR_W-1:0] st_dwr_entry; reg [BEAT_W-1:0] st_dwr_beat;
    reg  [AXI_DW-1:0] st_dwr_data; reg st_dwr_en;
    reg  [PTR_W-1:0] st_drd_entry; reg [BEAT_W-1:0] st_drd_beat; reg st_drd_en;
    wire [AXI_DW-1:0] st_drd_data; wire st_drd_valid;
    // Step 5: CAM ports removed — HT is sole metadata store

    stash #(.DEPTH(STASH_DEPTH)) u_stash (
        .clk(clk), .rst_n(rst_n),
        // Step 5: CAM ports disconnected — HT is sole metadata store
        .cam_slot_addr({SLOT_AW{1'b0}}), .cam_slot_en(1'b0),
        .cam_slot_hit(), .cam_slot_idx(),
        .cam_bkt_target({BUCKET_W{1'b0}}), .cam_bkt_en(1'b0),
        .cam_bkt_hit(), .cam_bkt_idx(),
        .cam_bkt_slot_addr(),
        .cam_bkt_inhibit_busy(),
        .ins_slot_addr(st_ins_slot), .ins_target_bucket(st_ins_tgt), .ins_en(st_ins_en),
        .ins_idx(st_ins_idx), .ins_full(st_ins_full),
        .remap_idx(st_remap_idx), .remap_new_bucket(st_remap_bkt), .remap_en(st_remap_en),
        .evict_idx(st_evict_idx), .evict_en(st_evict_en),
        .dwr_entry(st_dwr_entry), .dwr_beat(st_dwr_beat),
        .dwr_data(st_dwr_data), .dwr_en(st_dwr_en),
        .drd_entry(st_drd_entry), .drd_beat(st_drd_beat), .drd_en(st_drd_en),
        .drd_data(st_drd_data), .drd_valid(st_drd_valid),
        .ext_rd_req(),
        .ext_wr_req(),
        .ext_entry(),
        .ext_rd_done(st_ext_rd_done),
        .ext_wr_done(st_ext_wr_done),
        .ext_busy(st_ext_busy),
        .ext_lb_wr_data(st_ext_lb_wr_data),
        .ext_lb_wr_addr(st_ext_lb_wr_addr),
        .ext_lb_wr_en(st_ext_lb_wr_en),
        .ext_lb_rd_addr(st_ext_lb_rd_addr),
        .ext_lb_rd_en(st_ext_lb_rd_en),
        .ext_lb_rd_data(st_ext_lb_rd_data),
        .ext_lb_rd_valid(st_ext_lb_rd_valid),
        .occupancy(stash_occupancy)
    );

    // --- axi_master ---
    reg axim_cmd_read, axim_cmd_write; reg [BUCKET_W-1:0] axim_cmd_bucket;
    wire axim_read_done, axim_write_done, axim_busy;
    wire [AXI_DW-1:0] axim_rdata_data; wire axim_rdata_valid; wire [9:0] axim_rdata_beat;
    wire [9:0] axim_wdata_beat_req;

    axi_master u_axi_master (
        .clk(clk), .rst_n(rst_n),
        .cmd_read(axim_cmd_read), .cmd_write(axim_cmd_write),
        .cmd_bucket(axim_cmd_bucket),
        .cmd_read_done(axim_read_done), .cmd_write_done(axim_write_done),
        .cmd_busy(axim_busy),
        .rdata_data(axim_rdata_data), .rdata_valid(axim_rdata_valid),
        .rdata_beat_addr(axim_rdata_beat),
        .wdata_data(scr_stream_out_data), .wdata_valid(scr_stream_out_valid),
        .wdata_ready(), .wdata_beat_req(axim_wdata_beat_req),
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
        .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready)
    );

    assign scr_stream_in_data      = axim_rdata_data;
    assign scr_stream_in_valid     = axim_rdata_valid;
    assign scr_stream_in_beat_addr = axim_rdata_beat;
    assign scr_stream_out_beat_addr = axim_wdata_beat_req;
    assign scr_stream_out_rd_en     = (state == S_DDR_WRITE);
    assign client_wdata_beat_req = beat_cnt;

    // =========================================================================
    // Helper functions
    // =========================================================================
    function [SLOT_AW-1:0] get_slot_at;
        input [Z*SLOT_W-1:0] list; input [POS_W-1:0] pos;
        integer base;
        begin base = pos * SLOT_W; get_slot_at = list[base +: SLOT_W] << 12; end
    endfunction

    function [Z*SLOT_W-1:0] set_slot_at;
        input [Z*SLOT_W-1:0] list; input [POS_W-1:0] pos; input [SLOT_W-1:0] val;
        integer base;
        begin set_slot_at = list; base = pos * SLOT_W; set_slot_at[base +: SLOT_W] = val; end
    endfunction

    function [9:0] scr_addr;
        input [POS_W-1:0] p; input [BEAT_W-1:0] b;
        begin scr_addr = {p, b[6:0]}; end
    endfunction

    // =========================================================================
    // Debug
    // =========================================================================
    assign dbg_state          = state;
    assign dbg_client_done    = client_done;
    assign dbg_client_req     = client_req;
    assign dbg_req_b          = req_b;
    assign dbg_req_b_new      = req_b_new;
    assign dbg_found_in_bucket = found_in_bucket;
    assign dbg_found_in_stash = found_in_stash;
    assign dbg_same_bucket    = same_bucket;
    assign dbg_err_stash_ovf  = err_stash_overflow;
    assign dbg_stash_occ      = stash_occupancy;
    assign dbg_gcm_tag_valid = gcm_tag_valid;
    assign dbg_gcm_tag_match = gcm_tag_match;
    assign dbg_ht_state       = ht_state;
    assign dbg_ht_op          = ht_op;
    assign dbg_ht_done        = ht_done;
    assign dbg_ht_latch_lookup = ht_latch_slot_lookup;
    assign dbg_ht_latch_insert = ht_latch_slot_insert;
    assign dbg_ht_latch_delete = ht_latch_slot_delete;
    assign dbg_ht_sng_rd_req  = st_sng_rd_req;
    assign dbg_ht_sng_wr_req  = st_sng_wr_req;
    assign dbg_ht_sng_done    = st_sng_done;

    reg              scan_hit;
    reg [POS_W-1:0]  scan_hit_pos;
    integer          scan_i;

    // =========================================================================
    // Main FSM  (Step 4: S_SCAN2 fires HT lookup, S_HT_LOOKUP_WAIT branches,
    //            S_EVICT uses ht_evict_list, evict encrypt uses evict_stash_idx)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            client_done <= 0; client_stall <= 0; client_rdata_valid <= 0;
            err_bucket_overflow <= 0; err_stash_overflow <= 0; err_tag_mismatch <= 0;
            beat_cnt <= 0; evict_slots_remaining <= 0;
            evict_list_idx <= 0; evict_stash_idx <= 0;         // Step 4
            found_in_bucket <= 0; found_in_stash <= 0;
            found_pos <= 0; found_stash_idx <= 0; stash_alloc_idx <= 0;
            req_b <= 0; req_b_new <= 0; bkt_fill_count <= 0;
            bkt_slot_list <= 0; same_bucket <= 0;
            pm_rd_en <= 0; pm_wr_en <= 0; pm_wr_status <= `ST_DUMMY;
            bm_rd_en <= 0; bm_wr_en <= 0; prng_en <= 0;
            st_ins_en <= 0; st_remap_en <= 0; st_evict_en <= 0;
            st_dwr_en <= 0; st_drd_en <= 0;
            axim_cmd_read <= 0; axim_cmd_write <= 0;
            scr_fsm_rd_en <= 0; scr_fsm_wr_en <= 0;
            gcm_start <= 0; gcm_data_valid <= 0; gcm_data_last <= 0;
            gcm_encrypt <= 0; gcm_iv_reg <= 0;
            aes_half <= 0; aes_blk_cnt <= 0;
            aes_out_beat <= 0; aes_out_half <= 0;
            ivt_rd_en <= 0; ivt_wr_en <= 0;
            st_ext_rd_req <= 0; st_ext_wr_req <= 0; st_ext_entry <= 0;
            ht_cmd_slot_lookup <= 0; ht_cmd_slot_insert <= 0;
            ht_cmd_slot_delete <= 0; ht_cmd_bkt_find <= 0;
            ht_cmd_remap <= 0;
            st_return_state <= S_IDLE;
        end else begin
            // Defaults
            client_done <= 0; client_rdata_valid <= 0;
            pm_rd_en <= 0; pm_wr_en <= 0;
            bm_rd_en <= 0; bm_wr_en <= 0; prng_en <= 0;
            st_ins_en <= 0; st_remap_en <= 0; st_evict_en <= 0;
            st_dwr_en <= 0; st_drd_en <= 0;
            axim_cmd_read <= 0; axim_cmd_write <= 0;
            scr_fsm_rd_en <= 0; scr_fsm_wr_en <= 0;
            gcm_start <= 0; gcm_data_valid <= 0; gcm_data_last <= 0;
            ivt_rd_en <= 0; ivt_wr_en <= 0;
            ht_cmd_slot_lookup <= 0; ht_cmd_slot_insert <= 0;
            ht_cmd_slot_delete <= 0; ht_cmd_bkt_find <= 0;

            case (state)

                S_IDLE: begin
                    client_stall <= 0;
                    if (client_req) begin
                        req_slot_addr <= client_slot_addr;
                        req_op <= client_op;
                     `ifndef SYNTHESIS   
                        $display("[ORAM_FSM] t=%0t S_IDLE: captured req_op=%b slot=0x%08h",
                                 $time, client_op, client_slot_addr);
                      `endif  
                        client_stall <= 1;
                        err_bucket_overflow <= 0; err_stash_overflow <= 0; err_tag_mismatch <= 0;
                        if (!pm_busy) begin
                            pm_rd_addr <= client_slot_addr; pm_rd_en <= 1;
                            prng_en <= 1;
                            state <= S_POS_LOOKUP;
                        end else begin
                            state <= S_PM_WAIT;
                        end
                    end
                end

                S_PM_WAIT: begin
                    if (!pm_busy) begin
                        pm_rd_addr <= req_slot_addr; pm_rd_en <= 1;
                        prng_en <= 1;
                        state <= S_POS_LOOKUP;
                    end
                end

                S_POS_LOOKUP: begin
                    if (pm_rd_valid) begin
                        req_b <= pm_rd_bucket;
                        req_b_new <= dbg_force_same_bucket ? pm_rd_bucket :
                                     dbg_prng_override_en  ? dbg_prng_override :
                                                             prng_bucket;
                        same_bucket <= dbg_force_same_bucket ? 1'b1 :
                                      dbg_prng_override_en  ? (dbg_prng_override == pm_rd_bucket) :
                                                              (prng_bucket == pm_rd_bucket);
                        pm_wr_addr <= req_slot_addr;
                        pm_wr_bucket <= dbg_force_same_bucket ? pm_rd_bucket :
                                       dbg_prng_override_en  ? dbg_prng_override :
                                                               prng_bucket;
                        pm_wr_status <= `ST_VALID;
                        pm_wr_en <= 1;
                        bm_rd_bucket <= pm_rd_bucket; bm_rd_en <= 1;
                        axim_cmd_bucket <= pm_rd_bucket; axim_cmd_read <= 1;
                        state <= S_DDR_READ;
                    end
                end

                S_DDR_READ: begin
                    if (bm_rd_valid) begin
                        bkt_slot_list <= bm_rd_slot_list;
                        bkt_fill_count <= bm_rd_fill;
                    end
                    if (axim_read_done) state <= S_SCAN;
                end

                S_SCAN: begin
                    scan_hit = 0; scan_hit_pos = 0;
                    for (scan_i = 0; scan_i < Z; scan_i = scan_i + 1)
                        if (scan_i < bkt_fill_count)
                            if (get_slot_at(bkt_slot_list, scan_i[POS_W-1:0]) == req_slot_addr)
                                begin scan_hit = 1; scan_hit_pos = scan_i[POS_W-1:0]; end
                    found_in_bucket <= scan_hit;
                    found_pos <= scan_hit_pos;
                    // Step 5: found_in_stash set by S_HT_LOOKUP_WAIT from HT result
                    found_in_stash <= 0;
                    found_stash_idx <= 0;
                    state <= S_SCAN2;
                end

                // =============================================================
                // S_SCAN2: fire HT lookup, go to wait state (Step 4)
                // No longer branches here — S_HT_LOOKUP_WAIT does that.
                // =============================================================
                S_SCAN2: begin
                    ht_cmd_addr <= req_slot_addr;
                    ht_cmd_slot_lookup <= 1;

                    `ifndef SYNTHESIS
                    $display("[SCAN2] t=%0t slot=0x%08h in_bkt=%b same_bkt=%b op=%b found_pos=%0d",
                             $time, req_slot_addr, found_in_bucket,
                             same_bucket, req_op, found_pos);
                    `endif
                    state <= S_HT_LOOKUP_WAIT;
                end

                // =============================================================
                // S_HT_LOOKUP_WAIT: wait for HT lookup result, then branch
                // (Step 4: HT authoritative — uses ht_slot_found not BRAM CAM)
                // Guard: ht_op == HT_OP_SLOT_LOOKUP prevents misinterpreting
                // a stale INSERT/DELETE completion as the LOOKUP result.
                // =============================================================
                S_HT_LOOKUP_WAIT: begin
                    if (ht_done && ht_op == HT_OP_SLOT_LOOKUP) begin
                        // Latch HT results as authoritative
                        found_in_stash <= ht_slot_found;
                        found_stash_idx <= ht_found_idx;

                        if (found_in_bucket) begin
                            stash_alloc_idx <= st_ins_idx;
                            st_ins_slot <= req_slot_addr;
                            st_ins_tgt <= req_b_new;

                            if (same_bucket) begin
                                if (req_op == 0) begin
                                    ivt_rd_addr <= req_slot_addr;
                                    ivt_rd_en <= 1;
                                    state <= S_EXT_IV_RD;
                                end else begin
                                    gcm_encrypt <= 1;
                                    gcm_iv_reg <= aes_iv_seed;
                                    gcm_start <= 1;
                                    beat_cnt <= 0;
                                    aes_half <= 0; aes_blk_cnt <= 0;
                                    aes_out_beat <= 0; aes_out_half <= 0;
                                    state <= S_SB_WR_ENC_START;
                                end
                            end else begin
                                if (req_op == 0) begin
                                    ivt_rd_addr <= req_slot_addr;
                                    ivt_rd_en <= 1;
                                    state <= S_EXT_IV_RD;
                                end else begin
                                    beat_cnt <= 0;
                                    state <= S_EXTRACT_WR;
                                end
                            end
                        end else begin
                            state <= S_STASH_SEARCH;
                        end
                    end
                end

                // =============================================================
                // DECRYPT PATH (unchanged from original)
                // =============================================================
                S_EXT_IV_RD: begin
                    if (ivt_rd_valid) begin
                        `ifndef SYNTHESIS
                        $display("[DEC_IV] t=%0t slot=0x%08h IV=0x%024h TAG=0x%032h found_pos=%0d",
                                 $time, req_slot_addr, ivt_rd_iv, ivt_rd_tag, found_pos);
                        $display("[DEC_IV] scratch_mem[0][31:0]=0x%08h scratch_mem[1][31:0]=0x%08h scratch_mem[2][31:0]=0x%08h",
                                 u_scratch.mem[scr_addr(found_pos, 10'd0)][31:0],
                                 u_scratch.mem[scr_addr(found_pos, 10'd1)][31:0],
                                 u_scratch.mem[scr_addr(found_pos, 10'd2)][31:0]);
                        `endif
                        gcm_iv_reg <= ivt_rd_iv;
                        gcm_tag_in <= ivt_rd_tag;
                        gcm_encrypt <= 0;
                        gcm_start <= 1;
                        scr_fsm_rd_addr <= scr_addr(found_pos, 0);
                        scr_fsm_rd_en <= 1;
                        beat_cnt <= 0;
                        aes_half <= 0; aes_blk_cnt <= 0;
                        aes_out_beat <= 0; aes_out_half <= 0;
                        state <= S_EXT_DEC_START;
                    end
                end

                S_EXT_DEC_START: begin
                    if (gcm_input_ready)
                        state <= S_EXT_DEC_FEED;
                end

                S_EXT_DEC_FEED: begin
                    // Re-issue scratch read if no valid data yet (cold-start only)
                    if (!scr_fsm_rd_valid && !aes_half) begin
                        scr_fsm_rd_addr <= scr_addr(found_pos, beat_cnt);
                        scr_fsm_rd_en <= 1;
                    end

                    // LOW HALF: needs BRAM valid - feed low 128 bits
                    if (scr_fsm_rd_valid && gcm_input_ready && !aes_half) begin
                        `ifndef SYNTHESIS
                        if (beat_cnt < 3)
                            $display("[DEC_FEED] t=%0t beat=%0d CT_lo=0x%032h CT_hi=0x%032h scr_addr=%0d",
                                     $time, beat_cnt, scr_fsm_rd_data[127:0],
                                     scr_fsm_rd_data[255:128],
                                     scr_addr(found_pos, beat_cnt));
                        `endif  
                        gcm_data_in <= scr_fsm_rd_data[127:0];
                        gcm_data_valid <= 1;
                        aes_beat_buf <= scr_fsm_rd_data;
                        aes_half <= 1;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                    end

                    // HIGH HALF: uses aes_beat_buf + prefetch next beat
                    if (gcm_input_ready && aes_half) begin
                        gcm_data_in <= aes_beat_buf[255:128];
                        gcm_data_valid <= 1;
                        gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                        aes_half <= 0;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        // PREFETCH: issue next read NOW so it arrives by next low-half
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            scr_fsm_rd_addr <= scr_addr(found_pos, beat_cnt + 1);
                            scr_fsm_rd_en <= 1;
                            beat_cnt <= beat_cnt + 1;
                        end else begin
                            state <= S_EXT_DEC_RECV;
                        end
                    end
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            `ifndef SYNTHESIS
                            if (aes_out_beat < 2)
                                $display("[DEC_PT] t=%0t beat=%0d PT=0x%064h",
                                         $time, aes_out_beat, {gcm_data_out, aes_out_lo});
                            `endif  
                            client_rdata <= {gcm_data_out, aes_out_lo};
                            client_rdata_beat <= aes_out_beat;
                            client_rdata_valid <= 1;
                            st_dwr_entry <= stash_alloc_idx;
                            st_dwr_beat <= aes_out_beat;
                            st_dwr_data <= {gcm_data_out, aes_out_lo};
                            st_dwr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                end

                S_EXT_DEC_RECV: begin
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            client_rdata <= {gcm_data_out, aes_out_lo};
                            client_rdata_beat <= aes_out_beat;
                            client_rdata_valid <= 1;
                            st_dwr_entry <= stash_alloc_idx;
                            st_dwr_beat <= aes_out_beat;
                            st_dwr_data <= {gcm_data_out, aes_out_lo};
                            st_dwr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                    if (aes_out_beat == BEATS[BEAT_W-1:0] && !aes_out_half)
                        state <= S_EXT_DEC_WAIT;
                end

                S_EXT_DEC_WAIT: begin
                    if (gcm_tag_valid) begin
                        if (!gcm_tag_match)
                            err_tag_mismatch <= 1;
                        if (same_bucket) begin
                            gcm_encrypt <= 1;
                            gcm_iv_reg <= aes_iv_seed;
                            gcm_start <= 1;
                            st_drd_entry <= stash_alloc_idx;
                            st_drd_beat <= 0;
                            st_drd_en <= 1;
                            beat_cnt <= 0;
                            aes_half <= 0; aes_blk_cnt <= 0;
                            aes_out_beat <= 0; aes_out_half <= 0;
                            state <= S_SB_RD_ENC_START;
                        end else begin
                            st_ins_en <= 1;
                            ht_cmd_addr <= req_slot_addr;
                            ht_cmd_bkt <= req_b_new;
                            ht_cmd_idx <= stash_alloc_idx;
                            ht_cmd_slot_insert <= 1;
                            if (st_ins_full) err_stash_overflow <= 1;
                            pm_wr_addr <= req_slot_addr;
                            pm_wr_bucket <= req_b_new;
                            pm_wr_status <= `ST_IN_STASH;
                            pm_wr_en <= 1;
                            bkt_slot_list <= set_slot_at(bkt_slot_list, found_pos,
                                bkt_slot_list[(bkt_fill_count-1)*SLOT_W +: SLOT_W]);
                            bkt_fill_count <= bkt_fill_count - 1;
                            st_ext_entry <= stash_alloc_idx;
                            st_return_state <= S_COMPACT;
                            state <= S_ST_FLUSH;
                        end
                    end
                end

                // =============================================================
                // SAME-BUCKET READ: re-encrypt (unchanged)
                // =============================================================
                S_SB_RD_ENC_START: begin
                    if (gcm_input_ready) state <= S_SB_RD_ENC_FEED;
                end

                S_SB_RD_ENC_FEED: begin
                    // Re-issue stash read if no valid data yet (cold-start)
                    if (!st_drd_valid && !aes_half) begin
                        st_drd_entry <= stash_alloc_idx;
                        st_drd_beat <= beat_cnt;
                        st_drd_en <= 1;
                    end

                    // LOW HALF: needs stash valid - feed low half
                    if (st_drd_valid && gcm_input_ready && !aes_half) begin
                        gcm_data_in <= st_drd_data[127:0];
                        gcm_data_valid <= 1;
                        aes_beat_buf <= st_drd_data;
                        aes_half <= 1;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                    end

                    // HIGH HALF: uses aes_beat_buf + prefetch next beat
                    if (gcm_input_ready && aes_half) begin
                        gcm_data_in <= aes_beat_buf[255:128];
                        gcm_data_valid <= 1;
                        gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                        aes_half <= 0;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        // PREFETCH: issue next read NOW so it arrives by next low-half
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            st_drd_entry <= stash_alloc_idx;
                            st_drd_beat <= beat_cnt + 1;
                            st_drd_en <= 1;
                            beat_cnt <= beat_cnt + 1;
                        end else begin
                            state <= S_SB_RD_ENC_RECV;
                        end
                    end
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            scr_fsm_wr_addr <= scr_addr(found_pos, aes_out_beat);
                            scr_fsm_wr_data <= {gcm_data_out, aes_out_lo};
                            scr_fsm_wr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                end

                S_SB_RD_ENC_RECV: begin
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            scr_fsm_wr_addr <= scr_addr(found_pos, aes_out_beat);
                            scr_fsm_wr_data <= {gcm_data_out, aes_out_lo};
                            scr_fsm_wr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                    if (aes_out_beat == BEATS[BEAT_W-1:0] && !aes_out_half)
                        state <= S_SB_RD_ENC_TAG;
                end

                S_SB_RD_ENC_TAG: begin
                    if (gcm_tag_valid) begin
                        ivt_wr_addr <= req_slot_addr;
                        ivt_wr_iv <= gcm_iv_out;
                        ivt_wr_tag <= gcm_tag_out;
                        ivt_wr_en <= 1;
                        state <= S_COMPACT;
                    end
                end

                // =============================================================
                // SAME-BUCKET WRITE: encrypt client data (unchanged)
                // =============================================================
                S_SB_WR_ENC_START: begin
                    if (gcm_input_ready) state <= S_SB_WR_ENC_FEED;
                end

                S_SB_WR_ENC_FEED: begin
                    if (client_wdata_valid && gcm_input_ready) begin
                        if (!aes_half) begin
                            gcm_data_in <= client_wdata[127:0];
                            gcm_data_valid <= 1;
                            aes_beat_buf <= client_wdata;
                            aes_half <= 1;
                            aes_blk_cnt <= aes_blk_cnt + 1;
                        end else begin
                            gcm_data_in <= aes_beat_buf[255:128];
                            gcm_data_valid <= 1;
                            gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                            aes_half <= 0;
                            aes_blk_cnt <= aes_blk_cnt + 1;
                            if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                                beat_cnt <= beat_cnt + 1;
                            end else begin
                                state <= S_SB_WR_ENC_RECV;
                            end
                        end
                    end
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            scr_fsm_wr_addr <= scr_addr(found_pos, aes_out_beat);
                            scr_fsm_wr_data <= {gcm_data_out, aes_out_lo};
                            scr_fsm_wr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                end

                S_SB_WR_ENC_RECV: begin
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            scr_fsm_wr_addr <= scr_addr(found_pos, aes_out_beat);
                            scr_fsm_wr_data <= {gcm_data_out, aes_out_lo};
                            scr_fsm_wr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                    if (aes_out_beat == BEATS[BEAT_W-1:0] && !aes_out_half)
                        state <= S_SB_WR_ENC_TAG;
                end

                S_SB_WR_ENC_TAG: begin
                    if (gcm_tag_valid) begin
                        ivt_wr_addr <= req_slot_addr;
                        ivt_wr_iv <= gcm_iv_out;
                        ivt_wr_tag <= gcm_tag_out;
                        ivt_wr_en <= 1;
                        state <= S_COMPACT;
                    end
                end

                // =============================================================
                // WRITE PATH: different bucket (unchanged)
                // =============================================================
                S_EXTRACT_WR: begin
                    if (client_wdata_valid) begin
                        st_dwr_entry <= stash_alloc_idx;
                        st_dwr_beat <= beat_cnt;
                        st_dwr_data <= client_wdata;
                        st_dwr_en <= 1;
                        if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                            if (!found_in_stash) begin
                                st_ins_en <= 1;
                                ht_cmd_addr <= req_slot_addr;
                                ht_cmd_bkt <= req_b_new;
                                ht_cmd_idx <= stash_alloc_idx;
                                ht_cmd_slot_insert <= 1;
                                if (st_ins_full) err_stash_overflow <= 1;
                                pm_wr_addr <= req_slot_addr;
                                pm_wr_bucket <= req_b_new;
                                pm_wr_status <= `ST_IN_STASH;
                                pm_wr_en <= 1;
                                if (found_in_bucket) begin
                                    bkt_slot_list <= set_slot_at(bkt_slot_list, found_pos,
                                        bkt_slot_list[(bkt_fill_count-1)*SLOT_W +: SLOT_W]);
                                    bkt_fill_count <= bkt_fill_count - 1;
                                end
                            end
                            st_ext_entry <= stash_alloc_idx;
                            st_return_state <= S_COMPACT;
                            state <= S_ST_FLUSH;
                        end else begin
                            beat_cnt <= beat_cnt + 1;
                        end
                    end
                end

                // =============================================================
                // STASH paths (Step 3: INSERT on found_in_stash remap)
                // =============================================================
                S_STASH_SEARCH: begin
                    if (found_in_stash) begin
                        st_remap_idx <= found_stash_idx;
                        st_remap_bkt <= req_b_new;
                        st_remap_en <= 1;
                        // Step 3: re-insert/update HT entry with new bucket
                        ht_cmd_addr <= req_slot_addr;
                        ht_cmd_bkt <= req_b_new;
                        ht_cmd_idx <= found_stash_idx;
                        ht_cmd_slot_insert <= 1;
                        if (req_op) begin
                            beat_cnt <= 0;
                            stash_alloc_idx <= found_stash_idx;
                            state <= S_EXTRACT_WR;
                        end else begin
                            st_ext_entry <= found_stash_idx;
                            st_return_state <= S_STASH_READ;
                            beat_cnt <= 0;
                            state <= S_ST_LOAD;
                        end
                    end else if (req_op == 1'b1) begin
                        stash_alloc_idx <= st_ins_idx;
                        st_ins_slot <= req_slot_addr;
                        st_ins_tgt <= req_b_new;
                        beat_cnt <= 0;
                        state <= S_EXTRACT_WR;
                    end else begin
                        `ifndef SYNTHESIS
                        $display("[WARN] t=%0t READ of DUMMY slot 0x%08h - no data", $time, req_slot_addr);
                        `endif
                        state <= S_COMPACT;
                    end
                end

                S_STASH_READ: begin
                    if (st_drd_valid) begin
                        client_rdata <= st_drd_data;
                        client_rdata_beat <= beat_cnt;
                        client_rdata_valid <= 1;
                        if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                            st_drd_en <= 0;
                            state <= S_COMPACT;
                        end else begin
                            st_drd_entry <= found_stash_idx;
                            st_drd_beat <= beat_cnt + 1;
                            st_drd_en <= 1;
                            beat_cnt <= beat_cnt + 1;
                        end
                    end else begin
                        st_drd_entry <= found_stash_idx;
                        st_drd_beat <= beat_cnt;
                        st_drd_en <= 1;
                    end
                end

                // =============================================================
                // S_COMPACT (Step 4: HT authoritative BKT_FIND, no CAM trigger)
                // =============================================================
                S_COMPACT: begin
                    evict_slots_remaining <= {{(FILL_W-POS_W-1){1'b0}}, Z[POS_W:0]} - {1'b0, bkt_fill_count};
                    beat_cnt <= 0;
                    // Step 4: HT authoritative — BKT_FIND provides eviction candidates
                    ht_cmd_bkt <= req_b;
                    ht_cmd_bkt_find <= 1;
                    evict_list_idx <= 0;
                    state <= S_EVICT;
                end

                // =============================================================
                // EVICT (Step 4: walk ht_evict_list from BKT_FIND)
                // Uses ht_bkt_find_done (latched flag) instead of ht_done pulse.

                // Uses u_stash.slot_r[] hierarchical access for ev_slot_addr.
                // =============================================================
                S_EVICT: begin
                    if (!ht_bkt_find_done) begin
                        // Waiting for BKT_FIND to complete
                    end else if (evict_list_idx < ht_evict_count &&
                                 evict_slots_remaining > 0) begin
                        evict_stash_idx <= ht_evict_list[evict_list_idx];
                        ev_slot_addr <= u_stash.slot_r[ht_evict_list[evict_list_idx]];
                        `ifndef SYNTHESIS
                        $display("[EV_START] t=%0t evict slot=0x%08h stash_idx=%0d -> bucket=%0d pos=%0d list_idx=%0d",
                                 $time, u_stash.slot_r[ht_evict_list[evict_list_idx]],
                                 ht_evict_list[evict_list_idx], req_b, bkt_fill_count,
                                 evict_list_idx);
                        `endif
                        st_ext_entry <= ht_evict_list[evict_list_idx];
                        st_return_state <= S_EV_ENC_START;
                        gcm_encrypt <= 1;
                        gcm_iv_reg <= aes_iv_seed;
                        gcm_start <= 1;
                        aes_half <= 0; aes_blk_cnt <= 0;
                        aes_out_beat <= 0; aes_out_half <= 0;
                        beat_cnt <= 0;
                        evict_list_idx <= evict_list_idx + 1;
                        state <= S_ST_LOAD;
                    end else begin
                        `ifndef SYNTHESIS
                        $display("[ORAM_FSM] t=%0t S_EVICT: no more candidates, -> S_DDR_WRITE bucket=%0d fill=%0d",
                                 $time, req_b, bkt_fill_count);
                        `endif
                        bm_wr_bucket <= req_b;
                        bm_wr_slot_list <= bkt_slot_list;
                        bm_wr_fill <= bkt_fill_count;
                        bm_wr_en <= 1;
                        axim_cmd_bucket <= req_b;
                        axim_cmd_write <= 1;
                        state <= S_DDR_WRITE;
                    end
                end


                S_EV_ENC_START: begin
                    st_drd_entry <= evict_stash_idx;
                    st_drd_beat <= 0;
                    st_drd_en <= 1;
                    if (gcm_input_ready)
                        state <= S_EV_ENC_FEED;
                end

                S_EV_ENC_FEED: begin
                    // Re-issue stash read if no valid data yet (cold-start)
                    if (!st_drd_valid && !aes_half) begin
                        st_drd_entry <= evict_stash_idx;
                        st_drd_beat <= beat_cnt;
                        st_drd_en <= 1;
                    end

                    // LOW HALF: needs stash valid - feed low half
                    if (st_drd_valid && gcm_input_ready && !aes_half) begin
                        `ifndef SYNTHESIS
                        if (beat_cnt < 2)
                            $display("[EV_FEED] t=%0t beat=%0d PT_lo=0x%032h",
                                     $time, beat_cnt, st_drd_data[127:0]);
                        `endif
                        gcm_data_in <= st_drd_data[127:0];
                        gcm_data_valid <= 1;
                        aes_beat_buf <= st_drd_data;
                        aes_half <= 1;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                    end

                    // HIGH HALF: uses aes_beat_buf + prefetch next beat
                    if (gcm_input_ready && aes_half) begin
                        gcm_data_in <= aes_beat_buf[255:128];
                        gcm_data_valid <= 1;
                        gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                        aes_half <= 0;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        // PREFETCH: issue next read NOW so it arrives by next low-half
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            st_drd_entry <= evict_stash_idx;
                            st_drd_beat <= beat_cnt + 1;
                            st_drd_en <= 1;
                            beat_cnt <= beat_cnt + 1;
                        end else begin
                            state <= S_EV_ENC_RECV;
                        end
                    end
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            `ifndef SYNTHESIS
                            if (aes_out_beat < 2)
                                $display("[EV_CT] t=%0t beat=%0d CT=0x%064h scr_addr=%0d",
                                         $time, aes_out_beat, {gcm_data_out, aes_out_lo},
                                         scr_addr(bkt_fill_count[POS_W-1:0], aes_out_beat));
                            `endif
                            scr_fsm_wr_addr <= scr_addr(bkt_fill_count[POS_W-1:0], aes_out_beat);
                            scr_fsm_wr_data <= {gcm_data_out, aes_out_lo};
                            scr_fsm_wr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                end

                S_EV_ENC_RECV: begin
                    if (gcm_data_out_valid) begin
                        if (!aes_out_half) begin
                            aes_out_lo <= gcm_data_out;
                            aes_out_half <= 1;
                        end else begin
                            scr_fsm_wr_addr <= scr_addr(bkt_fill_count[POS_W-1:0], aes_out_beat);
                            scr_fsm_wr_data <= {gcm_data_out, aes_out_lo};
                            scr_fsm_wr_en <= 1;
                            aes_out_half <= 0;
                            aes_out_beat <= aes_out_beat + 1;
                        end
                    end
                    if (aes_out_beat == BEATS[BEAT_W-1:0] && !aes_out_half)
                        state <= S_EV_ENC_TAG;
                end


                S_EV_ENC_TAG: begin
                    if (gcm_tag_valid) begin
                        `ifndef SYNTHESIS
                        $display("[EV_TAG] t=%0t slot=0x%08h IV=0x%024h TAG=0x%032h pos=%0d",
                                 $time, ev_slot_addr, gcm_iv_out, gcm_tag_out,
                                 bkt_fill_count);
                        `endif
                        ivt_wr_addr <= ev_slot_addr;
                        ivt_wr_iv <= gcm_iv_out;
                        ivt_wr_tag <= gcm_tag_out;
                        ivt_wr_en <= 1;
                        bkt_slot_list <= set_slot_at(bkt_slot_list,
                            bkt_fill_count[POS_W-1:0],
                            ev_slot_addr[SLOT_W+11:12]);
                        bkt_fill_count <= bkt_fill_count + 1;
                        st_evict_idx <= evict_stash_idx;
                        st_evict_en <= 1;
                        ht_cmd_addr <= ev_slot_addr;
                        ht_cmd_idx <= evict_stash_idx;  
                        ht_cmd_slot_delete <= 1;
                        pm_wr_addr <= ev_slot_addr;
                        pm_wr_bucket <= req_b;
                        pm_wr_status <= `ST_VALID;
                        pm_wr_en <= 1;
                        evict_slots_remaining <= evict_slots_remaining - 1;
                        beat_cnt <= 0;
                        state <= S_EVICT;
                    end
                end

                // =============================================================
                S_DDR_WRITE: begin
                    `ifndef SYNTHESIS
                    if (axim_write_done)
                        $display("[ORAM_FSM] t=%0t S_DDR_WRITE: axim_write_done! -> S_IDLE", $time);
                    `endif
                    if (axim_write_done) begin
                        client_done <= 1;
                        client_stall <= 0;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;

                S_ST_LOAD: begin
                    if (st_ext_rd_done) begin
                        $display("[ORAM_FSM] t=%0t S_ST_LOAD: ext_rd_done -> return state", $time);
                        state <= st_return_state;
                    end else if (!st_ext_busy && !st_ext_rd_req) begin
                        st_ext_rd_req <= 1'b1;
                        $display("[ORAM_FSM] t=%0t S_ST_LOAD: issuing ext_rd_req entry=%0d", $time, st_ext_entry);
                    end else if (st_ext_rd_req && st_ext_busy) begin
                        st_ext_rd_req <= 1'b0;
                    end
                end

                S_ST_FLUSH: begin
                    if (st_ext_wr_done) begin
                        $display("[ORAM_FSM] t=%0t S_ST_FLUSH: ext_wr_done -> return state", $time);
                        state <= st_return_state;
                    end else if (!st_ext_busy && !st_ext_wr_req) begin
                        st_ext_wr_req <= 1'b1;
                        $display("[ORAM_FSM] t=%0t S_ST_FLUSH: issuing ext_wr_req entry=%0d", $time, st_ext_entry);
                    end else if (st_ext_wr_req && st_ext_busy) begin
                        st_ext_wr_req <= 1'b0;
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Performance Counters (Step 4: S_HT_LOOKUP_WAIT added to scan phase)
    // =========================================================================
    localparam [1:0] CAT_RB = 2'b00,
                     CAT_RS = 2'b01,
                     CAT_WB = 2'b10,
                     CAT_WS = 2'b11;

    reg        perf_active;
    reg [1:0]  perf_cat;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            perf_cat <= CAT_RB;
        else if (state == S_SCAN2)
            perf_cat <= {req_op, ~found_in_bucket};
    end

    `define PERF_INC(field) \
        case (perf_cat) \
            CAT_RB: perf_rb_``field <= perf_rb_``field + 1; \
            CAT_RS: perf_rs_``field <= perf_rs_``field + 1; \
            CAT_WB: perf_wb_``field <= perf_wb_``field + 1; \
            CAT_WS: perf_ws_``field <= perf_ws_``field + 1; \
        endcase

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            perf_rb_ddr_rd  <= 0; perf_rb_decrypt <= 0; perf_rb_encrypt <= 0;
            perf_rb_compact <= 0; perf_rb_evict   <= 0; perf_rb_ddr_wr  <= 0;
            perf_rb_scan    <= 0; perf_rb_stash   <= 0; perf_rb_total   <= 0;
            perf_rb_ops     <= 0;
            perf_rs_ddr_rd  <= 0; perf_rs_decrypt <= 0; perf_rs_encrypt <= 0;
            perf_rs_compact <= 0; perf_rs_evict   <= 0; perf_rs_ddr_wr  <= 0;
            perf_rs_scan    <= 0; perf_rs_stash   <= 0; perf_rs_total   <= 0;
            perf_rs_ops     <= 0;
            perf_wb_ddr_rd  <= 0; perf_wb_decrypt <= 0; perf_wb_encrypt <= 0;
            perf_wb_compact <= 0; perf_wb_evict   <= 0; perf_wb_ddr_wr  <= 0;
            perf_wb_scan    <= 0; perf_wb_stash   <= 0; perf_wb_total   <= 0;
            perf_wb_ops     <= 0;
            perf_ws_ddr_rd  <= 0; perf_ws_decrypt <= 0; perf_ws_encrypt <= 0;
            perf_ws_compact <= 0; perf_ws_evict   <= 0; perf_ws_ddr_wr  <= 0;
            perf_ws_scan    <= 0; perf_ws_stash   <= 0; perf_ws_total   <= 0;
            perf_ws_ops     <= 0;
            perf_active     <= 0;
        end else if (perf_reset) begin
            perf_rb_ddr_rd  <= 0; perf_rb_decrypt <= 0; perf_rb_encrypt <= 0;
            perf_rb_compact <= 0; perf_rb_evict   <= 0; perf_rb_ddr_wr  <= 0;
            perf_rb_scan    <= 0; perf_rb_stash   <= 0; perf_rb_total   <= 0;
            perf_rb_ops     <= 0;
            perf_rs_ddr_rd  <= 0; perf_rs_decrypt <= 0; perf_rs_encrypt <= 0;
            perf_rs_compact <= 0; perf_rs_evict   <= 0; perf_rs_ddr_wr  <= 0;
            perf_rs_scan    <= 0; perf_rs_stash   <= 0; perf_rs_total   <= 0;
            perf_rs_ops     <= 0;
            perf_wb_ddr_rd  <= 0; perf_wb_decrypt <= 0; perf_wb_encrypt <= 0;
            perf_wb_compact <= 0; perf_wb_evict   <= 0; perf_wb_ddr_wr  <= 0;
            perf_wb_scan    <= 0; perf_wb_stash   <= 0; perf_wb_total   <= 0;
            perf_wb_ops     <= 0;
            perf_ws_ddr_rd  <= 0; perf_ws_decrypt <= 0; perf_ws_encrypt <= 0;
            perf_ws_compact <= 0; perf_ws_evict   <= 0; perf_ws_ddr_wr  <= 0;
            perf_ws_scan    <= 0; perf_ws_stash   <= 0; perf_ws_total   <= 0;
            perf_ws_ops     <= 0;
            perf_active     <= 0;
        end else begin
            if (state != S_IDLE)
                perf_active <= 1'b1;
            if (state == S_IDLE && perf_active) begin
                perf_active <= 1'b0;
                `PERF_INC(ops)
            end
            if (state != S_IDLE) begin
                `PERF_INC(total)
            end
            case (state)
                S_POS_LOOKUP, S_SCAN, S_SCAN2, S_HT_LOOKUP_WAIT: begin  // Step 4
                    `PERF_INC(scan)
                end
                S_DDR_READ: begin
                    `PERF_INC(ddr_rd)
                end
                S_EXT_IV_RD, S_EXT_DEC_START, S_EXT_DEC_FEED,
                S_EXT_DEC_RECV, S_EXT_DEC_WAIT: begin
                    `PERF_INC(decrypt)
                end
                S_STASH_SEARCH, S_STASH_READ, S_EXTRACT_WR: begin
                    `PERF_INC(stash)
                end
                S_SB_RD_ENC_START, S_SB_RD_ENC_FEED,
                S_SB_RD_ENC_RECV,  S_SB_RD_ENC_TAG,
                S_SB_WR_ENC_START, S_SB_WR_ENC_FEED,
                S_SB_WR_ENC_RECV,  S_SB_WR_ENC_TAG,
                S_EV_ENC_START,    S_EV_ENC_FEED,
                S_EV_ENC_RECV,     S_EV_ENC_TAG: begin
                    `PERF_INC(encrypt)
                end
                S_COMPACT: begin
                    `PERF_INC(compact)
                end
                S_EVICT: begin
                    `PERF_INC(evict)
                end
                S_DDR_WRITE: begin
                    `PERF_INC(ddr_wr)
                end
                default: ;
            endcase
        end
    end

    `undef PERF_INC

    // =========================================================================
    // Hash Table Logic — Step 4: HT is authoritative metadata store
    // =========================================================================
    // Runs in parallel with main FSM. Main FSM triggers HT operations by
    // setting ht_cmd_*. This block issues single-beat AXI reads/writes
    // via st_sng_* ports and updates the hash tables in HBM.
    // Step 4: HT results drive control flow (no longer shadow-only).
    // =========================================================================

    // HT command interface (set by main FSM, cleared by HT FSM)
    reg        ht_cmd_slot_lookup;   // lookup slot_addr in SLOT hash table
    reg        ht_cmd_slot_insert;   // insert entry into SLOT + BKT tables
    reg        ht_cmd_slot_delete;   // delete entry from SLOT + BKT tables
    reg        ht_cmd_bkt_find;      // find entries for bucket eviction
    reg        ht_cmd_remap;         // remap: move entry between bucket chains

    // HT command parameters (driven by main FSM only — read by latch capture)
    reg [SLOT_AW-1:0]  ht_cmd_addr;       // slot address for lookup/insert/delete
    reg [BUCKET_W-1:0] ht_cmd_bkt;        // target bucket for insert/bkt_find
    reg [PTR_W-1:0]    ht_cmd_idx;        // stash index for insert/delete
    reg [BUCKET_W-1:0] ht_old_bkt;        // for remap: old bucket
    reg [BUCKET_W-1:0] ht_new_bkt;        // for remap: new bucket

    // HT FSM working registers (driven by HT FSM only)
    reg [SLOT_AW-1:0]  ht_slot_addr;
    reg [BUCKET_W-1:0] ht_target_bkt;
    reg [PTR_W-1:0]    ht_stash_idx;

    // HT results
    reg        ht_done;
    reg        ht_slot_found;
    reg [PTR_W-1:0]   ht_found_idx;
    reg [BUCKET_W-1:0] ht_found_bkt;

    // BKT find results: list of eviction candidates
    reg [PTR_W-1:0]   ht_evict_list [0:7];  // up to Z=8 candidates
    reg [3:0]          ht_evict_count;
    reg                ht_bkt_find_done;   // Step 4: latched flag for BKT_FIND completion

    // Internal HT state
    reg [2:0]  ht_state;
    reg [2:0]  ht_op;                // which operation is in progress
    reg [AXI_AW-1:0] ht_addr;       // current HBM address
    reg [AXI_DW-1:0] ht_beat;       // read-back data beat
    reg [10:0] ht_hash;              // hash value
    reg [2:0]  ht_sub_op;            // sub-step within an operation (Step 3: 3-bit)
    reg [3:0]  ht_chain_count;       // chain traversal counter

    // SLOT write-back cache: 4-entry cache (one per SLOT beat).
    reg                 ht_wb_valid [0:3];
    reg [AXI_AW-1:0]   ht_wb_addr  [0:3];
    reg [AXI_DW-1:0]   ht_wb_data  [0:3];

    // Overlay: check all 4 cache entries for address match
    wire [1:0] ht_wb_idx = st_sng_addr[6:5];
    wire ht_wb_hit = ht_wb_valid[ht_wb_idx] && (st_sng_addr == ht_wb_addr[ht_wb_idx]);
    wire [AXI_DW-1:0] ht_slot_rdata = ht_wb_hit ? ht_wb_data[ht_wb_idx] : st_sng_rdata;

    // Module-level temporaries for HT FSM
    integer          ht_j;
    reg              ht_inserted;
    reg              ht_found;
    reg [AXI_DW-1:0] ht_new_beat;
    reg [15:0]       ht_old_head;
    reg [15:0]       ht_head_val;
    reg [15:0]       ht_next_val;
    reg [3:0]        ht_slot_in_beat;

    localparam HT_OP_NONE       = 3'd0;
    localparam HT_OP_SLOT_LOOKUP = 3'd1;
    localparam HT_OP_SLOT_INSERT = 3'd2;
    localparam HT_OP_SLOT_DELETE = 3'd3;
    localparam HT_OP_BKT_FIND   = 3'd4;
    localparam HT_OP_BKT_INSERT = 3'd5;
    localparam HT_OP_BKT_DELETE = 3'd6;

    // Helper: compute SLOT hash table HBM address from slot_addr
    function [AXI_AW-1:0] ht_slot_hbm_addr;
        input [SLOT_AW-1:0] addr;
        reg [10:0] h;
        begin
            h = addr[22:12];
            ht_slot_hbm_addr = `HT_SLOT_BASE + ({23'b0, h[10:2]} << 5);
        end
    endfunction

    // Helper: compute BKT head array HBM address from bucket_id
    function [AXI_AW-1:0] ht_bkt_head_addr;
        input [BUCKET_W-1:0] bkt;
        begin
            ht_bkt_head_addr = `HT_BKT_HEAD_BASE + ({21'b0, bkt[BUCKET_W-1:4]} << 5);
        end
    endfunction

    // Helper: compute BKT next array HBM address from stash_idx
    function [AXI_AW-1:0] ht_bkt_next_addr;
        input [PTR_W-1:0] idx;
        begin
            ht_bkt_next_addr = `HT_BKT_NEXT_BASE + ({24'b0, idx[PTR_W-1:4]} << 5);
        end
    endfunction

    // Latched commands
    reg ht_latch_slot_lookup, ht_latch_slot_insert, ht_latch_slot_delete;
    reg ht_latch_bkt_find;
    reg [SLOT_AW-1:0]  ht_latch_lu_addr;
    reg [SLOT_AW-1:0]  ht_latch_ins_addr;
    reg [BUCKET_W-1:0] ht_latch_ins_bkt;
    reg [PTR_W-1:0]    ht_latch_ins_idx;
    reg [SLOT_AW-1:0]  ht_latch_del_addr;
    reg [PTR_W-1:0]    ht_latch_del_idx;
    reg [BUCKET_W-1:0] ht_latch_find_bkt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ht_state <= HT_IDLE;
            ht_op <= HT_OP_NONE;
            ht_done <= 0;
            ht_slot_found <= 0;
            ht_found_idx <= 0;
            ht_found_bkt <= 0;
            ht_evict_count <= 0;
            ht_bkt_find_done <= 0;   // Step 4
            ht_sub_op <= 0;
            ht_chain_count <= 0;
            st_sng_rd_req <= 0;
            st_sng_wr_req <= 0;
            st_sng_addr <= 0;
            st_sng_wdata <= 0;
            ht_latch_slot_lookup <= 0;
            ht_latch_slot_insert <= 0;
            ht_latch_slot_delete <= 0;
            ht_latch_bkt_find <= 0;
            ht_wb_valid[0] <= 0; ht_wb_valid[1] <= 0;
            ht_wb_valid[2] <= 0; ht_wb_valid[3] <= 0;
        end else begin
            ht_done <= 0;
            st_sng_rd_req <= 0;
            st_sng_wr_req <= 0;

            // Latch incoming commands
            if (ht_cmd_slot_lookup) begin
                ht_latch_slot_lookup <= 1;
                ht_latch_lu_addr <= ht_cmd_addr;
            end
            if (ht_cmd_slot_insert) begin
                ht_latch_slot_insert <= 1;
                ht_latch_ins_addr <= ht_cmd_addr;
                ht_latch_ins_bkt <= ht_cmd_bkt;
                ht_latch_ins_idx <= ht_cmd_idx;
                `ifndef SYNTHESIS
                $display("[HT_LATCH] t=%0t INSERT slot=0x%08h bkt=%0d idx=%0d",
                         $time, ht_cmd_addr, ht_cmd_bkt, ht_cmd_idx);
                `endif
            end
            if (ht_cmd_slot_delete) begin
                ht_latch_slot_delete <= 1;
                ht_latch_del_addr <= ht_cmd_addr;
                ht_latch_del_idx <= ht_cmd_idx;
            end
            if (ht_cmd_bkt_find) begin
                ht_latch_bkt_find <= 1;
                ht_latch_find_bkt <= ht_cmd_bkt;
            end

            case (ht_state)

            HT_IDLE: begin
                ht_done <= 0;
                `ifndef SYNTHESIS
                if ((ht_latch_slot_insert || ht_latch_slot_lookup || ht_latch_slot_delete)
                    && (st_burst_busy ||
                    state == S_ST_LOAD || state == S_ST_FLUSH ||
                    state == S_DDR_READ || state == S_DDR_WRITE))
                    $display("[HT_GUARD] t=%0t blocked: burst_busy=%0d state=%0d ins=%0d lu=%0d del=%0d",
                             $time, st_burst_busy, state,
                             ht_latch_slot_insert, ht_latch_slot_lookup, ht_latch_slot_delete);
                `endif
                if (!st_burst_busy) begin
                    // Priority: INSERT > DELETE > LOOKUP > BKT_FIND
                    if (ht_latch_slot_insert) begin
                        ht_latch_slot_insert <= 0;
                        ht_slot_addr <= ht_latch_ins_addr;
                        ht_target_bkt <= ht_latch_ins_bkt;
                        ht_stash_idx <= ht_latch_ins_idx;
                        ht_hash <= ht_latch_ins_addr[22:12];
                        ht_addr <= ht_slot_hbm_addr(ht_latch_ins_addr);
                        st_sng_addr <= ht_slot_hbm_addr(ht_latch_ins_addr);
                        st_sng_rd_req <= 1;
                        ht_op <= HT_OP_SLOT_INSERT;
                        ht_sub_op <= 0;
                        ht_state <= HT_WAIT_RD;
                        `ifndef SYNTHESIS
                        $display("[HT_DISPATCH] t=%0t INSERT slot=0x%08h bkt=%0d idx=%0d addr=0x%0h",
                                 $time, ht_latch_ins_addr, ht_latch_ins_bkt,
                                 ht_latch_ins_idx, ht_slot_hbm_addr(ht_latch_ins_addr));
                        `endif
                    end else if (ht_latch_slot_delete) begin
                        ht_latch_slot_delete <= 0;
                        ht_slot_addr <= ht_latch_del_addr;
                        ht_stash_idx <= ht_latch_del_idx;
                        ht_hash <= ht_latch_del_addr[22:12];
                        ht_addr <= ht_slot_hbm_addr(ht_latch_del_addr);
                        st_sng_addr <= ht_slot_hbm_addr(ht_latch_del_addr);
                        st_sng_rd_req <= 1;
                        ht_op <= HT_OP_SLOT_DELETE;
                        ht_sub_op <= 0;
                        ht_state <= HT_WAIT_RD;
                    end else if (ht_latch_slot_lookup) begin
                        ht_latch_slot_lookup <= 0;
                        ht_slot_addr <= ht_latch_lu_addr;
                        ht_hash <= ht_latch_lu_addr[22:12];
                        ht_addr <= ht_slot_hbm_addr(ht_latch_lu_addr);
                        st_sng_addr <= ht_slot_hbm_addr(ht_latch_lu_addr);
                        st_sng_rd_req <= 1;
                        ht_op <= HT_OP_SLOT_LOOKUP;
                        ht_sub_op <= 0;
                        ht_state <= HT_WAIT_RD;
                    end else if (ht_latch_bkt_find) begin
                        ht_latch_bkt_find <= 0;
                        ht_bkt_find_done <= 0;   // Step 4: clear latched flag on new dispatch
                        ht_target_bkt <= ht_latch_find_bkt;
                        ht_addr <= ht_bkt_head_addr(ht_latch_find_bkt);
                        st_sng_addr <= ht_bkt_head_addr(ht_latch_find_bkt);
                        st_sng_rd_req <= 1;
                        ht_op <= HT_OP_BKT_FIND;
                        ht_evict_count <= 0;
                        ht_sub_op <= 0;
                        ht_state <= HT_WAIT_RD;
                        `ifndef SYNTHESIS
                        $display("[HT_DISPATCH] t=%0t BKT_FIND bkt=%0d addr=0x%0h",
                                 $time, ht_latch_find_bkt, ht_bkt_head_addr(ht_latch_find_bkt));
                        `endif
                    end
                end
            end

            HT_WAIT_RD: begin
                if (!st_sng_done)
                    st_sng_rd_req <= 1;
                else
                    st_sng_rd_req <= 0;
                if (st_sng_done) begin
                    ht_beat <= st_sng_rdata;

                    case (ht_op)

                    HT_OP_SLOT_LOOKUP: begin
                        ht_slot_found <= 0;
                        ht_found = 0;
                        `ifndef SYNTHESIS
                        $display("[HT_LOOKUP_RD] t=%0t slot=0x%08h addr=0x%0h hash=%0d rdata[63:0]=0x%016h entry[0..3]valid=%b%b%b%b wb_hit=%b",
                                 $time, ht_slot_addr, ht_addr, ht_hash,
                                 ht_slot_rdata[63:0],
                                 ht_slot_rdata[63], ht_slot_rdata[127], ht_slot_rdata[191], ht_slot_rdata[255],
                                 ht_wb_hit);
                        `endif
                        for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                            if (ht_slot_rdata[ht_j*64 + 63] &&
                                ht_slot_rdata[ht_j*64+8 +: 32] == ht_slot_addr) begin
                                ht_slot_found <= 1;
                                ht_found = 1;
                                ht_found_idx <= ht_slot_rdata[ht_j*64+53 +: 10];
                                ht_found_bkt <= ht_slot_rdata[ht_j*64+40 +: 13];
                            end
                        end
                        // Step 5: HT authoritative, no shadow
                        `ifndef SYNTHESIS
                        $display("[HT_LOOKUP] t=%0t slot=0x%08h: HT=%s idx=%0d bkt=%0d",
                                 $time, ht_slot_addr,
                                 ht_found ? "HIT" : "MISS",
                                 ht_found_idx, ht_found_bkt);
                        `endif
                        ht_done <= 1;
                        ht_state <= HT_IDLE;
                    end

                    HT_OP_SLOT_INSERT: begin
                        if (ht_sub_op == 0) begin
                            ht_inserted = 0;
                            ht_new_beat = ht_slot_rdata;
                            // First: check if slot_addr already exists (update in place)
                            for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                                if (!ht_inserted &&
                                    ht_slot_rdata[ht_j*64+8 +: 32] == ht_slot_addr) begin
                                    ht_new_beat[ht_j*64 + 63] = 1'b1;
                                    ht_new_beat[ht_j*64+53 +: 10] = ht_stash_idx;
                                    ht_new_beat[ht_j*64+40 +: 13] = ht_target_bkt;
                                    ht_new_beat[ht_j*64+8 +: 32] = ht_slot_addr;
                                    ht_new_beat[ht_j*64 +: 8] = 8'b0;
                                    ht_inserted = 1;
                                end
                            end
                            // Second: find empty slot if no existing entry
                            if (!ht_inserted) begin
                                for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                                    if (!ht_inserted && !ht_slot_rdata[ht_j*64 + 63]) begin
                                        ht_new_beat[ht_j*64 + 63] = 1'b1;
                                        ht_new_beat[ht_j*64+53 +: 10] = ht_stash_idx;
                                        ht_new_beat[ht_j*64+40 +: 13] = ht_target_bkt;
                                        ht_new_beat[ht_j*64+8 +: 32] = ht_slot_addr;
                                        ht_new_beat[ht_j*64 +: 8] = 8'b0;
                                        ht_inserted = 1;
                                    end
                                end
                            end
                            if (ht_inserted) begin
                                st_sng_addr <= ht_addr;
                                st_sng_wdata <= ht_new_beat;
                                st_sng_wr_req <= 1;
                                ht_wb_valid[ht_addr[6:5]] <= 1;
                                ht_wb_addr[ht_addr[6:5]] <= ht_addr;
                                ht_wb_data[ht_addr[6:5]] <= ht_new_beat;
                                ht_sub_op <= 1;
                                ht_state <= HT_WAIT_WR;
                                `ifndef SYNTHESIS
                                $display("[HT_INSERT_WR] t=%0t slot=0x%08h addr=0x%0h hash=%0d entry[0..3]valid=%b%b%b%b",
                                         $time, ht_slot_addr, ht_addr, ht_hash,
                                         ht_new_beat[63], ht_new_beat[127], ht_new_beat[191], ht_new_beat[255]);
                                `endif
                            end else begin
                                `ifndef SYNTHESIS
                                $display("[HT_WARN] t=%0t SLOT INSERT: no empty slot at hash=%0d", $time, ht_hash);
                                `endif
                                ht_done <= 1;
                                ht_state <= HT_IDLE;
                            end
                        end else if (ht_sub_op == 2) begin
                            // Read BKT head result
                            ht_slot_in_beat = ht_target_bkt[3:0];
                            ht_old_head = st_sng_rdata[ht_slot_in_beat*16 +: 16];
                            ht_new_beat = st_sng_rdata;
                            ht_new_beat[ht_slot_in_beat*16 +: 16] = {6'b0, ht_stash_idx};
                            st_sng_addr <= ht_bkt_head_addr(ht_target_bkt);
                            st_sng_wdata <= ht_new_beat;
                            st_sng_wr_req <= 1;
                            ht_beat <= {{(AXI_DW-16){1'b0}}, ht_old_head};
                            ht_sub_op <= 3;
                            ht_state <= HT_WAIT_WR;
                        end else if (ht_sub_op == 4) begin
                            // Read next array result
                            ht_slot_in_beat = ht_stash_idx[3:0];
                            ht_new_beat = st_sng_rdata;
                            ht_new_beat[ht_slot_in_beat*16 +: 16] = ht_beat[15:0];
                            st_sng_addr <= ht_bkt_next_addr(ht_stash_idx);
                            st_sng_wdata <= ht_new_beat;
                            st_sng_wr_req <= 1;
                            ht_sub_op <= 5;
                            ht_state <= HT_WAIT_WR;
                        end
                    end

                    HT_OP_SLOT_DELETE: begin
                        if (ht_sub_op == 0) begin
                            ht_found = 0;
                            ht_new_beat = ht_slot_rdata;
                            for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                                if (!ht_found && ht_slot_rdata[ht_j*64 + 63] &&
                                    ht_slot_rdata[ht_j*64+8 +: 32] == ht_slot_addr) begin
                                    ht_new_beat[ht_j*64 +: 64] = 64'b0;
                                    ht_found = 1;
                                end
                            end
                            if (ht_found) begin
                                st_sng_addr <= ht_addr;
                                st_sng_wdata <= ht_new_beat;
                                st_sng_wr_req <= 1;
                                ht_wb_valid[ht_addr[6:5]] <= 1;
                                ht_wb_addr[ht_addr[6:5]] <= ht_addr;
                                ht_wb_data[ht_addr[6:5]] <= ht_new_beat;
                                ht_sub_op <= 1;
                                ht_state <= HT_WAIT_WR;
                            end else begin
                                `ifndef SYNTHESIS
                                $display("[HT_WARN] t=%0t SLOT DELETE: not found addr=0x%08h", $time, ht_slot_addr);
                                `endif
                                ht_done <= 1;
                                ht_state <= HT_IDLE;
                            end
                        end
                    end

                    HT_OP_BKT_FIND: begin
                        ht_slot_in_beat = ht_target_bkt[3:0];
                        ht_head_val = st_sng_rdata[ht_slot_in_beat*16 +: 16];
                        if (ht_head_val[9:0] == `HT_EMPTY || ht_head_val == 16'hFFFF) begin
                            `ifndef SYNTHESIS
                            $display("[HT_BKT_FIND] t=%0t bkt=%0d empty chain",
                                     $time, ht_target_bkt);
                            `endif
                            ht_done <= 1;
                            ht_bkt_find_done <= 1;   // Step 4: latch completion
                            ht_state <= HT_IDLE;
                        end else begin
                            ht_evict_list[0] <= ht_head_val[PTR_W-1:0];
                            ht_evict_count <= 1;
                            st_sng_addr <= ht_bkt_next_addr(ht_head_val[PTR_W-1:0]);
                            st_sng_rd_req <= 1;
                            ht_state <= HT_BKT_CHAIN;
                        end
                    end

                    default: begin
                        ht_done <= 1;
                        ht_state <= HT_IDLE;
                    end
                    endcase
                end
            end

            HT_WAIT_WR: begin
                if (!st_sng_done)
                    st_sng_wr_req <= 1;
                else
                    st_sng_wr_req <= 0;
                if (st_sng_done) begin
                    case (ht_op)

                    HT_OP_SLOT_INSERT: begin
                        if (ht_sub_op == 1) begin
                            // SLOT write done, now insert into BKT chain
                            st_sng_addr <= ht_bkt_head_addr(ht_target_bkt);
                            st_sng_rd_req <= 1;
                            ht_sub_op <= 2;
                            ht_state <= HT_WAIT_RD;
                        end else if (ht_sub_op == 3) begin
                            // BKT head write done, now update next[new_idx]
                            st_sng_addr <= ht_bkt_next_addr(ht_stash_idx);
                            st_sng_rd_req <= 1;
                            ht_sub_op <= 4;
                            ht_state <= HT_WAIT_RD;
                        end else if (ht_sub_op == 5) begin
                            // All done: SLOT + BKT insert complete
                            `ifndef SYNTHESIS
                            $display("[HT] t=%0t INSERT slot=0x%08h bkt=%0d idx=%0d done",
                                     $time, ht_slot_addr, ht_target_bkt, ht_stash_idx);
                            `endif
                            ht_done <= 1;
                            ht_state <= HT_IDLE;
                        end
                    end

                    HT_OP_SLOT_DELETE: begin
                        if (ht_sub_op == 1) begin
                            // SLOT delete done (lazy: don't unlink from BKT chain)
                            `ifndef SYNTHESIS
                            $display("[HT] t=%0t DELETE slot=0x%08h idx=%0d done",
                                     $time, ht_slot_addr, ht_stash_idx);
                            `endif
                            ht_done <= 1;
                            ht_state <= HT_IDLE;
                        end
                    end

                    default: begin
                        ht_done <= 1;
                        ht_state <= HT_IDLE;
                    end
                    endcase
                end
            end

            HT_BKT_CHAIN: begin
                // Hold read request until stash_axi_master accepts it
                // (ht_safe gate may block during unsafe FSM states)
                if (!st_sng_done)
                    st_sng_rd_req <= 1;  // keep asserted until accepted
                else
                    st_sng_rd_req <= 0;
                if (st_sng_done) begin
                    ht_slot_in_beat = ht_evict_list[ht_evict_count-1][3:0];
                    ht_next_val = st_sng_rdata[ht_slot_in_beat*16 +: 16];

                    if (ht_next_val[9:0] == `HT_EMPTY || ht_next_val == 16'hFFFF ||
                        ht_evict_count >= `ORAM_Z) begin
                        `ifndef SYNTHESIS
                        $display("[HT_BKT_FIND] t=%0t bkt=%0d found %0d candidates",
                                 $time, ht_target_bkt, ht_evict_count);
                        `endif
                        ht_done <= 1;
                        ht_bkt_find_done <= 1;   // Step 4: latch completion
                        ht_state <= HT_IDLE;
                    end else begin
                        ht_evict_list[ht_evict_count] <= ht_next_val[PTR_W-1:0];
                        ht_evict_count <= ht_evict_count + 1;
                        st_sng_addr <= ht_bkt_next_addr(ht_next_val[PTR_W-1:0]);
                        st_sng_rd_req <= 1;
                        // Stay in HT_BKT_CHAIN
                    end
                end
            end

            default: begin
                ht_state <= HT_IDLE;
            end
            endcase
        end
    end

    // =========================================================================
    // AES FEED DIAGNOSTIC — minimal Verilator-safe version
    // =========================================================================
    `ifndef SYNTHESIS
    reg [15:0] fd_total, fd_feeding, fd_input_nr, fd_data_nr, fd_output;
    reg [15:0] fd_drd_wait, fd_scr_wait, fd_half_lo, fd_half_hi;
    reg [15:0] fd_start_total;
    reg [4:0]  fd_prev;
    reg        fd_alive;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fd_total <= 0; fd_feeding <= 0; fd_input_nr <= 0;
            fd_data_nr <= 0; fd_output <= 0; fd_drd_wait <= 0;
            fd_scr_wait <= 0; fd_half_lo <= 0; fd_half_hi <= 0;
            fd_start_total <= 0; fd_prev <= 0; fd_alive <= 0;
        end else begin
            // One-time alive check
            if (!fd_alive && state != 5'd0) begin
                fd_alive <= 1;
                $display("[AES_DIAG] ALIVE: diag block running, state=%0d", state);
            end

            // --- START states: 6, 15, 20, 24 ---
            if (state == 5'd6 || state == 5'd15 || state == 5'd20 || state == 5'd24) begin
                if (fd_prev != state)
                    fd_start_total <= 1;
                else
                    fd_start_total <= fd_start_total + 1;
            end

            // Print on exit from START
            if ((fd_prev == 5'd6 || fd_prev == 5'd15 || fd_prev == 5'd20 || fd_prev == 5'd24) &&
                state != fd_prev)
                $display("[AES_DIAG] START(st=%0d): total=%0d ir=%0d",
                         fd_prev, fd_start_total, gcm_input_ready);

            // --- FEED states: 7, 16, 21, 25 ---
            if (state == 5'd7 || state == 5'd16 || state == 5'd21 || state == 5'd25) begin
                if (fd_prev != state) begin
                    // Entry: reset
                    fd_total <= 1; fd_feeding <= 0; fd_input_nr <= 0;
                    fd_data_nr <= 0; fd_output <= 0; fd_drd_wait <= 0;
                    fd_scr_wait <= 0; fd_half_lo <= 0; fd_half_hi <= 0;
                    if (gcm_data_valid) fd_feeding <= 1;
                    if (!gcm_input_ready) fd_input_nr <= 1;
                    if (gcm_input_ready && !gcm_data_valid) fd_data_nr <= 1;
                    if (gcm_data_out_valid) fd_output <= 1;
                    if (!aes_half) fd_half_lo <= 1;
                    else fd_half_hi <= 1;
                    if ((state == 5'd16 || state == 5'd21) && !st_drd_valid && !aes_half)
                        fd_drd_wait <= 1;
                    if (state == 5'd7 && !scr_fsm_rd_valid && !aes_half)
                        fd_scr_wait <= 1;
                end else begin
                    // Steady: count
                    fd_total <= fd_total + 1;
                    if (gcm_data_valid) fd_feeding <= fd_feeding + 1;
                    if (!gcm_input_ready) fd_input_nr <= fd_input_nr + 1;
                    if (gcm_input_ready && !gcm_data_valid) fd_data_nr <= fd_data_nr + 1;
                    if (gcm_data_out_valid) fd_output <= fd_output + 1;
                    if (!aes_half) fd_half_lo <= fd_half_lo + 1;
                    else fd_half_hi <= fd_half_hi + 1;
                    if ((state == 5'd16 || state == 5'd21) && !st_drd_valid && !aes_half)
                        fd_drd_wait <= fd_drd_wait + 1;
                    if (state == 5'd7 && !scr_fsm_rd_valid && !aes_half)
                        fd_scr_wait <= fd_scr_wait + 1;
                end
            end

            // Print on exit from FEED
            if ((fd_prev == 5'd7 || fd_prev == 5'd16 || fd_prev == 5'd21 || fd_prev == 5'd25) &&
                state != fd_prev) begin
                $display("[AES_DIAG] FEED(st=%0d): total=%0d feeding=%0d input_nr=%0d data_nr=%0d output=%0d",
                         fd_prev, fd_total, fd_feeding, fd_input_nr, fd_data_nr, fd_output);
                $display("[AES_DIAG]   drd_wait=%0d scr_wait=%0d lo=%0d hi=%0d",
                         fd_drd_wait, fd_scr_wait, fd_half_lo, fd_half_hi);
            end

            fd_prev <= state;
        end
    end
    `endif

endmodule