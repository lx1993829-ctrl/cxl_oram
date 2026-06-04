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
    output wire [5:0]            dbg_state,
    output wire                  dbg_client_done,
    output wire                  dbg_client_req,
    output wire [BUCKET_W-1:0]  dbg_req_b,
    output wire [BUCKET_W-1:0]  dbg_req_b_new,
    output wire                  dbg_found_in_bucket,
    output wire                  dbg_found_in_stash,
    output wire                  dbg_same_bucket,
    output wire                  dbg_err_stash_ovf,
    output wire [PTR_W:0]       dbg_stash_occ,
    // HT operation tracking — per-op counters and overwrite detection
    output wire                  dbg_ht_ins_overwritten,   // sticky: INSERT latch was overwritten
    output wire                  dbg_ht_del_overwritten,   // sticky: DELETE latch was overwritten
    output wire [7:0]            dbg_ht_ins_issued,        // per-op INSERT commands issued
    output wire [7:0]            dbg_ht_ins_completed,     // per-op INSERT commands completed
    output wire [7:0]            dbg_ht_del_issued,        // per-op DELETE commands issued
    output wire [7:0]            dbg_ht_del_completed,     // per-op DELETE commands completed
    output wire                  dbg_ht_latch_ins_active,  // INSERT currently latched
    output wire                  dbg_ht_latch_del_active,  // DELETE currently latched
    // HT LOOKUP diagnostic: last lookup address, result, and entry valid bits
    output reg  [AXI_AW-1:0]    dbg_ht_lu_hbm_addr,      // HBM address read during last LOOKUP
    output reg  [3:0]            dbg_ht_lu_valid_bits,     // valid bits of 4 entries at that address
    output reg                   dbg_ht_lu_wb_hit,         // was write-back buffer used?
    output reg  [SLOT_AW-1:0]   dbg_ht_lu_slot_looked_up, // slot address looked up

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
    input  wire                     metadata_idle,  // safe for stash burst
    input  wire [PTR_W-1:0]        st_ext_lb_entry, // latched entry for line buffer addressing
    input  wire                     st_burst_busy,

    // Single-beat hash table access (via stash_axi_master)
    output reg                      st_sng_rd_req,
    output reg                      st_sng_wr_req,
    output reg  [AXI_AW-1:0]       st_sng_addr,
    output reg  [AXI_DW-1:0]       st_sng_wdata,
    input  wire [AXI_DW-1:0]       st_sng_rdata,
    input  wire                     st_sng_done,
    input  wire                     st_sng_accepted,

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

    // IV/TAG table (v2) AXI master -> mux (mirrors pm_axi_*)
    output wire [`AXI_ID_W-1:0]    ivt_axi_arid,
    output wire [`AXI_ADDR_W-1:0]  ivt_axi_araddr,
    output wire [`AXI_LEN_W-1:0]   ivt_axi_arlen,
    output wire [2:0]               ivt_axi_arsize,
    output wire [1:0]               ivt_axi_arburst,
    output wire                     ivt_axi_arvalid,
    input  wire                     ivt_axi_arready,
    input  wire [`AXI_ID_W-1:0]    ivt_axi_rid,
    input  wire [`AXI_DATA_W-1:0]  ivt_axi_rdata,
    input  wire [1:0]               ivt_axi_rresp,
    input  wire                     ivt_axi_rlast,
    input  wire                     ivt_axi_rvalid,
    output wire                     ivt_axi_rready,
    output wire [`AXI_ID_W-1:0]    ivt_axi_awid,
    output wire [`AXI_ADDR_W-1:0]  ivt_axi_awaddr,
    output wire [`AXI_LEN_W-1:0]   ivt_axi_awlen,
    output wire [2:0]               ivt_axi_awsize,
    output wire [1:0]               ivt_axi_awburst,
    output wire                     ivt_axi_awvalid,
    input  wire                     ivt_axi_awready,
    output wire [`AXI_DATA_W-1:0]  ivt_axi_wdata,
    output wire [`AXI_STRB_W-1:0]  ivt_axi_wstrb,
    output wire                     ivt_axi_wlast,
    output wire                     ivt_axi_wvalid,
    input  wire                     ivt_axi_wready,
    input  wire [`AXI_ID_W-1:0]    ivt_axi_bid,
    input  wire [1:0]               ivt_axi_bresp,
    input  wire                     ivt_axi_bvalid,
    output wire                     ivt_axi_bready,
    output wire                     ivt_busy,
    output wire [2:0]               ivt_dbg_state,

    // slot_r table (per-stash-entry) AXI master -> mux (mirrors ivt_axi_*)
    output wire [`AXI_ID_W-1:0]    sr_axi_arid,
    output wire [`AXI_ADDR_W-1:0]  sr_axi_araddr,
    output wire [`AXI_LEN_W-1:0]   sr_axi_arlen,
    output wire [2:0]               sr_axi_arsize,
    output wire [1:0]               sr_axi_arburst,
    output wire                     sr_axi_arvalid,
    input  wire                     sr_axi_arready,
    input  wire [`AXI_ID_W-1:0]    sr_axi_rid,
    input  wire [`AXI_DATA_W-1:0]  sr_axi_rdata,
    input  wire [1:0]               sr_axi_rresp,
    input  wire                     sr_axi_rlast,
    input  wire                     sr_axi_rvalid,
    output wire                     sr_axi_rready,
    output wire [`AXI_ID_W-1:0]    sr_axi_awid,
    output wire [`AXI_ADDR_W-1:0]  sr_axi_awaddr,
    output wire [`AXI_LEN_W-1:0]   sr_axi_awlen,
    output wire [2:0]               sr_axi_awsize,
    output wire [1:0]               sr_axi_awburst,
    output wire                     sr_axi_awvalid,
    input  wire                     sr_axi_awready,
    output wire [`AXI_DATA_W-1:0]  sr_axi_wdata,
    output wire [`AXI_STRB_W-1:0]  sr_axi_wstrb,
    output wire                     sr_axi_wlast,
    output wire                     sr_axi_wvalid,
    input  wire                     sr_axi_wready,
    input  wire [`AXI_ID_W-1:0]    sr_axi_bid,
    input  wire [1:0]               sr_axi_bresp,
    input  wire                     sr_axi_bvalid,
    output wire                     sr_axi_bready,
    output wire                     slotr_busy,
    output wire [2:0]               slotr_dbg_state,

    // bucket_meta (per-bucket) AXI master -> mux (mirrors ivt_axi_*)
    output wire [`AXI_ID_W-1:0]    bm_axi_arid,
    output wire [`AXI_ADDR_W-1:0]  bm_axi_araddr,
    output wire [`AXI_LEN_W-1:0]   bm_axi_arlen,
    output wire [2:0]               bm_axi_arsize,
    output wire [1:0]               bm_axi_arburst,
    output wire                     bm_axi_arvalid,
    input  wire                     bm_axi_arready,
    input  wire [`AXI_ID_W-1:0]    bm_axi_rid,
    input  wire [`AXI_DATA_W-1:0]  bm_axi_rdata,
    input  wire [1:0]               bm_axi_rresp,
    input  wire                     bm_axi_rlast,
    input  wire                     bm_axi_rvalid,
    output wire                     bm_axi_rready,
    output wire [`AXI_ID_W-1:0]    bm_axi_awid,
    output wire [`AXI_ADDR_W-1:0]  bm_axi_awaddr,
    output wire [`AXI_LEN_W-1:0]   bm_axi_awlen,
    output wire [2:0]               bm_axi_awsize,
    output wire [1:0]               bm_axi_awburst,
    output wire                     bm_axi_awvalid,
    input  wire                     bm_axi_awready,
    output wire [`AXI_DATA_W-1:0]  bm_axi_wdata,
    output wire [`AXI_STRB_W-1:0]  bm_axi_wstrb,
    output wire                     bm_axi_wlast,
    output wire                     bm_axi_wvalid,
    input  wire                     bm_axi_wready,
    input  wire [`AXI_ID_W-1:0]    bm_axi_bid,
    input  wire [1:0]               bm_axi_bresp,
    input  wire                     bm_axi_bvalid,
    output wire                     bm_axi_bready,
    output wire                     bmeta_busy,
    output wire [2:0]               bmeta_dbg_state,

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
    localparam [5:0]
        S_IDLE          = 6'd0,
        S_POS_LOOKUP    = 6'd1,
        S_DDR_READ      = 6'd2,
        S_SCAN          = 6'd3,
        S_SCAN2         = 6'd4,
        S_EXT_IV_RD     = 6'd5,
        S_EXT_DEC_START = 6'd6,
        S_EXT_DEC_FEED  = 6'd7,
        S_EXT_DEC_RECV  = 6'd8,
        S_EXT_DEC_WAIT  = 6'd9,
        S_EXTRACT_WR    = 6'd10,
        S_STASH_SEARCH  = 6'd11,
        S_STASH_READ    = 6'd12,
        S_COMPACT       = 6'd13,
        S_EVICT         = 6'd14,
        S_EV_ENC_START  = 6'd15,
        S_EV_ENC_FEED   = 6'd16,
        S_EV_ENC_RECV   = 6'd17,
        S_EV_ENC_TAG    = 6'd18,
        S_DDR_WRITE     = 6'd19,
        S_SB_RD_ENC_START = 6'd20,
        S_SB_RD_ENC_FEED  = 6'd21,
        S_SB_RD_ENC_RECV  = 6'd22,
        S_SB_RD_ENC_TAG   = 6'd23,
        S_SB_WR_ENC_START = 6'd24,
        S_SB_WR_ENC_FEED  = 6'd25,
        S_SB_WR_ENC_RECV  = 6'd26,
        S_SB_WR_ENC_TAG   = 6'd27,
        S_ST_LOAD       = 6'd28,
        S_ST_FLUSH      = 6'd29,
        S_PM_WAIT       = 6'd30,
        S_HT_LOOKUP_WAIT = 6'd31,  // Step 4: wait for HT slot lookup result
        S_EVICT_SLOTR_WAIT = 6'd32;  // wait slot_r HBM read at eviction

    // Hash table FSM states
    localparam [2:0]
        HT_IDLE         = 3'd0,
        HT_WAIT_RD      = 3'd1,
        HT_WAIT_WR      = 3'd2,
        HT_BKT_CHAIN    = 3'd3,
        HT_SLOT_PROBE   = 3'd4,
        HT_WAIT_RD_SETTLE = 3'd5;  // 1-cycle settle: st_sng_done asserts a cycle
                                   // before st_sng_rdata is stable, so sample the
                                   // settled value here, not on the done edge.

    reg [5:0] state;
    reg [5:0] st_return_state;

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
    reg [POS_W-1:0]     ev_pos;            // coherent placement pos (lowest free hole) for current eviction
    // bucket_meta moved to HBM: rd_valid is a 1-cycle pulse that may arrive
    // before OR after the bucket data burst finishes. This sticky flag records
    // that the metadata read has returned so S_DDR_READ can gate its exit on
    // BOTH the data burst (axim_read_done) and the metadata (bm_rd_done).
    reg                 bm_rd_done;
    reg                 remap_phase;   // two-phase remap: 0=DELETE old, 1=INSERT new

    // Eviction valid-check: combinational read of stash valid_bm
    // to skip eviction of already-freed entries (stale chain links).
    wire [PTR_W-1:0] ev_valid_check_idx = ht_evict_list[evict_list_idx];
    wire             ev_valid_check_out;

    // Per-stash-entry assigned bucket: recorded at INSERT, checked at S_EVICT
    // to prevent evicting a block to the wrong bucket via a stale BKT_FIND
    // chain link. If stash_bkt[candidate] != req_b, the candidate belongs to
    // a different bucket and must be skipped.
    reg [BUCKET_W-1:0] stash_bkt [0:STASH_DEPTH-1];
    wire               ev_bkt_match = (stash_bkt[ev_valid_check_idx] == req_b);

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
    // =========================================================================
    // IVT now uses PHYSICAL addressing: (bucket, pos) instead of slot_addr.
    reg  [BUCKET_W-1:0]  ivt_rd_bucket;
    reg  [POS_W-1:0]     ivt_rd_pos;
    reg                  ivt_rd_en;
    wire [95:0]          ivt_rd_iv;
    wire [127:0]         ivt_rd_tag;
    wire                 ivt_rd_valid;
    reg  [BUCKET_W-1:0]  ivt_wr_bucket;
    reg  [POS_W-1:0]     ivt_wr_pos;
    reg  [95:0]          ivt_wr_iv;
    reg  [127:0]         ivt_wr_tag;
    reg                  ivt_wr_en;

    iv_slot_table u_iv_table (
        .clk(clk), .rst_n(rst_n),
        .rd_bucket(ivt_rd_bucket), .rd_pos(ivt_rd_pos), .rd_en(ivt_rd_en),
        .rd_iv(ivt_rd_iv), .rd_tag(ivt_rd_tag), .rd_valid(ivt_rd_valid),
        .wr_bucket(ivt_wr_bucket), .wr_pos(ivt_wr_pos), .wr_iv(ivt_wr_iv),
        .wr_tag(ivt_wr_tag), .wr_en(ivt_wr_en),
        .busy(ivt_busy), .dbg_state(ivt_dbg_state),
        .m_axi_arid(ivt_axi_arid), .m_axi_araddr(ivt_axi_araddr),
        .m_axi_arlen(ivt_axi_arlen), .m_axi_arsize(ivt_axi_arsize),
        .m_axi_arburst(ivt_axi_arburst), .m_axi_arvalid(ivt_axi_arvalid),
        .m_axi_arready(ivt_axi_arready),
        .m_axi_rid(ivt_axi_rid), .m_axi_rdata(ivt_axi_rdata),
        .m_axi_rresp(ivt_axi_rresp), .m_axi_rlast(ivt_axi_rlast),
        .m_axi_rvalid(ivt_axi_rvalid), .m_axi_rready(ivt_axi_rready),
        .m_axi_awid(ivt_axi_awid), .m_axi_awaddr(ivt_axi_awaddr),
        .m_axi_awlen(ivt_axi_awlen), .m_axi_awsize(ivt_axi_awsize),
        .m_axi_awburst(ivt_axi_awburst), .m_axi_awvalid(ivt_axi_awvalid),
        .m_axi_awready(ivt_axi_awready),
        .m_axi_wdata(ivt_axi_wdata), .m_axi_wstrb(ivt_axi_wstrb),
        .m_axi_wlast(ivt_axi_wlast), .m_axi_wvalid(ivt_axi_wvalid),
        .m_axi_wready(ivt_axi_wready),
        .m_axi_bid(ivt_axi_bid), .m_axi_bresp(ivt_axi_bresp),
        .m_axi_bvalid(ivt_axi_bvalid), .m_axi_bready(ivt_axi_bready)
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

    bucket_meta_hbm u_bucket_meta (
        .clk(clk), .rst_n(rst_n),
        .rd_bucket(bm_rd_bucket), .rd_en(bm_rd_en),
        .rd_slot_list(bm_rd_slot_list), .rd_fill_count(bm_rd_fill), .rd_valid(bm_rd_valid),
        .wr_bucket(bm_wr_bucket_mux), .wr_slot_list(bm_wr_slot_list_mux),
        .wr_fill_count(bm_wr_fill_mux), .wr_en(bm_wr_en_mux),
        .busy(bmeta_busy), .dbg_state(bmeta_dbg_state),
        .m_axi_arid(bm_axi_arid), .m_axi_araddr(bm_axi_araddr),
        .m_axi_arlen(bm_axi_arlen), .m_axi_arsize(bm_axi_arsize),
        .m_axi_arburst(bm_axi_arburst), .m_axi_arvalid(bm_axi_arvalid),
        .m_axi_arready(bm_axi_arready),
        .m_axi_rid(bm_axi_rid), .m_axi_rdata(bm_axi_rdata),
        .m_axi_rresp(bm_axi_rresp), .m_axi_rlast(bm_axi_rlast),
        .m_axi_rvalid(bm_axi_rvalid), .m_axi_rready(bm_axi_rready),
        .m_axi_awid(bm_axi_awid), .m_axi_awaddr(bm_axi_awaddr),
        .m_axi_awlen(bm_axi_awlen), .m_axi_awsize(bm_axi_awsize),
        .m_axi_awburst(bm_axi_awburst), .m_axi_awvalid(bm_axi_awvalid),
        .m_axi_awready(bm_axi_awready),
        .m_axi_wdata(bm_axi_wdata), .m_axi_wstrb(bm_axi_wstrb),
        .m_axi_wlast(bm_axi_wlast), .m_axi_wvalid(bm_axi_wvalid),
        .m_axi_wready(bm_axi_wready),
        .m_axi_bid(bm_axi_bid), .m_axi_bresp(bm_axi_bresp),
        .m_axi_bvalid(bm_axi_bvalid), .m_axi_bready(bm_axi_bready)
    );

    // --- slot_r table (per-stash-entry slot address, moved off-chip) ---
    reg  [PTR_W-1:0]     sr_rd_idx;  reg sr_rd_en;
    wire [SLOT_AW-1:0]   sr_rd_slot_addr; wire sr_rd_valid;
    reg  [PTR_W-1:0]     sr_wr_idx;  reg [SLOT_AW-1:0] sr_wr_slot_addr; reg sr_wr_en;

    slot_r_table u_slot_r (
        .clk(clk), .rst_n(rst_n),
        .rd_idx(sr_rd_idx), .rd_en(sr_rd_en),
        .rd_slot_addr(sr_rd_slot_addr), .rd_valid(sr_rd_valid),
        .wr_idx(sr_wr_idx), .wr_slot_addr(sr_wr_slot_addr), .wr_en(sr_wr_en),
        .busy(slotr_busy), .dbg_state(slotr_dbg_state),
        .m_axi_arid(sr_axi_arid), .m_axi_araddr(sr_axi_araddr),
        .m_axi_arlen(sr_axi_arlen), .m_axi_arsize(sr_axi_arsize),
        .m_axi_arburst(sr_axi_arburst), .m_axi_arvalid(sr_axi_arvalid),
        .m_axi_arready(sr_axi_arready),
        .m_axi_rid(sr_axi_rid), .m_axi_rdata(sr_axi_rdata),
        .m_axi_rresp(sr_axi_rresp), .m_axi_rlast(sr_axi_rlast),
        .m_axi_rvalid(sr_axi_rvalid), .m_axi_rready(sr_axi_rready),
        .m_axi_awid(sr_axi_awid), .m_axi_awaddr(sr_axi_awaddr),
        .m_axi_awlen(sr_axi_awlen), .m_axi_awsize(sr_axi_awsize),
        .m_axi_awburst(sr_axi_awburst), .m_axi_awvalid(sr_axi_awvalid),
        .m_axi_awready(sr_axi_awready),
        .m_axi_wdata(sr_axi_wdata), .m_axi_wstrb(sr_axi_wstrb),
        .m_axi_wlast(sr_axi_wlast), .m_axi_wvalid(sr_axi_wvalid),
        .m_axi_wready(sr_axi_wready),
        .m_axi_bid(sr_axi_bid), .m_axi_bresp(sr_axi_bresp),
        .m_axi_bvalid(sr_axi_bvalid), .m_axi_bready(sr_axi_bready)
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
        // CAM search ports removed — HT (in HBM) is sole slot->idx metadata store.
        .ins_slot_addr(st_ins_slot), .ins_target_bucket(st_ins_tgt), .ins_en(st_ins_en),
        .ins_idx(st_ins_idx), .ins_full(st_ins_full),
        .remap_idx(st_remap_idx), .remap_new_bucket(st_remap_bkt), .remap_en(st_remap_en),
        .evict_idx(st_evict_idx), .evict_en(st_evict_en),
        .valid_check_idx(ev_valid_check_idx), .valid_check_out(ev_valid_check_out),
        .dwr_entry(st_dwr_entry), .dwr_beat(st_dwr_beat),
        .dwr_data(st_dwr_data), .dwr_en(st_dwr_en),
        .drd_entry(st_drd_entry), .drd_beat(st_drd_beat), .drd_en(st_drd_en),
        .drd_data(st_drd_data), .drd_valid(st_drd_valid),
        .ext_rd_req(),
        .ext_wr_req(),
        .ext_entry(st_ext_lb_entry),
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

    // Lowest empty (hole) position in a bucket slot_list. With remove-in-place
    // compaction, holes can appear anywhere below the high-water mark, so the
    // next eviction must fill the lowest hole (not append at fill count) to
    // keep ciphertext/IVT position == slot_list position. Returns Z if full.
    function [POS_W:0] lowest_free_pos;
        input [Z*SLOT_W-1:0] list;
        integer p;
        reg found;
        begin
            lowest_free_pos = Z[POS_W:0];   // default: full
            found = 1'b0;
            for (p = 0; p < Z; p = p + 1)
                if (!found && (list[p*SLOT_W +: SLOT_W] == {SLOT_W{1'b0}})) begin
                    lowest_free_pos = p[POS_W:0];
                    found = 1'b1;
                end
        end
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
    assign dbg_ht_ins_overwritten  = ht_ins_overwritten_r;
    assign dbg_ht_del_overwritten  = ht_del_overwritten_r;
    assign dbg_ht_ins_issued       = ht_ins_issued_r;
    assign dbg_ht_ins_completed    = ht_ins_completed_r;
    assign dbg_ht_del_issued       = ht_del_issued_r;
    assign dbg_ht_del_completed    = ht_del_completed_r;
    assign dbg_ht_latch_ins_active = ht_latch_slot_insert;
    assign dbg_ht_latch_del_active = ht_latch_slot_delete;
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
            ev_pos <= 0;
            found_in_bucket <= 0; found_in_stash <= 0;
            found_pos <= 0; found_stash_idx <= 0; stash_alloc_idx <= 0;
            req_b <= 0; req_b_new <= 0; bkt_fill_count <= 0;
            bkt_slot_list <= 0; same_bucket <= 0;
            pm_rd_en <= 0; pm_wr_en <= 0; pm_wr_status <= `ST_DUMMY;
            bm_rd_en <= 0; bm_wr_en <= 0; prng_en <= 0;
            bm_rd_done <= 0;
            remap_phase <= 0;
            st_ins_en <= 0; st_remap_en <= 0; st_evict_en <= 0;
            st_dwr_en <= 0; st_drd_en <= 0;
            axim_cmd_read <= 0; axim_cmd_write <= 0;
            scr_fsm_rd_en <= 0; scr_fsm_wr_en <= 0;
            gcm_start <= 0; gcm_data_valid <= 0; gcm_data_last <= 0;
            gcm_encrypt <= 0; gcm_iv_reg <= 0;
            aes_half <= 0; aes_blk_cnt <= 0;
            aes_out_beat <= 0; aes_out_half <= 0;
            ivt_rd_en <= 0; ivt_wr_en <= 0;
            sr_rd_en <= 0; sr_wr_en <= 0;
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
            sr_rd_en <= 0; sr_wr_en <= 0;
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
                        bm_rd_done <= 1'b0;   // arm sticky flag for this read
                        axim_cmd_bucket <= pm_rd_bucket; axim_cmd_read <= 1;
                        `ifndef SYNTHESIS
                        $display("[PM_WR] t=%0t slot=0x%08h old_bkt=%0d new_bkt=%0d same=%b",
                                 $time, req_slot_addr, pm_rd_bucket,
                                 dbg_force_same_bucket ? pm_rd_bucket :
                                 dbg_prng_override_en  ? dbg_prng_override : prng_bucket,
                                 dbg_force_same_bucket ? 1'b1 :
                                 dbg_prng_override_en  ? (dbg_prng_override == pm_rd_bucket) :
                                                         (prng_bucket == pm_rd_bucket));
                        $display("[DDR_RD] t=%0t LAUNCH: bucket=%0d data-read + bucket_meta-read (HBM) issued in parallel -> S_DDR_READ",
                                 $time, pm_rd_bucket);
                        `endif
                        state <= S_DDR_READ;
                    end
                end

                S_DDR_READ: begin
                    if (bm_rd_valid) begin
                        bkt_slot_list <= bm_rd_slot_list;
                        bkt_fill_count <= bm_rd_fill;
                        bm_rd_done <= 1'b1;   // sticky: metadata has returned
                        `ifndef SYNTHESIS
                        $display("[DDR_RD] t=%0t bucket_meta arrived: fill=%0d slot_list=0x%030h (axim_read_done=%b so far)",
                                 $time, bm_rd_fill, bm_rd_slot_list, axim_read_done);
                        `endif
                    end
                    `ifndef SYNTHESIS
                    if (axim_read_done && !(bm_rd_done || bm_rd_valid))
                        $display("[DDR_RD] t=%0t HELD: bucket DATA done but bucket_meta NOT yet (bm_rd_done=%b bm_rd_valid=%b) — waiting",
                                 $time, bm_rd_done, bm_rd_valid);
                    `endif
                    // Exit only when BOTH the bucket data burst AND the
                    // bucket_meta HBM read have completed. With bucket_meta in
                    // HBM the metadata read may finish after the data burst, so
                    // gating on axim_read_done alone would advance to S_SCAN
                    // with stale bkt_slot_list/fill -> wrong scan. (bm_rd_done
                    // may already be set this cycle, or bm_rd_valid arriving now.)
                    if (axim_read_done && (bm_rd_done || bm_rd_valid)) begin
                        `ifndef SYNTHESIS
                        $display("[DDR_RD] t=%0t -> S_SCAN: data+meta both done (bm via %s)",
                                 $time, bm_rd_valid ? "this-cycle" : "sticky");
                        `endif
                        state <= S_SCAN;
                    end
                end

                S_SCAN: begin
                    // Position coherence: a block's slot_list position IS the
                    // position of its ciphertext (scr_addr) and IVT (bucket,pos).
                    // Compaction now removes by clearing a slot in place (leaving
                    // a hole = slot field 0) and never moves other blocks, so a
                    // block is always found at the exact position its data lives.
                    // Scan ALL Z positions and skip empty/hole slots (field 0).
                    scan_hit = 0; scan_hit_pos = 0;
                    for (scan_i = 0; scan_i < Z; scan_i = scan_i + 1)
                        if (get_slot_at(bkt_slot_list, scan_i[POS_W-1:0]) != {SLOT_AW{1'b0}})
                            if (get_slot_at(bkt_slot_list, scan_i[POS_W-1:0]) == req_slot_addr)
                                begin scan_hit = 1; scan_hit_pos = scan_i[POS_W-1:0]; end
                    found_in_bucket <= scan_hit;
                    found_pos <= scan_hit_pos;
                    `ifndef SYNTHESIS
                    // [SCAN_OCC] full occupancy view: which positions hold which
                    // slots, and where (if anywhere) req matched. Lets us verify
                    // hole-skipping and that found_pos is the block's TRUE pos.
                    $display("[SCAN_OCC] t=%0t req=0x%08h bkt=%0d fill=%0d hit=%b hit_pos=%0d | p0=0x%08h p1=0x%08h p2=0x%08h p3=0x%08h p4=0x%08h p5=0x%08h p6=0x%08h p7=0x%08h",
                             $time, req_slot_addr, req_b, bkt_fill_count, scan_hit, scan_hit_pos,
                             get_slot_at(bkt_slot_list, 3'd0), get_slot_at(bkt_slot_list, 3'd1),
                             get_slot_at(bkt_slot_list, 3'd2), get_slot_at(bkt_slot_list, 3'd3),
                             get_slot_at(bkt_slot_list, 3'd4), get_slot_at(bkt_slot_list, 3'd5),
                             get_slot_at(bkt_slot_list, 3'd6), get_slot_at(bkt_slot_list, 3'd7));
                    `endif
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

                        if (ht_slot_found) begin
                            // Block is in the stash — ALWAYS use stash path.
                            // The stash has the latest data (from the most recent
                            // WRITE). If the block is also in the bucket (stale
                            // DDR init or prior eviction), remove it from the
                            // bucket slot list so the fill count stays correct.
                            if (found_in_bucket) begin
                                // Remove in place: clear this slot (hole). Do NOT
                                // swap the tail block down — moving its slot_list
                                // position without moving its ciphertext/IVT would
                                // break position coherence. fill is occupancy count.
                                bkt_slot_list <= set_slot_at(bkt_slot_list, found_pos,
                                    {SLOT_W{1'b0}});
                                bkt_fill_count <= bkt_fill_count - 1;
                                `ifndef SYNTHESIS
                                $display("[STASH_PRIO] t=%0t slot=0x%08h found in BOTH stash(idx=%0d) and bucket(pos=%0d) -> using stash, removing stale bucket entry",
                                         $time, req_slot_addr, ht_found_idx, found_pos);
                                $display("[COMPACT_HOLE] t=%0t site=STASH_PRIO slot=0x%08h cleared pos=%0d bkt=%0d fill:%0d->%0d",
                                         $time, req_slot_addr, found_pos, req_b, bkt_fill_count, bkt_fill_count-1);
                                `endif
                            end
                            state <= S_STASH_SEARCH;
                        end else if (found_in_bucket) begin
                            stash_alloc_idx <= st_ins_idx;
                            st_ins_slot <= req_slot_addr;
                            st_ins_tgt <= req_b_new;

                            if (same_bucket) begin
                                if (req_op == 0) begin
                                    ivt_rd_bucket <= req_b;
                                    ivt_rd_pos <= found_pos;
                                    ivt_rd_en <= 1;
                                    state <= S_EXT_IV_RD;
                                    `ifndef SYNTHESIS
                                    $display("[IVT_FSM] t=%0t issue RD (same_bucket) slot=0x%08h bkt=%0d pos=%0d -> S_EXT_IV_RD",
                                             $time, req_slot_addr, req_b, found_pos);
                                    `endif
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
                                    ivt_rd_bucket <= req_b;
                                    ivt_rd_pos <= found_pos;
                                    ivt_rd_en <= 1;
                                    state <= S_EXT_IV_RD;
                                    `ifndef SYNTHESIS
                                    $display("[IVT_FSM] t=%0t issue RD (diff_bucket) slot=0x%08h bkt=%0d pos=%0d -> S_EXT_IV_RD",
                                             $time, req_slot_addr, req_b, found_pos);
                                    `endif
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
                        $display("[IVT_FSM] t=%0t RD result valid -> consume IV/TAG", $time);
                        $display("[DEC_IV] t=%0t slot=0x%08h IV=0x%024h TAG=0x%032h found_pos=%0d",
                                 $time, req_slot_addr, ivt_rd_iv, ivt_rd_tag, found_pos);
                        // [COHERE_CHK] The read uses found_pos (from SCAN of the
                        // slot_list) to address BOTH the IVT (bucket,found_pos) and
                        // the ciphertext (scr_addr(found_pos)). Invariant: this is
                        // the SAME pos the block was stored at on eviction. We print
                        // slot/bkt/pos so a mismatch vs the eviction's EV_TAG pos for
                        // this slot is directly visible (grep slot across EV_TAG/COHERE_CHK).
                        $display("[COHERE_CHK] t=%0t READ slot=0x%08h bkt=%0d decrypt_pos=%0d (must match this slot's EV_TAG pos)",
                                 $time, req_slot_addr, req_b, found_pos);
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
                            `ifndef SYNTHESIS
                            if (aes_out_beat < 3 || gcm_data_out[31:0] == 32'habcd0000)
                                $display("[DECWR_SRC] t=%0t beat=%0d idx=%0d dec_pt[127:0]=0x%032h gcm_lo=0x%08h (staging to stash via decrypt)",
                                         $time, aes_out_beat, stash_alloc_idx,
                                         {gcm_data_out[63:0], aes_out_lo[63:0]}, gcm_data_out[31:0]);
                            `endif
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
                            // slot_r moved to HBM: write the per-entry slot addr
                            // for this stash entry (mirrors the HT insert below).
                            sr_wr_idx <= stash_alloc_idx;
                            sr_wr_slot_addr <= req_slot_addr;
                            sr_wr_en <= 1;
                            stash_bkt[stash_alloc_idx] <= req_b_new;
                            `ifndef SYNTHESIS
                            $display("[SLOTR_FSM] t=%0t INSERT(SB_RD path) idx=%0d slot=0x%08h (async HBM write)",
                                     $time, stash_alloc_idx, req_slot_addr);
                            `endif
                            ht_cmd_addr <= req_slot_addr;
                            ht_cmd_bkt <= req_b_new;
                            ht_cmd_idx <= stash_alloc_idx;
                            ht_cmd_slot_insert <= 1;
                            if (st_ins_full) err_stash_overflow <= 1;
                            pm_wr_addr <= req_slot_addr;
                            pm_wr_bucket <= req_b_new;
                            pm_wr_status <= `ST_IN_STASH;
                            pm_wr_en <= 1;
                            // Remove in place (hole); preserve position coherence.
                            bkt_slot_list <= set_slot_at(bkt_slot_list, found_pos,
                                {SLOT_W{1'b0}});
                            bkt_fill_count <= bkt_fill_count - 1;
                            `ifndef SYNTHESIS
                            $display("[COMPACT_HOLE] t=%0t site=DEC_WAIT slot=0x%08h cleared pos=%0d bkt=%0d fill:%0d->%0d",
                                     $time, req_slot_addr, found_pos, req_b, bkt_fill_count, bkt_fill_count-1);
                            `endif
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
                        ivt_wr_bucket <= req_b;
                        ivt_wr_pos <= found_pos;
                        ivt_wr_iv <= gcm_iv_out;
                        ivt_wr_tag <= gcm_tag_out;
                        ivt_wr_en <= 1;
                        state <= S_COMPACT;
                        `ifndef SYNTHESIS
                        $display("[IVT_FSM] t=%0t WR launch (SB_RD) slot=0x%08h bkt=%0d pos=%0d IV=0x%024h TAG=0x%032h -> S_COMPACT",
                                 $time, req_slot_addr, req_b, found_pos, gcm_iv_out, gcm_tag_out);
                        `endif
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
                        ivt_wr_bucket <= req_b;
                        ivt_wr_pos <= found_pos;
                        ivt_wr_iv <= gcm_iv_out;
                        ivt_wr_tag <= gcm_tag_out;
                        ivt_wr_en <= 1;
                        state <= S_COMPACT;
                        `ifndef SYNTHESIS
                        $display("[IVT_FSM] t=%0t WR launch (SB_WR) slot=0x%08h bkt=%0d pos=%0d IV=0x%024h TAG=0x%032h -> S_COMPACT",
                                 $time, req_slot_addr, req_b, found_pos, gcm_iv_out, gcm_tag_out);
                        `endif
                    end
                end

                // =============================================================
                // WRITE PATH: different bucket (unchanged)
                // =============================================================
                S_EXTRACT_WR: begin
                    if (client_wdata_valid) begin
                        `ifndef SYNTHESIS
                        if (beat_cnt == 0)
                            $display("[EXTWR_ENTRY] t=%0t slot=0x%08h idx=%0d found_in_stash=%b found_in_bucket=%b same_bkt? req_b=%0d req_b_new=%0d found_stash_idx=%0d st_ins_idx=%0d",
                                     $time, req_slot_addr, stash_alloc_idx,
                                     found_in_stash, found_in_bucket, req_b, req_b_new,
                                     found_stash_idx, st_ins_idx);
                        `endif
                        st_dwr_entry <= stash_alloc_idx;
                        st_dwr_beat <= beat_cnt;
                        st_dwr_data <= client_wdata;
                        st_dwr_en <= 1;
                        `ifndef SYNTHESIS
                        if (beat_cnt < 3 || client_wdata[31:0] == 32'habcd0000)
                            $display("[EXTWR_SRC] t=%0t beat=%0d idx=%0d client_wdata[127:0]=0x%032h valid=%b (staging to stash)",
                                     $time, beat_cnt, stash_alloc_idx,
                                     client_wdata[127:0], client_wdata_valid);
                        `endif
                        if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                            if (!found_in_stash) begin
                                st_ins_en <= 1;
                                // slot_r moved to HBM: write per-entry slot addr.
                                sr_wr_idx <= stash_alloc_idx;
                                sr_wr_slot_addr <= req_slot_addr;
                                sr_wr_en <= 1;
                                stash_bkt[stash_alloc_idx] <= req_b_new;
                                `ifndef SYNTHESIS
                                $display("[SLOTR_FSM] t=%0t INSERT(EXTRACT_WR path) idx=%0d slot=0x%08h (async HBM write)",
                                         $time, stash_alloc_idx, req_slot_addr);
                                `endif
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
                                    // Remove in place (hole); preserve coherence.
                                    bkt_slot_list <= set_slot_at(bkt_slot_list, found_pos,
                                        {SLOT_W{1'b0}});
                                    bkt_fill_count <= bkt_fill_count - 1;
                                    `ifndef SYNTHESIS
                                    $display("[COMPACT_HOLE] t=%0t site=EXTRACT_WR slot=0x%08h cleared pos=%0d bkt=%0d fill:%0d->%0d",
                                             $time, req_slot_addr, found_pos, req_b, bkt_fill_count, bkt_fill_count-1);
                                    `endif
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
                        if (!remap_phase) begin
                            // Phase 0: DELETE from the OLD bucket's chain first.
                            // Without this, the entry stays in the old chain AND
                            // gets inserted into the new chain → double membership
                            // → BKT_FIND on either chain emits it → double-free.
                            // Skip if old == new (same_bucket → no chain move).
                            if (ht_found_bkt != req_b_new) begin
                                ht_cmd_addr <= req_slot_addr;
                                ht_cmd_bkt <= ht_found_bkt;   // OLD bucket
                                ht_cmd_idx <= found_stash_idx;
                                ht_cmd_slot_delete <= 1;
                                `ifndef SYNTHESIS
                                $display("[REMAP_DEL] t=%0t slot=0x%08h idx=%0d old_bkt=%0d -> delete from old chain first",
                                         $time, req_slot_addr, found_stash_idx, ht_found_bkt);
                                `endif
                            end
                            remap_phase <= 1;
                            // Stay in S_STASH_SEARCH for one more cycle
                        end else begin
                            // Phase 1: INSERT into new chain + remap + transition
                            remap_phase <= 0;
                            st_remap_idx <= found_stash_idx;
                            st_remap_bkt <= req_b_new;
                            st_remap_en <= 1;
                            ht_cmd_addr <= req_slot_addr;
                            ht_cmd_bkt <= req_b_new;          // NEW bucket
                            ht_cmd_idx <= found_stash_idx;
                            ht_cmd_slot_insert <= 1;
                            stash_bkt[found_stash_idx] <= req_b_new;
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
                //
                // slot_r moved to HBM: instead of the old instant hierarchical
                // read u_stash.slot_r[idx], issue a slot_r_table HBM read and
                // wait for it in S_EVICT_SLOTR_WAIT before proceeding to encrypt.
                // =============================================================
                S_EVICT: begin
                    if (!ht_bkt_find_done) begin
                        // Waiting for BKT_FIND to complete
                    end else if (evict_list_idx < ht_evict_count &&
                                 evict_slots_remaining > 0) begin
                        // Skip stale chain entries: if the candidate's valid_bm
                        // bit is already 0, it was freed by a prior eviction but
                        // still has a stale link in the chain. Skip it.
                        if (!ev_valid_check_out) begin
                            `ifndef SYNTHESIS
                            $display("[EV_SKIP] t=%0t skipping freed stash_idx=%0d (valid_bm=0, stale chain entry)",
                                     $time, ht_evict_list[evict_list_idx]);
                            `endif
                            evict_list_idx <= evict_list_idx + 1;
                        end else if (!ev_bkt_match) begin
                            `ifndef SYNTHESIS
                            $display("[EV_SKIP_BKT] t=%0t skipping stash_idx=%0d: assigned bkt=%0d != evict bkt=%0d (stale chain link)",
                                     $time, ht_evict_list[evict_list_idx],
                                     stash_bkt[ht_evict_list[evict_list_idx]], req_b);
                            `endif
                            evict_list_idx <= evict_list_idx + 1;
                        end else begin
                        // Issue the slot_r read for this eviction candidate;
                        // capture the index, then wait for the HBM read result.
                        evict_stash_idx <= ht_evict_list[evict_list_idx];
                        ev_pos <= lowest_free_pos(bkt_slot_list);  // coherent placement
                        sr_rd_idx <= ht_evict_list[evict_list_idx];
                        sr_rd_en  <= 1;
                        `ifndef SYNTHESIS
                        $display("[EV_PLACE] t=%0t bkt=%0d chose ev_pos=%0d (lowest free hole) from p0=0x%08h p1=0x%08h p2=0x%08h p3=0x%08h p4=0x%08h p5=0x%08h p6=0x%08h p7=0x%08h fill=%0d",
                                 $time, req_b, lowest_free_pos(bkt_slot_list),
                                 get_slot_at(bkt_slot_list, 3'd0), get_slot_at(bkt_slot_list, 3'd1),
                                 get_slot_at(bkt_slot_list, 3'd2), get_slot_at(bkt_slot_list, 3'd3),
                                 get_slot_at(bkt_slot_list, 3'd4), get_slot_at(bkt_slot_list, 3'd5),
                                 get_slot_at(bkt_slot_list, 3'd6), get_slot_at(bkt_slot_list, 3'd7),
                                 bkt_fill_count);
                        $display("[EV_START] t=%0t evict stash_idx=%0d -> bucket=%0d pos=%0d list_idx=%0d (slot_r read issued)",
                                 $time, ht_evict_list[evict_list_idx], req_b,
                                 lowest_free_pos(bkt_slot_list), evict_list_idx);
                        `endif
                        state <= S_EVICT_SLOTR_WAIT;
                        end // close valid-check else
                    end else begin
                        // No more eviction candidates. Before entering S_DDR_WRITE
                        // (where ht_bus_blocked=1 and the HT stalls), we MUST wait
                        // for any pending HT DELETE to complete. Otherwise the SLOT
                        // entry isn't cleared and the next op finds a stale entry.
                        //
                        // RACE FIX: the guard must also block on the command PULSE
                        // (ht_cmd_slot_delete/insert), not just the latched bits.
                        // A DELETE issued at line 1658 is a one-cycle pulse that
                        // becomes ht_latch_slot_delete the NEXT cycle. If we only
                        // check the latch, on the issue cycle the latch is still 0
                        // and ht_state is still HT_IDLE, so the guard passes and we
                        // enter S_DDR_WRITE — which raises ht_bus_blocked and starves
                        // the just-latched DELETE forever (it can only dispatch when
                        // !ht_bus_blocked). Blocking on the pulse closes that window.
                        // We also require ht_state==HT_IDLE so a DELETE already in
                        // flight (ht_state!=IDLE) finishes before we block the bus.
                        if (ht_state == HT_IDLE &&
                            !ht_cmd_slot_delete && !ht_cmd_slot_insert &&
                            !ht_latch_slot_delete && !ht_latch_slot_insert) begin
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
                            `ifndef SYNTHESIS
                            $display("[DDR_WR] t=%0t LAUNCH: bucket=%0d data-write + bucket_meta-write(fill=%0d, async HBM) -> S_DDR_WRITE",
                                     $time, req_b, bkt_fill_count);
                            $display("[BM_WR] t=%0t writeback bkt=%0d fill=%0d slot_list=0x%030h (this is the metadata a future read of bkt=%0d will see)",
                                     $time, req_b, bkt_fill_count, bkt_slot_list, req_b);
                            `endif
                            state <= S_DDR_WRITE;
                        end
                        `ifndef SYNTHESIS
                        else begin
                            $display("[ORAM_FSM] t=%0t S_EVICT: waiting for HT DELETE to complete (ht_state=%0d latch_del=%b)",
                                     $time, ht_state, ht_latch_slot_delete);
                        end
                        `endif
                    end
                end


                // =============================================================
                // S_EVICT_SLOTR_WAIT: wait for the slot_r HBM read launched in
                // S_EVICT. When sr_rd_valid, capture ev_slot_addr and run the
                // original encrypt-launch sequence (formerly inline in S_EVICT,
                // when slot_r was an instant on-chip read).
                // =============================================================
                S_EVICT_SLOTR_WAIT: begin
                    if (sr_rd_valid) begin
                        ev_slot_addr <= sr_rd_slot_addr;
                        `ifndef SYNTHESIS
                        $display("[EV_SLOTR] t=%0t slot_r read done: slot=0x%08h stash_idx=%0d",
                                 $time, sr_rd_slot_addr, evict_stash_idx);
                        // Phantom-eviction detector: slot_r is zero-initialized,
                        // so a read of 0x0 for an entry we're about to evict means
                        // the HT chain handed us an entry that was never inserted
                        // (the old non-terminating-chain bug). After the unlink
                        // fix this should never fire.
                        if (sr_rd_slot_addr == {SLOT_AW{1'b0}})
                            $display("[EV_PHANTOM] WARN t=%0t evicting stash_idx=%0d with slot_r=0x0 -- entry never inserted? (bucket=%0d) chain may be corrupt",
                                     $time, evict_stash_idx, req_b);
                        `endif
                        st_ext_entry <= evict_stash_idx;
                        st_return_state <= S_EV_ENC_START;
                        gcm_encrypt <= 1;
                        gcm_iv_reg <= aes_iv_seed;
                        gcm_start <= 1;
                        aes_half <= 0; aes_blk_cnt <= 0;
                        aes_out_beat <= 0; aes_out_half <= 0;
                        beat_cnt <= 0;
                        evict_list_idx <= evict_list_idx + 1;
                        state <= S_ST_LOAD;
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
                                         scr_addr(ev_pos, aes_out_beat));
                            `endif
                            scr_fsm_wr_addr <= scr_addr(ev_pos, aes_out_beat);
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
                            scr_fsm_wr_addr <= scr_addr(ev_pos, aes_out_beat);
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
                                 ev_pos);
                        `endif
                        ivt_wr_bucket <= req_b;
                        ivt_wr_pos <= ev_pos;
                        ivt_wr_iv <= gcm_iv_out;
                        ivt_wr_tag <= gcm_tag_out;
                        ivt_wr_en <= 1;
                        `ifndef SYNTHESIS
                        $display("[IVT_FSM] t=%0t WR launch (EVICT) slot=0x%08h bkt=%0d pos=%0d IV=0x%024h TAG=0x%032h evict_idx=%0d",
                                 $time, ev_slot_addr, req_b, ev_pos,
                                 gcm_iv_out, gcm_tag_out, evict_stash_idx);
                        `endif
                        bkt_slot_list <= set_slot_at(bkt_slot_list,
                            ev_pos,
                            ev_slot_addr[SLOT_W+11:12]);
                        bkt_fill_count <= bkt_fill_count + 1;
                        st_evict_idx <= evict_stash_idx;
                        st_evict_en <= 1;
                        // NOTE: eviction keeps the original lazy SLOT delete. The
                        // read path locates bucket-resident blocks via pos_map +
                        // bucket SCAN (found_in_bucket), NOT via HT_LOOKUP, so the
                        // HT entry is not required for correct reads. (The HT MISS
                        // on an evicted block is benign for the read path.)
                        ht_cmd_addr <= ev_slot_addr;
                        ht_cmd_idx <= evict_stash_idx;
                        ht_cmd_bkt <= req_b;
                        ht_cmd_slot_delete <= 1;
                        `ifndef SYNTHESIS
                        $display("[EVICT_DEL_ISSUE] t=%0t slot=0x%08h idx=%0d bkt=%0d (ht_cmd_slot_delete<=1, state=%0d ht_state=%0d ht_bus_blocked=%b)",
                                 $time, ev_slot_addr, evict_stash_idx, req_b, state, ht_state, ht_bus_blocked);
                        `endif
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
                        $display("[DDR_WR] t=%0t bucket data write done (ivt_busy=%b slotr_busy=%b bmeta_busy=%b)",
                                 $time, ivt_busy, slotr_busy, bmeta_busy);
                    // Per-master HELD diagnostics: show exactly what is blocking
                    // op completion, so a stuck metadata write is obvious.
                    if (axim_write_done && (ivt_busy || slotr_busy || bmeta_busy))
                        $display("[DDR_WR] t=%0t op completion HELD: waiting on%s%s%s",
                                 $time,
                                 ivt_busy   ? " IVT"     : "",
                                 slotr_busy ? " slot_r"  : "",
                                 bmeta_busy ? " bucket_meta" : "");
                    `endif
                    // Safety interlock: do not complete the op until every
                    // queued/in-flight metadata write has returned its B-response.
                    // This covers IVT (iv/tag), slot_r (per-entry slot addr), and
                    // bucket_meta (per-bucket directory) — all async writes that
                    // launched at/around S_EVICT->S_DDR_WRITE. No subsequent op
                    // may begin and read metadata until this op's metadata is
                    // durable in HBM.
                    if (axim_write_done && !ivt_busy && !slotr_busy && !bmeta_busy) begin
                        client_done <= 1;
                        client_stall <= 0;
                        state <= S_IDLE;
                        `ifndef SYNTHESIS
                        $display("[DDR_WR] t=%0t op COMPLETE: bucket + IVT + slot_r + bucket_meta all durable -> S_IDLE",
                                 $time);
                        `endif
                    end
                end

                default: state <= S_IDLE;

                S_ST_LOAD: begin
                    if (st_ext_rd_done) begin
                        $display("[ORAM_FSM] t=%0t S_ST_LOAD: ext_rd_done -> return state", $time);
                        state <= st_return_state;
                    end else if (!st_ext_busy && !st_ext_rd_req && metadata_idle) begin
                        st_ext_rd_req <= 1'b1;
                        `ifndef SYNTHESIS
                        $display("[ORAM_FSM] t=%0t S_ST_LOAD: issuing ext_rd_req entry=%0d (metadata clear)", $time, st_ext_entry);
                        `endif
                    end else if (st_ext_rd_req && st_ext_busy) begin
                        st_ext_rd_req <= 1'b0;
                    end
                end

                S_ST_FLUSH: begin
                    if (st_ext_wr_done) begin
                        $display("[ORAM_FSM] t=%0t S_ST_FLUSH: ext_wr_done -> return state", $time);
                        state <= st_return_state;
                    end else if (!st_ext_busy && !st_ext_wr_req && metadata_idle) begin
                        st_ext_wr_req <= 1'b1;
                        `ifndef SYNTHESIS
                        $display("[ORAM_FSM] t=%0t S_ST_FLUSH: issuing ext_wr_req entry=%0d (metadata clear)", $time, st_ext_entry);
                        `endif
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
                S_EVICT, S_EVICT_SLOTR_WAIT: begin
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
    reg              ht_rd_acc;     // set when THIS FSM's single read was accepted
    reg [14:0] ht_hash;              // hash value (15-bit for 32768-slot coverage)
    reg [3:0]  ht_sub_op;            // sub-step within an operation (widened to 4-bit for unlink steps 8,9)

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

    // HT SLOT entry layout (64-bit, 4 entries per 256-bit beat):
    //   [3:0]                            = status (4 bits, fixed)
    //   [HT_BKT_OFS-1 : 4]              = slot_addr (SLOT_ADDR_W bits)
    //   [HT_IDX_OFS-1 : HT_BKT_OFS]     = bucket_id (BUCKET_ID_W bits)
    //   [HT_VAL_BIT-1 : HT_IDX_OFS]      = stash_idx (STASH_PTR_W bits)
    //   [HT_VAL_BIT]                      = valid (1 bit)
    localparam HT_STAT_OFS = 0;
    localparam HT_ADDR_OFS = 4;
    localparam HT_BKT_OFS  = HT_ADDR_OFS + SLOT_AW;    // 4+32 = 36
    localparam HT_IDX_OFS  = HT_BKT_OFS + BUCKET_W;    // 36+13 = 49
    localparam HT_VAL_BIT  = HT_IDX_OFS + PTR_W;        // 49+14 = 63

    // Helper: compute SLOT hash table HBM address from slot_addr
    // 8192 beats × 4 entries/beat = 32768 capacity (2× STASH_DEPTH).
    // Hash uses addr[26:12] (15 bits, covers 32768 slots at 0x1000 spacing),
    // truncated to 13-bit beat index. XOR with addr[11:10] spreads sub-page
    // addresses (0x400-spaced). Collisions from the truncation are handled
    // by the 4-way associative scan (checks full 32-bit slot_addr).
    function [AXI_AW-1:0] ht_slot_hbm_addr;
        input [SLOT_AW-1:0] addr;
        reg [14:0] h;
        reg [12:0] beat;
        begin
            h = addr[26:12];
            beat = h[12:0] ^ {11'b0, addr[11:10]};
            ht_slot_hbm_addr = `HT_SLOT_BASE + ({19'b0, beat} << 5);
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
    reg [BUCKET_W-1:0] ht_latch_del_bkt;     // bucket to unlink the entry from
    // --- Per-op HT operation tracking ---
    reg        ht_ins_overwritten_r, ht_del_overwritten_r;
    reg [7:0]  ht_ins_issued_r, ht_ins_completed_r;
    reg [7:0]  ht_del_issued_r, ht_del_completed_r;
    reg        ht_wb_hit_latched;  // capture wb_hit at read-done time
    // --- bucket-chain unlink (singly-linked list removal) working state ---
    reg [BUCKET_W-1:0] ht_del_bkt;           // bucket being unlinked from
    reg [PTR_W-1:0]    ht_del_target;        // stash idx to remove from chain
    reg [PTR_W-1:0]    ht_del_cur;           // current node in the walk
    reg [PTR_W-1:0]    ht_del_next;          // next[cur], decoded from beat
    reg [15:0]         ht_del_save_next;     // next[target] (raw 16b), to splice in
    reg [3:0]          ht_del_field;         // which 16b field within the beat
    reg [14:0]         ht_del_hops;          // walk hop counter (cycle guard)
    reg [BUCKET_W-1:0] ht_latch_find_bkt;

    // Single-entry HEAD beat cache: prevents stale-zero propagation during
    // RMW. After every HEAD write, the beat is cached. On the next HEAD read
    // (same address), the cache is used instead of potentially-stale st_sng_rdata.
    // This breaks the cycle where a stale zero in one field gets written back
    // by an RMW targeting a different field, erasing a valid chain head.

    // =========================================================================
    // FREE-RUNNING per-cycle monitor of the single-beat HT port. Unlike the
    // sub_op-gated traces, this fires EVERY clock the port is active, so it
    // captures the exact edge st_sng_rdata changes and what it changes to. The
    // goal: catch the cycle where a HEAD/NEXT read (addr 0x142/0x143) returns a
    // value that looks like a slot address (0xN000, i.e. low 12 bits zero and
    // a small nonzero high nibble) — the poison.
    // =========================================================================
    `ifndef SYNTHESIS
    reg [AXI_DW-1:0] mon_prev_rdata;
    reg              mon_prev_done;
    always @(posedge clk) begin
        if (rst_n) begin
            // Log whenever the port has a request, is done, or rdata changed.
            if (st_sng_rd_req || st_sng_wr_req || st_sng_done ||
                (st_sng_rdata[255:0] !== mon_prev_rdata)) begin
                $display("[SNGMON] t=%0t ht_st=%0d ht_op=%0d sub=%0d | rd_req=%b wr_req=%b done=%b | addr=0x%010h | rdata[15:0]=0x%04h rdata[47:0]=0x%012h wdata[15:0]=0x%04h",
                         $time, ht_state, ht_op, ht_sub_op,
                         st_sng_rd_req, st_sng_wr_req, st_sng_done,
                         st_sng_addr,
                         st_sng_rdata[15:0], st_sng_rdata[47:0], st_sng_wdata[15:0]);
                // Log EVERY completed read on the HEAD(0x42)/NEXT(0x43) region
                // with the full low-64 bits, so we see exactly what each chain
                // read returns — no guessing about what "poison" looks like.
                if (st_sng_done &&
                    ((st_sng_addr[27:20] == 8'h42) || (st_sng_addr[27:20] == 8'h43))) begin
                    $display("[SNGMON-CHAINRD] t=%0t addr=0x%010h rdata[63:0]=0x%016h (field0=0x%04h f1=0x%04h f2=0x%04h f3=0x%04h) ht_op=%0d sub=%0d",
                             $time, st_sng_addr, st_sng_rdata[63:0],
                             st_sng_rdata[15:0], st_sng_rdata[31:16],
                             st_sng_rdata[47:32], st_sng_rdata[63:48], ht_op, ht_sub_op);
                    // Flag if any field looks like a slot address (0xN000).
                    if ((st_sng_rdata[11:0]==12'h0 && st_sng_rdata[15:12]!=4'h0 && st_sng_rdata[15:0]!=16'hFFFF) ||
                        (st_sng_rdata[27:16]==12'h0 && st_sng_rdata[31:28]!=4'h0 && st_sng_rdata[31:16]!=16'hFFFF))
                        $display("[SNGMON] *** POISON t=%0t addr=0x%010h rdata[31:0]=0x%08h looks like slot-addr in chain read",
                                 $time, st_sng_addr, st_sng_rdata[31:0]);
                end
                // Also log every completed read on the SLOT region (0x41), so we
                // can see if a SLOT beat's data later appears in a HEAD/NEXT read.
                if (st_sng_done && (st_sng_addr[27:20] == 8'h41)) begin
                    $display("[SNGMON-SLOTRD] t=%0t addr=0x%010h rdata[63:0]=0x%016h ht_op=%0d sub=%0d",
                             $time, st_sng_addr, st_sng_rdata[63:0], ht_op, ht_sub_op);
                end
                // Log every WRITE to HEAD/NEXT/SLOT regions: what beat we're
                // writing so we can compare against subsequent reads.
                if (st_sng_wr_req &&
                    ((st_sng_addr[27:20] == 8'h42) || (st_sng_addr[27:20] == 8'h43))) begin
                    $display("[SNGMON-CHAINWR] t=%0t addr=0x%010h wdata[63:0]=0x%016h (f0=0x%04h f1=0x%04h f2=0x%04h f3=0x%04h) ht_op=%0d sub=%0d",
                             $time, st_sng_addr, st_sng_wdata[63:0],
                             st_sng_wdata[15:0], st_sng_wdata[31:16],
                             st_sng_wdata[47:32], st_sng_wdata[63:48], ht_op, ht_sub_op);
                end
                if (st_sng_wr_req && (st_sng_addr[27:20] == 8'h41)) begin
                    $display("[SNGMON-SLOTWR] t=%0t addr=0x%010h wdata[63:0]=0x%016h ht_op=%0d sub=%0d",
                             $time, st_sng_addr, st_sng_wdata[63:0], ht_op, ht_sub_op);
                end
            end
            mon_prev_rdata <= st_sng_rdata;
            mon_prev_done  <= st_sng_done;
        end
    end
    `endif

    // ht_bus_blocked: mirrors the ht_safe blacklist in secure_oram_top.
    // When the main FSM is in one of these states, the AXI mux routes AWAY
    // from stash (sel_stash=0), so st_sng requests can never complete.
    // The HT FSM must stall in these states to avoid deadlock.
    wire ht_bus_blocked = st_burst_busy ||
        (state == S_POS_LOOKUP)   ||  // 1
        (state == S_DDR_READ)     ||  // 2
        (state == S_STASH_SEARCH) ||  // 11
        (state == S_STASH_READ)   ||  // 12
        (state == S_DDR_WRITE)    ||  // 19
        (state == S_ST_LOAD)      ||  // 28
        (state == S_ST_FLUSH);        // 29

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
            ht_del_bkt <= 0; ht_del_target <= 0; ht_del_cur <= 0;
            ht_del_save_next <= 0; ht_del_hops <= 0;
            st_sng_rd_req <= 0;
            st_sng_wr_req <= 0;
            ht_rd_acc <= 0;
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

            `ifndef SYNTHESIS
            // Raw arbiter-side view of the delete command pulse — proves whether
            // the FSM-block pulse is visible to this (separate) arbiter block.
            if (ht_cmd_slot_delete || ht_latch_slot_delete)
                $display("[ARB_DEL_VIEW] t=%0t ht_cmd_slot_delete=%b ht_latch_slot_delete=%b ht_state=%0d ht_bus_blocked=%b state=%0d",
                         $time, ht_cmd_slot_delete, ht_latch_slot_delete, ht_state, ht_bus_blocked, state);
            `endif

            // Latch incoming commands (with overwrite detection)
            if (ht_cmd_slot_lookup) begin
                ht_latch_slot_lookup <= 1;
                ht_latch_lu_addr <= ht_cmd_addr;
            end
            if (ht_cmd_slot_insert) begin
                `ifndef SYNTHESIS
                if (ht_latch_slot_insert)
                    $display("[HT_OVERWRITE] t=%0t INSERT overwritten! old=0x%08h new=0x%08h state=%0d ht_state=%0d",
                             $time, ht_latch_ins_addr, ht_cmd_addr, state, ht_state);
                $display("[HT_LATCH] t=%0t INSERT slot=0x%08h bkt=%0d idx=%0d",
                         $time, ht_cmd_addr, ht_cmd_bkt, ht_cmd_idx);
                `endif
                ht_latch_slot_insert <= 1;
                ht_latch_ins_addr <= ht_cmd_addr;
                ht_latch_ins_bkt <= ht_cmd_bkt;
                ht_latch_ins_idx <= ht_cmd_idx;
            end
            if (ht_cmd_slot_delete) begin
                `ifndef SYNTHESIS
                if (ht_latch_slot_delete)
                    $display("[HT_OVERWRITE] t=%0t DELETE overwritten! old=0x%08h new=0x%08h state=%0d ht_state=%0d",
                             $time, ht_latch_del_addr, ht_cmd_addr, state, ht_state);
                $display("[HT_LATCH_DEL] t=%0t arbiter latched DELETE slot=0x%08h idx=%0d bkt=%0d",
                         $time, ht_cmd_addr, ht_cmd_idx, ht_cmd_bkt);
                `endif
                ht_latch_slot_delete <= 1;
                ht_latch_del_addr <= ht_cmd_addr;
                ht_latch_del_idx <= ht_cmd_idx;
                ht_latch_del_bkt <= ht_cmd_bkt;
            end
            if (ht_cmd_bkt_find) begin
                ht_latch_bkt_find <= 1;
                ht_latch_find_bkt <= ht_cmd_bkt;
            end

            case (ht_state)

            HT_IDLE: begin
                ht_done <= 0;
                ht_rd_acc <= 0;  // fresh op: no read accepted yet
                `ifndef SYNTHESIS
                if ((ht_latch_slot_insert || ht_latch_slot_lookup || ht_latch_slot_delete)
                    && (st_burst_busy ||
                    state == S_ST_LOAD || state == S_ST_FLUSH ||
                    state == S_DDR_READ || state == S_DDR_WRITE))
                    $display("[HT_GUARD] t=%0t blocked: burst_busy=%0d state=%0d ins=%0d lu=%0d del=%0d",
                             $time, st_burst_busy, state,
                             ht_latch_slot_insert, ht_latch_slot_lookup, ht_latch_slot_delete);
                `endif
                if (!ht_bus_blocked) begin
                    // Priority: DELETE > INSERT > LOOKUP > BKT_FIND
                    // DELETE first so remap (DELETE old chain + INSERT new chain)
                    // clears the old SLOT entry before INSERT creates the new one.
                    if (ht_latch_slot_delete) begin
                        ht_latch_slot_delete <= 0;
                        ht_slot_addr <= ht_latch_del_addr;
                        ht_stash_idx <= ht_latch_del_idx;
                        ht_del_bkt    <= ht_latch_del_bkt;    // unlink bucket
                        ht_del_target <= ht_latch_del_idx;    // idx to remove
                        ht_hash <= ht_latch_del_addr[26:12];
                        ht_addr <= ht_slot_hbm_addr(ht_latch_del_addr);
                        st_sng_addr <= ht_slot_hbm_addr(ht_latch_del_addr);
                        st_sng_rd_req <= 1;
                        ht_op <= HT_OP_SLOT_DELETE;
                        ht_sub_op <= 0;
                        ht_state <= HT_WAIT_RD;
                        `ifndef SYNTHESIS
                        $display("[HT_DISPATCH] t=%0t DELETE slot=0x%08h idx=%0d unlink_bkt=%0d",
                                 $time, ht_latch_del_addr, ht_latch_del_idx, ht_latch_del_bkt);
                        `endif
                    end else if (ht_latch_slot_insert) begin
                        ht_latch_slot_insert <= 0;
                        ht_slot_addr <= ht_latch_ins_addr;
                        ht_target_bkt <= ht_latch_ins_bkt;
                        ht_stash_idx <= ht_latch_ins_idx;
                        ht_hash <= ht_latch_ins_addr[26:12];
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
                    end else if (ht_latch_slot_lookup) begin
                        ht_latch_slot_lookup <= 0;
                        ht_slot_addr <= ht_latch_lu_addr;
                        ht_hash <= ht_latch_lu_addr[26:12];
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
                // Bus-blocked hold: the HT single-beat port shares the AXI bus
                // via stash_axi_master. When the main FSM is in a state where
                // sel_stash=0 (S_DDR_READ, S_DDR_WRITE, etc.) or a stash burst
                // is active, st_sng requests are gated/blocked. Stall here
                // until the bus is available; the request re-issues automatically.
                if (ht_bus_blocked) begin
                    st_sng_rd_req <= 0;
                end else begin
                    if (!st_sng_done)
                        st_sng_rd_req <= 1;
                    else
                        st_sng_rd_req <= 0;
                // Track acceptance of THIS read. cmd_single_accepted pulses when
                // stash_axi_master takes our request (ST_IDLE->ST_SNG_READ).
                if (st_sng_accepted)
                    ht_rd_acc <= 1;
                // Only consume done once OUR read was accepted. A done pulsing
                // before acceptance belongs to a PRIOR single-read still draining
                // on the shared port; consuming it would decode stale rdata.
                if (st_sng_done && ht_rd_acc) begin
                    ht_beat <= ht_slot_rdata;
                    ht_wb_hit_latched <= ht_wb_hit;
                    st_sng_rd_req <= 0;
                    ht_rd_acc <= 0;
                    ht_state <= HT_WAIT_RD_SETTLE;
                end
                end
            end

            // SETTLE: decode the read data latched at the done cycle (ht_beat).
            // We do NOT read st_sng_rdata/ht_slot_rdata live here — by this cycle
            // the shared single-beat port may have re-accepted another read and
            // overwritten it. All per-op read-result handling decodes ht_beat.
            HT_WAIT_RD_SETTLE: begin
                begin
                    case (ht_op)

                    HT_OP_SLOT_LOOKUP: begin
                        ht_slot_found <= 0;
                        ht_found = 0;
                        // Latch LOOKUP diagnostic
                        dbg_ht_lu_hbm_addr <= ht_addr;
                        dbg_ht_lu_valid_bits <= {ht_beat[3*64+HT_VAL_BIT],
                                                  ht_beat[2*64+HT_VAL_BIT],
                                                  ht_beat[1*64+HT_VAL_BIT],
                                                  ht_beat[0*64+HT_VAL_BIT]};
                        dbg_ht_lu_wb_hit <= ht_wb_hit_latched;
                        dbg_ht_lu_slot_looked_up <= ht_slot_addr;
                        `ifndef SYNTHESIS
                        $display("[HT_LOOKUP_RD] t=%0t slot=0x%08h addr=0x%0h hash=%0d rdata[63:0]=0x%016h entry[0..3]valid=%b%b%b%b wb_hit=%b",
                                 $time, ht_slot_addr, ht_addr, ht_hash,
                                 ht_beat[63:0],
                                 ht_beat[63], ht_beat[127], ht_beat[191], ht_beat[255],
                                 ht_wb_hit);
                        `endif
                        for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                            if (ht_beat[ht_j*64 + HT_VAL_BIT] &&
                                ht_beat[ht_j*64+HT_ADDR_OFS +: SLOT_AW] == ht_slot_addr) begin
                                ht_slot_found <= 1;
                                ht_found = 1;
                                ht_found_idx <= ht_beat[ht_j*64+HT_IDX_OFS +: PTR_W];
                                ht_found_bkt <= ht_beat[ht_j*64+HT_BKT_OFS +: BUCKET_W];
                            end
                        end
                        `ifndef SYNTHESIS
                        // Per-entry decode of the home beat: valid|slot|bkt|idx for
                        // each of the 4 entries. Audits HT SLOT-table contents.
                        $display("[HT_LU_ENTRIES] t=%0t lu_slot=0x%08h addr=0x%0h | e0:v%b s=0x%08h b=%0d i=%0d | e1:v%b s=0x%08h b=%0d i=%0d | e2:v%b s=0x%08h b=%0d i=%0d | e3:v%b s=0x%08h b=%0d i=%0d",
                                 $time, ht_slot_addr, ht_addr,
                                 ht_beat[0*64+HT_VAL_BIT], ht_beat[0*64+HT_ADDR_OFS +:SLOT_AW], ht_beat[0*64+HT_BKT_OFS +:BUCKET_W], ht_beat[0*64+HT_IDX_OFS +:PTR_W],
                                 ht_beat[1*64+HT_VAL_BIT], ht_beat[1*64+HT_ADDR_OFS +:SLOT_AW], ht_beat[1*64+HT_BKT_OFS +:BUCKET_W], ht_beat[1*64+HT_IDX_OFS +:PTR_W],
                                 ht_beat[2*64+HT_VAL_BIT], ht_beat[2*64+HT_ADDR_OFS +:SLOT_AW], ht_beat[2*64+HT_BKT_OFS +:BUCKET_W], ht_beat[2*64+HT_IDX_OFS +:PTR_W],
                                 ht_beat[3*64+HT_VAL_BIT], ht_beat[3*64+HT_ADDR_OFS +:SLOT_AW], ht_beat[3*64+HT_BKT_OFS +:BUCKET_W], ht_beat[3*64+HT_IDX_OFS +:PTR_W]);
                        `endif
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
                            ht_new_beat = ht_beat;
                            // First: check if slot_addr already exists (update in place)
                            for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                                if (!ht_inserted &&
                                    ht_beat[ht_j*64+HT_ADDR_OFS +: SLOT_AW] == ht_slot_addr) begin
                                    ht_new_beat[ht_j*64 + HT_VAL_BIT] = 1'b1;
                                    ht_new_beat[ht_j*64+HT_IDX_OFS +: PTR_W] = ht_stash_idx;
                                    ht_new_beat[ht_j*64+HT_BKT_OFS +: BUCKET_W] = ht_target_bkt;
                                    ht_new_beat[ht_j*64+HT_ADDR_OFS +: SLOT_AW] = ht_slot_addr;
                                    ht_new_beat[ht_j*64 +: 4] = 4'b0;
                                    ht_inserted = 1;
                                end
                            end
                            // Second: find empty slot if no existing entry
                            if (!ht_inserted) begin
                                for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                                    if (!ht_inserted && !ht_beat[ht_j*64 + HT_VAL_BIT]) begin
                                        ht_new_beat[ht_j*64 + HT_VAL_BIT] = 1'b1;
                                        ht_new_beat[ht_j*64+HT_IDX_OFS +: PTR_W] = ht_stash_idx;
                                        ht_new_beat[ht_j*64+HT_BKT_OFS +: BUCKET_W] = ht_target_bkt;
                                        ht_new_beat[ht_j*64+HT_ADDR_OFS +: SLOT_AW] = ht_slot_addr;
                                        ht_new_beat[ht_j*64 +: 4] = 4'b0;
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
                            // Read BKT head result. Head-insert: new entry becomes
                            // head, its next = old_head.
                            ht_slot_in_beat = ht_target_bkt[3:0];
                            ht_old_head = st_sng_rdata[ht_slot_in_beat*16 +: 16];
                            `ifndef SYNTHESIS
                            $display("[PIN5-RD2] t=%0t HEAD read addr=0x%010h field=%0d old_head=0x%04h",
                                     $time, st_sng_addr, ht_slot_in_beat, ht_old_head);
                            `endif
                            ht_new_beat = st_sng_rdata;
                            ht_new_beat[ht_slot_in_beat*16 +: 16] = {2'b01, ht_stash_idx};
                            `ifndef SYNTHESIS
                            $display("[PIN5-WR2] t=%0t HEAD write addr=0x%010h field=%0d head[bkt=%0d]<=0x%04h (idx %0d)",
                                     $time, ht_bkt_head_addr(ht_target_bkt), ht_slot_in_beat,
                                     ht_target_bkt, {2'b01, ht_stash_idx}, ht_stash_idx);
                            `endif
                            st_sng_addr <= ht_bkt_head_addr(ht_target_bkt);
                            st_sng_wdata <= ht_new_beat;
                            st_sng_wr_req <= 1;
                            // SELF-LOOP GUARD: if the index being inserted is ALREADY
                            // the head of this bucket's chain (and the head is a real
                            // pointer, not a terminator), linking next=old_head would
                            // create next[idx]=idx (a 1-node cycle). Terminate instead.
                            // Defends against double-insert of a reused idx before its
                            // prior chain membership was unlinked.
                            if (ht_old_head[15:14] == 2'b01 &&
                                ht_old_head[PTR_W-1:0] == ht_stash_idx) begin
                                `ifndef SYNTHESIS
                                $display("[HT_INSERT] WARN t=%0t self-link avoided: idx=%0d already head of bkt=%0d, terminating next",
                                         $time, ht_stash_idx, ht_target_bkt);
                                `endif
                                ht_beat <= {{(AXI_DW-16){1'b0}}, 16'hFFFF};  // terminator
                            end else begin
                                ht_beat <= {{(AXI_DW-16){1'b0}}, ht_old_head};
                            end
                            ht_sub_op <= 3;
                            ht_state <= HT_WAIT_WR;
                        end else if (ht_sub_op == 4) begin
                            // Read next array result
                            ht_slot_in_beat = ht_stash_idx[3:0];
                            ht_new_beat = st_sng_rdata;
                            ht_new_beat[ht_slot_in_beat*16 +: 16] = ht_beat[15:0];
                            `ifndef SYNTHESIS
                            // PINPOINT #5: what value goes into next[stash_idx]?
                            // ht_beat[15:0] should be old_head (a small idx or 0xFFFF).
                            // If it's 0xN000 (a slot address), the bug is that ht_beat
                            // captured a slot addr at sub_op 2 (i.e. the head read was
                            // already wrong). addr is where next[idx] is written.
                            $display("[PIN5-WR4] t=%0t NEXT write addr=0x%010h field=%0d next[idx=%0d]<=0x%04h (ht_beat[15:0], should be old_head idx/term)",
                                     $time, ht_bkt_next_addr(ht_stash_idx), ht_slot_in_beat,
                                     ht_stash_idx, ht_beat[15:0]);
                            `endif
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
                            ht_new_beat = ht_beat;
                            for (ht_j = 0; ht_j < 4; ht_j = ht_j + 1) begin
                                if (!ht_found && ht_beat[ht_j*64 + HT_VAL_BIT] &&
                                    ht_beat[ht_j*64+HT_ADDR_OFS +: SLOT_AW] == ht_slot_addr) begin
                                    ht_new_beat[ht_j*64 +: 64] = 64'b0;
                                    ht_found = 1;
                                    `ifndef SYNTHESIS
                                    $display("[HT_DEL_SLOT] t=%0t slot=0x%08h addr=0x%0h MATCH entry%0d (was valid=1 bkt=%0d idx=%0d) -> cleared",
                                             $time, ht_slot_addr, ht_addr, ht_j,
                                             ht_beat[ht_j*64+HT_BKT_OFS +: BUCKET_W],
                                             ht_beat[ht_j*64+HT_IDX_OFS +: PTR_W]);
                                    `endif
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
                        end else if (ht_sub_op == 2) begin
                            // Head beat arrived. Is the target the head of the chain?
                            // Use HEAD cache to avoid stale-zero propagation.
                            ht_del_field = ht_del_bkt[3:0];
                            ht_del_next  = st_sng_rdata[ht_del_field*16 +: PTR_W];
                            if (st_sng_rdata[ht_del_field*16 + 14 +: 2] != 2'b01) begin
                                // Empty chain — nothing to unlink (shouldn't happen).
                                `ifndef SYNTHESIS
                                $display("[HT_DEL] t=%0t WARN unlink: bkt=%0d chain empty, target=%0d not present",
                                         $time, ht_del_bkt, ht_del_target);
                                `endif
                                ht_done <= 1;
                                ht_state <= HT_IDLE;
                            end else if (ht_del_next == ht_del_target) begin
                                // Target IS the head: need next[target] to splice.
                                `ifndef SYNTHESIS
                                $display("[HT_DEL] t=%0t target=%0d is chain head, read its next",
                                         $time, ht_del_target);
                                `endif
                                st_sng_addr  <= ht_bkt_next_addr(ht_del_target);
                                st_sng_rd_req <= 1;
                                ht_sub_op <= 3;
                                ht_state  <= HT_WAIT_RD;
                            end else begin
                                // Walk: cur = head; read next[cur].
                                ht_del_cur <= ht_del_next;
                                ht_del_hops <= 0;          // start hop counter
                                st_sng_addr  <= ht_bkt_next_addr(ht_del_next);
                                st_sng_rd_req <= 1;
                                ht_sub_op <= 5;
                                ht_state  <= HT_WAIT_RD;
                            end
                        end else if (ht_sub_op == 3) begin
                            // next[target] arrived: capture it, then re-read the
                            // head beat for a clean read-modify-write (the bus has
                            // moved on, so we cannot reuse the earlier head beat).
                            ht_del_field = ht_del_target[3:0];
                            ht_del_save_next <= st_sng_rdata[ht_del_field*16 +: 16];
                            st_sng_addr  <= ht_bkt_head_addr(ht_del_bkt);
                            st_sng_rd_req <= 1;
                            ht_sub_op <= 8;   // read head beat for RMW
                            ht_state  <= HT_WAIT_RD;
                        end else if (ht_sub_op == 8) begin
                            // Head beat re-read for RMW: set head field = save_next.
                            ht_del_field = ht_del_bkt[3:0];
                            ht_new_beat = st_sng_rdata;
                            ht_new_beat[ht_del_field*16 +: 16] = ht_del_save_next;
                            st_sng_addr  <= ht_bkt_head_addr(ht_del_bkt);
                            st_sng_wdata <= ht_new_beat;
                            st_sng_wr_req <= 1;
                            ht_sub_op <= 4;
                            ht_state  <= HT_WAIT_WR;
                        end else if (ht_sub_op == 5) begin
                            // next[cur] arrived. Is next[cur] == target?
                            ht_del_field = ht_del_cur[3:0];
                            ht_del_next  = st_sng_rdata[ht_del_field*16 +: PTR_W];
                            if (ht_del_next == ht_del_target) begin
                                // Found predecessor. Read next[target] to splice.
                                `ifndef SYNTHESIS
                                $display("[HT_DEL] t=%0t pred=%0d -> target=%0d, read next[target]",
                                         $time, ht_del_cur, ht_del_target);
                                `endif
                                st_sng_addr  <= ht_bkt_next_addr(ht_del_target);
                                st_sng_rd_req <= 1;
                                ht_sub_op <= 6;
                                ht_state  <= HT_WAIT_RD;
                            end else if (st_sng_rdata[ht_del_field*16 + 14 +: 2] != 2'b01) begin
                                // Reached end without finding target — chain corrupt
                                // or target already gone. Stop (don't loop forever).
                                `ifndef SYNTHESIS
                                $display("[HT_DEL] t=%0t WARN unlink: target=%0d not found in bkt=%0d chain (end reached)",
                                         $time, ht_del_target, ht_del_bkt);
                                `endif
                                ht_done <= 1;
                                ht_state <= HT_IDLE;
                            end else if (ht_del_hops >= STASH_DEPTH) begin
                                // Cycle guard: a healthy chain reaches target or a
                                // terminator within ORAM_Z hops. Exceeding it means
                                // the chain has a CYCLE (corrupt insert linkage).
                                // Abort the unlink rather than hang the whole sim.
                                `ifndef SYNTHESIS
                                $display("[HT_DEL] WARN t=%0t CHAIN CYCLE DETECTED: bkt=%0d target=%0d cur=%0d hops=%0d next=0x%04h -- ABORT unlink (insert linkage is building cycles)",
                                         $time, ht_del_bkt, ht_del_target, ht_del_cur,
                                         ht_del_hops, st_sng_rdata[ht_del_field*16 +: 16]);
                                `endif
                                ht_done <= 1;
                                ht_state <= HT_IDLE;
                            end else begin
                                // Advance: cur = next[cur]; read next[cur].
                                ht_del_cur <= ht_del_next;
                                ht_del_hops <= ht_del_hops + 1;
                                st_sng_addr  <= ht_bkt_next_addr(ht_del_next);
                                st_sng_rd_req <= 1;
                                ht_sub_op <= 5;   // keep walking
                                ht_state  <= HT_WAIT_RD;
                            end
                        end else if (ht_sub_op == 6) begin
                            // next[target] arrived: capture, then RMW next[cur].
                            ht_del_field = ht_del_target[3:0];
                            ht_del_save_next <= st_sng_rdata[ht_del_field*16 +: 16];
                            st_sng_addr  <= ht_bkt_next_addr(ht_del_cur);
                            st_sng_rd_req <= 1;
                            ht_sub_op <= 9;   // read next[cur] beat for RMW
                            ht_state  <= HT_WAIT_RD;
                        end else if (ht_sub_op == 9) begin
                            // next[cur] beat re-read for RMW: next[cur] = save_next.
                            ht_del_field = ht_del_cur[3:0];
                            ht_new_beat = st_sng_rdata;
                            ht_new_beat[ht_del_field*16 +: 16] = ht_del_save_next;
                            st_sng_addr  <= ht_bkt_next_addr(ht_del_cur);
                            st_sng_wdata <= ht_new_beat;
                            st_sng_wr_req <= 1;
                            ht_sub_op <= 7;
                            ht_state  <= HT_WAIT_WR;
                        end
                    end

                    HT_OP_BKT_FIND: begin
                        ht_slot_in_beat = ht_target_bkt[3:0];
                        ht_head_val = st_sng_rdata[ht_slot_in_beat*16 +: 16];
                        `ifndef SYNTHESIS
                        // PINPOINT #5: head read at BKT_FIND. Compare addr against
                        // the head addr WR2 wrote. If head_val is 0xN000 here, the
                        // HEAD region itself holds a slot addr.
                        $display("[PIN5-FIND-HEAD] t=%0t bkt=%0d addr=0x%010h field=%0d head_val=0x%04h",
                                 $time, ht_target_bkt, st_sng_addr, ht_slot_in_beat, ht_head_val);
                        `endif
                        if (ht_head_val[15:14] != 2'b01) begin
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
                // Burst-busy hold (same rationale as HT_WAIT_RD): a dropped or
                // raced single-beat WRITE during a burst means the chain field
                // never updates and a later read sees stale data. Stall here.
                if (ht_bus_blocked) begin
                    st_sng_wr_req <= 0;
                end else begin
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
                            // SLOT-table entry cleared. Now UNLINK the node from
                            // the bucket chain (proper singly-linked removal).
                            // Begin by reading the BKT head beat for del_bkt.
                            `ifndef SYNTHESIS
                            $display("[HT_DEL] t=%0t slot cleared, begin chain unlink: bkt=%0d target_idx=%0d",
                                     $time, ht_del_bkt, ht_del_target);
                            `endif
                            st_sng_addr  <= ht_bkt_head_addr(ht_del_bkt);
                            st_sng_rd_req <= 1;
                            ht_sub_op <= 2;          // -> read head
                            ht_state  <= HT_WAIT_RD;
                        end else if (ht_sub_op == 4) begin
                            // Head was the target: head updated to next[target].
                            `ifndef SYNTHESIS
                            $display("[HT_DEL] t=%0t unlink done (was head) bkt=%0d",
                                     $time, ht_del_bkt);
                            `endif
                            ht_done <= 1;
                            ht_state <= HT_IDLE;
                        end else if (ht_sub_op == 7) begin
                            // Predecessor's next patched to skip target.
                            `ifndef SYNTHESIS
                            $display("[HT_DEL] t=%0t unlink done (mid-chain) bkt=%0d pred=%0d",
                                     $time, ht_del_bkt, ht_del_cur);
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
            end

            HT_BKT_CHAIN: begin
                // Burst-busy hold: same shared-master hazard. A chain-walk read
                // racing a burst returns stale st_sng_rdata -> phantom indices.
                if (ht_bus_blocked) begin
                    st_sng_rd_req <= 0;
                end else begin
                if (!st_sng_done)
                    st_sng_rd_req <= 1;  // keep asserted until accepted
                else
                    st_sng_rd_req <= 0;
                if (st_sng_done) begin
                    ht_slot_in_beat = ht_evict_list[ht_evict_count-1][3:0];
                    ht_next_val = st_sng_rdata[ht_slot_in_beat*16 +: 16];
                    `ifndef SYNTHESIS
                    // PINPOINT #5: chain-walk next read. addr is ht_bkt_next_addr
                    // of the PREVIOUS node (ht_evict_list[count-1]). Compare this
                    // addr+value against what WR4 wrote for that same idx. If they
                    // differ, write and read disagree on location (addressing bug);
                    // if WR4 already wrote 0xN000, the bug is upstream at sub_op 2.
                    $display("[PIN5-FIND-NEXT] t=%0t prev_idx=%0d addr=0x%010h field=%0d next_val=0x%04h",
                             $time, ht_evict_list[ht_evict_count-1], st_sng_addr,
                             ht_slot_in_beat, ht_next_val);
                    `endif

                    if (ht_next_val[15:14] != 2'b01 ||
                        ht_evict_count >= `ORAM_Z) begin
                        `ifndef SYNTHESIS
                        $display("[HT_BKT_FIND] t=%0t bkt=%0d found %0d candidates",
                                 $time, ht_target_bkt, ht_evict_count);
                        // Cap-hit is the signature of a non-terminating / corrupt
                        // chain (the old lazy-delete bug). A healthy chain ends on
                        // HT_EMPTY/0xFFFF before reaching ORAM_Z entries.
                        if (ht_evict_count >= `ORAM_Z &&
                            ht_next_val[15:14] == 2'b01)
                            $display("[HT_BKT_FIND] WARN t=%0t bkt=%0d chain hit ORAM_Z cap (%0d) WITHOUT terminator -- possible chain corruption (next_val=0x%04h)",
                                     $time, ht_target_bkt, `ORAM_Z, ht_next_val);
                        `endif
                        ht_done <= 1;
                        ht_bkt_find_done <= 1;   // Step 4: latch completion
                        ht_state <= HT_IDLE;
                    end else begin
                        ht_evict_list[ht_evict_count] <= ht_next_val[PTR_W-1:0];
                        ht_evict_count <= ht_evict_count + 1;
                        `ifndef SYNTHESIS
                        $display("[HT_BKT_FIND] t=%0t bkt=%0d chain[%0d]=idx %0d (next=0x%04h)",
                                 $time, ht_target_bkt, ht_evict_count,
                                 ht_next_val[PTR_W-1:0], ht_next_val);
                        `endif
                        st_sng_addr <= ht_bkt_next_addr(ht_next_val[PTR_W-1:0]);
                        st_sng_rd_req <= 1;
                        // Stay in HT_BKT_CHAIN
                    end
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
    // --- Per-op HT operation tracking (separate always block to avoid multi-driver) ---
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ht_ins_overwritten_r <= 0; ht_del_overwritten_r <= 0;
            ht_ins_issued_r <= 0; ht_ins_completed_r <= 0;
            ht_del_issued_r <= 0; ht_del_completed_r <= 0;
        end else begin
            // Reset counters at op start
            if (state == S_IDLE && client_req) begin
                ht_ins_overwritten_r <= 0; ht_del_overwritten_r <= 0;
                ht_ins_issued_r <= 0; ht_ins_completed_r <= 0;
                ht_del_issued_r <= 0; ht_del_completed_r <= 0;
            end
            // Count issued commands
            if (ht_cmd_slot_insert) begin
                ht_ins_issued_r <= ht_ins_issued_r + 1;
                if (ht_latch_slot_insert)
                    ht_ins_overwritten_r <= 1;
            end
            if (ht_cmd_slot_delete) begin
                ht_del_issued_r <= ht_del_issued_r + 1;
                if (ht_latch_slot_delete)
                    ht_del_overwritten_r <= 1;
            end
            // Count completed operations
            if (ht_done) begin
                if (ht_op == HT_OP_SLOT_INSERT)
                    ht_ins_completed_r <= ht_ins_completed_r + 1;
                else if (ht_op == HT_OP_SLOT_DELETE)
                    ht_del_completed_r <= ht_del_completed_r + 1;
            end
        end
    end
    `ifndef SYNTHESIS
    reg [15:0] fd_total, fd_feeding, fd_input_nr, fd_data_nr, fd_output;
    reg [15:0] fd_drd_wait, fd_scr_wait, fd_half_lo, fd_half_hi;
    reg [15:0] fd_start_total;
    reg [5:0]  fd_prev;
    reg        fd_alive;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fd_total <= 0; fd_feeding <= 0; fd_input_nr <= 0;
            fd_data_nr <= 0; fd_output <= 0; fd_drd_wait <= 0;
            fd_scr_wait <= 0; fd_half_lo <= 0; fd_half_hi <= 0;
            fd_start_total <= 0; fd_prev <= 0; fd_alive <= 0;
        end else begin
            // One-time alive check
            if (!fd_alive && state != 6'd0) begin
                fd_alive <= 1;
                $display("[AES_DIAG] ALIVE: diag block running, state=%0d", state);
            end

            // --- START states: 6, 15, 20, 24 ---
            if (state == 6'd6 || state == 6'd15 || state == 6'd20 || state == 6'd24) begin
                if (fd_prev != state)
                    fd_start_total <= 1;
                else
                    fd_start_total <= fd_start_total + 1;
            end

            // Print on exit from START
            if ((fd_prev == 6'd6 || fd_prev == 6'd15 || fd_prev == 6'd20 || fd_prev == 6'd24) &&
                state != fd_prev)
                $display("[AES_DIAG] START(st=%0d): total=%0d ir=%0d",
                         fd_prev, fd_start_total, gcm_input_ready);

            // --- FEED states: 7, 16, 21, 25 ---
            if (state == 6'd7 || state == 6'd16 || state == 6'd21 || state == 6'd25) begin
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
                    if ((state == 6'd16 || state == 6'd21) && !st_drd_valid && !aes_half)
                        fd_drd_wait <= 1;
                    if (state == 6'd7 && !scr_fsm_rd_valid && !aes_half)
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
                    if ((state == 6'd16 || state == 6'd21) && !st_drd_valid && !aes_half)
                        fd_drd_wait <= fd_drd_wait + 1;
                    if (state == 6'd7 && !scr_fsm_rd_valid && !aes_half)
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