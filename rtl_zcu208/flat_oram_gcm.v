`timescale 1ns / 1ps
`include "oram_params.vh"

// =============================================================================
// flat_oram_gcm.v - ORAM Top with AES-GCM Encryption (v2 - same-bucket fix)
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

    // Performance counters - 4 categories:
    //   rb = READ,  found in bucket    (needs decrypt + re-encrypt)
    //   rs = READ,  found in stash     (stash read, no decrypt)
    //   wb = WRITE, found in bucket    (encrypt to scratch)
    //   ws = WRITE, found in stash     (stash overwrite)
    // Each category tracks 8 phases + total + ops count.
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
    // FSM States
    // =========================================================================
    localparam [4:0]
        S_IDLE          = 5'd0,
        S_POS_LOOKUP    = 5'd1,
        S_DDR_READ      = 5'd2,
        S_SCAN          = 5'd3,
        S_SCAN2         = 5'd4,
        // --- Decrypt: read, slot in bucket, different bucket ---
        S_EXT_IV_RD     = 5'd5,
        S_EXT_DEC_START = 5'd6,
        S_EXT_DEC_FEED  = 5'd7,
        S_EXT_DEC_RECV  = 5'd8,
        S_EXT_DEC_WAIT  = 5'd9,
        // --- Write: slot in bucket, different bucket (no decrypt) ---
        S_EXTRACT_WR    = 5'd10,
        // --- Stash paths ---
        S_STASH_SEARCH  = 5'd11,
        S_STASH_READ    = 5'd12,
        S_COMPACT       = 5'd13,
        // --- Eviction encrypt ---
        S_EVICT         = 5'd14,
        S_EV_ENC_START  = 5'd15,
        S_EV_ENC_FEED   = 5'd16,
        S_EV_ENC_RECV   = 5'd17,
        S_EV_ENC_TAG    = 5'd18,
        // --- DDR write-back ---
        S_DDR_WRITE     = 5'd19,
        // --- Same-bucket read: after decrypt, re-encrypt back to scratch ---
        S_SB_RD_ENC_START = 5'd20,
        S_SB_RD_ENC_FEED  = 5'd21,
        S_SB_RD_ENC_RECV  = 5'd22,
        S_SB_RD_ENC_TAG   = 5'd23,
        // --- Same-bucket write: encrypt client data to scratch ---
        S_SB_WR_ENC_START = 5'd24,
        S_SB_WR_ENC_FEED  = 5'd25,
        S_SB_WR_ENC_RECV  = 5'd26,
        S_SB_WR_ENC_TAG   = 5'd27;

    reg [4:0] state;

    // =========================================================================
    // Internal registers
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
    reg [SLOT_AW-1:0]   ev_slot_addr;     // registered copy of st_cam_bkt_slot_addr
    wire [PTR_W:0] stash_occupancy;
    reg [BEAT_W-1:0]    beat_cnt;
    reg [POS_W-1:0]     scan_pos;
    reg [FILL_W-1:0]    evict_slots_remaining;

    // AES-GCM feed control
    reg                  aes_half;
    reg [AXI_DW-1:0]    aes_beat_buf;
    reg [8:0]            aes_blk_cnt;
    reg [BEAT_W-1:0]    aes_out_beat;
    reg                  aes_out_half;
    reg [AES_BLK_W-1:0] aes_out_lo;

    // Same-bucket read: buffer decrypted plaintext for re-encryption
    // We use stash data_mem as temporary storage (at stash_alloc_idx)
    // even though the slot won't be inserted into stash.
    // Alternative: a dedicated 4KB buffer. For simplicity, reuse stash data port.
    // Actually simpler: buffer the 128 plaintext beats in a small BRAM.
    // But we already have the plaintext coming out of GCM - we can pipeline
    // the re-encrypt right after decrypt finishes.
    // Approach: save the decrypted beats into stash data memory (as scratch space),
    // then read them back for re-encryption. This reuses existing stash data ports.
    // OR: pipeline decrypt output directly into encrypt input.
    //
    // Chosen approach: After decrypt finishes and data is returned to client,
    // we start a fresh encrypt. We read the plaintext from the stash data memory
    // where we temporarily stored it during decrypt (using stash_alloc_idx).
    // After encrypt, we DON'T call st_ins_en (don't actually insert into stash).
    // The stash entry is just scratch space.

    // Same-bucket write: we need to feed 128 beats of client data to GCM encrypt.
    // The client provides data beat-by-beat via client_wdata.

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
    // Submodules (unchanged from original)
    // =========================================================================

    // --- pos_map ---
    reg  [SLOT_AW-1:0]  pm_rd_addr;  reg pm_rd_en;
    wire [BUCKET_W-1:0]  pm_rd_bucket; wire [1:0] pm_rd_status; wire pm_rd_valid;
    reg  [SLOT_AW-1:0]  pm_wr_addr;  reg [BUCKET_W-1:0] pm_wr_bucket;
    reg  [1:0]           pm_wr_status; reg pm_wr_en;

    wire [SLOT_AW-1:0]  pm_wr_addr_mux   = init_mode ? init_pm_wr_addr   : pm_wr_addr;
    wire [BUCKET_W-1:0] pm_wr_bucket_mux = init_mode ? init_pm_wr_bucket : pm_wr_bucket;
    wire [1:0]          pm_wr_status_mux = init_mode ? init_pm_wr_status : pm_wr_status;
    wire                pm_wr_en_mux     = init_mode ? init_pm_wr_en     : pm_wr_en;

    pos_map u_pos_map (
        .clk(clk), .rst_n(rst_n),
        .rd_slot_addr(pm_rd_addr), .rd_en(pm_rd_en),
        .rd_bucket(pm_rd_bucket), .rd_status(pm_rd_status), .rd_valid(pm_rd_valid),
        .wr_slot_addr(pm_wr_addr_mux), .wr_bucket(pm_wr_bucket_mux),
        .wr_status(pm_wr_status_mux), .wr_en(pm_wr_en_mux)
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
    wire st_cam_slot_hit; wire [PTR_W-1:0] st_cam_slot_idx;
    reg  [BUCKET_W-1:0] st_cam_bkt_target; reg st_cam_bkt_en;
    wire st_cam_bkt_hit; wire [PTR_W-1:0] st_cam_bkt_idx;
    wire [SLOT_AW-1:0] st_cam_bkt_slot_addr; wire st_cam_bkt_inhibit_busy;

    stash #(.DEPTH(STASH_DEPTH)) u_stash (
        .clk(clk), .rst_n(rst_n),
        .cam_slot_addr(req_slot_addr), .cam_slot_en(1'b1),
        .cam_slot_hit(st_cam_slot_hit), .cam_slot_idx(st_cam_slot_idx),
        .cam_bkt_target(st_cam_bkt_target), .cam_bkt_en(st_cam_bkt_en),
        .cam_bkt_hit(st_cam_bkt_hit), .cam_bkt_idx(st_cam_bkt_idx),
        .cam_bkt_slot_addr(st_cam_bkt_slot_addr),
        .cam_bkt_inhibit_busy(st_cam_bkt_inhibit_busy),
        .ins_slot_addr(st_ins_slot), .ins_target_bucket(st_ins_tgt), .ins_en(st_ins_en),
        .ins_idx(st_ins_idx), .ins_full(st_ins_full),
        .remap_idx(st_remap_idx), .remap_new_bucket(st_remap_bkt), .remap_en(st_remap_en),
        .evict_idx(st_evict_idx), .evict_en(st_evict_en),
        .dwr_entry(st_dwr_entry), .dwr_beat(st_dwr_beat),
        .dwr_data(st_dwr_data), .dwr_en(st_dwr_en),
        .drd_entry(st_drd_entry), .drd_beat(st_drd_beat), .drd_en(st_drd_en),
        .drd_data(st_drd_data), .drd_valid(st_drd_valid),
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
    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= S_IDLE;
            client_done <= 0; client_stall <= 0; client_rdata_valid <= 0;
            err_bucket_overflow <= 0; err_stash_overflow <= 0; err_tag_mismatch <= 0;
            beat_cnt <= 0; evict_slots_remaining <= 0;
            found_in_bucket <= 0; found_in_stash <= 0;
            found_pos <= 0; found_stash_idx <= 0; stash_alloc_idx <= 0;
            req_b <= 0; req_b_new <= 0; bkt_fill_count <= 0;
            bkt_slot_list <= 0; same_bucket <= 0;
            pm_rd_en <= 0; pm_wr_en <= 0; pm_wr_status <= `ST_DUMMY;
            bm_rd_en <= 0; bm_wr_en <= 0; prng_en <= 0;
            st_ins_en <= 0; st_remap_en <= 0; st_evict_en <= 0;
            st_dwr_en <= 0; st_drd_en <= 0; st_cam_bkt_en <= 0;
            axim_cmd_read <= 0; axim_cmd_write <= 0;
            scr_fsm_rd_en <= 0; scr_fsm_wr_en <= 0;
            gcm_start <= 0; gcm_data_valid <= 0; gcm_data_last <= 0;
            gcm_encrypt <= 0; gcm_iv_reg <= 0;
            aes_half <= 0; aes_blk_cnt <= 0;
            aes_out_beat <= 0; aes_out_half <= 0;
            ivt_rd_en <= 0; ivt_wr_en <= 0;
        end else begin
            // Defaults
            client_done <= 0; client_rdata_valid <= 0;
            pm_rd_en <= 0; pm_wr_en <= 0;
            bm_rd_en <= 0; bm_wr_en <= 0; prng_en <= 0;
            st_ins_en <= 0; st_remap_en <= 0; st_evict_en <= 0;
            st_dwr_en <= 0; st_drd_en <= 0; st_cam_bkt_en <= 0;
            axim_cmd_read <= 0; axim_cmd_write <= 0;
            scr_fsm_rd_en <= 0; scr_fsm_wr_en <= 0;
            gcm_start <= 0; gcm_data_valid <= 0; gcm_data_last <= 0;
            ivt_rd_en <= 0; ivt_wr_en <= 0;

            case (state)

                // =============================================================
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
                        pm_rd_addr <= client_slot_addr; pm_rd_en <= 1;
                        prng_en <= 1;
                        state <= S_POS_LOOKUP;
                    end
                end

                S_POS_LOOKUP: begin
                    if (pm_rd_valid) begin
                        req_b <= pm_rd_bucket;
                        req_b_new <= dbg_force_same_bucket ? pm_rd_bucket :
                                     (|dbg_prng_override)  ? dbg_prng_override :
                                                             prng_bucket;
                        same_bucket <= dbg_force_same_bucket ? 1'b1 :
                                      (|dbg_prng_override)  ? (dbg_prng_override == pm_rd_bucket) :
                                                              (prng_bucket == pm_rd_bucket);
                        pm_wr_addr <= req_slot_addr;
                        pm_wr_bucket <= dbg_force_same_bucket ? pm_rd_bucket :
                                       (|dbg_prng_override)  ? dbg_prng_override :
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
                    begin : scan_block
                        integer i; reg hit; reg [POS_W-1:0] hit_pos;
                        hit = 0; hit_pos = 0;
                        for (i = 0; i < Z; i = i + 1)
                            if (i < bkt_fill_count)
                                if (get_slot_at(bkt_slot_list, i[POS_W-1:0]) == req_slot_addr)
                                    begin hit = 1; hit_pos = i[POS_W-1:0]; end
                        found_in_bucket <= hit;
                        found_pos <= hit_pos;
                    end
                    found_in_stash <= st_cam_slot_hit;
                    found_stash_idx <= st_cam_slot_idx;
                    state <= S_SCAN2;
                end

                // =============================================================
                // S_SCAN2: branch based on found_in_bucket, same_bucket, req_op
                // =============================================================
                S_SCAN2: begin
                    `ifndef SYNTHESIS
                    $display("[SCAN2] t=%0t slot=0x%08h in_bkt=%b in_stash=%b same_bkt=%b op=%b found_pos=%0d",
                             $time, req_slot_addr, found_in_bucket, found_in_stash,
                             same_bucket, req_op, found_pos);
                    `endif
                    if (found_in_bucket) begin
                        stash_alloc_idx <= st_ins_idx;
                        st_ins_slot <= req_slot_addr;
                        st_ins_tgt <= req_b_new;

                        if (same_bucket) begin
                            // SAME BUCKET: data stays in bucket, no stash insert
                            if (req_op == 0) begin
                                // Same-bucket READ: decrypt -> client -> re-encrypt to scratch
                                ivt_rd_addr <= req_slot_addr;
                                ivt_rd_en <= 1;
                                state <= S_EXT_IV_RD;
                                // Flag: S_EXT_DEC_WAIT will branch to re-encrypt
                            end else begin
                                // Same-bucket WRITE: encrypt client data to scratch
                                gcm_encrypt <= 1;
                                gcm_iv_reg <= aes_iv_seed;
                                gcm_start <= 1;
                                beat_cnt <= 0;
                                aes_half <= 0; aes_blk_cnt <= 0;
                                aes_out_beat <= 0; aes_out_half <= 0;
                                state <= S_SB_WR_ENC_START;
                            end
                        end else begin
                            // DIFFERENT BUCKET: slot moves to stash
                            if (req_op == 0) begin
                                // Decrypt from scratch -> stash + client
                                ivt_rd_addr <= req_slot_addr;
                                ivt_rd_en <= 1;
                                state <= S_EXT_IV_RD;
                            end else begin
                                // Client data direct to stash
                                beat_cnt <= 0;
                                state <= S_EXTRACT_WR;
                            end
                        end
                    end else begin
                        state <= S_STASH_SEARCH;
                    end
                end

                // =============================================================
                // DECRYPT PATH (read from bucket - both same and different bucket)
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

                // Feed ciphertext to GCM decrypt, receive plaintext
                S_EXT_DEC_FEED: begin
                    // Re-issue scratch read if no valid data yet (cold-start only)
                    if (!scr_fsm_rd_valid && !aes_half) begin
                        scr_fsm_rd_addr <= scr_addr(found_pos, beat_cnt);
                        scr_fsm_rd_en <= 1;
                    end

                    // LOW HALF: needs BRAM valid - feed low 128 bits + prefetch next beat
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
                        // PREFETCH: issue next read 1 cycle earlier
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            scr_fsm_rd_addr <= scr_addr(found_pos, beat_cnt + 1);
                            scr_fsm_rd_en <= 1;
                        end
                    end

                    // HIGH HALF: uses aes_beat_buf - does NOT need BRAM valid
                    if (gcm_input_ready && aes_half) begin
                        gcm_data_in <= aes_beat_buf[255:128];
                        gcm_data_valid <= 1;
                        gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                        aes_half <= 0;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            beat_cnt <= beat_cnt + 1;
                        end else begin
                            state <= S_EXT_DEC_RECV;
                        end
                    end
                    // Receive decrypted output
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
                            // Write to stash temp storage (used by both paths)
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

                // After decrypt: branch based on same_bucket
                S_EXT_DEC_WAIT: begin
                    if (gcm_tag_valid) begin
                        if (!gcm_tag_match)
                            err_tag_mismatch <= 1;

                        if (same_bucket) begin
                            // Same bucket READ: re-encrypt from stash temp back to scratch
                            gcm_encrypt <= 1;
                            gcm_iv_reg <= aes_iv_seed;
                            gcm_start <= 1;
                            // Read plaintext from stash (where we just wrote it)
                            st_drd_entry <= stash_alloc_idx;
                            st_drd_beat <= 0;
                            st_drd_en <= 1;
                            beat_cnt <= 0;
                            aes_half <= 0; aes_blk_cnt <= 0;
                            aes_out_beat <= 0; aes_out_half <= 0;
                            state <= S_SB_RD_ENC_START;
                        end else begin
                            // Different bucket: insert into stash
                            st_ins_en <= 1;
                            if (st_ins_full) err_stash_overflow <= 1;
                            pm_wr_addr <= req_slot_addr;
                            pm_wr_bucket <= req_b_new;
                            pm_wr_status <= `ST_IN_STASH;
                            pm_wr_en <= 1;
                            bkt_slot_list <= set_slot_at(bkt_slot_list, found_pos,
                                bkt_slot_list[(bkt_fill_count-1)*SLOT_W +: SLOT_W]);
                            bkt_fill_count <= bkt_fill_count - 1;
                            state <= S_COMPACT;
                        end
                    end
                end

                // =============================================================
                // SAME-BUCKET READ: re-encrypt plaintext back to scratch
                // =============================================================
                S_SB_RD_ENC_START: begin
                    if (gcm_input_ready)
                        state <= S_SB_RD_ENC_FEED;
                end

                S_SB_RD_ENC_FEED: begin
                    // Re-issue stash read if no valid data yet (cold-start)
                    if (!st_drd_valid && !aes_half) begin
                        st_drd_entry <= stash_alloc_idx;
                        st_drd_beat <= beat_cnt;
                        st_drd_en <= 1;
                    end

                    // LOW HALF: needs stash valid - feed low + prefetch next
                    if (st_drd_valid && gcm_input_ready && !aes_half) begin
                        gcm_data_in <= st_drd_data[127:0];
                        gcm_data_valid <= 1;
                        aes_beat_buf <= st_drd_data;
                        aes_half <= 1;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            st_drd_entry <= stash_alloc_idx;
                            st_drd_beat <= beat_cnt + 1;
                            st_drd_en <= 1;
                        end
                    end

                    // HIGH HALF: uses aes_beat_buf - no valid gate
                    if (gcm_input_ready && aes_half) begin
                        gcm_data_in <= aes_beat_buf[255:128];
                        gcm_data_valid <= 1;
                        gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                        aes_half <= 0;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
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
                        // Slot stays in bucket - no stash insert, no compaction
                        state <= S_COMPACT;
                    end
                end

                // =============================================================
                // SAME-BUCKET WRITE: encrypt client data to scratch
                // =============================================================
                S_SB_WR_ENC_START: begin
                    if (gcm_input_ready)
                        state <= S_SB_WR_ENC_FEED;
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
                    // Receive ciphertext to scratch
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
                        // Slot stays in bucket - no stash, no compaction needed
                        state <= S_COMPACT;
                    end
                end

                // =============================================================
                // WRITE PATH: different bucket, client data direct to stash
                // =============================================================
                S_EXTRACT_WR: begin
                    if (client_wdata_valid) begin
                        st_dwr_entry <= stash_alloc_idx;
                        st_dwr_beat <= beat_cnt;
                        st_dwr_data <= client_wdata;
                        st_dwr_en <= 1;
                        if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                            if (!found_in_stash) begin
                                // WRITE/BUCKET or WRITE/DUMMY: new stash entry
                                st_ins_en <= 1;
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
                            // WRITE/STASH: slot already in stash, remap already done
                            state <= S_COMPACT;
                        end else begin
                            beat_cnt <= beat_cnt + 1;
                        end
                    end
                end

                // =============================================================
                // STASH paths (unchanged)
                // =============================================================
                S_STASH_SEARCH: begin
                    if (found_in_stash) begin
                        st_remap_idx <= found_stash_idx;
                        st_remap_bkt <= req_b_new;
                        st_remap_en <= 1;
                        if (req_op) begin
                            // WRITE/STASH: overwrite stash entry directly, no read needed
                            beat_cnt <= 0;
                            stash_alloc_idx <= found_stash_idx;
                            state <= S_EXTRACT_WR;
                        end else begin
                            // READ/STASH: read stash data to client
                            st_drd_entry <= found_stash_idx;
                            st_drd_beat <= 0; st_drd_en <= 1;
                            beat_cnt <= 0;
                            state <= S_STASH_READ;
                        end
                    end else if (req_op == 1'b1) begin
                        // WRITE to a DUMMY slot (never accessed before)
                        // Allocate a stash entry and write client data there
                        stash_alloc_idx <= st_ins_idx;
                        st_ins_slot <= req_slot_addr;
                        st_ins_tgt <= req_b_new;
                        beat_cnt <= 0;
                        state <= S_EXTRACT_WR;
                    end else begin
                        // READ of a DUMMY slot - no data exists
                        `ifndef SYNTHESIS
                        $display("[WARN] t=%0t READ of DUMMY slot 0x%08h - no data", $time, req_slot_addr);
                        `endif
                        state <= S_COMPACT;
                    end
                end

                S_STASH_READ: begin
                    // READ/STASH only - return stash data to client
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
                    end
                end

                // =============================================================
                S_COMPACT: begin
                    evict_slots_remaining <= {{(FILL_W-POS_W-1){1'b0}}, Z[POS_W:0]} - {1'b0, bkt_fill_count};
                    st_cam_bkt_target <= req_b;
                    st_cam_bkt_en <= 1;
                    beat_cnt <= 0;
                    state <= S_EVICT;
                end

                // =============================================================
                // EVICT with encryption
                // =============================================================
                S_EVICT: begin
                    st_cam_bkt_en <= 1;
                    st_cam_bkt_target <= req_b;
                    if (st_cam_bkt_inhibit_busy) begin
                        st_drd_en <= 0;
                    end else if (st_cam_bkt_hit && (evict_slots_remaining > 0)) begin
                        `ifndef SYNTHESIS
                        $display("[EV_START] t=%0t evict slot=0x%08h stash_idx=%0d -> bucket=%0d pos=%0d",
                                 $time, st_cam_bkt_slot_addr, st_cam_bkt_idx, req_b, bkt_fill_count);
                        `endif
                        gcm_encrypt <= 1;
                        gcm_iv_reg <= aes_iv_seed;
                        gcm_start <= 1;
                        aes_half <= 0; aes_blk_cnt <= 0;
                        aes_out_beat <= 0; aes_out_half <= 0;
                        beat_cnt <= 0;
                        st_drd_entry <= st_cam_bkt_idx;
                        st_drd_beat <= 0; st_drd_en <= 1;
                        ev_slot_addr <= st_cam_bkt_slot_addr;  // register CAM output
                        state <= S_EV_ENC_START;
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
                    // Wait for GCM init to complete
                    if (gcm_input_ready)
                        state <= S_EV_ENC_FEED;
                end

                S_EV_ENC_FEED: begin
                    // Re-issue stash read if no valid data yet (cold-start)
                    if (!st_drd_valid && !aes_half) begin
                        st_drd_entry <= st_cam_bkt_idx;
                        st_drd_beat <= beat_cnt;
                        st_drd_en <= 1;
                    end

                    // LOW HALF: needs stash valid - feed low + prefetch next
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
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
                            st_drd_entry <= st_cam_bkt_idx;
                            st_drd_beat <= beat_cnt + 1;
                            st_drd_en <= 1;
                        end
                    end

                    // HIGH HALF: uses aes_beat_buf - no valid gate
                    if (gcm_input_ready && aes_half) begin
                        gcm_data_in <= aes_beat_buf[255:128];
                        gcm_data_valid <= 1;
                        gcm_data_last <= (beat_cnt == BEATS[BEAT_W-1:0] - 1);
                        aes_half <= 0;
                        aes_blk_cnt <= aes_blk_cnt + 1;
                        if (beat_cnt < BEATS[BEAT_W-1:0] - 1) begin
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
                        st_evict_idx <= st_cam_bkt_idx;
                        st_evict_en <= 1;
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
            endcase
        end
    end

    // =========================================================================
    // Performance Counters - 4-way: rd_bkt, rd_stash, wr_bkt, wr_stash
    // Category captured at S_SCAN2 from req_op and found_in_bucket.
    // =========================================================================
    localparam [1:0] CAT_RB = 2'b00,  // READ,  found in bucket
                     CAT_RS = 2'b01,  // READ,  found in stash
                     CAT_WB = 2'b10,  // WRITE, found in bucket
                     CAT_WS = 2'b11;  // WRITE, found in stash

    reg        perf_active;
    reg [1:0]  perf_cat;      // latched at S_SCAN2

    // Latch category when S_SCAN2 is active (combinational, used by FSM)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            perf_cat <= CAT_RB;
        else if (state == S_SCAN2)
            perf_cat <= {req_op, ~found_in_bucket};
    end

    // Increment helper: select counter set based on perf_cat
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
            // Active tracking + ops count
            if (state != S_IDLE)
                perf_active <= 1'b1;
            if (state == S_IDLE && perf_active) begin
                perf_active <= 1'b0;
                `PERF_INC(ops)
            end

            // Total
            if (state != S_IDLE) begin
                `PERF_INC(total)
            end

            // Per-phase
            case (state)
                S_POS_LOOKUP, S_SCAN, S_SCAN2: begin
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

endmodule