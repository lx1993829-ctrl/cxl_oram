// =============================================================================
// flat_oram_top.v - Flat ORAM Top-Level FSM (v46 - 1-beat-per-cycle pipeline)
// =============================================================================
// Orchestrates all submodules to implement the 8-step ORAM access protocol.
//
// Client interface:
//   Client sends: op (RD/WR), slot_addr (32-bit logical), wdata (4K for WR)
//   ORAM returns: rdata (4K), done pulse
//
// Submodules:
//   pos_map      - slot_addr ? bucket_id BRAM
//   bucket_meta  - bucket ? slot_list + fill_count BRAM
//   slot_status  - slot ? VALID/DUMMY/IN_STASH BRAM
//   scratch_buf  - Z×4K working buffer
//   stash        - overflow stash with CAM
//   prng_lfsr    - random bucket generator
//   axi_master   - DDR burst read/write

`include "oram_params.vh"

module flat_oram_top #(
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
    parameter TOTAL_BEATS  = `BEATS_PER_BKT
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // -----------------------------------------------------------------
    // Client Interface
    // -----------------------------------------------------------------
    input  wire                 client_req,
    input  wire                 client_op,          // 0=READ, 1=WRITE
    input  wire [SLOT_AW-1:0]   client_slot_addr,
    input  wire [AXI_DW-1:0]   client_wdata,
    input  wire                 client_wdata_valid,
    input  wire [BEAT_W-1:0]    client_wdata_beat,
    output wire [BEAT_W-1:0]    client_wdata_beat_req,
    output reg  [AXI_DW-1:0]   client_rdata,
    output reg                  client_rdata_valid,
    output reg  [BEAT_W-1:0]    client_rdata_beat,
    output reg                  client_done,
    output reg                  client_stall,

    // -----------------------------------------------------------------
    // Overflow / Error Flags
    // -----------------------------------------------------------------
    output reg                  err_bucket_overflow,
    output reg                  err_stash_overflow,

    // -----------------------------------------------------------------
    // BRAM Init Interface (active only when init_mode=1 and FSM is idle)
    // -----------------------------------------------------------------
    input  wire                     init_mode,
    input  wire [SLOT_AW-1:0]       init_pm_wr_addr,
    input  wire [BUCKET_W-1:0]      init_pm_wr_bucket,
    input  wire [1:0]               init_pm_wr_status,
    input  wire                     init_pm_wr_en,
    input  wire [BUCKET_W-1:0]      init_bm_wr_bucket,
    input  wire [Z*SLOT_W-1:0]      init_bm_wr_slot_list,
    input  wire [FILL_W-1:0]        init_bm_wr_fill,
    input  wire                     init_bm_wr_en,

    // -----------------------------------------------------------------
    // AXI4 Master ? DDR
    // -----------------------------------------------------------------
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

    // -----------------------------------------------------------------
    // Debug outputs (directly exposed for ILA probing)
    // -----------------------------------------------------------------
    output wire [3:0]            dbg_state,
    output wire                  dbg_client_done,
    output wire                  dbg_client_req,
    output wire [BUCKET_W-1:0]  dbg_req_b,
    output wire [BUCKET_W-1:0]  dbg_req_b_new,
    output wire                  dbg_found_in_bucket,
    output wire                  dbg_found_in_stash,
    output wire                  dbg_same_bucket,
    output wire                  dbg_err_stash_ovf,
    output wire [PTR_W:0]       dbg_stash_occ
);

    // =========================================================================
    // Internal registers
    // =========================================================================
    reg [SLOT_AW-1:0]   req_slot_addr;
    reg                  req_op;
    reg [BUCKET_W-1:0]   req_b;
    reg [BUCKET_W-1:0]   req_b_new;

    reg [Z*SLOT_W-1:0]   bkt_slot_list;
    reg [FILL_W-1:0]     bkt_fill_count;

    reg                  same_bucket;

    reg                  found_in_bucket;
    reg                  found_in_stash;
    reg [POS_W-1:0]      found_pos;
    reg [PTR_W-1:0]      found_stash_idx;

    reg [PTR_W-1:0]      stash_alloc_idx;

    // =========================================================================
    // FSM State
    // =========================================================================
    reg [3:0] state;

    reg [BEAT_W-1:0]    beat_cnt;
    reg [POS_W-1:0]     scan_pos;
    reg [POS_W-1:0]     compact_src;
    reg [FILL_W-1:0]    evict_slots_remaining;

    // =========================================================================
    // Submodule wires - pos_map
    // =========================================================================
    reg  [SLOT_AW-1:0]   pm_rd_addr;
    reg                   pm_rd_en;
    wire [BUCKET_W-1:0]   pm_rd_bucket;
    wire [1:0]            pm_rd_status;
    wire                  pm_rd_valid;
    reg  [SLOT_AW-1:0]   pm_wr_addr;
    reg  [BUCKET_W-1:0]   pm_wr_bucket;
    reg  [1:0]            pm_wr_status;
    reg                   pm_wr_en;

    // Mux: init_mode selects between init ports and FSM-driven regs
    wire [SLOT_AW-1:0]   pm_wr_addr_mux   = init_mode ? init_pm_wr_addr   : pm_wr_addr;
    wire [BUCKET_W-1:0]  pm_wr_bucket_mux = init_mode ? init_pm_wr_bucket : pm_wr_bucket;
    wire [1:0]           pm_wr_status_mux = init_mode ? init_pm_wr_status : pm_wr_status;
    wire                 pm_wr_en_mux     = init_mode ? init_pm_wr_en     : pm_wr_en;

    pos_map u_pos_map (
        .clk(clk), .rst_n(rst_n),
        .rd_slot_addr(pm_rd_addr), .rd_en(pm_rd_en),
        .rd_bucket(pm_rd_bucket),  .rd_status(pm_rd_status), .rd_valid(pm_rd_valid),
        .wr_slot_addr(pm_wr_addr_mux), .wr_bucket(pm_wr_bucket_mux),
        .wr_status(pm_wr_status_mux),  .wr_en(pm_wr_en_mux)
    );

    // =========================================================================
    // Submodule wires - bucket_meta
    // =========================================================================
    reg  [BUCKET_W-1:0]   bm_rd_bucket;
    reg                    bm_rd_en;
    wire [Z*SLOT_W-1:0]   bm_rd_slot_list;
    wire [FILL_W-1:0]      bm_rd_fill;
    wire                   bm_rd_valid;
    reg  [BUCKET_W-1:0]   bm_wr_bucket;
    reg  [Z*SLOT_W-1:0]   bm_wr_slot_list;
    reg  [FILL_W-1:0]      bm_wr_fill;
    reg                    bm_wr_en;

    // Mux: init_mode selects between init ports and FSM-driven regs
    wire [BUCKET_W-1:0]      bm_wr_bucket_mux    = init_mode ? init_bm_wr_bucket    : bm_wr_bucket;
    wire [Z*SLOT_W-1:0]     bm_wr_slot_list_mux = init_mode ? init_bm_wr_slot_list : bm_wr_slot_list;
    wire [FILL_W-1:0]        bm_wr_fill_mux      = init_mode ? init_bm_wr_fill      : bm_wr_fill;
    wire                     bm_wr_en_mux        = init_mode ? init_bm_wr_en        : bm_wr_en;

    bucket_meta u_bucket_meta (
        .clk(clk), .rst_n(rst_n),
        .rd_bucket(bm_rd_bucket), .rd_en(bm_rd_en),
        .rd_slot_list(bm_rd_slot_list), .rd_fill_count(bm_rd_fill), .rd_valid(bm_rd_valid),
        .wr_bucket(bm_wr_bucket_mux), .wr_slot_list(bm_wr_slot_list_mux),
        .wr_fill_count(bm_wr_fill_mux), .wr_en(bm_wr_en_mux)
    );

    // =========================================================================
    // Submodule wires - PRNG
    // =========================================================================
    reg   prng_en;
    wire [BUCKET_W-1:0] prng_bucket;

    prng_lfsr #(.OUT_W(BUCKET_W)) u_prng (
        .clk(clk), .rst_n(rst_n),
        .seed({BUCKET_W{1'b0}}), .seed_valid(1'b0),
        .en(prng_en),
        .rand_out(), .rand_bucket(prng_bucket)
    );

    // =========================================================================
    // Submodule wires - scratch_buf
    // =========================================================================
    wire [AXI_DW-1:0]  scr_stream_in_data;
    wire                scr_stream_in_valid;
    wire [9:0]          scr_stream_in_beat_addr;

    reg  [9:0]          scr_fsm_rd_addr;
    reg                 scr_fsm_rd_en;
    wire [AXI_DW-1:0]  scr_fsm_rd_data;
    wire                scr_fsm_rd_valid;

    reg  [9:0]          scr_fsm_wr_addr;
    reg  [AXI_DW-1:0]  scr_fsm_wr_data;
    reg                 scr_fsm_wr_en;

    wire [9:0]          scr_stream_out_beat_addr;
    wire                scr_stream_out_rd_en;
    wire [AXI_DW-1:0]  scr_stream_out_data;
    wire                scr_stream_out_valid;

    scratch_buf u_scratch (
        .clk(clk), .rst_n(rst_n),
        .stream_in_data(scr_stream_in_data),
        .stream_in_valid(scr_stream_in_valid),
        .stream_in_beat_addr(scr_stream_in_beat_addr),
        .stream_in_ready(),
        .fsm_rd_addr(scr_fsm_rd_addr), .fsm_rd_en(scr_fsm_rd_en),
        .fsm_rd_data(scr_fsm_rd_data), .fsm_rd_valid(scr_fsm_rd_valid),
        .fsm_wr_addr(scr_fsm_wr_addr), .fsm_wr_data(scr_fsm_wr_data),
        .fsm_wr_en(scr_fsm_wr_en),
        .stream_out_beat_addr(scr_stream_out_beat_addr),
        .stream_out_rd_en(scr_stream_out_rd_en),
        .stream_out_data(scr_stream_out_data),
        .stream_out_valid(scr_stream_out_valid)
    );

    // =========================================================================
    // Submodule wires - stash
    // =========================================================================
    reg  [SLOT_AW-1:0]   st_ins_slot;
    reg  [BUCKET_W-1:0]   st_ins_tgt;
    reg                   st_ins_en;
    wire [PTR_W-1:0]      st_ins_idx;
    wire                  st_ins_full;

    reg  [PTR_W-1:0]      st_remap_idx;
    reg  [BUCKET_W-1:0]   st_remap_bkt;
    reg                   st_remap_en;

    reg  [PTR_W-1:0]      st_evict_idx;
    reg                   st_evict_en;

    reg  [PTR_W-1:0]      st_dwr_entry;
    reg  [BEAT_W-1:0]     st_dwr_beat;
    reg  [AXI_DW-1:0]     st_dwr_data;
    reg                   st_dwr_en;

    reg  [PTR_W-1:0]      st_drd_entry;
    reg  [BEAT_W-1:0]     st_drd_beat;
    reg                   st_drd_en;
    wire [AXI_DW-1:0]     st_drd_data;
    wire                  st_drd_valid;

    wire                  st_cam_slot_hit;
    wire [PTR_W-1:0]      st_cam_slot_idx;

    reg  [BUCKET_W-1:0]   st_cam_bkt_target;
    reg                   st_cam_bkt_en;
    wire                  st_cam_bkt_hit;
    wire [PTR_W-1:0]      st_cam_bkt_idx;
    wire [SLOT_AW-1:0]    st_cam_bkt_slot_addr;
    wire                  st_cam_bkt_inhibit_busy;

    stash #(.DEPTH(STASH_DEPTH)) u_stash (
        .clk(clk), .rst_n(rst_n),
        .cam_slot_addr(req_slot_addr), .cam_slot_en(1'b1),
        .cam_slot_hit(st_cam_slot_hit), .cam_slot_idx(st_cam_slot_idx),
        .cam_bkt_target(st_cam_bkt_target), .cam_bkt_en(st_cam_bkt_en),
        .cam_bkt_hit(st_cam_bkt_hit),   .cam_bkt_idx(st_cam_bkt_idx),
        .cam_bkt_slot_addr(st_cam_bkt_slot_addr),
        .cam_bkt_inhibit_busy(st_cam_bkt_inhibit_busy),
        .ins_slot_addr(st_ins_slot), .ins_target_bucket(st_ins_tgt), .ins_en(st_ins_en),
        .ins_idx(st_ins_idx), .ins_full(st_ins_full),
        .remap_idx(st_remap_idx), .remap_new_bucket(st_remap_bkt), .remap_en(st_remap_en),
        .evict_idx(st_evict_idx), .evict_en(st_evict_en),
        .dwr_entry(st_dwr_entry), .dwr_beat(st_dwr_beat),
        .dwr_data(st_dwr_data),   .dwr_en(st_dwr_en),
        .drd_entry(st_drd_entry), .drd_beat(st_drd_beat),
        .drd_en(st_drd_en),
        .drd_data(st_drd_data),   .drd_valid(st_drd_valid),
        .occupancy()
    );

    // =========================================================================
    // Submodule wires - axi_master
    // =========================================================================
    reg                   axim_cmd_read;
    reg                   axim_cmd_write;
    reg  [BUCKET_W-1:0]   axim_cmd_bucket;
    wire                  axim_read_done;
    wire                  axim_write_done;
    wire                  axim_busy;

    wire [AXI_DW-1:0]    axim_rdata_data;
    wire                  axim_rdata_valid;
    wire [9:0]            axim_rdata_beat;

    wire [9:0]            axim_wdata_beat_req;

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

    // Wire AXI read data ? scratch stream-in
    assign scr_stream_in_data      = axim_rdata_data;
    assign scr_stream_in_valid     = axim_rdata_valid;
    assign scr_stream_in_beat_addr = axim_rdata_beat;

    // Wire scratch stream-out ? AXI write data
    assign scr_stream_out_beat_addr = axim_wdata_beat_req;
    assign scr_stream_out_rd_en     = (state == `S_DDR_WRITE);

    // Expose current beat index to client
    assign client_wdata_beat_req = beat_cnt;


    // =========================================================================
    // Helper functions
    // =========================================================================
    function [SLOT_AW-1:0] get_slot_at;
        input [Z*SLOT_W-1:0] list;
        input [POS_W-1:0]    pos;
        integer base;
        begin
            base = pos * SLOT_W;
            get_slot_at = list[base +: SLOT_W] << 12;
        end
    endfunction

    function [Z*SLOT_W-1:0] set_slot_at;
        input [Z*SLOT_W-1:0] list;
        input [POS_W-1:0]    pos;
        input [SLOT_W-1:0]   val;
        integer base;
        begin
            set_slot_at = list;
            base = pos * SLOT_W;
            set_slot_at[base +: SLOT_W] = val;
        end
    endfunction

    function [9:0] scr_addr;
        input [POS_W-1:0]  p;
        input [BEAT_W-1:0] b;
        begin
            scr_addr = {p, b[6:0]};
        end
    endfunction

    // =========================================================================
    // Debug assigns
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
    assign dbg_stash_occ      = u_stash.occupancy;

    // =========================================================================
    // Main FSM
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= `S_IDLE;
            client_done         <= 1'b0;
            client_stall        <= 1'b0;
            client_rdata_valid  <= 1'b0;
            err_bucket_overflow <= 1'b0;
            err_stash_overflow  <= 1'b0;
            beat_cnt              <= {BEAT_W{1'b0}};
            evict_slots_remaining <= {FILL_W{1'b0}};
            found_in_bucket       <= 1'b0;
            found_in_stash        <= 1'b0;
            found_pos             <= {POS_W{1'b0}};
            found_stash_idx       <= {PTR_W{1'b0}};
            stash_alloc_idx       <= {PTR_W{1'b0}};
            req_b                 <= {BUCKET_W{1'b0}};
            req_b_new             <= {BUCKET_W{1'b0}};
            bkt_fill_count        <= {FILL_W{1'b0}};
            bkt_slot_list         <= {(Z*SLOT_W){1'b0}};
            pm_rd_en  <= 1'b0; pm_wr_en  <= 1'b0; pm_wr_status <= `ST_DUMMY;
            same_bucket <= 1'b0;
            bm_rd_en  <= 1'b0; bm_wr_en  <= 1'b0;
            prng_en   <= 1'b0;
            st_ins_slot <= {SLOT_AW{1'b0}};
            st_ins_tgt  <= {BUCKET_W{1'b0}};
            st_ins_en <= 1'b0; st_remap_en <= 1'b0;
            st_evict_en <= 1'b0; st_dwr_en <= 1'b0;
            st_drd_en <= 1'b0; st_cam_bkt_en <= 1'b0; st_cam_bkt_target <= {BUCKET_W{1'b0}};
            axim_cmd_read <= 1'b0; axim_cmd_write <= 1'b0;
            axim_cmd_bucket <= {BUCKET_W{1'b0}};
            scr_fsm_rd_en <= 1'b0; scr_fsm_wr_en <= 1'b0;
        end else begin
            // Defaults
            client_done        <= 1'b0;
            client_rdata_valid <= 1'b0;
            pm_rd_en    <= 1'b0; pm_wr_en   <= 1'b0; pm_wr_status <= `ST_DUMMY;
            bm_rd_en    <= 1'b0; bm_wr_en   <= 1'b0;
            prng_en     <= 1'b0;
            st_ins_en   <= 1'b0; st_remap_en  <= 1'b0;
            st_evict_en <= 1'b0; st_dwr_en    <= 1'b0;
            st_drd_en   <= 1'b0; st_cam_bkt_en <= 1'b0;
            axim_cmd_read  <= 1'b0;
            axim_cmd_write <= 1'b0;
            scr_fsm_rd_en  <= 1'b0; scr_fsm_wr_en <= 1'b0;

            case (state)

                // =============================================================
                `S_IDLE: begin
                    // synthesis translate_off
                    $display("[VERSION] flat_oram_top v46 t=%0t", $time);
                    // synthesis translate_on
                    client_stall <= 1'b0;
                    if (client_req) begin
                        req_slot_addr <= client_slot_addr;
                        req_op        <= client_op;
                        client_stall  <= 1'b1;
                        err_bucket_overflow <= 1'b0;
                        err_stash_overflow  <= 1'b0;
                        pm_rd_addr    <= client_slot_addr;
                        pm_rd_en      <= 1'b1;
                        prng_en       <= 1'b1;
                        state         <= `S_POS_LOOKUP;
                    end
                end

                // =============================================================
                `S_POS_LOOKUP: begin
                    if (pm_rd_valid) begin
                        req_b       <= pm_rd_bucket;
                        req_b_new   <= prng_bucket;
                        same_bucket <= (prng_bucket == pm_rd_bucket);
                        // synthesis translate_off
                        $display("[POS_LOOKUP] t=%0t slot=0x%08X pm_bucket=%0d prng_bucket=%0d same=%b",
                            $time, req_slot_addr, pm_rd_bucket, prng_bucket,
                            (prng_bucket == pm_rd_bucket));
                        // synthesis translate_on

                        pm_wr_addr   <= req_slot_addr;
                        pm_wr_bucket <= prng_bucket;
                        pm_wr_status <= `ST_VALID;
                        pm_wr_en     <= 1'b1;

                        bm_rd_bucket <= pm_rd_bucket;
                        bm_rd_en     <= 1'b1;

                        axim_cmd_bucket <= pm_rd_bucket;
                        axim_cmd_read   <= 1'b1;

                        state <= `S_DDR_READ;
                    end
                end

                // =============================================================
                `S_DDR_READ: begin
                    if (bm_rd_valid) begin
                        bkt_slot_list  <= bm_rd_slot_list;
                        bkt_fill_count <= bm_rd_fill;
                    end
                    if (axim_read_done) begin
                        state <= `S_SCAN;
                    end
                end

                // =============================================================
                `S_SCAN: begin
                    begin : scan_block
                        integer i;
                        reg hit;
                        reg [POS_W-1:0] hit_pos;
                        hit     = 1'b0;
                        hit_pos = {POS_W{1'b0}};
                        for (i = 0; i < Z; i = i + 1) begin
                            if (i < bkt_fill_count) begin
                                if (get_slot_at(bkt_slot_list, i[POS_W-1:0])
                                    == req_slot_addr) begin
                                    hit     = 1'b1;
                                    hit_pos = i[POS_W-1:0];
                                end
                            end
                        end
                        found_in_bucket <= hit;
                        found_pos       <= hit_pos;
                    end
                    found_in_stash  <= st_cam_slot_hit;
                    found_stash_idx <= st_cam_slot_idx;

                    // synthesis translate_off
                    $display("[SCAN_DBG] t=%0t cam_slot_hit=%b cam_slot_idx=%0d occ=%0d req_slot=0x%08X",
                        $time, st_cam_slot_hit, st_cam_slot_idx,
                        u_stash.occ_r, req_slot_addr);
                    // synthesis translate_on

                    state <= `S_SCAN2;
                end

                // =============================================================
                `S_SCAN2: begin
                    if (found_in_bucket) begin
                        stash_alloc_idx <= st_ins_idx;

                        // Set stash insert metadata EARLY (many cycles before
                        // st_ins_en fires on the last beat of S_EXTRACT)
                        st_ins_slot <= req_slot_addr;
                        st_ins_tgt  <= req_b_new;

                        scr_fsm_rd_addr <= scr_addr(found_pos, 0);
                        scr_fsm_rd_en   <= 1'b1;
                        beat_cnt        <= {BEAT_W{1'b0}};
                        state           <= `S_EXTRACT;
                    end else begin
                        state <= `S_STASH_SEARCH;
                    end
                end

                // =============================================================
                // S_EXTRACT - 1 beat per cycle (after 1-cycle BRAM startup)
                //
                // S_SCAN2 issues rd_en for beat 0. On the first cycle here,
                // rd_valid=0 (BRAM latency). The default clears rd_en. Next
                // cycle rd_valid=1 (beat 0 data). The FSM processes it and
                // re-asserts rd_en for beat 1. From then on, rd_valid stays
                // high every cycle because the FSM always re-asserts rd_en
                // before the default can take effect (last NBA wins).
                // Total: 129 cycles (1 startup + 128 beats).
                // =============================================================
                `S_EXTRACT: begin
                    // synthesis translate_off
                    // Uncomment for debug:
                    // $display("[EXTRACT_DBG] t=%0t beat_cnt=%0d rd_valid=%b rd_data[31:0]=%08X",
                    //     $time, beat_cnt, scr_fsm_rd_valid, scr_fsm_rd_data[31:0]);
                    // synthesis translate_on

                    if (scr_fsm_rd_valid) begin
                        client_rdata       <= scr_fsm_rd_data;
                        client_rdata_beat  <= beat_cnt;
                        client_rdata_valid <= 1'b1;

                        st_dwr_entry <= stash_alloc_idx;
                        st_dwr_beat  <= beat_cnt;
                        st_dwr_data  <= (req_op == 1'b1) ? client_wdata : scr_fsm_rd_data;
                        st_dwr_en    <= 1'b1;

                        if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                            // Last beat - stop reading, move on
                            scr_fsm_rd_en <= 1'b0;

                            if (same_bucket) begin
                                scr_fsm_wr_addr <= scr_addr(found_pos, beat_cnt);
                                scr_fsm_wr_data <= (req_op == 1'b1)
                                                   ? client_wdata
                                                   : scr_fsm_rd_data;
                                scr_fsm_wr_en   <= 1'b1;
                            end else begin
                                // st_ins_slot/st_ins_tgt set in S_SCAN2
                                st_ins_en   <= 1'b1;

                                if (st_ins_full)
                                    err_stash_overflow <= 1'b1;

                                pm_wr_addr   <= req_slot_addr;
                                pm_wr_bucket <= req_b_new;
                                pm_wr_status <= `ST_IN_STASH;
                                pm_wr_en     <= 1'b1;
                            end

                            if (!same_bucket) begin
                                bkt_slot_list  <= set_slot_at(
                                    bkt_slot_list,
                                    found_pos,
                                    bkt_slot_list[(bkt_fill_count-1)*SLOT_W +: SLOT_W]
                                );
                                bkt_fill_count <= bkt_fill_count - 1'b1;
                            end

                            state <= `S_COMPACT;
                        end else begin
                            // Pipeline: issue read for next beat immediately
                            scr_fsm_rd_addr <= scr_addr(found_pos, beat_cnt + 1'b1);
                            scr_fsm_rd_en   <= 1'b1;
                            beat_cnt        <= beat_cnt + 1'b1;
                        end
                    end
                end

                // =============================================================
                `S_STASH_SEARCH: begin
                    if (found_in_stash) begin
                        st_remap_idx <= found_stash_idx;
                        st_remap_bkt <= req_b_new;
                        st_remap_en  <= 1'b1;

                        st_drd_entry <= found_stash_idx;
                        st_drd_beat  <= {BEAT_W{1'b0}};
                        st_drd_en    <= 1'b1;
                        beat_cnt     <= {BEAT_W{1'b0}};
                        state        <= `S_STASH_READ;
                    end else begin
                        // synthesis translate_off
                        $display("[WARN] S_STASH_SEARCH: slot 0x%08X not found in bucket or stash at t=%0t",
                            req_slot_addr, $time);
                        // synthesis translate_on
                        state <= `S_COMPACT;
                    end
                end

                // =============================================================
                `S_STASH_READ: begin
                    if (st_drd_valid) begin
                        client_rdata       <= st_drd_data;
                        client_rdata_beat  <= beat_cnt;
                        client_rdata_valid <= 1'b1;

                        if (req_op == 1'b1) begin
                            st_dwr_entry <= found_stash_idx;
                            st_dwr_beat  <= beat_cnt;
                            st_dwr_data  <= client_wdata;
                            st_dwr_en    <= 1'b1;
                        end

                        if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                            st_drd_en <= 1'b0;
                            state     <= `S_COMPACT;
                        end else begin
                            st_drd_entry <= found_stash_idx;
                            st_drd_beat  <= beat_cnt + 1'b1;
                            st_drd_en    <= 1'b1;
                            beat_cnt     <= beat_cnt + 1'b1;
                        end
                    end
                end

                // =============================================================
                `S_COMPACT: begin
                    evict_slots_remaining <= {{(FILL_W-POS_W-1){1'b0}}, Z[POS_W:0]} - {1'b0, bkt_fill_count};
                    st_cam_bkt_target     <= req_b;
                    st_cam_bkt_en         <= 1'b1;
                    beat_cnt              <= {BEAT_W{1'b0}};
                    state                 <= `S_EVICT;
                end

                // =============================================================
                `S_EVICT: begin
                    st_cam_bkt_en     <= 1'b1;
                    st_cam_bkt_target <= req_b;

                    if (st_cam_bkt_inhibit_busy) begin
                        st_drd_en <= 1'b0;
                    end else if (st_cam_bkt_hit && (evict_slots_remaining > 0)) begin
                        if (st_drd_valid) begin
                            scr_fsm_wr_addr <= scr_addr(bkt_fill_count[POS_W-1:0], beat_cnt);
                            scr_fsm_wr_data <= st_drd_data;
                            scr_fsm_wr_en   <= 1'b1;

                            if (beat_cnt == BEATS[BEAT_W-1:0] - 1) begin
                                st_drd_en <= 1'b0;

                                bkt_slot_list <= set_slot_at(
                                    bkt_slot_list,
                                    bkt_fill_count[POS_W-1:0],
                                    st_cam_bkt_slot_addr[SLOT_W+11:12]
                                );
                                bkt_fill_count <= bkt_fill_count + 1'b1;

                                st_evict_idx <= st_cam_bkt_idx;
                                st_evict_en  <= 1'b1;

                                pm_wr_addr   <= st_cam_bkt_slot_addr;
                                pm_wr_bucket <= req_b;
                                pm_wr_status <= `ST_VALID;
                                pm_wr_en     <= 1'b1;

                                beat_cnt              <= {BEAT_W{1'b0}};
                                evict_slots_remaining <= evict_slots_remaining - 1'b1;
                            end else begin
                                st_drd_entry <= st_cam_bkt_idx;
                                st_drd_beat  <= beat_cnt + 1'b1;
                                st_drd_en    <= 1'b1;
                                beat_cnt     <= beat_cnt + 1'b1;
                            end
                        end else begin
                            // No valid data yet - issue initial read for this candidate
                            st_drd_entry <= st_cam_bkt_idx;
                            st_drd_beat  <= {BEAT_W{1'b0}};
                            st_drd_en    <= 1'b1;
                        end
                    end else begin
                        bm_wr_bucket    <= req_b;
                        bm_wr_slot_list <= bkt_slot_list;
                        bm_wr_fill      <= bkt_fill_count;
                        bm_wr_en        <= 1'b1;

                        axim_cmd_bucket <= req_b;
                        axim_cmd_write  <= 1'b1;
                        state           <= `S_DDR_WRITE;
                    end
                end

                // =============================================================
                `S_DDR_WRITE: begin
                    if (axim_write_done) begin
                        client_done  <= 1'b1;
                        client_stall <= 1'b0;
                        state        <= `S_IDLE;
                    end
                end

                default: state <= `S_IDLE;

            endcase
        end
    end

endmodule