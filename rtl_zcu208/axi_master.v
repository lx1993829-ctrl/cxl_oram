`timescale 1ns / 1ps
// =============================================================================
// axi_master.v - v4 pipelined: AXI4, 8 bursts x 128 beats
// Pipelined AR/AW grafted from v6, keeping v4's AXI4 burst parameters.
// =============================================================================
`include "oram_params.vh"

module axi_master #(
    parameter AXI_AW     = `AXI_ADDR_W,
    parameter AXI_DW     = `AXI_DATA_W,
    parameter AXI_SW     = `AXI_STRB_W,
    parameter AXI_IDW    = `AXI_ID_W,
    parameter AXI_LENW   = `AXI_LEN_W,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter BSHIFT     = `BUCKET_SHIFT,
    parameter DDR_BASE   = `DDR_BASE,
    parameter TOTAL_BEATS= `BEATS_PER_BKT,
    parameter BEAT_AW    = 10
)(
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 cmd_read,
    input  wire                 cmd_write,
    input  wire [BUCKET_W-1:0]  cmd_bucket,
    output wire                 cmd_read_done,
    output wire                 cmd_write_done,
    output wire                 cmd_busy,

    output reg  [AXI_DW-1:0]   rdata_data,
    output reg                  rdata_valid,
    output reg  [BEAT_AW-1:0]  rdata_beat_addr,

    input  wire [AXI_DW-1:0]   wdata_data,
    input  wire                 wdata_valid,
    output wire                 wdata_ready,
    output reg  [BEAT_AW-1:0]  wdata_beat_req,

    output reg  [AXI_IDW-1:0]   m_axi_arid,
    output reg  [AXI_AW-1:0]    m_axi_araddr,
    output reg  [AXI_LENW-1:0]  m_axi_arlen,
    output reg  [2:0]            m_axi_arsize,
    output reg  [1:0]            m_axi_arburst,
    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,

    input  wire [AXI_IDW-1:0]   m_axi_rid,
    input  wire [AXI_DW-1:0]    m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output reg                   m_axi_rready,

    output reg  [AXI_IDW-1:0]   m_axi_awid,
    output reg  [AXI_AW-1:0]    m_axi_awaddr,
    output reg  [AXI_LENW-1:0]  m_axi_awlen,
    output reg  [2:0]            m_axi_awsize,
    output reg  [1:0]            m_axi_awburst,
    output reg                   m_axi_awvalid,
    input  wire                  m_axi_awready,

    output reg  [AXI_DW-1:0]    m_axi_wdata,
    output reg  [AXI_SW-1:0]    m_axi_wstrb,
    output reg                   m_axi_wlast,
    output reg                   m_axi_wvalid,
    input  wire                  m_axi_wready,

    input  wire [AXI_IDW-1:0]   m_axi_bid,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output reg                   m_axi_bready
);

    wire [AXI_AW-1:0] bucket_base = DDR_BASE
        + ({{(AXI_AW-BUCKET_W){1'b0}}, cmd_bucket} << BSHIFT);

    // =========================================================================
    // AXI4 burst parameters: 128 beats per burst (requires AXI_LEN_W >= 8)
    // =========================================================================
    localparam SUB_BEATS  = `BEATS_PER_BLOCK;              // 128
    localparam SUB_LEN    = SUB_BEATS - 1;                 // 127
    localparam NUM_BURSTS = TOTAL_BEATS / SUB_BEATS;       // 8
    localparam AXI_SIZE   = 3'd5;
    localparam AXI_BURST  = 2'b01;
    localparam BURST_BYTES = SUB_BEATS * (AXI_DW/8);      // 4096 = 0x1000

    // =========================================================================
    // FSM states
    // Opt 2: ST_WDRAIN removed. ST_WRITE returns directly to ST_IDLE as soon
    // as all W-data is accepted; BRESPs drain in the background via the
    // pending_bresps counter (see below) while the upstream FSM is free to
    // begin its next operation.
    // =========================================================================
    localparam ST_IDLE   = 3'd0;
    localparam ST_READ   = 3'd1;
    localparam ST_WRITE  = 3'd2;

    reg [2:0] state;

    // =========================================================================
    // Read path
    // =========================================================================
    reg [3:0]          ar_issued;
    reg [3:0]          r_received;
    reg [BEAT_AW-1:0]  r_beat_cnt;
    reg [AXI_AW-1:0]   ar_next_addr;
    reg                 ar_pending;
    reg                 rd_done_r;

    // =========================================================================
    // Write path
    // =========================================================================
    reg [3:0]          aw_issued;
    reg [3:0]          w_completed;
    reg [BEAT_AW-1:0]  w_beat_cnt;
    reg [AXI_AW-1:0]   aw_next_addr;
    reg                 aw_pending;
    reg                 wr_done_r;

    // =========================================================================
    // Opt 2: Background BRESP tracker (always-on, independent of FSM state).
    // Incremented on AW handshake, decremented on B handshake. m_axi_bready is
    // held high unconditionally (AXI permits this; BRESPs only fire in response
    // to prior AW+W). Allows ST_WRITE -> ST_IDLE transition immediately on
    // last W-beat, with BRESPs draining concurrently with subsequent ORAM work.
    //
    // Width: 5 bits (max 31 outstanding). NUM_BURSTS=8 per ORAM op, and a
    // second op may fire while the first's BRESPs are still draining; worst
    // case is ~16 outstanding momentarily. 5 bits gives 2x headroom and costs
    // nothing. The decrement is guarded against underflow as defense against
    // any spurious BRESP (AXI-spec slaves never send unsolicited BRESPs, but
    // the guard is free).
    // =========================================================================
    reg [4:0] pending_bresps;

    reg [AXI_DW-1:0]  hold_buf;
    reg                hold_valid;
    reg                scr_inflight;  // scratch BRAM read in flight

    // Sub-beat index within current sub-burst (wraps correctly)
    localparam SUB_BEAT_W = $clog2(SUB_BEATS); // 4 for 16-beat, 7 for 128-beat
    wire [SUB_BEAT_W-1:0] w_sub_beat = w_beat_cnt[SUB_BEAT_W-1:0];

    // Combinational: post-W-handshake effective state (for absorb overlap)
    wire w_hs_now   = m_axi_wvalid && m_axi_wready;
    wire w_last_now = (w_sub_beat == SUB_LEN[SUB_BEAT_W-1:0]);
    // Post-handshake effective wvalid:
    //   - Final burst (w_xfer_done): wvalid forced to 0
    //   - Otherwise: hold_valid (hold shifts to wdata, or wvalid cleared)
    wire wv_eff     = w_hs_now ? (w_xfer_done ? 1'b0 : hold_valid) : m_axi_wvalid;
    wire hv_eff     = w_hs_now ? 1'b0 : hold_valid;
    wire [BEAT_AW-1:0] w_beat_eff = w_hs_now ? (w_beat_cnt + 1'b1) : w_beat_cnt;

    // Post-absorb effective state: accounts for STEP 2 filling a slot
    wire scr_arriving  = scr_inflight && wdata_valid;
    // Guard: transfer completes on this cycle - STEP 2/3 must NOT fire
    wire w_xfer_done   = w_hs_now && w_last_now
                         && (w_completed + 1 == NUM_BURSTS);
    wire absorb_to_wv  = scr_arriving && !w_xfer_done && !wv_eff;
    wire absorb_to_hv  = scr_arriving && !w_xfer_done && wv_eff && !hv_eff;
    wire absorbed       = absorb_to_wv || absorb_to_hv;
    wire wv_post_abs   = wv_eff  || absorb_to_wv;
    wire hv_post_abs   = hv_eff  || absorb_to_hv;
    wire room_after_abs = !wv_post_abs || !hv_post_abs;

    assign cmd_read_done  = rd_done_r;
    assign cmd_write_done = wr_done_r;
    assign cmd_busy       = (state != ST_IDLE);
    assign wdata_ready    = (state == ST_WRITE) && m_axi_wready && m_axi_wvalid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            rd_done_r     <= 0; wr_done_r    <= 0;
            ar_issued     <= 0; r_received   <= 0; r_beat_cnt <= 0;
            ar_next_addr  <= 0; ar_pending   <= 0;
            aw_issued     <= 0; w_completed  <= 0;
            w_beat_cnt    <= 0; aw_next_addr <= 0; aw_pending <= 0;
            m_axi_arvalid <= 0; m_axi_rready <= 0;
            m_axi_awvalid <= 0; m_axi_wvalid <= 0;
            m_axi_wlast   <= 0; m_axi_bready <= 1; // Opt 2: always ready
            rdata_valid   <= 0; wdata_beat_req <= 0;
            hold_buf      <= 0; hold_valid   <= 0;
            scr_inflight  <= 0;
        end else begin
            // Opt 2: hold bready high every cycle to drain BRESPs in any state
            m_axi_bready <= 1'b1;
            rd_done_r   <= 0;
            wr_done_r   <= 0;
            rdata_valid <= 0;

            case (state)


            // =================================================================
            ST_IDLE: begin
                hold_valid <= 0;
                if (cmd_read) begin
                    m_axi_arid    <= 0;
                    m_axi_araddr  <= bucket_base;
                    m_axi_arlen   <= SUB_LEN[AXI_LENW-1:0];
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= AXI_BURST;
                    m_axi_arvalid <= 1;
                    // FIX: do NOT assert m_axi_rready until the first AR has
                    // been handshake-accepted. Asserting rready before the
                    // slave has committed creates a window where the very
                    // first rvalid/rdata pulse can be sampled for two
                    // consecutive cycles (slave NBA latency to advance its
                    // beat counter), causing beat-0 data to be captured
                    // twice. Deferring rready by one cycle (gated on the
                    // first AR handshake) eliminates this race without
                    // affecting steady-state throughput.
                    m_axi_rready  <= 0;
                    ar_pending    <= 1;
                    ar_issued     <= 4'd1;
                    r_received    <= 0;
                    r_beat_cnt    <= 0;
                    ar_next_addr  <= bucket_base + BURST_BYTES;
                    state         <= ST_READ;
                end else if (cmd_write) begin
                    m_axi_awid    <= 0;
                    m_axi_awaddr  <= bucket_base;
                    m_axi_awlen   <= SUB_LEN[AXI_LENW-1:0];
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= AXI_BURST;
                    m_axi_awvalid <= 1;
                    m_axi_wstrb   <= {AXI_SW{1'b1}};
                    aw_pending    <= 1;
                    aw_issued     <= 4'd1;
                    w_completed   <= 0;
                    w_beat_cnt    <= 0;
                    aw_next_addr  <= bucket_base + BURST_BYTES;
                    wdata_beat_req <= 0;
                    scr_inflight   <= 0;  // no read in flight yet
                    state         <= ST_WRITE;
                end
            end

            // =================================================================
            // PIPELINED READ: issue AR N+1 as soon as arready fires for AR N,
            // without waiting for rlast.
            //
            // RREADY THROTTLING: rready alternates 1/0/1/0 across cycles,
            // =================================================================
            // PIPELINED READ: issue AR N+1 as soon as arready fires for AR N.
            // rready deferred until first AR handshake accepted.
            // =================================================================
            ST_READ: begin
                if (ar_pending && m_axi_arready) begin
                    ar_pending <= 0;
                    m_axi_rready <= 1;
                    if (ar_issued < NUM_BURSTS) begin
                        m_axi_araddr  <= ar_next_addr;
                        m_axi_arvalid <= 1;
                        ar_pending    <= 1;
                        ar_issued     <= ar_issued + 1;
                        ar_next_addr  <= ar_next_addr + BURST_BYTES;
                    end else begin
                        m_axi_arvalid <= 0;
                    end
                end

                if (m_axi_rvalid && m_axi_rready) begin
                    rdata_data      <= m_axi_rdata;
                    rdata_valid     <= 1;
                    rdata_beat_addr <= r_beat_cnt;
                    r_beat_cnt      <= r_beat_cnt + 1;

                    if (m_axi_rlast) begin
                        r_received <= r_received + 1;
                        if (r_received + 1 == NUM_BURSTS) begin
                            m_axi_rready  <= 0;
                            m_axi_arvalid <= 0;
                            rd_done_r     <= 1;
                            state         <= ST_IDLE;
                        end
                    end
                end
            end



            // =================================================================
            // PIPELINED WRITE: issue AW N+1 as soon as awready fires for AW N,
            // during W-data phase. BRESPs consumed in background.
            // Transition to ST_WDRAIN after last W beat.
            // =================================================================
            ST_WRITE: begin
                // --- AW channel (unchanged) ---
                if (aw_pending && m_axi_awready) begin
                    aw_pending <= 0;
                    if (aw_issued < NUM_BURSTS) begin
                        m_axi_awaddr  <= aw_next_addr;
                        m_axi_awvalid <= 1;
                        aw_pending    <= 1;
                        aw_issued     <= aw_issued + 1;
                        aw_next_addr  <= aw_next_addr + BURST_BYTES;
                    end else begin
                        m_axi_awvalid <= 0;
                    end
                end

                // =========================================================
                // W-channel pipeline (rewritten for 1-cycle BRAM latency).
                //
                // scratch_buf: addr at cycle X → data at cycle X+1.
                // Pipeline stages: scratch → absorb → wdata/hold_buf → W.
                // scr_inflight: 1 = read issued last cycle, data arriving.
                //
                // STEP 1: W handshake (drain wdata to slave).
                // STEP 2: Absorb arriving scratch data into wdata/hold.
                //         Uses EFFECTIVE post-handshake state via comb wires
                //         so absorb and handshake can overlap in same cycle.
                // STEP 3: Issue next scratch read (keep pipeline full).
                // =========================================================

                begin
                    // Uses combinational wires wv_eff, hv_eff, w_beat_eff
                    // defined outside this always block.

                    // --- STEP 1: W handshake ---
                    if (w_hs_now) begin
                        w_beat_cnt <= w_beat_cnt + 1;
                        if (w_last_now) begin
                            w_completed  <= w_completed + 1;
                            if (w_completed + 1 == NUM_BURSTS) begin
                                // Final sub-burst - full cleanup
                                m_axi_wvalid <= 0;
                                m_axi_wlast  <= 0;
                                hold_valid   <= 0;
                                wr_done_r    <= 1;
                                state        <= ST_IDLE;
                            end else if (hold_valid) begin
                                // Non-final boundary, hold has next beat
                                // Shift hold→wdata for first beat of new sub-burst
                                m_axi_wdata  <= hold_buf;
                                m_axi_wlast  <= 1'b0; // beat 0 of new burst
                                // wvalid stays 1
                                hold_valid   <= 0;
                            end else begin
                                // Non-final boundary, no buffered data - stall
                                m_axi_wvalid <= 0;
                                m_axi_wlast  <= 0;
                            end
                        end else if (hold_valid) begin
                            m_axi_wdata  <= hold_buf;
                            m_axi_wlast  <= (w_sub_beat + 1 == SUB_LEN[SUB_BEAT_W-1:0]);
                            hold_valid   <= 0;
                        end else begin
                            m_axi_wvalid <= 0;
                        end
                    end

                    // --- STEP 2: Absorb scratch data ---
                    // Guard: skip if transfer just completed (STEP 1 is
                    // cleaning up wvalid/hold/state - our NBA would override).
                    if (scr_inflight && wdata_valid && !w_xfer_done) begin
                        if (!wv_eff) begin
                            // wdata slot free: load directly
                            m_axi_wdata  <= wdata_data;
                            m_axi_wlast  <= (w_beat_eff[SUB_BEAT_W-1:0] == SUB_LEN[SUB_BEAT_W-1:0]);
                            m_axi_wvalid <= 1;
                        end else if (!hv_eff) begin
                            // hold slot free: buffer
                            hold_buf   <= wdata_data;
                            hold_valid <= 1;
                        end
                        // else: both full → stall (scr_inflight clears below)
                    end

                    // --- STEP 3: Issue next scratch read ---
                    // Room check uses POST-ABSORB state so we don't
                    // overrun when STEP 2 just filled the last free slot.
                    // Guard: skip entirely when transfer just completed.
                    if (!w_xfer_done) begin
                        if (scr_arriving && absorbed && room_after_abs) begin
                            // Absorbed AND room remains → chain pipeline
                            wdata_beat_req <= wdata_beat_req + 1;
                            scr_inflight   <= 1;
                        end else if (scr_arriving && absorbed && !room_after_abs) begin
                            // Absorbed but now full → pause pipeline.
                            // Don't increment beat_req (consumed this beat).
                            scr_inflight <= 0;
                        end else if (scr_arriving && !absorbed) begin
                            // Both slots were full, couldn't absorb → retry.
                            // Keep scr_inflight=1 so STEP 2 retries next cycle
                            // with the same address (BRAM output is stable).
                            scr_inflight <= 1;
                        end else if (!scr_inflight) begin
                            // Cold start or resume after stall
                            if (!wv_eff || !hv_eff) begin
                                wdata_beat_req <= wdata_beat_req + 1;
                                scr_inflight   <= 1;
                            end
                        end
                    end
                end // w_channel
            end

            // =================================================================
            // (Opt 2: ST_WDRAIN removed - BRESPs drain in background)
            // =================================================================

            default: state <= ST_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Opt 2: Always-on BRESP tracker. Runs in parallel with the main FSM and
    // remains active in every state (including ST_IDLE/ST_READ), so BRESPs
    // pending from a completed write are absorbed while the upstream FSM
    // proceeds to subsequent ORAM work.
    //
    // pending_bresps holds the number of writes for which an AW has been
    // issued but the corresponding BRESP has not yet been received. With
    // AWID hardcoded to 0 in this master, AXI requires same-ID BRESPs to
    // return in issue order, so this single counter is sufficient.
    //
    // 5-bit width supports up to 31 outstanding writes - enough headroom
    // for two back-to-back ORAM ops (~16 in flight) plus margin. The
    // decrement is guarded against underflow against spurious BRESPs.
    // =========================================================================
    wire bresp_aw_fire = m_axi_awvalid && m_axi_awready;
    wire bresp_b_fire  = m_axi_bvalid  && m_axi_bready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pending_bresps <= 5'd0;
        end else begin
            case ({bresp_aw_fire, bresp_b_fire})
                2'b10: pending_bresps <= pending_bresps + 5'd1;
                2'b01: if (pending_bresps != 5'd0)
                           pending_bresps <= pending_bresps - 5'd1;
                       // else: spurious B response (AXI-spec violation by slave);
                       // ignore to avoid wrap-around to all-1s.
                default: ;  // 2'b00 or 2'b11: no net change
            endcase
        end
    end

    // ========================================================================
    // BUILD-VERIFY BANNER. Prints once at sim start. If you do NOT see this
    // print, XSim is running a stale compiled version of this file and the
    // rready-deferred fix is NOT in the running build.
    // ========================================================================
endmodule

