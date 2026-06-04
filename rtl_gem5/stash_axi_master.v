`timescale 1ns / 1ps
// =============================================================================
// stash_axi_master.v - v5: AXI4, burst + single-beat modes
// =============================================================================
// Burst mode: one stash entry = 4KB = 128 beats = 1 AXI4 burst (ARLEN=127).
// Single-beat mode: 1 beat read/write for hash table operations.
// =============================================================================
`include "oram_params.vh"

module stash_axi_master #(
    parameter AXI_AW       = `AXI_ADDR_W,
    parameter AXI_DW       = `AXI_DATA_W,
    parameter AXI_SW       = `AXI_STRB_W,
    parameter AXI_IDW      = `AXI_ID_W,
    parameter AXI_LENW     = `AXI_LEN_W,
    parameter PTR_W        = `STASH_PTR_W,
    parameter BEAT_W       = `BEAT_CNT_W,
    parameter BEATS        = `STASH_BEATS_PER_ENTRY,   // 128
    parameter STASH_BASE   = `STASH_DDR_BASE,
    parameter BLOCK_SHIFT  = `STASH_BLOCK_SHIFT,
    parameter BEAT_AW      = 7                          // log2(128)
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // --- Burst commands (stash data: 128-beat read/write) ---
    input  wire                 cmd_read,
    input  wire                 cmd_write,
    input  wire [PTR_W-1:0]    cmd_entry,
    output wire                 cmd_read_done,
    output wire                 cmd_write_done,
    output wire                 cmd_busy,
    output wire                 cmd_burst_busy,  // high only during burst ops (not single-beat)

    // --- Single-beat commands (hash table: 1-beat read/write) ---
    input  wire                 cmd_single_read,
    input  wire                 cmd_single_write,
    input  wire [AXI_AW-1:0]   cmd_single_addr,
    input  wire [AXI_DW-1:0]   cmd_single_wdata,
    output reg  [AXI_DW-1:0]   cmd_single_rdata,
    output wire                 cmd_single_done,
    output reg                  cmd_single_accepted,  // 1-cyc pulse: single read accepted

    // Line buffer write port (AXI read -> line buffer)
    output reg  [AXI_DW-1:0]   lb_wr_data,
    output reg  [BEAT_AW-1:0]  lb_wr_addr,
    output reg                  lb_wr_en,
    // Latched entry index for line buffer addressing (valid throughout the
    // burst, unlike the live cmd_entry which the FSM changes mid-burst).
    output reg  [PTR_W-1:0]    lb_entry,

    // Line buffer read port (line buffer -> AXI write)
    output reg  [BEAT_AW-1:0]  lb_rd_addr,
    output reg                  lb_rd_en,
    input  wire [AXI_DW-1:0]   lb_rd_data,
    input  wire                 lb_rd_valid,

    // AXI4 Master
    output reg  [AXI_IDW-1:0]  m_axi_arid,
    output reg  [AXI_AW-1:0]   m_axi_araddr,
    output reg  [AXI_LENW-1:0] m_axi_arlen,
    output reg  [2:0]           m_axi_arsize,
    output reg  [1:0]           m_axi_arburst,
    output reg                  m_axi_arvalid,
    input  wire                 m_axi_arready,

    input  wire [AXI_IDW-1:0]  m_axi_rid,
    input  wire [AXI_DW-1:0]   m_axi_rdata,
    input  wire [1:0]           m_axi_rresp,
    input  wire                 m_axi_rlast,
    input  wire                 m_axi_rvalid,
    output reg                  m_axi_rready,

    output reg  [AXI_IDW-1:0]  m_axi_awid,
    output reg  [AXI_AW-1:0]   m_axi_awaddr,
    output reg  [AXI_LENW-1:0] m_axi_awlen,
    output reg  [2:0]           m_axi_awsize,
    output reg  [1:0]           m_axi_awburst,
    output reg                  m_axi_awvalid,
    input  wire                 m_axi_awready,

    output reg  [AXI_DW-1:0]   m_axi_wdata,
    output reg  [AXI_SW-1:0]   m_axi_wstrb,
    output reg                  m_axi_wlast,
    output reg                  m_axi_wvalid,
    input  wire                 m_axi_wready,

    input  wire [AXI_IDW-1:0]  m_axi_bid,
    input  wire [1:0]           m_axi_bresp,
    input  wire                 m_axi_bvalid,
    output reg                  m_axi_bready
);

    wire [AXI_AW-1:0] entry_base = STASH_BASE
        + ({{(AXI_AW-PTR_W){1'b0}}, cmd_entry} << BLOCK_SHIFT);

    // AXI4 burst parameters
    localparam SUB_BEATS   = BEATS;
    localparam SUB_LEN     = SUB_BEATS - 1;
    localparam NUM_BURSTS  = 1;
    localparam AXI_SIZE    = 3'd5;
    localparam AXI_BURST_T = 2'b01;
    localparam BURST_BYTES = SUB_BEATS * (AXI_DW/8);

    // =========================================================================
    localparam ST_IDLE      = 3'd0;
    localparam ST_READ      = 3'd1;
    localparam ST_WRITE     = 3'd2;
    localparam ST_WDRAIN    = 3'd3;
    localparam ST_SNG_READ  = 3'd4;   // single-beat read
    localparam ST_SNG_WRITE = 3'd5;   // single-beat write
    localparam ST_SNG_WDONE = 3'd6;   // single-beat write: wait BRESP

    reg [2:0] fsm_state;

    // Read path
    reg [3:0]          ar_issued;
    reg [3:0]          r_received;
    reg [BEAT_AW-1:0]  r_beat_cnt;
    reg [AXI_AW-1:0]   ar_next_addr;
    reg                 ar_pending;
    reg                 rd_done_r;

    // Write path
    reg [3:0]          aw_issued;
    reg [3:0]          w_completed;
    reg [3:0]          b_received;
    reg [BEAT_AW-1:0]  w_beat_cnt;
    reg [AXI_AW-1:0]   aw_next_addr;
    reg                 aw_pending;
    reg                 wr_done_r;

    // Hold buffer
    reg [AXI_DW-1:0]  hold_buf;
    reg                hold_valid;

    // Single-beat done
    reg                sng_done_r;

    wire [BEAT_AW-1:0] w_sub_beat = w_beat_cnt[BEAT_AW-1:0];

    assign cmd_read_done  = rd_done_r;
    assign cmd_write_done = wr_done_r;
    assign cmd_single_done = sng_done_r;
    assign cmd_busy       = (fsm_state != ST_IDLE);
    assign cmd_burst_busy = (fsm_state == ST_READ || fsm_state == ST_WRITE || fsm_state == ST_WDRAIN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_state     <= ST_IDLE;
            rd_done_r     <= 0; wr_done_r     <= 0; sng_done_r <= 0;
            ar_issued     <= 0; r_received    <= 0; r_beat_cnt <= 0;
            ar_next_addr  <= 0; ar_pending    <= 0;
            aw_issued     <= 0; w_completed   <= 0; b_received <= 0;
            w_beat_cnt    <= 0; aw_next_addr  <= 0; aw_pending <= 0;
            m_axi_arvalid <= 0; m_axi_rready  <= 0;
            m_axi_awvalid <= 0; m_axi_wvalid  <= 0;
            m_axi_wlast   <= 0; m_axi_bready  <= 0;
            lb_wr_en      <= 0; lb_rd_en      <= 0;
            lb_entry      <= 0;
            hold_buf      <= 0; hold_valid    <= 0;
            cmd_single_rdata <= 0;
            cmd_single_accepted <= 0;
        end else begin
            rd_done_r <= 0;
            wr_done_r <= 0;
            sng_done_r <= 0;
            cmd_single_accepted <= 0;
            lb_wr_en  <= 0;
            lb_rd_en  <= 0;
            `ifndef SYNTHESIS
            // Bug-2 probe: a single-read request that arrives while the FSM is
            // NOT in ST_IDLE cannot issue an AR this cycle. If the HT FSM then
            // observes sng_done (a leftover/spurious pulse) it will consume stale
            // cmd_single_rdata. Flag the request-while-busy condition and what
            // cmd_single_rdata currently holds (the stale value that would be read).
            if (cmd_single_read && fsm_state != ST_IDLE)
                $display("[SAM_SNG_RD_BUSY] t=%0t req addr=0x%010h but fsm_state=%0d (NOT IDLE) -> no AR; stale cmd_single_rdata=0x%016h",
                         $time, cmd_single_addr, fsm_state, cmd_single_rdata);
            `endif

            case (fsm_state)

            // =================================================================
            ST_IDLE: begin
                hold_valid <= 0;
                if (cmd_read) begin
                    // Burst read: 128-beat stash entry
                    lb_entry      <= cmd_entry;   // latch for line buffer addressing
                    m_axi_arid    <= 0;
                    m_axi_araddr  <= entry_base;
                    m_axi_arlen   <= SUB_LEN[AXI_LENW-1:0];
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= AXI_BURST_T;
                    m_axi_arvalid <= 1;
                    m_axi_rready  <= 1;
                    ar_pending    <= 1;
                    ar_issued     <= 4'd1;
                    r_received    <= 0;
                    r_beat_cnt    <= 0;
                    ar_next_addr  <= entry_base + BURST_BYTES;
                    fsm_state     <= ST_READ;
                end else if (cmd_write) begin
                    // Burst write: 128-beat stash entry
                    lb_entry      <= cmd_entry;   // latch for line buffer addressing
                    m_axi_awid    <= 0;
                    m_axi_awaddr  <= entry_base;
                    m_axi_awlen   <= SUB_LEN[AXI_LENW-1:0];
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= AXI_BURST_T;
                    m_axi_awvalid <= 1;
                    m_axi_bready  <= 1;
                    m_axi_wstrb   <= {AXI_SW{1'b1}};
                    aw_pending    <= 1;
                    aw_issued     <= 4'd1;
                    w_completed   <= 0;
                    b_received    <= 0;
                    w_beat_cnt    <= 0;
                    aw_next_addr  <= entry_base + BURST_BYTES;
                    lb_rd_addr    <= 0;
                    lb_rd_en      <= 1;
                    fsm_state     <= ST_WRITE;
                end else if (cmd_single_read) begin
                    // Single-beat read: hash table lookup
                    m_axi_arid    <= 0;
                    m_axi_araddr  <= cmd_single_addr;
                    m_axi_arlen   <= 0;   // 1 beat
                    m_axi_arsize  <= AXI_SIZE;
                    m_axi_arburst <= 2'b01;
                    m_axi_arvalid <= 1;
                    m_axi_rready  <= 1;
                    fsm_state     <= ST_SNG_READ;
                    cmd_single_accepted <= 1;
                    `ifndef SYNTHESIS
                    $display("[SAM_SNG_RD_ISSUE] t=%0t addr=0x%010h (AR issued, ST_IDLE->ST_SNG_READ)",
                             $time, cmd_single_addr);
                    `endif
                end else if (cmd_single_write) begin
                    // Single-beat write: hash table update
                    m_axi_awid    <= 0;
                    m_axi_awaddr  <= cmd_single_addr;
                    m_axi_awlen   <= 0;   // 1 beat
                    m_axi_awsize  <= AXI_SIZE;
                    m_axi_awburst <= 2'b01;
                    m_axi_awvalid <= 1;
                    m_axi_wdata   <= cmd_single_wdata;
                    m_axi_wstrb   <= {AXI_SW{1'b1}};
                    m_axi_wlast   <= 1;
                    m_axi_wvalid  <= 1;
                    m_axi_bready  <= 1;
                    fsm_state     <= ST_SNG_WRITE;
                end
            end

            // =================================================================
            // BURST READ (unchanged from v4)
            // =================================================================
            ST_READ: begin
                if (ar_pending && m_axi_arready) begin
                    ar_pending <= 0;
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
                    lb_wr_data <= m_axi_rdata;
                    lb_wr_addr <= r_beat_cnt;
                    lb_wr_en   <= 1;
                    r_beat_cnt <= r_beat_cnt + 1;

                    if (m_axi_rlast) begin
                        r_received <= r_received + 1;
                        if (r_received + 1 == NUM_BURSTS) begin
                            m_axi_rready  <= 0;
                            m_axi_arvalid <= 0;
                            rd_done_r     <= 1;
                            fsm_state     <= ST_IDLE;
                        end
                    end
                end
            end

            // =================================================================
            // BURST WRITE (unchanged from v4)
            // =================================================================
            ST_WRITE: begin
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

                if (m_axi_bvalid && m_axi_bready)
                    b_received <= b_received + 1;

                if (!m_axi_wvalid) begin
                    if (hold_valid) begin
                        m_axi_wdata  <= hold_buf;
                        m_axi_wlast  <= (w_sub_beat == SUB_LEN[BEAT_AW-1:0]);
                        m_axi_wvalid <= 1;
                        hold_valid   <= 0;
                        if (w_sub_beat + 1 <= SUB_LEN[BEAT_AW-1:0]) begin
                            lb_rd_addr <= w_beat_cnt + 1;
                            lb_rd_en   <= 1;
                        end
                    end else if (lb_rd_valid) begin
                        m_axi_wdata  <= lb_rd_data;
                        m_axi_wlast  <= (w_sub_beat == SUB_LEN[BEAT_AW-1:0]);
                        m_axi_wvalid <= 1;
                        if (w_sub_beat + 1 <= SUB_LEN[BEAT_AW-1:0]) begin
                            lb_rd_addr <= w_beat_cnt + 1;
                            lb_rd_en   <= 1;
                        end
                    end
                end else if (m_axi_wvalid && m_axi_wready) begin
                    w_beat_cnt <= w_beat_cnt + 1;

                    if (w_sub_beat == SUB_LEN[BEAT_AW-1:0]) begin
                        w_completed  <= w_completed + 1;
                        m_axi_wvalid <= 0;
                        hold_valid   <= 0;
                        lb_rd_addr   <= w_beat_cnt + 1;
                        lb_rd_en     <= 1;

                        if (w_completed + 1 == NUM_BURSTS)
                            fsm_state <= ST_WDRAIN;
                    end else if (hold_valid) begin
                        m_axi_wdata  <= hold_buf;
                        m_axi_wlast  <= (w_sub_beat + 1 == SUB_LEN[BEAT_AW-1:0]);
                        hold_valid   <= 0;
                        if (w_sub_beat + 2 < SUB_LEN[BEAT_AW-1:0]) begin
                            lb_rd_addr <= w_beat_cnt + 3;
                            lb_rd_en   <= 1;
                        end
                    end else if (lb_rd_valid) begin
                        m_axi_wdata  <= lb_rd_data;
                        m_axi_wlast  <= (w_sub_beat + 1 == SUB_LEN[BEAT_AW-1:0]);
                        if (w_sub_beat + 2 < SUB_LEN[BEAT_AW-1:0]) begin
                            lb_rd_addr <= w_beat_cnt + 3;
                            lb_rd_en   <= 1;
                        end
                    end else begin
                        m_axi_wvalid <= 0;
                    end
                end else if (m_axi_wvalid && !m_axi_wready) begin
                    if (lb_rd_valid && !hold_valid) begin
                        hold_buf   <= lb_rd_data;
                        hold_valid <= 1;
                    end
                end
            end

            // =================================================================
            // WDRAIN: wait for remaining BRESPs (burst write)
            // =================================================================
            ST_WDRAIN: begin
                m_axi_wvalid  <= 0;
                m_axi_awvalid <= 0;
                m_axi_bready  <= 1;

                if (m_axi_bvalid && m_axi_bready)
                    b_received <= b_received + 1;

                if (b_received >= NUM_BURSTS ||
                    (m_axi_bvalid && m_axi_bready && (b_received + 1 >= NUM_BURSTS))) begin
                    m_axi_bready <= 0;
                    wr_done_r    <= 1;
                    fsm_state    <= ST_IDLE;
                end
            end

            // =================================================================
            // SINGLE-BEAT READ: issue AR(len=0), wait for 1 R beat
            // =================================================================
            ST_SNG_READ: begin
                // AR handshake
                if (m_axi_arvalid && m_axi_arready)
                    m_axi_arvalid <= 0;

                // R handshake: capture data
                if (m_axi_rvalid && m_axi_rready) begin
                    cmd_single_rdata <= m_axi_rdata;
                    m_axi_rready     <= 0;
                    sng_done_r       <= 1;
                    fsm_state        <= ST_IDLE;
                    `ifndef SYNTHESIS
                    $display("[SAM_SNG_RD_DONE] t=%0t araddr=0x%010h rdata=0x%016h (R captured, sng_done<=1)",
                             $time, m_axi_araddr, m_axi_rdata);
                    `endif
                end
            end

            // =================================================================
            // SINGLE-BEAT WRITE: issue AW+W simultaneously, wait for handshakes
            // =================================================================
            ST_SNG_WRITE: begin
                // AW handshake
                if (m_axi_awvalid && m_axi_awready)
                    m_axi_awvalid <= 0;

                // W handshake
                if (m_axi_wvalid && m_axi_wready)
                    m_axi_wvalid <= 0;

                // Both AW and W accepted: wait for BRESP
                if (!m_axi_awvalid && !m_axi_wvalid)
                    fsm_state <= ST_SNG_WDONE;
                // Also transition if we're already past both handshakes this cycle
                if ((m_axi_awvalid && m_axi_awready) && !m_axi_wvalid)
                    fsm_state <= ST_SNG_WDONE;
                if (!m_axi_awvalid && (m_axi_wvalid && m_axi_wready))
                    fsm_state <= ST_SNG_WDONE;
                if ((m_axi_awvalid && m_axi_awready) && (m_axi_wvalid && m_axi_wready))
                    fsm_state <= ST_SNG_WDONE;
            end

            // =================================================================
            // SINGLE-BEAT WRITE DONE: wait for BRESP
            // =================================================================
            ST_SNG_WDONE: begin
                m_axi_awvalid <= 0;
                m_axi_wvalid  <= 0;
                m_axi_bready  <= 1;

                if (m_axi_bvalid && m_axi_bready) begin
                    m_axi_bready <= 0;
                    sng_done_r   <= 1;
                    fsm_state    <= ST_IDLE;
                end
            end

            default: fsm_state <= ST_IDLE;
            endcase
        end
    end

endmodule