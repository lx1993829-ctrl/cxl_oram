// =============================================================================
// axi_master.v - AXI4 Master for DDR Burst Read / Write
// =============================================================================
// SINGLE FIX from original: wdata_beat_req only advances when wvalid && wready.
// Everything else is identical to the version that passed simulation.

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
    parameter TOTAL_BEATS= `BEATS_PER_BKT,     // 1024
    parameter BEAT_AW    = 10                   // log2(1024)
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
    output reg  [BEAT_AW-1:0]   rdata_beat_addr,

    input  wire [AXI_DW-1:0]   wdata_data,
    input  wire                 wdata_valid,
    output wire                 wdata_ready,
    output reg  [BEAT_AW-1:0]   wdata_beat_req,

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

    // -------------------------------------------------------------------------
    // Bucket base address
    // -------------------------------------------------------------------------
    wire [AXI_AW-1:0] bucket_base = DDR_BASE
        + ({{(AXI_AW-BUCKET_W){1'b0}}, cmd_bucket} << BSHIFT);

    // -------------------------------------------------------------------------
    // AXI burst parameters
    // -------------------------------------------------------------------------
    localparam SUB_BEATS  = `BEATS_PER_BLOCK;      // 128
    localparam SUB_LEN    = SUB_BEATS - 1;          // 127
    localparam NUM_BURSTS = `ORAM_Z;                // 8
    localparam AXI_SIZE   = 3'd5;                   // 32 bytes per beat
    localparam AXI_BURST  = 2'b01;                  // INCR

    // -------------------------------------------------------------------------
    // FSM
    // -------------------------------------------------------------------------
    localparam ST_IDLE    = 3'd0;
    localparam ST_AR      = 3'd1;
    localparam ST_RDATA   = 3'd2;
    localparam ST_AW      = 3'd3;
    localparam ST_WDATA   = 3'd4;
    localparam ST_WRESP   = 3'd5;

    reg [2:0]       state;
    reg [BEAT_AW-1:0] beat_cnt;
    reg [3:0]       burst_num;
    reg [AXI_AW-1:0] burst_base;

    reg rd_done_r, wr_done_r;
    assign cmd_read_done  = rd_done_r;
    assign cmd_write_done = wr_done_r;
    assign cmd_busy       = (state != ST_IDLE);
    assign wdata_ready    = (state == ST_WDATA) && m_axi_wready && wdata_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            beat_cnt        <= {BEAT_AW{1'b0}};
            burst_num       <= 4'd0;
            burst_base      <= {AXI_AW{1'b0}};
            rd_done_r       <= 1'b0;
            wr_done_r       <= 1'b0;
            m_axi_arvalid   <= 1'b0;
            m_axi_rready    <= 1'b0;
            m_axi_awvalid   <= 1'b0;
            m_axi_wvalid    <= 1'b0;
            m_axi_wlast     <= 1'b0;
            m_axi_bready    <= 1'b0;
            rdata_valid     <= 1'b0;
            wdata_beat_req  <= {BEAT_AW{1'b0}};
        end else begin
            rd_done_r   <= 1'b0;
            wr_done_r   <= 1'b0;
            rdata_valid <= 1'b0;

            case (state)
                // ---------------------------------------------------------
                ST_IDLE: begin
                    beat_cnt  <= {BEAT_AW{1'b0}};
                    burst_num <= 4'd0;
                    if (cmd_read) begin
                        burst_base    <= bucket_base;
                        m_axi_arid    <= {AXI_IDW{1'b0}};
                        m_axi_araddr  <= bucket_base;
                        m_axi_arlen   <= SUB_LEN[AXI_LENW-1:0];
                        m_axi_arsize  <= AXI_SIZE;
                        m_axi_arburst <= AXI_BURST;
                        m_axi_arvalid <= 1'b1;
                        state         <= ST_AR;
                    end else if (cmd_write) begin
                        burst_base     <= bucket_base;
                        m_axi_awid     <= {AXI_IDW{1'b0}};
                        m_axi_awaddr   <= bucket_base;
                        m_axi_awlen    <= SUB_LEN[AXI_LENW-1:0];
                        m_axi_awsize   <= AXI_SIZE;
                        m_axi_awburst  <= AXI_BURST;
                        m_axi_awvalid  <= 1'b1;
                        wdata_beat_req <= {BEAT_AW{1'b0}};
                        state          <= ST_AW;
                    end
                end

                // ---------------------------------------------------------
                ST_AR: begin
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        m_axi_rready  <= 1'b1;
                        state         <= ST_RDATA;
                    end
                end

                // ---------------------------------------------------------
                ST_RDATA: begin
                    if (m_axi_rvalid) begin
                        rdata_data      <= m_axi_rdata;
                        rdata_valid     <= 1'b1;
                        rdata_beat_addr <= beat_cnt;
                        beat_cnt        <= beat_cnt + 1'b1;

                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            burst_num    <= burst_num + 1'b1;

                            if (burst_num == NUM_BURSTS - 1) begin
                                rd_done_r <= 1'b1;
                                state     <= ST_IDLE;
                            end else begin
                                burst_base    <= burst_base + (SUB_BEATS * (AXI_DW/8));
                                m_axi_araddr  <= burst_base + (SUB_BEATS * (AXI_DW/8));
                                m_axi_arlen   <= SUB_LEN[AXI_LENW-1:0];
                                m_axi_arvalid <= 1'b1;
                                state         <= ST_AR;
                            end
                        end
                    end
                end

                // ---------------------------------------------------------
                ST_AW: begin
                    // synthesis translate_off
                    $display("[AXI_ST] t=%0t ST_AW awvalid=%b awready=%b burst=%0d",
                        $time, m_axi_awvalid, m_axi_awready, burst_num);
                    // synthesis translate_on
                    if (m_axi_awready) begin
                        m_axi_awvalid  <= 1'b0;
                        // DO NOT assert wvalid here - wdata is stale.
                        // wvalid will be asserted in ST_WDATA after first
                        // valid scratch data arrives.
                        m_axi_wstrb    <= {AXI_SW{1'b1}};
                        state          <= ST_WDATA;
                    end
                end

                // ---------------------------------------------------------
                // FIX: wdata_beat_req only advances when handshake completes.
                // FIX2: wvalid is asserted here (not in ST_AW) to ensure
                // wdata is always valid when wvalid goes high.
                // ---------------------------------------------------------
                ST_WDATA: begin
                    // synthesis translate_off
                    if (m_axi_wvalid && m_axi_wready && (beat_cnt[6:0] == SUB_LEN[6:0]))
                        $display("[AXI_W] t=%0t LAST BEAT burst=%0d beat=%0d ? ST_WRESP",
                            $time, burst_num, beat_cnt);
                    // synthesis translate_on
                    if (!m_axi_wvalid && wdata_valid) begin
                        // Load phase: capture scratch data, assert wvalid
                        m_axi_wdata  <= wdata_data;
                        m_axi_wlast  <= (beat_cnt[6:0] == SUB_LEN[6:0]);
                        m_axi_wvalid <= 1'b1;
                    end
                    else if (m_axi_wvalid && m_axi_wready) begin
                        // Handshake: MIG accepted the beat
                        m_axi_wvalid <= 1'b0;  // deassert for 1 cycle
                        beat_cnt     <= beat_cnt + 1'b1;

                        if (beat_cnt[6:0] == SUB_LEN[6:0]) begin
                            // Last beat done - wlast is already 1 from load phase
                            // Keep it high this cycle so MIG sees wlast=1 with handshake
                            // It gets cleared in ST_WRESP
                            m_axi_bready <= 1'b1;
                            state        <= ST_WRESP;
                            // synthesis translate_off
                            $display("[AXI_W] t=%0t LAST BEAT burst=%0d ? ST_WRESP", $time, burst_num);
                            // synthesis translate_on
                        end else begin
                            // Request next beat from scratch
                            wdata_beat_req <= beat_cnt + 1'b1;
                            // Next cycle: !wvalid && wdata_valid ? reload
                        end
                    end
                    // wvalid=1 && wready=0: hold stable (AXI spec)
                end

                // ---------------------------------------------------------
                ST_WRESP: begin
                    // synthesis translate_off
                    $display("[AXI_ST] t=%0t ST_WRESP bvalid=%b bready=%b burst=%0d",
                        $time, m_axi_bvalid, m_axi_bready, burst_num);
                    // synthesis translate_on
                    m_axi_wvalid <= 1'b0;
                    m_axi_wlast  <= 1'b0;
                    if (m_axi_bvalid) begin
                        m_axi_bready <= 1'b0;
                        burst_num    <= burst_num + 1'b1;

                        if (burst_num == NUM_BURSTS - 1) begin
                            wr_done_r <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            burst_base    <= burst_base + (SUB_BEATS * (AXI_DW/8));
                            m_axi_awaddr  <= burst_base + (SUB_BEATS * (AXI_DW/8));
                            m_axi_awlen   <= SUB_LEN[AXI_LENW-1:0];
                            m_axi_awvalid <= 1'b1;
                            state         <= ST_AW;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule