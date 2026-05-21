// =============================================================================
// ddr_pattern_init.v - Fill DDR with test pattern before ORAM self-test
// =============================================================================
`include "oram_params.vh"

module ddr_pattern_init #(
    parameter AXI_DW   = `AXI_DATA_W,
    parameter AXI_AW   = `AXI_ADDR_W,
    parameter AXI_SW   = `AXI_STRB_W,
    parameter AXI_IDW  = `AXI_ID_W,
    parameter AXI_LENW = `AXI_LEN_W,
    parameter NUM_BUCKETS = `ORAM_B,  // init ALL buckets (was 3 - caused uninitialized DDR reads)
    parameter BEATS_PER_BURST = 128,
    parameter BURSTS_PER_BKT  = `ORAM_Z
)(
    input  wire                clk,
    input  wire                rst_n,
    input  wire                start,
    output reg                 done,
    output reg                 busy,

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
    output reg                  m_axi_bready,

    output wire [AXI_IDW-1:0]  m_axi_arid,
    output wire [AXI_AW-1:0]   m_axi_araddr,
    output wire [AXI_LENW-1:0] m_axi_arlen,
    output wire [2:0]           m_axi_arsize,
    output wire [1:0]           m_axi_arburst,
    output wire                 m_axi_arvalid,
    input  wire                 m_axi_arready,
    input  wire [AXI_IDW-1:0]  m_axi_rid,
    input  wire [AXI_DW-1:0]   m_axi_rdata,
    input  wire [1:0]           m_axi_rresp,
    input  wire                 m_axi_rlast,
    input  wire                 m_axi_rvalid,
    output wire                 m_axi_rready
);

    assign m_axi_arid    = {AXI_IDW{1'b0}};
    assign m_axi_araddr  = {AXI_AW{1'b0}};
    assign m_axi_arlen   = {AXI_LENW{1'b0}};
    assign m_axi_arsize  = 3'd0;
    assign m_axi_arburst = 2'd0;
    assign m_axi_arvalid = 1'b0;
    assign m_axi_rready  = 1'b0;

    localparam TOTAL_BURSTS = NUM_BUCKETS * BURSTS_PER_BKT;
    localparam BYTES_PER_BEAT = AXI_DW / 8;

    localparam S_IDLE  = 2'd0;
    localparam S_AW    = 2'd1;
    localparam S_WDATA = 2'd2;
    localparam S_BRESP = 2'd3;

    reg [1:0]  state;
    reg [31:0] global_beat;
    reg [7:0]  beat_in_burst;
    reg [15:0] burst_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            done          <= 1'b0;
            busy          <= 1'b0;
            global_beat   <= 32'd0;
            beat_in_burst <= 8'd0;
            burst_cnt     <= 16'd0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_wdata   <= {AXI_DW{1'b0}};
            m_axi_wlast   <= 1'b0;
            m_axi_bready  <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start && !busy) begin
                        busy          <= 1'b1;
                        global_beat   <= 32'd0;
                        beat_in_burst <= 8'd0;
                        burst_cnt     <= 16'd0;
                        m_axi_awid    <= {AXI_IDW{1'b0}};
                        m_axi_awaddr  <= `DDR_BASE;
                        m_axi_awlen   <= BEATS_PER_BURST - 1;
                        m_axi_awsize  <= 3'd5;
                        m_axi_awburst <= 2'b01;
                        m_axi_awvalid <= 1'b1;
                        state         <= S_AW;
                    end
                end

                S_AW: begin
                    if (m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b1;
                        m_axi_wstrb   <= {AXI_SW{1'b1}};
                        m_axi_wdata   <= {8{global_beat}};
                        m_axi_wlast   <= (BEATS_PER_BURST == 1);
                        beat_in_burst <= 8'd0;
                        state         <= S_WDATA;
                    end
                end

                S_WDATA: begin
                    if (m_axi_wready) begin
                        if (beat_in_burst == BEATS_PER_BURST - 1) begin
                            m_axi_wvalid <= 1'b0;
                            m_axi_wlast  <= 1'b0;
                            m_axi_bready <= 1'b1;
                            global_beat  <= global_beat + 1;
                            state        <= S_BRESP;
                        end else begin
                            beat_in_burst <= beat_in_burst + 1;
                            global_beat   <= global_beat + 1;
                            m_axi_wdata   <= {8{global_beat + 1}};
                            m_axi_wlast   <= (beat_in_burst == BEATS_PER_BURST - 2);
                        end
                    end
                end

                S_BRESP: begin
                    if (m_axi_bvalid) begin
                        m_axi_bready <= 1'b0;
                        burst_cnt    <= burst_cnt + 1;
                        if (burst_cnt == TOTAL_BURSTS - 1) begin
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            state <= S_IDLE;
                        end else begin
                            m_axi_awaddr  <= `DDR_BASE + (global_beat * BYTES_PER_BEAT);
                            m_axi_awlen   <= BEATS_PER_BURST - 1;
                            m_axi_awvalid <= 1'b1;
                            state         <= S_AW;
                        end
                    end
                end
            endcase
        end
    end

endmodule