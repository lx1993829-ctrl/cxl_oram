`timescale 1ns / 1ps
//============================================================================
// AES-GCM AXI Integration - 4KB Burst (BRAM, Backpressure-Safe)
//
// Write burst: 3 cycles per beat (read BRAM -> load wdata -> wait wready)
// Read burst: back-to-back (just writing into BRAM, no read latency issue)
// Decrypt feed: 2 wait cycles per new BRAM entry for read latency
//============================================================================
module aes_gcm_axi_integration #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 128,
    parameter NUM_BLOCKS = 255,
    parameter NUM_BEATS  = 128
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  start_encrypt,
    input  wire                  start_decrypt,
    input  wire [127:0]          key,
    input  wire [95:0]           iv,
    input  wire [ADDR_WIDTH-1:0] ddr_addr,
    output wire                  busy,
    output wire                  done,
    output wire                  tag_match,
    output wire [95:0]           active_iv,
    output wire                  iv_updated,
    output wire                  counter_overflow,
    input  wire [DATA_WIDTH-1:0] plaintext_in,
    input  wire                  plaintext_valid,
    input  wire                  plaintext_last,
    output wire                  plaintext_ready,
    output reg  [DATA_WIDTH-1:0] plaintext_out,
    output reg                   plaintext_out_valid,
    output reg                   plaintext_out_last,
    input  wire                  plaintext_out_ready,
    output wire [ADDR_WIDTH-1:0] m_axi_awaddr,
    output wire [7:0]            m_axi_awlen,
    output wire [2:0]            m_axi_awsize,
    output wire [1:0]            m_axi_awburst,
    output wire                  m_axi_awvalid,
    input  wire                  m_axi_awready,
    output wire [255:0]          m_axi_wdata,
    output wire [31:0]           m_axi_wstrb,
    output wire                  m_axi_wlast,
    output wire                  m_axi_wvalid,
    input  wire                  m_axi_wready,
    input  wire [1:0]            m_axi_bresp,
    input  wire                  m_axi_bvalid,
    output wire                  m_axi_bready,
    output wire [ADDR_WIDTH-1:0] m_axi_araddr,
    output wire [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output wire                  m_axi_arvalid,
    input  wire                  m_axi_arready,
    input  wire [255:0]          m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready
);

    //=========================================================================
    // States
    //=========================================================================
    localparam [4:0]
        ST_IDLE             = 5'd0,
        ST_ENC_RECV_EVEN    = 5'd1,
        ST_ENC_WAIT_CT0     = 5'd2,
        ST_ENC_RECV_ODD     = 5'd3,
        ST_ENC_WAIT_CT1     = 5'd4,
        ST_ENC_RECV_LAST    = 5'd5,
        ST_ENC_WAIT_CT_LAST = 5'd6,
        ST_ENC_WAIT_TAG     = 5'd7,
        ST_ENC_BURST_ADDR   = 5'd8,
        ST_ENC_BURST_RD1    = 5'd9,   // BRAM read cycle 1 (addr registered)
        ST_ENC_BURST_RD2    = 5'd10,  // BRAM read cycle 2 (data valid, load wdata)
        ST_ENC_BURST_SEND   = 5'd11,  // Assert wvalid, wait for wready
        ST_ENC_BURST_RESP   = 5'd12,
        ST_DEC_BURST_ADDR   = 5'd13,
        ST_DEC_BURST_DATA   = 5'd14,
        ST_DEC_RD1          = 5'd15,  // BRAM read cycle 1
        ST_DEC_RD2          = 5'd16,  // BRAM read cycle 2 (data valid)
        ST_DEC_FEED_CT      = 5'd17,
        ST_DEC_WAIT_PT      = 5'd18,
        ST_DEC_OUT_PT       = 5'd19,
        ST_DEC_WAIT_TAG     = 5'd20;

    reg [4:0] state;

    //=========================================================================
    // BRAM Buffer: 128 × 256 bits
    //=========================================================================
    (* ram_style = "block" *) reg [255:0] burst_buf [0:NUM_BEATS-1];

    reg  [6:0]   buf_rd_addr;
    reg  [255:0] buf_rd_data;
    reg          buf_wr_en;
    reg  [6:0]   buf_wr_addr;
    reg  [255:0] buf_wr_data;

    always @(posedge clk) begin
        if (buf_wr_en)
            burst_buf[buf_wr_addr] <= buf_wr_data;
        buf_rd_data <= burst_buf[buf_rd_addr];
    end

    //=========================================================================
    // Registers
    //=========================================================================
    reg [7:0]  blk_cnt;
    reg [6:0]  beat_cnt;
    reg [6:0]  dec_beat_cnt;
    reg [6:0]  buf_wr_idx;
    reg [127:0] ct_even;
    reg [127:0] tag_reg;
    reg [127:0] stored_tag;
    reg lo_not_hi;
    reg [ADDR_WIDTH-1:0] base_addr_reg;
    reg axi_awvalid_reg, axi_wvalid_reg, axi_arvalid_reg, axi_rready_reg;
    reg is_encrypt_mode;
    reg gcm_start_pulse;
    reg [127:0] gcm_din_reg;
    reg gcm_dvalid_reg, gcm_dlast_reg;
    reg done_reg, tag_match_reg;
    reg [255:0] wdata_reg;
    reg wdata_last_reg;

    wire [127:0] gcm_dout;
    wire gcm_dout_valid, gcm_dout_last;
    wire [127:0] gcm_tag_out;
    wire gcm_tag_valid, gcm_tag_match_out;
    wire gcm_ready, gcm_busy_wire, gcm_input_ready;
    wire [95:0] gcm_iv_out;
    wire gcm_iv_updated, gcm_counter_overflow;

    //=========================================================================
    // FSM
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            blk_cnt         <= 8'd0;
            beat_cnt        <= 7'd0;
            dec_beat_cnt    <= 7'd0;
            buf_wr_idx      <= 7'd0;
            base_addr_reg   <= {ADDR_WIDTH{1'b0}};
            axi_awvalid_reg <= 1'b0;
            axi_wvalid_reg  <= 1'b0;
            axi_arvalid_reg <= 1'b0;
            axi_rready_reg  <= 1'b0;
            done_reg        <= 1'b0;
            tag_match_reg   <= 1'b0;
            gcm_start_pulse <= 1'b0;
            gcm_dvalid_reg  <= 1'b0;
            gcm_dlast_reg   <= 1'b0;
            gcm_din_reg     <= 128'b0;
            is_encrypt_mode <= 1'b1;
            ct_even         <= 128'b0;
            tag_reg         <= 128'b0;
            stored_tag      <= 128'b0;
            lo_not_hi       <= 1'b1;
            plaintext_out   <= 128'b0;
            plaintext_out_valid <= 1'b0;
            plaintext_out_last  <= 1'b0;
            buf_wr_en       <= 1'b0;
            buf_wr_addr     <= 7'd0;
            buf_wr_data     <= 256'b0;
            buf_rd_addr     <= 7'd0;
            wdata_reg       <= 256'b0;
            wdata_last_reg  <= 1'b0;
        end else begin
            gcm_start_pulse     <= 1'b0;
            gcm_dvalid_reg      <= 1'b0;
            gcm_dlast_reg       <= 1'b0;
            done_reg            <= 1'b0;
            plaintext_out_valid <= 1'b0;
            plaintext_out_last  <= 1'b0;
            buf_wr_en           <= 1'b0;

            case (state)

                ST_IDLE: begin
                    axi_awvalid_reg <= 1'b0;
                    axi_wvalid_reg  <= 1'b0;
                    axi_arvalid_reg <= 1'b0;
                    axi_rready_reg  <= 1'b0;
                    blk_cnt    <= 8'd0;
                    beat_cnt   <= 7'd0;
                    dec_beat_cnt <= 7'd0;
                    buf_wr_idx <= 7'd0;
                    lo_not_hi  <= 1'b1;
                    if (start_encrypt) begin
                        is_encrypt_mode <= 1'b1;
                        base_addr_reg   <= ddr_addr;
                        gcm_start_pulse <= 1'b1;
                        state           <= ST_ENC_RECV_EVEN;
                    end else if (start_decrypt) begin
                        is_encrypt_mode <= 1'b0;
                        base_addr_reg   <= ddr_addr;
                        gcm_start_pulse <= 1'b1;
                        state           <= ST_DEC_BURST_ADDR;
                    end
                end

                //=============================================================
                // ENCRYPT: fill buffer
                //=============================================================
                ST_ENC_RECV_EVEN: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_din_reg <= plaintext_in;
                        gcm_dvalid_reg <= 1'b1;
                        state <= ST_ENC_WAIT_CT0;
                    end
                end

                ST_ENC_WAIT_CT0: begin
                    if (gcm_dout_valid) begin
                        ct_even <= gcm_dout;
                        blk_cnt <= blk_cnt + 1;
                        state <= ST_ENC_RECV_ODD;
                    end
                end

                ST_ENC_RECV_ODD: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_din_reg <= plaintext_in;
                        gcm_dvalid_reg <= 1'b1;
                        state <= ST_ENC_WAIT_CT1;
                    end
                end

                ST_ENC_WAIT_CT1: begin
                    if (gcm_dout_valid) begin
                        buf_wr_en   <= 1'b1;
                        buf_wr_addr <= buf_wr_idx;
                        buf_wr_data <= {gcm_dout, ct_even};
                        buf_wr_idx  <= buf_wr_idx + 1;
                        blk_cnt     <= blk_cnt + 1;
                        if (blk_cnt + 1 == 8'd254)
                            state <= ST_ENC_RECV_LAST;
                        else
                            state <= ST_ENC_RECV_EVEN;
                    end
                end

                ST_ENC_RECV_LAST: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_din_reg <= plaintext_in;
                        gcm_dvalid_reg <= 1'b1;
                        gcm_dlast_reg <= 1'b1;
                        state <= ST_ENC_WAIT_CT_LAST;
                    end
                end

                ST_ENC_WAIT_CT_LAST: begin
                    if (gcm_dout_valid) begin
                        ct_even <= gcm_dout;
                        state <= ST_ENC_WAIT_TAG;
                    end
                end

                ST_ENC_WAIT_TAG: begin
                    if (gcm_tag_valid) begin
                        tag_reg     <= gcm_tag_out;
                        buf_wr_en   <= 1'b1;
                        buf_wr_addr <= 7'd127;
                        buf_wr_data <= {gcm_tag_out, ct_even};
                        state       <= ST_ENC_BURST_ADDR;
                    end
                end

                //=============================================================
                // ENCRYPT: burst write (read-then-send per beat)
                //=============================================================

                // AW handshake
                ST_ENC_BURST_ADDR: begin
                    axi_awvalid_reg <= 1'b1;
                    if (m_axi_awready && axi_awvalid_reg) begin
                        axi_awvalid_reg <= 1'b0;
                        beat_cnt    <= 7'd0;
                        buf_rd_addr <= 7'd0;   // Start reading beat 0
                        state       <= ST_ENC_BURST_RD1;
                    end
                end

                // BRAM read cycle 1: address registered, data not yet valid
                ST_ENC_BURST_RD1: begin
                    state <= ST_ENC_BURST_RD2;
                end

                // BRAM read cycle 2: buf_rd_data now has burst_buf[beat_cnt]
                ST_ENC_BURST_RD2: begin
                    wdata_reg      <= buf_rd_data;
                    wdata_last_reg <= (beat_cnt == 7'd127);
                    axi_wvalid_reg <= 1'b1;
                    state          <= ST_ENC_BURST_SEND;
                end

                // Wait for wready, then advance or finish
                ST_ENC_BURST_SEND: begin
                    if (m_axi_wready && axi_wvalid_reg) begin
                        axi_wvalid_reg <= 1'b0;
                        if (beat_cnt == 7'd127) begin
                            state <= ST_ENC_BURST_RESP;
                        end else begin
                            beat_cnt    <= beat_cnt + 1;
                            buf_rd_addr <= beat_cnt + 7'd1;
                            state       <= ST_ENC_BURST_RD1;
                        end
                    end
                end

                // Write response
                ST_ENC_BURST_RESP: begin
                    if (m_axi_bvalid) begin
                        done_reg      <= 1'b1;
                        tag_match_reg <= 1'b1;
                        state         <= ST_IDLE;
                    end
                end

                //=============================================================
                // DECRYPT: burst read into buffer
                //=============================================================

                ST_DEC_BURST_ADDR: begin
                    axi_arvalid_reg <= 1'b1;
                    if (m_axi_arready && axi_arvalid_reg) begin
                        axi_arvalid_reg <= 1'b0;
                        axi_rready_reg  <= 1'b1;
                        beat_cnt        <= 7'd0;
                        state           <= ST_DEC_BURST_DATA;
                    end
                end

                ST_DEC_BURST_DATA: begin
                    if (m_axi_rvalid && axi_rready_reg) begin
                        buf_wr_en   <= 1'b1;
                        buf_wr_addr <= beat_cnt;
                        buf_wr_data <= m_axi_rdata;
                        if (beat_cnt == 7'd127) begin
                            axi_rready_reg <= 1'b0;
                            stored_tag     <= m_axi_rdata[255:128];
                            dec_beat_cnt   <= 7'd0;
                            lo_not_hi      <= 1'b1;
                            buf_rd_addr    <= 7'd0;
                            state          <= ST_DEC_RD1;
                        end else begin
                            beat_cnt <= beat_cnt + 1;
                        end
                    end
                end

                //=============================================================
                // DECRYPT: feed CT from buffer to GCM
                //=============================================================

                // BRAM read cycle 1
                ST_DEC_RD1: begin
                    state <= ST_DEC_RD2;
                end

                // BRAM read cycle 2: buf_rd_data valid
                ST_DEC_RD2: begin
                    state <= ST_DEC_FEED_CT;
                end

                // Feed lo or hi half to GCM
                ST_DEC_FEED_CT: begin
                    if (gcm_input_ready) begin
                        if (lo_not_hi)
                            gcm_din_reg <= buf_rd_data[127:0];
                        else
                            gcm_din_reg <= buf_rd_data[255:128];
                        gcm_dvalid_reg <= 1'b1;
                        gcm_dlast_reg  <= (dec_beat_cnt == 7'd127) && lo_not_hi;
                        state          <= ST_DEC_WAIT_PT;
                    end
                end

                ST_DEC_WAIT_PT: begin
                    if (gcm_dout_valid) begin
                        plaintext_out       <= gcm_dout;
                        plaintext_out_valid <= 1'b1;
                        plaintext_out_last  <= (dec_beat_cnt == 7'd127) && lo_not_hi;
                        state               <= ST_DEC_OUT_PT;
                    end
                end

                ST_DEC_OUT_PT: begin
                    if (plaintext_out_ready) begin
                        plaintext_out_valid <= 1'b0;
                        if (dec_beat_cnt == 7'd127 && lo_not_hi) begin
                            state <= ST_DEC_WAIT_TAG;
                        end else if (lo_not_hi) begin
                            // Upper half: same buf_rd_data, no re-read needed
                            lo_not_hi <= 1'b0;
                            state     <= ST_DEC_FEED_CT;
                        end else begin
                            // Next entry: need BRAM read
                            dec_beat_cnt <= dec_beat_cnt + 1;
                            lo_not_hi    <= 1'b1;
                            buf_rd_addr  <= dec_beat_cnt + 1;
                            state        <= ST_DEC_RD1;
                        end
                    end
                end

                ST_DEC_WAIT_TAG: begin
                    if (gcm_tag_valid) begin
                        tag_match_reg <= gcm_tag_match_out;
                        done_reg      <= 1'b1;
                        state         <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    //=========================================================================
    // GCM Instance
    //=========================================================================
    aes_gcm_pipelined aes_gcm_inst (
        .clk(clk), .rst_n(rst_n), .enable(1'b1),
        .start(gcm_start_pulse), .encrypt(is_encrypt_mode),
        .key(key), .iv(iv),
        .data_in(gcm_din_reg), .data_valid(gcm_dvalid_reg),
        .data_last(gcm_dlast_reg), .data_bytes_valid(4'd0),
        .data_out(gcm_dout), .data_out_valid(gcm_dout_valid),
        .data_out_last(gcm_dout_last),
        .tag_in(stored_tag), .tag_out(gcm_tag_out),
        .tag_valid(gcm_tag_valid), .tag_match(gcm_tag_match_out),
        .ready(gcm_ready), .busy(gcm_busy_wire),
        .input_ready(gcm_input_ready),
        .iv_out(gcm_iv_out), .iv_updated(gcm_iv_updated),
        .counter_overflow(gcm_counter_overflow)
    );

    //=========================================================================
    // AXI Outputs
    //=========================================================================
    assign m_axi_awaddr  = base_addr_reg;
    assign m_axi_awlen   = 8'd127;
    assign m_axi_awsize  = 3'b101;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awvalid = axi_awvalid_reg;
    assign m_axi_wdata   = wdata_reg;
    assign m_axi_wstrb   = 32'hFFFFFFFF;
    assign m_axi_wlast   = axi_wvalid_reg && wdata_last_reg;
    assign m_axi_wvalid  = axi_wvalid_reg;
    assign m_axi_bready  = 1'b1;
    assign m_axi_araddr  = base_addr_reg;
    assign m_axi_arlen   = 8'd127;
    assign m_axi_arsize  = 3'b101;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = axi_arvalid_reg;
    assign m_axi_rready  = axi_rready_reg;

    //=========================================================================
    // Status
    //=========================================================================
    assign busy             = (state != ST_IDLE);
    assign done             = done_reg;
    assign tag_match        = tag_match_reg;
    assign active_iv        = gcm_iv_out;
    assign iv_updated       = gcm_iv_updated;
    assign counter_overflow = gcm_counter_overflow;
    assign plaintext_ready  = (state == ST_ENC_RECV_EVEN ||
                               state == ST_ENC_RECV_ODD  ||
                               state == ST_ENC_RECV_LAST) && gcm_input_ready;

endmodule