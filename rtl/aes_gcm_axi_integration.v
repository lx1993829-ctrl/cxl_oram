`timescale 1ns / 1ps

`timescale 1ns / 1ps
//============================================================================
// AES-GCM AXI Integration - 4KB Streaming Version (Optimised FSM)
//
// ENCRYPT: For every pair of PT blocks received from client:
//   - Encrypt block N   ? CT[N]
//   - Encrypt block N+1 ? CT[N+1]
//   - AXI single write {CT[N+1], CT[N]} ? ADDR + (N/2)*0x20
//   Block 254 (last, unpaired):
//   - Encrypt ? CT[254], wait for TAG
//   - AXI single write {TAG, CT[254]} ? ADDR + 127*0x20
//     (reuses ST_ENC_WR_ADDR/DATA/RESP - tag_written flag routes to IDLE)
//
// DECRYPT: For each AXI read beat at ADDR + beat*0x20:
//   - Beats 0..126: unpack {CT[2b+1], CT[2b]}, decrypt both, output to client
//   - Beat  127:    unpack {TAG, CT[254]},      decrypt CT[254], verify TAG
//     (ST_DEC_FEED_CT/WAIT_PT/OUT_PT shared for both lo and hi blocks
//      via lo_not_hi flag)
//
// State count: 17 (down from 24)
//   Removed: ST_ENC_WR_ADDR_LAST, ST_ENC_WR_DATA_LAST, ST_ENC_WR_RESP_LAST
//            (merged into existing WR_ADDR/DATA/RESP via tag_written flag)
//            ST_DONE
//            (merged into terminal states directly)
//            ST_DEC_FEED_LO, ST_DEC_WAIT_PT_LO, ST_DEC_OUT_LO,
//            ST_DEC_FEED_HI, ST_DEC_WAIT_PT_HI, ST_DEC_OUT_HI
//            (merged into ST_DEC_FEED_CT/WAIT_PT/OUT_PT via lo_not_hi flag)
//============================================================================
module aes_gcm_axi_integration #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 128,
    parameter NUM_BLOCKS = 255,
    parameter NUM_BEATS  = 128
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Control
    input  wire                  start_encrypt,
    input  wire                  start_decrypt,
    input  wire [127:0]          key,
    input  wire [95:0]           iv,
    input  wire [ADDR_WIDTH-1:0] ddr_addr,

    // Status
    output wire                  busy,
    output wire                  done,
    output wire                  tag_match,

    // Plaintext INPUT stream (encrypt)
    input  wire [DATA_WIDTH-1:0] plaintext_in,
    input  wire                  plaintext_valid,
    input  wire                  plaintext_last,
    output wire                  plaintext_ready,

    // Plaintext OUTPUT stream (decrypt)
    output reg  [DATA_WIDTH-1:0] plaintext_out,
    output reg                   plaintext_out_valid,
    output reg                   plaintext_out_last,
    input  wire                  plaintext_out_ready,

    // AXI Master (256-bit)
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
    // State Encoding - 17 states
    //=========================================================================
    localparam [4:0]
        ST_IDLE             = 5'd0,

        // --- Encrypt ---
        ST_ENC_RECV_EVEN    = 5'd1,   // wait for even PT block from client
        ST_ENC_WAIT_CT0     = 5'd2,   // wait for GCM to output CT[even]
        ST_ENC_RECV_ODD     = 5'd3,   // wait for odd PT block from client
        ST_ENC_WAIT_CT1     = 5'd4,   // wait for GCM to output CT[odd]
        ST_ENC_WR_ADDR      = 5'd5,   // AXI write address (pairs AND last beat)
        ST_ENC_WR_DATA      = 5'd6,   // AXI write data
        ST_ENC_WR_RESP      = 5'd7,   // AXI write response - routes to next state
        ST_ENC_RECV_LAST    = 5'd8,   // wait for block 254 from client
        ST_ENC_WAIT_CT_LAST = 5'd9,   // wait for CT[254]
        ST_ENC_WAIT_TAG     = 5'd10,  // wait for GCM authentication tag

        // --- Decrypt ---
        ST_DEC_RD_ADDR      = 5'd11,  // AXI read address
        ST_DEC_RD_DATA      = 5'd12,  // AXI read data - unpacks lo and hi
        ST_DEC_FEED_CT      = 5'd13,  // feed CT to GCM (lo OR hi via lo_not_hi)
        ST_DEC_WAIT_PT      = 5'd14,  // wait for GCM plaintext output
        ST_DEC_OUT_PT       = 5'd15,  // wait for client to consume PT block
        ST_DEC_WAIT_TAG     = 5'd16;  // wait for GCM tag verification

    reg [4:0] state;

    //=========================================================================
    // Counters
    //=========================================================================
    reg [7:0] blk_cnt;    // 0..254  encrypt block index
    reg [6:0] beat_cnt;   // 0..127  decrypt DDR beat index

    //=========================================================================
    // Staging Registers
    //=========================================================================
    reg [127:0] ct_even;      // CT of even block, waiting to be paired   (encrypt)
    reg [127:0] ct_lo;        // lower CT block from current DDR beat      (decrypt)
    reg [127:0] ct_hi;        // upper CT block from current DDR beat      (decrypt)
    reg [127:0] tag_reg;      // computed GCM tag                          (encrypt)
    reg [127:0] stored_tag;   // tag read from DDR                         (decrypt)

    //=========================================================================
    // Control Flags
    //=========================================================================
    // tag_written: set when WAIT_TAG packs {TAG,CT[254]} into wdata_reg.
    // ST_ENC_WR_RESP checks this to go to ST_IDLE instead of ST_ENC_RECV_EVEN.
    reg tag_written;

    // lo_not_hi: 1 = currently processing lower CT block of a DDR beat,
    //            0 = currently processing upper CT block.
    // Used by the merged ST_DEC_FEED_CT / WAIT_PT / OUT_PT states.
    reg lo_not_hi;

    //=========================================================================
    // AXI Registers
    //=========================================================================
    reg [ADDR_WIDTH-1:0] cur_wr_addr;
    reg [ADDR_WIDTH-1:0] cur_rd_addr;
    reg         axi_awvalid_reg;
    reg         axi_wvalid_reg;
    reg         axi_arvalid_reg;
    reg         axi_rready_reg;
    reg [255:0] wdata_reg;

    //=========================================================================
    // GCM Interface Registers
    //=========================================================================
    reg         is_encrypt_mode;
    reg         gcm_start_pulse;
    reg [127:0] gcm_din_reg;
    reg         gcm_dvalid_reg;
    reg         gcm_dlast_reg;

    wire [127:0] gcm_dout;
    wire         gcm_dout_valid;
    wire         gcm_dout_last;
    wire [127:0] gcm_tag_out;
    wire         gcm_tag_valid;
    wire         gcm_tag_match_out;
    wire         gcm_ready;
    wire         gcm_busy_wire;
    wire         gcm_input_ready;

    //=========================================================================
    // Output Registers
    //=========================================================================
    reg done_reg;
    reg tag_match_reg;

    //=========================================================================
    // FSM
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            blk_cnt             <= 8'd0;
            beat_cnt            <= 7'd0;
            cur_wr_addr         <= {ADDR_WIDTH{1'b0}};
            cur_rd_addr         <= {ADDR_WIDTH{1'b0}};
            axi_awvalid_reg     <= 1'b0;
            axi_wvalid_reg      <= 1'b0;
            axi_arvalid_reg     <= 1'b0;
            axi_rready_reg      <= 1'b0;
            wdata_reg           <= 256'b0;
            done_reg            <= 1'b0;
            tag_match_reg       <= 1'b0;
            gcm_start_pulse     <= 1'b0;
            gcm_dvalid_reg      <= 1'b0;
            gcm_dlast_reg       <= 1'b0;
            gcm_din_reg         <= 128'b0;
            is_encrypt_mode     <= 1'b1;
            ct_even             <= 128'b0;
            ct_lo               <= 128'b0;
            ct_hi               <= 128'b0;
            tag_reg             <= 128'b0;
            stored_tag          <= 128'b0;
            tag_written         <= 1'b0;
            lo_not_hi           <= 1'b1;
            plaintext_out       <= 128'b0;
            plaintext_out_valid <= 1'b0;
            plaintext_out_last  <= 1'b0;
        end else begin
            //------------------------------------------------------------------
            // Default: clear all single-cycle pulse signals
            //------------------------------------------------------------------
            gcm_start_pulse     <= 1'b0;
            gcm_dvalid_reg      <= 1'b0;
            gcm_dlast_reg       <= 1'b0;
            done_reg            <= 1'b0;
            plaintext_out_valid <= 1'b0;
            plaintext_out_last  <= 1'b0;

            case (state)

                //=============================================================
                ST_IDLE: begin
                    axi_awvalid_reg <= 1'b0;
                    axi_wvalid_reg  <= 1'b0;
                    axi_arvalid_reg <= 1'b0;
                    axi_rready_reg  <= 1'b0;
                    blk_cnt         <= 8'd0;
                    beat_cnt        <= 7'd0;
                    tag_written     <= 1'b0;
                    lo_not_hi       <= 1'b1;

                    if (start_encrypt) begin
                        is_encrypt_mode <= 1'b1;
                        cur_wr_addr     <= ddr_addr;
                        gcm_start_pulse <= 1'b1;
                        state           <= ST_ENC_RECV_EVEN;
                        $display("[%0t] [4KB-ENC] START addr=0x%08h",
                                 $time, ddr_addr);
                    end
                    else if (start_decrypt) begin
                        is_encrypt_mode <= 1'b0;
                        cur_rd_addr     <= ddr_addr;
                        gcm_start_pulse <= 1'b1;
                        state           <= ST_DEC_RD_ADDR;
                        $display("[%0t] [4KB-DEC] START addr=0x%08h",
                                 $time, ddr_addr);
                    end
                end

                //=============================================================
                // ENCRYPTION PATH
                //=============================================================

                //--------------------------------------------------------------
                // Receive even block (0, 2, 4, ... 252)
                //--------------------------------------------------------------
                ST_ENC_RECV_EVEN: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_din_reg    <= plaintext_in;
                        gcm_dvalid_reg <= 1'b1;
                        gcm_dlast_reg  <= 1'b0;
                        state          <= ST_ENC_WAIT_CT0;
                        $display("[%0t] [4KB-ENC] Feed EVEN blk=%0d",
                                 $time, blk_cnt);
                    end
                end

                //--------------------------------------------------------------
                // Wait for CT[even]
                //--------------------------------------------------------------
                ST_ENC_WAIT_CT0: begin
                    if (gcm_dout_valid) begin
                        ct_even <= gcm_dout;
                        blk_cnt <= blk_cnt + 1'b1;   // now points to odd block
                        state   <= ST_ENC_RECV_ODD;
                        $display("[%0t] [4KB-ENC] Got CT[%0d]=0x%032h",
                                 $time, blk_cnt, gcm_dout);
                    end
                end

                //--------------------------------------------------------------
                // Receive odd block (1, 3, 5, ... 253)
                //--------------------------------------------------------------
                ST_ENC_RECV_ODD: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_din_reg    <= plaintext_in;
                        gcm_dvalid_reg <= 1'b1;
                        gcm_dlast_reg  <= 1'b0;
                        state          <= ST_ENC_WAIT_CT1;
                        $display("[%0t] [4KB-ENC] Feed ODD blk=%0d",
                                 $time, blk_cnt);
                    end
                end

                //--------------------------------------------------------------
                // Wait for CT[odd] - pack pair and go write
                //--------------------------------------------------------------
                ST_ENC_WAIT_CT1: begin
                    if (gcm_dout_valid) begin
                        wdata_reg <= {gcm_dout, ct_even};   // {CT[odd], CT[even]}
                        blk_cnt   <= blk_cnt + 1'b1;        // advance to next even
                        state     <= ST_ENC_WR_ADDR;
                        $display("[%0t] [4KB-ENC] Got CT[%0d]=0x%032h, writing pair",
                                 $time, blk_cnt, gcm_dout);
                    end
                end

                //--------------------------------------------------------------
                // AXI write address - shared by pairs and last beat
                //--------------------------------------------------------------
                ST_ENC_WR_ADDR: begin
                    axi_awvalid_reg <= 1'b1;
                    if (m_axi_awready && axi_awvalid_reg) begin
                        axi_awvalid_reg <= 1'b0;
                        axi_wvalid_reg  <= 1'b1;
                        state           <= ST_ENC_WR_DATA;
                    end
                end

                //--------------------------------------------------------------
                // AXI write data - shared by pairs and last beat
                //--------------------------------------------------------------
                ST_ENC_WR_DATA: begin
                    if (m_axi_wready && axi_wvalid_reg) begin
                        axi_wvalid_reg <= 1'b0;
                        state          <= ST_ENC_WR_RESP;
                        $display("[%0t] [4KB-ENC] Wrote to 0x%08h hi=0x%032h lo=0x%032h",
                                 $time, cur_wr_addr,
                                 wdata_reg[255:128], wdata_reg[127:0]);
                    end
                end

                //--------------------------------------------------------------
                // AXI write response - shared routing:
                //   tag_written=1 ? last beat done ? go to IDLE
                //   blk_cnt==254  ? pairs done, last block next ? RECV_LAST
                //   else          ? continue with next pair ? RECV_EVEN
                //--------------------------------------------------------------
                ST_ENC_WR_RESP: begin
                    if (m_axi_bvalid) begin
                        cur_wr_addr <= cur_wr_addr + 32'h00000020;

                        if (tag_written) begin
                            // Last beat {TAG, CT[254]} written - done
                            done_reg      <= 1'b1;
                            tag_match_reg <= 1'b1;
                            state         <= ST_IDLE;
                            $display("[%0t] [4KB-ENC] All 128 beats written. DONE.",
                                     $time);
                        end else if (blk_cnt == 8'd254) begin
                            // All pairs written, last unpaired block next
                            state <= ST_ENC_RECV_LAST;
                        end else begin
                            // More pairs to process
                            state <= ST_ENC_RECV_EVEN;
                        end
                    end
                end

                //--------------------------------------------------------------
                // Receive last (unpaired) block - block index 254
                //--------------------------------------------------------------
                ST_ENC_RECV_LAST: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_din_reg    <= plaintext_in;
                        gcm_dvalid_reg <= 1'b1;
                        gcm_dlast_reg  <= 1'b1;   // signals end of plaintext to GCM
                        state          <= ST_ENC_WAIT_CT_LAST;
                        $display("[%0t] [4KB-ENC] Feed LAST blk=254", $time);
                    end
                end

                //--------------------------------------------------------------
                // Wait for CT[254]
                //--------------------------------------------------------------
                ST_ENC_WAIT_CT_LAST: begin
                    if (gcm_dout_valid) begin
                        ct_even <= gcm_dout;    // reuse ct_even to hold CT[254]
                        state   <= ST_ENC_WAIT_TAG;
                        $display("[%0t] [4KB-ENC] Got CT[254]=0x%032h",
                                 $time, gcm_dout);
                    end
                end

                //--------------------------------------------------------------
                // Wait for GCM authentication tag - pack {TAG, CT[254]}
                // Set tag_written so ST_ENC_WR_RESP knows to go to IDLE
                //--------------------------------------------------------------
                ST_ENC_WAIT_TAG: begin
                    if (gcm_tag_valid) begin
                        tag_reg     <= gcm_tag_out;
                        wdata_reg   <= {gcm_tag_out, ct_even};   // {TAG, CT[254]}
                        tag_written <= 1'b1;
                        state       <= ST_ENC_WR_ADDR;   // reuse write states
                        $display("[%0t] [4KB-ENC] Got TAG=0x%032h",
                                 $time, gcm_tag_out);
                    end
                end

                //=============================================================
                // DECRYPTION PATH
                //=============================================================

                //--------------------------------------------------------------
                // Issue AXI read address for current beat
                //--------------------------------------------------------------
                ST_DEC_RD_ADDR: begin
                    axi_arvalid_reg <= 1'b1;
                    if (m_axi_arready && axi_arvalid_reg) begin
                        axi_arvalid_reg <= 1'b0;
                        axi_rready_reg  <= 1'b1;
                        state           <= ST_DEC_RD_DATA;
                    end
                end

                //--------------------------------------------------------------
                // Receive DDR beat - unpack lo and hi CT blocks
                // For beat 127: hi is TAG not CT ? store in stored_tag
                //--------------------------------------------------------------
                ST_DEC_RD_DATA: begin
                    if (m_axi_rvalid && axi_rready_reg) begin
                        axi_rready_reg <= 1'b0;
                        ct_lo          <= m_axi_rdata[127:0];

                        if (beat_cnt == 7'd127)
                            stored_tag <= m_axi_rdata[255:128];
                        else
                            ct_hi      <= m_axi_rdata[255:128];

                        lo_not_hi <= 1'b1;   // always process lower block first
                        state     <= ST_DEC_FEED_CT;
                        $display("[%0t] [4KB-DEC] Read beat %0d", $time, beat_cnt);
                    end
                end

                //--------------------------------------------------------------
                // Feed CT block to GCM - lo_not_hi selects ct_lo or ct_hi
                // gcm_dlast_reg: only set for very last block (CT[254])
                //--------------------------------------------------------------
                ST_DEC_FEED_CT: begin
                    if (gcm_input_ready) begin
                        gcm_din_reg    <= lo_not_hi ? ct_lo : ct_hi;
                        gcm_dvalid_reg <= 1'b1;
                        // CT[254] is in beat 127 lower block - it is the last block
                        gcm_dlast_reg  <= (beat_cnt == 7'd127) && lo_not_hi;
                        state          <= ST_DEC_WAIT_PT;
                    end
                end

                //--------------------------------------------------------------
                // Wait for GCM to output decrypted plaintext block
                //--------------------------------------------------------------
                ST_DEC_WAIT_PT: begin
                    if (gcm_dout_valid) begin
                        plaintext_out       <= gcm_dout;
                        plaintext_out_valid <= 1'b1;
                        // Last output block: beat 127, lower block (CT[254])
                        plaintext_out_last  <= (beat_cnt == 7'd127) && lo_not_hi;
                        state               <= ST_DEC_OUT_PT;
                        $display("[%0t] [4KB-DEC] PT beat=%0d lo=%b = 0x%032h",
                                 $time, beat_cnt, lo_not_hi, gcm_dout);
                    end
                end

                //--------------------------------------------------------------
                // Wait for client to consume PT block, then decide next step:
                //   beat==127 && lo: last block ? wait for tag verification
                //   lo_not_hi==1:    lower done ? process upper block of same beat
                //   lo_not_hi==0:    upper done ? advance to next DDR beat
                //--------------------------------------------------------------
                ST_DEC_OUT_PT: begin
                    if (plaintext_out_ready) begin
                        plaintext_out_valid <= 1'b0;

                        if (beat_cnt == 7'd127 && lo_not_hi) begin
                            // CT[254] delivered - wait for tag
                            state <= ST_DEC_WAIT_TAG;
                        end else if (lo_not_hi) begin
                            // Lower block done - process upper block same beat
                            lo_not_hi <= 1'b0;
                            state     <= ST_DEC_FEED_CT;
                        end else begin
                            // Upper block done - advance to next beat
                            beat_cnt    <= beat_cnt + 1'b1;
                            cur_rd_addr <= cur_rd_addr + 32'h00000020;
                            lo_not_hi   <= 1'b1;
                            state       <= ST_DEC_RD_ADDR;
                        end
                    end
                end

                //--------------------------------------------------------------
                // Wait for GCM tag verification result
                //--------------------------------------------------------------
                ST_DEC_WAIT_TAG: begin
                    if (gcm_tag_valid) begin
                        tag_match_reg <= gcm_tag_match_out;
                        done_reg      <= 1'b1;
                        state         <= ST_IDLE;
                        $display("[%0t] [4KB-DEC] Tag verify match=%b",
                                 $time, gcm_tag_match_out);
                    end
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

    //=========================================================================
    // AES-GCM Instance
    //=========================================================================
    aes_gcm_pipelined aes_gcm_inst (
        .clk              (clk),
        .rst_n            (rst_n),
        .enable           (1'b1),
        .start            (gcm_start_pulse),
        .encrypt          (is_encrypt_mode),
        .key              (key),
        .iv               (iv),
        .data_in          (gcm_din_reg),
        .data_valid       (gcm_dvalid_reg),
        .data_last        (gcm_dlast_reg),
        .data_bytes_valid (4'd0),
        .data_out         (gcm_dout),
        .data_out_valid   (gcm_dout_valid),
        .data_out_last    (gcm_dout_last),
        .tag_in           (stored_tag),
        .tag_out          (gcm_tag_out),
        .tag_valid        (gcm_tag_valid),
        .tag_match        (gcm_tag_match_out),
        .ready            (gcm_ready),
        .busy             (gcm_busy_wire),
        .input_ready      (gcm_input_ready)
    );

    //=========================================================================
    // AXI Output Assignments - single beat per write/read (AWLEN=0, ARLEN=0)
    //=========================================================================
    assign m_axi_awaddr  = cur_wr_addr;
    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'b101;        // 32 bytes per beat
    assign m_axi_awburst = 2'b01;         // INCR
    assign m_axi_awvalid = axi_awvalid_reg;

    assign m_axi_wdata   = wdata_reg;
    assign m_axi_wstrb   = 32'hFFFFFFFF;
    assign m_axi_wlast   = axi_wvalid_reg;
    assign m_axi_wvalid  = axi_wvalid_reg;
    assign m_axi_bready  = 1'b1;

    assign m_axi_araddr  = cur_rd_addr;
    assign m_axi_arlen   = 8'd0;
    assign m_axi_arsize  = 3'b101;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = axi_arvalid_reg;
    assign m_axi_rready  = axi_rready_reg;

    //=========================================================================
    // Status Outputs
    //=========================================================================
    assign busy            = (state != ST_IDLE);
    assign done            = done_reg;
    assign tag_match       = tag_match_reg;

    // plaintext_ready: high when FSM is waiting for a PT block from client
    // and GCM is ready to accept it
    assign plaintext_ready = (state == ST_ENC_RECV_EVEN ||
                              state == ST_ENC_RECV_ODD  ||
                              state == ST_ENC_RECV_LAST) && gcm_input_ready;
                              

endmodule

/*
module aes_gcm_axi_integration #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 128  // Plaintext interface stays 128-bit
)(
    input  wire                     clk,
    input  wire                     rst_n,
    
    // Control
    input  wire                     start_encrypt,
    input  wire                     start_decrypt,
    input  wire [127:0]             key,
    input  wire [95:0]              iv,
    input  wire [ADDR_WIDTH-1:0]    ddr_addr,  // Single address for {tag, ciphertext}
    
    // Status
    output wire                     busy,
    output wire                     done,
    output wire                     tag_match,
    
    // Plaintext input (for encryption)
    input  wire [DATA_WIDTH-1:0]    plaintext_in,
    input  wire                     plaintext_valid,
    input  wire                     plaintext_last,
    output wire                     plaintext_ready,
    
    // Plaintext output (for decryption)
    output wire [DATA_WIDTH-1:0]    plaintext_out,
    output wire                     plaintext_out_valid,
    output wire                     plaintext_out_last,
    input  wire                     plaintext_out_ready,
    
    // AXI Master Interface (256-bit to MIG)
    output wire [ADDR_WIDTH-1:0]    m_axi_awaddr,
    output wire [7:0]               m_axi_awlen,
    output wire [2:0]               m_axi_awsize,
    output wire [1:0]               m_axi_awburst,
    output wire                     m_axi_awvalid,
    input  wire                     m_axi_awready,
    output wire [255:0]             m_axi_wdata,
    output wire [31:0]              m_axi_wstrb,
    output wire                     m_axi_wlast,
    output wire                     m_axi_wvalid,
    input  wire                     m_axi_wready,
    input  wire [1:0]               m_axi_bresp,
    input  wire                     m_axi_bvalid,
    output wire                     m_axi_bready,
    output wire [ADDR_WIDTH-1:0]    m_axi_araddr,
    output wire [7:0]               m_axi_arlen,
    output wire [2:0]               m_axi_arsize,
    output wire [1:0]               m_axi_arburst,
    output wire                     m_axi_arvalid,
    input  wire                     m_axi_arready,
    input  wire [255:0]             m_axi_rdata,
    input  wire [1:0]               m_axi_rresp,
    input  wire                     m_axi_rlast,
    input  wire                     m_axi_rvalid,
    output wire                     m_axi_rready
);

    //=========================================================================
    // Main State Machine
    //=========================================================================
    localparam [3:0]
        ST_IDLE         = 4'd0,
        // Encryption states
        ST_ENC_WAIT_PT  = 4'd1,   // Wait for plaintext
        ST_ENC_RUN      = 4'd2,   // GCM processing
        ST_ENC_WAIT_TAG = 4'd3,   // Wait for tag
        ST_ENC_WR_ADDR  = 4'd4,   // AXI write address
        ST_ENC_WR_DATA  = 4'd5,   // AXI write data
        ST_ENC_WR_RESP  = 4'd6,   // AXI write response
        // Decryption states
        ST_DEC_RD_ADDR  = 4'd7,   // AXI read address
        ST_DEC_RD_DATA  = 4'd8,   // AXI read data
        ST_DEC_START    = 4'd9,   // Start GCM
        ST_DEC_RUN      = 4'd10,  // GCM processing
        ST_DEC_WAIT_TAG = 4'd11,  // Wait for tag verification
        // Done
        ST_DONE         = 4'd12;
    
    reg [3:0] state;

    //=========================================================================
    // Internal Registers
    //=========================================================================
    reg         is_encrypt_mode;
    reg         gcm_start_pulse;
    reg [127:0] stored_ciphertext;
    reg [127:0] stored_tag;
    reg [127:0] computed_tag;
    reg [ADDR_WIDTH-1:0] addr_reg;
    
    // AXI registers
    reg         axi_awvalid_reg;
    reg         axi_wvalid_reg;
    reg         axi_arvalid_reg;
    reg         axi_rready_reg;
    
    // GCM data registers
    reg [127:0] gcm_data_in_reg;
    reg         gcm_data_valid_reg;
    reg         gcm_data_last_reg;
    
    //=========================================================================
    // GCM Interface Signals
    //=========================================================================
    wire [127:0] gcm_data_out;
    wire         gcm_data_out_valid;
    wire         gcm_data_out_last;
    wire [127:0] gcm_tag_out;
    wire         gcm_tag_valid;
    wire         gcm_tag_match_out;
    wire         gcm_ready;
    wire         gcm_busy;
    wire         gcm_input_ready;
    
    //=========================================================================
    // Output Registers
    //=========================================================================
    reg         done_reg;
    reg         tag_match_reg;
    reg [127:0] plaintext_out_reg;
    reg         plaintext_out_valid_reg;
     //=========================================================================
    // Internal Registers (ADD THESE)
    //=========================================================================
    reg         stored_ciphertext_valid;
    reg         stored_tag_valid;
    reg         axi_write_started;
    //=========================================================================
    // Main State Machine
    //=========================================================================
     always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state               <= ST_IDLE;
            is_encrypt_mode     <= 1'b1;
            gcm_start_pulse     <= 1'b0;
            stored_ciphertext   <= 128'b0;
            stored_tag          <= 128'b0;
            computed_tag        <= 128'b0;
            addr_reg            <= 0;
            axi_awvalid_reg     <= 1'b0;
            axi_wvalid_reg      <= 1'b0;
            axi_arvalid_reg     <= 1'b0;
            axi_rready_reg      <= 1'b0;
            gcm_data_in_reg     <= 128'b0;
            gcm_data_valid_reg  <= 1'b0;
            gcm_data_last_reg   <= 1'b0;
            done_reg            <= 1'b0;
            tag_match_reg       <= 1'b0;
            plaintext_out_reg   <= 128'b0;
            plaintext_out_valid_reg <= 1'b0;
            stored_ciphertext_valid <= 1'b0;  // ADD
            stored_tag_valid        <= 1'b0;  // ADD
            axi_write_started       <= 1'b0;  // ADD
        end else begin
            // Default: clear pulses
            gcm_start_pulse     <= 1'b0;
            gcm_data_valid_reg  <= 1'b0;
            done_reg            <= 1'b0;
            plaintext_out_valid_reg <= 1'b0;
            
            case (state)
                //=============================================================
                // IDLE
                //=============================================================
                ST_IDLE: begin
                    axi_awvalid_reg <= 1'b0;
                    axi_wvalid_reg  <= 1'b0;
                    axi_arvalid_reg <= 1'b0;
                    axi_rready_reg  <= 1'b0;
                    gcm_data_last_reg <= 1'b0;  // ADD THIS LINE
                    
                    if (start_encrypt) begin
                        $display("[%0t] [AXI-DBG] start_encrypt pulse, ddr_addr=0x%08h", $time, ddr_addr);
                        is_encrypt_mode <= 1'b1;
                        addr_reg        <= ddr_addr;
                        gcm_start_pulse <= 1'b1;
                        state           <= ST_ENC_WAIT_PT;
                    end
                    else if (start_decrypt) begin
                        is_encrypt_mode <= 1'b0;
                        addr_reg        <= ddr_addr;
                        state           <= ST_DEC_RD_ADDR;
                    end
                end
                
                //=============================================================
                // ENCRYPTION FLOW
                //=============================================================
                ST_ENC_WAIT_PT: begin
                    if (gcm_input_ready && plaintext_valid) begin
                        gcm_data_in_reg    <= plaintext_in;
                        gcm_data_valid_reg <= 1'b1;
                        gcm_data_last_reg  <= 1'b1;
                        state              <= ST_ENC_RUN;
                    end
                end
                
                ST_ENC_RUN: begin
                    if (gcm_data_out_valid) begin
                        stored_ciphertext <= gcm_data_out;
                        state             <= ST_ENC_WAIT_TAG;
                    end
                end
                
                ST_ENC_WAIT_TAG: begin
                    if (gcm_tag_valid) begin
                        computed_tag <= gcm_tag_out;
                        state        <= ST_ENC_WR_ADDR;
                    end
                end
                
                ST_ENC_WR_ADDR: begin
                    axi_awvalid_reg <= 1'b1;
                    
                    if (m_axi_awready && axi_awvalid_reg) begin
                        axi_awvalid_reg <= 1'b0;
                        axi_wvalid_reg  <= 1'b1;
                        state           <= ST_ENC_WR_DATA;
                    end
                end
                
                ST_ENC_WR_DATA: begin
                    axi_awvalid_reg <= 1'b0;
                    
                    if (m_axi_wready && axi_wvalid_reg) begin
                        axi_wvalid_reg <= 1'b0;
                        state          <= ST_ENC_WR_RESP;
                    end
                end
                
                ST_ENC_WR_RESP: begin
                    axi_wvalid_reg <= 1'b0;
                    
                    if (m_axi_bvalid) begin
                        done_reg      <= 1'b1;
                        tag_match_reg <= 1'b1;
                        state         <= ST_DONE;
                    end
                end
 

        
                //=============================================================
                // DECRYPTION FLOW
                //=============================================================
                ST_DEC_RD_ADDR: begin
                    axi_arvalid_reg <= 1'b1;
                    
                    if (m_axi_arready && axi_arvalid_reg) begin
                        axi_arvalid_reg <= 1'b0;
                        axi_rready_reg  <= 1'b1;
                        state           <= ST_DEC_RD_DATA;
                    end
                end
                
                ST_DEC_RD_DATA: begin
                    axi_arvalid_reg <= 1'b0;
                    
                    if (m_axi_rvalid && axi_rready_reg) begin
                        axi_rready_reg    <= 1'b0;
                        stored_tag        <= m_axi_rdata[255:128];
                        stored_ciphertext <= m_axi_rdata[127:0];
                        $display("[%0t] [AES-CAPTURE] stored_ciphertext=0x%032h from rdata=0x%064h", 
                        $time, m_axi_rdata[127:0], m_axi_rdata);
                        state             <= ST_DEC_START;
                    end
                end
                
                ST_DEC_START: begin
                    axi_rready_reg  <= 1'b0;
                    gcm_start_pulse <= 1'b1;
                    state           <= ST_DEC_RUN;
                end
                
                ST_DEC_RUN: begin
                    if (gcm_input_ready) begin
                        gcm_data_in_reg    <= stored_ciphertext;
                        $display("[%0t] [AES-FEED] Feeding gcm_data_in_reg=0x%032h", $time, stored_ciphertext);  // ADD THIS LINE
                        gcm_data_valid_reg <= 1'b1;
                        gcm_data_last_reg  <= 1'b1;
                        state              <= ST_DEC_WAIT_TAG;
                    end
                end
                
                ST_DEC_WAIT_TAG: begin
                    if (gcm_data_out_valid) begin
                        plaintext_out_reg       <= gcm_data_out;
                        plaintext_out_valid_reg <= 1'b1;
                    end
                    
                    if (gcm_tag_valid) begin
                        tag_match_reg <= gcm_tag_match_out;
                        done_reg      <= 1'b1;
                        state         <= ST_DONE;
                    end
                end
                
                //=============================================================
                // DONE
                //=============================================================
                ST_DONE: begin
                    stored_ciphertext_valid <= 1'b0;
                    stored_tag_valid        <= 1'b0;
                    axi_write_started       <= 1'b0;
                    state <= ST_IDLE;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
    
    //=========================================================================
    // AES-GCM Instance
    //=========================================================================
    aes_gcm_pipelined aes_gcm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(1'b1),
        
        .start(gcm_start_pulse),
        .encrypt(is_encrypt_mode),
        .key(key),
        .iv(iv),
        
        .data_in(gcm_data_in_reg),
        .data_valid(gcm_data_valid_reg),
        .data_last(gcm_data_last_reg),
        .data_bytes_valid(4'd0),  // Full 16 bytes
        
        .data_out(gcm_data_out),
        .data_out_valid(gcm_data_out_valid),
        .data_out_last(gcm_data_out_last),
        
        .tag_in(stored_tag),
        .tag_out(gcm_tag_out),
        .tag_valid(gcm_tag_valid),
        .tag_match(gcm_tag_match_out),
        
        .ready(gcm_ready),
        .busy(gcm_busy),
        .input_ready(gcm_input_ready)
    );
    
    //=========================================================================
    // AXI Output Assignments (256-bit interface)
    //=========================================================================
    
    // Write channel
    assign m_axi_awaddr  = addr_reg;
    assign m_axi_awlen   = 8'd0;           // Single beat
    assign m_axi_awsize  = 3'b101;         // 32 bytes (256-bit)
    assign m_axi_awburst = 2'b01;          // INCR
    assign m_axi_awvalid = axi_awvalid_reg;
    
    assign m_axi_wdata   = {computed_tag, stored_ciphertext};  // {tag[255:128], ct[127:0]}
    assign m_axi_wstrb   = 32'hFFFFFFFF;   // All bytes valid
    assign m_axi_wlast   = axi_wvalid_reg; // Single beat, so WLAST = WVALID
    assign m_axi_wvalid  = axi_wvalid_reg;
    
    assign m_axi_bready  = 1'b1;           // Always ready for response
    
    // Read channel
    assign m_axi_araddr  = addr_reg;
    assign m_axi_arlen   = 8'd0;           // Single beat
    assign m_axi_arsize  = 3'b101;         // 32 bytes (256-bit)
    assign m_axi_arburst = 2'b01;          // INCR
    assign m_axi_arvalid = axi_arvalid_reg;
    
    assign m_axi_rready  = axi_rready_reg;
    
    //=========================================================================
    // Output Assignments
    //=========================================================================
    assign busy       = (state != ST_IDLE);
    assign done       = done_reg;
    assign tag_match  = tag_match_reg;
    
    assign plaintext_ready     = (state == ST_ENC_WAIT_PT) && gcm_input_ready;
    assign plaintext_out       = plaintext_out_reg;
    assign plaintext_out_valid = plaintext_out_valid_reg;
    assign plaintext_out_last  = plaintext_out_valid_reg;  // Single block, so always last

endmodule
*/