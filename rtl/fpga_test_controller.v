`timescale 1ns / 1ps
//============================================================================
// FPGA Test Controller for secure_memory_system - v3 FIXED
//
// Fixes over v2:
//   - Removed record_fail task (synthesis-unsafe NBA in task)
//   - No competing mstate assignments (proper if/else if chains)
//   - Violation checked directly on access_violation signal
//   - Clean priority: timeout > violation > normal handshake
//============================================================================

module fpga_test_controller #(
    parameter ADDR_WIDTH     = 32,
    parameter DATA_WIDTH     = 128,
    parameter NUM_CLIENTS    = 2,
    parameter NUM_REGIONS    = 16,
    parameter TOKEN_WIDTH    = 32,
    parameter LEASE_ID_WIDTH = 8,
    parameter NUM_BLOCKS     = 255,
    parameter TIMEOUT_CYCLES = 100000
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,

    // Management Interface
    output reg                                mgmt_req,
    output reg  [2:0]                         mgmt_op,
    output reg  [LEASE_ID_WIDTH-1:0]          mgmt_lease_id,
    output reg  [$clog2(NUM_CLIENTS)-1:0]     mgmt_client_id,
    output reg  [ADDR_WIDTH-1:0]              mgmt_base_addr,
    output reg  [ADDR_WIDTH-1:0]              mgmt_size,
    output reg  [31:0]                        mgmt_duration,
    output reg  [TOKEN_WIDTH-1:0]             mgmt_token_in,
    input  wire                               mgmt_ack,
    input  wire                               mgmt_error,
    input  wire [TOKEN_WIDTH-1:0]             mgmt_token_out,

    // Client Interfaces
    output reg  [NUM_CLIENTS-1:0]                 client_req,
    output reg  [NUM_CLIENTS-1:0]                 client_wr_en,
    output reg  [NUM_CLIENTS*ADDR_WIDTH-1:0]      client_addr,
    output reg  [NUM_CLIENTS*DATA_WIDTH-1:0]      client_wdata,
    output reg  [NUM_CLIENTS*TOKEN_WIDTH-1:0]     client_token,
    output reg  [NUM_CLIENTS*LEASE_ID_WIDTH-1:0]  client_lease_id,
    input  wire [NUM_CLIENTS-1:0]                 client_ack,
    input  wire [NUM_CLIENTS*DATA_WIDTH-1:0]      client_rdata,
    input  wire [NUM_CLIENTS-1:0]                 client_rdata_valid,
    input  wire [NUM_CLIENTS-1:0]                 access_violation,

    // Plaintext ready (proper port from SMS)
    input  wire                                   plaintext_ready,

    // IV monitoring
    input  wire [95:0]                            active_iv,
    input  wire                                   iv_updated,

    // AES status
    input  wire                                   aes_tag_match,

    // Test Status
    output reg          test_running,
    output reg          test_done,
    output reg          test_pass,
    output reg  [3:0]   test_id,
    output reg  [3:0]   test_fail_id,
    output reg  [7:0]   error_count
);

    //=========================================================================
    // Constants
    //=========================================================================
    localparam [2:0] OP_GRANT  = 3'd0;
    localparam [2:0] OP_REVOKE = 3'd1;

    //=========================================================================
    // Master FSM States
    //=========================================================================
    localparam [5:0]
        M_IDLE              = 6'd0,
        M_RESET_WAIT        = 6'd1,
        M_T1_GRANT_C0       = 6'd2,
        M_T1_GRANT_C0_WAIT  = 6'd3,
        M_T1_GRANT_C1       = 6'd4,
        M_T1_GRANT_C1_WAIT  = 6'd5,
        M_T1_GAP            = 6'd6,
        M_T2_ENC_PULSE      = 6'd7,
        M_T2_ENC_STREAM     = 6'd8,
        M_T2_ENC_WAIT_ACK   = 6'd9,
        M_T2_ENC_GAP        = 6'd10,
        M_T2_DEC_PULSE      = 6'd11,
        M_T2_DEC_RUN        = 6'd12,
        M_T2_DEC_GAP        = 6'd13,
        M_T3_ENC_PULSE      = 6'd14,
        M_T3_ENC_STREAM     = 6'd15,
        M_T3_ENC_WAIT_ACK   = 6'd16,
        M_T3_ENC_GAP        = 6'd17,
        M_T3_DEC_PULSE      = 6'd18,
        M_T3_DEC_RUN        = 6'd19,
        M_T3_DEC_GAP        = 6'd20,
        M_T4_PULSE           = 6'd21,
        M_T4_RUN             = 6'd22,
        M_T4_GAP             = 6'd23,
        M_T5_PULSE           = 6'd24,
        M_T5_RUN             = 6'd25,
        M_T5_GAP             = 6'd26,
        M_T6_REVOKE          = 6'd27,
        M_T6_REVOKE_WAIT     = 6'd28,
        M_T6_ATK_PULSE       = 6'd29,
        M_T6_ATK_RUN         = 6'd30,
        M_T6_ATK_GAP         = 6'd31,
        M_T6_C1_PULSE        = 6'd32,
        M_T6_C1_STREAM       = 6'd33,
        M_T6_C1_WAIT_ACK     = 6'd34,
        M_T6_GAP             = 6'd35,
        M_T7_GRANT           = 6'd36,
        M_T7_GRANT_WAIT      = 6'd37,
        M_T7_ENC_PULSE       = 6'd38,
        M_T7_ENC_STREAM      = 6'd39,
        M_T7_ENC_WAIT_ACK    = 6'd40,
        M_T7_ENC_GAP         = 6'd41,
        M_T7_DEC_PULSE       = 6'd42,
        M_T7_DEC_RUN         = 6'd43,
        M_T7_DEC_GAP         = 6'd44,
        M_DONE               = 6'd45;

    reg [5:0] mstate;

    //=========================================================================
    // Internal Registers
    //=========================================================================
    reg [TOKEN_WIDTH-1:0]  client0_token;
    reg [TOKEN_WIDTH-1:0]  client1_token;
    reg [7:0]   blk_cnt;
    reg [7:0]   dec_blk_cnt;
    reg [7:0]   dec_mismatch_cnt;
    reg [16:0]  timeout_cnt;
    reg [4:0]   gap_cnt;
    reg         got_violation;
    reg         enc_client;
    reg [95:0]  iv_after_first_enc;
    reg [95:0]  iv_after_second_enc;
    reg         iv_captured_first;
    
    // Latch IV on iv_updated pulse (reliable timing)
    reg [95:0]  last_enc_iv;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            last_enc_iv <= 96'b0;
        else if (iv_updated) begin
            last_enc_iv <= active_iv;
            $display("[%0t] [TC] iv_updated pulse: active_iv=0x%024h", $time, active_iv);
        end
    end

    //=========================================================================
    // Plaintext Generators (combinational)
    //=========================================================================
    wire [DATA_WIDTH-1:0] pt_c0 = {96'hDEADBEEF_C0FFEE00_12345678, 24'h000000, blk_cnt[7:0]};
    wire [DATA_WIDTH-1:0] pt_c1 = {96'hCAFEBABE_FACADE00_87654321, 24'hFFFFFF, blk_cnt[7:0]};
    wire [DATA_WIDTH-1:0] exp_c0 = {96'hDEADBEEF_C0FFEE00_12345678, 24'h000000, dec_blk_cnt[7:0]};
    wire [DATA_WIDTH-1:0] exp_c1 = {96'hCAFEBABE_FACADE00_87654321, 24'hFFFFFF, dec_blk_cnt[7:0]};

    //=========================================================================
    // Master FSM
    //=========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mstate          <= M_IDLE;
            test_running    <= 1'b0;
            test_done       <= 1'b0;
            test_pass       <= 1'b0;
            test_id         <= 4'd0;
            test_fail_id    <= 4'd0;
            error_count     <= 8'd0;
            mgmt_req        <= 1'b0;
            mgmt_op         <= 3'd0;
            mgmt_lease_id   <= {LEASE_ID_WIDTH{1'b0}};
            mgmt_client_id  <= 1'b0;
            mgmt_base_addr  <= 32'b0;
            mgmt_size       <= 32'b0;
            mgmt_duration   <= 32'b0;
            mgmt_token_in   <= {TOKEN_WIDTH{1'b0}};
            client_req      <= {NUM_CLIENTS{1'b0}};
            client_wr_en    <= {NUM_CLIENTS{1'b0}};
            client_addr     <= {NUM_CLIENTS*ADDR_WIDTH{1'b0}};
            client_wdata    <= {NUM_CLIENTS*DATA_WIDTH{1'b0}};
            client_token    <= {NUM_CLIENTS*TOKEN_WIDTH{1'b0}};
            client_lease_id <= {NUM_CLIENTS*LEASE_ID_WIDTH{1'b0}};
            client0_token   <= {TOKEN_WIDTH{1'b0}};
            client1_token   <= {TOKEN_WIDTH{1'b0}};
            blk_cnt         <= 8'd0;
            dec_blk_cnt     <= 8'd0;
            dec_mismatch_cnt<= 8'd0;
            timeout_cnt     <= 17'd0;
            gap_cnt         <= 5'd0;
            enc_client      <= 1'b0;
            got_violation   <= 1'b0;
            iv_after_first_enc  <= 96'b0;
            iv_after_second_enc <= 96'b0;
            iv_captured_first   <= 1'b0;
        end else begin
            // Defaults
            mgmt_req  <= 1'b0;
            test_done <= 1'b0;
            
            case (mstate)

                //=============================================================
                M_IDLE: begin
                    if (start) begin
                        test_running      <= 1'b1;
                        test_pass         <= 1'b1;
                        error_count       <= 8'd0;
                        test_fail_id      <= 4'd0;
                        iv_captured_first <= 1'b0;
                        gap_cnt           <= 5'd20;
                        mstate            <= M_RESET_WAIT;
                    end
                end

                M_RESET_WAIT: begin
                    if (gap_cnt == 0) mstate <= M_T1_GRANT_C0;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 1: Grant Leases
                //=============================================================
                M_T1_GRANT_C0: begin
                    test_id <= 4'd1;
                    mgmt_req <= 1'b1; mgmt_op <= OP_GRANT;
                    mgmt_lease_id <= 8'd0; mgmt_client_id <= 1'b0;
                    mgmt_base_addr <= 32'h00010000; mgmt_size <= 32'h00008000;
                    mgmt_duration <= 32'hFFFFFFFF; mgmt_token_in <= 32'h0;
                    timeout_cnt <= 17'd0;
                    mstate <= M_T1_GRANT_C0_WAIT;
                end

                M_T1_GRANT_C0_WAIT: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (mgmt_ack) begin
                        client0_token <= mgmt_token_out;
                        gap_cnt <= 5'd5; mstate <= M_T1_GRANT_C1;
                    end else if (mgmt_error || timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd1;
                        mstate <= M_DONE;
                    end
                end

                M_T1_GRANT_C1: begin
                    if (gap_cnt != 0) gap_cnt <= gap_cnt - 1;
                    else begin
                        mgmt_req <= 1'b1; mgmt_op <= OP_GRANT;
                        mgmt_lease_id <= 8'd1; mgmt_client_id <= 1'b1;
                        mgmt_base_addr <= 32'h00020000; mgmt_size <= 32'h00008000;
                        mgmt_duration <= 32'hFFFFFFFF; mgmt_token_in <= 32'h0;
                        timeout_cnt <= 17'd0;
                        mstate <= M_T1_GRANT_C1_WAIT;
                    end
                end

                M_T1_GRANT_C1_WAIT: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (mgmt_ack) begin
                        client1_token <= mgmt_token_out;
                        gap_cnt <= 5'd5; mstate <= M_T1_GAP;
                    end else if (mgmt_error || timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd1;
                        mstate <= M_DONE;
                    end
                end

                M_T1_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T2_ENC_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 2: Client 0 Encrypt 4KB
                //=============================================================
                M_T2_ENC_PULSE: begin
                    test_id <= 4'd2;
                    enc_client <= 1'b0;
                    blk_cnt <= 8'd0;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[0] <= 1'b1;
                    client_wr_en[0] <= 1'b1;
                    client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00010000;
                    client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= client0_token;
                    client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd0;
                    client_wdata[0*DATA_WIDTH +: DATA_WIDTH] <=
                        {96'hDEADBEEF_C0FFEE00_12345678, 24'h000000, 8'h00};
                    mstate <= M_T2_ENC_STREAM;
                end

                M_T2_ENC_STREAM: begin
                    client_req[0] <= 1'b0;
                    client_wr_en[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    // Priority: timeout > violation > streaming
                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd2;
                        mstate <= M_DONE;
                    end else if (access_violation[0] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd2;
                        gap_cnt <= 5'd10; mstate <= M_T2_ENC_GAP;
                    end else if (plaintext_ready) begin
                        if (blk_cnt < NUM_BLOCKS - 1) begin
                            blk_cnt <= blk_cnt + 1;
                            client_wdata[0*DATA_WIDTH +: DATA_WIDTH] <=
                                {96'hDEADBEEF_C0FFEE00_12345678, 24'h000000, blk_cnt[7:0] + 8'd1};
                        end else begin
                            mstate <= M_T2_ENC_WAIT_ACK;
                        end
                    end
                end

                M_T2_ENC_WAIT_ACK: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (client_ack[0]) begin
                        if (!iv_captured_first) begin
                            iv_after_first_enc <= last_enc_iv;
                            iv_captured_first <= 1'b1;
                            $display("[%0t] [TC] T2: Captured first IV = 0x%024h", $time, last_enc_iv);
                        end
                        gap_cnt <= 5'd20; mstate <= M_T2_ENC_GAP;
                    end else if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd2;
                        mstate <= M_DONE;
                    end
                end

                M_T2_ENC_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T2_DEC_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 2: Client 0 Decrypt 4KB
                //=============================================================
                M_T2_DEC_PULSE: begin
                    dec_blk_cnt <= 8'd0;
                    dec_mismatch_cnt <= 8'd0;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[0] <= 1'b1;
                    client_wr_en[0] <= 1'b0;
                    client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00010000;
                    client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= client0_token;
                    client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd0;
                    mstate <= M_T2_DEC_RUN;
                end

                M_T2_DEC_RUN: begin
                    client_req[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    if (client_rdata_valid[0] && !got_violation) begin
                        if (client_rdata[0*DATA_WIDTH +: DATA_WIDTH] != exp_c0)
                            dec_mismatch_cnt <= dec_mismatch_cnt + 1;
                        dec_blk_cnt <= dec_blk_cnt + 1;
                    end

                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd2;
                        mstate <= M_DONE;
                    end else if (client_ack[0]) begin
                        $display("[%0t] [TC] T2 DEC done: violation=%b tag_match=%b mismatches=%0d blk_cnt=%0d",
                                 $time, got_violation, aes_tag_match, dec_mismatch_cnt, dec_blk_cnt);
                        if (got_violation || !aes_tag_match || dec_mismatch_cnt != 0) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd2;
                        end
                        gap_cnt <= 5'd20; mstate <= M_T2_DEC_GAP;
                    end else if (access_violation[0] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd2;
                        gap_cnt <= 5'd10; mstate <= M_T2_DEC_GAP;
                    end
                end

                M_T2_DEC_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T3_ENC_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 3: Client 1 Encrypt 4KB
                //=============================================================
                M_T3_ENC_PULSE: begin
                    test_id <= 4'd3;
                    enc_client <= 1'b1;
                    blk_cnt <= 8'd0;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[1] <= 1'b1;
                    client_wr_en[1] <= 1'b1;
                    client_addr[1*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00020000;
                    client_token[1*TOKEN_WIDTH +: TOKEN_WIDTH] <= client1_token;
                    client_lease_id[1*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd1;
                    client_wdata[1*DATA_WIDTH +: DATA_WIDTH] <=
                        {96'hCAFEBABE_FACADE00_87654321, 24'hFFFFFF, 8'h00};
                    mstate <= M_T3_ENC_STREAM;
                end

                M_T3_ENC_STREAM: begin
                    client_req[1] <= 1'b0;
                    client_wr_en[1] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[1]) got_violation <= 1'b1;

                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd3;
                        mstate <= M_DONE;
                    end else if (access_violation[1] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd3;
                        gap_cnt <= 5'd10; mstate <= M_T3_ENC_GAP;
                    end else if (plaintext_ready) begin
                        if (blk_cnt < NUM_BLOCKS - 1) begin
                            blk_cnt <= blk_cnt + 1;
                            client_wdata[1*DATA_WIDTH +: DATA_WIDTH] <=
                                {96'hCAFEBABE_FACADE00_87654321, 24'hFFFFFF, blk_cnt[7:0] + 8'd1};
                        end else begin
                            mstate <= M_T3_ENC_WAIT_ACK;
                        end
                    end
                end

                M_T3_ENC_WAIT_ACK: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (client_ack[1]) begin
                        gap_cnt <= 5'd20; mstate <= M_T3_ENC_GAP;
                    end else if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd3;
                        mstate <= M_DONE;
                    end
                end

                M_T3_ENC_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T3_DEC_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 3: Client 1 Decrypt 4KB
                //=============================================================
                M_T3_DEC_PULSE: begin
                    dec_blk_cnt <= 8'd0;
                    dec_mismatch_cnt <= 8'd0;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[1] <= 1'b1;
                    client_wr_en[1] <= 1'b0;
                    client_addr[1*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00020000;
                    client_token[1*TOKEN_WIDTH +: TOKEN_WIDTH] <= client1_token;
                    client_lease_id[1*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd1;
                    mstate <= M_T3_DEC_RUN;
                end

                M_T3_DEC_RUN: begin
                    client_req[1] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[1]) got_violation <= 1'b1;

                    if (client_rdata_valid[1] && !got_violation) begin
                        if (client_rdata[1*DATA_WIDTH +: DATA_WIDTH] != exp_c1)
                            dec_mismatch_cnt <= dec_mismatch_cnt + 1;
                        dec_blk_cnt <= dec_blk_cnt + 1;
                    end

                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd3;
                        mstate <= M_DONE;
                    end else if (client_ack[1]) begin
                        if (got_violation || !aes_tag_match || dec_mismatch_cnt != 0) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd3;
                        end
                        gap_cnt <= 5'd20; mstate <= M_T3_DEC_GAP;
                    end else if (access_violation[1] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd3;
                        gap_cnt <= 5'd10; mstate <= M_T3_DEC_GAP;
                    end
                end

                M_T3_DEC_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T4_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 4: Wrong Token Attack (expect violation)
                //=============================================================
                M_T4_PULSE: begin
                    test_id <= 4'd4;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[0] <= 1'b1;
                    client_wr_en[0] <= 1'b1;
                    client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00010000;
                    client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= 32'hDEADBEEF;
                    client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd0;
                    client_wdata[0*DATA_WIDTH +: DATA_WIDTH] <= 128'h0;
                    mstate <= M_T4_RUN;
                end

                M_T4_RUN: begin
                    client_req[0] <= 1'b0;
                    client_wr_en[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    if (timeout_cnt >= 17'd1000) begin
                        if (!got_violation) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd4;
                        end
                        gap_cnt <= 5'd10; mstate <= M_T4_GAP;
                    end else if (access_violation[0] || got_violation) begin
                        // PASS - violation detected
                        gap_cnt <= 5'd10; mstate <= M_T4_GAP;
                    end else if (client_ack[0]) begin
                        // FAIL - should not succeed
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd4;
                        gap_cnt <= 5'd10; mstate <= M_T4_GAP;
                    end
                end

                M_T4_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T5_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 5: Cross-Client Attack (expect violation)
                //=============================================================
                M_T5_PULSE: begin
                    test_id <= 4'd5;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[0] <= 1'b1;
                    client_wr_en[0] <= 1'b0;
                    client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00020000;
                    client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= client0_token;
                    client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd0;
                    mstate <= M_T5_RUN;
                end

                M_T5_RUN: begin
                    client_req[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    if (timeout_cnt >= 17'd1000) begin
                        if (!got_violation) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd5;
                        end
                        gap_cnt <= 5'd10; mstate <= M_T5_GAP;
                    end else if (access_violation[0] || got_violation) begin
                        // PASS
                        gap_cnt <= 5'd10; mstate <= M_T5_GAP;
                    end else if (client_ack[0]) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd5;
                        gap_cnt <= 5'd10; mstate <= M_T5_GAP;
                    end
                end

                M_T5_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T6_REVOKE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 6: Revoke + Attack + Confirm C1
                //=============================================================
                M_T6_REVOKE: begin
                    test_id <= 4'd6;
                    mgmt_req <= 1'b1; mgmt_op <= OP_REVOKE;
                    mgmt_lease_id <= 8'd0; mgmt_client_id <= 1'b0;
                    mgmt_token_in <= 32'h0;
                    timeout_cnt <= 17'd0;
                    mstate <= M_T6_REVOKE_WAIT;
                end

                M_T6_REVOKE_WAIT: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (mgmt_ack) begin
                        gap_cnt <= 5'd10; mstate <= M_T6_ATK_PULSE;
                    end else if (mgmt_error || timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd6;
                        mstate <= M_DONE;
                    end
                end

                M_T6_ATK_PULSE: begin
                    if (gap_cnt != 0) gap_cnt <= gap_cnt - 1;
                    else begin
                        got_violation <= 1'b0;
                        timeout_cnt <= 17'd0;
                        client_req[0] <= 1'b1;
                        client_wr_en[0] <= 1'b1;
                        client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00010000;
                        client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= client0_token;
                        client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd0;
                        client_wdata[0*DATA_WIDTH +: DATA_WIDTH] <= 128'h0;
                        mstate <= M_T6_ATK_RUN;
                    end
                end

                M_T6_ATK_RUN: begin
                    client_req[0] <= 1'b0;
                    client_wr_en[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    if (timeout_cnt >= 17'd1000) begin
                        if (!got_violation) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd6;
                        end
                        gap_cnt <= 5'd10; mstate <= M_T6_ATK_GAP;
                    end else if (access_violation[0] || got_violation) begin
                        // PASS
                        gap_cnt <= 5'd10; mstate <= M_T6_ATK_GAP;
                    end else if (client_ack[0]) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd6;
                        gap_cnt <= 5'd10; mstate <= M_T6_ATK_GAP;
                    end
                end

                M_T6_ATK_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T6_C1_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                // Confirm C1 still works
                M_T6_C1_PULSE: begin
                    enc_client <= 1'b1;
                    blk_cnt <= 8'd0;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[1] <= 1'b1;
                    client_wr_en[1] <= 1'b1;
                    client_addr[1*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00020000;
                    client_token[1*TOKEN_WIDTH +: TOKEN_WIDTH] <= client1_token;
                    client_lease_id[1*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd1;
                    client_wdata[1*DATA_WIDTH +: DATA_WIDTH] <=
                        {96'hCAFEBABE_FACADE00_87654321, 24'hFFFFFF, 8'h00};
                    mstate <= M_T6_C1_STREAM;
                end

                M_T6_C1_STREAM: begin
                    client_req[1] <= 1'b0;
                    client_wr_en[1] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[1]) got_violation <= 1'b1;

                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd6;
                        mstate <= M_DONE;
                    end else if (access_violation[1] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd6;
                        gap_cnt <= 5'd10; mstate <= M_T6_GAP;
                    end else if (plaintext_ready) begin
                        if (blk_cnt < NUM_BLOCKS - 1) begin
                            blk_cnt <= blk_cnt + 1;
                            client_wdata[1*DATA_WIDTH +: DATA_WIDTH] <=
                                {96'hCAFEBABE_FACADE00_87654321, 24'hFFFFFF, blk_cnt[7:0] + 8'd1};
                        end else begin
                            mstate <= M_T6_C1_WAIT_ACK;
                        end
                    end
                end

                M_T6_C1_WAIT_ACK: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (client_ack[1]) begin
                        if (got_violation) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd6;
                        end
                        gap_cnt <= 5'd20; mstate <= M_T6_GAP;
                    end else if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd6;
                        mstate <= M_DONE;
                    end
                end

                M_T6_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T7_GRANT;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                // TEST 7: Re-grant C0, re-encrypt, IV check, decrypt
                //=============================================================
                M_T7_GRANT: begin
                    test_id <= 4'd7;
                    mgmt_req <= 1'b1; mgmt_op <= OP_GRANT;
                    mgmt_lease_id <= 8'd2; mgmt_client_id <= 1'b0;
                    mgmt_base_addr <= 32'h00010000; mgmt_size <= 32'h00008000;
                    mgmt_duration <= 32'hFFFFFFFF; mgmt_token_in <= 32'h0;
                    timeout_cnt <= 17'd0;
                    mstate <= M_T7_GRANT_WAIT;
                end

                M_T7_GRANT_WAIT: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (mgmt_ack) begin
                        client0_token <= mgmt_token_out;
                        gap_cnt <= 5'd5; mstate <= M_T7_ENC_PULSE;
                    end else if (mgmt_error || timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd7;
                        mstate <= M_DONE;
                    end
                end

                M_T7_ENC_PULSE: begin
                    if (gap_cnt != 0) gap_cnt <= gap_cnt - 1;
                    else begin
                        enc_client <= 1'b0;
                        blk_cnt <= 8'd0;
                        got_violation <= 1'b0;
                        timeout_cnt <= 17'd0;
                        client_req[0] <= 1'b1;
                        client_wr_en[0] <= 1'b1;
                        client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00010000;
                        client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= client0_token;
                        client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd2;
                        client_wdata[0*DATA_WIDTH +: DATA_WIDTH] <=
                            {96'hDEADBEEF_C0FFEE00_12345678, 24'h000000, 8'h00};
                        mstate <= M_T7_ENC_STREAM;
                    end
                end

                M_T7_ENC_STREAM: begin
                    client_req[0] <= 1'b0;
                    client_wr_en[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd7;
                        mstate <= M_DONE;
                    end else if (access_violation[0] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd7;
                        gap_cnt <= 5'd10; mstate <= M_T7_ENC_GAP;
                    end else if (plaintext_ready) begin
                        if (blk_cnt < NUM_BLOCKS - 1) begin
                            blk_cnt <= blk_cnt + 1;
                            client_wdata[0*DATA_WIDTH +: DATA_WIDTH] <=
                                {96'hDEADBEEF_C0FFEE00_12345678, 24'h000000, blk_cnt[7:0] + 8'd1};
                        end else begin
                            mstate <= M_T7_ENC_WAIT_ACK;
                        end
                    end
                end

                M_T7_ENC_WAIT_ACK: begin
                    timeout_cnt <= timeout_cnt + 1;
                    if (client_ack[0]) begin
                        iv_after_second_enc <= last_enc_iv;
                        $display("[%0t] [TC] T7: iv_after_first_enc = 0x%024h", $time, iv_after_first_enc);
                        $display("[%0t] [TC] T7: last_enc_iv        = 0x%024h", $time, last_enc_iv);
                        $display("[%0t] [TC] T7: equal? %b", $time, (iv_after_first_enc == last_enc_iv));
                        if (iv_after_first_enc == last_enc_iv) begin
                            // IV not rotated!
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd7;
                        end
                        gap_cnt <= 5'd20; mstate <= M_T7_ENC_GAP;
                    end else if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd7;
                        mstate <= M_DONE;
                    end
                end

                M_T7_ENC_GAP: begin
                    if (gap_cnt == 0) mstate <= M_T7_DEC_PULSE;
                    else gap_cnt <= gap_cnt - 1;
                end

                M_T7_DEC_PULSE: begin
                    dec_blk_cnt <= 8'd0;
                    dec_mismatch_cnt <= 8'd0;
                    got_violation <= 1'b0;
                    timeout_cnt <= 17'd0;
                    client_req[0] <= 1'b1;
                    client_wr_en[0] <= 1'b0;
                    client_addr[0*ADDR_WIDTH +: ADDR_WIDTH] <= 32'h00010000;
                    client_token[0*TOKEN_WIDTH +: TOKEN_WIDTH] <= client0_token;
                    client_lease_id[0*LEASE_ID_WIDTH +: LEASE_ID_WIDTH] <= 8'd2;
                    mstate <= M_T7_DEC_RUN;
                end

                M_T7_DEC_RUN: begin
                    client_req[0] <= 1'b0;
                    timeout_cnt <= timeout_cnt + 1;

                    if (access_violation[0]) got_violation <= 1'b1;

                    if (client_rdata_valid[0] && !got_violation) begin
                        if (client_rdata[0*DATA_WIDTH +: DATA_WIDTH] != exp_c0)
                            dec_mismatch_cnt <= dec_mismatch_cnt + 1;
                        dec_blk_cnt <= dec_blk_cnt + 1;
                    end

                    if (timeout_cnt >= TIMEOUT_CYCLES) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd7;
                        mstate <= M_DONE;
                    end else if (client_ack[0]) begin
                        $display("[%0t] [TC] T7 DEC done: violation=%b tag_match=%b mismatches=%0d blk_cnt=%0d",
                                 $time, got_violation, aes_tag_match, dec_mismatch_cnt, dec_blk_cnt);
                        if (got_violation || !aes_tag_match || dec_mismatch_cnt != 0) begin
                            error_count <= error_count + 1; test_pass <= 1'b0;
                            if (test_fail_id == 0) test_fail_id <= 4'd7;
                        end
                        gap_cnt <= 5'd20; mstate <= M_T7_DEC_GAP;
                    end else if (access_violation[0] || got_violation) begin
                        error_count <= error_count + 1; test_pass <= 1'b0;
                        if (test_fail_id == 0) test_fail_id <= 4'd7;
                        gap_cnt <= 5'd10; mstate <= M_T7_DEC_GAP;
                    end
                end

                M_T7_DEC_GAP: begin
                    if (gap_cnt == 0) mstate <= M_DONE;
                    else gap_cnt <= gap_cnt - 1;
                end

                //=============================================================
                M_DONE: begin
                    test_done    <= 1'b1;
                    test_running <= 1'b0;
                    mstate       <= M_IDLE;
                end

                default: mstate <= M_IDLE;

            endcase
        end
    end

endmodule