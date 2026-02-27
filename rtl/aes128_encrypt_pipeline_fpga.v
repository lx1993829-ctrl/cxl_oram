
//============================================================================
// AES-128 Unrolled Pipeline Encryption Core - FPGA Synthesizable
// 10 rounds fully pipelined - one encryption per clock after initial latency
// Latency: 11 clock cycles (initial round + 10 rounds)
// Throughput: 128 bits per clock cycle
//============================================================================

module aes128_encrypt_pipeline_fpga (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    input  wire [127:0] plaintext,
    input  wire [127:0] key,
    input  wire         valid_in,
    output wire [127:0] ciphertext,
    output wire         valid_out
);

    // Round keys from key expansion - explicit wires
    wire [127:0] rk0, rk1, rk2, rk3, rk4, rk5, rk6, rk7, rk8, rk9, rk10;

    // Key expansion (combinational)
    aes128_key_expansion_fpga key_exp (
        .key_in(key),
        .round_key_0(rk0),
        .round_key_1(rk1),
        .round_key_2(rk2),
        .round_key_3(rk3),
        .round_key_4(rk4),
        .round_key_5(rk5),
        .round_key_6(rk6),
        .round_key_7(rk7),
        .round_key_8(rk8),
        .round_key_9(rk9),
        .round_key_10(rk10)
    );

    // Pipeline stages
    wire [127:0] state_0, state_1, state_2, state_3, state_4;
    wire [127:0] state_5, state_6, state_7, state_8, state_9, state_10;
    wire valid_0, valid_1, valid_2, valid_3, valid_4;
    wire valid_5, valid_6, valid_7, valid_8, valid_9, valid_10;

    // Initial AddRoundKey (round 0)
    reg [127:0] state_r0;
    reg         valid_r0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r0 <= 128'b0;
            valid_r0 <= 1'b0;
        end else if (enable) begin
            state_r0 <= plaintext ^ rk0;
            valid_r0 <= valid_in;
        end
    end

    assign state_0 = state_r0;
    assign valid_0 = valid_r0;

    // Round 1
    aes128_round_fpga round1 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_0), .round_key(rk1),
        .valid_in(valid_0),
        .state_out(state_1), .valid_out(valid_1)
    );

    // Round 2
    aes128_round_fpga round2 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_1), .round_key(rk2),
        .valid_in(valid_1),
        .state_out(state_2), .valid_out(valid_2)
    );

    // Round 3
    aes128_round_fpga round3 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_2), .round_key(rk3),
        .valid_in(valid_2),
        .state_out(state_3), .valid_out(valid_3)
    );

    // Round 4
    aes128_round_fpga round4 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_3), .round_key(rk4),
        .valid_in(valid_3),
        .state_out(state_4), .valid_out(valid_4)
    );

    // Round 5
    aes128_round_fpga round5 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_4), .round_key(rk5),
        .valid_in(valid_4),
        .state_out(state_5), .valid_out(valid_5)
    );

    // Round 6
    aes128_round_fpga round6 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_5), .round_key(rk6),
        .valid_in(valid_5),
        .state_out(state_6), .valid_out(valid_6)
    );

    // Round 7
    aes128_round_fpga round7 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_6), .round_key(rk7),
        .valid_in(valid_6),
        .state_out(state_7), .valid_out(valid_7)
    );

    // Round 8
    aes128_round_fpga round8 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_7), .round_key(rk8),
        .valid_in(valid_7),
        .state_out(state_8), .valid_out(valid_8)
    );

    // Round 9
    aes128_round_fpga round9 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b0),
        .state_in(state_8), .round_key(rk9),
        .valid_in(valid_8),
        .state_out(state_9), .valid_out(valid_9)
    );

    // Round 10 (Final)
    aes128_round_fpga round10 (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .is_final_round(1'b1),
        .state_in(state_9), .round_key(rk10),
        .valid_in(valid_9),
        .state_out(state_10), .valid_out(valid_10)
    );

    // Output
    assign ciphertext = state_10;
    assign valid_out = valid_10;

endmodule
