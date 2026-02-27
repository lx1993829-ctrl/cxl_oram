
`timescale 1ns / 1ps
//============================================================================
// AES-128 Single Round Module - FPGA Synthesizable
// Performs one round of AES encryption
// Can be configured for final round (no MixColumns)
//============================================================================

module aes128_round_fpga (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    input  wire         is_final_round,
    input  wire [127:0] state_in,
    input  wire [127:0] round_key,
    input  wire         valid_in,
    output reg  [127:0] state_out,
    output reg          valid_out
);

    wire [127:0] after_sub_bytes;
    wire [127:0] after_shift_rows;
    wire [127:0] after_mix_columns;
    wire [127:0] after_add_round_key;

    // SubBytes
    aes_sub_bytes sub_bytes_inst (
        .state_in(state_in),
        .state_out(after_sub_bytes)
    );

    // ShiftRows
    aes_shift_rows shift_rows_inst (
        .state_in(after_sub_bytes),
        .state_out(after_shift_rows)
    );

    // MixColumns
    aes_mix_columns mix_columns_inst (
        .state_in(after_shift_rows),
        .state_out(after_mix_columns)
    );

    // Select based on final round
    wire [127:0] pre_add_key;
    assign pre_add_key = is_final_round ? after_shift_rows : after_mix_columns;

    // AddRoundKey
    aes_add_round_key add_key_inst (
        .state_in(pre_add_key),
        .round_key(round_key),
        .state_out(after_add_round_key)
    );

    // Pipeline register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_out <= 128'b0;
            valid_out <= 1'b0;
        end else if (enable) begin
            state_out <= after_add_round_key;
            valid_out <= valid_in;
        end
    end

endmodule
