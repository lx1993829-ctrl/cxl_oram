`timescale 1ns / 1ps
//============================================================================
// AES-128 Key Expansion Module - FPGA Synthesizable
// Generates all 11 round keys from the initial 128-bit key
// Fully combinational - NO ARRAYS for maximum compatibility
//============================================================================

module aes128_key_expansion_fpga (
    input  wire [127:0] key_in,
    output wire [127:0] round_key_0,
    output wire [127:0] round_key_1,
    output wire [127:0] round_key_2,
    output wire [127:0] round_key_3,
    output wire [127:0] round_key_4,
    output wire [127:0] round_key_5,
    output wire [127:0] round_key_6,
    output wire [127:0] round_key_7,
    output wire [127:0] round_key_8,
    output wire [127:0] round_key_9,
    output wire [127:0] round_key_10
);

    // Round constants (explicit, no array)
    localparam [7:0] RCON0 = 8'h01;
    localparam [7:0] RCON1 = 8'h02;
    localparam [7:0] RCON2 = 8'h04;
    localparam [7:0] RCON3 = 8'h08;
    localparam [7:0] RCON4 = 8'h10;
    localparam [7:0] RCON5 = 8'h20;
    localparam [7:0] RCON6 = 8'h40;
    localparam [7:0] RCON7 = 8'h80;
    localparam [7:0] RCON8 = 8'h1b;
    localparam [7:0] RCON9 = 8'h36;

    // Key words - explicit wires (no arrays!)
    wire [31:0] w0, w1, w2, w3;
    wire [31:0] w4, w5, w6, w7;
    wire [31:0] w8, w9, w10, w11;
    wire [31:0] w12, w13, w14, w15;
    wire [31:0] w16, w17, w18, w19;
    wire [31:0] w20, w21, w22, w23;
    wire [31:0] w24, w25, w26, w27;
    wire [31:0] w28, w29, w30, w31;
    wire [31:0] w32, w33, w34, w35;
    wire [31:0] w36, w37, w38, w39;
    wire [31:0] w40, w41, w42, w43;

    // Initial key words
    assign w0 = key_in[127:96];
    assign w1 = key_in[95:64];
    assign w2 = key_in[63:32];
    assign w3 = key_in[31:0];

    // SubWord outputs
    wire [31:0] sw_out0, sw_out1, sw_out2, sw_out3, sw_out4;
    wire [31:0] sw_out5, sw_out6, sw_out7, sw_out8, sw_out9;
    
    // RotWord: rotate left by 8 bits = {[23:0], [31:24]}
    wire [31:0] rot_w3  = {w3[23:0],  w3[31:24]};
    wire [31:0] rot_w7  = {w7[23:0],  w7[31:24]};
    wire [31:0] rot_w11 = {w11[23:0], w11[31:24]};
    wire [31:0] rot_w15 = {w15[23:0], w15[31:24]};
    wire [31:0] rot_w19 = {w19[23:0], w19[31:24]};
    wire [31:0] rot_w23 = {w23[23:0], w23[31:24]};
    wire [31:0] rot_w27 = {w27[23:0], w27[31:24]};
    wire [31:0] rot_w31 = {w31[23:0], w31[31:24]};
    wire [31:0] rot_w35 = {w35[23:0], w35[31:24]};
    wire [31:0] rot_w39 = {w39[23:0], w39[31:24]};

    // SubWord instances
    aes_sub_word sw_inst0 (.word_in(rot_w3),  .word_out(sw_out0));
    aes_sub_word sw_inst1 (.word_in(rot_w7),  .word_out(sw_out1));
    aes_sub_word sw_inst2 (.word_in(rot_w11), .word_out(sw_out2));
    aes_sub_word sw_inst3 (.word_in(rot_w15), .word_out(sw_out3));
    aes_sub_word sw_inst4 (.word_in(rot_w19), .word_out(sw_out4));
    aes_sub_word sw_inst5 (.word_in(rot_w23), .word_out(sw_out5));
    aes_sub_word sw_inst6 (.word_in(rot_w27), .word_out(sw_out6));
    aes_sub_word sw_inst7 (.word_in(rot_w31), .word_out(sw_out7));
    aes_sub_word sw_inst8 (.word_in(rot_w35), .word_out(sw_out8));
    aes_sub_word sw_inst9 (.word_in(rot_w39), .word_out(sw_out9));

    // Round 1: w4-w7
    assign w4 = w0 ^ sw_out0 ^ {RCON0, 24'h000000};
    assign w5 = w4 ^ w1;
    assign w6 = w5 ^ w2;
    assign w7 = w6 ^ w3;
    
    // Round 2: w8-w11
    assign w8  = w4 ^ sw_out1 ^ {RCON1, 24'h000000};
    assign w9  = w8  ^ w5;
    assign w10 = w9  ^ w6;
    assign w11 = w10 ^ w7;
    
    // Round 3: w12-w15
    assign w12 = w8  ^ sw_out2 ^ {RCON2, 24'h000000};
    assign w13 = w12 ^ w9;
    assign w14 = w13 ^ w10;
    assign w15 = w14 ^ w11;
    
    // Round 4: w16-w19
    assign w16 = w12 ^ sw_out3 ^ {RCON3, 24'h000000};
    assign w17 = w16 ^ w13;
    assign w18 = w17 ^ w14;
    assign w19 = w18 ^ w15;
    
    // Round 5: w20-w23
    assign w20 = w16 ^ sw_out4 ^ {RCON4, 24'h000000};
    assign w21 = w20 ^ w17;
    assign w22 = w21 ^ w18;
    assign w23 = w22 ^ w19;
    
    // Round 6: w24-w27
    assign w24 = w20 ^ sw_out5 ^ {RCON5, 24'h000000};
    assign w25 = w24 ^ w21;
    assign w26 = w25 ^ w22;
    assign w27 = w26 ^ w23;
    
    // Round 7: w28-w31
    assign w28 = w24 ^ sw_out6 ^ {RCON6, 24'h000000};
    assign w29 = w28 ^ w25;
    assign w30 = w29 ^ w26;
    assign w31 = w30 ^ w27;
    
    // Round 8: w32-w35
    assign w32 = w28 ^ sw_out7 ^ {RCON7, 24'h000000};
    assign w33 = w32 ^ w29;
    assign w34 = w33 ^ w30;
    assign w35 = w34 ^ w31;
    
    // Round 9: w36-w39
    assign w36 = w32 ^ sw_out8 ^ {RCON8, 24'h000000};
    assign w37 = w36 ^ w33;
    assign w38 = w37 ^ w34;
    assign w39 = w38 ^ w35;
    
    // Round 10: w40-w43
    assign w40 = w36 ^ sw_out9 ^ {RCON9, 24'h000000};
    assign w41 = w40 ^ w37;
    assign w42 = w41 ^ w38;
    assign w43 = w42 ^ w39;

    // Assign round keys
    assign round_key_0  = {w0,  w1,  w2,  w3};
    assign round_key_1  = {w4,  w5,  w6,  w7};
    assign round_key_2  = {w8,  w9,  w10, w11};
    assign round_key_3  = {w12, w13, w14, w15};
    assign round_key_4  = {w16, w17, w18, w19};
    assign round_key_5  = {w20, w21, w22, w23};
    assign round_key_6  = {w24, w25, w26, w27};
    assign round_key_7  = {w28, w29, w30, w31};
    assign round_key_8  = {w32, w33, w34, w35};
    assign round_key_9  = {w36, w37, w38, w39};
    assign round_key_10 = {w40, w41, w42, w43};

endmodule
