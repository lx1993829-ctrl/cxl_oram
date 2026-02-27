`timescale 1ns / 1ps
//============================================================================
// AES S-Box Module - FPGA Synthesizable
// Implements AES S-Box as a lookup table using case statement
//============================================================================

module aes_sbox (
    input  wire [7:0] data_in,
    output reg  [7:0] data_out
);

    always @(*) begin
        case(data_in)
            8'h00: data_out = 8'h63; 8'h01: data_out = 8'h7c; 8'h02: data_out = 8'h77; 8'h03: data_out = 8'h7b;
            8'h04: data_out = 8'hf2; 8'h05: data_out = 8'h6b; 8'h06: data_out = 8'h6f; 8'h07: data_out = 8'hc5;
            8'h08: data_out = 8'h30; 8'h09: data_out = 8'h01; 8'h0a: data_out = 8'h67; 8'h0b: data_out = 8'h2b;
            8'h0c: data_out = 8'hfe; 8'h0d: data_out = 8'hd7; 8'h0e: data_out = 8'hab; 8'h0f: data_out = 8'h76;
            8'h10: data_out = 8'hca; 8'h11: data_out = 8'h82; 8'h12: data_out = 8'hc9; 8'h13: data_out = 8'h7d;
            8'h14: data_out = 8'hfa; 8'h15: data_out = 8'h59; 8'h16: data_out = 8'h47; 8'h17: data_out = 8'hf0;
            8'h18: data_out = 8'had; 8'h19: data_out = 8'hd4; 8'h1a: data_out = 8'ha2; 8'h1b: data_out = 8'haf;
            8'h1c: data_out = 8'h9c; 8'h1d: data_out = 8'ha4; 8'h1e: data_out = 8'h72; 8'h1f: data_out = 8'hc0;
            8'h20: data_out = 8'hb7; 8'h21: data_out = 8'hfd; 8'h22: data_out = 8'h93; 8'h23: data_out = 8'h26;
            8'h24: data_out = 8'h36; 8'h25: data_out = 8'h3f; 8'h26: data_out = 8'hf7; 8'h27: data_out = 8'hcc;
            8'h28: data_out = 8'h34; 8'h29: data_out = 8'ha5; 8'h2a: data_out = 8'he5; 8'h2b: data_out = 8'hf1;
            8'h2c: data_out = 8'h71; 8'h2d: data_out = 8'hd8; 8'h2e: data_out = 8'h31; 8'h2f: data_out = 8'h15;
            8'h30: data_out = 8'h04; 8'h31: data_out = 8'hc7; 8'h32: data_out = 8'h23; 8'h33: data_out = 8'hc3;
            8'h34: data_out = 8'h18; 8'h35: data_out = 8'h96; 8'h36: data_out = 8'h05; 8'h37: data_out = 8'h9a;
            8'h38: data_out = 8'h07; 8'h39: data_out = 8'h12; 8'h3a: data_out = 8'h80; 8'h3b: data_out = 8'he2;
            8'h3c: data_out = 8'heb; 8'h3d: data_out = 8'h27; 8'h3e: data_out = 8'hb2; 8'h3f: data_out = 8'h75;
            8'h40: data_out = 8'h09; 8'h41: data_out = 8'h83; 8'h42: data_out = 8'h2c; 8'h43: data_out = 8'h1a;
            8'h44: data_out = 8'h1b; 8'h45: data_out = 8'h6e; 8'h46: data_out = 8'h5a; 8'h47: data_out = 8'ha0;
            8'h48: data_out = 8'h52; 8'h49: data_out = 8'h3b; 8'h4a: data_out = 8'hd6; 8'h4b: data_out = 8'hb3;
            8'h4c: data_out = 8'h29; 8'h4d: data_out = 8'he3; 8'h4e: data_out = 8'h2f; 8'h4f: data_out = 8'h84;
            8'h50: data_out = 8'h53; 8'h51: data_out = 8'hd1; 8'h52: data_out = 8'h00; 8'h53: data_out = 8'hed;
            8'h54: data_out = 8'h20; 8'h55: data_out = 8'hfc; 8'h56: data_out = 8'hb1; 8'h57: data_out = 8'h5b;
            8'h58: data_out = 8'h6a; 8'h59: data_out = 8'hcb; 8'h5a: data_out = 8'hbe; 8'h5b: data_out = 8'h39;
            8'h5c: data_out = 8'h4a; 8'h5d: data_out = 8'h4c; 8'h5e: data_out = 8'h58; 8'h5f: data_out = 8'hcf;
            8'h60: data_out = 8'hd0; 8'h61: data_out = 8'hef; 8'h62: data_out = 8'haa; 8'h63: data_out = 8'hfb;
            8'h64: data_out = 8'h43; 8'h65: data_out = 8'h4d; 8'h66: data_out = 8'h33; 8'h67: data_out = 8'h85;
            8'h68: data_out = 8'h45; 8'h69: data_out = 8'hf9; 8'h6a: data_out = 8'h02; 8'h6b: data_out = 8'h7f;
            8'h6c: data_out = 8'h50; 8'h6d: data_out = 8'h3c; 8'h6e: data_out = 8'h9f; 8'h6f: data_out = 8'ha8;
            8'h70: data_out = 8'h51; 8'h71: data_out = 8'ha3; 8'h72: data_out = 8'h40; 8'h73: data_out = 8'h8f;
            8'h74: data_out = 8'h92; 8'h75: data_out = 8'h9d; 8'h76: data_out = 8'h38; 8'h77: data_out = 8'hf5;
            8'h78: data_out = 8'hbc; 8'h79: data_out = 8'hb6; 8'h7a: data_out = 8'hda; 8'h7b: data_out = 8'h21;
            8'h7c: data_out = 8'h10; 8'h7d: data_out = 8'hff; 8'h7e: data_out = 8'hf3; 8'h7f: data_out = 8'hd2;
            8'h80: data_out = 8'hcd; 8'h81: data_out = 8'h0c; 8'h82: data_out = 8'h13; 8'h83: data_out = 8'hec;
            8'h84: data_out = 8'h5f; 8'h85: data_out = 8'h97; 8'h86: data_out = 8'h44; 8'h87: data_out = 8'h17;
            8'h88: data_out = 8'hc4; 8'h89: data_out = 8'ha7; 8'h8a: data_out = 8'h7e; 8'h8b: data_out = 8'h3d;
            8'h8c: data_out = 8'h64; 8'h8d: data_out = 8'h5d; 8'h8e: data_out = 8'h19; 8'h8f: data_out = 8'h73;
            8'h90: data_out = 8'h60; 8'h91: data_out = 8'h81; 8'h92: data_out = 8'h4f; 8'h93: data_out = 8'hdc;
            8'h94: data_out = 8'h22; 8'h95: data_out = 8'h2a; 8'h96: data_out = 8'h90; 8'h97: data_out = 8'h88;
            8'h98: data_out = 8'h46; 8'h99: data_out = 8'hee; 8'h9a: data_out = 8'hb8; 8'h9b: data_out = 8'h14;
            8'h9c: data_out = 8'hde; 8'h9d: data_out = 8'h5e; 8'h9e: data_out = 8'h0b; 8'h9f: data_out = 8'hdb;
            8'ha0: data_out = 8'he0; 8'ha1: data_out = 8'h32; 8'ha2: data_out = 8'h3a; 8'ha3: data_out = 8'h0a;
            8'ha4: data_out = 8'h49; 8'ha5: data_out = 8'h06; 8'ha6: data_out = 8'h24; 8'ha7: data_out = 8'h5c;
            8'ha8: data_out = 8'hc2; 8'ha9: data_out = 8'hd3; 8'haa: data_out = 8'hac; 8'hab: data_out = 8'h62;
            8'hac: data_out = 8'h91; 8'had: data_out = 8'h95; 8'hae: data_out = 8'he4; 8'haf: data_out = 8'h79;
            8'hb0: data_out = 8'he7; 8'hb1: data_out = 8'hc8; 8'hb2: data_out = 8'h37; 8'hb3: data_out = 8'h6d;
            8'hb4: data_out = 8'h8d; 8'hb5: data_out = 8'hd5; 8'hb6: data_out = 8'h4e; 8'hb7: data_out = 8'ha9;
            8'hb8: data_out = 8'h6c; 8'hb9: data_out = 8'h56; 8'hba: data_out = 8'hf4; 8'hbb: data_out = 8'hea;
            8'hbc: data_out = 8'h65; 8'hbd: data_out = 8'h7a; 8'hbe: data_out = 8'hae; 8'hbf: data_out = 8'h08;
            8'hc0: data_out = 8'hba; 8'hc1: data_out = 8'h78; 8'hc2: data_out = 8'h25; 8'hc3: data_out = 8'h2e;
            8'hc4: data_out = 8'h1c; 8'hc5: data_out = 8'ha6; 8'hc6: data_out = 8'hb4; 8'hc7: data_out = 8'hc6;
            8'hc8: data_out = 8'he8; 8'hc9: data_out = 8'hdd; 8'hca: data_out = 8'h74; 8'hcb: data_out = 8'h1f;
            8'hcc: data_out = 8'h4b; 8'hcd: data_out = 8'hbd; 8'hce: data_out = 8'h8b; 8'hcf: data_out = 8'h8a;
            8'hd0: data_out = 8'h70; 8'hd1: data_out = 8'h3e; 8'hd2: data_out = 8'hb5; 8'hd3: data_out = 8'h66;
            8'hd4: data_out = 8'h48; 8'hd5: data_out = 8'h03; 8'hd6: data_out = 8'hf6; 8'hd7: data_out = 8'h0e;
            8'hd8: data_out = 8'h61; 8'hd9: data_out = 8'h35; 8'hda: data_out = 8'h57; 8'hdb: data_out = 8'hb9;
            8'hdc: data_out = 8'h86; 8'hdd: data_out = 8'hc1; 8'hde: data_out = 8'h1d; 8'hdf: data_out = 8'h9e;
            8'he0: data_out = 8'he1; 8'he1: data_out = 8'hf8; 8'he2: data_out = 8'h98; 8'he3: data_out = 8'h11;
            8'he4: data_out = 8'h69; 8'he5: data_out = 8'hd9; 8'he6: data_out = 8'h8e; 8'he7: data_out = 8'h94;
            8'he8: data_out = 8'h9b; 8'he9: data_out = 8'h1e; 8'hea: data_out = 8'h87; 8'heb: data_out = 8'he9;
            8'hec: data_out = 8'hce; 8'hed: data_out = 8'h55; 8'hee: data_out = 8'h28; 8'hef: data_out = 8'hdf;
            8'hf0: data_out = 8'h8c; 8'hf1: data_out = 8'ha1; 8'hf2: data_out = 8'h89; 8'hf3: data_out = 8'h0d;
            8'hf4: data_out = 8'hbf; 8'hf5: data_out = 8'he6; 8'hf6: data_out = 8'h42; 8'hf7: data_out = 8'h68;
            8'hf8: data_out = 8'h41; 8'hf9: data_out = 8'h99; 8'hfa: data_out = 8'h2d; 8'hfb: data_out = 8'h0f;
            8'hfc: data_out = 8'hb0; 8'hfd: data_out = 8'h54; 8'hfe: data_out = 8'hbb; 8'hff: data_out = 8'h16;
        endcase
    end

endmodule

//============================================================================
// GF(2^8) Multiply by 2 (xtime) - FPGA Synthesizable
//============================================================================
module gf_mult2 (
    input  wire [7:0] data_in,
    output wire [7:0] data_out
);
    assign data_out = {data_in[6:0], 1'b0} ^ (8'h1b & {8{data_in[7]}});
endmodule

//============================================================================
// GF(2^8) Multiply by 3 - FPGA Synthesizable
//============================================================================
module gf_mult3 (
    input  wire [7:0] data_in,
    output wire [7:0] data_out
);
    wire [7:0] mult2;
    assign mult2 = {data_in[6:0], 1'b0} ^ (8'h1b & {8{data_in[7]}});
    assign data_out = mult2 ^ data_in;
endmodule

//============================================================================
// SubBytes - 16 parallel S-Boxes (explicit instantiation for FPGA)
//============================================================================
module aes_sub_bytes (
    input  wire [127:0] state_in,
    output wire [127:0] state_out
);
    aes_sbox sb0  (.data_in(state_in[127:120]), .data_out(state_out[127:120]));
    aes_sbox sb1  (.data_in(state_in[119:112]), .data_out(state_out[119:112]));
    aes_sbox sb2  (.data_in(state_in[111:104]), .data_out(state_out[111:104]));
    aes_sbox sb3  (.data_in(state_in[103:96]),  .data_out(state_out[103:96]));
    aes_sbox sb4  (.data_in(state_in[95:88]),   .data_out(state_out[95:88]));
    aes_sbox sb5  (.data_in(state_in[87:80]),   .data_out(state_out[87:80]));
    aes_sbox sb6  (.data_in(state_in[79:72]),   .data_out(state_out[79:72]));
    aes_sbox sb7  (.data_in(state_in[71:64]),   .data_out(state_out[71:64]));
    aes_sbox sb8  (.data_in(state_in[63:56]),   .data_out(state_out[63:56]));
    aes_sbox sb9  (.data_in(state_in[55:48]),   .data_out(state_out[55:48]));
    aes_sbox sb10 (.data_in(state_in[47:40]),   .data_out(state_out[47:40]));
    aes_sbox sb11 (.data_in(state_in[39:32]),   .data_out(state_out[39:32]));
    aes_sbox sb12 (.data_in(state_in[31:24]),   .data_out(state_out[31:24]));
    aes_sbox sb13 (.data_in(state_in[23:16]),   .data_out(state_out[23:16]));
    aes_sbox sb14 (.data_in(state_in[15:8]),    .data_out(state_out[15:8]));
    aes_sbox sb15 (.data_in(state_in[7:0]),     .data_out(state_out[7:0]));
endmodule

//============================================================================
// ShiftRows - Pure wiring, no logic
//============================================================================
module aes_shift_rows (
    input  wire [127:0] state_in,
    output wire [127:0] state_out
);
    // State layout (column-major):
    // s0  s4  s8  s12
    // s1  s5  s9  s13
    // s2  s6  s10 s14
    // s3  s7  s11 s15
    
    // Row 0: no shift
    // Row 1: shift left by 1
    // Row 2: shift left by 2
    // Row 3: shift left by 3
    
    wire [7:0] s [0:15];
    
    // Unpack
    assign s[0]  = state_in[127:120];
    assign s[1]  = state_in[119:112];
    assign s[2]  = state_in[111:104];
    assign s[3]  = state_in[103:96];
    assign s[4]  = state_in[95:88];
    assign s[5]  = state_in[87:80];
    assign s[6]  = state_in[79:72];
    assign s[7]  = state_in[71:64];
    assign s[8]  = state_in[63:56];
    assign s[9]  = state_in[55:48];
    assign s[10] = state_in[47:40];
    assign s[11] = state_in[39:32];
    assign s[12] = state_in[31:24];
    assign s[13] = state_in[23:16];
    assign s[14] = state_in[15:8];
    assign s[15] = state_in[7:0];
    
    // Pack with shift rows applied
    assign state_out = {s[0],  s[5],  s[10], s[15],
                        s[4],  s[9],  s[14], s[3],
                        s[8],  s[13], s[2],  s[7],
                        s[12], s[1],  s[6],  s[11]};
endmodule

//============================================================================
// MixColumns - One column
//============================================================================
module aes_mix_column (
    input  wire [31:0] col_in,
    output wire [31:0] col_out
);
    wire [7:0] s0, s1, s2, s3;
    wire [7:0] t0, t1, t2, t3;
    wire [7:0] s0_x2, s1_x2, s2_x2, s3_x2;
    wire [7:0] s0_x3, s1_x3, s2_x3, s3_x3;
    
    assign s0 = col_in[31:24];
    assign s1 = col_in[23:16];
    assign s2 = col_in[15:8];
    assign s3 = col_in[7:0];
    
    // Multiply by 2
    gf_mult2 m2_0 (.data_in(s0), .data_out(s0_x2));
    gf_mult2 m2_1 (.data_in(s1), .data_out(s1_x2));
    gf_mult2 m2_2 (.data_in(s2), .data_out(s2_x2));
    gf_mult2 m2_3 (.data_in(s3), .data_out(s3_x2));
    
    // Multiply by 3
    gf_mult3 m3_0 (.data_in(s0), .data_out(s0_x3));
    gf_mult3 m3_1 (.data_in(s1), .data_out(s1_x3));
    gf_mult3 m3_2 (.data_in(s2), .data_out(s2_x3));
    gf_mult3 m3_3 (.data_in(s3), .data_out(s3_x3));
    
    // MixColumn matrix multiplication
    assign t0 = s0_x2 ^ s1_x3 ^ s2    ^ s3;
    assign t1 = s0    ^ s1_x2 ^ s2_x3 ^ s3;
    assign t2 = s0    ^ s1    ^ s2_x2 ^ s3_x3;
    assign t3 = s0_x3 ^ s1    ^ s2    ^ s3_x2;
    
    assign col_out = {t0, t1, t2, t3};
endmodule

//============================================================================
// MixColumns - Full 128-bit state
//============================================================================
module aes_mix_columns (
    input  wire [127:0] state_in,
    output wire [127:0] state_out
);
    aes_mix_column col0 (.col_in(state_in[127:96]), .col_out(state_out[127:96]));
    aes_mix_column col1 (.col_in(state_in[95:64]),  .col_out(state_out[95:64]));
    aes_mix_column col2 (.col_in(state_in[63:32]),  .col_out(state_out[63:32]));
    aes_mix_column col3 (.col_in(state_in[31:0]),   .col_out(state_out[31:0]));
endmodule

//============================================================================
// AddRoundKey
//============================================================================
module aes_add_round_key (
    input  wire [127:0] state_in,
    input  wire [127:0] round_key,
    output wire [127:0] state_out
);
    assign state_out = state_in ^ round_key;
endmodule

//============================================================================
// SubWord for key expansion
//============================================================================
module aes_sub_word (
    input  wire [31:0] word_in,
    output wire [31:0] word_out
);
    aes_sbox sb0 (.data_in(word_in[31:24]), .data_out(word_out[31:24]));
    aes_sbox sb1 (.data_in(word_in[23:16]), .data_out(word_out[23:16]));
    aes_sbox sb2 (.data_in(word_in[15:8]),  .data_out(word_out[15:8]));
    aes_sbox sb3 (.data_in(word_in[7:0]),   .data_out(word_out[7:0]));
endmodule

