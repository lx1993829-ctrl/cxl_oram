`timescale 1ns / 1ps
//============================================================================
// GHASH GF(2^128) Multiplier - Fully Combinational
// Based on VHDL reference: ghash_gfmul
//============================================================================

module ghash_gfmul_comb (
    input  wire [127:0] h,
    input  wire [127:0] x,
    output wire [127:0] y
);

    localparam [127:0] R = 128'hE1000000000000000000000000000000;
    
    wire [127:0] v [0:128];
    wire [127:0] z [0:128];
    
    assign v[0] = h;
    assign z[0] = 128'b0;
    
    genvar i;
    generate
        for (i = 0; i < 128; i = i + 1) begin : gf_stage
            wire [127:0] v_shifted = {1'b0, v[i][127:1]};
            assign v[i+1] = v[i][0] ? (v_shifted ^ R) : v_shifted;
            assign z[i+1] = x[127-i] ? (z[i] ^ v[i]) : z[i];
        end
    endgenerate
    
    assign y = z[128];

endmodule

