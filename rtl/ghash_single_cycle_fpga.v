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

//============================================================================
// Single-Cycle GHASH Module  
// Based on VHDL reference: gcm_ghash
//============================================================================

module ghash_single_cycle_fpga (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         enable,
    input  wire         start,
    input  wire [127:0] h_key,
    input  wire [127:0] data_in,
    input  wire         data_valid,
    output wire [127:0] ghash_out,
    output wire         ghash_valid,
    output wire         ready
);

    // Registers
    reg [127:0] h_q;
    reg [127:0] y_q;
    reg         h_loaded;
    reg         valid_out;
    
    // Combinational GF multiplication
    wire [127:0] gf_x;
    wire [127:0] gf_y;
    
    assign gf_x = data_in ^ y_q;
    
    ghash_gfmul_comb u_gfmul (
        .h(h_q),
        .x(gf_x),
        .y(gf_y)
    );
    
    // Control signal
    wire do_update = data_valid && h_loaded && !start;
    
    // Sequential logic  
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_q <= 128'b0;
            y_q <= 128'b0;
            h_loaded <= 1'b0;
            valid_out <= 1'b0;
        end else if (enable) begin
            valid_out <= do_update;
            
            if (start) begin
                h_q <= h_key;
                h_loaded <= 1'b1;
                y_q <= 128'b0;
                valid_out <= 1'b0;
            end else if (do_update) begin
                y_q <= gf_y;
            end
        end
    end
    
    assign ghash_out = y_q;
    assign ghash_valid = valid_out;
    assign ready = h_loaded;

endmodule
