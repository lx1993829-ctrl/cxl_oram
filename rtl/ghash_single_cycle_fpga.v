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
