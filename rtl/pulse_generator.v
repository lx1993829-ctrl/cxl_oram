`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/10/2026 11:10:29 AM
// Design Name: 
// Module Name: pulse_generator
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module pulse_generator_1 (
    input  wire clk,      // System clock
    input  wire rst_n,    // Active-low synchronous reset
    input  wire trigger,  // Input signal (can be multiple cycles long)
    output wire pulse_out // High for exactly 1 clock cycle
);

    reg trigger_d1; // Delayed version of the trigger

    always @(posedge clk) begin
        if (!rst_n) begin
            trigger_d1 <= 1'b0;
        end else begin
            // Shift the trigger signal into the register
            trigger_d1 <= trigger;
        end
    end

    // The pulse is high ONLY when the current trigger is high 
    // AND the previous cycle (d1) was low.
    assign pulse_out = trigger & (~trigger_d1);

endmodule
