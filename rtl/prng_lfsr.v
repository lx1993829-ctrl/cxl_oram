// =============================================================================
// prng_lfsr.v - LFSR-based PRNG for bucket ID generation
// =============================================================================
// 12-bit Fibonacci LFSR (period = 4095) used internally regardless of OUT_W.
// Output is mapped to [0, 2^OUT_W - 2] via (lfsr_reg - 1) truncated to OUT_W.
// For small OUT_W (e.g., 6 for 64 buckets), the lower bits provide sufficient
// randomness from the 12-bit LFSR state.
//
// Primitive polynomial: x^12 + x^11 + x^10 + x^4 + 1
// Taps: bits 11, 10, 9, 3

`include "oram_params.vh"

module prng_lfsr #(
    parameter SEED_VAL = 12'hACE,
    parameter OUT_W    = `BUCKET_ID_W
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire [OUT_W-1:0]     seed,
    input  wire                 seed_valid,
    input  wire                 en,
    output wire [OUT_W-1:0]     rand_out,
    output wire [OUT_W-1:0]     rand_bucket
);

    // Internal LFSR is always 12 bits for maximal period
    localparam LFSR_W = 12;

    reg [LFSR_W-1:0] lfsr_reg;

    // Fibonacci LFSR: taps at bits 11, 10, 9, 3
    wire feedback = lfsr_reg[11] ^ lfsr_reg[10] ^ lfsr_reg[9] ^ lfsr_reg[3];
    wire [LFSR_W-1:0] lfsr_next = {lfsr_reg[LFSR_W-2:0], feedback};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lfsr_reg <= SEED_VAL;
        else if (seed_valid)
            lfsr_reg <= ({{(LFSR_W-OUT_W){1'b0}}, seed} == {LFSR_W{1'b0}})
                        ? {{(LFSR_W-1){1'b0}}, 1'b1}
                        : {{(LFSR_W-OUT_W){1'b0}}, seed};
        else if (en)
            lfsr_reg <= lfsr_next;
    end

    // Truncate to OUT_W bits and subtract 1 to map [1..2^OUT_W] -> [0..2^OUT_W-1]
    assign rand_out    = lfsr_reg[OUT_W-1:0];
    assign rand_bucket = lfsr_reg[OUT_W-1:0] - {{(OUT_W-1){1'b0}}, 1'b1};

endmodule