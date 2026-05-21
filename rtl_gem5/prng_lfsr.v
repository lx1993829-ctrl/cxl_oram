// =============================================================================
// prng_lfsr.v - LFSR-based PRNG with XOR folding for bucket ID
// =============================================================================
// 16-bit Fibonacci LFSR (period = 65535). Supports BUCKET_ID_W up to 16.
// Primitive polynomial: x^16 + x^14 + x^13 + x^11 + 1
// Taps: bits 15, 13, 12, 10
//
// When OUT_W < LFSR_W: XOR fold upper ^ lower for decorrelation
// When OUT_W >= LFSR_W: use raw LFSR minus 1 (maps 1..2^W-1 to 0..2^W-2)
// =============================================================================
`include "oram_params.vh"

module prng_lfsr #(
    parameter SEED_VAL = (1 << (OUT_W - 1)) | 1,  // non-zero default for any width
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

    // LFSR width matches OUT_W → subtract-1 path gives perfect uniformity.
    // Requires B = 2^LFSR_W - 1 (set in oram_params.vh).
    localparam LFSR_W = OUT_W;

    reg [LFSR_W-1:0] lfsr_reg;

    // Maximal-length feedback taps per LFSR width (primitive polynomials)
    wire feedback;
    generate
        if (LFSR_W == 9)       // x^9 + x^4 + 1
            assign feedback = lfsr_reg[8] ^ lfsr_reg[3];
        else if (LFSR_W == 10) // x^10 + x^3 + 1
            assign feedback = lfsr_reg[9] ^ lfsr_reg[2];
        else if (LFSR_W == 12) // x^12 + x^11 + x^10 + x^4 + 1
            assign feedback = lfsr_reg[11] ^ lfsr_reg[10] ^ lfsr_reg[9] ^ lfsr_reg[3];
        else if (LFSR_W == 13) // x^13 + x^4 + x^3 + x + 1
            assign feedback = lfsr_reg[12] ^ lfsr_reg[3] ^ lfsr_reg[2] ^ lfsr_reg[0];
        else if (LFSR_W == 16) // x^16 + x^14 + x^13 + x^11 + 1
            assign feedback = lfsr_reg[15] ^ lfsr_reg[13] ^ lfsr_reg[12] ^ lfsr_reg[10];
        else                    // fallback: XNOR of top two bits (not maximal — add your poly)
            assign feedback = lfsr_reg[LFSR_W-1] ^ lfsr_reg[LFSR_W-2];
    endgenerate
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

    assign rand_out    = lfsr_reg[OUT_W-1:0];
    generate
        if (OUT_W < LFSR_W)
            assign rand_bucket = lfsr_reg[LFSR_W-1:LFSR_W-OUT_W] ^ lfsr_reg[OUT_W-1:0];
        else
            assign rand_bucket = lfsr_reg[OUT_W-1:0] - {{(OUT_W-1){1'b0}}, 1'b1};
    endgenerate

endmodule