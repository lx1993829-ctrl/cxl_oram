// =============================================================================
// prng_lfsr.v - LFSR-based PRNG with XOR folding for bucket ID
// =============================================================================
// 12-bit Fibonacci LFSR (period = 4095).
// Bucket ID = lfsr[11:6] ^ lfsr[5:0]  (XOR fold for independence)
// This gives all 64 values (0-63) with near-uniform distribution and
// breaks the sequential correlation of raw lower bits.
//
// Primitive polynomial: x^12 + x^11 + x^10 + x^4 + 1
// Taps: bits 11, 10, 9, 3
 
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
 
    localparam LFSR_W = 12;
 
    reg [LFSR_W-1:0] lfsr_reg;
 
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
 
    // When OUT_W < LFSR_W: XOR fold upper ^ lower for decorrelation
    // When OUT_W >= LFSR_W: use raw LFSR minus 1
    //   LFSR produces 1..2^LFSR_W-1 (never 0). Subtract 1 to get 0..B-1.
    assign rand_out    = lfsr_reg[OUT_W-1:0];
    generate
        if (OUT_W < LFSR_W)
            assign rand_bucket = lfsr_reg[LFSR_W-1:LFSR_W-OUT_W] ^ lfsr_reg[OUT_W-1:0];
        else
            assign rand_bucket = lfsr_reg[OUT_W-1:0] - {{(OUT_W-1){1'b0}}, 1'b1};
    endgenerate
 
endmodule