`timescale 1ns / 1ps
//============================================================================
// GF(2^128) Multiplier - Karatsuba-Ofman Algorithm
// Single-cycle, fully combinational. Same interface as the original
// bit-serial version but with O(n^1.58) gate complexity and
// O(log n) critical path depth instead of O(n).
//
// GCM reduction polynomial: x^128 + x^7 + x^2 + x + 1
// Reflected bit order: R = 0xE1000000...0 (bit-reversed convention)
//============================================================================

// --- 64×64 → 128 bit GF(2) polynomial multiply (Karatsuba, 1 level) ---
// Splits into 3 × 32×32 multiplies
(* dont_touch = "true" *)
module gf_mul_64 (
    input  wire [63:0] a,
    input  wire [63:0] b,
    output wire [127:0] p
);
    wire [31:0] a_lo = a[31:0];
    wire [31:0] a_hi = a[63:32];
    wire [31:0] b_lo = b[31:0];
    wire [31:0] b_hi = b[63:32];
    wire [31:0] a_xor = a_lo ^ a_hi;
    wire [31:0] b_xor = b_lo ^ b_hi;

    wire [63:0] p_ll, p_hh, p_mm;

    gf_mul_32 u_ll (.a(a_lo), .b(b_lo), .p(p_ll));
    gf_mul_32 u_hh (.a(a_hi), .b(b_hi), .p(p_hh));
    gf_mul_32 u_mm (.a(a_xor), .b(b_xor), .p(p_mm));

    wire [63:0] mid = p_mm ^ p_ll ^ p_hh;

    assign p = {p_hh, 64'b0} ^ {32'b0, mid, 32'b0} ^ {64'b0, p_ll};
endmodule

// --- 32×32 → 64 bit GF(2) polynomial multiply (Karatsuba, 1 level) ---
// Splits into 3 × 16×16 multiplies
(* dont_touch = "true" *)
module gf_mul_32 (
    input  wire [31:0] a,
    input  wire [31:0] b,
    output wire [63:0] p
);
    wire [15:0] a_lo = a[15:0];
    wire [15:0] a_hi = a[31:16];
    wire [15:0] b_lo = b[15:0];
    wire [15:0] b_hi = b[31:16];
    wire [15:0] a_xor = a_lo ^ a_hi;
    wire [15:0] b_xor = b_lo ^ b_hi;

    wire [31:0] p_ll, p_hh, p_mm;

    gf_mul_16 u_ll (.a(a_lo), .b(b_lo), .p(p_ll));
    gf_mul_16 u_hh (.a(a_hi), .b(b_hi), .p(p_hh));
    gf_mul_16 u_mm (.a(a_xor), .b(b_xor), .p(p_mm));

    wire [31:0] mid = p_mm ^ p_ll ^ p_hh;

    assign p = {p_hh, 32'b0} ^ {16'b0, mid, 16'b0} ^ {32'b0, p_ll};
endmodule

// --- 16×16 → 32 bit GF(2) polynomial multiply (schoolbook) ---
// At this size schoolbook is efficient: 16 AND-XOR rows, ~4 LUT levels
(* dont_touch = "true" *)
module gf_mul_16 (
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [31:0] p
);
    wire [30:0] pp [0:15]; // partial products
    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : pp_gen
            assign pp[i] = {15'b0, a} & {31{b[i]}};
        end
    endgenerate

    // XOR tree - synthesizer will balance this automatically
    wire [30:0] sum =
        (pp[0])        ^ (pp[1]  << 1) ^ (pp[2]  << 2)  ^ (pp[3]  << 3) ^
        (pp[4]  << 4)  ^ (pp[5]  << 5) ^ (pp[6]  << 6)  ^ (pp[7]  << 7) ^
        (pp[8]  << 8)  ^ (pp[9]  << 9) ^ (pp[10] << 10) ^ (pp[11] << 11) ^
        (pp[12] << 12) ^ (pp[13] << 13) ^ (pp[14] << 14) ^ (pp[15] << 15);

    assign p = {1'b0, sum};
endmodule

// --- 128×128 → 256 GF(2) polynomial multiply (Karatsuba, top level) ---
(* dont_touch = "true" *)
module gf_pmul_128 (
    input  wire [127:0] a,
    input  wire [127:0] b,
    output wire [255:0] p
);
    wire [63:0] a_lo = a[63:0];
    wire [63:0] a_hi = a[127:64];
    wire [63:0] b_lo = b[63:0];
    wire [63:0] b_hi = b[127:64];
    wire [63:0] a_xor = a_lo ^ a_hi;
    wire [63:0] b_xor = b_lo ^ b_hi;

    wire [127:0] p_ll, p_hh, p_mm;

    gf_mul_64 u_ll (.a(a_lo), .b(b_lo), .p(p_ll));
    gf_mul_64 u_hh (.a(a_hi), .b(b_hi), .p(p_hh));
    gf_mul_64 u_mm (.a(a_xor), .b(b_xor), .p(p_mm));

    wire [127:0] mid = p_mm ^ p_ll ^ p_hh;

    assign p = {p_hh, 128'b0} ^ {64'b0, mid, 64'b0} ^ {128'b0, p_ll};
endmodule

// --- GF(2^128) reduction in STANDARD bit order ---
// P = x^128 + x^7 + x^2 + x + 1
// Reduce 256-bit product c[255:0] to 128 bits.
// Standard order: bit N represents coefficient of x^N.
// c[255:128] needs to be reduced: each x^(128+k) term becomes
//   x^(128+k) mod P = x^k * (x^7 + x^2 + x + 1)
// So c_hi[k] contributes x^(k+7) + x^(k+2) + x^(k+1) + x^(k) to the result.

module gf_reduce_128 (
    input  wire [255:0] c,
    output wire [127:0] r
);
    wire [127:0] c_lo = c[127:0];
    wire [127:0] c_hi = c[255:128];

    // Standard-order reduction: x^128 mod P = x^7 + x^2 + x + 1
    // For each set bit k in c_hi, contribute x^k + x^(k+1) + x^(k+2) + x^(k+7)
    // Use 135-bit accumulator to handle overflow (max k=127, k+7=134)
    reg [134:0] redux_wide;
    integer i;
    always @(*) begin
        redux_wide = 135'b0;
        for (i = 0; i < 128; i = i + 1) begin
            if (c_hi[i]) begin
                redux_wide = redux_wide ^ (135'b1 << i);
                redux_wide = redux_wide ^ (135'b1 << (i+1));
                redux_wide = redux_wide ^ (135'b1 << (i+2));
                redux_wide = redux_wide ^ (135'b1 << (i+7));
            end
        end
    end

    // Bits 128-134 of redux_wide need re-reduction
    // Each overflow bit j (0..6) contributes x^j + x^(j+1) + x^(j+2) + x^(j+7)
    // j+7 max = 13, fits in 128 bits
    wire [6:0] ov = redux_wide[134:128];
    reg [127:0] ov_fix;
    integer j;
    always @(*) begin
        ov_fix = 128'b0;
        for (j = 0; j < 7; j = j + 1) begin
            if (ov[j]) begin
                ov_fix = ov_fix ^ (128'b1 << j);
                ov_fix = ov_fix ^ (128'b1 << (j+1));
                ov_fix = ov_fix ^ (128'b1 << (j+2));
                ov_fix = ov_fix ^ (128'b1 << (j+7));
            end
        end
    end

    assign r = c_lo ^ redux_wide[127:0] ^ ov_fix;
endmodule

// --- Top-level: GF(2^128) multiply with reduction (KOA-based) ---
// The KOA polynomial multiply works in standard bit order.
// GHASH uses reflected bit order. So we reflect inputs before
// multiply, and reflect the output after reduction.
module ghash_gfmul_comb (
    input  wire [127:0] h,
    input  wire [127:0] x,
    output wire [127:0] y
);
    // Bit-reflect function (bit 0 <-> bit 127, etc.)
    function [127:0] reflect128;
        input [127:0] d;
        integer j;
        begin
            for (j = 0; j < 128; j = j + 1)
                reflect128[j] = d[127-j];
        end
    endfunction

    wire [127:0] h_r = reflect128(h);
    wire [127:0] x_r = reflect128(x);

    wire [255:0] product;

    gf_pmul_128 u_pmul (
        .a(h_r), .b(x_r), .p(product)
    );

    // Standard (non-reflected) reduction: P = x^128 + x^7 + x^2 + x + 1
    // After reduction in standard order, reflect back to GHASH convention.
    wire [127:0] reduced;

    gf_reduce_128 u_reduce (
        .c(product), .r(reduced)
    );

    assign y = reflect128(reduced);

endmodule

//============================================================================
// Single-Cycle GHASH Module - same as original, uses KOA multiplier
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

    reg [127:0] h_q;
    reg [127:0] y_q;
    reg         h_loaded;

    // Bit-reflect function
    function [127:0] reflect128;
        input [127:0] d;
        integer j;
        begin
            for (j = 0; j < 128; j = j + 1)
                reflect128[j] = d[127-j];
        end
    endfunction

    // ---- Stage 1: XOR + reflect + polynomial multiply ----
    wire [127:0] gf_x = data_in ^ y_q;
    wire [127:0] h_r  = reflect128(h_q);
    wire [127:0] x_r  = reflect128(gf_x);
    wire [255:0] product;

    gf_pmul_128 u_pmul (
        .a(h_r), .b(x_r), .p(product)
    );

    // Pipeline register: capture 256-bit product
    reg [255:0] product_reg;
    reg         stage2_valid;

    wire do_update = data_valid && h_loaded && !start;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            product_reg  <= 256'b0;
            stage2_valid <= 1'b0;
        end else if (enable) begin
            stage2_valid <= do_update;
            if (do_update)
                product_reg <= product;
        end
    end

    // ---- Stage 2: reduction + reflect back ----
    wire [127:0] reduced;
    gf_reduce_128 u_reduce (
        .c(product_reg), .r(reduced)
    );

    wire [127:0] gf_y = reflect128(reduced);

    // Update y_q from stage 2 output
    reg valid_out;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_q       <= 128'b0;
            y_q       <= 128'b0;
            h_loaded  <= 1'b0;
            valid_out <= 1'b0;
        end else if (enable) begin
            valid_out <= stage2_valid;

            if (start) begin
                h_q      <= h_key;
                h_loaded <= 1'b1;
                y_q      <= 128'b0;
                valid_out <= 1'b0;
            end else if (stage2_valid) begin
                y_q <= gf_y;
            end
        end
    end

    assign ghash_out   = y_q;
    assign ghash_valid = valid_out;
    assign ready       = h_loaded && !stage2_valid && !do_update;

endmodule
