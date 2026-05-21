`timescale 1ns / 1ps
// =============================================================================
// iv_slot_table.v - Per-Slot IV and TAG Storage
// =============================================================================
// BRAM table that stores the AES-GCM IV and authentication TAG for each
// encrypted slot. Indexed by slot_id (slot_addr[SLOT_W+11:12]).
//
// On encrypt: FSM writes IV and TAG after encryption completes.
// On decrypt: FSM reads IV and TAG before starting decryption.
//
// Entry layout: {tag[127:0], iv[95:0]} = 224 bits per entry
// For 512 slots: 512 × 224 = 114,688 bits ? 3-4 BRAMs
// =============================================================================
`include "oram_params.vh"

module iv_slot_table #(
    parameter N       = `ORAM_N,
    parameter SLOT_W  = `SLOT_ID_W,
    parameter ADDR_W  = `SLOT_ADDR_W,
    parameter IV_W    = 96,
    parameter TAG_W   = 128,
    parameter ENTRY_W = IV_W + TAG_W    // 224 bits
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Read port
    input  wire [ADDR_W-1:0]   rd_slot_addr,
    input  wire                 rd_en,
    output reg  [IV_W-1:0]     rd_iv,
    output reg  [TAG_W-1:0]    rd_tag,
    output reg                  rd_valid,

    // Write port
    input  wire [ADDR_W-1:0]   wr_slot_addr,
    input  wire [IV_W-1:0]     wr_iv,
    input  wire [TAG_W-1:0]    wr_tag,
    input  wire                 wr_en
);

    // BRAM storage
    (* ram_style = "block" *)
    reg [ENTRY_W-1:0] mem [0:N-1];

    // Slot index = drop 12-bit page offset
    localparam IDX_W = $clog2(N);  // 12 bits for N=4096
    wire [IDX_W-1:0] rd_idx = rd_slot_addr[IDX_W+11:12];
    wire [IDX_W-1:0] wr_idx = wr_slot_addr[IDX_W+11:12];

    // BRAM read - NO async reset on data outputs (enables BRAM inference)
    always @(posedge clk) begin
        if (rd_en) begin
            rd_iv  <= mem[rd_idx][IV_W-1:0];
            rd_tag <= mem[rd_idx][ENTRY_W-1:IV_W];
        end
    end

    // Valid flag - can have async reset (just a flip-flop)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en;
    end

    // Write port
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_idx] <= {wr_tag, wr_iv};
    end

    // Initialization
    integer i;
    initial begin
        for (i = 0; i < N; i = i + 1)
            mem[i] = {ENTRY_W{1'b0}};
    end

endmodule