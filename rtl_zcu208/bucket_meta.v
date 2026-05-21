// bucket_meta.v - Bucket Metadata BRAM
// =============================================================================
// For each bucket stores:
//   slot_list[Z]  : which logical slot numbers occupy each position
//   fill_count    : how many valid slots are in this bucket (0..Z)
//
// slot_list is a flat array of Z entries of SLOT_ID_W bits each,
// stored as one wide word per bucket:
//   word width = Z * SLOT_ID_W bits
//
// Supports:
//   - Full bucket read (returns slot_list + fill_count)
//   - Full bucket write (update after compaction/eviction)
//   - Fill count only read (for quick checks)
//
// Latency: 1 cycle (registered BRAM output)
`include "oram_params.vh"
module bucket_meta #(
    parameter B          = `ORAM_B,
    parameter Z          = `ORAM_Z,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter SLOT_W     = `SLOT_ID_W,
    parameter FILL_W     = `FILL_CNT_W,
    parameter POS_W      = `POS_IN_BKT_W,
    // Total word width per bucket entry
    parameter ENTRY_W    = Z * SLOT_W + FILL_W   // slot_list + fill_count
)(
    input  wire                 clk,
    input  wire                 rst_n,
    // Read port
    input  wire [BUCKET_W-1:0]  rd_bucket,
    input  wire                 rd_en,
    output reg  [Z*SLOT_W-1:0]  rd_slot_list,    // flat: [slot[Z-1],...,slot[0]]
    output reg  [FILL_W-1:0]    rd_fill_count,
    output reg                  rd_valid,
    // Write port - full entry update
    input  wire [BUCKET_W-1:0]  wr_bucket,
    input  wire [Z*SLOT_W-1:0]  wr_slot_list,
    input  wire [FILL_W-1:0]    wr_fill_count,
    input  wire                 wr_en
);
    // -------------------------------------------------------------------------
    // BRAM: B entries   ENTRY_W bits
    // -------------------------------------------------------------------------
    (* ram_style = "block" *)
    reg [ENTRY_W-1:0] mem [0:B-1];
    // Pack/unpack helpers
    // mem layout: [FILL_W-1:0]=fill_count, [ENTRY_W-1:FILL_W]=slot_list
    wire [ENTRY_W-1:0] wr_word = {wr_slot_list, wr_fill_count};
    // -------------------------------------------------------------------------
    // Read port - NO async reset on data outputs (enables BRAM inference)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rd_en) begin
            rd_fill_count <= mem[rd_bucket][FILL_W-1:0];
            rd_slot_list  <= mem[rd_bucket][ENTRY_W-1:FILL_W];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_valid <= 1'b0;
        else
            rd_valid <= rd_en;
    end
    // -------------------------------------------------------------------------
    // Write port
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (wr_en) begin
            mem[wr_bucket] <= wr_word;
        end
    end
    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------
    integer i;
    initial begin
        for (i = 0; i < B; i = i + 1)
            mem[i] = {ENTRY_W{1'b0}};
    end
endmodule