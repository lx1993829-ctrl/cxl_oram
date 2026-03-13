// =============================================================================
// pos_map.v - Position Map + Slot Status BRAM (combined)
// =============================================================================
// Maps logical slot address ? current bucket ID + slot status.
// Combining both into one BRAM saves a memory instance at zero logic cost:
//   - Same index (slot_addr >> 12) for both fields
//   - Same read/write timing
//   - 2 status bits fit in BRAM parity bits (no extra tiles consumed)
//
// Word layout per entry (BUCKET_W+2 bits total):
//   [ status (2 bits) | bucket_id (BUCKET_W bits) ]
//     bit BUCKET_W+1:BUCKET_W    bit BUCKET_W-1:0
//
// status encoding:
//   ST_DUMMY    = 2'b00  - slot is empty / unused
//   ST_VALID    = 2'b01  - slot holds live data in DDR bucket
//   ST_IN_STASH = 2'b10  - slot data currently held in stash
//
// Addressing: slot_addr is 32-bit client address (4K aligned).
//   BRAM index = slot_addr[SLOT_ID_W+11:12]  (drop lower 12-bit page offset)
//
// Latency: 1 cycle read latency (registered output)
`include "oram_params.vh"
module pos_map #(
    parameter N          = `ORAM_N,
    parameter BUCKET_W   = `BUCKET_ID_W,
    parameter SLOT_W     = `SLOT_ID_W,
    parameter ADDR_W     = `SLOT_ADDR_W,
    parameter WORD_W     = BUCKET_W + 2     // bucket_id + 2 status bits
)(
    input  wire                 clk,
    input  wire                 rst_n,
    // Read port - lookup bucket + status for a slot
    input  wire [ADDR_W-1:0]    rd_slot_addr,   // 32-bit logical address
    input  wire                 rd_en,
    output reg  [BUCKET_W-1:0]  rd_bucket,      // registered, valid next cycle
    output reg  [1:0]           rd_status,      // ST_DUMMY / ST_VALID / ST_IN_STASH
    output reg                  rd_valid,       // output valid flag
    // Write port - update bucket + status together in one write
    input  wire [ADDR_W-1:0]    wr_slot_addr,
    input  wire [BUCKET_W-1:0]  wr_bucket,
    input  wire [1:0]           wr_status,
    input  wire                 wr_en
);
    // -------------------------------------------------------------------------
    // BRAM: N entries × (BUCKET_W + 2) bits
    // 20-bit words for default params - fits in BRAM parity bits, no extra tiles
    // -------------------------------------------------------------------------
    (* ram_style = "block" *)
    reg [WORD_W-1:0] mem [0:N-1];
    // Slot index = drop the 12-bit 4K page offset
    wire [SLOT_W-1:0] rd_idx = rd_slot_addr[SLOT_W+11:12];
    wire [SLOT_W-1:0] wr_idx = wr_slot_addr[SLOT_W+11:12];
    // -------------------------------------------------------------------------
    // Read port - unpack word into bucket + status fields
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_bucket <= {BUCKET_W{1'b0}};
            rd_status <= `ST_DUMMY;
            rd_valid  <= 1'b0;
        end else begin
            rd_valid <= rd_en;
            if (rd_en) begin
                rd_bucket <= mem[rd_idx][BUCKET_W-1:0];
                rd_status <= mem[rd_idx][WORD_W-1:BUCKET_W];
            end
        end
    end
    // -------------------------------------------------------------------------
    // Write port - pack status + bucket into one word
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_idx] <= {wr_status, wr_bucket};
    end
    // -------------------------------------------------------------------------
    // Initialization - all slots start as DUMMY, pointing to bucket 0
    // -------------------------------------------------------------------------
    integer i;
    initial begin
        for (i = 0; i < N; i = i + 1)
            mem[i] = {`ST_DUMMY, {BUCKET_W{1'b0}}};
    end
endmodule