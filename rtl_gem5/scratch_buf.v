
/*
// =============================================================================
// scratch_buf.v - Scratch Buffer (One Bucket = Z × 4K)
// =============================================================================
// Holds one full bucket during processing.
// Written via AXI stream as DDR burst arrives.
// Read by FSM to return data to client or write into stash.
// Written by FSM during stash eviction.
//
// Memory size: Z * 4096 bytes = Z * 128 AXI beats (256-bit bus)
// Addressed by: {slot_pos[POS_W-1:0], beat[BEAT_CNT_W-1:0]}
//
// Two port pairs:
//   Port A: stream_in (write) + stream_out (read)  - tied to AXI burst
//   Port B: FSM write + FSM read                   - FSM-driven address
//
// For Vivado BRAM inference: both write ports are in a single always block.
// Write priority: stream_in > FSM write (they never conflict in practice
// since stream_in is active during DDR_READ and FSM write during EVICT).
// =============================================================================
`include "oram_params.vh"

module scratch_buf #(
    parameter Z            = `ORAM_Z,
    parameter POS_W        = `POS_IN_BKT_W,
    parameter AXI_DATA_W   = `AXI_DATA_W,
    parameter BEATS        = `BEATS_PER_BLOCK,
    parameter BEAT_W       = `BEAT_CNT_W,
    parameter TOTAL_BEATS  = Z * BEATS         // 8 * 128 = 1024
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // --- Stream-in port (from AXI read data channel, DDR -> scratch) ---
    input  wire [AXI_DATA_W-1:0]    stream_in_data,
    input  wire                     stream_in_valid,
    input  wire [9:0]               stream_in_beat_addr,  // 0..TOTAL_BEATS-1
    output wire                     stream_in_ready,      // always ready (BRAM)

    // --- FSM read port (FSM reads a specific slot) ---
    input  wire [9:0]               fsm_rd_addr,          // {slot_pos, beat}
    input  wire                     fsm_rd_en,
    output reg  [AXI_DATA_W-1:0]    fsm_rd_data,
    output reg                      fsm_rd_valid,

    // --- FSM write port (FSM writes evicted stash slot into scratch) ---
    input  wire [9:0]               fsm_wr_addr,          // {slot_pos, beat}
    input  wire [AXI_DATA_W-1:0]    fsm_wr_data,
    input  wire                     fsm_wr_en,

    // --- Stream-out port (to AXI write data channel, scratch -> DDR) ---
    input  wire [9:0]               stream_out_beat_addr, // driven by AXI master
    input  wire                     stream_out_rd_en,
    output reg  [AXI_DATA_W-1:0]    stream_out_data,
    output reg                      stream_out_valid
);

    // -------------------------------------------------------------------------
    // Simple dual-port BRAM with muxed write port
    //
    // stream_in and fsm_wr NEVER fire simultaneously (different ORAM phases):
    //   stream_in_valid: active during S_DDR_READ
    //   fsm_wr_en:       active during eviction/re-encrypt
    //
    // Single write port avoids Vivado TDP inference issues.
    // Two independent read ports (stream_out + fsm_rd).
    // -------------------------------------------------------------------------
    (* ram_style = "block" *)
    reg [AXI_DATA_W-1:0] mem [0:TOTAL_BEATS-1];

    // Muxed write port
    wire        wr_en   = stream_in_valid | fsm_wr_en;
    wire [9:0]  wr_addr = stream_in_valid ? stream_in_beat_addr : fsm_wr_addr;
    wire [AXI_DATA_W-1:0] wr_data = stream_in_valid ? stream_in_data : fsm_wr_data;

    // Single write + Port A read (stream_out)
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        if (stream_out_rd_en)
            stream_out_data <= mem[stream_out_beat_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stream_out_valid <= 1'b0;
        else
            stream_out_valid <= stream_out_rd_en;
    end

    // Port B read (FSM)
    always @(posedge clk) begin
        if (fsm_rd_en)
            fsm_rd_data <= mem[fsm_rd_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fsm_rd_valid <= 1'b0;
        else
            fsm_rd_valid <= fsm_rd_en;
    end

    assign stream_in_ready = 1'b1; // BRAM always accepts

endmodule

*/

// =============================================================================
// scratch_buf.v - Scratch Buffer (One Bucket = Z × 4K)
// =============================================================================
// Holds one full bucket during processing.
// Written via AXI stream as DDR burst arrives.
// Read by FSM to return data to client or write into stash.
// Written by FSM during stash eviction.
//
// Memory size: Z * 4096 bytes = Z * 128 AXI beats (256-bit bus)
// Addressed by: {slot_pos[POS_W-1:0], beat[BEAT_CNT_W-1:0]}
//
// Two port pairs:
//   Port A: stream_in (write) + stream_out (read)  - tied to AXI burst
//   Port B: FSM write + FSM read                   - FSM-driven address
//
// For Vivado BRAM inference: both write ports are in a single always block.
// Write priority: stream_in > FSM write (they never conflict in practice
// since stream_in is active during DDR_READ and FSM write during EVICT).
// =============================================================================
`include "oram_params.vh"

module scratch_buf #(
    parameter Z            = `ORAM_Z,
    parameter POS_W        = `POS_IN_BKT_W,
    parameter AXI_DATA_W   = `AXI_DATA_W,
    parameter BEATS        = `BEATS_PER_BLOCK,
    parameter BEAT_W       = `BEAT_CNT_W,
    parameter TOTAL_BEATS  = Z * BEATS         // 8 * 128 = 1024
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // --- Stream-in port (from AXI read data channel, DDR -> scratch) ---
    input  wire [AXI_DATA_W-1:0]    stream_in_data,
    input  wire                     stream_in_valid,
    input  wire [9:0]               stream_in_beat_addr,  // 0..TOTAL_BEATS-1
    output wire                     stream_in_ready,      // always ready (BRAM)

    // --- FSM read port (FSM reads a specific slot) ---
    input  wire [9:0]               fsm_rd_addr,          // {slot_pos, beat}
    input  wire                     fsm_rd_en,
    output wire [AXI_DATA_W-1:0]    fsm_rd_data,
    output wire                     fsm_rd_valid,

    // --- FSM write port (FSM writes evicted stash slot into scratch) ---
    input  wire [9:0]               fsm_wr_addr,          // {slot_pos, beat}
    input  wire [AXI_DATA_W-1:0]    fsm_wr_data,
    input  wire                     fsm_wr_en,

    // --- Stream-out port (to AXI write data channel, scratch -> DDR) ---
    input  wire [9:0]               stream_out_beat_addr, // driven by AXI master
    input  wire                     stream_out_rd_en,
    output reg  [AXI_DATA_W-1:0]    stream_out_data,
    output reg                      stream_out_valid
);

    // -------------------------------------------------------------------------
    // Simple dual-port BRAM with muxed write port
    //
    // stream_in and fsm_wr NEVER fire simultaneously (different ORAM phases):
    //   stream_in_valid: active during S_DDR_READ
    //   fsm_wr_en:       active during eviction/re-encrypt
    //
    // Single write port avoids Vivado TDP inference issues.
    // Two independent read ports (stream_out + fsm_rd).
    // -------------------------------------------------------------------------
    (* ram_style = "block" *)
    reg [AXI_DATA_W-1:0] mem [0:TOTAL_BEATS-1];

    // Muxed write port
    wire        wr_en   = stream_in_valid | fsm_wr_en;
    wire [9:0]  wr_addr = stream_in_valid ? stream_in_beat_addr : fsm_wr_addr;
    wire [AXI_DATA_W-1:0] wr_data = stream_in_valid ? stream_in_data : fsm_wr_data;

    // Single write + Port A read (stream_out)
    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        if (stream_out_rd_en)
            stream_out_data <= mem[stream_out_beat_addr];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stream_out_valid <= 1'b0;
        else
            stream_out_valid <= stream_out_rd_en;
    end

    // Port B read (FSM) — COMBINATIONAL to match URAM timing
    // Registered version added an extra NB boundary causing 129 scratch
    // read stalls per 4KB decryption (385 vs 256 cycles).
    assign fsm_rd_valid = fsm_rd_en;
    assign fsm_rd_data  = fsm_rd_en ? mem[fsm_rd_addr] : {AXI_DATA_W{1'b0}};

    assign stream_in_ready = 1'b1; // BRAM always accepts

endmodule