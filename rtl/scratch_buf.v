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
//   STREAM ports : sequential, address counter-driven, tied to AXI burst
//   FSM ports    : FSM-driven address, any slot/beat combination
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
    // --- Stream-in port (from AXI read data channel, DDR ? scratch) ---
    input  wire [AXI_DATA_W-1:0]    stream_in_data,
    input  wire                     stream_in_valid,
    input  wire [9:0]               stream_in_beat_addr,  // 0..TOTAL_BEATS-1
    output wire                     stream_in_ready,      // always ready (BRAM)
    // --- FSM read port (FSM reads a specific slot to send to client) ---
    input  wire [9:0]               fsm_rd_addr,          // {slot_pos, beat}
    input  wire                     fsm_rd_en,
    output reg  [AXI_DATA_W-1:0]    fsm_rd_data,
    output reg                      fsm_rd_valid,
    // --- FSM write port (FSM writes evicted stash slot into scratch) ---
    input  wire [9:0]               fsm_wr_addr,          // {slot_pos, beat}
    input  wire [AXI_DATA_W-1:0]    fsm_wr_data,
    input  wire                     fsm_wr_en,
    // --- Stream-out port (to AXI write data channel, scratch ? DDR) ---
    input  wire [9:0]               stream_out_beat_addr, // driven by AXI master
    input  wire                     stream_out_rd_en,
    output reg  [AXI_DATA_W-1:0]    stream_out_data,
    output reg                      stream_out_valid
);
    // -------------------------------------------------------------------------
    // True dual-port BRAM: TOTAL_BEATS × AXI_DATA_W bits
    // Port A: stream_in writes  + stream_out reads
    // Port B: FSM reads         + FSM writes
    // -------------------------------------------------------------------------
    (* ram_style = "ultra" *)
    reg [AXI_DATA_W-1:0] mem [0:TOTAL_BEATS-1];
    // Port A - stream in (write) / stream out (read)
    always @(posedge clk) begin
        if (stream_in_valid)
            mem[stream_in_beat_addr] <= stream_in_data;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            stream_out_data  <= {AXI_DATA_W{1'b0}};
            stream_out_valid <= 1'b0;
        end else begin
            stream_out_valid <= stream_out_rd_en;
            if (stream_out_rd_en)
                stream_out_data <= mem[stream_out_beat_addr];
        end
    end
    // Port B - FSM read / FSM write
    always @(posedge clk) begin
        if (fsm_wr_en)
            mem[fsm_wr_addr] <= fsm_wr_data;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm_rd_data  <= {AXI_DATA_W{1'b0}};
            fsm_rd_valid <= 1'b0;
        end else begin
            fsm_rd_valid <= fsm_rd_en;
            if (fsm_rd_en)
                fsm_rd_data <= mem[fsm_rd_addr];
        end
    end
    assign stream_in_ready = 1'b1; // BRAM always accepts
endmodule