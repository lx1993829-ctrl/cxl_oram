// =============================================================================
// oram_params.vh - ORAM System Parameters (Board Test - Reduced Size)
// =============================================================================
// Proportionally reduced for ZCU208 on-chip memory constraints:
//   STASH_DEPTH = 64    (was 4096)
//   B = 64 buckets      (was 4095)
//   N = 512 slots       (was 16380)
//   Stash data_mem = 64 * 128 * 256b = 2 Mb (fits in ~7 URAMs)
//
// Slot/bucket ratios preserved: stash:buckets ? 1:1

`ifndef ORAM_PARAMS_VH
`define ORAM_PARAMS_VH

// --- ORAM tree geometry ---
`define ORAM_N              512       // total slots (B * Z)
`define ORAM_Z              8         // slots per bucket
`define ORAM_B              64        // number of buckets
`define ORAM_C              512       // capacity = N
`define ORAM_STASH_DEPTH    64        // stash entries

// --- Bit widths ---
`define BUCKET_ID_W         6         // ceil(log2(B)) = ceil(log2(64)) = 6
`define SLOT_ID_W           9         // ceil(log2(N)) = ceil(log2(512)) = 9
`define SLOT_ADDR_W         32        // full client address width
`define FILL_CNT_W          4         // ceil(log2(Z+1)) = 4, holds 0..8
`define POS_IN_BKT_W        3         // ceil(log2(Z)) = 3
`define STASH_PTR_W         6         // ceil(log2(STASH_DEPTH)) = 6

// --- AXI parameters ---
`define AXI_DATA_W          256       // 256-bit data bus (32 bytes per beat)
`define AXI_ADDR_W          32
`define AXI_STRB_W          32        // AXI_DATA_W / 8
`define AXI_ID_W            4
`define AXI_LEN_W           8         // AXI4 standard: 8-bit ARLEN/AWLEN

// --- Beat/block parameters ---
`define BEATS_PER_BLOCK     128       // 4096 bytes / 32 bytes per beat
`define BEAT_CNT_W          10        // must hold 0..1023 (Z * BEATS_PER_BLOCK - 1)
`define BEATS_PER_BKT       1024      // Z * BEATS_PER_BLOCK = 8 * 128

// --- DDR layout ---
`define DDR_BASE            32'h0000_0000
`define BUCKET_SHIFT        15        // log2(BEATS_PER_BKT * 32) = log2(32768)

// --- Slot status encoding ---
`define ST_DUMMY            2'b00
`define ST_VALID            2'b01
`define ST_IN_STASH         2'b10

// --- FSM states ---
`define S_IDLE              4'd0
`define S_POS_LOOKUP        4'd1
`define S_DDR_READ          4'd2
`define S_SCAN              4'd3
`define S_SCAN2             4'd4
`define S_EXTRACT           4'd5
`define S_STASH_SEARCH      4'd6
`define S_STASH_READ        4'd7
`define S_COMPACT           4'd8
`define S_EVICT             4'd9
`define S_DDR_WRITE         4'd10
`define S_RESPOND           4'd11

`endif
