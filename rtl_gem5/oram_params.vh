`ifndef ORAM_PARAMS_VH
`define ORAM_PARAMS_VH
`define ORAM_N            32764
`define ORAM_C            4
`define ORAM_Z            8
`define ORAM_B            8191
`define ORAM_STASH_DEPTH  1024
`define SLOT_ADDR_W       32
`define DDR_ADDR_W        32
`define BUCKET_ID_W       13
`define SLOT_ID_W         15
`define FILL_CNT_W        4
`define STASH_PTR_W       10
`define POS_IN_BKT_W      3
`define AXI_DATA_W        256
`define AXI_STRB_W        32
`define AXI_ADDR_W        34
`define AXI_ID_W          6
`define AXI_LEN_W         8
`define AXI3_MAX_BURST    16
`define AXI3_MAX_LEN      15
// AXI4 burst constraints (used by both bucket and stash masters)
`define AXI4_MAX_BURST    256
`define AXI4_MAX_LEN      255
`define BLOCK_BYTES       4096
`define BLOCK_BITS        32768
`define BEATS_PER_BLOCK   128
`define BEAT_CNT_W        10
`define BUCKET_BYTES      32768
`define BEATS_PER_BKT     1024
`define DDR_BASE          34'h0_0000_0000
`define BUCKET_SHIFT      15
`define STASH_DDR_BASE         34'h0_1000_0000
`define STASH_BLOCK_SHIFT      12
// Stash AXI4 burst: one entry = 128 beats in a single AXI4 burst
`define STASH_BEATS_PER_ENTRY  128
`define STASH_SUB_BURST_LEN    127
`define STASH_NUM_BURSTS       1
`define ST_DUMMY          2'b00
`define ST_VALID          2'b01
`define ST_IN_STASH       2'b10

// Hash table parameters — placed above bucket region (max bucket addr
// = ORAM_N * 32KB = ~1GB) and in the stash DRAM range (0x10000000+).
// Previous mux corruption at this address was fixed by the ht_safe
// whitelist gate in secure_oram_top.v.
`define HT_SLOT_BASE      34'h0_1050_0000
`define HT_SLOT_ENTRIES    2048
`define HT_SLOT_MASK       11'h7FF          // hash = slot_addr[22:12]
`define HT_BKT_HEAD_BASE  34'h0_1060_0000
`define HT_BKT_NEXT_BASE  34'h0_1070_0000
`define HT_EMPTY           10'h3FF          // 0x3FF = empty pointer (max PTR_W)
`endif
