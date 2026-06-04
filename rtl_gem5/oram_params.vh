`ifndef ORAM_PARAMS_VH
`define ORAM_PARAMS_VH
// === Primary parameters — compiled for MAX capacity ===
// Runtime uses fewer slots/buckets/stash via --num-slots.
// Never needs recompilation unless max capacity increases.
`define ORAM_B            8192
`define ORAM_Z            8
`define ORAM_C            4

// === Derived (auto-scale with B, Z, C) ===
`define ORAM_N            (`ORAM_B * `ORAM_C)           // 32768 max real slots
`define ORAM_STASH_DEPTH  (`ORAM_B * `ORAM_Z / 4)      // 16384 max stash entries

// === Bit widths (sized for max B/N/STASH) ===
`define BUCKET_ID_W       13          // $clog2(8192)
`define SLOT_ID_W         15          // $clog2(32768)
`define STASH_PTR_W       14          // $clog2(16384)
`define SLOT_ADDR_W       32
`define DDR_ADDR_W        32
`define FILL_CNT_W        4
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

// =============================================================================
// Off-chip metadata address map (sized for max: 8192 buckets, 16384 stash).
//   DDR bucket region:   8192 buckets × 32KB = 256 MB  [0x00000000, 0x10000000)
//   STASH_DDR_BASE:      0x10000000    64 MB  (16384 × 4KB)  [0x10000000, 0x14000000)
//
//   At runtime with --num-slots=N:
//     active_buckets = N / C       (e.g. 16384/4 = 4096)
//     active_stash   = N / 2       (e.g. 16384/2 = 8192)
//     Only active entries are initialized; rest stay zeroed.
//
//   region            base          budget   max use         headroom
//   STASH_DDR_BASE    0x10000000    64 MB    16384 x 4KB      exact
//   PM_BASE*          0x14000000     1 MB    128 KB            8x    (*in pos_map.v)
//   HT_SLOT_BASE      0x14100000     1 MB    16 KB            64x
//   HT_BKT_HEAD_BASE  0x14200000     1 MB    16 KB            64x
//   HT_BKT_NEXT_BASE  0x14300000     1 MB    64 KB            16x
//   IVT_BASE          0x14400000     4 MB     2.0 MB          2.0x  (per-physical-slot)
//   SLOT_R_BASE       0x14800000     1 MB     0.5 MB          2.0x  (per-stash-entry)
//   BUCKET_META_BASE  0x14900000     1 MB     0.25 MB         4.0x  (per-bucket)
//   (next free)       0x14A00000
// =============================================================================
`define HT_SLOT_BASE      34'h0_1410_0000
`define HT_SLOT_ENTRIES    2048
`define HT_SLOT_MASK       11'h7FF          // hash = slot_addr[22:12]
`define HT_BKT_HEAD_BASE  34'h0_1420_0000
`define HT_BKT_NEXT_BASE  34'h0_1430_0000
`define HT_EMPTY           14'h3FFF         // empty pointer (max PTR_W=14)

// IV/TAG table — PER-PHYSICAL-SLOT addressing.
//   max entries = numBuckets * Z = 8192 * 8 = 65536  (<= 2^16)
//   size        = 65536 * 32B = 2.0 MB ; 4 MB budget gives 2x headroom.
`define IVT_BASE          34'h0_1440_0000
`define IVT_ENTRY_BYTES   32
`define IVT_PHYS_SLOTS    65536          // 2^16, covers numBuckets*Z=65536

// slot_r table — per-stash-ENTRY slot address.
//   max entries = ORAM_STASH_DEPTH = 16384 ; size = 16384 * 32B = 0.5 MB.
`define SLOT_R_BASE       34'h0_1480_0000

// bucket_meta table — per-bucket directory.
//   max entries = ORAM_B = 8192 ; size = 8192 * 32B = 0.25 MB.
`define BUCKET_META_BASE  34'h0_1490_0000
//   (next free)          0x14A00000
`endif
