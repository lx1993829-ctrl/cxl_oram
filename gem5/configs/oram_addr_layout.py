# =============================================================================
# Phase D — Multi-instance address layout (v5).
#
# v5 changes (from v4):
#  - HBM_PER_INSTANCE fixed: 0x020000000 (512 MB). v4 had 0x002000000
#    (32 MB) which couldn't hold the HT at offset 0x10500000 (261 MB).
#  - DDR_AGG_BASE pushed from 0x200000000 to 0x400000000 so 16×512 MB
#    HBM (ending at 0x300000000) doesn't overlap the DDR aggregate.
#  - DDR_PER_INSTANCE comment corrected: 64 MB (not "1 GB").
#
# Layout:
#
# CPU-touched 32-bit:
#   main DRAM:        0x000000000 - 0x040000000  (1 GB)
#   ORAM_CMD MMIO:    0x0E0000000 - 0x0E0010000  (16 × 4 KB)
#
# HBM (per-instance, 64-bit):
#   HBM:              0x100000000 + i * 0x020000000  (16 × 512 MB)
#     Internal layout per 512 MB instance (matches oram_params.vh / pos_map.v):
#       buckets:   [0x00000000, variable)        — depends on num_slots
#                                                  (num_slots=32764 -> 255 MB)
#       stash:     [0x10000000, 0x14000000)      — 64 MB (16384 × 4KB, PTR_W=14)
#       posmap:    [0x14000000, 0x14100000)      — 1 MB
#       HT slots:  [0x14100000, 0x14200000)      — 1 MB (512 × 32B)
#       bkt_head:  [0x14200000, 0x14300000)      — 1 MB
#       bkt_next:  [0x14300000, 0x14308000)      — 32 KB (1024 × 32B, PTR_W=14)
#       IVT:       [0x14400000, 0x14600000)      — 4 MB (per-physical, ~2MB used)
#     metadata top = 0x14600000 (326 MB) — fits in 512 MB instance.
#
# Shared DDR5 aggregate (CPU-touched + ORAM-only) at 64-bit:
#   cmd_ring:         0x400000000 - 0x400010000  (16 × 4 KB)
#   result_buf:       0x410000000 - 0x420000000  (16 × 16 MB)
#   ORAM DDR slabs:   0x500000000 + i * 0x004000000  (16 × 64 MB)
#
# All cmd_ring / result_buf / DDR slab addresses fall inside the single
# DDR5 aggregate range so they get interleaved across all 8 channels.
# =============================================================================

# ---- Main CPU DRAM ----
MAIN_DRAM_BASE = 0x000000000
MAIN_DRAM_SIZE = 0x040000000        # 1 GB

# ---- CPU-touched MMIO ----
ORAM_CMD_BASE      = 0x0E0000000

# ---- HBM (per-instance) ----
HBM_BASE           = 0x100000000    # 4 GB
HBM_PER_INSTANCE   = 0x020000000   # 512 MB — must hold metadata top 0x14600000 (326 MB)

# ---- Per-instance internal metadata offsets (MUST match oram_params.vh) ----
# Contiguous block above the bucket region. Stash deepened to 64 MB
# (STASH_PTR_W=14, 16384 entries); IVT is per-physical-slot (4 MB budget).
STASH_OFFSET        = 0x10000000    # RTL STASH_DDR_BASE
STASH_REGION_BYTES  = 0x04000000    # 64 MB (16384 × 4KB)
PM_OFFSET           = 0x14000000    # RTL PM_BASE (pos_map.v)
HT_SLOT_OFFSET      = 0x14100000
HT_BKT_HEAD_OFFSET  = 0x14200000
HT_BKT_NEXT_OFFSET  = 0x14300000
IVT_OFFSET          = 0x14400000    # per-physical-slot IV/TAG
METADATA_TOP_OFFSET = 0x14600000    # top of IVT (4 MB budget)
# Highest bucket-region address the largest num_slots can reach. At
# num_slots=32764 -> 8191 buckets × 32KB = 255 MB, just under stash base.
BUCKET_REGION_MAX   = 0x0FF80000    # 8191 × 32768 (num_slots=32764)

# ---- Shared DDR5 aggregate ----
# Covers cmd_ring + result_buf + per-instance ORAM DDR slabs.
# Starts at 16 GB to avoid overlap with 16 × 512 MB HBM (ends at 12 GB).
DDR_AGG_BASE       = 0x600000000    # 24 GB — pushed for N≤32 (HBM ends at 0x500000000)

CMD_RING_BASE      = DDR_AGG_BASE + 0x000000000    # 16 × 4 KB
RESULT_BUF_BASE    = DDR_AGG_BASE + 0x010000000    # 16 × 16 MB
DDR_SLAB_BASE      = DDR_AGG_BASE + 0x100000000    # 16 × 64 MB; offset 4 GB into agg
DDR_PER_INSTANCE   = 0x020000000                   # 256 MB per slab (bucket tree ≈ 128 MB)


def per_instance_addrs(i):
    """Return the per-instance address dict for instance i ∈ [0, N)."""
    return dict(
        hbm        = HBM_BASE        + i * HBM_PER_INSTANCE,
        ddr        = DDR_SLAB_BASE   + i * DDR_PER_INSTANCE,
        cmd_ring   = CMD_RING_BASE   + i * 0x1000,
        result_buf = RESULT_BUF_BASE + i * 0x1000000,
        cmd_port   = ORAM_CMD_BASE   + i * 0x1000,
    )


def ddr_aggregate_size(N):
    """Total size of the DDR5 aggregate range for N instances."""
    return (DDR_SLAB_BASE - DDR_AGG_BASE) + N * DDR_PER_INSTANCE


def _assert_no_overlap(N):
    """No-overlap invariant for any N up to 16."""
    main_end = MAIN_DRAM_BASE + MAIN_DRAM_SIZE
    cmd_mmio_end   = ORAM_CMD_BASE   + N * 0x1000
    cmd_ring_end   = CMD_RING_BASE   + N * 0x1000
    result_buf_end = RESULT_BUF_BASE + N * 0x1000000
    hbm_end = HBM_BASE + N * HBM_PER_INSTANCE
    ddr_slab_start = DDR_SLAB_BASE
    ddr_slab_end   = DDR_SLAB_BASE + N * DDR_PER_INSTANCE

    assert main_end <= ORAM_CMD_BASE,      f"main DRAM ends 0x{main_end:x} overlaps CMD MMIO"
    assert cmd_mmio_end <= HBM_BASE,       f"CMD MMIO ends 0x{cmd_mmio_end:x} overlaps HBM"
    assert hbm_end <= DDR_AGG_BASE,        f"HBM ends 0x{hbm_end:x} overlaps DDR aggregate 0x{DDR_AGG_BASE:x}"
    assert cmd_ring_end <= RESULT_BUF_BASE, f"cmd_ring ends 0x{cmd_ring_end:x} overlaps result_buf"
    assert result_buf_end <= ddr_slab_start, f"result_buf ends 0x{result_buf_end:x} overlaps DDR slabs"

    # =====================================================================
    # Per-instance internal layout (must match oram_params.vh / pos_map.v).
    # Metadata is now a contiguous block above the bucket region:
    #   stash 0x10000000 (64MB) -> posmap -> HT -> IVT, top = 0x14600000.
    # =====================================================================
    # buckets must end before the stash base; metadata top must fit in HBM.
    assert BUCKET_REGION_MAX <= STASH_OFFSET, \
        f"bucket region max 0x{BUCKET_REGION_MAX:x} overlaps stash base 0x{STASH_OFFSET:x}"
    assert METADATA_TOP_OFFSET <= HBM_PER_INSTANCE, \
        f"metadata top 0x{METADATA_TOP_OFFSET:x} exceeds HBM_PER_INSTANCE 0x{HBM_PER_INSTANCE:x}"
    # stash region (64 MB) must not collide with posmap that follows it
    assert STASH_OFFSET + STASH_REGION_BYTES <= PM_OFFSET, \
        f"stash end 0x{STASH_OFFSET + STASH_REGION_BYTES:x} overlaps posmap 0x{PM_OFFSET:x}"


_assert_no_overlap(16)


if __name__ == '__main__':
    import sys
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 16
    _assert_no_overlap(N)
    agg_end = DDR_AGG_BASE + ddr_aggregate_size(N)
    print(f"Phase D address layout v5 for N={N}:")
    print(f"  Main CPU DRAM: [0x{MAIN_DRAM_BASE:09x}, 0x{MAIN_DRAM_BASE+MAIN_DRAM_SIZE:09x})  = 1 GB  (CPU)")
    print(f"  ORAM_CMD MMIO: [0x{ORAM_CMD_BASE:09x}, 0x{ORAM_CMD_BASE + N*0x1000:09x})    (CPU MMIO only)")
    print(f"  HBM aggregate: [0x{HBM_BASE:09x}, 0x{HBM_BASE + N*HBM_PER_INSTANCE:09x})  ({N} x {HBM_PER_INSTANCE//(1024*1024)} MB) (ORAM only)")
    print(f"  DDR5 aggregate: [0x{DDR_AGG_BASE:09x}, 0x{agg_end:09x}) — interleaved across 8 channels")
    print(f"    cmd_ring:    [0x{CMD_RING_BASE:09x}, 0x{CMD_RING_BASE + N*0x1000:09x}) ({N} x 4 KB)")
    print(f"    result_buf:  [0x{RESULT_BUF_BASE:09x}, 0x{RESULT_BUF_BASE + N*0x1000000:09x}) ({N} x 16 MB)")
    print(f"    DDR slabs:   [0x{DDR_SLAB_BASE:09x}, 0x{DDR_SLAB_BASE + N*DDR_PER_INSTANCE:09x}) ({N} x {DDR_PER_INSTANCE//(1024*1024)} MB)")
