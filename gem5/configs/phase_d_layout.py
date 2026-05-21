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
#     Internal layout per 512 MB instance:
#       buckets:   [0x00000000, variable)        — depends on num_slots
#       posmap:    [0x10400000, 0x10500000)      — 1 MB
#       HT slots:  [0x10500000, 0x10600000)      — 1 MB (512 × 32B)
#       bkt_head:  [0x10600000, 0x10700000)      — 1 MB
#       bkt_next:  [0x10700000, 0x10700800)      — 2 KB
#       stash:     [0x1F000000, 0x20000000)      — 16 MB (at HBM_PER - 16 MB)
#
# Shared DDR5 aggregate (CPU-touched + ORAM-only) at 64-bit:
#   cmd_ring:         0x400000000 - 0x400010000  (16 × 4 KB)
#   result_buf:       0x410000000 - 0x411000000  (16 × 1 MB)
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
HBM_PER_INSTANCE   = 0x020000000   # 512 MB — must hold HT at 0x10500000 (261 MB)

# ---- Shared DDR5 aggregate ----
# Covers cmd_ring + result_buf + per-instance ORAM DDR slabs.
# Starts at 16 GB to avoid overlap with 16 × 512 MB HBM (ends at 12 GB).
DDR_AGG_BASE       = 0x400000000    # 16 GB

CMD_RING_BASE      = DDR_AGG_BASE + 0x000000000    # 16 × 4 KB
RESULT_BUF_BASE    = DDR_AGG_BASE + 0x010000000    # 16 × 1 MB
DDR_SLAB_BASE      = DDR_AGG_BASE + 0x100000000    # 16 × 64 MB; offset 4 GB into agg
DDR_PER_INSTANCE   = 0x020000000                   # 512 MB per slab (matches HBM_PER_INSTANCE)


def per_instance_addrs(i):
    """Return the per-instance address dict for instance i ∈ [0, N)."""
    return dict(
        hbm        = HBM_BASE        + i * HBM_PER_INSTANCE,
        ddr        = DDR_SLAB_BASE   + i * DDR_PER_INSTANCE,
        cmd_ring   = CMD_RING_BASE   + i * 0x1000,
        result_buf = RESULT_BUF_BASE + i * 0x100000,
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
    result_buf_end = RESULT_BUF_BASE + N * 0x100000
    hbm_end = HBM_BASE + N * HBM_PER_INSTANCE
    ddr_slab_start = DDR_SLAB_BASE
    ddr_slab_end   = DDR_SLAB_BASE + N * DDR_PER_INSTANCE

    assert main_end <= ORAM_CMD_BASE,      f"main DRAM ends 0x{main_end:x} overlaps CMD MMIO"
    assert cmd_mmio_end <= HBM_BASE,       f"CMD MMIO ends 0x{cmd_mmio_end:x} overlaps HBM"
    assert hbm_end <= DDR_AGG_BASE,        f"HBM ends 0x{hbm_end:x} overlaps DDR aggregate 0x{DDR_AGG_BASE:x}"
    assert cmd_ring_end <= RESULT_BUF_BASE, f"cmd_ring ends 0x{cmd_ring_end:x} overlaps result_buf"
    assert result_buf_end <= ddr_slab_start, f"result_buf ends 0x{result_buf_end:x} overlaps DDR slabs"

    # Per-instance internal: HT must fit in HBM
    HT_END_OFFSET = 0x10700800  # bkt_next end
    assert HT_END_OFFSET <= HBM_PER_INSTANCE, \
        f"HT region end 0x{HT_END_OFFSET:x} exceeds HBM_PER_INSTANCE 0x{HBM_PER_INSTANCE:x}"

    # stash must not overlap HT/posmap metadata
    stash_offset = HBM_PER_INSTANCE - 0x01000000
    assert stash_offset >= HT_END_OFFSET, \
        f"stash_offset 0x{stash_offset:x} overlaps HT region ending at 0x{HT_END_OFFSET:x}"


_assert_no_overlap(20)


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
    print(f"    result_buf:  [0x{RESULT_BUF_BASE:09x}, 0x{RESULT_BUF_BASE + N*0x100000:09x}) ({N} x 1 MB)")
    print(f"    DDR slabs:   [0x{DDR_SLAB_BASE:09x}, 0x{DDR_SLAB_BASE + N*DDR_PER_INSTANCE:09x}) ({N} x {DDR_PER_INSTANCE//(1024*1024)} MB)")