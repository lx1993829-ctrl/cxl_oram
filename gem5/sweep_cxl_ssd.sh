#!/bin/bash
# ================================================================
# sweep_cxl_ssd.sh — CXL-SSD ORAM N-scaling benchmark
#
# Sweeps N = 1, 2, 3, 4 instances with 100k ops each.
# Temporarily modifies SimpleSSD sample.cfg, restores on exit.
#
# Run from gem5 root:  bash sweep_cxl_ssd.sh
# ================================================================

set -e

# ---- Configuration ----
BINARY_SRC="configs/oram_workload.c"
BINARY="configs/oram_workload"
CONFIG="configs/oram_ssd.py"
SSD_CFG="src/mem/ssd/simplessd/config/sample.cfg"
SSD_CFG_BAK="${SSD_CFG}.sweep_bak"
NUM_OPS=10000
NUM_SLOTS=32768
OUTDIR="m5out"

# Total usable NAND logical pages (from SimpleSSD FTL log with
# EnableMultiPlaneOperation=0):
#   FTL::PageMapping: Total logical pages: 3145728
# This differs from the raw calculation because SimpleSSD's page
# counting depends on MultiPlane setting. With MultiPlane=0, each
# plane is counted separately, doubling the logical page count.
TOTAL_NAND_PAGES=3145728

# Pages per ORAM instance bucket data: 8192 buckets × 32KB / 16KB page = 16384 pages
# SSD allocation per instance: DDR_PER_INSTANCE = 512 MB / 16KB = 32768 pages
# Instance i starts at page i × 32768, accesses pages i×32768 to i×32768+16383

# ---- Step 0: Compile binary ----
echo "=== Compiling $BINARY_SRC ==="
musl-gcc -O0 -static -o "$BINARY" "$BINARY_SRC"
echo "  Binary: $(ls -la $BINARY | awk '{print $5, $9}')"
echo ""

# ---- Step 1: Backup sample.cfg ----
cp "$SSD_CFG" "$SSD_CFG_BAK"
trap 'echo "Restoring sample.cfg..."; cp "$SSD_CFG_BAK" "$SSD_CFG"; rm -f "$SSD_CFG_BAK"' EXIT

# ---- Step 2: Sweep ----
echo "=== Starting N-scaling sweep (ops=$NUM_OPS, slots=$NUM_SLOTS) ==="
echo ""

for N in 1 2 3 4; do
    # ---- Compute per-N SimpleSSD parameters ----
    CACHE=$((262144 * N))
    FILL=$(python3 -c "highest=($N-1)*32768+16384; print(f'{highest/$TOTAL_NAND_PAGES:.5f}')")
    LOGFILE="${OUTDIR}/n${N}_ssd_100k.log"

    echo "=== N=$N  CacheSize=$CACHE  FillRatio=$FILL ==="
    echo "  Log: $LOGFILE"

    # ---- Patch sample.cfg with documented reasons ----
    cp "$SSD_CFG_BAK" "$SSD_CFG"

    # FIX 1: EnableMultiPlaneOperation = 0
    # ROOT CAUSE: With MultiPlane=1, ioUnitInPage=2 but ICL's
    # lineCountInSuperPage=8 (from SuperblockSize=C). The FTL's
    # readInternal loops idx 0..1 (ioUnitInPage) but ICL requests
    # idx = LCA % 8 (lineCountInSuperPage). Any read with idx >= 2
    # finds no FTL mapping → PAL never called → zero NAND latency.
    # This artificially speeds up ~75% of reads and creates asymmetric
    # performance across ORAM instances. Setting MultiPlane=0 makes
    # ioUnitInPage=1=lineCountInSuperPage, ensuring every read hits
    # a valid FTL mapping and gets real NAND timing.
    sed -i "s/^EnableMultiPlaneOperation = 1/EnableMultiPlaneOperation = 0/" "$SSD_CFG"

    # FIX 2: Scale CacheSize with N
    # REASON: One SimpleSSD controller is shared by N ORAM instances.
    # The ICL read/write cache (CacheSize) is shared. To give each
    # instance the same effective cache as the N=1 baseline (256 KB),
    # scale total cache to N × 256 KB. Without scaling, per-instance
    # effective cache shrinks with N, penalizing multi-instance runs.
    sed -i "s/^CacheSize = .*/CacheSize = $CACHE/" "$SSD_CFG"

    # FIX 3: Scale FillRatio with N
    # REASON: FillRatio fills pages sequentially from LBA 0. Each ORAM
    # instance uses 256 MB of bucket data (8192 buckets × 32 KB) within
    # a 512 MB SSD allocation. Instance i starts at page i × 32768.
    # The highest accessed page is (N-1) × 32768 + 16384. FillRatio
    # must cover all pages up to this point so every page has a valid
    # FTL mapping. Without this, reads to unmapped pages skip the PAL
    # entirely and return zero NAND latency.
    sed -i "s/^FillRatio = 0.002/FillRatio = $FILL/" "$SSD_CFG"

    # ---- Run simulation ----
    # Estimated runtime: ~20-40 hours per N for 100k ops.
    # Use nohup-style background if running unattended.
    timeout 172800 ./build/ALL/gem5.opt "$CONFIG" \
        --binary="$BINARY" \
        --num-instances=$N \
        --num-ops=$NUM_OPS \
        --num-slots=$NUM_SLOTS \
        --use-ssd --dram-cache=0B \
        > "$LOGFILE" 2>&1

    # ---- Report per-instance results ----
    TOTAL_EXPECTED=$((N * NUM_OPS))
    TOTAL_DONE=$(grep "ORAM\[" "$LOGFILE" | wc -l)
    echo "  Completed: $TOTAL_DONE / $TOTAL_EXPECTED ops"

    for inst in $(seq 0 $((N-1))); do
        grep "ORAM\[$inst\]" "$LOGFILE" | grep -oP '\d+ cyc' | awk -v i=$inst '{
            sum+=$1; n++;
            if($1<5000) fast++
            else if($1<30000) lsb++
            else if($1<40000) msb++
            else slow++
        } END {
            printf "  inst %d: avg=%.0f cyc (%.1f us) | stash=%d lsb=%d msb=%d slow=%d | %d ops\n",
                   i, sum/n, sum/n/300, fast, lsb, msb, slow, n
        }'
    done
    echo ""
done

# ---- Step 3: Summary table ----
echo "=============================================="
echo "  CXL-SSD ORAM N-Scaling Summary"
echo "  ${NUM_SLOTS} slots, ${NUM_OPS} ops/instance"
echo "=============================================="
printf "%-4s  %10s  %10s  %8s\n" "N" "avg cyc" "avg us" "vs N=1"

BASELINE=0
for N in 1 2 3 4; do
    LOGFILE="${OUTDIR}/n${N}_ssd_100k.log"
    AVG=$(grep "ORAM\[" "$LOGFILE" | grep -oP '\d+ cyc' | awk '{sum+=$1; n++} END {printf "%.0f", n>0 ? sum/n : 0}')
    AVG_US=$(python3 -c "print(f'{$AVG/300:.1f}')" 2>/dev/null || echo "?")

    if [ "$N" -eq 1 ]; then
        BASELINE=$AVG
        DELTA="baseline"
    else
        DELTA=$(python3 -c "print(f'+{($AVG-$BASELINE)*100/$BASELINE:.1f}%')" 2>/dev/null || echo "?")
    fi

    printf "%-4d  %10s  %8s us  %8s\n" "$N" "$AVG" "$AVG_US" "$DELTA"
done

echo ""
echo "=== sample.cfg restored ==="