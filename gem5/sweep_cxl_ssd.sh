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
NUM_OPS=100000
NUM_SLOTS=32768
OUTDIR="m5out"

# Total usable NAND pages (from NAND geometry in sample.cfg):
#   8ch × 4pkg × 2die × 2plane × 512blk × 512pg = 33,554,432 raw pages
#   Usable = 33,554,432 / 1.25 (OverProvisioningRatio) = 26,843,545 pages
TOTAL_NAND_PAGES=26843545

# Pages per ORAM instance: DDR_PER_INSTANCE / PageSize
#   = 0x020000000 (512 MB) / 16384 = 32768 pages
PAGES_PER_INSTANCE=32768

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
    CACHE=$((8388608 * N))
    FILL=$(python3 -c "needed=$N*$PAGES_PER_INSTANCE; print(f'{needed/$TOTAL_NAND_PAGES:.5f}')")
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
    # instance the same effective cache as the N=1 baseline (8 MB),
    # scale total cache to N × 8 MB. Without scaling, per-instance
    # effective cache shrinks with N, penalizing multi-instance runs.
    sed -i "s/^CacheSize = 8388608/CacheSize = $CACHE/" "$SSD_CFG"

    # FIX 3: Scale FillRatio with N
    # REASON: FillRatio fills pages sequentially from LBA 0. Each ORAM
    # instance occupies DDR_PER_INSTANCE = 512 MB = 32768 pages of SSD
    # address space. With N instances, the SSD range is N × 512 MB.
    # FillRatio must cover all N × 32768 pages so every page has a
    # valid FTL mapping. Without scaling, reads to unmapped pages
    # return zero latency (no physical page allocated → FTL skips PAL).
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
