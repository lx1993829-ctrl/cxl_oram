#!/bin/bash
# ================================================================
# sweep_cxl_ssd.sh — CXL-SSD ORAM N-scaling benchmark (sequential)
#
# Sweeps N = 1, 2, 3, 4 instances with configurable ops each.
# Temporarily modifies SimpleSSD sample.cfg, restores on exit.
#
# Run from gem5 root:  bash sweep_cxl_ssd.sh
# For parallel version:  bash sweep_cxl_ssd_multicore.sh
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

TOTAL_NAND_PAGES=3145728

# ---- Step 0: Compile binary ----
echo "=== Compiling $BINARY_SRC ==="
musl-gcc -O0 -static -o "$BINARY" "$BINARY_SRC"
echo "  Binary: $(ls -la $BINARY | awk '{print $5, $9}')"
echo ""

# ---- Step 1: Backup sample.cfg ----
cp "$SSD_CFG" "$SSD_CFG_BAK"
trap 'echo "Restoring sample.cfg..."; cp "$SSD_CFG_BAK" "$SSD_CFG"; rm -f "$SSD_CFG_BAK"' EXIT

# ---- Step 2: Sweep (SEQUENTIAL) ----
echo "=== Starting CXL-SSD N-scaling sweep (ops=$NUM_OPS, slots=$NUM_SLOTS) ==="
echo ""

for N in 1 2 3 4; do
    CACHE=$((262144 * N))
    FILL=$(python3 -c "highest=($N-1)*32768+16384; print(f'{highest/$TOTAL_NAND_PAGES:.5f}')")
    LOGFILE="${OUTDIR}/n${N}_ssd_${NUM_OPS}op.log"

    echo "=== N=$N  CacheSize=$CACHE  FillRatio=$FILL ==="

    cp "$SSD_CFG_BAK" "$SSD_CFG"
    sed -i "s/^EnableMultiPlaneOperation = .*/EnableMultiPlaneOperation = 0/" "$SSD_CFG"
    sed -i "s/^CacheSize = .*/CacheSize = $CACHE/" "$SSD_CFG"
    sed -i "s/^FillRatio = .*/FillRatio = $FILL/" "$SSD_CFG"

    ./build/ALL/gem5.opt -d "${OUTDIR}/ssd_n${N}" "$CONFIG" \
        --binary="$BINARY" \
        --num-instances=$N \
        --num-ops=$NUM_OPS \
        --num-slots=$NUM_SLOTS \
        --use-ssd --dram-cache=0B \
        --ssd-config="$SSD_CFG" \
        > "$LOGFILE" 2>&1

    TOTAL_DONE=$(grep -c "ORAM\[" "$LOGFILE" || true)
    echo "=== N=$N DONE: $TOTAL_DONE / $((N * NUM_OPS)) ops ==="
    echo ""
done

# ---- Step 3: Summary table ----
echo "=============================================="
echo "  CXL-SSD ORAM N-Scaling Summary"
echo "  ${NUM_SLOTS} slots, ${NUM_OPS} ops/instance"
echo "=============================================="
printf "%-4s  %10s  %10s  %10s  %8s\n" "N" "ORAM cyc" "E2E us/op" "E2E cyc" "vs N=1"

BASELINE_E2E=0
for N in 1 2 3 4; do
    LOGFILE="${OUTDIR}/n${N}_ssd_${NUM_OPS}op.log"

    ORAM_CYC=$(grep 'Avg.*cyc/op' "$LOGFILE" | head -1 | grep -oP '[\d.]+(?= cyc/op)' || echo "0")

    E2E_US=$(grep 'STEADY-STATE:' "$LOGFILE" | head -1 | grep -oP '[\d.]+(?= ns)' | head -1 |
             python3 -c "import sys; ns=float(sys.stdin.read().strip()); print(f'{ns/1000:.1f}')" 2>/dev/null || echo "0")

    if [ "$E2E_US" = "0" ]; then
        E2E_US=$(grep "ORAM\[" "$LOGFILE" | grep -oP '[\d.]+ us elapsed' | awk '{sum+=$1; n++} END {if(n>0) printf "%.1f", sum/n; else print "0"}')
    fi

    E2E_CYC=$(python3 -c "print(f'{float(\"$E2E_US\") * 300:.0f}')" 2>/dev/null || echo "0")

    if [ "$N" -eq 1 ]; then
        BASELINE_E2E="$E2E_US"
        DELTA="baseline"
    else
        DELTA=$(python3 -c "print(f'{float(\"$E2E_US\")/float(\"$BASELINE_E2E\"):.2f}x')" 2>/dev/null || echo "?")
    fi

    printf "%-4d  %10s  %8s us  %8s  %8s\n" "$N" "$ORAM_CYC" "$E2E_US" "$E2E_CYC" "$DELTA"
done

echo ""
echo "=== sample.cfg restored ==="
