#!/bin/bash
# ================================================================
# sweep_nvme_ssd.sh — PCIe NVMe SSD ORAM N-scaling benchmark
#
# Sweeps N = 1, 2, 3, 4 instances with configurable ops each.
# Temporarily modifies SimpleSSD sample.cfg, restores on exit.
#
# Run from gem5 root:  bash sweep_nvme_ssd.sh
# ================================================================

set -e

# ---- Configuration ----
BINARY_SRC="configs/oram_nvme_test.c"
BINARY="configs/oram_nvme_test"
CONFIG="configs/oram_ssd.py"
SSD_CFG="src/mem/ssd/simplessd/config/sample.cfg"
SSD_CFG_BAK="${SSD_CFG}.sweep_bak"
NUM_OPS=10000            # ORAM ops per instance
NUM_SLOTS=32768
OUTDIR="m5out"
NVME_BAR0="0xf0000000"

# Total usable NAND logical pages (from SimpleSSD FTL log with
# EnableMultiPlaneOperation=0):
TOTAL_NAND_PAGES=3145728

# ---- Step 0: Compile binary ----
echo "=== Compiling $BINARY_SRC ==="
musl-gcc -O2 -Wall -static -Itests/ -o "$BINARY" "$BINARY_SRC"
echo "  Binary: $(ls -la $BINARY | awk '{print $5, $9}')"
echo ""

# ---- Step 1: Backup sample.cfg ----
cp "$SSD_CFG" "$SSD_CFG_BAK"
trap 'echo "Restoring sample.cfg..."; cp "$SSD_CFG_BAK" "$SSD_CFG"; rm -f "$SSD_CFG_BAK"' EXIT

# ---- Step 2: Sweep ----
echo "=== Starting NVMe SSD N-scaling sweep (ops=$NUM_OPS, slots=$NUM_SLOTS) ==="
echo ""

for N in 1 2 3 4; do
    # ---- Compute per-N SimpleSSD parameters ----
    CACHE=$((262144 * N))
    FILL=$(python3 -c "highest=($N-1)*32768+16384; print(f'{highest/$TOTAL_NAND_PAGES:.5f}')")
    LOGFILE="${OUTDIR}/n${N}_nvme_${NUM_OPS}op.log"

    echo "=== N=$N  CacheSize=$CACHE  FillRatio=$FILL ==="
    echo "  Log: $LOGFILE"

    # ---- Patch sample.cfg ----
    cp "$SSD_CFG_BAK" "$SSD_CFG"

    # FIX 1: EnableMultiPlaneOperation = 0
    # Same root cause as CXL SSD: ioUnitInPage mismatch causes
    # 75% of reads to skip PAL with zero NAND latency.
    sed -i "s/^EnableMultiPlaneOperation = .*/EnableMultiPlaneOperation = 0/" "$SSD_CFG"

    # FIX 2: Scale CacheSize with N (256 KB per instance)
    sed -i "s/^CacheSize = .*/CacheSize = $CACHE/" "$SSD_CFG"

    # FIX 3: Scale FillRatio with N
    # Instance i's bucket data starts at page i × 32768.
    # Highest accessed page = (N-1) × 32768 + 16384.
    sed -i "s/^FillRatio = .*/FillRatio = $FILL/" "$SSD_CFG"

    # Verify config
    echo "  Config:"
    grep -E 'CacheSize|FillRatio|EnableMultiPlane' "$SSD_CFG" | sed 's/^/    /'

    # ---- Run simulation ----
    ./build/ALL/gem5.opt "$CONFIG" \
        --binary="$BINARY" \
        --num-instances=$N \
        --use-nvme \
        --nvme-bar0=$NVME_BAR0 \
        --num-ops=$NUM_OPS \
        --num-slots=$NUM_SLOTS \
        --dram-cache=0B \
        > "$LOGFILE" 2>&1

    # ---- Check pass/fail ----
    PASS_COUNT=$(grep -c '|||PASS oram_nvme_test' "$LOGFILE" || true)
    FAIL_COUNT=$(grep -c '|||FAIL' "$LOGFILE" || true)
    echo "  Instances PASS: $PASS_COUNT / $N"
    if [ "$FAIL_COUNT" -gt 0 ]; then
        echo "  *** FAILURES: $FAIL_COUNT ***"
        grep '|||FAIL' "$LOGFILE" | sed 's/^/    /'
    fi

    # ---- ORAM RTL performance ----
    echo "  ORAM RTL:"
    grep 'Avg.*cyc/op' "$LOGFILE" | sed 's/^.*info: /    /'
    grep 'STEADY-STATE' "$LOGFILE" | sed 's/^.*info: /    /'

    # ---- NVMe E2E throughput ----
    # Total wall-clock time / total ops across all instances
    python3 -c "
import re

first_all = float('inf')
last_all = 0

for Q in range(1, $N+1):
    # Extract doorbells for queue Q
    with open('$LOGFILE', errors='ignore') as f:
        ticks = []
        for line in f:
            if ('SQ %d ' % Q) in line and 'Doorbell' in line and 'tail' in line:
                m = re.match(r'(\d+):', line)
                if m: ticks.append(int(m.group(1)))
            elif ('CQ %d ' % Q) in line and 'Doorbell' in line and 'head' in line:
                m = re.match(r'(\d+):', line)
                if m: ticks.append(int(m.group(1)))

    # Skip smoke test (first 4 doorbells)
    t = ticks[4:]
    if len(t) >= 2:
        if t[0] < first_all: first_all = t[0]
        if t[-1] > last_all: last_all = t[-1]

total_us = (last_all - first_all) / 1e6
total_ops = $N * $NUM_OPS
avg_us = total_us / total_ops
avg_cyc = avg_us * 300  # 300 MHz → cycles

print(f'  NVMe E2E:')
print(f'    Total wall-clock: {total_us:.0f} us')
print(f'    Total ops:        {total_ops}')
print(f'    Throughput:        {avg_us:.1f} us/op ({avg_cyc:.0f} cyc/op)')
"
    echo ""
done

# ---- Step 3: Summary table ----
echo "=============================================="
echo "  NVMe SSD ORAM N-Scaling Summary"
echo "  ${NUM_SLOTS} slots, ${NUM_OPS} ops/instance"
echo "  Two-pass: all writes then all reads"
echo "  Zero-copy + pipelined staging"
echo "=============================================="
printf "%-4s  %10s  %10s  %10s  %8s\n" "N" "ORAM cyc" "E2E us/op" "E2E cyc" "vs N=1"

BASELINE_E2E=0
for N in 1 2 3 4; do
    LOGFILE="${OUTDIR}/n${N}_nvme_${NUM_OPS}op.log"

    # ORAM avg cycles
    ORAM_CYC=$(grep 'Avg.*cyc/op' "$LOGFILE" | head -1 | grep -oP '[\d.]+(?= cyc/op)' || echo "0")

    # NVMe E2E
    E2E_US=$(python3 -c "
import re
first_all = float('inf')
last_all = 0
for Q in range(1, $N+1):
    with open('$LOGFILE', errors='ignore') as f:
        ticks = []
        for line in f:
            if ('SQ %d ' % Q) in line and 'Doorbell' in line and 'tail' in line:
                m = re.match(r'(\d+):', line)
                if m: ticks.append(int(m.group(1)))
            elif ('CQ %d ' % Q) in line and 'Doorbell' in line and 'head' in line:
                m = re.match(r'(\d+):', line)
                if m: ticks.append(int(m.group(1)))
    t = ticks[4:]
    if len(t) >= 2:
        if t[0] < first_all: first_all = t[0]
        if t[-1] > last_all: last_all = t[-1]
total_us = (last_all - first_all) / 1e6
print(f'{total_us / ($N * $NUM_OPS):.1f}')
")
    E2E_CYC=$(python3 -c "print(f'{$E2E_US * 300:.0f}')")

    if [ "$N" -eq 1 ]; then
        BASELINE_E2E="$E2E_US"
        DELTA="baseline"
    else
        DELTA=$(python3 -c "print(f'{$E2E_US/$BASELINE_E2E:.2f}x')" 2>/dev/null || echo "?")
    fi

    printf "%-4d  %10s  %8s us  %8s  %8s\n" "$N" "$ORAM_CYC" "$E2E_US" "$E2E_CYC" "$DELTA"
done

echo ""
echo "=== sample.cfg restored ==="