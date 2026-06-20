#!/bin/bash
# =============================================================================
# sweep_scaling.sh — Instance scaling sweep: 5 fabric types × N=1..8
#
# Extracts per run: E2E cyc/op, RTL cyc/op, DDR_READ cyc, DDR_WRITE cyc
# Runs up to MAX_JOBS simulations in parallel.
#
# Output:
#   m5out/scaling_sweep/results.csv           — all metrics, one row per run
#   m5out/scaling_sweep/{type}_N{n}.log       — full gem5 log per run
#   m5out/scaling_sweep/{type}_N{n}/          — gem5 stats/config per run
#
# Usage:
#   cd /mnt/f/gem5
#   bash configs/sweep_scaling.sh
# =============================================================================
set -e

GEM5=build/ALL/gem5.opt
CFGDIR=configs
BIN=$CFGDIR/oram_workload
SLOTS=32768
OPS=10000           # total ops: 5k writes + 5k reads
OUTDIR=m5out/scaling_sweep
RESULTS=$OUTDIR/results.csv
MAX_JOBS=4

mkdir -p $OUTDIR

# Compile binary
echo "=== Compiling ${BIN}.c ==="
musl-gcc -O0 -static -o "$BIN" "${BIN}.c"
echo "  Binary: $(stat -c '%s %n' $BIN)"
echo ""

extract() {
    local LOG=$1
    local E2E=$(grep "cyc/op" "$LOG" | tail -1 | sed 's/.*Avg: \([0-9.]*\) cyc.*/\1/')
    local RTL=$(grep -i "RTL.*avg\|RTL.*cyc/op" "$LOG" | tail -1 | grep -oP '[0-9]+\.?[0-9]*(?=\s*cyc)' | head -1)
    local RD=$(grep "DDR_READ" "$LOG" | grep "cyc" | tail -1 | grep -oP '\d+(?=\s*cyc)')
    local WR=$(grep "DDR_WRITE" "$LOG" | grep "cyc" | tail -1 | grep -oP '\d+(?=\s*cyc)')
    echo "${E2E:-NA},${RTL:-NA},${RD:-NA},${WR:-NA}"
}

run_one() {
    local TYPE=$1 N=$2 CFG=$3 EXTRA_ARGS=$4
    local TAG="${TYPE}_N${N}"
    local LOG="$OUTDIR/${TAG}.log"

    echo "=== START $TAG ==="
    $GEM5 -d "$OUTDIR/$TAG" "$CFGDIR/$CFG" \
        --binary=$BIN --num-instances=$N --num-ops=$OPS --num-slots=$SLOTS \
        $EXTRA_ARGS > "$LOG" 2>&1

    local VALS=$(extract "$LOG")
    echo "$TYPE,$N,$VALS" > "$OUTDIR/${TAG}.csv"
    echo "=== DONE  $TAG → $VALS ==="
}

echo "=== HBM2 sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "hbm2" $N oram_sim_pcie.py "--local-pct=100 --local-mem=hbm2"
done

echo "=== LPDDR5 1x16 sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "lpddr5_1x16" $N oram_sim_pcie.py "--local-pct=100 --local-mem=lpddr5_1x16"
done

echo "=== LPDDR5 2x16 sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "lpddr5_2x16" $N oram_sim_pcie.py "--local-pct=100 --local-mem=lpddr5_2x16"
done

echo "=== PCIe sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "pcie" $N oram_sim_pcie.py "--local-pct=0"
done

echo "=== CXL sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "cxl" $N oram_sim_cxl.py "--local-pct=0"
done
echo ""

# Merge per-run CSVs into results.csv
echo "type,N,e2e_cyc,rtl_cyc,ddr_read_cyc,ddr_write_cyc" > $RESULTS
cat $OUTDIR/*.csv | sort >> $RESULTS
rm -f $OUTDIR/*.csv

echo "=== RESULTS ==="
column -t -s, $RESULTS
echo ""
echo "CSV: $RESULTS"
