#!/bin/bash
# =============================================================================
# sweep_local_pct.sh — Local memory percentage sweep: PCIe vs CXL
#
# Varies --local-pct from 0% to 100% in 10% steps.
# N=1, slots=32768, ops=100000, local memory=HBM2, host memory=DDR5-6400.
# Runs up to MAX_JOBS simulations in parallel.
#
# Output:
#   m5out/local_pct_sweep/results.csv              — all metrics per run
#   m5out/local_pct_sweep/{fabric}_pct{P}.log      — full gem5 log per run
#   m5out/local_pct_sweep/{fabric}_pct{P}/         — gem5 stats/config per run
#
# Usage:
#   cd /mnt/f/gem5
#   bash configs/sweep_local_pct.sh
# =============================================================================
set -e

GEM5=build/ALL/gem5.opt
CFGDIR=configs
BIN=$CFGDIR/oram_workload
SLOTS=32768
OPS=10000            # total ops: 5k writes + 5k reads
N=1
OUTDIR=m5out/local_pct_sweep
RESULTS=$OUTDIR/results.csv
MAX_JOBS=1

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

run_pct() {
    local FABRIC=$1 PCT=$2 CFG=$3 EXTRA=$4
    local TAG="${FABRIC}_pct${PCT}"
    local LOG="$OUTDIR/${TAG}.log"

    echo "=== START $TAG ==="
    $GEM5 -d "$OUTDIR/$TAG" "$CFGDIR/$CFG" \
        --binary=$BIN --num-instances=$N --num-ops=$OPS --num-slots=$SLOTS \
        --local-pct=$PCT $EXTRA > "$LOG" 2>&1

    local VALS=$(extract "$LOG")
    echo "$FABRIC,$PCT,$VALS" > "$OUTDIR/${TAG}.csv"
    echo "=== DONE  $TAG → $VALS ==="
}

echo "=== PCIe sweep ==="
for PCT in 0 10 20 30 40 50 60 70 80 90 100; do
    run_pct "pcie" $PCT oram_sim_pcie.py "--local-mem=hbm2"
done

echo "=== CXL sweep ==="
for PCT in 0 10 20 30 40 50 60 70 80 90 100; do
    run_pct "cxl"  $PCT oram_sim_cxl.py  ""
done
echo ""

# Merge per-run CSVs into results.csv
echo "fabric,local_pct,e2e_cyc,rtl_cyc,ddr_read_cyc,ddr_write_cyc" > $RESULTS
cat $OUTDIR/*.csv | sort >> $RESULTS
rm -f $OUTDIR/*.csv

echo "=== RESULTS ==="
column -t -s, $RESULTS
echo ""
echo "CSV: $RESULTS"
