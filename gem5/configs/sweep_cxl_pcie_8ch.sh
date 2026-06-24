#!/bin/bash
# =============================================================================
# sweep_cxl_pcie_8ch.sh — CXL vs PCIe 8-channel instance scaling sweep: N=1..8
#
# Uses oram_sim_cxl_8ch.py and oram_sim_pcie_8ch.py (8 DDR5 subchannels)
#
# Output:
#   m5out/scaling_sweep_8ch/results.csv
#   m5out/scaling_sweep_8ch/{type}_N{n}.log
#
# Usage:
#   cd /mnt/f/gem5
#   bash configs/sweep_cxl_pcie_8ch.sh
# =============================================================================
set -e
GEM5=build/ALL/gem5.opt
CFGDIR=configs
BIN=$CFGDIR/oram_workload
SLOTS=32768
OPS=10000           # total ops: 5k writes + 5k reads
OUTDIR=m5out/scaling_sweep_8ch
RESULTS=$OUTDIR/results.csv
mkdir -p $OUTDIR

extract() {
    local LOG=$1
    local E2E=$(grep "Avg:.*cyc/op" "$LOG" | \
        grep -oP '[0-9.]+(?= cyc)' | \
        awk '{s+=$1;n++} END{if(n>0) printf "%.1f",s/n; else print "NA"}')
    local RD=$(grep "DDR_READ" "$LOG" | \
        grep -oP '\d+(?=\s*cyc)' | \
        awk '{s+=$1;n++} END{if(n>0) printf "%.1f",s/n; else print "NA"}')
    local WR=$(grep "DDR_WRITE" "$LOG" | \
        grep -oP '\d+(?=\s*cyc)' | \
        awk '{s+=$1;n++} END{if(n>0) printf "%.1f",s/n; else print "NA"}')
    echo "${E2E},${RD},${WR}"
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
    echo "$TYPE,$N,$VALS" > "$OUTDIR/${TAG}.tmp"
    echo "=== DONE  $TAG → $VALS ==="
}

echo "=== PCIe 8ch sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "pcie_8ch" $N oram_sim_pcie_8ch.py "--local-pct=0"
done

echo "=== CXL 8ch sweep ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "cxl_8ch" $N oram_sim_cxl_8ch.py "--local-pct=0"
done

echo ""
echo "type,N,e2e_cyc,ddr_read_cyc,ddr_write_cyc" > $RESULTS
cat $OUTDIR/*.tmp | sort >> $RESULTS
rm -f $OUTDIR/*.tmp

echo "=== RESULTS ==="
column -t -s, $RESULTS
echo ""
echo "CSV: $RESULTS"
