#!/bin/bash
OUTDIR=m5out/scaling_sweep
GEM5=build/ALL/gem5.opt
BIN=configs/oram_workload
OPS=10000
SLOTS=32768

# Compile binary
echo "=== Compiling ${BIN}.c ==="
musl-gcc -O0 -static -o "$BIN" "${BIN}.c"
echo "  Binary: $(stat -c '%s %n' $BIN)"
echo ""

extract() {
    local LOG=$1
    local E2E=$(grep "Avg:.*cyc/op" "$LOG" | grep -oP '[0-9.]+(?= cyc)' | awk '{s+=$1;n++} END{if(n>0) printf "%.1f",s/n; else print "NA"}')
    local RD=$(grep "DDR_READ" "$LOG" | grep -oP '\d+(?=\s*cyc)' | awk '{s+=$1;n++} END{if(n>0) printf "%.1f",s/n; else print "NA"}')
    local WR=$(grep "DDR_WRITE" "$LOG" | grep -oP '\d+(?=\s*cyc)' | awk '{s+=$1;n++} END{if(n>0) printf "%.1f",s/n; else print "NA"}')
    echo "${E2E},${RD},${WR}"
}

run_one() {
    local TYPE=$1 N=$2 CFG=$3 EXTRA=$4
    local TAG="${TYPE}_N${N}"
    local LOG="$OUTDIR/${TAG}.log"
    echo "=== START $TAG ==="
    $GEM5 -d "$OUTDIR/$TAG" "configs/$CFG" \
        --binary=$BIN --num-instances=$N --num-ops=$OPS --num-slots=$SLOTS \
        $EXTRA > "$LOG" 2>&1 || true
    if grep -q "fatal\|Aborted" "$LOG"; then
        echo "$TYPE,$N,CRASH,CRASH,CRASH" > "$OUTDIR/${TAG}.tmp"
        echo "=== CRASH $TAG ==="
    else
        VALS=$(extract "$LOG")
        echo "$TYPE,$N,$VALS" > "$OUTDIR/${TAG}.tmp"
        echo "=== DONE  $TAG → $VALS ==="
    fi
}

echo "=== PCIe N=7,8 ==="
for N in 7 8; do
    run_one "pcie" $N oram_sim_pcie.py "--local-pct=0"
done

echo "=== CXL N=1..8 ==="
for N in 1 2 3 4 5 6 7 8; do
    run_one "cxl" $N oram_sim_cxl.py "--local-pct=0"
done

echo ""
echo "=== Append new results ==="
if [ ! -f $OUTDIR/results.csv ]; then
    echo "type,N,e2e_cyc,ddr_read_cyc,ddr_write_cyc" > $OUTDIR/results.csv
fi
cat $OUTDIR/*.tmp | sort >> $OUTDIR/results.csv
rm -f $OUTDIR/*.tmp
echo ""
echo "=== ALL RESULTS ==="
column -t -s, $OUTDIR/results.csv
