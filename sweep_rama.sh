#!/bin/bash
# sweep_rama.sh -- two sweeps of the baseline (Option A + RAMA, response-write fix) on the U55C, each run 1,000,000
# commands, a random 50% GET / 50% PUT mix, 20% of the bucket data in HBM, every GET checked (host_phases, after the
# preload of every record):
#   1. records N = 100K, 200K, 1M, 2M at 64 B blocks (~/rama_b2)
#   2. value size 16 B .. 4 KB at N = 1M: 16 B and 32 B on the 32 B block build (~/rama_b1; a 16 B value fills half of
#      a 32 B block, so its run is the 32 B run), then 64 B .. 2 KB (~/rama_b2 .. ~/rama_b64) and 4 KB (~/optarama_fix)
# Runs hold the card lock (~/.u55c_card.lock), so they never share the card with another run of these scripts.
# Results: ~/sweep_results/summary.txt (and a CSV, sweep.csv); each run's full output in ~/sweep_results/<name>.log.
#   bash sweep_rama.sh            (CMDS=1000000 HBM=20 to change)
set -u
source /opt/xilinx/xrt/setup.sh > /dev/null
# CLK=<MHz> (e.g. 200): the 300 MHz builds, each run from a copy of its bitstream with the kernel clock lowered to CLK
# (set_clock.sh, next to this script or in ~); results in ~/sweep_results_clk<MHz>. MHZ=<MHz>: builds made with
# build.sh MHZ=<MHz> (folders ~/<build>_<MHz>). Each run line also gives the clock measured on the card.
CMDS=${CMDS:-1000000}; HBM=${HBM:-20}; MHZ=${MHZ:-300}; SUF=$([ "$MHZ" = 300 ] || echo "_$MHZ")
SETCLK=$(ls "$(dirname "$0")/set_clock.sh" "$HOME/set_clock.sh" 2>/dev/null | head -1)
[ -n "${CLK:-}" ] && { [ -n "$SETCLK" ] || { echo "CLK= needs set_clock.sh next to this script or in ~"; exit 1; }; }
R=$HOME/sweep_results$SUF${CLK:+_clk$CLK}; mkdir -p "$R"; CSV=$R/sweep.csv
log() { echo "$(date '+%F %T') $*" | tee -a "$R/summary.txt"; }
[ -f "$CSV" ] || echo "sweep,value_B,block_B,build,N,KQPS,us_per_cmd,device_cycles_per_cmd,get_lat_cyc,put_lat_cyc,p50_cyc,p99_cyc,max_cyc,correct" > "$CSV"

run() {   # run <sweep> <value bytes> <build folder> <N>
    local sw=$1 val=$2 b=$3$SUF n=$4 d=$HOME/$3$SUF/fpga_zcu208/vitis blk name f
    [ -f "$d/doma_hw.xclbin" ] || { log "$sw value $val B N $n: no bitstream in $d: skipped"; return; }
    grep -q "PASS: timing closed at $MHZ MHz" "$HOME/$b/build.log" "$HOME/$b/timing.txt" 2>/dev/null \
        || { log "$sw value $val B N $n: $b did not close at $MHZ MHz: skipped"; return; }
    cd "$d" || return
    [ -x host_phases ] || g++ -std=c++17 -O2 host_phases.cpp -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread -o host_phases
    blk=$(( $(head -1 "$HOME/$b/build.log" | grep -oE 'block size: [0-9]+ beats' | grep -oE '[0-9]+') * 32 ))
    name=${sw}_v${val}_n${n}; f=$R/$name.log
    local X=doma_hw.xclbin                  # CLK=<MHz> below the build's clock: a copy with that clock (set_clock.sh)
    if [ -n "${CLK:-}" ]; then X=$R/xclbin/${b}_${CLK}mhz.xclbin; mkdir -p "$R/xclbin"
        [ -f "$X" ] || bash "$SETCLK" doma_hw.xclbin "$CLK" "$X" >> "$R/summary.txt" 2>&1 || { log "$sw value $val B N $n: set_clock to $CLK MHz failed"; return; }; fi
    flock "$HOME/.u55c_card.lock" env DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=$blk DOMA_N=$n DOMA_HBM_PCT=$HBM \
        DOMA_PHASES=mix:$CMDS timeout 3600 ./host_phases "$X" 65536 > "$f" 2>&1
    local ph mx lt ok kq us cy gl pl p50 p99 mxl
    ph=$(grep -aiE '^phase 1: [0-9]+ mix' "$f"); mx=$(grep -aE '^phase 1: mix of' "$f"); lt=$(grep -aE '^phase 1: latency' "$f")
    ok=$(grep -aqE '^ALL PHASES CORRECT' "$f" && echo yes || echo NO)
    kq=$(echo "$ph" | grep -oE '[0-9.]+ KQPS' | cut -d' ' -f1); us=$(echo "$ph" | grep -oE '[0-9.]+ us per' | cut -d' ' -f1)
    cy=$(echo "$ph" | grep -oE 'device [0-9.]+' | cut -d' ' -f2)
    gl=$(echo "$mx" | grep -oE 'GET \(mean latency [0-9.]+' | grep -oE '[0-9.]+$'); pl=$(echo "$mx" | grep -oE 'PUT \(mean latency [0-9.]+' | grep -oE '[0-9.]+$')
    p50=$(echo "$lt" | grep -oE 'p50 [0-9]+' | cut -d' ' -f2); p99=$(echo "$lt" | grep -oE 'p99 [0-9]+' | cut -d' ' -f2); mxl=$(echo "$lt" | grep -oE 'max [0-9]+' | cut -d' ' -f2)
    local mc; mc=$(awk -v c="${cy:-0}" -v u="${us:-0}" 'BEGIN{if (u > 0) printf "%.0f", c / u}')
    log "$sw: value $val B (block $blk B, $b), N $n, clock measured ${mc:-?} MHz: ${kq:-?} KQPS, ${us:-?} us/cmd, device ${cy:-?} cycles/cmd; latency GET ${gl:-?} / PUT ${pl:-?} cycles, p50 ${p50:-?} p99 ${p99:-?} max ${mxl:-?}; all correct: $ok"
    echo "$sw,$val,$blk,$b,$n,$kq,$us,$cy,$gl,$pl,$p50,$p99,$mxl,$ok" >> "$CSV"
}

log "== kernel clock ${CLK:-$MHZ} MHz$([ -n "${CLK:-}" ] && echo " (300 MHz builds, clock lowered at load)")"; log "== sweep 1: N at 64 B blocks, $CMDS commands, 50/50 GET/PUT, $HBM% in HBM"
for n in 100000 200000 1000000 2000000; do run N 64 rama_b2 $n; done
log "== sweep 2: value size at N = 1M, $CMDS commands, 50/50 GET/PUT, $HBM% in HBM"
run V 16 rama_b1 1000000
log "V: value 32 B uses the same 32 B block build as 16 B: its run is the one above"
echo "V,32,32,rama_b1,1000000,(same run as 16 B),,,,,,,," >> "$CSV"
for vb in "64 rama_b2" "128 rama_b4" "256 rama_b8" "512 rama_b16" "1024 rama_b32" "2048 rama_b64" "4096 optarama_fix"; do
    set -- $vb; run V $1 $2 1000000; done
log "== done ($CSV)"
