#!/bin/bash
# baseline_all.sh -- build, measure and clean up the non-oblivious baseline (PLAIN=1) for every block size.
#
#   nohup bash baseline_all.sh > ~/baseline_all.log 2>&1 &        # then: tail -f ~/baseline_all.log
#
# For each block size (32 B, 4 KB, 64 B ... 2 KB), in two build lanes:
#   * all results already recorded  -> save the design's bitstreams and log if present, delete the design;
#   * design built and matching     -> measure, record, save, delete;
#   * design building               -> wait for it;
#   * design missing / not matching -> build it (PLAIN=1), then measure, record, save, delete.
# Measuring (one test at a time on the card, under a lock): the bitstream as built (300 MHz), then a copy with its
# kernel clock set to 230 MHz (a different bitstream is loaded in between, so XRT really reprograms the clock).
# Every row records the measured clock (device cycles / query time) and whether every response was correct; a design
# is deleted only when all its rows are correct and every clock is within range. 8- and 16-byte values use the 32-byte
# slot bitstream (8 B packed 4 per slot; 16 B: same single-beat access, not run separately: the host's value size is 8).
# Results: ~/keep/results/baseline/baseline_results.csv. Bitstreams: ~/keep/bitstreams/baseline/.
# DRYRUN=1: no builds and no card, every step simulated (for testing this script).
set -u
PKG=${PKG:-$HOME/pkg_hbm8/fpga_zcu208}                 # package with PLAIN=1 and doma_big8.cfg
FLUSH_DIR=${FLUSH_DIR:-$HOME/doma_z8_b128/vitis}      # another bitstream + host_doma, loaded between 300 and 230 MHz
CMDS=${CMDS:-20000}
DRY=${DRYRUN:-0}
SLOTS=${SLOTS:-"32 4096 64 128 256 512 1024 2048"}    # first the two the comparisons need first
KEEP=$HOME/keep; RES=$KEEP/results/baseline/baseline_results.csv; BITS=$KEEP/bitstreams/baseline; LOGS=$KEEP/results/baseline
Q=$KEEP/results/baseline/.queue; LOCK_Q=/tmp/baseline_queue.lock; LOCK_CARD=/tmp/baseline_card.lock
mkdir -p "$BITS" "$LOGS"
[ -f "$RES" ] || echo "slot_b,value_b,per_slot,clock_mhz,n,kqps,device_cycles,cycles_per_cmd,measured_mhz,correct,clock_ok,date" > "$RES"
[ "$DRY" = 1 ] || { source /tools/Xilinx/Vitis/2024.1/settings64.sh; source /opt/xilinx/xrt/setup.sh; }
say() { echo "$(date '+%F %T') [$1 B] $2"; }

nmax()  { [ "$1" = 32 ] && echo 4194304 || echo 2097152; }
cfg()   { [ "$1" = 32 ] && echo "" || echo doma_big8.cfg; }
tests() {                        # one line per run: value_bytes records_per_slot n
    if [ "$1" = 32 ]; then
        for n in 100000 200000 500000 1000000 2000000 5000000 10000000; do echo "8 4 $n"; done
        for n in 100000 200000 500000 1000000 2000000; do echo "32 1 $n"; done
    else
        for n in 100000 200000 500000 1000000 2000000; do echo "$1 1 $n"; done
    fi
}
recorded()     { grep -q "^$1,$2,$3,$4,$5," "$RES"; }
all_recorded() { local s=$1 v p n c; while read v p n; do for c in 300 230; do recorded $s $v $p $c $n || return 1; done; done < <(tests $s); }
all_good()     { awk -F, -v s=$1 '$1==s && ($10!=1 || $11!=1){bad=1} END{exit bad}' "$RES"; }
building()     { local p; for p in $(pgrep -f "vivado|v\+\+" 2>/dev/null); do case "$(readlink /proc/$p/cwd 2>/dev/null)" in "$HOME/plain_b$1"|"$HOME/plain_b$1"/*) return 0;; esac; done; return 1; }
built_ok()     { grep -aq "PASS: timing closed at 300 MHz" "$1/vitis/build.log" 2>/dev/null && [ -f "$1/vitis/doma_hw.xclbin" ]; }
matches() {                      # the design is the baseline for this size, with enough blocks and the right layout
    local s=$1 d=$2 c=$d/rtl/oram_build_cfg.vh
    grep -q 'define ORAM_PLAIN' "$c" 2>/dev/null || return 1
    grep -qE "define ORAM_BLOCK_BEATS $(( s / 32 ))\b" "$c" || return 1
    [ "$(grep -oE 'define ORAM_N [0-9]+' "$c" | awk '{print $3}')" -ge "$(nmax $s)" ] 2>/dev/null || return 1
    [ "$s" = 32 ] || grep -q "cfg=doma_big8.cfg" "$d/.baseline_sig" 2>/dev/null
}

import_32() {                    # step 1: the 32-byte runs already made (run_two.sh), if not yet recorded
    local f r
    for f in "$HOME/plain_b32/vitis/plain32_sweep.csv" "$LOGS/plain32_sweep.csv"; do [ -f "$f" ] && break; done
    [ -f "$f" ] || return 0
    tail -n +2 "$f" | while IFS=, read clk n k cyc mhz ok; do
        c=${clk%MHz}; recorded 32 8 4 $c $n && continue
        awk -v c=$c -v n=$n -v k=$k -v cyc=$cyc -v mhz=$mhz -v ok=$ok -v cmds=$CMDS 'BEGIN{
            lo = (c == 300) ? 280 : 215; hi = (c == 300) ? 310 : 240
            printf "32,8,4,%s,%s,%s,%s,%.1f,%s,%s,%d,imported\n", c, n, k, cyc, cyc / cmds, mhz, ok, (mhz >= lo && mhz <= hi)}' >> "$RES"
    done
    say 32 "imported the earlier 32-byte runs from $f"
}

host_run() {                     # $1 dir $2 xclbin $3 slot $4 per_slot $5 n -> prints "kqps cycles time ok"
    if [ "$DRY" = 1 ]; then
        local mhz=300; case "$2" in *230mhz*) mhz=230;; esac
        echo "2100.0 $(( CMDS * 138 * mhz / 300 )) $(awk -v c=$CMDS 'BEGIN{print c / 2100000}') 1"; return
    fi
    local out
    out=$(cd "$1" && if [ "$4" -gt 1 ]; then DOMA_PACK=1; export DOMA_PACK; else unset DOMA_PACK; fi
          DOMA_BLOCK=$3 DOMA_N=$5 DOMA_HBM_PCT=75 DOMA_POOL=64 ./host_doma "$2" $CMDS 2>&1)
    echo "$out" >> "$1/baseline_runs.log"
    echo "$(echo "$out" | grep -oE '= [0-9.]+ KQPS' | awk '{print $2}') $(echo "$out" | grep -oE '^device: [0-9]+' | awk '{print $2}') $(echo "$out" | grep -oE 'in [0-9.]+ s' | awk '{print $2}') $(echo "$out" | grep -c 'ALL RESPONSES CORRECT')"
}
measure_clock() {                # $1 slot $2 dir $3 xclbin $4 clock label
    local s=$1 d=$2 x=$3 c=$4 v p n k cyc t ok mhz lo hi
    lo=$([ $c = 300 ] && echo 280 || echo 215); hi=$([ $c = 300 ] && echo 310 || echo 240)
    while read v p n; do
        recorded $s $v $p $c $n && continue
        [ "$DRY" = 1 ] || sleep 3
        read k cyc t ok < <(host_run "$d" "$x" $s $p $n)
        mhz=$(awk -v cyc=${cyc:-0} -v t=${t:-0} 'BEGIN{ if (t > 0) printf "%.1f", cyc / t / 1e6; else print 0 }')
        echo "$s,$v,$p,$c,$n,${k:-0},${cyc:-0},$(awk -v cyc=${cyc:-0} -v m=$CMDS 'BEGIN{printf "%.1f", cyc / m}'),$mhz,${ok:-0},$(awk -v m=$mhz -v lo=$lo -v hi=$hi 'BEGIN{print (m >= lo && m <= hi) ? 1 : 0}'),$(date +%F_%T)" >> "$RES"
        say $s "$c MHz, ${v}-byte values ($p per slot), N=$n: ${k:-?} KQPS, measured ${mhz} MHz, correct ${ok:-0}"
    done < <(tests $s)
}
measure() {                      # $1 slot $2 dir: both clocks, one test at a time on the card
    local s=$1 v=$2/vitis
    (   flock 9
        if [ "$DRY" != 1 ]; then
            [ -x "$v/host_doma" ] || (cd "$v" && g++ -std=c++17 -O2 host_doma.cpp -I$XILINX_XRT/include -L$XILINX_XRT/lib -lxrt_coreutil -pthread -o host_doma)
            [ -x "$FLUSH_DIR/host_doma" ] || (cd "$FLUSH_DIR" && g++ -std=c++17 -O2 host_doma.cpp -I$XILINX_XRT/include -L$XILINX_XRT/lib -lxrt_coreutil -pthread -o host_doma)
        fi
        measure_clock $s "$v" doma_hw.xclbin 300
        if [ "$DRY" = 1 ]; then cp "$v/doma_hw.xclbin" "$v/doma_hw_230mhz.xclbin"
        else
            ( cd "$v" && xclbinutil --input doma_hw.xclbin --dump-section CLOCK_FREQ_TOPOLOGY:JSON:clk300.json --force > /dev/null &&
              python3 - <<'PY' && xclbinutil --input doma_hw.xclbin --replace-section CLOCK_FREQ_TOPOLOGY:JSON:clk230.json --output doma_hw_230mhz.xclbin --force > /dev/null ) || { say $s "could not make the 230 MHz copy"; exit 1; }
import json, sys
j = json.load(open("clk300.json")); hits = []
def walk(o):
    if isinstance(o, dict):
        if str(o.get("m_type", "")).upper() == "DATA" and "m_freq_Mhz" in o: hits.append(o)
        for x in o.values(): walk(x)
    elif isinstance(o, list):
        for x in o: walk(x)
walk(j)
if len(hits) != 1: sys.exit("unexpected clock section")
hits[0]["m_freq_Mhz"] = "230"; json.dump(j, open("clk230.json", "w"), indent=4)
PY
            ( cd "$FLUSH_DIR" && DOMA_N=4096 ./host_doma doma_hw.xclbin 10 > /dev/null 2>&1 )      # load another bitstream
        fi
        measure_clock $s "$v" doma_hw_230mhz.xclbin 230
    ) 9> "$LOCK_CARD"
}
finish() {                       # $1 slot $2 dir: save the artifacts, delete the design if everything checked out
    local s=$1 d=$2
    if ! all_recorded $s; then say $s "not every run is recorded: design kept in $d"; return; fi
    if ! all_good $s; then say $s "a run was incorrect or its clock out of range (see $RES): design kept in $d"; return; fi
    if [ -d "$d" ]; then
        [ -f "$d/vitis/doma_hw.xclbin" ] && { [ -e "$BITS/plain_${s}B_300mhz.xclbin" ] || cp "$d/vitis/doma_hw.xclbin" "$BITS/plain_${s}B_300mhz.xclbin"; }
        [ -f "$d/vitis/doma_hw_230mhz.xclbin" ] && { [ -e "$BITS/plain_${s}B_230mhz.xclbin" ] || cp "$d/vitis/doma_hw_230mhz.xclbin" "$BITS/plain_${s}B_230mhz.xclbin"; }
        [ -f "$d/vitis/build.log" ] && { [ -e "$LOGS/build_plain_${s}B.log" ] || cp "$d/vitis/build.log" "$LOGS/build_plain_${s}B.log"; }
        [ -f "$d/vitis/baseline_runs.log" ] && cp "$d/vitis/baseline_runs.log" "$LOGS/runs_plain_${s}B.log"
        rm -rf "$d"; say $s "results complete, bitstreams saved, design deleted"
    else say $s "results complete"; fi
}
build() {                        # $1 slot $2 dir
    local s=$1 d=$2 c
    [ -d "$PKG" ] || { say $s "package $PKG not found: nothing built"; return 1; }
    rm -rf "$d" && cp -r "$PKG" "$d" || { say $s "could not copy the package to $d"; return 1; }
    c=$(cfg $s); echo "slot=$s nmax=$(nmax $s) cfg=${c:-default}" > "$d/.baseline_sig"
    say $s "building (BEATS=$(( s / 32 )), NMAX=$(nmax $s), layout ${c:-default})"
    if [ "$DRY" = 1 ]; then
        printf '`define ORAM_BLOCK_BEATS %s\n`define ORAM_N %s\n`define ORAM_PLAIN\n' $(( s / 32 )) $(nmax $s) > "$d/rtl/oram_build_cfg.vh"
        echo "PASS: timing closed at 300 MHz" > "$d/vitis/build.log"; echo bitstream > "$d/vitis/doma_hw.xclbin"; return
    fi
    ( cd "$d/vitis" && env PLAIN=1 BEATS=$(( s / 32 )) NMAX=$(nmax $s) SCHED=arrival TARGET=hw ${c:+CFG=$c} bash build.sh > build.log 2>&1 )
}
process() {                      # one block size, start to finish
    local s=$1 d=$HOME/plain_b$1
    if all_recorded $s; then finish $s "$d"; return; fi
    while building $s; do say $s "a build is running in $d: waiting"; sleep 600; done
    if [ -d "$d" ] && built_ok "$d" && matches $s "$d"; then say $s "existing design matches: measuring"
    else
        [ -d "$d" ] && { [ -f "$d/vitis/build.log" ] && cp "$d/vitis/build.log" "$LOGS/build_plain_${s}B_replaced.log"; say $s "existing design missing, failed or not matching (blocks/layout): rebuilding"; }
        build $s "$d" || return
        built_ok "$d" || { say $s "build FAILED or not at 300 MHz: see $d/vitis/build.log (design kept)"; return; }
    fi
    measure $s "$d" && finish $s "$d"
}
next() { ( flock 8; s=$(head -1 "$Q" 2>/dev/null); [ -n "$s" ] && sed -i 1d "$Q"; echo "$s" ) 8> "$LOCK_Q"; }
lane() { local s; while s=$(next) && [ -n "$s" ]; do process $s; done; }

import_32
printf '%s\n' $SLOTS > "$Q"
lane & lane & wait
echo "$(date '+%F %T') all block sizes done; results in $RES"
