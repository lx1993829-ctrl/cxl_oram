#!/bin/bash
# respfix_run2.sh -- the corrected scheduler of fpga_zcu208_v14c_respfix_full.zip (replaces build_all_respfix.sh).
# Fixes: the card runs no longer set DOMA_PACK (which packs 512 records into one 4 KB block: 4,096 records became 8
# blocks, every access the same bucket); host_leak_ov and card_rate_check.py are found under your home folder.
# It takes over from a running build_all_respfix.sh without disturbing its builds: a design whose build.sh is still
# running counts as building; a design with a build log and no running build counts as built and gets checked; a design
# never started is built. Every design is (re)checked once by this script (marker ~/<name>/checked.v2).
#
#   pkill -f build_all_respfix.sh          # stop the old scheduler only; its running builds carry on
#   ZIP=~/respfix_pkg/pkg.zip nohup bash respfix_run2.sh > ~/respfix_run2.log 2>&1 &
#   ONLY="p3rx300fix optarama_fix" ...      # just these designs;  JOBS=3 builds at once (default)
#
# Checks per design (only if its timing closed), one card program at a time:
#   rate    host_phases get / put / mix / get, n = 4,096, 20% on package, 500,000 commands each: device cycles per
#           command of every phase within 0.2% of the first GET phase, every GET correct
#   leak    overlap only: host_leak_ov acceptance test + card_rate_check.py
#   kqps    host_doma, n = 1,000,000 (one record per block), 0 / 20 / 100% on package, 2,000,000 requests
#   phases  traced build: host_phases mix, n = 1,000,000, 20% on package, the per-phase cycles
set -u
JOBS=${JOBS:-3}; RUN_N=${RUN_N:-1000000}; RUN_REQ=${RUN_REQ:-2000000}; RATE_N=${RATE_N:-500000}; POLL=${POLL:-30}
ZIP=${ZIP:-$HOME/respfix_pkg/pkg.zip}
LEAK_HOST=${LEAK_HOST:-$(find "$HOME" -maxdepth 6 -name host_leak_ov -type f -perm -u+x 2>/dev/null | head -1)}
LEAK_PY=${LEAK_PY:-$(find "$HOME" -maxdepth 6 -name card_rate_check.py -type f 2>/dev/null | head -1)}
RES=$HOME/respfix_results.txt; LOCK=$HOME/.u55c_card.lock
COMMON="SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw"
declare -A ARGS=(
  [p3rx300fix]="OVERLAP=4 P3=1 OV_PMIN=1600 OV_GA=1600 OV_XWB=300 BEATS=128"
  [optarama_fix]="FIXWB=1 RAMA=1 BEATS=128"
  [optarama_tr]="FIXWB=1 RAMA=1 TRACE=1 BEATS=128"
  [rama_b1]="FIXWB=1 RAMA=1 BEATS=1"   [rama_b2]="FIXWB=1 RAMA=1 BEATS=2"   [rama_b4]="FIXWB=1 RAMA=1 BEATS=4"
  [rama_b8]="FIXWB=1 RAMA=1 BEATS=8"   [rama_b16]="FIXWB=1 RAMA=1 BEATS=16" [rama_b32]="FIXWB=1 RAMA=1 BEATS=32"
  [rama_b64]="FIXWB=1 RAMA=1 BEATS=64")
ORDER=${ONLY:-"p3rx300fix optarama_fix optarama_tr rama_b1 rama_b2 rama_b4 rama_b8 rama_b16 rama_b32 rama_b64"}
say() { echo "$(date '+%m-%d %H:%M') $*" | tee -a "$RES"; }
[ -n "${XILINX_XRT:-}" ] || { echo "source /opt/xilinx/xrt/setup.sh (and the Vitis settings64.sh) first"; exit 1; }
command -v v++ > /dev/null || { echo "source /tools/Xilinx/Vitis/2024.1/settings64.sh first"; exit 1; }
for n in $ORDER; do [ -n "${ARGS[$n]:-}" ] || { echo "unknown design $n"; exit 1; }; done
beats_of() { echo "${ARGS[$1]}" | grep -oE "BEATS=[0-9]+" | cut -d= -f2; }

running_build() {   # a "bash build.sh" process working in this design's vitis/ folder (started by either scheduler)
    local want=$HOME/$1/fpga_zcu208/vitis p
    for p in $(pgrep -f "bash build.sh" 2>/dev/null); do [ "$(readlink /proc/$p/cwd 2>/dev/null)" = "$want" ] && return 0; done
    return 1
}
state() {
    local n=$1
    if [ -f "$HOME/$n/checked.v2" ]; then echo checked
    elif [ -f "$HOME/$n/.checking" ]; then echo checking
    elif [ -f "$HOME/$n/.building" ] || running_build "$n"; then echo building
    elif [ -f "$HOME/$n/.built" ] || [ -s "$HOME/$n/build.log" ] || [ -f "$HOME/$n/fpga_zcu208/vitis/doma_hw.xclbin" ]; then echo built
    else echo new; fi
}

check() {   # the card checks of one finished design
    local n=$1 d=$HOME/$1/fpga_zcu208/vitis b blk
    b=$(beats_of "$n"); blk=$((b * 32))
    cd "$d" 2>/dev/null || { say "$n: no package folder: not checked"; return; }
    if [ ! -f doma_hw.xclbin ]; then say "$n: BUILD FAILED (no doma_hw.xclbin; see ~/$n/build.log)"; return; fi
    bash check_timing.sh > ../../timing.txt 2>&1; local t=$?
    if [ $t != 0 ] || grep -q "TIMING NOT CLOSED" ../../build.log; then
        say "$n: built, TIMING NOT CLOSED ($(grep -E 'kernel clock in|WNS=' ../../timing.txt | head -2 | tr '\n' ' ')): not measured"; return; fi
    say "$n: built, timing closed ($(grep -oE 'kernel clock in doma_hw.xclbin: [0-9.]+ MHz' ../../timing.txt))"
    [ -x host_phases ] || g++ -std=c++17 -O2 host_phases.cpp -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread -o host_phases
    (
      flock 9
      say "$n: card checks started"
      local E="DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=$blk"          # one record per block (no DOMA_PACK)
      [ "$n" = p3rx300fix ] && { bash check_overlap_build.sh 4 > ../../overlap_build.txt 2>&1 \
          && say "$n: overlap design present in the placed netlist" || say "$n: check_overlap_build.sh FAILED (~/$n/overlap_build.txt)"; }
      env $E DOMA_N=4096 DOMA_HBM_PCT=20 DOMA_PHASES=get:$RATE_N,put:$RATE_N,mix:$RATE_N,get:$RATE_N ./host_phases doma_hw.xclbin 65536 > ../../rate.log 2>&1
      local cyc; cyc=$(grep -oE "^phase [0-9]+: .*device [0-9.]+ cycles per command" ../../rate.log | sed -E 's/.*device ([0-9.]+) cycles.*/\1/' | tr '\n' ' ')
      if [ "$(echo $cyc | wc -w)" = 4 ] && ! grep -qiE "wrong data|bad header|errors 0x[1-9a-f]|halt" ../../rate.log; then
          python3 -c "import sys; v=[float(x) for x in sys.argv[1:]]; d=max(abs(x-v[0])/v[0] for x in v); print(f'{d*100:.3f}'); sys.exit(0 if d <= 0.002 else 1)" $cyc > ../../rate_dev.txt \
            && say "$n: RATE PASS: device cycles per command get / put / mix / get = $cyc(largest difference $(cat ../../rate_dev.txt)%)" \
            || say "$n: RATE FAIL: get / put / mix / get = $cyc(difference $(cat ../../rate_dev.txt)%, over 0.2%): the operation reaches the schedule"
      else say "$n: RATE run failed, halted or a GET was wrong (~/$n/rate.log)"; fi
      if [ "$n" = p3rx300fix ]; then
          if [ -n "$LEAK_HOST" ] && [ -n "$LEAK_PY" ]; then
              env DOMA_LAMBDA=7 DOMA_N=4096 DOMA_HBM_PCT=20 DOMA_PHASES=get:500000,put:500000,zero:500000,mix:500000,fixed:500000,rereq:500000,scan:500000,get:500000 \
                  "$LEAK_HOST" doma_hw.xclbin 65536 > ../../leak_rate.log 2>&1
              python3 "$LEAK_PY" ../../leak_rate.log > ../../leak_check.txt 2>&1
              say "$n: LEAK acceptance ($LEAK_HOST): $(grep -E 'RATE |ALL PHASES|PHASE' ../../leak_check.txt | tr '\n' ' ') (~/$n/leak_check.txt)"
          else say "$n: LEAK acceptance skipped: host_leak_ov or card_rate_check.py not found under $HOME (set LEAK_HOST= and LEAK_PY=)"; fi
      fi
      if [ "$n" != optarama_tr ]; then
          for p in 0 20 100; do
              env $E DOMA_N=$RUN_N DOMA_HBM_PCT=$p ./host_doma doma_hw.xclbin $RUN_REQ > ../../kqps_$p.log 2>&1
              say "$n: $p% on package, $RUN_N records, $RUN_REQ requests: $(grep -oE '[0-9.]+ KQPS' ../../kqps_$p.log | tail -1), $(grep -E 'ALL RESPONSES CORRECT|ERRORS FOUND' ../../kqps_$p.log | tail -1)$(grep -qiE 'halt' ../../kqps_$p.log && echo ', HALT (see the log)')"
          done
      else
          env $E DOMA_N=$RUN_N DOMA_HBM_PCT=20 DOMA_PHASE_TRACE=1 DOMA_PHASES=mix:200000 ./host_phases doma_hw.xclbin 65536 > ../../phases.log 2>&1
          say "$n: phases at 20% on package, $RUN_N records: $(grep -E '^phase 1: phases, mean cycles' ../../phases.log | sed 's/^phase 1: //')"
      fi
      say "$n: card checks done"
    ) 9> "$LOCK"
}

start_build() {   # synchronously marks the design building, then unzips and builds it in the background
    local n=$1
    rm -rf "$HOME/$n"; mkdir -p "$HOME/$n"; touch "$HOME/$n/.building"
    (
      if ! (cd "$HOME/$n" && unzip -q "$ZIP"); then say "$n: unzip of $ZIP failed"; echo "unzip failed" > "$HOME/$n/build.log"; touch "$HOME/$n/.built"; rm -f "$HOME/$n/.building"; exit; fi
      badf=$(cd "$HOME/$n/fpga_zcu208" && sha256sum -c MANIFEST.sha256 2>/dev/null | grep -vc ': OK$')
      if [ "$badf" != 0 ]; then say "$n: $badf package files fail their checksum: not built"; echo "checksum failed" > "$HOME/$n/build.log"; touch "$HOME/$n/.built"; rm -f "$HOME/$n/.building"; exit; fi
      say "$n: build started (${ARGS[$n]})"
      ( cd "$HOME/$n/fpga_zcu208/vitis" && env ${ARGS[$n]} $COMMON bash build.sh ) > "$HOME/$n/build.log" 2>&1
      say "$n: build ended"
      touch "$HOME/$n/.built"; rm -f "$HOME/$n/.building"
    ) &
}

say "== respfix_run2 start: $ORDER ($JOBS builds at once; card checks one at a time; host_leak_ov: ${LEAK_HOST:-not found})"
for n in $ORDER; do rm -f "$HOME/$n/.checking"; say "$n: $(state $n)"; done
while :; do
    nb=0; for n in $ORDER; do [ "$(state $n)" = building ] && nb=$((nb + 1)); done
    left=0
    for n in $ORDER; do
        case $(state $n) in
          new)   if [ "$nb" -lt "$JOBS" ]; then
                     if [ ! -f "$ZIP" ]; then say "$n: package zip $ZIP not found: not built"; mkdir -p "$HOME/$n"; echo "no zip" > "$HOME/$n/build.log"; touch "$HOME/$n/.built"
                     else start_build "$n"; nb=$((nb + 1)); sleep 5; fi
                 fi; left=$((left + 1)) ;;
          built) touch "$HOME/$n/.checking"; ( check "$n" > "$HOME/$n/check.log" 2>&1; touch "$HOME/$n/checked.v2"; rm -f "$HOME/$n/.checking" ) &
                 left=$((left + 1)) ;;
          checked) ;;
          *)     left=$((left + 1)) ;;
        esac
    done
    [ "$left" = 0 ] && break
    sleep "$POLL"
done
wait
say "== respfix_run2 done: summary in $RES"
