#!/bin/bash
# krt_run.sh -- builds the phase 3 overlap design with the run-time in-flight cap (one bitstream for K = 2, 3, 4) in
# ~/p3rx300krt, then measures it at K = 4, 3, 2 on the card (one card program at a time, ~/.u55c_card.lock).
#   source /opt/xilinx/xrt/setup.sh; source /tools/Xilinx/Vitis/2024.1/settings64.sh
#   ZIP=~/fpga_zcu208_v14c_respfix_krt_full.zip nohup bash krt_run.sh > ~/krt_run.log 2>&1 &
#   tail -f ~/krt_results.txt
# Rerunning skips the build when ~/p3rx300krt has a bitstream (CHECK_ONLY=1 never builds).
# Per K: rate   host_phases get / put / mix / get, n = 4,096, 20% on package, 500,000 commands each: device cycles
#               per command within 0.2% of the first GET phase, every GET correct
#        kqps   host_doma, n = 1,000,000, 0 / 20 / 100% on package, 2,000,000 requests
# K = 4 should match p3rx300fix (187.1 / 187.2 / 187.3 KQPS).
set -u
N=p3rx300krt; D=$HOME/$N; V=$D/fpga_zcu208/vitis; RES=$HOME/krt_results.txt; LOCK=$HOME/.u55c_card.lock
ZIP=${ZIP:-$HOME/fpga_zcu208_v14c_respfix_krt_full.zip}
ARGS="OVERLAP=4 P3=1 OV_PMIN=1600 OV_GA=1600 OV_XWB=300 BEATS=128 SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw"
say() { echo "$(date '+%m-%d %H:%M') $*" | tee -a "$RES"; }
[ -n "${XILINX_XRT:-}" ] || { echo "source /opt/xilinx/xrt/setup.sh first"; exit 1; }
if [ ! -f "$V/doma_hw.xclbin" ] && [ "${CHECK_ONLY:-0}" != 1 ]; then
    command -v v++ > /dev/null || { echo "source /tools/Xilinx/Vitis/2024.1/settings64.sh first"; exit 1; }
    [ -f "$ZIP" ] || { echo "no package $ZIP"; exit 1; }
    rm -rf "$D"; mkdir -p "$D"; (cd "$D" && unzip -q "$ZIP") || { say "$N: unzip failed"; exit 1; }
    bad=$(cd "$D/fpga_zcu208" && sha256sum -c MANIFEST.sha256 2>/dev/null | grep -vc ': OK$')
    [ "$bad" = 0 ] || { say "$N: $bad package files fail their checksum: not built"; exit 1; }
    grep -q "cfg_ov_k" "$D/fpga_zcu208/rtl/flat_oram_gcm.v" || { say "$N: the package has no run-time K"; exit 1; }
    say "$N: build started ($ARGS)"
    ( cd "$V" && env $ARGS bash build.sh ) > "$D/build.log" 2>&1
    say "$N: build ended"
fi
cd "$V" || exit 1
[ -f doma_hw.xclbin ] || { say "$N: BUILD FAILED (no doma_hw.xclbin; see $D/build.log)"; exit 1; }
WANT_MHZ=300 bash check_timing.sh > "$D/timing.txt" 2>&1; t=$?
if [ $t != 0 ] || grep -q "TIMING NOT CLOSED" "$D/build.log"; then
    say "$N: TIMING NOT CLOSED ($(grep -E 'kernel clock in|WNS=' "$D/timing.txt" | head -2 | tr '\n' ' ')): not measured"; exit 1; fi
say "$N: timing closed ($(grep -oE 'kernel clock in doma_hw.xclbin: [0-9.]+ MHz' "$D/timing.txt"))"
bash check_overlap_build.sh 4 > "$D/overlap_build.txt" 2>&1 && say "$N: overlap design present in the placed netlist" \
    || say "$N: check_overlap_build.sh FAILED ($D/overlap_build.txt)"
for h in host_phases host_doma; do
    g++ -std=c++17 -O2 $h.cpp -I. -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread -o $h || { say "$N: $h does not compile"; exit 1; }
    grep -q DOMA_OV_K $h.cpp || { say "$N: $h.cpp has no DOMA_OV_K"; exit 1; }
done
E="DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=4096"
(
  flock 9
  for k in 4 3 2; do
      L=$D/k$k; mkdir -p "$L"
      env $E DOMA_OV_K=$k DOMA_N=4096 DOMA_HBM_PCT=20 DOMA_PHASES=get:500000,put:500000,mix:500000,get:500000 \
          ./host_phases doma_hw.xclbin 65536 > "$L/rate.log" 2>&1
      grep -q "in-flight cap: K = $k" "$L/rate.log" || say "K=$k: the host did not report K = $k ($L/rate.log)"
      cyc=$(grep -oE "^phase [0-9]+: .*device [0-9.]+ cycles per command" "$L/rate.log" | sed -E 's/.*device ([0-9.]+) cycles.*/\1/' | tr '\n' ' ')
      if [ "$(echo $cyc | wc -w)" = 4 ] && ! grep -qiE "wrong data|bad header|errors 0x[1-9a-f]|halt|refused" "$L/rate.log"; then
          dev=$(python3 -c "import sys; v=[float(x) for x in sys.argv[1:]]; print(f'{max(abs(x-v[0])/v[0] for x in v)*100:.3f}')" $cyc)
          awk -v d="$dev" 'BEGIN{exit !(d <= 0.2)}' && say "K=$k: RATE PASS: get / put / mix / get = $cyc cycles per command (largest difference $dev%)" \
              || say "K=$k: RATE FAIL: get / put / mix / get = $cyc (difference $dev%, over 0.2%)"
      else say "K=$k: RATE run failed, halted or a GET was wrong ($L/rate.log)"; fi
      for p in 0 20 100; do
          env $E DOMA_OV_K=$k DOMA_N=1000000 DOMA_HBM_PCT=$p ./host_doma doma_hw.xclbin 2000000 > "$L/kqps_$p.log" 2>&1
          say "K=$k: $p% on package, 1,000,000 records, 2,000,000 requests: $(grep -oE '[0-9.]+ KQPS' "$L/kqps_$p.log" | tail -1), $(grep -E 'ALL RESPONSES CORRECT|ERRORS FOUND' "$L/kqps_$p.log" | tail -1)$(grep -qiE 'halt|refused' "$L/kqps_$p.log" && echo ', HALT OR REFUSED (see the log)')"
      done
  done
) 9> "$LOCK"
say "$N: done (logs in $D/k2 .. k4)"
