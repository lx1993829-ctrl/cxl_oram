#!/bin/bash
# ncov_run.sh -- builds the NOCRYPTO phase 3 overlap design at 256 B (docs/nocrypto_overlap.md) in ~/ncov_b8, then
# measures it on the card (one card program at a time, ~/.u55c_card.lock): N = 1,000,000, 20% of buckets in HBM,
# 256 B values, 1,000,000 requests, at three settings of the run-time times (loose -> tight). Every run checks every GET.
#   source /opt/xilinx/xrt/setup.sh; source /tools/Xilinx/Vitis/2024.1/settings64.sh
#   ZIP=~/fpga_zcu208_v14c_krt_nocrypto_full.zip nohup bash ~/ncov_run.sh > ~/ncov_run.log 2>&1 &
#   tail -f ~/ncov_results.txt
# Rerunning skips the build when ~/ncov_b8 has a bitstream (CHECK_ONLY=1 never builds). BUILD_ONLY=1 stops after the
# timing check. More settings later, no rebuild: SETS="name:ctrd,pmin,ga,wb,rel,l ..." bash ~/ncov_run.sh
# Settings (cycles): ctrd read gate after the start, pmin least start spacing, ga least arrival spacing, wb write-back
# and rel release after the effective arrival t', l context free after t' (the access four later may start).
# A too small time can only stop the run (DOMA HALT, error 0x108), never return a wrong value or change the schedule.
set -u
N=ncov_b8; D=$HOME/$N; V=$D/fpga_zcu208/vitis; RES=$HOME/ncov_results.txt; LOCK=$HOME/.u55c_card.lock
ZIP=${ZIP:-$HOME/fpga_zcu208_v14c_krt_nocrypto_full.zip}
# the build's times (the loosest set below); every one is replaced at run time by the host's DOMA_* settings
ARGS="NOCRYPTO=1 OVERLAP=4 P3=1 BEATS=8 SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw OV_CTRD=400 OV_PMIN=400 OV_GA=400 NC_WB=400 NC_REL=600 OV_L=900"
SETS=${SETS:-"build:400,400,400,400,600,900 mid:400,350,350,350,550,800 tight:300,350,350,330,530,780"}
VALUE=${VALUE:-256}; CMDS=${CMDS:-1000000}; NREC=${NREC:-1000000}; HBM=${HBM:-20}
say() { echo "$(date '+%m-%d %H:%M') $*" | tee -a "$RES"; }
[ -n "${XILINX_XRT:-}" ] || { echo "source /opt/xilinx/xrt/setup.sh first"; exit 1; }
if [ ! -f "$V/doma_hw.xclbin" ] && [ "${CHECK_ONLY:-0}" != 1 ]; then
    command -v v++ > /dev/null || { echo "source /tools/Xilinx/Vitis/2024.1/settings64.sh first"; exit 1; }
    [ -f "$ZIP" ] || { echo "no package $ZIP"; exit 1; }
    rm -rf "${D:?}"; mkdir -p "$D"; (cd "$D" && unzip -q "$ZIP") || { say "$N: unzip failed"; exit 1; }
    bad=$(cd "$D/fpga_zcu208" && sha256sum -c MANIFEST.sha256 2>/dev/null | grep -vc ': OK$')
    [ "$bad" = 0 ] || { say "$N: $bad package files fail their checksum: not built"; exit 1; }
    grep -q "cfg_nc_pmin" "$D/fpga_zcu208/rtl/flat_oram_gcm.v" || { say "$N: the package has no NOCRYPTO overlap"; exit 1; }
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
bash check_overlap_build.sh 4 > "$D/overlap_build.txt" 2>&1 && say "$N: overlap design present, no AES-GCM core in the placed netlist" \
    || say "$N: check_overlap_build.sh FAILED ($D/overlap_build.txt)"
[ "${BUILD_ONLY:-0}" = 1 ] && exit 0
g++ -std=c++17 -O2 -DDOMA_VALUE_BYTES=$VALUE host_doma.cpp -I. -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread \
    -o host_doma_v$VALUE || { say "$N: host_doma does not compile"; exit 1; }
grep -q DOMA_PMIN host_doma.cpp || { say "$N: host_doma.cpp has no phase 3 run-time times"; exit 1; }
E="DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=$((VALUE < 32 ? 32 : VALUE)) DOMA_N=$NREC DOMA_HBM_PCT=$HBM"
(
  flock 9
  for s in $SETS; do
      name=${s%%:*}; IFS=, read -r c p g w r l <<< "${s#*:}"
      L=$D/run_$name.log
      env $E DOMA_CTRD=$c DOMA_PMIN=$p DOMA_GA=$g DOMA_WB=$w DOMA_REL=$r DOMA_L=$l ./host_doma_v$VALUE doma_hw.xclbin $CMDS > "$L" 2>&1
      res=$(grep -E 'ALL RESPONSES CORRECT|ERRORS FOUND' "$L" | tail -1)
      say "$name (ctrd $c, pmin $p, ga $g, wb $w, rel $r, l $l): $(grep -oE '[0-9.]+ KQPS' "$L" | tail -1), ${res:-no result line}$(grep -qiE 'halt|refused|0x108' "$L" && echo ', HALT OR REFUSED (see the log)')"
  done
) 9> "$LOCK"
say "$N: done (logs in $D/run_*.log)"
