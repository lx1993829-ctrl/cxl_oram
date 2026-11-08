#!/bin/bash
# margins_run.sh -- card stage times for overlap options 1 (start spacing P) and 2 (read gate), from the existing 4 KB
# serial TRACE=1 bitstream (~/optarama_tr). No new bitstream, no card setting: only a host program that also writes the
# 16 trace timestamps the bitstream already sends for every command (host_phases.cpp, DOMA_TRACE_CSV).
#   put host_phases.cpp and margins.py next to this script (or in ~), then:  bash margins_run.sh
#   CMDS=1000000 N=1000000 PCTS="0 20 100" to change. Results: ~/margins/summary_<pct>.txt
set -u
source /opt/xilinx/xrt/setup.sh > /dev/null
H=$(dirname "$(readlink -f "$0")")
src=$(ls "$H/host_phases.cpp" "$HOME/host_phases.cpp" 2>/dev/null | head -1)
py=$(ls "$H/margins.py" "$HOME/margins.py" 2>/dev/null | head -1)
[ -n "$src" ] && [ -n "$py" ] || { echo "need host_phases.cpp and margins.py next to this script or in ~"; exit 1; }
grep -q DOMA_TRACE_CSV "$src" || { echo "$src is not the version with DOMA_TRACE_CSV"; exit 1; }
V=$HOME/optarama_tr/fpga_zcu208/vitis
[ -f "$V/doma_hw.xclbin" ] || { echo "no bitstream in $V"; exit 1; }
grep -q "phase trace: on" "$HOME/optarama_tr/build.log" 2>/dev/null || { echo "~/optarama_tr/build.log does not say phase trace: on"; exit 1; }
CMDS=${CMDS:-1000000}; N=${N:-1000000}; PCTS=${PCTS:-"0 20 100"}
R=$HOME/margins; mkdir -p "$R"
cp "$src" "$R/host_phases.cpp"
g++ -std=c++17 -O2 "$R/host_phases.cpp" -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread -o "$R/host_phases_tr" || exit 1
cd "$V" || exit 1        # host_phases checks lambda and Z against ../rtl of the build it runs
for p in $PCTS; do
    echo "$(date +%T) $p% of buckets in HBM, $N records, $CMDS commands (50/50 GET/PUT)"
    flock "$HOME/.u55c_card.lock" env DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=4096 DOMA_N=$N DOMA_HBM_PCT=$p \
        DOMA_PHASE_TRACE=1 DOMA_TRACE_CSV="$R/trace_$p.csv" DOMA_PHASES=mix:$CMDS \
        timeout 3600 "$R/host_phases_tr" doma_hw.xclbin 65536 > "$R/run_$p.log" 2>&1
    grep -E "^phase 1: [0-9]+ mix|ALL RESPONSES CORRECT|ERRORS FOUND|halt" "$R/run_$p.log" | head -4 | sed 's/^/  /'
    python3 "$py" "$R/trace_$p.csv" > "$R/summary_$p.txt" 2>&1
    gzip -f "$R/trace_$p.csv"
    sed 's/^/  /' "$R/summary_$p.txt"
done
echo "done: $R/summary_*.txt (records: $R/trace_*.csv.gz)"
