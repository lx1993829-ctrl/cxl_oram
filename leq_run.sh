#!/bin/bash
# leq_run.sh -- the phase 3 overlap design with a RUN-TIME same-bucket gap (and run-time K), one bitstream, then the card
# runs that show (a) the default is unchanged and (b) whether a shorter gap is safe on this card.
#   The gap is the wait, after an earlier request in flight got the same bucket, before a later read of that bucket.
#   Default 6,409 cycles = write-back time 2,085 + 1,024 write beats + 3,300 margin for a host write that is readable
#   only some time after its acknowledgement. DOMA_OV_LEQ=<cycles> (3,816 .. 8,184, multiple of 8) sets it per run.
#   It is a public constant for the run (latched at the boot), so the schedule stays a fixed function of public events;
#   too short a gap can only end in the fail-stop integrity halt, never in wrong data.
#
#   source /opt/xilinx/xrt/setup.sh; source /tools/Xilinx/Vitis/2024.1/settings64.sh
#   ZIP=~/fpga_zcu208_v14c_respfix_leq_full.zip nohup bash leq_run.sh > ~/leq_run.log 2>&1 &
#   tail -f ~/leq_results.txt
#
# Build (no card use): ~/p3rx300leq, the p3rx300krt build arguments, placed with the over-constraint that closed
# p3rx300krt (0.15 ns extra setup uncertainty on the kernel clock until routing, removed and checked before routing).
# If it misses 300 MHz, ~/p3rx300leq_eto is built with placement ExtraTimingOpt (the other setting that closed).
# Rerunning skips the build when a bitstream that meets timing is there; CHECK_ONLY=1 never builds.
#
# Card runs (one program at a time, ~/.u55c_card.lock), K = 4:
#   1 default gap: get / put / mix / get rate check (n = 4,096, 20% in HBM; within 0.2%), and 1M records at 20% in HBM
#     (2M requests): should equal p3rx300krt (187.2 KQPS)
#   2 per gap in LEQS (default "4000"): 1M records at 20% in HBM (2M requests), then the STRESS runs: 4,096 records
#     (586 buckets, so about 1 request in 200 reads a bucket in flight) at 0% in HBM (every bucket in host memory) and
#     at 20%, STRESS_CMDS mixed commands each (default 20,000,000: about 100,000 same-bucket reads per run). A clean run
#     (every GET correct, no halt) means a host write was readable within the shorter gap every time.
set -u
ZIP=${ZIP:-$HOME/fpga_zcu208_v14c_respfix_leq_full.zip}
RES=$HOME/leq_results.txt; LOCK=$HOME/.u55c_card.lock
ARGS="OVERLAP=4 P3=1 OV_PMIN=1600 OV_GA=1600 OV_XWB=300 BEATS=128 SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw"
LEQS=${LEQS:-4000}; STRESS_CMDS=${STRESS_CMDS:-20000000}
say() { echo "$(date '+%m-%d %H:%M') $*" | tee -a "$RES"; }
[ -n "${XILINX_XRT:-}" ] || { echo "source /opt/xilinx/xrt/setup.sh first"; exit 1; }

ovc_setup() {   # $1 = vitis dir: +0.15 ns on the kernel clock for placement and phys_opt, removed and checked before routing
    local V=$1 c=$1/doma_big2m_rama.cfg
    [ "$(grep -E '^\[' "$c" | tail -1)" = "[vivado]" ] || return 1
    cat > "$V/ovc_on.tcl" <<EOF
proc ovc_unc {c} {
    set s [report_timing -quiet -setup -max_paths 1 -from \$c -to \$c -return_string]
    if {[regexp {Clock Uncertainty:\s+([-0-9.]+)\s*ns} \$s -> u]} { return \$u }
    return "na"
}
if {[catch {
    set c [get_clocks -quiet clk_kernel_00*]
    if {[llength \$c] == 0} { puts "OVC: no clk_kernel_00 clock: not over-constrained" } else {
        set u0 [ovc_unc \$c]
        set f [open $V/ovc_u0.txt w]; puts \$f \$u0; close \$f
        set_clock_uncertainty -setup 0.15 \$c
        puts "OVC: on, [llength \$c] kernel clock(s) [get_property NAME \$c] +0.15 ns setup uncertainty (kernel path uncertainty before: \$u0)"
    }
} e]} { puts "OVC: error in ovc_on.tcl (not over-constrained): \$e" }
EOF
    cat > "$V/ovc_off.tcl" <<EOF
proc ovc_unc {c} {
    set s [report_timing -quiet -setup -max_paths 1 -from \$c -to \$c -return_string]
    if {[regexp {Clock Uncertainty:\s+([-0-9.]+)\s*ns} \$s -> u]} { return \$u }
    return "na"
}
if {[catch {
    set c [get_clocks -quiet clk_kernel_00*]
    if {[llength \$c]} {
        set_clock_uncertainty -setup 0 \$c
        set u1 [ovc_unc \$c]
        set u0 "na"; if {[file exists $V/ovc_u0.txt]} { set f [open $V/ovc_u0.txt r]; set u0 [string trim [read \$f]]; close \$f }
        if {\$u0 eq ""} { set u0 "na" }
        puts "OVC: off before routing (kernel path uncertainty now \$u1, before over-constraining \$u0)"
        if {\$u0 eq "na" || \$u1 eq "na"} { puts "OVC UNCHECKED: the clock uncertainty was not readable (now \$u1, originally \$u0)"
        } elseif {abs(\$u1 - \$u0) > 0.002} { puts "OVC MISMATCH: do not use (uncertainty now \$u1, originally \$u0)" }
    }
} e]} { puts "OVC MISMATCH: do not use (error in ovc_off.tcl: \$e)" }
EOF
    printf 'prop=run.impl_1.STEPS.PLACE_DESIGN.TCL.PRE=%s\nprop=run.impl_1.STEPS.ROUTE_DESIGN.TCL.PRE=%s\n' "$V/ovc_on.tcl" "$V/ovc_off.tcl" >> "$c"
}

build_one() {   # $1 folder name, $2 placement directive; returns 0 when timing closed at 300 MHz
    local n=$1 dv=$2 D=$HOME/$1 V=$HOME/$1/fpga_zcu208/vitis
    rm -rf "$D"; mkdir -p "$D" && (cd "$D" && unzip -q "$ZIP") || { say "$n: unzip failed"; return 1; }
    local bad=$(cd "$D/fpga_zcu208" && sha256sum -c MANIFEST.sha256 2>/dev/null | grep -vc ': OK$')
    [ "$bad" = 0 ] || { say "$n: $bad package files fail their checksum: not built"; return 1; }
    grep -q "cfg_ov_leq" "$D/fpga_zcu208/rtl/flat_oram_gcm.v" || { say "$n: the package has no run-time same-bucket gap"; return 1; }
    local c=$V/doma_big2m_rama.cfg
    if [ "$dv" != ExtraNetDelay_high ]; then
        grep -q 'PLACE_DESIGN.ARGS.DIRECTIVE=ExtraNetDelay_high' "$c" || { say "$n: placement directive line not found"; return 1; }
        sed -i "s/PLACE_DESIGN.ARGS.DIRECTIVE=ExtraNetDelay_high/PLACE_DESIGN.ARGS.DIRECTIVE=$dv/" "$c"
    fi
    ovc_setup "$V" || { say "$n: could not add the over-constraint"; return 1; }
    say "$n: build started (placement $dv, over-constrained by 0.15 ns until routing; $ARGS)"
    ( cd "$V" && env $ARGS bash build.sh ) > "$D/build.log" 2>&1
    [ -f "$V/doma_hw.xclbin" ] || { say "$n: BUILD FAILED (no doma_hw.xclbin; see $D/build.log)"; return 1; }
    local lg=$(ls "$V"/_x/link/vivado/vpl/prj/prj.runs/impl_1/runme.log "$V"/_x/link/vivado/vpl/vivado.log 2>/dev/null)
    local ov=$(grep -h "^OVC" $lg 2>/dev/null | sort -u); echo "$ov" > "$D/ovc.txt"
    if echo "$ov" | grep -q "OVC MISMATCH" || ! echo "$ov" | grep -q "^OVC: off"; then
        say "$n: OVER-CONSTRAINT NOT CLEANLY REMOVED: not used ($D/ovc.txt)"; mv "$V/doma_hw.xclbin" "$V/doma_hw.xclbin.unchecked"; return 1; fi
    echo "$ov" | grep -q "OVC UNCHECKED" && say "$n: note: the over-constraint removal could not be compared ($D/ovc.txt)"
    if (cd "$V" && WANT_MHZ=300 bash check_timing.sh > "$D/timing.txt" 2>&1); then
        say "$n: TIMING CLOSED at 300 MHz ($(grep -oE 'WNS=[-0-9.]+ ns' "$D/timing.txt" | head -1))"; return 0; fi
    say "$n: timing not closed ($(grep -E 'WNS=|kernel clock in' "$D/timing.txt" | head -2 | tr '\n' ' '))"; return 2
}
closed() { [ -f "$HOME/$1/fpga_zcu208/vitis/doma_hw.xclbin" ] && (cd "$HOME/$1/fpga_zcu208/vitis" && WANT_MHZ=300 bash check_timing.sh > /dev/null 2>&1); }

# ---- build
N=""
for cand in p3rx300leq p3rx300leq_eto; do closed $cand && { N=$cand; say "$N: built and timing closed, not rebuilt"; break; }; done
if [ -z "$N" ] && [ "${CHECK_ONLY:-0}" != 1 ]; then
    command -v v++ > /dev/null || { echo "source /tools/Xilinx/Vitis/2024.1/settings64.sh first"; exit 1; }
    [ -f "$ZIP" ] || { echo "no package $ZIP"; exit 1; }
    if build_one p3rx300leq ExtraNetDelay_high; then N=p3rx300leq
    elif build_one p3rx300leq_eto ExtraTimingOpt; then N=p3rx300leq_eto; fi
fi
[ -n "$N" ] || { say "no p3rx300leq bitstream that meets 300 MHz: not measured"; exit 1; }
D=$HOME/$N; V=$D/fpga_zcu208/vitis
bash "$V/check_overlap_build.sh" 4 > "$D/overlap_build.txt" 2>&1 && say "$N: overlap design present in the placed netlist" \
    || say "$N: check_overlap_build.sh FAILED ($D/overlap_build.txt)"
for h in host_phases host_doma; do
    (cd "$V" && g++ -std=c++17 -O2 $h.cpp -I. -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread -o $h) \
        || { say "$N: $h does not compile"; exit 1; }
    grep -q DOMA_OV_LEQ "$V/$h.cpp" || { say "$N: $h.cpp has no DOMA_OV_LEQ"; exit 1; }
done

# ---- card
E="DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=4096 DOMA_OV_K=4"
L=$D/leq; mkdir -p "$L"
bad() { grep -qiE "wrong data|bad header|errors 0x[1-9a-f]|halt|refused|ERRORS FOUND" "$1"; }
kq() {   # $1 log, $2 label
    say "$N: $2: $(grep -oE '[0-9.]+ KQPS' "$1" | tail -1), $(grep -aE 'ALL RESPONSES CORRECT|ERRORS FOUND' "$1" | tail -1)$(bad "$1" && echo ', HALT, ERROR OR REFUSED (see the log)')"; }
(
  flock 9
  cd "$V" || exit 1
  # 1 default gap
  env $E DOMA_N=4096 DOMA_HBM_PCT=20 DOMA_PHASES=get:500000,put:500000,mix:500000,get:500000 ./host_phases doma_hw.xclbin 65536 > "$L/rate_def.log" 2>&1
  cyc=$(grep -oE "^phase [0-9]+: .*device [0-9.]+ cycles per command" "$L/rate_def.log" | sed -E 's/.*device ([0-9.]+) cycles.*/\1/' | tr '\n' ' ')
  if [ "$(echo $cyc | wc -w)" = 4 ] && ! bad "$L/rate_def.log"; then
      dev=$(python3 -c "import sys; v=[float(x) for x in sys.argv[1:]]; print(f'{max(abs(x-v[0])/v[0] for x in v)*100:.3f}')" $cyc)
      awk -v d="$dev" 'BEGIN{exit !(d <= 0.2)}' && say "$N: default gap: RATE PASS: get / put / mix / get = $cyc cycles per command ($dev%)" \
          || say "$N: default gap: RATE FAIL: $cyc ($dev%)"
  else say "$N: default gap: rate run failed, halted or a GET was wrong ($L/rate_def.log)"; fi
  env $E DOMA_N=1000000 DOMA_HBM_PCT=20 ./host_doma doma_hw.xclbin 2000000 > "$L/kqps_def.log" 2>&1
  kq "$L/kqps_def.log" "default gap (6,409), 1M records, 20% in HBM, 2M requests (p3rx300krt: 187.2 KQPS)"
  # 2 shorter gaps
  for g in $LEQS; do
      env $E DOMA_OV_LEQ=$g DOMA_N=1000000 DOMA_HBM_PCT=20 ./host_doma doma_hw.xclbin 2000000 > "$L/kqps_$g.log" 2>&1
      grep -q "same-bucket gap: " "$L/kqps_$g.log" || say "$N: gap $g: the host did not report the gap ($L/kqps_$g.log)"
      kq "$L/kqps_$g.log" "gap $g, 1M records, 20% in HBM, 2M requests"
      for p in 0 20; do
          f=$L/stress_${g}_$p.log
          env $E DOMA_OV_LEQ=$g DOMA_N=4096 DOMA_HBM_PCT=$p DOMA_PHASES=mix:$STRESS_CMDS ./host_phases doma_hw.xclbin 65536 > "$f" 2>&1
          say "$N: STRESS gap $g, 4,096 records, $p% in HBM, $STRESS_CMDS commands: $(grep -aoE 'device [0-9.]+ cycles per command' "$f" | head -1), $(grep -aE 'ALL RESPONSES CORRECT|ERRORS FOUND' "$f" | tail -1)$(bad "$f" && echo ', HALT OR ERROR (see the log): the gap is too short on this card')"
      done
  done
) 9> "$LOCK"
say "$N: done (logs in $L)"
