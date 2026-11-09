#!/bin/bash
# krt_rebuild.sh -- the run-time K design missed 300 MHz by 0.095 ns (p3rx300krt, 2026-11-09). The failing paths are
# spread over existing logic (scratch writes, stash, GHASH, platform interconnect), none on the new K logic: placement
# luck. Vivado is deterministic, so the same settings give the same miss. This rebuilds the same package with
# OVER-CONSTRAINING (UG949): during placement and physical optimization the kernel clock carries extra setup
# uncertainty (0.15 ns by default, so the placer works as if at about 314 MHz), and the extra is removed again before
# routing, so the final timing check, v++'s clock choice and check_timing.sh all judge the real 300 MHz requirement.
# Two variants build in parallel (VARIANTS="<place directive>:<extra ns> ..."):
#   ~/p3rx300krt_extranetdelay_high   the original placement directive (it came within 0.095 ns) + 0.15 ns
#   ~/p3rx300krt_extratimingopt       place_design -directive ExtraTimingOpt + 0.15 ns
# Everything else is p3rx300krt's build. No card use.
#   source /opt/xilinx/xrt/setup.sh; source /tools/Xilinx/Vitis/2024.1/settings64.sh
#   ZIP=~/fpga_zcu208_v14c_respfix_krt_full.zip nohup bash krt_rebuild.sh > ~/krt_rebuild.log 2>&1 &
#   tail -f ~/krt_results.txt
# Safety of the over-constraint: it only ever adds uncertainty before routing; ovc_off.tcl sets the user uncertainty
# back to 0 and compares the kernel clock's own uncertainty with the value before ovc_on.tcl. A mismatch (the platform
# had its own user uncertainty) is reported as "OVC MISMATCH: do not use", and the bitstream is not used.
set -u
ZIP=${ZIP:-$HOME/fpga_zcu208_v14c_respfix_krt_full.zip}
ARGS="OVERLAP=4 P3=1 OV_PMIN=1600 OV_GA=1600 OV_XWB=300 BEATS=128 SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw"
RES=$HOME/krt_results.txt
VARIANTS=${VARIANTS:-"ExtraNetDelay_high:0.15 ExtraTimingOpt:0.15"}
say() { echo "$(date '+%m-%d %H:%M') $*" | tee -a "$RES"; }
[ -n "${XILINX_XRT:-}" ] || { echo "source /opt/xilinx/xrt/setup.sh first"; exit 1; }
command -v v++ > /dev/null || { echo "source /tools/Xilinx/Vitis/2024.1/settings64.sh first"; exit 1; }
[ -f "$ZIP" ] || { echo "no package $ZIP"; exit 1; }
for vr in $VARIANTS; do
    dv=${vr%%:*}; ov=${vr##*:}
    awk -v v="$ov" 'BEGIN{exit !(v+0 > 0 && v+0 <= 0.3)}' || { say "variant $vr: the extra uncertainty must be in (0, 0.3] ns"; continue; }
    n=p3rx300krt_$(echo "$dv" | tr 'A-Z' 'a-z'); D=$HOME/$n; V=$D/fpga_zcu208/vitis
    [ -e "$D" ] && { say "$n: already exists, skipped (remove it to rebuild)"; continue; }
    mkdir -p "$D" && (cd "$D" && unzip -q "$ZIP") || { say "$n: unzip failed"; continue; }
    bad=$(cd "$D/fpga_zcu208" && sha256sum -c MANIFEST.sha256 2>/dev/null | grep -vc ': OK$')
    [ "$bad" = 0 ] || { say "$n: $bad package files fail their checksum: not built"; continue; }
    grep -q "cfg_ov_k" "$D/fpga_zcu208/rtl/flat_oram_gcm.v" || { say "$n: the package has no run-time K: not built"; continue; }
    c=$V/doma_big2m_rama.cfg
    grep -q 'PLACE_DESIGN.ARGS.DIRECTIVE=ExtraNetDelay_high' "$c" || { say "$n: placement directive line not found in $c"; continue; }
    sed -i "s/PLACE_DESIGN.ARGS.DIRECTIVE=ExtraNetDelay_high/PLACE_DESIGN.ARGS.DIRECTIVE=$dv/" "$c"
    # the over-constraint: on before placement, off before routing (Tcl hooks of the implementation run)
    cat > "$V/ovc_on.tcl" <<EOF
# ovc_on.tcl -- extra setup uncertainty on the kernel clock for placement and physical optimization (krt_rebuild.sh)
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
        set_clock_uncertainty -setup $ov \$c
        puts "OVC: on, [llength \$c] kernel clock(s) [get_property NAME \$c] +$ov ns setup uncertainty (kernel path uncertainty before: \$u0)"
    }
} e]} { puts "OVC: error in ovc_on.tcl (not over-constrained): \$e" }
EOF
    cat > "$V/ovc_off.tcl" <<EOF
# ovc_off.tcl -- removes the extra uncertainty before routing, and checks the kernel clock is back to its own value
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
    [ "$(grep -E '^\[' "$c" | tail -1)" = "[vivado]" ] || { say "$n: $c does not end with its [vivado] section"; continue; }
    printf 'prop=run.impl_1.STEPS.PLACE_DESIGN.TCL.PRE=%s\nprop=run.impl_1.STEPS.ROUTE_DESIGN.TCL.PRE=%s\n' "$V/ovc_on.tcl" "$V/ovc_off.tcl" >> "$c"
    say "$n: build started (placement $dv, over-constrained by $ov ns until routing; $ARGS)"
    (
        cd "$V" && env $ARGS bash build.sh > "$D/build.log" 2>&1
        if [ ! -f "$V/doma_hw.xclbin" ]; then say "$n: BUILD FAILED (no doma_hw.xclbin; see $D/build.log)"; exit 0; fi
        lg=$(ls "$V"/_x/link/vivado/vpl/prj/prj.runs/impl_1/runme.log "$V"/_x/link/vivado/vpl/vivado.log 2>/dev/null)
        o=$(grep -h "^OVC" $lg 2>/dev/null | sort -u)
        echo "$o" > "$D/ovc.txt"
        on=$(echo "$o" | grep -c "^OVC: on"); off=$(echo "$o" | grep -c "^OVC: off")
        if echo "$o" | grep -q "OVC MISMATCH" || { [ "$on" != 0 ] && [ "$off" = 0 ]; }; then
            say "$n: OVER-CONSTRAINT NOT CLEANLY REMOVED: do not use this bitstream ($D/ovc.txt)"; exit 0; fi
        [ "$on" = 0 ] && say "$n: note: the over-constraint did not apply ($D/ovc.txt); a plain build with placement $dv"
        echo "$o" | grep -q "OVC UNCHECKED" && say "$n: note: the removal could not be compared ($D/ovc.txt): send me that file before using it"
        if WANT_MHZ=300 bash check_timing.sh > "$D/timing.txt" 2>&1; then
            say "$n: TIMING CLOSED at 300 MHz ($(grep -oE 'WNS=[-0-9.]+ ns' "$D/timing.txt" | head -1); $(echo "$o" | grep '^OVC: off' | head -1))"
        else
            say "$n: timing not closed ($(grep -E 'WNS=|kernel clock in' "$D/timing.txt" | head -2 | tr '\n' ' '))"
        fi
    ) &
done
wait
say "krt_rebuild: done"
# Measuring the one that closed (here _extratimingopt; use the folder name the results line gives):
#   mv ~/p3rx300krt ~/p3rx300krt_first && mv ~/p3rx300krt_extratimingopt ~/p3rx300krt
#   CHECK_ONLY=1 nohup bash ~/krt_run.sh > ~/krt_run2.log 2>&1 &
