#!/bin/bash
# bl_clk_run.sh -- the serial baseline (v14d1 + Option A + RAMA, the optarama_fix design) at 64 B and 4 KB, 200 and
# 300 MHz, measured at N = 500,000 records.
#   builds (in parallel, no card use), from the run-time K package (it has MHZ=; a baseline build from it differs from
#   optarama_fix only in refusing a nonzero K field at boot):
#     ~/bl64_200   BEATS=2   MHZ=200          ~/bl64_300   BEATS=2   MHZ=300          ~/bl4k_200   BEATS=128 MHZ=200
#     4 KB at 300 MHz is ~/optarama_fix, the measured baseline (not rebuilt; REBUILD_4K300=1 builds ~/bl4k_300 too)
#     every build: FIXWB=1 RAMA=1 SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw
#   a 300 MHz build that misses timing is rebuilt once in <name>_ovc with the over-constraint that closed p3rx300krt
#   (0.15 ns extra setup uncertainty on the kernel clock until routing, removed and checked before routing)
#   measurements (one card program at a time, ~/.u55c_card.lock), per design, N = 500,000, lambda 7:
#     host_doma, 2,000,000 requests (50/50 GET/PUT), 0 / 20 / 100% of buckets in HBM: KQPS, every response correct
#     host_phases, 20% in HBM, mix of 1,000,000: device cycles per command and the latency distribution
#   source /opt/xilinx/xrt/setup.sh; source /tools/Xilinx/Vitis/2024.1/settings64.sh
#   ZIP=~/fpga_zcu208_v14c_respfix_krt_full.zip nohup bash bl_clk_run.sh > ~/bl_clk_run.log 2>&1 &
#   tail -f ~/blclk_results.txt
# Rerunning skips designs already built; CHECK_ONLY=1 never builds (measures what is there).
set -u
ZIP=${ZIP:-$HOME/fpga_zcu208_v14c_respfix_krt_full.zip}
RES=$HOME/blclk_results.txt; LOCK=$HOME/.u55c_card.lock
COMMON="FIXWB=1 RAMA=1 SCHED=arrival CFG=doma_big2m_rama.cfg NMAX=2097152 LAMBDA=7 TARGET=hw"
VARIANTS="bl64_200:2:200 bl64_300:2:300 bl4k_200:128:200"
[ "${REBUILD_4K300:-0}" = 1 ] && VARIANTS="$VARIANTS bl4k_300:128:300"
say() { echo "$(date '+%m-%d %H:%M') $*" | tee -a "$RES"; }
[ -n "${XILINX_XRT:-}" ] || { echo "source /opt/xilinx/xrt/setup.sh first"; exit 1; }

ovc_setup() {   # $1 = vitis dir: over-constrain the kernel clock by 0.15 ns for placement and phys_opt, off before routing
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

build_one() {   # $1 name, $2 beats, $3 MHz, $4 = 1 for the over-constrained rebuild
    local n=$1 b=$2 m=$3 o=${4:-0} D=$HOME/$1 V=$HOME/$1/fpga_zcu208/vitis
    rm -rf "$D"; mkdir -p "$D" && (cd "$D" && unzip -q "$ZIP") || { say "$n: unzip failed"; return 1; }
    local bad=$(cd "$D/fpga_zcu208" && sha256sum -c MANIFEST.sha256 2>/dev/null | grep -vc ': OK$')
    [ "$bad" = 0 ] || { say "$n: $bad package files fail their checksum: not built"; return 1; }
    grep -q 'KMHZ=${MHZ:-300}' "$V/build.sh" || { say "$n: this package has no MHZ= option: not built"; return 1; }
    if [ "$o" = 1 ]; then ovc_setup "$V" || { say "$n: could not add the over-constraint to the cfg"; return 1; }; fi
    say "$n: build started ($COMMON BEATS=$b MHZ=$m$([ "$o" = 1 ] && echo ', over-constrained by 0.15 ns until routing'))"
    ( cd "$V" && env $COMMON BEATS=$b MHZ=$m bash build.sh ) > "$D/build.log" 2>&1
    [ -f "$V/doma_hw.xclbin" ] || { say "$n: BUILD FAILED (no doma_hw.xclbin; see $D/build.log)"; return 1; }
    if [ "$o" = 1 ]; then
        local lg=$(ls "$V"/_x/link/vivado/vpl/prj/prj.runs/impl_1/runme.log "$V"/_x/link/vivado/vpl/vivado.log 2>/dev/null)
        local ov=$(grep -h "^OVC" $lg 2>/dev/null | sort -u); echo "$ov" > "$D/ovc.txt"
        if echo "$ov" | grep -q "OVC MISMATCH" || { echo "$ov" | grep -q "^OVC: on" && ! echo "$ov" | grep -q "^OVC: off"; }; then
            say "$n: OVER-CONSTRAINT NOT CLEANLY REMOVED: do not use ($D/ovc.txt)"; rm -f "$V/doma_hw.xclbin"; return 1; fi
        echo "$ov" | grep -q "OVC UNCHECKED" && say "$n: note: the over-constraint removal could not be compared ($D/ovc.txt)"
    fi
    if (cd "$V" && WANT_MHZ=$m bash check_timing.sh > "$D/timing.txt" 2>&1); then
        say "$n: TIMING CLOSED at $m MHz ($(grep -oE 'WNS=[-0-9.]+ ns' "$D/timing.txt" | head -1))"; return 0
    fi
    say "$n: timing not closed at $m MHz ($(grep -E 'WNS=|kernel clock in' "$D/timing.txt" | head -2 | tr '\n' ' '))"; return 2
}

build_variant() {   # build, and for 300 MHz retry once over-constrained
    local n=$1 b=$2 m=$3
    build_one "$n" "$b" "$m" 0; local r=$?
    if [ $r = 2 ] && [ "$m" = 300 ]; then say "$n: rebuilding over-constrained in ${n}_ovc"; build_one "${n}_ovc" "$b" "$m" 1; fi
}

# ---- builds
if [ "${CHECK_ONLY:-0}" != 1 ]; then
    command -v v++ > /dev/null || { echo "source /tools/Xilinx/Vitis/2024.1/settings64.sh first"; exit 1; }
    [ -f "$ZIP" ] || { echo "no package $ZIP"; exit 1; }
    for v in $VARIANTS; do
        IFS=: read n b m <<< "$v"
        if [ -f "$HOME/$n/fpga_zcu208/vitis/doma_hw.xclbin" ] || [ -f "$HOME/${n}_ovc/fpga_zcu208/vitis/doma_hw.xclbin" ]; then
            say "$n: already built, not rebuilt"; continue; fi
        build_variant "$n" "$b" "$m" &
    done
    wait
fi

# ---- measurements
pick() {   # the folder to measure for a variant: the over-constrained one if it exists and closed, else the plain one if it closed
    local n=$1 m=$2
    for d in "$HOME/${n}_ovc" "$HOME/$n"; do
        [ -f "$d/fpga_zcu208/vitis/doma_hw.xclbin" ] || continue
        if (cd "$d/fpga_zcu208/vitis" && WANT_MHZ=$m bash check_timing.sh > /dev/null 2>&1); then echo "$d"; return 0; fi
    done
    return 1
}
LIST=""
for v in $VARIANTS; do IFS=: read n b m <<< "$v"; d=$(pick "$n" "$m") && LIST="$LIST $d:$b:$m" || say "$n: no bitstream that meets timing at $m MHz: not measured"; done
[ "${REBUILD_4K300:-0}" = 1 ] || LIST="$LIST $HOME/optarama_fix:128:300"
for e in $LIST; do
    IFS=: read d b m <<< "$e"; V=$d/fpga_zcu208/vitis; n=$(basename "$d")
    [ -f "$V/doma_hw.xclbin" ] || { say "$n: no doma_hw.xclbin in $V: not measured"; continue; }
    for h in host_doma host_phases; do
        [ -x "$V/$h" ] && [ "$V/$h" -nt "$V/$h.cpp" ] && continue
        (cd "$V" && g++ -std=c++17 -O2 $h.cpp -I. -I"$XILINX_XRT/include" -L"$XILINX_XRT/lib" -lxrt_coreutil -pthread -o $h) \
            || { say "$n: $h does not compile"; continue 2; }
    done
    B=$((b * 32)); E="DOMA_LAMBDA=7 DOMA_NMAX=2097152 DOMA_BLOCK=$B DOMA_N=500000"; L=$d/blclk; mkdir -p "$L"
    (
      flock 9
      for p in 0 20 100; do
          f=$L/kqps_$p.log
          (cd "$V" && env $E DOMA_HBM_PCT=$p ./host_doma doma_hw.xclbin 2000000) > "$f" 2>&1
          say "$n ($B B, $m MHz): $p% in HBM, 500,000 records, 2,000,000 requests: $(grep -oE '[0-9.]+ KQPS' "$f" | tail -1), $(grep -aE 'ALL RESPONSES CORRECT|ERRORS FOUND' "$f" | tail -1)$(grep -qiE 'halt|refused' "$f" && echo ', HALT OR REFUSED (see the log)')"
      done
      f=$L/lat_20.log
      (cd "$V" && env $E DOMA_HBM_PCT=20 DOMA_PHASES=mix:1000000 ./host_phases doma_hw.xclbin 65536) > "$f" 2>&1
      say "$n ($B B, $m MHz): 20% in HBM, mix of 1,000,000: $(grep -oE 'device [0-9.]+ cycles per command' "$f" | head -1) | $(grep -oE 'latency min .*' "$f" | head -1) | $(grep -aE 'ALL RESPONSES CORRECT|ERRORS FOUND' "$f" | tail -1)"
    ) 9> "$LOCK"
done
say "bl_clk_run: done (logs in each folder's blclk/)"
