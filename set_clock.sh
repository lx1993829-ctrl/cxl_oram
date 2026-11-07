#!/bin/bash
# set_clock.sh <in.xclbin> <MHz> [out.xclbin] -- a copy of a bitstream with its kernel clock lowered (the method of
# docs/README_FPGA.md, "Clocks"): xclbinutil dumps CLOCK_FREQ_TOPOLOGY, the kernel clock (DATA_CLK) is set to <MHz>, and
# the section is replaced in a copy; XRT programs that clock when it loads the copy. No rebuild, no card setting.
# Only lowering is allowed: a design is guaranteed only up to the clock it closed timing at.
# Default output: <in without .xclbin>_<MHz>mhz.xclbin. Check a run's real clock: device cycles / microseconds.
set -eu
in=$1; mhz=$2; out=${3:-${in%.xclbin}_${mhz}mhz.xclbin}
command -v xclbinutil > /dev/null || { echo "xclbinutil not found: source /opt/xilinx/xrt/setup.sh"; exit 1; }
clk() { xclbinutil --info -i "$1" 2>/dev/null | awk '/Name:.*DATA_CLK/{f=1} f && /Frequency:/{print $2; exit}'; }
built=$(clk "$in"); [ -n "$built" ] || { echo "no DATA_CLK in $in"; exit 1; }
awk -v a="$mhz" -v b="$built" 'BEGIN{exit !(a > 0 && a <= b)}' || { echo "refused: $mhz MHz is above the $built MHz this bitstream closed at"; exit 1; }
t=$(mktemp -d); trap 'rm -rf "$t"' EXIT
xclbinutil -i "$in" --dump-section CLOCK_FREQ_TOPOLOGY:JSON:"$t/clk.json" > /dev/null
python3 - "$t/clk.json" "$mhz" <<'EOF'
import json, sys
p, mhz = sys.argv[1], sys.argv[2]
j = json.load(open(p)); n = 0
for c in j["clock_freq_topology"]["m_clock_freq"]:
    if c.get("m_name", "").endswith("DATA_CLK") or c.get("m_type", "") == "DATA":
        c["m_freq_Mhz"] = str(int(float(mhz))); n += 1
if n != 1: sys.exit(f"expected one kernel clock (DATA_CLK), found {n}")
json.dump(j, open(p, "w"), indent=4)
EOF
xclbinutil -i "$in" --replace-section CLOCK_FREQ_TOPOLOGY:JSON:"$t/clk.json" -o "$out" --force > /dev/null
now=$(clk "$out"); [ "$now" = "$mhz" ] || { echo "the copy reports $now MHz, not $mhz"; exit 1; }
echo "$out: kernel clock $now MHz (built and closed at $built MHz)"
