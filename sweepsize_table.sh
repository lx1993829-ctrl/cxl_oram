#!/bin/bash
# sweepsize_table.sh -- Tiny ORAM over value sizes (64 B to 4 KB, one record per block), N = 1M records (L = 18), each
# with the tree top of the DOMA-at-20% value table:
#
#   value   DOMA at 20%   Tiny ORAM top (B)   Tiny ORAM tree (L = 18)   top / tree
#   64 B    56.1 MiB      50,479,104            198.0 MiB               24.3%
#   128 B   85.0 MiB      80,744,448            482.5 MiB               16.0%
#   256 B   142.7 MiB     141,283,328           613.2 MiB               22.0%
#   512 B   258.2 MiB     262,352,896          1233.1 MiB               20.3%
#   1 KB    489.1 MiB     504,500,224          2393.6 MiB               20.1%
#   2 KB    951.0 MiB     988,786,688          6537.1 MiB               14.4%
#   4 KB    1,874.7 MiB   1,957,363,712       13760.0 MiB               13.6%
#
#   bash sweepsize_table.sh               runs the seven rows, then prints the summary table
#   bash sweepsize_table.sh summary       the summary of the last run again
#
# Measured as DOMA's host_doma.cpp measures doma_kernel (the same program, with Tiny ORAM's arguments): BOLT's
# generator (mt19937 seed 1, 50% PUT, uniform keys, 8-byte values written into the block), every record preloaded during
# init, the timed region = launch + wait() of CMDS commands (default 1,000,000, one launch), every response checked.
# Bitstreams: the builds of ~/tinypm (QUEUE=<dir> another queue): for each row, the DONE build of that block size with
# the flat position map in HBM (tag tinyoram_<size>_..._hbmpm) that runs L = 18: a run-time-depth one with L_max >= 18
# first (64 B: id 50), else a fixed-depth one at L = 18 (128 B to 4 KB: ids 41-45, 38). Each is checked to have the
# posmap argument before it runs, and the host checks the block size and depth it runs; a row with no such build is
# skipped and named at the end. Run at 200 MHz: a copy of the bitstream with its kernel clock set to 200 MHz (CLK=0
# keeps the build's clock). The tree below the top is in host memory: 11.6 GiB at 4 KB.
# Log: ~/tinypm/results/tinysize_table.log; summary: ~/tinypm/results/tinysize_table.txt
set -u
RES=${RES:-$HOME/tinypm/results}; mkdir -p "$RES"
LOG=$RES/tinysize_table.log; SUM=$RES/tinysize_table.txt
CMDS=${CMDS:-1000000}; CLK=${CLK:-200}
ROWS=${ROWS:-"64:1000000:50479104 128:1000000:80744448 256:1000000:141283328 512:1000000:262352896 1k:1000000:504500224 2k:1000000:988786688 4k:1000000:1957363712"}
XRT_SETUP=${XRT_SETUP:-/opt/xilinx/xrt/setup.sh}
HERE=$(cd "$(dirname "$0")" && pwd); VIT=$(cd "$HERE/../../vitis" && pwd)

summary() {
    python3 - "$LOG" <<'PY' | tee "$SUM"
import re, sys
rows = []; cur = None
for l in open(sys.argv[1], errors='replace'):
    m = re.match(r'^==== VALUE=(\S+) N=(\d+) DOMA_TINY_TOP=(\d+)', l)
    if m: cur = dict(v=m.group(1), n=int(m.group(2)), top=int(m.group(3))); rows.append(cur); continue
    if cur is None: continue
    m = re.match(r'^Tiny ORAM bitstream: run-time depth, L = \d+\.\.\d+; running at L=(\d+)', l) or re.match(r'^Tiny ORAM bitstream: N=\d+ blocks \(L=(\d+)\)', l)
    if m: cur['L'] = int(m.group(1))
    m = re.match(r'^placement: tree (\d+) B = (\d+) B on-package', l)
    if m: cur['tree'] = int(m.group(1)); cur['on'] = int(m.group(2))
    m = re.match(r'^query: (\d+) commands .* in ([\d.]+) s = ([\d.]+) KQPS', l)
    if m: cur['cmds'] = int(m.group(1)); cur['t'] = float(m.group(2)); cur['kqps'] = float(m.group(3))
    m = re.match(r'^device: (\d+) cycles total, per-command latency (\d+)\.\.(\d+) cycles, completed (\d+), errors (0x[0-9a-f]+)', l)
    if m: cur['dev'] = int(m.group(1)); cur['err'] = m.group(5)
    m = re.match(r'^latency spread: min (\d+), p50 (\d+), p90 (\d+), p99 (\d+), p99\.9 (\d+), max (\d+), mean ([\d.]+)', l)
    if m: cur.update(p50=int(m.group(2)), p99=int(m.group(4)), p999=int(m.group(5)), mx=int(m.group(6)), mean=float(m.group(7)))
    if l.startswith('kernel clock: '): cur['mhz'] = float(l.split()[2])
    if l.startswith('ALL RESPONSES CORRECT'): cur['ok'] = 'yes'
    if l.startswith('ERRORS FOUND'): cur['ok'] = 'NO'
print('Tiny ORAM, one record per block, by block (value) size; latency in cycles (microseconds at the kernel clock)')
print('%-6s %-8s %-4s %12s %8s %10s | %9s | %-20s %-20s %-20s %-20s | %7s %s' % ('value', 'N', 'L', 'top B', 'tree MiB', 'top/tree', 'KQPS', 'mean', 'p50', 'p99', 'max', 'MHz*', 'correct'))
for r in rows:
    f = r.get('mhz') or 0
    lat = lambda c: ('%8.0f (%7.2f us)' % (c, c / f)) if (c is not None and f) else ('%8s' % (c if c is not None else '-'))
    eff = (r['dev'] / r['t'] / 1e6) if r.get('dev') and r.get('t') else 0
    print('%-6s %-8d %-4s %12d %8.1f %9.1f%% | %9s | %-20s %-20s %-20s %-20s | %7.0f %s' % (
        r['v'], r['n'], r.get('L', '-'), r.get('on', r['top']), r.get('tree', 0) / 2**20, 100.0 * r.get('on', 0) / r['tree'] if r.get('tree') else 0,
        '%.1f' % r['kqps'] if 'kqps' in r else '-', lat(r.get('mean')), lat(r.get('p50')), lat(r.get('p99')), lat(r.get('mx')),
        eff, r.get('ok', 'NO (see the log)') + ('' if r.get('err', '0x0') == '0x0' else ', errors ' + r['err'])))
print('* MHz: device cycles / query seconds, the clock the card really ran at (slightly low: host time included)')
PY
}
[ "${1:-}" = summary ] && { summary; exit 0; }

command -v xclbinutil > /dev/null || source "$XRT_SETUP" > /dev/null 2>&1
clk() { xclbinutil --info -i "$1" 2>/dev/null | awk '/Name:.*DATA_CLK/{f=1} f && /Frequency:/{print $2; exit}'; }
QUEUE=${QUEUE:-$HOME/tinypm/queue}
depth() { local L=6; while [ $((4 << L)) -lt $1 ]; do L=$((L + 1)); done; echo $L; }
# pick <size> <L>: "<id> <xclbin>" of a DONE build of that block size with the map in HBM that runs depth L: first a run-time-depth one
# (tag _rt_..._hbmpm, patch 11) with L_max >= L, so every row runs on the same design; else a fixed-depth one at L.
# The hardware depends only on the block size and L: a build queued with packed records (tag _p8, e.g. id 40: 1M
# records, L = 15) is the same bitstream as one queued for 131,072 single-record blocks.
pick() {
    for want in rt fixed; do
    for st in "$QUEUE"/*.status; do
        [ -f "$st" ] || continue
        grep -q '^stage=DONE' "$st" || continue
        t=$(grep '^tag=' "$st" | cut -d= -f2-)
        case "$t" in tinyoram_$1_n*_hbmpm) ;; *) continue ;; esac
        lb=$(grep '^geometry=' "$st" | sed -nE 's/.*, L=([0-9]+)$/\1/p'); [ -n "$lb" ] || continue
        case "$t" in *_rt_*) [ $want = rt ] && [ "$lb" -ge "$2" ] || continue ;; *) [ $want = fixed ] && [ "$lb" = "$2" ] || continue ;; esac
        x=$(grep '^xclbin=' "$st" | cut -d= -f2-); [ -f "$x" ] && { echo "$(basename "$st" .status) $x"; return; }
    done; done
}
# at <xclbin>: a copy at CLK MHz (prints the file to run)
at() {
X=$1; f=$(clk "$X")
if [ "$CLK" != 0 ] && [ "$f" != "$CLK" ]; then
    Y=${X%.xclbin}_${CLK}mhz.xclbin
    if [ "$(clk "$Y" 2>/dev/null)" != "$CLK" ]; then
        J=${X%.xclbin}_clk.json; rm -f "$J"
        xclbinutil --quiet --force --input "$X" --dump-section CLOCK_FREQ_TOPOLOGY:JSON:"$J" || { echo "STOP: cannot read the clock section of $X" >&2; exit 2; }
        python3 - "$J" "$CLK" <<'PY' || { echo "STOP: the clock section of $X is not as expected" >&2; exit 2; }
import json, sys
p, f = sys.argv[1], int(sys.argv[2]); d = json.load(open(p)); n = 0
def walk(o):
    global n
    if isinstance(o, dict):
        if o.get('m_type') == 'DATA' and 'm_freq_Mhz' in o: o['m_freq_Mhz'] = type(o['m_freq_Mhz'])(f); n += 1
        for v in o.values(): walk(v)
    elif isinstance(o, list):
        for v in o: walk(v)
walk(d)
if n != 1: sys.exit(1)
json.dump(d, open(p, 'w'), indent=4)
PY
        xclbinutil --quiet --force --input "$X" --replace-section CLOCK_FREQ_TOPOLOGY:JSON:"$J" --output "$Y" || { echo "STOP: cannot write $Y" >&2; exit 2; }
    fi
    X=$Y
fi
echo "$X"
}
# the host program: this package's host_doma.cpp (DOMA's host with Tiny ORAM's top, depth and latency spread)
H=$RES/../host_doma
if [ ! -x "$H" ] || [ "$VIT/host_doma.cpp" -nt "$H" ]; then
    g++ -std=c++17 -O2 "$VIT/host_doma.cpp" -I$XILINX_XRT/include -L$XILINX_XRT/lib -lxrt_coreutil -pthread -o "$H" || { echo "STOP: host build failed"; exit 2; }
fi
: > "$LOG"
echo "Tiny ORAM value-size table sweep, builds from $QUEUE, $CMDS commands, $(date '+%F %T')" | tee -a "$LOG"
fail=0; missing=""
for r in $ROWS; do
    sz=${r%%:*}; rest=${r#*:}; n=${rest%%:*}; t=${rest#*:}; L=$(depth $n)
    case $sz in 1k) bb=1024 ;; 2k) bb=2048 ;; 4k) bb=4096 ;; *) bb=$sz ;; esac
    read -r id X0 <<< "$(pick $sz $L)"
    if [ -z "${X0:-}" ]; then echo "$sz: no DONE $sz build with the map in HBM at L=$L in $QUEUE: skipped"; missing="$missing $sz(L=$L)"; continue; fi
    xclbinutil --info -i "$X0" | grep -qE "Argument: +posmap" || { echo "STOP: $X0 has no posmap argument (not a map-in-HBM build)"; exit 2; }
    X=$(at "$X0") || exit 2; f=$(clk "$X")
    echo "$sz: queue id $id, $(basename "$X"), $f MHz, posmap argument present" | tee -a "$LOG"
    echo "==== VALUE=$sz N=$n DOMA_TINY_TOP=$t" >> "$LOG"
    echo "bitstream: queue id $id, $X (posmap argument present)" >> "$LOG"; echo "kernel clock: $f MHz" >> "$LOG"
    out=$(DOMA_KERNEL=tinyoram_kernel DOMA_BLOCK=$bb DOMA_N=$n DOMA_TINY_TOP=$t "$H" "$X" $CMDS 2>&1) || fail=1
    echo "$out" >> "$LOG"
    q=$(echo "$out" | sed -nE 's/^query:.*= ([0-9.]+) KQPS/\1 KQPS/p'); c=$(echo "$out" | grep -E '^(ALL RESPONSES CORRECT|ERRORS FOUND)')
    printf "%-4s N=%-8s top %10s B: %s, %s\n" $sz $n $t "${q:-no result}" "${c:-$(echo "$out" | tail -1 | cut -c1-100)}"
done
echo; summary
[ -n "$missing" ] && echo "not measured (no build yet):$missing"
[ $fail = 0 ] && echo "PASS: every measured row correct" || echo "FAIL: see $LOG"
exit $fail
