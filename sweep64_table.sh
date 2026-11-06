#!/bin/bash
# sweep64_table.sh -- Tiny ORAM at 64-byte blocks over N, each with the tree top of the DOMA-at-20% table:
#
#   N          DOMA at 20%   Tiny ORAM top (B)   Tiny ORAM tree        top / tree
#   100K       5.8 MiB       5,017,600           30.0 MiB (L = 15)     15.9%
#   500K       28.2 MiB      25,407,488          102.0 MiB (L = 17)    23.7%
#   1M         56.1 MiB      50,479,104          198.0 MiB (L = 18)    24.3%
#   2M         111.9 MiB     100,507,648         390.0 MiB (L = 19)    24.6%
#
#   bash sweep64_table.sh                 runs the four rows, then prints the summary table
#   bash sweep64_table.sh summary         the summary of the last run again
#
# Measured as DOMA's host_doma.cpp measures doma_kernel (it is the same program, with Tiny ORAM's arguments): BOLT's
# generator (mt19937 seed 1, 50% PUT, uniform keys, 8-byte values), every record preloaded during init, the timed
# region = launch + wait() of CMDS commands (default 1,000,000, one launch), every response checked. One record per
# 64-byte block (DOMA_N blocks, as the table's trees), no packing.
# Bitstream: the 64-byte run-time-depth build (L_max = 19 covers all four N without a rebuild): queue id 31
# (plaintext top) if DONE in ~/tiny/queue, else id 8; XCLBIN=<file> picks another. Run at 200 MHz: a copy of the
# bitstream with its kernel clock set to 200 MHz (CLK=0 keeps the build's clock; CLK=<MHz> another one).
# Log: ~/tinypm/results/tiny64_table.log; summary: ~/tinypm/results/tiny64_table.txt
set -u
RES=${RES:-$HOME/tinypm/results}; mkdir -p "$RES"
LOG=$RES/tiny64_table.log; SUM=$RES/tiny64_table.txt
CMDS=${CMDS:-1000000}; CLK=${CLK:-200}
ROWS=${ROWS:-"100000:5017600 500000:25407488 1000000:50479104 2000000:100507648"}
XRT_SETUP=${XRT_SETUP:-/opt/xilinx/xrt/setup.sh}
HERE=$(cd "$(dirname "$0")" && pwd); VIT=$(cd "$HERE/../../vitis" && pwd)

summary() {
    python3 - "$LOG" <<'PY' | tee "$SUM"
import re, sys
rows = []; cur = None
for l in open(sys.argv[1], errors='replace'):
    m = re.match(r'^==== N=(\d+) DOMA_TINY_TOP=(\d+)', l)
    if m: cur = dict(n=int(m.group(1)), top=int(m.group(2))); rows.append(cur); continue
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
print('Tiny ORAM, 64-byte blocks, one 8-byte record per block; latency in cycles (microseconds at the kernel clock)')
print('%-8s %-4s %12s %8s %10s | %9s | %-20s %-20s %-20s %-20s | %7s %s' % ('N', 'L', 'top B', 'tree MiB', 'top/tree', 'KQPS', 'mean', 'p50', 'p99', 'max', 'MHz*', 'correct'))
for r in rows:
    f = r.get('mhz') or 0
    lat = lambda c: ('%8.0f (%7.2f us)' % (c, c / f)) if (c is not None and f) else ('%8s' % (c if c is not None else '-'))
    eff = (r['dev'] / r['t'] / 1e6) if r.get('dev') and r.get('t') else 0
    print('%-8d %-4s %12d %8.1f %9.1f%% | %9s | %-20s %-20s %-20s %-20s | %7.0f %s' % (
        r['n'], r.get('L', '-'), r.get('on', r['top']), r.get('tree', 0) / 2**20, 100.0 * r.get('on', 0) / r['tree'] if r.get('tree') else 0,
        '%.1f' % r['kqps'] if 'kqps' in r else '-', lat(r.get('mean')), lat(r.get('p50')), lat(r.get('p99')), lat(r.get('mx')),
        eff, r.get('ok', 'NO (see the log)') + ('' if r.get('err', '0x0') == '0x0' else ', errors ' + r['err'])))
print('* MHz: device cycles / query seconds, the clock the card really ran at (slightly low: host time included)')
PY
}
[ "${1:-}" = summary ] && { summary; exit 0; }

command -v xclbinutil > /dev/null || source "$XRT_SETUP" > /dev/null 2>&1
clk() { xclbinutil --info -i "$1" 2>/dev/null | awk '/Name:.*DATA_CLK/{f=1} f && /Frequency:/{print $2; exit}'; }
# the bitstream
X=${XCLBIN:-}
if [ -z "$X" ]; then
    for id in 31 8; do
        st=$HOME/tiny/queue/$id.status; [ -f "$st" ] || continue
        grep -q '^stage=DONE' "$st" || continue
        X=$(grep '^xclbin=' "$st" | cut -d= -f2-); [ -f "$X" ] && { echo "bitstream: queue id $id, $(basename "$X")"; break; }; X=""
    done
fi
[ -n "$X" ] && [ -f "$X" ] || { echo "STOP: no 64-byte run-time-depth bitstream DONE (ids 31 or 8 in ~/tiny/queue); set XCLBIN=<file>"; exit 2; }
# the kernel clock
f=$(clk "$X")
if [ "$CLK" != 0 ] && [ "$f" != "$CLK" ]; then
    Y=${X%.xclbin}_${CLK}mhz.xclbin
    if [ "$(clk "$Y" 2>/dev/null)" != "$CLK" ]; then
        J=${X%.xclbin}_clk.json; rm -f "$J"
        xclbinutil --quiet --force --input "$X" --dump-section CLOCK_FREQ_TOPOLOGY:JSON:"$J" || { echo "STOP: cannot read the clock section of $X"; exit 2; }
        python3 - "$J" "$CLK" <<'PY' || { echo "STOP: the clock section of $X is not as expected"; exit 2; }
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
        xclbinutil --quiet --force --input "$X" --replace-section CLOCK_FREQ_TOPOLOGY:JSON:"$J" --output "$Y" || { echo "STOP: cannot write $Y"; exit 2; }
    fi
    echo "kernel clock: $f MHz in the build; running a copy at $CLK MHz ($(basename "$Y"))"; X=$Y; f=$CLK
fi
# the host program: this package's host_doma.cpp (DOMA's host with Tiny ORAM's top, depth and latency spread)
H=$RES/../host_doma
if [ ! -x "$H" ] || [ "$VIT/host_doma.cpp" -nt "$H" ]; then
    g++ -std=c++17 -O2 "$VIT/host_doma.cpp" -I$XILINX_XRT/include -L$XILINX_XRT/lib -lxrt_coreutil -pthread -o "$H" || { echo "STOP: host build failed"; exit 2; }
fi
: > "$LOG"
echo "Tiny ORAM 64-byte table sweep: $(basename "$X") at $f MHz, $CMDS commands, $(date '+%F %T')" | tee -a "$LOG"
fail=0
for r in $ROWS; do
    n=${r%%:*}; t=${r#*:}
    echo "==== N=$n DOMA_TINY_TOP=$t" >> "$LOG"; echo "kernel clock: $f MHz" >> "$LOG"
    out=$(DOMA_KERNEL=tinyoram_kernel DOMA_BLOCK=64 DOMA_N=$n DOMA_TINY_TOP=$t "$H" "$X" $CMDS 2>&1) || fail=1
    echo "$out" >> "$LOG"
    q=$(echo "$out" | sed -nE 's/^query:.*= ([0-9.]+) KQPS/\1 KQPS/p'); c=$(echo "$out" | grep -E '^(ALL RESPONSES CORRECT|ERRORS FOUND)')
    printf "N=%-8s top %10s B: %s, %s\n" $n $t "${q:-no result}" "${c:-$(echo "$out" | tail -1 | cut -c1-100)}"
done
echo; summary
[ $fail = 0 ] && echo "PASS: every row measured, all responses correct" || echo "FAIL: see $LOG"
exit $fail
