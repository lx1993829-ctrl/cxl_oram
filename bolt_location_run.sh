#!/bin/bash
# HBM-only vs host-only query batches on BOLT's released 20%-HBM kernels (Alveo U55C).
#
# For each value size: patch a copy of BOLT's host program (bolt_location_patch.py; the original and BOLT's kernels
# are not changed), then time batches of 5000 distinct keys that all start in HBM ("hbm"), all start in host
# pages ("host"), or are drawn from all keys ("mixed", about 20% HBM). Same seed = same GET/PUT sequence and PUT
# values, so batches of one repetition differ only in where their keys are. Each run is a fresh process with the
# same initialization, as in BOLT's own runs. The order of the three kinds rotates between repetitions.
# For the first value size it also runs all-GET and all-PUT batches (hbm and host).
#
# Usage:  bash bolt_location_run.sh                       # sizes 8 64 256, 6 repetitions
#         SIZES="8" REPS=8 bash bolt_location_run.sh
# Needs:  ~/bolt_location_patch.py, BOLT's repo at ~/BOLT-benchmark (or BOLT_DIR=/path/to/BOLT-benchmark/BOLT)
set -u
SIZES=${SIZES:-"8 64 256"}
REPS=${REPS:-6}
SPLIT_REPS=${SPLIT_REPS:-5}
E=${BOLT_DIR:-$HOME/BOLT-benchmark/BOLT}
PATCH=${PATCH:-$HOME/bolt_location_patch.py}
XRT=${XRT:-"-I/opt/xilinx/xrt/include -L/opt/xilinx/xrt/lib -lxrt_coreutil -lxrt_core -w -Wl,-rpath,/opt/xilinx/xrt/lib -std=c++17"}
LOG=$HOME/bolt_location_$(date +%m%d_%H%M).txt
SUM=$HOME/bolt_location_summary.py
ulimit -s unlimited 2>/dev/null
[ -f /opt/xilinx/xrt/setup.sh ] && source /opt/xilinx/xrt/setup.sh >/dev/null 2>&1

cfg() {  # value size -> directory, host source, xclbin (BOLT's own pairing; 32 B runs padded on the 64 B kernel)
  case $1 in
    8)   echo "$E/hbm_distribution_experiment/20_percent_hbm host.cpp load_balance_1d_array_20_percent_hbm_8_value_size.xclbin" ;;
    16)  echo "$E/value_scaling_experiment/20_percent_hbm/value_size_16 host_16.cpp load_balance_1d_array_20_percent_hbm_16_value_size.xclbin" ;;
    32)  echo "$E/value_scaling_experiment/20_percent_hbm/value_size_32 padding_host_32.cpp load_balance_1d_array_20_percent_hbm_64_value_size.xclbin" ;;
    64|128|256) echo "$E/value_scaling_experiment/20_percent_hbm/value_size_$1 host_$1.cpp load_balance_1d_array_20_percent_hbm_$1_value_size.xclbin" ;;
    *) echo "unknown value size $1" >&2; exit 1 ;;
  esac
}

run_one() {  # value_size kind put_fraction rep seed
  local v=$1 kind=$2 put=$3 rep=$4 seed=$5 dir src xcl out
  read dir src xcl <<< "$(cfg $v)"
  out=$(cd "$dir" && BOLT_BATCH=$kind BOLT_SEED=$seed BOLT_PUT_FRAC=$put ./loc_bolt_$v "$xcl" < /dev/null 2>&1)
  { echo "=== v=$v kind=$kind put=$put rep=$rep seed=$seed"; echo "$out"; } >> "$LOG"
  echo "$out" | awk -v h="v=$v $kind put=$put rep $rep" '
    /^total time =/ { t = $4 } /^GET check:/ { g = $3 "/" $6 } /^STATE/ { split($2, a, "="); p = a[2] }
    END { printf "%-28s %s ms   GET wrong %s   host pages changed %s\n", h, (t == "" ? "FAILED" : sprintf("%.3f", t * 1e3)), g, p }'
}

cat > "$SUM" <<'PYEOF'
#!/usr/bin/env python3
"""Summarize a bolt_location_run.sh log: per-query times by key location, and what each run changed."""
import math, re, statistics as st, sys
from collections import defaultdict

runs, cur = [], None
for line in open(sys.argv[1], errors="replace"):
    line = line.rstrip("\n")
    m = re.match(r"=== v=(\d+) kind=(\w+) put=([\d.]+) rep=(\d+) seed=(\d+)", line)
    if m:
        cur = dict(v=int(m[1]), kind=m[2], put=m[3], rep=int(m[4]), seed=int(m[5])); runs.append(cur); continue
    if cur is None:
        continue
    if line.startswith("BATCH"):
        cur["batch"] = dict(kv.split("=", 1) for kv in line.split()[1:])
    elif line.startswith("total time ="):
        cur["t"] = float(line.split("=")[1].strip().rstrip("s"))
    elif line.startswith("GET check:"):
        a = line.split(); cur["bad"], cur["gets"] = int(a[2]), int(a[5])
    elif line.startswith("Insert error count is"):
        cur["ins_err"] = int(line.split()[-1])
    elif line.startswith("STATE"):
        head, _, locs = line.partition("queried_keys_location_type:")
        s = {k: int(v) for k, v in (kv.split("=") for kv in head.split()[1:])}
        for kv in locs.split():
            k, v = kv.split("="); s["missing" if k == "missing" else "type" + k] = int(v)
        cur["state"] = s

good = [r for r in runs if "t" in r and "batch" in r]
bad = [r for r in runs if r not in good]
for r in good:
    r["us"] = r["t"] / int(r["batch"]["size"]) * 1e6
groups = defaultdict(list)
for r in good:
    groups[(r["v"], r["put"], r["kind"])].append(r)
order = {"hbm": 0, "host": 1, "mixed": 2, "uniform": 3}
keys = sorted(groups, key=lambda k: (k[0], k[1] != "0.5", k[1], order.get(k[2], 9)))

def sd(x):
    return st.stdev(x) if len(x) > 1 else 0.0

print(f"\nRuns: {len(good)} complete, {len(bad)} incomplete" + (" (see the log)" if bad else ""))
print("\n1. Time per query (batch time / 5000), microseconds")
print(f"{'value':>6} {'PUT share':>9} {'keys start in':>13} {'runs':>4} {'median':>8} {'mean':>8} {'sd':>7} "
      f"{'min':>8} {'max':>8} {'GET wrong':>10} {'resident HBM/host':>18}")
for k in keys:
    rs = groups[k]; us = [r["us"] for r in rs]
    res = sorted({(r["batch"]["hbm_resident"], r["batch"]["host_resident"]) for r in rs})
    resid = "/".join(res[0]) if len(res) == 1 else "varies"
    wrong = f"{sum(r.get('bad', 0) for r in rs)}/{sum(r.get('gets', 0) for r in rs)}"
    print(f"{k[0]:>5}B {k[1]:>9} {k[2]:>13} {len(us):>4} {st.median(us):8.3f} {st.mean(us):8.3f} {sd(us):7.3f} "
          f"{min(us):8.3f} {max(us):8.3f} {wrong:>10} {resid:>18}")

print("\n2. Host-resident vs HBM-resident keys (same seeds, so pairs share the GET/PUT sequence)")
print(f"{'value':>6} {'PUT share':>9} {'HBM mean':>9} {'host mean':>9} {'host-HBM':>9} {'host/HBM':>8} "
      f"{'paired diff min..max':>21} {'Welch t':>8} {'ranges overlap':>14}")
for v, put in sorted({(k[0], k[1]) for k in keys}, key=lambda x: (x[0], x[1] != "0.5", x[1])):
    h, o = groups.get((v, put, "hbm")), groups.get((v, put, "host"))
    if not h or not o:
        continue
    hu, ou = [r["us"] for r in h], [r["us"] for r in o]
    hs, os_ = {r["seed"]: r["us"] for r in h}, {r["seed"]: r["us"] for r in o}
    diffs = [os_[s] - hs[s] for s in hs if s in os_]
    se = math.sqrt(sd(hu) ** 2 / len(hu) + sd(ou) ** 2 / len(ou)) if len(hu) > 1 and len(ou) > 1 else float("nan")
    t = (st.mean(ou) - st.mean(hu)) / se if se and se == se and se > 0 else float("nan")
    overlap = "yes" if max(min(hu), min(ou)) <= min(max(hu), max(ou)) else "no"
    pd = f"{min(diffs):+.3f}..{max(diffs):+.3f}" if diffs else "-"
    print(f"{v:>5}B {put:>9} {st.mean(hu):9.3f} {st.mean(ou):9.3f} {st.mean(ou) - st.mean(hu):+9.3f} "
          f"{st.mean(ou) / st.mean(hu):8.3f} {pd:>21} {t:8.1f} {overlap:>14}")
    m = groups.get((v, put, "mixed"))
    if m:
        f = st.mean(int(r["batch"]["hbm_resident"]) / int(r["batch"]["size"]) for r in m)
        pred = f * st.mean(hu) + (1 - f) * st.mean(ou)
        print(f"{'':>16} mixed ({100 * f:.1f}% HBM keys): measured {st.mean(r['us'] for r in m):.3f}, "
              f"predicted from HBM and host means {pred:.3f}")

print("\n3. What the runs changed (medians over runs; read back after the timed run)")
print(f"{'value':>6} {'PUT share':>9} {'keys start in':>13} {'host pages changed':>18} {'start pages changed':>21} "
      f"{'evict-buffer entries':>20} {'page counts changed':>19}  queried keys afterwards (location type: count)")
for k in keys:
    rs = [r for r in groups[k] if "state" in r]
    if not rs:
        continue
    med = lambda f: st.median(r["state"].get(f, 0) for r in rs)
    types = sorted({t for r in rs for t in r["state"] if t.startswith("type")}, key=lambda t: int(t[4:]))
    locs = ", ".join(f"{t[4:]}: {med(t):g}" for t in types) + (f", missing: {med('missing'):g}" if med("missing") else "")
    spc = f"{med('of_which_start_pages'):g} of {med('start_pages'):g}"
    print(f"{k[0]:>5}B {k[1]:>9} {k[2]:>13} {med('pages_changed'):>18g} {spc:>21} "
          f"{med('eviction_buffer_entries'):>20g} {med('page_counts_changed'):>19g}  {locs}")
print("\nLocation types: 0 = HBM and 1 = host page, as BOLT's host code writes them; other codes are set by the kernel.")
PYEOF

echo "log: $LOG"
for v in $SIZES; do
  read dir src xcl <<< "$(cfg $v)"
  [ -f "$dir/$xcl" ] || { echo "missing $dir/$xcl" >&2; exit 1; }
  ( cd "$dir" && python3 "$PATCH" "$src" && g++ "loc_$src" -o "loc_bolt_$v" $XRT ) || { echo "build failed for $v B" >&2; exit 1; }
done
kinds=(hbm host mixed)
first=1
for v in $SIZES; do
  for rep in $(seq 1 $REPS); do
    for j in 0 1 2; do
      run_one $v ${kinds[$(( (rep - 1 + j) % 3 ))]} 0.5 $rep $rep
    done
  done
  if [ $first = 1 ]; then                       # all-GET and all-PUT batches, first value size only
    for rep in $(seq 1 $SPLIT_REPS); do
      for put in 0 1; do
        if [ $((rep % 2)) = 1 ]; then pair="hbm host"; else pair="host hbm"; fi
        for kind in $pair; do run_one $v $kind $put $rep $((100 + rep)); done
      done
    done
    first=0
  fi
done
python3 "$SUM" "$LOG" | tee "$LOG.summary.txt"
echo "log: $LOG   summary: $LOG.summary.txt"
