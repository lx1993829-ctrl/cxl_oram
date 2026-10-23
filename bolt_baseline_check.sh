#!/bin/bash
# Does BOLT's all-HBM "FPGA baseline" kernel still remap keys like BOLT does?
#
# Runs the same batches -- 5000 distinct keys, all in HBM, half GETs and half PUTs -- on BOLT's baseline build and
# on BOLT's 20%-HBM build, at 8 B and 64 B, REPS times each. Host-side only (bolt_location_patch.py makes patched
# copies; BOLT's kernels and host programs are unchanged). After each timed run the patched program reads back the
# position map, the free-slot queue, the eviction buffer and the per-page counts and reports what changed.
# A plain key-value store moves nothing: GETs change no entries, PUTs overwrite values in place.
#
# Usage:  bash bolt_baseline_check.sh          (REPS=3 by default)
# Needs:  ~/bolt_location_patch.py, BOLT's repo at ~/BOLT-benchmark (or BOLT_DIR=/path/to/BOLT-benchmark/BOLT)
set -u
REPS=${REPS:-3}
E=${BOLT_DIR:-$HOME/BOLT-benchmark/BOLT}
PATCH=${PATCH:-$HOME/bolt_location_patch.py}
XRT=${XRT:-"-I/opt/xilinx/xrt/include -L/opt/xilinx/xrt/lib -lxrt_coreutil -lxrt_core -w -Wl,-rpath,/opt/xilinx/xrt/lib -std=c++17"}
LOG=$HOME/bolt_baseline_check_$(date +%m%d_%H%M).txt
ulimit -s unlimited 2>/dev/null
[ -f /opt/xilinx/xrt/setup.sh ] && source /opt/xilinx/xrt/setup.sh >/dev/null 2>&1

cfg() {  # build -> directory, host source, xclbin
  case $1 in
    base8)  echo "$E/hbm_distribution_experiment/baseline host.cpp load_balance_1d_array_full_hbm_8_value_size.xclbin" ;;
    bolt8)  echo "$E/hbm_distribution_experiment/20_percent_hbm host.cpp load_balance_1d_array_20_percent_hbm_8_value_size.xclbin" ;;
    base64) echo "$E/value_scaling_experiment/baseline/value_size_64 host_64.cpp load_balance_1d_array_100_percent_hbm_64_value_size.xclbin" ;;
    bolt64) echo "$E/value_scaling_experiment/20_percent_hbm/value_size_64 host_64.cpp load_balance_1d_array_20_percent_hbm_64_value_size.xclbin" ;;
  esac
}
BUILDS="base8 bolt8 base64 bolt64"

echo "log: $LOG"
for c in $BUILDS; do
  read dir src xcl <<< "$(cfg $c)"
  [ -f "$dir/$xcl" ] || { echo "missing $dir/$xcl" >&2; exit 1; }
  ( cd "$dir" && python3 "$PATCH" "$src" > /dev/null && g++ "loc_$src" -o "loc_chk_$c" $XRT ) || { echo "build failed: $c" >&2; exit 1; }
done
for c in $BUILDS; do
  read dir src xcl <<< "$(cfg $c)"
  for rep in $(seq 1 $REPS); do
    out=$(cd "$dir" && BOLT_BATCH=hbm BOLT_SEED=$rep BOLT_PUT_FRAC=0.5 "./loc_chk_$c" "$xcl" < /dev/null 2>&1)
    { echo "=== build=$c rep=$rep"; echo "$out"; } >> "$LOG"
    echo "$out" | awk -v h="$c rep $rep" '/^total time =/ { t = $4 } /^MOVES/ { m = $0 }
      END { printf "%-12s %s ms   %s\n", h, (t == "" ? "FAILED" : sprintf("%.3f", t * 1e3)), m }'
  done
done

python3 - "$LOG" <<'PYEOF'
import re, statistics as st, sys
from collections import defaultdict
runs, cur = [], None
for line in open(sys.argv[1], errors="replace"):
    line = line.rstrip("\n")
    m = re.match(r"=== build=(\w+) rep=(\d+)", line)
    if m:
        cur = {"build": m[1]}; runs.append(cur); continue
    if cur is None:
        continue
    if line.startswith("total time ="):
        cur["us"] = float(line.split("=")[1].strip().rstrip("s")) / 5000 * 1e6
    elif line.startswith("GET check:"):
        a = line.split(); cur["bad"] = int(a[2])
    elif line.startswith("STATE"):
        head, _, locs = line.partition("queried_keys_location_type:")
        cur.update({k: int(v) for k, v in (kv.split("=") for kv in head.split()[1:])})
        cur["locs"] = locs.strip()
    elif line.startswith("MOVES"):
        for kv in line.split()[1:]:
            k, v = kv.split("=")
            cur[k] = v
g = defaultdict(list)
for r in runs:
    if "us" in r and "position_map_entries_changed" in r:
        g[r["build"]].append(r)
names = {"base8": "baseline, 8 B", "bolt8": "BOLT 20%, 8 B", "base64": "baseline, 64 B", "bolt64": "BOLT 20%, 64 B"}
print("\nSame batches on both builds: 5000 distinct keys, all in HBM, half GETs and half PUTs\n")
print(f"{'build':16} {'runs':>4} {'us/query':>9} {'GET wrong':>9} {'map entries changed':>20} {'GET keys moved':>15} "
      f"{'PUT keys moved':>15} {'queue changed':>13} {'evict-buffer':>12} {'page counts':>11}  queried keys afterwards")
for b in ("base8", "bolt8", "base64", "bolt64"):
    rs = g.get(b)
    if not rs:
        print(f"{names[b]:16} no complete runs"); continue
    med = lambda k: st.median(int(str(r[k]).split("/")[0]) for r in rs)
    den = lambda k: rs[0][k].split("/")[1]
    print(f"{names[b]:16} {len(rs):>4} {st.mean(r['us'] for r in rs):9.3f} {sum(r.get('bad', 0) for r in rs):>9} "
          f"{med('position_map_entries_changed'):>20g} {med('get_keys_moved'):>9g}/{den('get_keys_moved'):<5} "
          f"{med('put_keys_moved'):>9g}/{den('put_keys_moved'):<5} {med('queue_entries_changed'):>13g} "
          f"{med('eviction_buffer_entries'):>12g} {med('page_counts_changed'):>11g}  {rs[0]['locs']}")
print("\nMedians over runs. Location types: 0 = HBM, 1 = host page, as BOLT's host code writes them; other codes"
      " are set by the kernel.")
PYEOF
echo "log: $LOG"
