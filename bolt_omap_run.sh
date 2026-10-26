#!/bin/bash
# bolt_omap_run.sh -- what key handling costs BOLT, measured on its released U55C kernels (300 MHz), host side only.
#
# BOLT's kernel source is not released and its bitstreams carry no performance monitors, so the kernels cannot be
# instrumented. Every experiment here changes only which keys the host stores and which commands it sends
# (bolt_omap_patch.py writes omap_*.cpp copies; the .xclbin files and BOLT's host programs are untouched):
#
#   20%-HBM build, 64 B values, 1M entries
#     random/uniform   BOLT's own workload (keys and commands), 20,000 commands
#     random/hbm       distinct keys initialization put in HBM      -> no host-page access
#     random/host      distinct keys initialization put in host pages
#     dense/uniform    keys 0..N-1: BOLT's bucket index (key mod 3*2^20) becomes the identity, one key per bucket
#     dense/hbm        same, HBM keys only
#     collide/slot0    every bucket holds 16 keys; HBM keys sitting in slot 0 of their bucket
#     collide/slot15   same buckets, keys in slot 15                -> does the lookup cost depend on scan depth?
#   20%-HBM build, 8 B values, 1M entries: random/hbm and random/host
#   BOLT's all-HBM "baseline" build, 64 B values (the artifact's own non-oblivious comparison point)
#     1M entries: random/uniform, dense/uniform, collide/slot0, collide/slot15
#     100K, 500K, 2M, 5M, 10M entries: random/uniform        (compared with your direct-indexed baseline kernel)
#   MISS=1 adds GETs of keys that are not stored (hash + lookup with nothing to fetch), at the very end.
#     Off by default: the released kernels were never shown handling an absent key, and a hung kernel needs a card
#     reset (root). Run it only if that risk is acceptable on the shared card.
#
# Each run is a fresh process with BOLT's own initialization. REPS repetitions of every 1M-entry row (keys and
# GET/PUT sequence change with the repetition for hbm/host/slot batches; BOLT's uniform batch is fixed by its seed).
#
# Usage:  bash bolt_omap_run.sh                 # about 20-30 minutes
#         REPS=5 bash bolt_omap_run.sh
# Needs:  ~/bolt_omap_patch.py, BOLT's repo at ~/BOLT-benchmark (or BOLT_DIR=...). Uses the card lock of baseline_all.sh.
# Output: ~/bolt_omap_<date>.txt (log), .csv, .summary.txt
set -u
REPS=${REPS:-3}
MISS=${MISS:-0}
SWEEP_NS=${SWEEP_NS:-"100000 500000 2000000 5000000 10000000"}
E=${BOLT_DIR:-$HOME/BOLT-benchmark/BOLT}
PATCH=${PATCH:-$HOME/bolt_omap_patch.py}
LOCK=${LOCK:-/tmp/baseline_card.lock}
BASE_CSV=${BASE_CSV:-$HOME/keep/results/baseline/baseline_results.csv}
XRT=${XRT:-"-I/opt/xilinx/xrt/include -L/opt/xilinx/xrt/lib -lxrt_coreutil -lxrt_core -w -Wl,-rpath,/opt/xilinx/xrt/lib -std=c++17"}
OUT=${OUT:-$HOME}
STAMP=$(date +%m%d_%H%M)
LOG=$OUT/bolt_omap_$STAMP.txt; CSV=$OUT/bolt_omap_$STAMP.csv; SUM=$OUT/bolt_omap_$STAMP.summary.txt
ulimit -s unlimited 2>/dev/null
[ -f /opt/xilinx/xrt/setup.sh ] && source /opt/xilinx/xrt/setup.sh >/dev/null 2>&1
[ -f "$PATCH" ] || { echo "missing $PATCH"; exit 1; }

cfg() {  # build -> directory, host source, xclbin
  case $1 in
    bolt64) echo "$E/value_scaling_experiment/20_percent_hbm/value_size_64 host_64.cpp load_balance_1d_array_20_percent_hbm_64_value_size.xclbin" ;;
    bolt8)  echo "$E/hbm_distribution_experiment/20_percent_hbm host.cpp load_balance_1d_array_20_percent_hbm_8_value_size.xclbin" ;;
    base64) echo "$E/value_scaling_experiment/baseline/value_size_64 host_64.cpp load_balance_1d_array_100_percent_hbm_64_value_size.xclbin" ;;
    *) echo "unknown build $1" >&2; return 1 ;;
  esac
}

# rows: build N keys batch commands reps
ROWS=()
for r in "bolt64 random uniform 20000" "bolt64 random hbm 10000" "bolt64 random host 10000" \
         "bolt64 dense uniform 20000" "bolt64 dense hbm 10000" "bolt64 collide slot0 10000" "bolt64 collide slot15 10000" \
         "bolt8 random hbm 10000" "bolt8 random host 10000" \
         "base64 random uniform 20000" "base64 dense uniform 20000" "base64 collide slot0 10000" "base64 collide slot15 10000"; do
  set -- $r; ROWS+=("$1 1000000 $2 $3 $4 $REPS")
done
for n in $SWEEP_NS; do ROWS+=("base64 $n random uniform 20000 1"); done
[ "$MISS" = 1 ] && ROWS+=("bolt64 1000000 random miss 1000 1" "base64 1000000 random miss 1000 1")

echo "log: $LOG"
declare -A PATCHED BUILT
for row in "${ROWS[@]}"; do
  set -- $row; b=$1; n=$2
  read dir src xcl <<< "$(cfg $b)"
  [ -f "$dir/$xcl" ] || { echo "missing $dir/$xcl"; exit 1; }
  if [ -z "${PATCHED[$dir]:-}" ]; then
    ( cd "$dir" && python3 "$PATCH" "$src" ) || { echo "patch failed: $b"; exit 1; }
    PATCHED[$dir]=1
  fi
  if [ -z "${BUILT[$b:$n]:-}" ]; then
    echo "building $b, N = $n"
    ( cd "$dir" && g++ "omap_$src" -o "omap_${b}_$n" -DINIT_SIZE=$n -DCMD_SIZE=20000 $XRT ) || { echo "build failed: $b N=$n"; exit 1; }
    BUILT[$b:$n]=1
  fi
done

oncard() { if command -v flock > /dev/null; then flock "$LOCK" "$@"; else "$@"; fi; }
run_row() {  # build N keys batch cmds rep
  local b=$1 n=$2 k=$3 t=$4 c=$5 rep=$6 dir src xcl out
  read dir src xcl <<< "$(cfg $b)"
  out=$(cd "$dir" && oncard env BOLT_KEYS=$k BOLT_BATCH=$t BOLT_CMDS=$c BOLT_SEED=$rep "./omap_${b}_$n" "$xcl" < /dev/null 2>&1)
  { echo "=== build=$b n=$n keys=$k batch=$t cmds=$c rep=$rep"; echo "$out"; } >> "$LOG"
  echo "$out" | awk -v h="$(printf '%-6s N=%-8s %-7s %-7s rep %s' $b $n $k $t $rep)" '
    /^TIMED/ { split($4, a, "="); t = sprintf("%7.3f us/query", a[2]) }
    /^GET check:/ { g = $3 "/" $6 } /^FINAL/ { f = $2 " " $3 " " $4 } /^ABSENT/ { m = $0 }
    END { printf "%s  %s  GET wrong %s  %s %s\n", h, (t == "" ? "FAILED (see log)" : t), g, f, m }'
}
MAXR=$REPS
for rep in $(seq 1 $MAXR); do

  for row in "${ROWS[@]}"; do
    set -- $row
    [ "$4" = miss ] && continue
    [ "$rep" -le "$6" ] && run_row $1 $2 $3 $4 $5 $rep
  done
done
for row in "${ROWS[@]}"; do set -- $row; [ "$4" = miss ] && run_row $1 $2 $3 $4 $5 1; done

cat > "$OUT/.bolt_omap_summary.py" <<'PYEOF'
import csv, os, re, statistics as st, sys
from collections import defaultdict
log, csv_out, base_csv = sys.argv[1:4]
runs, cur = [], None
kv = lambda s: dict(x.split("=", 1) for x in s.split() if "=" in x)
for line in open(log, errors="replace"):
    line = line.rstrip("\n")
    m = re.match(r"=== build=(\w+) n=(\d+) keys=(\w+) batch=(\w+) cmds=(\d+) rep=(\d+)", line)
    if m:
        cur = dict(build=m[1], n=int(m[2]), keys=m[3], batch=m[4], cmds=int(m[5]), rep=int(m[6])); runs.append(cur); continue
    if cur is None: continue
    if line.startswith("TIMED"): cur["us"] = float(kv(line)["us_per_command"])
    elif line.startswith("GET check:"):
        a = re.match(r"GET check: (\d+) wrong of (\d+)", line); cur["bad"], cur["gets"] = int(a[1]), int(a[2])
    elif line.startswith("RESP"): cur["acked"] = int(line.split()[1].split("=")[1])
    elif line.startswith("ABSENT"): cur.update({k: int(v) for k, v in kv(line).items()})
    elif line.startswith("FINAL"):
        d = kv(line); cur["final"] = d
        cur["final_bad"] = sum(int(v.split("/")[1]) - int(v.split("/")[0]) for k, v in d.items() if "/" in v)
        cur["final_n"] = sum(int(v.split("/")[1]) for k, v in d.items() if "/" in v)
    elif line.startswith("PAGEWRITES"): cur["pw"] = {k: int(v) for k, v in kv(line).items()}
    elif line.startswith("FOOTPRINT"): cur["fp"] = {k: int(v) for k, v in kv(line).items()}
    elif line.startswith("RESIDENCY"):
        b, a = line.split("before:")[1].split("after:")
        cur["hbm0"] = int(dict(x.split("=") for x in b.split()).get("0", 0))
        cur["hbm1"] = int(dict(x.split("=") for x in a.split() if x[0].isdigit()).get("0", 0))
        cur["nkeys"] = sum(int(x.split("=")[1]) for x in b.split())
good = [r for r in runs if "us" in r and "final" in r]
with open(csv_out, "w") as f:
    w = csv.writer(f)
    w.writerow(["build", "entries", "keys", "batch", "commands", "rep", "us_per_query", "get_wrong", "gets", "acked",
                "final_mismatch", "final_keys", "page_tuples_written", "page_tuples_plaintext_current", "absent_err", "absent_ack"])
    for r in good:
        w.writerow([r["build"], r["n"], r["keys"], r["batch"], r["cmds"], r["rep"], r["us"], r.get("bad"), r.get("gets"),
                    r.get("acked"), r["final_bad"], r["final_n"], r.get("pw", {}).get("kernel_written_tuples"),
                    r.get("pw", {}).get("current_plaintext"), r.get("answered_err", ""), r.get("answered_ack", "")])
g = defaultdict(list)
for r in good: g[(r["build"], r["n"], r["keys"], r["batch"])].append(r)
def mean(k): return st.mean(r["us"] for r in g[k]) if g.get(k) else None
def sd(k): return st.stdev([r["us"] for r in g[k]]) if len(g.get(k, [])) > 1 else 0.0
NAME = {"bolt64": "BOLT 20%, 64 B", "bolt8": "BOLT 20%, 8 B", "base64": "BOLT all-HBM, 64 B"}
print(f"Runs: {len(good)} complete, {len(runs) - len(good)} incomplete" + (" (see the log)" if len(runs) > len(good) else ""))
print("\n1. Time per query (one kernel call per run; mean and sd over runs)")
print(f"{'build':20} {'entries':>9} {'keys':>8} {'batch':>8} {'cmds':>6} {'runs':>4} {'us/query':>9} {'sd':>6} "
      f"{'GET wrong':>11} {'final mismatch':>15} {'pages written/plaintext':>24} {'HBM keys':>15}")
order = {"bolt64": 0, "bolt8": 1, "base64": 2}
for k in sorted(g, key=lambda k: (order.get(k[0], 9), k[1] != 1000000, k[1], k[2], k[3])):
    rs = g[k]; r0 = rs[0]
    pw = f"{sum(r.get('pw', {}).get('kernel_written_tuples', 0) for r in rs)}/{sum(r.get('pw', {}).get('current_plaintext', 0) for r in rs)}"
    hb = f"{100 * r0['hbm0'] / r0['nkeys']:.1f}%->{100 * r0['hbm1'] / r0['nkeys']:.1f}%" if r0.get("nkeys") else "-"
    extra = f"  absent: ERR {r0.get('answered_err')}, ACK {r0.get('answered_ack')}" if k[3] == "miss" else ""
    print(f"{NAME.get(k[0], k[0]):20} {k[1]:>9} {k[2]:>8} {k[3]:>8} {r0['cmds']:>6} {len(rs):>4} {mean(k):9.3f} {sd(k):6.3f} "
          f"{str(sum(r.get('bad', 0) for r in rs)) + '/' + str(sum(r.get('gets', 0) for r in rs)):>11} "
          f"{str(sum(r['final_bad'] for r in rs)) + '/' + str(sum(r['final_n'] for r in rs)):>15} {pw:>24} {hb:>15}{extra}")

def diff(label, a, b):
    ma, mb = mean(a), mean(b)
    if ma is None or mb is None: return
    print(f"  {label:58} {mb:7.3f} - {ma:7.3f} = {mb - ma:+7.3f} us  ({100 * (mb - ma) / ma:+.1f}%)")
M = 1000000
print("\n2. Differences (same build, same entries)")
diff("host-page keys vs HBM keys, 64 B", ("bolt64", M, "random", "hbm"), ("bolt64", M, "random", "host"))
diff("host-page keys vs HBM keys, 8 B", ("bolt8", M, "random", "hbm"), ("bolt8", M, "random", "host"))
diff("slot 15 vs slot 0 of a full bucket, BOLT 20% (HBM keys)", ("bolt64", M, "collide", "slot0"), ("bolt64", M, "collide", "slot15"))
diff("slot 15 vs slot 0 of a full bucket, all-HBM build", ("base64", M, "collide", "slot0"), ("base64", M, "collide", "slot15"))
diff("full bucket (slot 0) vs one key per bucket, BOLT 20% HBM keys", ("bolt64", M, "dense", "hbm"), ("bolt64", M, "collide", "slot0"))
diff("random keys vs dense keys, BOLT 20%, BOLT's workload", ("bolt64", M, "dense", "uniform"), ("bolt64", M, "random", "uniform"))
diff("random keys vs dense keys, all-HBM build", ("base64", M, "dense", "uniform"), ("base64", M, "random", "uniform"))
diff("random HBM keys vs dense HBM keys, BOLT 20%", ("bolt64", M, "dense", "hbm"), ("bolt64", M, "random", "hbm"))
if g.get(("bolt64", M, "random", "miss")):
    diff("stored HBM key vs absent key, BOLT 20%", ("bolt64", M, "random", "miss"), ("bolt64", M, "random", "hbm"))

print("\n3. BOLT's all-HBM build vs your direct-indexed baseline kernel (64 B, 300 MHz, 1 record per slot)")
direct = {100000: 2351.4, 500000: 2352.1, 1000000: 2354.6, 2000000: 2332.9}   # baseline_results.csv as of the last run
src = "values recorded earlier"
if os.path.exists(base_csv):
    for r in csv.DictReader(open(base_csv)):
        if r["slot_b"] == "64" and r["value_b"] == "64" and r["per_slot"] == "1" and r["clock_mhz"] == "300" and r["correct"] == "1":
            direct[int(r["n"])] = float(r["kqps"])
    src = base_csv
print(f"  (direct-indexed: {src})")
print(f"  {'entries':>9} {'BOLT all-HBM us':>16} {'direct us':>10} {'difference us':>14} {'ratio':>6}")
for n in sorted({k[1] for k in g if k[0] == "base64" and k[2] == "random" and k[3] == "uniform"}):
    b = mean(("base64", n, "random", "uniform")); d = 1e3 / direct[n] if n in direct else None
    print(f"  {n:>9} {b:16.3f} {(f'{d:.3f}' if d else '-'):>10} {(f'{b - d:+.3f}' if d else '-'):>14} {(f'{b / d:.1f}x' if d else '-'):>6}")

fp = next((r["fp"] for r in good if r["build"] == "bolt64" and r["n"] == M and "fp" in r), None)
if fp:
    MiB = lambda x: x / 2**20
    print("\n4. On-card buffers BOLT's kernels are given (64 B values, 1M entries; bytes allocated by BOLT's host code)")
    print(f"  position map   {MiB(fp['position_map']):8.1f} MiB  = {fp['buckets']:,} buckets x {fp['slots_per_bucket']} slots x "
          f"{fp['map_entry_bytes']} B; {fp['entries_used']:,} entries in use = {MiB(fp['entries_used'] * fp['map_entry_bytes']):.1f} MiB")
    print(f"  eviction buf.  {MiB(fp['eviction_buffer']):8.1f} MiB")
    print(f"  free-slot queue{MiB(fp['queue']):8.1f} MiB")
    print(f"  page counts    {MiB(fp['page_counts']):8.1f} MiB")
    print(f"  value array    {MiB(fp['value']):8.1f} MiB  (a slot for every entry)")
    print(f"  host pages     {MiB(fp['host_pages']):8.1f} MiB  (host memory)")
print("\nGET wrong: replies differing from the latest value written to the key. final mismatch: stored values (found through")
print("the position map the kernel left) that differ from the latest value written. pages written/plaintext: host-page")
print("tuples the kernel wrote during the runs / of those, the ones the map points at and that hold the latest value in")
print("plaintext. HBM keys: share of keys in HBM at initialization and after the run (first run of the row).")
PYEOF
python3 "$OUT/.bolt_omap_summary.py" "$LOG" "$CSV" "$BASE_CSV" | tee "$SUM"
echo "log: $LOG   csv: $CSV   summary: $SUM"
