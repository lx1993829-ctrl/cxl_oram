#!/bin/bash
# bolt_avg_run.sh -- BOLT's released 20%-HBM kernels on the U55C, 1M commands per data point as 50 independent runs
# of 20,000 commands (BOLT's kernels return wrong values in long single runs; at 20,000 commands both kernel families
# are correct and BOLT stays near its 20%-HBM configuration).
#
#   value-size sweep: 8, 16, 32, 64, 128, 256 B at N = 1M entries
#   data-size sweep:  N = 100K, 500K, 1M, 2M at 64 B
#
# Each run is a fresh process: BOLT's own initialization (the same keys, values and 20% HBM placement every time),
# then one timed kernel call over CMDS commands from BOLT's generator (uniform keys, each a PUT with probability 0.5)
# with command seed 1, 2, .. RUNS (seed 1 is BOLT's own batch). A data point is all its runs together:
# time per query = total kernel time / total commands. Host side only: bolt_cmds_patch.py writes cmd_*.cpp copies;
# BOLT's kernels (.xclbin) and host programs are not changed. One run at a time on the card (baseline_all.sh's lock).
# All runs of one point go back to back, so the card is reprogrammed only when the value size changes.
#
# Usage:  bash bolt_avg_run.sh                                    # 9 points x 50 runs, about 1.5-2 hours
#         VSIZES="64" NS="100000 500000 1000000 2000000" bash bolt_avg_run.sh     # only the data-size sweep
#         RUNS=10 bash bolt_avg_run.sh                            # 200K commands per point, for a quick look
# Needs:  ~/bolt_cmds_patch.py (with BOLT_SEED), BOLT's repo at ~/BOLT-benchmark (or BOLT_DIR=...)
# Output: ~/bolt_avg_<date>.txt (full log), .csv (one row per run), .summary.txt (one row per point)
set -u
CMDS=${CMDS:-20000}
RUNS=${RUNS:-50}
VSIZES=${VSIZES:-"8 16 32 64 128 256"}
NS=${NS:-"100000 500000 1000000 2000000"}
E=${BOLT_DIR:-$HOME/BOLT-benchmark/BOLT}
PATCH=${PATCH:-$HOME/bolt_cmds_patch.py}
LOCK=${LOCK:-/tmp/baseline_card.lock}
XRT=${XRT:-"-I/opt/xilinx/xrt/include -L/opt/xilinx/xrt/lib -lxrt_coreutil -lxrt_core -w -Wl,-rpath,/opt/xilinx/xrt/lib -std=c++17"}
OUT=${OUT:-$HOME}
STAMP=$(date +%m%d_%H%M)
LOG=$OUT/bolt_avg_$STAMP.txt; CSV=$OUT/bolt_avg_$STAMP.csv; SUM=$OUT/bolt_avg_$STAMP.summary.txt
ulimit -s unlimited 2>/dev/null
[ -f /opt/xilinx/xrt/setup.sh ] && source /opt/xilinx/xrt/setup.sh >/dev/null 2>&1
[ -f "$PATCH" ] || { echo "missing $PATCH"; exit 1; }
grep -q BOLT_SEED "$PATCH" || { echo "$PATCH has no BOLT_SEED: fetch the current bolt_cmds_patch.py"; exit 1; }

cfg() {  # value size -> directory, host source, xclbin (BOLT's own pairing; 32 B padded on the 64 B kernel)
  case $1 in
    8)   echo "$E/hbm_distribution_experiment/20_percent_hbm host.cpp load_balance_1d_array_20_percent_hbm_8_value_size.xclbin" ;;
    16)  echo "$E/value_scaling_experiment/20_percent_hbm/value_size_16 host_16.cpp load_balance_1d_array_20_percent_hbm_16_value_size.xclbin" ;;
    32)  echo "$E/value_scaling_experiment/20_percent_hbm/value_size_32 padding_host_32.cpp load_balance_1d_array_20_percent_hbm_64_value_size.xclbin" ;;
    64|128|256) echo "$E/value_scaling_experiment/20_percent_hbm/value_size_$1 host_$1.cpp load_balance_1d_array_20_percent_hbm_$1_value_size.xclbin" ;;
    *) echo "unknown value size $1" >&2; return 1 ;;
  esac
}

PAIRS=""                                  # "value:N": value-size sweep at 1M, then the 64 B data-size sweep
for v in $VSIZES; do PAIRS="$PAIRS $v:1000000"; done
for n in $NS; do case " $PAIRS " in *" 64:$n "*) ;; *) PAIRS="$PAIRS 64:$n" ;; esac; done

echo "log: $LOG"
echo "points: ${PAIRS# } | $RUNS runs x $CMDS commands per point (command seeds 1..$RUNS)"
declare -A PATCHED
for p in $PAIRS; do
  v=${p%%:*}; n=${p##*:}
  read dir src xcl <<< "$(cfg $v)" || exit 1
  [ -f "$dir/$xcl" ] || { echo "missing $dir/$xcl"; exit 1; }
  if [ -z "${PATCHED[$dir]:-}" ]; then
    ( cd "$dir" && python3 "$PATCH" "$src" ) || { echo "patch failed for $v B"; exit 1; }
    PATCHED[$dir]=1
  fi
  echo "building $v B, N = $n"
  ( cd "$dir" && g++ "cmd_$src" -o "boltavg_${v}_$n" -DINIT_SIZE=$n -DCMD_SIZE=$CMDS $XRT ) || { echo "build failed: $v B, N = $n"; exit 1; }
done

oncard() { if command -v flock > /dev/null; then flock "$LOCK" "$@"; else "$@"; fi; }
for p in $PAIRS; do
  v=${p%%:*}; n=${p##*:}
  read dir src xcl <<< "$(cfg $v)"
  for s in $(seq 1 $RUNS); do
    out=$(cd "$dir" && oncard env BOLT_CMDS=$CMDS BOLT_SEED=$s "./boltavg_${v}_$n" "$xcl" < /dev/null 2>&1)
    { echo "=== v=$v n=$n cmds=$CMDS seed=$s"; echo "$out"; } >> "$LOG"
    echo "$out" | awk -v h="$(printf '%4s B  N=%-8s seed %3s' $v $n $s)" '
      /^TIMED/ { split($5, k, "="); t = sprintf("%7.1f KQPS", k[2]) }
      /^GET check:/ { g = $3 "/" $6 } /^RESP/ { split($2, a, "="); r = a[2] }
      END { printf "%s  %s  GET wrong %s  acked %s\n", h, (t == "" ? "FAILED (see log)" : t), g, r }'
  done
done

cat > "$OUT/.bolt_avg_summary.py" <<'PYEOF'
import math, re, statistics as st, sys
from collections import defaultdict
log, csv_path = sys.argv[1], sys.argv[2]
# single 20,000-command runs (seed 1) from the earlier sweep, KQPS
ONE_VS = {8: 176.0, 16: 171.6, 32: 144.6, 64: 144.2, 128: 116.9, 256: 82.4}               # N = 1M
ONE_DS = {100000: 151.6, 500000: 145.3, 1000000: 144.2, 2000000: 144.1}                    # 64 B
runs, cur = [], None
kv = lambda s: dict(x.split("=", 1) for x in s.split() if "=" in x)
for line in open(log, errors="replace"):
    line = line.rstrip("\n")
    m = re.match(r"=== v=(\d+) n=(\d+) cmds=(\d+) seed=(\d+)", line)
    if m:
        cur = dict(v=int(m[1]), n=int(m[2]), cmds=int(m[3]), seed=int(m[4])); runs.append(cur); continue
    if cur is None: continue
    if line.startswith("SEED"): cur["seed_seen"] = int(kv(line)["command_seed"])
    elif line.startswith("TIMED"): cur["s"] = float(kv(line)["seconds"])
    elif line.startswith("RESP"): cur["acked"] = int(line.split()[1].split("=")[1])
    elif line.startswith("GET check:"):
        a = re.match(r"GET check: (\d+) wrong of (\d+)", line); cur["bad"], cur["gets"] = int(a[1]), int(a[2])
    elif line.startswith("RESIDENCY"):
        b, a = line.split("before:")[1].split("after:")
        bb = {k: int(x) for k, x in (y.split("=") for y in b.split())}
        aa = {k: int(x) for k, x in (y.split("=") for y in a.split() if y[0].isdigit())}
        cur["hbm0"] = bb.get("0", 0) / max(1, sum(bb.values())); cur["hbm1"] = aa.get("0", 0) / max(1, sum(bb.values()))
        cur["missing"] = int(kv(" ".join(y for y in a.split() if not y[0].isdigit()))["missing_keys"])
    elif line.startswith("PAGES"): cur["over"] = int(kv(line)["pages_over_capacity"])
good = [r for r in runs if "s" in r and "bad" in r and "hbm1" in r and r.get("seed_seen") == r["seed"]]
with open(csv_path, "w") as f:
    f.write("value_b,entries,commands,seed,seconds,us_per_query,kqps,gets,get_wrong,acked,hbm_share_start,hbm_share_end,"
            "missing_keys,pages_over_capacity\n")
    for r in good:
        f.write(",".join(str(x) for x in (r["v"], r["n"], r["cmds"], r["seed"], r["s"], r["s"] / r["cmds"] * 1e6,
                r["cmds"] / r["s"] / 1e3, r["gets"], r["bad"], r.get("acked", ""), round(r["hbm0"], 4),
                round(r["hbm1"], 4), r["missing"], r.get("over", ""))) + "\n")
print(f"Runs: {len(good)} complete, {len(runs) - len(good)} incomplete" + (" (see the log)" if len(runs) > len(good) else ""))
g = defaultdict(list)
for r in good: g[(r["v"], r["n"])].append(r)

def table(title, keys, one, keyof):
    print(f"\n{title}")
    print(f"{'value':>6} {'entries':>9} {'runs':>4} {'commands':>9} {'us/query':>9} {'+-95%':>6} {'KQPS':>7} {'run KQPS min..max':>18} "
          f"{'1 run (seed 1)':>14} {'GET wrong':>14} {'acked':>8} {'HBM at end':>10} {'missing':>7} {'pages>8':>7}")
    for k in keys:
        rs = g[k]; T = sum(r["s"] for r in rs); C = sum(r["cmds"] for r in rs)
        us = [r["s"] / r["cmds"] * 1e6 for r in rs]; kq = [r["cmds"] / r["s"] / 1e3 for r in rs]
        ci = 1.96 * st.stdev(us) / math.sqrt(len(us)) if len(us) > 1 else 0.0
        acked = "all" if all(r.get("acked") == r["cmds"] for r in rs) else "NO"
        o = one.get(keyof(k))
        print(f"{k[0]:>5}B {k[1]:>9} {len(rs):>4} {C:>9} {T / C * 1e6:9.3f} {ci:6.3f} {C / T / 1e3:7.1f} "
              f"{f'{min(kq):.1f}..{max(kq):.1f}':>18} {(f'{o:.1f}' if o else '-'):>14} "
              f"{str(sum(r['bad'] for r in rs)) + '/' + str(sum(r['gets'] for r in rs)):>14} {acked:>8} "
              f"{100 * st.mean(r['hbm1'] for r in rs):9.1f}% {max(r['missing'] for r in rs):>7} {max(r.get('over', 0) for r in rs):>7}")
table("1. Value-size sweep, 1M entries", sorted(k for k in g if k[1] == 1000000), ONE_VS, lambda k: k[0])
table("2. Data-size sweep, 64 B values", sorted(k for k in g if k[0] == 64), ONE_DS, lambda k: k[1])
print("\nEach point: its runs together (seeds 1..runs, each a fresh initialization). us/query = total kernel time / total")
print("commands; +-95% = 1.96 x sd / sqrt(runs) of the per-run time per query; KQPS = total commands / total time.")
print("1 run (seed 1): the earlier single 20,000-command run. GET wrong: replies that differ from the latest value")
print("written to the key, over all runs; acked: every command of every run answered with ACK; HBM at end: share of")
print("keys in HBM after a run (20% at the start), mean over runs; missing / pages>8: worst run.")
PYEOF
python3 "$OUT/.bolt_avg_summary.py" "$LOG" "$CSV" | tee "$SUM"
echo "log: $LOG   csv: $CSV   summary: $SUM"
