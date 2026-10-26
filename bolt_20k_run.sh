#!/bin/bash
# bolt_20k_run.sh -- BOLT's released 20%-HBM kernels on the U55C, 20,000 commands per run (the same count as the
# non-oblivious baseline), instead of the 5,000 in BOLT's own host programs.
#
#   value-size sweep: 8, 16, 32, 64, 128, 256 B at N = 1M entries
#   data-size sweep:  N = 100K, 500K, 1M, 2M, 5M, 10M at 64 B
#
# Each run is a fresh process: BOLT's own initialization (20% of entries in HBM, 80% in host pages), then one timed
# kernel call over the batch from BOLT's generator (uniform keys, each command a PUT with probability 0.5, seed 1;
# its first 5,000 commands are exactly the earlier 5,000-command batch). Host side only: bolt_cmds_patch.py writes
# cmd_*.cpp copies; BOLT's kernels (.xclbin) and host programs are not changed. One run at a time on the card
# (same lock file as baseline_all.sh).
#
# Usage:  bash bolt_20k_run.sh                         # about 10-20 minutes
#         CMDS="5000 20000" bash bolt_20k_run.sh        # also rerun the 5,000-command batch, same session
#         VSIZES="8 64" NS="1000000" REPS=3 bash bolt_20k_run.sh
# Needs:  ~/bolt_cmds_patch.py, BOLT's repo at ~/BOLT-benchmark (or BOLT_DIR=/path/to/BOLT-benchmark/BOLT)
# Output: ~/bolt_20k_<date>.txt (full log), .csv (one row per run), .summary.txt (tables)
set -u
CMDS=${CMDS:-20000}
VSIZES=${VSIZES:-"8 16 32 64 128 256"}
NS=${NS:-"100000 500000 1000000 2000000 5000000 10000000"}
REPS=${REPS:-1}
E=${BOLT_DIR:-$HOME/BOLT-benchmark/BOLT}
PATCH=${PATCH:-$HOME/bolt_cmds_patch.py}
LOCK=${LOCK:-/tmp/baseline_card.lock}
XRT=${XRT:-"-I/opt/xilinx/xrt/include -L/opt/xilinx/xrt/lib -lxrt_coreutil -lxrt_core -w -Wl,-rpath,/opt/xilinx/xrt/lib -std=c++17"}
OUT=${OUT:-$HOME}
STAMP=$(date +%m%d_%H%M)
LOG=$OUT/bolt_20k_$STAMP.txt; CSV=$OUT/bolt_20k_$STAMP.csv; SUM=$OUT/bolt_20k_$STAMP.summary.txt
ulimit -s unlimited 2>/dev/null
[ -f /opt/xilinx/xrt/setup.sh ] && source /opt/xilinx/xrt/setup.sh >/dev/null 2>&1
[ -f "$PATCH" ] || { echo "missing $PATCH"; exit 1; }

cfg() {  # value size -> directory, host source, xclbin (BOLT's own pairing; 32 B padded on the 64 B kernel)
  case $1 in
    8)   echo "$E/hbm_distribution_experiment/20_percent_hbm host.cpp load_balance_1d_array_20_percent_hbm_8_value_size.xclbin" ;;
    16)  echo "$E/value_scaling_experiment/20_percent_hbm/value_size_16 host_16.cpp load_balance_1d_array_20_percent_hbm_16_value_size.xclbin" ;;
    32)  echo "$E/value_scaling_experiment/20_percent_hbm/value_size_32 padding_host_32.cpp load_balance_1d_array_20_percent_hbm_64_value_size.xclbin" ;;
    64|128|256) echo "$E/value_scaling_experiment/20_percent_hbm/value_size_$1 host_$1.cpp load_balance_1d_array_20_percent_hbm_$1_value_size.xclbin" ;;
    *) echo "unknown value size $1" >&2; return 1 ;;
  esac
}

# runs: "value N" pairs, value-size sweep first, then the data-size sweep at 64 B (1M at 64 B only once)
PAIRS=""
for v in $VSIZES; do PAIRS="$PAIRS $v:1000000"; done
for n in $NS; do case " $PAIRS " in *" 64:$n "*) ;; *) PAIRS="$PAIRS 64:$n" ;; esac; done
MAXC=$(printf '%s\n' $CMDS | sort -n | tail -1)

echo "log: $LOG"
echo "runs: ${PAIRS# } | commands: $CMDS | repetitions: $REPS"
# patch each folder's program once, build one binary per (value, N)
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
  ( cd "$dir" && g++ "cmd_$src" -o "bolt20k_${v}_$n" -DINIT_SIZE=$n -DCMD_SIZE=$MAXC $XRT ) || { echo "build failed: $v B, N = $n"; exit 1; }
done

oncard() { if command -v flock > /dev/null; then flock "$LOCK" "$@"; else "$@"; fi; }
for rep in $(seq 1 $REPS); do
  for c in $CMDS; do
    for p in $PAIRS; do
      v=${p%%:*}; n=${p##*:}
      read dir src xcl <<< "$(cfg $v)"
      out=$(cd "$dir" && oncard env BOLT_CMDS=$c "./bolt20k_${v}_$n" "$xcl" < /dev/null 2>&1)
      { echo "=== v=$v n=$n cmds=$c rep=$rep"; echo "$out"; } >> "$LOG"
      echo "$out" | awk -v h="$(printf '%4s B  N=%-9s %6s cmds  rep %s' $v $n $c $rep)" '
        /^TIMED/ { split($3, a, "="); split($5, k, "="); t = sprintf("%8.3f ms  %7.1f KQPS", a[2] * 1e3, k[2]) }
        /^GET check:/ { g = $3 "/" $6 } /^RESP/ { r = $2 " of " $4 }
        END { printf "%s  %s  GET wrong %s  %s\n", h, (t == "" ? "FAILED (see log)" : t), g, r }'
    done
  done
done

cat > "$OUT/.bolt_20k_summary.py" <<'PYEOF'
import re, statistics as st, sys
from collections import defaultdict
log, csv_path = sys.argv[1], sys.argv[2]
# earlier 5,000-command runs on this card (tiny_vs_bolt.py): total ms per 5,000 commands
PREV_VS = {8: 28.53, 16: 29.48, 32: 34.97, 64: 34.70, 128: 43.04, 256: 61.09}            # N = 1M
PREV_DS = {100000: 34.4608, 500000: 34.7783, 1000000: 34.7533, 2000000: 34.9687}       # 64 B
runs, cur = [], None
for line in open(log, errors="replace"):
    line = line.rstrip("\n")
    m = re.match(r"=== v=(\d+) n=(\d+) cmds=(\d+) rep=(\d+)", line)
    if m:
        cur = dict(v=int(m[1]), n=int(m[2]), cmds=int(m[3]), rep=int(m[4])); runs.append(cur); continue
    if cur is None:
        continue
    kv = lambda s: dict(x.split("=", 1) for x in s.split() if "=" in x)
    if line.startswith("TIMED"):
        d = kv(line); cur["s"] = float(d["seconds"])
    elif line.startswith("  GET commands:"):
        cur["get"] = int(line.split()[2])
    elif line.startswith("  PUT commands:"):
        cur["put"] = int(line.split()[2])
    elif line.startswith("Insert error count is"):
        cur["ins"] = int(line.split()[-1])
    elif line.startswith("RESP"):
        a = line.split()[1].split("="); cur["acked"] = int(a[1])
    elif line.startswith("GET check:"):
        a = re.match(r"GET check: (\d+) wrong of (\d+) \(on keys initialization could not place: (\d+)\)", line)
        cur["bad"], cur["gets"], cur["bad_ins"] = int(a[1]), int(a[2]), int(a[3])
    elif line.startswith("WORKLOAD"):
        d = kv(line); cur["distinct"] = int(d["distinct_keys"]); cur["raw"] = int(d["gets_after_put_same_key"])
    elif line.startswith("RESIDENCY"):
        b, a = line.split("before:")[1].split("after:")
        cur["before"] = {k: int(v) for k, v in (x.split("=") for x in b.split())}
        rest = a.split()
        cur["after"] = {k: int(v) for k, v in (x.split("=") for x in rest if x[0].isdigit())}
        d = kv(" ".join(x for x in rest if not x[0].isdigit())); cur["missing"] = int(d["missing_keys"])
    elif line.startswith("PAGES"):
        d = kv(line); cur["over"] = int(d["pages_over_capacity"]); cur["maxcnt"] = int(d["max_page_count"])
good = [r for r in runs if "s" in r and "bad" in r and "after" in r]
with open(csv_path, "w") as f:
    f.write("value_b,entries,commands,rep,seconds,us_per_query,kqps,gets,puts,get_wrong,get_wrong_unplaced_keys,acked,"
            "insert_errors,distinct_keys,gets_after_put,hbm_keys_before,host_keys_before,hbm_keys_after,host_keys_after,"
            "evict_keys_after,missing_keys,max_page_count,pages_over_capacity\n")
    for r in good:
        b, a = r["before"], r["after"]
        f.write(",".join(str(x) for x in (r["v"], r["n"], r["cmds"], r["rep"], r["s"], r["s"] / r["cmds"] * 1e6,
                r["cmds"] / r["s"] / 1e3, r.get("get", ""), r.get("put", ""), r["bad"], r["bad_ins"], r.get("acked", ""),
                r.get("ins", ""), r.get("distinct", ""), r.get("raw", ""), b.get("0", 0), b.get("1", 0), a.get("0", 0),
                a.get("1", 0), a.get("2", 0), r["missing"], r.get("maxcnt", ""), r.get("over", ""))) + "\n")
print(f"Runs: {len(good)} complete, {len(runs) - len(good)} incomplete" + (" (see the log)" if len(runs) > len(good) else ""))
g = defaultdict(list)
for r in good:
    g[(r["v"], r["n"], r["cmds"])].append(r)

def table(title, keys, prev):
    print(f"\n{title}")
    print(f"{'value':>6} {'entries':>9} {'cmds':>6} {'runs':>4} {'us/query':>9} {'KQPS':>7} {'5k-run KQPS':>11} "
          f"{'GET/PUT':>11} {'GET wrong':>11} {'acked':>11} {'HBM keys before->after':>23} {'evict':>6} {'missing':>7} {'pages>8':>7}")
    for k in keys:
        rs = g[k]; us = [r["s"] / r["cmds"] * 1e6 for r in rs]; kq = [r["cmds"] / r["s"] / 1e3 for r in rs]
        r0 = rs[0]; n_keys = sum(r0["before"].values())
        hb, ha = r0["before"].get("0", 0), r0["after"].get("0", 0)
        p = prev.get(k[0] if prev is PREV_VS else k[1])
        print(f"{k[0]:>5}B {k[1]:>9} {k[2]:>6} {len(rs):>4} {st.mean(us):9.3f} {st.mean(kq):7.1f} "
              f"{(f'{5000 / p:.1f}' if p else '-'):>11} {str(r0.get('get', '?')) + '/' + str(r0.get('put', '?')):>11} "
              f"{str(sum(r['bad'] for r in rs)) + '/' + str(sum(r['gets'] for r in rs)):>11} "
              f"{str(min(r.get('acked', 0) for r in rs)) + '/' + str(k[2]):>11} "
              f"{f'{100 * hb / n_keys:.1f}% -> {100 * ha / n_keys:.1f}%':>23} {r0['after'].get('2', 0):>6} "
              f"{max(r['missing'] for r in rs):>7} {max(r.get('over', 0) for r in rs):>7}")
vs = sorted(k for k in g if k[1] == 1000000)
ds = sorted((k for k in g if k[0] == 64), key=lambda k: (k[2], k[1]))
table("1. Value-size sweep, 1M entries", sorted(vs, key=lambda k: (k[2], k[0])), PREV_VS)
table("2. Data-size sweep, 64 B values", ds, PREV_DS)
print("\nus/query = timed kernel call / commands; KQPS = commands / time. 5k-run KQPS: the earlier 5,000-command runs.")
print("GET wrong counts replies that differ from the latest value written to the key (PUTs earlier in the batch")
print("included); acked = replies of type ACK (all should be); HBM keys = share of all keys whose position-map entry")
print("is HBM (type 0), at initialization and after the run; evict = keys left in the eviction buffer (type 2);")
print("missing = placed keys no longer in the position map; pages>8 = pages whose tuple count exceeds 8.")
PYEOF
python3 "$OUT/.bolt_20k_summary.py" "$LOG" "$CSV" | tee "$SUM"
echo "log: $LOG   csv: $CSV   summary: $SUM"
