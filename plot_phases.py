#!/usr/bin/env python3
"""plot_phases.py -- per-access cycle breakdown from host_phases' phase trace (DOMA_PHASE_TRACE=1, a TRACE=1 bitstream).

    python3 plot_phases.py mix_trace.log                 # the last phase in the log -> phase_breakdown.pdf / .png
    python3 plot_phases.py mix_trace.log --phase 1 --out fig_breakdown --title "4 KB, Z = 8, lambda = 6"

Reads the line
    phase K: trace, mean cycles (sum = E2E): handoff ..., lookup ..., ..., result ... = E2E ...
and draws one bar per phase (log-scale cycles, the value on each bar), in the style of BOLT's Figure 8.
"""
import argparse
import re
import sys

PHASES = ["handoff", "lookup", "read gate", "bucket read", "decrypt/verify", "operation", "eviction",
          "re-encrypt", "write-back wait", "bucket write", "release gate", "result"]
LABELS = ["Handoff", "Lookup", "Read\ngate", "Bucket\nread", "Decrypt\n+verify", "Block\nop.", "Eviction",
          "Re-\nencrypt", "WB\nwait", "Bucket\nwrite", "Release\ngate", "Result"]
# memory transfers blue, crypto purple, stash/operation green, padding grey, interface light
COLORS = ["#dbe9f6", "#dbe9f6", "#d9d9d9", "#4a7fb5", "#b9b3dc", "#a8d5a2", "#a8d5a2",
          "#b9b3dc", "#d9d9d9", "#4a7fb5", "#d9d9d9", "#dbe9f6"]


def parse(path, want):
    rows = {}
    pat = re.compile(r"^phase (\d+): trace, mean cycles \(sum = E2E\): (.*?) = E2E ([0-9.]+)")
    for line in open(path, errors="replace"):
        m = pat.match(line.strip())
        if not m:
            continue
        vals = dict((k.strip(), float(v)) for k, v in (kv.rsplit(" ", 1) for kv in m.group(2).split(", ")))
        rows[int(m.group(1))] = (vals, float(m.group(3)))
    if not rows:
        sys.exit(f"{path}: no 'phase K: trace, ...' line (was it run with DOMA_PHASE_TRACE=1 on a TRACE=1 bitstream?)")
    k = want if want is not None else max(rows)
    if k not in rows:
        sys.exit(f"{path}: no trace line for phase {k} (have {sorted(rows)})")
    vals, e2e = rows[k]
    missing = [p for p in PHASES if p not in vals]
    if missing:
        sys.exit(f"{path}: phase {k} lacks {missing}")
    return [vals[p] for p in PHASES], e2e, k


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--phase", type=int, help="which host_phases phase (default: the last one in the log)")
    ap.add_argument("--out", default="phase_breakdown", help="output name without extension")
    ap.add_argument("--title", default="", help="title above the plot")
    a = ap.parse_args()
    v, e2e, k = parse(a.log, a.phase)
    s = sum(v)
    if abs(s - e2e) > 0.5:
        print(f"note: phases add up to {s:.1f}, E2E is {e2e:.1f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 9, "pdf.fonttype": 42, "ps.fonttype": 42})
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
    x = range(len(v))
    ax.bar(x, v, color=COLORS, edgecolor="black", linewidth=0.6, width=0.7, zorder=3)
    ax.set_yscale("log")
    ax.set_ylim(1, 3 * 10 ** 4)
    ax.set_ylabel("Cycles")
    ax.set_xticks(list(x))
    ax.set_xticklabels(LABELS, fontsize=7.5)
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.4, color="#cccccc", zorder=0)
    for i, y in enumerate(v):
        ax.text(i, y * 1.15, f"{y:,.0f}" if y >= 10 else f"{y:.1f}", ha="center", va="bottom", fontsize=7.5)
    from matplotlib.patches import Patch
    groups = [("#4a7fb5", "Bucket transfer"), ("#b9b3dc", "Crypto"), ("#a8d5a2", "Stash/operation"),
              ("#d9d9d9", "Padding/scheduling"), ("#dbe9f6", "Request/response")]
    ax.legend(handles=[Patch(facecolor=c, edgecolor="black", linewidth=0.6, label=l) for c, l in groups],
              loc="upper left", ncol=5, fontsize=6.5, frameon=False, handlelength=1.2, columnspacing=1.0,
              bbox_to_anchor=(0.0, 1.0))
    ax.set_title(a.title or f"Per-access cycle breakdown (E2E {e2e:,.0f} cycles)", fontsize=9, fontweight="bold")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{a.out}.{ext}", dpi=200)
    print(f"phase {k}: E2E {e2e:.1f} cycles; wrote {a.out}.pdf and {a.out}.png")


if __name__ == "__main__":
    main()
