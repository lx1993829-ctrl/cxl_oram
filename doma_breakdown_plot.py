#!/usr/bin/env python3
"""doma_breakdown_plot.py -- per-access cycle breakdown of DOMA (BOLT Figure 8 style).

    python3 doma_breakdown_plot.py                     # the measured values below -> doma_breakdown.pdf / .png
    python3 doma_breakdown_plot.py mix_trace.log       # take the values from a host_phases log instead

Everything you may want to change is in the SETTINGS block. Needs matplotlib (pip install matplotlib).
"""
import re
import sys

# =========================== SETTINGS: edit freely ===========================

# Measured on the card (traced 4 KB bitstream, Z = 8, lambda = 6, 300 MHz, 20% HBM): 500,000 random accesses,
# 50% GET / 50% PUT, uniform keys over 500,000 records. Mean cycles per access; the 12 phases add up to E2E.
DATA = {
    "handoff": 134.0, "lookup": 75.1, "read gate": 192.9, "bucket read": 1419.3, "decrypt/verify": 1275.5,
    "operation": 222.7, "eviction": 5.3, "re-encrypt": 2221.7, "write-back wait": 1075.0, "bucket write": 1023.8,
    "release gate": 221.0, "result": 131.0,
}
E2E = 7997.3

# Bars, left to right: (label, phases summed into the bar, colour). Every phase must appear in exactly one bar.
GROUPS = [
    ("Lookup",          ["handoff", "lookup", "read gate"],                     "#dbe9f6"),
    ("Bucket\naccess",  ["bucket read", "bucket write"],                        "#4a7fb5"),
    ("Crypto",          ["decrypt/verify", "re-encrypt"],                        "#b9b3dc"),
    ("Padding",         ["write-back wait"],                                     "#d9d9d9"),
    ("Others",          ["operation", "eviction", "release gate", "result"],     "#a8d5a2"),
]

# Legend entries: (colour, text). Set LEGEND to an empty list for no legend.
LEGEND = []

TITLE = "DOMA (4 KB, Z = 8, λ = 6)\n500K random accesses, 50% read / 50% write"   # "\n" breaks the line; "" for no title
YLABEL = "Cycles"
OUT = "doma_breakdown"            # writes OUT.pdf and OUT.png
FIGSIZE = (5.2, 2.6)              # inches (width, height)
LOG_SCALE = True
YLIM = (1, 3e4)                   # y-axis range (used with LOG_SCALE; ignored otherwise)
FONT = 9                          # base font size
TICK_FONT = 8                     # bar labels
VALUE_FONT = 8                    # numbers on the bars
LEGEND_FONT = 7
BAR_WIDTH = 0.65
EDGE = ("black", 0.6)             # bar edge colour and width

# ============================================================================


def from_log(path):
    """The last 'phase K: trace, mean cycles (sum = E2E): ...' line of a host_phases log."""
    pat = re.compile(r"phase \d+: trace, mean cycles \(sum = E2E\): (.*?) = E2E ([0-9.]+)")
    found = None
    for line in open(path, errors="replace"):
        m = pat.search(line)
        if m:
            found = m
    if not found:
        sys.exit(f"{path}: no 'phase K: trace, ...' line (run host_phases with DOMA_PHASE_TRACE=1 on a TRACE=1 bitstream)")
    vals = dict((k.strip(), float(v)) for k, v in (kv.rsplit(" ", 1) for kv in found.group(1).split(", ")))
    return vals, float(found.group(2))


def main():
    data, e2e = (from_log(sys.argv[1]) if len(sys.argv) > 1 else (DATA, E2E))
    used = [p for _, members, _ in GROUPS for p in members]
    missing, unknown = set(data) - set(used), set(used) - set(data)
    if missing or unknown or len(used) != len(set(used)):
        sys.exit(f"GROUPS must use every phase exactly once (missing {sorted(missing)}, unknown {sorted(unknown)})")
    labels = [g[0] for g in GROUPS]
    values = [int(sum(data[p] for p in members)) for _, members, _ in GROUPS]   # integer part
    colors = [g[2] for g in GROUPS]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    plt.rcParams.update({"font.size": FONT, "pdf.fonttype": 42, "ps.fonttype": 42})
    fig, ax = plt.subplots(figsize=FIGSIZE)
    x = range(len(values))
    ax.bar(x, values, color=colors, edgecolor=EDGE[0], linewidth=EDGE[1], width=BAR_WIDTH, zorder=3)
    if LOG_SCALE:
        ax.set_yscale("log")
        ax.set_ylim(*YLIM)
    ax.set_ylabel(YLABEL)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, fontsize=TICK_FONT)
    ax.grid(axis="y", which="major", linewidth=0.4, color="#cccccc", zorder=0)
    for i, v in enumerate(values):
        ax.text(i, v * (1.15 if LOG_SCALE else 1.0) + (0 if LOG_SCALE else max(values) * 0.01), f"{v:,}",
                ha="center", va="bottom", fontsize=VALUE_FONT)
    if LEGEND:
        ax.legend(handles=[Patch(facecolor=c, edgecolor=EDGE[0], linewidth=EDGE[1], label=t) for c, t in LEGEND],
                  loc="upper left", ncol=len(LEGEND), fontsize=LEGEND_FONT, frameon=False,
                  handlelength=1.2, columnspacing=0.9)
    if TITLE:
        ax.set_title(TITLE, fontsize=FONT, fontweight="bold")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}.{ext}", dpi=200)
    print(f"bars: {dict(zip([l.replace(chr(10), ' ') for l in labels], values))}; sum {sum(values):,}, E2E {int(e2e):,}")
    print(f"wrote {OUT}.pdf and {OUT}.png")


if __name__ == "__main__":
    main()
