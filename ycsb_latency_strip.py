#!/usr/bin/env python3
"""ycsb_latency_strip.py -- per-request latency of each YCSB workload as a strip plot (one row per workload).

    python3 ycsb_latency_strip.py ycsb_hist_p1.csv ycsb_hist_p2.csv

Input: the histograms host_ycsb writes with DOMA_LAT_HIST=<file> (rows: workload,op,latency_cycles,count), one file
per pass. GETs and PUTs are pooled, and so are the passes. Each row shows the workload's whole distribution:
  - the body, up to the TAIL_Q quantile, as BODY_DOTS evenly spaced quantiles (dot density follows the
    distribution, the same in every row);
  - every request beyond the TAIL_Q quantile as its own dot, so outliers appear exactly as they occurred.
Dots are spread vertically at random within the row so overlapping ones stay visible.
Writes ycsb_latency_strip.pdf / .png and prints each workload's percentiles.
"""
import csv
import random
import sys
from collections import defaultdict

# =========================== SETTINGS: edit freely ===========================
ROWS = [("A", "A: 50/50 read/update"), ("B", "B: 95/5 read/update"), ("C", "C: 100 read"),
        ("D", "D: 95/5 read/insert"), ("E", "E: 95/5 scan/insert"), ("F", "F: 50/50 read/RMW")]
BODY_DOTS = 400                    # quantile dots per workload for the body
TAIL_Q = 0.999                     # every request beyond this quantile is drawn as its own dot
JITTER = 0.28                      # vertical spread within a row (row spacing is 1)
DOT = 5                            # dot area (points^2)
COLOR = "#2a78d6"
XLABEL = "latency per request (cycles)"
XLIM = None                        # e.g. (4900, 5600); None: from the data
FIGSIZE = (3.5, 2.2)               # inches: one column
OUT = "ycsb_latency_strip"
TEXT = "#000000"
LABEL_FONT, TICK_FONT = 8, 7
SEED = 1                           # fixed: the same figure every time
# ============================================================================


def load(paths):
    hist = defaultdict(lambda: defaultdict(int))        # workload -> {latency: count}, GET and PUT pooled
    for p in paths:
        with open(p) as f:
            for r in csv.DictReader(f):
                hist[r["workload"]][int(r["latency_cycles"])] += int(r["count"])
    return hist


def quantile(items, n, q):
    """items: sorted (latency, count); the q-quantile of n requests."""
    target = q * (n - 1); acc = 0
    for x, c in items:
        acc += c
        if acc - 1 >= target:
            return x
    return items[-1][0]


def dots(h):
    items = sorted(h.items()); n = sum(c for _, c in items)
    body = [quantile(items, n, TAIL_Q * (i + 0.5) / BODY_DOTS) for i in range(BODY_DOTS)]   # the body up to TAIL_Q
    cut = quantile(items, n, TAIL_Q)
    tail = [x for x, c in items if x > cut for _ in range(c)]
    return body, tail, items, n


def _font():
    import os
    from matplotlib import font_manager
    for d in [os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts"), "/mnt/c/Windows/Fonts",
              "/usr/share/fonts/truetype/msttcorefonts", "/Library/Fonts", "/System/Library/Fonts/Supplemental"]:
        for name in ("arial.ttf", "Arial.ttf"):
            p = os.path.join(d, name)
            if os.path.isfile(p):
                try:
                    font_manager.fontManager.addfont(p)
                except Exception:
                    pass
    for fam in ("Arial", "Liberation Sans"):
        try:
            font_manager.findfont(font_manager.FontProperties(family=fam), fallback_to_default=False)
            return fam
        except ValueError:
            continue
    return "DejaVu Sans"


def main(paths):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": [_font()], "font.size": LABEL_FONT,
                         "axes.labelsize": LABEL_FONT, "xtick.labelsize": TICK_FONT, "ytick.labelsize": TICK_FONT,
                         "text.color": TEXT, "axes.labelcolor": TEXT, "xtick.labelcolor": TEXT,
                         "ytick.labelcolor": TEXT, "pdf.fonttype": 42, "ps.fonttype": 42})
    hist = load(paths)
    rows = [(w, label) for w, label in ROWS if w in hist]
    rnd = random.Random(SEED)
    fig, ax = plt.subplots(figsize=FIGSIZE)
    lo, hi = None, None
    print(f"per-request latency, cycles (GET and PUT pooled; passes pooled: {', '.join(paths)})")
    print(f"{'':4}{'n':>9}{'min':>7}{'p1':>7}{'p50':>7}{'p99':>7}{'p99.9':>8}{'p99.99':>8}{'max':>7}  dots beyond p{100 * TAIL_Q:g}")
    for i, (w, label) in enumerate(rows):
        body, tail, items, n = dots(hist[w])
        y = len(rows) - 1 - i                                   # A at the top
        xs = body + tail
        ax.scatter(xs, [y + rnd.uniform(-JITTER, JITTER) for _ in xs], s=DOT, color=COLOR, alpha=0.55,
                   linewidths=0, zorder=3)
        q = lambda f: quantile(items, n, f)
        print(f"{w:4}{n:>9}{items[0][0]:>7}{q(0.01):>7}{q(0.5):>7}{q(0.99):>7}{q(0.999):>8}{q(0.9999):>8}"
              f"{items[-1][0]:>7}  {len(tail)}")
        lo = items[0][0] if lo is None else min(lo, items[0][0])
        hi = items[-1][0] if hi is None else max(hi, items[-1][0])
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([label for _, label in reversed(rows)])
    ax.set_ylim(-0.6, len(rows) - 0.4)
    ax.set_xlim(*(XLIM or (lo - 0.02 * (hi - lo), hi + 0.02 * (hi - lo))))
    ax.set_xlabel(XLABEL)
    ax.grid(True, axis="x", color="#cccccc", lw=0.4, zorder=0)
    for y in range(len(rows) - 1):
        ax.axhline(y + 0.5, color="#e5e5e5", lw=0.4, zorder=0)
    ax.tick_params(axis="y", length=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    fig.tight_layout(pad=0.2)
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}.{ext}", dpi=300)
    plt.close(fig)
    print(f"wrote {OUT}.pdf and {OUT}.png")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
