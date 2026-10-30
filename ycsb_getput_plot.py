#!/usr/bin/env python3
"""ycsb_getput_plot.py -- per-request latency of GETs and PUTs in each YCSB workload, one column per operation.

    python3 ycsb_getput_plot.py strip_by_op.txt        # -> ycsb_getput.pdf / .png

Input: the strip block of `ycsb_latency_analyze.py --by-op` (lines "A/GET n=.. cut=.. max=..", "A/GET body ...",
"A/GET tail ..."). Each column shows its operation's whole distribution:
  - a bar from p1 to p99 (interpolated from the quantiles);
  - the body: its quantile dots (100 up to the 99.9th percentile; each stands for 1% of that column's requests),
    spread sideways at random so overlapping dots stay visible;
  - hollow dots: the slowest OUTLIER_FRAC of the column's requests, one per request (a column with more than
    PER_COL requests is sampled down to that many; the slowest request is always drawn, at its exact latency).
The y axis is broken: the lower panel shows the body in detail, the upper one the slow requests.
"""
import random
import sys

# =========================== SETTINGS: edit freely ===========================
WORKLOADS = ["A", "B", "C", "D", "E", "F"]
COLORS = {"GET": "#2a78d6", "PUT": "#eb6834"}
OUTLIER_FRAC = 1e-4               # the slowest 0.01%
PER_COL = 1_000_000               # outliers are drawn for at most this many requests per column
BIN = 10                          # the tail's bin width in the input
LOW = (4730, 5160)                # lower panel: the body
HIGH = (5160, 7450)               # upper panel: the slow requests
HEIGHT_RATIO = (1.0, 1.3)         # upper : lower panel height
COL_GAP = 0.36                    # GET and PUT columns sit at workload -/+ COL_GAP / 2
JITTER = 0.11
DOT = 3.5
YLABEL = "latency per request (cycles)"
FIGSIZE = (3.5, 2.6)
OUT = "ycsb_getput"
TEXT = "#000000"
LABEL_FONT, TICK_FONT, SMALL_FONT = 8, 7, 6.5
SEED = 1
# ============================================================================


def parse(path):
    data = {}
    for line in open(path):
        p = line.split()
        if len(p) < 2 or "/" not in p[0]:
            continue
        w, op = p[0].split("/")
        d = data.setdefault((w, op), {"body": [], "tail": []})
        if p[1].startswith("n="):
            for kv in p[1:]:
                k, v = kv.split("=")
                d[k] = int(v)
        elif p[1] in ("body", "tail"):
            for tok in p[2:]:
                x, _, r = tok.partition("*")
                d[p[1]].extend([int(x)] * (int(r) if r else 1))
    return data


def interp(body, q, tail_q=0.999):
    """the q-quantile from body quantiles taken at tail_q * (i + 0.5) / len(body)"""
    n = len(body)
    pos = q / tail_q * n - 0.5
    i = max(0, min(n - 2, int(pos)))
    f = min(1.0, max(0.0, pos - i))
    return body[i] + f * (body[i + 1] - body[i])


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


def main(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": [_font()], "font.size": LABEL_FONT,
                         "axes.labelsize": LABEL_FONT, "xtick.labelsize": TICK_FONT, "ytick.labelsize": TICK_FONT,
                         "text.color": TEXT, "axes.labelcolor": TEXT, "xtick.labelcolor": TEXT,
                         "ytick.labelcolor": TEXT, "pdf.fonttype": 42, "ps.fonttype": 42})
    data = parse(path)
    wl = [w for w in WORKLOADS if (w, "GET") in data or (w, "PUT") in data]
    rnd = random.Random(SEED)
    fig, (top, bot) = plt.subplots(2, 1, sharex=True, figsize=FIGSIZE,
                                   gridspec_kw={"height_ratios": HEIGHT_RATIO, "hspace": 0.06})
    for xi, w in enumerate(wl):
        for op, dx in (("GET", -COL_GAP / 2), ("PUT", COL_GAP / 2)):
            if (w, op) not in data:
                continue
            d, x0, col = data[(w, op)], xi + dx, COLORS[op]
            lo, hi = interp(d["body"], 0.01), interp(d["body"], 0.99)
            for ax in (top, bot):
                ax.plot([x0, x0], [lo, hi], color="#555555", lw=0.8, zorder=2)
                ax.plot([x0 - 0.06, x0 + 0.06], [lo, lo], color="#555555", lw=0.8, zorder=2)
                ax.plot([x0 - 0.06, x0 + 0.06], [hi, hi], color="#555555", lw=0.8, zorder=2)
                ax.scatter([x0 + rnd.uniform(-JITTER, JITTER) for _ in d["body"]], d["body"], s=DOT, color=col,
                           alpha=0.55, linewidths=0, zorder=3)
            tail = sorted(d["tail"])
            k = round(d["n"] * OUTLIER_FRAC)
            slow = tail[-(k - 1):] if k > 1 else []
            want = round(PER_COL * OUTLIER_FRAC) - 1
            if len(slow) > want:
                slow = rnd.sample(slow, want)
            ys = [y + rnd.uniform(0, BIN) for y in slow] + [d["max"]]
            for ax in (top, bot):
                ax.scatter([x0 + rnd.uniform(-JITTER, JITTER) for _ in ys], ys, s=DOT + 2, facecolors="none",
                           edgecolors=col, linewidths=0.6, zorder=4)
    bot.set_ylim(*LOW)
    top.set_ylim(*HIGH)
    top.spines["bottom"].set_visible(False)
    bot.spines["top"].set_visible(False)
    top.tick_params(axis="x", length=0)
    for ax in (top, bot):
        ax.spines["right"].set_visible(False)
        ax.grid(True, axis="y", color="#e2e2e2", lw=0.4, zorder=0)
    top.spines["top"].set_visible(False)
    kw = dict(color="k", clip_on=False, lw=0.6)                    # break marks on the y axis
    top.plot([-0.012, 0.012], [-0.02, 0.02], transform=top.transAxes, **kw)
    bot.plot([-0.012, 0.012], [0.98, 1.02], transform=bot.transAxes, **kw)
    bot.set_xticks(range(len(wl)))
    bot.set_xticklabels(wl)
    bot.set_xlim(-0.6, len(wl) - 0.4)
    bot.set_xlabel("YCSB workload")
    fig.supylabel(YLABEL, fontsize=LABEL_FONT, x=0.02)
    handles = [Line2D([], [], ls="none", marker="o", ms=3, color=COLORS["GET"], label="GET"),
               Line2D([], [], ls="none", marker="o", ms=3, color=COLORS["PUT"], label="PUT"),
               Line2D([], [], ls="none", marker="o", ms=3.4, mfc="none", mec="#555555", mew=0.6,
                      label="slowest 0.01%"),
               Line2D([], [], color="#555555", lw=0.8, label="p1\u2013p99")]
    fig.legend(handles=handles, loc="upper center", ncol=4, frameon=False, fontsize=SMALL_FONT, handletextpad=0.3,
               columnspacing=0.9, bbox_to_anchor=(0.55, 1.0))
    fig.subplots_adjust(left=0.17, right=0.98, top=0.9, bottom=0.14)
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}.{ext}", dpi=300)
    plt.close(fig)
    print(f"wrote {OUT}.pdf and {OUT}.png")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
