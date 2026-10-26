#!/usr/bin/env python3
"""doma_figures.py -- DOMA's per-access cycle breakdown and memory allocation, side by side (BOLT Figures 8/9 style).

    python3 doma_figures.py          # -> doma_figures.pdf / .png, and each panel separately
                                     #    (doma_breakdown_cycles, doma_breakdown_memory) for subfigures

Both panels use measured values from the card (4 KB blocks, Z = 8, lambda = 7, 300 MHz, 20% of the buckets in HBM,
v9 bitstreams with the boot fix):
  (a) cycles per access, host_phases DOMA_PHASES=mix:1000000 with DOMA_PHASE_TRACE=1 on the traced bitstream
      (1M random accesses, 50% GET / 50% PUT, uniform keys over 1M records);
  (b) memory XRT allocated for 1M records, host_phases DOMA_MEMREPORT=1 on the untraced bitstream.
Everything you may want to change is in the SETTINGS block. Needs matplotlib (pip install matplotlib).
"""

# =========================== SETTINGS: edit freely ===========================

# Colour palette shared by both panels
LIGHT_BLUE, BLUE, PURPLE, GREY, GREEN = "#dbe9f6", "#4a7fb5", "#b9b3dc", "#d9d9d9", "#a8d5a2"

# ---- (a) cycles per access: the 12 measured phases (mean cycles; they add up to E2E) ----
PHASES = {
    # traced 4 KB build, lambda = 7 (doma_4096t_l7_v9), 1M records, 20% HBM, 1M mixed GET/PUT accesses (E2E 7081.1)
    "handoff": 134.0, "lookup": 78.1, "read gate": 189.9, "bucket read": 1419.1, "decrypt/verify": 1279.4,
    "operation": 224.1, "eviction": 5.3, "re-encrypt": 2221.4, "write-back wait": 180.3, "bucket write": 1023.8,
    "release gate": 194.6, "result": 131.0,
}
# Bars: (label, phases summed into the bar, colour). Every phase must appear in exactly one bar.
PERF_BARS = [
    ("Lookup",          ["handoff", "lookup", "read gate"],                 LIGHT_BLUE),
    ("Bucket\naccess",  ["bucket read", "bucket write"],                    BLUE),
    ("Crypto",          ["decrypt/verify", "re-encrypt"],                   PURPLE),
    ("Padding",         ["write-back wait"],                                GREY),
    ("Others",          ["operation", "eviction", "release gate", "result"], GREEN),
]
PERF_TITLE = "a. Cycles per access"
PERF_YLABEL = "Cycles"
PERF_XLABEL = "Phases"

# ---- (b) memory allocated for 1M records (MiB, as reported by XRT) ----
MEM_BARS = [
    ("$MAP_{pos}$",      6.10,    LIGHT_BLUE),   # position map (HBM)       -- lambda = 7, untraced 4 KB (doma_4096_l7_v9)
    ("Bucket\nrecords",  8.72,    PURPLE),       # per-bucket GCM records (HBM)
    ("Stash",            967.02,  GREEN),        # stash pool (HBM)
    ("$B_{hbm}$",        892.84,  BLUE),         # buckets in HBM
    ("$B_{host}$",       3571.47, BLUE),         # buckets in host memory
]
MEM_TITLE = "b. Memory allocation (1M records)"
MEM_YLABEL = "Memory (MiB)"
MEM_XLABEL = "Objects"

# ---- common ----
INTEGER = True                    # show each value rounded to an integer (False: one decimal)
OUT = "doma_figures"              # writes OUT.pdf and OUT.png
FIGSIZE = (7.0, 2.5)              # inches (width, height) for the whole figure
LOG_SCALE = True
YLIM = (1, 1e4)                   # same range in both panels
FONT = 9
TICK_FONT = 8
VALUE_FONT = 8
TITLE_FONT = 8.5
BAR_WIDTH = 0.7
EDGE = ("black", 0.6)

# ---- separate panels, for two subfigures (written in addition to the combined figure) ----
SEPARATE = True
SEP_FIGSIZE = (3.4, 2.4)          # inches per panel: two fit side by side across a 7-inch text width
SEP_OUT = {"perf": "doma_breakdown_cycles", "mem": "doma_breakdown_memory"}   # -> .pdf and .png each
SEP_TITLES = False                # no title in a separate panel: the subfigure caption names it

# ============================================================================


def perf_values():
    used = [p for _, members, _ in PERF_BARS for p in members]
    missing, unknown = set(PHASES) - set(used), set(used) - set(PHASES)
    if missing or unknown or len(used) != len(set(used)):
        raise SystemExit(f"PERF_BARS must use every phase exactly once (missing {sorted(missing)}, unknown {sorted(unknown)})")
    return [(label, sum(PHASES[p] for p in members), colour) for label, members, colour in PERF_BARS]


def draw(ax, bars, title, ylabel, xlabel):
    labels = [b[0] for b in bars]
    values = [b[1] for b in bars]
    x = range(len(bars))
    ax.bar(x, values, color=[b[2] for b in bars], edgecolor=EDGE[0], linewidth=EDGE[1], width=BAR_WIDTH, zorder=3)
    if LOG_SCALE:
        ax.set_yscale("log")
        ax.set_ylim(*YLIM)
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, fontsize=TICK_FONT)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(axis="y", which="major", linewidth=0.4, color="#cccccc", zorder=0)
    if title:
        ax.set_title(title, fontsize=TITLE_FONT, fontweight="bold")
    for j, v in enumerate(values):
        text = f"{round(v):,}" if INTEGER else f"{v:,.1f}"
        ax.text(j, v * (1.15 if LOG_SCALE else 1.0), text, ha="center", va="bottom", fontsize=VALUE_FONT)


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": FONT, "pdf.fonttype": 42, "ps.fonttype": 42})
    fig, (a, b) = plt.subplots(1, 2, figsize=FIGSIZE)
    perf = perf_values()
    draw(a, perf, PERF_TITLE, PERF_YLABEL, PERF_XLABEL)
    draw(b, MEM_BARS, MEM_TITLE, MEM_YLABEL, MEM_XLABEL)
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}.{ext}", dpi=200)
    print("(a) " + ", ".join(f"{l.replace(chr(10), ' ')} {round(v):,}" for l, v, _ in perf)
          + f"; sum {sum(round(v) for _, v, _ in perf):,} cycles")
    print("(b) " + ", ".join(f"{l} {round(v):,}" for l, v, _ in MEM_BARS) + " MiB")
    print(f"wrote {OUT}.pdf and {OUT}.png")
    if SEPARATE:
        for key, bars, title, ylabel, xlabel in (("perf", perf, PERF_TITLE, PERF_YLABEL, PERF_XLABEL),
                                                 ("mem", MEM_BARS, MEM_TITLE, MEM_YLABEL, MEM_XLABEL)):
            f, ax = plt.subplots(figsize=SEP_FIGSIZE)
            draw(ax, bars, title.split(". ", 1)[-1] if SEP_TITLES else None, ylabel, xlabel)
            f.tight_layout()
            for ext in ("pdf", "png"):
                f.savefig(f"{SEP_OUT[key]}.{ext}", dpi=200)
            print(f"wrote {SEP_OUT[key]}.pdf and {SEP_OUT[key]}.png")


if __name__ == "__main__":
    main()
