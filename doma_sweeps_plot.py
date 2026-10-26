#!/usr/bin/env python3
"""
doma_sweeps_plot.py -- time per request (1,000 / KQPS) for the sweeps, two panels for two subfigures in one column:
  (a) throughput against the number of records N, 64 B values (and DOMA 4 KB) -> doma_sweep_n.pdf / .png
  (b) throughput against the value size, N = 1M records         -> doma_sweep_value.pdf / .png
Each panel: DOMA at 300 and 230 MHz, Tiny ORAM at 230 MHz (its clock), BOLT at 300 MHz; one shared
legend strip (doma_sweep_legend.pdf) goes above both panels.

Sources (all measured on the same U55C card; 20% of the buckets in HBM for DOMA, 20,000 commands per run):
  DOMA:      doma_sweeps_l7.csv (lambda = 7, Z = 8, v9 bitstreams), columns sweep, mhz, value_b, n, kqps.
  BOLT, Tiny ORAM: sweeps_bolt_tiny.csv (20,000 requests per point; BOLT at 300 MHz with its own benchmark, Tiny ORAM
             at 230 MHz through host_doma with the doma:20 layout).
Run:  python3 doma_sweeps_plot.py   (needs matplotlib; reads doma_sweeps_l7.csv from this folder)
"""
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FixedFormatter, NullLocator

# ============================== SETTINGS ==============================
CSV = "doma_sweeps_l7.csv"
OTHERS = "sweeps_bolt_tiny.csv"     # BOLT and Tiny ORAM: design, mhz, sweep (n | vs), value_b, n, requests, kqps
N_MAX_PLOT = 2000000              # panel (a) shows records up to 2M (the CSV also has BOLT at 5M and 10M)

FIGSIZE = (1.72, 1.85)             # inches per panel: two side by side in one 3.5-inch column
FONT, TICK_FONT, LEGEND_FONT = 7, 6, 5.2
YLABEL = "Time per request (" + "\u03bc" + "s)"   # 1,000 / KQPS: wall time per request
YLIM_N, YTICKS_N = (2, 50), [2, 5, 10, 20, 50]               # panel (a), log scale
YLIM_V, YTICKS_V = (2, 200), [2, 5, 10, 20, 50, 100, 200]    # panel (b), log scale: every point shown
STYLE = {   # label: (color, marker, linestyle)
    "DOMA (300 MHz)":       ("#3b75af", "o", "-"),
    "DOMA (230 MHz)":       ("#7fb0dc", "s", "--"),
    "DOMA, 4 KB (300 MHz)": ("#1f4e79", "D", ":"),
    "Tiny ORAM (230 MHz)":  ("#8e6bbf", "^", "-"),
    "BOLT (300 MHz)":       ("#e1812c", "*", "-"),
}
LEGEND_SIZE = (3.5, 0.34)          # inches: one legend strip across the column, above both panels
MS, LW = 3.2, 1.0
# ======================================================================

rows = list(csv.DictReader(open(CSV)))
others = list(csv.DictReader(open(OTHERS)))
def other(design, sweep, key):
    pts = sorted((int(r[key]), float(r["kqps"])) for r in others if r["design"] == design and r["sweep"] == sweep
                 and (sweep != "n" or int(r["n"]) <= N_MAX_PLOT))
    return [p[0] for p in pts], [p[1] for p in pts]
def doma(sweep, mhz, key):
    pts = sorted((int(r[key]), float(r["kqps"])) for r in rows if r["sweep"] == sweep and r["mhz"] == str(mhz))
    return [p[0] for p in pts], [p[1] for p in pts]

plt.rcParams.update({"font.size": FONT, "font.family": "DejaVu Sans"})

def finish(ax, xticks, xlabels, xlabel, outname, ylim, yticks):
    ax.set_xscale("log"); ax.set_yscale("log"); ax.set_ylim(*ylim)
    ax.yaxis.set_major_locator(FixedLocator(yticks)); ax.yaxis.set_major_formatter(FixedFormatter([str(t) for t in yticks]))
    ax.yaxis.set_minor_locator(NullLocator())
    ax.xaxis.set_major_locator(FixedLocator(xticks)); ax.xaxis.set_major_formatter(FixedFormatter(xlabels))
    ax.xaxis.set_minor_locator(NullLocator())
    ax.set_xlabel(xlabel, labelpad=1); ax.set_ylabel(YLABEL, labelpad=1)
    ax.tick_params(axis="both", labelsize=TICK_FONT, length=2, pad=1.5)
    ax.grid(axis="y", which="major", color="0.85", linewidth=0.5); ax.set_axisbelow(True)
    ax.figure.tight_layout(pad=0.3)
    for ext in ("pdf", "png"):
        ax.figure.savefig(f"{outname}.{ext}", dpi=200)
    print(f"wrote {outname}.pdf and {outname}.png")

def line(ax, label, xs, ys, open_marker=False):
    c, m, ls = STYLE[label]
    ys = [1000.0 / y for y in ys]          # KQPS -> microseconds per request
    ax.plot(xs, ys, color=c, marker=m, linestyle=ls, markersize=MS * (1.6 if m == "*" else 1), linewidth=LW,
            markerfacecolor="white" if open_marker else c, label=label)

# (a) N sweep: 64 B values, plus DOMA with 4 KB values
f, ax = plt.subplots(figsize=FIGSIZE)
line(ax, "DOMA (300 MHz)", *doma("n64", 300, "n"))
line(ax, "DOMA (230 MHz)", *doma("n64", 230, "n"))
line(ax, "DOMA, 4 KB (300 MHz)", *doma("n4k", 300, "n"))
line(ax, "Tiny ORAM (230 MHz)", *other("Tiny ORAM", "n", "n"))
line(ax, "BOLT (300 MHz)", *other("BOLT", "n", "n"))
ax.set_xlim(8e4, 2.5e6)
finish(ax, [100000, 500000, 1000000, 2000000], ["100K", "500K", "1M", "2M"], "Records", "doma_sweep_n", YLIM_N, YTICKS_N)

# (b) value sweep, 1M records
f, ax = plt.subplots(figsize=FIGSIZE)
line(ax, "DOMA (300 MHz)", *doma("vs", 300, "value_b"))
line(ax, "DOMA (230 MHz)", *doma("vs", 230, "value_b"))
line(ax, "Tiny ORAM (230 MHz)", *other("Tiny ORAM", "vs", "value_b"))
line(ax, "BOLT (300 MHz)", *other("BOLT", "vs", "value_b"))
ax.set_xlim(6, 5500)
finish(ax, [8, 64, 512, 4096], ["8", "64", "512", "4K"], "Value size (B)", "doma_sweep_value", YLIM_V, YTICKS_V)

# the shared legend strip
from matplotlib.lines import Line2D
handles = [Line2D([], [], color=c, marker=m, linestyle=ls, linewidth=LW, markersize=MS * (1.6 if m == "*" else 1),
                  label=lab) for lab, (c, m, ls) in STYLE.items()]
f = plt.figure(figsize=LEGEND_SIZE)
f.legend(handles=handles, loc="center", ncol=3, fontsize=LEGEND_FONT + 0.6, frameon=False, handlelength=2.0,
         columnspacing=1.0, labelspacing=0.3)
for ext in ("pdf", "png"):
    f.savefig(f"doma_sweep_legend.{ext}", dpi=200)
print("wrote doma_sweep_legend.pdf and doma_sweep_legend.png")
