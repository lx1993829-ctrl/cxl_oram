#!/usr/bin/env python3
"""Tiny ORAM vs BOLT on the U55C (tiny_vs_bolt.pdf): four panels side by side, full text width.

  (a) query time vs value size (1M entries)     (b) slowdown vs value size (1M entries)
  (c) query time vs data size (64-byte values)  (d) slowdown vs data size (64-byte values)

Query time = 1 / throughput. Slowdown = a design's time per query / its baseline's time per query.
  Data-size sweep (d): both designs against the same non-oblivious baseline kernel (one record per slot, all
    in HBM), at each design's own clock: BOLT 300 MHz, Tiny ORAM 230 MHz.
  Value-size sweep (b): Tiny ORAM against that baseline at 230 MHz. BOLT against BASE_300_VS_KQPS once every
    entry is filled in; until then against BOLT's own all-HBM build (a different, much slower baseline), and
    the script says so when it runs.

Data (all measured on the same U55C):
  BOLT: released artifact, 20% of the data in HBM, 300 MHz, 5000 commands per run (value sizes: value_scaling_
        experiment, 8 B from hbm_distribution_experiment; data sizes: value_size_64 with INIT_SIZE set per run).
  Tiny ORAM: U55C port, top of the tree on-package (doma:20), 230 MHz.
  Baseline: non-oblivious kernel, one record per slot, all in HBM, 20,000 commands per run.

Standalone: requires matplotlib only.
    python tiny_vs_bolt.py [--out tiny_vs_bolt.pdf]
"""
import argparse

# ---- value-size sweep, 1M entries --------------------------------------------------------------------------
VS_BOLT = [8, 16, 32, 64, 128, 256]
VS_BOLT_MS = [28.53, 29.48, 34.97, 34.70, 43.04, 61.09]          # BOLT, total for 5000 commands
VS_BOLT_OWN_BASE_MS = [13.24, 13.13, 15.15, 15.15, 14.23, 16.31]  # BOLT's own all-HBM build, same runs
BASE_300_VS_KQPS = [None, None, None, 2354.6, None, None]         # baseline kernel at 300 MHz (fill in 8-256 B)
VS_TINY = [64, 128, 256, 512, 1024, 2048]
VS_TINY_KQPS = [59.8, 53.5, 44.5, 25.6, 13.8, 7.2]
VS_BASE_230_KQPS = [2113.6, 2106.2, 1946.8, 1778.9, 1065.3, 534.5]

# ---- data-size sweep, 64-byte values -----------------------------------------------------------------------
DS_N = [100_000, 500_000, 1_000_000, 2_000_000]
DS_BOLT_MS = [34.4608, 34.7783, 34.7533, 34.9687]
DS_TINY_KQPS = [79.3, 75.7, 59.9, 57.2]
DS_BASE_300_KQPS = [2351.4, 2352.1, 2354.6, 2332.9]
DS_BASE_230_KQPS = [2109.2, 2113.4, 2114.9, 2115.0]

# ---- derived -------------------------------------------------------------------------------------------------
vs_bolt_kqps = [5000 / ms for ms in VS_BOLT_MS]
vs_bolt_us = [1e3 / k for k in vs_bolt_kqps]
vs_tiny_us = [1e3 / k for k in VS_TINY_KQPS]
VS_COMMON = all(b is not None for b in BASE_300_VS_KQPS)
vs_bolt_slow = ([b / k for k, b in zip(vs_bolt_kqps, BASE_300_VS_KQPS)] if VS_COMMON
                else [a / b for a, b in zip(VS_BOLT_MS, VS_BOLT_OWN_BASE_MS)])
vs_tiny_slow = [b / k for k, b in zip(VS_TINY_KQPS, VS_BASE_230_KQPS)]

ds_bolt_kqps = [5000 / ms for ms in DS_BOLT_MS]
ds_bolt_us = [1e3 / k for k in ds_bolt_kqps]
ds_tiny_us = [1e3 / k for k in DS_TINY_KQPS]
ds_bolt_slow = [b / k for k, b in zip(ds_bolt_kqps, DS_BASE_300_KQPS)]
ds_tiny_slow = [b / k for k, b in zip(DS_TINY_KQPS, DS_BASE_230_KQPS)]

# ---- figure style (same as the paper's other figures) ------------------------------------------------------
SERIES = ["#2a78d6", "#eb6834"]
INK, INK2, GRID, AXIS = "#0b0b0b", "#52514e", "#e1e0d9", "#c3c2b7"
FS_TICK, FS_LABEL, FS_NOTE = 6, 6.5, 5.6
LW, MS = 1.1, 3.2


def _style(plt):
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "Nimbus Roman", "Nimbus Roman No9 L", "DejaVu Serif"],
        "mathtext.fontset": "stix", "font.size": 8, "axes.labelsize": 8,
        "axes.edgecolor": AXIS, "axes.linewidth": 0.6, "axes.labelcolor": INK2, "text.color": INK,
        "xtick.color": INK2, "ytick.color": INK2, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
        "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.minor.size": 0, "ytick.minor.size": 0,
        "axes.spines.top": False, "axes.spines.right": False,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.dpi": 300,
    })


def _fmt(v):
    return f"{v:.0f}" if v >= 10 else f"{v:.1f}"


def _panel(ax, xb, yb, xt, yt, *, title, ylabel, xlabel, xticks, xticklabels, xlim, ylog, yticks, ylim, xbase):
    from matplotlib.ticker import FixedLocator, NullFormatter, NullLocator
    ax.plot(xb, yb, color=SERIES[0], lw=LW, marker="o", ms=MS, zorder=3)
    ax.plot(xt, yt, color=SERIES[1], lw=LW, marker="s", ms=MS * 0.9, zorder=3)
    ax.set_xscale("log", base=xbase)
    if ylog:
        ax.set_yscale("log")
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.xaxis.set_major_locator(FixedLocator(xticks))
    ax.set_xticklabels(xticklabels, fontsize=FS_TICK)
    ax.xaxis.set_minor_locator(NullLocator())
    ax.xaxis.set_minor_formatter(NullFormatter())
    ax.yaxis.set_major_locator(FixedLocator(yticks))
    ax.set_yticklabels([f"{v:g}" for v in yticks], fontsize=FS_TICK)
    ax.yaxis.set_minor_locator(NullLocator())
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.tick_params(pad=1.5)
    ax.grid(True, axis="y", color=GRID, lw=0.5, zorder=0)
    ax.set_xlabel(xlabel, fontsize=FS_LABEL, labelpad=1.5)
    ax.set_ylabel(ylabel, fontsize=FS_LABEL, labelpad=1.5)
    ax.set_title(title, loc="left", color=INK, fontsize=FS_LABEL, pad=3)
    # selective direct labels: the values at each line's ends
    for xs, ys, va, dy in ((xb, yb, "top", -4), (xt, yt, "bottom", 3.5)):
        for i in (0, len(xs) - 1):
            ax.annotate(_fmt(ys[i]), (xs[i], ys[i]), xytext=(0, dy), textcoords="offset points", ha="center",
                        va=va, fontsize=FS_NOTE, color=INK2, zorder=4)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="tiny_vs_bolt.pdf", help="figure path (.pdf also writes a .png)")
    ap.add_argument("--figsize", type=float, nargs=2, default=[7.16, 1.95], metavar=("W", "H"))
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec
    from matplotlib.lines import Line2D
    _style(plt)

    fig = plt.figure(figsize=args.figsize)
    gs = GridSpec(1, 5, figure=fig, width_ratios=[1, 1, 0.16, 1, 1], left=0.052, right=0.993, bottom=0.2,
                  top=0.78, wspace=0.42)
    axa, axb, axc, axd = (fig.add_subplot(gs[0, i]) for i in (0, 1, 3, 4))

    vs_ticks = [8, 16, 32, 64, 128, 256, 512, 1024, 2048]
    vs_lab = [{8: "8", 32: "32", 128: "128", 512: "512", 2048: "2K"}.get(v, "") for v in vs_ticks]
    vs = dict(xlabel="value size (B)", xticks=vs_ticks, xticklabels=vs_lab, xlim=(6, 2900), xbase=2, ylog=True)
    _panel(axa, VS_BOLT, vs_bolt_us, VS_TINY, vs_tiny_us, title="(a) query time", ylabel="time per query (µs)",
           yticks=[2, 5, 10, 20, 50, 100, 200], ylim=(2.5, 300), **vs)
    _panel(axb, VS_BOLT, vs_bolt_slow, VS_TINY, vs_tiny_slow, title="(b) slowdown", ylabel="slowdown (×)",
           yticks=[1, 2, 5, 10, 20, 50, 100], ylim=(1, 150), **vs)

    ds_lab = ["100K", "500K", "1M", "2M"]
    ds = dict(xlabel="data size (entries)", xticks=DS_N, xticklabels=ds_lab, xlim=(75_000, 2_700_000), xbase=10,
              ylog=False)
    _panel(axc, DS_N, ds_bolt_us, DS_N, ds_tiny_us, title="(c) query time", ylabel="time per query (µs)",
           yticks=[0, 5, 10, 15, 20], ylim=(0, 21), **ds)
    _panel(axd, DS_N, ds_bolt_slow, DS_N, ds_tiny_slow, title="(d) slowdown", ylabel="slowdown (×)",
           yticks=[0, 10, 20, 30, 40], ylim=(0, 42), **ds)

    # sweep labels over each pair of panels, legend in between
    for (l, r), text in (((axa, axb), "value-size sweep, 1M entries"), ((axc, axd), "data-size sweep, 64-byte values")):
        x0, x1 = l.get_position().x0, r.get_position().x1
        fig.text((x0 + x1) / 2, 0.875, text, ha="center", va="bottom", fontsize=FS_LABEL, color=INK)
        fig.add_artist(Line2D([x0, x1], [0.865, 0.865], color=AXIS, lw=0.6))
    handles = [
        Line2D([], [], color=SERIES[0], lw=LW, marker="o", ms=MS, label="BOLT (20% of data in HBM, 300 MHz)"),
        Line2D([], [], color=SERIES[1], lw=LW, marker="s", ms=MS * 0.9,
               label="Tiny ORAM (tree top on-package, 230 MHz)"),
    ]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 1.02), ncol=2, frameon=False,
               fontsize=FS_LABEL, handlelength=1.8, borderaxespad=0.2, columnspacing=2.0)

    fig.savefig(args.out)
    if args.out.lower().endswith(".pdf"):
        fig.savefig(args.out[:-4] + ".png", dpi=300)
    plt.close(fig)
    print("wrote", args.out)
    if not VS_COMMON:
        print("note: (b) uses BOLT's own all-HBM build as BOLT's baseline; fill in BASE_300_VS_KQPS for 8-256 B "
              "to use the same baseline kernel as (d)")

    f = lambda x, d=2: f"{x:.{d}f}" if x is not None else "-"
    print(f"\nvalue-size sweep (1M)\n{'value':>6} {'BOLT us':>8} {'Tiny us':>8} {'BOLT slow':>9} {'Tiny slow':>9}")
    for v in sorted(set(VS_BOLT) | set(VS_TINY)):
        b = vs_bolt_us[VS_BOLT.index(v)] if v in VS_BOLT else None
        t = vs_tiny_us[VS_TINY.index(v)] if v in VS_TINY else None
        bs = vs_bolt_slow[VS_BOLT.index(v)] if v in VS_BOLT else None
        ts = vs_tiny_slow[VS_TINY.index(v)] if v in VS_TINY else None
        print(f"{v:>5}B {f(b):>8} {f(t):>8} {f(bs):>9} {f(ts, 1):>9}")
    print(f"\ndata-size sweep (64 B)\n{'N':>9} {'BOLT us':>8} {'Tiny us':>8} {'BOLT slow':>9} {'Tiny slow':>9}")
    for i, n in enumerate(DS_N):
        print(f"{n:>9} {ds_bolt_us[i]:8.2f} {ds_tiny_us[i]:8.2f} {ds_bolt_slow[i]:9.1f} {ds_tiny_slow[i]:9.1f}")


if __name__ == "__main__":
    main()
