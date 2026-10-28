#!/usr/bin/env python3
"""DOMA on-package memory against data size: capacity table and figure (capacity.pdf).

For one (lambda, Z), metadata + stash as a share of the data size, against the data size, one line per block
size. The on-package budget (default 16 GB, the U55C's HBM) is the curve metadata + stash = budget; each line is
solid while it fits and dotted beyond, with a marker labelled with the most data the budget protects. The table
gives metadata, stash and their total for a few data sizes.

Examples
  python capacity_plot.py                                # lambda 7, Z 8; 32 B .. 4 KB; capacity.pdf + table
  python capacity_plot.py --lam 4 --Z 8 --out capacity_l4.pdf
  python capacity_plot.py --data 64 256 1000 --sizes 64 4096 --tex capacity.tex --csv capacity.csv
  python capacity_plot.py --breakdown                    # metadata split into map / records / heads
  python capacity_plot.py --records per_slot             # before per-bucket GCM: one IV/tag per position

Standalone: requires numpy and matplotlib only.
"""
import argparse
import csv
import functools
import math
import sys

import numpy as np

# ======================================================================================================
# Model (identical in capacity_plot.py and z_lambda_sweep.py)
#   metadata = position map + bucket records (slot list, fill, IV/tag) + stash heads; every field sized to what
#              it addresses (log2 of N, of the bucket count B, of the pool size P) and packed, no word formats
#   stash    = pool blocks + links; pool size = ceil of Claim 1 at 2^-64 per access (any access sequence), from
#              stash_bound.py, the only place the bound lives (keep it in this script's folder)
#   N is rounded down to a multiple of lambda, so B = N / lambda as the stash analysis assumes
# ======================================================================================================
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stash_bound import claim1          # noqa: E402

RECORD_LAYOUTS = ("per_bucket", "per_slot", "tag_out", "ctr_only")
ROUND_BITS = False           # True: round each field up to whole bits instead of exact log2 widths


@functools.lru_cache(maxsize=None)
def pool_entries(n, lam, Z):
    """Pool entries: ceil of Claim 1 at 2^-64 per access (stash_bound.py); n is a multiple of lambda."""
    return min(n, math.ceil(claim1(n, n // lam, Z, splits=99)))


def clog2(x):
    return max(1, math.ceil(math.log2(x)))

def trusted(n, S, lam=8, Z=8, records="per_bucket"):
    """On-package bytes by structure for n blocks of S bytes, lambda real blocks per bucket on average, Z positions.
       position map, per block : status (2) + bucket (log2 B) + pool index of a stashed block (log2 P)
       bucket record, per bucket: slot list Z * log2 N + fill log2(Z + 1) + integrity state (records=)
       stash head, per bucket   : (Z + 1) eviction candidates + the overflow-chain head, each {valid, pool index}
       pool entry               : the block (S bytes) + links: slot (log2 N) + next (log2 P + 1) + prev (log2 P)
    records = 'per_bucket': one 96-bit IV and one 128-bit tag per bucket (per-bucket GCM, the current design)
              'per_slot'  : one IV and tag per position (before per-bucket GCM: 224 bits per slot)
              'tag_out'   : a 64-bit write counter per bucket; tags stored with the ciphertext
              'ctr_only'  : as tag_out, and the slot list and fill also travel with the ciphertext
    """
    n = max(lam, n // lam * lam)            # B = n / lambda exactly (the stash analysis assumes it)
    B = n // lam
    P = pool_entries(n, lam, Z)
    w = clog2 if ROUND_BITS else (lambda x: math.log2(max(x, 2)))
    SW, BW, FW, PW = w(n), w(B), w(Z + 1), w(P)
    ivtag_bits = {"per_bucket": 96 + 128, "per_slot": Z * (96 + 128), "tag_out": 64, "ctr_only": 64}[records]
    pm = math.ceil(n * (2 + BW + PW) / 8)
    rec = math.ceil(B * (ivtag_bits + (0 if records == "ctr_only" else Z * SW + FW)) / 8)
    ivtag = math.ceil(B * ivtag_bits / 8)
    heads = math.ceil(B * (Z + 2) * (PW + 1) / 8)
    links = math.ceil(P * (SW + (PW + 1) + PW) / 8)
    blocks = P * S
    metadata, stash = pm + rec + heads, links + blocks
    return dict(n=n, S=S, lam=lam, Z=Z, records=records, B=B, P=P, pool_frac=P / n,
                pm=pm, rec=rec, ivtag=ivtag, heads=heads, metadata=metadata, links=links, blocks=blocks,
                stash=stash, total=metadata + stash, data=n * S, ratio=(metadata + stash) / (n * S))

def blocks_for(D, S):
    """Blocks needed to hold D bytes of data in S-byte blocks."""
    return max(16, int(round(D / S)))


# ---------------- formatting ----------------
UNITS = {"si": (1e3, ["B", "KB", "MB", "GB", "TB", "PB"]), "iec": (1024.0, ["B", "KiB", "MiB", "GiB", "TiB", "PiB"])}
_UNIT = UNITS_ACTIVE = UNITS["si"]


def fmt(x, digits=3, pad=True):
    base, names = _UNIT
    i = 0
    while abs(x) >= base and i < len(names) - 1:
        x /= base
        i += 1
    s = (f"{x:#.{digits}g}".rstrip(".") if pad else f"{x:.{digits}g}") if x < 1000 else f"{x:,.0f}"
    if "e" in s:
        s = f"{x:.0f}"
    return f"{s} {names[i]}"


def fmt_block(S):
    return f"{S // 1024} KB" if S >= 1024 and S % 1024 == 0 else f"{S} B"


def config_header(lam, Z, records):
    rec_txt = {"per_bucket": "slot list + one 96-bit IV and 128-bit tag per bucket (per-bucket GCM)",
               "per_slot": "slot list + one IV and tag per position (before per-bucket GCM)",
               "tag_out": "slot list + 64-bit write counter per bucket; tags stored with the ciphertext",
               "ctr_only": "64-bit write counter per bucket; slot list and tag stored with the ciphertext"}[records]
    lay_txt = "fields sized to what they address (log2 N, log2 B, log2 P) and packed" + \
        (", whole bits" if ROUND_BITS else "")
    mar_txt = "ceil of Claim 1 at 2^-64 per access (stash_bound.py)"
    lines = [f"lambda = {lam}, Z = {Z}: N/lambda buckets of {Z} positions, {Z + 1} eviction candidates per head",
             f"layout: {lay_txt}", f"records: {rec_txt}", f"stash pool: {mar_txt}"]
    return lines


# ---------------- table, CSV, LaTeX ----------------
def make_rows(datas, sizes, lam, Z, records):
    rows = []
    for D in datas:
        for S in sizes:
            r = trusted(blocks_for(D, S), S, lam, Z, records)
            r["D"] = D
            rows.append(r)
    return rows


def print_table(rows, lam, Z, records, budget_bytes):
    print()
    for line in config_header(lam, Z, records):
        print("  " + line)
    fits_hdr = f"fits {fmt(budget_bytes, pad=False)}" if budget_bytes > 0 else ""
    hdr = (f"{'data':>10} {'block':>6} | {'metadata':>10} {'stash':>10} {'(pool)':>8} | {'meta+stash':>10} "
           f"{'/data':>6} {fits_hdr:>10}")
    print(hdr)
    print("-" * len(hdr))
    last = None
    for r in rows:
        if last is not None and r["D"] != last:
            print()
        last = r["D"]
        fits = ("yes" if r["total"] <= budget_bytes else "no") if budget_bytes > 0 else ""
        print(f"{fmt(r['D']):>10} {fmt_block(r['S']):>6} | {fmt(r['metadata']):>10} {fmt(r['stash']):>10} "
              f"{100 * r['pool_frac']:7.1f}% | {fmt(r['total']):>10} {r['ratio']:6.3f} {fits:>10}")
    print("metadata = position map + bucket records + stash heads; stash = pool blocks + their links")
    if budget_bytes > 0:
        print(f"fits = metadata + stash <= {fmt(budget_bytes, pad=False)}")


def print_breakdown(rows, lam, Z):
    print(f"\n  metadata by structure (lambda = {lam}, Z = {Z})")
    hdr = (f"{'data':>10} {'block':>6} | {'position map':>18} {'records':>18} {'(of which IV/tag)':>18} "
           f"{'stash heads':>18} | {'metadata':>18}")
    print(hdr)
    print("-" * len(hdr))
    last = None
    for r in rows:
        if last is not None and r["D"] != last:
            print()
        last = r["D"]
        n = r["n"]

        def cell(x):
            return f"{fmt(x)} ({x / n:.2f} B)"
        print(f"{fmt(r['D']):>10} {fmt_block(r['S']):>6} | {cell(r['pm']):>18} {cell(r['rec']):>18} "
              f"{cell(r['ivtag']):>18} {cell(r['heads']):>18} | {cell(r['metadata']):>18}")
    print("(bytes per block in parentheses)")


CSV_FIELDS = ["lam", "Z", "records", "data_target_bytes", "block_bytes", "N", "data_bytes", "buckets",
              "pool_entries", "pool_frac", "posmap_bytes", "records_bytes", "ivtag_bytes", "heads_bytes",
              "metadata_bytes", "stash_links_bytes", "stash_blocks_bytes", "stash_bytes", "total_bytes",
              "trusted_over_data"]


def write_csv(path, all_rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_FIELDS)
        for r in all_rows:
            w.writerow([r["lam"], r["Z"], r["records"], int(r["D"]), r["S"], r["n"],
                        r["data"], r["B"], r["P"], f"{r['pool_frac']:.6f}", r["pm"], r["rec"], r["ivtag"],
                        r["heads"], r["metadata"], r["links"], r["blocks"], r["stash"], r["total"],
                        f"{r['ratio']:.6f}"])
    print(f"wrote {path}")


def write_tex(path, configs, rows_by_cfg, budget_bytes, budget_name):
    out = []
    b = fmt(budget_bytes, pad=False)
    for (lam, Z), rows in zip(configs, rows_by_cfg):
        cols = "llrrrr" + ("c" if budget_bytes > 0 else "")
        head = r"Data & Block & Metadata & Stash & Total & Total/data"
        head += r" & Fits \\" if budget_bytes > 0 else r" \\"
        out += [r"\begin{table}[t]", r"\centering",
                rf"\caption{{On-package memory at $\lambda={lam}$, $Z={Z}$. Metadata: position map, bucket "
                r"records (IV/tag, slot list) and stash heads, each field sized to what it addresses. Stash: pool "
                r"blocks and links, sized by Claim~1 at $2^{-64}$ per access. Total = metadata + stash" +
                (rf"; it fits when the total is at most {b}, the {budget_name}'s HBM." if budget_bytes > 0 else ".") +
                "}",
                rf"\label{{tab:capacity_l{lam}_z{Z}}}", r"\small", r"\setlength{\tabcolsep}{3pt}",
                rf"\begin{{tabular}}{{{cols}}}", r"\toprule", head, r"\midrule"]
        last = None
        for r in rows:
            if last is not None and r["D"] != last:
                out.append(r"\addlinespace")
            dl = fmt(r["D"], pad=False) if r["D"] != last else ""
            last = r["D"]
            line = (f"{dl} & {fmt_block(r['S'])} & {fmt(r['metadata'])} & {fmt(r['stash'])} & "
                    f"{fmt(r['total'])} & {r['ratio']:.2f}")
            if budget_bytes > 0:
                line += " & " + ("yes" if r["total"] <= budget_bytes else "no")
            out.append(line + r" \\")
        out += [r"\bottomrule", r"\end{tabular}", r"\end{table}", ""]
    with open(path, "w") as f:
        f.write("\n".join(out))
    print(f"wrote {path}")

# ---------------- figure style (reference palette, light mode) ----------------
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
INK, INK2, MUTED, GRID, AXIS, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7", "#ffffff"


def _style(plt):
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Times New Roman", "Times", "Nimbus Roman", "Nimbus Roman No9 L", "DejaVu Serif"],
        "mathtext.fontset": "stix", "font.size": 8, "axes.titlesize": 8.5, "axes.labelsize": 8,
        "xtick.labelsize": 7, "ytick.labelsize": 7, "legend.fontsize": 7,
        "axes.edgecolor": AXIS, "axes.linewidth": 0.6, "axes.labelcolor": INK2, "text.color": INK,
        "xtick.color": INK2, "ytick.color": INK2, "xtick.major.size": 2.5, "ytick.major.size": 2.5,
        "xtick.major.width": 0.6, "ytick.major.width": 0.6, "xtick.minor.size": 0, "ytick.minor.size": 0,
        "axes.spines.top": False, "axes.spines.right": False,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.dpi": 300, "hatch.linewidth": 0.5,
    })


def _spread_labels(ax, items, min_px=9.0):
    """items: [(y_data, text, color, marker)] placed at the right edge; nudged apart vertically in display space."""
    if not items:
        return
    trans = ax.transData
    inv = trans.inverted()
    x_end = ax.get_xlim()[1]
    pts = sorted(((trans.transform((x_end, y))[1], txt, col, mk) for y, txt, col, mk in items), key=lambda t: t[0])
    ys = [p[0] for p in pts]
    for i in range(1, len(ys)):
        ys[i] = max(ys[i], ys[i - 1] + min_px)
    for i in range(len(ys) - 2, -1, -1):          # pull back down if the stack ran over the top
        ys[i] = min(ys[i], ys[i + 1] - min_px)
    for (y0, txt, col, mk), y in zip(pts, ys):
        yd = inv.transform((trans.transform((x_end, 1))[0], y))[1]
        ax.annotate(txt, xy=(1.0, yd), xycoords=("axes fraction", "data"), xytext=(7, 0),
                    textcoords="offset points", va="center", ha="left", color=INK2, fontsize=7)
        if mk == "dash":
            ax.annotate("", xy=(1.0, yd), xycoords=("axes fraction", "data"), xytext=(6, 0), textcoords="offset points",
                        arrowprops=dict(arrowstyle="-", color=col, lw=1.3, ls=(0, (2, 1)), shrinkA=0, shrinkB=0))
        else:
            ax.plot([1.0], [yd], marker="s", ms=4, color=col, transform=ax.get_yaxis_transform(), clip_on=False,
                    zorder=5)


def save(fig, plt, out):
    fig.savefig(out)
    if out.lower().endswith(".pdf"):
        png = out[:-4] + ".png"
        fig.savefig(png, dpi=300)
        print(f"wrote {out} and {png}")
    else:
        print(f"wrote {out}")
    plt.close(fig)

# colour follows the block size, not its position in --sizes, so figures from different runs agree
SIZE_SLOT = {32: 0, 64: 1, 256: 2, 4096: 3, 128: 4, 512: 5, 1024: 6, 2048: 7}


def size_color(S, sizes):
    if S in SIZE_SLOT:
        return SERIES[SIZE_SLOT[S]]
    free = [i for i in range(len(SERIES)) if i not in {SIZE_SLOT[x] for x in sizes if x in SIZE_SLOT}]
    others = [x for x in sizes if x not in SIZE_SLOT]
    return SERIES[free[others.index(S) % len(free)]] if free else SERIES[others.index(S) % len(SERIES)]


def budget_crossing(S, lam, Z, records, budget, lo, hi):
    """Data size (bytes) at which the trusted total first reaches the budget (bisection in log D)."""
    def over(lg):
        D = math.exp(lg)
        return trusted(blocks_for(D, S), S, lam, Z, records)["total"] >= budget
    a, b = math.log(lo), math.log(hi)
    if over(a):
        return lo
    if not over(b):
        return None
    for _ in range(40):
        m = (a + b) / 2
        a, b = (a, m) if over(m) else (m, b)
    return math.exp(b)


def data_ticks(lo, hi):
    """Powers of four in GB (1, 4, 16, 64, 256 GB, 1 TB, 4 TB, ...), in the active units."""
    base = UNITS_ACTIVE[0]
    out, v = [], 1.0
    while True:
        x = v * base ** 3
        if base == 1e3 and v >= 1000:                    # decimal units: 1024 GB -> 1 TB, 4096 GB -> 4 TB
            x = round(v / 1024 * 1000) * base ** 3
        if x > hi * 1.001:
            return out
        if x >= lo * 0.999:
            out.append(x)
        v *= 4


def plot_config(path, lam, Z, sizes, records, budget_bytes, drange, figsize=(3.5, 2.1), budget_name="U55C",
                ymax_axis=1.0):
    """One panel: trusted memory / protected data against the protected data size, one line per block size.
    The budget is one curve, trusted = budget, i.e. y = budget / data; each line is solid while it fits and
    dotted beyond, with a diamond (labelled with the data size) where it reaches the budget."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import FixedLocator, FuncFormatter, MultipleLocator, NullLocator

    _style(plt)
    fig, ax = plt.subplots(figsize=figsize)
    W, H = figsize
    fig.subplots_adjust(left=0.135, right=0.955, top=1 - 0.40 / H, bottom=0.36 / H)
    lo, hi = drange
    grid = list(np.geomspace(lo, hi, 80))
    ymax, ends, crossings = 0.0, [], []

    def ratio(D, S):
        return trusted(blocks_for(D, S), S, lam, Z, records)["ratio"]

    for S in sizes:
        col = size_color(S, sizes)
        dc = budget_crossing(S, lam, Z, records, budget_bytes, lo, hi) if budget_bytes > 0 else None
        xs = sorted(set(grid + ([dc] if dc is not None else [])))
        ys = [ratio(D, S) for D in xs]
        ymax = max(ymax, max(ys))
        fit = [(D, y) for D, y in zip(xs, ys) if dc is None or D <= dc]
        out = [(D, y) for D, y in zip(xs, ys) if dc is not None and D >= dc]
        if len(fit) > 1:
            ax.plot(*zip(*fit), color=col, lw=1.6, solid_capstyle="round", zorder=3)
        if len(out) > 1:
            ax.plot(*zip(*out), color=col, lw=1.3, ls=(0, (1, 1.3)), dash_capstyle="round", zorder=3)
        if dc is not None and lo < dc:
            yc = ratio(dc, S)
            ax.plot([dc], [yc], ls="none", marker="D", ms=4.2, mfc=SURFACE, mec=col, mew=1.3, zorder=5)
            crossings.append((yc, dc, S))
        ends.append((ys[-1], fmt_block(S), col, "sq"))
    top = ymax_axis if ymax_axis else max(1.4, math.ceil(ymax * 10 + 1) / 10)

    ax.set_xscale("log")
    ax.set_xlim(lo, hi)
    ax.set_ylim(0, top)
    ticks = data_ticks(lo, hi)
    ax.xaxis.set_major_locator(FixedLocator(ticks))
    ax.xaxis.set_minor_locator(NullLocator())
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: fmt(v, 3, pad=False).replace(" ", " ")))
    ax.yaxis.set_major_locator(MultipleLocator(0.2))
    ax.grid(True, axis="y", color=GRID, lw=0.5, zorder=0)
    ax.axhline(1.0, color=INK2, lw=0.8, ls=(0, (3, 2)), zorder=2)

    if budget_bytes > 0:
        b = fmt(budget_bytes, pad=False)
        # the budget: trusted = budget  <=>  y = budget / data
        bx = np.geomspace(max(lo, budget_bytes / top), hi, 120)
        ax.plot(bx, budget_bytes / bx, color=INK2, lw=0.9, ls=(0, (4, 1.5, 1, 1.5)), zorder=2)
    # label each budget marker with the data size it protects, on the side with more room
    def label_up(S, D, yc):
        others = [ratio(D, S2) for S2 in sizes if S2 != S] + [1.0]
        above = min([y - yc for y in others if y > yc] + [top - yc])
        below = min([yc - y for y in others if y < yc] + [yc])
        return above >= below
    for yc, D, S in crossings:
        # the budget curve runs up-left and down-right through each marker, so labels go up-right or down-left
        up = label_up(S, D, yc)
        ax.annotate(fmt(D, 2, pad=False), xy=(D, yc), xytext=(4, 4) if up else (-4, -4), textcoords="offset points",
                    ha="left" if up else "right", va="bottom" if up else "top", fontsize=6.5, color=INK2, zorder=6)

    ax.set_xlabel("data size")
    ax.set_ylabel("(metadata + stash) / protected data")
    # the configuration, left of the budget legend, on the row just above the plot
    fig.text(0.135, 1 - 0.27 / H, rf"$\lambda={lam}$, $Z={Z}$", ha="left", va="center", color=INK, fontsize=8)
    # legend: what each colour and each line style means
    from matplotlib.lines import Line2D
    blank = Line2D([], [], ls="none", label="block size:")
    size_h = [blank] + [Line2D([], [], color=size_color(S, sizes), lw=1.6, label=fmt_block(S)) for S in sizes]
    l1 = fig.legend(handles=size_h, loc="upper center", bbox_to_anchor=(0.5, 1 - 0.02 / H), ncol=len(size_h), frameon=False,
                    fontsize=6.4, handlelength=1.6, columnspacing=1.0, handletextpad=0.5, borderaxespad=0.1)
    fig.add_artist(l1)
    if budget_bytes > 0:
        b = fmt(budget_bytes, pad=False)
        style_h = [Line2D([], [], color=INK2, lw=0.9, ls=(0, (4, 1.5, 1, 1.5)),
                          label=f"metadata + stash = {b} ({budget_name})")]
        fig.legend(handles=style_h, loc="center", bbox_to_anchor=(0.6, 1 - 0.27 / H), ncol=1, frameon=False,
                   fontsize=6.4, handlelength=1.8, handletextpad=0.45, borderaxespad=0.1)
    fig.savefig(path)
    if path.lower().endswith(".pdf"):
        png = path[:-4] + ".png"
        fig.savefig(png, dpi=300)
        print(f"wrote {path} and {png}")
    else:
        print(f"wrote {path}")
    plt.close(fig)


def main(argv=None):
    global _UNIT, UNITS_ACTIVE, ROUND_BITS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lam", type=int, default=7, help="lambda, average real blocks per bucket (default 7)")
    ap.add_argument("--Z", type=int, default=8, help="Z, positions per bucket (default 8)")
    ap.add_argument("--sizes", type=int, nargs="+", default=[32, 64, 256, 4096], metavar="BYTES",
                    help="block sizes in bytes, one line each (default 32 64 256 4096)")
    ap.add_argument("--data", type=float, nargs="+", metavar="GB",
                    help="data sizes for the table, in GB (default 16 64 256 1000; iec: 16 64 256 1024)")
    ap.add_argument("--records", choices=RECORD_LAYOUTS, default="per_bucket",
                    help="integrity state in the bucket records (default per_bucket: one IV and tag per bucket)")
    ap.add_argument("--round-bits", action="store_true", help="round each field up to whole bits")
    ap.add_argument("--units", choices=("si", "iec"), default="si", help="si: 1 GB = 1e9 B; iec: 1 GiB = 2^30 B")
    ap.add_argument("--budget", type=float, default=16.0,
                    help="on-package budget in GB (GiB with --units iec); 0 = none (default 16, the U55C's HBM)")
    ap.add_argument("--budget-name", default="U55C", help="name shown with the budget (default U55C)")
    ap.add_argument("--breakdown", action="store_true", help="also print metadata by structure")
    ap.add_argument("--csv", metavar="PATH", help="write the table rows to a CSV file")
    ap.add_argument("--tex", metavar="PATH", help="write the table as LaTeX (booktabs)")
    ap.add_argument("--out", default="capacity.pdf", help="figure path (.pdf also writes a .png; default capacity.pdf)")
    ap.add_argument("--plot-range", type=float, nargs=2, metavar=("LO", "HI"),
                    help="x-axis data-size range in GB (default 1 4000; iec: 1 4096)")
    ap.add_argument("--figsize", type=float, nargs=2, default=[3.5, 2.1], metavar=("W", "H"))
    ap.add_argument("--ymax", type=float, default=1.0, help="top of the y axis (default 1.0; 0 = automatic)")
    args = ap.parse_args(argv)
    if not 1 <= args.lam <= args.Z:
        ap.error("need 1 <= lambda <= Z (lambda is the average number of real blocks per bucket, Z its positions)")
    _UNIT = UNITS_ACTIVE = UNITS[args.units]
    ROUND_BITS = args.round_bits
    G = _UNIT[0] ** 3
    budget = args.budget * G
    datas = [x * G for x in (args.data or ([16, 64, 256, 1000] if args.units == "si" else [16, 64, 256, 1024]))]
    drange = [x * G for x in (args.plot_range or ([1, 4000] if args.units == "si" else [1, 4096]))]
    if min(datas) <= 0 or drange[0] <= 0 or drange[1] <= drange[0]:
        ap.error("data sizes and the plot range must be positive (and LO < HI)")
    rows = make_rows(datas, args.sizes, args.lam, args.Z, args.records)
    print_table(rows, args.lam, args.Z, args.records, budget)
    if args.breakdown:
        print_breakdown(rows, args.lam, args.Z)
    if args.csv:
        write_csv(args.csv, rows)
    if args.tex:
        write_tex(args.tex, [(args.lam, args.Z)], [rows], budget, args.budget_name)
    plot_config(args.out, args.lam, args.Z, args.sizes, args.records, budget, drange,
                tuple(args.figsize), args.budget_name, args.ymax)


if __name__ == "__main__":
    sys.exit(main())
