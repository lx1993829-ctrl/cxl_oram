#!/usr/bin/env python3
"""DOMA: choosing Z and lambda when a share p of the bucket data is kept on-package (z_lambda_sweep.pdf).

On-package memory = metadata + stash + p of the bucket data (p of the buckets, each Z * S bytes including its
slack positions), i.e. p * Z / lambda of the data size N * S. Two half-width panels side by side in one
column (3.5 in), sharing the y axis; one line per p, the minimum of each line circled and labelled:
  (a) lambda fixed (default 6), Z swept from lambda to --zmax; a second row of x labels gives the bytes per
      request, 2ZS
  (b) Z fixed (default 8), lambda swept from --lmin (default Z/2) to Z; the traffic per request is 2ZS throughout

Examples
  python z_lambda_sweep.py                             # (a) lambda 6, (b) Z 8; 4 KB, p = 0, 10, 20, 30%
  python z_lambda_sweep.py --lam 6 --Z 12 --zmax 16
  python z_lambda_sweep.py --block 256 --out z_lambda_sweep_256.pdf

Standalone: requires numpy and matplotlib only.
"""
import argparse
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


def total(lam, Z, S, data_bytes, share, records="per_bucket"):
    """On-package memory as a fraction of the data size N * S: metadata + stash + share * Z / lambda (share = the
    share of the bucket data kept on-package: that many buckets of Z * S bytes, slack positions included)."""
    t = trusted(blocks_for(data_bytes, S), S, lam, Z, records)
    return t["ratio"] + share * Z / lam


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


def save(fig, plt, out):
    fig.savefig(out)
    if out.lower().endswith(".pdf"):
        png = out[:-4] + ".png"
        fig.savefig(png, dpi=300)
        print(f"wrote {out} and {png}")
    else:
        print(f"wrote {out}")
    plt.close(fig)


def best(xs, c):
    return min(range(len(xs)), key=lambda j: c[j])


LW, MS, MS_MIN = 1.2, 2.0, 5.0          # line width, point size, minimum-circle size (half-width panels)
FS_TICK, FS_LABEL, FS_MIN = 6, 6.5, 6   # font sizes: ticks, axis labels / titles / legend, minimum labels


def panel(ax, xs, curves):
    """One line per share p, the minimum of each circled. Returns the minima [(x, y, color)]."""
    mins = []
    for k, (p, c) in enumerate(curves):
        col = SERIES[k % len(SERIES)]
        ys = [100 * v for v in c]
        ax.plot(xs, ys, color=col, lw=LW, marker="o", ms=MS, mec=col, mfc=col, zorder=3)
        i = best(xs, c)
        ax.plot([xs[i]], [ys[i]], ls="none", marker="o", ms=MS_MIN, mfc=SURFACE, mec=col, mew=1.1, zorder=5)
        mins.append((xs[i], ys[i], col))
    ax.set_xlim(xs[0] - 0.5, xs[-1] + 0.5)
    ax.grid(True, axis="y", color=GRID, lw=0.5, zorder=0)
    ax.tick_params(labelsize=FS_TICK, pad=2)
    return mins


def _darken(hex_color, f=0.28):
    """The line colour, darkened so a small label in it stays legible (the label takes its line's colour)."""
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % tuple(int(v * (1 - f)) for v in (r, g, b))


# where a minimum's label may go, in order of preference: (dx, dy) in points, ha, va
SPOTS = [(0, 5, "center", "bottom"), (0, -3.5, "center", "top"),
         (3.5, 3.5, "left", "bottom"), (-3.5, 3.5, "right", "bottom"),
         (3.5, -3, "left", "top"), (-3.5, -3, "right", "top"),
         (5, 0, "left", "center"), (-5, 0, "right", "center")]


def label_minima(fig, ax, xs, curves, mins, sym):
    """Label each circled minimum '<sym>=x', in its line's colour, at the first spot that touches no line, circle
    or earlier label (and is not closer to another line than to its own circle)."""
    fig.canvas.draw()
    rend = fig.canvas.get_renderer()
    pt = fig.dpi / 72.0
    tr = ax.transData
    lines = []
    for _, c in curves:
        d = tr.transform(list(zip(xs, [100 * v for v in c])))
        seg = [(x0 + t * (x1 - x0), y0 + t * (y1 - y0))
               for (x0, y0), (x1, y1) in zip(d[:-1], d[1:]) for t in np.linspace(0, 1, 40)]
        lines.append(np.array(seg))
    pts = np.vstack(lines)
    circles = [tr.transform((x, y)) for x, y, _ in mins]
    frame = ax.get_window_extent(rend)
    r_min = MS_MIN / 2 * pt
    placed = []

    def gap(bb, p):
        p = np.atleast_2d(p)
        dx = np.maximum(0, np.maximum(bb.x0 - p[:, 0], p[:, 0] - bb.x1))
        dy = np.maximum(0, np.maximum(bb.y0 - p[:, 1], p[:, 1] - bb.y1))
        return float(np.min(np.hypot(dx, dy)))

    def clashes(bb):
        pad = 0.9 * pt
        x0, x1, y0, y1 = bb.x0 - pad, bb.x1 + pad, bb.y0 - pad, bb.y1 + pad
        n = int(np.sum((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)))
        n += sum(1 for c in circles if gap(bb, c) < r_min + 1.5 * pt)
        n += sum(1 for o in placed if bb.overlaps(o))
        if bb.x0 < frame.x0 or bb.x1 > frame.x1 or bb.y0 < frame.y0 or bb.y1 > frame.y1:
            n += 100
        return n

    def ambiguous(bb, k):
        own = gap(bb, circles[k]) - r_min
        other = min(gap(bb, l) for j, l in enumerate(lines) if j != k)
        return other < own + 2.0 * pt

    for k, (x, y, col) in enumerate(mins):
        best_ann, best_score = None, None
        for dx, dy, ha, va in SPOTS:
            ann = ax.annotate(f"{sym}={x}", xy=(x, y), xytext=(dx, dy), textcoords="offset points",
                              ha=ha, va=va, fontsize=FS_MIN, color=_darken(col), zorder=6)
            bb = ann.get_window_extent(rend)
            score = 10 * clashes(bb) + (1 if ambiguous(bb, k) else 0)
            if best_score is None or score < best_score:
                if best_ann is not None:
                    best_ann.remove()
                best_ann, best_score = ann, score
            else:
                ann.remove()
            if score == 0:
                break
        placed.append(best_ann.get_window_extent(rend))


def main(argv=None):
    global ROUND_BITS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lam", type=int, default=6, help="panel (a): fixed lambda (default 6)")
    ap.add_argument("--zmax", type=int, default=20, help="panel (a): largest Z (default 20); Z starts at lambda")
    ap.add_argument("--Z", type=int, default=8, help="panel (b): fixed Z (default 8)")
    ap.add_argument("--lmin", type=int, help="panel (b): smallest lambda (default Z/2); the sweep ends at Z")
    ap.add_argument("--block", type=int, default=4096, help="block size in bytes (default 4096)")
    ap.add_argument("--shares", type=float, nargs="+", default=[0, 10, 20, 30], metavar="PCT",
                    help="share of the bucket data kept on-package, in percent; one line each (default 0 10 20 30)")
    ap.add_argument("--data", type=float, default=64.0,
                    help="data size in GB used for the model (default 64; the result barely depends on it)")
    ap.add_argument("--records", choices=RECORD_LAYOUTS, default="per_bucket")
    ap.add_argument("--round-bits", action="store_true", help="round each field up to whole bits")
    ap.add_argument("--figsize", type=float, nargs=2, default=[3.5, 2.5], metavar=("W", "H"),
                    help="figure size in inches (default 3.5 2.5: one column, each panel half of it)")
    ap.add_argument("--out", default="z_lambda_sweep.pdf", help="figure path (.pdf also writes a .png)")
    args = ap.parse_args(argv)
    lmin = args.lmin if args.lmin is not None else max(1, args.Z // 2)
    if args.lam < 1 or args.zmax < args.lam:
        ap.error("panel (a) needs 1 <= --lam <= --zmax")
    if not 1 <= lmin <= args.Z:
        ap.error("panel (b) needs 1 <= --lmin <= --Z")
    ROUND_BITS = args.round_bits
    S, D, lam_a, Z_b = args.block, args.data * 1e9, args.lam, args.Z
    za = list(range(lam_a, args.zmax + 1))
    lb = list(range(lmin, Z_b + 1))
    ca = [(p, [total(lam_a, Z, S, D, p / 100.0, args.records) for Z in za]) for p in args.shares]
    cb = [(p, [total(lam, Z_b, S, D, p / 100.0, args.records) for lam in lb]) for p in args.shares]

    hdr = " ".join(f"{'p=' + format(p, 'g') + '%':>8}" for p in args.shares)
    print(f"on-package memory (% of data size) = metadata + stash + p * Z / lambda, {S}-byte blocks")
    print(f"\n(a) lambda = {lam_a}\n{'Z':>4} {'2ZS':>8} | {hdr}")
    for i, Z in enumerate(za):
        print(f"{Z:>4} {str(2 * Z * S // 1024) + ' KB':>8} | " + " ".join(f"{100 * c[i]:7.1f}%" for _, c in ca))
    for p, c in ca:
        i = best(za, c)
        print(f"  p = {p:g}%: best Z = {za[i]} ({100 * c[i]:.1f}%; Z = {lam_a}: {100 * c[0]:.1f}%)")
    print(f"\n(b) Z = {Z_b} (2ZS = {2 * Z_b * S // 1024} KB per request)\n{'lambda':>6} | {hdr}")
    for i, lam in enumerate(lb):
        print(f"{lam:>6} | " + " ".join(f"{100 * c[i]:7.1f}%" for _, c in cb))
    for p, c in cb:
        i = best(lb, c)
        print(f"  p = {p:g}%: best lambda = {lb[i]} ({100 * c[i]:.1f}%; lambda = {Z_b}: {100 * c[-1]:.1f}%)")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FixedLocator, MultipleLocator, PercentFormatter

    _style(plt)
    ymax = max(100 * max(c) for _, c in ca + cb)
    top = 20 * (int(ymax // 20) + 1)
    blk = f"{S // 1024} KB" if S >= 1024 else f"{S} B"

    # one column, two half-width panels side by side sharing the y axis; positions in inches
    W, H = args.figsize
    left, gap_w, right, bot, hdr_h = 0.44, 0.10, 0.05, 0.50, 0.56
    pw, ph = (W - left - gap_w - right) / 2, H - bot - hdr_h
    fig = plt.figure(figsize=(W, H))
    axa = fig.add_axes([left / W, bot / H, pw / W, ph / H])
    axb = fig.add_axes([(left + pw + gap_w) / W, bot / H, pw / W, ph / H], sharey=axa)

    mins_a = panel(axa, za, ca)
    step = 4 if len(za) > 8 else 2
    ticks = [z for z in za if (z - za[0]) % step == 0]
    axa.xaxis.set_major_locator(FixedLocator(ticks))
    axa.set_xticklabels([f"{z}\n{2 * z * S // 1024} KB" for z in ticks], linespacing=1.25)
    axa.set_xlabel("Z (top), 2ZS (bottom)", fontsize=FS_LABEL, labelpad=1.5)
    axa.set_title(rf"(a) $\lambda={lam_a}$, sweep $Z$", loc="left", color=INK, fontsize=FS_LABEL, pad=3)

    mins_b = panel(axb, lb, cb)
    axb.xaxis.set_major_locator(FixedLocator(lb))
    axb.set_xlabel(r"$\lambda$", fontsize=FS_LABEL, labelpad=1.5)
    axb.set_title(rf"(b) $Z={Z_b}$ (2ZS = {2 * Z_b * S // 1024} KB), sweep $\lambda$", loc="left", color=INK,
                  fontsize=FS_LABEL, pad=3)
    axb.tick_params(axis="y", labelleft=False, length=0)

    axa.set_ylim(0, top)
    axa.yaxis.set_major_locator(MultipleLocator(20))
    axa.yaxis.set_major_formatter(PercentFormatter(decimals=0))
    axa.set_ylabel("on-package memory (% of data)", fontsize=FS_LABEL, labelpad=2)

    label_minima(fig, axa, za, ca, mins_a, "Z")
    label_minima(fig, axb, lb, cb, mins_b, r"$\lambda$")

    # header: what p is, then the legend on its own row
    handles = [Line2D([], [], color=SERIES[k % len(SERIES)], lw=LW, marker="o", ms=MS, label=f"p = {p:g}%")
               for k, p in enumerate(args.shares)]
    head = fig.text(left / W, (H - 0.10) / H, f"{blk} blocks; p = share of bucket data on-package;", ha="left",
                    va="center", color=INK, fontsize=FS_LABEL)
    # after the header, on the same line: the key for the circled minima
    fig.canvas.draw()
    x_end = fig.transFigure.inverted().transform(head.get_window_extent(fig.canvas.get_renderer()).p1)[0]
    fig.legend(handles=[Line2D([], [], ls="none", marker="o", ms=MS_MIN, mfc=SURFACE, mec=INK2, mew=1.1,
                               label="minimum")], loc="center left", bbox_to_anchor=(x_end + 0.004, (H - 0.10) / H),
               frameon=False, fontsize=FS_LABEL, handlelength=1.0, handletextpad=0.3, borderaxespad=0, borderpad=0)
    fig.legend(handles=handles, loc="center left", bbox_to_anchor=(left / W - 0.012, (H - 0.26) / H),
               ncol=len(handles), frameon=False, fontsize=FS_LABEL, handlelength=1.5, handletextpad=0.4,
               columnspacing=1.0, borderaxespad=0, borderpad=0)
    save(fig, plt, args.out)


if __name__ == "__main__":
    sys.exit(main())
