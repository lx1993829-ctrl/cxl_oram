#!/usr/bin/env python3
"""DOMA vs BOLT at matched configurations: work per request and on-package memory (bolt_tradeoff.pdf).

Theoretical, no measurements. Both designs get the same average load per bucket and the same bucket size:
  load    : BOLT's c (average tuples per bin) = DOMA's lambda (average real blocks per bucket)
  bucket  : BOLT's page l_max = c + ceil(log2 log2 N) (their Claim 4.2, unit constant) = DOMA's Z
Panel (a): blocks moved per request (read + written back): DOMA one bucket, 2Z; BOLT two bins, 4 * l_max.
           DOMA also decrypts, verifies and re-encrypts its Z blocks; BOLT's cycle counts include no cryptography.
Panel (b): on-package memory (metadata + stash, % of data), no bucket data on-package.
           DOMA: stash pool ceil(Claim 1) at 2^-64 per access (stash_bound.py), position map, bucket records
           (IV/tag), heads; N rounded down to a multiple of lambda.
           BOLT: stash M/2 + l_max * sqrt(M ln N) (Claim 4.4, alpha = 0), address-indexed position map
           (two bin indices per block), no IV/tag; its stash bookkeeping (reverse index, count list) is not counted
           (negligible at 4 KB). BOLT's bound holds with probability 1 - 1/O(N), with its O-term constant set to 1.

Examples
  python bolt_tradeoff.py                      # 4 KB blocks, N = 2^24, c = lambda = 2..16
  python bolt_tradeoff.py --block 64 --out bolt_tradeoff_64.pdf

Requires numpy, matplotlib, and stash_bound.py in the same folder.
"""
import argparse
import functools
import math
import sys


# ======================================================================================================
# Model (identical in capacity_plot.py, z_lambda_sweep.py and bolt_tradeoff.py)
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



def _darken(hex_color, f=0.28):
    """The line colour, darkened so a small label in it stays legible."""
    r, g, b = (int(hex_color[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % tuple(int(v * (1 - f)) for v in (r, g, b))


def bolt_point(n, S, c):
    """BOLT at average bin load c, alpha = 0. Returns (l_max, blocks per request, share, stash share, map share)."""
    lmax = c + math.ceil(math.log2(math.log2(n)))
    M = math.ceil(n / c)
    stash = M / 2 + lmax * math.sqrt(M * math.log(n))
    pm = n * 2 * math.log2(M) / 8
    return lmax, 4 * lmax, (stash * S + pm) / (n * S), stash / n, pm / (n * S)



def main(argv=None):
    global ROUND_BITS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--block", type=int, default=4096, help="block size in bytes (default 4096)")
    ap.add_argument("--logn", type=int, default=24, help="N = 2^logn blocks (default 24)")
    ap.add_argument("--loads", type=int, nargs="+", default=[2, 4, 6, 8, 10, 12, 14, 16], metavar="C",
                    help="average loads c = lambda to compare (default 2 4 ... 16)")
    ap.add_argument("--round-bits", action="store_true", help="round each DOMA field up to whole bits")
    ap.add_argument("--figsize", type=float, nargs=2, default=[3.5, 2.3], metavar=("W", "H"))
    ap.add_argument("--out", default="bolt_tradeoff.pdf", help="figure path (.pdf also writes a .png)")
    args = ap.parse_args(argv)
    ROUND_BITS = args.round_bits
    S, n = args.block, 2 ** args.logn

    rows = []
    for c in args.loads:
        lmax, wb, rb, _, _ = bolt_point(n, S, c)
        rd = trusted(n, S, c, lmax)["ratio"]
        rows.append((c, lmax, 2 * lmax, wb, rd, rb))
    print(f"N = 2^{args.logn}, {S}-byte blocks; matched load c = lambda and bucket Z = l_max")
    print(" c=lam  Z=l_max | blocks/request DOMA  BOLT | on-package DOMA  BOLT | BOLT/DOMA work, DOMA/BOLT memory")
    for c, z, wd, wb, rd, rb in rows:
        print(f"  {c:4d}  {z:6d}  | {wd:13d} {wb:5d} | {100 * rd:13.1f}% {100 * rb:5.1f}% | {wb / wd:.2f}, {rd / rb:.2f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FixedLocator, MultipleLocator, PercentFormatter
    from matplotlib import transforms
    ROW_PT = 7.5                                   # spacing of the two rows under the x axis

    _style(plt)
    W, H = args.figsize
    left, gap, right, bot, top_h = 0.42, 0.42, 0.06, 0.50, 0.52
    pw, ph = (W - left - gap - right) / 2, H - bot - top_h
    fig = plt.figure(figsize=(W, H))
    axa = fig.add_axes([left / W, bot / H, pw / W, ph / H])
    axb = fig.add_axes([(left + pw + gap) / W, bot / H, pw / W, ph / H])
    cd, cb = SERIES[0], SERIES[1]
    cs = [r[0] for r in rows]
    for ax, yd, yb in ((axa, [r[2] for r in rows], [r[3] for r in rows]),
                       (axb, [100 * r[4] for r in rows], [100 * r[5] for r in rows])):
        ax.plot(cs, yd, color=cd, lw=1.3, marker="o", ms=2.2, zorder=3)
        ax.plot(cs, yb, color=cb, lw=1.3, marker="o", ms=2.2, zorder=3)
        ax.set_xlim(cs[0] - 0.8, cs[-1] + 0.8)
        ax.grid(True, axis="y", color=GRID, lw=0.5, zorder=0)
        ax.tick_params(labelsize=6)
        ticks = [c for c in cs if c % 4 == 0]
        ax.xaxis.set_major_locator(FixedLocator(ticks))
        ax.tick_params(axis="x", labelbottom=False)
        # two rows under the axis: lambda, then Z, each with its symbol at the left
        base = plt.rcParams["xtick.major.size"] + plt.rcParams["xtick.major.pad"]
        for row, (sym, vals) in enumerate(((r"$\lambda$", ticks), (r"$Z$", [bolt_point(n, S, c)[0] for c in ticks]))):
            dy = -(base + row * ROW_PT)
            tr = transforms.offset_copy(ax.get_xaxis_transform(), fig=fig, x=0, y=dy, units="points")
            for c, v in zip(ticks, vals):
                ax.text(c, 0, str(v), transform=tr, ha="center", va="top", fontsize=6, color=INK2)
            tr0 = transforms.offset_copy(ax.transAxes, fig=fig, x=1.5, y=dy, units="points")
            ax.text(0, 0, sym, transform=tr0, ha="left", va="top", fontsize=6.5, color=INK)

    axa.set_ylim(0, 5 * (int(max(r[3] for r in rows) / 5) + 1))
    axa.yaxis.set_major_locator(MultipleLocator(20))
    axa.set_ylabel("slots accessed per request", fontsize=6.5, labelpad=2)
    axa.set_title("(a) slots moved", loc="left", fontsize=6.5, color=INK, pad=3)
    axb.set_ylim(0, 10 * math.ceil(max(100 * r[4] for r in rows) / 10))
    axb.yaxis.set_major_locator(MultipleLocator(10))
    axb.yaxis.set_major_formatter(PercentFormatter(decimals=0))
    axb.set_ylabel("on-package memory (% of data)", fontsize=6.5, labelpad=2)
    axb.set_title("(b) on-package memory", loc="left", fontsize=6.5, color=INK, pad=3)

    # one x label shared by both panels
    fig.text((left + pw + gap / 2) / W, 0.05 / H,
             r"average load $\lambda$ and bucket size $Z$",
             ha="center", va="bottom", color=INK2, fontsize=6.5)
    blk = f"{S // 1024} KB" if S >= 1024 else f"{S} B"
    fig.text(left / W - 0.03, (H - 0.10) / H, f"{blk} slots, N = $2^{{{args.logn}}}$, same load and bucket size",
             ha="left", va="center", color=INK, fontsize=6.5)
    handles = [Line2D([], [], color=cd, lw=1.3, marker="o", ms=2.2, label="DOMA: 1 bucket per request"),
               Line2D([], [], color=cb, lw=1.3, marker="o", ms=2.2, label="BOLT: 2 buckets per request")]
    fig.legend(handles=handles, loc="center left", bbox_to_anchor=(left / W - 0.03, (H - 0.26) / H), ncol=2,
               frameon=False, fontsize=6.5, handlelength=1.2, handletextpad=0.35, columnspacing=0.8,
               borderaxespad=0, borderpad=0)
    save(fig, plt, args.out)


if __name__ == "__main__":
    sys.exit(main())
