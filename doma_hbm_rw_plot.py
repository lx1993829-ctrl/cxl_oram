#!/usr/bin/env python3
"""doma_hbm_rw_plot.py -- DOMA 4 KB: bucket read and bucket write cycles vs the share of buckets in HBM.

    python3 doma_hbm_rw_plot.py          # -> doma_hbm_rw.pdf / .png

Measured on the card: traced 4 KB bitstream with the 8 GiB HBM bucket layout (doma_big8.cfg), Z = 8, lambda = 6,
1M records, 500,000 random accesses per point (50% GET / 50% PUT), every GET checked. Mean cycles per access.
Everything you may want to change is in the SETTINGS block. Needs matplotlib.
"""
# =========================== SETTINGS: edit freely ===========================
HBM_PCT      = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
BUCKET_READ  = [1490.2, 1452.3, 1420.7, 1390.6, 1357.9, 1323.9, 1298.1, 1263.4, 1235.8, 1205.8, 1174.3]
BUCKET_WRITE = [1024.0, 1023.9, 1023.8, 1023.7, 1023.6, 1023.5, 1023.4, 1023.3, 1023.2, 1023.1, 1023.0]
LINES = [("Bucket read", BUCKET_READ, "#4a7fb5", "o"), ("Bucket write", BUCKET_WRITE, "#b9b3dc", "s")]
TITLE = ""                         # "" for no title
XLABEL = "Buckets in HBM (%)"
YLABEL = "Cycles per access"
YLIM = (0, 1750)                   # starts at 0 so the change is shown to scale
OUT = "doma_hbm_rw"
FIGSIZE = (4.2, 2.6)
FONT = 9
LABEL_ENDS = True                  # print the values at 0% and 100%
# ============================================================================


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": FONT, "pdf.fonttype": 42, "ps.fonttype": 42})
    fig, ax = plt.subplots(figsize=FIGSIZE)
    for name, ys, colour, marker in LINES:
        ax.plot(HBM_PCT, ys, marker=marker, color=colour, markeredgecolor="black", markeredgewidth=0.5,
                linewidth=1.6, markersize=4.5, label=name, zorder=3)
        if LABEL_ENDS:
            for x, y, ha in ((HBM_PCT[0], ys[0], "left"), (HBM_PCT[-1], ys[-1], "right")):
                ax.annotate(f"{int(y):,}", (x, y), textcoords="offset points", xytext=(0, 7), ha=ha, fontsize=FONT - 1)
    ax.set_xlim(-3, 103)
    ax.set_ylim(*YLIM)
    ax.set_xticks(HBM_PCT)
    ax.set_xlabel(XLABEL)
    ax.set_ylabel(YLABEL)
    ax.grid(linewidth=0.4, color="#cccccc", zorder=0)
    ax.legend(frameon=False, loc="lower left")
    if TITLE:
        ax.set_title(TITLE, fontsize=FONT, fontweight="bold")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(f"{OUT}.{ext}", dpi=200)
    print(f"wrote {OUT}.pdf and {OUT}.png")


if __name__ == "__main__":
    main()
