#!/usr/bin/env python3
"""
plot_local_pct.py — 1x2 subplot: Mem READ and Mem Write vs local_pct.

Usage:  python3 plot_local_pct.py m5out/local_pct_sweep/results.csv
"""
import sys, csv, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FABRICS = {
    'pcie': {'label': 'PCIe', 'color': '#e74c3c', 'marker': 'o'},
    'cxl':  {'label': 'CXL',  'color': '#3498db', 'marker': 's'},
}

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <results.csv>"); sys.exit(1)

    csv_path = sys.argv[1]
    out_dir = os.path.dirname(csv_path) or '.'

    # Parse: fabric -> {metric -> [(pct, val)]}
    data = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            fab = row['fabric']
            pct = int(row['local_pct'])
            for col in ['ddr_read_cyc', 'ddr_write_cyc']:
                v = row.get(col, 'NA')
                if v == 'NA' or v == '':
                    continue
                data.setdefault(fab, {}).setdefault(col, []).append((pct, float(v)))

    for fab in data:
        for col in data[fab]:
            data[fab][col].sort()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle('Memory Cycles (N=1)', fontsize=15, fontweight='bold', y=1.02)

    # (a) Mem READ
    for fab in ['pcie', 'cxl']:
        if fab in data and 'ddr_read_cyc' in data[fab]:
            cfg = FABRICS[fab]
            pts = data[fab]['ddr_read_cyc']
            ax1.plot([p[0] for p in pts], [p[1] for p in pts],
                     marker=cfg['marker'], markersize=7, linewidth=2.2,
                     color=cfg['color'], label=cfg['label'])
    ax1.set_xlabel('Local Mem Percentage (%)', fontsize=12)
    ax1.set_ylabel('Avg Mem Read Cycles / Op', fontsize=12)
    ax1.set_title('(a) Mem READ', fontsize=13, fontweight='bold')
    ax1.set_xticks(range(0, 110, 10))
    ax1.set_xlim(-5, 105)
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.legend(fontsize=11)

    # (b) Mem Write
    for fab in ['pcie', 'cxl']:
        if fab in data and 'ddr_write_cyc' in data[fab]:
            cfg = FABRICS[fab]
            pts = data[fab]['ddr_write_cyc']
            ax2.plot([p[0] for p in pts], [p[1] for p in pts],
                     marker=cfg['marker'], markersize=7, linewidth=2.2,
                     color=cfg['color'], label=cfg['label'])
    ax2.set_xlabel('Local Memory Percentage (%)', fontsize=12)
    ax2.set_ylabel('Avg Mem Write Cycles', fontsize=12)
    ax2.set_title('(b) Mem Write', fontsize=13, fontweight='bold')
    ax2.set_xticks(range(0, 110, 10))
    ax2.set_xlim(-5, 105)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.legend(fontsize=11)

    plt.tight_layout()
    for ext in ['png', 'pdf']:
        p = os.path.join(out_dir, f'local_pct_plot.{ext}')
        fig.savefig(p, dpi=200, bbox_inches='tight')
        print(f"Saved: {p}")

if __name__ == '__main__':
    main()
