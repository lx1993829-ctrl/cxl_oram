#!/usr/bin/env python3
"""
plot_scaling.py — 2x2 subplot: E2E, RTL, DDR_READ, DDR_WRITE scaling.

Usage:  python3 plot_scaling.py m5out/scaling_sweep/results.csv
"""
import sys, csv, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

TYPES = {
    'hbm2':        {'label': 'Local HBM2 (8 ctrl)',              'color': '#2ecc71', 'marker': 'o'},
    'lpddr5_1x16': {'label': 'Local LPDDR5 1×16 (8 ctrl)',       'color': '#27ae60', 'marker': 's'},
    'lpddr5_2x16': {'label': 'Local LPDDR5 2×16 (8 ctrl)',       'color': '#1abc9c', 'marker': '^'},
    'cxl':         {'label': 'CXL (DDR5-6400, 16 ch)',           'color': '#3498db', 'marker': 'o'},
    'pcie':        {'label': 'PCIe (DDR5-6400, 16 ch)',          'color': '#e74c3c', 'marker': 'o'},
}
PLOT_ORDER = ['hbm2', 'lpddr5_1x16', 'lpddr5_2x16', 'cxl', 'pcie']
METRICS = [
    ('e2e_cyc',       'E2E Avg Cycles / Op'),
    ('rtl_cyc',       'RTL Avg Cycles / Op'),
    ('ddr_read_cyc',  'DDR_READ Cycles / Op'),
    ('ddr_write_cyc', 'DDR_WRITE Cycles / Op'),
]

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <results.csv>"); sys.exit(1)

    csv_path = sys.argv[1]
    out_dir = os.path.dirname(csv_path) or '.'

    # Parse
    data = {}  # type -> {metric -> [(N, val)]}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            t = row['type']
            n = int(row['N'])
            for col, _ in METRICS:
                v = row.get(col, 'NA')
                if v == 'NA' or v == '':
                    continue
                data.setdefault(t, {}).setdefault(col, []).append((n, float(v)))

    # Sort
    for t in data:
        for col in data[t]:
            data[t][col].sort()

    # Plot 2x2
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Instance Scaling: Local LPDDR5 vs HBM2 vs PCIe vs CXL',
                 fontsize=16, fontweight='bold', y=0.98)

    for idx, (col, title) in enumerate(METRICS):
        ax = axes[idx // 2][idx % 2]
        for t in PLOT_ORDER:
            if t not in data or col not in data[t]:
                continue
            cfg = TYPES.get(t, {'label': t, 'color': 'gray', 'marker': 'x'})
            ns = [p[0] for p in data[t][col]]
            vals = [p[1] for p in data[t][col]]
            ax.plot(ns, vals, marker=cfg['marker'], markersize=6,
                    linewidth=2, color=cfg['color'], label=cfg['label'])
        ax.set_xlabel('N', fontsize=11)
        ax.set_ylabel('Cycles', fontsize=11)
        ax.set_title(title, fontsize=13, fontweight='bold')
        ax.set_xticks(range(1, 9))
        ax.set_xlim(0.5, 8.5)
        ax.grid(True, alpha=0.3, linestyle='--')
        if idx == 0:
            ax.legend(fontsize=8, loc='upper left')

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    for ext in ['png', 'pdf']:
        p = os.path.join(out_dir, f'scaling_plot.{ext}')
        fig.savefig(p, dpi=200)
        print(f"Saved: {p}")

if __name__ == '__main__':
    main()
