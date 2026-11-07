#!/usr/bin/env python3
# sweep_table.py -- the sweep table from ~/sweep_results/*.log (sweep_rama.sh's runs), with KQPS, us per command and
# device cycles per command; writes ~/sweep_results/sweep_fixed.csv
import glob, os, re
R = os.path.expanduser('~/sweep_results')
rows = []
for f in glob.glob(R + '/*_v*_n*.log'):
    m = re.match(r'.*/([NV])_v(\d+)_n(\d+)\.log$', f)
    if not m: continue
    sw, v, n = m[1], int(m[2]), int(m[3]); t = open(f, errors='ignore').read()
    ph = re.search(r'^phase 1: \d+ MIX in \d+ launch\w*, [\d.]+ s = ([\d.]+) KQPS, ([\d.]+) us per command; device ([\d.]+) cycles per command, errors (0x\w+)', t, re.M)
    mx = re.search(r'^phase 1: mix of (\d+) GET \(mean latency ([\d.]+) cycles\) and (\d+) PUT \(mean latency ([\d.]+) cycles\)', t, re.M)
    lt = re.search(r'^phase 1: latency min (\d+), p50 (\d+), p90 (\d+), p99 (\d+), p99\.9 (\d+), max (\d+)', t, re.M)
    blk = re.search(r'^block: (\d+) bytes', t, re.M)
    ok = 'yes' if re.search(r'^ALL PHASES CORRECT', t, re.M) else 'NO'
    rows.append(dict(sweep=sw, value_B=v, block_B=blk[1] if blk else '?', N=n,
                     KQPS=ph[1] if ph else '?', us_per_cmd=ph[2] if ph else '?', device_cyc=ph[3] if ph else '?', errors=ph[4] if ph else '?',
                     gets=mx[1] if mx else '?', get_lat=mx[2] if mx else '?', puts=mx[3] if mx else '?', put_lat=mx[4] if mx else '?',
                     p50=lt[2] if lt else '?', p99=lt[4] if lt else '?', max=lt[6] if lt else '?', correct=ok))
rows.sort(key=lambda r: (r['sweep'], r['value_B'], r['N']))
cols = list(rows[0].keys()) if rows else []
with open(R + '/sweep_fixed.csv', 'w') as o:
    o.write(','.join(cols) + '\n')
    for r in rows: o.write(','.join(str(r[c]) for c in cols) + '\n')
for sw, title in (('N', 'Sweep 1: records N, 64 B blocks'), ('V', 'Sweep 2: value size, N = 1M (16 B runs on the 32 B block)')):
    print(f"\n{title}  (1M commands, 50/50 GET/PUT, 20% in HBM)")
    print(f"{'value':>7} {'block':>6} {'N':>9} {'KQPS':>8} {'us/cmd':>7} {'dev cyc':>8} {'GET lat':>8} {'PUT lat':>8} {'p50':>6} {'p99':>6} {'max':>6}  correct")
    for r in rows:
        if r['sweep'] != sw: continue
        print(f"{r['value_B']:>5} B {r['block_B']:>4} B {r['N']:>9} {r['KQPS']:>8} {r['us_per_cmd']:>7} {r['device_cyc']:>8} {r['get_lat']:>8} {r['put_lat']:>8} {r['p50']:>6} {r['p99']:>6} {r['max']:>6}  {r['correct']}")
print(f"\n(latencies in cycles at 300 MHz; table also in {R}/sweep_fixed.csv)")
