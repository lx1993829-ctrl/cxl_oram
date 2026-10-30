#!/usr/bin/env python3
"""ycsb_latency_analyze.py -- analyze the YCSB per-request latency histograms (Python standard library only).

    python3 ycsb_latency_analyze.py ycsb_hist_p1.csv ycsb_hist_p2.csv
    python3 ycsb_latency_analyze.py --by-op ycsb_hist_p1.csv     # strip data per workload AND operation

Input: the histograms host_ycsb writes with DOMA_LAT_HIST=<file> (rows: workload,op,latency_cycles,count), one file
per pass, in pass order. Prints:
  1. each workload's distribution, GET and PUT pooled, both passes pooled: percentiles, mean, standard deviation;
  2. the same per pass, to show the two passes agree;
  3. the largest gap between workloads at each percentile, in cycles;
  4. a coarse histogram of every workload side by side (counts per bin of BIN cycles), to show the shape and modes;
  5. per workload, GET against PUT: percentile differences in cycles and the Kolmogorov-Smirnov distance;
  6. per workload, GET against PUT split at SPLIT cycles: the share of each in the fast group (with a z-score),
     and within each group the mean difference and the KS distance.
  7. the strip plot's data, compact enough to copy from the terminal (per workload: the body as evenly spaced
     quantiles up to TAIL_Q, and the requests beyond TAIL_Q counted in BIN-cycle bins, with the exact maximum).
"""
import csv
import math
import sys
from collections import defaultdict

WORKLOADS = ["A", "B", "C", "D", "E", "F"]
PCTS = [0.0001, 0.001, 0.01, 0.10, 0.50, 0.90, 0.99, 0.999, 0.9999]
BIN = 10                     # cycles per bin of the text histogram
BODY_DOTS = 200              # strip plot: body quantiles per workload
TAIL_Q = 0.999               # strip plot: every request beyond this quantile is kept individually
SPLIT = 4950                 # cycles: requests below are the fast group (on-package buckets), the rest host buckets


def load(paths):
    h = defaultdict(lambda: defaultdict(int))        # (pass, workload, op) -> {latency: count}
    for k, p in enumerate(paths, 1):
        with open(p) as f:
            for r in csv.DictReader(f):
                h[(k, r["workload"], r["op"])][int(r["latency_cycles"])] += int(r["count"])
    return h


def merge(*hs):
    out = defaultdict(int)
    for x in hs:
        for k, v in x.items():
            out[k] += v
    return out


class Dist:
    def __init__(self, h):
        self.items = sorted(h.items())
        self.n = sum(c for _, c in self.items)
        self.cum = []
        acc = 0
        for _, c in self.items:
            acc += c
            self.cum.append(acc)

    def q(self, f):
        target = f * (self.n - 1)
        lo, hi = 0, len(self.cum) - 1
        while lo < hi:                                  # first index whose cumulative count covers the target
            mid = (lo + hi) // 2
            if self.cum[mid] - 1 >= target:
                hi = mid
            else:
                lo = mid + 1
        return self.items[lo][0]

    def mean(self):
        return sum(x * c for x, c in self.items) / self.n

    def std(self):
        m = self.mean()
        return math.sqrt(sum(c * (x - m) ** 2 for x, c in self.items) / self.n)


def ks(a, b):
    xs = sorted(set(x for x, _ in a.items) | set(x for x, _ in b.items))
    da, db = dict(a.items), dict(b.items)
    ca = cb = 0
    d = 0.0
    for x in xs:
        ca += da.get(x, 0)
        cb += db.get(x, 0)
        d = max(d, abs(ca / a.n - cb / b.n))
    return d, 1.358 * math.sqrt((a.n + b.n) / (a.n * b.n))


def pct_name(f):
    return "p" + ("%g" % (100 * f))


def row(name, d):
    return (f"{name:8}{d.n:>9}{d.items[0][0]:>7}" + "".join(f"{d.q(f):>8}" for f in PCTS) +
            f"{d.items[-1][0]:>7}{d.mean():>9.1f}{d.std():>7.1f}")


def header(first):
    return f"{first:8}{'n':>9}{'min':>7}" + "".join(f"{pct_name(f):>8}" for f in PCTS) + f"{'max':>7}{'mean':>9}{'std':>7}"


def main(paths):
    h = load(paths)
    passes = sorted(set(k for k, _, _ in h))
    wl = [w for w in WORKLOADS if any((k, w, op) in h for k in passes for op in ("GET", "PUT"))]
    pooled = {w: Dist(merge(*[h[(k, w, op)] for k in passes for op in ("GET", "PUT") if (k, w, op) in h])) for w in wl}

    print(f"== 1. per-request latency (cycles at 300 MHz), GET and PUT pooled, passes pooled ({', '.join(paths)})")
    print(header("workload"))
    for w in wl:
        print(row(w, pooled[w]))

    print("\n== 2. per pass")
    print(header("wl/pass"))
    for w in wl:
        for k in passes:
            parts = [h[(k, w, op)] for op in ("GET", "PUT") if (k, w, op) in h]
            if parts:
                print(row(f"{w} p{k}", Dist(merge(*parts))))

    print("\n== 3. largest gap between workloads at each percentile (cycles), pooled distributions")
    print("   " + ", ".join(f"{pct_name(f)} {max(pooled[w].q(f) for w in wl) - min(pooled[w].q(f) for w in wl)}"
                            for f in PCTS))
    print("   means: " + ", ".join(f"{w} {pooled[w].mean():.1f}" for w in wl) +
          f"; spread {max(pooled[w].mean() for w in wl) - min(pooled[w].mean() for w in wl):.1f}")

    allp = Dist(merge(*[dict(pooled[w].items) for w in wl]))
    lo = (allp.q(0.0001) // BIN) * BIN
    hi = (allp.q(0.9999) // BIN + 1) * BIN
    print(f"\n== 4. histogram, counts per {BIN}-cycle bin from {lo} to {hi} (outside: below / above), pooled")
    print(f"{'bin':>11}" + "".join(f"{w:>9}" for w in wl))
    below = {w: sum(c for x, c in pooled[w].items if x < lo) for w in wl}
    above = {w: sum(c for x, c in pooled[w].items if x >= hi) for w in wl}
    print(f"{'< ' + str(lo):>11}" + "".join(f"{below[w]:>9}" for w in wl))
    for b in range(int(lo), int(hi), BIN):
        counts = [sum(c for x, c in pooled[w].items if b <= x < b + BIN) for w in wl]
        print(f"{b:>6}-{b + BIN - 1:<4}" + "".join(f"{c:>9}" for c in counts))
    print(f"{'>= ' + str(hi):>11}" + "".join(f"{above[w]:>9}" for w in wl))

    print("\n== 5. GET against PUT per workload, passes pooled: percentile difference PUT - GET (cycles), KS distance")
    for w in wl:
        g = [h[(k, w, "GET")] for k in passes if (k, w, "GET") in h]
        p = [h[(k, w, "PUT")] for k in passes if (k, w, "PUT") in h]
        if not g or not p:
            print(f"   {w}: only {'GETs' if g else 'PUTs'}, no comparison")
            continue
        G, P = Dist(merge(*g)), Dist(merge(*p))
        d, c = ks(G, P)
        diffs = ", ".join(f"{pct_name(f)} {P.q(f) - G.q(f):+d}" for f in (0.01, 0.10, 0.50, 0.90, 0.99, 0.999, 0.9999))
        print(f"   {w}: GET n={G.n}, PUT n={P.n}; mean {P.mean() - G.mean():+.2f}; {diffs}; "
              f"KS D = {d:.5f} (5% critical {c:.5f}: {'same' if d < c else 'differ'})")

    print(f"\n== 6. GET against PUT, split at {SPLIT} cycles (fast group: on-package buckets), passes pooled")
    for w in wl:
        g = [h[(k, w, "GET")] for k in passes if (k, w, "GET") in h]
        p = [h[(k, w, "PUT")] for k in passes if (k, w, "PUT") in h]
        if not g or not p:
            continue
        G, P = merge(*g), merge(*p)
        nG, nP = sum(G.values()), sum(P.values())
        fG = sum(c for x, c in G.items() if x < SPLIT) / nG
        fP = sum(c for x, c in P.items() if x < SPLIT) / nP
        f0 = (fG * nG + fP * nP) / (nG + nP)
        z = (fP - fG) / math.sqrt(f0 * (1 - f0) * (1 / nG + 1 / nP)) if 0 < f0 < 1 else 0.0
        parts = []
        for name, keep in (("fast", lambda x: x < SPLIT), ("slow", lambda x: x >= SPLIT)):
            g2 = {x: c for x, c in G.items() if keep(x)}
            p2 = {x: c for x, c in P.items() if keep(x)}
            if g2 and p2:
                Dg, Dp = Dist(g2), Dist(p2)
                d, c = ks(Dg, Dp)
                parts.append(f"{name}: mean {Dp.mean() - Dg.mean():+.2f}, p50 {Dp.q(0.5) - Dg.q(0.5):+d}, "
                             f"p99 {Dp.q(0.99) - Dg.q(0.99):+d}, KS D = {d:.5f} (crit {c:.5f}: {'same' if d < c else 'differ'})")
        print(f"   {w}: fast-group share GET {100 * fG:.3f}%, PUT {100 * fP:.3f}% (z = {z:+.1f}); " + "; ".join(parts))

    # 7. the strip plot's data, printed compactly so it can be copied from the terminal:
    #    body = BODY_DOTS quantiles up to TAIL_Q (value, or value*repeats); tail = requests beyond TAIL_Q counted in
    #    BIN-cycle bins (bin start*count); max = the largest latency
    print(f"\n== 7. strip-plot data (copy everything from BEGIN to END)")
    groups = [(w, None) for w in wl]
    if BY_OP:                                          # one block per workload and operation, 100 quantiles each
        groups = [(w, op) for w in wl for op in ("GET", "PUT")
                  if any((k, w, op) in h for k in passes)]
    nq = 100 if BY_OP else BODY_DOTS
    print(f"BEGIN STRIP body={nq} q={TAIL_Q} bin={BIN} files={','.join(paths)}{' by-op' if BY_OP else ''}")
    for w, op in groups:
        d = pooled[w] if op is None else Dist(merge(*[h[(k, w, op)] for k in passes if (k, w, op) in h]))
        tag = w if op is None else f"{w}/{op}"
        body = [d.q(TAIL_Q * (i + 0.5) / nq) for i in range(nq)]
        rle, i = [], 0
        while i < len(body):
            j = i
            while j < len(body) and body[j] == body[i]:
                j += 1
            rle.append(f"{body[i]}" if j - i == 1 else f"{body[i]}*{j - i}")
            i = j
        cut = d.q(TAIL_Q)
        bins = defaultdict(int)
        for x, c in d.items:
            if x > cut:
                bins[(x // BIN) * BIN] += c
        print(f"{tag} n={d.n} cut={cut} max={d.items[-1][0]}")
        print(f"{tag} body " + " ".join(rle))
        print(f"{tag} tail " + " ".join(f"{x}*{c}" if c > 1 else f"{x}" for x, c in sorted(bins.items())))
    print("END STRIP")

if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if a != "--by-op"]
    BY_OP = "--by-op" in sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    main(args)
