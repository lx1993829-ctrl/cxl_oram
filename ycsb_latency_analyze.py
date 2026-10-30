#!/usr/bin/env python3
"""ycsb_latency_analyze.py -- analyze the YCSB per-request latency histograms (Python standard library only).

    python3 ycsb_latency_analyze.py ycsb_hist_p1.csv ycsb_hist_p2.csv

Input: the histograms host_ycsb writes with DOMA_LAT_HIST=<file> (rows: workload,op,latency_cycles,count), one file
per pass, in pass order. Prints:
  1. each workload's distribution, GET and PUT pooled, both passes pooled: percentiles, mean, standard deviation;
  2. the same per pass, to show the two passes agree;
  3. the largest gap between workloads at each percentile, in cycles;
  4. a coarse histogram of every workload side by side (counts per bin of BIN cycles), to show the shape and modes;
  5. per workload, GET against PUT: percentile differences in cycles and the Kolmogorov-Smirnov distance.
Writes ycsb_strip_data.txt, a small file with what the strip plot needs (per workload: the body as evenly spaced
quantiles up to TAIL_Q, and every request beyond TAIL_Q as latency x count). Send that file back for the figure.
"""
import csv
import math
import sys
from collections import defaultdict

WORKLOADS = ["A", "B", "C", "D", "E", "F"]
PCTS = [0.0001, 0.001, 0.01, 0.10, 0.50, 0.90, 0.99, 0.999, 0.9999]
BIN = 10                     # cycles per bin of the text histogram
BODY_DOTS = 400              # strip plot: body quantiles per workload
TAIL_Q = 0.999               # strip plot: every request beyond this quantile is kept individually
OUT = "ycsb_strip_data.txt"


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

    with open(OUT, "w") as f:
        f.write(f"# strip-plot data: per workload, body = {BODY_DOTS} quantiles up to q={TAIL_Q} (value or "
                f"value*repeats), tail = every request beyond it (latency*count); GET+PUT, passes pooled\n")
        for w in wl:
            d = pooled[w]
            body = [d.q(TAIL_Q * (i + 0.5) / BODY_DOTS) for i in range(BODY_DOTS)]
            rle, i = [], 0
            while i < len(body):
                j = i
                while j < len(body) and body[j] == body[i]:
                    j += 1
                rle.append(f"{body[i]}" if j - i == 1 else f"{body[i]}*{j - i}")
                i = j
            cut = d.q(TAIL_Q)
            tail = [f"{x}*{c}" if c > 1 else f"{x}" for x, c in d.items if x > cut]
            f.write(f"{w} n={d.n} body {' '.join(rle)}\n")
            f.write(f"{w} tail {' '.join(tail)}\n")
    print(f"\nwrote {OUT} (send it back for the strip plot)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
