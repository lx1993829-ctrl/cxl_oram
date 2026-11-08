#!/usr/bin/env python3
"""margins.py TRACE.csv [...] -- the card's per-access stage times from the 4 KB serial TRACE=1 bitstream (~/optarama_tr),
and what they allow for the overlap design's start spacing P (option 1) and read gate (option 2).

TRACE.csv: host_phases with DOMA_PHASE_TRACE=1 DOMA_TRACE_CSV=<file> (one line per command: phase, cmd, op, lat,
t0 .. t15, the bitstream's 16 phase timestamps in cycles since the request, docs/README_FPGA.md "Phase trace"):
t0 lookup entered, t1 read gate entered (lookup done), t2 bucket read request, t3 first bucket beat, t4 last bucket beat,
t8 re-encryption, t10 write-back state (re-encryption end; t11 if earlier), t11 / t12 first / last write-back beat.
An access is on package when it has no re-encryption (t10 - t8 < 100: on-package buckets are not re-encrypted).

Quantities per access (cycles):
  lookup      t1 - t0     position-map lookup (option 2: the read gate must cover it)
  read        t4 - t2     bucket read, request to last beat (one 4 KB bucket)
  first beat  t3 - t2
  after beat  t8 - t4     decryption check, block operation and eviction after the last beat (the main stage's own work)
  re-encrypt  enc - t8    off package only (enc = min(t10, t11) when t11 is set)
  write       t12 - t11   first to last write-back beat: how long one bucket holds the write path
Margins use the budgets' rule, ceil(worst x 1.10) + 74.

These are serial numbers: no other request shares the memories. In the overlap design a lookup and a read can wait
behind other requests' traffic, so every number here is a floor for the overlap design, never a proof that it is safe."""
import csv, math, sys
L, GATE, P_NOW, PM_BUDGET = 3634, 600, 1600, 250        # overlap context release, read gate, start spacing; serial lookup budget
BEAT_STREAM = 1024                                       # one 4 KB bucket = 1,024 beats through the main stage
def M(x): return math.ceil(x * 1.10) + 74
def pct(v, p):
    v = sorted(v); return v[min(len(v) - 1, max(0, math.ceil(p * len(v)) - 1))] if v else None
Q = ("lookup", "read", "first beat", "after beat", "re-encrypt", "write")
D = {c: {q: [] for q in Q} for c in ("off package", "on package")}
n = bad = 0
for f in sys.argv[1:]:
    for r in csv.DictReader(open(f)):
        t = [int(r["t%d" % k]) for k in range(16)]
        if 0 in (t[0], t[1], t[2], t[3], t[4], t[8], t[10], t[11], t[12]) or 65535 in t[:13]: bad += 1; continue
        enc = t[11] if t[10] > t[11] else t[10]
        c = D["on package" if t[10] - t[8] < 100 else "off package"]; n += 1
        c["lookup"].append(t[1] - t[0]); c["read"].append(t[4] - t[2]); c["first beat"].append(t[3] - t[2])
        c["after beat"].append(t[8] - t[4]); c["write"].append(t[12] - t[11])
        if t[10] - t[8] >= 100: c["re-encrypt"].append(enc - t[8])
if not n: sys.exit("no complete trace records: run host_phases with DOMA_PHASE_TRACE=1 DOMA_TRACE_CSV=<file> on a TRACE=1 bitstream")
print(f"{n} accesses ({bad} incomplete records skipped), {len(sys.argv) - 1} file(s)")
for c, d in D.items():
    k = len(d["read"])
    if not k: continue
    print(f"\n{c}: {k} accesses ({100 * k / n:.1f}%)        mean     p50    p99  p99.9 p99.99  worst")
    for q in Q:
        v = d[q]
        if v: print(f"  {q:12s} {sum(v) / len(v):17.1f} {pct(v, .5):7d} {pct(v, .99):6d} {pct(v, .999):6d} {pct(v, .9999):6d} {max(v):6d}")
allv = lambda q: D["off package"][q] + D["on package"][q]
lk, rd, ab, wr, en = allv("lookup"), allv("read"), allv("after beat"), D["off package"]["write"], D["off package"]["re-encrypt"]
wr_all = allv("write")
print("\nOption 2, the read gate (now %d cycles after the start):" % GATE)
print(f"  lookup worst {max(lk)} (serial budget {PM_BUDGET}); with the margin rule the gate could be {M(max(lk)) + 16} "
      f"(+16 as in the serial CT_RD = 16 + lookup budget), {GATE - M(max(lk)) - 16} cycles earlier than now -- if overlap traffic did not slow the lookup.")
print("\nOption 1, the start spacing P (now %d) and arrival spacing GA (now 1600):" % P_NOW)
if en: print(f"  re-encryption engine per off-package bucket: worst {max(en)} -> {M(max(en))} with the margin rule")
print(f"  write path per bucket: worst {max(wr_all)} (off package {max(wr) if wr else '-'}) -> {M(max(wr_all))} with the margin rule")
ew = max(M(max(en)) if en else 0, M(max(wr_all)))
mb = BEAT_STREAM + sum(ab) / len(ab)
print(f"  main stage per access, mean: {BEAT_STREAM} beats + {sum(ab) / len(ab):.0f} after the last beat = {mb:.0f}")
cms = {}
for g, name in ((GATE, "gate now"), (M(max(lk)) + 16, "gate from the lookup")):
    cms[name] = cm = (g + sum(rd) / len(rd) + L) / 4
    print(f"  four sets ({name}, {g}): ({g} + mean read {sum(rd) / len(rd):.0f} + L {L}) / 4 = {cm:.0f} per start on average")
print(f"\nSafety floor (encryption/write {ew}) and throughput limits (main stage {mb:.0f}, four sets), on these card numbers:")
for name, cm in cms.items():
    lo = max(ew, mb, cm)
    print(f"  {name:22s}: lowest useful P {lo:.0f} cycles, {300e3 / lo:.1f} KQPS at 300 MHz against {300e3 / P_NOW:.1f} now "
          f"({100 * (P_NOW / lo - 1):+.1f}%), set by {'encryption/write' if lo == ew else 'the main stage' if lo == mb else 'the four sets'}")
print("A floor only: the overlap design adds the other requests' traffic, so a lower P must still pass a build at that P "
      "(simulation regression, then the card runs with no halt).")
