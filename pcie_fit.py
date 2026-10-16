#!/usr/bin/env python3
"""pcie_fit.py -- fit a pipelined PCIe host-memory model to the latency kernel's calibration sweep.

Input: the sweep's CSV files (LAT_CSV) or its printed output (.txt), one or more:
    python3 pcie_fit.py pcie_cold.csv pcie_dual.csv [--tdram 114] [--sm-read-ns 908] [--sm-wack-ns 321]

It fits a few physical parameters, then predicts every measurement from them with a pipelined model (each stage
has a latency and a throughput limit: throughput grows linearly with requests in flight until the first limit
binds; latency stays flat until then and grows by queueing after) and prints the error of each prediction.
Rows the model misses point at a missing mechanism, not just a wrong number.

Fitted:
  A        unloaded 32-byte read (ns)                           dependent read, 32 B
  BW1      streaming rate within one 4 KB burst (GB/s)          dependent reads, 256 B .. 4 KB
  BW2      streaming rate across back-to-back bursts (GB/s)     dependent reads, 4 KB -> 32 KB
  W0       write acknowledgement of one beat (ns)               write ack, 32 B
  LINK_R   read plateau (GB/s) -> completion size               independent reads, 4 KB, largest K, most instances
  LINK_W   write plateau (GB/s) -> write TLP / header size      independent writes, 4 KB, likewise
  NR       reads accepted in flight per instance                Little's law at the single-instance 256 B plateau
  RRATE    aggregate small-read rate (reads/s)                  independent reads, 256 B, most instances
  WGAP     idle cycles per write burst at the bridge's input    single-instance 256 B write plateau
Fixed by the build: the kernel's port (32 bytes x clock).
"""
import argparse, csv, math, re, sys

LINK_RAW = 15.754          # Gen3 x16 after 128b/130b encoding, GB/s
KOVH = 2                   # cycles the kernel adds to a dependent sample

def parse(path):
    rows, mhz = [], None
    if path.endswith('.csv'):
        for r in csv.DictReader(open(path)):
            rows.append(dict(kind=r['kind'], inst=int(r['instances']), S=int(r['bytes']), K=int(r['outstanding']),
                             avg=float(r['avg_cyc']), mn=float(r['min_cyc']), mx=float(r['max_cyc']),
                             gbps=float(r['GBps']), mhz=float(r['MHz'])))
        return rows
    lines = open(path).read().splitlines()
    for i, l in enumerate(lines):
        m = re.search(r'kernel clock assumed ([\d.]+) MHz', l)
        if m: mhz = float(m.group(1)); continue
        m = re.match(r'\s*(dependent read|write ack|indep read|indep write) (\d+) B(?:, K=(\d+))?( x2)?\s+(\d+) samples\s+'
                     r'min\s+([\d.]+)\s+avg\s+([\d.]+)\s+max\s+([\d.]+)', l)
        if not m: continue
        kind = {'dependent read': 'dep_read', 'write ack': 'write_ack', 'indep read': 'ind_read', 'indep write': 'ind_write'}[m.group(1)]
        gb = 0.0
        for j in range(i + 1, min(i + 3, len(lines))):
            g = re.match(r'\s*([\d.]+) GB/s', lines[j])
            if g: gb = float(g.group(1)); break
        rows.append(dict(kind=kind, inst=2 if m.group(4) else 1, S=int(m.group(2)), K=int(m.group(3) or 1),
                         avg=float(m.group(7)), mn=float(m.group(6)), mx=float(m.group(8)), gbps=gb, mhz=mhz or 300.0))
    return rows

def ns(r, field='avg'):
    v = r[field] - (KOVH if r['kind'] == 'dep_read' else 0)
    return v * 1e3 / r['mhz']

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('files', nargs='+')
    ap.add_argument('--tdram', type=float, default=114.0, help='host DRAM latency in ns (mem_lat plateau)')
    ap.add_argument('--sm-read-ns', type=float, default=908.0, help='cold 32 B read average, single-master build (ns)')
    ap.add_argument('--sm-wack-ns', type=float, default=321.0, help='32 B write ack, single-master build (ns)')
    ap.add_argument('--bridge-ns', type=float, default=15.0, help='request-path bridge delay assumed in the model (ns)')
    a = ap.parse_args()
    rows = []
    for f in a.files: rows += parse(f)
    if not rows: sys.exit("no measurements found")
    mhz = rows[0]['mhz']; PORT = 32 * mhz / 1e3                                  # GB/s per instance
    dep = {}
    for r in rows:
        if r['kind'] == 'dep_read': dep.setdefault(r['S'], []).append(ns(r))
    dep = {S: sum(v) / len(v) for S, v in dep.items()}
    wack = {}
    for r in rows:
        if r['kind'] == 'write_ack': wack.setdefault(r['S'], []).append(ns(r))
    wack = {S: sum(v) / len(v) for S, v in wack.items()}
    def series(kind, S, inst): return sorted([r for r in rows if r['kind'] == kind and r['S'] == S and r['inst'] == inst], key=lambda r: r['K'])
    maxinst = max(r['inst'] for r in rows)

    # ---------------- fit ----------------
    A = dep[32]
    pts = [(S - 32, dep[S] - A) for S in (256, 1024, 4096) if S in dep]
    BW1 = sum(x * x for x, _ in pts) / sum(x * y for x, y in pts)                 # bytes per ns = GB/s
    BW2 = (32768 - 4096) / (dep[32768] - dep[4096]) if 32768 in dep and 4096 in dep else PORT
    def L_lin(S):                                                                 # the per-byte model (gem5 mapping)
        return A + (min(S, 4096) - 32) / BW1 + max(0, S - 4096) / BW2
    sizes = sorted(dep)
    def L_read(S):                                                                # the measured curve, interpolated
        if S in dep: return dep[S]
        lo = max([x for x in sizes if x < S], default=None); hi = min([x for x in sizes if x > S], default=None)
        if lo is None or hi is None: return L_lin(S)
        f = (math.log(S) - math.log(lo)) / (math.log(hi) - math.log(lo))
        return dep[lo] + f * (dep[hi] - dep[lo])
    W0 = wack[32]
    LINK_R = max(r['gbps'] for r in series('ind_read', 4096, maxinst))
    LINK_W = max(r['gbps'] for r in series('ind_write', 4096, maxinst))
    def W_ack(S):
        return W0 + (S - 32) / PORT + (S - 32) / LINK_W                          # beats out of the kernel, then the link
    s1 = series('ind_read', 256, 1)[-1]
    NRS = {S: series('ind_read', S, 1)[-1]['gbps'] * ns(series('ind_read', S, 1)[-1]) / S for S in (256, 4096)}
    NR = NRS[256]                                                                # Little's law, per instance
    RRATE_I = s1['gbps'] * 1e9 / 256                                             # one instance's small-read rate
    r4 = series('ind_read', 4096, 1)[-1]
    RGAP = (4096 / r4['gbps']) * mhz / 1e3 - 4096 / 32                            # idle cycles per 4 KB read burst
    RRATE = max(r['gbps'] for r in series('ind_read', 256, maxinst)) * 1e9 / 256
    w1 = series('ind_write', 256, 1)[-1]
    WGAP = (256 / w1['gbps']) * mhz / 1e3 - 256 / 32                              # cycles per write minus its beats
    WRATE = max(r['gbps'] for r in series('ind_write', 256, maxinst)) * 1e9 / 256   # aggregate small-write rate
    NW = {S: series('ind_write', S, 1)[-1]['gbps'] * ns(series('ind_write', S, 1)[-1]) / S for S in (256, 4096)}

    comp = min((64, 128, 256), key=lambda p: abs(LINK_RAW * p / (p + 20) - LINK_R))
    hdr = min((12, 16), key=lambda h: abs(LINK_RAW * 256 / (256 + h + 8) - LINK_W))
    print(f"clock {mhz:.0f} MHz: one instance's port carries {PORT:.2f} GB/s; Gen3 x16 raw {LINK_RAW} GB/s\n")
    print("FITTED PARAMETERS")
    print(f"  A      unloaded 32 B read                    {A:8.1f} ns")
    print(f"  BW1    streaming within one 4 KB burst       {BW1:8.2f} GB/s   (dependent 256 B .. 4 KB)")
    print(f"  BW2    streaming across back-to-back bursts  {BW2:8.2f} GB/s   (4 KB -> 32 KB; port is {PORT:.2f})")
    print(f"  W0     write acknowledgement of one beat     {W0:8.1f} ns")
    print(f"  LINK_R read plateau                          {LINK_R:8.2f} GB/s   -> {comp}-byte completions "
          f"(theory {LINK_RAW * comp / (comp + 20):.2f}; 64/128/256 B: " + "/".join(f"{LINK_RAW*p/(p+20):.1f}" for p in (64,128,256)) + ")")
    print(f"  LINK_W write plateau                         {LINK_W:8.2f} GB/s   -> 256-byte writes with {hdr}-byte headers "
          f"(theory {LINK_RAW * 256 / (256 + hdr + 8):.2f})")
    print(f"  NR     reads accepted in flight / instance   256 B: {NRS[256]:.1f}, 4 KB: {NRS[4096]:.1f}")
    print(f"  RRATE1 one instance's small-read rate        {RRATE_I/1e6:8.1f} M reads/s  ({1e9/RRATE_I:.0f} ns per read)")
    print(f"  RGAP   idle cycles per 4 KB read burst       {RGAP:8.1f}")
    print(f"  RRATE  aggregate small-read rate             {RRATE/1e6:8.1f} M reads/s  ({1e9/RRATE:.0f} ns per read)")
    print(f"  WGAP   idle cycles per write burst           {WGAP:8.1f}")
    print(f"  WRATE  aggregate small-write rate            {WRATE/1e6:8.1f} M writes/s  ({1e9/WRATE:.0f} ns per write)")
    print(f"  NW     writes accepted in flight / instance  256 B: {NW[256]:.1f}, 4 KB: {NW[4096]:.1f}\n")

    # ---------------- predict ----------------
    def pred(r):
        S, K, inst = r['S'], r['K'], r['inst']
        if r['kind'] == 'dep_read':  return L_read(S), None
        if r['kind'] == 'write_ack': return W_ack(S), None
        if r['kind'] == 'ind_read':
            L0 = L_read(S)
            port_r = S / ((S / 32 + (RGAP if S >= 4096 else 0)) * 1e3 / mhz)
            cap_i = min(port_r, RRATE_I * S / 1e9)
            thr = min(inst * K * S / L0, inst * cap_i, LINK_R, RRATE * S / 1e9)
            lat = max(L0, inst * min(K, NRS.get(S, NR)) * S / thr)
            return lat, thr
        L0 = W_ack(S)
        port_w = S / ((S / 32 + WGAP) * 1e3 / mhz)
        thr = min(inst * K * S / L0, inst * port_w, LINK_W, WRATE * S / 1e9)
        nw = NW[S] if S in NW else NW[256]
        lat = max(L0, inst * min(K, nw) * S / thr)
        return lat, thr
    print("PREDICTIONS (pipelined model with the parameters above) vs MEASURED")
    print(f"  {'measurement':30s} {'latency ns: meas':>16s} {'pred':>7s} {'err':>6s}   {'GB/s: meas':>10s} {'pred':>6s} {'err':>6s}")
    worst = []
    for r in rows:
        lat_p, thr_p = pred(r)
        lm = ns(r); el = 100 * (lat_p - lm) / lm
        name = {'dep_read': 'dependent read', 'write_ack': 'write ack', 'ind_read': 'indep read', 'ind_write': 'indep write'}[r['kind']]
        label = f"{name} {r['S']} B" + (f", K={r['K']}" if r['kind'].startswith('ind') else '') + (' x2' if r['inst'] > 1 else '')
        line = f"  {label:30s} {lm:16.0f} {lat_p:7.0f} {el:+5.0f}%"
        if thr_p is not None:
            eb = 100 * (thr_p - r['gbps']) / r['gbps'] if r['gbps'] else 0
            line += f"   {r['gbps']:10.2f} {thr_p:6.2f} {eb:+5.0f}%"
            worst.append((abs(eb), label, 'GB/s', eb))
        worst.append((abs(el), label, 'latency', el))
        print(line)
    worst.sort(reverse=True)
    print("  (single-instance plateaus fit their own caps; the two-instance rows and the unsaturated rows are the test)")
    print("\nLARGEST DEVIATIONS (a mechanism the model lacks, if large):")
    for w in worst[:6]: print(f"  {w[1]:30s} {w[2]:8s} {w[3]:+.0f}%")

    # ---------------- gem5 mapping ----------------
    off = A - a.sm_read_ns
    lfix = a.sm_read_ns - a.tdram - 11.5
    print("\nFOR THE gem5 MODEL (single-master anchors; this build adds about %.0f ns to a read)" % off)
    print(f"  fixed read round trip L_fix = {a.sm_read_ns:.0f} - t_DRAM {a.tdram:.0f} - 11.5 (serialization, CDC) = {lfix:.0f} ns")
    print(f"    folded into one pipelined stage per direction: {lfix/2:.0f} ns each way "
          f"(or rc_latency = {(lfix - a.bridge_ns)/2:.0f} ns with bridge_pipeline_delay = {a.bridge_ns:.0f} ns)")
    print(f"    fit it to the MINIMUM instead if your DRAM model supplies the variability (cold 32 B min, single master: ~760 ns)")
    print(f"  write response: {a.sm_wack_ns:.0f} ns fixed for one beat, plus the burst's beats and its link serialization "
          f"(4 KB predicted {W_ack(4096) - off:.0f} ns single-master)")
    print(f"  completion size {comp} B; mps 256; use_64bit_addr {'True' if hdr == 16 else 'False'}")
    print(f"  reads accepted in flight per port ~{NRS[256]:.0f} (256 B) / ~{NRS[4096]:.0f} (4 KB): per request, size-dependent")
    print(f"  read request service: {1e9/RRATE_I:.0f} ns per port, {1e9/RRATE:.0f} ns aggregate; {RGAP:.0f} idle cycles per 4 KB read burst")
    print(f"  streaming: {BW1:.1f} GB/s within a 4 KB burst; bursts overlap ({BW2:.1f} GB/s across bursts)")
    print("  latency against size is concave: per byte it misses " + ", ".join(
        f"{S} B by {100*(L_lin(S)-dep[S])/dep[S]:+.0f}%" for S in sizes if abs(L_lin(S)-dep[S])/dep[S] > 0.02) +
        " (more cost per byte at small sizes)")
    print("  RC stage rule: exit = max(arrive + latency, previous exit + throughput); latency is flat below saturation")
    print(f"  write request service: {1e9/WRATE:.0f} ns aggregate; writes accepted ahead of their data: ~{NW[256]:.0f} per port when the")
    print(f"    link limits (two instances), ~{NW[4096]:.0f} for 4 KB when the port limits: acceptance tracks the saturated stage")
    print("\nNOT COVERED BY THIS SWEEP: reads right after writes (PCIe ordering), host power states, Gen5/CXL terms.")

if __name__ == '__main__':
    main()
