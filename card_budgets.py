#!/usr/bin/env python3
"""card_budgets.py -- v14b/v14c: DOMA's on-package budgets (AR_WB_H, AR_REL_H) from the card's own per-access records.

    python3 card_budgets.py [options] RECORDS.csv [RECORDS.csv ...]

RECORDS.csv: the per-access records of a debug bitstream (build.sh DEBUG=1 TRACE=1), as host_phases writes them with
DOMA_PHASE_TRACE=1 DOMA_ONPKG_CSV=<file> (the testbench writes the same format with +dbg_csv=<file>). One line per
access: launch, cmd, flags, then the core's raw access-cycle timestamps (16 bits, 65535 = did not happen):
tarr (bucket read done), ea (write-back issued), dg (release gate reached: the write handed off), wl / wf (last / first
W accepted), bm (bucket record in), so (stash open issued), sx (stash ready: the decisions start), se (S_SCAN entered),
cl (last eviction copy landed), wst (cycles W held by the memory), wsv (cycles writing with no W offered), ar / r1
(bucket read address, first beat), end. Flags: [0] on package, [1] plaintext path, [2] deadline (1) missed,
[3] deadline (2) missed, [4] scratch bank conflict, [6] the command is a PUT, [11:8] eviction copies queued.
v14c debug bitstreams (128-byte records) add s0 .. s15, the access's stash open on the stash port (raw 16-bit words,
rtl/flat_oram_gcm.v dbg_st_rec): s0 {reads [15:8], open deferred until the stash was quiet [7], b has a chain [6],
bt == b [5], requested block stashed [4], b's window entries [3:0]}, s1 read beats, s2 first read address, s3 first
read beat, s4 head(b) landed, s5 last read beat, s6 longest read (address to first beat), s7 sum of read latencies
(saturating), s8 cycles a read address waited, s9 cycles with reads out and no beat, s10 stash write beats during the
open, s11 cycles from the last stash write response to the open, s12 position lookup done, s13 first level-2 read
address, s14 {outstanding stash writes at the open [15:8], the bucket's fill [7:0]}, s15 the open complete.

The deadline checks the controller makes, recomputed per access exactly as the RTL does them:
  write-back start   ws = ea - tarr   must be <= AR_WB_H    (deadline (1), the early check)
  release gate       rg = dg - tarr   must be <= AR_REL_H   (deadline (2): the write handed off by the release time)

Rule (per block size, lambda 7), with M(x) = ceil(x * s * (1 + pct/100)) + abs cycles (s: the clock scaling, --mhz):
  AR_WB_H(card)  = M(card worst ws)
  AR_REL_H(card) = max(M(card worst rg), AR_WB_H(card) + worst (rg - ws) * s)   a write that starts at the last allowed
                                                                            cycle and then takes its worst observed
                                                                            time still makes the release
  AR_WB_H  = max(AR_WB_H in oram_params.vh, AR_WB_H(card))      never below the simulation's cycle-model worst case
  AR_REL_H = max(AR_REL_H in oram_params.vh, AR_REL_H(card))
  AR_DONE_H = AR_REL_H + (AR_DONE_H - AR_REL_H in oram_params.vh) the acknowledgement allowance, unchanged
Defaults: abs = 74 (the lambda = 7 margin of sim/budgets_l7.py and sim/budgets_v14.py), pct = 10 (the margin chosen
for v14c at 4 KB).

Tail check: the top --pot records of ws and of rg (default 50) are fitted with an exponential tail (peaks over a
threshold); it gives the expected number of accesses beyond a budget in --runs accesses (default 20M), and the budget
at which that expectation is --risk (default 0.01: a 1% chance that a 20M-access run halts). An exponential tail is the
cautious choice for a quantity that is in fact bounded (a finite queue of reads); the report says when the records
reach far enough to trust it.

Options:
  --params FILE   rtl/oram_params.vh (default: ../rtl/oram_params.vh next to this script)
  --beats B       block size in 32-byte beats (default 128 = 4 KB)
  --abs N --pct P margins (defaults 74, 10)
  --mhz F         the debug bitstream's achieved kernel clock (v++ may lower it; default 300). Memory latencies are fixed
                  in time, so a debug bitstream slower than the production clock (--target-mhz, default 300) sees fewer
                  cycles: the card's worst values are scaled by target/F (when above 1) before the margins
  --tail N        how many of the slowest accesses (by stash ready, by write-back start) to print (default 30)
  --pot K --runs R --risk X   the tail check (defaults 50, 20000000, 0.01)
  --write         write the three defines ORAM_AR_{WB,REL,DONE}_H_L7_B<B> into --params (CRLF kept)
  --define-only   print only the three define lines (for scripts)
Everything is printed as text (nothing else is written unless --write). Large files are read in chunks (numpy if it is
installed, 15M records in about 1-2 minutes; otherwise pure Python, about 10x slower); memory stays small.
"""
import heapq, math, os, re, sys

# v11g's lambda = 7 on-package budgets (the encrypted path), for the saving that remains
V11G = {1: (798, 834, 842), 2: (802, 844, 858), 4: (810, 869, 890), 8: (826, 919, 953), 16: (858, 1018, 1080),
        32: (969, 1266, 1381), 64: (1321, 1891, 2112), 128: (2025, 3141, 3575)}
NONE = 65535
BASE = "launch cmd flags tarr ea dg wl wf bm so sx se cl wst wsv ar r1 end".split()
STASH = ["s%d" % i for i in range(16)]
OFF = 32768                                   # histogram offset: values -32768 .. 32767

try:
    import numpy as np
except ImportError:
    np = None

# quantities: (key, label, stash?) -- computed per on-package access
QS = [("ws", "write-back start (ws)", 0), ("rg", "release gate (rg, handed off)", 0),
      ("wr", "write: start to hand-off (rg - ws)", 0), ("wl", "last W accepted", 0), ("wf", "first W accepted", 0),
      ("bm", "bucket record in", 0), ("so", "stash open issued", 0), ("sx", "stash ready (decisions start)", 0),
      ("se", "S_SCAN entered", 0), ("dec", "decisions (stash ready to write-back)", 0), ("cl", "last eviction copy landed", 0),
      ("wst", "W held by the memory (cycles)", 0), ("wsv", "W not offered while writing (cycles)", 0),
      ("brl", "bucket read latency (AR to first R)", 0), ("tarr", "bucket read done (t_arr, access cycle)", 0),
      ("tail", "access end after the release gate", 0),
      ("sdone", "stash open complete", 1), ("sar1", "stash: first read address", 1), ("sha", "stash: head(b) landed", 1),
      ("sl2a", "stash: first level-2 read address", 1), ("srl", "stash: last read beat", 1),
      ("slk", "stash: position lookup done", 1),
      ("sdur", "stash open: duration (cycles)", 1), ("shrt", "stash open: head round trip (cycles)", 1),
      ("sl2", "stash open: head landed to level 2 (cycles)", 1), ("sdat", "stash open: level-2 reads (cycles)", 1),
      ("snr", "stash open: reads", 1), ("snb", "stash open: read beats", 1), ("swin", "stash open: b's window entries", 1),
      ("slmax", "stash open: longest read (cycles)", 1), ("slmean", "stash open: mean read latency", 1),
      ("sarw", "stash open: read address waiting", 1), ("sgap", "stash open: reads out, no beat", 1),
      ("swb", "stash open: stash write beats", 1), ("sqb", "stash: last write response to open", 1),
      ("swq", "stash: writes outstanding at open", 1)]
KEYS = [k for k, _, _ in QS]
SURV = ("sx", "ws", "rg", "sdur")


def pct_hist(h, n, p):
    """nearest rank from a histogram"""
    if not n: return None
    k = max(1, math.ceil(p * n)); c = 0
    for i, x in enumerate(h):
        c += x
        if c >= k: return i - OFF
    return None


class Acc:
    def __init__(self, tail):
        self.h = {k: [0] * 65536 for k in KEYS}
        self.n = {k: 0 for k in KEYS}
        self.mx = {k: -OFF for k in KEYS}; self.mn = {k: OFF for k in KEYS}
        self.top = {k: [] for k in ("ws", "rg")}         # the largest values (heaps) for the tail fit
        self.slow = {k: [] for k in ("sx", "ws", "rg")}  # the slowest accesses (heaps of (value, seq, row))
        self.tail = tail; self.seq = 0
        self.n_all = 0; self.n_on = 0; self.n_bad = 0; self.stash = False
        self.cls = {}                                    # sx by class: {split: {class: [n, sx > 100, sx > 200, worst]}}
        self.prev_cp = 0; self.so_ref = None             # the previous access's copies; the usual stash-open cycle
        self.reg = [0, 0.0, 0.0, 0.0, 0.0]               # n, sum b, sum d, sum b*b, sum b*d: open duration vs beats
        self.win = {}                                    # window entries -> [n, sum dur, max dur, sum sx>100]

    def put(self, k, v):
        v = max(-OFF, min(OFF - 1, int(v)))
        self.h[k][v + OFF] += 1; self.n[k] += 1
        if v > self.mx[k]: self.mx[k] = v
        if v < self.mn[k]: self.mn[k] = v

    def keep(self, heap, item, size):
        if len(heap) < size: heapq.heappush(heap, item)
        elif item[:2] > heap[0][:2]: heapq.heapreplace(heap, item)   # (value, -seq): on ties the earliest stays


def quantities(r, stash):
    """one access (dict of raw ints) -> {key: value}; None when the timestamp is missing"""
    t = r["tarr"]
    rel = lambda k: (r[k] - t) if r[k] != NONE else None
    q = {"ws": r["ea"] - t, "rg": r["dg"] - t, "wr": r["dg"] - r["ea"], "wl": rel("wl"), "wf": rel("wf"),
         "bm": rel("bm"), "so": rel("so"), "sx": rel("sx"), "se": rel("se"),
         "dec": (r["ea"] - r["sx"]) if r["sx"] != NONE else None, "cl": rel("cl"), "wst": r["wst"], "wsv": r["wsv"],
         "brl": (r["r1"] - r["ar"]) if NONE not in (r["r1"], r["ar"]) else None, "tarr": t,
         "tail": (r["end"] - r["dg"]) if r["end"] != NONE else None}
    if stash and r["s15"] != NONE:
        s = [r["s%d" % i] for i in range(16)]
        nr = s[0] >> 8
        q.update({"sdone": s[15] - t, "sar1": (s[2] - t) if s[2] != NONE else None, "sha": (s[4] - t) if s[4] != NONE else None,
                  "sl2a": (s[13] - t) if s[13] != NONE else None, "srl": (s[5] - t) if s[5] != NONE else None,
                  "slk": (s[12] - t) if s[12] != NONE else None,
                  "sdur": (s[15] - r["so"]) if r["so"] != NONE else None,
                  "shrt": (s[4] - s[2]) if NONE not in (s[4], s[2]) else None,
                  "sl2": (s[13] - s[4]) if NONE not in (s[13], s[4]) else None,
                  "sdat": (s[5] - s[13]) if NONE not in (s[5], s[13]) else None,
                  "snr": nr, "snb": s[1], "swin": s[0] & 15, "slmax": s[6], "slmean": (s[7] // nr) if nr else None,
                  "sarw": s[8], "sgap": s[9], "swb": s[10], "sqb": s[11] if s[11] != NONE else None, "swq": s[14] >> 8})
    return q


def classes(cp, pcp, put, so, so_ref, se):
    """the splits of the stash-ready table: (split, class)"""
    return [("eviction copies queued (this access)", "%d" % cp), ("eviction copies queued (the on-package access before)", "%d" % pcp),
            ("command", "PUT" if put else "GET"),
            ("stash open issued", "on time" if so == NONE or so_ref is None or so <= so_ref + 4 else
             "deferred %d-%d" % (((so - so_ref) // 40) * 40, ((so - so_ref) // 40) * 40 + 39)),
            ("S_SCAN entered (after the arrival)", "<= 4" if se is None or se <= 4 else "%d-%d" % ((se // 50) * 50, (se // 50) * 50 + 49))]


def add_row(A, r):
    A.n_all += 1
    if not (r["flags"] & 2): return
    if NONE in (r["tarr"], r["ea"], r["dg"]): A.n_bad += 1; return
    A.n_on += 1
    q = quantities(r, A.stash)
    for k, v in q.items():
        if v is not None: A.put(k, v)
    A.seq += 1
    for k in ("ws", "rg"): A.keep(A.top[k], (q[k], -A.seq), A.pot)
    for k in ("sx", "ws", "rg"):
        if q[k] is not None: A.keep(A.slow[k], (q[k], -A.seq, dict(r)), A.tail)
    # sx by class
    sx = q["sx"]; cp = (r["flags"] >> 8) & 15
    if A.so_ref is None and r["so"] != NONE: A.so_ref = r["so"]
    if sx is not None:
        for sp, c in classes(cp, A.prev_cp, r["flags"] & 64, r["so"], A.so_ref, q["se"]):
            e = A.cls.setdefault(sp, {}).setdefault(c, [0, 0, 0, -OFF])
            e[0] += 1; e[1] += sx > 100; e[2] += sx > 200; e[3] = max(e[3], sx)
    A.prev_cp = cp
    if A.stash and q.get("sdur") is not None:
        w = (q["swin"], (r["s0"] >> 4) & 1)
        e = A.win.setdefault(w, [0, 0, -OFF, 0, -OFF, 0]); d = q["sdur"]
        e[0] += 1; e[1] += d; e[2] = max(e[2], d); e[3] += (sx or 0) > 100; e[4] = max(e[4], sx or 0); e[5] += q["snb"]
        if q.get("sl2a") is not None:                    # opens with level-2 reads: two round trips + the beats
            b = q["snb"]; R = A.reg; R[0] += 1; R[1] += b; R[2] += d; R[3] += b * b; R[4] += b * d


def add_chunk_np(A, M, col):
    """vectorised add_row over a chunk (rows x columns int64)"""
    g = lambda k: M[:, col[k]]
    A.n_all += M.shape[0]
    on = (g("flags") & 2) != 0
    M = M[on]
    ok = (M[:, col["tarr"]] != NONE) & (M[:, col["ea"]] != NONE) & (M[:, col["dg"]] != NONE)
    A.n_bad += int((~ok).sum()); M = M[ok]
    if not M.shape[0]: return
    A.n_on += M.shape[0]
    g = lambda k: M[:, col[k]]
    t = g("tarr")
    Q = {}
    def rel(k, base=None):
        v = g(k); return v - (t if base is None else base), v != NONE
    Q["ws"] = (g("ea") - t, None); Q["rg"] = (g("dg") - t, None); Q["wr"] = (g("dg") - g("ea"), None)
    for k in ("wl", "wf", "bm", "so", "sx", "se", "cl"): Q[k] = rel(k)
    Q["dec"] = (g("ea") - g("sx"), g("sx") != NONE)
    Q["wst"] = (g("wst"), None); Q["wsv"] = (g("wsv"), None)
    Q["brl"] = (g("r1") - g("ar"), (g("r1") != NONE) & (g("ar") != NONE))
    Q["tarr"] = (t, None); Q["tail"] = (g("end") - g("dg"), g("end") != NONE)
    if A.stash:
        s = [g("s%d" % i) for i in range(16)]
        so = s[15] != NONE
        nr = s[0] >> 8
        Q["sdone"] = (s[15] - t, so)
        for k, i in (("sar1", 2), ("sha", 4), ("sl2a", 13), ("srl", 5), ("slk", 12)): Q[k] = (s[i] - t, so & (s[i] != NONE))
        Q["sdur"] = (s[15] - g("so"), so & (g("so") != NONE))
        Q["shrt"] = (s[4] - s[2], so & (s[4] != NONE) & (s[2] != NONE))
        Q["sl2"] = (s[13] - s[4], so & (s[13] != NONE) & (s[4] != NONE))
        Q["sdat"] = (s[5] - s[13], so & (s[5] != NONE) & (s[13] != NONE))
        Q["snr"] = (nr, so); Q["snb"] = (s[1], so); Q["swin"] = (s[0] & 15, so); Q["slmax"] = (s[6], so)
        Q["slmean"] = (s[7] // np.maximum(nr, 1), so & (nr > 0))
        Q["sarw"] = (s[8], so); Q["sgap"] = (s[9], so); Q["swb"] = (s[10], so); Q["sqb"] = (s[11], so & (s[11] != NONE))
        Q["swq"] = (s[14] >> 8, so)
    for k, (v, m) in Q.items():
        if m is not None: v = v[m]
        if not v.size: continue
        v = np.clip(v, -OFF, OFF - 1)
        bc = np.bincount(v + OFF, minlength=65536)
        h = A.h[k]
        for i in np.nonzero(bc)[0]: h[i] += int(bc[i])
        A.n[k] += int(v.size); A.mx[k] = max(A.mx[k], int(v.max())); A.mn[k] = min(A.mn[k], int(v.min()))
    # the largest ws, rg (for the tail fit) and the slowest accesses
    seq0 = A.seq; A.seq += M.shape[0]
    for k in ("ws", "rg"):
        v = Q[k][0]
        idx = np.sort(np.lexsort((np.arange(v.size), -v))[:A.pot])          # the largest, earliest first on ties
        for i in idx: A.keep(A.top[k], (int(v[i]), -(seq0 + int(i) + 1)), A.pot)
    names = list(col.keys())
    for k in ("sx", "ws", "rg"):
        v, m = Q[k]
        vv = np.where(m, v, -OFF) if m is not None else v
        idx = np.sort(np.lexsort((np.arange(vv.size), -vv))[:A.tail])      # as the row-by-row path: seq order
        for i in idx:
            if vv[i] == -OFF: continue
            A.keep(A.slow[k], (int(vv[i]), -(seq0 + int(i) + 1), {n: int(M[i, col[n]]) for n in names}), A.tail)
    # classes (the same splits as classes())
    fl = g("flags"); sx, sxm = Q["sx"]
    cp = (fl >> 8) & 15
    pcp = np.concatenate(([A.prev_cp], cp[:-1])); A.prev_cp = int(cp[-1])
    so = g("so")
    if A.so_ref is None:
        v = so[so != NONE]
        if v.size: A.so_ref = int(v[0])
    ref = A.so_ref if A.so_ref is not None else 0
    dfr = np.where((so == NONE) | (so <= ref + 4), -1, ((so - ref) // 40) * 40)
    se, sem = Q["se"]; sev = np.where(sem & (se > 4), (se // 50) * 50, -1)
    splits = [("eviction copies queued (this access)", cp, lambda c: "%d" % c),
              ("eviction copies queued (the on-package access before)", pcp, lambda c: "%d" % c),
              ("command", (fl & 64) != 0, lambda c: "PUT" if c else "GET"),
              ("stash open issued", dfr, lambda c: "on time" if c < 0 else "deferred %d-%d" % (c, c + 39)),
              ("S_SCAN entered (after the arrival)", sev, lambda c: "<= 4" if c < 0 else "%d-%d" % (c, c + 49))]
    for sp, key, name in splits:
        for c in np.unique(key[sxm]):
            m = sxm & (key == c)
            e = A.cls.setdefault(sp, {}).setdefault(name(int(c)), [0, 0, 0, -OFF])
            v = sx[m]; e[0] += int(v.size); e[1] += int((v > 100).sum()); e[2] += int((v > 200).sum()); e[3] = max(e[3], int(v.max()))
    if A.stash:
        d, dm = Q["sdur"]; w = (s[0] & 15); rq = (s[0] >> 4) & 1
        sxv = np.where(sxm, sx, 0)
        for wi in np.unique(w[dm]):
            for r in (0, 1):
                m = dm & (w == wi) & (rq == r)
                if not m.any(): continue
                e = A.win.setdefault((int(wi), r), [0, 0, -OFF, 0, -OFF, 0])
                e[0] += int(m.sum()); e[1] += int(d[m].sum()); e[2] = max(e[2], int(d[m].max()))
                e[3] += int((sxv[m] > 100).sum()); e[4] = max(e[4], int(sxv[m].max())); e[5] += int(s[1][m].sum())
        m2 = dm & (s[13] != NONE)                         # opens with level-2 reads: two round trips + the beats
        b = s[1][m2].astype(float); dd = d[m2].astype(float); R = A.reg
        R[0] += int(b.size); R[1] += b.sum(); R[2] += dd.sum(); R[3] += (b * b).sum(); R[4] += (b * dd).sum()


def read(files, A):
    for f in files:
        with open(f) as fh:
            hdr = fh.readline().strip().split(",")
            col = {k: i for i, k in enumerate(hdr)}
            miss = [k for k in BASE if k not in col]
            if miss: sys.exit(f"card_budgets.py: {f}: columns missing: {' '.join(miss)}")
            st = all(k in col for k in STASH)
            if A.n_all and st != A.stash: sys.exit("card_budgets.py: mix of files with and without the stash columns")
            A.stash = st
            nc = len(hdr)
            if np is not None:
                while True:
                    lines = fh.readlines(16 << 20)
                    if not lines: break
                    lines = [l for l in lines if l.count(",") == nc - 1]
                    if not lines: continue
                    M = np.fromstring("".join(lines).replace(",", " ").replace("\n", " "), dtype=np.int64, sep=" ")
                    if M.size != len(lines) * nc:                       # a malformed field: parse line by line
                        M = np.array([[int(x) for x in l.split(",")] for l in lines], dtype=np.int64)
                    M = M.reshape(-1, nc)
                    add_chunk_np(A, M, col)
            else:
                for line in fh:
                    v = line.split(",")
                    if len(v) != nc: continue
                    add_row(A, {k: int(v[i]) for k, i in col.items()})


def tail_fit(top, n, budget, sc, runs):
    """exponential peaks-over-threshold on the largest values: expected count beyond the budget in `runs` accesses"""
    v = sorted((x[0] for x in top), reverse=True)
    if len(v) < 10: return None
    K = len(v) - 1; u = v[-1]                                  # threshold: the smallest kept; K exceedances above it
    sigma = max(1.0, sum(x - u for x in v[:-1]) / K)
    p_u = K / n
    lim = budget / sc                                          # the budget in the debug bitstream's cycles
    exp_n = runs * p_u * math.exp(-(lim - u) / sigma) if lim > u else runs * p_u
    return u, sigma, p_u, exp_n


def main(a):
    opt = {"params": os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "oram_params.vh"),
           "beats": 128, "abs": 74, "pct": 10.0, "write": False, "mhz": 300.0, "target-mhz": 300.0, "tail": 30,
           "pot": 50, "runs": 20000000, "risk": 0.01, "define-only": False}
    files = []
    i = 0
    while i < len(a):
        if a[i] in ("--write", "--define-only"): opt[a[i][2:]] = True
        elif a[i] in ("--params", "--beats", "--abs", "--pct", "--mhz", "--target-mhz", "--tail", "--pot", "--runs", "--risk"):
            k = a[i][2:]
            opt[k] = a[i + 1] if k == "params" else (float(a[i + 1]) if k in ("pct", "mhz", "target-mhz", "risk") else int(a[i + 1])); i += 1
        elif a[i].startswith("-"): sys.exit(__doc__)
        else: files.append(a[i])
        i += 1
    if not files: sys.exit(__doc__)
    b = opt["beats"]
    with open(opt["params"], newline="") as fh: raw = fh.read()
    crlf = "\r\n" in raw; s = raw.replace("\r\n", "\n")
    def get(name):
        m = re.search(r"^`define %s +(\d+)" % name, s, re.M)
        if not m: sys.exit(f"card_budgets.py: {name} not found in the params file")
        return int(m.group(1))
    wb0, rel0, done0 = (get(f"ORAM_AR_{k}_H_L7_B{b}") for k in ("WB", "REL", "DONE"))
    A = Acc(opt["tail"]); A.pot = opt["pot"]
    read(files, A)
    if not A.n_on: sys.exit("card_budgets.py: no on-package (plaintext-path) records in the input")
    out = [] if opt["define-only"] else None
    P = (lambda *x, **y: None) if opt["define-only"] else print
    n = A.n_on
    P(f"card_budgets.py: {n} on-package accesses ({A.n_all} records in {len(files)} file(s); {A.n_bad} incomplete)"
      f", block {b * 32} B ({b} beats), lambda 7{', with stash-open records (v14c)' if A.stash else ''}"
      f"{'' if np is not None else ' [numpy not installed: pure Python]'}")
    P(f"  {'cycles after the arrival unless noted':42s} {'p50':>7s} {'p90':>7s} {'p99':>7s} {'p99.9':>7s} {'p99.99':>7s} {'max':>7s} {'min':>7s}")
    for k, label, st in QS:
        if not A.n[k]: continue
        if st and k == "sdone": P("  -- the stash open (v14c records) --")
        P(f"  {label:42s} " + " ".join(f"{pct_hist(A.h[k], A.n[k], p):7d}" for p in (0.5, 0.9, 0.99, 0.999, 0.9999))
          + f" {A.mx[k]:7d} {A.mn[k]:7d}")
    # survival tables
    for k in SURV:
        if not A.n[k]: continue
        lab = dict((x, y) for x, y, _ in QS)[k]
        p50 = pct_hist(A.h[k], A.n[k], 0.5); step = 50 if A.mx[k] - p50 <= 1200 else 100
        t0 = (p50 // step) * step
        row = []
        for t in range(t0, A.mx[k] + step, step):
            c = sum(A.h[k][t + 1 + OFF:])
            row.append(f"> {t}: {c} ({100.0 * c / A.n[k]:.5f}%)")
            if c == 0: break
        P(f"survival, {lab}: " + "; ".join(row))
    # sx by class
    if A.cls:
        P("stash ready (sx) by class: accesses, sx > 100 (share of the class), sx > 200 (share), worst sx")
        for sp in A.cls:
            P(f"  by {sp}:")
            def k(c):
                m = re.match(r"(?:deferred )?(\d+)", c); return (0, int(m.group(1))) if m else (-1, c)
            for c in sorted(A.cls[sp], key=k):
                e = A.cls[sp][c]
                if e[0]: P(f"    {c:16s} {e[0]:10d}  {e[1]:9d} ({100.0 * e[1] / e[0]:7.3f}%)  {e[2]:8d} ({100.0 * e[2] / e[0]:7.4f}%)  {e[3]:6d}")
    if A.stash and A.win:
        P("stash open by b's window entries and the requested block (rq 1: stashed): opens, mean / max duration, mean beats, sx > 100, worst sx")
        for w in sorted(A.win):
            e = A.win[w]
            P(f"  window {w[0]}, rq {w[1]}: {e[0]:10d}  {e[1] / e[0]:7.0f} / {e[2]:5d}  {e[5] / e[0]:7.0f}  {e[3]:9d}  {e[4]:6d}")
        R = A.reg
        if R[0] > 1 and R[3] * R[0] - R[1] ** 2 > 0:
            sl = (R[0] * R[4] - R[1] * R[2]) / (R[0] * R[3] - R[1] ** 2); ic = (R[2] - sl * R[1]) / R[0]
            P(f"stash open duration ~ {ic:.0f} + {sl:.3f} x read beats (least squares over the {int(R[0])} opens with level-2 reads): "
              f"{ic:.0f} cycles fixed per open (two read round trips: head(b), then level 2; head round trip p50 "
              f"{pct_hist(A.h['shrt'], A.n['shrt'], 0.5)}), {sl:.3f} cycles per beat ({1 / sl if sl > 0 else 0:.2f} beats per cycle while streaming)")
    against = []
    for k, lim in (("ws", wb0), ("rg", rel0)):
        m = sum(A.h[k][lim + 1 + OFF:])
        against.append(m)
    P(f"against the bitstream's budgets (AR_WB_H {wb0}, AR_REL_H {rel0}): deadline (1) missed by {against[0]} accesses "
      f"({100.0 * against[0] / n:.5f}%), deadline (2) by {against[1]} ({100.0 * against[1] / n:.5f}%)"
      + ("" if sum(against) == 0 else "  <- the production bitstream would halt"))
    # the slowest accesses
    def show(k, title):
        L = sorted(A.slow[k], key=lambda x: (-x[0], -x[1]))
        if not L: return
        P(f"the {len(L)} slowest by {title} (cycles after the arrival: record in, stash open, stash ready, S_SCAN, write-back, "
          f"hand-off, end; copies; W held/starved; bucket AR/first R{'; stash: window, rq, reads, beats, first AR, head in, level-2 AR, last R, done, longest read, mean, AR wait, no-beat, deferred' if A.stash else ''}):")
        for v, _, r in L:
            t = r["tarr"]; f = lambda x: "-" if r[x] == NONE else str(r[x] - t)
            line = (f"  {r['launch']}/{r['cmd']:<6d} rec {f('bm'):>5s} open {f('so'):>6s} ready {f('sx'):>5s} scan {f('se'):>4s} "
                    f"ws {f('ea'):>5s} rg {f('dg'):>5s} end {f('end'):>5s}; copies {(r['flags'] >> 8) & 15} {'PUT' if r['flags'] & 64 else 'GET'}; "
                    f"W {r['wst']}/{r['wsv']}; AR {f('ar')} R {f('r1')}")
            if A.stash and r.get("s15", NONE) != NONE:
                g = lambda i: "-" if r["s%d" % i] == NONE else str(r["s%d" % i] - t)
                s0 = r["s0"]; nr = s0 >> 8
                line += (f"; stash w{s0 & 15} rq{(s0 >> 4) & 1} {nr}r {r['s1']}b AR {g(2)} head {g(4)} L2 {g(13)} last {g(5)} done {g(15)} "
                         f"lmax {r['s6']} lmean {r['s7'] // nr if nr else 0} arw {r['s8']} gap {r['s9']}{' deferred' if s0 & 128 else ''}")
            P(line)
    show("sx", "stash ready (sx)")
    show("ws", "write-back start (ws)")
    # budgets
    sc = max(1.0, opt["target-mhz"] / opt["mhz"])
    if sc > 1.0: P(f"the debug bitstream ran at {opt['mhz']} MHz: worst values scaled by {sc:.4f} to {opt['target-mhz']} MHz")
    wsmax, rgmax, wrmax = A.mx["ws"], A.mx["rg"], A.mx["wr"]
    def budgets(ab, pc):
        mm = lambda x: math.ceil(x * sc * (1 + pc / 100.0)) + ab
        wbc = mm(wsmax); relc = max(mm(rgmax), wbc + math.ceil(wrmax * sc))
        return wbc, relc, max(wb0, wbc), max(rel0, relc)
    v11 = V11G.get(b)
    P(f"budgets (margin: ceil(worst x (1 + pct)) + abs; never below the current values; v11g's REL {v11[1] if v11 else '?'}):")
    rows = []
    for ab, pc in sorted({(opt["abs"], 0.0), (opt["abs"], 5.0), (opt["abs"], opt["pct"]), (opt["abs"], 15.0), (opt["abs"], 20.0)}, key=lambda x: x[1]):
        wbc, relc, wbn, reln = budgets(ab, pc)
        donen = reln + (done0 - rel0)
        rows.append((ab, pc, wbn, reln, donen))
        P(f"  abs {ab}, pct {pc:4.1f}: card AR_WB_H {wbc}, AR_REL_H {relc} -> AR_WB_H {wbn}, AR_REL_H {reln}, AR_DONE_H {donen}"
          + (f"; release {v11[1] - reln} cycles earlier than v11g per on-package access" if v11 else ""))
    # the tail check
    runs = opt["runs"]
    P(f"tail check (exponential fit to the top {opt['pot']} of ws and rg; expected accesses beyond the budget per {runs:,} accesses):")
    for ab, pc, wbn, reln, donen in rows:
        f1 = tail_fit(A.top["ws"], n, wbn, sc, runs); f2 = tail_fit(A.top["rg"], n, reln, sc, runs)
        if f1 and f2:
            P(f"  pct {pc:4.1f} (AR_WB_H {wbn}, AR_REL_H {reln}): deadline (1) {f1[3]:.3g}, deadline (2) {f2[3]:.3g}; "
              f"P(a {runs / 1e6:g}M run halts) ~ {1 - math.exp(-(f1[3] + f2[3])):.3g}")
    f1 = tail_fit(A.top["ws"], n, wb0, sc, runs); f2 = tail_fit(A.top["rg"], n, rel0, sc, runs)
    if f1 and f2:
        need = lambda f: math.ceil((f[0] + f[1] * math.log(runs * f[2] / opt["risk"])) * sc)
        P(f"  fit: ws above {f1[0]} decays by e every {f1[1]:.1f} cycles, rg above {f2[0]} every {f2[1]:.1f}; "
          f"the records reach {n:,} accesses ({n / runs:.2f} x a {runs / 1e6:g}M run)")
        P(f"  budgets for an expected {opt['risk']} halts per {runs / 1e6:g}M accesses (each deadline): AR_WB_H >= {need(f1)}, "
          f"AR_REL_H >= {max(need(f2), need(f1) + math.ceil(wrmax * sc))}")
    ab, pc, wbn, reln, donen = [r for r in rows if r[1] == opt["pct"]][0]
    if opt["define-only"]:
        for k, v in (("WB", wbn), ("REL", reln), ("DONE", donen)): print(f"`define ORAM_AR_{k}_H_L7_B{b} {v}")
    else:
        P(f"chosen (abs {ab}, pct {pc}): ORAM_AR_WB_H_L7_B{b} {wbn}, ORAM_AR_REL_H_L7_B{b} {reln}, ORAM_AR_DONE_H_L7_B{b} {donen}")
        for k, v in (("WB", wbn), ("REL", reln), ("DONE", donen)): P(f"`define ORAM_AR_{k}_H_L7_B{b} {v}")
        f1 = tail_fit(A.top["ws"], n, wbn, sc, runs); f2 = tail_fit(A.top["rg"], n, reln, sc, runs)
        wbc, relc, _, _ = budgets(ab, pc)
        P(f"SUMMARY beats={b} mhz={opt['mhz']:g} n={n} wsmax={wsmax} rgmax={rgmax} wbc={wbc} relc={relc} wb={wbn} rel={reln} "
          f"done={donen} v11={(v11[1] - reln) if v11 else 0} halts={(f1[3] + f2[3]) if f1 and f2 else -1:.2g}")
    if opt["write"]:
        for k, v in (("WB", wbn), ("REL", reln), ("DONE", donen)):
            pat = re.compile(r"^(`define ORAM_AR_%s_H_L7_B%d +)\d+" % (k, b), re.M)
            if len(pat.findall(s)) != 1: sys.exit(f"card_budgets.py: ORAM_AR_{k}_H_L7_B{b} not found exactly once")
            s = pat.sub(lambda m: m.group(1) + str(v), s)
        with open(opt["params"], "w", newline="") as fh: fh.write(s.replace("\n", "\r\n") if crlf else s)
        P(f"written {opt['params']}")


if __name__ == "__main__":
    main(sys.argv[1:])
