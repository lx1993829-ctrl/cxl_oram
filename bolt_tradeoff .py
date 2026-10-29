#!/usr/bin/env python3
"""DOMA against BOLT, partition ORAM and Path ORAM: on-package memory and work per access (bolt_tradeoff.pdf).

Theoretical, no measurements.
Panel (a): on-package memory (metadata + stash, % of data) with BOLT's c (average tuples per bin) = DOMA's lambda
           and BOLT's page l_max = c + ceil(log2 log2 N) (their Claim 4.2, unit constant) = DOMA's Z; no bucket
           data on-package. Same models as in (b).
Panel (b): work per access against on-package memory for four designs, each at its best parameters. Work =
           slots read and written per access, each one decrypted or encrypted, wherever the slot is stored.
           On-package memory = position map + stash or cache + every other structure the design keeps on package,
           as a share of the data; every stash or cache is sized at 2^-64 overflow probability per access.
  DOMA     : Z = 2..32, lambda < Z re-chosen per Z to minimise on-package memory; stash ceil(x_R + x_O) from Claim 1
             at 2^-64 per access (no margin); position map, bucket records with IV/tag, stash heads; 2Z per access.
  BOLT     : its own analysis, alpha = 0: bins l_max = c + ceil(log2 log2 N) (Claim 4.2, unit constant); stash
             M/2 + l_max sqrt(M ln(1/gamma)) (Claim 4.4 with its 1/N failure term set to gamma = 2^-64); position map
             with two bin indices per block, bin counts, stash labels; 4 l_max per access (two bins read and written).
  Partition ORAM (Stefanov, Shi, Song, NDSS'12): P hierarchical partitions (levels 0..T), piggybacked + nu
             round-robin evictions; average work (1 + T/2) + (1 + nu)(1.5 T + 1 + 2 cap / 2^T), worst access
             T + 1 + (1 + ceil(nu))(2^(T+1) - 1 + 2 cap); eviction cache by a Chernoff bound on its stationary
             per-slot distribution (checked against simulation), partition capacity cap from the binomial tail;
             shuffle buffer of cap blocks. This basic variant is printed, not plotted.
           De-amortized (SSS Sec. 6), plotted: every write is a shuffle job in a FIFO queue and each access does at
             most W = alpha x (mean shuffle work per access) of it, so the worst access is T + 1 + W; the average is
             the basic variant's. The evicted blocks waiting in queued jobs stay on package: (1 + ceil(nu)) per
             access over the longest wait, from a Cramer-Lundberg bound on the queued work at 2^-64. Solid: least
             worst access at each budget; dashed: least average (each line with its own best parameters).
             Conservative: the queued-block bound is 3-10x the peaks simulated with per-partition level counters
             and 2-3.6x their tail extrapolated to 2^-64, it counts dummy evictions as blocks, and job merging is
             not modelled, so partition ORAM's on-package memory is overstated by up to about 1.5% of the data
             (--pend-scale 0 drops the waiting blocks entirely).
  Ring ORAM (Ren et al., USENIX Security'15), standard: ReadPath reads one slot per bucket on the path (L + 1);
             every A accesses EvictPath (reverse-lexicographic order) reads Z and writes Z + S slots per bucket on its
             path; a bucket read S times since it was last written is reshuffled early (Z read, Z + S written). Worst
             access with the eviction at once (L + 1)(1 + 2Z + S); de-amortized, each eviction spread over the next
             A accesses, (L + 1) + ceil((L + 1)(2Z + S) / A) (early reshuffles not included). Bucket metadata on
             package (as DOMA's bucket records), position map with leaf and 64-bit PMMAC counter. On-package block
             storage, simulated (ringsim.c, sequential scan) and fitted at A x 2^-64 per eviction (2^-64 per access
             over a deployment's lifetime). Eviction at once, three ways: path-buffered (the standard EvictPath holds
             the stash and the whole path), oblivious streaming (residents first, each bucket read in full and
             rewritten, root to leaf), stash only (a lower bound, printed only). De-amortized, the blocks requested
             while the eviction runs join the stash (A (l + 1) / (L + 1) after level l, equal work per level, root to
             leaf): streaming = peak of (streaming buffer + those blocks), path-buffered = whole path held + those
             blocks. The average pairs with the eviction-at-once memory, the de-amortized worst access with the
             de-amortized memory. Parameters searched: 3-19 levels, 35-75% of real slots used, A = 0.5-2 Z; S chosen
             per (Z, A) for the least average work. Plotted: de-amortized worst access (streaming, solid) and the
             average with the eviction at once (streaming, dash-dot).
  Path ORAM (Tiny ORAM's configuration, as in the U55C port): Z = 4, leaves 2^(log2 N - 2); position map with leaf
             label and 64-bit PMMAC counter per block; stash from simulation (exponential tail extrapolated to
             2^-64); remaining budget caches whole top levels. Plotted: 2 Z (L + 1) per access, every level read and
             written wherever it is stored; the count without cached levels, 2 Z (L + 1 - k), is printed only.

Examples
  python bolt_tradeoff.py                      # 4 KB blocks, N = 2^20, c = lambda = 2..16
  python bolt_tradeoff.py --logn 22 --out bolt_tradeoff_n22.pdf     # both panels at N = 2^22
  python bolt_tradeoff.py --block 64 --out bolt_tradeoff_64.pdf

Standalone: requires numpy and matplotlib only.
"""
import argparse
import functools
import math
import sys

import numpy as np

# ======================================================================================================
# Model (identical in capacity_plot.py, z_lambda_sweep.py, bolt_frontier.py and bolt_tradeoff.py)
#   metadata = position map + bucket records (slot list, fill, IV/tag) + stash heads; every field sized to what
#              it addresses (log2 of N, of the bucket count B, of the pool size P) and packed, no word formats
#   stash    = pool blocks + links; pool size from Claim 1 (bound at 2^-64 per access, any access sequence),
#              ported from vitis/host_doma.cpp, provisioned at ceil(x_R + x_O) with no margin (--margin host adds
#              the host's 1% of N + 64, --margin exact adds Z + 2)
# ======================================================================================================
G64 = 64 * math.log(2.0)
U_R = np.arange(1, 400) * 0.005
U_O = np.arange(1, 400) * 0.01
RECORD_LAYOUTS = ("per_bucket", "per_slot", "tag_out", "ctr_only")
ROUND_BITS = False           # True: round each field up to whole bits instead of exact log2 widths


def _ln_mgf_A(t, Bk, n):
    p = 1.0 / Bk
    lr = np.log1p(-p) + t
    out = np.empty_like(t)
    small = np.abs(lr) < 1e-15
    pos = (lr > 0) & ~small
    neg = (lr < 0) & ~small
    out[small] = math.log(p * n + 1.0)
    lrn = n * lr[pos]
    with np.errstate(over="ignore", invalid="ignore"):
        v = np.log(p * np.expm1(lrn) / np.expm1(lr[pos]) + np.exp(lrn))
    v[lrn > 700] = np.inf
    out[pos] = v
    rn = np.exp(n * lr[neg])
    out[neg] = np.log(p * (1 - rn) / (-np.expm1(lr[neg])) + rn)
    return out

def _vR(Bk, n, extra_reads=0):
    """ln E[exp((e^u - 1) A / B)] on the U_R grid (A: accesses since a bucket was last read, capped at N)."""
    # bucket j goes unread for i accesses with prob (1-1/B)^(i(1+k)); remaps into j still arrive at rate 1/B
    Be = 1.0 / (1.0 - (1.0 - 1.0 / Bk) ** (1 + extra_reads)) if extra_reads else Bk
    v = _ln_mgf_A(np.expm1(U_R) / Bk, Be, n)
    ok = np.isfinite(v)
    if not ok.all():                                  # the C++ loop stops at the first non-finite value
        first_bad = int(np.argmax(~ok))
        return v[:first_bad], U_R[:first_bad]
    return v, U_R

def _xR(Bk, n, lg, extra_reads=0):
    v, u = _vR(Bk, n, extra_reads)
    return float(np.min((Bk * v + lg) / u))

def _vO(Bk, n, Z):
    """ln E[exp(u (X - Z)^+)] on the U_O grid, X ~ Binomial(n, 1/Bk) (Claim 1)."""
    p = 1.0 / Bk
    mu = n * p
    k = np.arange(0, min(n, int(mu + 14 * math.sqrt(mu) + 80)) + 1)
    lc = np.concatenate(([0.0], np.cumsum(np.log(n - np.arange(len(k) - 1, dtype=float)))))   # ln n!/(n-k)!
    lt0 = lc - np.array([math.lgamma(x + 1.0) for x in k]) + k * math.log(p) + (n - k) * math.log1p(-p)
    lt = lt0[None, :] + U_O[:, None] * np.maximum(0, k - Z)[None, :]
    mx = lt.max(axis=1)
    return mx + np.log(np.exp(lt - mx[:, None]).sum(axis=1))

def _xO(Bk, n, Z, lg):
    return float(np.min((Bk * _vO(Bk, n, Z) + lg) / U_O))

SPLITS = np.linspace(0.002, 0.998, 499)      # gamma_R / gamma for the split search

@functools.lru_cache(maxsize=None)
def stash_bound(n, lam, Z, extra_reads=0):
    """Claim 1: x_R + x_O, minimised over splits of gamma = 2^-64 per access (any access sequence); the stash
    exceeds it with probability <= 2^-64 per access."""
    Bk = math.ceil(n / lam)
    vR, uR = _vR(Bk, n, extra_reads)
    vO = _vO(Bk, n, Z)
    xR = np.min((Bk * vR[None, :] + (G64 - np.log(SPLITS))[:, None]) / uR[None, :], axis=1)
    xO = np.min((Bk * vO[None, :] + (G64 - np.log1p(-SPLITS))[:, None]) / U_O[None, :], axis=1)
    return float(np.min(xR + xO))

def pool_entries(n, lam, Z, margin="none"):
    """Pool entries: ceil(x_R + x_O) from Claim 1 (margin='none', the paper's rule); 'host' adds the host's provisional
    margin (1% of n + 64), 'exact' adds Z + 2."""
    b = math.ceil(stash_bound(n, lam, Z))
    extra = {"none": 0, "host": math.ceil(0.01 * n) + 64, "exact": Z + 2}[margin]
    return min(n, b + extra)


def clog2(x):
    return max(1, math.ceil(math.log2(x)))

def trusted(n, S, lam=8, Z=8, records="per_bucket", margin="none"):
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
    B = math.ceil(n / lam)
    P = pool_entries(n, lam, Z, margin)
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
    return dict(n=n, S=S, lam=lam, Z=Z, records=records, margin=margin, B=B, P=P, pool_frac=P / n,
                pm=pm, rec=rec, ivtag=ivtag, heads=heads, metadata=metadata, links=links, blocks=blocks,
                stash=stash, total=metadata + stash, data=n * S, ratio=(metadata + stash) / (n * S))

def blocks_for(D, S):
    """Blocks needed to hold D bytes of data in S-byte blocks."""
    return max(16, int(round(D / S)))


def total(lam, Z, S, data_bytes, share, records="per_bucket", margin="none"):
    """On-package memory as a fraction of the data size N * S: metadata + stash + share * Z / lambda (share = the
    share of the bucket data kept on-package: that many buckets of Z * S bytes, slack positions included)."""
    t = trusted(blocks_for(data_bytes, S), S, lam, Z, records, margin)
    return t["ratio"] + share * Z / lam


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
    """BOLT at average bin load c (panels (a), (b)): the same model as panel (c), stash at 2^-64 per access.
    Returns (l_max, slots per request, on-package share)."""
    r = bolt_own(n, S, c)
    return r["lmax"], r["work"], r["share"]


# ---------------- panel (c): four designs, work per access against on-package memory ----------------
PATH_Z = 4
PEND_SCALE = 1.0      # scales the queued-block bound of de-amortized partition ORAM (--pend-scale; sensitivity only)
# Path ORAM stash at 2^-64 per access (Z = 4 has no analytical bound): simulated 10^9 accesses at N = 2^20 (pathsim.c,
# greedy write-back, stash counted after each write-back), peak 24; Pr[stash >= R] falls by about 0.55 per block from
# R = 10 to 22, and fits over those ranges reach 2^-64 at R = 63-69, so 70. The tail does not grow with N (Stefanov
# et al.), so N = 2^22 uses the same value. For comparison, their proven Z = 5 bound 14 * 0.6002^R gives 93.
PATH_STASH = {20: 70, 22: 70}


def doma_rows(n, S, zs=range(2, 33)):
    """DOMA at each Z: the lambda with the least on-package memory; pool = ceil(x_R + x_O) from Claim 1."""
    rows = []
    for Z in zs:
        lam = min(range(1, Z), key=lambda l: trusted(n, S, l, Z)["ratio"])     # every lambda < Z
        t = trusted(n, S, lam, Z)
        rows.append(dict(Z=Z, lam=lam, work=2 * Z, share=t["ratio"], pool=t["P"]))
    return rows


def bolt_own(n, S, c, lg=G64):
    """BOLT from its own Claims 4.2 and 4.4 (unit constants, alpha = 0), stash at e^-lg per access."""
    lmax = c + math.ceil(math.log2(math.log2(n)))
    M = math.ceil(n / c)
    stash = math.ceil(M / 2 + lmax * math.sqrt(M * lg))
    bits = n * 2 * math.log2(M) + M * math.log2(lmax + 1) + stash * (math.log2(n) + math.log2(M))
    mem = stash * S + bits / 8                               # stash, position map, bin counts, stash labels
    return dict(c=c, lmax=lmax, work=4 * lmax, share=mem / (n * S), stash=stash)


def path_rows(n, S, logn):
    """Path ORAM (Z = 4, L = log2 N - 2) caching its top k levels, k = 0 .. L + 1."""
    L = logn - 2
    lv = L + 1
    stash = PATH_STASH.get(logn) or PATH_STASH[20]
    tagb = (math.log2(n) + L) / 8                             # address + leaf per slot
    base = n * (L + 64) / 8 + stash * (S + tagb)                # map (leaf + PMMAC counter), stash
    return [dict(k=k, share=(base + (2 ** k - 1) * PATH_Z * (S + tagb)) / (n * S), all=2 * PATH_Z * lv,
                 uncached=2 * PATH_Z * (lv - k), L=L) for k in range(0, lv + 1)]


def _expm_bd(G, t, q=2.0):
    """exp(G t) by uniformization, G a birth-death generator with exit rates <= q."""
    K = G.shape[0]
    Pm, out, term, w = np.eye(K) + G / q, np.zeros((K, K)), np.eye(K), math.exp(-q * t)
    for k in range(int(q * t + 12 * math.sqrt(q * t + 1) + 30) + 1):
        out += w * term
        term = term @ Pm
        w *= q * t / (k + 1)
    return out


@functools.lru_cache(maxsize=None)
def _cache_lambda(nu, K=160, phases=48):
    """Partition ORAM eviction cache, one slot: log-MGF of its occupancy averaged over the round-robin phase.
    Time in units of P accesses; a scan (the worst case) adds blocks at rate 1 and piggybacked evictions remove
    them at rate 1; the round-robin eviction removes one every 1/nu."""
    G = np.zeros((K, K))
    for i in range(K):
        if i + 1 < K:
            G[i, i + 1] = 1.0
        if i > 0:
            G[i, i - 1] = 1.0
        G[i, i] = -G[i].sum()
    R = np.zeros((K, K))
    R[0, 0] = 1.0
    R[np.arange(1, K), np.arange(K - 1)] = 1.0
    w, v = np.linalg.eig((_expm_bd(G, 1.0 / nu) @ R).T)
    pi = np.real(v[:, np.argmin(abs(w - 1))])
    pi = np.maximum(pi / pi.sum(), 0)             # the eigenvector may come back with either sign
    pi /= pi.sum()
    Eh = _expm_bd(G, 1.0 / nu / phases)
    u = np.arange(1, 300) * 0.005
    lam, d, k = np.zeros_like(u), pi, np.arange(K)
    for _ in range(phases):
        d = d @ Eh
        lam += np.log((d[None, :] * np.exp(np.outer(u, k))).sum(axis=1))
    return u, lam / phases


def _binom_cap(n, p, lg):
    """Smallest c with Pr[Binomial(n, p) > c] <= e^-lg."""
    mu = n * p
    k = np.arange(int(mu), int(mu + 40 * math.sqrt(mu + 1) + 200))
    lp = (np.array([math.lgamma(n + 1) - math.lgamma(x + 1) - math.lgamma(n - x + 1) for x in k])
          + k * math.log(p) + (n - k) * math.log1p(-p))
    tail = np.logaddexp.accumulate(lp[::-1])[::-1]
    return int(k[np.nonzero(tail <= -lg)[0][0]] - 1)


def _job_dist(T, cap):
    """Cost of one partition write (eviction) and its probability: merge into level t < T costs 3 * 2^t - 1 and
    happens on a 2^-(t+1) share of writes; the full reshuffle into the top costs 2^(T+1) - 1 + 2 cap, 2^-T of writes."""
    vals = [3 * 2 ** t - 1 for t in range(T)] + [2 ** (T + 1) - 1 + 2 * cap]
    probs = [2.0 ** -(t + 1) for t in range(T)] + [2.0 ** -T]
    return np.array(vals, float), np.log(np.array(probs))


def _backlog_bound(vals, lprobs, nu, W, lg):
    """De-amortized partition ORAM: shuffle work queued, at most W served per access (W, lg: arrays). Each access
    brings 1 + nu writes (piggybacked + round robin) of independent cost. Cramer-Lundberg: Pr[backlog >= b] <=
    exp(-theta b), theta > 0 the root of ln E[exp(theta J)] = theta W. Returns b with exp(-theta b) = e^-lg."""
    W, lg = np.asarray(W, float), np.asarray(lg, float)
    m0, f = 1 + math.floor(nu), nu - math.floor(nu)

    def g(th):
        lnM = np.logaddexp.reduce(lprobs[None, :] + th[:, None] * vals[None, :], axis=1)
        lnJ = m0 * lnM if f == 0 else np.logaddexp(math.log1p(-f) + m0 * lnM, math.log(f) + (m0 + 1) * lnM)
        return lnJ - th * W

    lo, hi = np.zeros_like(W), np.full_like(W, 1e-3)
    for _ in range(80):
        neg = g(hi) < 0
        if not neg.any():
            break
        lo, hi = np.where(neg, hi, lo), np.where(neg, hi * 2, hi)
    for _ in range(60):
        mid = (lo + hi) / 2
        neg = g(mid) < 0
        lo, hi = np.where(neg, mid, lo), np.where(neg, hi, mid)
    return lg / lo


def partition_points(n, S, nus=(0.125, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0), deamortized=False,
                     alphas=(1.02, 1.05, 1.1, 1.2, 1.35, 1.5, 2.0, 3.0)):
    """Partition ORAM configurations: P partitions (sizes 2^(k/4)), top level T, eviction rate nu.
    deamortized: SSS Sec. 6. Every write becomes a shuffle job in a FIFO queue; each access does at most
    W = alpha * (mean shuffle work per access) of it. Blocks of queued jobs wait on package."""
    parts = 3 if deamortized else 2                            # cache, top level (and backlog) share 2^-64
    lg = G64 + math.log(parts)
    pts = []
    for size in sorted({int(round(2 ** (k / 4))) for k in range(4, 4 * int(math.log2(n)) + 1)}):
        P = int(round(n / size))
        if P < 2:
            continue
        cap = _binom_cap(n, 1.0 / P, lg)
        for nu in nus:
            u, lam = _cache_lambda(nu)
            cache = math.ceil(float(np.min((P * lam + lg) / u)))
            for T in range(0, math.ceil(math.log2(cap)) + 2):
                full = 2 ** (T + 1) - 1 + 2 * cap                # one full reshuffle into the top level
                slots = 2 ** (T + 1) - 2 + 2 ** T + cap          # per partition
                bits = (n * (math.log2(P) + math.log2(T + 2) + 1)                     # position map
                        + P * ((T + 1) * (128 + math.log2(slots + 1) + 1) + slots)     # level keys, read bitmaps
                        + cache * (math.log2(n) + 2 * math.log2(cache + 2)))           # cache entry tags
                mem = (cache + cap) * S + bits / 8                                     # cache + shuffle buffer
                mu = (1 + nu) * (1.5 * T + 1 + 2 * cap / 2 ** T)                       # shuffle work per access
                avg = (1 + T / 2) + mu
                base = dict(P=P, T=T, nu=nu, cap=cap, cache=cache, size=n / P, avg=avg)
                if not deamortized:
                    pts.append(dict(base, share=mem / (n * S), worst=(T + 1) + (1 + math.ceil(nu)) * full))
                    continue
                vals, lprobs = _job_dist(T, cap)
                Ws = np.array([math.ceil(al * mu) for al in alphas], float)
                win = np.ones_like(Ws)
                for _ in range(3):               # union bound over the accesses a queued job can wait
                    bl = _backlog_bound(vals, lprobs, nu, Ws, lg + np.log(win))
                    win = np.ceil((bl + full) / Ws) + 1
                for al, W, b_, w_ in zip(alphas, Ws, bl, win):
                    pend = int(PEND_SCALE * (1 + math.ceil(nu)) * w_)   # evicted blocks waiting in queued jobs
                    m2 = mem + pend * (S + (math.log2(n) + math.log2(P) + 32) / 8)
                    pts.append(dict(base, share=m2 / (n * S), worst=(T + 1) + int(W), W=int(W), alpha=al,
                                    backlog=float(b_), pend=pend))
    return pts


# Ring ORAM on-package block storage, simulated (ringsim.c, sequential scan): (N, L, Z, A) -> blocks for
# (path-buffered, oblivious streaming, stash only) with the eviction at once, then (path-buffered, oblivious
# streaming) with the eviction de-amortized over the next A accesses. Path-buffered: the standard EvictPath holds the
# stash and every real block of the eviction path. Oblivious streaming: residents first, each bucket read in full and
# then rewritten, root to leaf; the peak of stash + residents of the bucket being rewritten + blocks moving down not
# yet placed. Stash only: the stash before eviction, a lower bound. De-amortized: equal work per level, root to leaf,
# so A (l + 1) / (L + 1) newly requested blocks have joined the stash after level l; streaming = peak of (streaming
# buffer + those blocks), path-buffered = whole path held + those blocks (= path + A). Each is an exponential fit to the top 10% of the simulated
# per-eviction values, extrapolated to A x 2^-64 per eviction (an overflow can only happen at an eviction, once every
# A accesses: 2^-64 per access over a deployment's lifetime). Stable configurations only (ringsim's verdict).
RING_SIM = {(1048576, 2, 199729, 99864): (418109, 254620, 99864, 517973, 321196),
            (1048576, 2, 199729, 149797): (458041, 305856, 149797, 607838, 405720),
            (1048576, 2, 199729, 199729): (528902, 351891, 199729, 728631, 485043),
            (1048576, 2, 199729, 299594): (628318, 491515, 328725, 927912, 628318),
            (1048576, 2, 299593, 149796): (418589, 268977, 149796, 568385, 418773),
            (1048576, 2, 299593, 224695): (491745, 343133, 224695, 716440, 493306),
            (1048576, 2, 299593, 299593): (566866, 453695, 299593, 866459, 618406),
            (1048576, 2, 299593, 449390): (717846, 680457, 449390, 1167236, 830253),
            (1048576, 2, 299593, 599186): (828616, 828616, 605612, 1427802, 1028344),
            (1048576, 2, 427990, 213995): (483705, 327734, 213995, 697700, 481168),
            (1048576, 2, 427990, 320992): (590094, 485022, 320992, 911086, 641829),
            (1048576, 2, 427990, 427990): (694571, 645985, 427990, 1122561, 788648),
            (1048576, 2, 427990, 641985): (850914, 850914, 641985, 1492899, 1064909),
            (1048576, 2, 427990, 855980): (956186, 956186, 863325, 1812166, 1241512),
            (1048576, 3, 93207, 46604): (254882, 150619, 46604, 301486, 185572),
            (1048576, 3, 93207, 69905): (311317, 168927, 69905, 381222, 221355),
            (1048576, 3, 93207, 93207): (343966, 191407, 93207, 437173, 246154),
            (1048576, 3, 139810, 69905): (241531, 134831, 69905, 311436, 204736),
            (1048576, 3, 139810, 104858): (294140, 162234, 104858, 398998, 241191),
            (1048576, 3, 139810, 139810): (346203, 213864, 139810, 486013, 311522),
            (1048576, 3, 139810, 209715): (451247, 319463, 209715, 660962, 424378),
            (1048576, 3, 139810, 279620): (548751, 424150, 285843, 828371, 548974),
            (1048576, 3, 199729, 99864): (286946, 155036, 99864, 386810, 235440),
            (1048576, 3, 199729, 149797): (361840, 231144, 149797, 511637, 325606),
            (1048576, 3, 199729, 199729): (437136, 307408, 199729, 636865, 407272),
            (1048576, 3, 199729, 299594): (568277, 452965, 299594, 867871, 567401),
            (1048576, 3, 199729, 399458): (667444, 603612, 405049, 1066902, 703476),
            (1048576, 4, 45100, 22550): (162243, 81477, 22550, 184793, 99517),
            (1048576, 4, 45100, 33825): (175536, 85808, 33825, 209361, 112410),
            (1048576, 4, 45100, 45100): (215443, 93613, 45100, 260543, 124194),
            (1048576, 4, 67650, 33825): (137502, 69084, 33825, 171327, 102909),
            (1048576, 4, 67650, 50738): (170787, 79460, 50738, 221525, 120428),
            (1048576, 4, 67650, 67650): (205612, 106511, 67650, 273262, 157661),
            (1048576, 4, 67650, 101475): (275246, 157263, 101475, 376721, 218640),
            (1048576, 4, 67650, 135300): (338614, 207725, 140828, 473914, 284459),
            (1048576, 4, 96643, 48322): (166804, 76597, 48322, 215126, 116827),
            (1048576, 4, 96643, 72482): (216488, 112343, 72482, 288970, 164796),
            (1048576, 4, 96643, 96643): (262639, 148827, 96643, 359282, 206825),
            (1048576, 4, 96643, 144964): (354027, 221779, 144964, 498991, 296604),
            (1048576, 4, 96643, 193286): (426419, 297164, 200756, 619705, 373352),
            (1048576, 5, 22192, 11096): (83525, 38459, 11096, 94621, 47705),
            (1048576, 5, 22192, 16644): (104285, 49063, 16644, 120929, 55781),
            (1048576, 5, 22192, 22192): (129750, 46628, 22192, 151942, 62616),
            (1048576, 5, 33288, 16644): (77854, 35173, 16644, 94498, 51817),
            (1048576, 5, 33288, 24966): (99754, 40483, 24966, 124720, 61628),
            (1048576, 5, 33288, 33288): (120248, 52618, 33288, 153536, 80944),
            (1048576, 5, 33288, 49932): (162026, 79256, 49932, 211958, 112433),
            (1048576, 5, 33288, 66576): (204729, 105843, 72504, 271305, 149940),
            (1048576, 5, 47554, 23777): (96058, 38234, 23777, 119835, 59147),
            (1048576, 5, 47554, 35666): (125930, 56775, 35666, 161596, 83783),
            (1048576, 5, 47554, 47554): (155940, 74556, 47554, 203494, 106254),
            (1048576, 5, 47554, 71331): (212098, 111409, 71331, 283429, 152939),
            (1048576, 5, 47554, 95108): (261535, 148033, 100813, 356643, 195622),
            (1048576, 6, 11009, 5504): (46734, 20215, 5504, 52238, 24932),
            (1048576, 6, 11009, 8257): (60554, 26843, 8257, 68811, 28336),
            (1048576, 6, 11009, 11009): (77784, 23592, 11009, 88793, 31530),
            (1048576, 6, 16513, 8256): (44445, 16513, 8256, 52701, 24769),
            (1048576, 6, 16513, 12385): (56648, 20858, 12385, 69033, 31777),
            (1048576, 6, 16513, 16513): (69385, 26982, 16513, 85898, 41195),
            (1048576, 6, 16513, 24770): (94521, 39973, 24770, 119291, 57633),
            (1048576, 6, 16513, 33026): (121376, 55866, 39421, 154402, 78761),
            (1048576, 6, 23590, 11795): (54708, 19890, 11795, 66503, 30420),
            (1048576, 6, 23590, 17692): (73205, 29016, 17692, 90897, 43264),
            (1048576, 6, 23590, 23590): (91019, 37860, 23590, 114609, 55115),
            (1048576, 6, 23590, 35385): (125353, 56542, 35385, 160738, 79054),
            (1048576, 6, 23590, 47180): (155441, 75250, 51872, 202621, 102065),
            (1048576, 7, 5483, 2742): (26269, 10486, 2742, 29011, 12885),
            (1048576, 7, 5483, 4112): (33905, 14037, 4112, 38017, 15278),
            (1048576, 7, 5483, 5483): (46217, 12174, 5483, 51700, 15861),
            (1048576, 7, 8224, 4112): (25074, 8224, 4112, 29186, 12336),
            (1048576, 7, 8224, 6168): (32345, 11276, 6168, 38513, 16956),
            (1048576, 7, 8224, 8224): (39577, 13962, 8224, 47801, 21627),
            (1048576, 7, 8224, 12336): (55106, 20605, 12336, 67442, 30296),
            (1048576, 7, 8224, 16448): (73489, 32257, 24061, 89937, 44743),
            (1048576, 7, 11749, 5874): (31364, 10269, 5874, 37238, 15657),
            (1048576, 7, 11749, 8812): (41539, 14855, 8812, 50351, 22254),
            (1048576, 7, 11749, 11749): (52331, 19449, 11749, 64080, 28640),
            (1048576, 7, 11749, 17624): (72627, 28584, 17624, 90251, 41131),
            (1048576, 7, 11749, 23498): (93545, 42121, 30306, 117043, 56762),
            (1048576, 8, 2736, 1368): (14486, 5508, 1368, 15854, 6724),
            (1048576, 8, 2736, 2052): (18810, 6991, 2052, 20862, 8037),
            (1048576, 8, 2736, 2736): (31946, 6874, 6360, 34682, 8785),
            (1048576, 8, 4104, 2052): (14219, 4104, 2052, 16271, 6156),
            (1048576, 8, 4104, 3078): (18444, 6159, 3078, 21522, 9063),
            (1048576, 8, 4104, 4104): (22716, 7491, 4104, 26820, 11548),
            (1048576, 8, 4104, 6156): (31257, 10815, 6156, 37413, 15729),
            (1048576, 8, 4104, 8208): (47048, 22554, 18400, 55256, 28867),
            (1048576, 8, 5863, 2932): (17743, 5385, 2932, 20675, 8162),
            (1048576, 8, 5863, 4397): (23841, 7910, 4397, 28238, 11561),
            (1048576, 8, 5863, 5863): (29886, 10343, 5863, 35749, 15007),
            (1048576, 8, 5863, 8794): (41711, 15083, 8794, 50505, 21350),
            (1048576, 8, 5863, 11726): (57149, 25021, 19202, 68875, 33076),
            (1048576, 9, 1367, 684): (8048, 2902, 684, 8732, 3517),
            (1048576, 9, 1367, 1025): (10616, 3612, 1025, 11641, 4269),
            (1048576, 9, 2050, 1025): (8058, 2050, 1025, 9083, 3075),
            (1048576, 9, 2050, 1538): (10547, 3373, 1538, 12085, 4912),
            (1048576, 9, 2050, 2050): (13016, 4095, 2050, 15066, 6148),
            (1048576, 9, 2050, 3075): (17787, 5643, 3075, 20862, 8255),
            (1048576, 9, 2050, 4100): (32747, 18163, 16126, 36847, 21585),
            (1048576, 9, 2929, 1464): (10055, 2927, 1464, 11519, 4307),
            (1048576, 9, 2929, 2197): (13537, 4196, 2197, 15734, 6122),
            (1048576, 9, 2929, 2929): (17143, 5446, 2929, 20072, 7962),
            (1048576, 9, 2929, 4394): (23824, 7885, 4394, 28218, 11313),
            (1048576, 9, 2929, 5858): (37200, 18441, 15670, 43058, 22639),
            (1048576, 10, 683, 342): (4675, 1533, 342, 5017, 1841),
            (1048576, 10, 683, 512): (6056, 1895, 512, 6568, 2244),
            (1048576, 10, 683, 683): (13598, 7051, 6376, 14281, 7638),
            (1048576, 10, 1025, 512): (4736, 1025, 512, 5248, 1537),
            (1048576, 10, 1025, 769): (6137, 1975, 769, 6906, 2794),
            (1048576, 10, 1025, 1025): (7460, 2249, 1025, 8485, 3385),
            (1048576, 10, 1025, 1538): (10222, 3087, 1538, 11760, 4435),
            (1048576, 10, 1025, 2050): (25125, 16868, 15850, 27175, 18596),
            (1048576, 10, 1464, 732): (5787, 1641, 732, 6519, 2380),
            (1048576, 10, 1464, 1098): (7789, 2276, 1098, 8887, 3314),
            (1048576, 10, 1464, 1464): (9733, 2924, 1464, 11197, 4240),
            (1048576, 10, 1464, 2196): (13574, 4199, 2196, 15770, 5928),
            (1048576, 10, 1464, 2928): (25893, 15142, 13648, 28821, 17211),
            (1048576, 11, 341, 170): (2771, 854, 170, 2941, 986),
            (1048576, 11, 341, 256): (3481, 1018, 256, 3737, 1203),
            (1048576, 11, 341, 341): (13329, 10112, 9770, 13670, 10408),
            (1048576, 11, 512, 256): (2756, 1114, 256, 3012, 1097),
            (1048576, 11, 512, 384): (3548, 1173, 384, 3932, 1600),
            (1048576, 11, 512, 512): (4356, 1284, 512, 4868, 1876),
            (1048576, 11, 512, 768): (5905, 1702, 768, 6673, 2437),
            (1048576, 11, 512, 1024): (26078, 21553, 21044, 27102, 22330),
            (1048576, 11, 732, 366): (3372, 931, 366, 3738, 1310),
            (1048576, 11, 732, 549): (4497, 1295, 549, 5046, 1819),
            (1048576, 11, 732, 732): (5615, 1639, 732, 6347, 2343),
            (1048576, 11, 732, 1098): (7777, 2299, 1098, 8875, 3187),
            (1048576, 11, 732, 1464): (20981, 14878, 14165, 22445, 15967),
            (1048576, 12, 171, 86): (1662, 505, 86, 1748, 559), (1048576, 12, 171, 128): (2162, 578, 128, 2290, 666),
            (1048576, 12, 256, 128): (1636, 613, 128, 1764, 723),
            (1048576, 12, 256, 192): (2108, 711, 192, 2300, 948),
            (1048576, 12, 256, 256): (2557, 763, 256, 2813, 1092),
            (1048576, 12, 256, 384): (3436, 970, 384, 3820, 1362),
            (1048576, 12, 366, 183): (1996, 560, 183, 2179, 766),
            (1048576, 12, 366, 274): (2648, 766, 274, 2922, 1039),
            (1048576, 12, 366, 366): (3276, 941, 366, 3642, 1326),
            (1048576, 12, 366, 549): (4524, 1291, 549, 5073, 1777),
            (1048576, 12, 366, 732): (18988, 15575, 15205, 19720, 16126),
            (1048576, 13, 85, 42): (1059, 303, 42, 1101, 330), (1048576, 13, 85, 64): (1454, 332, 64, 1518, 375),
            (1048576, 13, 128, 64): (1017, 402, 64, 1081, 464), (1048576, 13, 128, 96): (1278, 473, 96, 1374, 606),
            (1048576, 13, 128, 128): (1530, 461, 128, 1658, 650),
            (1048576, 13, 128, 192): (2053, 589, 192, 2245, 798), (1048576, 13, 183, 92): (1219, 345, 92, 1311, 456),
            (1048576, 13, 183, 137): (1584, 457, 137, 1721, 606),
            (1048576, 13, 183, 183): (1930, 561, 183, 2113, 769),
            (1048576, 13, 183, 274): (2627, 765, 274, 2901, 1003), (1048576, 14, 43, 22): (688, 184, 22, 710, 197),
            (1048576, 14, 43, 32): (907, 210, 32, 939, 231), (1048576, 14, 64, 32): (633, 273, 32, 665, 326),
            (1048576, 14, 64, 48): (806, 293, 48, 854, 370), (1048576, 14, 64, 64): (961, 286, 64, 1025, 388),
            (1048576, 14, 64, 96): (1240, 349, 152, 1336, 458), (1048576, 14, 91, 46): (750, 233, 46, 796, 287),
            (1048576, 14, 91, 68): (969, 295, 68, 1037, 371), (1048576, 14, 91, 91): (1174, 353, 91, 1265, 457),
            (1048576, 14, 91, 136): (1553, 454, 136, 1689, 600), (1048576, 15, 21, 10): (475, 121, 10, 485, 126),
            (1048576, 15, 21, 16): (824, 135, 16, 840, 146), (1048576, 15, 32, 16): (421, 190, 16, 437, 223),
            (1048576, 15, 32, 24): (526, 186, 24, 550, 238), (1048576, 15, 32, 32): (605, 197, 32, 637, 198),
            (1048576, 15, 32, 48): (812, 207, 138, 860, 243), (1048576, 15, 46, 23): (501, 164, 23, 524, 183),
            (1048576, 15, 46, 34): (609, 193, 34, 643, 242), (1048576, 15, 46, 46): (752, 236, 46, 798, 290),
            (1048576, 15, 46, 69): (980, 274, 140, 1049, 357), (1048576, 16, 11, 6): (335, 89, 6, 341, 92),
            (1048576, 16, 11, 8): (661, 243, 226, 669, 254), (1048576, 16, 16, 8): (281, 110, 8, 289, 120),
            (1048576, 16, 16, 12): (348, 111, 12, 360, 121), (1048576, 16, 16, 16): (410, 132, 16, 426, 154),
            (1048576, 16, 16, 24): (2483, 2245, 2227, 2507, 2260), (1048576, 16, 23, 12): (335, 120, 12, 347, 136),
            (1048576, 16, 23, 17): (401, 141, 17, 418, 156), (1048576, 16, 23, 23): (475, 163, 23, 498, 189),
            (1048576, 16, 23, 34): (641, 173, 155, 675, 210), (1048576, 17, 8, 4): (206, 74, 4, 210, 81),
            (1048576, 17, 8, 6): (241, 85, 6, 247, 91), (1048576, 17, 8, 8): (281, 93, 48, 289, 97),
            (1048576, 17, 11, 6): (235, 92, 6, 241, 98), (1048576, 17, 11, 8): (267, 96, 8, 275, 106),
            (1048576, 17, 11, 11): (318, 111, 11, 329, 122), (1048576, 17, 11, 16): (957, 767, 757, 973, 790),
            (1048576, 18, 4, 2): (158, 61, 2, 160, 60), (1048576, 18, 4, 3): (182, 65, 44, 185, 67),
            (1048576, 18, 4, 4): (1779, 1714, 1714, 1783, 1715), (1048576, 18, 6, 3): (168, 73, 3, 171, 73),
            (1048576, 18, 6, 4): (189, 79, 4, 193, 81), (1048576, 18, 6, 6): (232, 78, 72, 238, 84)}


def _poisson_tail(lam, kmax):
    """Pr[X >= k], k = 0..kmax, X ~ Poisson(lam)."""
    k = np.arange(kmax + 1)
    lf = np.concatenate(([0.0], np.cumsum(np.log(np.arange(1, kmax + 1)))))
    lp = -lam + k * math.log(lam) - lf
    return np.exp(np.logaddexp.accumulate(lp[::-1])[::-1])


def _binom_tail(n, p, kmax):
    """Pr[X >= k], k = 0..kmax, X ~ Binomial(n, p)."""
    kmax = min(kmax, n)
    k = np.arange(kmax + 1)
    ln = np.log(np.arange(1, n + 1, dtype=float))
    lfn = np.concatenate(([0.0], np.cumsum(ln)))
    lp = lfn[n] - lfn[k] - lfn[n - k] + k * math.log(p) + (n - k) * math.log1p(-p)
    return np.exp(np.logaddexp.accumulate(lp[::-1])[::-1])


def ring_cost(n, S_blk, L, Z, A, peaks):
    """Ring ORAM (Ren et al., USENIX Security'15): ReadPath reads one slot per bucket (L + 1); EvictPath every A
    accesses reads Z and writes Z + S slots per bucket on its path; a bucket read S times since it was last written is
    reshuffled early (Z read, Z + S written). Bucket reads per eviction cycle: exactly A at the root, Binomial(2A, 1/2)
    and Binomial(4A, 1/4) at levels 1 and 2, Poisson(A) below (slightly more reshuffles than the exact binomial).
    S = number of dummy slots, chosen for the least average work. Bucket metadata (addresses, offsets, valid bits,
    read count) is kept on package, like DOMA's bucket records. peaks = block storage (path-buffered, streaming,
    stash only) with the eviction at once, then (path-buffered, streaming) de-amortized. Worst access with the
    eviction at once: the one that runs an EvictPath, (L + 1)(1 + 2Z + S); de-amortized, each eviction spread evenly
    over the next A accesses, (L + 1) + ceil((L + 1)(2Z + S) / A), early reshuffles not included.
    Returns a dict with avg, S, worst, worst_deam and share_<variant> for path, stream, pre, path_deam, stream_deam."""
    kmax = int(A + 14 * math.sqrt(A) + 40)
    Sv = np.arange(A + 1, kmax + 2)

    def reshuffles(t):                                           # E[floor(R / S)] for every S in Sv
        out = np.zeros(len(Sv))
        for j in range(1, len(t) // (A + 1) + 2):
            idx = j * Sv
            ok = idx < len(t)
            out[ok] += t[idx[ok]]
        return out

    early = ((A // Sv) + reshuffles(_binom_tail(2 * A, 0.5, kmax)) + reshuffles(_binom_tail(4 * A, 0.25, kmax))
             + max(0, L - 2) * reshuffles(_poisson_tail(A, kmax))) / A
    if L < 2:
        early = (A // Sv) / A
    avgv = (L + 1) + (L + 1) * (2 * Z + Sv) / A + early * (2 * Z + Sv)
    i = int(np.argmin(avgv))
    best = (float(avgv[i]), int(Sv[i]), float(early[i]))
    avg, S, early = best
    nb = 2 ** (L + 1) - 1
    fixed = (n * (L + 64)                                                           # leaf + PMMAC counter
             + nb * (Z * (math.log2(n) + math.log2(Z + S)) + (Z + S) + math.log2(S + 1)))  # bucket metadata
    share = lambda pk: (pk * S_blk + (fixed + pk * (math.log2(n) + L)) / 8) / (n * S_blk)   # + stash entry tags
    out = dict(L=L, Z=Z, A=A, S=S, avg=avg, early=early, worst=(L + 1) + (L + 1) * (2 * Z + S),
               worst_deam=(L + 1) + math.ceil((L + 1) * (2 * Z + S) / A))
    for v, pk in zip(("path", "stream", "pre", "path_deam", "stream_deam"), peaks):
        out["peak_" + v], out["share_" + v] = pk, share(pk)
    return out


@functools.lru_cache(maxsize=None)
def _ring_configs(n, S_blk):
    return tuple(ring_cost(n, S_blk, L, Z, A, pk) for (N, L, Z, A), pk in RING_SIM.items() if N == n)


def ring_points(n, S_blk, variant="path"):
    """Ring ORAM configurations with share = on-package memory for one block-storage variant."""
    return [dict(c, share=c["share_" + variant], peak=c["peak_" + variant]) for c in _ring_configs(n, S_blk)]


def lower_envelope(pts, key):
    out, best = [], math.inf
    for p in sorted(pts, key=lambda p: (p["share"], p[key])):
        if p[key] < best - 1e-9:
            out.append(p)
            best = p[key]
    return out


def best_at(rows, budget, key):
    ok = [r for r in rows if r["share"] <= budget]
    return min(ok, key=lambda r: r[key]) if ok else None


def panel_c_data(n, S, logn):
    pts = partition_points(n, S)
    ptd = partition_points(n, S, deamortized=True)
    rp, rs, r0 = ring_points(n, S, "path"), ring_points(n, S, "stream"), ring_points(n, S, "pre")
    rpd, rsd = ring_points(n, S, "path_deam"), ring_points(n, S, "stream_deam")
    return dict(dom=doma_rows(n, S), bol=[bolt_own(n, S, c) for c in range(1, 17)],
                par=lower_envelope(pts, "avg"), par_w=lower_envelope(pts, "worst"),
                dea=lower_envelope(ptd, "worst"), dea_avg=lower_envelope(ptd, "avg"), pth=path_rows(n, S, logn),
                ring=lower_envelope(rp, "avg"), ring_s=lower_envelope(rs, "avg"), ring_0=lower_envelope(r0, "avg"),
                ring_w=lower_envelope(rpd, "worst_deam"), ring_ws=lower_envelope(rsd, "worst_deam"))


def _fmt_at(rows, budget, key, spec):
    r = best_at(rows, budget, key)
    return format(r[key], spec) if r else format("-", ">5")


def _work_at(rows, budget, key):
    r = best_at(rows, budget, key)
    return r[key] if r else math.inf


def crossings(a_rows, a_key, b_rows, b_key, lo=1e-3, hi=1.0, steps=4000):
    """Budget intervals (on a fine log grid) where design a does less, equal, or more work than design b."""
    out, cur = [], None
    for i in range(steps + 1):
        x = lo * (hi / lo) ** (i / steps)
        wa, wb = _work_at(a_rows, x, a_key), _work_at(b_rows, x, b_key)
        if wa == math.inf:
            continue
        rel = "less" if wa < wb - 1e-9 else ("equal" if abs(wa - wb) < 1e-9 else "more")
        if rel != cur:
            out.append([rel, x, x])
            cur = rel
        out[-1][2] = x
    return out


def print_panel_c(n, S, logn, d):
    dom, bol, par, par_w, dea, dea_avg, pth = (d[k] for k in ("dom", "bol", "par", "par_w", "dea", "dea_avg", "pth"))
    ring, ring_s, ring_0, ring_w, ring_ws = (d[k] for k in ("ring", "ring_s", "ring_0", "ring_w", "ring_ws"))
    pc = lambda x: f"{100 * x:6.2f}%"
    print(f"\n(c) N = 2^{logn}, {S}-byte blocks; work = slots read + written per access; every stash/cache at 2^-64")
    print("DOMA:   Z  lambda  work  on-package")
    for r in dom:
        print(f"      {r['Z']:3d} {r['lam']:6d} {r['work']:5d}  {pc(r['share'])}")
    print("BOLT:   c  l_max  work  on-package   stash (% of N)")
    for r in bol:
        print(f"      {r['c']:3d} {r['lmax']:5d} {r['work']:5d}  {pc(r['share'])}  {100 * r['stash'] / n:6.2f}%")
    print("equal work (DOMA Z = 2 l_max): work | BOLT c l_max on-package | DOMA Z lambda on-package | BOLT/DOMA")
    dz = {r["Z"]: r for r in dom}
    for b in bol:
        e = dz.get(2 * b["lmax"])
        if e:
            print(f"      {b['work']:4d} | {b['c']:2d} {b['lmax']:3d} {pc(b['share'])} | {e['Z']:3d} {e['lam']:3d} "
                  f"{pc(e['share'])} | {b['share'] / e['share']:.2f}x")
    hdr = "   on-package   avg  worst   P        size  T   nu    cap   cache"
    row = lambda p: (f"   {pc(p['share'])} {p['avg']:6.2f} {p['worst']:6.0f} {p['P']:7d} {p['size']:7.1f} {p['T']:2d} "
                     f"{p['nu']:5.3f} {p['cap']:5d} {p['cache']:7d}")
    print("Partition ORAM, basic: least average work at each budget")
    print(hdr)
    for p in par:
        print(row(p))
    print("Partition ORAM, basic: least worst access at each budget")
    print(hdr)
    for p in par_w:
        print(row(p))
    print("Partition ORAM, de-amortized: least worst access at each budget (W = shuffle work per access, pend = "
          "evicted blocks waiting in queued jobs at 2^-64)")
    print(hdr + "     W  backlog   pend")
    for p in dea:
        print(row(p) + f" {p['W']:5d} {p['backlog']:8.0f} {p['pend']:6d}")
    if ring:
        rh = ("   on-package    avg  worst (at once)  worst (de-amortized)  levels       Z       A       S"
              "   blocks (A x 2^-64)   early/access")
        rr = lambda p: (f"   {pc(p['share'])} {p['avg']:6.2f} {p['worst']:15,.0f} {p['worst_deam']:21,.0f}  {p['L'] + 1:5d} "
                        f"{p['Z']:7d} {p['A']:7d} {p['S']:7d} {p['peak']:20,.0f}   {p['early']:.1e}")
        print("Ring ORAM, eviction at once (the access that evicts does (L + 1)(1 + 2Z + S) slots); on-package memory "
              "with the eviction at once")
        for name, env in (("path-buffered (standard EvictPath)", ring), ("oblivious streaming", ring_s),
                          ("stash only, a lower bound", ring_0)):
            print(f"Ring ORAM, {name}, eviction at once: least average work at each budget")
            print(rh)
            for p in env:
                print(rr(p))
        print("Ring ORAM, de-amortized (each eviction spread over the next A accesses); on-package memory includes "
              "the blocks those accesses add to the stash")
        for name, env in (("path-buffered (whole path held)", ring_w), ("oblivious streaming", ring_ws)):
            print(f"Ring ORAM, {name}, de-amortized: least worst access at each budget")
            print(rh)
            for p in env:
                print(rr(p))
    else:
        print(f"Ring ORAM: no simulated eviction peaks for N = 2^{logn} (RING_SIM), not shown")
    print(f"Path ORAM: Z = {PATH_Z}, L = {pth[0]['L']} ({pth[0]['L'] + 1} levels), "
          f"stash {PATH_STASH.get(logn) or PATH_STASH[20]} blocks")
    print("   cached levels k  on-package  work (all levels)  work (cached levels excluded)")
    for r in pth:
        print(f"   {r['k']:3d}  {pc(r['share'])} {r['all']:6d} {r['uncached']:6d}")
    for Z in (8, 16):
        e = dz[Z]
        x = e["share"]
        bb, pp, pw_ = best_at(bol, x, "work"), best_at(par, x, "avg"), best_at(par_w, x, "worst")
        dd, da, tt = best_at(dea, x, "worst"), best_at(dea_avg, x, "avg"), best_at(pth, x, "uncached")
        print(f"\nat DOMA Z = {Z}'s budget ({pc(x)}): DOMA {e['work']} on every access (lambda = {e['lam']})")
        print(f"   BOLT {bb['work']} on every access (c = {bb['c']}, {pc(bb['share'])})" if bb else "   BOLT: none fits")
        print(f"   partition, basic: least average {pp['avg']:.1f} (its worst {pp['worst']:.0f}; P = {pp['P']}, T = {pp['T']}, "
              f"nu = {pp['nu']}); least worst {pw_['worst']:.0f} (avg {pw_['avg']:.1f}, T = {pw_['T']})")
        print(f"   partition, de-amortized: least worst {dd['worst']:.0f} (avg {dd['avg']:.1f}; P = {dd['P']}, T = {dd['T']}, "
              f"nu = {dd['nu']}, W = {dd['W']}, waiting blocks {dd['pend']}); least average {da['avg']:.1f}")
        if ring:
            for name, env in (("path-buffered", ring), ("oblivious streaming", ring_s),
                              ("stash only (lower bound)", ring_0)):
                ra = best_at(env, x, "avg")
                if ra is None:
                    print(f"   Ring ORAM, {name}, eviction at once: no configuration fits")
                    continue
                print(f"   Ring ORAM, {name}, eviction at once: least average {ra['avg']:.1f} ({ra['L'] + 1} levels, "
                      f"Z = {ra['Z']}, A = {ra['A']}, S = {ra['S']}; share {pc(ra['share'])}, stash-only share "
                      f"{pc(ra['share_pre'])}; worst access {ra['worst']:,} at once)")
            for name, wenv in (("path-buffered (whole path held)", ring_w), ("oblivious streaming", ring_ws)):
                rw = best_at(wenv, x, "worst_deam")
                if rw is None:
                    print(f"   Ring ORAM, {name}, de-amortized: no configuration fits")
                    continue
                print(f"   Ring ORAM, {name}, de-amortized: least worst access {rw['worst_deam']:,} ({rw['L'] + 1} "
                      f"levels, Z = {rw['Z']}, A = {rw['A']}, S = {rw['S']}; share {pc(rw['share'])} with arrivals, "
                      f"{pc(rw['share_' + ('path' if wenv is ring_w else 'stream')])} at once; average {rw['avg']:.1f})")
        print(f"   Path ORAM {tt['all']} counting cached levels, {tt['uncached']} without ({tt['k']} levels cached)")
    print("\nDOMA's low-work points (Z <= 8) and the other designs at the same on-package memory:")
    print("     Z lam  memory | DOMA | BOLT | partition de-amortized worst / avg | Ring de-amortized worst / avg at once "
          "(path-buffered; streaming) | Path (all levels)")
    for e in dom:
        if e["Z"] > 8:
            continue
        x = e["share"]
        bb, dd, da, tt = best_at(bol, x, "work"), best_at(dea, x, "worst"), best_at(dea_avg, x, "avg"), best_at(pth, x, "all")
        print(f"   {e['Z']:3d} {e['lam']:3d} {pc(x)} | {e['work']:4d} | " + (f"{bb['work']:4d}" if bb else "   -")
              + f" | {dd['worst']:5.0f} / {da['avg']:5.1f}{'':19s} | "
              + (f"{_fmt_at(ring_w, x, 'worst_deam', '5,.0f')} / {_fmt_at(ring, x, 'avg', '5.1f')}; "
                 f"{_fmt_at(ring_ws, x, 'worst_deam', '5,.0f')} / {_fmt_at(ring_s, x, 'avg', '5.1f')}"
                 if ring else "      -")
              + f" | {tt['all']:4d}")
    print("\nbudget intervals (DOMA's work against ...):")
    for name, rows, key in (("Path ORAM, cached levels not counted", pth, "uncached"),
                            ("partition ORAM basic, average", par, "avg"),
                            ("partition ORAM de-amortized, worst access", dea, "worst"),
                            ("partition ORAM de-amortized, average", dea_avg, "avg"),
                            ("BOLT", bol, "work")) + ((("Ring ORAM path-buffered, average, eviction at once", ring, "avg"),
                                                      ("Ring ORAM oblivious streaming, average, eviction at once",
                                                       ring_s, "avg"),
                                                      ("Ring ORAM path-buffered (whole path held), de-amortized worst "
                                                       "access", ring_w, "worst_deam"),
                                                      ("Ring ORAM oblivious streaming, de-amortized worst access",
                                                       ring_ws, "worst_deam")) if ring else ()):
        iv = crossings(dom, "work", rows, key)
        print(f"   {name}: " + "; ".join(f"DOMA {r} at {100 * a:.2f}-{100 * b:.2f}%" for r, a, b in iv))
    if ring:
        frm = lambda rows, key: next((a for r, a, b in reversed(crossings(dom, "work", rows, key)) if r == "less"), None)
        print(f"DOMA does less than partition ORAM (de-amortized) from {100 * frm(dea_avg, 'avg'):.2f}% (average) and "
              f"{100 * frm(dea, 'worst'):.2f}% (worst access)")


def main(argv=None):
    global ROUND_BITS, PEND_SCALE
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--block", type=int, default=4096, help="block size in bytes (default 4096)")
    ap.add_argument("--logn", type=int, default=20, help="N = 2^logn blocks, both panels (default 20)")
    ap.add_argument("--loads", type=int, nargs="+", default=[2, 4, 6, 8, 10, 12, 14, 16], metavar="C",
                    help="panel (a): average loads c = lambda to compare (default 2 4 ... 16)")
    ap.add_argument("--mark", type=int, default=8, help="panel (a): load at which to annotate the ratio (default 8)")
    ap.add_argument("--round-bits", action="store_true", help="round each DOMA field up to whole bits")
    ap.add_argument("--pend-scale", type=float, default=1.0,
                    help="scale the de-amortized partition ORAM's queued-block bound (sensitivity; default 1)")
    ap.add_argument("--figsize", type=float, nargs=2, metavar=("W", "H"),
                    help="figure size in inches (default 7.16 x 2.65 with Ring ORAM, 7.16 x 2.5 without)")
    ap.add_argument("--out", default="bolt_tradeoff.pdf", help="figure path (.pdf also writes a .png)")
    args = ap.parse_args(argv)
    ROUND_BITS, PEND_SCALE = args.round_bits, args.pend_scale
    S, n = args.block, 2 ** args.logn

    rows = []
    for c in args.loads:
        lmax, wb, rb = bolt_point(n, S, c)
        rd = trusted(n, S, c, lmax)["ratio"]
        rows.append((c, lmax, 2 * lmax, wb, rd, rb))
    print(f"(a) N = 2^{args.logn}, {S}-byte blocks; matched load c = lambda and bucket Z = l_max")
    print(" c=lam  Z=l_max | slots/request DOMA  BOLT | on-package DOMA  BOLT | BOLT/DOMA work, DOMA/BOLT memory")
    for c, z, wd, wb, rd, rb in rows:
        print(f"  {c:4d}  {z:6d}  | {wd:13d} {wb:5d} | {100 * rd:13.1f}% {100 * rb:5.1f}% | {wb / wd:.2f}, {rd / rb:.2f}")
    d = panel_c_data(n, S, args.logn)
    print_panel_c(n, S, args.logn, d)
    dom, bol, dea, dea_avg, pth, ring, ring_s, ring_ws = (d[k] for k in ("dom", "bol", "dea", "dea_avg", "pth", "ring",
                                                                         "ring_s", "ring_ws"))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.ticker import FixedLocator, PercentFormatter

    _style(plt)
    W, H = args.figsize if args.figsize else (7.16, 2.65 if ring else 2.5)
    left, gap, right, bot, top_h = 0.42, 0.62, 0.15, 0.50, 0.80 if ring else 0.66
    avail = W - left - gap - right
    pwa, pwb = 0.42 * avail, 0.58 * avail
    ph = H - bot - top_h
    fig = plt.figure(figsize=(W, H))
    axa = fig.add_axes([left / W, bot / H, pwa / W, ph / H])
    axb = fig.add_axes([(left + pwa + gap) / W, bot / H, pwb / W, ph / H])
    cd, cb, cp, cq = SERIES[0], SERIES[1], SERIES[2], SERIES[3]
    dash = (0, (3, 1.6))

    # (a) on-package memory at matched load and bucket size
    cs = [r[0] for r in rows]
    axa.plot(cs, [100 * r[4] for r in rows], color=cd, lw=1.3, marker="o", ms=2.2, zorder=3)
    axa.plot(cs, [100 * r[5] for r in rows], color=cb, lw=1.3, marker="o", ms=2.2, zorder=3)
    axa.set_xlim(cs[0] - 0.8, cs[-1] + 0.8)
    axa.grid(True, axis="y", color=GRID, lw=0.5, zorder=0)
    axa.tick_params(labelsize=6)
    ticks = cs
    axa.xaxis.set_major_locator(FixedLocator(ticks))
    axa.set_xticklabels([str(c) for c in ticks])
    zoff = bolt_point(n, S, cs[0])[0] - cs[0]                       # Z = l_max = lambda + ceil(log2 log2 N)
    m = next(r for r in rows if r[0] == args.mark)
    lo, hi = 100 * m[5], 100 * m[4]
    axa.annotate("", xy=(m[0], hi), xytext=(m[0], lo),
                 arrowprops=dict(arrowstyle="<->", color=INK2, lw=0.7, shrinkA=2, shrinkB=2))
    axa.annotate(f"{m[4] / m[5]:.1f}$\\times$", xy=(m[0], 0.86 * (lo * hi) ** 0.5), xytext=(3, 0),
                 textcoords="offset points", ha="left", va="center", fontsize=6.5, color=INK2)
    axa.set_yscale("log")
    axa.set_ylim(3, 60)
    axa.yaxis.set_major_locator(FixedLocator([3, 5, 10, 20, 50]))
    axa.yaxis.set_minor_locator(FixedLocator([]))
    axa.yaxis.set_major_formatter(PercentFormatter(decimals=0))
    axa.set_ylabel("on-package memory (% of data)", fontsize=6.5, labelpad=2)
    axa.set_xlabel(f"average load $\\lambda$ (bucket size Z = $\\lambda$ + {zoff},\n"
                   "BOLT's bin size from its own analysis)", fontsize=6.5, labelpad=2, linespacing=1.2)
    axa.set_title("(a) on-package memory", loc="left", fontsize=6.5, color=INK, pad=3)

    # (b) work per access against on-package memory, best parameters at each budget
    xmax = 0.8
    axb.plot([100 * r["share"] for r in dom], [r["work"] for r in dom], color=cd, lw=1.3, marker="o", ms=2.0, zorder=4)
    bb = [r for r in bol if r["share"] <= xmax]
    axb.plot([100 * r["share"] for r in bb], [r["work"] for r in bb], color=cb, lw=1.3, marker="o", ms=2.0, zorder=4)
    for env, key, ls, lw in ((dea, "worst", "-", 1.2), (dea_avg, "avg", dash, 0.9)):   # de-amortized partition ORAM
        e = [p for p in env if p["share"] <= xmax]
        axb.step([100 * p["share"] for p in e] + [100 * xmax], [p[key] for p in e] + [e[-1][key]], where="post",
                 color=cp, lw=lw, ls=ls, zorder=3)
    dashdot = (0, (4, 1.4, 1, 1.4))
    for env, key, ls, lw in ((ring_ws, "worst_deam", "-", 1.2), (ring_s, "avg", dashdot, 0.9)):
        e = [p for p in env if p["share"] <= xmax]
        if e:
            axb.step([100 * p["share"] for p in e] + [100 * xmax], [p[key] for p in e] + [e[-1][key]], where="post",
                     color=SERIES[6], lw=lw, ls=ls, zorder=3)
    axb.plot([100 * pth[0]["share"], 100 * xmax], [pth[0]["all"]] * 2, color=cq, lw=1.3, zorder=3)   # Path ORAM
    z8 = next(r for r in dom if r["Z"] == 8)
    axb.axvline(100 * z8["share"], color=MUTED, lw=0.6, ls=(0, (1, 1.5)), zorder=1)                  # DOMA Z = 8
    axb.set_yscale("log")
    axb.set_xlim(0, 100 * xmax)
    axb.set_ylim(3, 200)                                     # curves above 200 are clipped
    axb.xaxis.set_major_locator(FixedLocator(list(range(0, 81, 10))))
    axb.xaxis.set_minor_locator(FixedLocator([]))
    axb.set_xticklabels([f"{v}%" for v in range(0, 81, 10)])
    axb.yaxis.set_major_locator(FixedLocator([5, 10, 20, 50, 100, 200]))
    axb.yaxis.set_minor_locator(FixedLocator([]))
    axb.set_yticklabels(["5", "10", "20", "50", "100", "200"])
    axb.grid(True, axis="y", color=GRID, lw=0.5, zorder=0)
    axb.tick_params(labelsize=6)
    axb.set_ylabel("slots read and written per access", fontsize=6.5, labelpad=2)
    axb.set_xlabel("on-package memory (% of data)", fontsize=6.5, labelpad=2)
    axb.set_title("(b) work per access against on-package memory", loc="left", fontsize=6.5, color=INK, pad=3)

    blk = f"{S // 1024} KB" if S >= 1024 else f"{S} B"
    hdr = (f"{blk} blocks, N = $2^{{{args.logn}}}$. (a): same load and bucket size. "
           f"(b): stashes and caches at $2^{{-64}}$ per access, best parameters at each budget")
    fig.text(left / W, (H - 0.10) / H, hdr, ha="left", va="center", color=INK, fontsize=6.5)
    L2 = lambda c, lab, ls="-", lw=1.3, mk="o": Line2D([], [], color=c, lw=lw, ls=ls, marker=mk, ms=2.2, label=lab)
    handles = [L2(cd, "DOMA: 1 bucket per request"), L2(cb, "BOLT: 2 bins per request"),
               L2(cq, "Path ORAM (Tiny ORAM)", mk=None),
               L2(cp, "partition ORAM (de-amortized): worst access", lw=1.2, mk=None),
               L2(cp, "partition ORAM (de-amortized): average", ls=dash, lw=0.9, mk=None)]
    if ring:
        handles += [Line2D([], [], lw=0, label=" "),
                    L2(SERIES[6], "Ring ORAM (de-amortized, streaming): worst access", lw=1.2, mk=None),
                    L2(SERIES[6], "Ring ORAM (streaming, eviction at once): average", ls=(0, (4, 1.4, 1, 1.4)),
                       lw=0.9, mk=None)]
    fig.legend(handles=handles, loc="upper left", bbox_to_anchor=(left / W - 0.012, (H - 0.20) / H), ncol=3,
               frameon=False, fontsize=6.5, handlelength=2.0, handletextpad=0.4, columnspacing=1.4,
               borderaxespad=0, borderpad=0, labelspacing=0.35)
    save(fig, plt, args.out)


if __name__ == "__main__":
    sys.exit(main())
