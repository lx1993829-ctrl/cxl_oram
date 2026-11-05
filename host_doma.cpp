// host_doma.cpp -- XRT host program for doma_kernel, built to match BOLT's benchmark host
// (BOLT-benchmark/BOLT/*/host.cpp) so the two are measured the same way:
//   * same workload generator: std::mt19937 seed 1, 50% PUT, keys uniform over the dataset,
//     VALUE_SIZE random bytes per PUT (DOMA stores them at the start of a 4 KB block)
//   * keys are dense: key index i is DOMA slot i (BOLT looks keys up through its hash map)
//   * commands are converted and copied to the card BEFORE timing starts, as BOLT does
//   * query time = one kernel launch processing all CMD_SIZE commands, plus wait(); when they do not fit
//     the command and response buffers (1M requests from 512-byte blocks up), batches of the largest count
//     that fits, each staged before its timer starts, and the time is the sum of their launch + wait()
//   * init time = boot run (key load, bucket layout, lease) + preloading every slot,
//     the analogue of BOLT's data preparation and loading
// Usage: ./host_doma <xclbin> [CMD_SIZE] [--kernel NAME]
//   NAME (or DOMA_KERNEL=NAME): doma_kernel (default) or tinyoram_kernel, the Tiny ORAM baseline.
//   Everything else -- generator, seeds, preload counted as initialization, the timed region of one
//   launch plus wait(), the DOMA_* variables -- is the same for both kernels. For tinyoram_kernel:
//     DOMA_N      records; must match the bitstream's tree (Tiny ORAM's depth is fixed per build:
//                 N = 4 * 2^L blocks, L >= 6 the smallest with 4 * 2^L >= blocks, as build_tinyoram.sh picks it);
//                 default: the bitstream's N
//     DOMA_BLOCK  Tiny ORAM's block size in bytes (64, 128, 256 or 4096), used to size the command, response
//                 and tree buffers before the first launch (default 4096, which also covers the smaller builds)
//     DOMA_PACK   as for DOMA: BLOCK / 8 records per block (offset and length in the command header,
//                 written through Tiny ORAM's byte write mask); DOMA_N records then need N/(BLOCK/8) blocks
//   Tiny ORAM launches with 4 arguments (cmd, resp, pages, size) and needs no metadata buffers.
#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <random>
#include <string>
#include <vector>
#include "doma_debug.h"                    // v14b: halt record and debug-build records

// must match the hardware build (rtl/oram_params.vh, single-client default)
// run-time size (DOMA_N=<records>): n blocks in NB = ceil(n / LAMBDA) buckets, passed to the kernel as nblk
// (the build's maximum is rtl/oram_params.vh ORAM_N; build.sh NMAX=...). Default: 256 blocks, as before.
static uint32_t LAMBDA     = 6;                // rtl/oram_params.vh ORAM_C; DOMA_LAMBDA=<lambda> for a LAMBDA=7 build (checked at init)
static const uint32_t ZPOS       = 8;                // rtl/oram_params.vh ORAM_Z

// Stash pool size: Claim 1 of the stash analysis (docs/stash_chain_design.md). The stash exceeds xR + xO with
// probability at most gamma = 2^-64 per access, for every access sequence (Chernoff bounds over negatively
// associated per-bucket terms; the best split of gamma between the two terms). Per-bucket counts are
// Binomial(n, 1/B) with B = n / lambda from the start: since v11b the boot places every block at an independent
// uniform bucket (the excess over Z waits in the stash, nothing else does), and every remap keeps it so.
static double ln_mgf_A(double t, double Bk, double n) {     // ln E[e^{tA}], P(A >= i) = (1 - 1/B)^i, A capped at n
    const double p = 1.0 / Bk, lr = std::log1p(-p) + t;
    if (std::fabs(lr) < 1e-15) return std::log(p * n + 1.0);
    if (lr > 0) { const double lrn = n * lr; if (lrn > 700) return INFINITY;
                  return std::log(p * std::expm1(lrn) / std::expm1(lr) + std::exp(lrn)); }
    const double rn = std::exp(n * lr);
    return std::log(p * (1 - rn) / (-std::expm1(lr)) + rn);
}
static double stash_xR(double Bk, double n, double lg) {   // lg = ln(1 / gamma_R)
    double best = INFINITY;
    for (int i = 1; i < 400; i++) { const double u = i * 0.005, v = ln_mgf_A(std::expm1(u) / Bk, Bk, n);
        if (!std::isfinite(v)) break; best = std::min(best, (Bk * v + lg) / u); }
    return best;
}
static double stash_xO(double Bk, double n, int Z, double lg) {   // per-bucket count Binomial(n, 1/B)
    double best = INFINITY; std::vector<double> lt(160); const double p = 1.0 / Bk, lp = std::log(p), lq = std::log1p(-p);
    for (int i = 1; i < 400; i++) { const double u = i * 0.01; double mx = -INFINITY, sum = 0;
        for (int k = 0; k < 160; k++) { lt[k] = std::lgamma(n + 1) - std::lgamma(k + 1.0) - std::lgamma(n - k + 1) + k * lp + (n - k) * lq
                                                + u * std::max(0, k - Z); mx = std::max(mx, lt[k]); }
        for (int k = 0; k < 160; k++) sum += std::exp(lt[k] - mx);
        best = std::min(best, (Bk * (mx + std::log(sum)) + lg) / u); }
    return best;
}
static double stash_bound(double n, double lam, int Z) {
    const double Bk = std::ceil(n / lam), G = 64 * std::log(2.0);   // n is a multiple of lambda (the host rounds it): B = n / lambda
    double best = INFINITY;
    for (double f : {0.1, 0.3, 0.5, 0.7, 0.9})
        best = std::min(best, stash_xR(Bk, n, G - std::log(f)) + stash_xO(Bk, n, int(Z), G - std::log(1 - f)));
    return best;
}
static uint32_t       NBLK       = 256;              // blocks (slots)
static uint32_t       NB         = 64;               // buckets
static const size_t   MAX_BLOCK  = 4096;             // largest block (4 KB); buffers are sized for it
static size_t         BLOCK      = 4096;             // bytes per slot: the kernel reports its own after boot
static const size_t   BUCKET_STRIDE = 32768;         // bucket spacing in DOMA's memory map
static const int      VALUE_SIZE = 8;                // BOLT's default value size (bytes)
static uint32_t       PACK       = 1;                // records per block: 1, or BLOCK / VALUE_SIZE with DOMA_PACK=1
static uint32_t       NKEYS      = 256;              // keys (records); slots x records per block by default
static const uint8_t  T_GET = 0x01, T_PUT = 0x02, T_ACK = 0x10, T_SUM = 0x5A;

static size_t roundup4k(size_t x) { return (x + 4095) & ~size_t(4095); }

struct Cmd { uint8_t type; uint32_t key; uint8_t value[VALUE_SIZE]; };

// BOLT's generate_uniform_commands(), with the key index used directly as the key
static std::vector<Cmd> generate_uniform_commands(int size) {
    std::vector<Cmd> cmds;
    uint32_t seed = 1;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> op_dis(0.0, 1.0);
    std::uniform_int_distribution<int> key_dis(0, NKEYS - 1);
    std::uniform_int_distribution<int> value_dis(0, 255);
    for (int i = 0; i < size; i++) {
        Cmd c{};
        double rw = op_dis(gen);
        c.key = key_dis(gen);
        if (rw < 0.5) { c.type = T_PUT; for (int j = 0; j < VALUE_SIZE; j++) c.value[j] = (uint8_t)value_dis(gen); }
        else c.type = T_GET;
        cmds.push_back(c);
    }
    return cmds;
}

struct Buffers {
    xrt::bo cmd, resp, pages, meta, brec, stash;   // meta: position map; brec: bucket records; stash (all in HBM)
    xrt::bo pages_hbm;                             // DOMA: buckets kept in HBM (DOMA_HBM_PCT, fixed boundary)
    size_t hc = 0, hr = 0;
};

// lay commands out as doma_kernel expects: 32-byte headers, then PUT data on 4 KB boundaries
static void stage(Buffers& b, const std::vector<Cmd>& cmds) {
    size_t n = cmds.size();
    b.hc = roundup4k(n * 32);
    b.hr = roundup4k((n + SUMREC) * 32);              // after records [n] (summary), [n+1] (traffic) (and [n+2..n+4], debug)
    auto p = b.cmd.map<uint8_t*>();
    std::memset(p, 0, b.hc + n * BLOCK);
    for (size_t i = 0; i < n; i++) {
        // key k is record k % PACK of block k / PACK; a packed PUT writes only its record
        // (offset and length in header bytes 8-15); unpacked commands use length 0 (whole block)
        uint32_t slot = cmds[i].key / PACK, off = (cmds[i].key % PACK) * VALUE_SIZE;
        uint32_t len  = (PACK > 1) ? uint32_t(VALUE_SIZE) : 0u;
        p[i * 32] = cmds[i].type;
        std::memcpy(p + i * 32 + 4, &slot, 4);
        std::memcpy(p + i * 32 + 8, &off, 4);
        std::memcpy(p + i * 32 + 12, &len, 4);
        if (cmds[i].type == T_PUT) std::memcpy(p + b.hc + i * BLOCK + off, cmds[i].value, VALUE_SIZE);
    }
    b.cmd.sync(XCL_BO_SYNC_BO_TO_DEVICE, b.hc + n * BLOCK, 0);
    // DOMA_DIAG=1 (diagnosis): fill the response data slots with 0xA5 before the launch, so a data slot the kernel
    // never writes reads back as A5.., not as zeros or an older launch's data
    if (std::getenv("DOMA_DIAG")) {
        auto r = b.resp.map<uint8_t*>();
        std::memset(r + b.hr, 0xA5, n * BLOCK);
        b.resp.sync(XCL_BO_SYNC_BO_TO_DEVICE, n * BLOCK, b.hr);
    }
}

struct Summary { uint64_t total, rd_beats, wr_beats; uint32_t lam, beats, lat_min, lat_max, errors, done, dbg; bool ok, traffic;
                 uint32_t geo_n; size_t geo_tree; };   // tinyoram_kernel only: traffic-record bytes 24-27 blocks, 28-31 tree (64-byte bursts)

static Summary read_summary(Buffers& b, size_t n) {
    // at least through the summary record itself (b.hr is 0 before the first stage())
    b.resp.sync(XCL_BO_SYNC_BO_FROM_DEVICE, std::max(b.hr + n * BLOCK, (n + SUMREC) * size_t(32)), 0);
    auto r = b.resp.map<uint8_t*>() + n * 32;
    Summary s{};
    s.ok = (r[0] == T_SUM); s.lam = r[1] ? r[1] : 6;   // byte 1: lambda when not 6
    s.dbg = s.ok ? r[2] : 0;                           // byte 2: debug bitstream flags (doma_debug.h)
    std::memcpy(&s.beats, r + 4, 4);   // block size in beats (0 from bitstreams before this field)
    std::memcpy(&s.total, r + 8, 8);   std::memcpy(&s.lat_min, r + 16, 4); std::memcpy(&s.lat_max, r + 20, 4);
    std::memcpy(&s.errors, r + 24, 4); std::memcpy(&s.done, r + 28, 4);
    // record [n+1]: traffic on the bucket port during the launch (0x5B; absent from older bitstreams)
    s.traffic = (r[32] == 0x5B);
    if (s.traffic) { std::memcpy(&s.rd_beats, r + 40, 8); std::memcpy(&s.wr_beats, r + 48, 8);
                     uint32_t gt; std::memcpy(&s.geo_n, r + 56, 4); std::memcpy(&gt, r + 60, 4); s.geo_tree = size_t(gt) * 64; }
    return s;
}

// Tiny ORAM's tree span in bytes for a block of `block` bytes and depth L (ORAML), Z = 4, PMMAC on:
// the formulas of its include/BucketLocal.vh (buckets of 64-byte bursts, subtree layout per DDR3 row)
static size_t tinyoram_tree_bytes(size_t block, unsigned L) {
    const unsigned Z = 4, U = 32, E = 64, H = 128;
    unsigned hdr = E + 8 * ((Z + 7) / 8) + U * Z + L * Z + H * Z;                // header bits
    size_t bkt = (hdr + 511) / 512 + Z * ((block * 8 + 511) / 512);             // bucket, in 64-byte bursts
    size_t words = bkt * 8, x = 8192 / words + 1;
    unsigned lst = 0; while ((size_t(2) << lst) <= x) lst++;                      // log2 floor
    if (lst == 0) lst = 1;
    unsigned nst = (L + 1 + lst - 1) / lst;
    size_t tot = ((size_t(1) << (nst * lst)) - 1) / ((size_t(1) << lst) - 1);
    return 64 * ((size_t(1) << (L + 1)) + tot) * bkt;
}
static unsigned tinyoram_depth(size_t blocks) { unsigned L = 6; while ((size_t(4) << L) < blocks) L++; return L; }

int main(int argc, char** argv) {
    if (const char* l = std::getenv("DOMA_LAMBDA")) LAMBDA = uint32_t(std::stoul(l));   // the bitstream's lambda (build.sh LAMBDA)
    if (argc < 2) { std::printf("usage: %s <xclbin> [CMD_SIZE] [--kernel doma_kernel|tinyoram_kernel]\n", argv[0]); return 1; }
    int CMD_SIZE = 5000;                                                   // BOLT: 5000
    std::string kname = std::getenv("DOMA_KERNEL") ? std::getenv("DOMA_KERNEL") : "doma_kernel";
    for (int a = 2; a < argc; a++) {
        std::string o = argv[a];
        if (o == "--kernel" && a + 1 < argc) kname = argv[++a];
        else if (a == 2 && !o.empty() && o[0] != '-') CMD_SIZE = std::stoi(o);
        else { std::printf("unknown argument %s\n", o.c_str()); return 1; }
    }
    const bool TINY = (kname == "tinyoram_kernel");
    if (!TINY && kname != "doma_kernel") { std::printf("unknown kernel %s (doma_kernel or tinyoram_kernel)\n", kname.c_str()); return 1; }
    auto device = xrt::device(0);
    auto uuid   = device.load_xclbin(argv[1]);
    auto krnl   = xrt::kernel(device, uuid, kname);

    // size: DOMA_BLOCK (bytes, default 4 KB) sizes the buffers before boot; checked against the kernel after
    const size_t BLOCKA = std::getenv("DOMA_BLOCK") ? std::stoul(std::getenv("DOMA_BLOCK")) : MAX_BLOCK;
    if (BLOCKA < 32 || BLOCKA > MAX_BLOCK || (BLOCKA & (BLOCKA - 1))) { std::printf("DOMA_BLOCK must be a power of two, 32..4096\n"); return 1; }
    const uint32_t pack_a = std::getenv("DOMA_PACK") ? uint32_t(BLOCKA / VALUE_SIZE) : 1u;
    if (std::getenv("DOMA_N")) {
        NKEYS = uint32_t(std::stoul(std::getenv("DOMA_N")));
        NBLK  = (NKEYS + pack_a - 1) / pack_a;
    }
    // the stash analysis takes B = n / lambda exactly: round n up to a multiple of lambda (at most lambda - 1 extra
    // blocks, never requested). (Since v11b the boot places every block at an independent uniform bucket, so the
    // per-bucket counts are Binomial(n, 1/B) for any n; the rounding is kept, and costs at most lambda - 1 blocks.) The kernel clamps n to its build's maximum: DOMA_NMAX=<that maximum> makes a rounding past it an error.
    const uint32_t nblk_req = NBLK;
    if (!TINY) NBLK = ((NBLK + LAMBDA - 1) / LAMBDA) * LAMBDA;
    if (!TINY && NBLK != nblk_req) std::printf("n: %u blocks rounded up to %u, a multiple of lambda = %u (the extra blocks are never requested)\n", nblk_req, NBLK, LAMBDA);
    if (!TINY && std::getenv("DOMA_NMAX") && NBLK > std::stoul(std::getenv("DOMA_NMAX"))) {
        std::printf("n = %u exceeds the build's maximum %s: the kernel would clamp it to a size that is not a multiple of lambda\n", NBLK, std::getenv("DOMA_NMAX")); return 1; }
    NB = (NBLK + LAMBDA - 1) / LAMBDA;
    const size_t stride = size_t(ZPOS) * BLOCKA;                              // compact layout: exactly Z blocks per bucket
    // stash pool: Claim 1's bound at 2^-64 per access, rounded up, plus one; never more than n. DOMA_POOL=<entries> overrides it.
    const double pool_bnd = stash_bound(double(NBLK), double(LAMBDA), int(ZPOS));
    uint32_t POOL = uint32_t(std::min<double>(NBLK, std::ceil(pool_bnd) + 1));
    if (std::getenv("DOMA_POOL")) POOL = uint32_t(std::min<unsigned long>(NBLK, std::stoul(std::getenv("DOMA_POOL"))));
    // Tiny ORAM: its tree for the requested size (the bitstream reports the real one at init; checked below)
    const unsigned tiny_L = tinyoram_depth(NBLK);
    // DOMA_HBM_PCT: the percentage of buckets kept in HBM (fixed boundary: buckets 0 .. HBMK-1), the rest in pages
    const unsigned hbm_pct = std::getenv("DOMA_HBM_PCT") ? unsigned(std::stoul(std::getenv("DOMA_HBM_PCT"))) : 0u;
    if (hbm_pct > 100) { std::printf("DOMA_HBM_PCT must be 0..100\n"); return 1; }
    if (TINY && hbm_pct) { std::printf("DOMA_HBM_PCT is for doma_kernel only\n"); return 1; }
    const uint32_t HBMK = uint32_t((uint64_t(NB) * hbm_pct) / 100);
    const size_t pages_bytes = TINY ? roundup4k(tinyoram_tree_bytes(BLOCKA, tiny_L))
                                    : size_t(NB - HBMK) * stride + 4096;      // + up to 4 KB: the kernel rebases by a 4 KB multiple
    if (TINY) std::printf("size: %u blocks, Tiny ORAM tree for L=%u (N=%u), %zu-byte blocks: %zu bytes\n",
                          NBLK, tiny_L, 4u << tiny_L, BLOCKA, pages_bytes);
    if (!TINY) std::printf("stash pool: %u entries (Claim 1 bound %.1f at 2^-64 per access, rounded up, + 1)%s\n", POOL, pool_bnd,
                           std::getenv("DOMA_POOL") ? " [DOMA_POOL]" : "");
    if (TINY) {} else std::printf("size: %u blocks (nblk), %u buckets (%u in HBM = %u%%, %u in pages), buffers sized for %zu-byte blocks\n",
                     NBLK, NB, HBMK, hbm_pct, NB - HBMK, BLOCKA);
    // requests per launch: all CMD_SIZE when they fit the command and response buffers (one launch, as BOLT
    // times it); otherwise batches of the largest count that does. The buffers sit in 512 MiB HBM
    // pseudo-channels; each stays below 448 MiB. DOMA_BATCH=<n> sets the batch (testing).
    size_t batch = std::max<size_t>(1, (size_t(448) << 20) / (32 + BLOCKA));
    if (const char* e = std::getenv("DOMA_BATCH")) batch = std::max<size_t>(1, std::stoul(e));
    batch = std::min<size_t>(batch, size_t(CMD_SIZE));
    const size_t max_cmds = std::max<size_t>(batch, std::min<size_t>(NBLK, 5000));
    Buffers b;
    b.cmd   = xrt::bo(device, roundup4k(max_cmds * 32) + max_cmds * BLOCKA, krnl.group_id(0));
    b.resp  = xrt::bo(device, roundup4k((max_cmds + 5) * 32) + max_cmds * BLOCKA, krnl.group_id(1));   // (+5: a debug bitstream's records)
    b.pages = std::getenv("DOMA_PAGES_HBM") ? xrt::bo(device, pages_bytes, krnl.group_id(2)) : xrt::bo(device, pages_bytes, xrt::bo::flags::host_only, krnl.group_id(2));
    if (!TINY) {
    // DOMA's trusted metadata (position map, 4 bytes per slot) in on-package HBM; DOMA writes all of it at boot
    b.meta  = xrt::bo(device, std::max<size_t>(4096, ((size_t(NBLK) + 4) / 5) * 32), krnl.group_id(4));   // 48-bit entries, 5 per 32-byte beat
    // bucket records: 256 bytes per bucket (metadata + each position's IV/tag), written by DOMA at boot
    b.brec  = xrt::bo(device, std::max<size_t>(4096, size_t(NB) * 64), krnl.group_id(5));    // 64-byte records (per-bucket GCM)
    // the stash: 512-byte lists per bucket, then a pool with one block per slot
    // the stash: 32-byte heads per bucket, 16-byte links per pool entry, then the pool's blocks
    // heads: 64 bytes per bucket. A head holds Z + 2 entries; at Z = 10 it takes two 32-byte beats when the build's
    // pool index needs more than 20 bits (NMAX above 2^20), and the kernel then lays out its regions after NB * 64.
    b.stash = xrt::bo(device, roundup4k(size_t(NB) * 64) + roundup4k(size_t(POOL) * 16) + size_t(POOL) * BLOCKA, krnl.group_id(6));
    // buckets kept in HBM: buckets 0 .. HBMK-1 (DOMA formats them at boot, like pages)
    b.pages_hbm = xrt::bo(device, std::max<size_t>(4096, size_t(HBMK) * stride), krnl.group_id(8));
    }
    // DOMA: 10 arguments; Tiny ORAM: cmd, resp, pages, size
    auto launch = [&](uint32_t n) {
        return TINY ? krnl(b.cmd, b.resp, b.pages, n)
                    : krnl(b.cmd, b.resp, b.pages, n, b.meta, b.brec, b.stash, NBLK, b.pages_hbm, HBMK, POOL);
    };

    // ---- init: boot run, then preload every key (BOLT's init includes loading its dataset)
    auto init_start = std::chrono::system_clock::now();
    auto r0 = launch(0u);                                                  // DOMA: key load, layout, lease; Tiny ORAM: latch
    r0.wait();
    {   // the kernel reports its block size; the data layout must match the bitstream
        Summary s0 = read_summary(b, 0);
        if (!TINY) std::printf("boot: %llu device cycles, errors 0x%x%s", (unsigned long long)s0.total, s0.errors, s0.errors ? "" : "\n");
        if (!TINY && s0.errors) std::printf(" (halted at boot:%s%s%s)\n",
            (s0.errors & 0x100) ? " timing overrun" : "", (s0.errors & 0x200) ? " integrity failure" : "", (s0.errors & 0x400) ? " stash overflow" : "");
        if (s0.errors & 0x8) print_halt_record("boot", s0.lat_min, s0.lat_max);   // in place of the latency fields
        if (!TINY && s0.ok) { const uint8_t sb[3] = {0, 0, uint8_t(s0.dbg)}; dbg_from_summary(sb); }
        if (!TINY && s0.ok && s0.lam != LAMBDA) {   // before any other check: a lambda mismatch also mis-sizes the stash pool
            std::printf("the bitstream's lambda is %u but the host sized the ORAM for lambda = %u: set DOMA_LAMBDA=%u\n", s0.lam, LAMBDA, s0.lam);
            return 1; }
        if (s0.ok && s0.beats > 0 && s0.beats * 32 <= MAX_BLOCK) BLOCK = size_t(s0.beats) * 32;
        std::printf("block: %zu bytes per slot%s\n", BLOCK, (s0.beats == 0) ? " (bitstream does not report it; assuming 4 KB)" : "");
        if (VALUE_SIZE > BLOCK) { std::printf("VALUE_SIZE %d exceeds the block\n", VALUE_SIZE); return 1; }
        if (BLOCK != BLOCKA && (std::getenv("DOMA_BLOCK") || std::getenv("DOMA_N"))) {
            std::printf("the kernel's block is %zu bytes but DOMA_BLOCK sized the buffers for %zu: set DOMA_BLOCK=%zu\n", BLOCK, BLOCKA, BLOCK);
            return 1; }
    }
    if (TINY) {   // the bitstream's geometry; Tiny ORAM has touched no memory yet (it zero-fills at the next launch)
        Summary g = read_summary(b, 0);
        if (!g.ok || !g.traffic || g.geo_n == 0) { std::printf("tinyoram_kernel did not report its geometry\n"); return 1; }
        unsigned bl = 1; while ((4u << bl) < g.geo_n) bl++;
        std::printf("Tiny ORAM bitstream: N=%u blocks (L=%u), tree %zu bytes\n", g.geo_n, bl, g.geo_tree);
        if (std::getenv("DOMA_N") && g.geo_n != (4u << tiny_L)) {
            std::printf("DOMA_N=%u records (%u blocks of %u) need a Tiny ORAM build with L=%u (N=%u blocks); this bitstream has N=%u (L=%u)\n",
                        NKEYS, NBLK, pack_a, tiny_L, 4u << tiny_L, g.geo_n, bl);
            return 1; }
        if (!std::getenv("DOMA_N")) NBLK = g.geo_n;
        if (g.geo_tree > b.pages.size()) {
            std::printf("this bitstream's tree (%zu bytes) is larger than the pages buffer (%zu bytes): run with DOMA_BLOCK=%zu and DOMA_N for its N=%u blocks\n",
                        g.geo_tree, b.pages.size(), BLOCK, g.geo_n);
            return 1; }
    }
    if (std::getenv("DOMA_PACK")) PACK = uint32_t(BLOCK / VALUE_SIZE);    // records share a block
    if (!std::getenv("DOMA_N")) NKEYS = NBLK * PACK;
    std::printf("records: %u keys of %d bytes, %u per block\n", NKEYS, VALUE_SIZE, PACK);
    std::vector<uint8_t> ref(size_t(NKEYS) * VALUE_SIZE);
    std::vector<Cmd> preload;
    std::mt19937 vgen(16);                                                // BOLT seeds its dataset with 16
    std::uniform_int_distribution<int> vdis(0, 255);
    for (uint32_t k = 0; k < NKEYS; k++) {
        Cmd c{}; c.type = T_PUT; c.key = k;
        for (int j = 0; j < VALUE_SIZE; j++) c.value[j] = (uint8_t)vdis(vgen);
        std::memcpy(&ref[size_t(k) * VALUE_SIZE], c.value, VALUE_SIZE);
        preload.push_back(c);
    }
    uint32_t pre_err = 0; bool halt_shown = false;
    // a halted launch carries the halt record in its latency fields: print it from the first one, and again from the
    // last (its missed time is final there if it was still counting in the first)
    Summary last_halt{}; std::string last_where;
    auto note_halt = [&](const Summary& p, const std::string& where) {
        if (!(p.errors & 0x8) || TINY) return;
        if (!halt_shown) { print_halt_record(where.c_str(), p.lat_min, p.lat_max); halt_shown = true; }
        last_halt = p; last_where = where;
    };
    for (size_t at = 0; at < preload.size(); at += max_cmds) {             // in batches that fit the buffers
        std::vector<Cmd> part(preload.begin() + at, preload.begin() + std::min(preload.size(), at + max_cmds));
        stage(b, part);
        auto r1 = launch((uint32_t)part.size());
        r1.wait();
        Summary p = read_summary(b, part.size());
        pre_err |= p.errors;
        note_halt(p, "preload, launch at key " + std::to_string(at));
        if (SUMREC == 5 && at + max_cmds >= preload.size())
            print_debug_records("after the preload", b.resp.map<uint8_t*>() + (part.size() + 2) * 32);
    }
    auto init_end = std::chrono::system_clock::now();
    std::printf("init: %.6f s (boot + preload of %u keys), preload errors=0x%x\n",
                std::chrono::duration<double>(init_end - init_start).count(), NKEYS, pre_err);

    // ---- query run, timed exactly like BOLT: launch + wait, commands already on the card. Requests that do not
    // fit one launch's buffers run in batches: each is staged before its timer starts, the time is the sum of the
    // batches' launch + wait, and every response is checked batch by batch against the same reference. A batch
    // that reports an error ends the run (no further requests go to a halted design).
    auto cmds = generate_uniform_commands(CMD_SIZE);
    double t = 0; size_t launches = 0;
    Summary s{}; s.ok = true; s.traffic = true; s.lat_min = UINT32_MAX;
    int bad = 0, gets = 0, puts = 0;
    uint32_t dmin = UINT32_MAX, dmax = 0, wmin = UINT32_MAX, wmax = 0;   // Tiny ORAM's secondary latencies
    for (size_t at = 0; at < cmds.size(); at += batch) {
        std::vector<Cmd> part(cmds.begin() + at, cmds.begin() + std::min(cmds.size(), at + batch));
        stage(b, part);
        auto start = std::chrono::system_clock::now();
        auto run = launch((uint32_t)part.size());
        run.wait();
        auto end = std::chrono::system_clock::now();
        t += std::chrono::duration<double>(end - start).count(); launches++;
        Summary p = read_summary(b, part.size());
        s.ok = s.ok && p.ok; s.lam = p.lam; s.beats = p.beats; s.total += p.total; s.done += p.done; s.errors |= p.errors;
        s.traffic = s.traffic && p.traffic; s.rd_beats += p.rd_beats; s.wr_beats += p.wr_beats;
        s.lat_min = std::min(s.lat_min, p.lat_min); s.lat_max = std::max(s.lat_max, p.lat_max);
        if (p.errors) { s.lat_min = p.lat_min; s.lat_max = p.lat_max; }   // a halted launch reports its halt record here
        note_halt(p, "query, launch " + std::to_string(launches));
        if (SUMREC == 5 && (p.errors || at + batch >= cmds.size()))
            print_debug_records("after the query", b.resp.map<uint8_t*>() + (part.size() + 2) * 32);
        // ---- verify every response of this batch against the reference
        auto resp = b.resp.map<uint8_t*>();
        for (size_t jj = 0; jj < part.size(); jj++) {
            const size_t i = at + jj;
            const uint8_t* h = resp + jj * 32;
            uint32_t slot; std::memcpy(&slot, h + 4, 4);                  // the kernel echoes the block
            uint32_t key = part[jj].key, off = (key % PACK) * VALUE_SIZE;
            if (h[0] != T_ACK || slot != key / PACK) { if (bad++ < 5) std::printf("cmd %zu: bad header 0x%02x block %u\n", i, h[0], slot); continue; }
            if (part[jj].type == T_PUT) { std::memcpy(&ref[size_t(key) * VALUE_SIZE], part[jj].value, VALUE_SIZE); puts++; }
            else {
                gets++;
                if (std::memcmp(resp + b.hr + jj * BLOCK + off, &ref[size_t(key) * VALUE_SIZE], VALUE_SIZE) != 0) {
                    if (bad++ < 5) std::printf("cmd %zu: GET key %u returned wrong data\n", i, key);
                    if (std::getenv("DOMA_DIAG") && bad <= 5) {
                        // what the slot holds: unwritten (A5), zeros, or other data; and where the expected value is
                        const uint8_t* d = resp + b.hr + jj * BLOCK;
                        size_t za = 0, a5 = 0, nz = 0;
                        for (size_t k = 0; k < BLOCK; k++) { if (d[k] == 0) za++; else if (d[k] == 0xA5) a5++; else nz++; }
                        std::printf("  DIAG cmd %zu: data slot %zu bytes: %zu zero, %zu A5 (never written), %zu other; first 16:", i, BLOCK, za, a5, nz);
                        for (int k = 0; k < 16; k++) std::printf(" %02x", d[k]);
                        std::printf("\n  DIAG expected"); for (int k = 0; k < (int)VALUE_SIZE; k++) std::printf(" %02x", ref[size_t(key) * VALUE_SIZE + k]);
                        std::printf("; found at:");
                        int nf = 0;
                        for (size_t q = 0; q + VALUE_SIZE <= part.size() * BLOCK && nf < 4; q++)
                            if (std::memcmp(resp + b.hr + q, &ref[size_t(key) * VALUE_SIZE], VALUE_SIZE) == 0) {
                                std::printf(" slot %zu byte %zu", q / BLOCK, q % BLOCK); nf++; }
                        std::printf(nf ? "\n" : " nowhere in this launch's data slots\n");
                        std::printf("  DIAG header:"); for (int k = 0; k < 32; k++) std::printf(" %02x", h[k]); std::printf("\n");
                    }
                }
            }
            if (TINY) { uint32_t d, w; std::memcpy(&d, h + 16, 4); std::memcpy(&w, h + 20, 4);
                        dmin = std::min(dmin, d); dmax = std::max(dmax, d); wmin = std::min(wmin, w); wmax = std::max(wmax, w); }
        }
        if (p.errors || !p.ok) break;
    }
    if (launches > 1)
        std::printf("batches: %zu launches of up to %zu commands (the requests do not fit one launch's buffers); "
                    "time = sum of launch + wait\n", launches, batch);
    std::printf("query: %d commands (%d GET, %d PUT) in %.6f s = %.1f KQPS\n", CMD_SIZE, gets, puts, t, CMD_SIZE / t / 1e3);
    std::printf("device: %llu cycles total, per-command latency %u..%u cycles, completed %u, errors 0x%x, summary %s\n",
                (unsigned long long)s.total, s.lat_min, s.lat_max, s.done, s.errors, s.ok ? "ok" : "MISSING");
    if (TINY && dmax)   // Tiny ORAM's two secondary latencies (response header bytes 16-19 and 20-23)
        std::printf("latency: to data ready %u..%u, to last write-back beat %u..%u, to write-back acknowledged %u..%u cycles\n",
                    dmin, dmax, wmin, wmax, s.lat_min, s.lat_max);
    if (s.traffic && s.done)
        std::printf("traffic: bucket port (pages, off-package) %llu B read, %llu B written = %.0f B read, %.0f B written per command\n",
                    (unsigned long long)(s.rd_beats * 32), (unsigned long long)(s.wr_beats * 32),
                    32.0 * s.rd_beats / s.done, 32.0 * s.wr_beats / s.done);
    if (halt_shown && last_where.size()) print_halt_record(("last halted launch: " + last_where).c_str(), last_halt.lat_min, last_halt.lat_max);
    const bool all = (bad == 0 && s.ok && s.errors == 0 && size_t(gets + puts) == cmds.size());
    std::printf("%s\n", all ? "ALL RESPONSES CORRECT" : "ERRORS FOUND");
    return all ? 0 : 1;
}
