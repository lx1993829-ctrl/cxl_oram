// host_phases.cpp -- host_doma.cpp's boot, preload, staging, launch and checks, with the query replaced by phases.
//   DOMA_PHASES="put:250000,get:250000" (default): each phase issues that many commands of one type, keys uniform
//   over the dataset (std::mt19937 seed 1, continued across phases), PUT values random. A phase is split into
//   launches of at most <commands per launch> (default 65536: the command and response buffers' capacity at 4 KB);
//   each launch is timed like host_doma's query (launch + wait, commands already on the card), and the phase time is
//   their sum. Every GET is checked against the latest value written (preload or an earlier PUT).
//   Refuses to run if LAMBDA / ZPOS below differ from ../rtl/oram_params.vh, when that file is present.
//   DOMA_PHASE_TRACE=1 (a TRACE=1 bitstream): every command also writes a 32-byte record of 16 phase timestamps after
//   the launch's response data; each phase prints the mean of 12 phases that add up to E2E (see host_ycsb.cpp).
#include <sstream>
#include <fstream>
#include <regex>
// host_doma.cpp -- XRT host program for doma_kernel, built to match BOLT's benchmark host
// (BOLT-benchmark/BOLT/*/host.cpp) so the two are measured the same way:
//   * same workload generator: std::mt19937 seed 1, 50% PUT, keys uniform over the dataset,
//     VALUE_SIZE random bytes per PUT (DOMA stores them at the start of a 4 KB block)
//   * keys are dense: key index i is DOMA slot i (BOLT looks keys up through its hash map)
//   * commands are converted and copied to the card BEFORE timing starts, as BOLT does
//   * query time = one kernel launch processing all CMD_SIZE commands, plus wait()
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

// must match the hardware build (rtl/oram_params.vh, single-client default)
// run-time size (DOMA_N=<records>): n blocks in NB = ceil(n / LAMBDA) buckets, passed to the kernel as nblk
// (the build's maximum is rtl/oram_params.vh ORAM_N; build.sh NMAX=...). Default: 256 blocks, as before.
static const uint32_t LAMBDA     = 6;                // rtl/oram_params.vh ORAM_C
static const uint32_t ZPOS       = 8;                // rtl/oram_params.vh ORAM_Z

// Stash pool size: Claim 1 of the stash analysis (docs/stash_chain_design.md). The stash exceeds xR + xO with
// probability at most gamma = 2^-64 per access, for every access sequence (Chernoff bounds over negatively
// associated per-bucket terms; the best split of gamma between the two terms).
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
static double stash_xO(double Bk, double lam, int Z, double lg) {
    double best = INFINITY; std::vector<double> lt(160);
    for (int i = 1; i < 400; i++) { const double u = i * 0.01; double mx = -INFINITY, sum = 0;
        for (int k = 0; k < 160; k++) { lt[k] = -lam + k * std::log(lam) - std::lgamma(k + 1.0) + u * std::max(0, k - Z); mx = std::max(mx, lt[k]); }
        for (int k = 0; k < 160; k++) sum += std::exp(lt[k] - mx);
        best = std::min(best, (Bk * (mx + std::log(sum)) + lg) / u); }
    return best;
}
static double stash_bound(double n, double lam, int Z) {
    const double Bk = std::ceil(n / lam), G = 64 * std::log(2.0);
    double best = INFINITY;
    for (double f : {0.1, 0.3, 0.5, 0.7, 0.9})
        best = std::min(best, stash_xR(Bk, n, G - std::log(f)) + stash_xO(Bk, lam, int(Z), G - std::log(1 - f)));
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
    b.hr = roundup4k((n + 2) * 32);                   // after records [n] (summary) and [n+1] (traffic)
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
}

struct Summary { uint64_t total, rd_beats, wr_beats; uint32_t beats, lat_min, lat_max, errors, done; bool ok, traffic;
                 uint32_t geo_n; size_t geo_tree; };   // tinyoram_kernel only: traffic-record bytes 24-27 blocks, 28-31 tree (64-byte bursts)

static Summary read_summary(Buffers& b, size_t n) {
    // at least through the summary record itself (b.hr is 0 before the first stage())
    b.resp.sync(XCL_BO_SYNC_BO_FROM_DEVICE, std::max(b.hr + n * BLOCK, (n + 2) * size_t(32)), 0);
    auto r = b.resp.map<uint8_t*>() + n * 32;
    Summary s{};
    s.ok = (r[0] == T_SUM);
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
    if (argc < 2) { std::printf("usage: %s <xclbin> [commands per launch, default 65536]   (phases: DOMA_PHASES=put:N,get:N,...)\n", argv[0]); return 1; }
    int CMD_SIZE = 65536;                                                  // commands per launch (buffer capacity)
    {   // the constants above must match the RTL this program is used with
        std::ifstream pf("../rtl/oram_params.vh"); std::string line; int z = -1, c = -1; std::smatch m;
        while (std::getline(pf, line)) {
            if (std::regex_search(line, m, std::regex("`define ORAM_Z\\s+(\\d+)"))) z = std::stoi(m[1]);
            if (std::regex_search(line, m, std::regex("`define ORAM_C\\s+(\\d+)"))) c = std::stoi(m[1]);
        }
        if (z >= 0 && c >= 0 && (uint32_t(z) != ZPOS || uint32_t(c) != LAMBDA)) {
            std::printf("host_phases is built for Z = %u, lambda = %u, but ../rtl/oram_params.vh has Z = %d, lambda = %d: refusing to run\n", ZPOS, LAMBDA, z, c);
            return 1;
        }
    }
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
    NB = (NBLK + LAMBDA - 1) / LAMBDA;
    const size_t stride = size_t(ZPOS) * BLOCKA;                              // compact layout: exactly Z blocks per bucket
    // stash pool: Claim 1's bound at 2^-64 per access, plus a margin for the window effect (PROVISIONAL: 1% of n
    // + 64, until its formal derivation); never more than n. DOMA_POOL=<entries> overrides it.
    const double pool_bnd = stash_bound(double(NBLK), double(LAMBDA), int(ZPOS));
    uint32_t POOL = uint32_t(std::min<double>(NBLK, std::ceil(pool_bnd) + std::ceil(0.01 * NBLK) + 64));
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
    if (!TINY) std::printf("stash pool: %u entries (Claim 1 bound %.0f at 2^-64 per access, + window margin)%s\n", POOL, pool_bnd,
                           std::getenv("DOMA_POOL") ? " [DOMA_POOL]" : "");
    if (TINY) {} else std::printf("size: %u blocks (nblk), %u buckets (%u in HBM = %u%%, %u in pages), buffers sized for %zu-byte blocks\n",
                     NBLK, NB, HBMK, hbm_pct, NB - HBMK, BLOCKA);
    const size_t max_cmds = std::max<size_t>(CMD_SIZE, std::min<size_t>(NBLK, 5000));
    Buffers b;
    b.cmd   = xrt::bo(device, roundup4k(max_cmds * 32) + max_cmds * BLOCKA, krnl.group_id(0));
    const bool PTRACE = std::getenv("DOMA_PHASE_TRACE") && std::string(std::getenv("DOMA_PHASE_TRACE")) != "0";
    b.resp  = xrt::bo(device, roundup4k((max_cmds + 2) * 32) + max_cmds * BLOCKA + (PTRACE ? max_cmds * 32 : 0), krnl.group_id(1));
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
                    : krnl(b.cmd, b.resp, b.pages, n | ((PTRACE && n) ? 0x80000000u : 0u), b.meta, b.brec, b.stash, NBLK, b.pages_hbm, HBMK, POOL);   // never flag an init launch (size 0)
    };

    // ---- init: boot run, then preload every key (BOLT's init includes loading its dataset)
    auto init_start = std::chrono::system_clock::now();
    auto r0 = launch(0u);                                                  // DOMA: key load, layout, lease; Tiny ORAM: latch
    r0.wait();
    {   // the kernel reports its block size; the data layout must match the bitstream
        Summary s0 = read_summary(b, 0);
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
    uint32_t pre_err = 0;
    for (size_t at = 0; at < preload.size(); at += max_cmds) {             // in batches that fit the buffers
        std::vector<Cmd> part(preload.begin() + at, preload.begin() + std::min(preload.size(), at + max_cmds));
        stage(b, part);
        auto r1 = launch((uint32_t)part.size());
        r1.wait();
        pre_err |= read_summary(b, part.size()).errors;
    }
    auto init_end = std::chrono::system_clock::now();
    std::printf("init: %.6f s (boot + preload of %u keys), preload errors=0x%x\n",
                std::chrono::duration<double>(init_end - init_start).count(), NKEYS, pre_err);

    // ---- phases (see the top of this file)
    std::string spec = std::getenv("DOMA_PHASES") ? std::getenv("DOMA_PHASES") : "put:250000,get:250000";
    struct Phase { uint8_t type; size_t n; std::string name; };
    std::vector<Phase> phases;
    {
        std::stringstream ss(spec); std::string item;
        while (std::getline(ss, item, ',')) {
            auto c = item.find(':');
            std::string k = c == std::string::npos ? "" : item.substr(0, c);
            size_t n = c == std::string::npos ? 0 : std::strtoul(item.c_str() + c + 1, nullptr, 10);
            uint8_t t = (k == "put" || k == "PUT") ? T_PUT : (k == "get" || k == "GET") ? T_GET : 0;
            if (!t || n == 0) { std::printf("bad DOMA_PHASES item '%s' (want put:N or get:N, comma-separated)\n", item.c_str()); return 1; }
            phases.push_back({t, n, t == T_PUT ? "PUT" : "GET"});
        }
    }
    std::mt19937 pgen(1);
    std::uniform_int_distribution<uint32_t> pkey(0, NKEYS - 1);
    std::uniform_int_distribution<int> pval(0, 255);
    const size_t chunk = size_t(CMD_SIZE);
    bool all_ok = true;
    for (size_t pi = 0; pi < phases.size(); pi++) {
        const Phase& ph = phases[pi];
        double t = 0; uint64_t cyc = 0, rdb = 0, wrb = 0; uint32_t errs = 0; bool sok = true, traffic = true;
        int bad = 0; size_t launches = 0; std::vector<uint32_t> lv; lv.reserve(ph.n);
        uint64_t pn = 0, plook = 0, pread = 0, pproc = 0, pwrite = 0, ptail = 0; size_t pzero = 0, pbad = 0;
        uint64_t tn = 0, te2e = 0, tph[12] = {0}, tovl = 0; size_t tbad = 0;     // phase-trace records
        for (size_t at = 0; at < ph.n; at += chunk) {
            const size_t n = std::min(chunk, ph.n - at);
            std::vector<Cmd> cmds(n);
            for (auto& c : cmds) {
                c = Cmd{}; c.type = ph.type; c.key = pkey(pgen);
                if (c.type == T_PUT) for (int j = 0; j < VALUE_SIZE; j++) c.value[j] = uint8_t(pval(pgen));
            }
            stage(b, cmds);
            auto t0 = std::chrono::system_clock::now();
            auto run = launch((uint32_t)n);
            run.wait();
            auto t1 = std::chrono::system_clock::now();
            t += std::chrono::duration<double>(t1 - t0).count(); launches++;
            Summary s = read_summary(b, n);
            if (PTRACE) b.resp.sync(XCL_BO_SYNC_BO_FROM_DEVICE, n * 32, b.hr + n * BLOCK);   // the launch's trace records
            cyc += s.total; errs |= s.errors; sok = sok && s.ok && s.done == n; traffic = traffic && s.traffic;
            rdb += s.rd_beats; wrb += s.wr_beats;
            auto resp = b.resp.map<uint8_t*>();
            for (size_t i = 0; i < n; i++) {
                const uint8_t* h = resp + i * 32;
                uint32_t slot; std::memcpy(&slot, h + 4, 4);
                const uint32_t key = cmds[i].key, off = (key % PACK) * VALUE_SIZE;
                if (h[0] != T_ACK || slot != key / PACK) {
                    if (bad++ < 5) std::printf("%s phase, command %zu: bad header 0x%02x block %u\n", ph.name.c_str(), at + i, h[0], slot);
                    continue;
                }
                uint32_t l; std::memcpy(&l, h + 8, 4); lv.push_back(l);
                {   // phase timestamps, header bytes 16-31 (cycles since the request; 0: a bitstream without them)
                    uint32_t tar, trl, twf, twl;
                    std::memcpy(&tar, h + 16, 4); std::memcpy(&trl, h + 20, 4); std::memcpy(&twf, h + 24, 4); std::memcpy(&twl, h + 28, 4);
                    if ((tar | trl | twf | twl) == 0) pzero++;
                    else if (tar == 0 || tar > trl || trl > twf || twf > twl || twl > l) {
                        if (pbad++ < 5) std::printf("%s phase, command %zu: phase timestamps out of order (%u %u %u %u, done %u)\n",
                                                    ph.name.c_str(), at + i, tar, trl, twf, twl, l);
                    } else { pn++; plook += tar; pread += trl - tar; pproc += twf - trl; pwrite += twl - twf; ptail += l - twl; }
                }
                if (PTRACE) {   // 12 phases, as in host_ycsb.cpp (slot 9, the write-back gate state, is unused in arrival mode)
                    uint16_t v[16]; std::memcpy(v, resp + b.hr + n * BLOCK + i * 32, 32);
                    const uint32_t ch[12] = {v[0], v[1], v[2], v[4], v[6], v[7], v[8], v[10], v[11], v[12], v[14], l};
                    bool tok = v[3] != 0 && v[5] != 0;
                    for (int k = 0; k < 11 && tok; k++) tok = ch[k] != 0 && ch[k] <= ch[k + 1];
                    if (!tok) { if (tbad++ < 3) { std::printf("%s phase, command %zu: trace record out of order:", ph.name.c_str(), at + i);
                                                  for (int k = 0; k < 16; k++) std::printf(" %u", v[k]); std::printf(" | done %u\n", l); } }
                    else { tn++; te2e += l; tph[0] += ch[0]; for (int k = 1; k < 12; k++) tph[k] += ch[k] - ch[k - 1];
                           tovl += (v[5] < v[4]) ? v[4] - v[5] : 0; }
                }
                if (cmds[i].type == T_PUT) std::memcpy(&ref[size_t(key) * VALUE_SIZE], cmds[i].value, VALUE_SIZE);
                else if (std::memcmp(resp + b.hr + i * BLOCK + off, &ref[size_t(key) * VALUE_SIZE], VALUE_SIZE) != 0)
                    if (bad++ < 5) std::printf("%s phase, command %zu: key %u returned wrong data\n", ph.name.c_str(), at + i, key);
            }
        }
        const bool ok = bad == 0 && sok && errs == 0 && pbad == 0 && tbad == 0;
        std::printf("phase %zu: %zu %s in %zu launch%s, %.6f s = %.2f KQPS, %.2f us per command; device %.1f cycles per command, errors 0x%x\n",
                    pi + 1, ph.n, ph.name.c_str(), launches, launches == 1 ? "" : "es", t, ph.n / t / 1e3, 1e6 * t / ph.n,
                    double(cyc) / ph.n, errs);
        if (!lv.empty()) {
            std::sort(lv.begin(), lv.end());
            auto pct = [&](double q) { return lv[std::min(lv.size() - 1, size_t(q * (lv.size() - 1) + 0.5))]; };
            double mean = 0; for (uint32_t x : lv) mean += x; mean /= lv.size();
            std::printf("phase %zu: latency min %u, p50 %u, p90 %u, p99 %u, p99.9 %u, max %u, mean %.1f cycles (%zu commands)\n",
                        pi + 1, lv.front(), pct(0.5), pct(0.9), pct(0.99), pct(0.999), lv.back(), mean, lv.size());
        }
        if (pn) {
            auto m = [&](uint64_t x) { return double(x) / pn; };
            std::printf("phase %zu: phases, mean cycles: lookup %.1f, bucket read %.1f, processing %.1f, bucket write %.1f, after the write %.1f (%zu commands%s)\n",
                        pi + 1, m(plook), m(pread), m(pproc), m(pwrite), m(ptail), pn, pbad ? "" : ", all in order");
        } else if (pzero) std::printf("phase %zu: phases not reported by this bitstream\n", pi + 1);
        if (pbad) std::printf("phase %zu: %zu commands with phase timestamps out of order\n", pi + 1, pbad);
        if (tn) {
            static const char* PH[12] = {"handoff", "lookup", "read gate", "bucket read", "decrypt/verify", "operation", "eviction",
                                         "re-encrypt", "write-back wait", "bucket write", "release gate", "result"};
            std::printf("phase %zu: trace, mean cycles (sum = E2E):", pi + 1);
            for (int k = 0; k < 12; k++) std::printf("%s %s %.1f", k ? "," : "", PH[k], double(tph[k]) / tn);
            std::printf(" = E2E %.1f; decryption overlaps the bucket read by %.1f (%zu records)\n", double(te2e) / tn, double(tovl) / tn, tn);
        } else if (PTRACE) std::printf("phase %zu: no trace records (is this a TRACE=1 bitstream?)\n", pi + 1);
        if (tbad) std::printf("phase %zu: %zu trace records out of order\n", pi + 1, tbad);
        if (traffic)
            std::printf("phase %zu: bucket port (pages, off-package) %.0f B read, %.0f B written per command\n",
                        pi + 1, 32.0 * rdb / ph.n, 32.0 * wrb / ph.n);
        std::printf("phase %zu: %s\n", pi + 1, ok ? "ALL RESPONSES CORRECT" : "ERRORS FOUND");
        all_ok = all_ok && ok;
    }
    std::printf("%s\n", all_ok ? "ALL PHASES CORRECT" : "ERRORS FOUND");
    return all_ok ? 0 : 1;
}
