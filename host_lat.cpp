// host_lat.cpp -- runs hbm_lat_kernel and prints memory latency statistics.
//   ./host_lat lat_hw.xclbin [n]     n = samples per mode (default 1000000; writes and bursts use n/5)
//   LAT_CHAIN_HOST=1 ./host_lat lat_host.xclbin   the chain in host memory (lat_host.cfg / lat_pcie.cfg build)
//   LAT_CU=<instance>     run one kernel instance of a multi-instance build (lat_multi.cfg, lat_pcie.cfg: lat_1, lat_2)
//   LAT_SPAN_MB=<MB>      the chains cover this much of the buffer (default 16), so accesses land across a channel range
//                         (or, in host memory, far beyond the CPU's last-level cache: a cold, DRAM-served run)
//   LAT_OFFSET_MB=<MB>    the chains start this far into the buffer (default 0): one pseudo-channel at a time
//   LAT_MHZ=<MHz>         the kernel clock for the ns and GB/s conversions (default 300; 299 if v++ scaled the build)
// Default modes: dependent 32-byte reads, write acknowledgements, dependent 4 KB bursts (to the last beat).
// The kernel adds a constant 2 cycles to a dependent sample (issue, then registering the data); "memory" subtracts them.
//
// Calibration sweep (LAT_SWEEP=1), for fitting a PCIe model:
//   dependent reads of 32 B, 256 B, 1 KB, 4 KB and 32 KB (above 4 KB: back-to-back 4 KB bursts), to the last beat;
//   write acknowledgements of 32 B, 256 B and 4 KB bursts (one in flight, AW handshake to B);
//   independent reads and writes of 256 B and 4 KB with K = 1, 2, 4 .. 64 in flight: per-request latency and GB/s.
//   LAT_CU2=<instance>    the bandwidth runs use this second instance at the same time (lat_2): aggregate GB/s
//   LAT_CSV=<file>        every result also as a CSV row, for fitting
//   LAT_DEP_SIZES=512,1024,2048,4096   the dependent-read sizes (bytes, powers of two, 32 .. 32768)
//   LAT_SWEEP=dep         only the dependent reads (no write acks, no independent runs)
#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <random>
#include <string>
#include <vector>

static const uint32_t LINES = 1u << 16;          // 32-byte lines in the read/write chain (2 MB)
static const uint32_t PAGES = 1u << 12;          // 4 KB pages in the burst chain (16 MB)
static const size_t   BUF   = size_t(PAGES) * 4096;
static double         MHZ   = 300.0;             // kernel clock (check_timing.sh confirms 300 MHz)
static const int      KOVH  = 2;                 // cycles the kernel itself adds to each dependent sample
static FILE*          CSV   = nullptr;

struct Res { uint32_t magic, min, max, cnt; uint64_t sum, total; uint32_t hist[128]; };
struct Arg { uint32_t n, mode, beats, outst, span, seed; };

// one random cycle over all entries (Sattolo), so every access lands somewhere unpredictable
static void build_chain(uint8_t* p, uint32_t n, size_t stride, uint32_t seed) {
    std::vector<uint32_t> perm(n); std::iota(perm.begin(), perm.end(), 0u);
    std::mt19937 g(seed);
    for (uint32_t k = n - 1; k > 0; k--) { std::uniform_int_distribution<uint32_t> d(0, k - 1); std::swap(perm[k], perm[d(g)]); }
    std::memset(p, 0, size_t(n) * stride);
    for (uint32_t k = 0; k < n; k++) std::memcpy(p + size_t(k) * stride, &perm[k], 4);
}

static Res collect(xrt::bo& res) {
    res.sync(XCL_BO_SYNC_BO_FROM_DEVICE, 17 * 32, 0);
    const uint8_t* q = res.map<uint8_t*>(); Res o;
    std::memcpy(&o.magic, q, 4); std::memcpy(&o.min, q + 4, 4); std::memcpy(&o.max, q + 8, 4); std::memcpy(&o.cnt, q + 12, 4);
    std::memcpy(&o.sum, q + 16, 8); std::memcpy(&o.total, q + 24, 8); std::memcpy(o.hist, q + 32, 512);
    return o;
}

static Res run(xrt::kernel& k, xrt::bo& chain, xrt::bo& res, const Arg& a, uint32_t shift) {
    auto r = k(chain, res, a.n, a.mode | (shift << 8), a.beats, a.outst, a.span, a.seed); r.wait();
    return collect(res);
}

// percentile from the histogram: upper edge of the bin holding it (bin 127 = at least its lower edge)
static uint32_t pctv(const Res& o, uint32_t shift, double p, bool* over = nullptr) {
    uint64_t want = uint64_t(p * o.cnt + 0.999999), c = 0;
    for (int b = 0; b < 128; b++) {
        c += o.hist[b];
        if (c >= want) { if (over) *over = (b == 127); return (b == 127) ? (127u << shift) : (((uint32_t(b) + 1) << shift) - 1); }
    }
    return 0;
}
static std::string pct(const Res& o, uint32_t shift, double p) {
    bool over = false; uint32_t v = pctv(o, shift, p, &over);
    char s[64]; std::snprintf(s, sizeof s, "%s%u", over ? ">=" : "<=", v); return s;
}

static void report(const char* name, const Res& o, uint32_t shift, int ovh = KOVH) {
    double avg = o.cnt ? double(o.sum) / o.cnt : 0;
    std::printf("%-28s %8u samples  min %5u  avg %8.1f  max %6u cycles   (memory: min %.0f ns, avg %.0f ns, max %.0f ns)\n",
                name, o.cnt, o.min, avg, o.max, (o.min - ovh) * 1e3 / MHZ, (avg - ovh) * 1e3 / MHZ, (o.max - ovh) * 1e3 / MHZ);
    std::printf("%-28s percentiles (cycles, bins of %u): p50 %s  p99 %s  p99.9 %s  p99.99 %s\n", "",
                1u << shift, pct(o, shift, 0.50).c_str(), pct(o, shift, 0.99).c_str(), pct(o, shift, 0.999).c_str(), pct(o, shift, 0.9999).c_str());
    if (o.magic != 0x1A7E0001u) std::printf("  WARNING: bad result header 0x%08x\n", o.magic);
}

static void csv(const char* kind, int cus, uint32_t bytes, uint32_t outst, const Res& o, uint32_t shift, double gbps) {
    if (!CSV) return;
    double avg = o.cnt ? double(o.sum) / o.cnt : 0;
    std::fprintf(CSV, "%s,%d,%u,%u,%u,%u,%.1f,%u,%u,%u,%u,%llu,%.3f,%.0f\n", kind, cus, bytes, outst, o.cnt, o.min, avg,
                 pctv(o, shift, 0.5), pctv(o, shift, 0.99), pctv(o, shift, 0.999), o.max, (unsigned long long)o.total, gbps, MHZ);
    std::fflush(CSV);
}

// run, widening the histogram bins until no sample overflows its last bin
static Res measure_r(xrt::kernel& k, xrt::bo& chain, xrt::bo& res, const Arg& a, uint32_t& shift) {
    Res o;
    for (;;) { o = run(k, chain, res, a, shift); if (o.hist[127] == 0 || shift >= 12) break; shift++; }
    return o;
}
static void measure(xrt::kernel& k, xrt::bo& chain, xrt::bo& res, uint32_t n, uint32_t mode, uint32_t shift, const char* name) {
    Arg a{n, mode, 0, 0, 0, 0};
    Res o = measure_r(k, chain, res, a, shift);
    report(name, o, shift);
}
static uint32_t pow2_floor(uint64_t x) { uint32_t p = 1; while (uint64_t(p) * 2 <= x && p < (1u << 31)) p <<= 1; return p; }
static double gbps(uint64_t bytes, uint64_t cycles) { return cycles ? bytes / (cycles / (MHZ * 1e6)) / 1e9 : 0; }

int main(int argc, char** argv) {
    if (argc < 2) { std::printf("usage: %s <xclbin> [samples]\n", argv[0]); return 1; }
    uint32_t n = (argc > 2) ? uint32_t(std::atoi(argv[2])) : 1000000u;
    bool host = std::getenv("LAT_CHAIN_HOST") != nullptr;
    if (std::getenv("LAT_MHZ")) MHZ = std::atof(std::getenv("LAT_MHZ"));
    xrt::device device(0);
    auto uuid = device.load_xclbin(argv[1]);
    const char* cu = std::getenv("LAT_CU");
    const std::string kname = cu ? std::string("hbm_lat_kernel:{") + cu + "}" : std::string("hbm_lat_kernel");
    xrt::kernel krnl(device, uuid, kname);
    const size_t span = std::getenv("LAT_SPAN_MB")   ? size_t(std::atoll(std::getenv("LAT_SPAN_MB")))   << 20 : BUF;
    const size_t off  = std::getenv("LAT_OFFSET_MB") ? size_t(std::atoll(std::getenv("LAT_OFFSET_MB"))) << 20 : 0;
    if (span < 65536 || span / 32 > 0xFFFFFFFFull) { std::printf("LAT_SPAN_MB out of range\n"); return 1; }
    auto alloc = [&](xrt::kernel& k, size_t bytes) {
        return host ? xrt::bo(device, bytes, xrt::bo::flags::host_only, k.group_id(0)) : xrt::bo(device, bytes, k.group_id(0)); };
    xrt::bo whole = alloc(krnl, off + span);
    xrt::bo chain = off ? xrt::bo(whole, span, off) : whole;     // the chains live in [off, off + span)
    const bool spanned = std::getenv("LAT_SPAN_MB") != nullptr;          // unset: exactly the original chains
    const uint32_t lines = spanned ? uint32_t(span / 32) : LINES, pages = spanned ? uint32_t(span / 4096) : PAGES;
    xrt::bo res(device, 4096, krnl.group_id(1));
    std::printf("chain in %s; %u samples per read run; kernel clock assumed %.0f MHz; the kernel adds %d cycles per sample\n",
                host ? "host memory (PCIe)" : "HBM", n, MHZ, KOVH);
    std::printf("kernel %s; chains over %zu MB starting %zu MB into the buffer (%u lines, %u pages)\n",
                kname.c_str(), span >> 20, off >> 20, lines, pages);
    {   const uint64_t ba = chain.address();                         // the bus address the kernel uses
        std::printf("buffer bus address 0x%llx .. 0x%llx (%s 4 GB: %s TLP headers)\n", (unsigned long long)ba,
                    (unsigned long long)(ba + span - 1), ba >= (1ull << 32) ? "above" : "below",
                    ba >= (1ull << 32) ? "4DW (16-byte)" : "3DW (12-byte) possible"); }
    uint8_t* p = chain.map<uint8_t*>();

    if (!std::getenv("LAT_SWEEP")) {                                      // the original three measurements
        build_chain(p, lines, 32, 7);
        chain.sync(XCL_BO_SYNC_BO_TO_DEVICE, size_t(lines) * 32, 0);
        measure(krnl, chain, res, n, 0, 2, "dependent 32-byte reads");
        measure(krnl, chain, res, n / 5, 1, 2, "write acknowledgements");
        build_chain(p, pages, 4096, 9);
        chain.sync(XCL_BO_SYNC_BO_TO_DEVICE, size_t(pages) * 4096, 0);
        measure(krnl, chain, res, n / 5, 2, 3, "dependent 4 KB bursts");
        return 0;
    }

    // ---------------- calibration sweep ----------------
    if (std::getenv("LAT_CSV")) {
        CSV = std::fopen(std::getenv("LAT_CSV"), "w");
        if (CSV) std::fprintf(CSV, "kind,instances,bytes,outstanding,samples,min_cyc,avg_cyc,p50_cyc,p99_cyc,p999_cyc,max_cyc,total_cyc,GBps,MHz\n");
    }
    const uint32_t nl = std::max<uint32_t>(1000, n / 5);                  // samples for the slower runs
    // 1. dependent reads of each size (latency against size; the chain spans the whole buffer)
    std::vector<uint32_t> depb{1u, 8u, 32u, 128u, 1024u};
    if (const char* ds = std::getenv("LAT_DEP_SIZES")) {
        depb.clear(); std::string t(ds); size_t a0 = 0;
        while (a0 <= t.size()) { size_t c = t.find(',', a0); if (c == std::string::npos) c = t.size();
            uint32_t S = uint32_t(std::atoi(t.substr(a0, c - a0).c_str()));
            if (S >= 32 && S <= 32768 && (S & (S - 1)) == 0) depb.push_back(S / 32);
            else std::printf("LAT_DEP_SIZES: %u ignored (a power of two, 32 .. 32768)\n", S);
            a0 = c + 1; }
    }
    for (uint32_t b : depb) {
        const uint32_t S = b * 32, cnt = uint32_t(std::min<uint64_t>(span / S, 0xFFFFFFFFull));
        build_chain(p, cnt, S, 7 + b);
        chain.sync(XCL_BO_SYNC_BO_TO_DEVICE, size_t(cnt) * S, 0);
        uint32_t shift = (b <= 8) ? 2 : (b <= 128) ? 3 : 5;
        Arg a{b <= 32 ? n : (b <= 128 ? nl : std::max<uint32_t>(200, nl / 8)), 0, b, 0, 0, 0};
        Res o = measure_r(krnl, chain, res, a, shift);
        char name[64]; std::snprintf(name, sizeof name, "dependent read %u B", S);
        report(name, o, shift);
        csv("dep_read", 1, S, 1, o, shift, gbps(uint64_t(o.cnt) * S, o.total));
    }
    if (std::string(std::getenv("LAT_SWEEP")) == "dep") { if (CSV) std::fclose(CSV); return 0; }
    // 2. independent reads and writes: K in flight, random blocks of the buffer
    xrt::kernel* k2 = nullptr; xrt::bo chain2, res2;
    const char* cu2 = std::getenv("LAT_CU2");
    if (cu2) {
        k2 = new xrt::kernel(device, uuid, std::string("hbm_lat_kernel:{") + cu2 + "}");
        chain2 = alloc(*k2, span); res2 = xrt::bo(device, 4096, k2->group_id(1));
        std::printf("bandwidth runs also on kernel instance %s (its own %zu MB buffer): aggregate GB/s\n", cu2, span >> 20);
    }
    for (uint32_t mode : {4u, 3u}) {                                      // writes first: a K = 1 sweep gives the acks
        const char* kind = (mode == 3) ? "ind_read" : "ind_write";
        if (mode == 4) for (uint32_t b : {1u, 8u, 128u}) {               // write acknowledgements of bursts, unloaded
            uint32_t shift = 2; Arg a{nl, 4, b, 1, pow2_floor(span / (b * 32)), 11};
            Res o = measure_r(krnl, chain, res, a, shift);
            char name[64]; std::snprintf(name, sizeof name, "write ack %u B", b * 32);
            report(name, o, shift, 0);
            csv("write_ack", 1, b * 32, 1, o, shift, gbps(uint64_t(o.cnt) * b * 32, o.total));
        }
        for (uint32_t b : {8u, 128u}) for (uint32_t K : {1u, 2u, 4u, 8u, 16u, 32u, 64u}) {
            const uint32_t S = b * 32, nr = (b == 8) ? n / 5 : n / 40;
            Arg a{std::max<uint32_t>(nr, 2000), mode, b, K, pow2_floor(span / S), 1000 + K};
            uint32_t shift = (b == 8) ? 3 : 5;
            Res o, o2; double bw;
            if (k2) {                                                       // both instances at once
                for (;;) {
                    auto r1 = krnl(chain, res, a.n, a.mode | (shift << 8), a.beats, a.outst, a.span, a.seed);
                    auto r2 = (*k2)(chain2, res2, a.n, a.mode | (shift << 8), a.beats, a.outst, a.span, a.seed + 7);
                    r1.wait(); r2.wait();
                    o = collect(res); o2 = collect(res2);
                    if ((o.hist[127] == 0 && o2.hist[127] == 0) || shift >= 12) break;
                    shift++;
                }
                bw = gbps(uint64_t(o.cnt) * S, o.total) + gbps(uint64_t(o2.cnt) * S, o2.total);
            } else { o = measure_r(krnl, chain, res, a, shift); bw = gbps(uint64_t(o.cnt) * S, o.total); }
            char name[80]; std::snprintf(name, sizeof name, "%s %u B, K=%u%s", mode == 3 ? "indep read" : "indep write", S, K, k2 ? " x2" : "");
            report(name, o, shift, 0);
            std::printf("%-28s %.2f GB/s%s\n", "", bw, k2 ? " (both instances)" : "");
            csv(kind, k2 ? 2 : 1, S, K, o, shift, bw);
        }
    }
    if (CSV) std::fclose(CSV);
    return 0;
}
