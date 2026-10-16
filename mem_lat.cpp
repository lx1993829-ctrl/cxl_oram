// mem_lat.cpp -- host memory latency by pointer chasing (CPU side; no root needed).
//
// Each load's address comes from the previous load (a random cycle over 64-byte lines), so loads never overlap
// and the prefetchers cannot help. Sweeping the buffer size shows the whole hierarchy: L1, L2, the last-level
// cache, then DRAM. The plateau at the largest sizes is the DRAM latency seen from a core (t_DRAM's anchor for a
// PCIe model: the device's inbound reads end in the same memory controllers and DRAM, but reach them over a
// different path, so treat it as an anchor, not an exact term). The difference between the cache plateau and the
// DRAM plateau compares with the card's cold-vs-cache-resident difference.
//
// TLB misses would add page walks to every load, so the buffer is a 1 GB huge page if one is free (the card's
// host memory already uses this pool), else 2 MB transparent huge pages, else ordinary pages; the output says which.
//
//   g++ -O2 -std=c++17 mem_lat.cpp -o mem_lat
//   ./mem_lat                 sweep 16 KB .. 1 GB
//   ./mem_lat 1024            one size (MB)
//   MEMLAT_CPU=3 ./mem_lat    pin to another core (default 2)
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numeric>
#include <random>
#include <vector>

static const size_t GB = size_t(1) << 30, MB2 = size_t(2) << 20, LINE = 64;

static char* alloc(size_t bytes, const char** kind) {
#ifdef MAP_HUGE_1GB
    if (bytes <= GB) {                                         // one 1 GB page covers every size of the sweep
        void* p = mmap(nullptr, GB, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB | MAP_HUGE_1GB, -1, 0);
        if (p != MAP_FAILED) { *kind = "one 1 GB huge page"; return static_cast<char*>(p); }
    }
#endif
    size_t len = bytes + MB2;
    void* p = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) { std::perror("mmap"); std::exit(1); }
    char* a = reinterpret_cast<char*>((reinterpret_cast<uintptr_t>(p) + MB2 - 1) & ~(MB2 - 1));
    madvise(a, bytes, MADV_HUGEPAGE);
    *kind = "2 MB transparent huge pages (if the kernel granted them; else 4 KB pages: latency includes page walks)";
    return a;
}

static double now_ns() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1e9 + t.tv_nsec; }

// ns per dependent load over a random cycle of `bytes`; best (min) and mean of `reps` timed passes
static void measure(char* base, size_t bytes, double* best, double* mean) {
    const size_t n = bytes / LINE;
    std::vector<uint32_t> perm(n); std::iota(perm.begin(), perm.end(), 0u);
    std::mt19937_64 g(12345);
    for (size_t k = n - 1; k > 0; k--) { std::uniform_int_distribution<size_t> d(0, k - 1); std::swap(perm[k], perm[d(g)]); }
    for (size_t k = 0; k < n; k++) { char* nxt = base + size_t(perm[k]) * LINE; std::memcpy(base + k * LINE, &nxt, sizeof nxt); }
    const size_t steps = std::max<size_t>(n * 2, 20000000), reps = 3;
    char* p = base;
    for (size_t k = 0; k < std::min<size_t>(n, 2000000); k++) p = *reinterpret_cast<char**>(p);   // warm the TLB
    double s = 0, b = 1e30;
    for (size_t r = 0; r < reps; r++) {
        double t0 = now_ns();
        for (size_t k = 0; k < steps; k++) p = *reinterpret_cast<char**>(p);
        double ns = (now_ns() - t0) / steps;
        s += ns; b = std::min(b, ns);
    }
    if (p == nullptr) std::printf("?");                      // keep the chase from being optimized away
    *best = b; *mean = s / reps;
}

int main(int argc, char** argv) {
    int cpu = std::getenv("MEMLAT_CPU") ? std::atoi(std::getenv("MEMLAT_CPU")) : 2;
    cpu_set_t set; CPU_ZERO(&set); CPU_SET(cpu, &set);
    if (sched_setaffinity(0, sizeof set, &set) != 0) std::printf("(could not pin to CPU %d)\n", cpu);
    std::vector<size_t> sizes;
    if (argc > 1) sizes.push_back(size_t(std::atof(argv[1]) * (1 << 20)));
    else for (size_t kb : {16, 256, 2048, 8192, 32768, 131072, 524288, 1048576}) sizes.push_back(kb << 10);
    const size_t maxb = *std::max_element(sizes.begin(), sizes.end());
    const char* kind = "";
    char* base = alloc(maxb, &kind);
    std::memset(base, 0, maxb);                               // fault every page in before timing
    std::printf("buffer: %s; pinned to CPU %d; dependent loads over random 64-byte lines\n", kind, cpu);
    std::printf("%12s %14s %14s\n", "size", "best ns/load", "mean ns/load");
    for (size_t b : sizes) {
        double best, mean; measure(base, b, &best, &mean);
        if (b >= (1u << 20)) std::printf("%9zu MB %14.1f %14.1f\n", b >> 20, best, mean);
        else                 std::printf("%9zu KB %14.1f %14.1f\n", b >> 10, best, mean);
        std::fflush(stdout);
    }
    std::printf("the plateau at the largest sizes is DRAM; the step below it is the last-level cache\n");
    return 0;
}
