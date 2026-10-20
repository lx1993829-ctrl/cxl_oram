// host_ycsb.cpp -- YCSB workloads A-F on doma_kernel (U55C), with oram_ycsb.c's generator, op for op.
//
// oram_ycsb.c is the gem5 workload: it reaches the ORAM through gem5's MMIO registers and command ring, which do not
// exist on the card. This program generates the SAME op sequence (same arguments, xorshift32 seed, scrambled and
// raw Zipfian, workload logic, even/odd slot layout, write payload words, read checks) and sends it to doma_kernel
// through XRT command buffers, the way host_doma does:
//   init launch (size 0): DOMA boots (key load, bucket format and layout, lease), as at every gem5 run's start
//   LOAD: init_occ PUTs to the even slots 0, 2, 4, ...   (oram_ycsb.c opIdx 0 .. init_occ-1)
//   RUN:  the workload's ops                             (opIdx init_occ ..)
// One op is one doma_kernel command: GET (read) or PUT (write) of a whole block, key = slot index. doma_kernel has
// one client with one lease, so gem5's two-lease split by slot does not apply (it only routes, it changes no op).
// The sequence never depends on read data, so each phase is staged on the card first and runs as one launch (or
// launches of YCSB_CHUNK commands), timed as launch + wait, as in host_doma. In gem5 the ring keeps up to 16
// commands queued; here every command is already on the card, so DOMA is never waiting for the host either way.
//
// Usage: ./host_ycsb <xclbin> <N> <instance_id> [n_iters] [num_slots] [workload] [init_occ] [scan_len] [--dry-run]
//   Everything after <xclbin> is oram_ycsb.c's argv, with its meaning; defaults and limits are larger than
//   oram_ycsb.c's (which are 100 / 128 / max 131072 iterations / max 32768 slots, sized for gem5's static arrays):
//     N            instances (unused, as in oram_ycsb.c)      instance_id  0..31 (seeds the generator)
//     n_iters      logical ops in the run phase: default and maximum 1,000,000
//     num_slots    slots: default and maximum 1,048,576 (2^20, the 4 KB build's NMAX)
//     workload     1..6 or A..F; several letters (ABCDEF) or "all" run each in turn, each from a fresh boot (default A)
//     init_occ     slots written by the load phase, 1..num_slots/2 (default num_slots/2; oram_ycsb.c's default,
//                  num_slots, fails its own check)
//     scan_len     maximum scan length for E (10)
//   A run phase may issue at most 10,000,000 ORAM ops (E issues up to n_iters * scan_len), so E with the defaults fits.
//   With the same arguments the op sequence is oram_ycsb.c's (the limits only gate the arguments).
//   1M slots need about 5.4 GB of buckets and an 856 MB stash: a large-layout bitstream (doma_big_hbm.cfg, or
//   doma_big.cfg with host memory) and DOMA_HBM_PCT=75; doma_hw.cfg (one 512 MB bank each) holds up to ~100K slots.
//   --dry-run      generate only (no card): op counts, the oram_ycsb summary line and YCSB_TRACE
// Environment (all optional):
//   DOMA_N         ORAM size in blocks (default num_slots; at least num_slots, at most the bitstream's NMAX)
//   DOMA_HBM_PCT   percentage of buckets kept in HBM (as host_doma; needs a build with pages_hbm)
//   DOMA_POOL      stash pool entries (default: host_doma's Claim 1 sizing)
//   DOMA_PAGES_HBM 1 / 0: pages buffer in device memory / host memory. Default: where the xclbin connects
//                  the pages argument (HBM[8] with doma_hw.cfg, HOST[0] with doma.cfg)
//   DOMA_BLOCK     block size the buffers are sized for (default 4096; the kernel reports its own at boot)
//   DOMA_MHZ       kernel clock for the microsecond figures (default 300; check_timing.sh prints the real one)
//   DOMA_DEVICE    device index or BDF (default 0)
//   YCSB_CHUNK     commands per launch (default 65536; the command and response buffers hold one launch)
//   YCSB_PAD=1     also issue gem5's padding reads (E/F: slot-0 reads up to init_occ + n_iters*{scan_len,2}),
//                  so the command count matches the gem5 run's; they are timed and checked, but reported apart
//   YCSB_DATA=gem5 every 32-byte beat of a written block = payload_word(opIdx, 0..7), exactly the gem5 device's
//                  fill. Default: beat 0 words 0-1 are the same payload words, the rest of the block carries
//                  opIdx, slot and beat index, so a stale or misplaced block cannot pass (payload_word aliases:
//                  0xABCD0000 | i<<8 is the same word for i and i + 256)
//   YCSB_TRACE=f   write the op sequence: "opIdx slot op w0 w1" (op 1 = write; w0/w1 = payload_word(opIdx, 0/1))
//   YCSB_CSV=f     per-command results: workload,phase,cmd,opIdx,slot,op,latency_cycles,status
// Build: g++ -std=c++17 -O2 -ffp-contract=off host_ycsb.cpp -I$XILINX_XRT/include -L$XILINX_XRT/lib -lxrt_coreutil -pthread -o host_ycsb
#include <xrt/xrt_bo.h>
#include <xrt/xrt_device.h>
#include <xrt/xrt_kernel.h>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>
#include <vector>

// ------------------------------------------------------------------ DOMA build constants (as host_doma.cpp)
static const uint32_t LAMBDA = 6;                    // rtl/oram_params.vh ORAM_C
static const uint32_t ZPOS   = 8;                    // rtl/oram_params.vh ORAM_Z
static const size_t   MAX_BLOCK = 4096;
static const uint8_t  T_GET = 0x01, T_PUT = 0x02, T_ACK = 0x10, T_SUM = 0x5A;
static size_t roundup4k(size_t x) { return (x + 4095) & ~size_t(4095); }

// stash pool sizing: Claim 1 (docs/stash_chain_design.md), copied from host_doma.cpp
static double ln_mgf_A(double t, double Bk, double n) {
    const double p = 1.0 / Bk, lr = std::log1p(-p) + t;
    if (std::fabs(lr) < 1e-15) return std::log(p * n + 1.0);
    if (lr > 0) { const double lrn = n * lr; if (lrn > 700) return INFINITY;
                  return std::log(p * std::expm1(lrn) / std::expm1(lr) + std::exp(lrn)); }
    const double rn = std::exp(n * lr);
    return std::log(p * (1 - rn) / (-std::expm1(lr)) + rn);
}
static double stash_xR(double Bk, double n, double lg) {
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

// ------------------------------------------------------------------ oram_ycsb.c's generator, unchanged in logic
// oram_ycsb.c: DEFAULT_N_ITERS 100, DEFAULT_NUM_SLOTS 128, MAX_SLOTS 32768, MAX_ITERS 131072, MAX_RUN_ORAM_OPS 2*MAX_ITERS
#define DEFAULT_N_ITERS   1000000
#define DEFAULT_NUM_SLOTS 1048576
#define DEFAULT_SCAN_LEN  10
#define MAX_SLOTS         1048576
#define MAX_ITERS         1000000
#define MAX_RUN_ORAM_OPS  (10ull*MAX_ITERS)         // E at the defaults issues up to n_iters * scan_len
#define WL_A 1
#define WL_B 2
#define WL_C 3
#define WL_D 4
#define WL_E 5
#define WL_F 6

static uint32_t xorshift32(uint32_t *s){
    uint32_t x=*s; x^=x<<13; x^=x>>17; x^=x<<5; *s=x; return x;
}
static inline uint32_t payload_word(uint32_t i,int j){
    return 0xABCD0000u | (i<<8) | (uint32_t)j;
}
struct zipfian { uint32_t num_items; double theta, zeta_n, zeta_2, alpha, eta; };
static double compute_zeta(uint32_t n, double theta){
    double sum=0.0;
    for(uint32_t i=1;i<=n;i++) sum+=1.0/pow((double)i,theta);
    return sum;
}
static void zipfian_init(struct zipfian *z, uint32_t n){
    z->num_items=n;
    z->theta=0.99;
    z->zeta_n=compute_zeta(n,z->theta);
    z->zeta_2=compute_zeta(2,z->theta);
    z->alpha=1.0/(1.0-z->theta);
    z->eta=(1.0-pow(2.0/(double)n,1.0-z->theta))
          /(1.0-z->zeta_2/z->zeta_n);
}
static void zipfian_inc(struct zipfian *z){
    z->num_items++;
    z->zeta_n+=1.0/pow((double)z->num_items,z->theta);
    z->eta=(1.0-pow(2.0/(double)z->num_items,1.0-z->theta))
          /(1.0-z->zeta_2/z->zeta_n);
}
static uint32_t fnv1a_hash(uint32_t val){
    uint32_t h=2166136261u;
    h^=(val      )&0xFF; h*=16777619u;
    h^=(val >>  8)&0xFF; h*=16777619u;
    h^=(val >> 16)&0xFF; h*=16777619u;
    h^=(val >> 24)&0xFF; h*=16777619u;
    return h;
}
static uint32_t zipfian_next(struct zipfian *z, uint32_t *rng){
    double u=(double)(xorshift32(rng)&0x7FFFFFFFu)/(double)0x7FFFFFFFu;
    double uz=u*z->zeta_n;
    uint32_t rank;
    if(uz<1.0) rank=0;
    else if(uz<1.0+pow(0.5,z->theta)) rank=1;
    else rank=(uint32_t)((double)z->num_items*pow(z->eta*u-z->eta+1.0, z->alpha));
    if(rank>=z->num_items) rank=z->num_items-1;
    return fnv1a_hash(rank)%z->num_items;
}
static uint32_t zipfian_next_raw(struct zipfian *z, uint32_t *rng){
    double u=(double)(xorshift32(rng)&0x7FFFFFFFu)/(double)0x7FFFFFFFu;
    double uz=u*z->zeta_n;
    uint32_t rank;
    if(uz<1.0) rank=0;
    else if(uz<1.0+pow(0.5,z->theta)) rank=1;
    else rank=(uint32_t)((double)z->num_items*pow(z->eta*u-z->eta+1.0, z->alpha));
    if(rank>=z->num_items) rank=z->num_items-1;
    return rank;
}

static const uint32_t NONE = 0xFFFFFFFFu;
struct Op { uint32_t opIdx, slot; uint8_t write; uint32_t expect; };   // expect: a read's last_write_opidx
struct Plan {
    int workload = 0;
    std::vector<Op> load, run, pad;            // pad: gem5's E/F padding reads (YCSB_PAD=1 only)
    uint32_t total_oram_ops = 0, run_reads = 0, run_writes = 0, insert_count = 0;
};

// oram_ycsb.c main(), from the LOAD phase through PAD, with issue_op() recording the op instead of ringing gem5
static Plan generate(int instance_id, int n_iters, int num_slots, int workload, int init_occ, int scan_len, bool pad) {
    Plan P; P.workload = workload;
    uint32_t rng=(uint32_t)(instance_id+1)*2654435761u;
    std::vector<uint32_t> last_write_opidx(MAX_SLOTS, NONE);
    std::vector<uint32_t> insert_log(MAX_SLOTS + MAX_ITERS), occupied(MAX_SLOTS);

    uint32_t load_ops=(uint32_t)init_occ;
    uint32_t insert_count=0, occ_count=0, insert_counter=0;
    for(uint32_t i=0;i<load_ops;i++){
        uint32_t slotIdx=i*2;
        last_write_opidx[slotIdx]=i;
        insert_log[insert_count++]=slotIdx;
        occupied[occ_count++]=slotIdx;
        P.load.push_back({i, slotIdx, 1, NONE});
    }
    insert_counter=load_ops;

    struct zipfian z;
    zipfian_init(&z, insert_count);
    uint64_t opIdx=(uint64_t)load_ops;
    uint32_t run_reads=0, run_writes=0;
    auto rd = [&](uint32_t slotIdx){ P.run.push_back({(uint32_t)opIdx, slotIdx, 0, last_write_opidx[slotIdx]}); run_reads++; opIdx++; };
    auto wr = [&](uint32_t slotIdx){ last_write_opidx[slotIdx]=(uint32_t)opIdx; P.run.push_back({(uint32_t)opIdx, slotIdx, 1, NONE});
                                     run_writes++; opIdx++; };
    uint32_t inserts = 0;                          // odd slots taken (only inserts write odd slots)
    auto hashed_free_odd = [&](){
        uint32_t half=(uint32_t)num_slots/2, slotIdx;
        // oram_ycsb.c loops forever once every odd slot is taken; stop with an error instead
        if (inserts >= half) throw std::runtime_error("workload " + std::string(1, char('A' + workload - 1)) + ": all " +
            std::to_string(half) + " odd slots already hold inserts (oram_ycsb.c would hang here); use a larger num_slots or fewer n_iters");
        inserts++;
        do { slotIdx=fnv1a_hash(insert_counter)%half*2+1; insert_counter++; } while(last_write_opidx[slotIdx]!=NONE);
        return slotIdx;
    };
    for(int i=0;i<n_iters;i++){
        switch(workload){
        case WL_A: case WL_B: case WL_C: {
            uint32_t slotIdx=occupied[zipfian_next(&z,&rng)];
            int is_read;
            if(workload==WL_C)      is_read=1;
            else if(workload==WL_A) is_read=(xorshift32(&rng)%100<50);
            else                    is_read=(xorshift32(&rng)%100<95);
            if(is_read) rd(slotIdx); else wr(slotIdx);
            break;
        }
        case WL_D: {
            int is_read=(xorshift32(&rng)%100<95);
            if(!is_read){
                uint32_t slotIdx=hashed_free_odd();
                insert_log[insert_count++]=slotIdx;
                occupied[occ_count++]=slotIdx;
                zipfian_inc(&z);
                wr(slotIdx);
            } else {
                uint32_t rank=zipfian_next_raw(&z,&rng);
                rd(insert_log[insert_count-1-rank]);
            }
            break;
        }
        case WL_E: {
            int is_scan=(xorshift32(&rng)%100<95);
            if(!is_scan){
                uint32_t slotIdx=hashed_free_odd();
                insert_log[insert_count++]=slotIdx;      // no zipfian_inc and not in occupied[], as in oram_ycsb.c
                wr(slotIdx);
            } else {
                uint32_t start_rank=zipfian_next(&z,&rng);
                uint32_t len=1+xorshift32(&rng)%(uint32_t)scan_len;
                for(uint32_t s=0;s<len;s++) rd(occupied[(start_rank+s)%occ_count]);
            }
            break;
        }
        case WL_F: {
            uint32_t slotIdx=occupied[zipfian_next(&z,&rng)];
            int is_rmw=(xorshift32(&rng)%100<50);
            rd(slotIdx);
            if(is_rmw) wr(slotIdx);
            break;
        }
        }
    }
    P.total_oram_ops=(uint32_t)opIdx; P.run_reads=run_reads; P.run_writes=run_writes; P.insert_count=insert_count;
    if (pad) {   // oram_ycsb.c's PAD: slot-0 reads up to the device's expected total (they follow every run op)
        uint32_t expected_total;
        if(workload==WL_F)      expected_total=(uint32_t)init_occ+2u*(uint32_t)n_iters;
        else if(workload==WL_E) expected_total=(uint32_t)init_occ+(uint32_t)n_iters*(uint32_t)scan_len;
        else                    expected_total=(uint32_t)init_occ+(uint32_t)n_iters;
        for (uint32_t t = P.total_oram_ops; t < expected_total; t++, opIdx++)
            P.pad.push_back({(uint32_t)opIdx, 0u, 0, last_write_opidx[0]});
    }
    return P;
}

// ------------------------------------------------------------------ block contents
static bool DATA_GEM5 = false;
static void fill_block(uint8_t* p, size_t block, uint32_t opIdx, uint32_t slot) {
    for (size_t b = 0; b < block / 32; b++) {
        uint32_t w[8];
        if (DATA_GEM5) { for (int j = 0; j < 8; j++) w[j] = payload_word(opIdx, j); }   // the gem5 device's fill
        else {
            w[0] = payload_word(opIdx, 0); w[1] = payload_word(opIdx, 1);
            w[2] = opIdx; w[3] = slot; w[4] = uint32_t(b); w[5] = ~opIdx;
            w[6] = fnv1a_hash(opIdx ^ (slot * 2654435761u)); w[7] = fnv1a_hash(uint32_t(b) + opIdx * 131u);
        }
        std::memcpy(p + b * 32, w, 32);
    }
}

// ------------------------------------------------------------------ statistics
struct Lat {
    std::vector<uint32_t> v;
    void add(uint32_t x) { v.push_back(x); }
    std::string str(double mhz) {
        if (v.empty()) return "none";
        std::vector<uint32_t> s = v; std::sort(s.begin(), s.end());
        double mean = 0; for (uint32_t x : s) mean += x; mean /= s.size();
        auto q = [&](double f) { return s[std::min(s.size() - 1, size_t(f * (s.size() - 1) + 0.5))]; };
        char buf[256];
        std::snprintf(buf, sizeof buf, "n=%zu min %u mean %.1f p50 %u p99 %u max %u cycles = %.2f / %.2f / %.2f us (min/mean/max)",
                      s.size(), s.front(), mean, q(0.5), q(0.99), s.back(), s.front() / mhz, mean / mhz, s.back() / mhz);
        return buf;
    }
    double mean() const { double m = 0; for (uint32_t x : v) m += x; return v.empty() ? 0 : m / v.size(); }
    uint32_t max() const { return v.empty() ? 0 : *std::max_element(v.begin(), v.end()); }
};
struct PhaseResult {
    size_t cmds = 0, gets = 0, puts = 0, bad_hdr = 0;
    double wall = 0; uint64_t cycles = 0; uint32_t errors = 0; bool summary_ok = true; uint32_t done = 0;
    Lat get, put, all;
    uint32_t pass = 0, fail = 0;          // read checks
};

static const char* WL_NAMES[] = {"?", "A", "B", "C", "D", "E", "F"};

int main(int argc, char** argv) {
    std::vector<std::string> pos; bool dry = false;
    for (int a = 1; a < argc; a++) { std::string s = argv[a]; if (s == "--dry-run") dry = true; else pos.push_back(s); }
    if (pos.size() < 3) {
        std::printf("usage: %s <xclbin> <N> <instance_id> [n_iters] [num_slots] [workload 1..6|A..F|ABCDEF|all] [init_occ] [scan_len] [--dry-run]\n", argv[0]);
        return 1;
    }
    // oram_ycsb.c's argv, shifted by the xclbin
    auto P_ = [&](size_t i) -> const char* { return i < pos.size() ? pos[i].c_str() : nullptr; };
    int N           = atoi(P_(1)); (void)N;
    int instance_id = atoi(P_(2));
    int n_iters     = P_(3) ? atoi(P_(3)) : DEFAULT_N_ITERS;
    int num_slots   = P_(4) ? atoi(P_(4)) : DEFAULT_NUM_SLOTS;
    std::string wls = P_(5) ? P_(5) : "1";
    int init_occ    = P_(6) ? atoi(P_(6)) : num_slots / 2;
    int scan_len    = P_(7) ? atoi(P_(7)) : DEFAULT_SCAN_LEN;
    std::vector<int> wl;
    if (wls == "all") wls = "ABCDEF";
    if (wls.size() == 1 && wls[0] >= '1' && wls[0] <= '6') wl.push_back(wls[0] - '0');
    else for (char c : wls) { char u = char(std::toupper((unsigned char)c));
                              if (u < 'A' || u > 'F') { std::fprintf(stderr, "bad workload %s (1=A..6=F, letters, or all)\n", wls.c_str()); return 1; }
                              wl.push_back(u - 'A' + 1); }
    // oram_ycsb.c's sanity checks
    if(instance_id<0||instance_id>=32) {fprintf(stderr,"bad instance_id %d\n",instance_id);return 1;}
    if(n_iters<1||n_iters>MAX_ITERS) {fprintf(stderr,"n_iters %d out of range [1,%d]\n",n_iters,MAX_ITERS);return 1;}
    if(num_slots<1||num_slots>MAX_SLOTS) {fprintf(stderr,"bad num_slots %d\n",num_slots);return 1;}
    if(init_occ<1||init_occ>num_slots/2) {fprintf(stderr,"bad init_occ %d (must be 1..%d for even-slot layout)\n",init_occ,num_slots/2);return 1;}
    if(scan_len<1||scan_len>num_slots) {fprintf(stderr,"bad scan_len %d\n",scan_len);return 1;}
    const bool PAD = std::getenv("YCSB_PAD") && std::string(std::getenv("YCSB_PAD")) != "0";
    DATA_GEM5 = std::getenv("YCSB_DATA") && std::string(std::getenv("YCSB_DATA")) == "gem5";
    const double MHZ = std::getenv("DOMA_MHZ") ? std::atof(std::getenv("DOMA_MHZ")) : 300.0;

    std::vector<Plan> plans;
    for (int w : wl) {
        uint64_t max_run_ops = (w==WL_F) ? 2ull*(uint64_t)n_iters : (w==WL_E) ? (uint64_t)n_iters*(uint64_t)scan_len : (uint64_t)n_iters;
        if(max_run_ops>MAX_RUN_ORAM_OPS) {fprintf(stderr,"run-phase ORAM ops %llu exceed MAX_RUN_ORAM_OPS %llu\n",
                                                  (unsigned long long)max_run_ops,(unsigned long long)MAX_RUN_ORAM_OPS);return 1;}
        if (w == WL_D && init_occ >= num_slots)
            fprintf(stderr,"WARN: workload D with init_occ==num_slots — hashed inserts will always collide with existing slots (updates, not new records)\n");
        try { plans.push_back(generate(instance_id, n_iters, num_slots, w, init_occ, scan_len, PAD)); }
        catch (const std::exception& e) { fprintf(stderr, "%s\n", e.what()); return 1; }
    }
    FILE* trace = std::getenv("YCSB_TRACE") ? std::fopen(std::getenv("YCSB_TRACE"), "w") : nullptr;
    if (trace) for (auto& p : plans) for (auto* v : {&p.load, &p.run, &p.pad}) for (auto& o : *v)
        std::fprintf(trace, "%u %u %u %08x %08x\n", o.opIdx, o.slot, o.write, o.write ? payload_word(o.opIdx, 0) : 0u,
                     o.write ? payload_word(o.opIdx, 1) : 0u);
    if (trace) std::fclose(trace);
    if (dry) {
        for (auto& p : plans) {
            std::printf("workload %s: load %zu PUT, run %zu ops (%u GET, %u PUT)%s\n", WL_NAMES[p.workload], p.load.size(), p.run.size(),
                        p.run_reads, p.run_writes, PAD ? (", + " + std::to_string(p.pad.size()) + " padding reads").c_str() : "");
            std::printf("oram_ycsb: inst=%d wl=%s total_oram_ops=%u run_reads=%u run_writes=%u pass=- fail=- insert_count=%u\n",
                        instance_id, WL_NAMES[p.workload], p.total_oram_ops, p.run_reads, p.run_writes, p.insert_count);
        }
        return 0;
    }

    // ---------------------------------------------------------------- card
    const std::string xclbin_path = pos[0], kname = "doma_kernel";
    const char* dev_env = std::getenv("DOMA_DEVICE");
    xrt::device device = (dev_env && std::strchr(dev_env, ':')) ? xrt::device(std::string(dev_env))
                                                                : xrt::device(dev_env ? unsigned(std::atoi(dev_env)) : 0u);
    xrt::xclbin xb(xclbin_path);
    auto uuid = device.load_xclbin(xb);
    auto krnl = xrt::kernel(device, uuid, kname);

    // where the xclbin connects the pages argument (HOST[0]: host memory; HBM[x]: device memory)
    std::string pages_tag;
    try { for (auto& m : xb.get_kernel(kname).get_arg(2).get_mems()) pages_tag += (pages_tag.empty() ? "" : ",") + m.get_tag(); }
    catch (const std::exception&) {}
    bool pages_host = pages_tag.empty() ? !std::getenv("DOMA_PAGES_HBM") : (pages_tag.rfind("HOST", 0) == 0);
    if (std::getenv("DOMA_PAGES_HBM")) pages_host = (std::string(std::getenv("DOMA_PAGES_HBM")) == "0");

    const size_t BLOCKA = std::getenv("DOMA_BLOCK") ? std::stoul(std::getenv("DOMA_BLOCK")) : MAX_BLOCK;
    if (BLOCKA < 32 || BLOCKA > MAX_BLOCK || (BLOCKA & (BLOCKA - 1))) { std::printf("DOMA_BLOCK must be a power of two, 32..4096\n"); return 1; }
    uint32_t NBLK = std::getenv("DOMA_N") ? uint32_t(std::stoul(std::getenv("DOMA_N"))) : uint32_t(num_slots);
    if (NBLK < uint32_t(num_slots)) { std::printf("DOMA_N=%u is smaller than num_slots=%d\n", NBLK, num_slots); return 1; }
    const uint32_t NB = (NBLK + LAMBDA - 1) / LAMBDA;
    const size_t stride = size_t(ZPOS) * BLOCKA;
    const double pool_bnd = stash_bound(double(NBLK), double(LAMBDA), int(ZPOS));
    uint32_t POOL = uint32_t(std::min<double>(NBLK, std::ceil(pool_bnd) + std::ceil(0.01 * NBLK) + 64));
    if (std::getenv("DOMA_POOL")) POOL = uint32_t(std::min<unsigned long>(NBLK, std::stoul(std::getenv("DOMA_POOL"))));
    const unsigned hbm_pct = std::getenv("DOMA_HBM_PCT") ? unsigned(std::stoul(std::getenv("DOMA_HBM_PCT"))) : 0u;
    if (hbm_pct > 100) { std::printf("DOMA_HBM_PCT must be 0..100\n"); return 1; }
    const uint32_t HBMK = uint32_t((uint64_t(NB) * hbm_pct) / 100);
    const size_t pages_bytes = size_t(NB - HBMK) * stride + 4096;

    size_t most = 1;
    for (auto& p : plans) most = std::max({most, p.load.size(), p.run.size() + p.pad.size()});
    const size_t CHUNK = std::getenv("YCSB_CHUNK") ? std::max<size_t>(1, std::stoul(std::getenv("YCSB_CHUNK"))) : 65536;
    const size_t max_cmds = std::min(most, CHUNK);

    std::printf("host_ycsb: oram_ycsb arguments inst=%d n_iters=%d num_slots=%d workload=%s init_occ=%d scan_len=%d%s\n",
                instance_id, n_iters, num_slots, wls.c_str(), init_occ, scan_len, PAD ? " (with gem5's padding reads)" : "");
    std::printf("doma_kernel: n=%u blocks in %u buckets (%u in HBM), stash pool %u (Claim 1 bound %.0f), pages in %s%s; "
                "up to %zu commands per launch\n", NBLK, NB, HBMK, POOL, pool_bnd,
                pages_tag.empty() ? (pages_host ? "host memory" : "device memory") : pages_tag.c_str(),
                std::getenv("DOMA_PAGES_HBM") ? " [DOMA_PAGES_HBM]" : "", max_cmds);

    xrt::bo cmd, resp, pages, meta, brec, stash, pages_hbm;
    {   // one buffer per argument; a failure names the buffer, its size and the usual cause
        const char* what = "";
        size_t bytes = 0;
        auto alloc = [&](const char* name, size_t sz, int arg) { what = name; bytes = sz; return xrt::bo(device, sz, krnl.group_id(arg)); };
        try {
            cmd   = alloc("cmd_input", roundup4k(max_cmds * 32) + max_cmds * BLOCKA, 0);
            resp  = alloc("resp_output", roundup4k((max_cmds + 2) * 32) + max_cmds * BLOCKA, 1);
            meta  = alloc("meta", std::max<size_t>(4096, ((size_t(NBLK) + 4) / 5) * 32), 4);
            brec  = alloc("brec", std::max<size_t>(4096, size_t(NB) * 64), 5);
            stash = alloc("stash", roundup4k(size_t(NB) * 64) + roundup4k(size_t(POOL) * 16) + size_t(POOL) * BLOCKA, 6);
            pages_hbm = alloc("pages_hbm", std::max<size_t>(4096, size_t(HBMK) * stride), 8);
        } catch (const std::exception& e) {
            std::printf("cannot allocate %s (%.1f MB): %s\n", what, bytes / 1e6, e.what());
            if (std::string(what) == "cmd_input" || std::string(what) == "resp_output")
                std::printf("lower YCSB_CHUNK (now %zu commands per launch) so the buffer fits its HBM bank\n", max_cmds);
            else
                std::printf("this xclbin's bank for %s is too small at n=%u: doma_hw.cfg (one 512 MB bank per argument) holds ~100K slots;\n"
                            "for up to 1M slots use a CFG=doma_big_hbm.cfg build (stash in HBM[12:14], HBM buckets in HBM[16:23]) with DOMA_HBM_PCT=75\n",
                            what, NBLK);
            return 1;
        }
    }
    try {
        pages = pages_host ? xrt::bo(device, pages_bytes, xrt::bo::flags::host_only, krnl.group_id(2))
                           : xrt::bo(device, pages_bytes, krnl.group_id(2));
    } catch (const std::exception& e) {
        std::printf("cannot allocate the pages buffer (%zu bytes) in %s: %s\n", pages_bytes, pages_host ? "host memory" : "device memory", e.what());
        if (pages_host) std::printf("This xclbin puts the buckets in host memory (HOST[0]), which needs host memory enabled on the card\n"
                                    "(xbutil configure --host-mem, an admin change). Use a build with the buckets in HBM\n"
                                    "(CFG=doma_hw.cfg up to ~100K slots; CFG=doma_big_hbm.cfg with DOMA_HBM_PCT=75 up to 1M).\n");
        else std::printf("The buckets outside HBM_PCT do not fit the bank this xclbin gives pages. doma_hw.cfg holds ~100K slots\n"
                         "(~200K with DOMA_HBM_PCT=50); for up to 1M slots use a CFG=doma_big_hbm.cfg build with DOMA_HBM_PCT=75.\n");
        return 1;
    }
    auto launch = [&](uint32_t n) { return krnl(cmd, resp, pages, n, meta, brec, stash, NBLK, pages_hbm, HBMK, POOL); };
    size_t BLOCK = BLOCKA;

    struct Summary { bool ok; uint32_t beats, lat_min, lat_max, errors, done; uint64_t total; };
    auto read_summary = [&](size_t n, bool with_data) {
        const size_t hr = roundup4k((n + 2) * 32);
        resp.sync(XCL_BO_SYNC_BO_FROM_DEVICE, with_data ? hr + n * BLOCK : (n + 2) * 32, 0);
        const uint8_t* r = resp.map<uint8_t*>() + n * 32;
        Summary s{}; s.ok = (r[0] == T_SUM);
        std::memcpy(&s.beats, r + 4, 4); std::memcpy(&s.total, r + 8, 8); std::memcpy(&s.lat_min, r + 16, 4);
        std::memcpy(&s.lat_max, r + 20, 4); std::memcpy(&s.errors, r + 24, 4); std::memcpy(&s.done, r + 28, 4);
        return s;
    };
    auto explain = [](uint32_t e) {
        std::string s;
        if (e & 0x01) s += " 0x1 lease refused;";
        if (e & 0x08) s += " 0x8 DOMA HALTED (integrity check, stash overflow or a timing budget missed: results invalid);";
        if (e & 0x10) s += " 0x10 command timeout;";
        if (e & 0x20) s += " 0x20 buffers differ from the init launch;";
        if (e & 0x80) s += " 0x80 n above the bitstream's maximum (rebuild with a larger NMAX, or lower DOMA_N/num_slots);";
        return s;
    };
    FILE* csv = std::getenv("YCSB_CSV") ? std::fopen(std::getenv("YCSB_CSV"), "w") : nullptr;
    if (csv) std::fprintf(csv, "workload,phase,cmd,opIdx,slot,op,latency_cycles,status\n");

    // run one phase's ops in launches of max_cmds; check headers and (for reads) the data
    auto run_phase = [&](const char* wname, const char* phase, const std::vector<Op>& ops, PhaseResult& R) -> bool {
        for (size_t at = 0; at < ops.size(); at += max_cmds) {
            const size_t n = std::min(max_cmds, ops.size() - at);
            const size_t hc = roundup4k(n * 32), hr = roundup4k((n + 2) * 32);
            uint8_t* p = cmd.map<uint8_t*>();
            std::memset(p, 0, hc);
            for (size_t i = 0; i < n; i++) {
                const Op& o = ops[at + i];
                p[i * 32] = o.write ? T_PUT : T_GET;
                std::memcpy(p + i * 32 + 4, &o.slot, 4);          // bytes 8-15: offset 0, length 0 = the whole block
                if (o.write) fill_block(p + hc + i * BLOCK, BLOCK, o.opIdx, o.slot);
                else std::memset(p + hc + i * BLOCK, 0, BLOCK);   // a GET's data slot is streamed too (ignored)
            }
            cmd.sync(XCL_BO_SYNC_BO_TO_DEVICE, hc + n * BLOCK, 0);
            auto t0 = std::chrono::steady_clock::now();
            auto run = launch(uint32_t(n));
            run.wait();
            R.wall += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            Summary s = read_summary(n, true);
            R.cycles += s.total; R.errors |= s.errors; R.summary_ok &= s.ok; R.done += s.done;
            const uint8_t* rp = resp.map<uint8_t*>();
            std::vector<uint8_t> want(BLOCK);
            for (size_t i = 0; i < n; i++) {
                const Op& o = ops[at + i];
                const uint8_t* h = rp + i * 32;
                uint32_t slot, lat, st; std::memcpy(&slot, h + 4, 4); std::memcpy(&lat, h + 8, 4); std::memcpy(&st, h + 12, 4);
                R.cmds++; (o.write ? R.puts : R.gets)++;
                if (csv) std::fprintf(csv, "%s,%s,%zu,%u,%u,%s,%u,0x%x\n", wname, phase, at + i, o.opIdx, o.slot, o.write ? "PUT" : "GET", lat, st);
                if (h[0] != T_ACK || slot != o.slot || st != 0) {
                    if (R.bad_hdr++ < 5) std::printf("  %s cmd %zu (opIdx %u, slot %u): response 0x%02x slot %u status 0x%x\n",
                                                     phase, at + i, o.opIdx, o.slot, h[0], slot, st);
                    if (!o.write) R.fail++;
                    continue;
                }
                (o.write ? R.put : R.get).add(lat); R.all.add(lat);
                if (o.write) continue;
                if (o.expect == NONE) { R.fail++; std::printf("  read of never-written slot %u (opIdx %u)\n", o.slot, o.opIdx); continue; }
                fill_block(want.data(), BLOCK, o.expect, o.slot);
                const uint8_t* got = rp + hr + i * BLOCK;
                if (std::memcmp(got, want.data(), BLOCK) == 0) R.pass++;
                else {
                    uint32_t g[4]; std::memcpy(g, got, 16);
                    if (R.fail < 5)
                        std::fprintf(stderr, "MISMATCH wl=%s opIdx=%u slot=%u exp_wr_idx=%u got=[%08x %08x %08x %08x] exp=[%08x %08x]\n",
                                     wname, o.opIdx, o.slot, o.expect, g[0], g[1], g[2], g[3], payload_word(o.expect, 0), payload_word(o.expect, 1));
                    R.fail++;
                }
            }
            if (s.errors & 0x18) { std::printf("  launch ended early: errors 0x%x:%s\n", s.errors, explain(s.errors).c_str()); return false; }
        }
        return true;
    };
    auto report = [&](const char* phase, PhaseResult& R) {
        std::printf("%s: %zu commands (%zu GET, %zu PUT) in %.6f s = %.2f Kops/s; device %llu cycles = %.1f cycles per command",
                    phase, R.cmds, R.gets, R.puts, R.wall, R.cmds / std::max(R.wall, 1e-12) / 1e3,
                    (unsigned long long)R.cycles, R.cmds ? double(R.cycles) / R.cmds : 0.0);
        if (R.wall > 0.05) std::printf(" (device cycles / wall time = %.0f MHz)", R.cycles / R.wall / 1e6);
        std::printf("\n  latency, request to done: GET %s\n", R.get.str(MHZ).c_str());
        std::printf("                            PUT %s\n", R.put.str(MHZ).c_str());
        if (R.errors || !R.summary_ok || R.bad_hdr)
            std::printf("  errors 0x%x:%s summary %s, %zu bad responses\n", R.errors, explain(R.errors).c_str(), R.summary_ok ? "ok" : "MISSING", R.bad_hdr);
    };

    struct Row { int w; PhaseResult load, run, pad; bool ok; };
    std::vector<Row> rows;
    bool all_ok = true;
    for (auto& P : plans) {
        const char* wn = WL_NAMES[P.workload];
        std::printf("\n==== workload %s ====\n", wn);
        // init launch: DOMA (re)boots with these buffers, as every gem5 run starts from a fresh boot
        auto t0 = std::chrono::steady_clock::now();
        auto r0 = launch(0u); r0.wait();
        const double tboot = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        Summary s0 = read_summary(0, false);
        if (!s0.ok || s0.errors) {
            std::printf("init launch failed: summary %s, errors 0x%x:%s\n", s0.ok ? "ok" : "MISSING", s0.errors, explain(s0.errors).c_str());
            return 1;
        }
        if (s0.beats) BLOCK = size_t(s0.beats) * 32;
        if (BLOCK > BLOCKA) { std::printf("the kernel's block is %zu bytes, larger than the buffers (DOMA_BLOCK=%zu)\n", BLOCK, BLOCKA); return 1; }
        std::printf("boot: %llu cycles (%.3f ms at %.0f MHz), %.3f s wall; block %zu bytes%s\n", (unsigned long long)s0.total,
                    s0.total / MHZ / 1e3, MHZ, tboot, BLOCK, s0.beats ? "" : " (not reported by the bitstream; assuming 4 KB)");
        Row row{P.workload, {}, {}, {}, true};
        bool ok = run_phase(wn, "load", P.load, row.load);
        report("LOAD", row.load);
        if (ok) { ok = run_phase(wn, "run", P.run, row.run); report("RUN ", row.run); }
        if (ok && !P.pad.empty()) { ok = run_phase(wn, "pad", P.pad, row.pad); report("PAD ", row.pad); }
        row.ok = ok && row.load.errors == 0 && row.run.errors == 0 && row.pad.errors == 0 && row.load.bad_hdr == 0
                 && row.run.bad_hdr == 0 && row.pad.bad_hdr == 0 && row.run.fail == 0 && row.pad.fail == 0
                 && row.run.summary_ok && row.load.summary_ok && row.run.cmds == P.run.size();
        std::printf("verify: %u run-phase reads checked, pass %u, fail %u%s\n", row.run.pass + row.run.fail, row.run.pass, row.run.fail,
                    P.pad.empty() ? "" : (", padding reads pass " + std::to_string(row.pad.pass) + " fail " + std::to_string(row.pad.fail)).c_str());
        // the same line as oram_ycsb.c prints at the end of a gem5 run
        std::printf("oram_ycsb: inst=%d wl=%s total_oram_ops=%u run_reads=%u run_writes=%u pass=%u fail=%u insert_count=%u\n",
                    instance_id, wn, P.total_oram_ops, P.run_reads, P.run_writes, row.run.pass, row.run.fail, P.insert_count);
        std::printf("%s\n", row.ok ? "WORKLOAD PASSED" : "WORKLOAD FAILED");
        all_ok &= row.ok;
        rows.push_back(std::move(row));
        if (!ok) break;
    }
    if (csv) std::fclose(csv);
    if (rows.size() > 1) {
        std::printf("\nsummary (run phase; latency = request to done, cycles; us at %.0f MHz)\n", MHZ);
        std::printf("wl   ops      GET      PUT   Kops/s  cyc/cmd  GET mean  PUT mean  all p99   max   mean us  result\n");
        for (auto& r : rows) {
            std::vector<uint32_t> a = r.run.all.v; std::sort(a.begin(), a.end());
            uint32_t p99 = a.empty() ? 0 : a[std::min(a.size() - 1, size_t(0.99 * (a.size() - 1) + 0.5))];
            std::printf("%-3s %6zu %8zu %8zu %8.2f %8.1f %9.1f %9.1f %8u %6u %9.2f  %s\n", WL_NAMES[r.w], r.run.cmds, r.run.gets, r.run.puts,
                        r.run.cmds / std::max(r.run.wall, 1e-12) / 1e3, r.run.cmds ? double(r.run.cycles) / r.run.cmds : 0.0,
                        r.run.get.mean(), r.run.put.mean(), p99, r.run.all.max(), r.run.all.mean() / MHZ, r.ok ? "PASS" : "FAIL");
        }
    }
    std::printf("%s\n", all_ok ? "ALL WORKLOADS PASSED" : "ERRORS FOUND");
    return all_ok ? 0 : 1;
}
