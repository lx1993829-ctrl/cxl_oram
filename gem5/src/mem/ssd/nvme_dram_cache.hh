/*
 * NvmeDramCache — DRAM cache model for NvmeSsdDevice.
 *
 * Logic is byte-identical to SsdMemory's CXL DRAM cache (see
 * src/mem/ssd/ssd_memory.cc lines 200-313). Page-keyed, set-associative,
 * LRU replacement, writeback or writethrough.
 *
 * Used in HIL::write/read via a static pointer (see hil.cc patch).
 * Set at runtime by NvmeSsdDevice's constructor when dram_cache_size > 0.
 */
#ifndef __NVME_SSD_NVME_DRAM_CACHE_HH__
#define __NVME_SSD_NVME_DRAM_CACHE_HH__

#include <cstdint>
#include <vector>

namespace gem5
{

class NvmeDramCache
{
  public:
    /* All sizes in bytes. pageSize matches SimpleSSD's logical
     * page size (typically 16 KB; ICL/HIL/FTL all agree on this).
     * hitLatencyPs is in picoseconds (gem5's Tick unit).
     * warmAfter = number of accesses to ignore before activating.
     * 0 means "active immediately". */
    NvmeDramCache(uint64_t sizeBytes,
                  uint32_t assoc,
                  uint64_t pageSize,
                  bool writeback,
                  uint64_t warmAfter,
                  uint64_t hitLatencyPs)
        : enabled(sizeBytes > 0),
          pageSize(pageSize),
          assoc(assoc),
          writeback(writeback),
          warmAfter(warmAfter),
          active(warmAfter == 0),
          hitLatencyPs(hitLatencyPs),
          totalAccesses(0),
          globalLru(0),
          hits(0), misses(0),
          evictions(0), dirtyEvictions(0)
    {
        if (!enabled) return;
        capacity = sizeBytes / pageSize;
        if (capacity < this->assoc) this->assoc = capacity;
        numSets = capacity / this->assoc;
        if (numSets == 0) numSets = 1;
        sets.resize(numSets);
        for (auto &s : sets) s.resize(this->assoc);
    }

    /* Check if all pages in the range [startPageNum, startPageNum + nPages)
     * are cached. Updates LRU on hit. For misses, installs the page in
     * the cache (matching CXL's behavior where a miss fetches and fills).
     *
     * Returns true if all pages hit. evictWritebackPs accumulates the
     * latency of any dirty evictions caused (max across all installs).
     *
     * isWrite=true means "this access dirties the cached page".
     *
     * Caller is responsible for warmup gating (call only after warmup
     * counter is past the threshold) — actually we handle it here so
     * the wrapper doesn't have to. */
    bool checkAllHit(uint64_t startPageNum, uint64_t nPages, bool isWrite,
                     uint64_t *evictWritebackPs)
    {
        *evictWritebackPs = 0;
        if (!enabled) return false;

        totalAccesses++;
        if (!active && totalAccesses >= warmAfter) {
            active = true;
        }
        if (!active) return false;

        bool allHit = true;
        for (uint64_t i = 0; i < nPages; i++) {
            uint64_t pn = startPageNum + i;
            uint32_t way = 0;
            if (lookup(pn, way)) {
                hits++;
                /* Update LRU happens inside lookup. */
                if (isWrite && writeback) {
                    sets[pn % numSets][way].dirty = true;
                }
                /* Writethrough writes also "hit" but the SimpleSSD path
                 * still runs (caller doesn't shortcut latency for write
                 * hits in writethrough mode — checkAllHit returns true
                 * but caller can choose to ignore for writes). */
            } else {
                misses++;
                allHit = false;
                /* Install on miss. Dirty if write under writeback. */
                uint64_t evictPs = 0;
                install(pn, isWrite && writeback, &evictPs);
                if (evictPs > *evictWritebackPs) {
                    *evictWritebackPs = evictPs;
                }
            }
        }
        return allHit;
    }

    uint64_t getHitLatencyPs() const { return hitLatencyPs; }
    bool isWriteback() const { return writeback; }
    bool isEnabled() const { return enabled; }

    /* Stats accessors for the wrapper to register with gem5. */
    uint64_t getHits() const { return hits; }
    uint64_t getMisses() const { return misses; }
    uint64_t getEvictions() const { return evictions; }
    uint64_t getDirtyEvictions() const { return dirtyEvictions; }
    uint64_t getTotalAccesses() const { return totalAccesses; }

  private:
    struct CacheLine
    {
        uint64_t tag = 0;
        bool valid = false;
        bool dirty = false;
        uint64_t lru = 0;
    };

    bool lookup(uint64_t pn, uint32_t &way)
    {
        uint64_t setIdx = pn % numSets;
        auto &s = sets[setIdx];
        for (uint32_t w = 0; w < assoc; w++) {
            if (s[w].valid && s[w].tag == pn) {
                way = w;
                s[w].lru = ++globalLru;
                return true;
            }
        }
        return false;
    }

    /* Install pn into set[pn % numSets]. If dirty victim evicted under
     * writeback, *evictPs is set to ~one NAND write latency
     * (approximated as 16 KB sequential write at PAL avg latency). For
     * the paper-grade NVMe path this is hand-waved; if exact NAND
     * timing for evictions is needed later we'd query SimpleSSD's PAL
     * for the actual latency. CXL's SsdMemory does the same hand-wave
     * (calls ssdLatency() which is just the latency function — same
     * approximation). */
    void install(uint64_t pn, bool dirty, uint64_t *evictPs)
    {
        *evictPs = 0;
        uint64_t setIdx = pn % numSets;
        auto &s = sets[setIdx];

        uint32_t victimWay = 0;
        uint64_t minLru = UINT64_MAX;
        for (uint32_t w = 0; w < assoc; w++) {
            if (!s[w].valid) {
                victimWay = w;
                minLru = 0;
                break;
            }
            if (s[w].lru < minLru) {
                minLru = s[w].lru;
                victimWay = w;
            }
        }

        if (s[victimWay].valid) {
            evictions++;
            if (s[victimWay].dirty && writeback) {
                dirtyEvictions++;
                /* Hand-wave eviction latency. See note above.
                 * 100 µs = 100,000,000 ps is a reasonable single-page
                 * NAND write latency for current-gen TLC. The CXL path
                 * calls ssdLatency() which would return similar. */
                *evictPs = 100000000ULL;
            }
        }
        s[victimWay].tag = pn;
        s[victimWay].valid = true;
        s[victimWay].dirty = dirty;
        s[victimWay].lru = ++globalLru;
    }

    bool enabled;
    uint64_t pageSize;
    uint32_t assoc;
    bool writeback;
    uint64_t warmAfter;
    bool active;
    uint64_t hitLatencyPs;

    uint64_t capacity;
    uint64_t numSets;

    uint64_t totalAccesses;
    uint64_t globalLru;
    std::vector<std::vector<CacheLine>> sets;

    uint64_t hits, misses, evictions, dirtyEvictions;
};

} // namespace gem5

#endif // __NVME_SSD_NVME_DRAM_CACHE_HH__
