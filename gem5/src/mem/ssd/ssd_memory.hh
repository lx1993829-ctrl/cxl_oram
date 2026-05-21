/*
 * ssd_memory.hh — SSD-backed memory device for gem5
 *
 * Features:
 *   - SimpleSSD NAND timing backend
 *   - Page coalescing: sub-page beats to same 4KB page share one access
 *   - DRAM cache: on-device page cache at DRAM speed, misses go to NAND
 */

#ifndef __MEM_SSD_SSD_MEMORY_HH__
#define __MEM_SSD_SSD_MEMORY_HH__

#include <deque>
#include <unordered_map>
#include <vector>

#include "mem/port.hh"
#include "mem/ssd/ssd_engine.hh"
#include "base/statistics.hh"
#include "params/SsdMemory.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

// SimpleSSD headers
#include "hil/hil.hh"
#include "util/simplessd.hh"

namespace gem5
{

class SsdMemory : public ClockedObject
{
  public:
    PARAMS(SsdMemory);
    SsdMemory(const Params &p);
    ~SsdMemory();

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
    void init() override;

  private:

    // ================================================================
    //  Memory port — faces the host xbar
    // ================================================================
    class MemoryPort : public ResponsePort
    {
      public:
        MemoryPort(const std::string &n, SsdMemory &owner);

      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        AddrRangeList getAddrRanges() const override;

      private:
        SsdMemory &owner;
    };

    MemoryPort port;

    // ================================================================
    //  Address range
    // ================================================================
    AddrRange addrRange;

    // ================================================================
    //  Backing store (data correctness)
    // ================================================================
    uint8_t *pmem;
    size_t pmemSize;

    Addr toOffset(Addr addr) const { return addr - addrRange.start(); }

    // ================================================================
    //  SimpleSSD backend (timing)
    //  IMPORTANT: engine must be declared BEFORE pHIL. C++ destroys
    //  members in reverse declaration order, so ~pHIL runs first
    //  (calling engine.deallocateEvent) while engine is still alive.
    //  Reordering these fields will cause use-after-free on shutdown.
    // ================================================================
    SsdEngine engine;
    SimpleSSD::ConfigReader ssdConfig;
    SimpleSSD::HIL::HIL *pHIL;

    uint32_t logicalPageSize;
    uint32_t latencyMultiplier;
    uint64_t nextReqId;

    // Call SimpleSSD for NAND timing (bypasses coalescing and cache)
    Tick ssdLatency(Addr byteAddr, unsigned size, bool isRead);

    // ================================================================
    //  Page coalescing
    // ================================================================
    // Tracks the last access time per page. If a second access to the
    // same page arrives within coalesceWindow, return coalesceLatency
    // instead of calling SimpleSSD.
    Tick coalesceWindow;    // 0 = disabled
    Tick coalesceLatency;   // fast latency for coalesced access

    // Map: page number → tick of last SSD access to that page
    std::unordered_map<uint64_t, Tick> lastPageAccessTick;

    // ================================================================
    //  DRAM cache
    // ================================================================
    // Set-associative page cache. Each entry holds one 4KB page tag.
    // On hit: return dramCacheHitLatency.
    // On miss: fetch from SSD (ssdLatency), install in cache,
    //          evict LRU entry (writeback dirty if needed).
    bool dramCacheEnabled;
    Tick dramCacheHitLatency;
    uint64_t dramCacheCapacity;   // total pages in cache
    uint32_t dramCacheAssoc;      // ways per set
    uint64_t dramCacheNumSets;    // sets = capacity / assoc
    bool dramCacheWriteBack;      // true = writeback, false = writethrough

    // Cache activation: bypass cache for first N accesses (init phase)
    uint64_t dramCacheWarmAfter;  // 0 = active immediately
    uint64_t totalAccessCount;    // counts all accesses, cache activates when this >= warmAfter
    bool dramCacheActive;         // flipped to true when totalAccessCount >= warmAfter

    struct CacheLine {
        uint64_t tag;       // page number
        bool valid;
        bool dirty;
        uint64_t lruCounter; // for LRU eviction
    };

    // Cache storage: sets × ways
    std::vector<std::vector<CacheLine>> dramCache;
    uint64_t globalLruCounter;

    // Cache operations — return the timing for this access
    Tick dramCacheAccess(Addr byteAddr, unsigned size, bool isRead);
    bool dramCacheLookup(uint64_t pageNum, uint32_t &way);
    void dramCacheInstall(uint64_t pageNum, bool dirty, Tick *evictLatency);

    // ================================================================
    //  Top-level latency computation (combines coalescing + cache + SSD)
    // ================================================================
    Tick accessLatency(Addr byteAddr, unsigned size, bool isRead);

    // ================================================================
    //  Pending requests (timing mode)
    // ================================================================
    struct PendingReq {
        PacketPtr pkt;
        Tick completionTick;
    };
    std::deque<PendingReq> pendingReqs;
    EventFunctionWrapper completionEvent;
    void processCompletion();

    void insertPending(PacketPtr pkt, Tick completionTick);
    void scheduleNextCompletion();

    // Response retry state
    bool retryResp;
    PacketPtr retryPkt;

    // Backpressure: limit pending queue depth (A2 fix)
    uint32_t maxQueueDepth;   // 0 = unlimited
    bool needRetry;           // upstream blocked, send retry when slot frees

    // ================================================================
    //  Statistics (gem5 Stats::Group — appears in stats.txt)
    // ================================================================
    struct SsdStats : public statistics::Group
    {
        SsdStats(SsdMemory &parent);

        statistics::Scalar numReads;
        statistics::Scalar numWrites;
        statistics::Scalar totalReadLatency;
        statistics::Scalar totalWriteLatency;
        statistics::Scalar coalesceHits;
        statistics::Scalar cacheHits;
        statistics::Scalar cacheMisses;
        statistics::Scalar cacheEvictions;
        statistics::Scalar cacheDirtyEvictions;
        statistics::Scalar queueFullRetries;
        statistics::Histogram readLatencyHist;
        statistics::Histogram writeLatencyHist;
        statistics::Formula avgReadLatency;
        statistics::Formula avgWriteLatency;
    } stats;
};

} // namespace gem5

#endif // __MEM_SSD_SSD_MEMORY_HH__
