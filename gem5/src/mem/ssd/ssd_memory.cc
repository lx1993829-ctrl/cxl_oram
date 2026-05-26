/*
 * ssd_memory.cc — SSD-backed memory device for gem5
 *
 * Features:
 *   - SimpleSSD NAND timing backend
 *   - Page coalescing: sub-page beats to same 4KB page share one access
 *   - DRAM cache: on-device page cache at DRAM speed, misses go to NAND
 */

#include "mem/ssd/ssd_memory.hh"

#include <algorithm>
#include <cstring>

#include "base/trace.hh"
#include "debug/SsdMemory.hh"
#include "sim/cur_tick.hh"
#include "sim/system.hh"

namespace gem5
{

// ====================================================================
//  Constructor / Destructor
// ====================================================================

SsdMemory::SsdMemory(const Params &p)
    : ClockedObject(p),
      port(p.name + ".port", *this),
      addrRange(p.range),
      pmem(nullptr),
      pmemSize(p.range.size()),
      engine(),
      pHIL(nullptr),
      logicalPageSize(p.logical_page_size),
      latencyMultiplier(p.latency_multiplier),
      nextReqId(0),
      // Page coalescing
      coalesceWindow(p.coalesce_window),
      coalesceLatency(p.coalesce_latency),
      // DRAM cache
      dramCacheEnabled(p.dram_cache_size > 0),
      dramCacheHitLatency(p.dram_cache_hit_latency),
      dramCacheCapacity(0),
      dramCacheAssoc(p.dram_cache_assoc),
      dramCacheNumSets(0),
      dramCacheWriteBack(p.dram_cache_write_policy == "writeback"),
      dramCacheWarmAfter(p.dram_cache_warm_after),
      totalAccessCount(0),
      dramCacheActive(p.dram_cache_warm_after == 0),
      globalLruCounter(0),
      // Completion
      completionEvent([this]{ processCompletion(); }, name()),
      retryResp(false),
      retryPkt(nullptr),
      maxQueueDepth(p.max_queue_depth),
      needRetry(false),
      stats(*this)
{
    // Allocate backing store (zeroed)
    pmem = (uint8_t *)calloc(1, pmemSize);
    if (!pmem) {
        fatal("SsdMemory: failed to allocate %lu bytes backing store",
              pmemSize);
    }
    inform("SsdMemory: allocated %lu MB backing store for range [%#lx, %#lx)",
           pmemSize / (1024*1024), addrRange.start(), addrRange.end());

    // Connect engine to gem5's event system FIRST — SimpleSSD init
    // or future revisions may schedule events during construction.
    engine.setEventManager(this);

    // Initialize SimpleSSD engine and parse config
    ssdConfig = initSimpleSSD(&engine, p.ssd_config);

    // Create the HIL (front desk)
    pHIL = new SimpleSSD::HIL::HIL(ssdConfig);

    inform("SsdMemory: SimpleSSD initialized, config=%s, pageSize=%u",
           p.ssd_config, logicalPageSize);

    // F1 fix: verify logicalPageSize matches SimpleSSD's config.
    // Mismatch causes slpn math to map to wrong channels/dies under
    // PageAllocation=CWDP, silently corrupting the parallelism model.
    {
        uint32_t ssdPageSize = ssdConfig.readUint(
            SimpleSSD::CONFIG_PAL, SimpleSSD::PAL::NAND_PAGE_SIZE);
        if (ssdPageSize > 0 && logicalPageSize != ssdPageSize) {
            fatal("SsdMemory: logical_page_size (%u) does not match "
                  "SimpleSSD config PageSize (%u). Set logical_page_size=%u "
                  "in the Python config or change PageSize in %s.",
                  logicalPageSize, ssdPageSize, ssdPageSize,
                  p.ssd_config.c_str());
        }
    }

    // Page coalescing
    if (coalesceWindow > 0) {
        inform("SsdMemory: page coalescing enabled, window=%lu ps, "
               "coalesce_latency=%lu ps",
               coalesceWindow, coalesceLatency);
    }

    // DRAM cache setup
    if (dramCacheEnabled) {
        uint64_t cacheSizeBytes = p.dram_cache_size;
        dramCacheCapacity = cacheSizeBytes / logicalPageSize;
        if (dramCacheCapacity < dramCacheAssoc) {
            dramCacheAssoc = dramCacheCapacity;
        }
        dramCacheNumSets = dramCacheCapacity / dramCacheAssoc;
        if (dramCacheNumSets == 0) dramCacheNumSets = 1;

        // Allocate cache array
        dramCache.resize(dramCacheNumSets);
        for (auto &set : dramCache) {
            set.resize(dramCacheAssoc);
            for (auto &line : set) {
                line.tag = 0;
                line.valid = false;
                line.dirty = false;
                line.lruCounter = 0;
            }
        }

        inform("SsdMemory: DRAM cache enabled, %lu MB (%lu pages), "
               "%lu sets × %u ways, %s, hit_latency=%lu ps, warm_after=%lu",
               cacheSizeBytes / (1024*1024), dramCacheCapacity,
               dramCacheNumSets, dramCacheAssoc,
               dramCacheWriteBack ? "writeback" : "writethrough",
               dramCacheHitLatency, dramCacheWarmAfter);
    }
}

SsdMemory::~SsdMemory()
{
    delete pHIL;
    free(pmem);

    inform("SsdMemory stats: reads=%lu writes=%lu "
           "avgReadLatency=%lu ps avgWriteLatency=%lu ps",
           (uint64_t)stats.numReads.value(),
           (uint64_t)stats.numWrites.value(),
           stats.numReads.value()  ? (uint64_t)(stats.totalReadLatency.value()  / stats.numReads.value())  : 0,
           stats.numWrites.value() ? (uint64_t)(stats.totalWriteLatency.value() / stats.numWrites.value()) : 0);

    if (coalesceWindow > 0) {
        inform("SsdMemory coalescing: hits=%lu",
               (uint64_t)stats.coalesceHits.value());
    }

    if (dramCacheEnabled) {
        uint64_t total = (uint64_t)(stats.cacheHits.value() + stats.cacheMisses.value());
        inform("SsdMemory DRAM cache: hits=%lu misses=%lu rate=%.1f%% "
               "evictions=%lu dirty_evictions=%lu",
               (uint64_t)stats.cacheHits.value(),
               (uint64_t)stats.cacheMisses.value(),
               total ? (double)stats.cacheHits.value() / total * 100.0 : 0.0,
               (uint64_t)stats.cacheEvictions.value(),
               (uint64_t)stats.cacheDirtyEvictions.value());
    }
}

void
SsdMemory::init()
{
    ClockedObject::init();

    if (!port.isConnected()) {
        fatal("SsdMemory: port not connected");
    }

    port.sendRangeChange();
}

Port &
SsdMemory::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "port") {
        return port;
    }
    return ClockedObject::getPort(if_name, idx);
}

// ====================================================================
//  SimpleSSD latency computation (raw NAND path)
// ====================================================================

Tick
SsdMemory::ssdLatency(Addr byteAddr, unsigned size, bool isRead)
{
    uint64_t slpn = byteAddr / logicalPageSize;
    uint64_t offset = byteAddr % logicalPageSize;

    uint64_t interval = 0;
    const uint64_t startTick = curTick();
    SimpleSSD::DMAFunction completion =
        [&interval, startTick](uint64_t tick, void *) {
            interval = tick > startTick ? tick - startTick : 0;
        };
    SimpleSSD::HIL::Request req(completion, nullptr);
    req.reqID = ++nextReqId;
    req.range.slpn = slpn;
    req.range.nlp = 1;
    req.offset = offset;
    req.length = size;
    req.function = [](uint64_t, void *) {};
    req.context = nullptr;

    if (isRead) {
        pHIL->read(req);
    } else {
        pHIL->write(req);
    }

    Tick latency = interval * latencyMultiplier;
    if (latency == 0) {
        latency = 1000;
    }

    return latency;
}

// ====================================================================
//  DRAM cache
// ====================================================================

bool
SsdMemory::dramCacheLookup(uint64_t pageNum, uint32_t &way)
{
    uint64_t setIdx = pageNum % dramCacheNumSets;
    auto &set = dramCache[setIdx];

    for (uint32_t w = 0; w < dramCacheAssoc; w++) {
        if (set[w].valid && set[w].tag == pageNum) {
            way = w;
            set[w].lruCounter = ++globalLruCounter;
            return true;
        }
    }
    return false;
}

void
SsdMemory::dramCacheInstall(uint64_t pageNum, bool dirty, Tick *evictLatency)
{
    *evictLatency = 0;

    uint64_t setIdx = pageNum % dramCacheNumSets;
    auto &set = dramCache[setIdx];

    // Find invalid line or LRU victim
    uint32_t victimWay = 0;
    uint64_t minLru = UINT64_MAX;

    for (uint32_t w = 0; w < dramCacheAssoc; w++) {
        if (!set[w].valid) {
            victimWay = w;
            minLru = 0;
            break;
        }
        if (set[w].lruCounter < minLru) {
            minLru = set[w].lruCounter;
            victimWay = w;
        }
    }

    // Evict victim if valid
    if (set[victimWay].valid) {
        stats.cacheEvictions++;
        if (set[victimWay].dirty && dramCacheWriteBack) {
            // Dirty eviction — must write back to NAND
            stats.cacheDirtyEvictions++;
            *evictLatency = ssdLatency(
                set[victimWay].tag * logicalPageSize,
                logicalPageSize, false /* write */);

            DPRINTF(SsdMemory, "  DRAM cache evict dirty page %lu, "
                    "writeback latency=%lu ps\n",
                    set[victimWay].tag, *evictLatency);
        }
    }

    // Install new line
    set[victimWay].tag = pageNum;
    set[victimWay].valid = true;
    set[victimWay].dirty = dirty;
    set[victimWay].lruCounter = ++globalLruCounter;
}

Tick
SsdMemory::dramCacheAccess(Addr byteAddr, unsigned size, bool isRead)
{
    uint64_t pageNum = byteAddr / logicalPageSize;
    uint32_t way;

    if (dramCacheLookup(pageNum, way)) {
        // Cache hit
        stats.cacheHits++;
        uint64_t setIdx = pageNum % dramCacheNumSets;

        if (!isRead) {
            if (dramCacheWriteBack) {
                dramCache[setIdx][way].dirty = true;
            } else {
                // Writethrough: also write to SSD
                Tick ssdLat = ssdLatency(byteAddr, size, false);
                return std::max(dramCacheHitLatency, ssdLat);
            }
        }

        DPRINTF(SsdMemory, "  DRAM cache HIT page %lu, latency=%lu ps\n",
                pageNum, dramCacheHitLatency);
        return dramCacheHitLatency;
    }

    // Cache miss
    stats.cacheMisses++;

    // Fetch page from SSD
    Tick fetchLatency = ssdLatency(byteAddr, size, isRead);

    // Install in cache (may evict dirty page)
    Tick evictLatency = 0;
    dramCacheInstall(pageNum, !isRead /* dirty if write */, &evictLatency);

    // Total latency: eviction writeback (if any) overlaps with fetch
    // In real hardware, eviction and fetch can be pipelined on different
    // NAND channels. Model as max(fetch, evict) for simplicity.
    Tick totalLatency = std::max(fetchLatency, evictLatency);

    DPRINTF(SsdMemory, "  DRAM cache MISS page %lu, fetch=%lu evict=%lu "
            "total=%lu ps\n",
            pageNum, fetchLatency, evictLatency, totalLatency);

    return totalLatency;
}

// ====================================================================
//  Top-level latency: coalescing → DRAM cache → SSD
// ====================================================================

Tick
SsdMemory::accessLatency(Addr byteAddr, unsigned size, bool isRead)
{
    uint64_t pageNum = byteAddr / logicalPageSize;
    Tick now = curTick();

    // Track total accesses for cache warm-up
    totalAccessCount++;

    // Activate cache after warm-up period (e.g. after ORAM write-init)
    if (dramCacheEnabled && !dramCacheActive &&
        totalAccessCount >= dramCacheWarmAfter) {
        dramCacheActive = true;
        DPRINTF(SsdMemory, "DRAM cache activated after %lu accesses\n",
                totalAccessCount);
    }

    // --- Page coalescing ---
    if (coalesceWindow > 0) {
        auto it = lastPageAccessTick.find(pageNum);
        if (it != lastPageAccessTick.end()) {
            Tick elapsed = now - it->second;
            if (elapsed <= coalesceWindow) {
                stats.coalesceHits++;
                // A6 fix: slide the window — recent activity extends
                // the page-open duration (LRU-like, not fixed-duration).
                it->second = now;
                DPRINTF(SsdMemory, "  coalesced page %lu (elapsed=%lu ps)\n",
                        pageNum, elapsed);
                return coalesceLatency;
            } else {
                // A5 fix: stale entry, evict to bound map growth.
                lastPageAccessTick.erase(it);
            }
        }
        lastPageAccessTick[pageNum] = now;
    }

    // --- DRAM cache (only if active) ---
    // Note: coalesced hits above model the on-device register/buffer
    // and intentionally bypass the DRAM cache tier. A coalesced access
    // does NOT refresh DRAM cache LRU state or install missing pages.
    if (dramCacheEnabled && dramCacheActive) {
        return dramCacheAccess(byteAddr, size, isRead);
    }

    // --- Raw SSD ---
    return ssdLatency(byteAddr, size, isRead);
}

// ====================================================================
//  Scheduling helpers
// ====================================================================

void
SsdMemory::insertPending(PacketPtr pkt, Tick completionTick)
{
    PendingReq entry{pkt, completionTick};

    auto it = pendingReqs.begin();
    while (it != pendingReqs.end() && it->completionTick <= completionTick) {
        ++it;
    }
    pendingReqs.insert(it, entry);
}

void
SsdMemory::scheduleNextCompletion()
{
    if (pendingReqs.empty() || retryResp)
        return;

    Tick nextTick = pendingReqs.front().completionTick;
    Tick now = curTick();
    if (nextTick < now) nextTick = now;

    if (!completionEvent.scheduled()) {
        schedule(completionEvent, nextTick);
    } else if (nextTick < completionEvent.when()) {
        reschedule(completionEvent, nextTick);
    }
}

// ====================================================================
//  MemoryPort implementation
// ====================================================================

SsdMemory::MemoryPort::MemoryPort(const std::string &n, SsdMemory &o)
    : ResponsePort(n), owner(o)
{
}

AddrRangeList
SsdMemory::MemoryPort::getAddrRanges() const
{
    return AddrRangeList{owner.addrRange};
}

Tick
SsdMemory::MemoryPort::recvAtomic(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();
    unsigned size = pkt->getSize();
    Addr offset = owner.toOffset(addr);

    if (pkt->isRead()) {
        pkt->setData(owner.pmem + offset);
    } else if (pkt->isWrite()) {
        pkt->writeData(owner.pmem + offset);
    }

    pkt->makeResponse();

    Addr ssdAddr = addr - owner.addrRange.start();
    Tick lat = owner.accessLatency(ssdAddr, size, pkt->isRead());

    DPRINTF(SsdMemory, "atomic %s addr=%#lx size=%u latency=%lu ps\n",
            pkt->isRead() ? "read" : "write", addr, size, lat);

    return lat;
}

void
SsdMemory::MemoryPort::recvFunctional(PacketPtr pkt)
{
    for (auto &pending : owner.pendingReqs) {
        if (pkt->trySatisfyFunctional(pending.pkt)) {
            pkt->makeResponse();
            return;
        }
    }
    if (owner.retryPkt && pkt->trySatisfyFunctional(owner.retryPkt)) {
        pkt->makeResponse();
        return;
    }

    Addr offset = owner.toOffset(pkt->getAddr());

    if (pkt->isRead()) {
        pkt->setData(owner.pmem + offset);
    } else if (pkt->isWrite()) {
        pkt->writeData(owner.pmem + offset);
    }

    pkt->makeResponse();
}

bool
SsdMemory::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    Addr addr = pkt->getAddr();
    unsigned size = pkt->getSize();
    Addr offset = owner.toOffset(addr);

    DPRINTF(SsdMemory, "timing %s addr=%#lx size=%u\n",
            pkt->isRead() ? "read" : "write", addr, size);

    // A2 fix: backpressure when queue is full
    if (owner.maxQueueDepth > 0 &&
        owner.pendingReqs.size() >= owner.maxQueueDepth) {
        DPRINTF(SsdMemory, "  queue full (%u/%u), returning false\n",
                (unsigned)owner.pendingReqs.size(), owner.maxQueueDepth);
        owner.needRetry = true;
        owner.stats.queueFullRetries++;
        return false;
    }

    if (pkt->cacheResponding()) {
        DPRINTF(SsdMemory, "  cache responding, not handling\n");
        return true;
    }

    if (pkt->cmd == MemCmd::CleanEvict ||
        pkt->cmd == MemCmd::WritebackClean) {
        DPRINTF(SsdMemory, "  clean evict, ignoring\n");
        if (pkt->needsResponse()) {
            owner.insertPending(pkt, curTick() + 1000);
            owner.scheduleNextCompletion();
        }
        return true;
    }

    // Data correctness
    if (pkt->isRead()) {
        pkt->setData(owner.pmem + offset);
    } else if (pkt->isWrite()) {
        pkt->writeData(owner.pmem + offset);
    }

    // Timing: coalescing → DRAM cache → SSD
    Addr ssdAddr = addr - owner.addrRange.start();
    Tick lat = owner.accessLatency(ssdAddr, size, pkt->isRead());

    // Stats
    if (pkt->isRead()) {
        owner.stats.numReads++;
        owner.stats.totalReadLatency += lat;
        owner.stats.readLatencyHist.sample(lat);
    } else {
        owner.stats.numWrites++;
        owner.stats.totalWriteLatency += lat;
        owner.stats.writeLatencyHist.sample(lat);
    }

    DPRINTF(SsdMemory, "  SSD latency=%lu ps (%lu us)\n",
            lat, lat / 1000000);

    if (pkt->needsResponse()) {
        Tick completionTick = curTick() + lat;
        owner.insertPending(pkt, completionTick);
        owner.scheduleNextCompletion();
    }

    return true;
}

void
SsdMemory::MemoryPort::recvRespRetry()
{
    if (owner.retryResp && owner.retryPkt) {
        DPRINTF(SsdMemory, "retrying response for addr=%#lx\n",
                owner.retryPkt->getAddr());
        if (sendTimingResp(owner.retryPkt)) {
            owner.retryResp = false;
            owner.retryPkt = nullptr;
            owner.scheduleNextCompletion();
        }
    }
}

// ====================================================================
//  Completion event processing
// ====================================================================

void
SsdMemory::processCompletion()
{
    if (retryResp) {
        return;
    }

    Tick now = curTick();

    while (!pendingReqs.empty()) {
        auto &front = pendingReqs.front();

        if (front.completionTick > now) {
            scheduleNextCompletion();
            return;
        }

        PacketPtr pkt = front.pkt;
        pendingReqs.pop_front();

        pkt->makeResponse();

        DPRINTF(SsdMemory, "completing %s addr=%#lx\n",
                pkt->isRead() ? "read" : "write", pkt->getAddr());

        if (!port.sendTimingResp(pkt)) {
            DPRINTF(SsdMemory, "  response blocked, waiting for retry\n");
            retryResp = true;
            retryPkt = pkt;
            return;
        }

        // A2 fix: a slot freed — if upstream was blocked, retry it.
        if (needRetry) {
            needRetry = false;
            port.sendRetryReq();
        }
    }
}

// ====================================================================
//  Statistics
// ====================================================================

SsdMemory::SsdStats::SsdStats(SsdMemory &parent)
    : statistics::Group(&parent),
      ADD_STAT(numReads, statistics::units::Count::get(),
               "Total read requests"),
      ADD_STAT(numWrites, statistics::units::Count::get(),
               "Total write requests"),
      ADD_STAT(totalReadLatency, statistics::units::Tick::get(),
               "Cumulative read latency (ps)"),
      ADD_STAT(totalWriteLatency, statistics::units::Tick::get(),
               "Cumulative write latency (ps)"),
      ADD_STAT(coalesceHits, statistics::units::Count::get(),
               "Page coalesce hits (same-page within window)"),
      ADD_STAT(cacheHits, statistics::units::Count::get(),
               "DRAM cache hits"),
      ADD_STAT(cacheMisses, statistics::units::Count::get(),
               "DRAM cache misses"),
      ADD_STAT(cacheEvictions, statistics::units::Count::get(),
               "DRAM cache evictions"),
      ADD_STAT(cacheDirtyEvictions, statistics::units::Count::get(),
               "DRAM cache dirty evictions (writeback to NAND)"),
      ADD_STAT(queueFullRetries, statistics::units::Count::get(),
               "Times recvTimingReq returned false due to full queue"),
      ADD_STAT(readLatencyHist, statistics::units::Tick::get(),
               "Read latency distribution"),
      ADD_STAT(writeLatencyHist, statistics::units::Tick::get(),
               "Write latency distribution"),
      ADD_STAT(avgReadLatency, statistics::units::Tick::get(),
               "Average read latency"),
      ADD_STAT(avgWriteLatency, statistics::units::Tick::get(),
               "Average write latency")
{
    readLatencyHist.init(100);
    writeLatencyHist.init(100);
    avgReadLatency = totalReadLatency / numReads;
    avgWriteLatency = totalWriteLatency / numWrites;
}

} // namespace gem5
