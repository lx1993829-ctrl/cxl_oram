/*
 * nvme_ssd_device.cc — PCIe NVMe SSD device for gem5
 *
 * DIAGNOSTIC BUILD: heavy DPRINTFs at every entry/exit + null guards
 * to pinpoint the segfault during first I/O.
 */

#include "mem/nvme_ssd/nvme_ssd_device.hh"

#include <cstdio>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/SsdMemory.hh"
#include "sim/cur_tick.hh"
#include "sim/system.hh"

/* SimpleSSD's hil.cc defines this function pointer (default null).
 * We extern-declare it here so the constructor can install our
 * trampoline when dram_cache_size > 0. */
namespace SimpleSSD {
extern bool (*nvmeCacheCheckAllHit)(uint64_t startPageNum,
                                     uint64_t nPages,
                                     bool isWrite,
                                     uint64_t *hitLatencyPs,
                                     uint64_t *evictWritebackPs);
}

namespace gem5
{

// ---- Static singleton for the cache hook (one active cache at a time).
NvmeDramCache *NvmeSsdDevice::gActiveCache = nullptr;

bool
NvmeSsdDevice::checkAllHitTrampoline(uint64_t startPageNum, uint64_t nPages,
                                      bool isWrite, uint64_t *hitLatencyPs,
                                      uint64_t *evictWritebackPs)
{
    if (!gActiveCache) {
        *hitLatencyPs = 0;
        *evictWritebackPs = 0;
        return false;
    }
    bool hit = gActiveCache->checkAllHit(startPageNum, nPages, isWrite,
                                          evictWritebackPs);
    *hitLatencyPs = gActiveCache->getHitLatencyPs();
    return hit;
}

// ====================================================================
//  Constructor / Destructor
// ====================================================================

NvmeSsdDevice::NvmeSsdDevice(const Params &p)
    : DmaDevice(p),
      pioAddr(p.pio_addr),
      pioSize(p.pio_size),
      pioDelay(p.pio_latency),
      ctrl(nullptr),
      engine(),
      interruptPending(false),
      dramCache(nullptr)
{
    // FIX #1: hook gem5 event system FIRST, before SimpleSSD allocates
    // or schedules events during initCPU(). If we wait, any
    // scheduleEvent call during init silently no-ops because
    // eventManager==nullptr.
    engine.setEventManager(this);

    // Now init SimpleSSD (parses config, creates CPU model, etc.)
    ssdConfig = initSimpleSSD(&engine, p.ssd_config);

    // Create the NVMe controller. 'this' implements
    // SimpleSSD::HIL::NVMe::Interface for DMA + interrupts.
    ctrl = new SimpleSSD::HIL::NVMe::Controller(this, ssdConfig);

    // ---- DRAM cache hook installation ----
    // Mirrors CXL SsdMemory's DRAM cache: page-keyed set-associative
    // LRU with optional warm-up gating. The cache lives in this
    // device; SimpleSSD's HIL queries it via a function-pointer hook
    // (declared in hil.cc). Default off — only activates when the
    // Python config sets dram_cache_size > 0.
    if (p.dram_cache_size > 0) {
        if (gActiveCache) {
            warn("NvmeSsdDevice: a second instance with dram_cache>0 "
                 "would clobber the SimpleSSD hook; only one cache "
                 "is supported. Cache will be disabled for this "
                 "instance.");
        } else {
            // SimpleSSD's logical page size: 16 KB by default for the
            // sample config. Must match what HIL passes as range.nlp
            // (number of pages). The cache is page-granular regardless.
            const uint64_t logicalPageSize = 16384;
            // Latency params are in picoseconds (gem5 Tick = ps).
            dramCache = new NvmeDramCache(
                p.dram_cache_size,
                p.dram_cache_assoc,
                logicalPageSize,
                p.dram_cache_write_policy == "writeback",
                p.dram_cache_warm_after,
                p.dram_cache_hit_latency
            );
            gActiveCache = dramCache;
            SimpleSSD::nvmeCacheCheckAllHit = &checkAllHitTrampoline;
            inform("NvmeSsdDevice: DRAM cache active (size=%lu B "
                   "assoc=%u policy=%s warm_after=%lu hit_lat=%lu ps)",
                   (unsigned long)p.dram_cache_size,
                   p.dram_cache_assoc,
                   p.dram_cache_write_policy.c_str(),
                   (unsigned long)p.dram_cache_warm_after,
                   (unsigned long)p.dram_cache_hit_latency);
        }
    }

    inform("NvmeSsdDevice: initialized, BAR0=%#lx size=%#lx config=%s",
           pioAddr, pioSize, p.ssd_config);
}

NvmeSsdDevice::~NvmeSsdDevice()
{
    // Tear down the SimpleSSD hook if we installed it. Safe even if
    // we never installed (gActiveCache == nullptr).
    if (gActiveCache == dramCache) {
        gActiveCache = nullptr;
        SimpleSSD::nvmeCacheCheckAllHit = nullptr;
    }
    delete dramCache;
    delete ctrl;
    inform("NvmeSsdDevice: destroyed");
}

void
NvmeSsdDevice::init()
{
    DmaDevice::init();
}

// ====================================================================
//  PIO (BAR0 register + doorbell access from CPU)
// ====================================================================

AddrRangeList
NvmeSsdDevice::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

Tick
NvmeSsdDevice::read(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - pioAddr;
    unsigned size = pkt->getSize();
    uint64_t delay = 0;

    DPRINTF(SsdMemory, "[PIO_READ enter] offset=%#lx size=%u\n",
            offset, size);

    if (!ctrl) {
        panic("NvmeSsdDevice::read: ctrl is null!");
    }

    ctrl->readRegister(offset, size, pkt->getPtr<uint8_t>(), delay);

    pkt->makeResponse();
    DPRINTF(SsdMemory, "[PIO_READ exit] offset=%#lx\n", offset);
    return pioDelay;
}

Tick
NvmeSsdDevice::write(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - pioAddr;
    unsigned size = pkt->getSize();
    uint64_t delay = 0;

    DPRINTF(SsdMemory, "[PIO_WRITE enter] offset=%#lx size=%u\n",
            offset, size);

    if (!ctrl) {
        panic("NvmeSsdDevice::write: ctrl is null!");
    }

    if (offset >= 0x1000) {
        // Doorbell region.
        // NVMe spec doorbell layout (DSTRD=0, stride=4 bytes):
        //   SQ y tail = 0x1000 + (2y) * 4
        //   CQ y head = 0x1000 + (2y+1) * 4
        uint32_t dbell_offset = offset - 0x1000;
        uint16_t qid = dbell_offset / 8;
        bool isSQ = ((dbell_offset / 4) % 2) == 0;
        uint32_t val;
        memcpy(&val, pkt->getConstPtr<uint8_t>(), sizeof(uint32_t));

        if (isSQ) {
            DPRINTF(SsdMemory, "  [DOORBELL_SQ] qid=%u tail=%u\n",
                    qid, val);
            ctrl->ringSQTailDoorbell(qid, (uint16_t)val, delay);
            DPRINTF(SsdMemory, "  [DOORBELL_SQ done] qid=%u\n", qid);
        } else {
            DPRINTF(SsdMemory, "  [DOORBELL_CQ] qid=%u head=%u\n",
                    qid, val);
            ctrl->ringCQHeadDoorbell(qid, (uint16_t)val, delay);
            DPRINTF(SsdMemory, "  [DOORBELL_CQ done] qid=%u\n", qid);
        }
    } else {
        // Controller config registers (CC, AQA, ASQ, ACQ, etc.)
        DPRINTF(SsdMemory, "  [REG_WRITE] offset=%#lx\n", offset);
        ctrl->writeRegister(offset, size,
                            const_cast<uint8_t*>(
                                pkt->getConstPtr<uint8_t>()),
                            delay);
        DPRINTF(SsdMemory, "  [REG_WRITE done] offset=%#lx\n", offset);
    }

    pkt->makeResponse();
    DPRINTF(SsdMemory, "[PIO_WRITE exit] offset=%#lx\n", offset);
    return pioDelay;
}

// ====================================================================
//  DMA interface
// ====================================================================

void
NvmeSsdDevice::dmaRead(uint64_t addr, uint64_t size, uint8_t *buffer,
                        SimpleSSD::DMAFunction &func, void *context)
{
    /* Sequential tag so we can correlate enter/done across many
     * concurrent DMAs. atomic-static so unique even across instances. */
    static uint64_t dma_seq = 0;
    uint64_t my_seq = ++dma_seq;

    fprintf(stderr,
            "|||DMA_RD_ENTER seq=%lu tick=%lu addr=%#lx size=%lu "
            "buf=%p%s ctx=%p\n",
            (unsigned long)my_seq, (unsigned long)curTick(),
            (unsigned long)addr, (unsigned long)size,
            (void*)buffer, buffer ? "" : "(NULL→alloc)", context);

    if (size == 0) {
        fprintf(stderr,
                "|||DMA_RD_SHORTCUT seq=%lu (size==0, calling cb sync)\n",
                (unsigned long)my_seq);
        func(curTick(), context);
        return;
    }
    if (size > (1ULL << 30)) {
        panic("NvmeSsdDevice::dmaRead: size=%lu too large", size);
    }

    /* SimpleSSD's HIL/NVMe dma.cc passes a NULL buffer in some paths.
     * Allocate one here; freed in dmaReadDone. */
    bool we_own_buffer = false;
    if (!buffer) {
        buffer = new uint8_t[size];
        we_own_buffer = true;
    }

    auto *cbd = new DmaCallbackData;
    cbd->func = func;
    cbd->context = context;
    cbd->issueTick = curTick();
    cbd->addr = addr;
    cbd->size = size;
    cbd->buffer = buffer;
    cbd->owns_buffer = we_own_buffer;
    cbd->seq = my_seq;

    auto *event = new EventFunctionWrapper(
        [this, cbd]{ dmaReadDone(cbd); },
        name(), true);

    DmaDevice::dmaRead(addr, (int)size, event, buffer, 0);

    fprintf(stderr,
            "|||DMA_RD_SUBMIT seq=%lu (gem5 DmaDevice::dmaRead issued)\n",
            (unsigned long)my_seq);
}

void
NvmeSsdDevice::dmaWrite(uint64_t addr, uint64_t size, uint8_t *buffer,
                         SimpleSSD::DMAFunction &func, void *context)
{
    static uint64_t dma_seq = 0;
    uint64_t my_seq = ++dma_seq;

    fprintf(stderr,
            "|||DMA_WR_ENTER seq=%lu tick=%lu addr=%#lx size=%lu "
            "buf=%p%s ctx=%p\n",
            (unsigned long)my_seq, (unsigned long)curTick(),
            (unsigned long)addr, (unsigned long)size,
            (void*)buffer, buffer ? "" : "(NULL→alloc)", context);

    // For all writes log the addr+size; small writes dump bytes,
    // large dump preview.
    if (buffer) {
        fprintf(stderr,
                "|||NVME_DEV_WRITE seq=%lu addr=%#lx size=%lu bytes:",
                (unsigned long)my_seq, (unsigned long)addr, (unsigned long)size);
        uint64_t preview = (size < 16) ? size : 16;
        for (uint64_t i = 0; i < preview; i++) {
            fprintf(stderr, " %02x", (unsigned)buffer[i]);
        }
        if (size > preview) fprintf(stderr, " ... (%lu more)", (unsigned long)(size - preview));
        fprintf(stderr, "\n");
    }

    if (size == 0) {
        fprintf(stderr,
                "|||DMA_WR_SHORTCUT seq=%lu (size==0, calling cb sync)\n",
                (unsigned long)my_seq);
        func(curTick(), context);
        return;
    }
    if (size > (1ULL << 30)) {
        panic("NvmeSsdDevice::dmaWrite: size=%lu too large", size);
    }

    bool we_own_buffer = false;
    if (!buffer) {
        buffer = new uint8_t[size]();
        we_own_buffer = true;
    }

    auto *cbd = new DmaCallbackData;
    cbd->func = func;
    cbd->context = context;
    cbd->issueTick = curTick();
    cbd->addr = addr;
    cbd->size = size;
    cbd->buffer = buffer;
    cbd->owns_buffer = we_own_buffer;
    cbd->seq = my_seq;

    auto *event = new EventFunctionWrapper(
        [this, cbd]{ dmaWriteDone(cbd); },
        name(), true);

    /* Pad sub-burst writes to a full 64-byte burst.
     *
     * The interleaved DDR5 MemCtrl's write-queue subsumption check
     * (addToReadQueue, line 227 of mem_ctrl.cc) compares de-interleaved
     * MemPacket addresses: p->addr <= addr && (addr+size) <= (p->addr+p->size).
     * For sub-burst writes (e.g. 16-byte CQE), the de-interleaved addr/size
     * may not subsume a later read to a different offset in the same burst,
     * causing the read to miss the write queue and return stale DRAM data.
     *
     * Padding to a full burst guarantees the write covers the entire
     * burst-aligned block, so any read within the block is subsumed.
     * We first read the current burst contents (via DmaDevice::dmaRead
     * would be complex), so instead we just zero-pad the surrounding
     * bytes. The CQE is the only meaningful data; the padding bytes
     * overwrite whatever was there, but since the CPU zeroed the entire
     * IOCQ at init and only reads the 16-byte CQE entry, this is safe.
     */
    static const uint64_t BURST_SIZE = 64;
    uint8_t *dma_buf = buffer;
    Addr     dma_addr = addr;
    uint64_t dma_size = size;

    uint8_t *padded_buf = nullptr;
    if (size < BURST_SIZE) {
        Addr aligned_addr = addr & ~(BURST_SIZE - 1);
        uint64_t offset_in_burst = addr - aligned_addr;

        padded_buf = new uint8_t[BURST_SIZE]();  // zero-filled
        memcpy(padded_buf + offset_in_burst, buffer, size);

        dma_buf  = padded_buf;
        dma_addr = aligned_addr;
        dma_size = BURST_SIZE;

        fprintf(stderr,
                "|||DMA_WR_PAD seq=%lu orig_addr=%#lx orig_size=%lu "
                "padded_addr=%#lx padded_size=%lu\n",
                (unsigned long)my_seq, (unsigned long)addr,
                (unsigned long)size, (unsigned long)aligned_addr,
                (unsigned long)BURST_SIZE);
    }

    DmaDevice::dmaWrite(dma_addr, (int)dma_size, event, dma_buf, 0);

    /* padded_buf is intentionally not freed here. DmaDevice::dmaWrite
     * may reference the buffer asynchronously until the DMA completes.
     * The 64-byte allocation per sub-burst write is negligible. */

    fprintf(stderr,
            "|||DMA_WR_SUBMIT seq=%lu (gem5 DmaDevice::dmaWrite issued)\n",
            (unsigned long)my_seq);
}

void
NvmeSsdDevice::dmaReadDone(DmaCallbackData *data)
{
    fprintf(stderr,
            "|||DMA_RD_DONE_ENTER seq=%lu tick=%lu latency=%lu ctx=%p\n",
            (unsigned long)data->seq, (unsigned long)curTick(),
            (unsigned long)(curTick() - data->issueTick), data->context);

    // Log first bytes returned (preview only for large)
    if (data->buffer) {
        fprintf(stderr,
                "|||NVME_DEV_READ_DONE seq=%lu addr=%#lx size=%lu bytes:",
                (unsigned long)data->seq,
                (unsigned long)data->addr, (unsigned long)data->size);
        uint64_t preview = (data->size < 64) ? data->size : 64;
        for (uint64_t i = 0; i < preview; i++) {
            fprintf(stderr, " %02x", (unsigned)data->buffer[i]);
        }
        if (data->size > preview)
            fprintf(stderr, " ... (%lu more)",
                    (unsigned long)(data->size - preview));
        fprintf(stderr, "\n");
    }

    fprintf(stderr,
            "|||DMA_RD_DONE_CALLING_CB seq=%lu ctx=%p\n",
            (unsigned long)data->seq, data->context);

    data->func(curTick(), data->context);

    fprintf(stderr,
            "|||DMA_RD_DONE_CB_RETURNED seq=%lu\n",
            (unsigned long)data->seq);

    if (data->owns_buffer) {
        delete[] data->buffer;
    }

    fprintf(stderr,
            "|||DMA_RD_DONE_EXIT seq=%lu\n",
            (unsigned long)data->seq);

    delete data;
}

void
NvmeSsdDevice::dmaWriteDone(DmaCallbackData *data)
{
    fprintf(stderr,
            "|||DMA_WR_DONE_ENTER seq=%lu tick=%lu latency=%lu ctx=%p\n",
            (unsigned long)data->seq, (unsigned long)curTick(),
            (unsigned long)(curTick() - data->issueTick), data->context);

    fprintf(stderr,
            "|||DMA_WR_DONE_CALLING_CB seq=%lu ctx=%p\n",
            (unsigned long)data->seq, data->context);

    data->func(curTick(), data->context);

    fprintf(stderr,
            "|||DMA_WR_DONE_CB_RETURNED seq=%lu\n",
            (unsigned long)data->seq);

    if (data->owns_buffer) {
        delete[] data->buffer;
    }

    fprintf(stderr,
            "|||DMA_WR_DONE_EXIT seq=%lu\n",
            (unsigned long)data->seq);

    delete data;
}

// ====================================================================
//  Interrupt
// ====================================================================

void
NvmeSsdDevice::updateInterrupt(uint16_t vector, bool post)
{
    DPRINTF(SsdMemory, "[IRQ] vector=%u post=%d\n", vector, post);
    interruptPending = post;
}

void
NvmeSsdDevice::getVendorID(uint16_t &vid, uint16_t &ssvid)
{
    vid = 0x8086;
    ssvid = 0x8086;
}

} // namespace gem5