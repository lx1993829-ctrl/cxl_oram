/*
 * nvme_ssd_device.hh — PCIe NVMe SSD device for gem5
 *
 * Wraps SimpleSSD's NVMe::Controller as a gem5 DmaDevice.
 *   - PIO port: BAR0 register + doorbell MMIO from CPU
 *   - DMA port: bus-mastered SQE fetch, data transfer, CQE post
 *               through PCIe model to host DRAM
 *
 * Implements SimpleSSD::HIL::NVMe::Interface which provides:
 *   - DMAInterface (dmaRead/dmaWrite → gem5 DMA through PCIe)
 *   - updateInterrupt (polling mode: just set a flag)
 *   - getVendorID
 *
 * IMPORTANT: Both DmaDevice and SimpleSSD::DMAInterface define
 * dmaRead/dmaWrite. We override SimpleSSD's virtual methods
 * (which the NVMe controller calls), and inside them we call
 * gem5's DmaDevice::dmaRead/dmaWrite (which generate PCIe DMA
 * packets). The signatures are different so there's no ambiguity
 * at the call site, but we use explicit DmaDevice:: qualification
 * in the .cc to be safe.
 */

#ifndef __MEM_NVME_SSD_NVME_SSD_DEVICE_HH__
#define __MEM_NVME_SSD_NVME_SSD_DEVICE_HH__

#include "dev/dma_device.hh"
#include "mem/nvme_ssd/nvme_dram_cache.hh"
#include "mem/ssd/ssd_engine.hh"
#include "params/NvmeSsdDevice.hh"

// SimpleSSD headers
#include "hil/nvme/controller.hh"
#include "hil/nvme/interface.hh"

namespace gem5
{

class NvmeSsdDevice : public DmaDevice,
                      public SimpleSSD::HIL::NVMe::Interface
{
  public:
    PARAMS(NvmeSsdDevice);
    NvmeSsdDevice(const Params &p);
    ~NvmeSsdDevice();

    // --- gem5 PioDevice interface (BAR0 access) ---
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
    AddrRangeList getAddrRanges() const override;

    void init() override;

    // --- SimpleSSD DMAInterface ---
    // These override SimpleSSD::DMAInterface virtual methods.
    // The NVMe controller calls these to DMA to/from host DRAM.
    // Inside, we call gem5's DmaDevice::dmaRead/dmaWrite.
    void dmaRead(uint64_t addr, uint64_t size, uint8_t *buffer,
                 SimpleSSD::DMAFunction &func,
                 void *context = nullptr) override;
    void dmaWrite(uint64_t addr, uint64_t size, uint8_t *buffer,
                  SimpleSSD::DMAFunction &func,
                  void *context = nullptr) override;

    // --- SimpleSSD NVMe::Interface ---
    void updateInterrupt(uint16_t interruptVector, bool post) override;
    void getVendorID(uint16_t &vid, uint16_t &ssvid) override;

  private:
    // BAR0 address, size, and latency
    const Addr pioAddr;
    const Addr pioSize;
    const Tick pioDelay;
    // SimpleSSD NVMe controller
    SimpleSSD::HIL::NVMe::Controller *ctrl;

    // SimpleSSD engine (time bridge)
    SsdEngine engine;
    SimpleSSD::ConfigReader ssdConfig;

    // Interrupt flag (polling mode)
    bool interruptPending;

    // DMA callback bridge
    struct DmaCallbackData {
        SimpleSSD::DMAFunction func;
        void *context;
        uint64_t issueTick;
        // Recorded for diagnostics: what address and size we DMA'd
        // and where the bytes were placed.
        uint64_t addr;
        uint64_t size;
        uint8_t *buffer;
        // True if `buffer` was internally allocated (caller passed NULL);
        // dmaReadDone/dmaWriteDone must `delete[] buffer` after use.
        bool owns_buffer;
        // Sequence number for tracing concurrent DMAs.
        uint64_t seq;
    };

    void dmaReadDone(DmaCallbackData *data);
    void dmaWriteDone(DmaCallbackData *data);

    // ---- DRAM cache (parallels CXL SsdMemory's cache) ----
    // Active only when p.dram_cache_size > 0. When active, the
    // constructor installs `checkAllHitTrampoline` into SimpleSSD's
    // hil.cc via the function-pointer hook `nvmeCacheCheckAllHit`.
    // HIL::write/read calls the trampoline on every request; the
    // trampoline forwards to gActiveCache.
    NvmeDramCache *dramCache;

    // Static singleton + trampoline. Singleton because SimpleSSD's hook
    // is a single function pointer with no user_data slot. We support
    // exactly one active NvmeDramCache (enforced by the constructor —
    // a second NvmeSsdDevice with cache enabled would clobber the hook).
    static NvmeDramCache *gActiveCache;
    static bool checkAllHitTrampoline(uint64_t startPageNum,
                                      uint64_t nPages,
                                      bool isWrite,
                                      uint64_t *hitLatencyPs,
                                      uint64_t *evictWritebackPs);
};

} // namespace gem5

#endif // __MEM_NVME_SSD_NVME_SSD_DEVICE_HH__
