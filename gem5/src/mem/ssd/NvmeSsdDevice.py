from m5.objects.Device import DmaDevice
from m5.params import *
from m5.proxy import *


class NvmeSsdDevice(DmaDevice):
    """PCIe NVMe SSD device wrapping SimpleSSD's NVMe controller.

    Subclasses DmaDevice (PIO + DMA ports):
      - PIO port: CPU writes NVMe registers and doorbells (BAR0)
      - DMA port: SSD initiates bus-mastered transfers to/from host DRAM
        (SQE fetch, data transfer, CQE post) through the PCIe model.

    The NVMe controller, queue management, command parsing, PRP walking,
    and completion posting are all handled by SimpleSSD's HIL::NVMe code.
    This device just bridges SimpleSSD's DMAInterface to gem5's DmaDevice.
    """

    type = 'NvmeSsdDevice'
    cxx_header = 'mem/nvme_ssd/nvme_ssd_device.hh'
    cxx_class = 'gem5::NvmeSsdDevice'

    ssd_config = Param.String("src/mem/ssd/simplessd/config/sample.cfg",
        "Path to SimpleSSD configuration file")

    # BAR0 for NVMe registers + doorbells
    pio_addr = Param.Addr(0xF0000000, "BAR0 base address")
    pio_size = Param.Addr(0x10000, "BAR0 size (64kB)")
    pio_latency = Param.Latency('1ns', "PIO register access latency")

    # ---- DRAM cache (matches SsdMemory's CXL DRAM cache) ----
    # When enabled, NvmeSsdDevice installs a hook in SimpleSSD's
    # HIL::write/read that clamps completion latency to dram_cache_hit_latency
    # for fully-cached LBA ranges. Cache state, hit/miss/eviction
    # accounting, and warm-up gating are all in NvmeDramCache.
    # See src/mem/nvme_ssd/nvme_dram_cache.hh.
    dram_cache_size = Param.MemorySize('0B',
        "Size of on-device DRAM cache in bytes. 0 = disabled. "
        "Match the CXL SsdMemory's dram_cache_size for fair comparison.")

    dram_cache_hit_latency = Param.Latency('50ns',
        "Latency for a DRAM cache hit. Models DDR4/5 on the SSD "
        "board. Match the CXL value for fair comparison.")

    dram_cache_assoc = Param.UInt32(16,
        "Set associativity for DRAM cache.")

    dram_cache_write_policy = Param.String('writeback',
        "Write policy: 'writeback' (dirty pages written to NAND on "
        "eviction) or 'writethrough'.")

    dram_cache_warm_after = Param.UInt64(0,
        "Number of accesses before the DRAM cache activates. "
        "0 = active from the start.")
