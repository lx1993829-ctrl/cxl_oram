from m5.objects.Device import DmaDevice
from m5.params import *
from m5.proxy import *


class NvmeSsdDevice(DmaDevice):
    """PCIe NVMe SSD device wrapping SimpleSSD's NVMe controller.

    One shared device serves all ORAM instances via LBA partitioning.
    Instance i uses LBAs [i * max_lba, (i+1) * max_lba).

    Wiring (from config script):
      CPU -> membus -> iobridge -> iobus -> nvme_ssd.pio  (BAR0)
      nvme_ssd.dma -> system.pcie -> cxl_host_xbar -> DDR5

    Multi-instance queue model:
      - One admin SQ/CQ pair for controller init (shared)
      - N I/O SQ/CQ pairs: qid = instance_id + 1
      - Instance 0's process creates all queue pairs during init
      - All queue/data buffers live in NVME_SHARED region inside DDR5

    Address placement (must fit phase_d_layout.py gaps):
      BAR0:  0xF0000000  (gap between ORAM_CMD_BASE and HBM_BASE)
      QREGION: DDR_AGG_BASE + 0x020000000 = 0x620000000
               (gap between result_buf end and DDR slab start)

    SSD capacity:
      Determined by SimpleSSD config (sample.cfg NAND params), not by
      any gem5 parameter. For N instances × 512 MB each, the NAND
      must hold >= N × 512 MB. Ensure [pal] Channel × Package × Die ×
      Plane × Block × Page × PageSize >= total bytes.
    """

    type = 'NvmeSsdDevice'
    cxx_header = 'mem/nvme_ssd/nvme_ssd_device.hh'
    cxx_class = 'gem5::NvmeSsdDevice'

    ssd_config = Param.String("src/mem/ssd/simplessd/config/sample.cfg",
        "Path to SimpleSSD configuration file")

    # BAR0: NVMe registers + doorbells. 0xF0000000 sits in the
    # [ORAM_CMD_BASE, HBM_BASE) gap per phase_d_layout.py.
    pio_addr = Param.Addr(0xF0000000,
        "BAR0 base address. Single BAR0 for the shared controller.")
    pio_size = Param.Addr(0x10000, "BAR0 size (64 KB)")
    pio_latency = Param.Latency('1ns', "PIO register access latency")

    # ---- DRAM cache (matches SsdMemory's CXL DRAM cache) ----
    dram_cache_size = Param.MemorySize('0B',
        "Size of on-device DRAM cache. 0 = disabled.")
    dram_cache_hit_latency = Param.Latency('50ns',
        "DRAM cache hit latency (ps).")
    dram_cache_assoc = Param.UInt32(16,
        "Set associativity for DRAM cache.")
    dram_cache_write_policy = Param.String('writeback',
        "Write policy: 'writeback' or 'writethrough'.")
    dram_cache_warm_after = Param.UInt64(0,
        "Accesses before DRAM cache activates. 0 = immediate.")