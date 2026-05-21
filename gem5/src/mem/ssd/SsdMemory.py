from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class SsdMemory(ClockedObject):
    """SSD-backed memory device using SimpleSSD for flash timing.

    Sits behind a host xbar as a memory controller replacement.
    Receives standard ReadReq/WriteReq packets, uses SimpleSSD's
    HIL->ICL->FTL->PAL stack to compute realistic NAND latency,
    and schedules responses accordingly.

    Data correctness is provided by a host-side backing store
    (calloc'd buffer).  SimpleSSD is used purely for timing.

    Optional features:
      - Page coalescing: multiple sub-page beats to the same 4KB page
        share one SSD page access instead of each paying full latency.
      - DRAM cache: an on-device DRAM buffer (like real CXL-SSDs have)
        that caches hot 4KB pages at DRAM speed (~50ns hit latency).
        Misses go to SimpleSSD NAND. Configurable size and eviction.
    """

    type = 'SsdMemory'
    cxx_header = 'mem/ssd/ssd_memory.hh'
    cxx_class = 'gem5::SsdMemory'

    port = ResponsePort("Port facing the host xbar")

    ssd_config = Param.String("src/mem/ssd/simplessd/config/sample.cfg",
        "Path to SimpleSSD configuration file (sample.cfg)")

    range = Param.AddrRange(AddrRange(0x800000000, size='16GiB'),
        "Address range served by this SSD. Default starts at 32GiB "
        "to avoid overlap with typical DDR5 at 0x0.")

    logical_page_size = Param.UInt32(16384,
        "Logical page size in bytes for addr-to-LPN conversion. "
        "Must match SimpleSSD config's PageSize in [pal] section. "
        "Default 16384 matches sample.cfg's PageSize=16384. "
        "A runtime assert in the constructor verifies consistency.")

    latency_multiplier = Param.UInt32(1,
        "Multiply SimpleSSD's returned latency by this factor. "
        "CXL-SSD-Sim used 10 (likely a unit bug). Start with 1 "
        "and validate against NAND specs in sample.cfg.")

    # ---- Page coalescing ----
    coalesce_window = Param.Latency('10us',
        "Time window for page coalescing. If a second access to the "
        "same 4KB page arrives within this window of the first, it "
        "gets the coalesced (fast) latency instead of a full SSD "
        "access. 0 = disabled. 10us is a good default (covers one "
        "SSD page read time so back-to-back beats to the same page "
        "are absorbed).")

    coalesce_latency = Param.Latency('100ns',
        "Latency returned for a coalesced (same-page) access. "
        "Models the internal buffer read time after the page has "
        "already been fetched. 100ns is conservative for on-device "
        "SRAM/register access.")

    # ---- DRAM cache ----
    dram_cache_size = Param.MemorySize('0B',
        "Size of on-device DRAM cache in bytes. 0 = disabled. "
        "Real CXL-SSDs use 256MB-2GB. For a 64MB SSD, 4-16MB is "
        "reasonable. The cache holds 4KB pages; capacity = size/4096 "
        "pages.")

    dram_cache_hit_latency = Param.Latency('50ns',
        "Latency for a DRAM cache hit. Models DDR4/5 on the SSD "
        "board. 50ns = conservative for on-board LPDDR4/5.")

    dram_cache_assoc = Param.UInt32(16,
        "Set associativity for DRAM cache. Higher = fewer conflict "
        "misses but more tag comparison overhead. 16 = typical.")

    dram_cache_write_policy = Param.String('writeback',
        "Write policy: 'writeback' (dirty pages written to NAND on "
        "eviction) or 'writethrough' (every write goes to NAND "
        "immediately, no dirty eviction).")

    dram_cache_warm_after = Param.UInt64(0,
        "Number of SSD accesses before the DRAM cache activates. "
        "During the first N accesses (e.g. ORAM write-init), all "
        "requests bypass the cache and go directly to NAND. After N "
        "accesses the cache starts cold — no init data is cached. "
        "0 = cache active from the start (no warm-up delay).")

    max_queue_depth = Param.UInt32(256,
        "Max outstanding requests before backpressure. Models the "
        "CXL endpoint's internal request buffer depth. When full, "
        "recvTimingReq returns false and the upstream xbar retries. "
        "Real CXL-SSD endpoints: 128-256. 0 = unlimited (dangerous "
        "under sustained NAND misses — queue grows to thousands).")

    system = Param.System(Parent.any, "System for requestor ID")