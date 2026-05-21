from m5.params import *
from m5.proxy import *
from m5.objects.ClockedObject import ClockedObject


class OramDevice(ClockedObject):
    """Secure ORAM device wrapping a Verilated RTL model.

    RTL limits: N=16380 slots, B=4095 buckets, Z=8 slots/bucket.
    Slots permanently split: lower X% → HBM, upper (100-X)% → host.

    Per-operation routing: when a client request targets a host slot,
    ALL AXI transactions for that ORAM operation (bucket read, stash,
    bucket write) go through pcie_port to host DDR5.
    Stash always goes to HBM regardless.

    Flow:
      1. Init pos_map + bucket_meta
      2. Grant lease
      3. Write-init all slots (each to its assigned memory)
      4. Run num_ops random read/write operations
    """

    type = "OramDevice"
    cxx_header = "oram/oram_device.hh"
    cxx_class = "gem5::OramDevice"

    hbm_port = RequestPort("Direct to HBM controllers")
    pcie_port = RequestPort("To PCIe bridge -> host DDR5")

    oram_freq = Param.Frequency("300MHz", "ORAM engine clock frequency")
    local_pct = Param.UInt32(50, "Percent of slots in local HBM")
    num_slots = Param.UInt32(32, "ORAM slots (max 16380)")
    num_ops = Param.UInt32(100, "Test operations to run")

    aes_key_0 = Param.UInt32(0x01234567, "AES key word 0")
    aes_key_1 = Param.UInt32(0x89ABCDEF, "AES key word 1")
    aes_key_2 = Param.UInt32(0xFEDCBA98, "AES key word 2")
    aes_key_3 = Param.UInt32(0x76543210, "AES key word 3")

    hbm_base = Param.Addr(0x0, "HBM base in gem5 address map")
    host_base = Param.Addr(0x400000000, "Host DDR5 base")
    stash_offset = Param.Addr(0x08000000, "Stash region offset")

    # ------------------------------------------------------------------
    # Phase 1 (CPU-driven mode) additions. When cpu_driven=False (default),
    # the device behaves exactly as before: internal RNG generates ops,
    # single lease granted to all slots.
    #
    # When cpu_driven=True:
    #   - Multiple leases (num_logical_clients) are granted at startup,
    #     each covering a disjoint slot range.
    #   - Internal RNG op generation is disabled; ops arrive via MMIO
    #     writes to cmd_port.
    #   - On op completion, a 64B result packet is written through
    #     pcie_port to result_buf_base + op_idx * 64.
    # ------------------------------------------------------------------
    cpu_driven = Param.Bool(False,
        "Enable CPU-driven mode: MMIO commands via cmd_port + result buffer "
        "writeback. When False, use internal RNG-driven op generation.")

    num_logical_clients = Param.UInt32(2,
        "Number of logical clients with independent leases (cpu_driven only). "
        "Each gets a disjoint slot range.")

    cmd_port = ResponsePort(
        "MMIO command port (cpu_driven mode): CPU writes mgmt/client "
        "commands; reads status and result registers.")

    cmd_base = Param.Addr(0x10000000,
        "Base physical address of the cmd_port MMIO region (256 bytes).")

    result_buf_base = Param.Addr(0x480000000,
        "Base physical address of the result buffer in host DDR5. "
        "One 64B cache line per op: {token, lease_id, op_idx, rdata[0..7]}.")

    result_buf_size = Param.UInt64(0x100000,
        "Size of the result buffer in bytes. Must be >= max_ops * 64. "
        "Default 1 MiB (0x100000) = 16384 entries — comfortable headroom "
        "for 10K ops.")

    # ------------------------------------------------------------------
    # Step 8: K-deep command queue inside OramDevice.
    #
    # cmd_queue_depth=1 reproduces Step 4-7 single-op MMIO behavior
    # (regression-safe). cmd_queue_depth>1 lets multiple ops be in
    # various phases of the lifecycle simultaneously: the head op is
    # IN_PROGRESS in the RTL, and PENDING ops queue behind it ready
    # to dispatch as soon as the RTL becomes idle.
    # ------------------------------------------------------------------
    cmd_queue_depth = Param.UInt32(1,
        "Internal command queue capacity. 1 = single in-flight (Step 4-7 "
        "behavior, doorbell rejected if queue full). >1 = multiple ops "
        "may be queued; doorbell only rejected when queue is full.")

    # ------------------------------------------------------------------
    # Step 9: memory-mapped command ring.
    #
    # When cmd_ring_base != 0, ORAM watches for an MMIO doorbell write
    # at cmd_base + 0x150 indicating the CPU has produced new ring
    # entries. ORAM then issues fabric reads of:
    #   1. The prod_idx field at cmd_ring_base + 0x00
    #   2. Each new ring entry at cmd_ring_base + 0x100 + slot * 64
    # Fetched entries are pushed into the internal cmdQueue.
    # ORAM writes its cons_idx back to cmd_ring_base + 0x40 so the CPU
    # sees backpressure relief.
    #
    # When cmd_ring_base == 0 (default), the ring path is disabled and
    # commands are accepted only via direct MMIO writes (Step 4-8).
    # ------------------------------------------------------------------
    cmd_ring_base = Param.Addr(0x0,
        "Base address of the command ring in fabric memory. 0 disables "
        "the ring path (Step 4-8 MMIO-only mode).")

    cmd_ring_depth = Param.UInt32(16,
        "Number of slots in the command ring. Each slot is 64 bytes. "
        "CPU's slot index = (op_seq % cmd_ring_depth). Backpressure "
        "kicks in when (op_seq - cons_idx) >= cmd_ring_depth.")

    # ------------------------------------------------------------------
    # Step 4: gated start — opt-in mechanism to hold ORAM in the
    # post-reset state without ticking until the binary explicitly
    # releases it via MMIO write to cmd_base + 0x180.
    #
    # Why: when ORAM is part of a setup that does extensive non-ORAM
    # work first (e.g. NVMe controller init, queue creation, LBA
    # seeding), having Verilator tick the ORAM RTL every simulated
    # cycle is wasteful — adds ~80x wallclock overhead. With
    # gated_start=True ORAM sleeps during that setup phase. Binary
    # writes 1 to cmd_base + 0x180 when ready for ORAM to come up,
    # then polls cmd_base + 0x184 for status==2 (ready for ops).
    #
    # Default False — existing tests/configs unchanged.
    # ------------------------------------------------------------------
    gated_start = Param.Bool(False,
        "Hold ORAM in post-reset state (no Verilator eval per cycle) "
        "until binary writes 1 to cmd_base + 0x180. Status readable at "
        "cmd_base + 0x184. Default False = auto-start as before.")

    system = Param.System(Parent.any, "System for requestor ID")
