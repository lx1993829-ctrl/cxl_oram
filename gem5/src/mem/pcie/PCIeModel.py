from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class PCIeModel(ClockedObject):
    """TLP-level pipelined PCIe Gen3/4/5 model with credit-based flow control.

    Real TLP byte arrays, PCIe core assembly pipeline, DW alignment
    padding, completion reordering, LCRC/framing/ECRC overhead accounting,
    ACK/NAK DLLP amortization, deferred credit return, and AXI bridge
    pipeline."""

    type = 'PCIeModel'
    cxx_header = 'mem/pcie/pcie_model.hh'
    cxx_class = 'gem5::PCIeModel'

    # ---- Ports ----
    device_side_port = VectorResponsePort("Ports facing FPGA/ORAM devices")
    host_side_port = VectorRequestPort("Ports facing host memory (DDR5 via xbar)")

    # ---- PCIe Link ----
    gen = Param.UInt32(5, "PCIe generation: 3, 4, or 5")
    lanes = Param.UInt32(16, "Number of PCIe lanes")

    # ---- TLP ----
    mps = Param.UInt32(256, "Max Payload Size bytes (128, 256, 512)")
    mrrs = Param.UInt32(512, "Max Read Request Size bytes (128-4096)")
    max_tags = Param.UInt32(256, "Max outstanding non-posted read tags")
    rcb = Param.UInt32(128, "Read Completion Boundary bytes (64 or 128)")

    # ---- Latency ----
    read_latency = Param.Latency('300ns', "Informational, not in timing path")
    write_latency = Param.Latency('150ns', "Informational, not in timing path")
    rc_latency = Param.Latency('150ns',
        "Root complex + CPU interconnect traversal. Applied both "
        "directions. From Krishnaps validated gem5 PCIe model.")
    fpga_clock_period = Param.Latency('3333ps',
        "PCIe Bug #7 fix: FPGA bridge clock period for CDC throughput "
        "and cycle-count reporting. Default 3333ps = 300MHz matches "
        "oram_freq default. Must match the Python config's fpga_clock "
        "if it differs from 300MHz, otherwise cycle counts in "
        "write-commit inform output will be miscomputed.")
    rc_throughput_delay = Param.Latency('5ns',
        "Per-response RC pipeline throughput cost when warm. "
        "Each response physically traverses the RC stages. "
        "5ns = ~5 stages at 1GHz.")
    burst_window = Param.Latency('10ns',
        "Burst window for RC optimization. Packets arriving within "
        "this window share one RC traversal. Bias #6 fix: increased "
        "from 5ns to match CXL's 10ns (both previously asymmetric at "
        "20ns CXL vs 5ns PCIe). Both protocols share the same physical "
        "RC pipeline depth, so windows should be equal.")

    # ---- Data Link Layer (PCIe spec, exact) ----
    lcrc_bytes = Param.UInt32(4, "LCRC per TLP (PCIe spec)")
    framing_bytes = Param.UInt32(2, "STP+END framing per TLP (Gen3+)")
    ecrc_bytes = Param.UInt32(0, "ECRC (0=disabled, 4=enabled)")
    dllp_ack_delay = Param.Latency('2ns',
        "ACK/NAK DLLP round-trip per TLP. Est: 1-5ns.")
    credit_return_delay = Param.Latency('20ns',
        "DLLP credit update traversal. Est: 10-30ns.")

    # ---- AXI Bridge Pipeline ----
    bridge_pipeline_delay = Param.Latency('15ns',
        "AXI clock domain crossing (300MHz to PCIe core clock). "
        "Xilinx model uses 0ns. Est: 10-20ns conservative.")

    # ---- PCIe Core Assembly Pipeline ----
    # These model the FPGA PCIe hard block's internal datapath.
    # The hard block assembles TLP bytes from AXI beats at its
    # core clock rate. Per-TLP assembly time = ceil(totalBytes /
    # datapath_width) * core_period.
    pcie_core_clock = Param.Latency('1ns',
        "PCIe hard block core clock PERIOD (not frequency). Drives "
        "assembly rate: assemblyDelay = totalBytes * core_period / "
        "datapath_width. Default 1ns (1 GHz) × 64B = 64 GB/s, which "
        "matches Gen5 x16 wire rate (~63 GB/s effective) so neither "
        "stage bottlenecks the other. Use 2ns (500 MHz = 32 GB/s) "
        "for narrower links (Gen5 x8, Gen4 x16) where that rate "
        "matches the wire.")
    pcie_core_width = Param.UInt32(64,
        "PCIe hard block internal datapath width in bytes. "
        "Gen3: 256-bit=32B, Gen5: 512-bit=64B. From Xilinx UG583.")

    # ---- Completion Reordering ----
    completion_reorder_depth = Param.UInt32(256,
        "Max completions the root complex can hold for reordering. "
        "Must be >= max_tags to prevent overflow. "
        "Typical Intel/AMD root complex: 16-64.")

    # ---- Backpressure ----
    max_outstanding = Param.UInt32(32,
        "Max outstanding requests (read or write) before backpressure. "
        "Models AXI-PCIe bridge internal buffer depth. "
        "0 = disabled (gem5 MemCtrl provides natural backpressure). "
        "32 = conservative. 128-256 = typical bridge FIFO.")
    max_outstanding_writes = Param.UInt32(128,
        "Max outstanding posted writes before backpressure. "
        "Models AXI-PCIe bridge posted write buffer depth. "
        "Separate from read maxOutstanding because posted writes "
        "don't consume tags. 128 = typical Xilinx bridge FIFO.")

    # ---- Endpoint Identity ----
    requester_id = Param.UInt16(0x0100,
        "PCIe requester ID (bus:dev.func) for this FPGA endpoint. "
        "Default: bus=1, dev=0, func=0.")
    completer_id = Param.UInt16(0x0000,
        "PCIe completer ID for root complex. "
        "Default: bus=0, dev=0, func=0.")

    # ---- Flow Control Credits (per-port) ----
    # Realistic values based on Intel/AMD root complex advertisements.
    # PD credits: 1 credit = 4 DW = 16 bytes (PCIe spec §2.6).
    # A 256B MWr TLP needs 16 PD credits. PD=128 → 8 TLPs before stall.
    credits_ph = Param.Int32(32, "Posted header credits (per port)")
    credits_pd = Param.Int32(128, "Posted data credits (per port, 1 credit = 16B)")
    credits_nph = Param.Int32(32, "Non-posted header credits (per port)")
    credits_npd = Param.Int32(0, "Non-posted data credits")
    credits_cplh = Param.Int32(64, "Completion header credits (per port)")
    credits_cpld = Param.Int32(256, "Completion data credits (per port)")

    # ---- Host Injection Pacing ----
    host_inject_interval = Param.Latency('1ns',
        "Minimum interval between successive host xbar injections. "
        "Models RC-to-xbar forwarding rate. 0 = send all in one tick "
        "(let xbar backpressure). 1ns = 1GHz host clock. "
        "2ns = match typical xbar layer occupancy.")

    # ---- Review item #1: Read Reorder Buffer ----
    rrb_depth = Param.UInt32(128,
        "AXI-side Read Reorder Buffer (RRB) depth. Models the internal "
        "completion buffer in the Xilinx AXI-PCIe bridge (PG194) that "
        "holds PCIe completions until they can retire in AXI-ID order. "
        "Separate resource from PCIe tag pool — can backpressure the "
        "link even when tags are free. Typical values: 128/256/512 "
        "(synthesis-time parameter). 0 = disabled.")

    # ---- Review item #3: Core-clock TLP gap ----
    # Note: this is the per-TLP emission gap enforced by the hard
    # block. It's NAMED differently from pcie_core_clock (the
    # Frequency param above used for assembly rate) because they
    # represent the same physical quantity but are used in different
    # contexts and gem5 requires distinct Param names.
    pcie_tlp_gap = Param.Latency('1ns',
        "Minimum gap between consecutive TLPs emitted by the PCIe "
        "hard block. Typically equal to the core clock period "
        "(pcie_core_clock). Matters only for streams of small TLPs; "
        "large coalesced TLPs are serialization-limited anyway. "
        "0 = disabled.")

    # ---- Review item #4: Credit return cadence ----
    credit_return_period = Param.Latency('8ns',
        "Minimum interval between DLLP credit-return frames. Real PCIe "
        "batches credit returns every few TLPs or time window rather "
        "than per-TLP. 8ns ≈ 4 TLPs at Gen5 x16 rates. 0 = per-TLP.")

    # ---- Review item #7: Completion timeout ----
    completion_timeout = Param.Latency('50ms',
        "PCIe completion timeout per spec §2.8. Tag is reclaimed and "
        "error raised if no response within this window. Never fires "
        "in healthy simulation; included for realism and to detect "
        "deadlock-class bugs. Default 50 ms is the typical Range B "
        "lower bound.")

    # ---- Review item #8: Header size (32 vs 64-bit addressing) ----
    use_64bit_addr = Param.Bool(True,
        "If true, use 4DW (16B) TLP headers for 64-bit addresses. "
        "If false, 3DW (12B) for 32-bit. ORAM workloads using host "
        "DDR5 above 4GB require 64-bit. Affects wire byte counts.")