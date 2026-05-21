from m5.objects.ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *


class CxlModel(ClockedObject):
    """FLIT-level pipelined CXL.mem model with credit flow control.

    Models CXL Type-3 device path with FLIT-based framing.
    Same Gen5 x16 physical layer as PCIe but with:
      - No TLP header overhead (6B FLIT header vs 12-16B TLP)
      - No DLLP ACK layer
      - FLIT-level credit flow control (simpler than PCIe's per-type credits)
      - Simpler endpoint integration (5ns vs PCIe 15ns bridge pipeline)
      - Same RC traversal latency as PCIe (150ns)

    Latency model:
      rc_latency (150ns) = full one-way RC traversal including Home Agent
        coherency processing and CPU interconnect. Same as PCIe.
      endpoint_delay (5ns) = CXL endpoint controller processing delay.
        Based on published Synopsys/Cadence CXL 3.0 IP figures (5-8ns).
      root_port_delay (5ns) = CXL root port integration stage. Unlike
        PCIe, integrated into Home Agent rather than a separate hop.
      flit_coalesce_window (0ns) = fixed-FLIT CXL doesn't coalesce.
      hostSendSpacing = per-packet injection rate (3.33ns = RTL clock).
      max_outstanding = backpressure cap (0 = disabled, default 32).
    """

    type = 'CxlModel'
    cxx_header = 'mem/cxl/cxl_model.hh'
    cxx_class = 'gem5::CxlModel'

    # ---- Ports ----
    device_side_port = VectorResponsePort("Ports facing FPGA/ORAM devices")
    host_side_port = VectorRequestPort(
        "Ports facing host memory (DDR5 via xbar). VectorRequestPort "
        "explicitly supports multi-binding (e.g. 4 hostPorts into "
        "host_xbar). Review fix #4: was RequestPort which relied on "
        "gem5's implicit repeated-assignment pattern; VectorRequestPort "
        "makes the intent explicit and matches PCIeModel.")

    # ---- CXL Link (same PHY as PCIe) ----
    gen = Param.UInt32(5, "CXL generation (maps to PCIe PHY: 5=Gen5)")
    lanes = Param.UInt32(16, "Number of lanes")

    # ---- FLIT parameters ----
    max_payload = Param.UInt32(256, "Max payload bytes per upstream write FLIT group (CXL M2S RwD spec: up to 4 × 64B slots per 256B FLIT)")
    max_read_size = Param.UInt32(512, "Max read request size bytes")
    max_tags = Param.UInt32(256, "Max outstanding read tags")
    cpl_combine_bytes = Param.UInt32(256,
        "Completion combining boundary bytes. Accumulate DDR5 "
        "responses until this many bytes are pending, then emit one "
        "combined CplD FLIT. Default 256 = full CXL.mem FLIT data "
        "area (one FLIT carries up to 256B of payload; framing is "
        "amortized into the slot/header structure). At 256B the "
        "wire FLIT carries 6 (FLIT hdr) + 16 (DRS hdr) + 256 "
        "(payload) + 16 (CRC) = 294B = 1 FLIT slot, ~87% wire-"
        "efficient on completions. Previously 192 (~76% efficient) "
        "to leave conservative headroom for header growth, but the "
        "header-slot budget within a 256B FLIT is fixed at 38B by "
        "Bug 17 fix elsewhere in the model — 256 is the natural "
        "boundary. At 256B the downstream FLIT count drops ~25% "
        "vs 192, proportionally reducing downstream wire contention. "
        "Lower values (e.g., 64) force more FLITs and increase wire "
        "time — useful only for RCB-strict comparisons with legacy "
        "PCIe.")

    # ---- Latency ----
    rc_latency = Param.Latency('150ns',
        "Full one-way RC traversal. Same as PCIe: CXL shares the same "
        "Gen5 PHY link and CPU die interconnect. CXL Home Agent adds "
        "coherency processing that offsets FLIT decode savings.")
    fpga_clock_period = Param.Latency('3333ps',
        "Bug #10 fix: FPGA bridge clock period for CDC throughput and "
        "cycle-count reporting. Default 3333ps = 300MHz matches the "
        "oram_freq default. Must match the Python config's fpga_clock "
        "if it differs from 300MHz, otherwise cycle counts in CXL-DIAG "
        "output will be miscomputed.")
    burst_window = Param.Latency('10ns',
        "Burst window for RC optimization. FLITs arriving within this "
        "window share one RC traversal. Bias #6 fix: reduced from 20ns "
        "to match PCIe's 10ns (both were previously asymmetric at "
        "20ns CXL vs 5ns PCIe, biasing CXL faster since it shared "
        "warm-RC across a 4x wider window). No physical reason CXL's "
        "burst window should differ from PCIe's — both reflect the "
        "same underlying RC pipeline depth.")
    root_port_delay = Param.Latency('5ns',
        "CXL root port integration delay. Unlike PCIe, the CXL root "
        "port is integrated into the CPU's Home Agent as a single "
        "pipeline stage rather than a separate decode hop. Real CXL 3.0 "
        "IP controllers (Synopsys, Cadence) show ~5ns for this stage. "
        "Previously 15ns matching PCIe's bridge_pipeline_delay — that "
        "assumed PCIe-equivalent architecture, which is wrong: CXL's "
        "root port is not a separate hop.")
    endpoint_delay = Param.Latency('5ns',
        "CXL endpoint controller processing delay. Published values "
        "for production CXL 3.0 endpoint IP: 5-8ns. Previous 10ns was "
        "defensively high without grounding. Real CXL endpoint decode "
        "is faster than PCIe TLP decode because fixed-size 256B FLITs "
        "avoid variable-length parsing overhead.")
    rc_throughput_delay = Param.Latency('5ns',
        "Per-response RC pipeline throughput cost when warm. "
        "Each response still physically traverses the RC stages. "
        "5ns = ~5 stages at 1GHz.")
    max_outstanding = Param.UInt32(32,
        "Max outstanding read requests before backpressure. "
        "Models CXL endpoint read tag/completion depth. "
        "0 = disabled. 32 = conservative. 64 = typical.")
    max_outstanding_writes = Param.UInt32(128,
        "Max outstanding posted writes before backpressure. "
        "Models CXL endpoint write buffer depth. "
        "Separate from read maxOutstanding because posted writes "
        "don't consume tags. 128 = typical CXL controller FIFO.")
    flit_credits = Param.UInt32(128,
        "FLIT credits granted by host RC per device-side port. Each "
        "upstream FLIT consumes one credit, returned after "
        "flit_credit_return_delay. NOTE: the implementation in "
        "cxl_model.cc allocates ONE pool of this size per device-side "
        "port (see flitCredits.assign(devicePorts.size(), p.flit_credits) "
        "near construction). At N device-side ports the total credit "
        "budget is therefore N * flit_credits. This deviates from real "
        "CXL hardware (one pool per virtual channel) but matches the "
        "PCIe model's per-port credit accounting and prevents one "
        "instance's credit exhaustion from blocking another. At N=16 "
        "and default 128, the aggregate 2048 credits are over-"
        "provisioned and credits are NOT the multi-instance "
        "saturation bottleneck — the wire (upstreamBusyUntil), "
        "core-clock gate, and shared RC pipeline are. 0 = disable "
        "credit flow control entirely.")
    flit_credit_return_delay = Param.Latency('20ns',
        "Time for FLIT credit to return from host RC. Credits piggyback "
        "on downstream FLITs, not a separate round-trip. Similar to "
        "PCIe DLLP credit return timing.")

    # ---- Host Injection Pacing ----
    host_inject_interval = Param.Latency('1ns',
        "Minimum interval between successive host xbar injections. "
        "Models RC-to-xbar forwarding rate. 0 = send all in one tick "
        "(let xbar backpressure). 1ns = 1GHz host clock. "
        "2ns = match typical xbar layer occupancy.")
    flit_coalesce_window = Param.Latency('0ns',
        "Time to wait for more writes/reads before flushing a partial "
        "FLIT. Set to 0 because real CXL 3.0 uses fixed-size 256B FLITs "
        "with no coalescing wait — slots are packed as they arrive and "
        "the FLIT is sealed immediately when full or when traffic idle. "
        "Previous 7ns modeled a wait that doesn't exist in CXL hardware. "
        "Kept as a param in case future CXL revisions add coalesce logic.")

    # ---- Review item #1 (symmetric to PCIe RRB) ----
    completion_buffer_depth = Param.UInt32(256,
        "Home Agent completion buffer depth. Mirrors PCIe's RRB — "
        "separate resource from the CXL tag pool. Must be >= max_tags "
        "to prevent overflow. Default 256 matches max_tags.")

    # ---- Review item #3 (symmetric to PCIe core-clock gap) ----
    cxl_core_clock = Param.Latency('1ns',
        "Period of the CXL hard-block core clock. Drives TWO things: "
        "(1) minimum gap between consecutive FLIT emissions, (2) "
        "assembly rate = datapath_width / core_period. Default 1ns "
        "(1 GHz) × 64B datapath = 64 GB/s, which matches the Gen5 "
        "x16 wire rate (63 GB/s effective). Previous 2ns default "
        "(500 MHz = 32 GB/s) capped upstream throughput BELOW the "
        "wire and caused multi-instance N=2+ contention (one 32 GB/s "
        "pipe shared by 2 ORAMs). For modeling narrower links "
        "(Gen5 x8 = 32 GB/s, Gen4 x16 = 32 GB/s), use 2ns to match "
        "the wire rate. 0 = disable both gap and assembly-rate "
        "modeling (wire is the only cap).")
