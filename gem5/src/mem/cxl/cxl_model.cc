/*
 * cxl_model.cc — FLIT-level pipelined CXL.mem model for gem5 v25.1
 *
 * FLIT-based framing with credit flow control:
 *   - CXL.mem fixed 256B FLITs (no TLP headers, no DLLP ACK)
 *   - Endpoint + root port pipeline (replaces PCIe bridge pipeline)
 *   - Same Gen5 x16 physical layer as PCIe (shared PHY)
 *   - FLIT-level credit flow control (shared pool)
 *   - Burst-window RC optimization (same as PCIe model)
 */

#include "mem/cxl/cxl_model.hh"
#include <algorithm>

#include "base/cprintf.hh"
#include "base/trace.hh"
#include "debug/CXL.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

// ====================================================================
//  Constructor
// ====================================================================

CxlModel::CxlModel(const Params &p)
    : ClockedObject(p),
      // .hh:75-77 — FLIT Parameters
      maxPayload(p.max_payload),
      maxReadSize(p.max_read_size),
      cplCombineBytes(p.cpl_combine_bytes),
      // .hh:82-87 — Latency Parameters
      rcLatency(p.rc_latency),
      endpointDelay(p.endpoint_delay),
      hostInjectInterval(p.host_inject_interval),
      flitCoalesceWindow(p.flit_coalesce_window),
      rcThroughputDelay(p.rc_throughput_delay),
      fpgaClockPeriod(p.fpga_clock_period),
      // .hh:95 — Completion buffer
      completionBufferDepth(p.completion_buffer_depth),
      // .hh:100-101 — Core-clock gap
      cxlCoreClock(p.cxl_core_clock),
      lastFlitEmitTick(0),
      // .hh:120 — Tag tracking
      maxTags(p.max_tags),
      // .hh:188-191 — Link serialization shared state
      upstreamBusyUntil(0),
      downstreamBusyUntil(0),
      rcDownstreamBusyUntil(0),
      rcUpstreamBusyUntil(0),
      // .hh:279-292 — Pipeline events (declaration order)
      drainDeviceEvent([this]{ drainDeviceRequests(); }, name()),
      upstreamEvent([this]{ processUpstreamQueue(); }, name()),
      downstreamEvent([this]{ processDownstreamQueue(); }, name()),
      deviceRetryEvent([this]{ retryDeviceSend(); }, name()),
      hostRetryEvent([this]{ retryHostSend(); }, name()),
      // .hh:329 — Response event
      responseEvent([this]{ trySendResponses(); }, name()),
      // .hh:380 — FLIT credit event
      flitCreditEvent([this]{ processFlitCreditReturn(); }, name()),
      // .hh:394 — Host send event
      hostSendEvent([this]{ trySendToHost(); }, name()),
      // .hh — Periodic diagnostic event
      diagEvent([this]{
          dumpCxlState("periodic");
          checkPortStalls();
          schedule(diagEvent, curTick() + 10000000);
      }, name()),
      // .hh:414 — Stats
      stats(*this)
{
    // Review fix #5: fpgaClockPeriod zero guard. Division by zero in
    // reporting paths if a user passes fpga_clock_period=0. Clamp to
    // ≥1 tick here.
    if (fpgaClockPeriod == 0) {
        warn("CxlModel: fpga_clock_period=0 is invalid; clamping to 1 tick. "
             "Set a realistic value (e.g. 3333ps for 300MHz).\n");
        fpgaClockPeriod = 1;
    }

    // Create device ports (one per connected ORAM instance)
    for (int i = 0; i < p.port_device_side_port_connection_count; i++) {
        devicePorts.push_back(new DeviceSidePort(
            csprintf("%s.device_side_port[%d]", name(), i), *this, i));
    }

    // Bug 3: create host ports (one per connected DDR5 xbar input).
    for (int i = 0; i < p.port_host_side_port_connection_count; i++) {
        hostPorts.push_back(new HostSidePort(
            csprintf("%s.host_side_port[%d]", name(), i), *this, i));
    }

    // Bug 1 (per-port CDC): each device port has independent
    // downstream bridge FIFO. Initialize to 0 (idle) for all.
    endpointBusyUntil.resize(devicePorts.size(), 0);
    // Bug 2 (per-port outstanding): per-port read counters
    perPortReadsOut.resize(devicePorts.size(), 0);

    // Phase A.1: per-port flow-control resources. Each device-side
    // port gets its own credit pool, write-budget, FLIT queues, host
    // injection queue, response queue, and credit-return tracker.
    // Pools are full per port (NOT split) — realistic for separate
    // AXI masters with their own controller channels.
    outstandingWrites.resize(devicePorts.size(), 0);
    // FLIT credits: SHARED pool. All ports compete for one link-level
    // credit budget stored at flitCredits[0] / flitCreditsMax[0].
    // flitCredits[1..N-1] are unused (zero). This models real CXL where
    // the RC advertises one credit set per VC, shared by all functions.
    // Matches PCIe model's credits[0] shared-pool convention.
    unsigned N = devicePorts.size();
    flitCreditsMax.assign(devicePorts.size(), 0);
    flitCredits.assign(devicePorts.size(), 0);
    flitCreditsMax[0] = p.flit_credits;  // shared pool at index 0
    flitCredits[0] = p.flit_credits;     // shared pool at index 0
    deferredFlitCredits.resize(devicePorts.size());
    upstreamQueue.resize(devicePorts.size());
    downstreamQueue.resize(devicePorts.size());
    pendingHostReqs.resize(devicePorts.size());
    responseQueue.resize(devicePorts.size());
    completionBufferOccupied.resize(devicePorts.size(), 0);

    // Phase B diagnostic: per-port last-drain tracking. Init to 0 so a
    // port that has never received traffic doesn't trigger a false
    // stuck-warning. Updated whenever a packet leaves any of that
    // port's queues (drain, dispatch, response-send).
    perPortLastDrainTick.resize(devicePorts.size(), 0);

    // Per-port request buffers (per-channel FIFOs in real AXI-CXL hw).
    deviceReadBuffers.resize(devicePorts.size());
    deviceWriteBuffers.resize(devicePorts.size());
    lastReadBufferTicks.resize(devicePorts.size(), 0);
    lastWriteBufferTicks.resize(devicePorts.size(), 0);

    // Debug accounting: one counter per device port / host port.
    perPortBeatsAccepted.resize(devicePorts.size(), 0);
    perPortReadsAccepted.resize(devicePorts.size(), 0);
    perPortReadsDelivered.resize(devicePorts.size(), 0);
    perPortWritesAccepted.resize(devicePorts.size(), 0);
    perPortBrespsDelivered.resize(devicePorts.size(), 0);
    perHostPortSent.resize(hostPorts.size(), 0);
    perHostPortRejected.resize(hostPorts.size(), 0);
    lastProgressTick = 0;
    lastDiagDumpTick = 0;
    lastStuckAlarmTick = 0;

    linkParams.gen = p.gen;
    linkParams.lanes = p.lanes;
    computeLinkParams();

    burstWindowTicks = p.burst_window;
    // Initialize to a value that guarantees the first request pays full
    // RC latency. Intentional unsigned wrap — produces a large value
    // so (now - lastUpstreamRcTick) > burstWindowTicks on first check.
    lastUpstreamRcTick = (Tick)0 - burstWindowTicks - 1;
    lastDownstreamRcTick = (Tick)0 - burstWindowTicks - 1;
    // outstandingWrites resized to per-port vector above; per-port
    // entries are zero-initialized by the resize() call.
    pendingDeviceRetry = false;
    nextRetryPort = 0;
    maxOutstanding = p.max_outstanding;
    maxOutstandingWrites = p.max_outstanding_writes;
    // Completion buffer: per-port occupancy tracking.
    rootPortDelay = p.root_port_delay;
    // hostSendSpacing removed — host xbar provides natural backpressure

    // FLIT credit flow control — per-port pools sized above.
    flitCreditReturnDelay = p.flit_credit_return_delay;

    // Bug 4: cap tag pool size — matches PCIe model's 4096 ceiling
    if (maxTags > 4096) {
        warn("CXL: max_tags=%u > 4096: capping to 4096\n", maxTags);
        maxTags = 4096;
    }

    // Initialize tag pool
    for (unsigned t = 0; t < maxTags; t++)
        freeTags.push_back(t);

    DPRINTF(CXL, "CXL Gen%u x%u: eff BW=%llu B/s, byte_delay=%llu ticks\n",
            linkParams.gen, linkParams.lanes,
            linkParams.effectiveBwBytesPerSec,
            linkParams.byteSerializationDelay);
    DPRINTF(CXL, "CXL: rc_latency=%llu ticks, endpoint_delay=%llu, "
            "root_port_delay=%llu, max_payload=%u, max_read=%u, tags=%u\n",
            rcLatency, endpointDelay, rootPortDelay,
            maxPayload, maxReadSize, maxTags);

    // Phase A.1: per-port resource sizing summary. Visible in inform log
    // so the substrate is verifiable per-run without needing a debug
    // flag — confirms each device-side port has its own credit pool,
    // write budget, and queue set.
    inform("CXL Phase A.1: %lu device port(s), %lu host port(s); "
           "per-port flit_credits=%u, max_outstanding_writes=%u, "
           "max_outstanding_reads=%u; shared tag pool=%u",
           devicePorts.size(), hostPorts.size(),
           flitCreditsMax.empty() ? 0 : flitCreditsMax[0],
           maxOutstandingWrites, maxOutstanding, maxTags);
}

void CxlModel::computeLinkParams()
{
    // CXL requires Gen5+ physical layer (32 GT/s, 128b/130b encoding)
    double transferRateGTs;
    switch (linkParams.gen) {
      case 5: transferRateGTs = 32.0; linkParams.encodingEff = 0.9846; break;
      case 6: transferRateGTs = 64.0; linkParams.encodingEff = 0.9846; break;
      default:
        warn("CXL: Gen%u not a standard CXL generation (CXL requires "
             "Gen5+). Defaulting to Gen5.\n", linkParams.gen);
        linkParams.gen = 5;
        transferRateGTs = 32.0; linkParams.encodingEff = 0.9846; break;
    }

    double rawBitsPerSec = transferRateGTs * 1e9 * linkParams.lanes;
    double effBytesPerSec = rawBitsPerSec * linkParams.encodingEff / 8.0;
    linkParams.effectiveBwBytesPerSec = (uint64_t)effBytesPerSec;

    // Ticks per byte on the wire — use gem5's tick rate
    linkParams.byteSerializationDelay =
        (Tick)((double)sim_clock::as_int::s / effBytesPerSec);
    if (linkParams.byteSerializationDelay == 0)
        linkParams.byteSerializationDelay = 1;
}

void CxlModel::init()
{
    ClockedObject::init();
    if (devicePorts.empty())
        fatal("CxlModel %s: no device_side_port connected\n", name());
    for (auto *dp : devicePorts)
        if (!dp->isConnected())
            fatal("CxlModel %s: device_side_port not connected\n", name());
    // Bug 3: check every host port is connected
    if (hostPorts.empty())
        fatal("CxlModel %s: no host_side_port connected\n", name());
    for (auto *hp : hostPorts)
        if (!hp->isConnected())
            fatal("CxlModel %s: host_side_port not connected\n", name());

    // Schedule periodic diagnostic (matches PCIe's diagEvent pattern)
    schedule(diagEvent, 10000000);  // first dump at 10M ticks
}

Port &CxlModel::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "device_side_port") {
        if (idx < devicePorts.size())
            return *devicePorts[idx];
        fatal("CxlModel %s: device_side_port idx %d out of range\n",
              name(), idx);
    }
    if (if_name == "host_side_port") {
        if (idx < hostPorts.size())
            return *hostPorts[idx];
        fatal("CxlModel %s: host_side_port idx %d out of range\n",
              name(), idx);
    }
    return ClockedObject::getPort(if_name, idx);
}

// ====================================================================
//  Tag Management
// ====================================================================

uint16_t CxlModel::allocateTag()
{
    assert(!freeTags.empty());
    uint16_t t = freeTags.front();
    freeTags.pop_front();
    return t;
}

void CxlModel::releaseTag(uint16_t t)
{
    freeTags.push_back(t);
}

// ====================================================================
//  Retry stalled device ports — matches PCIe's pattern
// ====================================================================

void CxlModel::retryStarvedPorts()
{
    unsigned n = devicePorts.size();
    if (n == 0) return;

    // Retry ONE port per call (matches PCIe pattern). Round-robin across
    // ports so no single port permanently wins retry races.
    // Shared-pool: outstanding reads/writes are global limits (one physical
    // CXL link), not per-port. Compute global sums for admission check.
    unsigned totalReadsOut = 0, totalWritesOut = 0;
    for (unsigned j = 0; j < n; j++) {
        totalReadsOut += perPortReadsOut[j];
        totalWritesOut += outstandingWrites[j];
    }
    for (unsigned i = 0; i < n; i++) {
        unsigned idx = (nextRetryPort + i) % n;
        bool canRead = hasFreeTags() &&
                       (maxOutstanding == 0 ||
                        totalReadsOut < maxOutstanding);
        bool canWrite = (maxOutstandingWrites == 0 ||
                         totalWritesOut < maxOutstandingWrites);
        if (!canRead && !canWrite) continue;

        auto *dp = devicePorts[idx];
        if (dp->needRetry) {
            dp->needRetry = false;
            dp->sendRetryReq();
            nextRetryPort = (idx + 1) % n;
            return;  // one port per call
        }
    }
}

// ====================================================================
//  Link Serialization
// ====================================================================

Tick CxlModel::serializationDelay(unsigned wireBytes) const
{
    return wireBytes * linkParams.byteSerializationDelay;
}

void CxlModel::enqueueUpstream(FlitEntry &flit)
{
    Tick now = curTick();

    // Bias #3 fix: apply rootPortDelay (declared in Python param and
    // assigned in constructor but previously never used in timing). This
    // is the FPGA-to-CXL-core bridge pipeline delay, analogous to PCIe's
    // bridgePipelineDelay (15 ns). Without it, CXL skipped ~15 ns/FLIT
    // that PCIe pays, systematically biasing CXL faster.
    //
    // Bias #8 fix: add assembly delay model matching PCIe. At Gen5 x16
    // with 64B datapath at 500 MHz core clock, assembling the wire bytes
    // onto the link takes ~31 ps/byte (totalBytes × 2000 / 64), often
    // exceeding serialization delay on payload-heavy FLITs. PCIe charges
    // this; CXL previously didn't, artificially reducing CXL throughput
    // cost on write-heavy phases.
    // Review fix #6: multiply before divide to preserve precision.
    // Previously `2000 / 64 = 31` (integer truncation) lost 3% per FLIT
    // (should be 31.25). Now: `wireBytes * 2000 / 64` — for wireBytes=256,
    // (256 * 2000) / 64 = 8000 ps exactly, vs old 256 * 31 = 7936 ps.
    // Hardcodes 500 MHz core / 64B width; PCIe exposes pcie_core_clock
    // and pcie_core_width as params — CXL should eventually follow if
    // modeling beyond Gen5 x16.
    // Assembly delay: FPGA endpoint datapath produces wire bytes at
    // (datapath_width × core_freq). Formula below uses cxlCoreClock
    // as core period (if set) else 2000 ps (500 MHz) default. At
    // 64B × 500 MHz = 32 GB/s this can be SLOWER than Gen5 x16 wire
    // (63 GB/s), making assembly the steady-state throughput cap.
    // Set --cxl-core-clock=1ns (1 GHz = Versal CPM5) for 64 GB/s
    // assembly matching the wire.
    //
    // linkDelay = max(asmDelay, serDelay) correctly models steady-
    // state throughput of the slower stage. Assembly and wire DO
    // pipeline (they can overlap for different FLITs), but the
    // overall rate is capped by whichever is slower.
    Tick corePeriodPs = (cxlCoreClock > 0) ? cxlCoreClock : 2000;
    Tick asmDelay = (flit.wireBytes * corePeriodPs + 63) / 64;  // ceiling
    Tick serDelay = serializationDelay(flit.wireBytes);
    Tick linkDelay = std::max(asmDelay, serDelay);

    // Track when assembly is the bottleneck (matches PCIe stat)
    if (asmDelay > serDelay)
        stats.assemblyBottleneckFlits++;

    // Bias #4 fix: apply flitCoalesceWindow on cold-start FLITs. Real
    // CXL endpoints wait briefly for slot-packing opportunities before
    // sealing a FLIT. If upstream has been idle, the first FLIT pays
    // this wait; subsequent pipelined FLITs don't (covered by busyUntil).
    Tick coalesceDelay = (upstreamBusyUntil < now) ?
                          flitCoalesceWindow : 0;

    // Review item #3 (symmetric to PCIe): enforce per-FLIT core-clock
    // gap. CXL hard block emits one FLIT per core-clock edge (~2ns at
    // 500 MHz). Matters for streams of small/back-to-back FLITs.
    Tick coreGateTick = (cxlCoreClock > 0 && lastFlitEmitTick > 0) ?
                        lastFlitEmitTick + cxlCoreClock : 0;

    // Endpoint controller processes, then bridge pipeline, then link.
    Tick earliest = now + endpointDelay + rootPortDelay + coalesceDelay;
    Tick startTick = std::max({earliest, upstreamBusyUntil, coreGateTick});
    if (coreGateTick > earliest && coreGateTick > upstreamBusyUntil) {
        DPRINTF(CXL, "  [CORE-GATE] FLIT delayed by core clock gap "
                "(last=%llu, period=%llu, gate=%llu) vs busy=%llu "
                "vs earliest=%llu → start=%llu\n",
                lastFlitEmitTick, cxlCoreClock, coreGateTick,
                upstreamBusyUntil, earliest, startTick);
    }
    flit.readyTick = startTick + linkDelay;
    upstreamBusyUntil = flit.readyTick;
    lastFlitEmitTick = startTick;  // Review item #3: track last emit

    // Phase A.1: push to per-port queue. The wire (upstreamBusyUntil)
    // and hard-block emit gate (lastFlitEmitTick) above remain shared,
    // so per-port queues do NOT add wire bandwidth — they only prevent
    // head-of-line blocking between independently-credited ports.
    int sp = flit.srcPortIdx;
    // Hard fail if srcPortIdx wasn't set or is out of range — a silent
    // default of 0 would route to port 0 regardless of true source,
    // producing wrong-port credit/queue routing without any visible
    // error. Catch it at the door instead.
    panic_if(sp < 0 || (unsigned)sp >= upstreamQueue.size(),
             "CXL enqueueUpstream: bad srcPortIdx=%d (numDevicePorts=%lu) "
             "type=%d tag=%u addr=0x%lx — caller forgot to set "
             "flit.srcPortIdx before enqueue",
             sp, upstreamQueue.size(), (int)flit.type,
             flit.tag, flit.addr);
    DPRINTF(CXL, "  [ENQUP] port=%d type=%d wireBytes=%u tag=%u "
            "readyTick=%llu queueDepth=%lu\n",
            sp, (int)flit.type, flit.wireBytes, flit.tag,
            flit.readyTick, upstreamQueue[sp].size() + 1);
    upstreamQueue[sp].push_back(flit);

    if (!upstreamEvent.scheduled())
        schedule(upstreamEvent, flit.readyTick);
}

void CxlModel::enqueueDownstream(FlitEntry &flit, Tick earliestStart)
{
    Tick startTick = std::max(
        std::max(curTick(), downstreamBusyUntil), earliestStart);
    Tick serDelay = serializationDelay(flit.wireBytes);
    flit.readyTick = startTick + serDelay;
    downstreamBusyUntil = flit.readyTick;

    // Phase A.1: push to per-port downstream queue keyed by source port.
    // The wire (downstreamBusyUntil) stays shared; queue split prevents
    // HOL blocking between completion streams to different ports.
    int sp = flit.srcPortIdx;
    panic_if(sp < 0 || (unsigned)sp >= downstreamQueue.size(),
             "CXL enqueueDownstream: bad srcPortIdx=%d (numDevicePorts=%lu) "
             "type=%d tag=%u — caller forgot to set flit.srcPortIdx",
             sp, downstreamQueue.size(), (int)flit.type, flit.tag);
    DPRINTF(CXL, "  [ENQDN] port=%d type=%d wireBytes=%u tag=%u "
            "readyTick=%llu queueDepth=%lu\n",
            sp, (int)flit.type, flit.wireBytes, flit.tag,
            flit.readyTick, downstreamQueue[sp].size() + 1);
    downstreamQueue[sp].push_back(flit);

    if (!downstreamEvent.scheduled())
        schedule(downstreamEvent, flit.readyTick);
    else if (downstreamEvent.when() > flit.readyTick)
        reschedule(downstreamEvent, flit.readyTick);
}

// ====================================================================
//  Burst-Window RC Helper — REMOVED (dead code)
//  Both upstream and downstream paths compute RC delay inline using
//  rcUpstreamBusyUntil / rcDownstreamBusyUntil serialization.
// ====================================================================

// ====================================================================
//  Port Implementations
// ====================================================================

CxlModel::DeviceSidePort::DeviceSidePort(
    const std::string &name, CxlModel &owner, int idx)
    : ResponsePort(name), owner(owner), portIdx(idx), needRetry(false) {}
Tick CxlModel::DeviceSidePort::recvAtomic(PacketPtr pkt)
{
    // Forward to host memory so atomic/fastforward mode gets real data.
    Tick hostLat = owner.hostPorts[0]->sendAtomic(pkt);
    return hostLat + owner.rcLatency;
}
bool CxlModel::DeviceSidePort::recvTimingReq(PacketPtr pkt)
{ return owner.handleDeviceRequest(pkt, portIdx); }
void CxlModel::DeviceSidePort::recvRespRetry()
{ owner.trySendResponses(); }
// Bug 3: forward functional/range-change queries to host port 0
// (all host ports share the same address range via the xbar).
void CxlModel::DeviceSidePort::recvFunctional(PacketPtr pkt)
{ owner.hostPorts[0]->sendFunctional(pkt); }
AddrRangeList CxlModel::DeviceSidePort::getAddrRanges() const
{ return owner.hostPorts[0]->getAddrRanges(); }

CxlModel::HostSidePort::HostSidePort(
    const std::string &name, CxlModel &owner, int idx)
    : RequestPort(name), owner(owner), portIdx(idx), needRetry(false) {}
bool CxlModel::HostSidePort::recvTimingResp(PacketPtr pkt)
{ return owner.handleHostResponse(pkt); }
void CxlModel::HostSidePort::recvReqRetry()
{
    DPRINTF(CXL, "  [PORT] recvReqRetry on host port[%d] @%llu\n",
            portIdx, curTick());
    // Bug 3: clear THIS port's retry flag (not a shared scalar)
    needRetry = false;
    owner.trySendToHost();
}
void CxlModel::HostSidePort::recvRangeChange()
{
    for (auto *dp : owner.devicePorts)
        dp->sendRangeChange();
}

// ====================================================================
//  Stage 1: Device Request → Build FLITs → Upstream
//
//  CXL.mem requests use lightweight FLIT headers (6B) instead of
//  PCIe TLP headers (12-16B). No credit checks, no DLLP ACK.
// ====================================================================

bool
CxlModel::handleDeviceRequest(PacketPtr pkt, int srcPort)
{
    DPRINTF(CXL, "CXL-RECV @%llu: srcPort=%d %s addr=0x%lx size=%u\n",
            curTick(), srcPort, pkt->isRead() ? "RD" : "WR",
            pkt->getAddr(), pkt->getSize());

    DPRINTF(CXL, "DevReq[%d]: %s addr=0x%x size=%u\n",
            srcPort, pkt->isRead() ? "RD" : "WR",
            pkt->getAddr(), pkt->getSize());
    checkInvariants("handleDeviceRequest");

    // ---- Backpressure at accept time ----
    // Reads: accept freely into buffer. Real CXL endpoints accept
    // requests into internal FIFOs. The actual limits are tags and
    // FLIT credits, checked in processBufferedRead at TLP/FLIT build
    // time — not at AXI acceptance. This matches PCIe's behavior
    // (Xilinx PG194: bridge accepts AR beats into FIFO, stalls only
    // when FIFO is physically full at 1024+ entries).
    if (pkt->isRead()) {
        // Accept into buffer — tags gate at processBufferedRead
    } else if (pkt->isWrite()) {
        // Shared-pool: sum writes across all ports for global limit check.
        unsigned totalWritesOut = 0;
        for (unsigned j = 0; j < devicePorts.size(); j++)
            totalWritesOut += outstandingWrites[j];
        if (maxOutstandingWrites > 0 &&
            totalWritesOut >= maxOutstandingWrites) {
            DPRINTF(CXL, "  WR BUFFER FULL: portBuf=%zu outstanding[%d]=%u totalWr=%u max=%u\n",
                    deviceWriteBuffers[srcPort].size(), srcPort,
                    outstandingWrites[srcPort], totalWritesOut, maxOutstandingWrites);
            devicePorts[srcPort]->needRetry = true;
            return false;
        }
    }

    if (pkt->isRead()) {
        deviceReadBuffers[srcPort].push_back({pkt, srcPort});
        lastReadBufferTicks[srcPort] = curTick();
        perPortBeatsAccepted[srcPort]++;
        perPortReadsAccepted[srcPort]++;
        lastProgressTick = curTick();
        DPRINTF(CXL, "  [ACCEPT-RD] port=%d addr=0x%lx size=%u "
                "portBuf=%lu portRd=%u @%llu\n",
                srcPort, pkt->getAddr(), pkt->getSize(),
                deviceReadBuffers[srcPort].size(), perPortReadsOut[srcPort],
                curTick());

        if (rdTracker.reqCount == 0) rdTracker.firstDevReq = curTick();
        rdTracker.lastDevReq = curTick();
        rdTracker.reqCount++;
        rdTracker.isRead = true;

    } else if (pkt->isWrite()) {
        deviceWriteBuffers[srcPort].push_back({pkt, srcPort});
        outstandingWrites[srcPort]++;
        lastWriteBufferTicks[srcPort] = curTick();
        perPortBeatsAccepted[srcPort]++;
        perPortWritesAccepted[srcPort]++;
        lastProgressTick = curTick();
        DPRINTF(CXL, "  [ACCEPT-WR] port=%d addr=0x%lx size=%u "
                "portBuf=%lu outWr[%d]=%u @%llu\n",
                srcPort, pkt->getAddr(), pkt->getSize(),
                deviceWriteBuffers[srcPort].size(), srcPort,
                outstandingWrites[srcPort], curTick());

        // Track timing
        if (wrTracker.reqCount == 0) wrTracker.firstDevReq = curTick();
        wrTracker.lastDevReq = curTick();
        wrTracker.reqCount++;
        wrTracker.isRead = false;

        if (firstWriteAccept == 0) firstWriteAccept = curTick();

        // Posted-write BRESP: deliver immediately at endpoint_delay.
        // BRESP is a local ack from the endpoint buffer — it does not
        // travel on the CXL link. The FLIT is forwarded upstream later.
        PacketPtr respPkt = new Packet(pkt->req, MemCmd::WriteResp);
        // Review fix #8: skip allocate() — WriteResp carries no payload
        // so the buffer is wasted. Matches PCIe Bug #16 fix. Consistent
        // across both models.
        // Bug #12: OWNERSHIP TRANSFER — the device's senderState chain
        // moves from pkt (the original device request) to respPkt (the
        // BRESP being returned). This is necessary because respPkt is
        // what the device port actually receives; it needs the upstream
        // chain to route correctly. pkt->senderState is cleared so later
        // code (write-path host send at ~line 810) doesn't double-free
        // when it deletes the devicePkt. If you ever add code between
        // here and that delete that dereferences pkt->senderState, you
        // will read null — move your access above this transfer.
        respPkt->senderState = pkt->senderState;
        pkt->senderState = nullptr;

        Tick deliverAt = curTick() + endpointDelay;
        responseQueue[srcPort].push_back({respPkt, srcPort, deliverAt});
        if (!responseEvent.scheduled())
            schedule(responseEvent, deliverAt);
        else if (deliverAt < responseEvent.when())
            reschedule(responseEvent, deliverAt);

        lastBrespDelivered = curTick() + endpointDelay;
        perPortBrespsDelivered[srcPort]++;  // Debug: per-port BRESP counter
        writeBrespCount++;

        if (wrTracker.firstDnDone == 0)
            wrTracker.firstDnDone = curTick() + endpointDelay;
        wrTracker.lastDnDone = curTick() + endpointDelay;
        wrTracker.respCount++;
        // NOTE: do NOT delete pkt here — it's retained in deviceWriteBuffer
        // so its data can be copied into host pkts after FLIT serialization.
        // processUpstreamQueue's write branch owns the delete.

    } else {
        // Bug #4 fix: non-R/W packets used to bypass the FLIT pipeline
        // and go straight to pendingHostReqs without a SenderState. When
        // the response came back, handleHostResponse would find no
        // SenderState and drop the pkt, stranding the originator.
        // ORAM workload is read/write only — we reject non-RW at entry
        // rather than plumb SenderState through a path that shouldn't
        // exist for this simulation. If future use cases need non-RW
        // (e.g., flushes, barriers), wire a minimal SenderState here.
        warn("CXL: rejecting non-R/W cmd=%s addr=0x%x (not supported)\n",
             pkt->cmdString(), pkt->getAddr());
        return false;
    }

    // Opts 1+2: schedule coalescing drain. We defer rather than
    // processing inline so all beats of one AXI burst land in the
    // buffer before processBufferedRead scans for sequential addresses.
    if (!drainDeviceEvent.scheduled())
        schedule(drainDeviceEvent, curTick());
    else if (drainDeviceEvent.when() > curTick())
        reschedule(drainDeviceEvent, curTick());
    return true;
}

// ====================================================================
//  Opt 1: Process buffered reads with coalescing.
//  Scan the front of deviceReadBuffer for sequential 32B reads from
//  the SAME device port and coalesce them into one group: 1 tag, 1
//  upstream FLIT slot, 1 upstream host read (expanded into N × 32B
//  in processUpstreamQueue for DDR5 xbar interleaving).
//  Returns true if at least one read was processed.
// ====================================================================
bool
CxlModel::processBufferedRead(PacketPtr pkt, int srcPort)
{
    unsigned beatSize = pkt->getSize();
    unsigned maxCoalesce = (beatSize > 0) ? maxReadSize / beatSize : 1;
    if (maxCoalesce == 0) maxCoalesce = 1;

    // Admission: tags and FLIT credits are the hard gates here.
    // Check FLIT credits BEFORE allocating a tag so tags don't drain
    // while the wire is idle (A1 fix: matches PCIe's nphCredits check
    // before allocateTag).
    if (!hasFreeTags())
        return false;
    if (flitCreditsMax[0] > 0 && flitCredits[0] == 0) {
        DPRINTF(CXL, "  RD FLIT CREDIT STALL: port=%d credits=%u/%u\n",
                srcPort, flitCredits[0], flitCreditsMax[0]);
        return false;
    }

    // Per-port buffer: all entries belong to this srcPort. Coalesce
    // contiguous-address front entries.
    auto &portBuf = deviceReadBuffers[srcPort];

    std::vector<PacketPtr> group;
    group.push_back(pkt);  // front of buffer

    Addr nextAddr = pkt->getAddr() + beatSize;
    size_t scanCount = 0;
    for (size_t idx = 1;
         group.size() < maxCoalesce && idx < portBuf.size();
         idx++) {
        auto &entry = portBuf[idx];
        if (entry.pkt->getAddr() == nextAddr &&
            entry.pkt->getSize() == beatSize) {
            group.push_back(entry.pkt);
            nextAddr += beatSize;
            scanCount = idx;
        } else {
            // Non-contiguous within this port's burst: stop.
            break;
        }
    }

    unsigned totalSize = group.size() * beatSize;

    // Pop positions 1..scanCount (front is popped by caller).
    if (scanCount > 0)
        portBuf.erase(portBuf.begin() + 1, portBuf.begin() + 1 + scanCount);

    uint16_t tag = allocateTag();
    perPortReadsOut[srcPort]++;  // Bug 2: track per-port (per GROUP)

    OutstandingRead orec;
    orec.pkt = pkt;
    orec.allPkts = std::move(group);
    orec.addr = pkt->getAddr();
    orec.totalBytes = totalSize;
    orec.issueTick = curTick();
    orec.srcPortIdx = srcPort;
    outstandingReads[tag] = std::move(orec);

    stats.readRequests++;
    stats.totalReadBytes += totalSize;
    stats.readCoalesceHist.sample(totalSize / beatSize);

    // Build the upstream FLIT. One coalesced read = one M2S Req slot.
    // CXL 3.0 packs up to 4 request slots per 256B FLIT. Under ORAM's
    // sustained burst traffic (64 read groups per DDR_READ), real CXL
    // achieves full slot packing. Amortized per-slot wire cost =
    // 256B / 4 slots = 64B. This is higher than PCIe's 22B MRd TLP
    // (real CXL penalty for fixed-size FLITs) but lower than charging
    // a full 256B FLIT per request (which ignores slot packing).
    static const unsigned SLOTS_PER_FLIT = 4;
    static const unsigned AMORTIZED_REQ_WIRE = FLIT_SIZE / SLOTS_PER_FLIT; // 64B
    FlitEntry flit;
    flit.wireBytes = AMORTIZED_REQ_WIRE;
    flit.type = FlitType::ReadReq;
    flit.tag = tag;
    flit.addr = outstandingReads[tag].addr;
    flit.payloadBytes = 0;
    flit.origPkt = nullptr;
    flit.issueTick = curTick();
    flit.isLast = true;
    flit.srcPortIdx = srcPort;
    // For reads, the coalesced group is stored in outstandingReads[tag].allPkts
    // and is expanded lazily in processUpstreamQueue. For writes, the
    // group is stored in flit.allWritePkts and expanded similarly.

    stats.totalWireBytes += flit.wireBytes;
    stats.totalReadFlits += 1;
    enqueueUpstream(flit);

    rdTracker.flitFlushCount++;
    rdTracker.totalFlitEntries += 1;
    rdTracker.totalWireBytes += flit.wireBytes;
    rdTracker.totalBeatsInFlits += outstandingReads[tag].allPkts.size();

    DPRINTF(CXL, "  [COALESCE] RD tag=%u addr=0x%x size=%u (%u beats) @%llu\n",
            tag, pkt->getAddr(), totalSize,
            (unsigned)outstandingReads[tag].allPkts.size(), curTick());
    return true;
}

// ====================================================================
//  Opt 2: Process buffered writes with coalescing.
//  Scan for sequential 32B writes from the same port, build one FLIT
//  that carries the group's device pkts via allWritePkts. Host pkts
//  with data are produced in processUpstreamQueue after FLIT serialization.
//  Returns true if at least one write was processed.
// ====================================================================
bool
CxlModel::processBufferedWrite(PacketPtr pkt, int srcPort)
{
    unsigned beatSize = pkt->getSize();
    unsigned maxCoalesce = (beatSize > 0) ? maxPayload / beatSize : 1;
    if (maxCoalesce == 0) maxCoalesce = 1;

    // FLIT credit gate. Posted writes consume upstream FLIT credits.
    // With FLIT_SIZE=256 capacity: 256B write = 1 FLIT = 1 credit.
    if (flitCreditsMax[0] > 0) {
        unsigned expectedFlits = (maxPayload + FLIT_SIZE - 1) / FLIT_SIZE;
        if (expectedFlits == 0) expectedFlits = 1;
        if (flitCredits[0] < expectedFlits) {
            DPRINTF(CXL, "  WR FLIT CREDIT STALL: port=%d credits=%u/%u "
                    "need=%u\n",
                    srcPort, flitCredits[0], flitCreditsMax[0],
                    expectedFlits);
            return false;
        }
    }

    auto &portBuf = deviceWriteBuffers[srcPort];

    std::vector<PacketPtr> group;
    group.push_back(pkt);

    Addr nextAddr = pkt->getAddr() + beatSize;
    size_t scanCount = 0;
    for (size_t idx = 1;
         group.size() < maxCoalesce && idx < portBuf.size();
         idx++) {
        auto &entry = portBuf[idx];
        if (entry.pkt->getAddr() == nextAddr &&
            entry.pkt->getSize() == beatSize) {
            group.push_back(entry.pkt);
            nextAddr += beatSize;
            scanCount = idx;
        } else {
            break;
        }
    }

    unsigned totalSize = group.size() * beatSize;

    // Pop positions 1..scanCount (front popped by caller).
    if (scanCount > 0)
        portBuf.erase(portBuf.begin() + 1, portBuf.begin() + 1 + scanCount);

    // Build the upstream FLIT for the coalesced write group.
    // CXL 3.0 fixed 256B FLITs carry up to 256B of user data per FLIT
    // (4 × 64B slots with minimal per-slot headers within the 256B frame).
    // Wire cost = ceil(payload / 256) * 256.
    // 256B write = 1 FLIT = 256B wire. 512B = 2 FLITs = 512B wire.
    unsigned wrWireBytes = ((totalSize + FLIT_SIZE - 1) / FLIT_SIZE)
                           * FLIT_SIZE;

    FlitEntry flit;
    flit.wireBytes = wrWireBytes;
    flit.type = FlitType::WriteReq;
    flit.tag = 0;
    flit.addr = pkt->getAddr();
    flit.payloadBytes = totalSize;
    flit.origPkt = nullptr;
    flit.issueTick = curTick();
    flit.isLast = true;
    flit.srcPortIdx = srcPort;
    flit.allWritePkts = std::move(group);

    stats.totalWireBytes += wrWireBytes;
    stats.totalWriteFlits += 1;
    stats.writeRequests++;
    stats.totalWriteBytes += totalSize;
    stats.writeCoalesceHist.sample(totalSize / beatSize);

    enqueueUpstream(flit);

    wrTracker.flitFlushCount++;
    wrTracker.totalFlitEntries += 1;
    wrTracker.totalWireBytes += wrWireBytes;
    wrTracker.totalBeatsInFlits += flit.allWritePkts.size();

    DPRINTF(CXL, "  [COALESCE] WR addr=0x%x size=%u (%u beats) @%llu\n",
            pkt->getAddr(), totalSize,
            (unsigned)flit.allWritePkts.size(), curTick());
    return true;
}

// ====================================================================
//  Opts 1+2: drain device buffers by building coalesced FLITs.
//  Called from handleDeviceRequest (same-tick drain) and from retry
//  callbacks. Write drain uses a coalesce window: wait until the
//  buffer has a full group or the window times out, whichever first.
// ====================================================================
void
CxlModel::drainDeviceRequests()
{
    // Per-port drain. Real Xilinx AXI-CXL bridge has independent
    // per-channel FIFOs — one per AXI master (per ORAM instance).
    // We model each port's buffer independently; coalescing within
    // a port's buffer is trivially correct because all entries are
    // same-port by construction.
    //
    // Coalescing timeout (Bug #7): each port has its own timeout
    // gate — beats arrive staggered at 300 MHz (~3333 ticks between
    // beats). We wait for rdCoalesceTarget beats OR timeout expiry
    // before emitting a FLIT.
    const Tick COALESCE_TIMEOUT = 60000;
    Tick now = curTick();

    unsigned n = devicePorts.size();
    Tick earliestDeadline = MaxTick;

    for (unsigned p = 0; p < n; p++) {
        auto &rdBuf = deviceReadBuffers[p];
        auto &wrBuf = deviceWriteBuffers[p];

        // --- reads on port p ---
        if (!rdBuf.empty()) {
            unsigned beatSize = rdBuf.front().pkt->getSize();
            unsigned rdTarget = 16;
            if (beatSize > 0 && maxReadSize > 0) {
                rdTarget = maxReadSize / beatSize;
                if (rdTarget == 0) rdTarget = 1;
            }
            // Step 9 fix: path-tree reads come in 32B beats (rdTarget=16
            // expected). Single non-burst reads should not wait for a
            // coalesce group that will never form. The original Step 9
            // condition only handled large single reads (cmd-ring fetch
            // at 64B+). Phase B fix: ALSO drain any single read whose
            // beatSize != 32 immediately. CPU MMIO polls (8 bytes) are
            // the trigger case — they would otherwise hang in the rd
            // buffer indefinitely because rdTarget = maxReadSize/8 = 64
            // is unreachable for a CPU poll, AND the strict-> timeout
            // check below has a boundary off-by-one. Detect: 1 entry,
            // beatSize anything other than the ORAM burst size of 32B.
            bool nonBurstReady = (rdBuf.size() == 1 && beatSize != 32);
            bool rdReady = nonBurstReady ||
                           rdBuf.size() >= rdTarget ||
                           now >= lastReadBufferTicks[p] + COALESCE_TIMEOUT;
            if (rdReady) {
                while (!rdBuf.empty()) {
                    PacketPtr frontPkt = rdBuf.front().pkt;
                    int frontSrcPort = rdBuf.front().srcPort;
                    if (!processBufferedRead(frontPkt, frontSrcPort)) break;
                    rdBuf.pop_front();
                    // Phase B diagnostic: track per-port progress.
                    if (frontSrcPort < (int)perPortLastDrainTick.size())
                        perPortLastDrainTick[frontSrcPort] = now;
                }
            } else {
                // Phase B diagnostic: rdBuf has work but is not ready
                // to drain. Surface why (size+timeout+target) so a
                // future deadlock investigation can grep DRAIN-WAIT
                // and see the gating reason without re-running.
                DPRINTF(CXL, "  [DRAIN-WAIT-RD] port=%u rdBuf=%lu beat=%u "
                        "target=%u nonBurst=%d now=%llu lastTick=%llu "
                        "deadline=%llu (in %llu ticks)\n",
                        p, rdBuf.size(), beatSize, rdTarget,
                        (int)nonBurstReady, now, lastReadBufferTicks[p],
                        lastReadBufferTicks[p] + COALESCE_TIMEOUT,
                        (lastReadBufferTicks[p] + COALESCE_TIMEOUT > now)
                            ? (lastReadBufferTicks[p] + COALESCE_TIMEOUT - now)
                            : 0);
            }
            if (!rdBuf.empty() && rdBuf.size() < rdTarget) {
                Tick dl = lastReadBufferTicks[p] + COALESCE_TIMEOUT;
                if (dl < earliestDeadline) earliestDeadline = dl;
            }
        }

        // --- writes on port p ---
        if (!wrBuf.empty()) {
            unsigned wrTarget = (maxPayload > 0) ? (maxPayload / 32) : 1;
            if (wrTarget == 0) wrTarget = 1;
            // Step 9 fix: same as reads — non-burst writes (cons_idx
            // writebacks, CPU MMIO writes) shouldn't wait for coalesce.
            // Phase B fix: ALSO drain any single write whose beatSize
            // != 32 immediately. CPU's 8-byte cmd-ring updates and
            // ORAM's 64B cons_idx writebacks both fall here.
            unsigned wrBeatSize = wrBuf.front().pkt->getSize();
            bool wrNonBurstReady = (wrBuf.size() == 1 && wrBeatSize != 32);
            bool wrReady = wrNonBurstReady ||
                           wrBuf.size() >= wrTarget ||
                           now >= lastWriteBufferTicks[p] + COALESCE_TIMEOUT;
            if (wrReady) {
                while (!wrBuf.empty()) {
                    PacketPtr frontPkt = wrBuf.front().pkt;
                    int frontSrcPort = wrBuf.front().srcPort;
                    if (!processBufferedWrite(frontPkt, frontSrcPort)) break;
                    wrBuf.pop_front();
                    // Phase B diagnostic: track per-port progress.
                    if (frontSrcPort < (int)perPortLastDrainTick.size())
                        perPortLastDrainTick[frontSrcPort] = now;
                }
            } else {
                DPRINTF(CXL, "  [DRAIN-WAIT-WR] port=%u wrBuf=%lu beat=%u "
                        "target=%u nonBurst=%d now=%llu lastTick=%llu "
                        "deadline=%llu (in %llu ticks)\n",
                        p, wrBuf.size(), wrBeatSize, wrTarget,
                        (int)wrNonBurstReady, now, lastWriteBufferTicks[p],
                        lastWriteBufferTicks[p] + COALESCE_TIMEOUT,
                        (lastWriteBufferTicks[p] + COALESCE_TIMEOUT > now)
                            ? (lastWriteBufferTicks[p] + COALESCE_TIMEOUT - now)
                            : 0);
            }
            if (!wrBuf.empty() && wrBuf.size() < wrTarget) {
                Tick dl = lastWriteBufferTicks[p] + COALESCE_TIMEOUT;
                if (dl < earliestDeadline) earliestDeadline = dl;
            }
        }
    }

    // Reschedule only for the coalesce-timeout case. Tag-stall and
    // new-beat arrival paths have external wakes (releaseTag,
    // handleDeviceRequest).
    if (earliestDeadline != MaxTick && earliestDeadline > now
        && !drainDeviceEvent.scheduled()) {
        schedule(drainDeviceEvent, earliestDeadline);
    }
}

// ====================================================================
//  Stage 2: Upstream Done → Forward to Host
//
//  After burst FLIT serialization completes, expand the coalesced
//  group into individual 32B host packets (read: from
//  outstandingReads[tag].allPkts; write: from flit.allWritePkts with
//  per-pkt data copy) and move them to pendingHostReqs.
// ====================================================================

void CxlModel::processUpstreamQueue()
{
    Tick now = curTick();
    unsigned n = upstreamQueue.size();
    if (n == 0) return;

    // Phase A.1: per-port round-robin dispatch. Each port has its own
    // credit pool and its own pendingHostReqs queue, so a port that's
    // out of credits cannot block another port's dispatch. Cursor
    // advances by one each call so port-0 doesn't permanently win
    // tie-breaks at equal readyTicks.
    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextUpstreamPort + i) % n;
        while (!upstreamQueue[sp].empty() &&
               upstreamQueue[sp].front().readyTick <= now) {

            // Per-port FLIT credit check
            if (flitCreditsMax[0] > 0 && flitCredits[0] == 0) {
                DPRINTF(CXL, "  [UP] port=%u FLIT credit stall @%llu\n",
                        sp, now);
                break;  // try other ports; this one waits for return
            }

            FlitEntry flit = std::move(upstreamQueue[sp].front());
            upstreamQueue[sp].pop_front();

            // Phase B diagnostic: track per-port progress
            if (sp < perPortLastDrainTick.size())
                perPortLastDrainTick[sp] = now;

            // Phase A.1 invariant: the FLIT we just popped from
            // upstreamQueue[sp] must declare the same source port.
            // A mismatch means someone enqueued to the wrong queue.
            panic_if(flit.srcPortIdx != (int)sp,
                     "CXL processUpstreamQueue: FLIT on queue[%u] has "
                     "srcPortIdx=%d (mismatch). type=%d tag=%u",
                     sp, flit.srcPortIdx, (int)flit.type, flit.tag);

            // Consume FLIT credits from this port's pool — one credit
            // per wire FLIT (256B). A 256B write that requires 2 wire
            // FLITs consumes 2 credits.
            if (flitCreditsMax[0] > 0) {
                unsigned wireFlits = (flit.wireBytes + FLIT_SIZE - 1) / FLIT_SIZE;
                if (wireFlits == 0) wireFlits = 1;
                // If not enough credits, stall — try other ports
                if (flitCredits[0] < wireFlits) {
                    DPRINTF(CXL, "  [UP] port=%u FLIT credit stall: "
                            "need=%u have=%u @%llu\n",
                            sp, wireFlits, flitCredits[0], now);
                    // Push back — we already popped, need to re-insert
                    upstreamQueue[sp].push_front(std::move(flit));
                    break;
                }
                flitCredits[0] -= wireFlits;
                for (unsigned fc = 0; fc < wireFlits; fc++) {
                    DeferredFlitCredit dc;
                    dc.returnTick = now + flitCreditReturnDelay;
                    deferredFlitCredits[sp].push_back(dc);
                }
                if (!flitCreditEvent.scheduled())
                    schedule(flitCreditEvent, now + flitCreditReturnDelay);
            }

            // Upstream RC pipeline (shared, serialized) — same model as
            // downstream. Cold = pipeline idle. Warm = serialize at
            // rcThroughputDelay via rcUpstreamBusyUntil.
            bool rcColdUp = (now > rcUpstreamBusyUntil + burstWindowTicks);
            Tick rcStartUp, rcEndUp;
            if (rcColdUp) {
                rcStartUp = now + rcLatency;
                rcEndUp = rcStartUp;
            } else {
                rcStartUp = std::max(now, rcUpstreamBusyUntil);
                rcEndUp = rcStartUp + rcThroughputDelay;
            }
            rcUpstreamBusyUntil = rcEndUp;
            Tick earliestSend = std::max(rcEndUp, curTick() + 1);
            unsigned moved = 0;

            if (flit.type == FlitType::ReadReq) {
                auto it = outstandingReads.find(flit.tag);
                if (it == outstandingReads.end()) {
                    DPRINTF(CXL, "  [UP] WARNING: read tag=%u not found!\n",
                            flit.tag);
                    continue;
                }
                OutstandingRead &orec = it->second;
                for (auto *devPkt : orec.allPkts) {
                    PacketPtr hostPkt = new Packet(devPkt->req, MemCmd::ReadReq);
                    hostPkt->allocate();

                    auto *ss = new SenderState;
                    ss->tag = flit.tag;
                    ss->origAddr = orec.addr;
                    ss->origSize = orec.totalBytes;
                    ss->issueTick = orec.issueTick;
                    ss->srcPortIdx = flit.srcPortIdx;
                    hostPkt->pushSenderState(ss);

                    pendingHostReqs[sp].push_back({hostPkt, earliestSend});
                    moved++;
                }
                DPRINTF(CXL, "  [UP] port=%u Read FLIT done: tag=%u "
                        "expanded=%u cold=%d rcEnd=%llu @%llu\n",
                        sp, flit.tag, moved, rcColdUp, rcEndUp, now);

            } else if (flit.type == FlitType::WriteReq) {
                for (auto *devicePkt : flit.allWritePkts) {
                    PacketPtr hostPkt = new Packet(devicePkt->req,
                                                   MemCmd::WriteReq);
                    hostPkt->allocate();
                    if (devicePkt->hasData())
                        hostPkt->setData(devicePkt->getConstPtr<uint8_t>());

                    auto *ss = new SenderState;
                    ss->tag = 0;
                    ss->origAddr = devicePkt->getAddr();
                    ss->origSize = devicePkt->getSize();
                    ss->issueTick = flit.issueTick;
                    ss->isPostedWrite = true;
                    ss->origDevicePkt = nullptr;
                    ss->srcPortIdx = flit.srcPortIdx;
                    hostPkt->pushSenderState(ss);

                    pendingHostReqs[sp].push_back({hostPkt, earliestSend});
                    moved++;
                    delete devicePkt;
                }
                flit.allWritePkts.clear();
                DPRINTF(CXL, "  [UP] port=%u Write FLIT done: expanded=%u "
                        "cold=%d rcEnd=%llu @%llu\n",
                        sp, moved, rcColdUp, rcEndUp, now);

            } else {
                DPRINTF(CXL, "  [UP] UNEXPECTED non-request FLIT type\n");
            }

            // Schedule host send at the earliest eligibility time
            if (moved > 0) {
                if (!hostSendEvent.scheduled())
                    schedule(hostSendEvent, earliestSend);
                else if (hostSendEvent.when() > earliestSend)
                    reschedule(hostSendEvent, earliestSend);
            }

            // Track timing
            PhaseTracker &t = (flit.type == FlitType::ReadReq)
                              ? rdTracker : wrTracker;
            if (t.firstFlitDone == 0) t.firstFlitDone = now;
            t.lastFlitDone = now;
        }
    }
    // Advance round-robin cursor so a different port leads next call.
    nextUpstreamPort = (nextUpstreamPort + 1) % n;

    // Reschedule if any port has FLITs pending in the future. Earliest
    // ready tick across all ports wins.
    if (!upstreamEvent.scheduled()) {
        Tick earliest = MaxTick;
        bool anyPending = false;
        for (unsigned sp = 0; sp < n; sp++) {
            if (!upstreamQueue[sp].empty()) {
                anyPending = true;
                Tick t = upstreamQueue[sp].front().readyTick;
                if (t < earliest) earliest = t;
            }
        }
        if (anyPending && earliest > curTick())
            schedule(upstreamEvent, earliest);
        // else: all ports either empty or credit-stalled; credit
        //       return path will wake us when credits free.
    }

    // Retry stalled device ports if anyone has room. Read tags are
    // global; write budget is per-port — check if at least one port
    // can accept a write.
    {
        bool canAcceptRead = hasFreeTags();
        // Shared-pool: check global write budget, not per-port.
        unsigned totalWritesOut = 0;
        for (unsigned sp = 0; sp < n; sp++)
            totalWritesOut += outstandingWrites[sp];
        bool canAcceptWrite = (maxOutstandingWrites == 0 ||
                               totalWritesOut < maxOutstandingWrites);
        if (canAcceptRead || canAcceptWrite)
            retryStarvedPorts();
    }
}

// ====================================================================
//  Stage 3: Host Response → Build Completion FLITs → Downstream
//
//  No completion reordering needed for CXL.mem — responses are
//  in-order per tag by protocol. No credit tracking needed.
// ====================================================================

bool CxlModel::handleHostResponse(PacketPtr pkt)
{
    DPRINTF(CXL, "CXL-RESP @%llu: %s addr=0x%lx size=%u\n",
            curTick(), pkt->isRead() ? "RD" : "WR",
            pkt->getAddr(), pkt->getSize());

    // Review fix #7: CXL's HostSidePort has no respBlocked / retry path
    // because handleHostResponse NEVER returns false — every return
    // below is `return true;`. Invariant: if the function ever grows a
    // `return false;` path (e.g., for credit-stalled completion emit),
    // HostSidePort must gain a respBlocked field and sendRetryResp()
    // call matching PCIe's pattern (see pcie_model HostSidePort).
    // Audited April 2026 — all return paths verified true.
    checkInvariants("handleHostResponse");
    auto *ss = dynamic_cast<SenderState *>(pkt->popSenderState());
    if (!ss) {
        warn("CXL: host response without SenderState!\n");
        delete pkt;
        return true;
    }

    if (ss->isPostedWrite) {
        DPRINTF(CXL, "  [HOST] Write done (posted, BRESP already returned): "
                "addr=0x%x @%llu\n", ss->origAddr, curTick());

        // Stats only — BRESP was already delivered at endpoint_delay.
        // Bug #6 fix: sample in ticks (matches declared unit Tick).
        // Previous /1000000 was converting ps → µs but label said Tick;
        // sub-µs latencies sampled as 0 and distribution was useless.
        if (ss->issueTick > 0) {
            Tick wrLat = curTick() - ss->issueTick;
            stats.totalWriteLatency += wrLat;
            stats.writeLatencyHist.sample(wrLat);
        }

        // Track in-flight writes: decrement when DDR5 actually completes,
        // not when injected to host. This maintains correct backpressure.
        // Phase A.1: per-port. The senderState carries srcPortIdx from
        // handleDeviceRequest → enqueueUpstream → upstream dispatch → host
        // request build, so the right port's count gets decremented here.
        assert(ss->srcPortIdx >= 0 &&
               (unsigned)ss->srcPortIdx < outstandingWrites.size());
        outstandingWrites[ss->srcPortIdx]--;
        lastDdr5WriteCommit = curTick();
        writeCommitCount++;

        // Print write commit comparison when ALL ports' writes committed
        unsigned totalOutstandingWr = 0;
        for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
            totalOutstandingWr += outstandingWrites[sp];
        if (totalOutstandingWr == 0 && firstWriteAccept > 0) {
            Tick rtlVisible = lastBrespDelivered - firstWriteAccept;
            Tick ddr5Commit = lastDdr5WriteCommit - firstWriteAccept;
            inform("CXL WRITE commit: RTL_visible=%llu cyc (%.1f ns), "
                   "DDR5_commit=%llu cyc (%.1f ns), "
                   "posted_advantage=%llu cyc (%.1f ns), "
                   "bresps=%u commits=%u",
                   rtlVisible / fpgaClockPeriod, rtlVisible / 1000.0,
                   ddr5Commit / fpgaClockPeriod, ddr5Commit / 1000.0,
                   (ddr5Commit - rtlVisible) / fpgaClockPeriod,
                   (ddr5Commit - rtlVisible) / 1000.0,
                   writeBrespCount, writeCommitCount);
            // Reset for next write phase
            firstWriteAccept = 0;
            lastBrespDelivered = 0;
            lastDdr5WriteCommit = 0;
            writeCommitCount = 0;
            writeBrespCount = 0;
        }

        // Send retry to ALL ports that need it (any ORAM could be stalled
        // on reads OR writes — check both)
        // Bug 2: coarse gate here; retryStarvedPorts checks per-port room.
        {
            bool canRead = hasFreeTags();
            unsigned totalWritesOut = 0;
            for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
                totalWritesOut += outstandingWrites[sp];
            bool canWrite = (maxOutstandingWrites == 0 ||
                             totalWritesOut < maxOutstandingWrites);
            if (canRead || canWrite)
                retryStarvedPorts();
        }

        // Track timing
        if (wrTracker.firstHostResp == 0) wrTracker.firstHostResp = curTick();
        wrTracker.lastHostResp = curTick();

        delete ss;
        delete pkt;
        return true;
    }

    // ---- Read Completion with combining (Bug 9) ----
    // Accumulate beats per-tag until we have cplCombineBytes worth of
    // data (default 128B = 4 beats at 32B each) or we hit the final beat
    // of the group. Flushing one combined cpl FLIT amortizes the CDC
    // crossing, RC throughput delay, and downstream wire serialization
    // across multiple beats — matching PCIe's RCB-based combining.
    uint16_t tag = ss->tag;
    unsigned responseSize = pkt->getSize();

    DPRINTF(CXL, "  [HOST] Read resp: tag=%u addr=0x%x size=%u @%llu\n",
            tag, ss->origAddr, responseSize, curTick());

    auto readIt = outstandingReads.find(tag);
    if (readIt == outstandingReads.end()) {
        warn("CXL: completion for unknown tag %u\n", tag);
        delete ss;
        delete pkt;
        return true;
    }
    OutstandingRead &orec = readIt->second;

    // Review item #1 (symmetric to PCIe RRB): Home Agent completion
    // buffer occupancy tracking. In real CXL, S2M DRS FLITs are held
    // in an internal buffer between the Home Agent and the AXI R
    // channel until they can retire in AXI-ID order. We track
    // occupancy here. With default max_tags=256 bounding total
    // in-flight reads and completionBufferDepth=128, the buffer is
    // the tighter constraint — a warn fires if exceeded (represents
    // a configuration that would stall real hardware). Full
    // backpressure would require extending HostSidePort with
    // respBlocked logic (see PCIe for that pattern); deferred as
    // the default config doesn't reach the limit.
    // Phase A.1: per-port. Each AXI master has its own bridge HA
    // buffer; track occupancy against the tag's owning port.
    int orecPort = orec.srcPortIdx;
    assert(orecPort >= 0 &&
           (unsigned)orecPort < completionBufferOccupied.size());
    completionBufferOccupied[orecPort]++;
    DPRINTF(CXL, "  [CPLBUF-ACCEPT] tag=%u addr=0x%lx port=%d occ=%u/%u\n",
            tag, pkt->getAddr(), orecPort,
            completionBufferOccupied[orecPort], completionBufferDepth);
    if (completionBufferDepth > 0 &&
        completionBufferOccupied[orecPort] > completionBufferDepth) {
        warn("CXL: port=%d completion buffer occupancy %u exceeds "
             "completion_buffer_depth=%u (shared pool, transient).\n",
             orecPort, completionBufferOccupied[orecPort],
             completionBufferDepth);
    }

    // Accumulate: record this beat. pkt is the individual host read
    // that returned from DDR5 — its addr and data belong to ONE beat.
    orec.pendingCplBytes += responseSize;
    orec.pendingCplBeats.push_back({pkt->getAddr(), pkt});

    // DIAG: trace DDR5 response data for ring entry addresses
    if (pkt->getAddr() >= 0x600000000 && pkt->getAddr() < 0x600001000 &&
        pkt->hasData()) {
        const uint8_t *rd = pkt->getConstPtr<uint8_t>();
        uint32_t rdSlot = 0;
        memcpy(&rdSlot, rd, 4);
        inform("CXL-DDR5-RESP tag=%u addr=0x%lx size=%u "
               "ddr5_bytes[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x "
               "slot_addr=0x%x",
               tag, pkt->getAddr(), responseSize,
               rd[0],rd[1],rd[2],rd[3],rd[4],rd[5],rd[6],rd[7],
               rdSlot);
    }

    // Decide whether to flush now:
    //   (a) pendingCplBytes has reached the combining boundary, or
    //   (b) this is the final beat of the group (all bytes accumulated).
    bool isLastForTag = (orec.emittedBytes + orec.pendingCplBytes
                         >= orec.totalBytes);
    bool flushNow = (orec.pendingCplBytes >= cplCombineBytes) || isLastForTag;

    // Track timing on every response (even combined ones)
    if (rdTracker.firstHostResp == 0) rdTracker.firstHostResp = curTick();
    rdTracker.lastHostResp = curTick();

    if (!flushNow) {
        // Defer emission — wait for more responses to combine.
        DPRINTF(CXL, "  [COMBINE] tag=%u pending=%uB beats=%lu "
                "(< boundary=%u), defer\n",
                tag, orec.pendingCplBytes, orec.pendingCplBeats.size(),
                cplCombineBytes);
        delete ss;
        // NOTE: do NOT delete pkt here — it's held in pendingCplBeats
        // for per-beat delivery in processDownstreamQueue.
        return true;
    }

    // Flush: emit ONE combined cpl FLIT.
    unsigned combinedBytes = orec.pendingCplBytes;
    unsigned combinedBeats = orec.pendingCplBeats.size();

    // ================================================================
    // Downstream delay: RC pipeline → Gen5 wire → per-port CDC → RTL
    //
    // Same two bugs as PCIe (see pcie_model.cc for full description):
    //  Bug A: dead rcThroughputDelay — warm RC gave 0 instead of 5ns.
    //  Bug B: CDC↔wire ordering — CDC-done dominated wire serialization.
    // Fix: 3-stage serialized pipeline in physical order.
    // ================================================================
    Tick now = curTick();

    // Build the combined cpl FLIT first (need wireBytes for stage 2).
    unsigned cplWireBytes = ((combinedBytes + FLIT_SIZE - 1) / FLIT_SIZE)
                            * FLIT_SIZE;

    FlitEntry cplFlit;
    cplFlit.wireBytes = cplWireBytes;
    cplFlit.type = FlitType::ReadCompletion;
    cplFlit.tag = tag;
    cplFlit.addr = orec.pendingCplBeats.front().beatAddr;
    cplFlit.payloadBytes = combinedBytes;
    cplFlit.origPkt = nullptr;
    cplFlit.issueTick = orec.issueTick;
    cplFlit.isLast = isLastForTag;
    int dstPort = ss->srcPortIdx;
    cplFlit.srcPortIdx = dstPort;
    cplFlit.combinedBeats = std::move(orec.pendingCplBeats);
    orec.pendingCplBeats.clear();

    // Stage 1: RC pipeline (shared, serialized).
    // Cold detection uses pipeline state (rcDownstreamBusyUntil), not
    // DDR5 inter-arrival gap. See pcie_model.cc for full rationale.
    bool rcCold = (now > rcDownstreamBusyUntil + burstWindowTicks);
    Tick rcStart;
    Tick rcEnd;
    if (rcCold) {
        rcStart = now + rcLatency;
        rcEnd = rcStart;
    } else {
        rcStart = std::max(now, rcDownstreamBusyUntil);
        rcEnd = rcStart + rcThroughputDelay;
    }
    rcDownstreamBusyUntil = rcEnd;
    // lastDownstreamRcTick removed — was written but never read

    // Stage 2: Shared Gen5 wire serialization.
    Tick wireSerDelay = serializationDelay(cplFlit.wireBytes);
    Tick wireStart = std::max(rcEnd, downstreamBusyUntil);
    Tick wireEnd = wireStart + wireSerDelay;
    downstreamBusyUntil = wireEnd;

    // Stage 3: Per-port CDC endpoint — can't start until wire delivers.
    // The CDC FIFO drains at the FPGA-side AXI rate: 32B per FPGA cycle.
    // A 256B CplD FLIT occupies the CDC output for ceil(256/32)=8 cycles,
    // not the previous flat 2 cycles. This is the binding throughput
    // constraint between the CXL hard block and the FPGA fabric.
    unsigned cdcDrainBeats = (combinedBytes + 31) / 32;
    Tick cdcThroughput = cdcDrainBeats * fpgaClockPeriod;
    Tick startCdc = std::max(wireEnd, endpointBusyUntil[dstPort]);
    Tick doneCdc = startCdc + cdcThroughput;
    endpointBusyUntil[dstPort] = doneCdc;
    cplFlit.readyTick = doneCdc;

    DPRINTF(CXL, "  [COMBINE] FLUSH tag=%u %uB (%u beats) emitted=%u/%u "
            "isLast=%d @%llu\n",
            tag, combinedBytes, combinedBeats,
            orec.emittedBytes, orec.totalBytes, isLastForTag, now);
    DPRINTF(CXL, "  [DN RC→WIRE→CDC] port=%d tag=%u wire=%u "
            "rc=[%llu,%llu] wire=[%llu,%llu] cdc=[%llu,%llu]\n",
            dstPort, tag, cplFlit.wireBytes,
            rcStart, rcEnd, wireStart, wireEnd, startCdc, doneCdc);

    orec.pendingCplBytes = 0;
    orec.emittedBytes += combinedBytes;

    // Push to per-port downstream queue at CDC-done time.
    // Bypass enqueueDownstream (it would re-apply wire delay).
    panic_if(dstPort < 0 || (unsigned)dstPort >= downstreamQueue.size(),
             "CXL downstream: bad srcPortIdx=%d", dstPort);
    downstreamQueue[dstPort].push_back(cplFlit);
    stats.totalWireBytes += cplFlit.wireBytes;
    stats.totalCompletionFlits++;

    if (!downstreamEvent.scheduled())
        schedule(downstreamEvent, doneCdc);
    else if (downstreamEvent.when() > doneCdc)
        reschedule(downstreamEvent, doneCdc);

    delete ss;
    // pkt ownership of the LAST beat transfers via cplFlit.combinedBeats
    // into processDownstreamQueue. Earlier beats are also held in that
    // vector and have their lifetime managed there.
    return true;
}

// ====================================================================
//  Stage 4: Downstream → Deliver to Device
// ====================================================================

void CxlModel::processDownstreamQueue()
{
    Tick now = curTick();
    unsigned n = downstreamQueue.size();
    if (n == 0) return;

    // Phase A.1: round-robin per-port iteration. Each port has its own
    // downstream FLIT queue, so a port whose head FLIT isn't yet ready
    // doesn't block another port's deliveries.
    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextDownstreamPort + i) % n;
        while (!downstreamQueue[sp].empty() &&
               downstreamQueue[sp].front().readyTick <= now) {

            FlitEntry flit = std::move(downstreamQueue[sp].front());
            downstreamQueue[sp].pop_front();

            // Phase B diagnostic: track per-port progress
            if (sp < perPortLastDrainTick.size())
                perPortLastDrainTick[sp] = now;

            // Phase A.1 invariant: queue index must match flit's source.
            panic_if(flit.srcPortIdx != (int)sp,
                     "CXL processDownstreamQueue: FLIT on queue[%u] has "
                     "srcPortIdx=%d (mismatch). type=%d tag=%u",
                     sp, flit.srcPortIdx, (int)flit.type, flit.tag);

            if (flit.type == FlitType::ReadCompletion) {
                auto readIt = outstandingReads.find(flit.tag);
                if (readIt == outstandingReads.end()) {
                    warn("CXL: downstream read completion for unknown/already-"
                         "completed tag %u — possible duplicate FLIT\n",
                         flit.tag);
                    for (auto &bi : flit.combinedBeats)
                        if (bi.hostPkt) delete bi.hostPkt;
                    continue;
                }
                OutstandingRead &orec = readIt->second;
                int srcPort = flit.srcPortIdx;

                for (auto &bi : flit.combinedBeats) {
                    PacketPtr devPktForBeat = nullptr;
                    size_t matchIdx = orec.allPkts.size();
                    for (size_t i2 = 0; i2 < orec.allPkts.size(); i2++) {
                        auto *dp = orec.allPkts[i2];
                        if (dp && dp->getAddr() == bi.beatAddr) {
                            devPktForBeat = dp;
                            matchIdx = i2;
                            break;
                        }
                    }
                    if (!devPktForBeat) {
                        warn("CXL: beat addr 0x%lx has no matching "
                             "undelivered device pkt in tag %u (group "
                             "base 0x%lx size %u) — dropping\n",
                             bi.beatAddr, flit.tag,
                             orec.addr, orec.totalBytes);
                        unsigned beatSize = (bi.hostPkt) ?
                            bi.hostPkt->getSize() : 32;
                        orec.completedBytes += beatSize;
                        if (bi.hostPkt) delete bi.hostPkt;
                        continue;
                    }
                    orec.allPkts[matchIdx] = nullptr;

                    devPktForBeat->makeResponse();
                    if (bi.hostPkt && bi.hostPkt->hasData()) {
                        devPktForBeat->setData(
                            bi.hostPkt->getConstPtr<uint8_t>());

                        // DIAG: trace data for ring entry addresses
                        if (bi.beatAddr >= 0x600000000 &&
                            bi.beatAddr <  0x600001000) {
                            const uint8_t *hd = bi.hostPkt->getConstPtr<uint8_t>();
                            const uint8_t *dd = devPktForBeat->getConstPtr<uint8_t>();
                            uint32_t hSlot = 0, dSlot = 0;
                            memcpy(&hSlot, hd, 4);
                            memcpy(&dSlot, dd, 4);
                            inform("CXL-DATA-COPY tag=%u addr=0x%lx "
                                   "hostPkt[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x (slot=0x%x) "
                                   "devPkt[0..7]=%02x %02x %02x %02x %02x %02x %02x %02x (slot=0x%x) "
                                   "match=%d",
                                   flit.tag, bi.beatAddr,
                                   hd[0],hd[1],hd[2],hd[3],hd[4],hd[5],hd[6],hd[7], hSlot,
                                   dd[0],dd[1],dd[2],dd[3],dd[4],dd[5],dd[6],dd[7], dSlot,
                                   (memcmp(hd, dd, devPktForBeat->getSize()) == 0));
                        }
                    } else {
                        warn_once("CXL: read response hostPkt=%p hasData=%d "
                                  "for tag=%u beat_addr=0x%lx — device gets "
                                  "uninitialized data\n",
                                  bi.hostPkt,
                                  bi.hostPkt ? (int)bi.hostPkt->hasData() : -1,
                                  flit.tag, bi.beatAddr);
                    }

                    Tick latency = now - orec.issueTick;

                    // Phase A.1: per-port responseQueue
                    responseQueue[srcPort].push_back(
                        {devPktForBeat, srcPort, curTick()});
                    orec.completedBytes += devPktForBeat->getSize();
                    perPortReadsDelivered[srcPort]++;
                    lastProgressTick = curTick();

                    // Review item #1: retirement frees one buffer entry.
                    // Phase A.1: per-port completion buffer occupancy.
                    if (completionBufferOccupied[srcPort] > 0)
                        completionBufferOccupied[srcPort]--;
                    // Completion buffer freed a slot — reschedule host send
                    // to unblock reads that were backpressured.
                    if (!hostSendEvent.scheduled())
                        schedule(hostSendEvent, now);
                    DPRINTF(CXL, "  [CPLBUF-RETIRE] port=%d occ=%u/%u\n",
                            srcPort, completionBufferOccupied[srcPort],
                            completionBufferDepth);

                    DPRINTF(CXL, "  [DELIVER] tag=%u port=%d beat_addr=0x%x "
                            "done=%u/%u lat=%llu @%llu\n",
                            flit.tag, srcPort, bi.beatAddr,
                            orec.completedBytes, orec.totalBytes,
                            latency, now);

                    if (rdTracker.firstDnDone == 0) rdTracker.firstDnDone = now;
                    rdTracker.lastDnDone = now;
                    rdTracker.respCount++;

                    if (bi.hostPkt) delete bi.hostPkt;
                }
                flit.combinedBeats.clear();

                // Retirement: only when ALL bytes of the group have been
                // delivered.
                if (flit.isLast && orec.completedBytes >= orec.totalBytes) {
                    unsigned groupBeats = orec.allPkts.size();
                    Tick retireLatency = now - orec.issueTick;
                    stats.totalReadLatency += retireLatency;
                    stats.readLatencyHist.sample(retireLatency);
                    DPRINTF(CXL, "  [RETIRE] tag=%u port=%d beats=%u "
                            "portRd=%u→%u @%llu\n",
                            flit.tag, srcPort, groupBeats,
                            perPortReadsOut[srcPort],
                            perPortReadsOut[srcPort] > 0 ?
                              perPortReadsOut[srcPort] - 1 : 0,
                            now);
                    releaseTag(flit.tag);
                    outstandingReads.erase(readIt);
                    if (perPortReadsOut[srcPort] > 0)
                        perPortReadsOut[srcPort]--;
                    bool anyRdBuf = false;
                    for (auto &q : deviceReadBuffers) {
                        if (!q.empty()) { anyRdBuf = true; break; }
                    }
                    if (anyRdBuf) {
                        if (!drainDeviceEvent.scheduled())
                            schedule(drainDeviceEvent, curTick());
                    }
                    if (hasFreeTags())
                        pendingDeviceRetry = true;
                }

                if (rdTracker.respCount >= rdTracker.reqCount &&
                    rdTracker.reqCount > 0)
                    printPhaseBreakdown(rdTracker, "READ");
            }
        }
    }
    nextDownstreamPort = (nextDownstreamPort + 1) % n;

    // Schedule responseEvent if any port has responses pending
    bool anyResp = false;
    for (unsigned sp = 0; sp < n; sp++) {
        if (!responseQueue[sp].empty()) { anyResp = true; break; }
    }
    if (anyResp && !responseEvent.scheduled())
        schedule(responseEvent, curTick());

    // Periodic state dump + stuck detection — aggregate emptiness
    // across per-port queues.
    bool anyRdBufPending = false;
    for (auto &q : deviceReadBuffers) {
        if (!q.empty()) { anyRdBufPending = true; break; }
    }
    bool anyWrBufPending = false;
    for (auto &q : deviceWriteBuffers) {
        if (!q.empty()) { anyWrBufPending = true; break; }
    }
    bool anyDnPending = false;
    for (unsigned sp = 0; sp < n; sp++) {
        if (!downstreamQueue[sp].empty()) { anyDnPending = true; break; }
    }
    bool anyUpPending = false;
    for (unsigned sp = 0; sp < n; sp++) {
        if (!upstreamQueue[sp].empty()) { anyUpPending = true; break; }
    }
    bool anyHostPending = false;
    for (unsigned sp = 0; sp < n; sp++) {
        if (!pendingHostReqs[sp].empty()) {
            anyHostPending = true; break;
        }
    }
    bool hasPending = anyResp || anyDnPending ||
                      anyHostPending || anyUpPending ||
                      anyRdBufPending || anyWrBufPending;
    if (hasPending) {
        if (curTick() - lastDiagDumpTick > 10000000) {  // every 10M ticks (matches PCIe)
            lastDiagDumpTick = curTick();
            dumpCxlState("periodic");
        }
        if (lastProgressTick > 0 &&
            curTick() - lastProgressTick > 5000000 &&
            curTick() - lastStuckAlarmTick > 10000000) {
            lastStuckAlarmTick = curTick();
            inform("CXL STUCK ALARM: no progress for %.1f ms",
                   (curTick() - lastProgressTick) / 1000000.0);
            dumpCxlState("STUCK");
        }
        // Phase B diagnostic: per-port stall detection. Catches cases
        // where the global lastProgressTick is being kept alive by
        // one port while another port is frozen.
        checkPortStalls();
    }

    // Reschedule if any port has downstream FLITs pending. Earliest
    // ready tick across all ports wins.
    if (!downstreamEvent.scheduled()) {
        Tick earliest = MaxTick;
        bool any = false;
        for (unsigned sp = 0; sp < n; sp++) {
            if (!downstreamQueue[sp].empty()) {
                any = true;
                Tick t = downstreamQueue[sp].front().readyTick;
                if (t < earliest) earliest = t;
            }
        }
        if (any)
            schedule(downstreamEvent, std::max(earliest, curTick() + 1));
    }
}

// ====================================================================
//  Response + Host Send
// ====================================================================

// ====================================================================
//  FLIT Credit Return
// ====================================================================

void CxlModel::processFlitCreditReturn()
{
    Tick now = curTick();
    unsigned n = deferredFlitCredits.size();

    // Phase A.1: per-port credit return. Each port has its own deferred
    // queue and its own pool to return into. Iterate all ports.
    for (unsigned sp = 0; sp < n; sp++) {
        while (!deferredFlitCredits[sp].empty() &&
               deferredFlitCredits[sp].front().returnTick <= now) {
            deferredFlitCredits[sp].pop_front();
            flitCredits[0] = std::min(flitCredits[0] + 1,
                                        flitCreditsMax[0]);
            DPRINTF(CXL, "  [CREDIT] port=%u FLIT credit returned: "
                    "%u/%u @%llu\n",
                    sp, flitCredits[0], flitCreditsMax[0], now);
        }
    }

    // Shared credit pool: wakeup drain if any port has buffered requests
    if (!drainDeviceEvent.scheduled()) {
        for (unsigned sp2 = 0; sp2 < n; sp2++) {
            if (!deviceReadBuffers[sp2].empty() ||
                !deviceWriteBuffers[sp2].empty()) {
                schedule(drainDeviceEvent, curTick());
                break;
            }
        }
    }

    // Schedule next credit return at the earliest pending returnTick
    // across all ports.
    if (!flitCreditEvent.scheduled()) {
        Tick earliest = MaxTick;
        bool any = false;
        for (unsigned sp = 0; sp < n; sp++) {
            if (!deferredFlitCredits[sp].empty()) {
                any = true;
                Tick t = deferredFlitCredits[sp].front().returnTick;
                if (t < earliest) earliest = t;
            }
        }
        if (any)
            schedule(flitCreditEvent, std::max(earliest, curTick() + 1));
    }

    // Credits available — wake upstream if any port has a stalled FLIT
    // AND that port now has credits.
    bool wakeUpstream = false;
    for (unsigned sp = 0; sp < n; sp++) {
        if (flitCredits[0] > 0 && !upstreamQueue[sp].empty()) {
            wakeUpstream = true; break;
        }
    }
    if (wakeUpstream && !upstreamEvent.scheduled())
        schedule(upstreamEvent, now);

    // Retry stalled device ports — any port with read tags or write
    // budget can now accept new requests.
    {
        bool canRead = hasFreeTags();
        unsigned totalWritesOut = 0;
        for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
            totalWritesOut += outstandingWrites[sp];
        bool canWrite = (maxOutstandingWrites == 0 ||
                         totalWritesOut < maxOutstandingWrites);
        if (canRead || canWrite)
            retryStarvedPorts();
    }
}

void CxlModel::trySendResponses()
{
    // Phase A.1: per-port responseQueue. Each port drains its own deque
    // independently — a blocked sendTimingResp on one port does not
    // prevent another port's responses from going through. Per-port
    // FIFO ordering is naturally preserved because each deque is
    // single-producer (downstream/BRESP) single-consumer (this fn).
    unsigned n = responseQueue.size();
    if (n == 0) return;

    size_t beforeTotal = 0;
    for (unsigned sp = 0; sp < n; sp++) beforeTotal += responseQueue[sp].size();

    // Round-robin starting from nextResponsePort so port 0 doesn't
    // permanently win when multiple ports are simultaneously ready.
    Tick earliestPending = MaxTick;
    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextResponsePort + i) % n;
        while (!responseQueue[sp].empty()) {
            auto &entry = responseQueue[sp].front();

            // Skip entries whose readyTick hasn't arrived yet
            if (entry.readyTick > curTick()) {
                if (entry.readyTick < earliestPending)
                    earliestPending = entry.readyTick;
                break;  // FIFO — later entries can't be earlier
            }

            PacketPtr pkt = entry.pkt;
            int portIdx = entry.portIdx;

            if (portIdx < 0 || portIdx >= (int)devicePorts.size()) {
                warn("CXL trySendResponses: invalid portIdx=%d (max=%lu), "
                     "dropping",
                     portIdx, devicePorts.size());
                responseQueue[sp].pop_front();
                continue;
            }
            // Sanity: queue srcPort and entry.portIdx should match.
            assert((unsigned)portIdx == sp);

            DPRINTF(CXL, "  [RESP] → Device[%d]: %s addr=0x%x @%llu\n",
                    portIdx, pkt->isRead() ? "RD" : "WR",
                    pkt->getAddr(), curTick());
            if (!devicePorts[portIdx]->sendTimingResp(pkt)) {
                DPRINTF(CXL, "  [RESP] Device[%d] BUSY — skip port\n",
                        portIdx);
                break;  // this port is blocked; try other ports
            }
            responseQueue[sp].pop_front();
            // Phase B diagnostic: track per-port progress
            if (sp < perPortLastDrainTick.size())
                perPortLastDrainTick[sp] = curTick();
        }
    }
    nextResponsePort = (nextResponsePort + 1) % n;

    // Stuck-response warn — recompute total post-iteration.
    size_t afterTotal = 0;
    for (unsigned sp = 0; sp < n; sp++) afterTotal += responseQueue[sp].size();
    if (afterTotal > 0 && afterTotal == beforeTotal &&
        curTick() - lastStuckWarn > 5000000) {
        lastStuckWarn = curTick();
        // Find any port with a stuck front entry for the diagnostic.
        for (unsigned sp = 0; sp < n; sp++) {
            if (!responseQueue[sp].empty()) {
                auto &front = responseQueue[sp].front();
                inform("CXL STUCK-RESP @%llu: respQ_total=%lu "
                       "port=%u front.isRd=%d front.addr=0x%lx",
                       curTick(), afterTotal,
                       sp,
                       front.pkt->isRead() ? 1 : 0,
                       front.pkt->getAddr());
                break;
            }
        }
    }

    // Reschedule for the earliest pending entry not yet ready
    if (earliestPending != MaxTick && !responseEvent.scheduled())
        schedule(responseEvent, earliestPending);

    // BUG FIX: if responses are stuck (sendTimingResp returned false),
    // earliestPending stays MaxTick because the stuck entries ARE ready
    // (readyTick <= now) but can't be delivered.  Without this retry,
    // the response event is never rescheduled, the queue grows until
    // a downstream pointer becomes stale, and drainDeviceRequests
    // segfaults.  Retry in 1000 ticks (1 ns) — cheap and guarantees
    // forward progress once the device port unblocks.
    if (afterTotal > 0 && !responseEvent.scheduled()) {
        schedule(responseEvent, curTick() + 1000);
    }

    if (pendingDeviceRetry) {
        pendingDeviceRetry = false;
        retryStarvedPorts();
    }
}
void CxlModel::retryDeviceSend() { trySendResponses(); }

void CxlModel::trySendToHost()
{
    unsigned n = pendingHostReqs.size();
    if (n == 0) return;

    size_t totalPending = 0;
    for (unsigned sp = 0; sp < n; sp++)
        totalPending += pendingHostReqs[sp].size();
    DPRINTF(CXL, "  [SEND] trySendToHost: pending_total=%lu ports=%lu @%llu\n",
            totalPending, hostPorts.size(), curTick());

    if (totalPending == 0) return;

    // If ALL host ports are blocked, nothing we can do until recvReqRetry.
    bool allBlocked = true;
    for (auto *hp : hostPorts)
        if (!hp->needRetry) { allBlocked = false; break; }
    if (allBlocked) return;

    // Phase A.1: per-port pendingHostReqs — round-robin across device
    // ports. Drain each port's queue subject to host-port availability
    // and earliestSend eligibility.
    Tick earliestDeferred = MaxTick;
    size_t dispatched = 0;
    bool anyTimeDeferred = false;

    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextHostSendPort + i) % n;
        for (auto it = pendingHostReqs[sp].begin();
             it != pendingHostReqs[sp].end(); ) {

            if (curTick() < it->earliestSend) {
                if (it->earliestSend < earliestDeferred)
                    earliestDeferred = it->earliestSend;
                anyTimeDeferred = true;
                ++it;
                continue;
            }

            PacketPtr pkt = it->pkt;
            // Route by 64B-aligned address bits across host ports.
            unsigned portIdx = (pkt->getAddr() >> 6) % hostPorts.size();

            if (hostPorts[portIdx]->needRetry) {
                ++it;
                continue;
            }

            // Shared completion buffer backpressure: if this is a read
            // and the buffer is full, skip — leave it queued until a
            // completion drains and frees a slot.
            if (pkt->isRead() && completionBufferDepth > 0 &&
                completionBufferOccupied[sp] >= completionBufferDepth) {
                DPRINTF(CXL, "  [HOSTSEND-CPLBUF-FULL] port=%u occ=%u/%u "
                        "— backpressure read\n",
                        sp, completionBufferOccupied[sp],
                        completionBufferDepth);
                ++it;
                continue;
            }

            bool isWritePkt = pkt->isWrite();
            DPRINTF(CXL, "  [HOSTSEND] dev_port=%u host_port=%u %s "
                    "addr=0x%lx size=%u @%llu\n",
                    sp, portIdx, pkt->isRead() ? "RD" : "WR",
                    pkt->getAddr(), pkt->getSize(), curTick());
            if (!hostPorts[portIdx]->sendTimingReq(pkt)) {
                DPRINTF(CXL, "  [HOSTSEND-REJECT] host=%u xbar BUSY\n",
                        portIdx);
                perHostPortRejected[portIdx]++;
                hostPorts[portIdx]->needRetry = true;
                PhaseTracker &tr = isWritePkt ? wrTracker : rdTracker;
                tr.hostRejectCount++;
                ++it;
                continue;
            }

            perHostPortSent[portIdx]++;
            lastProgressTick = curTick();

            PhaseTracker &t = isWritePkt ? wrTracker : rdTracker;
            if (t.firstHostSend == 0) t.firstHostSend = curTick();
            t.lastHostSend = curTick();
            t.hostSendCount++;

            it = pendingHostReqs[sp].erase(it);
            dispatched++;
        }
    }
    nextHostSendPort = (nextHostSendPort + 1) % n;

    // Reschedule only if a time-deferred packet exists OR we made
    // forward progress this round (other packets may now be eligible).
    bool anyPending = false;
    for (unsigned sp = 0; sp < n; sp++) {
        if (!pendingHostReqs[sp].empty()) { anyPending = true; break; }
    }
    if (anyPending && !hostSendEvent.scheduled()) {
        if (anyTimeDeferred || dispatched > 0) {
            Tick base = (hostInjectInterval > 0)
                ? curTick() + hostInjectInterval : curTick() + 1;
            Tick next = (earliestDeferred != MaxTick)
                ? std::max(base, earliestDeferred) : base;
            schedule(hostSendEvent, next);
        } else {
            // BUG FIX: defensive retry when all packets are xbar-blocked.
            // recvReqRetry should wake us, but retry at 1 ns as fallback.
            schedule(hostSendEvent, curTick() + 1000);
        }
    }
}

void CxlModel::retryHostSend() {
    size_t totalPending = 0;
    for (auto &q : pendingHostReqs) totalPending += q.size();
    DPRINTF(CXL, "  [RETRY] Host retry received @%llu (pending_total=%lu)\n",
            curTick(), totalPending);
    trySendToHost();
}

void CxlModel::printPhaseBreakdown(PhaseTracker &t, const char *label)
{
    if (t.reqCount == 0) return;
    inform("CXL %s phase breakdown (%u reqs, %u resps):", label,
           t.reqCount, t.respCount);
    inform("  DevReq:    first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstDevReq, t.lastDevReq,
           t.lastDevReq - t.firstDevReq,
           (t.lastDevReq - t.firstDevReq) / 1000.0);
    inform("  FlitDone:  first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstFlitDone, t.lastFlitDone,
           t.lastFlitDone - t.firstFlitDone,
           (t.lastFlitDone - t.firstFlitDone) / 1000.0);
    inform("  HostSend:  first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstHostSend, t.lastHostSend,
           t.lastHostSend - t.firstHostSend,
           (t.lastHostSend - t.firstHostSend) / 1000.0);
    inform("  HostResp:  first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstHostResp, t.lastHostResp,
           t.lastHostResp - t.firstHostResp,
           (t.lastHostResp - t.firstHostResp) / 1000.0);
    inform("  DnDone:    first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstDnDone, t.lastDnDone,
           t.lastDnDone - t.firstDnDone,
           (t.lastDnDone - t.firstDnDone) / 1000.0);
    inform("  Total:     first_req=%llu last_dn=%llu total=%llu (%.1f ns)",
           t.firstDevReq, t.lastDnDone,
           t.lastDnDone - t.firstDevReq,
           (t.lastDnDone - t.firstDevReq) / 1000.0);
    inform("  Upstream:  DevReq→HostSend = %.1f ns",
           (t.firstHostSend - t.firstDevReq) / 1000.0);
    inform("  DDR5:      HostSend→HostResp = %.1f ns",
           (t.firstHostResp - t.firstHostSend) / 1000.0);
    inform("  Downstream: HostResp→DnDone = %.1f ns",
           (t.lastDnDone - t.firstHostResp) / 1000.0);
    if (!t.isRead && t.lastHostResp > t.lastDnDone) {
        // For writes: posted BRESP (DnDone) arrives before DDR5 commit (HostResp)
        Tick ddr5CommitCycles = (t.lastHostResp - t.firstDevReq) / fpgaClockPeriod;
        Tick rtlVisibleCycles = (t.lastDnDone - t.firstDevReq) / fpgaClockPeriod;
        inform("  DDR5 commit:  %llu cyc (%.1f ns after first write)",
               ddr5CommitCycles,
               (t.lastHostResp - t.firstDevReq) / 1000.0);
        inform("  RTL visible:  %llu cyc (%.1f ns — posted BRESP)",
               rtlVisibleCycles,
               (t.lastDnDone - t.firstDevReq) / 1000.0);
        inform("  Posted advantage: %llu cyc (%.1f ns hidden)",
               ddr5CommitCycles - rtlVisibleCycles,
               (t.lastHostResp - t.lastDnDone) / 1000.0);
    }
    // FLIT packing and host injection debug
    inform("  FLIT packing: %u flushes, %u FLIT entries, %uB wire, "
           "%u beats packed (%.1f beats/flush)",
           t.flitFlushCount, t.totalFlitEntries, t.totalWireBytes,
           t.totalBeatsInFlits,
           t.flitFlushCount > 0 ?
           (double)t.totalBeatsInFlits / t.flitFlushCount : 0.0);
    inform("  Host xbar:    %u sent, %u rejected (%.1f%% reject rate)",
           t.hostSendCount, t.hostRejectCount,
           (t.hostSendCount + t.hostRejectCount) > 0 ?
           100.0 * t.hostRejectCount / (t.hostSendCount + t.hostRejectCount) : 0.0);
    if (t.hostSendCount > 0) {
        Tick hostSpan = t.lastHostSend - t.firstHostSend;
        inform("  Host inject:  %.2f ns/pkt (%u pkts in %.1f ns)",
               t.hostSendCount > 1 ?
               (double)hostSpan / (t.hostSendCount - 1) / 1000.0 : 0.0,
               t.hostSendCount, hostSpan / 1000.0);
    }
    // Per-device-port distribution — critical for diagnosing multi-
    // instance fairness. If one port carries most beats while others
    // idle, RTL-side scheduling or request admission is biased.
    // NOTE: these counters are RUN-CUMULATIVE (not reset per-phase),
    // so inter-phase deltas are what matter here.
    if (devicePorts.size() > 1) {
        inform("  Per-port cumulative (read/write accepted | delivered):");
        for (size_t i = 0; i < devicePorts.size(); i++) {
            inform("    port[%lu]: rd=%lu/%lu wr=%lu bresp=%lu "
                   "in-flight=%u",
                   i,
                   perPortReadsAccepted[i],
                   perPortReadsDelivered[i],
                   perPortWritesAccepted[i],
                   perPortBrespsDelivered[i],
                   perPortReadsOut[i]);
        }
    }
    // Per-host-port load balance — if (addr >> 6) % N routing is
    // working, traffic should be roughly even. A heavy skew toward
    // one port means your ORAM address stride isn't 64B-interleaved.
    if (hostPorts.size() > 1) {
        uint64_t total = 0;
        for (auto v : perHostPortSent) total += v;
        if (total > 0) {
            inform("  Host-port load (cumulative):");
            for (size_t i = 0; i < hostPorts.size(); i++) {
                double pct = 100.0 * perHostPortSent[i] / total;
                const char *balWarn = (pct > 60.0) ? " WARN:HOT" : "";
                inform("    hostPort[%lu]: sent=%lu (%.1f%%)%s rejected=%lu",
                       i, perHostPortSent[i], pct, balWarn,
                       perHostPortRejected[i]);
            }
        }
    }
    t.reset();
}

// ====================================================================
//  Statistics
// ====================================================================

CxlModel::CxlStats::CxlStats(CxlModel &owner)
    : statistics::Group(&owner),
      ADD_STAT(readRequests, statistics::units::Count::get(),
               "CXL read requests"),
      ADD_STAT(writeRequests, statistics::units::Count::get(),
               "CXL write requests"),
      ADD_STAT(totalReadBytes, statistics::units::Byte::get(),
               "Total read data bytes"),
      ADD_STAT(totalWriteBytes, statistics::units::Byte::get(),
               "Total write data bytes"),
      ADD_STAT(totalReadFlits, statistics::units::Count::get(),
               "CXL read FLITs sent"),
      ADD_STAT(totalWriteFlits, statistics::units::Count::get(),
               "CXL write FLITs sent"),
      ADD_STAT(totalCompletionFlits, statistics::units::Count::get(),
               "CXL completion FLITs received"),
      ADD_STAT(totalWireBytes, statistics::units::Byte::get(),
               "Total wire bytes (all FLITs)"),
      ADD_STAT(assemblyBottleneckFlits, statistics::units::Count::get(),
               "FLITs where assembly was slower than serialization"),
      ADD_STAT(readLatencyHist, statistics::units::Tick::get(),
               "Read latency histogram (ticks) — samples at tag retirement"),
      ADD_STAT(writeLatencyHist, statistics::units::Tick::get(),
               "Write latency histogram (ticks) — samples at DDR5 commit"),
      ADD_STAT(readCoalesceHist, statistics::units::Count::get(),
               "Read coalesce group size (beats per upstream FLIT)"),
      ADD_STAT(writeCoalesceHist, statistics::units::Count::get(),
               "Write coalesce group size (beats per upstream FLIT)"),
      ADD_STAT(totalReadLatency, statistics::units::Tick::get(),
               "Sum of read latencies (per-group accumulation for average)"),
      ADD_STAT(totalWriteLatency, statistics::units::Tick::get(),
               "Sum of write latencies")
{
    readLatencyHist.init(100);
    writeLatencyHist.init(100);
    readCoalesceHist.init(20);
    writeCoalesceHist.init(20);
    // Review fix #2: REMOVED avgReadLatency and avgWriteLatency Formula
    // stats. They were incorrect: totalWriteLatency accumulates per-beat
    // while writeRequests counts per-group, inflating average by the
    // coalesce factor (16× typical). For accurate per-group read average,
    // see totalReadLatency / readRequests (both now per-group after
    // review fix #2). For per-beat distribution info, use readLatencyHist
    // and writeLatencyHist statistics — gem5 histograms report mean,
    // variance, min, max, percentiles directly.
}

// ====================================================================
//  Diagnostic helpers — structured state dump + invariant checks.
//
//  dumpCxlState emits a multi-line report at `inform` level so it
//  surfaces without needing --debug-flags=CXL. The trigger string
//  prefixes each block so you can grep:
//    grep "CXL-DIAG \[periodic\]"   — routine 1ms snapshots
//    grep "CXL-DIAG \[STUCK\]"      — watchdog fires (investigate these)
//    grep "CXL-DIAG \[ANOMALY\]"    — invariant violations
//  Per-port and per-host-port numbers make imbalance and starvation
//  immediately visible.
// ====================================================================
void
CxlModel::dumpCxlState(const char *trigger)
{
    inform("CXL-DIAG [%s] @%llu (%.1f us):",
           trigger, curTick(), curTick() / 1000000.0);

    // ---- Pipeline queues ----
    unsigned totalRdBuf = 0, totalWrBuf = 0;
    for (auto &q : deviceReadBuffers) totalRdBuf += q.size();
    for (auto &q : deviceWriteBuffers) totalWrBuf += q.size();
    inform("  queues: rdBuf=%u wrBuf=%u upQ=%lu pendHost=%lu "
           "dnQ=%lu respQ=%lu",
           totalRdBuf, totalWrBuf,
           upstreamQueue.size(), pendingHostReqs.size(),
           downstreamQueue.size(), responseQueue.size());

    // ---- Tag pool ----
    double tagUtil = maxTags > 0
        ? 100.0 * outstandingReads.size() / maxTags : 0.0;
    const char *tagWarn = tagUtil > 90.0 ? " WARN:NEAR-FULL" : "";
    inform("  tags: outstanding=%lu free=%lu max=%u util=%.1f%%%s",
           outstandingReads.size(), freeTags.size(), maxTags,
           tagUtil, tagWarn);

    // ---- Writes + FLIT credits (per-port, Phase A.1) ----
    unsigned totalOutWr = 0;
    for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
        totalOutWr += outstandingWrites[sp];
    unsigned totalFlitCr = flitCredits[0];
    unsigned totalFlitCrMax = flitCreditsMax[0];
    inform("  writes (total): outstanding=%u flitCredits=%u/%u",
           totalOutWr, totalFlitCr, totalFlitCrMax);

    // ---- Per-device-port breakdown ----
    // Columns: port | reads_acc | reads_del | outstanding | writes_acc
    //          | bresps | needRetry | cdc_busy (ticks ahead)
    inform("  device ports (%lu):", devicePorts.size());
    for (size_t i = 0; i < devicePorts.size(); i++) {
        Tick cdcLag = endpointBusyUntil[i] > curTick()
            ? endpointBusyUntil[i] - curTick() : 0;
        inform("    port[%lu]: rdAcc=%lu rdDel=%lu out=%u "
               "wrAcc=%lu bresp=%lu needRetry=%d cdcLag=%lluns",
               i,
               perPortReadsAccepted[i],
               perPortReadsDelivered[i],
               perPortReadsOut[i],
               perPortWritesAccepted[i],
               perPortBrespsDelivered[i],
               (int)devicePorts[i]->needRetry,
               cdcLag / 1000);
    }

    // ---- Per-host-port load balance ----
    uint64_t totalSent = 0;
    for (auto v : perHostPortSent) totalSent += v;
    inform("  host ports (%lu) load balance:", hostPorts.size());
    for (size_t i = 0; i < hostPorts.size(); i++) {
        double pct = totalSent > 0
            ? 100.0 * perHostPortSent[i] / totalSent : 0.0;
        // Flag imbalance: >60% of traffic on one port with >1 port
        // configured is almost certainly an address-routing issue.
        const char *balWarn = (hostPorts.size() > 1 && pct > 60.0)
            ? " WARN:IMBALANCED" : "";
        inform("    hostPort[%lu]: sent=%lu reject=%lu (%.1f%%)%s "
               "needRetry=%d",
               i, perHostPortSent[i], perHostPortRejected[i],
               pct, balWarn, (int)hostPorts[i]->needRetry);
    }

    // ---- Coalescing effectiveness so far (across the whole run) ----
    if (rdTracker.flitFlushCount > 0 || wrTracker.flitFlushCount > 0) {
        double rdCoal = rdTracker.flitFlushCount > 0
            ? (double)rdTracker.totalBeatsInFlits / rdTracker.flitFlushCount
            : 0.0;
        double wrCoal = wrTracker.flitFlushCount > 0
            ? (double)wrTracker.totalBeatsInFlits / wrTracker.flitFlushCount
            : 0.0;
        // Flag poor coalescing: <2 beats/group means we're paying per-beat
        // FLIT overhead, which defeats half the purpose of Opt 1/2.
        const char *rdWarn = (rdTracker.flitFlushCount > 10 && rdCoal < 2.0)
            ? " WARN:LOW" : "";
        const char *wrWarn = (wrTracker.flitFlushCount > 10 && wrCoal < 2.0)
            ? " WARN:LOW" : "";
        inform("  coalescing: rd=%.1f beats/group%s (%u groups), "
               "wr=%.1f beats/group%s (%u groups)",
               rdCoal, rdWarn, rdTracker.flitFlushCount,
               wrCoal, wrWarn, wrTracker.flitFlushCount);
    }

    // ---- Progress watchdog ----
    if (lastProgressTick > 0) {
        inform("  lastProgressTick=%llu (%.1f us ago)",
               lastProgressTick,
               (curTick() - lastProgressTick) / 1000000.0);
    }
}

void
CxlModel::checkInvariants(const char *where)
{
    // Bug #11 fix: always run the two CHEAP invariants (tag conservation
    // and per-port sum). These are O(1) and O(numPorts) respectively —
    // negligible cost even on the hot path, and surface tag leaks from
    // Bug #3 immediately instead of requiring trace flags.
    //
    // Expensive diagnostics (detailed anomaly messages) still gate on
    // DTRACE(CXL) below.
    size_t tagSum = freeTags.size() + outstandingReads.size();
    if (tagSum != maxTags) {
        // Tag conservation violation — always report.
        warn_once("CXL-INVARIANT %s: tag leak: free=%lu + out=%lu = %lu, "
                  "expected %u\n",
                  where, freeTags.size(), outstandingReads.size(),
                  tagSum, maxTags);
    }
    uint64_t portOutSum = 0;
    for (auto v : perPortReadsOut) portOutSum += v;
    if (portOutSum != outstandingReads.size()) {
        // Per-port accounting violation — always report.
        warn_once("CXL-INVARIANT %s: perPortReadsOut sum=%lu != "
                  "outstandingReads=%lu\n",
                  where, portOutSum, outstandingReads.size());
    }

    // Detailed diagnostics only with trace flag (preserves old behavior
    // for deeper debugging without flooding production logs).
    if (!DTRACE(CXL)) return;

    unsigned totalWrBufInv = 0;
    for (auto &q : deviceWriteBuffers) totalWrBufInv += q.size();
    unsigned totalOutWrInv = 0;
    for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
        totalOutWrInv += outstandingWrites[sp];
    if (totalOutWrInv < totalWrBufInv) {
        inform("CXL-DIAG [ANOMALY] %s: sum(outstandingWrites)=%u < "
               "wrBuf=%u (expected sum >= wrBuf)",
               where, totalOutWrInv, totalWrBufInv);
    }

    if (hostPorts.empty()) {
        inform("CXL-DIAG [ANOMALY] %s: hostPorts is empty post-init", where);
    }
}

// ====================================================================
//  Per-port stuck dump — comprehensive single-port state in one call.
//
//  Triggered by checkPortStalls() when a specific port has work pending
//  but hasn't made progress in >5ms. Shows everything needed to diagnose
//  WHERE the stall is: which queue is full, what the last drain time
//  was, what the pending event schedules look like.
//
//  Grep "CXL-PORT-STUCK" to find these in logs.
// ====================================================================
void
CxlModel::dumpPortState(unsigned p, const char *where)
{
    if (p >= devicePorts.size()) return;

    Tick rdLast = (p < lastReadBufferTicks.size()) ? lastReadBufferTicks[p] : 0;
    Tick wrLast = (p < lastWriteBufferTicks.size()) ? lastWriteBufferTicks[p] : 0;
    Tick drainLast = (p < perPortLastDrainTick.size())
                       ? perPortLastDrainTick[p] : 0;
    Tick now = curTick();

    inform("CXL-PORT-STUCK [%s] port=%u @%llu (%.2f ms idle since drain)",
           where, p, now, (now - drainLast) / 1e9);
    inform("  queues: rdBuf=%lu wrBuf=%lu upQ=%lu dnQ=%lu hostQ=%lu respQ=%lu",
           p < deviceReadBuffers.size() ? deviceReadBuffers[p].size() : 0,
           p < deviceWriteBuffers.size() ? deviceWriteBuffers[p].size() : 0,
           p < upstreamQueue.size() ? upstreamQueue[p].size() : 0,
           p < downstreamQueue.size() ? downstreamQueue[p].size() : 0,
           p < pendingHostReqs.size() ? pendingHostReqs[p].size() : 0,
           p < responseQueue.size() ? responseQueue[p].size() : 0);
    inform("  resources: flitCredits=%u/%u outWr=%u perPortRds=%u "
           "compBuf=%u defCred=%lu",
           p < flitCredits.size() ? flitCredits[0] : 0,
           p < flitCreditsMax.size() ? flitCreditsMax[0] : 0,
           p < outstandingWrites.size() ? outstandingWrites[p] : 0,
           p < perPortReadsOut.size() ? perPortReadsOut[p] : 0,
           p < completionBufferOccupied.size()
               ? completionBufferOccupied[p] : 0,
           p < deferredFlitCredits.size() ? deferredFlitCredits[p].size() : 0);
    inform("  timing: lastRdRecv=%llu (%.1f ms ago) lastWrRecv=%llu "
           "(%.1f ms ago) lastDrain=%llu",
           rdLast, (now - rdLast) / 1e9,
           wrLast, (now - wrLast) / 1e9,
           drainLast);
    inform("  events: drainSched=%d upSched=%d dnSched=%d hostSched=%d "
           "respSched=%d credSched=%d",
           (int)drainDeviceEvent.scheduled(),
           (int)upstreamEvent.scheduled(),
           (int)downstreamEvent.scheduled(),
           (int)hostSendEvent.scheduled(),
           (int)responseEvent.scheduled(),
           (int)flitCreditEvent.scheduled());

    // If buffers have entries but nothing is scheduled to drain them,
    // that's the smoking gun for a coalesce-window or reschedule bug.
    bool rdBufStuck = (p < deviceReadBuffers.size()) &&
                      !deviceReadBuffers[p].empty() &&
                      !drainDeviceEvent.scheduled();
    bool wrBufStuck = (p < deviceWriteBuffers.size()) &&
                      !deviceWriteBuffers[p].empty() &&
                      !drainDeviceEvent.scheduled();
    if (rdBufStuck || wrBufStuck) {
        inform("  >>> SMOKING GUN: buffer has entries but drainDeviceEvent "
               "is NOT scheduled. rdStuck=%d wrStuck=%d. "
               "This is a reschedule bug — packet will sit forever.",
               (int)rdBufStuck, (int)wrBufStuck);
    }

    // FLIT credit exhaustion check.
    if (p < flitCredits.size() && p < flitCreditsMax.size() &&
        flitCreditsMax[0] > 0 && flitCredits[0] == 0) {
        inform("  >>> CREDIT-STARVED: port=%u has 0/%u FLIT credits. "
               "Pending returns: %lu. If 0, credit return path is broken.",
               p, flitCreditsMax[0],
               (p < deferredFlitCredits.size())
                   ? deferredFlitCredits[p].size() : 0);
    }
}

// ====================================================================
//  checkPortStalls — periodic per-port stuck detection.
//
//  Replaces the global lastProgressTick check that was masking
//  per-port stalls (port 0 making progress would mark the model
//  as "alive" even if port 1 was frozen).
//
//  Called from processDownstreamQueue's existing periodic-dump path.
//  Throttled to fire at most once per 5ms to avoid log spam.
// ====================================================================
void
CxlModel::checkPortStalls()
{
    Tick now = curTick();
    if (now - lastPortStuckCheckTick < 5000000) return;  // 5ms throttle
    lastPortStuckCheckTick = now;

    for (unsigned p = 0; p < devicePorts.size(); p++) {
        bool portHasWork =
            (p < deviceReadBuffers.size() && !deviceReadBuffers[p].empty()) ||
            (p < deviceWriteBuffers.size() && !deviceWriteBuffers[p].empty()) ||
            (p < upstreamQueue.size() && !upstreamQueue[p].empty()) ||
            (p < downstreamQueue.size() && !downstreamQueue[p].empty()) ||
            (p < pendingHostReqs.size() && !pendingHostReqs[p].empty()) ||
            (p < responseQueue.size() && !responseQueue[p].empty());

        if (!portHasWork) continue;

        Tick lastDrain = (p < perPortLastDrainTick.size())
                           ? perPortLastDrainTick[p] : 0;
        // If the port has work and a drain has happened before, check
        // staleness. If lastDrain==0 the port has work but never drained
        // (post-init traffic that's been stuck since arrival); also a
        // stuck case.
        if (lastDrain == 0 || (now - lastDrain) > 5000000) {
            dumpPortState(p, "5ms-stall");
        }
    }
}

} // namespace gem5