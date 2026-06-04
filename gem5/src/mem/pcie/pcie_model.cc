/*
 * pcie_model.cc — TLP-level pipelined PCIe model for gem5 v25.1
 *
 * Real TLP byte construction per PCIe Base Spec 5.0:
 *   DW0: fmt[7:5] | type[4:0] | T9 | TC[6:4] | T8 | Attr | LN | TH
 *        | TD | EP | Attr | AT | Length[9:0]
 *   DW1: Requester ID[15:0] | Tag[7:0] | LastBE[3:0] | FirstBE[3:0]
 *   DW2: Address[31:2] | R | R   (3DW header)
 *   DW2+3: Address[63:2] | R | R  (4DW header)
 *
 * Assembly pipeline models Xilinx UltraScale+ PCIe hard block:
 *   - Core clock: 250MHz (Gen3) or 500MHz (Gen4/5)
 *   - Datapath width: 256 bits (32 bytes)
 *   - Assembly cycles = ceil(TLP total bytes / datapath width)
 *   - Per-TLP delay = max(assembly_time, serialization_time) + ACK
 */

#include "mem/pcie/pcie_model.hh"

#include <algorithm>
#include <cstring>

#include "base/cprintf.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/PCIe.hh"
#include "sim/core.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

// ====================================================================
//  Construction
// ====================================================================

PCIeModel::PCIeModel(const Params &p)
    : ClockedObject(p),
      // .hh:58 — TLP Parameters
      mps(p.mps),
      mrrs(p.mrrs),
      maxTags(p.max_tags),
      rcb(p.rcb),
      // .hh:63-64 — Latency Parameters
      readLatency(p.read_latency),
      writeLatency(p.write_latency),
      rcLatency(p.rc_latency),
      rcThroughputDelay(p.rc_throughput_delay),
      fpgaClockPeriod(p.fpga_clock_period),
      // .hh:70-72 — Data Link Layer
      lcrcBytes(p.lcrc_bytes),
      framingBytes(p.framing_bytes),
      ecrcBytes(p.ecrc_bytes),
      dllpAckDelay(p.dllp_ack_delay),
      creditReturnDelay(p.credit_return_delay),
      // .hh:77-80 — AXI Bridge + PCIe Core Pipeline
      bridgePipelineDelay(p.bridge_pipeline_delay),
      hostInjectInterval(p.host_inject_interval),
      pcieCoreWidthBytes(p.pcie_core_width),
      // .hh:198 — Credit return event (before rrbDepth per declaration order)
      creditReturnEvent([this]{ processDeferredCredits(); }, name()),
      // .hh:229-239 — Review items (RRB, core-clock gap, credit cadence)
      rrbDepth(p.rrb_depth),
      coreClockPeriod(p.pcie_tlp_gap),
      lastUpstreamEmit(0),
      lastDownstreamEmit(0),
      creditReturnPeriod(p.credit_return_period),
      nextCreditReturnTick(0),
      // .hh:242-246 — Completion timeout, header size
      completionTimeoutTicks(p.completion_timeout),
      use64bitAddr(p.use_64bit_addr),
      // .hh:255 — Completion reorder buffer
      completionReorderDepth(p.completion_reorder_depth),
      // .hh:282-285 — Link serialization shared state
      upstreamBusyUntil(0),
      downstreamBusyUntil(0),
      rcDownstreamBusyUntil(0),
      rcUpstreamBusyUntil(0),
      // .hh:370-380 — Pipeline events (declaration order)
      drainDeviceEvent([this]{ drainDeviceRequests(); }, name()),
      upstreamEvent([this]{ processUpstreamQueue(); }, name()),
      downstreamEvent([this]{ processDownstreamQueue(); }, name()),
      deviceRetryEvent([this]{ retryDeviceSend(); }, name()),
      hostRetryEvent([this]{ retryHostSend(); }, name()),
      // .hh:415 — Response event
      responseEvent([this]{ trySendResponses(); }, name()),
      // .hh:434 — Diagnostic event
      diagEvent([this]{ dumpDiagnostics(); }, name()),
      // .hh:457 — Host send event
      hostSendEvent([this]{ trySendToHost(); }, name()),
      // .hh:512-517 — Stats and endpoint identity
      stats(*this),
      requesterId(p.requester_id),
      completerId(p.completer_id)
{
    // Review fix #5: fpgaClockPeriod zero guard. Division by zero in
    // write-commit inform path if user passes fpga_clock_period=0.
    if (fpgaClockPeriod == 0) {
        warn("PCIeModel: fpga_clock_period=0 is invalid; clamping to 1 tick. "
             "Set a realistic value (e.g. 3333ps for 300MHz).\n");
        fpgaClockPeriod = 1;
    }

    // Create device ports (one per connected ORAM instance)
    for (int i = 0; i < p.port_device_side_port_connection_count; i++) {
        devicePorts.push_back(new DeviceSidePort(
            csprintf("%s.device_side_port[%d]", name(), i), *this, i));
    }

    // Create host ports (one per DDR5 xbar input for parallel access)
    for (int i = 0; i < p.port_host_side_port_connection_count; i++) {
        hostPorts.push_back(new HostSidePort(
            csprintf("%s.host_side_port[%d]", name(), i), *this, i));
    }

    // Per-device-port CDC serialization (models independent bridge FIFOs)
    bridgeBusyUntil.resize(devicePorts.size(), 0);
    // Per-device-port outstanding read counters
    perPortReadsOut.resize(devicePorts.size(), 0);
    // Per-device-port request buffers (per-channel FIFOs in real hw)
    deviceReadBuffers.resize(devicePorts.size());
    deviceWriteBuffers.resize(devicePorts.size());
    lastReadBufferTicks.resize(devicePorts.size(), 0);
    lastWriteBufferTicks.resize(devicePorts.size(), 0);

    // Phase A.1: per-port flow-control resources. Each device-side
    // port gets its own credit pool, write-budget, TLP queues, host
    // injection queue, response queue, deferred-credit return tracker,
    // and RRB occupancy. Pools are full per port (NOT split) — each
    // AXI master has its own controller channel with the full credit
    // budget.
    outstandingWrites.resize(devicePorts.size(), 0);
    upstreamQueue.resize(devicePorts.size());
    downstreamQueue.resize(devicePorts.size());
    pendingHostReqs.resize(devicePorts.size());
    responseQueue.resize(devicePorts.size());
    deferredCredits.resize(devicePorts.size());
    rrbOccupied.resize(devicePorts.size(), 0);
    credits.resize(devicePorts.size());

    // Phase B diagnostic: per-port last-drain tracking.
    perPortLastDrainTick.resize(devicePorts.size(), 0);

    linkParams.gen = p.gen;
    linkParams.lanes = p.lanes;
    computeLinkParams();

    burstWindowTicks = p.burst_window;
    // Initialize to a value that guarantees the first request pays full
    // RC latency. Intentional unsigned wrap — produces a large value
    // so (now - lastUpstreamRcTick) > burstWindowTicks on first check.
    lastUpstreamRcTick = (Tick)0 - burstWindowTicks - 1;
    lastDownstreamRcTick = (Tick)0 - burstWindowTicks - 1;

    // PCIe core period from frequency
    // gem5 Frequency param is stored as period in ticks (ps)
    // 500MHz → 2000 ticks (2ns)
    pcieCorePeriod = p.pcie_core_clock;

    if (maxTags > 4096) {
        warn("PCIe: max_tags=%u > 4096: capping to 4096\n", maxTags);
        maxTags = 4096;
    }

    for (uint16_t t = 0; t < maxTags; ++t)
        freeTags.push_back(t);

    // Phase A.1: SHARED credit pool. All device ports compete for one
    // link-level credit budget (stored in credits[0]). This models real
    // PCIe where the RC advertises one credit set per VC, shared by all
    // functions on the link. Credits consumed by port A reduce the pool
    // for all other ports — natural contention at high N.
    credits[0].phMax   = p.credits_ph;
    credits[0].pdMax   = p.credits_pd;
    credits[0].nphMax  = p.credits_nph;
    credits[0].npdMax  = p.credits_npd;
    credits[0].cplhMax = p.credits_cplh;
    credits[0].cpldMax = p.credits_cpld;
    unsigned N = devicePorts.size();
    initCredits();
    // outstandingWrites resized to per-port vector above.
    pendingDeviceRetry = false;
    nextRetryPort = 0;
    maxOutstanding = p.max_outstanding;
    maxOutstandingWrites = p.max_outstanding_writes;

    DPRINTF(PCIe, "PCIe core: period=%llu ticks (%.1f MHz), "
            "width=%u bytes\n",
            pcieCorePeriod,
            1e12 / (double)pcieCorePeriod / 1e6,
            pcieCoreWidthBytes);

    // Phase A.1: per-port resource sizing summary. Visible in inform
    // log to verify each device-side port has its own credit pool +
    // RRB + write budget + per-port queue set.
    inform("PCIe Phase A.1: %lu device port(s), %lu host port(s); "
           "per-port credits ph=%d pd=%d nph=%d cplh=%d cpld=%d; "
           "per-port rrb_depth=%u max_outstanding_writes=%u "
           "max_outstanding_reads=%u; shared tag pool=%u",
           devicePorts.size(), hostPorts.size(),
           p.credits_ph, p.credits_pd, p.credits_nph,
           p.credits_cplh, p.credits_cpld,
           rrbDepth, maxOutstandingWrites, maxOutstanding, maxTags);
}

void PCIeModel::init()
{
    ClockedObject::init();
    if (devicePorts.empty())
        fatal("PCIeModel %s: no device_side_port connected\n", name());
    // Review fix #4: add fatal on empty hostPorts (CXL has this guard;
    // PCIe missed it). Later code unconditionally derefs hostPorts[0]
    // for functional access and getAddrRanges, so an empty vector would
    // crash far from the root cause.
    if (hostPorts.empty())
        fatal("PCIeModel %s: no host_side_port connected\n", name());
    for (auto *dp : devicePorts)
        if (!dp->isConnected())
            fatal("PCIeModel %s: device_side_port not connected\n", name());
    for (auto *hp : hostPorts)
        if (!hp->isConnected())
            fatal("PCIeModel %s: host_side_port not connected\n", name());

    // Schedule periodic diagnostic for deadlock debugging
    schedule(diagEvent, 10000000);  // first dump at 10M ticks (~3000 cyc)
}

void PCIeModel::dumpDiagnostics()
{
    unsigned totalBufReads = 0, totalBufWrites = 0;
    for (auto &q : deviceReadBuffers) totalBufReads += q.size();
    for (auto &q : deviceWriteBuffers) totalBufWrites += q.size();

    unsigned hostBlocked = 0;
    for (auto *hp : hostPorts)
        if (hp->needRetry) hostBlocked++;

    unsigned devNeedRetry = 0;
    for (auto *dp : devicePorts)
        if (dp->needRetry) devNeedRetry++;

    unsigned totalCompBuf = 0;
    for (auto &kv : completionBuffer)
        totalCompBuf += kv.second.size();

    // Phase A.1: aggregate per-port state across ports for the diag line.
    unsigned totalOutWr = 0;
    for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
        totalOutWr += outstandingWrites[sp];
    unsigned totalUpQ = 0, totalDnQ = 0, totalHostQ = 0, totalRespQ = 0;
    for (unsigned sp = 0; sp < upstreamQueue.size(); sp++) {
        totalUpQ += upstreamQueue[sp].size();
        totalDnQ += downstreamQueue[sp].size();
        totalHostQ += pendingHostReqs[sp].size();
        totalRespQ += responseQueue[sp].size();
    }
    int totalNph = credits[0].nphCredits;
    int totalNphMax = credits[0].nphMax;
    // A9: invariant — sum of per-port outstanding reads must equal
    // the global outstandingReads map size. Catches refactor regressions.
    {
        unsigned sumPerPortRds = 0;
        for (auto v : perPortReadsOut) sumPerPortRds += v;
        assert(sumPerPortRds == outstandingReads.size() &&
               "perPortReadsOut / outstandingReads mismatch");
    }
    inform("PCIe DIAG @%llu: "
           "rdBuf=%u wrBuf=%u outstanding=%u outWr=%u "
           "freeTags=%u upQ=%u dnQ=%u hostQ=%u respQ=%u compBuf=%u "
           "upBusy=%llu dnBusy=%llu "
           "hostBlk=%u devRetry=%u pendRetry=%d "
           "nph=%d/%d perPort0=%u perPort1=%u "
           "evts: up=%d dn=%d host=%d resp=%d drain=%d cred=%d",
           curTick(),
           totalBufReads, totalBufWrites,
           (unsigned)outstandingReads.size(), totalOutWr,
           (unsigned)freeTags.size(),
           totalUpQ, totalDnQ, totalHostQ, totalRespQ,
           totalCompBuf,
           upstreamBusyUntil, downstreamBusyUntil,
           hostBlocked, devNeedRetry, (int)pendingDeviceRetry,
           totalNph, totalNphMax,
           perPortReadsOut.size() > 0 ? perPortReadsOut[0] : 0,
           perPortReadsOut.size() > 1 ? perPortReadsOut[1] : 0,
           (int)upstreamEvent.scheduled(),
           (int)downstreamEvent.scheduled(),
           (int)hostSendEvent.scheduled(),
           (int)responseEvent.scheduled(),
           (int)drainDeviceEvent.scheduled(),
           (int)creditReturnEvent.scheduled());

    // Review item #7: scan for timed-out tags. PCIe spec §2.8 says
    // tag is reclaimed + error if no completion within the timeout
    // (Range B default: 10-50 ms). Never fires in healthy runs;
    // detects deadlock/drop bugs. Advisory only — we warn but don't
    // force-release, since force-releasing could mask real bugs.
    if (completionTimeoutTicks > 0) {
        Tick now = curTick();
        for (auto &kv : tagIssueDeadline) {
            if (now > kv.second) {
                warn_once("PCIe: tag %u exceeded completion timeout "
                          "(issued before %llu, now %llu). Possible "
                          "deadlock or dropped completion.\n",
                          kv.first, kv.second - completionTimeoutTicks, now);
                break;  // one warn per scan — flood-prevent
            }
        }
    }

    // Phase B diagnostic: per-port stall detection. Catches per-port
    // freezes that the global tag-deadline check above would miss
    // (e.g., CPU port stuck in coalesce wait while ORAM port still
    // makes progress).
    checkPortStalls();

    schedule(diagEvent, curTick() + 10000000);  // every 10M ticks
}

Port &
PCIeModel::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "device_side_port") {
        if (idx < devicePorts.size())
            return *devicePorts[idx];
        fatal("PCIeModel %s: device_side_port idx %d out of range\n",
              name(), idx);
    }
    else if (if_name == "host_side_port") {
        if (idx < hostPorts.size())
            return *hostPorts[idx];
        fatal("PCIeModel %s: host_side_port idx %d out of range\n",
              name(), idx);
    }
    else return ClockedObject::getPort(if_name, idx);
}

// ====================================================================
//  Link Parameters
// ====================================================================

void PCIeModel::computeLinkParams()
{
    switch (linkParams.gen) {
      case 3: linkParams.transferRate = 8.0;  break;
      case 4: linkParams.transferRate = 16.0; break;
      case 5: linkParams.transferRate = 32.0; break;
      default: fatal("PCIeModel: unsupported gen %d\n", linkParams.gen);
    }
    // Gen3 uses 8b/10b encoding (0.8 efficiency)
    // Gen4+ uses 128b/130b encoding (~0.985 efficiency)
    switch (linkParams.gen) {
      case 3:
        linkParams.encodingEff = 8.0 / 10.0;
        break;
      default:
        linkParams.encodingEff = 128.0 / 130.0;
        break;
    }
    linkParams.effectiveBwBytesPerSec = (uint64_t)(
        linkParams.transferRate * 1e9 * linkParams.lanes / 8.0 *
        linkParams.encodingEff);

    linkParams.byteSerializationDelay =
        (linkParams.effectiveBwBytesPerSec > 0)
        ? std::max((Tick)1,
                   sim_clock::as_int::s / linkParams.effectiveBwBytesPerSec)
        : (Tick)1;

    DPRINTF(PCIe, "PCIe Gen%d x%d: eff BW=%llu B/s, "
            "byte_delay=%llu ticks\n",
            linkParams.gen, linkParams.lanes,
            linkParams.effectiveBwBytesPerSec,
            linkParams.byteSerializationDelay);
}

// ====================================================================
//  TLP Byte Construction Helpers
// ====================================================================

void PCIeModel::retryStarvedPorts()
{
    unsigned n = devicePorts.size();
    if (n == 0) return;
    for (unsigned i = 0; i < n; i++) {
        unsigned idx = (nextRetryPort + i) % n;
        auto *dp = devicePorts[idx];
        if (dp->needRetry) {
            dp->needRetry = false;
            dp->sendRetryReq();
            nextRetryPort = (idx + 1) % n;
            return;  // one port per call
        }
    }
}

unsigned PCIeModel::computeWireBytes(unsigned totalBytes) const
{ return totalBytes + lcrcBytes + framingBytes + ecrcBytes; }

unsigned PCIeModel::computePaddingBytes(Addr addr) const
{
    // PCIe payload must be DW-aligned. If the starting address
    // is not DW-aligned, padding bytes are inserted before payload.
    // For ORAM, addresses are always 32B-aligned (AXI), so padding = 0.
    // But we model it correctly for generality.
    return (4 - (addr & 0x3)) & 0x3;
}

Tick PCIeModel::assemblyDelay(unsigned totalBytes) const
{
    // PCIe hard block assembles TLPs in a pipelined fashion.
    // With TLP packing, the next TLP starts immediately after
    // the previous one — no wasted cycles on partial beats.
    // Throughput = pcieCoreWidthBytes × (1/pcieCorePeriod).
    return (Tick)totalBytes * pcieCorePeriod / pcieCoreWidthBytes;
}

void PCIeModel::buildTlpHeader(TlpPacket &tlp)
{
    tlp.header.resize(tlp.headerBytes, 0);
    bool is4dw = (tlp.headerBytes == 16);

    // DW0: fmt[7:5] | type[4:0] | TC=0 | Length
    uint8_t dw0_byte0 = (tlp.fmt << 5) | (tlp.type & 0x1F);
    uint16_t lengthDW = tlp.length;  // 0 means 1024 DW

    tlp.header[0] = dw0_byte0;
    tlp.header[1] = 0; // TC=0, Attr=0
    tlp.header[2] = (lengthDW >> 8) & 0x03;
    tlp.header[3] = lengthDW & 0xFF;

    // DW1: Requester ID | Tag | LastBE | FirstBE
    tlp.header[4] = (tlp.requesterId >> 8) & 0xFF;
    tlp.header[5] = tlp.requesterId & 0xFF;
    tlp.header[6] = tlp.tag & 0xFF;

    // FirstBE and LastBE
    if (tlp.payloadBytes > 0 || tlp.type == TYPE_MRD) {
        unsigned bytes = (tlp.payloadBytes > 0) ? tlp.payloadBytes
                         : (tlp.length * 4);
        uint8_t firstBE = 0xF; // all bytes valid
        uint8_t lastBE = (bytes > 4) ? 0xF : 0x0;
        tlp.header[7] = (lastBE << 4) | firstBE;
    } else {
        tlp.header[7] = 0;
    }

    if (tlp.type == TYPE_CPL || tlp.type == TYPE_CPLD) {
        // Completion header format is different:
        // DW1: Completer ID | Status | BCM | ByteCount
        tlp.header[4] = (tlp.completerId >> 8) & 0xFF;
        tlp.header[5] = tlp.completerId & 0xFF;
        // Status = 0 (successful), BCM = 0
        tlp.header[6] = (tlp.byteCount >> 8) & 0x0F;
        tlp.header[7] = tlp.byteCount & 0xFF;

        // DW2: Requester ID | Tag | LowerAddr
        tlp.header[8] = (tlp.requesterId >> 8) & 0xFF;
        tlp.header[9] = tlp.requesterId & 0xFF;
        tlp.header[10] = tlp.tag & 0xFF;
        tlp.header[11] = tlp.lowerAddr & 0x7F;
    } else {
        // Memory request: DW2 (and DW3 for 4DW) = address
        if (is4dw) {
            tlp.header[8]  = (tlp.addr >> 56) & 0xFF;
            tlp.header[9]  = (tlp.addr >> 48) & 0xFF;
            tlp.header[10] = (tlp.addr >> 40) & 0xFF;
            tlp.header[11] = (tlp.addr >> 32) & 0xFF;
            tlp.header[12] = (tlp.addr >> 24) & 0xFF;
            tlp.header[13] = (tlp.addr >> 16) & 0xFF;
            tlp.header[14] = (tlp.addr >> 8) & 0xFF;
            tlp.header[15] = tlp.addr & 0xFC; // bits[1:0] = 0
        } else {
            tlp.header[8]  = (tlp.addr >> 24) & 0xFF;
            tlp.header[9]  = (tlp.addr >> 16) & 0xFF;
            tlp.header[10] = (tlp.addr >> 8) & 0xFF;
            tlp.header[11] = tlp.addr & 0xFC;
        }
    }
}

PCIeModel::TlpPacket
PCIeModel::buildMemReadTlp(Addr addr, unsigned lengthBytes,
                            uint16_t tag, PacketPtr pkt)
{
    TlpPacket tlp;
    // Review item #8: force 4DW when use_64bit_addr, else auto-detect.
    bool is4dw = use64bitAddr || (addr >= 0x100000000ULL);

    tlp.fmt = is4dw ? FMT_4DW_NODATA : FMT_3DW_NODATA;
    tlp.type = TYPE_MRD;
    tlp.requesterId = requesterId;
    tlp.completerId = 0;
    tlp.tag = tag;
    assert((lengthBytes % 4) == 0 &&
           "PCIe memory read length must be DW-aligned (multiple of 4)");
    tlp.length = lengthBytes / 4; // in DW
    tlp.addr = addr;
    tlp.byteCount = 0;
    tlp.lowerAddr = 0;
    tlp.paddingBytes = 0; // reads have no payload
    tlp.payloadBytes = 0;
    tlp.headerBytes = is4dw ? 16 : 12;
    tlp.totalBytes = tlp.headerBytes;
    tlp.wireBytes = computeWireBytes(tlp.totalBytes);
    tlp.origPkt = pkt;
    tlp.creationTick = curTick();
    tlp.issueTick = curTick();
    tlp.isLastCompletion = false;
    tlp.origSize = lengthBytes;

    buildTlpHeader(tlp);
    return tlp;
}

PCIeModel::TlpPacket
PCIeModel::buildMemWriteTlp(Addr addr, unsigned payloadBytes,
                              PacketPtr pkt)
{
    TlpPacket tlp;
    // Review item #8: use_64bit_addr forces 4DW header when set;
    // otherwise auto-detect based on address range. Your ORAM layout
    // places DDR5 above 0x100000000ULL so this always picks 4DW.
    bool is4dw = use64bitAddr || (addr >= 0x100000000ULL);

    tlp.fmt = is4dw ? FMT_4DW_DATA : FMT_3DW_DATA;
    tlp.type = TYPE_MWR;
    tlp.requesterId = requesterId;
    tlp.completerId = 0;
    tlp.tag = 0;
    tlp.length = payloadBytes / 4;
    tlp.addr = addr;
    tlp.byteCount = 0;
    tlp.lowerAddr = 0;
    tlp.paddingBytes = computePaddingBytes(addr);
    tlp.payloadBytes = payloadBytes;
    tlp.headerBytes = is4dw ? 16 : 12;
    tlp.totalBytes = tlp.headerBytes + tlp.paddingBytes + payloadBytes;
    tlp.wireBytes = computeWireBytes(tlp.totalBytes);
    tlp.origPkt = pkt;
    tlp.creationTick = curTick();
    tlp.issueTick = curTick();
    tlp.isLastCompletion = false;
    tlp.origSize = payloadBytes;

    buildTlpHeader(tlp);

    stats.totalPaddingBytes += tlp.paddingBytes;
    return tlp;
}

PCIeModel::TlpPacket
PCIeModel::buildCompletionTlp(Addr addr, unsigned payloadBytes,
                                uint16_t tag, unsigned totalReqBytes,
                                unsigned completedSoFar,
                                PacketPtr pkt, bool isLast)
{
    TlpPacket tlp;

    tlp.fmt = FMT_3DW_DATA;
    tlp.type = TYPE_CPLD;
    tlp.requesterId = requesterId;
    tlp.completerId = completerId;
    tlp.tag = tag;
    assert((payloadBytes % 4) == 0 &&
           "CplD payload must be DW-aligned (multiple of 4)");
    tlp.length = payloadBytes / 4;
    tlp.addr = addr;
    tlp.byteCount = totalReqBytes - completedSoFar;
    tlp.lowerAddr = addr & 0x7F;
    tlp.paddingBytes = 0; // completions don't have alignment padding
    tlp.payloadBytes = payloadBytes;
    tlp.headerBytes = 12; // completions always 3DW
    tlp.totalBytes = 12 + payloadBytes;
    tlp.wireBytes = computeWireBytes(tlp.totalBytes);
    tlp.origPkt = isLast ? pkt : nullptr;
    tlp.creationTick = curTick();
    tlp.isLastCompletion = isLast;
    tlp.origSize = totalReqBytes;

    buildTlpHeader(tlp);
    return tlp;
}

// PCIe Bug #13 fix: deleted orphan buildCompletionNoDataTlp. After
// Bug #3 removed the dead BRESP downstream branch, this function had
// no callers. It was never invoked anywhere in the codebase, so
// removing it is safe and eliminates an undefined-reference risk.
// If you ever need non-data completions (e.g., for Memory Fence
// responses on a non-R/W path), reinstate this builder along with
// proper SenderState plumbing for the non-R/W case.

// ====================================================================
//  Credits
// ====================================================================

void PCIeModel::initCredits()
{
    // Shared pool: only credits[0] holds the link budget.
    auto &c = credits[0];
    c.phCredits = c.phMax;     c.pdCredits = c.pdMax;
    c.nphCredits = c.nphMax;   c.npdCredits = c.npdMax;
    c.cplhCredits = c.cplhMax; c.cpldCredits = c.cpldMax;
    inform("PCIe credits initialized (shared pool, %lu ports): "
           "ph=%d pd=%d nph=%d npd=%d cplh=%d cpld=%d",
           credits.size(),
           c.phMax, c.pdMax, c.nphMax,
           c.npdMax, c.cplhMax, c.cpldMax);
}

int PCIeModel::dataCreditsNeeded(unsigned pb) const
{
    if (pb == 0) return 0;
    // PCIe spec §2.6: FC data credit unit = 4 DW = 16 bytes, always.
    return (pb + 15) / 16;
}

bool PCIeModel::hasPostedCredits(unsigned sp, int dc) const
{
    panic_if(sp >= credits.size(),
             "PCIe hasPostedCredits: bad sp=%u (max=%lu)", sp, credits.size());
    return credits[0].phCredits >= 1 && credits[0].pdCredits >= dc;
}
bool PCIeModel::hasNonPostedCredits(unsigned sp) const
{
    panic_if(sp >= credits.size(),
             "PCIe hasNonPostedCredits: bad sp=%u (max=%lu)",
             sp, credits.size());
    return credits[0].nphCredits >= 1;
}
bool PCIeModel::hasCompletionCredits(unsigned sp, int dc) const
{
    panic_if(sp >= credits.size(),
             "PCIe hasCompletionCredits: bad sp=%u (max=%lu)",
             sp, credits.size());
    return credits[0].cplhCredits >= 1 && credits[0].cpldCredits >= dc;
}
void PCIeModel::consumePostedCredits(unsigned sp, int dc)
{
    panic_if(sp >= credits.size(),
             "PCIe consumePostedCredits: bad sp=%u dc=%d (max=%lu)",
             sp, dc, credits.size());
    credits[0].phCredits--;
    credits[0].pdCredits -= dc;
    DPRINTF(PCIe, "  [CR-CONS] port=%u Posted ph=1 pd=%d → ph=%d pd=%d\n",
            sp, dc, credits[0].phCredits, credits[0].pdCredits);
}
void PCIeModel::consumeNonPostedCredits(unsigned sp)
{
    panic_if(sp >= credits.size(),
             "PCIe consumeNonPostedCredits: bad sp=%u (max=%lu)",
             sp, credits.size());
    credits[0].nphCredits--;
    DPRINTF(PCIe, "  [CR-CONS] port=%u NonPosted nph=1 → nph=%d\n",
            sp, credits[0].nphCredits);
}
void PCIeModel::consumeCompletionCredits(unsigned sp, int dc)
{
    panic_if(sp >= credits.size(),
             "PCIe consumeCompletionCredits: bad sp=%u dc=%d (max=%lu)",
             sp, dc, credits.size());
    credits[0].cplhCredits--;
    credits[0].cpldCredits -= dc;
    DPRINTF(PCIe, "  [CR-CONS] port=%u Cpl cplh=1 cpld=%d → "
            "cplh=%d cpld=%d\n",
            sp, dc, credits[0].cplhCredits, credits[0].cpldCredits);
}
void PCIeModel::returnPostedCredits(unsigned sp, int dc)
{
    panic_if(sp >= credits.size(),
             "PCIe returnPostedCredits: bad sp=%u dc=%d (max=%lu)",
             sp, dc, credits.size());
    credits[0].phCredits++;
    credits[0].pdCredits += dc;
    wakeupDrainOnCreditReturn();
}
void PCIeModel::returnNonPostedCredits(unsigned sp)
{
    panic_if(sp >= credits.size(),
             "PCIe returnNonPostedCredits: bad sp=%u (max=%lu)",
             sp, credits.size());
    credits[0].nphCredits++;
    wakeupDrainOnCreditReturn();
}
void PCIeModel::returnCompletionCredits(unsigned sp, int dc)
{
    panic_if(sp >= credits.size(),
             "PCIe returnCompletionCredits: bad sp=%u dc=%d (max=%lu)",
             sp, dc, credits.size());
    credits[0].cplhCredits++;
    credits[0].cpldCredits += dc;
    wakeupDrainOnCreditReturn();
}

// Shared credit pool: when credits return, ANY port's buffered
// requests may now be processable. Reschedule the drain event.
void PCIeModel::wakeupDrainOnCreditReturn()
{
    if (drainDeviceEvent.scheduled()) return;
    for (unsigned p = 0; p < devicePorts.size(); p++) {
        if (!deviceReadBuffers[p].empty() || !deviceWriteBuffers[p].empty()) {
            schedule(drainDeviceEvent, curTick());
            return;
        }
    }
}

// Deferred credit return — per-port queue.
void PCIeModel::scheduleDeferredCreditReturn(
    unsigned sp, DeferredCredit::Type type, int dataCr)
{
    // Review item #4: align return tick to creditReturnPeriod grid so
    // multiple credits scheduled within the same window share one
    // UpdateFC batch. Real PCIe sends UpdateFC DLLPs periodically
    // (every few TLPs or ~128ns), not one per credit.
    Tick rt = curTick() + creditReturnDelay;
    if (creditReturnPeriod > 0) {
        Tick period = creditReturnPeriod;
        rt = ((rt + period - 1) / period) * period;
    }
    deferredCredits[sp].push_back({type, dataCr, rt});
    if (!creditReturnEvent.scheduled())
        schedule(creditReturnEvent, rt);
}

void PCIeModel::processDeferredCredits()
{
    Tick now = curTick();
    // Charge downstream wire for UpdateFC DLLP (8 bytes) ONCE per batch
    // of credit returns processed at the same tick. Real PCIe batches
    // multiple credit updates into one UpdateFC DLLP (credit_return_period
    // controls the cadence). Previously this charged per individual
    // credit return → at high N each completion's NPH return, each
    // write's PH+PD return, and each completion's CplH+CplD return all
    // incurred a full 8-byte DLLP serialization, effectively fabricating
    // tens of µs of downstream wire pressure per sim second.
    // Phase A.1: per-port credit return. Iterate every port's queue
    // and return credits to that port's pool.
    bool anyReturned = false;
    for (unsigned sp = 0; sp < deferredCredits.size(); sp++) {
        while (!deferredCredits[sp].empty() &&
               deferredCredits[sp].front().returnTick <= now) {
            auto &dc = deferredCredits[sp].front();
            switch (dc.type) {
              case DeferredCredit::Posted:
                returnPostedCredits(sp, dc.dataCredits); break;
              case DeferredCredit::NonPosted:
                returnNonPostedCredits(sp); break;
              case DeferredCredit::Completion:
                returnCompletionCredits(sp, dc.dataCredits); break;
            }
            anyReturned = true;
            deferredCredits[sp].pop_front();
        }
    }
    if (anyReturned) {
        const unsigned DLLP_WIRE_BYTES = 8;
        Tick dllpSer = serializationDelay(DLLP_WIRE_BYTES);
        downstreamBusyUntil = std::max(downstreamBusyUntil, now) + dllpSer;
    }
    // Schedule next event at earliest pending returnTick across all ports.
    if (!creditReturnEvent.scheduled()) {
        Tick earliest = MaxTick;
        bool any = false;
        for (unsigned sp = 0; sp < deferredCredits.size(); sp++) {
            if (!deferredCredits[sp].empty()) {
                any = true;
                Tick t = deferredCredits[sp].front().returnTick;
                if (t < earliest) earliest = t;
            }
        }
        if (any)
            schedule(creditReturnEvent, std::max(earliest, curTick() + 1));
    }

    // Drain buffered requests that were waiting for credits
    bool anyBuffered = false;
    for (auto &q : deviceReadBuffers) { if (!q.empty()) { anyBuffered = true; break; } }
    if (!anyBuffered)
        for (auto &q : deviceWriteBuffers) { if (!q.empty()) { anyBuffered = true; break; } }
    if (anyBuffered)
        drainDeviceRequests();

    // Retry starved ports — shared-pool: global outstanding limits.
    bool canAcceptRead = hasFreeTags();
    // Shared-pool: check global write budget, not per-port.
    unsigned totalWritesOut = 0;
    for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
        totalWritesOut += outstandingWrites[sp];
    bool canAcceptWrite = (maxOutstandingWrites == 0 ||
                           totalWritesOut < maxOutstandingWrites);
    if (canAcceptRead || canAcceptWrite)
        retryStarvedPorts();

}

// ====================================================================
//  Tag Tracking
// ====================================================================

uint16_t PCIeModel::allocateTag()
{
    assert(!freeTags.empty());
    auto t = freeTags.front();
    freeTags.pop_front();
    // Review item #7: stamp a deadline. Completion must arrive before
    // curTick()+completionTimeoutTicks or tag is considered timed out.
    if (completionTimeoutTicks > 0)
        tagIssueDeadline[t] = curTick() + completionTimeoutTicks;
    return t;
}

void PCIeModel::releaseTag(uint16_t tag)
{
    freeTags.push_back(tag);
    // Review item #7: clear deadline on normal release.
    tagIssueDeadline.erase(tag);
}

// ====================================================================
//  Link Serialization with Assembly Pipeline
// ====================================================================

Tick PCIeModel::serializationDelay(unsigned wireBytes) const
{ return (Tick)wireBytes * linkParams.byteSerializationDelay; }

Tick PCIeModel::tlpLinkDelay(const TlpPacket &tlp) const
{
    Tick asmDelay = assemblyDelay(tlp.totalBytes);
    Tick serDelay = serializationDelay(tlp.wireBytes);
    // The bottleneck is whichever is slower.
    Tick perTlpDelay = std::max(asmDelay, serDelay);
    // PCIe Data Link Layer: each TLP requires an ACK DLLP from the
    // receiver. ACK DLLPs (8 bytes) share the link and consume
    // bandwidth. The sender pipelines TLPs without waiting for ACKs,
    // but the ACK traffic reduces effective link throughput. Model
    // this as amortized per-TLP overhead. CXL has no DLLP layer.
    perTlpDelay += dllpAckDelay;
    return perTlpDelay;
}

void PCIeModel::enqueueUpstream(TlpPacket &tlp)
{
    Tick now = curTick();
    // Upstream TLP delay: assembly and serialization.
    // Assembly = FPGA datapath assembling wire bytes at (width × core_freq).
    // At 64B × 500 MHz = 32 GB/s this can be SLOWER than Gen5 x16 wire
    // (63 GB/s), making assembly the steady-state throughput cap.
    // Assembly and wire DO pipeline (overlap for different TLPs), but
    // the overall rate is capped by whichever stage is slower —
    // max(asmDelay, serDelay) is the correct throughput model.
    // Change assemblyDelay() if modeling a 1 GHz hard block.
    Tick asmDelay = assemblyDelay(tlp.totalBytes);
    Tick serDelay = serializationDelay(tlp.wireBytes);
    Tick delay = std::max(asmDelay, serDelay);
    Tick earliest = now + bridgePipelineDelay;

    // Review item #3: enforce per-core-clock TLP gap. Real PCIe hard
    // block can only emit one TLP per core-clock edge (typically 500 MHz
    // = 2ns). Matters for streams of small TLPs.
    Tick coreGateTick = (coreClockPeriod > 0 && lastUpstreamEmit > 0) ?
                        lastUpstreamEmit + coreClockPeriod : 0;

    Tick startSer = std::max({earliest, upstreamBusyUntil, coreGateTick});
    Tick endSer = startSer + delay;
    // Trace when core-clock gate is the binding constraint (means the
    // endpoint is emitting small TLPs back-to-back faster than core clock)
    if (coreGateTick > earliest && coreGateTick > upstreamBusyUntil) {
        DPRINTF(PCIe, "  [CORE-GATE] upstream TLP delayed by core clock "
                "gap (last=%llu, period=%llu, gate=%llu) vs busy=%llu "
                "vs earliest=%llu → start=%llu\n",
                lastUpstreamEmit, coreClockPeriod, coreGateTick,
                upstreamBusyUntil, earliest, startSer);
    }
    upstreamBusyUntil = endSer;
    lastUpstreamEmit = startSer;  // Review item #3: track last emit tick

    // Bias #1 note: DLLP ACKs (real PCIe) are BATCHED — one ACK
    // covers multiple TLPs (spec: ACK every ~8 TLPs or ~128 µs, via
    // ACK coalescing). Previously charged per-TLP (2 ns each),
    // fabricating tens of µs of fake reverse-direction wire pressure
    // at high N. Remove per-TLP charge. If you want to model ACK
    // overhead accurately, add a periodic event that charges
    // batched DLLP wire time on the reverse direction at the ACK
    // coalescing cadence — analogous to the credit_return_period
    // handling in processDeferredCredits.

    // Track if assembly was the bottleneck
    if (asmDelay > serDelay)
        stats.assemblyBottleneckTLPs++;

    // Phase A.1: push to per-port queue. The wire (upstreamBusyUntil)
    // and core-clock emit gate (lastUpstreamEmit) above remain shared,
    // so per-port queues do NOT add wire bandwidth — they only prevent
    // head-of-line blocking between independently-credited ports.
    int sp = tlp.srcPortIdx;
    // Hard fail on uninitialized/out-of-range srcPortIdx — silent
    // routing-to-port-0 default would mis-route credits and queues.
    panic_if(sp < 0 || (unsigned)sp >= upstreamQueue.size(),
             "PCIe enqueueUpstream: bad srcPortIdx=%d (numDevicePorts=%lu) "
             "fmt=0x%x type=0x%x tag=%u addr=0x%lx — caller forgot to set "
             "tlp.srcPortIdx before enqueue",
             sp, upstreamQueue.size(), tlp.fmt, tlp.type,
             tlp.tag, tlp.addr);
    DPRINTF(PCIe, "  [ENQUP] port=%d fmt=0x%x type=0x%x wire=%u tag=%u "
            "endSer=%llu queueDepth=%lu\n",
            sp, tlp.fmt, tlp.type, tlp.wireBytes, tlp.tag,
            endSer, upstreamQueue[sp].size() + 1);
    upstreamQueue[sp].push_back({tlp, endSer});
    stats.totalWireBytes += tlp.wireBytes;

    DPRINTF(PCIe, "  ^ TLP fmt=0x%x type=0x%x addr=0x%x "
            "hdr=%u pad=%u pay=%u wire=%u asm=%llu ser=%llu "
            "range=[%llu,%llu]\n",
            tlp.fmt, tlp.type, tlp.addr,
            tlp.headerBytes, tlp.paddingBytes, tlp.payloadBytes,
            tlp.wireBytes,
            assemblyDelay(tlp.totalBytes),
            serializationDelay(tlp.wireBytes),
            startSer, endSer);

    if (!upstreamEvent.scheduled())
        schedule(upstreamEvent, endSer);
}

void PCIeModel::enqueueDownstream(TlpPacket &tlp, Tick earliestStart)
{
    // Downstream TLPs (CplD) are generated by the Root Complex, not the FPGA.
    // The RC assembles at GHz+ speeds — wire serialization is the only limit.
    Tick delay = serializationDelay(tlp.wireBytes);

    // Review item #3: per-core-clock TLP gap applies on downstream too —
    // the FPGA's PCIe hard block deserializes one TLP per core cycle.
    Tick coreGateTick = (coreClockPeriod > 0 && lastDownstreamEmit > 0) ?
                        lastDownstreamEmit + coreClockPeriod : 0;

    Tick startSer = std::max({earliestStart, downstreamBusyUntil, coreGateTick});
    Tick endSer = startSer + delay;
    if (coreGateTick > earliestStart && coreGateTick > downstreamBusyUntil) {
        DPRINTF(PCIe, "  [CORE-GATE] downstream TLP delayed by core clock "
                "gap (last=%llu, period=%llu, gate=%llu) vs busy=%llu "
                "vs earliest=%llu → start=%llu\n",
                lastDownstreamEmit, coreClockPeriod, coreGateTick,
                downstreamBusyUntil, earliestStart, startSer);
    }
    downstreamBusyUntil = endSer;
    lastDownstreamEmit = startSer;

    // DLLP ACK on reverse direction: removed per-TLP charge
    // (batched in real PCIe — see enqueueUpstream comment).

    // Phase A.1: push to per-port downstream queue keyed by source port.
    int sp = tlp.srcPortIdx;
    panic_if(sp < 0 || (unsigned)sp >= downstreamQueue.size(),
             "PCIe enqueueDownstream: bad srcPortIdx=%d "
             "(numDevicePorts=%lu) fmt=0x%x tag=%u — caller forgot "
             "to set tlp.srcPortIdx",
             sp, downstreamQueue.size(), tlp.fmt, tlp.tag);
    DPRINTF(PCIe, "  [ENQDN] port=%d fmt=0x%x tag=%u pay=%u "
            "endSer=%llu queueDepth=%lu\n",
            sp, tlp.fmt, tlp.tag, tlp.payloadBytes,
            endSer, downstreamQueue[sp].size() + 1);
    downstreamQueue[sp].push_back({tlp, endSer});
    stats.totalWireBytes += tlp.wireBytes;

    DPRINTF(PCIe, "  v TLP fmt=0x%x tag=%u pay=%u wire=%u "
            "earliest=%llu start=%llu end=%llu\n",
            tlp.fmt, tlp.tag, tlp.payloadBytes, tlp.wireBytes,
            earliestStart, startSer, endSer);

    if (!downstreamEvent.scheduled())
        schedule(downstreamEvent, endSer);
    else if (downstreamEvent.when() > endSer)
        reschedule(downstreamEvent, endSer);
}

// ====================================================================
//  Completion Reordering Buffer
// ====================================================================

void PCIeModel::bufferCompletion(TlpPacket &tlp, Tick arrivalTick)
{
    // Count total entries across all tags
    unsigned totalBuffered = 0;
    for (auto &kv : completionBuffer)
        totalBuffered += kv.second.size();
    if (totalBuffered >= completionReorderDepth) {
        fatal("PCIe: completion reorder buffer full (%u >= %u entries). "
              "Set completion_reorder_depth >= max_tags (%u) to prevent "
              "this. Current config is undersized.\n",
              totalBuffered, completionReorderDepth, maxTags);
    }
    completionBuffer[tlp.tag].push_back({tlp, arrivalTick});
}

void PCIeModel::deliverCompletions(Tick now)
{
    // For each tag with buffered completions, deliver in-order
    for (auto it = completionBuffer.begin();
         it != completionBuffer.end(); ) {

        uint16_t tag = it->first;
        auto &queue = it->second;

        while (!queue.empty() && queue.front().arrivalTick <= now) {
            PendingCompletion &pc = queue.front();
            TlpPacket &tlp = pc.tlp;

            auto readIt = outstandingReads.find(tag);
            if (readIt == outstandingReads.end()) {
                // Phase A.1: Use tlp's srcPortIdx since outstandingReads
                // entry is already gone.
                scheduleDeferredCreditReturn(
                    (unsigned)tlp.srcPortIdx,
                    DeferredCredit::Completion,
                    dataCreditsNeeded(tlp.payloadBytes));
                queue.pop_front();
                continue;
            }

            OutstandingRead &orec = readIt->second;
            orec.completedBytes += tlp.payloadBytes;

            DPRINTF(PCIe, "  [DELIVER] tag=%u: %u/%u bytes @%llu\n",
                    tag, orec.completedBytes, orec.totalBytes, now);
            scheduleDeferredCreditReturn(
                (unsigned)orec.srcPortIdx,
                DeferredCredit::Completion,
                dataCreditsNeeded(tlp.payloadBytes));

            if (orec.completedBytes >= orec.totalBytes) {
                // PCIe Bug #8 fix: sample in ticks (matches declared unit Tick).
                // Reads on PCIe retire per-group at the last-CplD arrival,
                // so one sample per coalesced group is already correct
                // (no per-beat inflation like CXL had).
                Tick lat = now - orec.issueTick;
                stats.totalReadLatency += lat;
                stats.readLatencyHist.sample(lat);

                // Save fields BEFORE erase — orec reference
                // becomes dangling after outstandingReads.erase()
                int srcPort = orec.srcPortIdx;
                unsigned numCoalesced = orec.allPkts.size();
                // Move allPkts out before erase
                std::vector<PacketPtr> devPkts = std::move(orec.allPkts);

                DPRINTF(PCIe, "  RD DONE tag=%u lat=%llu (%.1fns) beats=%u\n",
                        tag, lat, (double)lat / 1000.0, numCoalesced);

                // One NPH credit per coalesced MRd TLP (always 1).
                scheduleDeferredCreditReturn(
                    (unsigned)srcPort,
                    DeferredCredit::NonPosted, 0);
                releaseTag(tag);
                outstandingReads.erase(readIt);
                perPortReadsOut[srcPort]--;  // one coalesced group done

                // Shared-pool: check global read budget.
                unsigned totalReadsOut = 0;
                for (auto v : perPortReadsOut) totalReadsOut += v;
                if (hasFreeTags() &&
                    (maxOutstanding == 0 ||
                     totalReadsOut < maxOutstanding)) {
                    pendingDeviceRetry = true;
                }

                bool anyRdBuf = false;
                for (auto &q : deviceReadBuffers) { if (!q.empty()) { anyRdBuf = true; break; } }
                if (anyRdBuf)
                    drainDeviceRequests();

                if (tlp.origPkt) delete tlp.origPkt;

                // Send individual responses for each coalesced read.
                // Phase A.1: per-port responseQueue.
                for (auto *devPkt : devPkts) {
                    devPkt->makeResponse();
                    responseQueue[srcPort].push_back(
                        {devPkt, srcPort, curTick()});
                }
                // Review item #1: retirement frees RRB entries. Each beat
                // now retired to AXI R channel releases one RRB slot.
                // Phase A.1: per-port RRB occupancy.
                unsigned freed = devPkts.size();
                if (rrbDepth > 0 && rrbOccupied[srcPort] >= freed)
                    rrbOccupied[srcPort] -= freed;
                else
                    rrbOccupied[srcPort] = 0;  // safety clamp
                DPRINTF(PCIe, "  [RRB-RETIRE] port=%d freed=%u beats, "
                        "occ=%u/%u\n",
                        srcPort, freed, rrbOccupied[srcPort], rrbDepth);

                if (!responseEvent.scheduled())
                    schedule(responseEvent, now);

                // Phase tracking
                if (rdTracker.firstDnDone == 0) rdTracker.firstDnDone = now;
                rdTracker.lastDnDone = now;
                rdTracker.respCount += numCoalesced;
                if (rdTracker.respCount >= rdTracker.reqCount &&
                    rdTracker.reqCount > 0)
                    printPhaseBreakdown(rdTracker, "READ");
            }

            queue.pop_front();
        }

        if (queue.empty())
            it = completionBuffer.erase(it);
        else
            ++it;
    }
}

// ====================================================================
//  Ports
// ====================================================================

PCIeModel::DeviceSidePort::DeviceSidePort(
    const std::string &name, PCIeModel &owner, int idx)
    : ResponsePort(name), owner(owner), portIdx(idx), needRetry(false) {}
Tick PCIeModel::DeviceSidePort::recvAtomic(PacketPtr pkt)
{
    // Forward to host memory so atomic/fastforward mode gets real data.
    // Previously returned readLatency + rcLatency without servicing the
    // packet — atomic mode got zeroed/stale data.
    Tick hostLat = owner.hostPorts[0]->sendAtomic(pkt);
    return hostLat + owner.rcLatency;
}
bool PCIeModel::DeviceSidePort::recvTimingReq(PacketPtr pkt)
{ return owner.handleDeviceRequest(pkt, portIdx); }
void PCIeModel::DeviceSidePort::recvRespRetry()
{ owner.trySendResponses(); }
void PCIeModel::DeviceSidePort::recvFunctional(PacketPtr pkt)
{ owner.hostPorts[0]->sendFunctional(pkt); }
AddrRangeList PCIeModel::DeviceSidePort::getAddrRanges() const
{ return owner.hostPorts[0]->getAddrRanges(); }

PCIeModel::HostSidePort::HostSidePort(
    const std::string &name, PCIeModel &owner, int idx)
    : RequestPort(name), owner(owner), portIdx(idx),
      needRetry(false) {}
bool PCIeModel::HostSidePort::recvTimingResp(PacketPtr pkt)
{
    return owner.handleHostResponse(pkt);
}
void PCIeModel::HostSidePort::recvReqRetry()
{
    DPRINTF(PCIe, "  [PORT] recvReqRetry on host port[%d] @%llu\n",
            portIdx, curTick());
    needRetry = false;
    owner.trySendToHost();
}
void PCIeModel::HostSidePort::recvRangeChange()
{
    for (auto *dp : owner.devicePorts)
        dp->sendRangeChange();
}

// ====================================================================
//  Stage 1: Device Request → Build TLPs → Upstream
// ====================================================================

bool
PCIeModel::handleDeviceRequest(PacketPtr pkt, int srcPort)
{
    DPRINTF(PCIe, "DevReq[%d]: %s addr=0x%x size=%u\n",
            srcPort, pkt->isRead() ? "RD" : "WR",
            pkt->getAddr(), pkt->getSize());

    // ---- Buffer depth check (models per-port bridge FIFO) ----
    if (pkt->isRead()) {
        // Accept reads freely into buffer. The real limits are
        // tags and credits, checked in processBufferedRead.
        // Buffer rarely exceeds ~1024 entries (one ORAM op).
    } else if (pkt->isWrite()) {
        // Review fix #1: was summing deviceWriteBuffer.size() + outstandingWrites,
        // but outstandingWrites is incremented below (line ~861) on buffer
        // push and includes every buffered write until DDR5 commit. Summing
        // double-counts. CXL has the same fix at handleDeviceRequest;
        // PCIe was missed. Effective cap was ~64 (half of max_outstanding_writes=128).
        // Phase A.1: shared-pool outstanding-write count.
        unsigned totalWritesOut = 0;
        for (unsigned j = 0; j < devicePorts.size(); j++)
            totalWritesOut += outstandingWrites[j];
        if (maxOutstandingWrites > 0 &&
            totalWritesOut >= maxOutstandingWrites) {
            DPRINTF(PCIe, "  WR BUFFER FULL: portBuf=%zu out[%d]=%u totalWr=%u max=%u\n",
                    deviceWriteBuffers[srcPort].size(), srcPort,
                    outstandingWrites[srcPort], totalWritesOut, maxOutstandingWrites);
            devicePorts[srcPort]->needRetry = true;
            return false;
        }
    }

    // ---- Accept into internal buffer ----
    if (pkt->isRead()) {
        deviceReadBuffers[srcPort].push_back({pkt, srcPort});
        lastReadBufferTicks[srcPort] = curTick();
        // perPortReadsOut incremented in processBufferedRead
        // (per coalesced group, not per beat)

        // Phase tracking at accept time
        if (rdTracker.reqCount == 0) rdTracker.firstDevReq = curTick();
        rdTracker.lastDevReq = curTick();
        rdTracker.reqCount++;
        rdTracker.isRead = true;

    } else if (pkt->isWrite()) {
        deviceWriteBuffers[srcPort].push_back({pkt, srcPort});
        outstandingWrites[srcPort]++;
        lastWriteBufferTicks[srcPort] = curTick();

        // Phase tracking at accept time
        if (wrTracker.reqCount == 0) wrTracker.firstDevReq = curTick();
        wrTracker.lastDevReq = curTick();
        wrTracker.reqCount++;
        wrTracker.isRead = false;

        if (firstWriteAccept == 0) firstWriteAccept = curTick();

        // Posted write BRESP: return immediately at bridge latency.
        // The MWr TLP is built later when credits are available.
        PacketPtr respPkt = new Packet(pkt->req, MemCmd::WriteResp);
        // PCIe Bug #16: WriteResp carries no payload — skip allocate()
        // to avoid a wasted buffer allocation per write beat.
        respPkt->senderState = pkt->senderState;
        pkt->senderState = nullptr;

        Tick deliverAt = curTick() + bridgePipelineDelay;
        responseQueue[srcPort].push_back({respPkt, srcPort, deliverAt});
        if (!responseEvent.scheduled())
            schedule(responseEvent, deliverAt);
        else if (deliverAt < responseEvent.when())
            reschedule(responseEvent, deliverAt);

        lastBrespDelivered = curTick() + bridgePipelineDelay;
        writeBrespCount++;

        if (wrTracker.firstDnDone == 0)
            wrTracker.firstDnDone = curTick() + bridgePipelineDelay;
        wrTracker.lastDnDone = curTick() + bridgePipelineDelay;
        wrTracker.respCount++;

    } else {
        // PCIe Bug #14 fix: non-R/W packets used to bypass to
        // pendingHostReqs without a SenderState, which stranded any
        // response (it couldn't be routed back). Reject at entry so
        // the originator sees a clean failure instead of hanging. ORAM
        // workload is R/W only. If future use cases need non-R/W
        // (flushes, barriers), wire a minimal SenderState here.
        warn("PCIe: rejecting non-R/W cmd=%s addr=0x%x (not supported)\n",
             pkt->cmdString(), pkt->getAddr());
        return false;
    }

    // Schedule drain.
    if (!drainDeviceEvent.scheduled())
        schedule(drainDeviceEvent, curTick());
    else if (drainDeviceEvent.when() > curTick())
        reschedule(drainDeviceEvent, curTick());
    return true;
}

// --------------------------------------------------------------------
//  Process buffered reads with coalescing: accumulate sequential
//  32B reads from the same port into one MRd TLP (up to MRRS=512B).
//  Models Xilinx AXI-PCIe bridge coalescing (PG194).
//  Returns true if at least one read was processed.
// --------------------------------------------------------------------
bool
PCIeModel::processBufferedRead(PacketPtr pkt, int srcPort)
{
    unsigned beatSize = pkt->getSize();
    // Review item #2: enforce MRRS boundary. A real AXI-PCIe bridge
    // splits AXI bursts exceeding MRRS into multiple MRd TLPs. We cap
    // coalesce group at mrrs/beatSize (e.g., 512/32 = 16).
    unsigned maxCoalesce = mrrs / beatSize;  // 512/32 = 16

    // Per-port buffer: all entries belong to this port by construction.
    // Coalesce contiguous-address front entries.
    auto &portBuf = deviceReadBuffers[srcPort];

    std::vector<PacketPtr> group;
    group.push_back(pkt);  // first packet (still at buffer front)

    Addr nextAddr = pkt->getAddr() + beatSize;
    size_t scanCount = 0;  // how many to erase starting at position 1

    for (size_t idx = 1;
         group.size() < maxCoalesce && idx < portBuf.size();
         idx++) {
        auto &entry = portBuf[idx];
        if (entry.pkt->getAddr() == nextAddr &&
            entry.pkt->getSize() == beatSize) {
            group.push_back(entry.pkt);
            nextAddr += beatSize;
            scanCount = idx;  // inclusive end
        } else {
            // Non-contiguous address within the same port: AXI
            // intra-master ordering requires us to stop here and
            // emit this group; the next drain call will handle the
            // next burst at its new front.
            break;
        }
    }

    unsigned totalSize = group.size() * beatSize;

    // Check tag (1 per coalesced group)
    if (!hasFreeTags()) {
        stats.tagExhausted++;
        return false;  // nothing modified
    }

    // Check NPH credit (1 MRd TLP for the entire group). Phase A.1:
    // per-port credit pool — this port's nphCredits.
    if (credits[0].nphCredits < 1) {
        stats.creditStalls++;
        DPRINTF(PCIe, "  RD CREDIT STALL: port=%d nph=%d < 1\n",
                srcPort, credits[0].nphCredits);
        return false;  // nothing modified
    }

    // Resources available — pop the coalesced prefix (positions 1..scanCount).
    // Front (position 0) is popped by the drain loop AFTER this returns.
    if (scanCount > 0)
        portBuf.erase(portBuf.begin() + 1, portBuf.begin() + 1 + scanCount);

    uint16_t tag = allocateTag();

    OutstandingRead orec;
    orec.pkt = pkt;
    orec.allPkts = std::move(group);
    orec.addr = pkt->getAddr();
    orec.totalBytes = totalSize;
    orec.completedBytes = 0;
    orec.issueTick = curTick();
    orec.srcPortIdx = srcPort;
    outstandingReads[tag] = std::move(orec);
    perPortReadsOut[srcPort]++;  // track coalesced groups in flight

    // Build ONE MRd TLP for the coalesced group
    TlpPacket tlp = buildMemReadTlp(pkt->getAddr(), totalSize, tag, pkt);
    tlp.isLastCompletion = true;  // one TLP per coalesced read
    tlp.srcPortIdx = srcPort;
    consumeNonPostedCredits(srcPort);
    enqueueUpstream(tlp);

    stats.totalReadTLPs += 1;
    stats.readRequests++;
    stats.totalReadBytes += totalSize;
    stats.readCoalesceHist.sample(totalSize / beatSize);

    DPRINTF(PCIe, "  [COALESCE] RD tag=%u addr=0x%x size=%u (%u beats) @%llu\n",
            tag, pkt->getAddr(), totalSize,
            (unsigned)outstandingReads[tag].allPkts.size(), curTick());

    return true;
}

// --------------------------------------------------------------------
//  Process buffered writes with coalescing: accumulate sequential
//  32B writes from the same port into one MWr TLP (up to MPS=256B).
//  BRESP already generated at accept time. outstandingWrites already
//  incremented. This only builds the MWr TLPs for upstream.
// --------------------------------------------------------------------
bool
PCIeModel::processBufferedWrite(PacketPtr pkt, int srcPort)
{
    unsigned beatSize = pkt->getSize();
    // Review item #2: enforce MPS boundary. AXI bursts exceeding MPS
    // split into multiple MWr TLPs. Cap at mps/beatSize (256/32 = 8).
    unsigned maxCoalesce = mps / beatSize;  // 256/32 = 8

    // Per-port buffer: all entries belong to srcPort. Coalesce
    // contiguous-address front entries.
    auto &portBuf = deviceWriteBuffers[srcPort];

    std::vector<PacketPtr> group;
    group.push_back(pkt);  // first packet (still at buffer front)

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
            // Non-contiguous same-port address: AXI intra-master
            // ordering — stop and emit the group we have.
            break;
        }
    }

    unsigned totalSize = group.size() * beatSize;

    // Check PH + PD credits for ONE coalesced MWr TLP. Phase A.1:
    // per-port credit pool — this port's ph/pd credits.
    int totalPd = dataCreditsNeeded(totalSize);
    if (credits[0].phCredits < 1 ||
        credits[0].pdCredits < totalPd) {
        stats.creditStalls++;
        DPRINTF(PCIe, "  WR CREDIT STALL: port=%d ph=%d<1 OR pd=%d<%d\n",
                srcPort, credits[0].phCredits,
                credits[0].pdCredits, totalPd);
        return false;  // nothing modified
    }

    // Resources available — pop the coalesced prefix (positions 1..scanCount).
    // Front (position 0) is popped by the drain loop AFTER this returns.
    if (scanCount > 0)
        portBuf.erase(portBuf.begin() + 1, portBuf.begin() + 1 + scanCount);

    // Build ONE MWr TLP for the coalesced group
    TlpPacket tlp = buildMemWriteTlp(pkt->getAddr(), totalSize, pkt);
    tlp.isLastCompletion = true;
    tlp.allWritePkts = std::move(group);
    tlp.srcPortIdx = srcPort;

    consumePostedCredits(srcPort, totalPd);
    enqueueUpstream(tlp);

    stats.totalWriteTLPs += 1;
    stats.writeRequests++;
    stats.totalWriteBytes += totalSize;
    stats.writeCoalesceHist.sample(totalSize / beatSize);

    DPRINTF(PCIe, "  [COALESCE] WR addr=0x%x size=%u (%u beats) @%llu\n",
            pkt->getAddr(), totalSize,
            (unsigned)tlp.allWritePkts.size(), curTick());

    return true;
}

// --------------------------------------------------------------------
//  Drain device request buffers: build TLPs when credits available.
//  Called inline from handleDeviceRequest (fast path) and from
//  credit return events (deferred path).
// --------------------------------------------------------------------
void
PCIeModel::drainDeviceRequests()
{
    // Per-port drain. Real AXI-PCIe bridge has independent per-channel
    // FIFOs; we model each port's buffer independently. Each port's
    // coalescing is simple: contiguous same-port entries starting at
    // the port's buffer front.
    //
    // Coalescing timeout (PCIe Bug #2): each port's FIFO gets its own
    // timeout gate — beats arrive staggered at 300 MHz (~3333 ticks
    // between beats), so we wait until the port has rdCoalesceTarget
    // beats OR its last-beat arrival was > 60000 ticks ago, whichever
    // fires first.
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
            if (beatSize > 0 && mrrs > 0) {
                rdTarget = mrrs / beatSize;
                if (rdTarget == 0) rdTarget = 1;
            }
            // Step 9 fix: path-tree reads come in 32B beats. Single
            // non-burst reads should not wait for a coalesce group
            // that will never form. Phase B fix: ALSO drain any
            // single read whose beatSize != 32 immediately. CPU MMIO
            // polls (8 bytes) are the trigger case — they would
            // otherwise hang in the rd buffer indefinitely. The
            // strict-> timeout check below also has a boundary
            // off-by-one; use >= to cover the exact-deadline tick.
            bool nonBurstReady = (rdBuf.size() == 1 && beatSize != 32);
            bool rdReady = nonBurstReady ||
                           rdBuf.size() >= rdTarget ||
                           now >= lastReadBufferTicks[p] + COALESCE_TIMEOUT;
            if (rdReady) {
                while (!rdBuf.empty()) {
                    auto &req = rdBuf.front();
                    if (!processBufferedRead(req.pkt, req.srcPort)) break;
                    rdBuf.pop_front();
                    if (req.srcPort < (int)perPortLastDrainTick.size())
                        perPortLastDrainTick[req.srcPort] = now;
                }
            } else {
                DPRINTF(PCIe, "  [DRAIN-WAIT-RD] port=%u rdBuf=%lu beat=%u "
                        "target=%u nonBurst=%d now=%llu lastTick=%llu "
                        "deadline=%llu (in %llu ticks)\n",
                        p, rdBuf.size(), beatSize, rdTarget,
                        (int)nonBurstReady, now, lastReadBufferTicks[p],
                        lastReadBufferTicks[p] + COALESCE_TIMEOUT,
                        (lastReadBufferTicks[p] + COALESCE_TIMEOUT > now)
                            ? (lastReadBufferTicks[p] + COALESCE_TIMEOUT - now)
                            : 0);
            }
            // Determine reschedule deadline for under-full coalesce
            if (!rdBuf.empty() && rdBuf.size() < rdTarget) {
                Tick dl = lastReadBufferTicks[p] + COALESCE_TIMEOUT;
                if (dl < earliestDeadline) earliestDeadline = dl;
            }
        }

        // --- writes on port p ---
        if (!wrBuf.empty()) {
            unsigned wrTarget = mps / 32;  // 256/32 = 8
            if (wrTarget == 0) wrTarget = 1;
            // Step 9 fix: same as reads — non-burst writes shouldn't
            // wait for coalesce. Phase B fix: drain any single write
            // whose beatSize != 32 immediately. CPU's 8-byte cmd-ring
            // updates and ORAM's 64B cons_idx writebacks both fall here.
            unsigned wrBeatSize = wrBuf.front().pkt->getSize();
            bool wrNonBurstReady = (wrBuf.size() == 1 && wrBeatSize != 32);
            bool wrReady = wrNonBurstReady ||
                           wrBuf.size() >= wrTarget ||
                           now >= lastWriteBufferTicks[p] + COALESCE_TIMEOUT;
            if (wrReady) {
                while (!wrBuf.empty()) {
                    auto &req = wrBuf.front();
                    if (!processBufferedWrite(req.pkt, req.srcPort)) break;
                    wrBuf.pop_front();
                    if (req.srcPort < (int)perPortLastDrainTick.size())
                        perPortLastDrainTick[req.srcPort] = now;
                }
            } else {
                DPRINTF(PCIe, "  [DRAIN-WAIT-WR] port=%u wrBuf=%lu beat=%u "
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

    // Reschedule ONLY for the coalesce-timeout case (under-full groups
    // on some port). Credit-stall case is handled by external wakes:
    //   * processDeferredCredits → drainDeviceRequests on credit return
    //   * handleDeviceRequest schedules drainDeviceEvent on new beat
    // Tag-free case is handled in deliverCompletions (drain kick).
    if (earliestDeadline != MaxTick && earliestDeadline > now
        && !drainDeviceEvent.scheduled()) {
        schedule(drainDeviceEvent, earliestDeadline);
    }
}

// ====================================================================
//  Stage 2: Upstream Done → Forward to Host
// ====================================================================

void PCIeModel::processUpstreamQueue()
{
    Tick now = curTick();
    unsigned n = upstreamQueue.size();
    if (n == 0) return;

    // Phase A.1: rotating-start FCFS per-port dispatch. Each call starts
    // at nextUpstreamPort and drains all ready TLPs from each port before
    // advancing. RC pipeline burst-window state (lastUpstreamRcTick) is
    // shared because the RC is one physical stage. pendingHostReqs is
    // per-port.
    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextUpstreamPort + i) % n;
        while (!upstreamQueue[sp].empty() &&
               upstreamQueue[sp].front().readyTick <= now) {

            LinkQueueEntry entry = upstreamQueue[sp].front();
            upstreamQueue[sp].pop_front();
            TlpPacket &tlp = entry.tlp;

            // Phase B diagnostic: track per-port progress
            if (sp < perPortLastDrainTick.size())
                perPortLastDrainTick[sp] = now;

            // Phase A.1 invariant: TLP on queue[sp] must declare srcPortIdx==sp.
            panic_if(tlp.srcPortIdx != (int)sp,
                     "PCIe processUpstreamQueue: TLP on queue[%u] has "
                     "srcPortIdx=%d (mismatch). fmt=0x%x type=0x%x tag=%u",
                     sp, tlp.srcPortIdx, tlp.fmt, tlp.type, tlp.tag);

            bool isRead = (tlp.fmt == FMT_3DW_NODATA ||
                           tlp.fmt == FMT_4DW_NODATA);
            bool isWrite = (tlp.fmt == FMT_3DW_DATA ||
                            tlp.fmt == FMT_4DW_DATA);

            if (isRead) {
                DPRINTF(PCIe, "  [UP] port=%u Read TLP done: tag=%u "
                        "addr=0x%x isLast=%d @%llu\n",
                        sp, tlp.tag, tlp.addr, tlp.isLastCompletion, now);
                if (!tlp.isLastCompletion) continue;

                auto it = outstandingReads.find(tlp.tag);
                if (it == outstandingReads.end()) {
                    DPRINTF(PCIe, "  [UP] WARNING: tag=%u not found!\n",
                            tlp.tag);
                    continue;
                }

                // Upstream RC pipeline (shared, serialized) — same model
                // as downstream. Cold = pipeline idle. Warm = serialize
                // at rcThroughputDelay via rcUpstreamBusyUntil.
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

                DPRINTF(PCIe, "  [UP] RC: cold=%d pipeIdle=%lld "
                        "rcEnd=%llu @%llu\n",
                        rcColdUp,
                        (long long)now - (long long)rcUpstreamBusyUntil,
                        rcEndUp, now);

                Tick sendTick = std::max(rcEndUp, curTick() + 1);

                // Temporary RC debug — first 10 read TLPs only
                {
                    // Bug 5 fix: now a member variable
                    if (rcDbgCount < 10) {
                        inform("RC-DBG READ[%d] tag=%u "
                               "firstDevReq=%llu tlpCreated=%llu tlpReady=%llu "
                               "upstreamNow=%llu cold=%d rcEnd=%llu "
                               "rcBusy=%llu sendTick=%llu "
                               "upstreamSpan=%llu",
                               rcDbgCount, tlp.tag,
                               rdTracker.firstDevReq,
                               tlp.creationTick,
                               entry.readyTick,
                               now,
                               rcColdUp, rcEndUp,
                               rcUpstreamBusyUntil, sendTick,
                               (sendTick > rdTracker.firstDevReq) ?
                                   (sendTick - rdTracker.firstDevReq) : 0);
                        rcDbgCount++;
                    }
                }

                OutstandingRead &orec = it->second;
                for (auto *devPkt : orec.allPkts) {
                    PacketPtr hostPkt = new Packet(devPkt->req,
                                                   MemCmd::ReadReq);
                    hostPkt->allocate();

                    auto *ss = new SenderState;
                    ss->tag = tlp.tag;
                    ss->origAddr = orec.addr;
                    ss->origSize = orec.totalBytes;
                    ss->issueTick = orec.issueTick;
                    ss->srcPortIdx = orec.srcPortIdx;
                    hostPkt->pushSenderState(ss);

                    pendingHostReqs[sp].push_back({hostPkt, sendTick});
                }

                if (!hostSendEvent.scheduled())
                    schedule(hostSendEvent, sendTick);
                else if (hostSendEvent.when() > sendTick)
                    reschedule(hostSendEvent, sendTick);

                DPRINTF(PCIe, "  [UP] → HostRD port=%u tag=%u addr=0x%x "
                        "size=%u cold=%d rcEnd=%llu send@%llu\n",
                        sp, tlp.tag, it->second.addr,
                        it->second.totalBytes, rcColdUp, rcEndUp, sendTick);

                if (rdTracker.firstTlpDone == 0) rdTracker.firstTlpDone = now;
                rdTracker.lastTlpDone = now;

            } else if (isWrite) {
                DPRINTF(PCIe, "  [UP] port=%u Write TLP done: addr=0x%x "
                        "pay=%u isLast=%d @%llu\n",
                        sp, tlp.addr, tlp.payloadBytes,
                        tlp.isLastCompletion, now);
                int dataCr = dataCreditsNeeded(tlp.payloadBytes);
                // Phase A.1: route credit return to the issuing port's pool
                scheduleDeferredCreditReturn(
                    sp, DeferredCredit::Posted, dataCr);

                if (!tlp.isLastCompletion) continue;

                // Upstream RC pipeline (shared with reads).
                bool rcColdUpW = (now > rcUpstreamBusyUntil + burstWindowTicks);
                Tick rcStartUpW, rcEndUpW;
                if (rcColdUpW) {
                    rcStartUpW = now + rcLatency;
                    rcEndUpW = rcStartUpW;
                } else {
                    rcStartUpW = std::max(now, rcUpstreamBusyUntil);
                    rcEndUpW = rcStartUpW + rcThroughputDelay;
                }
                rcUpstreamBusyUntil = rcEndUpW;

                Tick sendTick2 = std::max(rcEndUpW, curTick() + 1);

                // Temporary RC debug — first 10 write TLPs only
                {
                    // Bug 5 fix: now a member variable
                    if (rcWrDbgCount < 10) {
                        inform("RC-DBG WRITE[%d] "
                               "firstDevReq=%llu tlpCreated=%llu tlpReady=%llu "
                               "upstreamNow=%llu cold=%d rcEnd=%llu "
                               "rcBusy=%llu sendTick=%llu",
                               rcWrDbgCount,
                               wrTracker.firstDevReq,
                               tlp.creationTick,
                               entry.readyTick,
                               now,
                               rcColdUpW, rcEndUpW,
                               rcUpstreamBusyUntil, sendTick2);
                        rcWrDbgCount++;
                    }
                }

                auto &wrPkts = tlp.allWritePkts;
                if (!wrPkts.empty()) {
                    for (auto *devicePkt : wrPkts) {
                        PacketPtr hostPkt = new Packet(devicePkt->req,
                                                       MemCmd::WriteReq);
                        hostPkt->allocate();
                        if (devicePkt->hasData())
                            hostPkt->setData(
                                devicePkt->getConstPtr<uint8_t>());

                        auto *ss = new SenderState;
                        ss->tag = 0;
                        ss->origAddr = devicePkt->getAddr();
                        ss->origSize = devicePkt->getSize();
                        ss->issueTick = tlp.issueTick;
                        ss->isPostedWrite = true;
                        ss->origDevicePkt = nullptr;
                        ss->srcPortIdx = tlp.srcPortIdx;
                        hostPkt->pushSenderState(ss);

                        pendingHostReqs[sp].push_back({hostPkt, sendTick2});
                        delete devicePkt;
                    }
                    wrPkts.clear();
                } else {
                    PacketPtr devicePkt = tlp.origPkt;
                    PacketPtr hostPkt = new Packet(devicePkt->req,
                                                   MemCmd::WriteReq);
                    hostPkt->allocate();
                    if (devicePkt->hasData())
                        hostPkt->setData(devicePkt->getConstPtr<uint8_t>());

                    auto *ss = new SenderState;
                    ss->tag = 0;
                    ss->origAddr = devicePkt->getAddr();
                    ss->origSize = devicePkt->getSize();
                    ss->issueTick = tlp.issueTick;
                    ss->isPostedWrite = true;
                    ss->origDevicePkt = nullptr;
                    ss->srcPortIdx = tlp.srcPortIdx;
                    hostPkt->pushSenderState(ss);

                    pendingHostReqs[sp].push_back({hostPkt, sendTick2});
                    delete devicePkt;
                }

                if (!hostSendEvent.scheduled())
                    schedule(hostSendEvent, sendTick2);
                else if (hostSendEvent.when() > sendTick2)
                    reschedule(hostSendEvent, sendTick2);

                DPRINTF(PCIe, "  [UP] → HostWR port=%u addr=0x%x pay=%u "
                        "cold=%d rcEnd=%llu send@%llu\n",
                        sp, tlp.addr, tlp.payloadBytes,
                        rcColdUpW, rcEndUpW, sendTick2);

                if (wrTracker.firstTlpDone == 0) wrTracker.firstTlpDone = now;
                wrTracker.lastTlpDone = now;
            }
        }
    }
    nextUpstreamPort = (nextUpstreamPort + 1) % n;

    // Reschedule for earliest pending readyTick across all ports.
    if (!upstreamEvent.scheduled()) {
        Tick earliest = MaxTick;
        bool any = false;
        for (unsigned sp = 0; sp < n; sp++) {
            if (!upstreamQueue[sp].empty()) {
                any = true;
                Tick t = upstreamQueue[sp].front().readyTick;
                if (t < earliest) earliest = t;
            }
        }
        if (any && earliest > curTick())
            schedule(upstreamEvent, earliest);
    }

    // Retry only if we can actually accept a new request — per-port
    // outstandingWrites and per-port phCredits.
    {
        bool canAcceptRead = hasFreeTags();
        // Shared-pool: check global write budget.
        unsigned totalWritesOut = 0;
        for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
            totalWritesOut += outstandingWrites[sp];
        bool canAcceptWrite = credits[0].phCredits > 0 &&
            (maxOutstandingWrites == 0 ||
             totalWritesOut < maxOutstandingWrites);
        if (canAcceptRead || canAcceptWrite)
            retryStarvedPorts();
    }
}

// ====================================================================
//  Stage 3: Host Response → Build Completion TLPs → Downstream
// ====================================================================

bool PCIeModel::handleHostResponse(PacketPtr pkt)
{
    auto *ss = dynamic_cast<SenderState *>(pkt->popSenderState());
    if (!ss) {
        warn("PCIe: host response without SenderState!\n");
        delete pkt;
        return true;
    }

    if (ss->isPostedWrite) {
        DPRINTF(PCIe, "  [HOST] Write done (posted, BRESP already returned): "
                "addr=0x%x @%llu\n", ss->origAddr, curTick());

        // PCIe Bug #8 fix: sample in ticks (matches declared unit Tick).
        // Previous /1000000 converted ps → µs but label said Tick;
        // sub-µs latencies sampled as 0 and distribution was useless.
        if (ss->issueTick > 0) {
            Tick wrLat = curTick() - ss->issueTick;
            stats.totalWriteLatency += wrLat;
            stats.writeLatencyHist.sample(wrLat);
        }

        // Track in-flight writes: decrement when DDR5 actually completes.
        // BRESP was already delivered to RTL at bridge_pipeline_delay.
        // Phase A.1: decrement the issuing port's count via senderState.
        assert(ss->srcPortIdx >= 0 &&
               (unsigned)ss->srcPortIdx < outstandingWrites.size());
        outstandingWrites[ss->srcPortIdx]--;
        lastDdr5WriteCommit = curTick();
        writeCommitCount++;

        // Phase tracking
        if (wrTracker.firstHostResp == 0) wrTracker.firstHostResp = curTick();
        wrTracker.lastHostResp = curTick();

        // Print write commit comparison when all writes committed across
        // every port.
        unsigned totalOutWr = 0;
        for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
            totalOutWr += outstandingWrites[sp];
        if (totalOutWr == 0 && firstWriteAccept > 0) {
            Tick rtlVisible = lastBrespDelivered - firstWriteAccept;
            Tick ddr5Commit = lastDdr5WriteCommit - firstWriteAccept;
            inform("PCIe WRITE commit: RTL_visible=%llu cyc (%.1f ns), "
                   "DDR5_commit=%llu cyc (%.1f ns), "
                   "posted_advantage=%llu cyc (%.1f ns), "
                   "bresps=%u commits=%u",
                   rtlVisible / fpgaClockPeriod, rtlVisible / 1000.0,
                   ddr5Commit / fpgaClockPeriod, ddr5Commit / 1000.0,
                   (ddr5Commit - rtlVisible) / fpgaClockPeriod,
                   (ddr5Commit - rtlVisible) / 1000.0,
                   writeBrespCount, writeCommitCount);
            firstWriteAccept = 0;
            lastBrespDelivered = 0;
            lastDdr5WriteCommit = 0;
            writeCommitCount = 0;
            writeBrespCount = 0;
        }

        {
            bool canRead = hasFreeTags();
            // Shared-pool: check global write budget.
            unsigned totalWritesOut = 0;
            for (unsigned sp = 0; sp < outstandingWrites.size(); sp++)
                totalWritesOut += outstandingWrites[sp];
            bool canWrite = (maxOutstandingWrites == 0 ||
                             totalWritesOut < maxOutstandingWrites);
            if (canRead || canWrite)
                retryStarvedPorts();
        }

        delete ss;
        delete pkt;
        return true;
    }

    // ---- Read Completion with combining ----
    // Models RC completion combining (PCIe spec §2.3.2): accumulate
    // DDR5 responses and emit fewer, larger CplD TLPs up to RCB (128B).
    // Reduces per-TLP overhead (assembly + DLLP ack) on downstream.
    //
    // PCIe Bug #1: TIMING-ONLY MODEL. The host read-response packet arrives
    // here with DDR5 data, but the PCIe model does not copy that data into
    // devPkt before makeResponse() at deliverCompletions. The device
    // receives a read response with its original (request-time) buffer.
    // Review fix #3: COPY READ DATA from host pkt to matching device pkt.
    // Previously PCIe silently discarded DDR5 bytes; device received
    // request-time buffer. CXL copies correctly at its deliver site.
    // Here we match by address: each device pkt in orec.allPkts has its
    // own 32B address; the host response carries exactly that 32B at
    // ss->origAddr. Copy now while pkt is still alive; pkt gets deleted
    // shortly below.
    uint16_t tag = ss->tag;
    unsigned responseSize = pkt->getSize();

    DPRINTF(PCIe, "  [HOST] Read resp: tag=%u addr=0x%x size=%u @%llu\n",
            tag, ss->origAddr, responseSize, curTick());

    // Phase tracking
    if (rdTracker.firstHostResp == 0) rdTracker.firstHostResp = curTick();
    rdTracker.lastHostResp = curTick();

    auto readIt = outstandingReads.find(tag);
    if (readIt == outstandingReads.end()) {
        warn("PCIe: read completion for unknown tag %u\n", tag);
        delete ss;
        delete pkt;
        return true;
    }

    // Review item #1: RRB occupancy tracking. Real Xilinx AXI-PCIe
    // bridge (PG194) holds completions in an internal reorder buffer
    // until they can retire in AXI-ID order. Separate resource from
    // the PCIe tag pool.
    //
    // DEADLOCK FIX: originally this path returned false to set
    // HostSidePort.respBlocked and retry later. That created a
    // deadlock because the retry trigger was inside retirement, but
    // retirement can't happen if responses are rejected. Changed to
    // warn-only (consistent with CXL side). If sweeps show the warn
    // firing, increase rrb_depth via --pcie-rrb-depth. Full
    // backpressure modeling is deferred until a reliable retry
    // mechanism is wired (would need a periodic event, not a
    // retirement-triggered one).
    // Phase A.1: per-port RRB occupancy. Use the issuing port from orec.
    int orecPort = readIt->second.srcPortIdx;
    assert(orecPort >= 0 &&
           (unsigned)orecPort < rrbOccupied.size());
    if (rrbDepth > 0 && rrbOccupied[orecPort] >= rrbDepth) {
        warn_once("PCIe: port=%d RRB occupancy exceeded rrb_depth=%u. "
                  "Real hardware would backpressure here; this simulation "
                  "proceeds (occupancy tracking continues). Increase "
                  "--pcie-rrb-depth if you want to avoid this overcommit.\n",
                  orecPort, rrbDepth);
    }
    rrbOccupied[orecPort]++;  // Accepted into RRB
    DPRINTF(PCIe, "  [RRB-ACCEPT] tag=%u port=%d addr=0x%lx occ=%u/%u\n",
            tag, orecPort, pkt->getAddr(),
            rrbOccupied[orecPort], rrbDepth);

    OutstandingRead &orec = readIt->second;

    // Review fix #3 + regression patch: copy host data to matching
    // device pkt. Match by pkt->getAddr() — the host response's own
    // (per-beat) address from the DDR5 controller. Previously matched
    // against ss->origAddr which is set once per group to orec.addr
    // (the group start), so every beat after the first overwrote
    // allPkts[0] and left allPkts[1..N-1] with stale data. ss->origAddr
    // is kept as the group tag for DPRINTF logs only.
    if (pkt->hasData()) {
        bool matched = false;
        for (auto *dp : orec.allPkts) {
            if (dp && dp->getAddr() == pkt->getAddr() &&
                dp->getSize() == responseSize) {
                dp->setData(pkt->getConstPtr<uint8_t>());
                matched = true;
                break;
            }
        }
        if (!matched) {
            warn_once("PCIe: host response addr 0x%lx has no matching "
                      "device pkt in tag %u (group 0x%lx). Device will "
                      "see stale data for this beat.\n",
                      pkt->getAddr(), tag, orec.addr);
        }
    } else {
        warn_once("PCIe: read response pkt has no data "
                  "(tag=%u addr=0x%lx) — device will get stale data\n",
                  tag, pkt->getAddr());
    }

    orec.pendingCplBytes += responseSize;

    // Flush when accumulated bytes reach RCB, or when this is the
    // last response for the tag. Use emittedBytes (not completedBytes)
    // because completedBytes lags behind (updated in processDownstreamQueue).
    bool isLastForTag = (orec.emittedBytes + orec.pendingCplBytes >=
                         orec.totalBytes);
    bool flushNow = (orec.pendingCplBytes >= rcb) || isLastForTag;

    if (!flushNow) {
        // Defer: accumulate for next response
        stats.completionsCombined++;
        DPRINTF(PCIe, "  [COMBINE] tag=%u pending=%uB (< rcb=%u), defer\n",
                tag, orec.pendingCplBytes, rcb);
        delete ss;
        delete pkt;
        return true;
    }

    // Flush: emit one combined CplD TLP
    unsigned combinedSize = orec.pendingCplBytes;
    unsigned emittedSoFar = orec.emittedBytes;

    // Credit check — WARN-ONLY (was: return false and stall)
    //
    // Background: in real PCIe, CplH/CplD credits limit the DEVICE's
    // completion buffer; if exhausted, the device holds completions
    // locally. In THIS model, "completions" are responses from DDR5
    // traveling back through our host_xbar to PCIeModel's host port,
    // and then onward to the ORAM RTL. We are the transit, not a
    // buffering device. Rejecting responses here makes the xbar's
    // RespPacketQueue hold them — at N>=4 ORAM instances, 8 DDR5
    // channels respond faster than credit returns fire (even with
    // aggressive credit_return_period), and the xbar queue hits its
    // hard 1024-packet cap, panicking.
    //
    // Consistent with RRB (also warn-only). Both credit checks record
    // overflow for stats but never backpressure upstream. If you need
    // to model hard credit-starvation behavior, add an explicit
    // outstanding-response buffer in PCIeModel that holds pkts
    // internally rather than relying on the xbar's queue.
    int totalCpld = dataCreditsNeeded(combinedSize);
    if (credits[0].cplhCredits < 1 ||
        credits[0].cpldCredits < totalCpld) {
        DPRINTF(PCIe, "  [CREDIT-WARN] port=%d CplH/CplD depleted "
                "(H=%d D=%d need H=1 D=%d) — allowing anyway to avoid "
                "xbar queue overflow @%llu\n",
                orecPort, credits[0].cplhCredits,
                credits[0].cpldCredits, totalCpld,
                curTick());
        stats.creditStalls++;
        // Don't drain credits below floor — let the return path
        // catch up naturally.
    }

    // ================================================================
    // Downstream delay: RC pipeline → Gen5 wire → per-port CDC → RTL
    //
    // Two bugs fixed here:
    //
    // Bug A (dead rcThroughputDelay): warm RC gave rcDelay=0, making the
    //   RC pipeline free at N>1 (burst window always warm). The
    //   rcThroughputDelay param (5ns/CplD) was stored but never used.
    //   Fix: serialize RC pipeline throughput via rcDownstreamBusyUntil.
    //   Cold start → first CplD pays rcLatency (pipeline fill).
    //   Warm → each CplD pays rcThroughputDelay, queued behind prior
    //   CplDs from ALL instances on the shared RC pipeline.
    //
    // Bug B (CDC↔wire ordering): CDC was computed first and passed as
    //   earliestStart to enqueueDownstream. Since per-port CDC (6.67ns)
    //   >> wire per-CplD (~2.2ns), the CDC-done time always dominated
    //   downstreamBusyUntil, masking wire contention at high N.
    //   Fix: physical order RC → wire → CDC. Each stage starts only
    //   after the previous delivers the CplD.
    // ================================================================
    Tick now = curTick();

    DPRINTF(PCIe, "  [COMBINE] FLUSH tag=%u %uB CplD emitted=%u/%u @%llu\n",
            tag, combinedSize, emittedSoFar, orec.totalBytes, now);

    TlpPacket cplTlp = buildCompletionTlp(
        orec.addr + emittedSoFar, combinedSize, tag,
        orec.totalBytes, emittedSoFar,
        pkt, isLastForTag);
    unsigned port = orec.srcPortIdx;
    cplTlp.srcPortIdx = port;

    consumeCompletionCredits((unsigned)port, totalCpld);

    // Stage 1: RC pipeline (shared, serialized).
    // Cold detection: is the pipeline actually idle? Check whether
    // rcDownstreamBusyUntil has expired, NOT the DDR5 inter-arrival gap.
    // The old check (now - lastDownstreamRcTick > burstWindow) was wrong:
    // at RCB=128B combining, CplDs arrive every ~20 ns. With a 10 ns
    // burst window, almost every CplD was declared cold, bypassing
    // the serialized throughput path entirely.
    bool rcCold = (now > rcDownstreamBusyUntil + burstWindowTicks);
    Tick rcStart;
    Tick rcEnd;
    if (rcCold) {
        // Cold: pipeline must fill. First CplD exits after rcLatency.
        // No additional throughput slot — rcLatency IS the full traversal.
        rcStart = now + rcLatency;
        rcEnd = rcStart;
    } else {
        // Warm: one CplD per rcThroughputDelay, serialized across
        // all instances on the shared RC pipeline.
        rcStart = std::max(now, rcDownstreamBusyUntil);
        rcEnd = rcStart + rcThroughputDelay;
    }
    rcDownstreamBusyUntil = rcEnd;
    lastDownstreamRcTick = now;

    {
        // Bug 5 fix: now a member variable
        if (rcDnDbgCount < 10) {
            inform("RC-DBG DN-CPL[%d] tag=%u cold=%d pipeIdle=%lld "
                   "rcStart=%llu rcEnd=%llu rcBusy=%llu now=%llu",
                   rcDnDbgCount, tag, rcCold,
                   (long long)now - (long long)rcDownstreamBusyUntil,
                   rcStart, rcEnd, rcDownstreamBusyUntil, now);
            rcDnDbgCount++;
        }
    }

    // Stage 2: Shared Gen5 wire serialization.
    // Wire can't start until RC delivers the CplD.
    Tick wireSerDelay = serializationDelay(cplTlp.wireBytes);
    Tick coreGateTick = (coreClockPeriod > 0 && lastDownstreamEmit > 0) ?
                        lastDownstreamEmit + coreClockPeriod : 0;
    Tick wireStart = std::max({rcEnd, downstreamBusyUntil, coreGateTick});
    Tick wireEnd = wireStart + wireSerDelay;
    downstreamBusyUntil = wireEnd;
    lastDownstreamEmit = wireStart;

    // Stage 3: Per-port CDC bridge — can't start until wire delivers.
    const Tick cdcCycles = 2;
    const Tick cdcThroughput = cdcCycles * fpgaClockPeriod;
    Tick startCdc = std::max(wireEnd, bridgeBusyUntil[port]);
    Tick doneCdc = startCdc + cdcThroughput;
    bridgeBusyUntil[port] = doneCdc;

    DPRINTF(PCIe, "  [DN RC→WIRE→CDC] port=%u tag=%u wire=%u "
            "rc=[%llu,%llu] wire=[%llu,%llu] cdc=[%llu,%llu]\n",
            port, tag, cplTlp.wireBytes,
            rcStart, rcEnd, wireStart, wireEnd, startCdc, doneCdc);

    // Push to per-port downstream queue at CDC-done time.
    // Bypass enqueueDownstream (it would re-apply wire delay).
    panic_if((int)port < 0 || port >= downstreamQueue.size(),
             "PCIe downstream: bad srcPortIdx=%d", port);
    downstreamQueue[port].push_back({cplTlp, doneCdc});
    stats.totalWireBytes += cplTlp.wireBytes;

    if (!downstreamEvent.scheduled())
        schedule(downstreamEvent, doneCdc);
    else if (downstreamEvent.when() > doneCdc)
        reschedule(downstreamEvent, doneCdc);

    stats.totalCompletionTLPs++;
    stats.totalCompletionBytes += combinedSize;

    orec.pendingCplBytes = 0;
    orec.emittedBytes += combinedSize;

    // pkt ownership: for last CplD, pkt is stored as tlp.origPkt
    // and deleted in processDownstreamQueue. For non-last, origPkt
    // is nullptr so we must delete pkt here.
    if (!isLastForTag) delete pkt;
    delete ss;
    return true;
}

// ====================================================================
//  Stage 4: Downstream Done → Reorder → Deliver
// ====================================================================

void PCIeModel::processDownstreamQueue()
{
    Tick now = curTick();
    unsigned n = downstreamQueue.size();
    if (n == 0) return;

    // Phase A.1: rotating-start FCFS per-port iteration. The wire
    // (downstreamBusyUntil) and per-FLIT emit gate stay shared.
    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextDownstreamPort + i) % n;
        while (!downstreamQueue[sp].empty() &&
               downstreamQueue[sp].front().readyTick <= now) {

            LinkQueueEntry entry = downstreamQueue[sp].front();
            downstreamQueue[sp].pop_front();
            TlpPacket &tlp = entry.tlp;

            // Phase B diagnostic: track per-port progress
            if (sp < perPortLastDrainTick.size())
                perPortLastDrainTick[sp] = now;

            // Phase A.1 invariant: TLP on downstream[sp] must declare srcPortIdx==sp.
            panic_if(tlp.srcPortIdx != (int)sp,
                     "PCIe processDownstreamQueue: TLP on queue[%u] has "
                     "srcPortIdx=%d (mismatch). fmt=0x%x tag=%u",
                     sp, tlp.srcPortIdx, tlp.fmt, tlp.tag);

            if (tlp.fmt == FMT_3DW_DATA && tlp.type == TYPE_CPLD) {
                DPRINTF(PCIe, "  [DN] port=%u CplD tag=%u pay=%u @%llu\n",
                        sp, tlp.tag, tlp.payloadBytes, now);
                bufferCompletion(tlp, now);
                stats.completionsBuffered++;
            } else {
                DPRINTF(PCIe, "  [DN] UNKNOWN fmt=0x%x type=0x%x @%llu\n",
                        tlp.fmt, tlp.type, now);
            }
        }
    }
    nextDownstreamPort = (nextDownstreamPort + 1) % n;

    // Deliver any completions that are ready
    deliverCompletions(now);

    // Reschedule for earliest pending readyTick across ports.
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

    // Flag retry if any port has room and needs retry.
    if (hasFreeTags()) {
        unsigned totalReadsOut = 0;
        for (auto v : perPortReadsOut) totalReadsOut += v;
        for (unsigned i = 0; i < devicePorts.size(); i++) {
            if (devicePorts[i]->needRetry &&
                (maxOutstanding == 0 ||
                 totalReadsOut < maxOutstanding)) {
                pendingDeviceRetry = true;
                break;
            }
        }
    }
}

// ====================================================================
//  Response / Host Send / Retries
// ====================================================================

void PCIeModel::trySendResponses()
{
    // Phase A.1: per-port responseQueue with round-robin starting cursor.
    unsigned n = responseQueue.size();
    if (n == 0) return;

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
                warn("PCIe trySendResponses: invalid portIdx=%d (max=%lu), "
                     "dropping",
                     portIdx, devicePorts.size());
                responseQueue[sp].pop_front();
                continue;
            }
            assert((unsigned)portIdx == sp);

            DPRINTF(PCIe, "  [RESP] → Device[%d]: %s addr=0x%x @%llu\n",
                    portIdx, pkt->isRead() ? "RD" : "WR",
                    pkt->getAddr(), curTick());
            if (!devicePorts[portIdx]->sendTimingResp(pkt)) {
                DPRINTF(PCIe, "  [RESP] Device[%d] BUSY — skip port\n",
                        portIdx);
                break;  // this port blocked; try other ports
            }
            responseQueue[sp].pop_front();
            // Phase B diagnostic: track per-port progress
            if (sp < perPortLastDrainTick.size())
                perPortLastDrainTick[sp] = curTick();
        }
    }
    nextResponsePort = (nextResponsePort + 1) % n;

    // Reschedule for the earliest pending entry not yet ready
    if (earliestPending != MaxTick && !responseEvent.scheduled())
        schedule(responseEvent, earliestPending);

    if (pendingDeviceRetry) {
        pendingDeviceRetry = false;
        retryStarvedPorts();
    }
}
void PCIeModel::retryDeviceSend() { trySendResponses(); }

void PCIeModel::trySendToHost()
{
    unsigned n = pendingHostReqs.size();
    if (n == 0) return;

    size_t totalPending = 0;
    for (unsigned sp = 0; sp < n; sp++)
        totalPending += pendingHostReqs[sp].size();
    DPRINTF(PCIe, "  [SEND] trySendToHost: pending_total=%lu "
            "ports=%lu @%llu\n",
            totalPending, hostPorts.size(), curTick());

    if (totalPending == 0) return;

    bool allBlocked = true;
    for (auto *hp : hostPorts)
        if (!hp->needRetry) { allBlocked = false; break; }
    if (allBlocked) return;

    // Phase A.1: per-port pendingHostReqs — round-robin across device
    // ports. Drain each subject to host-port availability and earliestSend
    // eligibility.
    Tick earliestDeferred = MaxTick;

    for (unsigned i = 0; i < n; i++) {
        unsigned sp = (nextHostSendPort + i) % n;
        for (auto it = pendingHostReqs[sp].begin();
             it != pendingHostReqs[sp].end(); ) {

            if (curTick() < it->earliestSend) {
                if (it->earliestSend < earliestDeferred)
                    earliestDeferred = it->earliestSend;
                ++it;
                continue;
            }

            PacketPtr pkt = it->pkt;
            unsigned portIdx = (pkt->getAddr() >> 6) % hostPorts.size();

            if (hostPorts[portIdx]->needRetry) {
                ++it;
                continue;
            }

            DPRINTF(PCIe, "  [SEND] dev=%u → Host[%u]: %s addr=0x%x "
                    "size=%u @%llu\n",
                    sp, portIdx, pkt->isRead() ? "RD" : "WR",
                    pkt->getAddr(), pkt->getSize(), curTick());

            bool isWritePkt = pkt->isWrite();
            if (!hostPorts[portIdx]->sendTimingReq(pkt)) {
                DPRINTF(PCIe, "  [SEND] Host[%u] xbar BUSY\n", portIdx);
                hostPorts[portIdx]->needRetry = true;
                ++it;
                continue;
            }

            PhaseTracker &t = isWritePkt ? wrTracker : rdTracker;
            if (t.firstHostSend == 0) t.firstHostSend = curTick();
            t.lastHostSend = curTick();

            it = pendingHostReqs[sp].erase(it);
        }
    }
    nextHostSendPort = (nextHostSendPort + 1) % n;

    // Reschedule if entries remain. Look for earliest deferred across
    // all per-port queues.
    bool anyPending = false;
    Tick nextEligible = earliestDeferred;
    for (unsigned sp = 0; sp < n; sp++) {
        if (!pendingHostReqs[sp].empty()) {
            anyPending = true;
            for (auto &req : pendingHostReqs[sp])
                if (req.earliestSend < nextEligible)
                    nextEligible = req.earliestSend;
        }
    }
    if (anyPending && !hostSendEvent.scheduled()) {
        Tick next = std::max(curTick() + 1, nextEligible);
        schedule(hostSendEvent, next);
    }
}
void PCIeModel::retryHostSend() {
    size_t totalPending = 0;
    for (auto &q : pendingHostReqs) totalPending += q.size();
    DPRINTF(PCIe, "  [RETRY] Host retry @%llu (pending_total=%lu)\n",
            curTick(), totalPending);
    trySendToHost();
}

// ====================================================================
//  Phase Breakdown
// ====================================================================

void PCIeModel::printPhaseBreakdown(PhaseTracker &t, const char *label)
{
    if (t.reqCount == 0) return;
    inform("PCIe %s phase breakdown (%u reqs, %u resps):", label,
           t.reqCount, t.respCount);
    inform("  DevReq:    first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstDevReq, t.lastDevReq,
           t.lastDevReq - t.firstDevReq,
           (t.lastDevReq - t.firstDevReq) / 1000.0);
    inform("  TlpDone:   first=%llu last=%llu span=%llu (%.1f ns)",
           t.firstTlpDone, t.lastTlpDone,
           t.lastTlpDone - t.firstTlpDone,
           (t.lastTlpDone - t.firstTlpDone) / 1000.0);
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
    t.reset();
}

// ====================================================================
//  Statistics
// ====================================================================

PCIeModel::PCIeStats::PCIeStats(PCIeModel &owner)
    : statistics::Group(&owner),
      ADD_STAT(totalReadTLPs, statistics::units::Count::get(),
               "Memory Read TLPs"),
      ADD_STAT(totalWriteTLPs, statistics::units::Count::get(),
               "Memory Write TLPs"),
      ADD_STAT(totalCompletionTLPs, statistics::units::Count::get(),
               "Completion TLPs"),
      ADD_STAT(completionsCombined, statistics::units::Count::get(),
               "DDR5 responses deferred (combined into larger CplD)"),
      ADD_STAT(totalReadBytes, statistics::units::Byte::get(),
               "Read request bytes"),
      ADD_STAT(totalWriteBytes, statistics::units::Byte::get(),
               "Write request bytes"),
      ADD_STAT(totalCompletionBytes, statistics::units::Byte::get(),
               "Completion data bytes"),
      ADD_STAT(totalWireBytes, statistics::units::Byte::get(),
               "Total bytes on wire (incl LCRC/framing/ECRC)"),
      ADD_STAT(totalPaddingBytes, statistics::units::Byte::get(),
               "DW alignment padding bytes"),
      ADD_STAT(readRequests, statistics::units::Count::get(),
               "Read requests from device"),
      ADD_STAT(writeRequests, statistics::units::Count::get(),
               "Write requests from device"),
      ADD_STAT(tagExhausted, statistics::units::Count::get(),
               "Tag exhaustion stalls"),
      ADD_STAT(creditStalls, statistics::units::Count::get(),
               "Credit exhaustion stalls"),
      ADD_STAT(assemblyBottleneckTLPs, statistics::units::Count::get(),
               "TLPs where assembly was slower than serialization"),
      ADD_STAT(completionsBuffered, statistics::units::Count::get(),
               "Completions passed through reorder buffer"),
      ADD_STAT(readLatencyHist, statistics::units::Tick::get(),
               "Read latency distribution"),
      ADD_STAT(writeLatencyHist, statistics::units::Tick::get(),
               "Write latency distribution"),
      ADD_STAT(readCoalesceHist, statistics::units::Count::get(),
               "Read coalesce group size (beats per MRd TLP)"),
      ADD_STAT(writeCoalesceHist, statistics::units::Count::get(),
               "Write coalesce group size (beats per MWr TLP)"),
      ADD_STAT(avgReadLatency, statistics::units::Tick::get(),
               "Average read latency (per-group, correct)"),
      // Review fix #2: REMOVED avgWriteLatency. totalWriteLatency
      // accumulates per-beat at DDR5 commit (line ~1355), but
      // writeRequests counts per-group (line 1069), so the ratio was
      // inflated by coalesce factor. Use writeLatencyHist for accurate
      // per-beat distribution or totalWriteLatency / beatsCommitted
      // (not tracked) for per-beat average. avgReadLatency stays: PCIe
      // read sum is correctly per-group already.
      ADD_STAT(totalReadLatency, statistics::units::Tick::get(),
               "Cumulative read latency (per-group)"),
      ADD_STAT(totalWriteLatency, statistics::units::Tick::get(),
               "Cumulative write latency (per-beat at DDR5 commit)")
{
    readLatencyHist.init(100);
    writeLatencyHist.init(100);
    readCoalesceHist.init(20);   // 1-20 beats per group
    writeCoalesceHist.init(20);
    avgReadLatency = totalReadLatency / readRequests;
}

// ====================================================================
//  Per-port stuck dump — comprehensive single-port state in one call.
//  Mirror of CxlModel::dumpPortState. See cxl_model.cc for the full
//  rationale comment block.
// ====================================================================
void
PCIeModel::dumpPortState(unsigned p, const char *where)
{
    if (p >= devicePorts.size()) return;

    Tick rdLast = (p < lastReadBufferTicks.size()) ? lastReadBufferTicks[p] : 0;
    Tick wrLast = (p < lastWriteBufferTicks.size()) ? lastWriteBufferTicks[p] : 0;
    Tick drainLast = (p < perPortLastDrainTick.size())
                       ? perPortLastDrainTick[p] : 0;
    Tick now = curTick();

    inform("PCIe-PORT-STUCK [%s] port=%u @%llu (%.2f ms idle since drain)",
           where, p, now, (now - drainLast) / 1e9);
    inform("  queues: rdBuf=%lu wrBuf=%lu upQ=%lu dnQ=%lu hostQ=%lu respQ=%lu",
           p < deviceReadBuffers.size() ? deviceReadBuffers[p].size() : 0,
           p < deviceWriteBuffers.size() ? deviceWriteBuffers[p].size() : 0,
           p < upstreamQueue.size() ? upstreamQueue[p].size() : 0,
           p < downstreamQueue.size() ? downstreamQueue[p].size() : 0,
           p < pendingHostReqs.size() ? pendingHostReqs[p].size() : 0,
           p < responseQueue.size() ? responseQueue[p].size() : 0);
    if (p < credits.size()) {
        inform("  credits: ph=%d/%d pd=%d/%d nph=%d/%d "
               "cplh=%d/%d cpld=%d/%d",
               credits[0].phCredits, credits[0].phMax,
               credits[0].pdCredits, credits[0].pdMax,
               credits[0].nphCredits, credits[0].nphMax,
               credits[0].cplhCredits, credits[0].cplhMax,
               credits[0].cpldCredits, credits[0].cpldMax);
    }
    inform("  resources: outWr=%u perPortRds=%u rrb=%u defCred=%lu",
           p < outstandingWrites.size() ? outstandingWrites[p] : 0,
           p < perPortReadsOut.size() ? perPortReadsOut[p] : 0,
           p < rrbOccupied.size() ? rrbOccupied[p] : 0,
           p < deferredCredits.size() ? deferredCredits[p].size() : 0);
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
           (int)creditReturnEvent.scheduled());

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

    if (p < credits.size()) {
        if (credits[0].nphCredits == 0) {
            inform("  >>> NPH-STARVED: 0/%d. Pending returns: %lu.",
                   credits[0].nphMax,
                   p < deferredCredits.size() ? deferredCredits[p].size() : 0);
        }
        if (credits[0].phCredits == 0) {
            inform("  >>> PH-STARVED: 0/%d. Pending returns: %lu.",
                   credits[0].phMax,
                   p < deferredCredits.size() ? deferredCredits[p].size() : 0);
        }
        if (credits[0].cplhCredits == 0) {
            inform("  >>> CPLH-STARVED: 0/%d.", credits[0].cplhMax);
        }
    }
}

void
PCIeModel::checkPortStalls()
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
        if (lastDrain == 0 || (now - lastDrain) > 5000000) {
            dumpPortState(p, "5ms-stall");
        }
    }
}

} // namespace gem5