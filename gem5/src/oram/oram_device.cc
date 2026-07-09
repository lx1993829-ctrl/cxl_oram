#include "oram/oram_device.hh"
#include "oram/aes_gcm_sw.hh"

#include <cstring>
#include <algorithm>
#include <numeric>
#include <deque>
#include <unordered_map>
#include <unordered_set>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/Oram.hh"
#include "sim/sim_exit.hh"
#include "sim/system.hh"

namespace gem5
{

// Multi-instance coordination statics
unsigned OramDevice::totalInstances = 0;
unsigned OramDevice::completedInstances = 0;

// Per-instance BRESP posmap tracking moved to oram_device.hh as member variables.

// === DEBUG N=4 CRASH: file-scope flag to trigger send ring dump ===
static bool s_dbgSendDumpRequested[64] = {false};

bool OramDevice::MemPort::recvTimingResp(PacketPtr pkt)
{
    // Step 5/6/8: result-packet completions carry a ResultSenderState tag
    // (now with opIdx) pushed by sendResultPacket(). When the response
    // arrives, find the matching entry in cmdQueue (must be at front
    // because ops complete in dispatch order under depth=1; with depth>1
    // we still expect FIFO commit ordering for the head op). Mark it
    // COMMITTED, pop from queue front, increment cpuOpCount.
    Packet::SenderState *ss = pkt->popSenderState();
    if (auto *rss = dynamic_cast<OramDevice::ResultSenderState*>(ss)) {
        if (!d.cmdQueue.empty() &&
            d.cmdQueue.front().opIdx == rss->opIdx) {
            DPRINTF(Oram, "[%lu] result pkt opIdx=%lu committed, "
                    "popping from cmdQueue\n",
                    d.oramCycle, rss->opIdx);
            const CmdEntry &op = d.cmdQueue.front();
            // Snapshot rdata for legacy MMIO 0x200 / 0xD4 reads. With
            // depth=1 these stay in sync with the freshly-committed
            // op exactly like the prior cmdOp.rdata semantics.
            for (int i = 0; i < 8; i++)
                d.lastCompletedRdata[i] = op.rdata[i];
            d.lastCompletedRdataValid = op.rdata_valid;
            d.cmdQueue.front().phase = CmdEntry::Phase::COMMITTED;
            d.cmdQueue.front().commitTick = curTick();

            // --- E2E accumulation ---
            {
                const CmdEntry &e = d.cmdQueue.front();
                if (e.dispatchTick > 0 && e.rtlDoneTick > 0 && e.commitTick > 0) {
                    Tick clk       = d.oramClkPeriod;
                    Tick rtlTime   = e.rtlDoneTick - e.dispatchTick;
                    Tick writeback = e.commitTick   - e.rtlDoneTick;
                    inform("E2E[%u] op=%lu: RTL=%lu cyc (%.1f ns), "
                           "writeback=%lu cyc (%.1f ns), "
                           "steady=%lu cyc (%.1f ns)",
                           d.instanceId, e.opIdx,
                           rtlTime / clk, rtlTime / 1000.0,
                           writeback / clk, writeback / 1000.0,
                           (rtlTime + writeback) / clk,
                           (rtlTime + writeback) / 1000.0);
                    d.e2eRtlSum       += rtlTime;
                    d.e2eWritebackSum += writeback;
                    if (d.e2eOpsTracked == 0 && e.fetchTick > 0)
                        d.e2eColdStart = e.commitTick - e.fetchTick;
                    d.e2eOpsTracked++;
                }
            }

            // Pop queue and increment cpuOpCount when result BRESP arrives.
            d.cmdQueue.pop_front();
            d.cpuOpCount++;

            if (d.cpuDriven && d.cpuOpCount >= d.numOps && !d.statsPrinted)
                d.printStats();

            // Step 9: queue opened up — kick ring fetches if pending.
            if (d.cmdRingBase != 0) {
                while (d.ringConsIdx < d.ringProdIdxKnown &&
                       d.cmdQueue.size() + d.ringEntriesInFlight
                           < d.cmdQueueDepth) {
                    uint64_t slotIdx = d.ringConsIdx;
                    uint64_t ringPos = slotIdx % d.cmdRingDepth;
                    Addr entryAddr  = d.cmdRingBase
                                    + OramDevice::CMD_RING_ENTRIES_OFFSET
                                    + ringPos * OramDevice::CMD_RING_ENTRY_BYTES;
                    auto *entSs = new OramDevice::CmdRingFetchSenderState(
                        OramDevice::CmdRingFetchSenderState::Kind::CMD_ENTRY,
                        slotIdx, slotIdx);
                    d.sendCmdRingRead(entryAddr, entSs);
                    d.ringConsIdx++;
                }
                if (d.ringNeedsProdRead && !d.ringFetchInFlight) {
                    d.ringNeedsProdRead = false;
                    d.ringFetchInFlight = true;
                    Addr prodAddr = d.cmdRingBase
                                  + OramDevice::CMD_RING_PROD_IDX_OFFSET;
                    auto *prSs = new OramDevice::CmdRingFetchSenderState(
                        OramDevice::CmdRingFetchSenderState::Kind::PROD_IDX);
                    d.sendCmdRingRead(prodAddr, prSs);
                }
            }
        } else {
            warn("Result-pkt opIdx=%lu arrived but cmdQueue front "
                 "is %s opIdx=%lu — out-of-order commit not handled",
                 rss->opIdx,
                 d.cmdQueue.empty() ? "empty" : "occupied",
                 d.cmdQueue.empty() ? 0 : d.cmdQueue.front().opIdx);
        }
        delete ss;
        delete pkt;
        return true;
    }

    // Step 9: command-ring fetch responses — handled by ring fetcher.
    if (auto *css = dynamic_cast<OramDevice::CmdRingFetchSenderState*>(ss)) {
        // DIAG: dump raw packet data at arrival from CxlModel
        if (css->kind == OramDevice::CmdRingFetchSenderState::Kind::CMD_ENTRY &&
            pkt->hasData()) {
            uint8_t *raw = pkt->getPtr<uint8_t>();
            uint32_t rawSlot = 0;
            memcpy(&rawSlot, raw, 4);
            DPRINTF(Oram, "RING-RECV slot=%lu addr=0x%lx size=%u raw_slot_addr=0x%x "
                   "bytes[0..3]=%02x %02x %02x %02x\n",
                   css->ringSlot, pkt->getAddr(), pkt->getSize(), rawSlot,
                   raw[0], raw[1], raw[2], raw[3]);
        }
        d.handleCmdRingResp(pkt, css);
        delete ss;
        delete pkt;
        return true;
    }

    pkt->pushSenderState(ss);
    d.handleMemResp(pkt);
    return true;
}

void OramDevice::MemPort::recvReqRetry()
{
    DPRINTF(Oram, "recvReqRetry: %s unblocked (retry=%d)\n",
            isPcie ? "PCIe" : "HBM",
            isPcie ? (int)(d.pcieWriteRetryQ.size() + d.pcieReadRetryQ.size())
                   : (int)d.hbmRetryQueue.size());
    if (isPcie) { d.pcieBlocked = false; d.trySendRetries(true); }
    else        {
        DPRINTF(Oram, "[%lu] BP-RETRY: recvReqRetry fired, "
               "hbmBlocked false->clearing (retryQ=%d, snapshot=%d, FSM=%d)",
               d.oramCycle, (int)d.hbmRetryQueue.size(),
               (int)d.hbmBlockedSnapshot, (int)d.oram->dbg_oram_state);
        d.hbmBlocked = false;
        d.trySendRetries(false);
        d.drainPendingSends();
    }
}


// =============================================================================
// Constructor
// =============================================================================

OramDevice::OramDevice(const OramDeviceParams &p)
    : ClockedObject(p),
      // NOTE: cmdBase/resultBufBase/cpuDriven MUST be initialized BEFORE
      // cmdPort, because the cmdPort's getAddrRanges() depends on cmdBase.
      // Various gem5 infrastructure (xbar port binding, range queries)
      // may call getAddrRanges immediately after port construction. If
      // cmdBase is still 0, the xbar caches a bogus [0x0, 0x100) range.
      cpuDriven(p.cpu_driven),
      numLogicalClients(p.num_logical_clients),
      cmdBase(p.cmd_base),
      resultBufBase(p.result_buf_base),
      resultBufSize(p.result_buf_size),
      hbmPort(p.name + ".hbm_port", *this, false),
      pciePort(p.name + ".pcie_port", *this, true),
      cmdPort(p.name + ".cmd_port", *this),
      verilatedCtx(new VerilatedContext), oram(nullptr),
      oramCycle(0),
      tickEvent([this] { tick(); }, name()),
      hbmBase(p.hbm_base), hostBase(p.host_base),
      stashOffset(p.stash_offset),
      localPct(p.local_pct), numSlots(p.num_slots),
      currentOpIsPcie(false),
      rng(42 + totalInstances), nextBurstSeq(0),
      hbmBlocked(false), pcieBlocked(false), hbmBlockedSnapshot(false),
      prevRready(0), prevBready(0),
      ctrlState(OramState::RESET),
      initSlotIdx(0), initPhase(0), grantPhase(0), writeInitIdx(0),
      leaseToken(0), numOps(p.num_ops), opsCompleted(0),
      currentOpIsWrite(false), currentOpAddr(0),
      lastWrittenSlot(0),
      localOps(0), pcieOps(0),
      opStartCycle(0), totalOpCycles(0),
      statsPrinted(false), tagMismatchWarned(false), prevPmState(0),
      // Step 4: optional "gated start" — when true, ORAM holds in
      // post-reset state without ticking until the binary writes 1
      // to cmdBase + GATE_RELEASE_OFFSET. Skips the per-cycle
      // Verilator eval() cost during NVMe setup phases.
      gatedStart(p.gated_start),
      gateReleased(false)
{
    instanceId = totalInstances++;
    panic_if(instanceId >= 64,
             "OramDevice: instanceId=%u >= 64. Increase static debug "
             "array sizes in oram_device.cc", instanceId);
    oramClkPeriod = p.oram_freq;
    hbmCdcLatency = p.hbm_cdc_latency;
    wStallCount = 0;
    noProgressCount = 0;

    // Step 3: cpu_driven mode lease table
    leaseTokens.assign(numLogicalClients, 0);
    currentGrantClient = 0;
    ready = false;
    activeHwClient = 0;

    // Step 4: staging command slot all zero. Step 9: ring state initialized
    // to "no fetch in flight, ring empty". Step 8: lastCompletedRdata
    // snapshot for legacy MMIO reads.
    memset(&stagingCmd, 0, sizeof(stagingCmd));
    memset(lastCompletedRdata, 0, sizeof(lastCompletedRdata));
    lastCompletedRdataValid = false;
    cmdQueueDepth = p.cmd_queue_depth;
    cmdRingBase   = p.cmd_ring_base;
    cmdRingDepth  = p.cmd_ring_depth;
    ringConsIdx        = 0;
    ringProdIdxKnown   = 0;
    ringFetchInFlight  = false;
    ringNeedsProdRead  = false;
    ringEntriesInFlight = 0;

    // Step 5: shadow rdata buffers empty, op counter at 0
    memset(rdataShadow, 0, sizeof(rdataShadow));
    rdataShadowValid[0] = false;
    rdataShadowValid[1] = false;
    rdataValidEverSeen[0] = false;
    rdataValidEverSeen[1] = false;
    clientDoneSeen = false;
    clientDoneWaitCycles = 0;
    cpuOpCount = 0;
    wrDbg.reset();
    memset(phaseCycles, 0, sizeof(phaseCycles));
    memset(opPhaseCycles, 0, sizeof(opPhaseCycles));
    prevFsmState = 0;
    e2eOpsTracked = 0;
    e2eRtlSum = e2eWritebackSum = 0;
    e2eColdStart = 0;
    stashHbmBeats = stashPcieBeats = 0;
    bucketHbmBeats = bucketPcieBeats = 0;
    ivtBeats = slotrBeats = bmetaBeats = pmBeats = htBeats = 0;
    if (numSlots > MAX_SLOTS) {
        warn("num_slots=%u > N=%d, clamping", numSlots, MAX_SLOTS);
        numSlots = MAX_SLOTS;
    }
    hbmSlotCount = (numSlots * localPct) / 100;
    if (hbmSlotCount == 0 && localPct > 0) hbmSlotCount = 1;
    // Bucket routing: buckets whose slots are ALL local go to HBM.
    // Partially-local buckets go to host DDR5 (conservative: avoids
    // split-memory coherence bug where two ops on different slots in
    // the same bucket route to different physical memories).
    hbmBucketCount = hbmSlotCount / ORAM_C;
    inform("ORAM: %u slots, HBM=%u, host=%u (buckets: HBM=%u, host=%u)",
           numSlots, hbmSlotCount, numSlots - hbmSlotCount,
           hbmBucketCount,
           ((numSlots + ORAM_C - 1) / ORAM_C) - hbmBucketCount);
    if (cpuDriven) {
        inform("ORAM: cpu_driven mode, K=%u clients, cmd_base=0x%lx, "
               "result_buf_base=0x%lx",
               numLogicalClients, cmdBase, resultBufBase);
    }
}

OramDevice::~OramDevice()
{
    printStats();
    if (oram) { oram->final(); delete oram; }
    delete verilatedCtx;
}

void OramDevice::init()
{
    ClockedObject::init();
    if (!hbmPort.isConnected()) fatal("hbm_port not connected");
    if (!pciePort.isConnected()) fatal("pcie_port not connected");
    reqId = params().system->getRequestorId(this, name());

    // CRITICAL: cmd_port is a ResponsePort that advertises its address
    // range lazily via getAddrRanges(). We must notify the upstream xbar
    // so it can finalize its port map (gotAllAddrRanges). Without this
    // call, the xbar's findPort() asserts on every packet to cmd_port.
    // Matches the pattern used by PioDevice::init().
    if (cpuDriven && cmdPort.isConnected()) {
        cmdPort.sendRangeChange();
    }
}

void OramDevice::startup()
{
    const char *verilator_argv[] = {"gem5"};
    verilatedCtx->commandArgs(1, verilator_argv);
    oram = new Vsecure_oram_top(verilatedCtx);

    oram->clk = 0; oram->rst_n = 0;
    oram->mgmt_req = 0; oram->mgmt_op = 0;
    oram->mgmt_lease_id = 0; oram->mgmt_client_id = 0;
    oram->mgmt_base_addr = 0; oram->mgmt_size = 0;
    oram->mgmt_duration = 0; oram->mgmt_token_in = 0;
    oram->client_req = 0; oram->client_op = 0;
    oram->client_slot_addr = 0; oram->client_token = 0;
    oram->client_lease_id = 0;
    memset(&oram->client_wdata[0], 0, sizeof(oram->client_wdata));
    oram->client_wdata_valid = 0;
    oram->init_mode = 0; oram->init_pm_wr_addr = 0;
    oram->init_pm_wr_bucket = 0; oram->init_pm_wr_status = 0;
    oram->init_pm_wr_en = 0; oram->init_bm_wr_bucket = 0;
    memset(&oram->init_bm_wr_slot_list[0], 0,
           sizeof(oram->init_bm_wr_slot_list));
    oram->init_bm_wr_fill = 0; oram->init_bm_wr_en = 0;
    // Constrain bucket remaps to [0, numBuckets) instead of [0, 8191).
    // The RTL's 13-bit PRNG covers the full hardware range, but with
    // fewer slots the actual bucket count is much smaller.
    // dbg_prng_override_en + dbg_prng_override overrides the RTL PRNG
    // each cycle with a uniform value in [0, numBuckets).
    oram->dbg_prng_override = 0;
    oram->dbg_prng_override_en = 1;  // always use gem5's constrained PRNG
    oram->dbg_force_same_bucket = 0;
    // hold_stash / hold_ddr_wr removed — not in new RTL

    const auto &p = params();
    oram->aes_key[0] = p.aes_key_0; oram->aes_key[1] = p.aes_key_1;
    oram->aes_key[2] = p.aes_key_2; oram->aes_key[3] = p.aes_key_3;
    oram->aes_iv_seed[0] = 0xDEADBEEF;
    oram->aes_iv_seed[1] = 0xCAFEBABE;
    oram->aes_iv_seed[2] = 0x12345678;

    oram->m_axi_arready = 0; oram->m_axi_awready = 0;
    oram->m_axi_wready = 0;
    oram->m_axi_rvalid = 0; oram->m_axi_rid = 0;
    memset(&oram->m_axi_rdata[0], 0, sizeof(oram->m_axi_rdata));
    oram->m_axi_rresp = 0; oram->m_axi_rlast = 0;
    oram->m_axi_bvalid = 0; oram->m_axi_bid = 0; oram->m_axi_bresp = 0;

    for (int i = 0; i < 40; i++) { oram->clk = !oram->clk; oram->eval(); }
    oram->rst_n = 1;
    for (int i = 0; i < 20; i++) { oram->clk = !oram->clk; oram->eval(); }

    prevRready = 0; prevBready = 0;
    memset(lastWrittenData, 0, sizeof(lastWrittenData));
    ctrlState = OramState::INIT_SLOTS;

    // Step 4: gated start. When the Python config sets gated_start=True,
    // hold off the first tick until the binary writes 1 to
    // cmdBase + GATE_RELEASE_OFFSET. This avoids per-cycle Verilator
    // eval() cost during NVMe setup phases that don't need ORAM.
    // RTL state at this point: rst_n=1, post-reset cycles consumed,
    // ready to begin INIT_SLOTS as soon as we start ticking.
    if (gatedStart) {
        DPRINTF(Oram, "ORAM gated_start=1: holding init at INIT_SLOTS, "
                "waiting for gate release MMIO write to cmd_base+0x%x\n",
                GATE_RELEASE_OFFSET);
        // Don't scheduleTick(). The first tick will fire when
        // handleCmdPortReq sees a gate release write.
    } else {
        scheduleTick();
    }
}

Port &OramDevice::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "hbm_port") return hbmPort;
    if (if_name == "pcie_port") return pciePort;
    if (if_name == "cmd_port") return cmdPort;
    return ClockedObject::getPort(if_name, idx);
}

// =============================================================================
// Address routing
// =============================================================================

bool OramDevice::isStashAddr(Addr a) { return a >= stashOffset; }

// Classify an AXI address (instance-relative) into a metadata region.
// Used for debug logging + per-region beat accounting. Ranges match
// oram_params.vh. STASH is data, not counted as metadata here.
const char* OramDevice::metaRegionTag(Addr a)
{
    if (a >= BUCKET_META_BASE_ADDR) return "BUCKET_META";
    if (a >= SLOT_R_BASE_ADDR)      return "SLOT_R";
    if (a >= IVT_BASE_ADDR)         return "IVT";
    if (a >= HT_BKT_NEXT_ADDR)      return "HT_BKT_NEXT";
    if (a >= HT_BKT_HEAD_ADDR)      return "HT_BKT_HEAD";
    if (a >= HT_SLOT_BASE_ADDR)     return "HT_SLOT";
    if (a >= PM_BASE_ADDR)          return "PM";
    if (a >= STASH_BASE_ADDR)       return "STASH";
    return "BUCKET";
}
bool OramDevice::isHostSlot(uint32_t s) { return s >= hbmSlotCount; }
bool OramDevice::isHostBucket(uint32_t b) { return b >= hbmBucketCount; }

// =============================================================================
// Tick — the heart of the simulation
//
// AXI timing model:
//   1. Set ready signals (arready, awready, wready) and drive R/B channels
//   2. Sample ALL RTL master outputs BEFORE eval (they're registered from
//      the previous posedge — this is what the RTL is presenting NOW)
//   3. eval() — RTL transitions to next state based on our ready signals
//   4. Process handshakes using pre-eval saved values
//   5. Negedge eval for completeness
// =============================================================================

void OramDevice::scheduleTick()
{
    if (!tickEvent.scheduled())
        schedule(tickEvent, curTick() + oramClkPeriod);
}

void OramDevice::tick()
{
    oram->clk = 1;

    // Constrain RTL's bucket remap to [0, numBuckets-1].
    // numBuckets = ceil(numSlots / ORAM_C), where ORAM_C = 4.
    // ORAM_C is the nominal slots per bucket (ORAM_Z=8 total positions, ORAM_C=4 real + 4 slack).
    {
        uint32_t numBuckets = (numSlots + ORAM_C - 1) / ORAM_C;
        if (numBuckets > 1) {
            std::uniform_int_distribution<uint32_t> dist(0, numBuckets - 1);
            oram->dbg_prng_override = dist(rng);
        }
    }

    // Step 9: ring fetcher — kick off a prod_idx read when the CPU has
    // signaled (via doorbell) that it has produced new commands and we
    // don't already have a fetch in flight. The actual entry reads are
    // chained off the prod_idx response in handleCmdRingResp.
    if (cmdRingBase != 0 && ringNeedsProdRead && !ringFetchInFlight) {
        ringNeedsProdRead = false;
        ringFetchInFlight = true;
        Addr prodAddr = cmdRingBase + CMD_RING_PROD_IDX_OFFSET;
        auto *ss = new CmdRingFetchSenderState(
            CmdRingFetchSenderState::Kind::PROD_IDX);
        sendCmdRingRead(prodAddr, ss);
    }

    // --- Step 1: Set slave-side signals ---

    // AR/AW ready: stall when the target port can't accept.
    // For bucket traffic (the common case): use hbmBlockedSnapshot so
    // backpressure persists for 1 full tick (recvReqRetry clears
    // hbmBlocked between ticks, making it invisible otherwise).
    // For stash/posmap: these are short single-burst transactions;
    // the retry queue handles any transient backpressure.
    {
        // AR/AW ready: set based on whether gem5 can accept, not on which
        // RTL master is active. The RTL mux routes ready to the correct
        // master (pm_arready = sel_posmap ? m_axi_arready : 0, etc).
        //
        // Gate stash single-beat reads: the HT_WAIT_RD hold pattern keeps
        // st_arvalid high every cycle. Without the gate, gem5 handshakes
        // a new AR each cycle, flooding pendingReadBursts.
        // Gate stash single-beat writes: same issue with HT_WAIT_WR.
        //
        // The gate checks pendingReadBursts for totalBeats==1 (stash
        // single-beat) and activeWrites for isStash && len==0.
        bool stashReadPending = false;
        for (auto &rb : pendingReadBursts) {
            if (rb.totalBeats == 1) { stashReadPending = true; break; }
        }
        // Also hold off if a single-beat HT read's beat is still sitting in
        // rQueue undelivered. The single-beat port is single-outstanding on the
        // RTL side; allowing a 2nd single-beat AR while the 1st beat is still in
        // rQueue lets two single-read beats coexist and be delivered FIFO
        // (completion order) rather than AR-issue order -> the RTL captures the
        // wrong read's data. Keep one single-beat read in flight end-to-end.
        if (!stashReadPending) {
            for (auto &q : rQueue) {
                if (q.isSingle) { stashReadPending = true; break; }
            }
        }
        oram->m_axi_arready = (hbmBlocked || pcieBlocked || stashReadPending) ? 0 : 1;

        // === DEBUG N=4 CRASH: arready gate diagnostics ===
        {
            static uint64_t hbmBlockedCount[64] = {0};
            static uint64_t stashPendingCount[64] = {0};
            if (hbmBlocked) {
                hbmBlockedCount[instanceId]++;
                if (hbmBlockedCount[instanceId] == 1 ||
                    hbmBlockedCount[instanceId] == 100 ||
                    hbmBlockedCount[instanceId] % 5000 == 0)
                    warn("[DBG] inst=%u cyc=%lu hbmBlocked for %lu ticks "
                         "(retryQ=%d rQ=%d pendRd=%lu pendSend=%lu wFifo=%d)",
                         instanceId, oramCycle, hbmBlockedCount[instanceId],
                         (int)hbmRetryQueue.size(), (int)rQueue.size(),
                         pendingReadBursts.size(), pendingReadSends.size(),
                         (int)wFifo.size());
            } else {
                if (hbmBlockedCount[instanceId] > 50)
                    warn("[DBG] inst=%u cyc=%lu hbmBlocked CLEARED after %lu ticks",
                         instanceId, oramCycle, hbmBlockedCount[instanceId]);
                hbmBlockedCount[instanceId] = 0;
            }
            if (stashReadPending) {
                stashPendingCount[instanceId]++;
                if (stashPendingCount[instanceId] == 500 ||
                    stashPendingCount[instanceId] % 5000 == 0)
                    warn("[DBG] inst=%u cyc=%lu stashReadPending for %lu ticks "
                         "(pendRd=%lu front.beats=%d/%d front.flushed=%d rQ=%d "
                         "FSM=%d arvalid=%d)",
                         instanceId, oramCycle, stashPendingCount[instanceId],
                         pendingReadBursts.size(),
                         pendingReadBursts.empty() ? -1 : pendingReadBursts.front().beatsRecv,
                         pendingReadBursts.empty() ? -1 : pendingReadBursts.front().totalBeats,
                         pendingReadBursts.empty() ? -1 : (int)pendingReadBursts.front().flushedBeats,
                         (int)rQueue.size(),
                         (int)oram->dbg_oram_state,
                         (int)oram->m_axi_arvalid);
            } else {
                stashPendingCount[instanceId] = 0;
            }
        }
        // AW: always accept. The stash_axi_master issues AW+W simultaneously
        // (ST_IDLE → ST_SNG_WRITE sets both awvalid=1 and wvalid=1 on the same
        // cycle). If awready=0 but wready=1, the W handshake fires without a
        // matching activeWrites entry — the W beat is silently dropped and the
        // stash_axi_master waits for a BRESP that never comes (Group A deadlock).
        // hbmBlocked only affects the gem5→HBM path (wFifo/drainWriteFifo),
        // not the RTL→gem5 AXI handshake. Bucket AW flooding is not an issue
        // because the RTL's bucket master only issues 8 AWs per DDR_WRITE phase.
        oram->m_axi_awready = 1;
    }

    // W ready: depends on which port and which master.
    // Stash writes (sel_stash=1): always wready=1. The stash_axi_master's
    // W prefetch pipeline breaks if wready=0 at the wrong beat.
    // Bucket HBM writes (sel_stash=0, !isPcie): respect hbmBlocked for
    // realistic backpressure from the memory controller.
    // PCIe writes: respect pcieBlocked.
    if (!activeWrites.empty()) {
        bool wrPcie = activeWrites.front().isPcie;
        bool selStash = (oram->sel_stash_out != 0);
        if (wrPcie) {
            oram->m_axi_wready = pcieBlocked ? 0 : 1;
        } else if (selStash) {
            // Stash HBM writes: always accept (prefetch pipeline safety)
            oram->m_axi_wready = 1;
        } else {
            // Bucket HBM writes: backpressure when write FIFO is full
            oram->m_axi_wready = ((int)wFifo.size() >= W_FIFO_DEPTH) ? 0 : 1;
        }
    } else {
        // No active AW yet. Set wready=1 so the first W beat after AW
        // isn't missed (AW captured post-eval, wready set pre-eval).
        oram->m_axi_wready = 1;
    }

    // Flush rQueue on mux master transition (pre-eval check).
    // This must happen BEFORE flushCompletedReads so stale beats
    // from the previous master don't contaminate the new master.
    // Track burst_busy (not sel_stash) for stash: sel_stash goes
    // high for HT single-beat ops too, but those don't conflict
    // with bucket traffic — only burst stash ops do.
    {
        // Per-instance previous state (NOT shared across instances!)
        static uint8_t prevBurstBusy[64] = {0}, prevSelPosmap[64] = {0};
        uint8_t curBurstBusy = oram->st_burst_busy_out;
        uint8_t curSelPosmap = oram->sel_posmap_out;
        if (curBurstBusy != prevBurstBusy[instanceId] || curSelPosmap != prevSelPosmap[instanceId]) {
            DPRINTF(Oram, "[cyc %lu] MUX-SWITCH: burst_busy %d->%d, sel_pm %d->%d, rQ=%d flushed "
                   "FSM=%d ht_st=%d sel_st=%d\n",
                   oramCycle,
                   (int)prevBurstBusy[instanceId], (int)curBurstBusy,
                   (int)prevSelPosmap[instanceId], (int)curSelPosmap,
                   (int)rQueue.size(),
                   (int)oram->dbg_oram_state,
                   (int)oram->dbg_ht_state,
                   (int)oram->sel_stash_out);
            rQueue.clear();
            prevBurstBusy[instanceId] = curBurstBusy;
            prevSelPosmap[instanceId] = curSelPosmap;
        }
    }

    // Flush 1 completed read beat to rQueue. Allow up to 2 entries:
    // one being presented (will be popped by driveAxiR this tick),
    // one ready to present next tick. This prevents 1,0,1,0 alternation.
    int rqBefore = (int)rQueue.size();
    int reorderReady = 0;
    if (!pendingReadBursts.empty()) {
        auto &rb = pendingReadBursts.front();
        // Count how many contiguous beats are ready in reorder buf
        // (including CDC delay for HBM beats)
        Tick now = curTick();
        for (int i = rb.flushedBeats; i < rb.totalBeats &&
             rb.beatRecvd[i] && now >= rb.beatReadyTick[i]; i++)
            reorderReady++;
    }

    flushCompletedReads();

    int rqAfterFlush = (int)rQueue.size();

    // Drain write FIFO: send 1 queued write per tick to HBM
    drainWriteFifo();

    // Drive R and B channels to RTL
    driveAxiR();

    int rqAfterDrive = (int)rQueue.size();

    // Monitor R channel and stash HBM access
    {
        uint8_t fsm = oram->dbg_oram_state;
        static int stashMonCount[64] = {0}, ddrMonCount[64] = {0};
        static int stLoadMonCount[64] = {0}, stFlushMonCount[64] = {0};
        static int extractWrMonCount[64] = {0}, evictMonCount[64] = {0};

        // S_STASH_READ (12): reading from line_buf to client after HBM load
        if (fsm == 12 && stashMonCount[instanceId] < 50) {
            DPRINTF(Oram, "[cyc %lu] R-MON STASH_READ: rQ=%d reorder=%d rvalid=%d "
                   "rready=%d sel_st=%d sel_pm=%d pendBursts=%d wFifo=%d\n",
                   oramCycle, rqBefore, reorderReady,
                   (int)oram->m_axi_rvalid, (int)oram->m_axi_rready,
                   (int)oram->sel_stash_out, (int)oram->sel_posmap_out,
                   (int)pendingReadBursts.size(), (int)wFifo.size());
            stashMonCount[instanceId]++;
        }

        // S_ST_LOAD (28): loading stash entry from HBM into line_buf
        if (fsm == FSM_ST_LOAD && stLoadMonCount[instanceId] < 50) {
            DPRINTF(Oram, "[cyc %lu] STASH-LOAD: sel_st=%d arV=%d arR=%d rV=%d rR=%d "
                   "rQ=%d pendBursts=%d reorder=%d\n",
                   oramCycle,
                   (int)oram->sel_stash_out,
                   (int)oram->m_axi_arvalid, (int)oram->m_axi_arready,
                   (int)oram->m_axi_rvalid, (int)oram->m_axi_rready,
                   (int)rQueue.size(), (int)pendingReadBursts.size(),
                   reorderReady);
            stLoadMonCount[instanceId]++;
        }

        // S_ST_FLUSH (29): flushing line_buf to HBM
        if (fsm == FSM_ST_FLUSH && stFlushMonCount[instanceId] < 50) {
            DPRINTF(Oram, "[cyc %lu] STASH-FLUSH: sel_st=%d awV=%d awR=%d wV=%d wR=%d "
                   "bV=%d bR=%d wFifo=%d sel_sr=%d sel_bm=%d sel_iv=%d sel_pm=%d stFSM=%d\n",
                   oramCycle,
                   (int)oram->sel_stash_out,
                   (int)oram->m_axi_awvalid, (int)oram->m_axi_awready,
                   (int)oram->m_axi_wvalid, (int)oram->m_axi_wready,
                   (int)oram->m_axi_bvalid, (int)oram->m_axi_bready,
                   (int)wFifo.size(),
                   (int)oram->dbg_sel_slotr, (int)oram->dbg_sel_bmeta,
                   (int)oram->dbg_sel_ivt, (int)oram->dbg_sel_posmap,
                   (int)oram->dbg_sel_stash_fsm);
            stFlushMonCount[instanceId]++;
        }

        // S_EXTRACT_WR (10): writing client data to line_buf
        if (fsm == 10 && extractWrMonCount[instanceId] < 10) {
            DPRINTF(Oram, "[cyc %lu] EXTRACT_WR: beat=%d wdata_valid=%d\n",
                   oramCycle, (int)oram->oram_beat_cnt,
                   (int)oram->client_wdata_valid);
            extractWrMonCount[instanceId]++;
        }

        // S_EVICT (14): finding stash entries to evict
        if (fsm == 14 && evictMonCount[instanceId] < 10) {
            DPRINTF(Oram, "[cyc %lu] EVICT: sel_st=%d\n",
                   oramCycle, (int)oram->sel_stash_out);
            evictMonCount[instanceId]++;
        }

        // DDR_READ (2)
        if (fsm == FSM_DDR_READ && ddrMonCount[instanceId] < 30) {
            DPRINTF(Oram, "[cyc %lu] R-MON DDR_READ: rQ=%d reorder=%d rvalid=%d "
                   "rready=%d pendBursts=%d\n",
                   oramCycle, rqBefore, reorderReady,
                   (int)oram->m_axi_rvalid, (int)oram->m_axi_rready,
                   (int)pendingReadBursts.size());
            ddrMonCount[instanceId]++;
        }
    }

    driveAxiB();

    // Feed client write data if in active operation
    if (ctrlState == OramState::PROCESSING ||
        ctrlState == OramState::WRITE_INIT)
        feedClientWdata();

    // --- Step 2: Sample ALL RTL master outputs BEFORE eval ---
    // Bucket/stash AXI uses pre-eval (multi-beat, valid persists across ticks).
    // pos_map single-beat can be consumed within one eval — recovered below.
    uint8_t  s_arvalid = oram->m_axi_arvalid;
    Addr     s_araddr  = oram->m_axi_araddr;
    uint8_t  s_arlen   = oram->m_axi_arlen;
    uint8_t  s_arsize  = oram->m_axi_arsize;
    uint8_t  s_arid    = oram->m_axi_arid;
    uint8_t  s_arburst = oram->m_axi_arburst;

    uint8_t  s_awvalid = oram->m_axi_awvalid;
    Addr     s_awaddr  = oram->m_axi_awaddr;
    uint8_t  s_awlen   = oram->m_axi_awlen;
    uint8_t  s_awsize  = oram->m_axi_awsize;
    uint8_t  s_awid    = oram->m_axi_awid;
    uint8_t  s_awburst = oram->m_axi_awburst;

    uint8_t  s_wvalid  = oram->m_axi_wvalid;
    uint8_t  s_wlast   = oram->m_axi_wlast;
    uint32_t s_wstrb   = oram->m_axi_wstrb;
    uint8_t  s_wdata[AXI_DATA_BYTES];
    memcpy(s_wdata, &oram->m_axi_wdata[0], AXI_DATA_BYTES);

    uint8_t  s_rready  = oram->m_axi_rready;
    uint8_t  s_bready  = oram->m_axi_bready;

    uint8_t  s_sel_stash_pre = oram->sel_stash_out;
    uint8_t  s_sel_posmap_pre = oram->sel_posmap_out;

    uint8_t  s_arready = oram->m_axi_arready;
    uint8_t  s_awready = oram->m_axi_awready;
    uint8_t  s_wready  = oram->m_axi_wready;

    // --- Step 3: eval() — RTL transitions ---
    oram->eval();

    // Track pos_map state for debug
    uint8_t pmState = oram->dbg_pm_state;
    prevPmState = pmState;

    uint8_t  s_sel_stash_post = oram->sel_stash_out;
    uint8_t  s_sel_posmap_post = oram->sel_posmap_out;
    // Use POST-eval value: sel_stash/sel_posmap are combinational from fsm_state
    // which may transition during eval on the same cycle as awvalid
    uint8_t  s_sel_stash = s_sel_stash_post;
    uint8_t  s_sel_posmap = s_sel_posmap_post;

    // Warn if sel_stash/sel_posmap changed during a handshake — could mis-route
    if ((s_sel_stash_pre != s_sel_stash_post ||
         s_sel_posmap_pre != s_sel_posmap_post) &&
        (s_arvalid || s_awvalid)) {
        DPRINTF(Oram, "[%lu] WARN: sel changed during handshake "
                "(stash %d->%d posmap %d->%d arv=%d awv=%d)\n",
                oramCycle,
                (int)s_sel_stash_pre, (int)s_sel_stash_post,
                (int)s_sel_posmap_pre, (int)s_sel_posmap_post,
                (int)s_arvalid, (int)s_awvalid);
    }

    // Use pre-eval rready/bready for R/B channel consumption
    prevRready = s_rready;
    prevBready = s_bready;

    // Track FSM state cycles for phase breakdown
    uint8_t curFsmState = oram->dbg_oram_state;
    if (curFsmState < NUM_FSM_STATES) {
        phaseCycles[curFsmState]++;
        if (ctrlState == OramState::PROCESSING)
            opPhaseCycles[curFsmState]++;
    }

    // DDR_WRITE debug: track W channel behavior every cycle
    if (curFsmState == FSM_DDR_WRITE) {  // DDR_WRITE
        wrDbg.totalCycles++;
        if (s_wvalid && s_wready) {
            wrDbg.wHandshakes++;
            if (wrDbg.firstWcycle == 0) wrDbg.firstWcycle = oramCycle;
            wrDbg.lastWcycle = oramCycle;
        }
        else if (s_wvalid && !s_wready) wrDbg.wStalls++;
        else if (!s_wvalid)             wrDbg.wIdle++;
        if (s_awvalid && s_awready)     wrDbg.awHandshakes++;
        if (oram->m_axi_bvalid && oram->m_axi_bready) {
            wrDbg.brespRecv++;
            if (wrDbg.firstBcycle == 0) wrDbg.firstBcycle = oramCycle;
            wrDbg.lastBcycle = oramCycle;
        }
        if (oram->m_axi_bvalid)         wrDbg.brespAvail++;
        if (oram->m_axi_bvalid == 0 && !bQueue.empty()) wrDbg.brespNoMatch++;
        if (bQueue.size() > wrDbg.maxBqDepth) wrDbg.maxBqDepth = bQueue.size();
    }

    // DDR_READ debug: track R channel behavior every cycle
    if (curFsmState == FSM_DDR_READ) {  // DDR_READ
        rdDbg.totalCycles++;
        bool s_rvalid = oram->m_axi_rvalid;
        bool s_rready = oram->m_axi_rready;
        if (s_rvalid && s_rready) {
            rdDbg.rHandshakes++;
            if (rdDbg.firstRcycle == 0) rdDbg.firstRcycle = oramCycle;
            rdDbg.lastRcycle = oramCycle;
        }
        else if (s_rvalid && !s_rready) rdDbg.rStalls++;
        else if (!s_rvalid)             rdDbg.rIdle++;
        if (s_arvalid && s_arready)     rdDbg.arHandshakes++;
        if (rQueue.size() > rdDbg.maxRqDepth) rdDbg.maxRqDepth = rQueue.size();
        if (pendingReadBursts.size() > rdDbg.maxPendReads)
            rdDbg.maxPendReads = pendingReadBursts.size();
    }
    // --- Debug: track FSM transitions ---
    // Log DDR_READ/DDR_WRITE entry/exit with instance name.
    // Must compare BEFORE updating prevFsmState (was dead code before:
    // prevFsmState was set to curFsmState at line above, making the
    // check always false).
    if (curFsmState != prevFsmState && ctrlState == OramState::PROCESSING) {
        if (curFsmState == FSM_DDR_READ)
            inform("[%s] DDR_READ  ENTER  cyc=%lu op=%u",
                   name(), oramCycle, opsCompleted);
        else if (prevFsmState == FSM_DDR_READ)
            inform("[%s] DDR_READ  EXIT   cyc=%lu op=%u (%lu cyc in phase)",
                   name(), oramCycle, opsCompleted, opPhaseCycles[FSM_DDR_READ]);
        if (curFsmState == FSM_DDR_WRITE)
            inform("[%s] DDR_WRITE ENTER  cyc=%lu op=%u",
                   name(), oramCycle, opsCompleted);
        else if (prevFsmState == FSM_DDR_WRITE)
            inform("[%s] DDR_WRITE EXIT   cyc=%lu op=%u (%lu cyc in phase)",
                   name(), oramCycle, opsCompleted, opPhaseCycles[FSM_DDR_WRITE]);
        DPRINTF(Oram, "[%lu] FSM: %d -> %d (found_bkt=%d found_stash=%d "
               "gcm_tag_match=%d gcm_tag_valid=%d "
               "access_viol=%d oram_busy=%d)",
               oramCycle, (int)prevFsmState, (int)curFsmState,
               (int)oram->dbg_found_in_bucket,
               (int)oram->dbg_found_in_stash,
               (int)oram->dbg_gcm_tag_match,
               (int)oram->dbg_gcm_tag_valid,
               (int)oram->access_violation,
               (int)oram->oram_busy);
    }
    prevFsmState = curFsmState;

    // Log token and request details at op dispatch
    if (oram->client_req && ctrlState == OramState::PROCESSING) {
        DPRINTF(Oram, "[cyc %lu] OP-DISPATCH: req=%d op=%d addr=0x%lx "
               "token=0x%lx lease_id=%d wdata_valid=%d\n",
               oramCycle, (int)oram->client_req,
               (int)oram->client_op,
               (uint64_t)oram->client_slot_addr,
               (uint64_t)oram->client_token,
               (int)oram->client_lease_id,
               (int)oram->client_wdata_valid);
    }
    if (oram->client_req) oram->client_req = 0;

    checkRtlErrors();

    // --- Step 4: Process handshakes using pre-eval saved values ---

    // AR handshake
    if (s_arvalid && s_arready) {
        int numBeats = s_arlen + 1;
        int beatBytes = 1 << s_arsize;
        // Route: stash/posmap/HT metadata always goes to HBM.
        // sel_stash may be 0 if the AR was delayed by stashReadPending
        // gate past the whitelisted FSM state. Use address as ground truth:
        // AXI addr >= 0x10000000 is metadata region (always HBM).
        // Bucket data: route by BUCKET INDEX (not slot index) so all ops
        // on the same bucket hit the same physical memory. This prevents
        // the split-memory coherence bug where two slots sharing a bucket
        // route to different backing stores.
        bool isMetadataAddr = (s_araddr >= METADATA_REGION_START);
        bool pcie;
        if (s_sel_stash || s_sel_posmap || isMetadataAddr)
            pcie = false;  // metadata always HBM
        else
            pcie = isHostBucket((uint32_t)(s_araddr / BUCKET_BYTES));

        DPRINTF(Oram, "[%lu] AR: 0x%lx len=%d %s%s%s (sel_st=%d sel_pm=%d)\n",
                oramCycle, (uint64_t)s_araddr, (int)s_arlen,
                pcie ? "PCIe" : "HBM",
                s_sel_stash ? " (stash)" : "",
                s_sel_posmap ? " (posmap)" : "",
                (int)s_sel_stash, (int)s_sel_posmap);

        // Track routing for verification
        if (s_sel_stash || s_sel_posmap) {
            if (pcie) stashPcieBeats += numBeats; else stashHbmBeats += numBeats;
        } else {
            if (pcie) bucketPcieBeats += numBeats; else bucketHbmBeats += numBeats;
        }
        // Per-metadata-region accounting (HBM reads). All metadata must be HBM.
        if (isMetadataAddr) {
            const char* rtag = metaRegionTag(s_araddr);
            if      (!strcmp(rtag, "IVT"))         ivtBeats   += numBeats;
            else if (!strcmp(rtag, "SLOT_R"))      slotrBeats += numBeats;
            else if (!strcmp(rtag, "BUCKET_META")) bmetaBeats += numBeats;
            else if (!strcmp(rtag, "PM"))          pmBeats    += numBeats;
            else if (!strncmp(rtag, "HT", 2))      htBeats    += numBeats;
            DPRINTF(Oram, "[%lu] META-RD region=%s addr=0x%lx beats=%d %s\n",
                    oramCycle, rtag, (uint64_t)s_araddr, numBeats,
                    pcie ? "PCIe(!)" : "HBM");
            if (pcie)
                warn("[cyc %lu] META-RD MISROUTED to PCIe: region=%s addr=0x%lx",
                     oramCycle, rtag, (uint64_t)s_araddr);
        }

        size_t seq = nextBurstSeq++;
        pendingReadBursts.emplace_back(s_arid, numBeats, seq, pcie);
        
        // Data round-trip tracker: log read burst address
        Addr gem5ArAddr = pcie ? (hostBase + s_araddr) : (hbmBase + s_araddr);
        DPRINTF(Oram, "[cyc %lu] AR-ISSUE: addr=0x%lx (axi=0x%lx) len=%d seq=%lu FSM=%d %s "
               "ht_st=%d ht_op=%d sel_st=%d burst_busy=%d\n",
               oramCycle, gem5ArAddr, (uint64_t)s_araddr, (int)s_arlen,
               seq, (int)oram->dbg_oram_state,
               pcie ? "PCIe" : "HBM",
               (int)oram->dbg_ht_state, (int)oram->dbg_ht_op,
               (int)oram->sel_stash_out,
               (int)oram->st_burst_busy_out);

        // Send per-beat packets: required because the host xbar uses
        // address interleaving (bit 6) across DDR5 channels. A coalesced
        // 512B packet would span both channels and fail routing.
        // The PCIeModel handles TLP-level coalescing (MPS/MRRS splitting)
        // internally based on the packet size it receives.
        for (int i = 0; i < numBeats; i++) {
            Addr ba = axiBurstAddr(s_araddr, i, beatBytes,
                                    s_arburst, s_arlen);
            Addr ga = pcie ? (hostBase + ba) : (hbmBase + ba);
            pendingReadSends.push_back({ga, beatBytes, s_arid, i,
                                         numBeats, seq, pcie});
        }
    }

    // AW handshake — create PendingWriteBurst immediately so gem5 responses
    // that arrive before W-LAST can be matched
    if (s_awvalid && s_awready) {
        Addr axiAddr = s_awaddr;
        // Same metadata/bucket routing as AR — see comment above.
        bool isMetadataAddr = (axiAddr >= METADATA_REGION_START);
        bool pcie;
        if (s_sel_stash || s_sel_posmap || isMetadataAddr)
            pcie = false;  // metadata always HBM
        else
            pcie = isHostBucket((uint32_t)(axiAddr / BUCKET_BYTES));

        DPRINTF(Oram, "[%lu] AW: 0x%lx len=%d %s%s%s (sel_st=%d sel_pm=%d)\n",
                oramCycle, (uint64_t)axiAddr, (int)s_awlen,
                pcie ? "PCIe" : "HBM",
                s_sel_stash ? " (stash)" : "",
                s_sel_posmap ? " (posmap)" : "",
                (int)s_sel_stash, (int)s_sel_posmap);

        // Track routing for verification
        int awBeats = s_awlen + 1;
        if (s_sel_stash || s_sel_posmap) {
            if (pcie) stashPcieBeats += awBeats; else stashHbmBeats += awBeats;
        } else {
            if (pcie) bucketPcieBeats += awBeats; else bucketHbmBeats += awBeats;
        }
        // Per-metadata-region accounting (HBM writes).
        if (isMetadataAddr) {
            const char* rtag = metaRegionTag(axiAddr);
            if      (!strcmp(rtag, "IVT"))         ivtBeats   += awBeats;
            else if (!strcmp(rtag, "SLOT_R"))      slotrBeats += awBeats;
            else if (!strcmp(rtag, "BUCKET_META")) bmetaBeats += awBeats;
            else if (!strcmp(rtag, "PM"))          pmBeats    += awBeats;
            else if (!strncmp(rtag, "HT", 2))      htBeats    += awBeats;
            DPRINTF(Oram, "[%lu] META-WR region=%s addr=0x%lx beats=%d %s\n",
                    oramCycle, rtag, (uint64_t)axiAddr, awBeats,
                    pcie ? "PCIe(!)" : "HBM");
            if (pcie)
                warn("[cyc %lu] META-WR MISROUTED to PCIe: region=%s addr=0x%lx",
                     oramCycle, rtag, (uint64_t)axiAddr);
        }

        size_t seq = nextBurstSeq++;

        WriteBurst wb;
        wb.baseAddr = axiAddr;
        wb.len = s_awlen; wb.size = s_awsize;
        wb.id = s_awid; wb.burst = s_awburst;
        wb.beatsRecv = 0; wb.isPcie = pcie;
        wb.writeSeq = seq;
        // isStash: true for stash burst or HT single-beat writes.
        // Use post-eval sel_stash (high when stash_axi_master is driving AXI).
        // pos_map writes have sel_posmap=1 and sel_stash=0 — NOT tagged as stash.
        wb.isStash = oram->sel_stash_out ? true : false;
        activeWrites.push_back(wb);
        // PendingWriteBurst created at wlast, not here.
        // Creating it here causes deadlock if RTL issues AW
        // but never sends W data (oram_busy stall at N>2).

        // Debug: log every AW that targets the HT region
        {
            Addr gem5AwAddr = pcie ? (hostBase + axiAddr) : (hbmBase + axiAddr);
            Addr htRegionStart = hbmBase + HT_SLOT_BASE_ADDR;
            Addr htRegionEnd   = hbmBase + HT_REGION_END;   // through end of IVT
            if (gem5AwAddr >= htRegionStart && gem5AwAddr < htRegionEnd) {
                DPRINTF(Oram, "[cyc %lu] HT_AW_WRITE: addr=0x%lx (axi=0x%lx) len=%d "
                       "FSM=%d ht_st=%d sel_st=%d sel_pm=%d burst_busy=%d isStash=%d\n",
                       oramCycle, gem5AwAddr, (uint64_t)axiAddr, (int)s_awlen,
                       (int)oram->dbg_oram_state, (int)oram->dbg_ht_state,
                       (int)oram->sel_stash_out, (int)oram->sel_posmap_out,
                       (int)oram->st_burst_busy_out, (int)wb.isStash);
            }
        }
    }

    // W handshake — routing already decided by AW (wb.isPcie)
    if (s_wvalid && s_wready && !activeWrites.empty()) {
        WriteBurst &wb = activeWrites.front();
        int beatBytes = 1 << wb.size;
        Addr beatAddr = axiBurstAddr(wb.baseAddr, wb.beatsRecv, beatBytes,
                                      wb.burst, wb.len);
        Addr gem5Addr = wb.isPcie ? (hostBase + beatAddr) : (hbmBase + beatAddr);

        // Debug: log W data for HT region writes
        {
            Addr htRegionStart = hbmBase + HT_SLOT_BASE_ADDR;
            Addr htRegionEnd   = hbmBase + HT_REGION_END;   // through end of IVT
            if (gem5Addr >= htRegionStart && gem5Addr < htRegionEnd) {
                DPRINTF(Oram, "[cyc %lu] HT_W_DATA: addr=0x%lx beat=%d wstrb=0x%x "
                       "data[0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x "
                       "FSM=%d ht_st=%d sel_st=%d\n",
                       oramCycle, gem5Addr, wb.beatsRecv, (unsigned)s_wstrb,
                       s_wdata[7], s_wdata[6], s_wdata[5], s_wdata[4],
                       s_wdata[3], s_wdata[2], s_wdata[1], s_wdata[0],
                       (int)oram->dbg_oram_state, (int)oram->dbg_ht_state,
                       (int)oram->sel_stash_out);
            }
        }

        // Check WSTRB: partial strobe = pos_map write
        // But if the AW was tagged as stash (HT or stash burst),
        // always use full-strobe path — partial wstrb from pos_map mux
        // leakage must not trigger the RMW path for stash writes.
        bool allEnabled = true;
        if (!wb.isStash) {
            for (int b = 0; b < std::min(beatBytes, AXI_DATA_BYTES); b++) {
                if (!(s_wstrb & (1u << b))) { allEnabled = false; break; }
            }
        }

        if (!allEnabled) {
            // pos_map masked write: read-modify-write via timing path.
            // The timing read is kept for latency modeling, but the MERGE
            // step uses pmShadow (authoritative) instead of the timing-read
            // data, so stale reads cannot corrupt co-resident entries.

            // Update pmShadow if it already exists for this beat.
            // Do NOT create a new entry here (it would be zeros for the
            // other 15 entries, corrupting them on merge). The shadow is
            // first populated from rdData at merge time in recvTimingResp.
            {
                auto it = pmShadow.find(gem5Addr);
                if (it != pmShadow.end()) {
                    for (int b = 0; b < std::min(beatBytes, AXI_DATA_BYTES); b++)
                        if (s_wstrb & (1u << b))
                            it->second[b] = s_wdata[b];
                }
            }

            // Issue timing read (for latency modeling — data will be
            // replaced by pmShadow at merge time in recvTimingResp).
            auto rdReq = std::make_shared<Request>(gem5Addr, beatBytes, 0, reqId);
            PacketPtr rdPkt = new Packet(rdReq, MemCmd::ReadReq);
            uint8_t *rdBuf = new uint8_t[beatBytes]();
            rdPkt->dataDynamic(rdBuf);

            auto *ss = new PmRmwSenderState(gem5Addr, beatBytes, s_wstrb,
                                             wb.id, wb.beatsRecv, wb.len + 1,
                                             wb.writeSeq, wb.isPcie);
            memcpy(ss->wdata, s_wdata, AXI_DATA_BYTES);
            rdPkt->pushSenderState(ss);

            if (hbmBlocked) {
                hbmRetryQueue.push_back(rdPkt);
            } else if (!hbmPort.sendTimingReq(rdPkt)) {
                hbmBlocked = true;
                hbmRetryQueue.push_back(rdPkt);
            }

            wb.beatsRecv++;
            if (s_wlast) {
                PendingWriteBurst pwb;
                pwb.id = wb.id;
                pwb.seq = wb.writeSeq;
                pwb.totalBeats = 1;
                pwb.isHbm = !wb.isPcie;
                pwb.isStash = false;
                posmapWriteSeqs.insert(pwb.seq);
                auto earlyIt = earlyWriteResps.find(wb.writeSeq);
                if (earlyIt != earlyWriteResps.end()) {
                    pwb.responsesRecv = earlyIt->second;
                    earlyWriteResps.erase(earlyIt);
                    DPRINTF(Oram, "PM-EARLY-PICKUP[%s] inst=%u cyc=%lu: posmap PWB "
                           "seq=%lu picked up %d early resp → immediate BRESP\n",
                           name(), instanceId, oramCycle, pwb.seq,
                           pwb.responsesRecv);
                } else {
                    pwb.responsesRecv = 0;
                }
                if (pwb.responsesRecv >= pwb.totalBeats) {
                    bQueue.push_back({pwb.id, pwb.isHbm, pwb.isStash});
                    bool isPm = (posmapWriteSeqs.count(pwb.seq) > 0);
                    bQueueIsPosmap.push_back(isPm);
                    if (isPm) posmapWriteSeqs.erase(pwb.seq);
                } else {
                    pendingWriteResps.push_back(pwb);
                }
                activeWrites.pop_front();
            }
        } else {
            // Normal full-strobe write (bucket/stash): timing path
            auto req = std::make_shared<Request>(gem5Addr, beatBytes, 0, reqId);
            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
            uint8_t *buf = new uint8_t[beatBytes]();
            memcpy(buf, s_wdata, std::min(beatBytes, AXI_DATA_BYTES));
            pkt->dataDynamic(buf);

            // HT SLOT write-forwarding: record the data so a later HT read of
            // this address returns it even if the HBM commit lags BRESP.
            {
                Addr htSlotStart = hbmBase + HT_SLOT_BASE_ADDR;
                Addr htSlotEnd   = hbmBase + HT_SLOT_BASE_ADDR + 4096 * AXI_DATA_BYTES;
                if (gem5Addr >= htSlotStart && gem5Addr < htSlotEnd &&
                    beatBytes >= AXI_DATA_BYTES) {
                    std::array<uint8_t, 32> e;
                    memcpy(e.data(), s_wdata, AXI_DATA_BYTES);
                    htSlotShadow[gem5Addr] = e;
                    DPRINTF(Oram, "[cyc %lu] HT_SHADOW_REC at 0x%lx [0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x (mapsz=%zu)\n",
                           oramCycle, (uint64_t)gem5Addr,
                           e[7],e[6],e[5],e[4],e[3],e[2],e[1],e[0], htSlotShadow.size());
                } else if (gem5Addr >= htSlotStart && gem5Addr < htSlotEnd) {
                    // In HT range but failed beatBytes guard — the diagnostic case.
                    DPRINTF(Oram, "[cyc %lu] HT_SHADOW_SKIP at 0x%lx (beatBytes=%d < %d)\n",
                           oramCycle, (uint64_t)gem5Addr, beatBytes, AXI_DATA_BYTES);
                }
            }

            // Cycle-by-cycle stash W data trace (FSM=29 S_ST_FLUSH) — EVERY beat
            if (curFsmState == FSM_ST_FLUSH) {
                uint32_t *dw = (uint32_t *)s_wdata;
                DPRINTF(Oram, "[cyc %lu] STASH_W_DATA: beat=%d addr=0x%lx data[0..3]=%08x %08x %08x %08x\n",
                       oramCycle, wb.beatsRecv, (uint64_t)gem5Addr,
                       dw[0], dw[1], dw[2], dw[3]);
            }

            pkt->pushSenderState(
                new AxiSenderState(wb.id, wb.beatsRecv, wb.len + 1,
                                   true, wb.writeSeq));

            // Queue to write FIFO instead of sending directly.
            // drainWriteFifo() sends 1 per tick to HBM.
            wFifo.push_back({pkt, wb.isPcie});

            // Stash data shadow: record each stash beat written to HBM.
            // When S_ST_LOAD reads this entry back, it may get stale data
            // (write not committed). The shadow ensures correct forwarding.
            {
                Addr sdStart = hbmBase + STASH_BASE_ADDR;
                Addr sdEnd   = sdStart + STASH_REGION_BYTES; // 16384 entries × 4KB = 64 MB
                if (gem5Addr >= sdStart && gem5Addr < sdEnd) {
                    auto &sb = stashDataShadow[gem5Addr];
                    memcpy(sb.data(), s_wdata,
                           std::min(beatBytes, (int)sizeof(sb)));
                }
            }

            // HT BKT_HEAD / BKT_NEXT write-shadow: same RAW hazard as
            // HT SLOT and pos_map. INSERT/DELETE do read-modify-write on
            // HEAD beats; a subsequent chain walk that reads the same beat
            // before HBM commits gets stale chain pointers → lost entries.
            {
                Addr headStart = hbmBase + HT_BKT_HEAD_ADDR;
                Addr headEnd   = headStart + (MAX_BUCKETS / 16) * AXI_DATA_BYTES;
                Addr nextStart = hbmBase + HT_BKT_NEXT_ADDR;
                Addr nextEnd   = nextStart + (STASH_DEPTH / 16) * AXI_DATA_BYTES;
                if (gem5Addr >= headStart && gem5Addr < headEnd &&
                    beatBytes >= AXI_DATA_BYTES) {
                    std::array<uint8_t, 32> e;
                    memcpy(e.data(), s_wdata, AXI_DATA_BYTES);
                    htBktHeadShadow[gem5Addr] = e;
                } else if (gem5Addr >= nextStart && gem5Addr < nextEnd &&
                           beatBytes >= AXI_DATA_BYTES) {
                    std::array<uint8_t, 32> e;
                    memcpy(e.data(), s_wdata, AXI_DATA_BYTES);
                    htBktNextShadow[gem5Addr] = e;
                }
            }
            // IVT / SLOT_R / BUCKET_META write-shadows: same RAW hazard.
            // wFifo delays can cause a read to reach the MemCtrl before
            // the write, returning stale data. IVT is the most critical:
            // a stale IV → AES-GCM tag mismatch → silent data corruption.
            {
                Addr ivtStart  = hbmBase + IVT_BASE_ADDR;
                Addr ivtEnd    = ivtStart + 65536 * AXI_DATA_BYTES;  // IVT_PHYS_SLOTS
                Addr srStart   = hbmBase + SLOT_R_BASE_ADDR;
                Addr srEnd     = srStart  + STASH_DEPTH * AXI_DATA_BYTES;
                Addr bmStart   = hbmBase + BUCKET_META_BASE_ADDR;
                Addr bmEnd     = bmStart  + MAX_BUCKETS * AXI_DATA_BYTES;
                if (gem5Addr >= ivtStart && gem5Addr < ivtEnd &&
                    beatBytes >= AXI_DATA_BYTES) {
                    std::array<uint8_t, 32> e;
                    memcpy(e.data(), s_wdata, AXI_DATA_BYTES);
                    ivtShadow[gem5Addr] = e;
                } else if (gem5Addr >= srStart && gem5Addr < srEnd &&
                           beatBytes >= AXI_DATA_BYTES) {
                    std::array<uint8_t, 32> e;
                    memcpy(e.data(), s_wdata, AXI_DATA_BYTES);
                    slotRShadow[gem5Addr] = e;
                } else if (gem5Addr >= bmStart && gem5Addr < bmEnd &&
                           beatBytes >= AXI_DATA_BYTES) {
                    std::array<uint8_t, 32> e;
                    memcpy(e.data(), s_wdata, AXI_DATA_BYTES);
                    bmetaShadow[gem5Addr] = e;
                }
            }
            if (curFsmState == FSM_DDR_WRITE) {
                wrDbg.sendAccepted++;
            }
            // Always advance — the AXI handshake fired (wready=1),
            // so the RTL already consumed this beat and moved on.
            wb.beatsRecv++;

            if (s_wlast) {
                DPRINTF(Oram, "[%lu] W-LAST: id=%d beats=%d seq=%lu\n",
                        oramCycle, (int)wb.id, (int)wb.beatsRecv, wb.writeSeq);
                PendingWriteBurst pwb;
                pwb.id = wb.id;
                pwb.totalBeats = wb.beatsRecv;
                pwb.seq = wb.writeSeq;
                pwb.isHbm = !wb.isPcie;
                pwb.isStash = wb.isStash;
                auto earlyIt = earlyWriteResps.find(wb.writeSeq);
                if (earlyIt != earlyWriteResps.end()) {
                    pwb.responsesRecv = earlyIt->second;
                    earlyWriteResps.erase(earlyIt);
                } else {
                    pwb.responsesRecv = 0;
                }
                if (pwb.responsesRecv >= pwb.totalBeats) {
                    bQueue.push_back({pwb.id, pwb.isHbm, pwb.isStash});
                    bQueueIsPosmap.push_back(false);  // bucket/stash, not posmap
                    DPRINTF(Oram, "[%lu] W-LAST IMMEDIATE BRESP: seq=%lu beats=%d "
                           "early=%d isStash=%d",
                           oramCycle, wb.writeSeq, (int)pwb.totalBeats,
                           (int)pwb.responsesRecv, (int)pwb.isStash);
                } else {
                    pendingWriteResps.push_back(pwb);
                    DPRINTF(Oram, "[%lu] W-LAST PENDING: seq=%lu beats=%d "
                           "resp=%d isStash=%d",
                           oramCycle, wb.writeSeq, (int)pwb.totalBeats,
                           (int)pwb.responsesRecv, (int)pwb.isStash);
                }
                activeWrites.pop_front();
            }
        } // end else(allEnabled)
    } // end if(s_wvalid && s_wready)

    // Debug: W stall detection with deadlock abort
    if (s_wvalid && !s_wready) {
        wStallCount++;
        if (wStallCount <= 5 || wStallCount % 1000 == 0) {
            DPRINTF(Oram, "[%lu] W-STALL #%lu: activeWr=%d hbmBlk=%d "
                    "pcieBlk=%d pendSend=%d hbmRetry=%d\n",
                    oramCycle, wStallCount,
                    (int)activeWrites.size(),
                    (int)hbmBlocked, (int)pcieBlocked,
                    (int)pendingReadSends.size(),
                    (int)hbmRetryQueue.size());
        }
        if (wStallCount >= 50000) {
            warn_once("W-STALL[%s] inst=%u: %lu cycles. activeWr=%d "
                  "hbmBlk=%d pcieBlk=%d pendSend=%d pendRd=%d "
                  "pendWr=%d rQ=%d bQ=%d hbmRetry=%d pcieRetry=%d",
                  name(),instanceId,
                  wStallCount,
                  (int)activeWrites.size(),
                  (int)hbmBlocked, (int)pcieBlocked,
                  (int)pendingReadSends.size(),
                  (int)pendingReadBursts.size(),
                  (int)pendingWriteResps.size(),
                  (int)rQueue.size(), (int)bQueue.size(),
                  (int)hbmRetryQueue.size(),
                  (int)(pcieWriteRetryQ.size() + pcieReadRetryQ.size()));
        }
    } else {
        wStallCount = 0;
    }

    // Periodic state dump for debugging hangs
    if (oramCycle % 500 == 0 &&
        (s_arvalid || s_awvalid || s_wvalid ||
         !activeWrites.empty() || !pendingReadSends.empty() ||
         !pendingWriteResps.empty())) {
        DPRINTF(Oram, "[%lu] STATE: arv=%d awv=%d wv=%d "
                "arR=%d awR=%d wR=%d rV=%d bV=%d "
                "actWr=%d rQ=%d bQ=%d pendRd=%d pendWr=%d "
                "pendSend=%d hbmBlk=%d pcieBlk=%d "
                "hbmRetry=%d pcieRetry=%d\n",
                oramCycle,
                (int)s_arvalid, (int)s_awvalid, (int)s_wvalid,
                (int)s_arready, (int)s_awready, (int)s_wready,
                (int)oram->m_axi_rvalid, (int)oram->m_axi_bvalid,
                (int)activeWrites.size(), (int)rQueue.size(),
                (int)bQueue.size(),
                (int)pendingReadBursts.size(),
                (int)pendingWriteResps.size(),
                (int)pendingReadSends.size(),
                (int)hbmBlocked, (int)pcieBlocked,
                (int)hbmRetryQueue.size(),
                (int)(pcieWriteRetryQ.size() + pcieReadRetryQ.size()));
    }

    // === DEBUG N=4 CRASH: forced state dump every 50000 cycles ===
    if (oramCycle % 50000 == 0 && oramCycle > 0) {
        warn("[DBG-PERIODIC] inst=%u cyc=%lu FSM=%d ht_st=%d ht_op=%d "
             "sel_stash=%d burst_busy=%d ext_busy=%d "
             "sng_rd=%d sng_wr=%d sng_done=%d "
             "arv=%d arR=%d awv=%d wv=%d rV=%d bV=%d "
             "hbmBlk=%d pcieBlk=%d "
             "pendRd=%lu pendSend=%lu actWr=%d rQ=%d bQ=%d wFifo=%d "
             "hbmRetry=%d pcieRetry=%d ops=%u",
             instanceId, oramCycle,
             (int)oram->dbg_oram_state,
             (int)oram->dbg_ht_state, (int)oram->dbg_ht_op,
             (int)oram->sel_stash_out, (int)oram->st_burst_busy_out,
             (int)oram->st_ext_busy_out,
             (int)oram->dbg_ht_sng_rd_req, (int)oram->dbg_ht_sng_wr_req,
             (int)oram->dbg_ht_sng_done,
             (int)s_arvalid, (int)oram->m_axi_arready,
             (int)s_awvalid, (int)s_wvalid,
             (int)oram->m_axi_rvalid, (int)oram->m_axi_bvalid,
             (int)hbmBlocked, (int)pcieBlocked,
             pendingReadBursts.size(), pendingReadSends.size(),
             (int)activeWrites.size(), (int)rQueue.size(),
             (int)bQueue.size(), (int)wFifo.size(),
             (int)hbmRetryQueue.size(),
             (int)(pcieWriteRetryQ.size()+pcieReadRetryQ.size()),
             opsCompleted);
    }

    // Multi-instance deadlock detector: if no progress for 5000 cycles
    {
        bool progress = (s_wvalid && s_wready) ||
                        (s_arvalid && s_arready) ||
                        (s_awvalid && s_awready) ||
                        (oram->m_axi_bvalid && oram->m_axi_bready) ||
                        (oram->m_axi_rvalid && oram->m_axi_rready);
        if (progress) {
            noProgressCount = 0;
        } else {
            noProgressCount++;
        }
        // Print every 5000 cycles of no progress
        if (noProgressCount > 0 && noProgressCount % 5000 == 0) {
            // Count bQueue types
            int bqHbm = 0, bqPcie = 0;
            for (auto &br : bQueue) { if (br.isHbm) bqHbm++; else bqPcie++; }
            bool needHbm = (oram->sel_stash_out != 0);
            if (!needHbm) needHbm = !currentOpIsPcie;

            inform("DEADLOCK[%s] inst=%u cyc=%lu fsm=%d stall=%lu: "
                   "ctrlState=%d initIdx=%d initPh=%d grantPh=%d "
                   "wrInitIdx=%d opsComp=%u/%u "
                   "bQ=%d(hbm=%d,pcie=%d) pendWr=%d pendRd=%d actWr=%d "
                   "earlyWr=%d "
                   "hbmBlk=%d pcieBlk=%d hbmRetry=%d pcieRetry=%d "
                   "awV=%d awR=%d wV=%d wR=%d arV=%d arR=%d "
                   "bV=%d bR=%d rV=%d rR=%d "
                   "sel_stash=%d needHbm=%d opPcie=%d opWr=%d "
                   "pendSend=%d rQ=%d "
                   "oram_busy=%d client_done=%d client_req=%d "
                   "init_mode=%d pm_busy=%d mgmt_req=%d mgmt_ack=%d mgmt_err=%d",
                   name(), instanceId, oramCycle, curFsmState,
                   noProgressCount,
                   (int)ctrlState, initSlotIdx, initPhase, grantPhase,
                   writeInitIdx, opsCompleted, numOps,
                   (int)bQueue.size(), bqHbm, bqPcie,
                   (int)pendingWriteResps.size(),
                   (int)pendingReadBursts.size(), (int)activeWrites.size(),
                   (int)earlyWriteResps.size(),
                   (int)hbmBlocked, (int)pcieBlocked,
                   (int)hbmRetryQueue.size(), (int)(pcieWriteRetryQ.size() + pcieReadRetryQ.size()),
                   (int)s_awvalid, (int)s_awready,
                   (int)s_wvalid, (int)s_wready,
                   (int)s_arvalid, (int)oram->m_axi_arready,
                   (int)oram->m_axi_bvalid, (int)oram->m_axi_bready,
                   (int)oram->m_axi_rvalid, (int)oram->m_axi_rready,
                   (int)oram->sel_stash_out, (int)needHbm,
                   (int)currentOpIsPcie,
                   (int)currentOpIsWrite,
                   (int)pendingReadSends.size(), (int)rQueue.size(),
                   (int)oram->oram_busy, (int)(oram->client_done & 0x1),
                   (int)oram->client_req,
                   (int)oram->init_mode, (int)oram->pm_busy_out,
                   (int)oram->mgmt_req, (int)oram->mgmt_ack,
                   (int)oram->mgmt_error);
            inform("  HT-DBG: ht_state=%d ht_op=%d ht_done=%d "
                   "latch_lu=%d latch_ins=%d latch_del=%d "
                   "sng_rd=%d sng_wr=%d sng_done=%d "
                   "st_ext_busy=%d st_burst_busy=%d",
                   (int)oram->dbg_ht_state, (int)oram->dbg_ht_op,
                   (int)oram->dbg_ht_done,
                   (int)oram->dbg_ht_latch_lookup,
                   (int)oram->dbg_ht_latch_insert,
                   (int)oram->dbg_ht_latch_delete,
                   (int)oram->dbg_ht_sng_rd_req,
                   (int)oram->dbg_ht_sng_wr_req,
                   (int)oram->dbg_ht_sng_done,
                   (int)oram->st_ext_busy_out,  // includes single-beat
                   (int)oram->st_burst_busy_out);  // burst only
        }
        // Warn at 100000 cycles but don't abort — let simulation continue
        // to see if it recovers (like PCIe does)
        if (noProgressCount == 100000) {
            warn("DEADLOCK-WARN[%s] inst=%u stall=%lu cycles — continuing",
                  name(), instanceId, noProgressCount);
        }
    }

    // Mux-conflict detector: sel_stash=1 during bucket phases is a deadlock.
    // The bucket master can't get R/B data because the mux routes to stash.
    // This catches the multi-instance shared-HBM bug where an HT op's
    // st_ext_busy lingers into S_DDR_READ/S_DDR_WRITE.
    {
        static uint64_t muxConflictCount[64] = {0};
        uint8_t fsm = oram->dbg_oram_state;
        bool bucketPhase = (fsm == FSM_DDR_READ || fsm == FSM_DDR_WRITE);  // S_DDR_READ or S_DDR_WRITE
        bool selStash = (oram->sel_stash_out != 0);
        if (bucketPhase && selStash) {
            muxConflictCount[instanceId]++;
            if (muxConflictCount[instanceId] == 1 ||
                muxConflictCount[instanceId] == 100 ||
                muxConflictCount[instanceId] % 5000 == 0) {
                warn("MUX-CONFLICT[%s] inst=%u cyc=%lu fsm=%d: sel_stash=1 "
                     "during bucket phase! st_ext_busy=%d st_burst_busy=%d "
                     "ht_state=%d ht_op=%d sng_rd=%d sng_wr=%d "
                     "(count=%lu)",
                     name(), instanceId, oramCycle, fsm,
                     (int)oram->st_ext_busy_out,
                     (int)oram->st_burst_busy_out,
                     (int)oram->dbg_ht_state, (int)oram->dbg_ht_op,
                     (int)oram->dbg_ht_sng_rd_req,
                     (int)oram->dbg_ht_sng_wr_req,
                     muxConflictCount[instanceId]);
            }
        } else {
            muxConflictCount[instanceId] = 0;
        }
    }

    // === DEBUG N=4 CRASH: detect SAM_SNG_RD_BUSY from gem5 side ===
    // This mirrors the RTL $display at stash_axi_master.v:177.
    // st_ext_busy_out = SAM cmd_busy = (fsm_state != ST_IDLE).
    // dbg_ht_sng_rd_req = st_sng_rd_req after ht_safe gating.
    {
        static uint64_t samBusyCount[64] = {0};
        bool sngRdReq = (oram->dbg_ht_sng_rd_req != 0);
        bool samBusy  = (oram->st_ext_busy_out != 0);
        if (sngRdReq && samBusy) {
            samBusyCount[instanceId]++;
            if (samBusyCount[instanceId] <= 5 ||
                samBusyCount[instanceId] == 50 ||
                samBusyCount[instanceId] % 1000 == 0)
                warn("[DBG-SAM_BUSY] inst=%u cyc=%lu #%lu: sng_rd_req=1 but "
                     "SAM busy! FSM=%d ht_state=%d ht_op=%d "
                     "burst_busy=%d sng_wr=%d sng_done=%d "
                     "hbmBlk=%d pcieBlk=%d arready=%d "
                     "pendRd=%lu rQ=%d pendSend=%lu "
                     "hbmRetry=%d pcieRetry=%d",
                     instanceId, oramCycle, samBusyCount[instanceId],
                     (int)oram->dbg_oram_state,
                     (int)oram->dbg_ht_state, (int)oram->dbg_ht_op,
                     (int)oram->st_burst_busy_out,
                     (int)oram->dbg_ht_sng_wr_req,
                     (int)oram->dbg_ht_sng_done,
                     (int)hbmBlocked, (int)pcieBlocked,
                     (int)oram->m_axi_arready,
                     pendingReadBursts.size(), (int)rQueue.size(),
                     pendingReadSends.size(),
                     (int)hbmRetryQueue.size(),
                     (int)(pcieWriteRetryQ.size()+pcieReadRetryQ.size()));
            // Dump pendingReadBursts state on first occurrence
            if (samBusyCount[instanceId] == 1) {
                for (size_t pi = 0; pi < std::min(pendingReadBursts.size(), (size_t)8); pi++) {
                    auto &rb = pendingReadBursts[pi];
                    warn("[DBG-SAM_BUSY] inst=%u  pendRd[%zu]: seq=%lu "
                         "beats=%d/%d flushed=%d single=%d",
                         instanceId, pi, rb.seq,
                         rb.beatsRecv, rb.totalBeats,
                         (int)rb.flushedBeats,
                         (rb.totalBeats == 1) ? 1 : 0);
                }
                // Trigger send ring buffer dump on next sendPkt call
                s_dbgSendDumpRequested[instanceId] = true;
            }
        } else {
            if (samBusyCount[instanceId] > 0)
                warn("[DBG-SAM_BUSY] inst=%u cyc=%lu CLEARED after %lu ticks",
                     instanceId, oramCycle, samBusyCount[instanceId]);
            samBusyCount[instanceId] = 0;
        }
    }

    // Drain queued read requests to gem5
    drainPendingSends();

    // --- HBM MEMORY WATCHPOINT ---
    // Functional read of a specific HT_SLOT address every tick.
    // Detects the exact cycle when the data changes unexpectedly.
    if (instanceId == 0 && oramCycle > 1000) {
        static uint64_t watchAddr = 0;
        static uint64_t lastGoodData[4] = {};
        static bool watchActive = false;
        static bool corruptionDetected = false;

        if (!watchActive) {
            // The confirmed HBM address from the trace: 0x141015E0
            // (beat 175 in the HT_SLOT table, slot 24750's hash bucket)
            watchAddr = hbmBase + 0x141015E0ULL;
            watchActive = true;
        }

        if (watchActive && !corruptionDetected) {
            uint8_t buf[AXI_DATA_BYTES];
            auto fReq = std::make_shared<Request>(watchAddr, AXI_DATA_BYTES, 0, reqId);
            PacketPtr fPkt = new Packet(fReq, MemCmd::ReadReq);
            fPkt->dataStatic(buf);
            hbmPort.sendFunctional(fPkt);
            delete fPkt;

            uint64_t *d = (uint64_t*)buf;
            if (d[0] != lastGoodData[0] || d[1] != lastGoodData[1] ||
                d[2] != lastGoodData[2] || d[3] != lastGoodData[3]) {
                DPRINTF(Oram, "WATCHPOINT HIT cyc=%lu addr=0x%lx: "
                     "OLD=[%016lx %016lx %016lx %016lx] "
                     "NEW=[%016lx %016lx %016lx %016lx] "
                     "FSM=%d ht_st=%d ht_op=%d sel_st=%d sel_pm=%d "
                     "burst_busy=%d sng_wr=%d sng_rd=%d awvalid=%d\n",
                     oramCycle, watchAddr,
                     lastGoodData[0], lastGoodData[1],
                     lastGoodData[2], lastGoodData[3],
                     d[0], d[1], d[2], d[3],
                     (int)oram->dbg_oram_state, (int)oram->dbg_ht_state,
                     (int)oram->dbg_ht_op, (int)oram->sel_stash_out,
                     (int)oram->sel_posmap_out,
                     (int)oram->st_burst_busy_out,
                     (int)oram->dbg_ht_sng_wr_req,
                     (int)oram->dbg_ht_sng_rd_req,
                     (int)oram->dbg_mux_awvalid);
                if (d[0] == 0xFFFFFFFFFFFFFFFFULL)
                    corruptionDetected = true;
                lastGoodData[0] = d[0]; lastGoodData[1] = d[1];
                lastGoodData[2] = d[2]; lastGoodData[3] = d[3];
            }
        }
    }

    // Step 5: shadow-capture client_rdata for any hw-client whose
    // rdata_valid bit is currently set. This runs EVERY tick, so a
    // one-cycle valid pulse is never missed. completeOp consumes the
    // shadow. RTL (secure_oram_top.v:442) gates rdata_valid[ci] by
    // arb_grant_id, so at most one bit is set at a time.
    for (unsigned hw = 0; hw < 2; hw++) {
        if (oram->client_rdata_valid & (1u << hw)) {
            unsigned rb = hw * 8;
            for (int i = 0; i < 8; i++)
                rdataShadow[hw][i] = oram->client_rdata[rb + i];
            rdataShadowValid[hw] = true;
            if (!rdataValidEverSeen[hw]) {
                DPRINTF(Oram, "RDATA_FIRST_SEEN hw=%u cycle=%lu data[0]=%08x FSM=%d\n",
                       hw, oramCycle, rdataShadow[hw][0],
                       (int)oram->dbg_oram_state);
            }
            rdataValidEverSeen[hw] = true;
            DPRINTF(Oram, "[%lu] rdata shadow captured hw=%u "
                    "[0..3]=%08x %08x %08x %08x\n",
                    oramCycle, hw,
                    rdataShadow[hw][0], rdataShadow[hw][1],
                    rdataShadow[hw][2], rdataShadow[hw][3]);
        }
    }

    // Check operation completion.
    // client_done is a 2-bit vector (one bit per hw-client). We must
    // check the bit matching activeHwClient — otherwise a client-1 op
    // completes but we never see it (deadlock: writeInitIdx never
    // advances, same slot re-sent forever).
    uint8_t doneMask = (uint8_t)(1u << activeHwClient);
    if ((ctrlState == OramState::PROCESSING ||
         ctrlState == OramState::WRITE_INIT) &&
        (oram->client_done & doneMask))
    {
        clientDoneSeen = true;
        if (!cmdQueue.empty()) {
            cmdQueue.front().req_b     = (uint16_t)oram->dbg_req_b;
            cmdQueue.front().req_b_new = (uint16_t)oram->dbg_req_b_new;
        }
    }

    // For READ ops, defer completeOp until rdataShadow is captured.
    // The RTL may assert client_done before client_rdata_valid in some
    // paths (e.g. stash hits). clientDoneSeen latches the done signal
    // so we don't miss it if it's a one-cycle pulse.
    if (clientDoneSeen) {
        bool frontIsWrite = (!cmdQueue.empty() &&
                             cmdQueue.front().op == 1);
        if (frontIsWrite || rdataShadowValid[activeHwClient]) {
            completeOp();
            clientDoneSeen = false;
            clientDoneWaitCycles = 0;
        } else if (clientDoneWaitCycles == 0) {
            // First cycle after client_done — capture client_rdata NOW
            // before the RTL overwrites it with the next op's data.
            unsigned rb = activeHwClient * 8;
            for (int i = 0; i < 8; i++)
                rdataShadow[activeHwClient][i] = oram->client_rdata[rb + i];
            rdataShadowValid[activeHwClient] = true;
            warn("rdata_valid missing — immediate capture hw=%u slot=0x%lx "
                 "slotIdx=%u data=[%08x %08x %08x %08x] FSM=%d "
                 "found_bucket=%d found_stash=%d everSeen=%d "
                 "req_b=%u req_b_new=%u same_bucket=%d stash_occ=%u "
                 "HT: ins_issued=%u ins_completed=%u del_issued=%u del_completed=%u "
                 "ins_overwritten=%d del_overwritten=%d "
                 "latch_ins=%d latch_del=%d "
                 "LU_HBM: addr=0x%lx valid_bits=0x%x wb_hit=%d slot_queried=0x%x",
                 activeHwClient, currentOpAddr,
                 (unsigned)((currentOpAddr - LEASE_BASE) / SLOT_SIZE),
                 rdataShadow[activeHwClient][0], rdataShadow[activeHwClient][1],
                 rdataShadow[activeHwClient][2], rdataShadow[activeHwClient][3],
                 (int)oram->dbg_oram_state,
                 (int)oram->dbg_found_in_bucket,
                 (int)oram->dbg_found_in_stash,
                 (int)rdataValidEverSeen[activeHwClient],
                 (unsigned)oram->dbg_req_b,
                 (unsigned)oram->dbg_req_b_new,
                 (int)oram->dbg_same_bucket,
                 (unsigned)oram->dbg_stash_occ,
                 (unsigned)oram->dbg_ht_ins_issued,
                 (unsigned)oram->dbg_ht_ins_completed,
                 (unsigned)oram->dbg_ht_del_issued,
                 (unsigned)oram->dbg_ht_del_completed,
                 (int)oram->dbg_ht_ins_overwritten,
                 (int)oram->dbg_ht_del_overwritten,
                 (int)oram->dbg_ht_latch_ins_active,
                 (int)oram->dbg_ht_latch_del_active,
                 (unsigned long)oram->dbg_ht_lu_hbm_addr,
                 (unsigned)oram->dbg_ht_lu_valid_bits,
                 (int)oram->dbg_ht_lu_wb_hit,
                 (unsigned)oram->dbg_ht_lu_slot_looked_up);
            clientDoneWaitCycles = 1;
            // rdataShadowValid is now true — next tick hits the first
            // branch and calls completeOp.
        }
    }

    // State machine
    switch (ctrlState) {
      case OramState::INIT_SLOTS:  initNextSlot(); break;
      case OramState::GRANT_LEASE: grantLease(); break;
      case OramState::WRITE_INIT:  writeInitNextSlot(); break;
      case OramState::IDLE:
        // cpu_driven: dispatch a CPU-issued op if one is queued and
        // pending. Otherwise stay IDLE. Legacy mode (cpuDriven=False)
        // keeps the original RNG-driven op generator.
        //
        // Step 8: ops are queued in cmdQueue. Head op is dispatched when
        // RTL is idle. Head stays in queue (marked IN_PROGRESS) and is
        // popped only when its result-packet WriteResp arrives in
        // recvTimingResp. This means depth=1 reproduces Step 4-7
        // single-op behavior exactly, and depth>1 lets PENDING ops sit
        // behind IN_PROGRESS while waiting their turn.
        if (cpuDriven) {
            if (!cmdQueue.empty() &&
                cmdQueue.front().phase == CmdEntry::Phase::PENDING &&
                !oram->oram_busy) {
                CmdEntry &op = cmdQueue.front();

                activeHwClient      = op.hw_client;
                rdataValidEverSeen[op.hw_client] = false;
                currentOpIsWrite    = (op.op == 1);
                currentOpAddr       = op.slot_addr;
                // Capture the real write payload so VERIFY-WRITE logs what the
                // command actually carried (op.wdata), not a synthetic pattern.
                for (int i = 0; i < 8; i++) currentOpWdata[i] = op.wdata[i];
                currentOpIsPcie     = isHostSlot(
                    (currentOpAddr - LEASE_BASE) / SLOT_SIZE);

                uint8_t  reqBit      = (uint8_t)(1u << op.hw_client);
                uint8_t  opBit       = (uint8_t)(op.op << op.hw_client);
                uint64_t addrPacked  = ((uint64_t)op.slot_addr)
                                       << (32 * op.hw_client);
                uint64_t tokenPacked = ((uint64_t)op.token)
                                       << (32 * op.hw_client);
                uint16_t leaseIdPk   = ((uint16_t)op.lease_id)
                                       << (8  * op.hw_client);

                oram->client_req       = reqBit;
                oram->client_op        = opBit;
                oram->client_slot_addr = addrPacked;
                oram->client_token     = tokenPacked;
                oram->client_lease_id  = leaseIdPk;

                memset(&oram->client_wdata[0], 0,
                       sizeof(oram->client_wdata));
                if (currentOpIsWrite) {
                    unsigned wbase = op.hw_client * 8;
                    for (int i = 0; i < 8; i++)
                        oram->client_wdata[wbase + i] = op.wdata[i];
                    oram->client_wdata_valid =
                        (uint8_t)(1u << op.hw_client);
                } else {
                    oram->client_wdata_valid = 0;
                }

                op.phase       = CmdEntry::Phase::IN_PROGRESS;
                op.dispatchTick = curTick();
                op.rdata_valid = false;

                ctrlState = OramState::PROCESSING;
                opStartCycle = oramCycle;
                memset(opPhaseCycles, 0, sizeof(opPhaseCycles));

                DPRINTF(Oram, "[%lu] CPU-op dispatch: %s slot=0x%x "
                        "hwC=%u lease=%u opIdx=%lu\n",
                        oramCycle, currentOpIsWrite ? "WR" : "RD",
                        op.slot_addr, op.hw_client, op.lease_id,
                        op.opIdx);
                unsigned dispSlotIdx = (op.slot_addr - LEASE_BASE) / SLOT_SIZE;
                inform("DISPATCH inst=%u op=%u %s slotIdx=%u hw=%u addr=0x%lx opIdx=%lu wdata0=%08x",
                       instanceId, opsCompleted + 1,
                       currentOpIsWrite ? "WR" : "RD",
                       dispSlotIdx, op.hw_client,
                       (unsigned long)op.slot_addr, op.opIdx,
                       op.wdata[0]);
            }
            // else: no pending op, just tick idly.
        } else {
            generateNextOp();
        }
        break;
      case OramState::DONE:        break;
      default: break;
    }

    // Periodic heartbeat — full state dump every 10000 cycles
    if (oramCycle % 10000 == 0 && oramCycle > 0) {
        DPRINTF(Oram, "HEARTBEAT[%s] cyc=%lu ctrlState=%d fsm=%d "
               "initIdx=%d initPh=%d grantPh=%d wrInitIdx=%d "
               "opsComp=%u/%u oram_busy=%d client_req=%d client_done=%d "
               "init_mode=%d pm_busy=%d mgmt_req=%d mgmt_ack=%d "
               "PM: awV=%d wV=%d arV=%d bR=%d state=%d "
               "MUX: sel_pm=%d sel_st=%d mux_awV=%d "
               "TOP: awV=%d awR=%d wV=%d wR=%d bV=%d\n",
               name(), oramCycle, (int)ctrlState, (int)oram->dbg_oram_state,
               initSlotIdx, initPhase, grantPhase, writeInitIdx,
               opsCompleted, numOps,
               (int)oram->oram_busy, (int)oram->client_req,
               (int)(oram->client_done & 0x1),
               (int)oram->init_mode, (int)oram->pm_busy_out,
               (int)oram->mgmt_req, (int)oram->mgmt_ack,
               (int)oram->dbg_pm_awvalid, (int)oram->dbg_pm_wvalid,
               (int)oram->dbg_pm_arvalid, (int)oram->dbg_pm_bready,
               (int)oram->dbg_pm_state,
               (int)oram->sel_posmap_out, (int)oram->sel_stash_out,
               (int)oram->dbg_mux_awvalid,
               (int)oram->m_axi_awvalid, (int)oram->m_axi_awready,
               (int)oram->m_axi_wvalid, (int)oram->m_axi_wready,
               (int)oram->m_axi_bvalid);
    }

    // --- Step 5: Negedge ---
    oram->clk = 0;
    oram->eval();
    // Capture hbmBlocked for NEXT tick's ready signals.
    // recvReqRetry may clear hbmBlocked between ticks, so we snapshot here.
    hbmBlockedSnapshot = hbmBlocked;
    oramCycle++;

    if (ctrlState != OramState::DONE) scheduleTick();
}

// =============================================================================
// RTL error monitoring — tag mismatch only warned once to avoid flooding
// =============================================================================

void OramDevice::checkRtlErrors()
{
    if (oram->err_stash_overflow)
        fatal("ORAM RTL: stash overflow @ cycle %lu", oramCycle);
    if (oram->err_bucket_overflow)
        fatal("ORAM RTL: bucket overflow @ cycle %lu", oramCycle);
    if (oram->err_tag_mismatch && !tagMismatchWarned) {
        warn("ORAM RTL: AES-GCM tag mismatch @ cycle %lu inst=%u "
             "FSM=%d slot=0x%lx bucket=%u "
             "found_bkt=%d found_stash=%d same_bkt=%d "
             "opIsPcie=%d hbmBktCount=%u",
             oramCycle, instanceId,
             (int)oram->dbg_oram_state,
             (uint64_t)oram->client_slot_addr,
             (unsigned)oram->dbg_req_b,
             (int)oram->dbg_found_in_bucket,
             (int)oram->dbg_found_in_stash,
             (int)oram->dbg_same_bucket,
             (int)currentOpIsPcie,
             hbmBucketCount);
        tagMismatchWarned = true;
    }
    if (oram->access_violation & 0x1)
        fatal("ORAM RTL: access violation @ cycle %lu", oramCycle);
}

// =============================================================================
// Packet routing
// =============================================================================

bool OramDevice::sendPkt(PacketPtr pkt, bool isPcie)
{
    if (isPcie) {
        // Fast-path: bypass CXL timing during write-init.
        // sendFunctional goes CxlModel→hostPort[0]→xbar→SSD/DDR5 backing
        // store in zero sim-time.  SsdMemory::recvFunctional (and MemCtrl)
        // fills read data / writes pmem and calls makeResponse().
        // handleMemResp then either delivers read beats to rQueue or
        // accumulates write responses in earlyWriteResps — the existing
        // W-LAST path picks them up and pushes an immediate BRESP.
        if (ctrlState == OramState::WRITE_INIT) {
            pciePort.sendFunctional(pkt);
            handleMemResp(pkt);   // pkt is deleted inside
            return true;
        }
        // === DEBUG N=4 CRASH: validate packet before sending to fabric ===
        {
            Addr pa = pkt->getAddr();
            unsigned sz = pkt->getSize();
            if (pa == 0 || sz == 0 || sz > 4096) {
                warn("[DBG-CRASH] inst=%u cyc=%lu sendPkt BAD PKT: addr=0x%lx "
                     "size=%u %s FSM=%d pendRd=%lu pendSend=%lu",
                     instanceId, oramCycle, pa, sz,
                     pkt->isWrite() ? "WR" : "RD",
                     (int)oram->dbg_oram_state,
                     pendingReadBursts.size(), pendingReadSends.size());
            }
            // Ring buffer of last 64 PCIe sends
            struct SendLog { uint64_t cyc; Addr addr; unsigned sz; bool isWr; int fsm; };
            static SendLog sendRing[64][64];  // [instance][slot]
            static int sendRingIdx[64] = {0};
            static uint64_t sendTotal[64] = {0};
            auto &sl = sendRing[instanceId][sendRingIdx[instanceId] % 64];
            sl.cyc = oramCycle; sl.addr = pa; sl.sz = sz;
            sl.isWr = pkt->isWrite(); sl.fsm = (int)oram->dbg_oram_state;
            sendRingIdx[instanceId]++;
            sendTotal[instanceId]++;
            // Log first 5 and every 50000th
            if (sendTotal[instanceId] <= 5 || sendTotal[instanceId] % 50000 == 0)
                inform("[DBG-SEND] inst=%u cyc=%lu PCIe #%lu: addr=0x%lx "
                       "size=%u %s FSM=%d",
                       instanceId, oramCycle, sendTotal[instanceId],
                       pa, sz, pkt->isWrite() ? "WR" : "RD",
                       (int)oram->dbg_oram_state);
            // Dump ring on SAM_BUSY trigger
            if (s_dbgSendDumpRequested[instanceId]) {
                s_dbgSendDumpRequested[instanceId] = false;
                warn("[DBG-SENDRING] inst=%u dumping last 64 PCIe sends:", instanceId);
                for (int k = 0; k < 64; k++) {
                    int idx = (sendRingIdx[instanceId] + k) % 64;
                    auto &e = sendRing[instanceId][idx];
                    if (e.cyc > 0)
                        warn("  [%d] cyc=%lu addr=0x%lx sz=%u %s fsm=%d",
                             k, e.cyc, e.addr, e.sz,
                             e.isWr ? "WR" : "RD", e.fsm);
                }
            }
        }
        if (pcieBlocked) {
            DPRINTF(Oram, "[%lu] sendPkt: PCIe blocked (%s, wrQ=%d rdQ=%d)\n",
                    oramCycle, pkt->isWrite() ? "WR" : "RD",
                    (int)pcieWriteRetryQ.size(), (int)pcieReadRetryQ.size());
            if (pkt->isWrite()) pcieWriteRetryQ.push_back(pkt);
            else                pcieReadRetryQ.push_back(pkt);
            return false;
        }
        if (!pciePort.sendTimingReq(pkt)) {
            DPRINTF(Oram, "[%lu] sendPkt: PCIe rejected (%s)\n",
                    oramCycle, pkt->isWrite() ? "WR" : "RD");
            pcieBlocked = true;
            if (pkt->isWrite()) pcieWriteRetryQ.push_back(pkt);
            else                pcieReadRetryQ.push_back(pkt);
            return false;
        }
    } else {
        // HBM write: send directly via port (no write buffer).
        // On failure, queue for retry. The AXI handshake already fired
        // (wready was 1), so the RTL advanced. The packet MUST be sent
        // eventually via the retry queue — it cannot be deleted.
        if (pkt->isWrite()) {
            if (hbmBlocked) {
                DPRINTF(Oram, "[%lu] BP-WRITE: hbmBlocked, pkt queued\n", oramCycle);
                hbmRetryQueue.push_back(pkt);
                return false;
            }
            if (!hbmPort.sendTimingReq(pkt)) {
                hbmBlocked = true;
                DPRINTF(Oram, "[%lu] BP-WRITE: rejected, hbmBlocked=true\n", oramCycle);
                hbmRetryQueue.push_back(pkt);
                return false;
            }
            return true;
        }
        // HBM read buffer: same pattern as write buffer.
        // Accept reads into internal FIFO, drain to xbar asynchronously.
        if (pkt->isRead() && hbmReadBuffer.size() < HBM_READ_BUF_DEPTH) {
            hbmReadBuffer.push_back(pkt);
            drainHbmReadBuffer();
            return true;
        }
        // Buffer full: send directly via port
        if (hbmBlocked) {
            DPRINTF(Oram, "[%lu] sendPkt: HBM blocked (retry=%d)\n",
                    oramCycle, (int)hbmRetryQueue.size());
            hbmRetryQueue.push_back(pkt); return false;
        }
        if (!hbmPort.sendTimingReq(pkt)) {
            DPRINTF(Oram, "[%lu] sendPkt: HBM rejected\n", oramCycle);
            hbmBlocked = true; hbmRetryQueue.push_back(pkt); return false;
        }
    }
    return true;
}

void OramDevice::trySendRetries(bool isPcie)
{
    if (isPcie) {
        // === DEBUG N=4 CRASH: track pcieBlocked duration ===
        {
            static uint64_t pcieBlockedCount[64] = {0};
            if (pcieBlocked) {
                pcieBlockedCount[instanceId]++;
                if (pcieBlockedCount[instanceId] == 100 ||
                    pcieBlockedCount[instanceId] % 5000 == 0)
                    warn("[DBG] inst=%u cyc=%lu pcieBlocked for %lu calls "
                         "(wrRetryQ=%d rdRetryQ=%d)",
                         instanceId, oramCycle, pcieBlockedCount[instanceId],
                         (int)pcieWriteRetryQ.size(), (int)pcieReadRetryQ.size());
            } else {
                pcieBlockedCount[instanceId] = 0;
            }
        }
        // Drain write retries first (they were rejected first)
        while (!pcieWriteRetryQ.empty() && !pcieBlocked) {
            if (!pciePort.sendTimingReq(pcieWriteRetryQ.front())) {
                pcieBlocked = true; return;
            }
            wrDbg.retrySent++;
            pcieWriteRetryQ.pop_front();
        }
        // Then drain read retries
        while (!pcieReadRetryQ.empty() && !pcieBlocked) {
            if (!pciePort.sendTimingReq(pcieReadRetryQ.front())) {
                pcieBlocked = true; return;
            }
            wrDbg.retrySent++;
            pcieReadRetryQ.pop_front();
        }
    } else {
        auto &q = hbmRetryQueue;
        while (!q.empty() && !hbmBlocked) {
            if (!hbmPort.sendTimingReq(q.front())) {
                DPRINTF(Oram, "[%lu] BP-RETRYFAIL: retry sendTimingReq failed, "
                       "hbmBlocked=true again (remaining=%d, FSM=%d)",
                       oramCycle, (int)q.size(), (int)oram->dbg_oram_state);
                hbmBlocked = true; return;
            }
            wrDbg.retrySent++;
            DPRINTF(Oram, "[%lu] BP-RETRYOK: retry sent successfully "
                   "(remaining=%d, FSM=%d, isWrite=%d)",
                   oramCycle, (int)q.size() - 1, (int)oram->dbg_oram_state,
                   (int)q.front()->isWrite());
            q.pop_front();
        }
        if (q.empty() && wrDbg.retrySent > 0) {
            DPRINTF(Oram, "[%lu] BP-RETRYDONE: all retries sent\n", oramCycle);
        }
        // After retry queue drained, drain both buffers
        drainHbmWriteBuffer();
        drainHbmReadBuffer();
    }
}

// =============================================================================
// Step 5: Result-buffer writeback
//
// Builds a 64-byte result packet describing the just-completed CPU op and
// sends it via pcie_port to result_buf_base + opIdx*64. Uses the existing
// sendPkt machinery so the packet follows the same retry + flow-control
// path as AXI writes. The packet carries a ResultSenderState tag so the
// response handler can distinguish it from an AXI completion.
//
// Packet layout (64 B):
//   0x00: op_idx (u64)          — which CPU op this is
//   0x08: status (u64)          — bit 0: done, bit 1: was_write,
//                                 bit 2: was_read, bit 3: rdata_captured
//   0x10: lease_id_used (u32)
//   0x14: hw_client_used (u32)
//   0x18: slot_addr_used (u32)
//   0x1C: evict_info (u32)      — [15:0] = req_b, [31:16] = req_b_new
//   0x20..0x3F: rdata[0..7] (32 B, only meaningful for reads)
// =============================================================================

void OramDevice::sendResultPacket(uint64_t opIdx)
{
    constexpr unsigned PKT_SIZE = 64;
    Addr dstAddr = resultBufBase + opIdx * PKT_SIZE;

    // Bounds check — fatal on overflow so we don't silently corrupt
    // whatever lives past the result buffer.
    if ((opIdx + 1) * PKT_SIZE > resultBufSize) {
        fatal("Result buffer overflow: opIdx=%lu, buffer holds %lu "
              "entries (size=0x%lx). Increase result_buf_size.",
              opIdx, resultBufSize / PKT_SIZE, resultBufSize);
    }

    auto req = std::make_shared<Request>(dstAddr, PKT_SIZE, 0, reqId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();

    uint8_t *buf = pkt->getPtr<uint8_t>();
    memset(buf, 0, PKT_SIZE);

    // op_idx @ 0x00
    uint64_t v64 = opIdx;
    memcpy(buf + 0x00, &v64, 8);

    // Step 8: source op fields from the queue head. This is the op that
    // was just marked COMPUTE_DONE in completeOp(). The head stays in
    // the queue (in COMPUTE_DONE phase) until the WriteResp arrives,
    // so this access is safe and stable for the duration of this call.
    if (cmdQueue.empty()) {
        // Defensive — shouldn't happen since sendResultPacket is only
        // called from completeOp() which already checked the queue.
        warn("sendResultPacket(opIdx=%lu): cmdQueue unexpectedly empty",
             opIdx);
        delete pkt;
        return;
    }
    const CmdEntry &op = cmdQueue.front();

    // status @ 0x08
    uint64_t status = 1;  // done bit
    if (op.op == 1) status |= (1ull << 1);
    else            status |= (1ull << 2);
    if (op.rdata_valid) status |= (1ull << 3);
    memcpy(buf + 0x08, &status, 8);

    // metadata
    uint32_t v32;
    v32 = op.lease_id;    memcpy(buf + 0x10, &v32, 4);
    v32 = op.hw_client;   memcpy(buf + 0x14, &v32, 4);
    v32 = op.slot_addr;   memcpy(buf + 0x18, &v32, 4);
    v32 = (uint32_t)op.req_b | ((uint32_t)op.req_b_new << 16);
    memcpy(buf + 0x1C, &v32, 4);

    // rdata[0..7] @ 0x20..0x3F (32 B). Filled from op.rdata (which was
    // copied from rdataShadow in completeOp). For writes, this stays zero.
    memcpy(buf + 0x20, op.rdata, 32);

    // Tag with opIdx so recvTimingResp matches it back to this queue entry.
    pkt->pushSenderState(new ResultSenderState(opIdx));

    DPRINTF(Oram, "[%lu] result pkt: opIdx=%lu dst=0x%lx status=0x%lx "
            "rdata[0..1]=%08x %08x\n",
            oramCycle, opIdx, dstAddr, status,
            op.rdata[0], op.rdata[1]);

    // Use sendPkt so blocking/retry uses the same path as AXI writes.
    sendPkt(pkt, true);
}

// =============================================================================
// Step 9 ring-fetch infrastructure
// =============================================================================
//
// CPU writes commands into a memory-resident ring buffer in CXL/PCIe DDR5.
// Layout (cmd_ring_base + offset):
//
//   +0x000  prod_idx (uint64_t, LE)   CPU writes; ORAM reads.
//   +0x040  cons_idx (uint64_t, LE)   ORAM writes; CPU reads (backpressure).
//   +0x100  cmd_entries[cmd_ring_depth]  64 B each.
//
// CPU sequence per command:
//   1. Wait for ring slot: while ((i - cons_idx) >= cmd_ring_depth) poll cons_idx
//   2. Write entry to cmd_ring[i % cmd_ring_depth]      (64 B cache line)
//   3. Memory fence
//   4. Update prod_idx = i + 1                           (8 B store at +0x00)
//   5. Optional doorbell: MMIO write to cmd_base+0x150
//
// ORAM sequence on doorbell:
//   1. Issue 8 B read of prod_idx                       (kind=PROD_IDX)
//   2. On response: for slot in [ringConsIdx, ringProdIdxKnown):
//        issue 64 B read of cmd_ring[slot % depth]      (kind=CMD_ENTRY)
//   3. On entry response: parse CmdEntry, push to cmdQueue (PENDING),
//        increment ringConsIdx, occasionally write cons_idx back.
//
// Cache-line layout of one cmd_entry (matches workload writer):
//   offset 0x00 : slot_addr (uint32_t)
//   offset 0x04 : token (uint32_t)
//   offset 0x08 : lease_id (u8) | op (u8) | hw_client (u8) | rsvd (u8)
//   offset 0x0C : opIdx_lo (uint32_t)        — workload-assigned sequence
//   offset 0x10 : wdata[0..7] (32 B)
//   offset 0x30 : opIdx_hi (uint32_t)
//   offset 0x34 : valid (uint32_t, 1=valid)  — written last by workload
//   offset 0x38 : rsvd (8 B)

void OramDevice::sendCmdRingRead(Addr gem5Addr, CmdRingFetchSenderState *ss)
{
    if (!ss) return;

    // Always read 64 B (one cache line). The CXL/PCIe fabric models are
    // designed around 64 B beats — the path-tree traffic uses 64 B
    // exclusively and the model's flit packing, host-port routing,
    // and DDR5 interleaving all assume 64 B granularity. Issuing an
    // 8 B read for prod_idx would not be handled correctly by the
    // fabric's data path. We just over-read and ignore the upper
    // bytes for prod_idx (only first 8 are meaningful).
    constexpr unsigned READ_SIZE = 64;
    Addr alignedAddr = gem5Addr & ~(Addr)63;  // align to cache line

    auto req = std::make_shared<Request>(alignedAddr, READ_SIZE, 0, reqId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate();
    pkt->pushSenderState(ss);

    if (ss->kind == CmdRingFetchSenderState::Kind::PROD_IDX) {
        DPRINTF(Oram, "[%lu] cmd-ring read: PROD_IDX addr=0x%lx (aligned 0x%lx)\n",
                oramCycle, gem5Addr, alignedAddr);
    } else {
        DPRINTF(Oram, "[%lu] cmd-ring read: CMD_ENTRY addr=0x%lx slot=%lu\n",
                oramCycle, gem5Addr, ss->ringSlot);
    }

    DPRINTF(Oram, "[%lu] cmd-ring sendPkt: pcieBlocked=%d retryQ=%zu\n",
            oramCycle, (int)pcieBlocked, pcieReadRetryQ.size());

    bool accepted = sendPkt(pkt, true);

    DPRINTF(Oram, "[%lu] cmd-ring sendPkt RESULT: accepted=%d "
            "pcieBlocked=%d retryQ=%zu\n",
            oramCycle, (int)accepted, (int)pcieBlocked,
            pcieReadRetryQ.size());

    if (ss->kind == CmdRingFetchSenderState::Kind::CMD_ENTRY)
        ringEntriesInFlight++;
}

void OramDevice::handleCmdRingResp(PacketPtr pkt, CmdRingFetchSenderState *ss)
{
    if (!ss) return;

    // CONS_WRITEBACK: just an ack of our own cons_idx write. Nothing
    // to do beyond the recvTimingResp caller's pkt/ss cleanup.
    if (ss->kind == CmdRingFetchSenderState::Kind::CONS_WRITEBACK) {
        DPRINTF(Oram, "[%lu] cons_idx writeback ack\n", oramCycle);
        return;
    }

    uint8_t *data = pkt->getPtr<uint8_t>();

    if (ss->kind == CmdRingFetchSenderState::Kind::PROD_IDX) {
        uint64_t newProd;
        memcpy(&newProd, data, 8);
        DPRINTF(Oram, "[%lu] cmd-ring PROD_IDX response: prod=%lu "
                "(local cons=%lu, prevKnown=%lu)\n",
                oramCycle, newProd, ringConsIdx, ringProdIdxKnown);
        ringProdIdxKnown = newProd;
        ringFetchInFlight = false;

        // Issue entry reads for newly-produced slots, but only as many
        // as the queue has room for. Don't overflow cmdQueue.
        while (ringConsIdx < ringProdIdxKnown &&
               cmdQueue.size() + ringEntriesInFlight < cmdQueueDepth) {
            uint64_t slotIdx = ringConsIdx;
            uint64_t ringPos = slotIdx % cmdRingDepth;
            Addr entryAddr  = cmdRingBase
                            + CMD_RING_ENTRIES_OFFSET
                            + ringPos * CMD_RING_ENTRY_BYTES;
            auto *entSs = new CmdRingFetchSenderState(
                CmdRingFetchSenderState::Kind::CMD_ENTRY,
                slotIdx, slotIdx);
            sendCmdRingRead(entryAddr, entSs);
            ringConsIdx++;  // optimistically advance — entry will arrive
        }
        return;
    }

    // CMD_ENTRY
    if (ringEntriesInFlight > 0) ringEntriesInFlight--;

    CmdEntry e;
    memset(&e, 0, sizeof(e));
    memcpy(&e.slot_addr, data + 0x00, 4);
    memcpy(&e.token,     data + 0x04, 4);
    e.lease_id  = data[0x08];
    e.op        = data[0x09];
    e.hw_client = data[0x0A];
    uint32_t opIdxLo = 0, opIdxHi = 0;
    memcpy(&opIdxLo, data + 0x0C, 4);
    memcpy(e.wdata,  data + 0x10, 32);
    memcpy(&opIdxHi, data + 0x30, 4);
    uint32_t valid = 0;
    memcpy(&valid,   data + 0x34, 4);

    e.opIdx       = ((uint64_t)opIdxHi << 32) | (uint64_t)opIdxLo;
    e.phase       = CmdEntry::Phase::PENDING;
    e.rdata_valid = false;

    // ====== ROOT CAUSE DIAGNOSTICS: ring entry data corruption ======
    {
        Addr entryAddr = pkt->getAddr();

        // 1. Dump raw timing response bytes
        DPRINTF(Oram, "RING-TRACE slot=%lu addr=0x%lx pktSize=%u timing_bytes[0..15]="
               "%02x %02x %02x %02x  %02x %02x %02x %02x  "
               "%02x %02x %02x %02x  %02x %02x %02x %02x  "
               "parsed: slot_addr=0x%x token=0x%x hwC=%u lease=%u op=%u opIdx=%lu\n",
               ss->ringSlot, entryAddr, pkt->getSize(),
               data[0],  data[1],  data[2],  data[3],
               data[4],  data[5],  data[6],  data[7],
               data[8],  data[9],  data[10], data[11],
               data[12], data[13], data[14], data[15],
               e.slot_addr, e.token, e.hw_client, e.lease_id,
               e.op, e.opIdx);

        // 2. Functional read: bypass timing, read directly from DDR5 pmem
        auto fReq = std::make_shared<Request>(entryAddr, 64, 0, reqId);
        PacketPtr fPkt = new Packet(fReq, MemCmd::ReadReq);
        fPkt->allocate();
        pciePort.sendFunctional(fPkt);
        uint8_t *fData = fPkt->getPtr<uint8_t>();

        uint32_t funcSlotAddr = 0;
        memcpy(&funcSlotAddr, fData, 4);

        DPRINTF(Oram, "RING-FUNC  slot=%lu addr=0x%lx func_bytes[0..15]="
               "%02x %02x %02x %02x  %02x %02x %02x %02x  "
               "%02x %02x %02x %02x  %02x %02x %02x %02x  "
               "func_slot_addr=0x%x\n",
               ss->ringSlot, entryAddr,
               fData[0],  fData[1],  fData[2],  fData[3],
               fData[4],  fData[5],  fData[6],  fData[7],
               fData[8],  fData[9],  fData[10], fData[11],
               fData[12], fData[13], fData[14], fData[15],
               funcSlotAddr);

        // 3. Byte-level comparison and stale-data fixup.
        //    Cross-port DDR5 write ordering issue: CPU ring writes and
        //    OramDevice ring reads go through different DDR5 xbar ports.
        //    At high N, writes may not have committed when the timing
        //    read arrives. Detect via byte comparison, fix using
        //    functional data. The timing read still goes through the
        //    fabric for accurate load modeling.
        bool mismatch = false;
        for (int b = 0; b < 64; b++) {
            if (data[b] != fData[b]) { mismatch = true; break; }
        }
        if (mismatch) {
            warn("RING-FIXUP slot=%lu: using functional data (timing was stale)",
                 ss->ringSlot);
            memcpy(&e.slot_addr, fData + 0x00, 4);
            memcpy(&e.token,     fData + 0x04, 4);
            e.lease_id  = fData[0x08];
            e.op        = fData[0x09];
            e.hw_client = fData[0x0A];
            memcpy(&opIdxLo, fData + 0x0C, 4);
            memcpy(e.wdata,  fData + 0x10, 32);
            memcpy(&opIdxHi, fData + 0x30, 4);
            memcpy(&valid,   fData + 0x34, 4);
            e.opIdx = ((uint64_t)opIdxHi << 32) | (uint64_t)opIdxLo;
        }

        delete fPkt;
    }
    // ====== END DIAGNOSTICS ======

    if (valid != 1) {
        warn("cmd-ring entry slot=%lu had valid=%u (expected 1) — "
             "CPU/ORAM ordering bug?", ss->ringSlot, valid);
    }
    if (e.opIdx != ss->expectedIdx) {
        warn("cmd-ring entry slot=%lu had opIdx=%lu (expected %lu)",
             ss->ringSlot, e.opIdx, ss->expectedIdx);
    }

    DPRINTF(Oram, "[%lu] cmd-ring CMD_ENTRY response: slot=%lu opIdx=%lu "
            "%s addr=0x%x hwC=%u lease=%u → cmdQueue (size=%zu→%zu)\n",
            oramCycle, ss->ringSlot, e.opIdx,
            e.op ? "WR" : "RD", e.slot_addr, e.hw_client, e.lease_id,
            cmdQueue.size(), cmdQueue.size() + 1);

    e.fetchTick = curTick();
    e.dispatchTick = 0;
    e.rtlDoneTick = 0;
    e.commitTick = 0;
    cmdQueue.push_back(e);

    // Throttled cons_idx writeback. Only write back every 4 commits to
    // avoid flooding the fabric with small writes that would themselves
    // contend with command and result traffic. Empirically this is
    // enough granularity for the CPU to make forward progress.
    if ((ss->ringSlot + 1) % 4 == 0) {
        writeConsIdx(ss->ringSlot + 1);
    }

    // If the prod_idx advanced again while this entry was in flight,
    // and we have queue room, kick off another fetch round.
    if (ringConsIdx < ringProdIdxKnown &&
        cmdQueue.size() + ringEntriesInFlight < cmdQueueDepth) {
        uint64_t slotIdx = ringConsIdx;
        uint64_t ringPos = slotIdx % cmdRingDepth;
        Addr entryAddr  = cmdRingBase
                        + CMD_RING_ENTRIES_OFFSET
                        + ringPos * CMD_RING_ENTRY_BYTES;
        auto *entSs = new CmdRingFetchSenderState(
            CmdRingFetchSenderState::Kind::CMD_ENTRY,
            slotIdx, slotIdx);
        sendCmdRingRead(entryAddr, entSs);
        ringConsIdx++;
    }

    // If the doorbell was rung again while we were draining, re-read prod.
    if (ringNeedsProdRead && !ringFetchInFlight) {
        ringNeedsProdRead = false;
        ringFetchInFlight = true;
        Addr prodAddr = cmdRingBase + CMD_RING_PROD_IDX_OFFSET;
        auto *prSs = new CmdRingFetchSenderState(
            CmdRingFetchSenderState::Kind::PROD_IDX);
        sendCmdRingRead(prodAddr, prSs);
    }
}

void OramDevice::writeConsIdx(uint64_t newConsIdx)
{
    // Like reads, write 64 B aligned. cons_idx field is at +0x40 which
    // is naturally cache-line aligned. We zero the upper 56 bytes of
    // the line — those are reserved/padding in the ring header layout.
    constexpr unsigned WRITE_SIZE = 64;
    Addr dstAddr = cmdRingBase + CMD_RING_CONS_IDX_OFFSET;
    auto req = std::make_shared<Request>(dstAddr, WRITE_SIZE, 0, reqId);
    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
    pkt->allocate();
    uint8_t *buf = pkt->getPtr<uint8_t>();
    memset(buf, 0, WRITE_SIZE);
    memcpy(buf, &newConsIdx, 8);

    auto *ss = new CmdRingFetchSenderState(
        CmdRingFetchSenderState::Kind::CONS_WRITEBACK);
    pkt->pushSenderState(ss);

    DPRINTF(Oram, "[%lu] cons_idx writeback: %lu → addr=0x%lx\n",
            oramCycle, newConsIdx, dstAddr);
    sendPkt(pkt, true);
}

void OramDevice::drainHbmWriteBuffer()
{
    while (hbmWriteBuffer.size() >= 1 && !hbmBlocked) {
        PacketPtr pkt1 = hbmWriteBuffer.front();

        // Try to coalesce with next entry: same 64B block
        if (hbmWriteBuffer.size() >= 2) {
            PacketPtr pkt2 = hbmWriteBuffer[1];
            Addr addr1 = pkt1->getAddr();
            Addr addr2 = pkt2->getAddr();
            if (addr2 == addr1 + 32 && pkt1->getSize() == 32 &&
                pkt2->getSize() == 32 &&
                (addr1 & 63) == 0) {  // must be 64B aligned
                // Coalesce into 64B write
                auto req = std::make_shared<Request>(addr1, 64, 0, reqId);
                PacketPtr merged = new Packet(req, MemCmd::WriteReq);
                merged->allocate();
                memcpy(merged->getPtr<uint8_t>(),
                       pkt1->getConstPtr<uint8_t>(), 32);
                memcpy(merged->getPtr<uint8_t>() + 32,
                       pkt2->getConstPtr<uint8_t>(), 32);

                auto *ss1 = dynamic_cast<AxiSenderState*>(
                    pkt1->popSenderState());
                auto *ss2 = dynamic_cast<AxiSenderState*>(
                    pkt2->popSenderState());
                auto *mss = new AxiSenderState(
                    ss1->axiId, ss1->beatIdx, ss1->totalBeats,
                    true, ss1->burstSeq);
                mss->secondBeatIdx = ss2->beatIdx;
                mss->secondBurstSeq = ss2->burstSeq;
                merged->pushSenderState(mss);

                delete pkt1; delete pkt2;
                delete ss1; delete ss2;
                hbmWriteBuffer.pop_front();
                hbmWriteBuffer.pop_front();

                if (!hbmPort.sendTimingReq(merged)) {
                    hbmBlocked = true;
                    hbmRetryQueue.push_back(merged);
                    return;
                }
                continue;
            }
        }

        // Non-coalesceable: send single
        hbmWriteBuffer.pop_front();
        if (!hbmPort.sendTimingReq(pkt1)) {
            hbmBlocked = true;
            hbmRetryQueue.push_back(pkt1);
            return;
        }
    }
}

void OramDevice::drainHbmReadBuffer()
{
    while (!hbmReadBuffer.empty() && !hbmBlocked) {
        PacketPtr pkt = hbmReadBuffer.front();
        if (!hbmPort.sendTimingReq(pkt)) {
            hbmBlocked = true;
            hbmRetryQueue.push_back(pkt);
            hbmReadBuffer.pop_front();
            return;
        }
        hbmReadBuffer.pop_front();
    }
}

// =============================================================================
// Drain pending read sends — one at a time, respecting port backpressure
// =============================================================================

void OramDevice::drainPendingSends()
{
    // Drain retry queues first
    if (!hbmBlocked && !hbmRetryQueue.empty())
        trySendRetries(false);
    if (!pcieBlocked &&
        (!pcieWriteRetryQ.empty() || !pcieReadRetryQ.empty()))
        trySendRetries(true);

    // Send queued read beats, skipping entries for blocked ports.
    // PCIe reads are NOT skipped when pcieBlocked — they go to
    // pcieReadRetryQ via sendPkt, allowing reads to flow independently
    // of write backpressure. This models the separate AR/AW channels
    // in real AXI and separate NPH/PH credit pools in PCIe.
    auto it = pendingReadSends.begin();
    while (it != pendingReadSends.end()) {
        if (!it->isPcie && hbmBlocked) { ++it; continue; }

        bool isPcie = it->isPcie;

        // LOCAL read coalescing: pair two consecutive 32B reads into
        // one 64B read aligned to the HBM interleave boundary (64B).
        // This halves xbar transactions and eliminates same-channel
        // collisions (two 32B reads within one 64B block → same channel).
        auto next = std::next(it);
        bool coalesce = (!isPcie && next != pendingReadSends.end() &&
                         !next->isPcie &&
                         next->gem5Addr == it->gem5Addr + it->beatBytes &&
                         next->beatBytes == it->beatBytes &&
                         it->beatBytes == 32 &&
                         (it->gem5Addr & 63) == 0);  // must be 64B aligned

        if (coalesce) {
            // 64B coalesced read
            auto req = std::make_shared<Request>(it->gem5Addr, 64, 0, reqId);
            PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
            pkt->allocate();
            auto *ss = new AxiSenderState(it->axiId, it->beatIdx,
                                           it->totalBeats, false, it->seq,
                                           false);  // coalesced HBM read
            ss->secondBeatIdx = next->beatIdx;
            ss->secondBurstSeq = next->seq;
            pkt->pushSenderState(ss);

            // Erase both entries
            it = pendingReadSends.erase(it);
            it = pendingReadSends.erase(it);

            if (sendPkt(pkt, false)) {
                rdDbg.sendAccepted += 2;
            } else {
                rdDbg.sendRejected += 2;
                if (hbmBlocked && pcieBlocked) break;
            }
        } else {
            // Normal single read (PCIe or non-sequential LOCAL)
            auto req = std::make_shared<Request>(it->gem5Addr, it->beatBytes,
                                                  0, reqId);
            PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
            pkt->allocate();
            pkt->pushSenderState(new AxiSenderState(it->axiId, it->beatIdx,
                                                     it->totalBeats, false,
                                                     it->seq, isPcie));
            it = pendingReadSends.erase(it);

            if (sendPkt(pkt, isPcie)) {
                rdDbg.sendAccepted++;
            } else {
                rdDbg.sendRejected++;
                if (hbmBlocked && pcieBlocked) break;
            }
        }
    }
}

// =============================================================================
// Memory response handler
// =============================================================================

void OramDevice::handleMemResp(PacketPtr pkt)
{
    // Check for pos_map RMW read response first — peek before popping
    auto *pmRmw = dynamic_cast<PmRmwSenderState*>(pkt->senderState);
    if (pmRmw) {
        pkt->popSenderState();  // confirmed type, now pop

        // Read completed — merge masked bytes and issue timing write.
        // Use pmShadow as the authoritative beat contents instead of the
        // timing-read data (which can be stale under load, corrupting the
        // 15 co-resident pos_map entries sharing this 32-byte beat).
        const uint8_t *rdData = pkt->getConstPtr<uint8_t>();
        uint8_t *mergedBuf = new uint8_t[pmRmw->writeSize]();
        bool usedShadow = false;
        {
            auto it = pmShadow.find(pmRmw->writeAddr);
            if (it != pmShadow.end()) {
                // Shadow exists: use it (authoritative, immune to stale reads)
                memcpy(mergedBuf, it->second.data(), std::min(pmRmw->writeSize, (int)sizeof(it->second)));
                usedShadow = true;
                bool differs = (memcmp(it->second.data(), rdData,
                                std::min(pmRmw->writeSize, (int)sizeof(it->second))) != 0);
                if (differs)
                    DPRINTF(Oram, "[cyc %lu] PM_SHADOW_FIX at 0x%lx: shadow differs from timing-read "
                           "(stale read corrected)\n",
                           oramCycle, (uint64_t)pmRmw->writeAddr);
            } else {
                // First RMW to this beat: populate shadow from timing-read
                // (correct on first access — no contention yet)
                std::array<uint8_t, 32> newShadow{};
                memcpy(newShadow.data(), rdData, std::min(pmRmw->writeSize, (int)sizeof(newShadow)));
                pmShadow[pmRmw->writeAddr] = newShadow;
                memcpy(mergedBuf, rdData, pmRmw->writeSize);
            }
        }
        for (int b = 0; b < std::min(pmRmw->writeSize, (int)AXI_DATA_BYTES); b++) {
            if (pmRmw->wstrb & (1u << b))
                mergedBuf[b] = pmRmw->wdata[b];
        }
        // Update shadow with the merged beat (latest state of all 16 entries)
        {
            auto &sb = pmShadow[pmRmw->writeAddr];
            memcpy(sb.data(), mergedBuf, std::min(pmRmw->writeSize, (int)sizeof(sb)));
        }

        auto wrReq = std::make_shared<Request>(
            pmRmw->writeAddr, pmRmw->writeSize, 0, reqId);
        PacketPtr wrPkt = new Packet(wrReq, MemCmd::WriteReq);
        wrPkt->dataDynamic(mergedBuf);
        wrPkt->pushSenderState(
            new AxiSenderState(pmRmw->axiId, pmRmw->beatIdx,
                               pmRmw->totalBeats, true, pmRmw->burstSeq));
        sendPkt(wrPkt, pmRmw->isPcie);

        delete pmRmw;
        delete pkt;
        return;
    }

    // Normal AXI response handling
    auto *ss = dynamic_cast<AxiSenderState*>(pkt->popSenderState());
    assert(ss);

    if (!ss->isWrite) {
        rdDbg.memResps++;

        // === DEBUG N=4 CRASH: validate response data ===
        // The crash shows "stale cmd_single_rdata=0xffff..." — check for
        // all-0xFF responses which indicate uninitialized/garbage HBM data.
        if (ss->totalBeats == 1) {
            const uint8_t *rd = pkt->getConstPtr<uint8_t>();
            bool allFF = true, allZero = true;
            for (unsigned b = 0; b < std::min((unsigned)pkt->getSize(), 32u); b++) {
                if (rd[b] != 0xFF) allFF = false;
                if (rd[b] != 0x00) allZero = false;
            }
            if (allFF) {
                warn("[DBG-RESP] inst=%u cyc=%lu SINGLE-READ all-0xFF! "
                     "addr=0x%lx size=%u seq=%lu FSM=%d ht_st=%d "
                     "pendRd=%lu",
                     instanceId, oramCycle, pkt->getAddr(), pkt->getSize(),
                     ss->burstSeq, (int)oram->dbg_oram_state,
                     (int)oram->dbg_ht_state,
                     pendingReadBursts.size());
            }
        }

        // Helper lambda to deliver one beat to pendingReadBursts
        auto deliverBeat = [&](size_t seq, int beatIdx,
                               const uint8_t *data, unsigned dataOff,
                               bool isPcieResp) {
            for (auto &rb : pendingReadBursts) {
                if (rb.seq == seq) {
                    assert(beatIdx < rb.totalBeats);
                    if (rb.beatRecvd[beatIdx]) {
                        warn("DUPLICATE BEAT: inst=%u seq=%lu beatIdx=%d/%d "
                             "beatsRecv=%d pktAddr=0x%lx pktSize=%u "
                             "pendingReadBursts=%lu tick=%llu\n",
                             instanceId, seq, beatIdx,
                             rb.totalBeats, rb.beatsRecv,
                             pkt->getAddr(), pkt->getSize(),
                             pendingReadBursts.size(), curTick());
                        assert(false && "duplicate beat");
                    }
                    RBeat &beat = rb.beats[beatIdx];
                    memset(beat.data, 0, AXI_DATA_BYTES);
                    memcpy(beat.data, data + dataOff,
                           std::min((int)AXI_DATA_BYTES,
                                    (int)pkt->getSize() - (int)dataOff));
                    beat.id = ss->axiId;
                    beat.last = (beatIdx == rb.totalBeats - 1);
                    rb.beatRecvd[beatIdx] = true;
                    // HBM CDC: PCIe/CXL responses already include CDC
                    // in the fabric model; HBM responses need it here.
                    rb.beatReadyTick[beatIdx] = curTick() +
                        (isPcieResp ? 0 : hbmCdcLatency);
                    rb.beatsRecv++;
                    return true;
                }
            }
            return false;
        };

        // HT SLOT read-your-writes: if this single-beat read targets an HT SLOT
        // address we have a recorded write for, forward the shadowed data into
        // the packet before delivery. Negates the BRESP-vs-HBM-commit gap that
        // otherwise lets a lookup read pre-write memory (0) -> false MISS.
        {
            Addr pktAddr = pkt->getAddr();
            Addr htSlotStart = hbmBase + HT_SLOT_BASE_ADDR;
            Addr htSlotEnd   = hbmBase + HT_SLOT_BASE_ADDR + 4096 * AXI_DATA_BYTES;
            if (pktAddr >= htSlotStart && pktAddr < htSlotEnd &&
                ss->totalBeats == 1) {
                auto it = htSlotShadow.find(pktAddr);
                if (it != htSlotShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    DPRINTF(Oram, "[cyc %lu] HT_FWD at 0x%lx: shadow hit differs=%d [0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                           oramCycle, pktAddr, differs,
                           pd[7], pd[6], pd[5], pd[4], pd[3], pd[2], pd[1], pd[0]);
                } else {
                    DPRINTF(Oram, "[cyc %lu] HT_FWD_MISS at 0x%lx: no shadow entry (mapsz=%zu)\n",
                           oramCycle, pktAddr, htSlotShadow.size());
                }
            }
        }

        // pos_map read-your-writes: same pattern as HT SLOT forwarding.
        // If this single-beat read targets a pos_map beat we have a shadow
        // for, forward the shadowed data. Negates the BRESP-vs-commit gap
        // that lets a pos_map read return a stale beat (with corrupted
        // co-resident entries) before the prior RMW write lands in HBM.
        {
            Addr pktAddr = pkt->getAddr();
            Addr pmStart = hbmBase + PM_BASE_ADDR;
            Addr pmEnd   = hbmBase + PM_BASE_ADDR + 0x10000; // 2048 beats × 32B
            if (pktAddr >= pmStart && pktAddr < pmEnd &&
                ss->totalBeats == 1) {
                auto it = pmShadow.find(pktAddr);
                if (it != pmShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] PM_RD_FWD at 0x%lx: shadow corrected stale pos_map read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // Stash data read-your-writes: same pattern as PM/HT forwarding.
        // When S_ST_LOAD reads a stash entry from HBM, the timing read
        // may return stale data (prior stash flush not yet committed).
        // The shadow provides the authoritative beat data.
        {
            Addr pktAddr = pkt->getAddr();
            Addr sdStart = hbmBase + STASH_BASE_ADDR;
            Addr sdEnd   = sdStart + STASH_REGION_BYTES; // 16384 entries × 4KB = 64 MB
            if (pktAddr >= sdStart && pktAddr < sdEnd) {
                auto it = stashDataShadow.find(pktAddr);
                if (it != stashDataShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] STASH_FWD at 0x%lx: shadow corrected stale stash read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // HT BKT_HEAD read-your-writes: same RAW hazard as HT SLOT/PM.
        // Chain walk reads HEAD[bkt]; if a prior INSERT/DELETE wrote the
        // same beat and HBM hasn't committed, forward from shadow.
        {
            Addr pktAddr = pkt->getAddr();
            Addr headStart = hbmBase + HT_BKT_HEAD_ADDR;
            Addr headEnd   = headStart + (MAX_BUCKETS / 16) * AXI_DATA_BYTES;
            if (pktAddr >= headStart && pktAddr < headEnd &&
                ss->totalBeats == 1) {
                auto it = htBktHeadShadow.find(pktAddr);
                if (it != htBktHeadShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] HEAD_FWD at 0x%lx: shadow corrected stale BKT_HEAD read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // HT BKT_NEXT read-your-writes: same pattern.
        {
            Addr pktAddr = pkt->getAddr();
            Addr nextStart = hbmBase + HT_BKT_NEXT_ADDR;
            Addr nextEnd   = nextStart + (STASH_DEPTH / 16) * AXI_DATA_BYTES;
            if (pktAddr >= nextStart && pktAddr < nextEnd &&
                ss->totalBeats == 1) {
                auto it = htBktNextShadow.find(pktAddr);
                if (it != htBktNextShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] NEXT_FWD at 0x%lx: shadow corrected stale BKT_NEXT read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // IVT read-your-writes: a stale IV causes AES-GCM decryption
        // with the wrong IV → tag mismatch → silent data corruption.
        {
            Addr pktAddr = pkt->getAddr();
            Addr ivtStart = hbmBase + IVT_BASE_ADDR;
            Addr ivtEnd   = ivtStart + 65536 * AXI_DATA_BYTES;
            if (pktAddr >= ivtStart && pktAddr < ivtEnd &&
                ss->totalBeats == 1) {
                auto it = ivtShadow.find(pktAddr);
                if (it != ivtShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] IVT_FWD at 0x%lx: shadow corrected stale IVT read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // SLOT_R read-your-writes: stale slot_r → wrong slot address
        // for eviction → encrypt/write-back to wrong DDR location.
        {
            Addr pktAddr = pkt->getAddr();
            Addr srStart = hbmBase + SLOT_R_BASE_ADDR;
            Addr srEnd   = srStart + STASH_DEPTH * AXI_DATA_BYTES;
            if (pktAddr >= srStart && pktAddr < srEnd &&
                ss->totalBeats == 1) {
                auto it = slotRShadow.find(pktAddr);
                if (it != slotRShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] SLOTR_FWD at 0x%lx: shadow corrected stale SLOT_R read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // BUCKET_META read-your-writes: stale fill_count or slot_list
        // → wrong bucket occupancy → eviction to occupied position or
        // bucket overflow.
        {
            Addr pktAddr = pkt->getAddr();
            Addr bmStart = hbmBase + BUCKET_META_BASE_ADDR;
            Addr bmEnd   = bmStart + MAX_BUCKETS * AXI_DATA_BYTES;
            if (pktAddr >= bmStart && pktAddr < bmEnd &&
                ss->totalBeats == 1) {
                auto it = bmetaShadow.find(pktAddr);
                if (it != bmetaShadow.end()) {
                    uint8_t *pd = pkt->getPtr<uint8_t>();
                    bool differs = (memcmp(pd, it->second.data(), AXI_DATA_BYTES) != 0);
                    memcpy(pd, it->second.data(), AXI_DATA_BYTES);
                    if (differs)
                        DPRINTF(Oram, "[cyc %lu] BMETA_FWD at 0x%lx: shadow corrected stale BUCKET_META read\n",
                               oramCycle, pktAddr);
                }
            }
        }

        // First beat (always present)
        bool found = deliverBeat(ss->burstSeq, ss->beatIdx,
                                  pkt->getConstPtr<uint8_t>(), 0,
                                  ss->isPcie);

        // Debug: cross-check HT region reads with functional path
        {
            Addr pktAddr = pkt->getAddr();
            Addr htSlotStart = hbmBase + HT_SLOT_BASE_ADDR;
            Addr htSlotEnd   = hbmBase + HT_SLOT_BASE_ADDR + 4096 * AXI_DATA_BYTES; // 128KB (direct-indexed)
            if (pktAddr >= htSlotStart && pktAddr < htSlotEnd && ss->totalBeats == 1) {
                // Single-beat HT SLOT read — verify data via functional
                auto fReq = std::make_shared<Request>(pktAddr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr fPkt = new Packet(fReq, MemCmd::ReadReq);
                uint8_t fBuf[AXI_DATA_BYTES];
                fPkt->dataStatic(fBuf);
                hbmPort.sendFunctional(fPkt);
                bool match = (memcmp(pkt->getConstPtr<uint8_t>(), fBuf,
                              std::min((int)pkt->getSize(), AXI_DATA_BYTES)) == 0);
                if (!match) {
                    DPRINTF(Oram, "[cyc %lu] HT_XCHECK MISMATCH at 0x%lx: "
                           "timing[0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x "
                           "func[0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                           oramCycle, pktAddr,
                           pkt->getConstPtr<uint8_t>()[7], pkt->getConstPtr<uint8_t>()[6],
                           pkt->getConstPtr<uint8_t>()[5], pkt->getConstPtr<uint8_t>()[4],
                           pkt->getConstPtr<uint8_t>()[3], pkt->getConstPtr<uint8_t>()[2],
                           pkt->getConstPtr<uint8_t>()[1], pkt->getConstPtr<uint8_t>()[0],
                           fBuf[7], fBuf[6], fBuf[5], fBuf[4],
                           fBuf[3], fBuf[2], fBuf[1], fBuf[0]);
                } else {
                    DPRINTF(Oram, "[cyc %lu] HT_XCHECK OK at 0x%lx: [0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                           oramCycle, pktAddr,
                           pkt->getConstPtr<uint8_t>()[7], pkt->getConstPtr<uint8_t>()[6],
                           pkt->getConstPtr<uint8_t>()[5], pkt->getConstPtr<uint8_t>()[4],
                           pkt->getConstPtr<uint8_t>()[3], pkt->getConstPtr<uint8_t>()[2],
                           pkt->getConstPtr<uint8_t>()[1], pkt->getConstPtr<uint8_t>()[0]);
                }
                delete fPkt;
            }
        }

        // Second beat (coalesced 64B read)
        if (ss->secondBeatIdx >= 0) {
            rdDbg.memResps++;  // count as two responses
            bool found2 = deliverBeat(ss->secondBurstSeq, ss->secondBeatIdx,
                                       pkt->getConstPtr<uint8_t>(), 32,
                                       ss->isPcie);
            found = found || found2;
        }

        if (!found) {
            warn("[DBG-CRASH] inst=%u cyc=%lu READ-RESP-UNMATCHED: "
                 "seq=%lu beat=%d addr=0x%lx size=%u pendRd=%lu FSM=%d",
                 instanceId, oramCycle, ss->burstSeq, ss->beatIdx,
                 pkt->getAddr(), pkt->getSize(),
                 pendingReadBursts.size(),
                 (int)oram->dbg_oram_state);
        }
        // Don't flush to rQueue here — that runs between ticks and
        // would pre-fill rQueue before the mux transition check in tick().
        // flushCompletedReads is called from tick() after the mux check.
    } else {
        // Write response — count beats (coalesced 64B = 2 beats)
        int respCount = (ss->secondBeatIdx >= 0) ? 2 : 1;

        // Debug: verify HT region writes persisted (disabled - too noisy,
        // bucket DDR_WRITE responses flood the check)
        // Use HT_XCHECK on reads instead to verify data integrity.

        bool matched = false;
        for (auto &pwb : pendingWriteResps) {
            if (pwb.seq == ss->burstSeq) {
                pwb.responsesRecv += respCount;
                if (pwb.responsesRecv >= pwb.totalBeats) {
                    bQueue.push_back({pwb.id, pwb.isHbm, pwb.isStash});
                    bool isPm = (posmapWriteSeqs.count(pwb.seq) > 0);
                    bQueueIsPosmap.push_back(isPm);
                    if (isPm) posmapWriteSeqs.erase(pwb.seq);
                }
                matched = true;
                break;
            }
        }
        if (!matched) {
            // Response arrived before wlast created the PendingWriteBurst.
            earlyWriteResps[ss->burstSeq] += respCount;
            bool earlyIsPm = (posmapWriteSeqs.count(ss->burstSeq) > 0);
            if (earlyIsPm) {
                DPRINTF(Oram, "EARLY-PM-BRESP[%s] inst=%u cyc=%lu: posmap write resp "
                     "seq=%lu arrived before PendingWriteBurst created "
                     "(earlyCount=%d, FSM=%d, pm_busy=%d)\n",
                     name(), instanceId, oramCycle, ss->burstSeq,
                     earlyWriteResps[ss->burstSeq],
                     (int)oram->dbg_oram_state,
                     (int)oram->pm_busy_out);
            }
        }
        // Clean up completed write bursts
        auto it = pendingWriteResps.begin();
        while (it != pendingWriteResps.end()) {
            if (it->responsesRecv >= it->totalBeats)
                it = pendingWriteResps.erase(it);
            else ++it;
        }
    }
    delete ss;
    delete pkt;
}

void OramDevice::flushCompletedReads()
{
    // Push 1 beat per tick from reorder buffer to rQueue.
    // Allow rQueue up to 2 entries: one being presented by driveAxiR
    // (will be popped this tick if handshake fires), and one queued
    // to be presented next tick. This ensures rvalid=1 every cycle
    // when beats are available, without the 1,0,1,0 alternation that
    // occurs with a single-entry cap.
    if ((int)rQueue.size() >= 2) return;

    // === DEBUG N=4 CRASH: detect head-of-line blocking ===
    if (!pendingReadBursts.empty()) {
        auto &front = pendingReadBursts.front();
        if (front.flushedBeats < front.totalBeats &&
            !front.beatRecvd[front.flushedBeats]) {
            // Front entry is stalled — check if later entries have data ready
            bool laterReady = false;
            size_t laterIdx = 0;
            for (size_t pi = 1; pi < pendingReadBursts.size(); pi++) {
                auto &later = pendingReadBursts[pi];
                if (later.beatsRecv > 0 && later.flushedBeats < later.totalBeats) {
                    laterReady = true;
                    laterIdx = pi;
                    break;
                }
            }
            if (laterReady) {
                static uint64_t holCount[64] = {0};
                holCount[instanceId]++;
                if (holCount[instanceId] == 1 || holCount[instanceId] == 100 ||
                    holCount[instanceId] % 5000 == 0)
                    warn("[DBG-HOL] inst=%u cyc=%lu head-of-line block #%lu: "
                         "front(seq=%lu beats=%d/%d flushed=%d) stalled, "
                         "later[%zu](seq=%lu beats=%d/%d) ready. "
                         "pendRd=%lu FSM=%d",
                         instanceId, oramCycle, holCount[instanceId],
                         front.seq, front.beatsRecv, front.totalBeats,
                         (int)front.flushedBeats,
                         laterIdx,
                         pendingReadBursts[laterIdx].seq,
                         pendingReadBursts[laterIdx].beatsRecv,
                         pendingReadBursts[laterIdx].totalBeats,
                         pendingReadBursts.size(),
                         (int)oram->dbg_oram_state);
            }
        }
    }

    if (!pendingReadBursts.empty()) {
        auto &rb = pendingReadBursts.front();
        if (rb.flushedBeats < rb.totalBeats &&
            rb.beatRecvd[rb.flushedBeats] &&
            curTick() >= rb.beatReadyTick[rb.flushedBeats]) {
            RBeat beat = rb.beats[rb.flushedBeats];
            beat.isSingle = (rb.totalBeats == 1);
            // Diagnostic: detect two single-beat HT read beats coexisting in
            // rQueue — the out-of-order-delivery hazard. If this fires, the
            // gate failed to keep single reads serialized end-to-end.
            if (beat.isSingle) {
                for (auto &q : rQueue) {
                    if (q.isSingle) {
                        DPRINTF(Oram, "[cyc %lu] RQ_SINGLE_COEXIST: pushing single beat while "
                               "another single beat already in rQueue (size=%zu) — "
                               "out-of-order delivery possible\n",
                               oramCycle, rQueue.size());
                        break;
                    }
                }
            }
            rQueue.push_back(beat);
            rb.flushedBeats++;
            if (rb.flushedBeats >= rb.totalBeats)
                pendingReadBursts.pop_front();
        }
    }
}

void OramDevice::drainWriteFifo()
{
    // Send 1 queued write per tick to HBM, modeling the AXI port's
    // write data buffer draining at 1 beat per clock cycle.
    if (wFifo.empty()) return;

    auto &entry = wFifo.front();

    if (entry.isPcie) {
        // PCIe write: use sendPkt for retry/flow-control.
        // Always pop from wFifo — sendPkt pushes to pcieWriteRetryQ on failure,
        // so the packet is owned by exactly one queue.
        PacketPtr pkt = entry.pkt;
        bool isPcie = entry.isPcie;
        wFifo.pop_front();
        sendPkt(pkt, isPcie);
    } else {
        // HBM write: don't call sendPkt (it pushes to hbmRetryQueue on
        // failure, creating double ownership with wFifo). Instead, send
        // directly and manage retry here.
        if (hbmBlocked) return;  // leave in wFifo, wready will go low

        PacketPtr pkt = entry.pkt;
        wFifo.pop_front();
        if (!hbmPort.sendTimingReq(pkt)) {
            hbmBlocked = true;
            hbmRetryQueue.push_back(pkt);
        }
    }
}

// =============================================================================
// Drive R/B channels — uses prevRready/prevBready sampled BEFORE eval
// =============================================================================

void OramDevice::driveAxiR()
{
    if (!rQueue.empty() && prevRready && oram->m_axi_rvalid)
        rQueue.pop_front();
    if (!rQueue.empty()) {
        auto &b = rQueue.front();
        oram->m_axi_rvalid = 1; oram->m_axi_rid = b.id;
        oram->m_axi_rlast = b.last ? 1 : 0; oram->m_axi_rresp = 0;
        memcpy(&oram->m_axi_rdata[0], b.data, AXI_DATA_BYTES);
        // Cycle-by-cycle stash R data trace (FSM=28 S_ST_LOAD) — EVERY cycle
        uint8_t fsm = oram->dbg_oram_state;
        if (fsm == FSM_ST_LOAD) {
            uint32_t *dw = (uint32_t *)b.data;
            DPRINTF(Oram, "[cyc %lu] STASH_R_DATA: data[0..3]=%08x %08x %08x %08x (last=%d)\n",
                   oramCycle, dw[0], dw[1], dw[2], dw[3], b.last);
        }
    } else {
        oram->m_axi_rvalid = 0; oram->m_axi_rlast = 0;
    }
}

void OramDevice::driveAxiB()
{
    // Pop the entry that was consumed last cycle (tracked by bQueuePresentIdx)
    int &presentIdx = bQueuePresentIdx;

    if (prevBready && oram->m_axi_bvalid && !bQueue.empty()) {
        // The entry at presentIdx was consumed
        if (presentIdx >= 0 && presentIdx < (int)bQueue.size()) {
            DPRINTF(Oram, "[%lu] B-CONSUMED: id=%d isHbm=%d isStash=%d isPm=%d bQ=%d idx=%d\n",
                    oramCycle, (int)bQueue[presentIdx].id,
                    (int)bQueue[presentIdx].isHbm, (int)bQueue[presentIdx].isStash,
                    (presentIdx < (int)bQueueIsPosmap.size()) ?
                        (int)bQueueIsPosmap[presentIdx] : -1,
                    (int)bQueue.size(), presentIdx);
            bQueue.erase(bQueue.begin() + presentIdx);
            if (presentIdx < (int)bQueueIsPosmap.size())
                bQueueIsPosmap.erase(bQueueIsPosmap.begin() + presentIdx);
        }
        presentIdx = -1;
    }

    // Warn if bQueue is growing unboundedly
    if (bQueue.size() > 64) {
        warn_once("bQueue has %d entries — possible stale BRESP "
                  "accumulation\n", (int)bQueue.size());
    }

    // Scan bQueue for the first entry matching the active mux master.
    // This avoids head-of-line blocking (e.g., bucket BRESP at front
    // blocking posmap BRESP behind it while sel_posmap=1).
    bool selPosmap = (oram->sel_posmap_out != 0);
    bool selStash = (oram->sel_stash_out != 0);

    presentIdx = -1;
    for (int i = 0; i < (int)bQueue.size(); i++) {
        bool entryIsStash = bQueue[i].isStash;
        bool entryIsPosmap = (i < (int)bQueueIsPosmap.size()) && bQueueIsPosmap[i];

        bool match = false;
        if (selPosmap && entryIsPosmap) match = true;
        else if (selStash && entryIsStash) match = true;
        else if (!selPosmap && !selStash && !entryIsStash && !entryIsPosmap) match = true;

        if (match) {
            presentIdx = i;
            break;
        }
    }

    if (presentIdx >= 0) {
        oram->m_axi_bvalid = 1;
        oram->m_axi_bid = bQueue[presentIdx].id;
        oram->m_axi_bresp = 0;
    } else {
        oram->m_axi_bvalid = 0;
    }
}

// =============================================================================
// Phase 1: Init pos_map + bucket_meta
// =============================================================================

void OramDevice::initNextSlot()
{
    // =================================================================
    // Three-phase initialization matching the hardware testbench
    // (onboard_test_top.v / ddr_pattern_init.v):
    //
    //   Phase A (initPhase 0):  DDR fill — write dummy data to ALL bucket
    //                           addresses in HBM via sendFunctional so the
    //                           RTL never reads uninitialized DRAM.
    //   Phase B (initPhase 1+): Per-slot pos_map init via RTL's
    //                           init_pm_wr_* signals (writes to internal
    //                           BRAM, NOT HBM).
    //   Phase C (initPhase 3+): Per-slot bucket_meta init via RTL's
    //                           init_bm_wr_* signals.
    //
    // After all slots: init_mode=0 → GRANT_LEASE → straight to ops.
    // No WRITE_INIT phase — the testbench doesn't have one either.
    // =================================================================

    oram->init_mode = 1;

    // --- Phase A: DDR fill (runs once, before per-slot init) ---
    if (initPhase == 0) {
        // Encrypted DDR_FILL: write AES-GCM encrypted data for each REAL
        // slot position. Dummy positions get zeros. IVT gets matching {IV, tag}.
        // This eliminates WRITE_INIT (no stash/eviction during init).
        int usedBuckets = ((int)numSlots + ORAM_C - 1) / ORAM_C;
        int totalBuckets = std::min(usedBuckets, (int)MAX_BUCKETS);

        inform("[cyc %lu] DDR_FILL: encrypting %d slots (%d buckets × C=%d) via SW AES-GCM",
               oramCycle, totalBuckets * ORAM_C, totalBuckets, ORAM_C);

        // Initialize SW AES-GCM engine with same key as RTL
        AesGcmSw gcm;
        gcm.setKeyFromWords(params().aes_key_0, params().aes_key_1,
                            params().aes_key_2, params().aes_key_3);

        Addr ivtBase = hbmBase + IVT_BASE_ADDR;
        static constexpr int BLOCK_SZ = 4096;       // bytes per ORAM block
        static constexpr int AES_BLK = 16;           // AES block size
        static constexpr int BLKS_PER_ORAM = BLOCK_SZ / AES_BLK; // 256
        static constexpr int BEATS_PER_BLK = BLOCK_SZ / AXI_DATA_BYTES; // 128

        // --- Pre-zero IVT BEFORE the DDR fill ---
        // The DDR fill writes real IVT entries for positions 0..C-1.
        // Dummy positions (C..Z-1) keep zeros. The zero-fill MUST run
        // first so the DDR fill's real entries are not overwritten.
        {
            int ivtEntries = std::min(totalBuckets, (int)MAX_BUCKETS) * ORAM_Z;
            inform("[cyc %lu] IVT_FILL: pre-zeroing %d IV/TAG entries (%d KB) base=0x%lx",
                   oramCycle, ivtEntries, ivtEntries * AXI_DATA_BYTES / 1024,
                   (uint64_t)ivtBase);
            for (int i = 0; i < ivtEntries; i++) {
                Addr addr = ivtBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendAtomic(pkt);
                delete pkt;
            }
            inform("[cyc %lu] IVT_FILL: pre-zero complete", oramCycle);
        }

        for (int b = 0; b < totalBuckets; b++) {
            // Route bucket data to HBM or host DDR5 based on bucket index.
            // Must match the runtime AR/AW routing (isHostBucket) so the
            // RTL reads bucket data from the same memory it was initialized in.
            bool hostBkt = isHostBucket((uint32_t)b);
            Addr bucketBase = hostBkt ? (hostBase + (Addr)b * BUCKET_BYTES)
                                      : (hbmBase  + (Addr)b * BUCKET_BYTES);

            for (int pos = 0; pos < ORAM_Z; pos++) {
                Addr blockBase = bucketBase + (Addr)pos * BLOCK_SZ;
                int physSlot = b * ORAM_Z + pos; // IVT index

                if (pos < ORAM_C) {
                    // Real slot: encrypt with AES-GCM
                    int slotIdx = b * ORAM_C + pos;

                    // Plaintext: all zeros (binary will overwrite with real data)
                    uint8_t plain[BLOCK_SZ];
                    memset(plain, 0, BLOCK_SZ);

                    // Unique IV per slot (32-bit slot index in low bytes)
                    uint8_t iv_aes[12] = {};
                    iv_aes[8]  = (slotIdx >> 24) & 0xFF;
                    iv_aes[9]  = (slotIdx >> 16) & 0xFF;
                    iv_aes[10] = (slotIdx >>  8) & 0xFF;
                    iv_aes[11] =  slotIdx        & 0xFF;

                    // Convert plaintext from memory layout to AES byte order
                    // (reverse each 16-byte block) then encrypt
                    uint8_t plain_aes[BLOCK_SZ], cipher_aes[BLOCK_SZ];
                    for (int i = 0; i < BLKS_PER_ORAM; i++)
                        for (int j = 0; j < AES_BLK; j++)
                            plain_aes[i*AES_BLK + j] = plain[i*AES_BLK + (AES_BLK-1-j)];

                    uint8_t tag_aes[16];
                    gcm.encrypt(iv_aes, plain_aes, BLOCK_SZ, cipher_aes, tag_aes);

                    // Convert ciphertext from AES byte order back to memory layout
                    uint8_t cipher_mem[BLOCK_SZ];
                    for (int i = 0; i < BLKS_PER_ORAM; i++)
                        for (int j = 0; j < AES_BLK; j++)
                            cipher_mem[i*AES_BLK + j] = cipher_aes[i*AES_BLK + (AES_BLK-1-j)];

                    // Write encrypted block (128 beats × 32 bytes) to correct memory
                    for (int beat = 0; beat < BEATS_PER_BLK; beat++) {
                        Addr addr = blockBase + (Addr)beat * AXI_DATA_BYTES;
                        auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                        PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                        uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                        memcpy(buf, cipher_mem + beat * AXI_DATA_BYTES, AXI_DATA_BYTES);
                        pkt->dataDynamic(buf);
                        if (hostBkt) pciePort.sendFunctional(pkt);
                        else         hbmPort.sendFunctional(pkt);
                        delete pkt;
                    }

                    // Write IVT entry: { pad[31:0], tag[127:0], iv[95:0] }
                    // Convert IV and tag from AES byte order to memory (LE) layout
                    uint8_t ivtBeat[AXI_DATA_BYTES];
                    memset(ivtBeat, 0, AXI_DATA_BYTES);
                    // IV[95:0] in bytes 0-11: reverse from AES order
                    for (int j = 0; j < 12; j++)
                        ivtBeat[j] = iv_aes[11 - j];
                    // Tag[127:0] in bytes 12-27: reverse from AES order
                    for (int j = 0; j < 16; j++)
                        ivtBeat[12 + j] = tag_aes[15 - j];

                    Addr ivtAddr = ivtBase + (Addr)physSlot * AXI_DATA_BYTES;
                    auto ivtReq = std::make_shared<Request>(ivtAddr, AXI_DATA_BYTES, 0, reqId);
                    PacketPtr ivtPkt = new Packet(ivtReq, MemCmd::WriteReq);
                    uint8_t *ivtBuf = new uint8_t[AXI_DATA_BYTES];
                    memcpy(ivtBuf, ivtBeat, AXI_DATA_BYTES);
                    ivtPkt->dataDynamic(ivtBuf);
                    hbmPort.sendFunctional(ivtPkt);
                    delete ivtPkt;

                } else {
                    // Dummy position: write zeros to correct memory
                    for (int beat = 0; beat < BEATS_PER_BLK; beat++) {
                        Addr addr = blockBase + (Addr)beat * AXI_DATA_BYTES;
                        auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                        PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                        uint8_t *buf = new uint8_t[AXI_DATA_BYTES]();
                        pkt->dataDynamic(buf);
                        if (hostBkt) pciePort.sendFunctional(pkt);
                        else         hbmPort.sendFunctional(pkt);
                        delete pkt;
                    }
                    // IVT for dummy: already zeroed by IVT_FILL above
                }
            }
        }

        inform("[cyc %lu] DDR_FILL: done, %d buckets × %d real slots encrypted",
               oramCycle, totalBuckets, ORAM_C);

        // --- DDR_FILL VERIFICATION: confirm IVT entries survived ---
        // Read back the IVT entry for bucket 0, pos 0 (physSlot=0) and
        // verify it's non-zero. If the IVT zero-fill ran AFTER the DDR
        // fill, this would be all zeros → tag mismatch on first access.
        {
            Addr checkAddr = ivtBase;  // physSlot 0
            uint8_t checkBuf[AXI_DATA_BYTES];
            auto fReq = std::make_shared<Request>(checkAddr, AXI_DATA_BYTES, 0, reqId);
            PacketPtr fPkt = new Packet(fReq, MemCmd::ReadReq);
            fPkt->dataStatic(checkBuf);
            hbmPort.sendFunctional(fPkt);
            delete fPkt;
            bool allZero = true;
            for (int b = 0; b < AXI_DATA_BYTES; b++)
                if (checkBuf[b] != 0) { allZero = false; break; }
            if (allZero)
                warn("[cyc %lu] DDR_FILL BUG: IVT[0] is all-zero after DDR fill! "
                     "IVT zero-fill ran AFTER DDR fill and destroyed IV/tag entries.",
                     oramCycle);
            else
                inform("[cyc %lu] DDR_FILL VERIFY: IVT[0] non-zero (%02x%02x%02x%02x...) — "
                       "init ordering correct",
                       oramCycle, checkBuf[3], checkBuf[2], checkBuf[1], checkBuf[0]);
        }

        // --- Zero-fill stash data region ---
        // RTL compiled for max STASH_DEPTH entries. At runtime, only
        // numSlots/2 entries are initialized (= active_buckets * Z / 4).
        // This avoids zeroing 64 MB when testing with small slot counts.
        //
        // ADDRESS-BASE CAVEAT: the RTL stash master issues AXI at
        // STASH_DDR_BASE = 0x10000000 (== STASH_BASE_ADDR). The driver bases
        // this fill at (hbmBase + stashOffset). With the default
        // stash_offset=0x08000000 and hbm_base=0x0 these DO NOT match
        // (0x08000000 vs 0x10000000). If your config does not override
        // stash_offset to 0x10000000, the fill targets the wrong region.
        // Verify your config sets stash_offset == STASH_BASE_ADDR.
        {
            Addr stashBase = hbmBase + stashOffset;
            // Runtime stash sizing: stash = numSlots / 2 (= active_buckets * Z / 4).
            // RTL is compiled for STASH_DEPTH (max), but we only init what we need.
            int stashEntries = std::min((int)(numSlots / 2), (int)STASH_DEPTH);
            int beatsPerEntry = 128;  // STASH_BEATS_PER_ENTRY = 128 (4KB / 32B)
            inform("[cyc %lu] STASH_FILL: zeroing %d stash entries (%d MB) base=0x%lx",
                   oramCycle, stashEntries, stashEntries * 4 / 1024,
                   (uint64_t)stashBase);
            if (stashBase != hbmBase + STASH_BASE_ADDR)
                warn("[cyc %lu] STASH_FILL: base 0x%lx != RTL STASH_DDR_BASE 0x%lx "
                     "(check stash_offset)", oramCycle, (uint64_t)stashBase,
                     (uint64_t)(hbmBase + STASH_BASE_ADDR));
            for (int e = 0; e < stashEntries; e++) {
                for (int beat = 0; beat < beatsPerEntry; beat++) {
                    Addr addr = stashBase + (Addr)e * 4096 + (Addr)beat * AXI_DATA_BYTES;
                    auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                    uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                    memset(buf, 0, AXI_DATA_BYTES);
                    pkt->dataDynamic(buf);
                    hbmPort.sendFunctional(pkt);
                    delete pkt;
                }
            }
        }

        // --- Initialize hash table regions ---
        // Init order: SLOT (zeros, sendAtomic) FIRST, then IVT/SLOTR, then
        // HEAD/NEXT (0xFF, sendFunctional) LAST. HEAD/NEXT use sendFunctional
        // to bypass the HBM timing model — sendAtomic has side effects on
        // nearby addresses that were corrupting HEAD back to zeros.
        // SLOT stays sendAtomic (zeroing, so corruption is harmless — any
        // side effect from SLOT's sendAtomic on other regions is overwritten
        // by the later sendFunctional writes to HEAD/NEXT).
        {
            // SLOT table (direct-indexed): 32768 entries / 8 per beat = 4096 beats
            Addr slotHtBase = hbmBase + HT_SLOT_BASE_ADDR;
            int slotBeats = 4096;
            inform("[cyc %lu] HT_FILL: zeroing SLOT table (%d beats, %d KB)",
                   oramCycle, slotBeats, slotBeats * AXI_DATA_BYTES / 1024);
            for (int i = 0; i < slotBeats; i++) {
                Addr addr = slotHtBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendAtomic(pkt);
                delete pkt;
            }

            inform("[cyc %lu] HT_FILL: SLOT zeroed (HEAD/NEXT init deferred to after IVT/SLOTR)",
                   oramCycle);
        }

        // Metadata address map summary (per-instance offsets) for debug.
        inform("[cyc %lu] METADATA MAP (hbmBase=0x%lx): STASH=0x%lx PM=0x%lx "
               "HT_SLOT=0x%lx HT_HEAD=0x%lx HT_NEXT=0x%lx IVT=0x%lx "
               "SLOT_R=0x%lx BUCKET_META=0x%lx",
               oramCycle, (uint64_t)hbmBase,
               (uint64_t)(hbmBase + STASH_BASE_ADDR),
               (uint64_t)(hbmBase + PM_BASE_ADDR),
               (uint64_t)(hbmBase + HT_SLOT_BASE_ADDR),
               (uint64_t)(hbmBase + HT_BKT_HEAD_ADDR),
               (uint64_t)(hbmBase + HT_BKT_NEXT_ADDR),
               (uint64_t)(hbmBase + IVT_BASE_ADDR),
               (uint64_t)(hbmBase + SLOT_R_BASE_ADDR),
               (uint64_t)(hbmBase + BUCKET_META_BASE_ADDR));

        // (IVT zero-fill moved to before DDR fill — see above)

        // --- Initialize slot_r region (per-stash-entry slot address) ---
        // slot_r is written on insert before being read on eviction, so a
        // pre-init is not strictly required for correctness. Zero it anyway for
        // determinism / clean debug, matching the IVT/HT init style.
        // One 256-bit beat per stash entry: { pad[223:0], slot_addr[31:0] }.
        {
            Addr srBase = hbmBase + SLOT_R_BASE_ADDR;
            int srEntries = std::min((int)(numSlots / 2), (int)STASH_DEPTH);
            inform("[cyc %lu] SLOTR_FILL: zeroing %d slot_r entries (%d KB) base=0x%lx",
                   oramCycle, srEntries, srEntries * AXI_DATA_BYTES / 1024,
                   (uint64_t)srBase);
            for (int i = 0; i < srEntries; i++) {
                Addr addr = srBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendAtomic(pkt);
                delete pkt;
            }
            inform("[cyc %lu] SLOTR_FILL: slot_r region initialized", oramCycle);
        }

        // NOTE: bucket_meta region is initialized per-slot in Phase C (case 3)
        // via direct HBM writes at BUCKET_META_BASE_ADDR, so no bulk fill here.

        // --- BKT HEAD and NEXT arrays: MUST be initialized LAST ---
        // gem5 HBM2 model sendAtomic has side effects on nearby addresses (documented
        // in the original code). Any sendAtomic fill (SLOT at 0x141, IVT at 0x144,
        // SLOTR at 0x148) can corrupt HEAD (0x142) or NEXT (0x143) if they were
        // written earlier. Writing HEAD/NEXT dead last guarantees no subsequent
        // sendAtomic fill can overwrite them.
        {
            // BKT head array: 8191 buckets / 16 per beat = 512 beats → 0xFF
            // Uses sendFunctional (not sendAtomic) to bypass the HBM timing
            // model entirely — sendAtomic has side effects on nearby addresses
            // that corrupted HEAD when it was written before SLOT (bug #5).
            // sendFunctional writes directly to backing store, no side effects.
            Addr bktHeadBase = hbmBase + HT_BKT_HEAD_ADDR;
            int headBeats = 512;
            inform("[cyc %lu] HT_FILL: init BKT head array LAST (%d beats) -> 0xFF via sendFunctional",
                   oramCycle, headBeats);
            for (int i = 0; i < headBeats; i++) {
                Addr addr = bktHeadBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0xFF, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendFunctional(pkt);
                delete pkt;
            }

            // BKT next array: 1024 beats → 0xFF (also sendFunctional)
            Addr bktNextBase = hbmBase + HT_BKT_NEXT_ADDR;
            int nextBeats = 1024;
            inform("[cyc %lu] HT_FILL: init BKT next array LAST (%d beats) -> 0xFF via sendFunctional",
                   oramCycle, nextBeats);
            for (int i = 0; i < nextBeats; i++) {
                Addr addr = bktNextBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0xFF, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendFunctional(pkt);
                delete pkt;
            }

            inform("[cyc %lu] HT_FILL: HEAD + NEXT initialized (last step, safe from gem5 HBM2 model side effects)",
                   oramCycle);
        }

        initPhase = 1;
        initSlotIdx = 0;
        return;
    }

    // --- Phase B & C: per-slot pos_map + bucket_meta init ---
    if (initSlotIdx >= (int)numSlots) {
        oram->init_mode = 0;
        oram->init_pm_wr_en = 0;
        oram->init_bm_wr_en = 0;
        ctrlState = OramState::GRANT_LEASE;
        initPhase = 0;
        inform("INIT_SLOTS done: %u slots -> GRANT_LEASE (no WRITE_INIT needed)",
               numSlots);
        return;
    }

    Addr slotAddr = LEASE_BASE + initSlotIdx * SLOT_SIZE;
    // 8 slots per bucket: 32 slots → 4 buckets
    uint32_t bucketIdx = initSlotIdx / ORAM_C;
    int posInBucket = initSlotIdx % ORAM_C;
    if (bucketIdx >= MAX_BUCKETS)
        fatal("Slot %u -> bucket %u >= B=%d", initSlotIdx, bucketIdx, MAX_BUCKETS);

    // Slot ID: upper bits of slot address (slotAddr >> 12)
    uint32_t slotId = (uint32_t)(slotAddr >> 12);

    switch (initPhase) {
      case 1: {
        // --- pos_map init: write full 32B beats to HBM ---
        // Accumulate entries per beat. When a beat is full (16 entries)
        // or we're at the last slot, write the full 32B via sendFunctional.
        // This avoids narrow 2-byte writes.

        static constexpr int ENTRIES_PER_BEAT = AXI_DATA_BYTES / 2;  // 16
        static uint8_t pmBeatBuf[64][AXI_DATA_BYTES];
        static uint32_t pmCurrentBeat[64];
        // Initialize on first use
        static bool pmInited[64] = {false};
        if (!pmInited[instanceId]) {
            memset(pmBeatBuf[instanceId], 0, AXI_DATA_BYTES);
            pmCurrentBeat[instanceId] = 0xFFFFFFFF;
            pmInited[instanceId] = true;
        }

        uint32_t pmSlotIndex = (uint32_t)(slotAddr >> 12);
        uint32_t beatIndex = pmSlotIndex / ENTRIES_PER_BEAT;
        uint32_t entryOffset = pmSlotIndex % ENTRIES_PER_BEAT;

        // New beat? Flush previous and start fresh
        if (beatIndex != pmCurrentBeat[instanceId]) {
            // Flush previous beat if it had data
            if (pmCurrentBeat[instanceId] != 0xFFFFFFFF) {
                Addr prevAddr = hbmBase + PM_BASE_ADDR
                    + (Addr)pmCurrentBeat[instanceId] * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(
                    prevAddr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memcpy(buf, pmBeatBuf[instanceId], AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendFunctional(pkt);
                delete pkt;
            }
            memset(pmBeatBuf[instanceId], 0, AXI_DATA_BYTES);
            pmCurrentBeat[instanceId] = beatIndex;
        }

        // Pack entry into beat buffer
        // ST_DUMMY (0): bucket data is dummy fill, not real encrypted data.
        // First WRITE to this slot will encrypt and store real data,
        // then update pos_map to ST_VALID.
        uint16_t entryVal = (uint16_t)((0u << BUCKET_ID_BITS) | bucketIdx);  // ST_DUMMY
        pmBeatBuf[instanceId][entryOffset * 2]     = entryVal & 0xFF;
        pmBeatBuf[instanceId][entryOffset * 2 + 1] = (entryVal >> 8) & 0xFF;

        // Last slot? Flush this beat
        if (initSlotIdx == (int)numSlots - 1) {
            Addr beatAddr = hbmBase + PM_BASE_ADDR
                + (Addr)beatIndex * AXI_DATA_BYTES;
            auto req = std::make_shared<Request>(
                beatAddr, AXI_DATA_BYTES, 0, reqId);
            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
            uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
            memcpy(buf, pmBeatBuf[instanceId], AXI_DATA_BYTES);
            pkt->dataDynamic(buf);
            hbmPort.sendFunctional(pkt);
            delete pkt;
            pmCurrentBeat[instanceId] = 0xFFFFFFFF;  // reset for potential next instance
        }

        if (initSlotIdx < 3 || initSlotIdx == (int)numSlots - 1)
            inform("[cyc %lu] INIT pm: slot=%d addr=0x%x bkt=%u st=ST_DUMMY "
                   "beat=%u off=%u entry=0x%04x",
                   oramCycle, initSlotIdx, (uint32_t)slotAddr, bucketIdx,
                   beatIndex, entryOffset, entryVal);
        initPhase = 3; break;
      }
      case 3: {
        // --- bucket_meta init via RTL's init_bm_wr_* signals ---
        // slot_list is Z * SLOT_ID_W = 8 * 15 = 120 bits, packed into
        // uint32_t[3]. Each 12-bit field holds one slot ID.
        //
        // We must accumulate: each time we add a slot to the bucket,
        // we write the FULL slot_list with all slots so far.
        // On the first slot (posInBucket=0), clear and write position 0.
        // On subsequent slots, read back the accumulated list, add the
        // new slot, write the updated list.
        //
        // Since init_bm_wr_* is a one-shot write to BRAM each time,
        // we need to maintain the accumulated slot_list in software.

        // Accumulate in a per-instance array (reset when bucket changes)
        static uint32_t accum_slot_list[64][4];
        static uint32_t prev_bucket[64];
        static bool bmInited[64] = {false};
        if (!bmInited[instanceId]) {
            memset(accum_slot_list[instanceId], 0, sizeof(accum_slot_list[instanceId]));
            prev_bucket[instanceId] = 0xFFFFFFFF;
            bmInited[instanceId] = true;
        }
        if (bucketIdx != prev_bucket[instanceId]) {
            memset(accum_slot_list[instanceId], 0, sizeof(accum_slot_list[instanceId]));
            prev_bucket[instanceId] = bucketIdx;
        }

        // Pack slotId into position posInBucket within the 96-bit field.
        // Each slot is SLOT_ID_W=15 bits (must match oram_params.vh).
        static constexpr int SLOT_ID_W = 15;
        int bitPos = posInBucket * SLOT_ID_W;
        int wordIdx = bitPos / 32;
        int bitOff = bitPos % 32;

        // Clear the 12-bit field at this position
        if (bitOff + SLOT_ID_W <= 32) {
            accum_slot_list[instanceId][wordIdx] &= ~(((1u << SLOT_ID_W) - 1) << bitOff);
            accum_slot_list[instanceId][wordIdx] |= (slotId & ((1u << SLOT_ID_W) - 1)) << bitOff;
        } else {
            // Spans two words
            int lo_bits = 32 - bitOff;
            int hi_bits = SLOT_ID_W - lo_bits;
            accum_slot_list[instanceId][wordIdx] &= ~(((1u << lo_bits) - 1) << bitOff);
            accum_slot_list[instanceId][wordIdx] |= (slotId & ((1u << lo_bits) - 1)) << bitOff;
            accum_slot_list[instanceId][wordIdx + 1] &= ~((1u << hi_bits) - 1);
            accum_slot_list[instanceId][wordIdx + 1] |= (slotId >> lo_bits) & ((1u << hi_bits) - 1);
        }

        // bucket_meta now lives in HBM (bucket_meta_hbm master). Write the
        // packed 124-bit entry DIRECTLY to HBM via sendFunctional at
        // BUCKET_META_BASE, mirroring the pos_map init above. This avoids
        // pulsing init_bm_wr_* through the HBM master's 2-deep write queue
        // (which could overflow at the init cadence). Beat layout matches
        // bucket_meta_hbm: { pad, slot_list[119:0], fill_count[3:0] }.
        //
        // We rewrite the FULL entry every slot (accumulated slot_list + new
        // fill); last write for a bucket leaves the final correct contents.
        {
            static constexpr int SLOT_ID_W2 = 15;   // local copy of SLOT_ID_W
            static constexpr int FILL_W2     = 4;    // FILL_CNT_W
            uint8_t bmBeat[AXI_DATA_BYTES];
            memset(bmBeat, 0, AXI_DATA_BYTES);
            // fill_count in low 4 bits of byte 0
            uint32_t fillv = (uint32_t)(posInBucket + 1) & 0xF;
            // slot_list occupies bits [ENTRY_W-1:FILL_W] = [123:4].
            // Build a 128-bit value = (slot_list << 4) | fill, then emit LE.
            // accum_slot_list holds slot_list in bits [119:0] across 4 words.
            // Shift left by FILL_W2 (4) into a 160-bit staging buffer (5 words).
            uint32_t stage[5] = {0,0,0,0,0};
            for (int w = 0; w < 4; w++) {
                uint32_t v = accum_slot_list[instanceId][w];
                stage[w]   |= (v << FILL_W2);
                stage[w+1] |= (v >> (32 - FILL_W2));
            }
            stage[0] |= fillv;   // fill in low nibble
            // Emit stage[0..4] little-endian into bmBeat (covers 160 bits;
            // only 124 are meaningful, the rest are zero pad).
            for (int w = 0; w < 5; w++) {
                bmBeat[w*4 + 0] = (uint8_t)(stage[w] & 0xFF);
                bmBeat[w*4 + 1] = (uint8_t)((stage[w] >> 8) & 0xFF);
                bmBeat[w*4 + 2] = (uint8_t)((stage[w] >> 16) & 0xFF);
                bmBeat[w*4 + 3] = (uint8_t)((stage[w] >> 24) & 0xFF);
            }
            Addr bmAddr = hbmBase + BUCKET_META_BASE_ADDR
                + (Addr)bucketIdx * AXI_DATA_BYTES;
            auto req = std::make_shared<Request>(bmAddr, AXI_DATA_BYTES, 0, reqId);
            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
            uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
            memcpy(buf, bmBeat, AXI_DATA_BYTES);
            pkt->dataDynamic(buf);
            hbmPort.sendFunctional(pkt);
            delete pkt;
        }
        // Keep init_bm_wr_en deasserted — init no longer drives the RTL port.
        oram->init_bm_wr_en = 0;

        if (initSlotIdx < 3 || initSlotIdx == (int)numSlots - 1
            || posInBucket == ORAM_C - 1)
            inform("[cyc %lu] INIT bm(HBM): slot=%d bkt=%u pos=%d slotId=%u "
                   "fill=%d list=[0x%08x 0x%08x 0x%08x 0x%08x]",
                   oramCycle, initSlotIdx, bucketIdx, posInBucket, slotId,
                   posInBucket + 1,
                   accum_slot_list[instanceId][0], accum_slot_list[instanceId][1],
                   accum_slot_list[instanceId][2], accum_slot_list[instanceId][3]);
        initPhase = 4; break;
      }
      case 4:
        // Deassert bm_wr_en, advance to next slot
        oram->init_bm_wr_en = 0;
        initPhase = 1;  // back to pos_map phase for next slot
        initSlotIdx++;
        break;
    }
}

// =============================================================================
// Phase 2: Grant lease
// =============================================================================

void OramDevice::grantLease()
{
    // Legacy single-lease path (cpuDriven=False): one lease covering all
    // slots, client_id=0. Behavior unchanged from pre-Step-3 code.
    if (!cpuDriven) {
        Addr leaseSize = (Addr)numSlots * SLOT_SIZE;
        switch (grantPhase) {
          case 0:
            oram->mgmt_op = 0; oram->mgmt_lease_id = 1;
            oram->mgmt_client_id = 0;
            oram->mgmt_base_addr = LEASE_BASE;
            oram->mgmt_size = leaseSize;
            oram->mgmt_duration = 0xFFFFFFFF;
            oram->mgmt_token_in = 0; oram->mgmt_req = 1;
            inform("[cyc %lu] GRANT_LEASE ph0: mgmt_req=1 base=0x%lx size=%lu",
                   oramCycle, LEASE_BASE, leaseSize);
            grantPhase = 1; break;
          case 1:
            oram->mgmt_req = 0; grantPhase = 2; break;
          case 2:
            if (oram->mgmt_ack) {
                leaseToken = oram->mgmt_token_out;
                inform("[cyc %lu] GRANT_LEASE ph2: mgmt_ack=1 token=0x%08x -> IDLE (no WRITE_INIT)",
                       oramCycle, leaseToken);
                // No WRITE_INIT needed — data was initialized during
                // INIT_SLOTS (DDR fill + pos_map + bucket_meta).
                // Go straight to IDLE to start ops.
                ctrlState = OramState::IDLE;
                grantPhase = 0;
                // Reset debug counters
                wrDbg.reset();
                rdDbg.reset();
            } else if (oram->mgmt_error) fatal("Lease failed");
            else if (oramCycle % 5000 == 0) {
                inform("[cyc %lu] GRANT_LEASE ph2: WAITING mgmt_ack=%d mgmt_error=%d",
                       oramCycle, (int)oram->mgmt_ack, (int)oram->mgmt_error);
            }
            break;
        }
        return;
    }

    // cpuDriven multi-lease path: grant K leases with non-overlapping
    // slot ranges. Each client i gets slots [i * slotsPerClient,
    // (i+1)*slotsPerClient). Store the token returned by the RTL in
    // leaseTokens[i] for CPU to read later via MMIO.
    unsigned K = numLogicalClients;
    unsigned slotsPerClient = numSlots / K;
    if (slotsPerClient == 0) {
        fatal("cpu_driven: num_slots=%u < num_logical_clients=%u",
              numSlots, K);
    }
    Addr perClientSize = (Addr)slotsPerClient * SLOT_SIZE;
    Addr clientBase    = LEASE_BASE +
                         (Addr)currentGrantClient * slotsPerClient * SLOT_SIZE;

    switch (grantPhase) {
      case 0:
        // Phase 0: assert mgmt_req with this client's parameters.
        // RTL convention: lease_id is 1-indexed (0 reserved for "invalid").
        // Legacy single-lease code used lease_id=1. For K clients we use
        // lease_id = client_id + 1, so valid IDs are 1..K.
        oram->mgmt_op = 0;
        oram->mgmt_lease_id = currentGrantClient + 1;
        oram->mgmt_client_id = currentGrantClient;
        oram->mgmt_base_addr = clientBase;
        oram->mgmt_size = perClientSize;
        oram->mgmt_duration = 0xFFFFFFFF;
        oram->mgmt_token_in = 0;
        oram->mgmt_req = 1;
        grantPhase = 1;
        break;
      case 1:
        // Phase 1: de-assert mgmt_req (one-cycle pulse).
        oram->mgmt_req = 0;
        grantPhase = 2;
        break;
      case 2:
        // Phase 2: wait for ack, record token, advance to next client
        // (or to WRITE_INIT if we've granted all K).
        if (oram->mgmt_ack) {
            uint32_t tok = oram->mgmt_token_out;
            leaseTokens[currentGrantClient] = tok;
            inform("Lease %u: base=0x%lx size=%lu KB token=0x%08x",
                   currentGrantClient, clientBase,
                   perClientSize / 1024, tok);
            currentGrantClient++;
            grantPhase = 0;
            if (currentGrantClient >= K) {
                // All leases granted. Keep leaseToken set to the FIRST
                // client's token for compatibility with existing write-init
                // code paths that use `leaseToken`.
                leaseToken = leaseTokens[0];
                if (cpuDriven) {
                    // cpu_driven: skip WRITE_INIT — the binary writes before
                    // it reads (test_random tracks which slots were written).
                    // DDR_FILL + pos_map + HT + IVT are already initialized via
                    // sendFunctional. Going through the ORAM path for 32768 init
                    // writes overflows the stash at full capacity.
                    writeInitIdx = numSlots;
                    ctrlState = OramState::IDLE;
                    ready = true;
                    inform("[cyc %lu] cpu_driven: skipping WRITE_INIT, straight to READY",
                           oramCycle);
                } else {
                    writeInitIdx = 0;
                    ctrlState = OramState::WRITE_INIT;
                }
            }
            // else loop: tick() will re-enter grantLease with currentGrantClient++
        } else if (oram->mgmt_error) {
            fatal("Lease %u failed", currentGrantClient);
        }
        break;
    }
}

// =============================================================================
// Phase 3: Write-init all slots
// =============================================================================

void OramDevice::writeInitNextSlot()
{
    if (writeInitIdx >= (int)numSlots) {
        inform("[cyc %lu] WRITE_INIT done: HBM=%u host=%u -> IDLE",
               oramCycle, hbmSlotCount, numSlots - hbmSlotCount);
        ctrlState = OramState::IDLE;
        // Reset AXI debug stats — WRITE_INIT accumulated junk
        wrDbg.reset();
        rdDbg.reset();
        // Step 3: CPU polls the READY bit via MMIO to know when init
        // (including all K lease grants) is complete.
        if (cpuDriven) {
            ready = true;
            inform("cpu_driven: READY — %u leases granted, tokens:",
                   (unsigned)leaseTokens.size());
            for (unsigned i = 0; i < leaseTokens.size(); i++)
                inform("  token[%u] = 0x%08x", i, leaseTokens[i]);
        }
        return;
    }
    if (oram->oram_busy) return;

    Addr slotAddr = LEASE_BASE + writeInitIdx * SLOT_SIZE;
    currentOpIsPcie = isHostSlot(writeInitIdx);
    currentOpIsWrite = true;
    currentOpAddr = slotAddr;

    // Per-client lease dispatch.
    //
    // The RTL has 2 hardware client slots (NUM_CLIENTS=2). All client_*
    // signals are vectors with the low half for hardware-client-0 and
    // high half for hardware-client-1:
    //   client_req[1:0]          — bit 0 = hw-client 0, bit 1 = hw-client 1
    //   client_op[1:0]           — same
    //   client_wdata_valid[1:0]  — same
    //   client_lease_id[15:0]    — low byte = hw-client 0, high byte = hw-client 1
    //   client_slot_addr[63:0]   — low 32b = hw-client 0, high 32b = hw-client 1
    //   client_token[63:0]       — same layout
    //   client_wdata[0..7]       — hw-client 0 (32B)
    //   client_wdata[8..15]      — hw-client 1 (32B)
    //
    // In cpu_driven mode we have K=numLogicalClients leases, each covering
    // a disjoint slot range. The logical client index maps 1:1 to the
    // hardware client index (so hw-client 0 uses lease 1, hw-client 1 uses
    // lease 2). Write-init must use the correct hardware client slot for
    // the address it is writing — otherwise val_valid stays 0 and
    // access_violation fires (see secure_oram_top.v:160).
    //
    // In legacy mode (cpu_driven=False), all slots use hw-client 0 with
    // lease_id=1 (unchanged behavior).
    unsigned hwClient;
    uint8_t  thisLeaseId;
    uint32_t thisToken;
    if (cpuDriven) {
        unsigned slotsPerClient = numSlots / numLogicalClients;
        unsigned clientIdx = writeInitIdx / slotsPerClient;
        if (clientIdx >= numLogicalClients)
            clientIdx = numLogicalClients - 1;
        hwClient    = clientIdx;                         // 0 or 1
        thisLeaseId = (uint8_t)(clientIdx + 1);          // 1-indexed
        thisToken   = leaseTokens[clientIdx];
    } else {
        hwClient    = 0;
        thisLeaseId = 1;
        thisToken   = leaseToken;
    }

    // Drive the correct half of each vector signal for this hw client,
    // and zero out the other half (the other client is idle).
    activeHwClient = hwClient;   // feedClientWdata uses this for subsequent beats
    uint8_t  reqBit      = (uint8_t)(1u << hwClient);
    uint8_t  wdataValBit = (uint8_t)(1u << hwClient);
    uint64_t addrPacked  = ((uint64_t)slotAddr)   << (32 * hwClient);
    uint64_t tokenPacked = ((uint64_t)thisToken)  << (32 * hwClient);
    uint16_t leaseIdPk   = ((uint16_t)thisLeaseId) << (8  * hwClient);

    oram->client_req         = reqBit;
    oram->client_op          = (uint8_t)(1u << hwClient);   // op=1 (write) on this client
    oram->client_slot_addr   = addrPacked;
    oram->client_token       = tokenPacked;
    oram->client_lease_id    = leaseIdPk;

    // wdata[0..7] = hw-client 0's 32B; wdata[8..15] = hw-client 1's 32B.
    // Zero both first, then write into the half belonging to hwClient.
    memset(&oram->client_wdata[0], 0, sizeof(oram->client_wdata));
    unsigned wbase = hwClient * 8;
    for (int i = 0; i < 8; i++)
        oram->client_wdata[wbase + i] = writeInitIdx ^ 0 ^ i;
    oram->client_wdata_valid = wdataValBit;

    DPRINTF(Oram, "[%lu] Write-init slot %u/%u -> %s (hwC=%u lease=%u)\n",
            oramCycle, writeInitIdx, numSlots,
            currentOpIsPcie ? "host" : "HBM",
            hwClient, thisLeaseId);
}

// =============================================================================
// Phase 4: Test operations
// =============================================================================

void OramDevice::generateNextOp()
{
    if (opsCompleted >= numOps) {
        ctrlState = OramState::DONE;
        printStats();
        completedInstances++;
        if (completedInstances >= totalInstances)
            exitSimLoop("ORAM simulation complete");
        return;
    }
    if (oram->oram_busy) return;

    std::uniform_int_distribution<uint32_t> dist(0, numSlots - 1);

    // Decide write vs read. Alternate, but force write if nothing
    // has been written yet (can't read dummy data).
    bool wantWrite = (opsCompleted % 2 == 0);
    if (!wantWrite && writtenSlots.empty())
        wantWrite = true;  // no written slots to read — force write

    uint32_t slotIdx;
    if (wantWrite) {
        slotIdx = dist(rng);
        writtenSlots.insert(slotIdx);
    } else {
        // Pick randomly from writtenSlots
        std::uniform_int_distribution<uint32_t> wsDist(0, writtenSlots.size() - 1);
        auto it = writtenSlots.begin();
        std::advance(it, wsDist(rng));
        slotIdx = *it;
    }

    currentOpAddr = LEASE_BASE + slotIdx * SLOT_SIZE;
    currentOpIsWrite = wantWrite;
    currentOpIsPcie = isHostSlot(slotIdx);
    if (currentOpIsPcie) pcieOps++; else localOps++;

    oram->client_req = 1;
    oram->client_op = currentOpIsWrite ? 1 : 0;
    oram->client_slot_addr = currentOpAddr;
    oram->client_token = leaseToken;
    oram->client_lease_id = 1;

    if (currentOpIsWrite) {
        for (int i = 0; i < 16; i++)
            oram->client_wdata[i] = opsCompleted ^ i;
        oram->client_wdata_valid = 1;
    } else {
        oram->client_wdata_valid = 0;
    }

    ctrlState = OramState::PROCESSING;
    opStartCycle = oramCycle;
    memset(opPhaseCycles, 0, sizeof(opPhaseCycles));

    DPRINTF(Oram, "[%lu] Op %u/%u: %s slot=%u -> %s\n",
            oramCycle, opsCompleted + 1, numOps,
            currentOpIsWrite ? "WR" : "RD", slotIdx,
            currentOpIsPcie ? "PCIe" : "HBM");
}

void OramDevice::feedClientWdata()
{
    if (!currentOpIsWrite) return;
    int beat = oram->oram_beat_cnt;

    memset(&oram->client_wdata[0], 0, sizeof(oram->client_wdata));
    unsigned wbase = activeHwClient * 8;

    if (cpuDriven && ctrlState == OramState::PROCESSING) {
        // Step 4-9: cpu-driven WR op. The user's data was latched into
        // the head queue entry when the op was queued; drive that as
        // the RTL consumes wdata. The head op is in IN_PROGRESS phase
        // (see IDLE dispatcher); feedClientWdata only fires while
        // PROCESSING, so cmdQueue.front() is guaranteed to be the
        // currently-in-flight op. Constant value across all beats — the
        // RTL latches it when client_wdata_valid handshakes with the
        // client arbiter.
        if (cmdQueue.empty() ||
            cmdQueue.front().phase != CmdEntry::Phase::IN_PROGRESS) {
            // Defensive: should not happen. If it does, drive zeros.
            // (Logging once per occurrence is too noisy under sustained
            // ops; use warn_once equivalent if it becomes a problem.)
        } else {
            const CmdEntry &op = cmdQueue.front();
            for (int i = 0; i < 8; i++)
                oram->client_wdata[wbase + i] = op.wdata[i];
        }
    } else {
        // WRITE_INIT and legacy RNG-driven PROCESSING: synthetic pattern
        // seeded by writeInitIdx (init phase) or opsCompleted (legacy).
        // beat varies per cycle so the RTL writes distinct bytes into
        // each path-bucket beat for testing.
        uint32_t seed = (ctrlState == OramState::WRITE_INIT) ?
                        (uint32_t)writeInitIdx : opsCompleted;
        for (int i = 0; i < 8; i++)
            oram->client_wdata[wbase + i] = seed ^ beat ^ i;
    }
    oram->client_wdata_valid = (uint8_t)(1u << activeHwClient);
}

void OramDevice::completeOp()
{
    oram->client_wdata_valid = 0;

    if (ctrlState == OramState::WRITE_INIT) {
        writeInitIdx++;
        DPRINTF(Oram, "[%lu] Write-init slot %d done\n",
                oramCycle, writeInitIdx);
        return;
    }

    uint64_t cyc = oramCycle - opStartCycle;
    totalOpCycles += cyc;
    opsCompleted++;

    // E2E: stamp RTL completion
    if (!cmdQueue.empty() &&
        cmdQueue.front().phase == CmdEntry::Phase::IN_PROGRESS) {
        cmdQueue.front().rtlDoneTick = curTick();
    }

    // Progress tracking — always print so user sees forward movement
    inform("ORAM[%u] op %u/%u done (%s, %lu cyc, %.1f us elapsed)",
           instanceId, opsCompleted, numOps,
           currentOpIsPcie ? "PCIe" : "HBM", cyc,
           curTick() / 1e6);

    // HT operation anomaly detection
    if (oram->dbg_ht_ins_overwritten || oram->dbg_ht_del_overwritten ||
        oram->dbg_ht_ins_issued != oram->dbg_ht_ins_completed ||
        oram->dbg_ht_del_issued != oram->dbg_ht_del_completed ||
        oram->dbg_ht_latch_ins_active || oram->dbg_ht_latch_del_active) {
        DPRINTF(Oram, "HT_ANOMALY inst=%u op=%u: "
             "ins=%u/%u del=%u/%u overwrite_ins=%d overwrite_del=%d "
             "latch_ins=%d latch_del=%d\n",
             instanceId, opsCompleted,
             (unsigned)oram->dbg_ht_ins_issued,
             (unsigned)oram->dbg_ht_ins_completed,
             (unsigned)oram->dbg_ht_del_issued,
             (unsigned)oram->dbg_ht_del_completed,
             (int)oram->dbg_ht_ins_overwritten,
             (int)oram->dbg_ht_del_overwritten,
             (int)oram->dbg_ht_latch_ins_active,
             (int)oram->dbg_ht_latch_del_active);
    }
    // Log HT counters for every READ op (to check the op that LAST accessed a failing slot)
    if (!currentOpIsWrite) {
        DPRINTF(Oram, "HT_READ_SUMMARY inst=%u op=%u slotIdx=%u: "
               "ins=%u/%u del=%u/%u ovr_i=%d ovr_d=%d found_b=%d found_s=%d\n",
               instanceId, opsCompleted,
               (unsigned)((currentOpAddr - LEASE_BASE) / SLOT_SIZE),
               (unsigned)oram->dbg_ht_ins_issued,
               (unsigned)oram->dbg_ht_ins_completed,
               (unsigned)oram->dbg_ht_del_issued,
               (unsigned)oram->dbg_ht_del_completed,
               (int)oram->dbg_ht_ins_overwritten,
               (int)oram->dbg_ht_del_overwritten,
               (int)oram->dbg_found_in_bucket,
               (int)oram->dbg_found_in_stash);
    }

    // Round-trip data verification
    if (!currentOpIsWrite && rdataShadowValid[activeHwClient]) {
        DPRINTF(Oram, "VERIFY-READ op=%u addr=0x%lx rdata[0..3]=%08x %08x %08x %08x\n",
               opsCompleted, currentOpAddr,
               rdataShadow[activeHwClient][0], rdataShadow[activeHwClient][1],
               rdataShadow[activeHwClient][2], rdataShadow[activeHwClient][3]);
    } else if (!currentOpIsWrite) {
        DPRINTF(Oram, "VERIFY-READ op=%u addr=0x%lx NO RDATA SHADOW\n", 
               opsCompleted, currentOpAddr);
    }
    if (currentOpIsWrite) {
        DPRINTF(Oram, "VERIFY-WRITE op=%u addr=0x%lx beat0_wdata[0..3]=%08x %08x %08x %08x\n",
               opsCompleted, currentOpAddr,
               currentOpWdata[0], currentOpWdata[1],
               currentOpWdata[2], currentOpWdata[3]);
    }

    DPRINTF(Oram, "[%lu] Op %u done: %s 0x%lx %lu cyc (%s)\n",
            oramCycle, opsCompleted,
            currentOpIsWrite ? "WR" : "RD", currentOpAddr, cyc,
            currentOpIsPcie ? "PCIe" : "HBM");

    // Per-op phase breakdown
    static const char* stateNames[] = {
        "IDLE", "POS_LOOKUP", "DDR_READ", "SCAN", "SCAN2",
        "EXT_IV_RD", "EXT_DEC_START", "EXT_DEC_FEED", "EXT_DEC_RECV",
        "EXT_DEC_WAIT", "EXTRACT_WR", "STASH_SEARCH", "STASH_READ",
        "COMPACT", "EVICT", "EV_ENC_START", "EV_ENC_FEED",
        "EV_ENC_RECV", "EV_ENC_TAG", "DDR_WRITE",
        "SB_RD_ENC_START", "SB_RD_ENC_FEED", "SB_RD_ENC_RECV",
        "SB_RD_ENC_TAG", "SB_WR_ENC_START", "SB_WR_ENC_FEED",
        "SB_WR_ENC_RECV", "SB_WR_ENC_TAG", "ST_LOAD", "ST_FLUSH",
        "PM_WAIT"
    };
    inform("[%s] Op %u phase breakdown (%lu total cyc):", name(), opsCompleted, cyc);
    for (int i = 0; i < NUM_FSM_STATES; i++) {
        if (opPhaseCycles[i] > 0) {
            inform("    %-20s %6lu cyc (%4.1f%%)",
                   stateNames[i], opPhaseCycles[i],
                   100.0 * opPhaseCycles[i] / cyc);
        }
    }

    // DDR_WRITE debug breakdown
    if (wrDbg.totalCycles > 0) {
        DPRINTF(Oram, "  DDR_WRITE W-channel debug (%lu cycles):\n", wrDbg.totalCycles);
        DPRINTF(Oram, "    W handshakes:  %lu (RTL wvalid && wready)\n",
               wrDbg.wHandshakes);
        DPRINTF(Oram, "    W stalls:      %lu (RTL wvalid && !wready)\n",
               wrDbg.wStalls);
        DPRINTF(Oram, "    W idle:        %lu (RTL !wvalid, sub-burst gaps)\n",
               wrDbg.wIdle);
        DPRINTF(Oram, "    gem5 accepted: %lu (sendTimingReq true)\n",
               wrDbg.sendAccepted);
        DPRINTF(Oram, "    gem5 rejected: %lu (sendTimingReq false, queued)\n",
               wrDbg.sendRejected);
        DPRINTF(Oram, "    gem5 retries:  %lu (sent via retry queue)\n",
               wrDbg.retrySent);
        DPRINTF(Oram, "    AW handshakes: %lu\n", wrDbg.awHandshakes);
        DPRINTF(Oram, "    BRESPs avail:  %lu (bvalid asserted)\n", wrDbg.brespAvail);
        DPRINTF(Oram, "    BRESPs recv:   %lu (bvalid && bready consumed)\n",
               wrDbg.brespRecv);
        DPRINTF(Oram, "    BRESPs noMatch:%lu (bQueue non-empty but wrong type)\n",
               wrDbg.brespNoMatch);
        DPRINTF(Oram, "    maxBqDepth:    %lu\n", wrDbg.maxBqDepth);
        DPRINTF(Oram, "    W timing:      first=%lu last=%lu span=%lu cyc\n",
               wrDbg.firstWcycle, wrDbg.lastWcycle,
               wrDbg.lastWcycle - wrDbg.firstWcycle);
        DPRINTF(Oram, "    B timing:      first=%lu last=%lu span=%lu cyc\n",
               wrDbg.firstBcycle, wrDbg.lastBcycle,
               wrDbg.firstBcycle > 0 ? wrDbg.lastBcycle - wrDbg.firstBcycle : 0UL);
        DPRINTF(Oram, "    W→B latency:   %lu cyc (first BRESP - first W)\n",
               wrDbg.firstBcycle > wrDbg.firstWcycle ?
               wrDbg.firstBcycle - wrDbg.firstWcycle : 0UL);
        DPRINTF(Oram, "    Effective:     %.3f cyc/beat (%lu beats in %lu cyc)\n",
               wrDbg.wHandshakes > 0 ?
               (double)wrDbg.totalCycles / wrDbg.wHandshakes : 0.0,
               wrDbg.wHandshakes, wrDbg.totalCycles);
    }
    wrDbg.reset();

    // DDR_READ debug breakdown
    if (rdDbg.totalCycles > 0) {
        DPRINTF(Oram, "  DDR_READ R-channel debug (%lu cycles):\n", rdDbg.totalCycles);
        DPRINTF(Oram, "    R handshakes:  %lu (RTL rvalid && rready)\n",
               rdDbg.rHandshakes);
        DPRINTF(Oram, "    R stalls:      %lu (RTL rvalid && !rready)\n",
               rdDbg.rStalls);
        DPRINTF(Oram, "    R idle:        %lu (RTL !rvalid, waiting for data)\n",
               rdDbg.rIdle);
        DPRINTF(Oram, "    AR handshakes: %lu\n", rdDbg.arHandshakes);
        DPRINTF(Oram, "    gem5 accepted: %lu (read sendPkt true)\n",
               rdDbg.sendAccepted);
        DPRINTF(Oram, "    gem5 rejected: %lu (read sendPkt false, queued)\n",
               rdDbg.sendRejected);
        DPRINTF(Oram, "    gem5 retries:  %lu (sent via retry queue)\n",
               rdDbg.retrySent);
        DPRINTF(Oram, "    mem responses: %lu (handleMemResp reads)\n",
               rdDbg.memResps);
        DPRINTF(Oram, "    maxRqDepth:    %lu\n", rdDbg.maxRqDepth);
        DPRINTF(Oram, "    maxPendReads:  %lu\n", rdDbg.maxPendReads);
        DPRINTF(Oram, "    R timing:      first=%lu last=%lu span=%lu cyc\n",
               rdDbg.firstRcycle, rdDbg.lastRcycle,
               rdDbg.firstRcycle > 0 ? rdDbg.lastRcycle - rdDbg.firstRcycle : 0UL);
        DPRINTF(Oram, "    Effective:     %.3f cyc/beat (%lu beats in %lu cyc)\n",
               rdDbg.rHandshakes > 0 ?
               (double)rdDbg.totalCycles / rdDbg.rHandshakes : 0.0,
               rdDbg.rHandshakes, rdDbg.totalCycles);
    }
    rdDbg.reset();

    // Step 4-9: if CPU initiated this op, capture results into the head
    // queue entry, write a 64B result packet via pcie_port. The packet
    // write is asynchronous; queue entry is popped (and cpuOpCount
    // incremented) only when the WriteResp returns to recvTimingResp.
    // Reads draw from rdataShadow (populated in tick() when rdata_valid
    // was seen) — NEVER from the live client_rdata signal, which may
    // already be stale at this point.
    if (cpuDriven && !cmdQueue.empty() &&
        cmdQueue.front().phase == CmdEntry::Phase::IN_PROGRESS) {
        CmdEntry &op = cmdQueue.front();

        if (op.op != 1) {  // READ — use op.op, NOT currentOpIsWrite
                           // (currentOpIsWrite may be stale if next op dispatched)
            if (rdataShadowValid[activeHwClient]) {
                for (int i = 0; i < 8; i++)
                    op.rdata[i] = rdataShadow[activeHwClient][i];
                op.rdata_valid = true;
                rdataShadowValid[activeHwClient] = false;  // consume
            } else if (oram->client_rdata_valid & (1u << activeHwClient)) {
                // Shadow missed — capture directly from live RTL signal.
                // This can happen if client_done and client_rdata_valid
                // assert on the same cycle but the shadow capture order
                // within tick() missed it.
                unsigned rb = activeHwClient * 8;
                for (int i = 0; i < 8; i++)
                    op.rdata[i] = oram->client_rdata[rb + i];
                op.rdata_valid = true;
                inform("RDATA_LIVE_FALLBACK hw=%u slot=0x%lx "
                       "(shadow missed, live signal OK)",
                       activeHwClient, currentOpAddr);
            } else {
                // Truly not valid — defensive fallback.
                for (int i = 0; i < 8; i++) op.rdata[i] = 0;
                op.rdata_valid = false;
                warn("CPU-op RD done but rdataShadow not populated "
                     "(hw=%u, slot=0x%lx)", activeHwClient, currentOpAddr);
            }
        } else {
            op.rdata_valid = false;
        }
        op.phase = CmdEntry::Phase::COMPUTE_DONE;

        // Write 64B result packet to result_buf_base + opIdx*64. The
        // packet's sender state carries opIdx so recvTimingResp can
        // match it back to this queue entry on commit.
        // Step 6 design: cpuOpCount is NOT incremented here. It's
        // incremented in recvTimingResp when the WriteResp arrives.
        // This makes "cpuOpCount" semantically "ops committed to
        // memory" — exactly what the CPU should poll for verification
        // readiness.
        sendResultPacket(op.opIdx);
        DPRINTF(Oram, "[%lu] CPU-op compute-done: %s slot=0x%lx "
                "opIdx=%lu (rdata_valid=%d); awaiting result-pkt "
                "write completion\n",
                oramCycle, currentOpIsWrite ? "WR" : "RD",
                currentOpAddr, op.opIdx, (int)op.rdata_valid);
    }

    ctrlState = OramState::IDLE;

    // Purge stale bQueue entries that can't match any pending write.
    // Over many operations, wrong-type BRESPs can accumulate if
    // needHbm flips between ops.
    while (bQueue.size() > 32) {
        bQueue.pop_back();
        if (!bQueueIsPosmap.empty()) bQueueIsPosmap.pop_back();
    }
}

// =============================================================================
// Stats
// =============================================================================

void OramDevice::printStats()
{
    if (!opsCompleted || statsPrinted) return;
    statsPrinted = true;

    double avg = (double)totalOpCycles / opsCompleted;
    double ns = oramClkPeriod / 1000.0;

    inform("[%s] === ORAM Results ===", name());
    inform("  %u slots: HBM=%u host=%u", numSlots, hbmSlotCount,
           numSlots - hbmSlotCount);
    inform("  Ops: %u/%u  HBM=%lu  PCIe=%lu",
           opsCompleted, numOps, localOps, pcieOps);
    inform("  Avg: %.1f cyc/op (%.2f us)", avg, avg * ns / 1000.0);
    if (opsCompleted > 0)
        inform("  Throughput: %.0f ops/s",
               opsCompleted / (totalOpCycles * ns * 1e-9));
    // RTL bandwidth counters (bw_bucket_rd_beats etc.) removed — not in new RTL
    inform("  Routing: stash+posmap(HBM=%lu PCIe=%lu) bucket(HBM=%lu PCIe=%lu)",
           stashHbmBeats, stashPcieBeats,
           bucketHbmBeats, bucketPcieBeats);
    inform("  Metadata HBM beats: PM=%lu HT=%lu IVT=%lu SLOT_R=%lu BUCKET_META=%lu",
           pmBeats, htBeats, ivtBeats, slotrBeats, bmetaBeats);
    if (stashPcieBeats > 0)
        warn("  STASH ROUTING ERROR: %lu stash beats went to PCIe!", stashPcieBeats);

    // E2E latency summary (cpu_driven mode)
    if (e2eOpsTracked > 0) {
        Tick clk = oramClkPeriod;
        Tick avgRtl = e2eRtlSum / e2eOpsTracked;
        Tick avgWb  = e2eWritebackSum / e2eOpsTracked;
        Tick avgSS  = avgRtl + avgWb;
        inform("  === E2E LATENCY (%lu ops) ===", e2eOpsTracked);
        inform("    RTL processing:  avg %lu cyc (%.1f ns)",
               avgRtl / clk, (double)avgRtl / 1000.0);
        inform("    result write:    avg %lu cyc (%.1f ns)",
               avgWb / clk, (double)avgWb / 1000.0);
        inform("    STEADY-STATE:    avg %lu cyc (%.1f ns)  [RTL + writeback]",
               avgSS / clk, (double)avgSS / 1000.0);
        inform("    COLD-START:      %lu cyc (%.1f ns)  [first op, incl. ring fetch]",
               e2eColdStart / clk, (double)e2eColdStart / 1000.0);
        inform("    (ring fetch is pipelined — overlapped with prev op's RTL)");
    }
}

// =============================================================================
// cmd_port (cpu_driven mode) — Step 1 stubs.
//
// Inherits from SimpleTimingPort, which auto-implements recvTimingReq and
// recvFunctional by calling our recvAtomic. We only need recvAtomic plus
// getAddrRanges.
//
// Step 1: recvAtomic just logs the access and zero-fills read responses.
// Real mgmt/client command processing lands in Step 3+. When cpu_driven
// is False, nothing connects to this port, so these handlers never fire.
// =============================================================================

AddrRangeList OramDevice::CmdPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(AddrRange(dev.cmdBase, dev.cmdBase + CMD_REGION_BYTES));
    return ranges;
}

Tick OramDevice::CmdPort::recvAtomic(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - dev.cmdBase;
    DPRINTF(Oram, "cmd_port %s off=0x%lx size=%u @%lu\n",
            pkt->isRead() ? "RD" : "WR", offset, pkt->getSize(),
            curTick());

    // =========================================================================
    // Register map (little-endian, all 32-bit unless noted).
    //
    // READ:
    //   0x80           NUM_CLIENTS_HW (K)
    //   0x84+i*4       GRANTED_TOKEN[i]   (i = 0..15)
    //   0xC4           READY bit 0: init complete
    //   0xD0           OP_STATUS  bit 0: in_progress, bit 1: done
    //   0xD4           OP_RESULT_VALID bit 0: last op was a read with
    //                                  captured rdata
    //   0x200+i*4      OP_RDATA[i]        (i = 0..7)   — 32B of read data
    //
    // WRITE:
    //   0x100          CLIENT_SLOT_ADDR
    //   0x104          CLIENT_TOKEN
    //   0x108          CLIENT_LEASE_ID    (low byte only)
    //   0x10C          CLIENT_OP_CODE     (bit 0: 1=write, 0=read)
    //   0x110          CLIENT_HW_ID       (0 or 1)
    //   0x114+i*4      CLIENT_WDATA[i]    (i = 0..7)
    //   0x134          CLIENT_DOORBELL    (any value rings doorbell
    //                                      → cmdOp.pending = true)
    //   0xD8           OP_STATUS_CLEAR    (any value clears done bit)
    // =========================================================================

    if (pkt->isRead()) {
        uint8_t *buf = pkt->getPtr<uint8_t>();
        memset(buf, 0, pkt->getSize());
        uint32_t value = 0;

        if (offset == 0xC4) {
            value = dev.ready ? 1 : 0;
        } else if (offset == OramDevice::GATE_STATUS_OFFSET) {
            // Step 4 gate status:
            //   0 = held (gated_start=true and gate not yet released)
            //   1 = released, init still running
            //   2 = released and init complete (== ready bit at 0xC4)
            // Configs without gated_start always read 1 (released)
            // immediately, transitioning to 2 when init completes.
            if (dev.gatedStart && !dev.gateReleased) {
                value = 0;
            } else if (!dev.ready) {
                value = 1;
            } else {
                value = 2;
            }
        } else if (offset == 0x80) {
            value = (uint32_t)dev.numLogicalClients;
        } else if (offset >= 0x84 && offset < 0x84 + 16 * 4) {
            unsigned idx = (offset - 0x84) / 4;
            if (idx < dev.leaseTokens.size())
                value = dev.leaseTokens[idx];
        } else if (offset == 0xD0) {
            // Step 8: OP_STATUS reflects the queue. bit 0 (in_progress)
            // = head op is IN_PROGRESS or COMPUTE_DONE; bit 1 (done) =
            // queue is empty AND at least one op has committed (i.e.,
            // all queued ops are committed). Under depth=1 this matches
            // the prior PendingClientOp semantics; under depth>1 the
            // CPU should poll cpuOpCount (0xE0) instead.
            bool any_in_flight = false;
            bool any_done      = false;
            if (!dev.cmdQueue.empty()) {
                auto &front = dev.cmdQueue.front();
                if (front.phase == CmdEntry::Phase::IN_PROGRESS ||
                    front.phase == CmdEntry::Phase::COMPUTE_DONE)
                    any_in_flight = true;
            }
            if (dev.cmdQueue.empty() && dev.cpuOpCount > 0)
                any_done = true;
            value = (any_in_flight ? 1 : 0) | (any_done ? 2 : 0);
        } else if (offset == 0xD4) {
            // OP_RESULT_VALID reflects rdata from the most-recent
            // committed op. With depth>1 ops, this is brittle —
            // workloads should read from result_buf instead.
            value = (dev.cpuOpCount > 0 && dev.lastCompletedRdataValid) ? 1 : 0;
        } else if (offset == 0xE0) {
            // Step 5: CPU reads the device's committed-op counter.
            // Under Step 8 semantics this equals "number of result
            // packets that have landed in memory".
            value = (uint32_t)(dev.cpuOpCount & 0xFFFFFFFF);
        } else if (offset >= 0x200 && offset < 0x200 + 8 * 4) {
            unsigned idx = (offset - 0x200) / 4;
            value = dev.lastCompletedRdata[idx];
        }

        unsigned copyBytes = pkt->getSize() < 4 ? pkt->getSize() : 4;
        memcpy(buf, &value, copyBytes);
    } else {
        // Write decode. We only care about 32-bit writes aligned on 4B
        // boundaries. Non-aligned or non-4B writes silently ignored —
        // the workload is well-formed by design.
        if (pkt->getSize() == 4) {
            uint32_t v;
            memcpy(&v, pkt->getPtr<uint8_t>(), 4);

            // Debug echo registers written by the C test at end-of-run.
            // Surface PASS/FAIL to the console so the test verdict is visible
            // without inspecting the result buffer.
            if (offset == 0x20) {
                inform("[%s] CTEST DEBUG_PASS = %u", dev.name().c_str(), v);
            } else if (offset == 0x24) {
                inform("[%s] CTEST DEBUG_FAIL = %u", dev.name().c_str(), v);
            }

            if (offset == 0x100) {
                dev.stagingCmd.slot_addr = v;
            } else if (offset == 0x104) {
                dev.stagingCmd.token = v;
            } else if (offset == 0x108) {
                dev.stagingCmd.lease_id = (uint8_t)v;
            } else if (offset == 0x10C) {
                dev.stagingCmd.op = (uint8_t)(v & 1u);
            } else if (offset == 0x110) {
                dev.stagingCmd.hw_client = (uint8_t)(v & 1u);
            } else if (offset >= 0x114 && offset < 0x114 + 8 * 4) {
                unsigned idx = (offset - 0x114) / 4;
                dev.stagingCmd.wdata[idx] = v;
            } else if (offset == 0x134) {
                // Doorbell. Step 8: only accept if cmdQueue has space.
                // The op that gets queued is assigned an opIdx equal
                // to (cpuOpCount + queueLength), i.e. the index it
                // will have when committed.
                if (dev.cmdQueue.size() < dev.cmdQueueDepth) {
                    OramDevice::CmdEntry e = dev.stagingCmd;
                    e.phase = OramDevice::CmdEntry::Phase::PENDING;
                    e.opIdx = dev.cpuOpCount + dev.cmdQueue.size();
                    e.rdata_valid = false;
                    memset(e.rdata, 0, sizeof(e.rdata));
                    e.fetchTick = curTick();
                    e.dispatchTick = 0;
                    e.rtlDoneTick = 0;
                    e.commitTick = 0;
                    dev.cmdQueue.push_back(e);
                    DPRINTF(Oram, "CPU doorbell: %s slot=0x%x hwC=%u "
                            "lease=%u token=0x%x → queue[%zu] opIdx=%lu\n",
                            e.op ? "WR" : "RD",
                            e.slot_addr, e.hw_client, e.lease_id, e.token,
                            dev.cmdQueue.size() - 1, e.opIdx);
                } else {
                    // Queue full — silently drop. The CPU is expected
                    // to check queue depth (or use the ring path with
                    // its own backpressure) before ringing.
                    DPRINTF(Oram, "CPU doorbell: rejected, cmdQueue full "
                            "(%zu/%zu)\n",
                            dev.cmdQueue.size(), dev.cmdQueueDepth);
                }
            } else if (offset == 0xD8) {
                // OP_STATUS_CLEAR is now a no-op; legacy compat only.
                // The done bit at 0xD0 is computed from queue state
                // each read; nothing to clear here.
            } else if (offset == OramDevice::CMD_RING_DOORBELL_OFFSET) {
                // Step 9: ring doorbell. Wakes the ring fetcher to
                // re-read prod_idx and drain new entries. Implemented
                // in Rebuild B.
                dev.ringNeedsProdRead = true;
                DPRINTF(Oram, "CPU ring doorbell: ringNeedsProdRead=true\n");
            } else if (offset == OramDevice::GATE_RELEASE_OFFSET) {
                // Step 4: gate release. Only meaningful if
                // gated_start=true was passed in the Python config.
                // Releasing the gate kicks off the first tick, which
                // proceeds to INIT_SLOTS → write_init → IDLE → PROCESSING
                // exactly as a non-gated config would.
                //
                // Idempotent: writes after release are no-ops.
                if (v == 1 && dev.gatedStart && !dev.gateReleased) {
                    dev.gateReleased = true;
                    DPRINTF(Oram, "ORAM gate released by CPU MMIO; "
                            "starting tick at curTick=%lu\n", curTick());
                    dev.scheduleTick();
                } else if (v == 1 && !dev.gatedStart) {
                    // Config didn't request gating; warn-once-style
                    // diagnostic so misuse is visible.
                    DPRINTF(Oram, "ORAM gate release MMIO ignored: "
                            "gated_start=false in config\n");
                }
            }
            // else: unmapped writes ignored.
        }
    }
    pkt->makeAtomicResponse();
    return 0;
}

} // namespace gem5