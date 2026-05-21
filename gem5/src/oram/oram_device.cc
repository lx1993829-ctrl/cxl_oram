#include "oram/oram_device.hh"

#include <cstring>
#include <algorithm>
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

// Per-instance BRESP posmap tracking (parallel to bQueue, which is in the header).
// bQueueIsPosmap[i] tracks whether bQueue[i] is a posmap BRESP.
// posmapWriteSeqs tracks which PendingWriteBurst seqs are posmap RMW writes.
// These are indexed by OramDevice* since we can't add member variables.
static std::unordered_map<void*, std::deque<bool>> s_bQueueIsPosmap;
static std::unordered_map<void*, std::unordered_set<size_t>> s_posmapWriteSeqs;

// Per-instance accessors (called from member functions via 'this')
#define bQueueIsPosmap s_bQueueIsPosmap[this]
#define posmapWriteSeqs s_posmapWriteSeqs[this]

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
            d.cmdQueue.pop_front();
            d.cpuOpCount++;

            // Step 9: queue just opened up. If the ring has unfetched
            // entries waiting (ringConsIdx < ringProdIdxKnown) and now
            // there's room, kick off entry fetches. Without this, when
            // the queue fills to capacity during a steady stream of
            // commands, no further entries get fetched even after ops
            // complete and the queue drains — ORAM keeps polling
            // prod_idx but never advances cons_idx.
            //
            // We also re-issue a prod_idx read if doorbell came in
            // after we stopped fetching — covers the case where queue
            // was full when CPU rang.
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
            // Either the queue is empty or front opIdx doesn't match.
            // For depth=1 this should never happen. For depth>1, it
            // could mean an out-of-order completion (currently we don't
            // expect this since the RTL serializes ops, but be defensive).
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
    oramClkPeriod = p.oram_freq;
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
    cpuOpCount = 0;
    wrDbg.reset();
    memset(phaseCycles, 0, sizeof(phaseCycles));
    memset(opPhaseCycles, 0, sizeof(opPhaseCycles));
    prevFsmState = 0;
    stashHbmBeats = stashPcieBeats = 0;
    bucketHbmBeats = bucketPcieBeats = 0;
    if (numSlots > MAX_SLOTS) {
        warn("num_slots=%u > N=%d, clamping", numSlots, MAX_SLOTS);
        numSlots = MAX_SLOTS;
    }
    hbmSlotCount = (numSlots * localPct) / 100;
    if (hbmSlotCount == 0 && localPct > 0) hbmSlotCount = 1;
    inform("ORAM: %u slots, HBM=%u, host=%u",
           numSlots, hbmSlotCount, numSlots - hbmSlotCount);
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
    // Force all bucket remaps to bucket 8191 for step 3 eviction testing.
    // Bucket 8191 is a leaf frequently visited on ORAM paths.
    oram->dbg_prng_override = 0;
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
bool OramDevice::isHostSlot(uint32_t s) { return s >= hbmSlotCount; }

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
        oram->m_axi_arready = (hbmBlocked || stashReadPending ||
                               (currentOpIsPcie && pcieBlocked)) ? 0 : 1;
        // AW: always accept for stash (sel_stash=1) to avoid the AW+W desync
        // deadlock — stash_axi_master issues AW+W simultaneously and dropping
        // AW while accepting W causes a BRESP that never comes.
        // For bucket AWs (sel_stash=0), gate on pcieBlocked when the op
        // routes to PCIe. Bucket master only issues 8 AWs per DDR_WRITE
        // phase so this won't cause excessive stalls.
        {
            bool selStash = (oram->sel_stash_out != 0);
            if (selStash) {
                oram->m_axi_awready = 1;
            } else {
                oram->m_axi_awready = (currentOpIsPcie && pcieBlocked) ? 0 : 1;
            }
        }
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
            oram->m_axi_wready = (pcieBlocked || (int)wFifo.size() >= W_FIFO_DEPTH) ? 0 : 1;
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
        static uint8_t prevBurstBusy[16] = {0}, prevSelPosmap[16] = {0};
        uint8_t curBurstBusy = oram->st_burst_busy_out;
        uint8_t curSelPosmap = oram->sel_posmap_out;
        if (curBurstBusy != prevBurstBusy[instanceId] || curSelPosmap != prevSelPosmap[instanceId]) {
            inform("[cyc %lu] MUX-SWITCH: burst_busy %d->%d, sel_pm %d->%d, rQ=%d flushed "
                   "FSM=%d ht_st=%d sel_st=%d",
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
        for (int i = rb.flushedBeats; i < rb.totalBeats && rb.beatRecvd[i]; i++)
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
        static int stashMonCount[16] = {0}, ddrMonCount[16] = {0};
        static int stLoadMonCount[16] = {0}, stFlushMonCount[16] = {0};
        static int extractWrMonCount[16] = {0}, evictMonCount[16] = {0};

        // S_STASH_READ (12): reading from line_buf to client after HBM load
        if (fsm == 12 && stashMonCount[instanceId] < 50) {
            inform("[cyc %lu] R-MON STASH_READ: rQ=%d reorder=%d rvalid=%d "
                   "rready=%d sel_st=%d sel_pm=%d pendBursts=%d wFifo=%d",
                   oramCycle, rqBefore, reorderReady,
                   (int)oram->m_axi_rvalid, (int)oram->m_axi_rready,
                   (int)oram->sel_stash_out, (int)oram->sel_posmap_out,
                   (int)pendingReadBursts.size(), (int)wFifo.size());
            stashMonCount[instanceId]++;
        }

        // S_ST_LOAD (28): loading stash entry from HBM into line_buf
        if (fsm == 28 && stLoadMonCount[instanceId] < 50) {
            inform("[cyc %lu] STASH-LOAD: sel_st=%d arV=%d arR=%d rV=%d rR=%d "
                   "rQ=%d pendBursts=%d reorder=%d",
                   oramCycle,
                   (int)oram->sel_stash_out,
                   (int)oram->m_axi_arvalid, (int)oram->m_axi_arready,
                   (int)oram->m_axi_rvalid, (int)oram->m_axi_rready,
                   (int)rQueue.size(), (int)pendingReadBursts.size(),
                   reorderReady);
            stLoadMonCount[instanceId]++;
        }

        // S_ST_FLUSH (29): flushing line_buf to HBM
        if (fsm == 29 && stFlushMonCount[instanceId] < 50) {
            inform("[cyc %lu] STASH-FLUSH: sel_st=%d awV=%d awR=%d wV=%d wR=%d "
                   "bV=%d bR=%d wFifo=%d",
                   oramCycle,
                   (int)oram->sel_stash_out,
                   (int)oram->m_axi_awvalid, (int)oram->m_axi_awready,
                   (int)oram->m_axi_wvalid, (int)oram->m_axi_wready,
                   (int)oram->m_axi_bvalid, (int)oram->m_axi_bready,
                   (int)wFifo.size());
            stFlushMonCount[instanceId]++;
        }

        // S_EXTRACT_WR (10): writing client data to line_buf
        if (fsm == 10 && extractWrMonCount[instanceId] < 10) {
            inform("[cyc %lu] EXTRACT_WR: beat=%d wdata_valid=%d",
                   oramCycle, (int)oram->oram_beat_cnt,
                   (int)oram->client_wdata_valid);
            extractWrMonCount[instanceId]++;
        }

        // S_EVICT (14): finding stash entries to evict
        if (fsm == 14 && evictMonCount[instanceId] < 10) {
            inform("[cyc %lu] EVICT: sel_st=%d",
                   oramCycle, (int)oram->sel_stash_out);
            evictMonCount[instanceId]++;
        }

        // DDR_READ (2)
        if (fsm == 2 && ddrMonCount[instanceId] < 30) {
            inform("[cyc %lu] R-MON DDR_READ: rQ=%d reorder=%d rvalid=%d "
                   "rready=%d pendBursts=%d",
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
    if (curFsmState == 19) {  // DDR_WRITE
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
    if (curFsmState == 2) {  // DDR_READ
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
    prevFsmState = curFsmState;

    // --- Debug: track FSM transitions and SCAN results ---
    // Log every FSM state transition with full context
    if (curFsmState != prevFsmState && ctrlState == OramState::PROCESSING) {
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

    // Log token and request details at op dispatch
    if (oram->client_req && ctrlState == OramState::PROCESSING) {
        inform("[cyc %lu] OP-DISPATCH: req=%d op=%d addr=0x%lx "
               "token=0x%lx lease_id=%d wdata_valid=%d",
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
        bool isMetadataAddr = (s_araddr >= 0x10000000);
        bool pcie = (s_sel_stash || s_sel_posmap || isMetadataAddr)
                    ? false : currentOpIsPcie;

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

        size_t seq = nextBurstSeq++;
        pendingReadBursts.emplace_back(s_arid, numBeats, seq);
        
        // Data round-trip tracker: log read burst address
        Addr gem5ArAddr = pcie ? (hostBase + s_araddr) : (hbmBase + s_araddr);
        inform("[cyc %lu] AR-ISSUE: addr=0x%lx (axi=0x%lx) len=%d seq=%lu FSM=%d %s "
               "ht_st=%d ht_op=%d sel_st=%d burst_busy=%d",
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
        // Same metadata routing as AR — see comment above.
        bool isMetadataAddr = (axiAddr >= 0x10000000);
        bool pcie = (s_sel_stash || s_sel_posmap || isMetadataAddr)
                    ? false : currentOpIsPcie;

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
            Addr htRegionStart = hbmBase + 0x10500000;
            Addr htRegionEnd   = hbmBase + 0x10700800;
            if (gem5AwAddr >= htRegionStart && gem5AwAddr < htRegionEnd) {
                inform("[cyc %lu] HT_AW_WRITE: addr=0x%lx (axi=0x%lx) len=%d "
                       "FSM=%d ht_st=%d sel_st=%d sel_pm=%d burst_busy=%d isStash=%d",
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
            Addr htRegionStart = hbmBase + 0x10500000;
            Addr htRegionEnd   = hbmBase + 0x10700800;
            if (gem5Addr >= htRegionStart && gem5Addr < htRegionEnd) {
                inform("[cyc %lu] HT_W_DATA: addr=0x%lx beat=%d wstrb=0x%x "
                       "data[0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x "
                       "FSM=%d ht_st=%d sel_st=%d",
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
            // Send read directly (not through sendPkt which adds AxiSenderState
            // and pendingReadBursts tracking — this is NOT a bucket/stash read).

            auto rdReq = std::make_shared<Request>(gem5Addr, beatBytes, 0, reqId);
            PacketPtr rdPkt = new Packet(rdReq, MemCmd::ReadReq);
            uint8_t *rdBuf = new uint8_t[beatBytes]();
            rdPkt->dataDynamic(rdBuf);

            auto *ss = new PmRmwSenderState(gem5Addr, beatBytes, s_wstrb,
                                             wb.id, wb.beatsRecv, wb.len + 1,
                                             wb.writeSeq, wb.isPcie);
            memcpy(ss->wdata, s_wdata, AXI_DATA_BYTES);
            rdPkt->pushSenderState(ss);

            // Send directly — bypass sendPkt to avoid AxiSenderState/pendingReadBursts
            if (hbmBlocked) {
                // Already blocked — just queue for retry, don't call sendTimingReq
                hbmRetryQueue.push_back(rdPkt);
            } else if (!hbmPort.sendTimingReq(rdPkt)) {
                // HBM busy — queue for retry
                hbmBlocked = true;
                hbmRetryQueue.push_back(rdPkt);
            }

            wb.beatsRecv++;
            if (s_wlast) {
                // Create PendingWriteBurst so the RMW write's response
                // can match and generate a BRESP for the RTL.
                // The RMW write uses wb.writeSeq as its burstSeq.
                PendingWriteBurst pwb;
                pwb.id = wb.id;
                pwb.seq = wb.writeSeq;
                pwb.totalBeats = 1;  // single-beat pos_map write
                pwb.isHbm = !wb.isPcie;
                pwb.isStash = false;
                posmapWriteSeqs.insert(pwb.seq);  // track for BRESP matching
                // Check earlyWriteResps: the RMW write's response may have
                // arrived before this PendingWriteBurst was created.
                auto earlyIt = earlyWriteResps.find(wb.writeSeq);
                if (earlyIt != earlyWriteResps.end()) {
                    pwb.responsesRecv = earlyIt->second;
                    earlyWriteResps.erase(earlyIt);
                    inform("PM-EARLY-PICKUP[%s] inst=%u cyc=%lu: posmap PWB "
                           "seq=%lu picked up %d early resp → immediate BRESP",
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

            pkt->pushSenderState(
                new AxiSenderState(wb.id, wb.beatsRecv, wb.len + 1,
                                   true, wb.writeSeq));

            // Queue to write FIFO instead of sending directly.
            // drainWriteFifo() sends 1 per tick to HBM.
            wFifo.push_back({pkt, wb.isPcie});
            if (curFsmState == 19) {
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
                   (int)oram->oram_busy, (int)oram->client_done,
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
        static uint64_t muxConflictCount[16] = {0};
        uint8_t fsm = oram->dbg_oram_state;
        bool bucketPhase = (fsm == 2 || fsm == 19);  // S_DDR_READ or S_DDR_WRITE
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

    // Drain queued read requests to gem5
    drainPendingSends();

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

    // Deadlock E diagnostic: log every tick during WRITE_INIT where
    // client_done has ANY bit set, or every 500 cycles for idle state.
    if (ctrlState == OramState::WRITE_INIT) {
        uint8_t cd = oram->client_done;
        if (cd || (oramCycle % 500 == 0)) {
            inform("WRITE_INIT-DIAG[%s] cyc=%lu wrIdx=%d activeHw=%u "
                   "doneMask=0x%x client_done=0x%x oram_busy=%d "
                   "access_viol=0x%x fsm=%d",
                   name(), oramCycle, writeInitIdx, activeHwClient,
                   (int)doneMask, (int)cd, (int)oram->oram_busy,
                   (int)oram->access_violation,
                   (int)oram->dbg_oram_state);
        }
    }

    // Deadlock E fix: during WRITE_INIT only one op is ever in flight.
    // When writeInitIdx crosses the slotsPerClient boundary (e.g. slot 8
    // with num_logical_clients=2), activeHwClient switches from 0 to 1,
    // making doneMask=0x2. If the RTL's arbiter timing causes client_done
    // to pulse on bit 0 instead of bit 1, completeOp() never fires and
    // writeInitIdx never advances. Accept any client_done bit during
    // WRITE_INIT since there's no ambiguity with a single op in flight.
    uint8_t completionMask = (ctrlState == OramState::WRITE_INIT)
                             ? ((1u << numLogicalClients) - 1) : doneMask;

    if ((ctrlState == OramState::PROCESSING ||
         ctrlState == OramState::WRITE_INIT) &&
        (oram->client_done & completionMask))
        completeOp();

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
                currentOpIsWrite    = (op.op == 1);
                currentOpAddr       = op.slot_addr;
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
                op.rdata_valid = false;

                ctrlState = OramState::PROCESSING;
                opStartCycle = oramCycle;
                memset(opPhaseCycles, 0, sizeof(opPhaseCycles));

                DPRINTF(Oram, "[%lu] CPU-op dispatch: %s slot=0x%x "
                        "hwC=%u lease=%u opIdx=%lu\n",
                        oramCycle, currentOpIsWrite ? "WR" : "RD",
                        op.slot_addr, op.hw_client, op.lease_id,
                        op.opIdx);
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
        inform("HEARTBEAT[%s] cyc=%lu ctrlState=%d fsm=%d "
               "initIdx=%d initPh=%d grantPh=%d wrInitIdx=%d "
               "opsComp=%u/%u oram_busy=%d client_req=%d client_done=%d "
               "init_mode=%d pm_busy=%d mgmt_req=%d mgmt_ack=%d "
               "PM: awV=%d wV=%d arV=%d bR=%d state=%d "
               "MUX: sel_pm=%d sel_st=%d mux_awV=%d "
               "TOP: awV=%d awR=%d wV=%d wR=%d bV=%d",
               name(), oramCycle, (int)ctrlState, (int)oram->dbg_oram_state,
               initSlotIdx, initPhase, grantPhase, writeInitIdx,
               opsCompleted, numOps,
               (int)oram->oram_busy, (int)oram->client_req,
               (int)(oram->client_done),
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
        warn("ORAM RTL: AES-GCM tag mismatch @ cycle %lu "
             "(expected on first access to uninitialized data)", oramCycle);
        tagMismatchWarned = true;
    }
    if (oram->access_violation)
        fatal("ORAM RTL: access violation (0x%x) @ cycle %lu",
              (int)oram->access_violation, oramCycle);
}

// =============================================================================
// Packet routing
// =============================================================================

bool OramDevice::sendPkt(PacketPtr pkt, bool isPcie)
{
    if (isPcie) {
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
//   0x1C: reserved (u32 = 0)
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
    v32 = 0;              memcpy(buf + 0x1C, &v32, 4);

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
                                           it->totalBeats, false, it->seq);
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
                                                     it->seq));
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

        // Read completed — merge masked bytes and issue timing write
        const uint8_t *rdData = pkt->getConstPtr<uint8_t>();
        uint8_t *mergedBuf = new uint8_t[pmRmw->writeSize];
        memcpy(mergedBuf, rdData, pmRmw->writeSize);
        for (int b = 0; b < std::min(pmRmw->writeSize, (int)AXI_DATA_BYTES); b++) {
            if (pmRmw->wstrb & (1u << b))
                mergedBuf[b] = pmRmw->wdata[b];
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

        // Helper lambda to deliver one beat to pendingReadBursts
        auto deliverBeat = [&](size_t seq, int beatIdx,
                               const uint8_t *data, unsigned dataOff) {
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
                    rb.beatsRecv++;
                    return true;
                }
            }
            return false;
        };

        // First beat (always present)
        bool found = deliverBeat(ss->burstSeq, ss->beatIdx,
                                  pkt->getConstPtr<uint8_t>(), 0);

        // Debug: cross-check HT region reads with functional path
        {
            Addr pktAddr = pkt->getAddr();
            Addr htSlotStart = hbmBase + 0x10500000;
            Addr htSlotEnd   = hbmBase + 0x10500000 + 512 * AXI_DATA_BYTES; // 16KB
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
                    inform("[cyc %lu] HT_XCHECK MISMATCH at 0x%lx: "
                           "timing[0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x "
                           "func[0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x",
                           oramCycle, pktAddr,
                           pkt->getConstPtr<uint8_t>()[7], pkt->getConstPtr<uint8_t>()[6],
                           pkt->getConstPtr<uint8_t>()[5], pkt->getConstPtr<uint8_t>()[4],
                           pkt->getConstPtr<uint8_t>()[3], pkt->getConstPtr<uint8_t>()[2],
                           pkt->getConstPtr<uint8_t>()[1], pkt->getConstPtr<uint8_t>()[0],
                           fBuf[7], fBuf[6], fBuf[5], fBuf[4],
                           fBuf[3], fBuf[2], fBuf[1], fBuf[0]);
                } else {
                    inform("[cyc %lu] HT_XCHECK OK at 0x%lx: [0..7]=0x%02x%02x%02x%02x%02x%02x%02x%02x",
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
                                       pkt->getConstPtr<uint8_t>(), 32);
            found = found || found2;
        }

        if (!found) {
            DPRINTF(Oram, "[%lu] READ-RESP-UNMATCHED: seq=%lu beat=%d\n",
                    oramCycle, ss->burstSeq, ss->beatIdx);
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
                warn("EARLY-PM-BRESP[%s] inst=%u cyc=%lu: posmap write resp "
                     "seq=%lu arrived before PendingWriteBurst created "
                     "(earlyCount=%d, FSM=%d, pm_busy=%d)",
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

    if (!pendingReadBursts.empty()) {
        auto &rb = pendingReadBursts.front();
        if (rb.flushedBeats < rb.totalBeats &&
            rb.beatRecvd[rb.flushedBeats]) {
            rQueue.push_back(rb.beats[rb.flushedBeats]);
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
    } else {
        oram->m_axi_rvalid = 0; oram->m_axi_rlast = 0;
    }
}

void OramDevice::driveAxiB()
{
    // Pop the entry that was consumed last cycle (tracked by bQueuePresentIdx)
    static std::unordered_map<void*, int> s_bQueuePresentIdx;
    if (s_bQueuePresentIdx.find(this) == s_bQueuePresentIdx.end())
        s_bQueuePresentIdx[this] = -1;
    int &presentIdx = s_bQueuePresentIdx[this];

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
        // Fill entire bucket region with dummy pattern via sendFunctional.
        // Write one full bucket (32KB) per call to minimize overhead.
        // Only fill buckets we'll actually use (numSlots / SLOTS_PER_BUCKET),
        // plus a margin. No need to fill 256 MB for 4 buckets.
        int usedBuckets = ((int)numSlots + SLOTS_PER_BUCKET - 1) / SLOTS_PER_BUCKET;
        int totalBuckets = std::min(usedBuckets + 8, (int)MAX_BUCKETS); // small margin
        Addr totalBytes = (Addr)totalBuckets * BUCKET_BYTES;

        inform("[cyc %lu] DDR_FILL: writing %d buckets (%lu bytes, %lu MB) to HBM "
               "starting at 0x%lx",
               oramCycle, totalBuckets, totalBytes, totalBytes / (1024*1024),
               hbmBase);

        for (int b = 0; b < totalBuckets; b++) {
            Addr bucketBase = hbmBase + (Addr)b * BUCKET_BYTES;
            int beatsPerBucket = BUCKET_BYTES / AXI_DATA_BYTES;
            for (int beat = 0; beat < beatsPerBucket; beat++) {
                Addr addr = bucketBase + (Addr)beat * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                // Fill with pattern: global beat index
                uint32_t pattern = (uint32_t)(b * beatsPerBucket + beat);
                for (int off = 0; off < AXI_DATA_BYTES; off += 4)
                    memcpy(buf + off, &pattern, std::min(4, AXI_DATA_BYTES - off));
                pkt->dataDynamic(buf);
                hbmPort.sendFunctional(pkt);
                delete pkt;
            }
        }

        inform("[cyc %lu] DDR_FILL: done, %d buckets written", oramCycle, totalBuckets);

        // --- Zero-fill stash data region ---
        {
            Addr stashBase = hbmBase + stashOffset;
            int stashEntries = 1024;  // ORAM_STASH_DEPTH from RTL params
            int beatsPerEntry = 128;  // STASH_BEATS_PER_ENTRY = 128 (4KB / 32B)
            inform("[cyc %lu] STASH_FILL: zeroing %d stash entries (%d KB)",
                   oramCycle, stashEntries, stashEntries * 4);
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
        // Order: HEAD, NEXT, then SLOT LAST.
        // SLOT must be last because sendAtomic through Ramulator2 may
        // have side effects on nearby addresses. Writing SLOT last
        // guarantees SLOT contains zeros regardless of what HEAD/NEXT did.
        {
            // BKT head array: 8191 buckets / 16 per beat = 512 beats
            Addr bktHeadBase = hbmBase + 0x10600000;
            int headBeats = 512;
            inform("[cyc %lu] HT_FILL: init BKT head array (%d beats)",
                   oramCycle, headBeats);
            for (int i = 0; i < headBeats; i++) {
                Addr addr = bktHeadBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0xFF, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendAtomic(pkt);
                delete pkt;
            }

            // BKT next array: 1024 entries / 16 per beat = 64 beats
            Addr bktNextBase = hbmBase + 0x10700000;
            int nextBeats = 64;
            inform("[cyc %lu] HT_FILL: init BKT next array (%d beats)",
                   oramCycle, nextBeats);
            for (int i = 0; i < nextBeats; i++) {
                Addr addr = bktNextBase + (Addr)i * AXI_DATA_BYTES;
                auto req = std::make_shared<Request>(addr, AXI_DATA_BYTES, 0, reqId);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                uint8_t *buf = new uint8_t[AXI_DATA_BYTES];
                memset(buf, 0xFF, AXI_DATA_BYTES);
                pkt->dataDynamic(buf);
                hbmPort.sendAtomic(pkt);
                delete pkt;
            }

            // SLOT hash table LAST: 2048 entries / 4 per beat = 512 beats
            Addr slotHtBase = hbmBase + 0x10500000;
            int slotBeats = 512;
            inform("[cyc %lu] HT_FILL: zeroing SLOT hash table (%d beats, %d KB)",
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

            inform("[cyc %lu] HT_FILL: all hash tables initialized", oramCycle);
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
    uint32_t bucketIdx = initSlotIdx / SLOTS_PER_BUCKET;
    int posInBucket = initSlotIdx % SLOTS_PER_BUCKET;
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
        static uint8_t pmBeatBuf[16][AXI_DATA_BYTES];
        static uint32_t pmCurrentBeat[16];
        // Initialize on first use
        static bool pmInited[16] = {false};
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
        // slot_list is Z * SLOT_ID_W = 8 * 12 = 96 bits, packed into
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
        static uint32_t accum_slot_list[16][4];
        static uint32_t prev_bucket[16];
        static bool bmInited[16] = {false};
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

        oram->init_bm_wr_bucket = bucketIdx;
        // Copy slot_list — use min of src/dst size to prevent overflow
        // if Verilator and our accum have different sizes
        size_t copyLen = std::min(sizeof(accum_slot_list[instanceId]),
                                  sizeof(oram->init_bm_wr_slot_list));
        memcpy(&oram->init_bm_wr_slot_list[0], accum_slot_list[instanceId], copyLen);
        oram->init_bm_wr_fill = posInBucket + 1;
        oram->init_bm_wr_en = 1;

        if (initSlotIdx < 3 || initSlotIdx == (int)numSlots - 1
            || posInBucket == SLOTS_PER_BUCKET - 1)
            inform("[cyc %lu] INIT bm: slot=%d bkt=%u pos=%d slotId=%u "
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
                writeInitIdx = 0;
                ctrlState = OramState::WRITE_INIT;
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

    // Progress tracking — always print so user sees forward movement
    inform("ORAM[%u] op %u/%u done (%s, %lu cyc, %.1f us elapsed)",
           instanceId, opsCompleted, numOps,
           currentOpIsPcie ? "PCIe" : "HBM", cyc,
           curTick() / 1e6);

    // Round-trip data verification
    if (!currentOpIsWrite && rdataShadowValid[activeHwClient]) {
        inform("VERIFY-READ op=%u addr=0x%lx rdata[0..3]=%08x %08x %08x %08x",
               opsCompleted, currentOpAddr,
               rdataShadow[activeHwClient][0], rdataShadow[activeHwClient][1],
               rdataShadow[activeHwClient][2], rdataShadow[activeHwClient][3]);
    } else if (!currentOpIsWrite) {
        inform("VERIFY-READ op=%u addr=0x%lx NO RDATA SHADOW", 
               opsCompleted, currentOpAddr);
    }
    if (currentOpIsWrite) {
        uint32_t seed = opsCompleted - 1;  // opsCompleted was just incremented
        inform("VERIFY-WRITE op=%u addr=0x%lx beat0_wdata[0..3]=%08x %08x %08x %08x",
               opsCompleted, currentOpAddr,
               seed ^ 0 ^ 0, seed ^ 0 ^ 1, seed ^ 0 ^ 2, seed ^ 0 ^ 3);
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
    inform("  Op %u phase breakdown (%lu total cyc):", opsCompleted, cyc);
    for (int i = 0; i < NUM_FSM_STATES; i++) {
        if (opPhaseCycles[i] > 0) {
            inform("    %-20s %6lu cyc (%4.1f%%)",
                   stateNames[i], opPhaseCycles[i],
                   100.0 * opPhaseCycles[i] / cyc);
        }
    }

    // DDR_WRITE debug breakdown
    if (wrDbg.totalCycles > 0) {
        inform("  DDR_WRITE W-channel debug (%lu cycles):", wrDbg.totalCycles);
        inform("    W handshakes:  %lu (RTL wvalid && wready)",
               wrDbg.wHandshakes);
        inform("    W stalls:      %lu (RTL wvalid && !wready)",
               wrDbg.wStalls);
        inform("    W idle:        %lu (RTL !wvalid, sub-burst gaps)",
               wrDbg.wIdle);
        inform("    gem5 accepted: %lu (sendTimingReq true)",
               wrDbg.sendAccepted);
        inform("    gem5 rejected: %lu (sendTimingReq false, queued)",
               wrDbg.sendRejected);
        inform("    gem5 retries:  %lu (sent via retry queue)",
               wrDbg.retrySent);
        inform("    AW handshakes: %lu", wrDbg.awHandshakes);
        inform("    BRESPs avail:  %lu (bvalid asserted)", wrDbg.brespAvail);
        inform("    BRESPs recv:   %lu (bvalid && bready consumed)",
               wrDbg.brespRecv);
        inform("    BRESPs noMatch:%lu (bQueue non-empty but wrong type)",
               wrDbg.brespNoMatch);
        inform("    maxBqDepth:    %lu", wrDbg.maxBqDepth);
        inform("    W timing:      first=%lu last=%lu span=%lu cyc",
               wrDbg.firstWcycle, wrDbg.lastWcycle,
               wrDbg.lastWcycle - wrDbg.firstWcycle);
        inform("    B timing:      first=%lu last=%lu span=%lu cyc",
               wrDbg.firstBcycle, wrDbg.lastBcycle,
               wrDbg.firstBcycle > 0 ? wrDbg.lastBcycle - wrDbg.firstBcycle : 0UL);
        inform("    W→B latency:   %lu cyc (first BRESP - first W)",
               wrDbg.firstBcycle > wrDbg.firstWcycle ?
               wrDbg.firstBcycle - wrDbg.firstWcycle : 0UL);
        inform("    Effective:     %.3f cyc/beat (%lu beats in %lu cyc)",
               wrDbg.wHandshakes > 0 ?
               (double)wrDbg.totalCycles / wrDbg.wHandshakes : 0.0,
               wrDbg.wHandshakes, wrDbg.totalCycles);
    }
    wrDbg.reset();

    // DDR_READ debug breakdown
    if (rdDbg.totalCycles > 0) {
        inform("  DDR_READ R-channel debug (%lu cycles):", rdDbg.totalCycles);
        inform("    R handshakes:  %lu (RTL rvalid && rready)",
               rdDbg.rHandshakes);
        inform("    R stalls:      %lu (RTL rvalid && !rready)",
               rdDbg.rStalls);
        inform("    R idle:        %lu (RTL !rvalid, waiting for data)",
               rdDbg.rIdle);
        inform("    AR handshakes: %lu", rdDbg.arHandshakes);
        inform("    gem5 accepted: %lu (read sendPkt true)",
               rdDbg.sendAccepted);
        inform("    gem5 rejected: %lu (read sendPkt false, queued)",
               rdDbg.sendRejected);
        inform("    gem5 retries:  %lu (sent via retry queue)",
               rdDbg.retrySent);
        inform("    mem responses: %lu (handleMemResp reads)",
               rdDbg.memResps);
        inform("    maxRqDepth:    %lu", rdDbg.maxRqDepth);
        inform("    maxPendReads:  %lu", rdDbg.maxPendReads);
        inform("    R timing:      first=%lu last=%lu span=%lu cyc",
               rdDbg.firstRcycle, rdDbg.lastRcycle,
               rdDbg.firstRcycle > 0 ? rdDbg.lastRcycle - rdDbg.firstRcycle : 0UL);
        inform("    Effective:     %.3f cyc/beat (%lu beats in %lu cyc)",
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

        if (!currentOpIsWrite) {
            if (rdataShadowValid[activeHwClient]) {
                for (int i = 0; i < 8; i++)
                    op.rdata[i] = rdataShadow[activeHwClient][i];
                op.rdata_valid = true;
                rdataShadowValid[activeHwClient] = false;  // consume
            } else {
                // Shadow wasn't populated — defensive fallback.
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

    inform("=== ORAM Results ===");
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
    if (stashPcieBeats > 0)
        warn("  STASH ROUTING ERROR: %lu stash beats went to PCIe!", stashPcieBeats);
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
