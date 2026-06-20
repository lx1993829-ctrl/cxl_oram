/*
 * cxl_model.hh — FLIT-level pipelined CXL.mem model for gem5 v25.1
 *
 * Models CXL Type-3 (memory expansion) device path with
 * credit flow control and slot-amortized wire accounting:
 *   - CXL.mem FLIT framing (no TLP headers, no DLLP ACK)
 *   - Same Gen5 x16 physical layer as PCIe (shared PHY)
 *   - FLIT-level credit flow control (per-port pools)
 *   - No AXI-to-PCIe bridge pipeline (CXL endpoint handles directly)
 *   - Same RC traversal latency as PCIe (same PHY + CPU interconnect,
 *     CXL Home Agent coherency offsets FLIT decode savings)
 *   - Burst-window RC optimization (same as PCIe model)
 *
 * Key differences from PCIeModel:
 *   - No TLP header construction (FLITs use 2-6B headers vs 12-16B)
 *   - No LCRC/framing/ECRC per-packet overhead
 *   - Single FLIT credit pool per port (simpler than PCIe PH/PD/NPH/CPLH)
 *   - No DLLP ACK delay
 *   - No bridge pipeline delay
 *   - RC latency default 150ns (same as PCIe), includes Home Agent + interconnect
 */

#ifndef __MEM_CXL_CXL_MODEL_HH__
#define __MEM_CXL_CXL_MODEL_HH__

#include <deque>
#include <unordered_map>
#include <vector>

#include "mem/port.hh"
#include "params/CxlModel.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

class CxlModel : public ClockedObject
{
  public:
    PARAMS(CxlModel);
    CxlModel(const Params &p);

    Port &getPort(const std::string &if_name, PortID idx) override;
    void init() override;

  private:

    // ================================================================
    //  Link Parameters (same physical layer as PCIe Gen5)
    // ================================================================
    struct LinkParams {
        unsigned gen, lanes;
        double   encodingEff;
        uint64_t effectiveBwBytesPerSec;
        Tick     byteSerializationDelay;
    };
    LinkParams linkParams;
    void computeLinkParams();

    // ================================================================
    //  CXL FLIT Parameters
    // ================================================================
    // CXL 3.0 uses fixed 256-byte FLITs. Each FLIT contains:
    //   - 6-byte header (slot type, address, tag)
    //   - Payload (0-234 bytes of data)
    //   - 16-byte CRC
    // A FLIT is always 256 bytes on the wire regardless of payload.
    // Multiple requests from one AXI burst are packed into as few
    // FLITs as possible (burst FLIT packing in handleDeviceRequest).
    static const unsigned FLIT_SIZE = 256;         // fixed wire size
    static const unsigned FLIT_HEADER_BYTES = 6;
    static const unsigned FLIT_CRC_BYTES = 16;
    static const unsigned FLIT_MAX_PAYLOAD = 234;  // 256 - 6 - 16

    unsigned maxPayload;  // similar concept to MPS
    unsigned maxReadSize; // similar to MRRS
    unsigned cplCombineBytes;  // Bug 9: completion combining boundary

    // ================================================================
    //  Latency Parameters
    // ================================================================
    Tick rcLatency;       // Full one-way RC traversal (default 150ns, same as PCIe)
    Tick endpointDelay;   // CXL endpoint controller processing
    Tick hostInjectInterval; // min interval between host xbar sends
    Tick flitCoalesceWindow; // coalescing timer for partial FLITs
    Tick rcThroughputDelay; // Per-response RC pipeline throughput cost (warm)
    Tick fpgaClockPeriod; // Bug #10 fix: FPGA bridge clock period (from Param)

    // Review item #1 (symmetric to PCIe RRB): Home Agent completion
    // buffer depth. Holds returning S2M DRS FLITs pending AXI-order
    // retirement. Separate from the tag pool.
    // Phase A.1: per-port. Each AXI master has its own bridge-side
    // completion buffer; one master cannot stall another's reorder
    // capacity. completionBufferDepth still describes per-port size.
    unsigned completionBufferDepth;
    std::vector<unsigned> completionBufferOccupied;

    // Review item #3 (symmetric to PCIe core-clock gap): per-FLIT min
    // emit gap. Real hard block can only emit one FLIT per core cycle.
    Tick cxlCoreClock;
    Tick lastFlitEmitTick;
    // Bug 1 (per-port CDC): downstream endpoint serialization is
    // per-device-port. Each ORAM instance has its own bridge FIFO
    // crossing from host clock to 300MHz FPGA fabric, so CplDs for
    // different instances do not head-of-line-block each other.
    std::vector<Tick> endpointBusyUntil;
    Tick rcDownstreamBusyUntil;  // Shared RC pipeline throughput (downstream)
    Tick rcUpstreamBusyUntil;    // Shared RC pipeline throughput (upstream)

    // computeRcDelay removed — was dead code (never called). Both
    // upstream and downstream paths compute RC delay inline using
    // rcUpstreamBusyUntil / rcDownstreamBusyUntil.

    // ================================================================
    //  FLIT Type
    // ================================================================
    enum class FlitType { ReadReq, WriteReq, ReadCompletion, WriteCompletion };

    // ================================================================
    //  Tag Tracking (CXL still needs outstanding request tracking)
    // ================================================================
    unsigned maxTags;
    // Bug 9: BeatInfo tracks (addr, host-pkt) for per-beat delivery
    // in combined completions. Declared at class scope so both
    // OutstandingRead and FlitEntry can reference it.
    struct BeatInfo { Addr beatAddr; PacketPtr hostPkt; };

    struct OutstandingRead {
        PacketPtr pkt;            // first device packet in the group
        // Opt 1: all coalesced device packets. On completion delivery,
        // each one is responded to individually so each ORAM beat gets
        // its own sendTimingResp. Empty-singleton means no coalescing.
        std::vector<PacketPtr> allPkts;
        Addr addr;                // starting address of the group
        unsigned totalBytes;      // coalesced group size
        unsigned completedBytes = 0; // Opt 1: bytes responded so far
        Tick issueTick;
        int srcPortIdx = 0;       // Opt 1: which device port owns this
        // Bug 9 (completion combining): accumulate DDR5 responses until
        // we have a full FLIT's worth of data (or the last beat of the
        // group), then emit ONE cpl FLIT carrying all beats.
        unsigned pendingCplBytes = 0;
        unsigned emittedBytes = 0;  // bytes emitted in cpl FLITs so far
        std::vector<BeatInfo> pendingCplBeats;
    };
    std::unordered_map<uint16_t, OutstandingRead> outstandingReads;
    std::deque<uint16_t> freeTags;
    uint16_t allocateTag();
    void releaseTag(uint16_t tag);
    bool hasFreeTags() const { return !freeTags.empty(); }

    // ================================================================
    //  Link Serialization
    // ================================================================
    struct FlitEntry {
        Tick readyTick = 0;
        unsigned wireBytes = 0;
        FlitType type = FlitType::ReadReq;
        uint16_t tag = 0;
        Addr addr = 0;
        unsigned payloadBytes = 0;
        PacketPtr origPkt = nullptr;
        Tick issueTick = 0;
        bool isLast = false;
        int srcPortIdx = 0;  // which device port to respond to
        // Opt 1 reads: host pkts are expanded lazily in processUpstreamQueue
        //              from outstandingReads[tag].allPkts (keyed by tag).
        // Opt 2 writes: device-side write packets that get split back into
        //              individual 32B host pkts (with data copied) when the
        //              FLIT finishes upstream serialization.
        std::vector<PacketPtr> allWritePkts;
        // Bug 9 fix (completion combining): for ReadCompletion FLITs that
        // carry multiple beats amortized into one FLIT, we track the list
        // of (beat_addr, beat_host_pkt) pairs so processDownstreamQueue
        // can deliver each beat individually to the matching device pkt.
        // For single-beat completions, this is empty and we fall back to
        // (addr, origPkt).
        std::vector<BeatInfo> combinedBeats;
    };

    // Phase A.1: per-port upstream and downstream FLIT queues. The
    // wire (upstreamBusyUntil/downstreamBusyUntil), the RC pipeline
    // burst-window state, and the hard-block FLIT emit gate stay
    // shared — those model genuinely shared physical resources. Only
    // the per-port BUFFERS holding queued FLITs are split, so a
    // stalled FLIT on one port (waiting on its own credits / write
    // budget / endpoint serializer) does not head-of-line-block FLITs
    // from another port. Indexed by srcPortIdx.
    std::vector<std::deque<FlitEntry>> upstreamQueue;
    Tick upstreamBusyUntil;

    std::vector<std::deque<FlitEntry>> downstreamQueue;
    Tick downstreamBusyUntil;
    Tick lastDownstreamFlitEmit;  // Bug #5: core-clock gate for downstream

    Tick serializationDelay(unsigned wireBytes) const;
    void enqueueUpstream(FlitEntry &flit);
    void enqueueDownstream(FlitEntry &flit, Tick earliestStart = 0);

    // Phase A.1: rotating-start FCFS cursor for dispatch across per-port
    // queues. Each call drains all ready FLITs from each port before
    // advancing. Cursor advances by one each call entry so port-0
    // doesn't permanently win tie-breaks.
    unsigned nextUpstreamPort = 0;
    unsigned nextDownstreamPort = 0;

    // ================================================================
    //  Ports
    // ================================================================
    class DeviceSidePort : public ResponsePort
    {
      public:
        DeviceSidePort(const std::string &name, CxlModel &owner, int idx);
      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        void recvFunctional(PacketPtr pkt) override;
        AddrRangeList getAddrRanges() const override;
      private:
        CxlModel &owner;
        int portIdx;
        bool needRetry;
        friend class CxlModel;
    };

    class HostSidePort : public RequestPort
    {
      public:
        HostSidePort(const std::string &name, CxlModel &owner, int idx);
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
      private:
        CxlModel &owner;
        int portIdx;
        bool needRetry;
        friend class CxlModel;
    };

    std::vector<DeviceSidePort *> devicePorts;
    // Bug 3 (multi host port): vector of host ports to parallelize
    // DDR5 injection. Routing by (addr >> 6) % numPorts spreads
    // 64B-interleaved DDR5 traffic across host xbar inputs, removing
    // single-port head-of-line blocking.
    std::vector<HostSidePort *> hostPorts;

    // ================================================================
    //  SenderState
    // ================================================================
    struct SenderState : public Packet::SenderState
    {
        uint16_t tag = 0;
        Addr origAddr = 0;
        unsigned origSize = 0;
        Tick issueTick = 0;
        bool isPostedWrite = false;
        PacketPtr origDevicePkt = nullptr;
        int srcPortIdx = 0;  // which device port this request came from
    };

    // ================================================================
    //  Pipeline
    // ================================================================
    bool handleDeviceRequest(PacketPtr pkt, int srcPort = 0);

    // Opts 1+2: device request buffers + coalescing drain path.
    // handleDeviceRequest accepts each 32B beat into a buffer and
    // schedules drainDeviceRequests for the same tick. drainDeviceRequests
    // scans the buffer front for sequential 32B reads/writes from the
    // same port and coalesces them into one group: 1 tag per group for
    // reads (up to maxReadSize), 1 FLIT per group for writes (up to
    // maxPayload). This mirrors the PCIe model's buffer-and-defer design.
    struct BufferedDevReq { PacketPtr pkt; int srcPort; };
    std::vector<std::deque<BufferedDevReq>> deviceReadBuffers;
    std::vector<std::deque<BufferedDevReq>> deviceWriteBuffers;
    void drainDeviceRequests();
    bool processBufferedRead(PacketPtr pkt, int srcPort);
    bool processBufferedWrite(PacketPtr pkt, int srcPort);
    EventFunctionWrapper drainDeviceEvent;
    std::vector<Tick> lastWriteBufferTicks;  // per-port coalesce timeout
    std::vector<Tick> lastReadBufferTicks;   // per-port coalesce timeout
    Tick lastStuckWarn = 0;        // Review fix #9: was static local — caused cross-instance pollution

    void processUpstreamQueue();
    EventFunctionWrapper upstreamEvent;
    bool handleHostResponse(PacketPtr pkt);
    void processDownstreamQueue();
    EventFunctionWrapper downstreamEvent;
    void retryDeviceSend();
    EventFunctionWrapper deviceRetryEvent;
    void retryHostSend();
    EventFunctionWrapper hostRetryEvent;

    // ================================================================
    //  Burst-Window RC Optimization (same as PCIe)
    // ================================================================
    Tick lastUpstreamRcTick;    // DEAD — kept for ABI compat, never read
    Tick lastDownstreamRcTick;  // DEAD — kept for ABI compat, never read
    Tick burstWindowTicks;

    // ================================================================
    //  Phase B/A.1 Diagnostic Instrumentation
    //
    //  Per-port stuck detection: track when each port last consumed a
    //  packet from one of its queues. If a port has work pending but
    //  hasn't drained anything for >5ms, fire a comprehensive dump
    //  showing exactly which queue is stuck and why. Adds visibility
    //  for cases like the Phase B 8-byte CPU read coalesce bug where
    //  the GLOBAL lastProgressTick was advancing (port 0 still busy)
    //  but a specific port's queues were frozen.
    // ================================================================
    std::vector<Tick> perPortLastDrainTick;
    Tick lastPortStuckCheckTick = 0;
    void dumpPortState(unsigned p, const char *where);
    void checkPortStalls();

    struct ResponseEntry {
        PacketPtr pkt;
        int portIdx;
        Tick readyTick;   // earliest tick this response may be delivered
    };
    // Phase A.1: per-port response queues. Each device port can be
    // independently blocked on sendTimingResp; one stalled port must
    // not delay another port's responses. Cursor advances each call
    // entry to round-robin starting port across calls.
    std::vector<std::deque<ResponseEntry>> responseQueue;
    unsigned nextResponsePort = 0;
    void trySendResponses();
    void retryStarvedPorts();
    EventFunctionWrapper responseEvent;

    // ================================================================
    //  Backpressure + outstanding tracking
    // ================================================================
    unsigned maxOutstanding;        // 0 = disabled, from params (reads)
    unsigned maxOutstandingWrites;  // separate write buffer depth
    // Phase A.1: per-port outstanding-write counters. Mirrors the
    // existing perPortReadsOut pattern so one port cannot consume
    // every write slot and starve others. Each port can have up to
    // maxOutstandingWrites in flight (pool per port — realistic for
    // separate AXI masters with their own bridge FIFOs).
    std::vector<unsigned> outstandingWrites;
    std::vector<unsigned> outstandingReadBeats;  // per-beat read tracking (mirrors outstandingWrites)
    unsigned maxOutstandingReads;  // shared acceptance-time limit for reads
    // Bug 2 (per-port outstanding): per-device-port outstanding read
    // counters. Each instance gets its own full quota of maxOutstanding
    // rather than sharing a single global pool that starves at N > 1.
    std::vector<unsigned> perPortReadsOut;
    bool pendingDeviceRetry;   // deferred retry after BRESP delivery
    unsigned nextRetryPort;    // round-robin index for fair retry

    // Write commit tracking: compare posted BRESP (RTL visible) vs DDR5 commit
    Tick firstWriteAccept = 0;
    Tick lastBrespDelivered = 0;
    Tick lastDdr5WriteCommit = 0;
    unsigned writeCommitCount = 0;
    unsigned writeBrespCount = 0;

    // ================================================================
    //  FLIT Credit Flow Control
    //  Real CXL uses credit-based flow control: host RC grants N FLIT
    //  credits to the endpoint. Each upstream FLIT consumes one credit.
    //  Credit returned when host RC accepts the FLIT (round-trip delay).
    //
    //  Shared credit pool: all device-side ports share one credit budget
    //  at flitCredits[0] / flitCreditsMax[0]. The RC advertises one
    //  credit set per VC, shared by all functions behind the endpoint.
    //  Vectors are sized to devicePorts.size() for indexing convenience
    //  but only index 0 is used at runtime.
    //  Default 128 credits matches Synopsys/Cadence CXL 3.0 endpoint IP.
    // ================================================================
    std::vector<unsigned> flitCredits;      // shared pool at index [0]
    std::vector<unsigned> flitCreditsMax;   // shared max at index [0]
    Tick flitCreditReturnDelay; // time for credit to return from host RC

    struct DeferredFlitCredit {
        Tick returnTick;
    };
    // Phase A.1: per-port deferred-credit queues. Returns post to the
    // pool that issued the FLIT.
    std::vector<std::deque<DeferredFlitCredit>> deferredFlitCredits;
    void processFlitCreditReturn();
    EventFunctionWrapper flitCreditEvent;

    struct PendingHostReq {
        PacketPtr pkt;
        Tick earliestSend;  // RC traversal completion time
    };
    // Phase A.1: per-port pending host-bound queues. Avoids one port's
    // long pendingHostReqs queue blocking another port's host-bound
    // injection. Round-robin across ports in trySendToHost.
    std::vector<std::deque<PendingHostReq>> pendingHostReqs;
    unsigned nextHostSendPort = 0;
    Tick rootPortDelay;    // per-FLIT root port decode delay
    // hostSendSpacing removed — host xbar provides natural backpressure
    void trySendToHost();
    EventFunctionWrapper hostSendEvent;

    // ================================================================
    //  Statistics
    // ================================================================
    struct CxlStats : public statistics::Group
    {
        CxlStats(CxlModel &owner);
        statistics::Scalar readRequests, writeRequests;
        statistics::Scalar totalReadBytes, totalWriteBytes;
        statistics::Scalar totalReadFlits, totalWriteFlits;
        statistics::Scalar totalCompletionFlits;
        statistics::Scalar totalWireBytes;
        statistics::Scalar assemblyBottleneckFlits;
        statistics::Histogram readLatencyHist, writeLatencyHist;
        statistics::Histogram readCoalesceHist, writeCoalesceHist;
        statistics::Scalar totalReadLatency, totalWriteLatency;
        // Review fix #2: REMOVED avgReadLatency/avgWriteLatency Formulas.
        // They divided per-beat latency sum by per-group counter,
        // inflating averages by coalesce factor. Use histograms for
        // accurate distribution info.
    };
    CxlStats stats;

    // ================================================================
    //  Per-request timing breakdown (printed via DPRINTF)
    //  Tracks first/last timestamps at each pipeline stage
    //  to show where time is spent.
    // ================================================================
    struct PhaseTracker {
        Tick firstDevReq = 0;     // first handleDeviceRequest
        Tick lastDevReq = 0;      // last handleDeviceRequest
        Tick firstFlitDone = 0;   // first processUpstreamQueue
        Tick lastFlitDone = 0;    // last processUpstreamQueue
        Tick firstHostSend = 0;   // first trySendToHost
        Tick lastHostSend = 0;    // last trySendToHost
        Tick firstHostResp = 0;   // first handleHostResponse
        Tick lastHostResp = 0;    // last handleHostResponse
        Tick firstDnDone = 0;     // first processDownstreamQueue delivery
        Tick lastDnDone = 0;      // last processDownstreamQueue delivery
        unsigned reqCount = 0;
        unsigned respCount = 0;
        bool isRead = true;
        // FLIT packing debug
        unsigned flitFlushCount = 0;    // number of FLIT flushes
        unsigned totalFlitEntries = 0;  // total FLIT entries enqueued
        unsigned totalWireBytes = 0;    // total wire bytes
        unsigned totalBeatsInFlits = 0; // total beats packed
        unsigned hostSendCount = 0;     // trySendToHost successes
        unsigned hostRejectCount = 0;   // trySendToHost rejections (xbar busy)
        void reset() {
            firstDevReq = lastDevReq = 0;
            firstFlitDone = lastFlitDone = 0;
            firstHostSend = lastHostSend = 0;
            firstHostResp = lastHostResp = 0;
            firstDnDone = lastDnDone = 0;
            reqCount = respCount = 0;
            isRead = true;
            flitFlushCount = totalFlitEntries = totalWireBytes = 0;
            totalBeatsInFlits = hostSendCount = hostRejectCount = 0;
        }
    };
    PhaseTracker rdTracker, wrTracker;
    void printPhaseBreakdown(PhaseTracker &t, const char *label);

    // ================================================================
    //  Diagnostic Infrastructure
    //
    //  When something goes wrong at N>1, the pipeline is wide and it's
    //  hard to tell WHERE it's stuck. These counters and helpers make
    //  each stage visible in the inform log so you can grep quickly.
    // ================================================================

    // Per-device-port accounting (sized at init time)
    std::vector<uint64_t> perPortBeatsAccepted;    // reads+writes at accept
    std::vector<uint64_t> perPortReadsAccepted;    // reads accepted
    std::vector<uint64_t> perPortReadsDelivered;   // read beats delivered
    std::vector<uint64_t> perPortWritesAccepted;   // write beats accepted
    std::vector<uint64_t> perPortBrespsDelivered;  // BRESPs delivered
    // Per-host-port accounting
    std::vector<uint64_t> perHostPortSent;         // successful sendTimingReq
    std::vector<uint64_t> perHostPortRejected;     // sendTimingReq=false

    // Progress watchdog. Updated at every stage transition; checked
    // periodically. If no progress for stuckThreshold ticks AND there
    // are pending items, dumpCxlState is called with trigger="STUCK".
    Tick lastProgressTick = 0;
    Tick lastDiagDumpTick = 0;
    Tick lastStuckAlarmTick = 0;

    // Structured multi-line state dump. Called periodically from
    // processDownstreamQueue and on demand (stuck, anomaly). The
    // trigger string is logged so you can filter the cause.
    void dumpCxlState(const char *trigger);

    // Periodic diagnostic event — fires every 10M ticks regardless of
    // traffic, matching PCIe's diagEvent. Catches deadlocks that occur
    // before any downstream FLITs arrive (where the existing
    // processDownstreamQueue-based dump would never fire).
    EventFunctionWrapper diagEvent;

    // Invariant checks (DPRINTF-gated so production overhead is zero
    // when CXL debug flag is disabled).
    void checkInvariants(const char *where);
};

} // namespace gem5

#endif // __MEM_CXL_CXL_MODEL_HH__
