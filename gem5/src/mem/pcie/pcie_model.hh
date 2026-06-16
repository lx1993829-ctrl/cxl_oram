/*
 * pcie_model.hh — TLP-level pipelined PCIe Gen3/4/5 model for gem5 v25.1
 *
 * TLP-accurate with credit-based flow control and DLL-overhead accounting:
 *   - Real TLP byte arrays with header fields per PCIe spec
 *   - PCIe hard block assembly pipeline (core clock + datapath width)
 *   - DW alignment padding
 *   - Completion reordering by tag
 *   - LCRC/framing/ECRC, ACK/NAK, deferred credit return
 *   - AXI bridge pipeline (clock domain crossing)
 *
 * Models the Xilinx AXI Memory Mapped to PCI Express bridge IP path.
 */

#ifndef __MEM_PCIE_MODEL_HH__
#define __MEM_PCIE_MODEL_HH__

#include <deque>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include "mem/port.hh"
#include "params/PCIeModel.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

namespace gem5
{

class PCIeModel : public ClockedObject
{
  public:
    PARAMS(PCIeModel);
    PCIeModel(const Params &p);

    Port &getPort(const std::string &if_name, PortID idx) override;
    void init() override;

  private:

    // ================================================================
    //  Link Parameters
    // ================================================================
    struct LinkParams {
        unsigned gen, lanes;
        double   transferRate, encodingEff;
        uint64_t effectiveBwBytesPerSec;
        Tick     byteSerializationDelay;
    };
    LinkParams linkParams;
    void computeLinkParams();

    // ================================================================
    //  TLP Parameters
    // ================================================================
    unsigned mps, mrrs, maxTags, rcb;

    // ================================================================
    //  Latency Parameters
    // ================================================================
    Tick readLatency, writeLatency, rcLatency, rcThroughputDelay;
    Tick fpgaClockPeriod;  // PCIe Bug #7 fix: from Param.Latency
    std::vector<Tick> bridgeBusyUntil;  // Per-port downstream CDC serialization
    Tick rcDownstreamBusyUntil;  // Shared RC pipeline throughput (downstream)
    Tick rcUpstreamBusyUntil;    // Shared RC pipeline throughput (upstream)

    // ================================================================
    //  Data Link Layer
    // ================================================================
    unsigned lcrcBytes, framingBytes, ecrcBytes;
    Tick     dllpAckDelay;
    Tick     creditReturnDelay;

    // ================================================================
    //  AXI Bridge + PCIe Core Pipeline
    // ================================================================
    Tick     bridgePipelineDelay;   // AXI clock domain crossing
    Tick     hostInjectInterval;    // min interval between host xbar sends
    Tick     pcieCorePeriod;        // PCIe core clock period (ticks)
    unsigned pcieCoreWidthBytes;    // Internal datapath width in bytes

    // Assembly delay: how long the PCIe core takes to assemble
    // a TLP from AXI beats at pcieCorePeriod rate with
    // pcieCoreWidthBytes-wide datapath.
    Tick assemblyDelay(unsigned totalBytes) const;

    // ================================================================
    //  Real TLP Packet Structure (PCIe spec §2.2)
    // ================================================================

    // TLP Header DW0: fmt[7:5] | type[4:0] | T9 | TC[6:4] | ...
    // We build real byte arrays for validation and trace.

    // PCIe TLP format field values
    static const uint8_t FMT_3DW_NODATA = 0x0;  // 000
    static const uint8_t FMT_4DW_NODATA = 0x1;  // 001
    static const uint8_t FMT_3DW_DATA   = 0x2;  // 010
    static const uint8_t FMT_4DW_DATA   = 0x3;  // 011

    // PCIe TLP type field values
    static const uint8_t TYPE_MRD  = 0x00;  // Memory Read
    static const uint8_t TYPE_MWR  = 0x00;  // Memory Write (fmt has data)
    static const uint8_t TYPE_CPL  = 0x0A;  // Completion
    static const uint8_t TYPE_CPLD = 0x0A;  // Completion with Data

    struct TlpPacket {
        // Raw header bytes (12 or 16 bytes)
        std::vector<uint8_t> header;

        // DW alignment padding bytes (0-3 before payload)
        unsigned paddingBytes;

        // Payload data (may be empty for reads/Cpl)
        // We don't copy actual data — just track size
        unsigned payloadBytes;

        // Computed sizes
        unsigned headerBytes;    // 12 or 16
        unsigned totalBytes;     // header + padding + payload
        unsigned wireBytes;      // total + LCRC + framing + ECRC

        // Metadata
        uint8_t  fmt;
        uint8_t  type;
        uint16_t requesterId;
        uint16_t tag;
        uint16_t length;         // in DW (0 = 1024 DW)
        Addr     addr;
        uint16_t completerId;
        uint16_t byteCount;      // for completions
        uint8_t  lowerAddr;      // for completions

        // gem5 integration
        PacketPtr origPkt;
        std::vector<PacketPtr> allWritePkts;  // coalesced write packets (for DDR5 routing)
        Tick     creationTick;
        Tick     issueTick;       // original device request time (for stats)
        bool     isLastCompletion;
        unsigned origSize;
        int      srcPortIdx = 0;
    };

    // Build TLP packets with real header bytes
    TlpPacket buildMemReadTlp(Addr addr, unsigned length,
                               uint16_t tag, PacketPtr pkt);
    TlpPacket buildMemWriteTlp(Addr addr, unsigned payloadBytes,
                                PacketPtr pkt);
    TlpPacket buildCompletionTlp(Addr addr, unsigned payloadBytes,
                                  uint16_t tag, unsigned totalBytes,
                                  unsigned completedBytes,
                                  PacketPtr pkt, bool isLast);
    // PCIe Bug #13: buildCompletionNoDataTlp removed (was never called).

    void buildTlpHeader(TlpPacket &tlp);
    unsigned computeWireBytes(unsigned totalBytes) const;
    unsigned computePaddingBytes(Addr addr) const;
    int dataCreditsNeeded(unsigned payloadBytes) const;

    // ================================================================
    //  Credit-based Flow Control
    //
    //  Phase A.1: per-port credit pools. Each device-side port gets a
    //  full CreditPool (PH/PD/NPH/NPD/CPLH/CPLD) sized from params.
    //  Models real PCIe topology where each AXI master's bridge
    //  channel has its own credit budget — one master never depletes
    //  another's pool. Credit helpers gain a srcPort parameter so the
    //  right pool is consumed/returned.
    // ================================================================
    struct CreditPool {
        int phCredits, pdCredits, phMax, pdMax;
        int nphCredits, npdCredits, nphMax, npdMax;
        int cplhCredits, cpldCredits, cplhMax, cpldMax;
    };
    std::vector<CreditPool> credits;

    void initCredits();
    bool hasPostedCredits(unsigned sp, int dc) const;
    bool hasNonPostedCredits(unsigned sp) const;
    bool hasCompletionCredits(unsigned sp, int dc) const;
    void consumePostedCredits(unsigned sp, int dc);
    void consumeNonPostedCredits(unsigned sp);
    void consumeCompletionCredits(unsigned sp, int dc);
    void returnPostedCredits(unsigned sp, int dc);
    void returnNonPostedCredits(unsigned sp);
    void returnCompletionCredits(unsigned sp, int dc);
    void wakeupDrainOnCreditReturn();

    struct DeferredCredit {
        enum Type { Posted, NonPosted, Completion } type;
        int dataCredits;
        Tick returnTick;
    };
    // Phase A.1: per-port deferred-credit queues, returns post to the
    // pool that issued the request.
    std::vector<std::deque<DeferredCredit>> deferredCredits;
    void scheduleDeferredCreditReturn(unsigned sp,
                                       DeferredCredit::Type type, int dataCr);
    void processDeferredCredits();
    EventFunctionWrapper creditReturnEvent;

    // ================================================================
    //  Tag Tracking
    // ================================================================
    struct OutstandingRead {
        PacketPtr pkt;
        std::vector<PacketPtr> allPkts;  // all device packets (for coalesced response delivery)
        Addr addr;
        unsigned totalBytes, completedBytes;
        unsigned pendingCplBytes = 0;
        unsigned emittedBytes = 0;
        Tick issueTick;
        int srcPortIdx = 0;
    };
    std::unordered_map<uint16_t, OutstandingRead> outstandingReads;
    std::deque<uint16_t> freeTags;
    uint16_t allocateTag();
    void releaseTag(uint16_t tag);
    bool hasFreeTags() const { return !freeTags.empty(); }

    // ================================================================
    //  Read Reorder Buffer (RRB)
    //  Review item #1: AXI-PCIe bridge (Xilinx PG194) has an internal
    //  completion buffer between the PCIe completion stream and the
    //  AXI R channel. Separate resource from PCIe tag pool. Typically
    //  128-512 entries. Gates in-flight completions — can stall even
    //  when tags are free.
    //  Phase A.1: per-port. Each AXI master has its own RRB; one
    //  master's full RRB cannot stall another's completions.
    // ================================================================
    unsigned rrbDepth;                       // param-derived per-port max
    std::vector<unsigned> rrbOccupied;       // current in-RRB beats per port

    // Review item #3: per-core-clock TLP gap
    Tick coreClockPeriod;        // min gap between TLPs on upstream
    Tick lastUpstreamEmit;       // tick of last TLP emitted upstream
    Tick lastDownstreamEmit;     // tick of last TLP emitted downstream

    // Review item #4: batched credit-return cadence
    Tick creditReturnPeriod;
    Tick nextCreditReturnTick;

    // Review item #7: completion timeout (per-tag deadline)
    Tick completionTimeoutTicks;
    std::unordered_map<uint16_t, Tick> tagIssueDeadline;

    // Review item #8: TLP header size selection (32 vs 64-bit)
    bool use64bitAddr;
    unsigned tlpHeaderBytes() const { return use64bitAddr ? 16 : 12; }

    // ================================================================
    //  Completion Reordering Buffer
    //  Root complex may return completions out-of-order for different
    //  tags. Completions for the same tag must be in-order.
    //  We buffer completions and deliver per-tag in order.
    // ================================================================
    unsigned completionReorderDepth;

    struct PendingCompletion {
        TlpPacket tlp;
        Tick      arrivalTick;
    };
    // Map from tag → ordered list of completions for that tag
    std::map<uint16_t, std::deque<PendingCompletion>> completionBuffer;

    void bufferCompletion(TlpPacket &tlp, Tick arrivalTick);
    void deliverCompletions(Tick now);

    // ================================================================
    //  Link Serialization Queues
    //
    //  Phase A.1: per-port link queues. The wire (upstreamBusyUntil/
    //  downstreamBusyUntil), the RC pipeline burst-window state, and
    //  the per-FLIT emission gates stay shared — those model genuinely
    //  shared physical resources. Per-port BUFFERS only, so a stalled
    //  TLP on one port doesn't head-of-line-block another port.
    // ================================================================
    struct LinkQueueEntry {
        TlpPacket tlp;
        Tick readyTick;
    };

    std::vector<std::deque<LinkQueueEntry>> upstreamQueue;
    Tick upstreamBusyUntil;

    std::vector<std::deque<LinkQueueEntry>> downstreamQueue;
    Tick downstreamBusyUntil;

    Tick serializationDelay(unsigned wireBytes) const;
    void enqueueUpstream(TlpPacket &tlp);
    void enqueueDownstream(TlpPacket &tlp, Tick earliestStart = 0);

    // Rotating-start FCFS cursors for dispatch across per-port queues.
    unsigned nextUpstreamPort = 0;
    unsigned nextDownstreamPort = 0;

    // ================================================================
    //  Ports
    // ================================================================
    class DeviceSidePort : public ResponsePort
    {
      public:
        DeviceSidePort(const std::string &name, PCIeModel &owner, int idx);
      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;
        void recvFunctional(PacketPtr pkt) override;
        AddrRangeList getAddrRanges() const override;
      private:
        PCIeModel &owner;
        int portIdx;
        bool needRetry;
        friend class PCIeModel;
    };

    class HostSidePort : public RequestPort
    {
      public:
        HostSidePort(const std::string &name, PCIeModel &owner, int idx);
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
        void recvRangeChange() override;
      private:
        PCIeModel &owner;
        int portIdx;
        bool needRetry;      // blocked on sendTimingReq (request direction)
        friend class PCIeModel;
    };

    std::vector<DeviceSidePort *> devicePorts;
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
        int srcPortIdx = 0;
    };

    // ================================================================
    //  Pipeline
    // ================================================================
    bool handleDeviceRequest(PacketPtr pkt, int srcPort = 0);

    // Device request buffer: models AXI-PCIe bridge internal FIFO.
    // Decouples request acceptance from TLP construction.
    // Requests accepted into buffer immediately; TLPs built when
    // credits/tags are available. Backpressure only when buffer full.
    //
    // PER-PORT BUFFERS: real Xilinx AXI-PCIe bridge has independent
    // per-channel FIFOs — one per AXI master. Writes from different
    // masters never share a FIFO, so coalescing within a FIFO is
    // always against same-master contiguous beats. Model this literally
    // with std::vector<std::deque<...>> indexed by srcPort. Sized to
    // match devicePorts on init.
    struct BufferedDevReq { PacketPtr pkt; int srcPort; };
    std::vector<std::deque<BufferedDevReq>> deviceReadBuffers;
    std::vector<std::deque<BufferedDevReq>> deviceWriteBuffers;
    void drainDeviceRequests();
    bool processBufferedRead(PacketPtr pkt, int srcPort);
    bool processBufferedWrite(PacketPtr pkt, int srcPort);
    EventFunctionWrapper drainDeviceEvent;

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
    //  Burst-Window RC Optimization
    //
    //  On real hardware, the AXI-to-PCIe bridge coalesces an AXI burst
    //  into one TLP, paying RC traversal once. In our model, each AXI
    //  beat is a separate gem5 packet (required for DDR5 interleaving).
    //  To avoid paying RC latency per-beat, we track when the RC was
    //  last traversed. Packets arriving in the same burst window
    //  (same tick or within one ORAM cycle) share one RC traversal.
    // ================================================================
    Tick lastUpstreamRcTick;    // DEAD — kept for ABI compat, never read
    Tick lastDownstreamRcTick;  // DEAD — kept for ABI compat, never read
    Tick burstWindowTicks;      // packets within this window share RC

    // ================================================================
    //  Phase B/A.1 Diagnostic Instrumentation
    //
    //  Per-port stuck detection — see cxl_model.hh for rationale.
    // ================================================================
    std::vector<Tick> perPortLastDrainTick;
    Tick lastPortStuckCheckTick = 0;
    int rcDbgCount = 0;      // per-instance debug limit (was static local)
    int rcWrDbgCount = 0;
    int rcDnDbgCount = 0;
    void dumpPortState(unsigned p, const char *where);
    void checkPortStalls();

    struct ResponseEntry {
        PacketPtr pkt;
        int portIdx;
        Tick readyTick;   // earliest tick this response may be delivered
    };
    // Phase A.1: per-port response queues. Each port can be blocked
    // independently on sendTimingResp; round-robin across ports.
    std::vector<std::deque<ResponseEntry>> responseQueue;
    unsigned nextResponsePort = 0;
    void trySendResponses();
    EventFunctionWrapper responseEvent;

    // ================================================================
    //  Backpressure + outstanding tracking
    // ================================================================
    unsigned maxOutstanding;        // 0 = disabled, from params (reads)
    unsigned maxOutstandingWrites;  // separate write buffer depth
    // Phase A.1: per-port outstanding-write counters. Mirrors the
    // existing perPortReadsOut pattern so one port cannot consume
    // every write slot and starve others.
    std::vector<unsigned> outstandingWrites;
    std::vector<unsigned> perPortReadsOut;  // per-device-port outstanding reads
    bool pendingDeviceRetry;   // deferred retry after BRESP delivery
    unsigned nextRetryPort;    // round-robin index for fair retry

    void retryStarvedPorts();

    // Periodic diagnostic for debugging deadlocks
    void dumpDiagnostics();
    EventFunctionWrapper diagEvent;

    // Write commit tracking: compare posted BRESP (RTL visible) vs DDR5 commit
    Tick firstWriteAccept = 0;   // first handleDeviceRequest for write
    // PER-PORT: each port tracks its own last-buffered tick for the
    // coalescing timeout gate. Moved from global to per-port alongside
    // the buffer split so timeouts fire correctly on a per-port basis.
    std::vector<Tick> lastWriteBufferTicks;
    std::vector<Tick> lastReadBufferTicks;
    Tick lastBrespDelivered = 0; // last posted BRESP delivered to RTL
    Tick lastDdr5WriteCommit = 0; // last handleHostResponse for write
    unsigned writeCommitCount = 0;
    unsigned writeBrespCount = 0;

    struct PendingHostReq {
        PacketPtr pkt;
        Tick earliestSend;  // RC traversal completion time
    };
    // Phase A.1: per-port pending host-bound queues with round-robin
    // dispatch.
    std::vector<std::deque<PendingHostReq>> pendingHostReqs;
    unsigned nextHostSendPort = 0;
    void trySendToHost();
    EventFunctionWrapper hostSendEvent;

    // ================================================================
    //  Per-phase timing breakdown (same as CXL)
    // ================================================================
    struct PhaseTracker {
        Tick firstDevReq = 0;
        Tick lastDevReq = 0;
        Tick firstTlpDone = 0;    // first processUpstreamQueue
        Tick lastTlpDone = 0;
        Tick firstHostSend = 0;
        Tick lastHostSend = 0;
        Tick firstHostResp = 0;
        Tick lastHostResp = 0;
        Tick firstDnDone = 0;     // first processDownstreamQueue delivery
        Tick lastDnDone = 0;
        unsigned reqCount = 0;
        unsigned respCount = 0;
        bool isRead = true;
        void reset() {
            firstDevReq = lastDevReq = 0;
            firstTlpDone = lastTlpDone = 0;
            firstHostSend = lastHostSend = 0;
            firstHostResp = lastHostResp = 0;
            firstDnDone = lastDnDone = 0;
            reqCount = respCount = 0;
            isRead = true;
        }
    };
    PhaseTracker rdTracker, wrTracker;
    void printPhaseBreakdown(PhaseTracker &t, const char *label);

    // ================================================================
    //  Statistics
    // ================================================================
    struct PCIeStats : public statistics::Group
    {
        PCIeStats(PCIeModel &owner);
        statistics::Scalar totalReadTLPs, totalWriteTLPs;
        statistics::Scalar totalCompletionTLPs;
        statistics::Scalar completionsCombined;
        statistics::Scalar totalReadBytes, totalWriteBytes;
        statistics::Scalar totalCompletionBytes;
        statistics::Scalar totalWireBytes;
        statistics::Scalar totalPaddingBytes;
        statistics::Scalar readRequests, writeRequests;
        statistics::Scalar tagExhausted, creditStalls;
        statistics::Scalar creditOvercommits;
        statistics::Scalar assemblyBottleneckTLPs;
        statistics::Scalar completionsBuffered;
        statistics::Histogram readLatencyHist, writeLatencyHist;
        statistics::Histogram readCoalesceHist, writeCoalesceHist;
        // Review fix #2: removed avgWriteLatency (was inflated by
        // coalesce factor). avgReadLatency stays — correctly per-group.
        statistics::Formula avgReadLatency;
        statistics::Scalar totalReadLatency, totalWriteLatency;
    };
    PCIeStats stats;

    // Requester ID for this FPGA endpoint (bus:dev.func)
    uint16_t requesterId;
    // Completer ID for root complex
    uint16_t completerId;
};

} // namespace gem5

#endif // __MEM_PCIE_MODEL_HH__
