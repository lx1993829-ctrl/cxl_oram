#ifndef __ORAM_ORAM_DEVICE_HH__
#define __ORAM_ORAM_DEVICE_HH__

#include <deque>
#include <map>
#include <array>
#include <random>
#include <set>
#include <unordered_map>
#include <vector>

#include "mem/port.hh"
#include "mem/tport.hh"
#include "params/OramDevice.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"

#include "Vsecure_oram_top.h"
#include "verilated.h"

namespace gem5
{

class OramDevice : public ClockedObject
{
  public:
    PARAMS(OramDevice);
    OramDevice(const OramDeviceParams &p);
    ~OramDevice();
    void init() override;
    void startup() override;
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    // Multi-instance coordination
    static unsigned totalInstances;
    static unsigned completedInstances;
    unsigned instanceId;

  private:

    class MemPort : public RequestPort
    {
      public:
        MemPort(const std::string &n, OramDevice &d, bool isPcie)
            : RequestPort(n), d(d), isPcie(isPcie) {}
      protected:
        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;
      private:
        OramDevice &d;
        bool isPcie;
    };

    // ------------------------------------------------------------------
    // cmd_port: MMIO response port used in cpu_driven mode.
    //
    // Inherits from SimpleTimingPort — gem5's standard base for PIO-style
    // response ports. SimpleTimingPort automatically:
    //   - Implements recvTimingReq and recvFunctional via recvAtomic
    //   - Uses an internal RespPacketQueue to hold deferred timing responses
    //   - Handles upstream retry signalling
    //
    // We only need to override recvAtomic (with the actual register-access
    // logic) and getAddrRanges (to report our MMIO range).
    //
    // Step 1: stubbed — recvAtomic just logs and zero-fills reads. Real
    // command decode lands in Step 3+.
    // ------------------------------------------------------------------
    class CmdPort : public SimpleTimingPort
    {
      public:
        CmdPort(const std::string &n, OramDevice &d)
            : SimpleTimingPort(n, &d), dev(d) {}
      protected:
        Tick recvAtomic(PacketPtr pkt) override;
        AddrRangeList getAddrRanges() const override;
      private:
        OramDevice &dev;
    };

    // cpu_driven mode configuration. Declared FIRST (before cmdPort) so
    // these members initialize BEFORE cmdPort's constructor runs. The
    // cmdPort's getAddrRanges() reads dev.cmdBase — if cmdBase is still 0
    // at port construction, the xbar caches a bogus [0x0, 0x100) range
    // and packets to 0xE0000000 can't be routed.
    bool cpuDriven;
    unsigned numLogicalClients;
    Addr cmdBase;
    Addr resultBufBase;
    uint64_t resultBufSize;  // Step 5: bounds for sendResultPacket

    // Tracks which hardware client slot (0 or 1) is currently driving the
    // RTL's client_* vectors. Used by feedClientWdata to pack wdata beats
    // into the correct half of client_wdata[] and set the correct bit of
    // client_wdata_valid. Default 0 preserves legacy behavior (single-
    // client mode uses hw-client 0 always).
    unsigned activeHwClient;

    MemPort hbmPort, pciePort;
    CmdPort cmdPort;
    VerilatedContext *verilatedCtx;
    Vsecure_oram_top *oram;

    static constexpr unsigned CMD_REGION_BYTES = 0x1000;  // 4 KB MMIO region

    // Step 5: per-hw-client shadow buffer for client_rdata. The RTL may
    // assert client_rdata_valid[hw] on a cycle earlier than (or the same
    // as) client_done[hw]. Capturing rdata ONLY when checking done is
    // fragile — if rdata_valid was a one-cycle pulse, client_rdata may
    // already be stale by the time we look at it. Solution: every tick,
    // if rdata_valid[hw] is set, latch client_rdata[hw*8..hw*8+7] into
    // rdataShadow[hw][] and mark rdataShadowValid[hw]=true. completeOp
    // consumes the shadow (not the live signal) and clears the valid bit.
    uint32_t rdataShadow[2][8];
    bool     rdataShadowValid[2];
    bool     rdataValidEverSeen[2];  // latch: was rdata_valid EVER seen this op?

    // Deferred completion: latch client_done and wait for rdataShadow
    // before calling completeOp() for READ ops.
    bool     clientDoneSeen = false;
    unsigned clientDoneWaitCycles = 0;

    // Step 5: sender-state tag attached to result-packet writes. Used by
    // MemPort::recvTimingResp to distinguish result-buffer completions
    // (just delete the packet) from AXI completions (handleMemResp).
    // Carries the opIdx so recvTimingResp knows which queued op committed.
    class ResultSenderState : public Packet::SenderState
    {
      public:
        uint64_t opIdx;
        ResultSenderState(uint64_t i) : opIdx(i) {}
    };

    // Step 9: sender-state tag for command-ring fetches. ORAM reads
    // ring entries (and the prod_idx field) via pcie_port. Responses
    // come back into recvTimingResp; we distinguish them by this type
    // and route to the ring-fetcher state machine.
    class CmdRingFetchSenderState : public Packet::SenderState
    {
      public:
        // What this fetch is for. PROD_IDX = reading the prod_idx
        // field; CMD_ENTRY = reading one command entry (slot known
        // from ringSlot below); CONS_WRITEBACK = our outgoing write
        // of cons_idx, response is just acked-and-discarded.
        enum class Kind : uint8_t { PROD_IDX, CMD_ENTRY, CONS_WRITEBACK };
        Kind     kind;
        uint64_t ringSlot;    // valid iff kind==CMD_ENTRY (CPU's i, not slot%16)
        uint64_t expectedIdx; // sanity check: matches CPU-assigned opIdx in entry

        CmdRingFetchSenderState(Kind k, uint64_t slot = 0, uint64_t exp = 0)
            : kind(k), ringSlot(slot), expectedIdx(exp) {}
    };

    // Send a 64 B result packet via pcie_port to result_buf_base +
    // opIdx*64. Uses the same retry machinery as the AXI write path
    // (sendPkt with isPcie=true).
    void sendResultPacket(uint64_t opIdx);

    // Step 9: send a small read request to fetch one cache line from the
    // command ring. ss carries the kind (prod_idx vs cmd entry) and
    // sequence info so recvTimingResp can route the response.
    void sendCmdRingRead(Addr gem5Addr, CmdRingFetchSenderState *ss);

    // Step 9: handle the response from a ring-fetch read. Routes by
    // ss->kind: prod_idx response triggers new entry-reads; entry
    // response parses payload into a CmdEntry and pushes into cmdQueue.
    void handleCmdRingResp(PacketPtr pkt, CmdRingFetchSenderState *ss);

    // Step 9: write our local cons_idx back to the ring header so the
    // CPU sees backpressure relief.
    void writeConsIdx(uint64_t newConsIdx);

    // Step 4/9: command entry. CPU populates fields, doorbell or ring
    // delivery pushes a copy into cmdQueue. Tick loop dispatches the
    // queue head when RTL is idle. After completion, rdata (if read)
    // is captured. The result packet is written to result_buf and
    // cmdsCommitted is incremented when its WriteResp returns.
    struct CmdEntry {
        // Command fields (from MMIO writes pre-doorbell, OR from a
        // ring fetch).
        uint32_t slot_addr;
        uint32_t token;
        uint8_t  lease_id;
        uint8_t  op;           // 0=read, 1=write
        uint8_t  hw_client;    // 0 or 1
        uint32_t wdata[8];     // 32 B payload

        // Lifecycle
        enum class Phase : uint8_t {
            PENDING,        // queued, waiting to dispatch
            IN_PROGRESS,    // driven into RTL, waiting client_done
            COMPUTE_DONE,   // RTL done, result-pkt sent, awaiting WriteResp
            COMMITTED       // WriteResp received, ready to remove from queue
        };
        Phase phase;

        // Per-op identity. For Step 4-7 (single-op MMIO path) opIdx is
        // assigned at completeOp time from cpuOpCount. For Step 9 ring
        // path, opIdx is the CPU's loop counter, copied from the ring
        // entry.
        uint64_t opIdx;

        // Captured rdata
        uint32_t rdata[8];
        bool     rdata_valid;

        // --- E2E timing (gem5 Ticks) ---
        Tick fetchTick;      // when ring entry arrived / MMIO doorbell
        Tick dispatchTick;   // when dispatched to RTL (IN_PROGRESS)
        Tick rtlDoneTick;    // when RTL asserted client_done
        Tick commitTick;     // when result WriteResp arrived (COMMITTED)
    };

    // Step 9: queue of pending+in-flight ops. Capacity = cmdQueueDepth
    // SimObject param. Dispatch order = FIFO (push_back / front+pop_front).
    // The currently-dispatched op stays at queue front in IN_PROGRESS
    // phase; it's only popped when its result packet COMMITS. This keeps
    // RTL feedClientWdata sourcing from a stable slot.
    std::deque<CmdEntry> cmdQueue;
    size_t               cmdQueueDepth;   // SimObject param mirror

    // Step 4: staging slot for MMIO-path command writes. CPU writes to
    // MMIO regs populate stagingCmd; doorbell pushes a copy into
    // cmdQueue. Step 9 ring path bypasses this.
    CmdEntry stagingCmd;

    // Step 9: command ring state. Filled at construction from SimObject
    // params. CPU writes prod_idx into header at +0x00, ORAM writes
    // cons_idx at +0x40.
    Addr     cmdRingBase;       // 0 means ring path disabled
    uint64_t cmdRingDepth;      // number of slots
    static constexpr Addr CMD_RING_PROD_IDX_OFFSET = 0x00;
    static constexpr Addr CMD_RING_CONS_IDX_OFFSET = 0x40;
    static constexpr Addr CMD_RING_ENTRIES_OFFSET  = 0x100;
    static constexpr size_t CMD_RING_ENTRY_BYTES   = 64;

    // ORAM-side ring tracking
    uint64_t ringConsIdx;       // local: how many entries we've fetched
    uint64_t ringProdIdxKnown;  // last value we saw for prod_idx
    bool     ringFetchInFlight; // a prod_idx or entry read is outstanding
    bool     ringNeedsProdRead; // true after doorbell, until we issue prod read
    uint64_t ringEntriesInFlight; // count of entry-reads outstanding

    // Step 9 doorbell: CPU writes any value to MMIO offset 0x150 to
    // signal "I have produced new commands; please re-read prod_idx".
    static constexpr Addr CMD_RING_DOORBELL_OFFSET = 0x150;

    // Step 4 gate: CPU writes 1 to GATE_RELEASE_OFFSET to release ORAM
    // from its post-reset hold state when the Python config sets
    // gated_start=True. Reads from GATE_STATUS_OFFSET return:
    //   0 = held (gated_start=true and gate not yet released)
    //   1 = released, init still running
    //   2 = released and init complete (== ready bit)
    // Configs with gated_start=False (the default) auto-start as before
    // and read this register as 1→2 over time. Both offsets are unused
    // anywhere else in the cmd_port MMIO map.
    static constexpr Addr GATE_RELEASE_OFFSET = 0x180;
    static constexpr Addr GATE_STATUS_OFFSET  = 0x184;

    // Step 5/6/9: count of result packets that have COMMITTED to memory.
    // Increments in recvTimingResp when a ResultSenderState arrives.
    // CPU reads this at MMIO 0xE0 to know how many ops are fully done
    // and verifiable from result_buf.
    uint64_t cpuOpCount;

    // Step 8: snapshot of the most-recently-committed op's rdata, kept
    // for legacy MMIO reads at 0xD4 / 0x200. With depth=1 these match
    // the prior cmdOp.rdata semantics. With depth>1 the workload
    // should read from result_buf directly instead.
    uint32_t lastCompletedRdata[8];
    bool     lastCompletedRdataValid;

    Tick oramClkPeriod;
    uint64_t oramCycle;
    EventFunctionWrapper tickEvent;
    void tick();
    void scheduleTick();

    Addr hbmBase, hostBase, stashOffset;

    static constexpr int AXI_DATA_BYTES = 32;

    // =====================================================================
    // Off-chip metadata address map — MUST match oram_params.vh / pos_map.v.
    // Stash deepened to 16384 entries (STASH_PTR_W=14) => 64 MB region, so
    // every region above STASH shifted up by 60 MB vs the old 4 MB-stash map.
    //
    //   region            base (rel hbmBase)   size
    //   STASH             0x10000000           64 MB (16384 x 4KB)
    //   PM_BASE           0x14000000            1 MB
    //   HT_SLOT_BASE      0x14100000            1 MB
    //   HT_BKT_HEAD_BASE  0x14200000            1 MB
    //   HT_BKT_NEXT_BASE  0x14300000            1 MB
    //   IVT_BASE          0x14400000            4 MB (per-physical-slot, 2MB used)
    // =====================================================================
    static constexpr Addr STASH_BASE_ADDR     = 0x10000000;
    static constexpr Addr STASH_REGION_BYTES  = 0x04000000;   // 64 MB
    // Must match pos_map.v PM_BASE parameter
    static constexpr Addr PM_BASE_ADDR        = 0x14000000;
    static constexpr Addr HT_SLOT_BASE_ADDR   = 0x14100000;
    static constexpr Addr HT_BKT_HEAD_ADDR    = 0x14200000;
    static constexpr Addr HT_BKT_NEXT_ADDR    = 0x14300000;
    static constexpr Addr IVT_BASE_ADDR       = 0x14400000;
    static constexpr Addr SLOT_R_BASE_ADDR    = 0x14800000;   // per-stash-entry slot addr
    static constexpr Addr BUCKET_META_BASE_ADDR = 0x14900000; // per-bucket directory
    // Metadata region begins at STASH_BASE_ADDR; everything >= this is HBM.
    static constexpr Addr METADATA_REGION_START = STASH_BASE_ADDR;
    static constexpr Addr HT_REGION_END       = BUCKET_META_BASE_ADDR + 0x00100000; // top of bucket_meta (1MB)

    static constexpr int BUCKET_ID_BITS = 13;  // must match `BUCKET_ID_W
    static constexpr int BUCKET_BYTES = 32768;
    static constexpr Addr LEASE_BASE = 0x1000;
    static constexpr int SLOT_SIZE = 0x1000;
    static constexpr int MAX_SLOTS = 32768;       // ORAM_N (compiled max)
    static constexpr int MAX_BUCKETS = 8192;       // ORAM_B (compiled max)
    static constexpr int ORAM_Z = 8;               // total positions per bucket (c + s)
    static constexpr int ORAM_C = MAX_SLOTS / MAX_BUCKETS;  // = 4, nominal slots per bucket
    static constexpr int SLOTS_PER_BUCKET = ORAM_Z; // backward compat (physical capacity)
    // Stash compiled for max (16384). Runtime uses numSlots/2.
    static constexpr int STASH_DEPTH = 16384;

    uint32_t localPct, numSlots, hbmSlotCount;
    bool currentOpIsPcie;

    bool isStashAddr(Addr axiAddr);
    bool isHostSlot(uint32_t slotIdx);
    // Classify an AXI address into a metadata region for debug/accounting.
    // Returns a short tag string; counts the beats into the per-region totals.
    const char* metaRegionTag(Addr axiAddr);

    std::mt19937 rng;

    // === AXI bridge ===
    struct AxiSenderState : public Packet::SenderState
    {
        uint8_t axiId;
        int beatIdx, totalBeats;
        bool isWrite;
        size_t burstSeq;
        int secondBeatIdx = -1;  // >=0 when two 32B reads coalesced into 64B
        size_t secondBurstSeq = 0;
        AxiSenderState(uint8_t id, int idx, int total, bool wr, size_t seq)
            : axiId(id), beatIdx(idx), totalBeats(total),
              isWrite(wr), burstSeq(seq) {}
    };

    // pos_map read-modify-write: tagged on the RMW read so handleMemResp
    // can merge wdata and issue the timing write.
    struct PmRmwSenderState : public Packet::SenderState
    {
        Addr writeAddr;
        int writeSize;
        uint32_t wstrb;
        uint8_t wdata[32];
        uint8_t axiId;
        int beatIdx, totalBeats;
        size_t burstSeq;
        bool isPcie;
        PmRmwSenderState(Addr addr, int sz, uint32_t strb,
                          uint8_t id, int idx, int total,
                          size_t seq, bool pcie)
            : writeAddr(addr), writeSize(sz), wstrb(strb),
              axiId(id), beatIdx(idx), totalBeats(total),
              burstSeq(seq), isPcie(pcie) {
            memset(wdata, 0, sizeof(wdata));
        }
    };

    struct RBeat { uint8_t data[32] = {}; uint8_t id = 0; bool last = false; bool isSingle = false; };
    std::deque<RBeat> rQueue;

    struct ReadBurstReasm {
        uint8_t axiId; int totalBeats, beatsRecv, flushedBeats; size_t seq;
        std::vector<RBeat> beats; std::vector<bool> beatRecvd;
        ReadBurstReasm(uint8_t id, int total, size_t s)
            : axiId(id), totalBeats(total), beatsRecv(0), flushedBeats(0), seq(s),
              beats(total), beatRecvd(total, false) {}
    };
    std::deque<ReadBurstReasm> pendingReadBursts;
    size_t nextBurstSeq;
    void flushCompletedReads();

    struct PendingReadSend {
        Addr gem5Addr; int beatBytes;
        uint8_t axiId; int beatIdx, totalBeats;
        size_t seq; bool isPcie;
    };
    std::deque<PendingReadSend> pendingReadSends;

    struct BResp { uint8_t id; bool isHbm; bool isStash; };
    std::deque<BResp> bQueue;

    // HT SLOT write-forwarding shadow. The single-beat HT write commits to the
    // HBM backing store with BRESP-to-data latency; a lookup issued in a later
    // op can read the SLOT address before the prior write has functionally
    // landed, returning pre-write memory (0) -> false MISS. We record every HT
    // SLOT write's data keyed by gem5 address and forward it on HT SLOT reads,
    // guaranteeing read-your-writes regardless of memory commit latency.
    std::map<Addr, std::array<uint8_t, 32>> htSlotShadow;

    // pos_map write-shadow: same pattern as htSlotShadow. Tracks the
    // authoritative beat contents for each pos_map beat (16 entries/beat).
    // The PmRmw timing-read can return stale data under load; the merge
    // uses this shadow instead, so co-resident entries are never clobbered.
    std::map<Addr, std::array<uint8_t, 32>> pmShadow;

    // Stash data write-shadow: same pattern as pmShadow/htSlotShadow.
    // Tracks stash data beats written to HBM. When S_ST_LOAD reads a
    // stash entry back from HBM, the timing read may return stale data
    // (write not yet committed). The shadow provides authoritative data.
    std::map<Addr, std::array<uint8_t, 32>> stashDataShadow;

    // HT BKT_HEAD write-shadow: same RAW hazard as HT SLOT/PM.
    // INSERT/DELETE modify HEAD via read-modify-write; a subsequent
    // read before HBM commits gets stale data → chain corruption.
    std::map<Addr, std::array<uint8_t, 32>> htBktHeadShadow;

    // HT BKT_NEXT write-shadow: same pattern. INSERT writes
    // NEXT[idx] = old_head; if a later chain walk reads this before
    // HBM commits, it follows a stale pointer.
    std::map<Addr, std::array<uint8_t, 32>> htBktNextShadow;

    // Write data FIFO: models the AXI port write data buffer between
    // the RTL and HBM switch. W beats queue here, then drain to HBM
    // at 1 per tick (or as fast as HBM accepts).
    struct WFifoEntry {
        PacketPtr pkt;
        bool isPcie;
    };
    std::deque<WFifoEntry> wFifo;
    static constexpr int W_FIFO_DEPTH = 32;
    void drainWriteFifo();

    struct PendingWriteBurst {
        uint8_t id; int totalBeats, responsesRecv; size_t seq; bool isHbm; bool isStash;
    };
    std::deque<PendingWriteBurst> pendingWriteResps;
    std::unordered_map<size_t, unsigned> earlyWriteResps;  // burstSeq → count

    struct WriteBurst {
        Addr baseAddr;
        uint8_t len, size, id, burst;
        uint16_t beatsRecv;  // uint16_t: AXI4 supports up to 256 beats
        bool isPcie; bool isStash; size_t writeSeq;
    };
    std::deque<WriteBurst> activeWrites;

    static Addr axiBurstAddr(Addr start, int beatIdx, int beatBytes,
                              uint8_t burst, uint8_t len) {
        switch (burst) {
          case 0: return start;
          case 1: return start + beatIdx * beatBytes;
          case 2: { int w = (len+1)*beatBytes; Addr m = w-1;
                    return (start & ~m) + ((start + beatIdx*beatBytes) & m); }
          default: return start + beatIdx * beatBytes;
        }
    }

    std::deque<PacketPtr> hbmRetryQueue;
    std::deque<PacketPtr> pcieWriteRetryQ, pcieReadRetryQ;
    bool hbmBlocked, pcieBlocked;
    bool hbmBlockedSnapshot;  // captured at end of tick for next tick's ready signals

    // HBM write buffer: models Xilinx AXI SmartConnect internal FIFO.
    // Decouples RTL write rate from xbar/HBM acceptance rate.
    // RTL always sees wready=1. Buffer drains to xbar asynchronously.
    static const unsigned HBM_WRITE_BUF_DEPTH = 32;
    std::deque<PacketPtr> hbmWriteBuffer;
    void drainHbmWriteBuffer();

    // HBM read buffer: same pattern as write buffer.
    static const unsigned HBM_READ_BUF_DEPTH = 32;
    std::deque<PacketPtr> hbmReadBuffer;
    void drainHbmReadBuffer();
    RequestorID reqId;
    uint8_t prevRready, prevBready;

    void driveAxiR();
    void driveAxiB();
    void drainPendingSends();
    void handleMemResp(PacketPtr pkt);
    bool sendPkt(PacketPtr pkt, bool isPcie);
    void trySendRetries(bool isPcie);
    void checkRtlErrors();

    // === ORAM control ===
    enum class OramState {
        RESET, INIT_SLOTS, GRANT_LEASE, WRITE_INIT, IDLE, PROCESSING, DONE
    };
    OramState ctrlState;
    int initSlotIdx, initPhase, grantPhase, writeInitIdx;
    uint32_t leaseToken, numOps, opsCompleted;
    bool currentOpIsWrite;
    Addr currentOpAddr;
    uint32_t currentOpWdata[8];     // real write payload of in-flight op (for VERIFY-WRITE)
    uint32_t lastWrittenSlot;       // slot index of the last WRITE op
    std::set<uint32_t> writtenSlots; // slots that have been written at least once
    uint32_t lastWrittenData[64];   // data written by the last WRITE op

    // Step 3 additions (cpu_driven mode):
    //   leaseTokens[i] holds the token returned by the RTL when lease i
    //     was granted. Populated during GRANT_LEASE state when cpuDriven.
    //   currentGrantClient tracks which client is currently being granted
    //     (0..K-1 during the loop).
    //   ready is set when write-init completes — CPU polls this via MMIO
    //     before reading the token table.
    std::vector<uint32_t> leaseTokens;
    unsigned currentGrantClient;
    bool ready;

    // Step 4 gate (default-off opt-in for fast NVMe-only setup).
    // gatedStart is set from the Python parameter at construction.
    // gateReleased flips to true when CPU writes 1 to GATE_RELEASE_OFFSET.
    // When gatedStart=true and gateReleased=false, initOram() finishes
    // the post-reset cycles but does NOT call scheduleTick() — RTL
    // sits at INIT_SLOTS without consuming any per-cycle eval() cost.
    // The first tick is scheduled when handleCmdPortReq sees the
    // release write.
    bool gatedStart;
    bool gateReleased;

    void initNextSlot();
    void grantLease();
    void writeInitNextSlot();
    void generateNextOp();
    void completeOp();
    void feedClientWdata();

    uint64_t localOps, pcieOps, opStartCycle, totalOpCycles;
    uint64_t wStallCount;  // consecutive W-STALL cycles for deadlock detection
    uint64_t noProgressCount;  // consecutive cycles with no AXI progress

    // E2E latency aggregate counters (cpu_driven mode)
    uint64_t e2eOpsTracked;
    Tick     e2eRtlSum;        // sum of (rtlDoneTick - dispatchTick)
    Tick     e2eWritebackSum;  // sum of (commitTick - rtlDoneTick)
    Tick     e2eColdStart;     // first op's full E2E (commitTick - fetchTick)
    bool statsPrinted;
    bool tagMismatchWarned;  // only warn once for tag mismatch
    uint8_t prevPmState;     // previous pos_map FSM state

    // DDR_WRITE debug: cycle-level breakdown of W channel behavior
    struct WriteDebug {
        uint64_t wHandshakes = 0;    // wvalid && wready (RTL thinks beat sent)
        uint64_t wStalls = 0;        // wvalid && !wready (backpressure)
        uint64_t wIdle = 0;          // !wvalid (gap between sub-bursts)
        uint64_t sendAccepted = 0;   // sendTimingReq returned true
        uint64_t sendRejected = 0;   // sendTimingReq returned false (queued)
        uint64_t retrySent = 0;      // sent via retry queue
        uint64_t awHandshakes = 0;   // awvalid && awready
        uint64_t brespRecv = 0;      // bvalid && bready (RTL consumed BRESP)
        uint64_t brespAvail = 0;     // bvalid asserted (BRESP available)
        uint64_t brespNoMatch = 0;   // bQueue non-empty but no type match
        uint64_t totalCycles = 0;    // cycles in DDR_WRITE state
        uint64_t firstWcycle = 0;    // first W handshake cycle
        uint64_t lastWcycle = 0;     // last W handshake cycle
        uint64_t firstBcycle = 0;    // first BRESP consumed cycle
        uint64_t lastBcycle = 0;     // last BRESP consumed cycle
        uint64_t maxBqDepth = 0;     // peak bQueue depth during DDR_WRITE
        void reset() { wHandshakes = wStalls = wIdle = 0;
                        sendAccepted = sendRejected = retrySent = 0;
                        awHandshakes = brespRecv = brespAvail = brespNoMatch = 0;
                        totalCycles = firstWcycle = lastWcycle = 0;
                        firstBcycle = lastBcycle = maxBqDepth = 0; }
    } wrDbg;

    // DDR_READ debug: cycle-level breakdown of R channel behavior
    struct ReadDebug {
        uint64_t rHandshakes = 0;    // rvalid && rready (RTL consumed beat)
        uint64_t rStalls = 0;        // rvalid && !rready (device not ready)
        uint64_t rIdle = 0;          // !rvalid (waiting for data)
        uint64_t arHandshakes = 0;   // arvalid && arready
        uint64_t sendAccepted = 0;   // read sendPkt returned true
        uint64_t sendRejected = 0;   // read sendPkt returned false (queued)
        uint64_t retrySent = 0;      // read sent via retry queue
        uint64_t memResps = 0;       // handleMemResp read responses
        uint64_t totalCycles = 0;    // cycles in DDR_READ state
        uint64_t firstRcycle = 0;    // first R handshake cycle
        uint64_t lastRcycle = 0;     // last R handshake cycle
        uint64_t maxRqDepth = 0;     // peak rQueue depth during DDR_READ
        uint64_t maxPendReads = 0;   // peak pendingReadBursts during DDR_READ
        void reset() { rHandshakes = rStalls = rIdle = 0;
                        arHandshakes = sendAccepted = sendRejected = 0;
                        retrySent = memResps = totalCycles = 0;
                        firstRcycle = lastRcycle = 0;
                        maxRqDepth = maxPendReads = 0; }
    } rdDbg;

    // Per-phase cycle counters (accumulated across all ops)
    // FSM states from flat_oram_gcm.v
    static constexpr int NUM_FSM_STATES = 31;
    uint64_t phaseCycles[NUM_FSM_STATES];
    uint64_t opPhaseCycles[NUM_FSM_STATES]; // per-op accumulator
    uint8_t  prevFsmState;

    // Stash routing verification
    uint64_t stashHbmBeats, stashPcieBeats;
    uint64_t bucketHbmBeats, bucketPcieBeats;
    // Per-metadata-region beat counters (all should be HBM-only).
    uint64_t ivtBeats, slotrBeats, bmetaBeats, pmBeats, htBeats;

    void printStats();
};

} // namespace gem5
#endif
