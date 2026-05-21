/*
 * ssd_engine.hh — SimpleSSD <-> gem5 time bridge
 *
 * When SimpleSSD schedules an internal event (e.g. NVMe controller
 * work(), GC, write buffer flush), this engine creates a corresponding
 * gem5 event at the same tick. When the gem5 event fires, it calls
 * the SimpleSSD callback function.
 *
 * This is critical for NVMe mode where the controller is fully async.
 * For CXL-SSD (synchronous pHIL->read()), the events are optional
 * (only needed for background GC).
 */

#ifndef __MEM_SSD_SSD_ENGINE_HH__
#define __MEM_SSD_SSD_ENGINE_HH__

#include <list>
#include <unordered_map>

#include "sim/eventq.hh"          // gem5 EventManager, EventFunctionWrapper
#include "sim/simulator.hh"       // SimpleSSD's Simulator base class

namespace gem5
{

class SsdEngine : public SimpleSSD::Simulator
{
  private:
    SimpleSSD::Event counter;
    std::unordered_map<SimpleSSD::Event, SimpleSSD::EventFunction> eventList;
    std::list<std::pair<SimpleSSD::Event, uint64_t>> eventQueue;

    // gem5 event manager for scheduling real gem5 events
    EventManager *eventManager;

    // Map SimpleSSD event ID → gem5 EventFunctionWrapper
    std::unordered_map<SimpleSSD::Event, EventFunctionWrapper*> gem5Events;

    // Fire a SimpleSSD event callback
    void fireEvent(SimpleSSD::Event eid);

    bool insertEvent(SimpleSSD::Event eid, uint64_t tick,
                     uint64_t *pOldTick = nullptr);
    bool removeEvent(SimpleSSD::Event eid);
    bool isEventExist(SimpleSSD::Event eid,
                      uint64_t *pTick = nullptr);

  public:
    SsdEngine();
    ~SsdEngine();

    // Must be called before any events are scheduled.
    // Pass the owning SimObject (NvmeSsdDevice or SsdMemory).
    void setEventManager(EventManager *em) { eventManager = em; }

    uint64_t getCurrentTick() override;

    SimpleSSD::Event allocateEvent(SimpleSSD::EventFunction func) override;
    void scheduleEvent(SimpleSSD::Event eid, uint64_t tick) override;
    void descheduleEvent(SimpleSSD::Event eid) override;
    bool isScheduled(SimpleSSD::Event eid,
                     uint64_t *pTick = nullptr) override;
    void deallocateEvent(SimpleSSD::Event eid) override;
};

// Initialize SimpleSSD: set simulator pointer, init logging, parse config.
SimpleSSD::ConfigReader initSimpleSSD(SsdEngine *engine,
                                      const std::string &configPath);

} // namespace gem5

#endif // __MEM_SSD_SSD_ENGINE_HH__
