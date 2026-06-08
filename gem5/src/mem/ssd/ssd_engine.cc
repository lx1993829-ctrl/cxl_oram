/*
 * ssd_engine.cc — SimpleSSD <-> gem5 time bridge
 *
 * DIAGNOSTIC BUILD: panic if scheduleEvent is called before
 * setEventManager (catches init-order bugs), null-safe destructor,
 * boundary DPRINTFs.
 */

#include "mem/ssd/ssd_engine.hh"

#include "base/logging.hh"       // gem5 panic()
#include "base/trace.hh"
#include "debug/SsdMemory.hh"
#include "sim/cur_tick.hh"       // gem5 curTick()

// SimpleSSD headers
#include "sim/trace.hh"          // SimpleSSD panic, warn
#include "sim/config_reader.hh"
#include "sim/cpu.hh"
#include "sim/log.hh"
#include <iostream>

namespace gem5
{

// ====================================================================
//  SsdEngine
// ====================================================================

SsdEngine::SsdEngine()
    : SimpleSSD::Simulator(), counter(0), eventManager(nullptr)
{
}

SsdEngine::~SsdEngine()
{
    // Null-safe cleanup — avoid crashing during teardown if
    // setEventManager was never called.
    for (auto &pair : gem5Events) {
        if (pair.second) {
            if (eventManager && pair.second->scheduled()) {
                eventManager->deschedule(pair.second);
            }
            delete pair.second;
        }
    }
    gem5Events.clear();
}

uint64_t
SsdEngine::getCurrentTick()
{
    return curTick();
}

// Fire a SimpleSSD event callback when the gem5 event triggers
void
SsdEngine::fireEvent(SimpleSSD::Event eid)
{
    auto iter = eventList.find(eid);
    if (iter != eventList.end()) {
        DPRINTF(SsdMemory, "[FIRE_EVENT] eid=%lu\n", eid);
        // Remove from our event queue
        removeEvent(eid);

        // Call the SimpleSSD callback
        iter->second(curTick());
        DPRINTF(SsdMemory, "[FIRE_EVENT done] eid=%lu\n", eid);
    } else {
        warn("SsdEngine::fireEvent: eid=%lu not in eventList "
             "(double-fire?)", eid);
    }
}

bool
SsdEngine::insertEvent(SimpleSSD::Event eid, uint64_t tick,
                       uint64_t *pOldTick)
{
    bool found = false;
    bool flag = false;
    auto old = eventQueue.begin();
    auto insert = eventQueue.end();

    for (auto iter = eventQueue.begin(); iter != eventQueue.end(); iter++) {
        if (iter->first == eid) {
            found = true;
            old = iter;
            if (pOldTick) {
                *pOldTick = iter->second;
            }
        }
        if (iter->second > tick && !flag) {
            insert = iter;
            flag = true;
        }
    }

    if (found && pOldTick) {
        if (*pOldTick == tick) {
            return false;
        }
    }

    eventQueue.insert(insert, {eid, tick});

    if (found) {
        eventQueue.erase(old);
    }

    return found;
}

bool
SsdEngine::removeEvent(SimpleSSD::Event eid)
{
    for (auto iter = eventQueue.begin(); iter != eventQueue.end(); iter++) {
        if (iter->first == eid) {
            eventQueue.erase(iter);
            return true;
        }
    }
    return false;
}

bool
SsdEngine::isEventExist(SimpleSSD::Event eid, uint64_t *pTick)
{
    for (auto &iter : eventQueue) {
        if (iter.first == eid) {
            if (pTick) {
                *pTick = iter.second;
            }
            return true;
        }
    }
    return false;
}

SimpleSSD::Event
SsdEngine::allocateEvent(SimpleSSD::EventFunction func)
{
    auto iter = eventList.insert({++counter, func});

    if (!iter.second) {
        SimpleSSD::panic("SsdEngine: failed to allocate event");
    }

    DPRINTF(SsdMemory, "[ALLOC_EVENT] eid=%lu\n", counter);
    return counter;
}

void
SsdEngine::scheduleEvent(SimpleSSD::Event eid, uint64_t tick)
{
    auto iter = eventList.find(eid);

    if (iter == eventList.end()) {
        SimpleSSD::panic(
            "SsdEngine: event %" PRIu64 " does not exist", eid);
        return;
    }

    uint64_t now = getCurrentTick();
    if (tick < now) {
        tick = now;
    }

    uint64_t oldTick;
    insertEvent(eid, tick, &oldTick);

    // FIX #2: panic instead of silently no-op if eventManager isn't
    // hooked up. This catches init-order bugs at the moment they
    // happen.
    if (!eventManager) {
        panic("SsdEngine::scheduleEvent: eventManager is null! "
              "(eid=%lu tick=%lu) — setEventManager() must be called "
              "BEFORE initSimpleSSD()", eid, tick);
    }

    auto gem5It = gem5Events.find(eid);
    if (gem5It == gem5Events.end()) {
        // Create new gem5 event for this SimpleSSD event
        auto *ev = new EventFunctionWrapper(
            [this, eid]{ fireEvent(eid); },
            "SimpleSSD_event", false /* don't auto-delete */);
        gem5Events[eid] = ev;
        eventManager->schedule(ev, tick);
        DPRINTF(SsdMemory, "[SCHED_EVENT new] eid=%lu tick=%lu\n",
                eid, tick);
    } else {
        // Reschedule existing gem5 event
        EventFunctionWrapper *ev = gem5It->second;
        if (ev->scheduled()) {
            eventManager->reschedule(ev, tick);
        } else {
            eventManager->schedule(ev, tick);
        }
        DPRINTF(SsdMemory, "[SCHED_EVENT re] eid=%lu tick=%lu\n",
                eid, tick);
    }
}

void
SsdEngine::descheduleEvent(SimpleSSD::Event eid)
{
    auto iter = eventList.find(eid);

    if (iter == eventList.end()) {
        SimpleSSD::panic(
            "SsdEngine: event %" PRIu64 " does not exist", eid);
        return;
    }

    removeEvent(eid);

    if (eventManager) {
        auto gem5It = gem5Events.find(eid);
        if (gem5It != gem5Events.end() && gem5It->second->scheduled()) {
            eventManager->deschedule(gem5It->second);
        }
    }
}

bool
SsdEngine::isScheduled(SimpleSSD::Event eid, uint64_t *pTick)
{
    auto iter = eventList.find(eid);

    if (iter == eventList.end()) {
        SimpleSSD::panic(
            "SsdEngine: event %" PRIu64 " does not exist", eid);
    }

    return isEventExist(eid, pTick);
}

void
SsdEngine::deallocateEvent(SimpleSSD::Event eid)
{
    auto iter = eventList.find(eid);

    if (iter == eventList.end()) {
        SimpleSSD::panic(
            "SsdEngine: event %" PRIu64 " does not exist", eid);
        return;
    }

    removeEvent(eid);

    if (eventManager) {
        auto gem5It = gem5Events.find(eid);
        if (gem5It != gem5Events.end()) {
            if (gem5It->second->scheduled()) {
                eventManager->deschedule(gem5It->second);
            }
            delete gem5It->second;
            gem5Events.erase(gem5It);
        }
    }

    eventList.erase(iter);
}

// ====================================================================
//  Initialization helper
// ====================================================================

SimpleSSD::ConfigReader
initSimpleSSD(SsdEngine *engine, const std::string &configPath)
{
    SimpleSSD::ConfigReader conf;

    SimpleSSD::setSimulator(engine);
    SimpleSSD::initLogSystem(&std::cerr, &std::cerr);

    if (!conf.init(configPath)) {
        panic("SimpleSSD: failed to open config file '%s'",
              configPath.c_str());
    }

    SimpleSSD::initCPU(conf);

    return conf;
}

} // namespace gem5