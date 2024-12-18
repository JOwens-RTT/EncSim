#pragma once

#include "Arduino.h"

#if defined(__cplusplus) && __cplusplus >= 201103L
#    define USE_CPP11_CALLBACKS // comment out if you want to use the traditional void pointer pattern to pass state to callbacks
#endif                          // __cplusplus

#if defined(USE_CPP11_CALLBACKS)
#    include <functional>
#endif

class IntervalTimerEx : public IntervalTimer
{
 public:
#if defined(USE_CPP11_CALLBACKS)
    using callback_t = std::function<void()>;
#else
    using callback_t = void (*)(void*);
#endif
    using relay_t = void (*)();

 public:
    template <typename period_t> // begin is implemented as template to avoid replication the various versions of IntervalTimer::begin
#if defined(USE_CPP11_CALLBACKS)
    bool begin(callback_t callback, period_t period);
#else
    bool begin(callback_t callback, void* state, period_t period);
#endif

    void end();
    ~IntervalTimerEx();

 protected:
    unsigned index = 0;
    static callback_t callbacks[4]; // storage for callbacks
    static relay_t relays[4];       // storage for relay functions
#if !defined(USE_CPP11_CALLBACKS)
    static void* states[4]; // storage for state variables
#endif
};

// Inline implementation ===============================================

template <typename period_t>
bool IntervalTimerEx::begin(callback_t callback, period_t period)
{
    for (index = 0; index < 4; index++) // find the next free slot
    {
        if (callbacks[index] == nullptr) // ->free slot
        {
            if (IntervalTimer::begin(relays[index], period)) // we got a slot but we need to also get an actual timer
            {
                callbacks[index] = callback; // if ok -> store callback
#if !defined(USE_CPP11_CALLBACKS)
                states[index] = state; // ...and state
#endif
                return true;
            }
            return false;
        }
    }
    return false; // can never happen if bookkeeping is ok
}