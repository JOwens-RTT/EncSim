#include "IntervalTimerEx.h"

IntervalTimerEx::~IntervalTimerEx()
{
    end();
}

void IntervalTimerEx::end()
{
    callbacks[index] = nullptr;
    IntervalTimer::end();
}

// generate and preset the callback storage
IntervalTimerEx::callback_t IntervalTimerEx::callbacks[4]{
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};

#if defined(USE_CPP11_CALLBACKS)

IntervalTimerEx::relay_t IntervalTimerEx::relays[4]{
    [] { callbacks[0](); },
    [] { callbacks[1](); },
    [] { callbacks[2](); },
    [] { callbacks[3](); },
};

#else

// generate the static array of relay functions
IntervalTimerEx::relay_t IntervalTimerEx::relays[4]{
    [] { callbacks[0](states[0]); },
    [] { callbacks[1](states[1]); },
    [] { callbacks[2](states[2]); },
    [] { callbacks[3](states[3]); },
};

#endif