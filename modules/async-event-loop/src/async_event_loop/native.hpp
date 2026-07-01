#pragma once

#include <stddef.h>

#if defined(ARDUINO)
#include "async_event_loop_arduino/arduino_clock.hpp"
#include "async_event_loop_arduino/arduino_loop.hpp"

namespace async_event_loop {
using NativeClock = ArduinoClock;
using NativeScheduler = ArduinoLoop;

template<size_t MaxCallbacks>
using NativeSchedulerFor = ArduinoLoopImpl<MaxCallbacks>;
} // namespace async_event_loop

#elif defined(__linux__)
#include "async_event_loop_linux/monotonic_clock.hpp"
#include "async_event_loop_linux/libevent_loop.hpp"

namespace async_event_loop {
using NativeClock = LinuxMonotonicClock;
using NativeScheduler = LinuxLibeventLoop;

/** N is unused on Linux; scheduler capacity is unbounded and heap-backed. */
template<size_t>
using NativeSchedulerFor = LinuxLibeventLoop;
} // namespace async_event_loop

#else
#error "Unsupported pypilot-event-loop platform: define ARDUINO or build on Linux"
#endif
