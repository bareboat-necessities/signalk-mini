#pragma once

#if defined(ARDUINO)
#include "async_event_loop_arduino/arduino_pin_io.hpp"
#endif

#if defined(__linux__)
#include "async_event_loop_linux/linux_file_pin_io.hpp"
#if defined(ASYNC_EVENT_LOOP_ENABLE_LINUX_GPIOD_PIN_IO)
#include "async_event_loop_linux/linux_gpiod_pin_io.hpp"
#endif
#endif

namespace async_event_loop {

#if defined(ARDUINO)
using NativeDigitalInputPin = ArduinoDigitalInputPin;
using NativeDigitalOutputPin = ArduinoDigitalOutputPin;
using NativeAnalogInputPin = ArduinoAnalogInputPin;
using NativeAnalogOutputPin = ArduinoAnalogOutputPin;
#endif

#if defined(__linux__)
#if defined(ASYNC_EVENT_LOOP_ENABLE_LINUX_GPIOD_PIN_IO)
using NativeDigitalInputPin = LinuxGpiodDigitalInputPin;
using NativeDigitalOutputPin = LinuxGpiodDigitalOutputPin;
#else
using NativeDigitalInputPin = LinuxFileDigitalInputPin;
using NativeDigitalOutputPin = LinuxFileDigitalOutputPin;
#endif
using NativeAnalogInputPin = LinuxFileAnalogInputPin;
using NativeAnalogOutputPin = LinuxFileAnalogOutputPin;
#endif

} // namespace async_event_loop
