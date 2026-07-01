#pragma once

#include <stdint.h>

namespace async_event_loop {

enum class DigitalPinMode : uint8_t {
    Input,
    InputPullup,
    Output
};

class IDigitalInputPin {
public:
    virtual ~IDigitalInputPin() = default;
    virtual bool valid() const = 0;
    virtual bool read() = 0;
};

class IDigitalOutputPin {
public:
    virtual ~IDigitalOutputPin() = default;
    virtual bool valid() const = 0;
    virtual bool write(bool high) = 0;
};

class IAnalogInputPin {
public:
    virtual ~IAnalogInputPin() = default;
    virtual bool valid() const = 0;
    virtual int read() = 0;
    virtual int max_value() const = 0;
};

class IAnalogOutputPin {
public:
    virtual ~IAnalogOutputPin() = default;
    virtual bool valid() const = 0;
    virtual bool write(int value) = 0;
    virtual int max_value() const = 0;
};

/** Digital pin event modes. */
enum class PinEventType : uint8_t {
    RisingEdge,
    FallingEdge,
    Change,
    HighLevel,
    LowLevel
};

/** Normalized pin event record produced by IPinEventSource. */
struct PinEvent {
    PinEventType type = PinEventType::Change;
    uint64_t timestamp_us = 0;
    uint32_t source_id = 0;
    uint32_t sequence = 0;
    bool level = false;
};

/**
 * Abstract edge/state event source for GPIO-like inputs.
 *
 * Linux gpiod sources return a native fd and are integrated with libevent.
 * Arduino interrupt/cooperative sources return -1 and are checked from tick().
 * read_event() drains one pending event and returns true when one was available.
 */
class IPinEventSource {
public:
    virtual ~IPinEventSource() = default;
    virtual bool valid() const = 0;
    virtual int native_fd() const { return -1; }
    virtual bool read_event(PinEvent& event) = 0;
};

} // namespace async_event_loop
