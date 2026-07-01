#pragma once

#include <Arduino.h>
#include "async_event_loop/pin_event.hpp"

namespace async_event_loop {

/**
 * Cooperative Arduino digital pin event source.
 *
 * This implementation has no native fd. EventLoop::on_pin_event() polls it from
 * tick() and emits normalized PinEvent records when the selected edge/level
 * condition is observed.
 */
class ArduinoDigitalPinEventSource final : public IPinEventSource {
public:
    ArduinoDigitalPinEventSource(uint8_t pin,
                                 PinEventType event_type = PinEventType::Change,
                                 uint8_t mode = INPUT)
        : pin_(pin), event_type_(event_type), mode_(mode) {}

    void begin() {
        pinMode(pin_, mode_);
        ready_ = true;
    }

    bool valid() const override { return ready_; }

    bool read_event(PinEvent& event) override {
        if (!ready_) {
            return false;
        }

        const bool current = digitalRead(pin_) == HIGH;
        if (!has_last_) {
            last_level_ = current;
            has_last_ = true;
            return level_event(current, event);
        }

        const bool previous = last_level_;
        last_level_ = current;

        if (!matches(previous, current)) {
            return false;
        }

        event.type = edge_type(previous, current);
        event.timestamp_us = static_cast<uint64_t>(micros());
        event.source_id = pin_;
        event.sequence = ++sequence_;
        event.level = current;
        return true;
    }

    uint8_t pin() const { return pin_; }

private:
    bool level_event(bool current, PinEvent& event) {
        if ((event_type_ == PinEventType::HighLevel && current) ||
            (event_type_ == PinEventType::LowLevel && !current)) {
            event.type = event_type_;
            event.timestamp_us = static_cast<uint64_t>(micros());
            event.source_id = pin_;
            event.sequence = ++sequence_;
            event.level = current;
            return true;
        }
        return false;
    }

    bool matches(bool previous, bool current) const {
        switch (event_type_) {
        case PinEventType::RisingEdge:
            return !previous && current;
        case PinEventType::FallingEdge:
            return previous && !current;
        case PinEventType::Change:
            return previous != current;
        case PinEventType::HighLevel:
            return current;
        case PinEventType::LowLevel:
            return !current;
        }
        return false;
    }

    PinEventType edge_type(bool previous, bool current) const {
        if (!previous && current) {
            return PinEventType::RisingEdge;
        }
        if (previous && !current) {
            return PinEventType::FallingEdge;
        }
        return event_type_;
    }

    uint8_t pin_ = 0;
    PinEventType event_type_ = PinEventType::Change;
    uint8_t mode_ = INPUT;
    uint32_t sequence_ = 0;
    bool ready_ = false;
    bool has_last_ = false;
    bool last_level_ = false;
};

} // namespace async_event_loop
