#pragma once

#include <Arduino.h>

#include "async_event_loop/pin_event.hpp"
#include "async_event_loop_arduino/arduino_interrupt_guard.hpp"

namespace async_event_loop {

struct ArduinoInterruptEventInfo {
    uint32_t source_id = 0;
    uint16_t count = 0;
    uint16_t dropped = 0;
    uint32_t last_timestamp_us = 0;
};

/**
 * Interrupt-backed Arduino pin event source.
 *
 * The ISR calls notify_from_isr() only. EventLoop::on_pin_event() later drains
 * the pending count from normal loop context and invokes the user callback
 * outside ISR context.
 */
class ArduinoInterruptPinEventSource final : public IPinEventSource {
public:
    ArduinoInterruptPinEventSource(uint8_t pin,
                                   PinEventType event_type = PinEventType::Change,
                                   uint32_t source_id = 0)
        : pin_(pin), event_type_(event_type), source_id_(source_id == 0 ? pin : source_id) {}

    bool valid() const override { return true; }

    void notify_from_isr(bool level = false) {
        if (pending_count_ == 0xffffu) {
            ++dropped_count_;
            return;
        }
        ++pending_count_;
        last_timestamp_us_ = micros();
        last_level_ = level;
        has_level_ = true;
    }

    bool read_event(PinEvent& event) override {
        uint16_t count = 0;
        uint16_t dropped = 0;
        uint32_t timestamp = 0;
        bool level = false;
        bool has_level = false;

        {
            ArduinoInterruptGuard guard;
            if (pending_count_ == 0) {
                return false;
            }
            count = pending_count_;
            dropped = dropped_count_;
            timestamp = last_timestamp_us_;
            level = last_level_;
            has_level = has_level_;
            pending_count_ = 0;
            dropped_count_ = 0;
        }

        last_info_.source_id = source_id_;
        last_info_.count = count;
        last_info_.dropped = dropped;
        last_info_.last_timestamp_us = timestamp;

        event.type = event_type_;
        event.timestamp_us = timestamp;
        event.source_id = source_id_;
        event.sequence = ++sequence_;
        event.level = has_level ? level : false;
        return true;
    }

    const ArduinoInterruptEventInfo& last_info() const { return last_info_; }
    uint8_t pin() const { return pin_; }
    uint32_t source_id() const { return source_id_; }

private:
    uint8_t pin_ = 0;
    PinEventType event_type_ = PinEventType::Change;
    uint32_t source_id_ = 0;
    uint32_t sequence_ = 0;
    ArduinoInterruptEventInfo last_info_;

    volatile uint16_t pending_count_ = 0;
    volatile uint16_t dropped_count_ = 0;
    volatile uint32_t last_timestamp_us_ = 0;
    volatile bool last_level_ = false;
    volatile bool has_level_ = false;
};

} // namespace async_event_loop
