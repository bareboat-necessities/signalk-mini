#pragma once

#include <stdint.h>

namespace async_event_loop {

/**
 * Extend a wrapping 32-bit microsecond counter into a monotonic 64-bit value.
 *
 * This is intended for Arduino-style micros() implementations that expose a
 * uint32_t counter. The caller must sample at least once per 2^32 microseconds
 * rollover interval; if more than one full rollover happens between samples, no
 * software-only extender can infer the missed wraps from a single 32-bit value.
 */
class Micros32Extender final {
public:
    uint64_t update(uint32_t raw_us) {
        if (!initialized_) {
            initialized_ = true;
            last_raw_us_ = raw_us;
            extended_us_ = raw_us;
            return extended_us_;
        }

        const uint32_t delta = static_cast<uint32_t>(raw_us - last_raw_us_);
        extended_us_ += static_cast<uint64_t>(delta);
        last_raw_us_ = raw_us;
        return extended_us_;
    }

    void reset(uint32_t raw_us = 0) {
        initialized_ = true;
        last_raw_us_ = raw_us;
        extended_us_ = raw_us;
    }

    bool initialized() const { return initialized_; }
    uint32_t last_raw_micros() const { return last_raw_us_; }
    uint64_t extended_micros() const { return extended_us_; }

private:
    uint64_t extended_us_ = 0;
    uint32_t last_raw_us_ = 0;
    bool initialized_ = false;
};

} // namespace async_event_loop
