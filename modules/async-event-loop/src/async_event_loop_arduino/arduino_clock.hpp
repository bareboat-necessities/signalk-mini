#pragma once

#include <Arduino.h>
#include "async_event_loop/scheduler.hpp"
#include "async_event_loop/micros32_extender.hpp"

namespace async_event_loop {

class ArduinoClock final : public IClock {
public:
    uint64_t micros() const override {
        return extender_.update(static_cast<uint32_t>(::micros()));
    }

private:
    mutable Micros32Extender extender_{};
};

} // namespace async_event_loop
