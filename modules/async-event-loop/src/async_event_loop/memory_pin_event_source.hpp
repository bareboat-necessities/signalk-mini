#pragma once

#include <stddef.h>

#include "pin_io.hpp"

namespace async_event_loop {

template<size_t Capacity>
class MemoryPinEventSource final : public IPinEventSource {
public:
    bool valid() const override { return true; }

    bool push(const PinEvent& event) {
        if (count_ >= Capacity) {
            return false;
        }
        events_[tail_] = event;
        tail_ = (tail_ + 1) % Capacity;
        ++count_;
        return true;
    }

    bool read_event(PinEvent& event) override {
        if (count_ == 0) {
            return false;
        }
        event = events_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }

    size_t size() const { return count_; }
    size_t capacity() const { return Capacity; }
    size_t available_for_write() const { return Capacity - count_; }
    bool empty() const { return count_ == 0; }
    bool full() const { return count_ == Capacity; }

    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    PinEvent events_[Capacity]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
};

} // namespace async_event_loop
