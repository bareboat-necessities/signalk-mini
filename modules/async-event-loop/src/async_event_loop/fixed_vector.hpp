#pragma once

#include <assert.h>
#include <stddef.h>
#include <type_traits>

namespace async_event_loop {

/**
 * Small fixed-capacity vector for trivially managed element types.
 *
 * This container intentionally avoids placement-new/destructor bookkeeping for
 * Arduino-size builds. It is only valid for types that can be overwritten,
 * shifted, and logically erased without running per-element destructors.
 */
template<typename T, size_t Capacity>
class FixedVector final {
    static_assert(std::is_trivially_destructible<T>::value,
                  "FixedVector requires trivially destructible element types");
    static_assert(std::is_trivially_copy_assignable<T>::value,
                  "FixedVector requires trivially copy assignable element types");

public:
    bool push_back(const T& value) {
        if (size_ >= Capacity) {
            return false;
        }
        items_[size_++] = value;
        return true;
    }

    bool erase(size_t index) {
        if (index >= size_) {
            return false;
        }
        for (size_t i = index + 1; i < size_; ++i) {
            items_[i - 1] = items_[i];
        }
        --size_;
        return true;
    }

    bool pop_back() {
        if (size_ == 0) {
            return false;
        }
        --size_;
        return true;
    }

    void clear() { size_ = 0; }
    size_t size() const { return size_; }
    size_t capacity() const { return Capacity; }
    bool empty() const { return size_ == 0; }
    bool full() const { return size_ >= Capacity; }

    T& operator[](size_t index) {
        assert(index < size_);
        return items_[index];
    }

    const T& operator[](size_t index) const {
        assert(index < size_);
        return items_[index];
    }

private:
    T items_[Capacity]{};
    size_t size_ = 0;
};

} // namespace async_event_loop
