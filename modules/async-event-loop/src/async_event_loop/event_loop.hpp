#pragma once

#include <stddef.h>
#include <stdint.h>

#include "native.hpp"
#include "pin_io.hpp"
#include "scheduler.hpp"

#ifndef ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS
#if defined(ARDUINO)
#define ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS 32
#else
#define ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS 128
#endif
#endif

#ifndef ASYNC_EVENT_LOOP_DEFAULT_CALLBACK_STORAGE_SIZE
#define ASYNC_EVENT_LOOP_DEFAULT_CALLBACK_STORAGE_SIZE 64
#endif

namespace async_event_loop {

/**
 * Portable byte-oriented stream interface.
 *
 * This is the single readiness interface used by EventLoop. Ordinary streams
 * and datagram transports both implement it. Datagram implementations return
 * true from is_datagram(); their read/write operations receive or send one
 * datagram at a time according to the concrete transport's rules.
 *
 * Linux fd-backed implementations return a non-negative native_fd() so the
 * native scheduler can use fd readiness. Cooperative backends return -1 and
 * are polled from tick()/run_once().
 */
class IByteStream {
public:
    virtual ~IByteStream() = default;

    virtual int read(uint8_t* dst, size_t max_len) = 0;
    virtual int write(const uint8_t* src, size_t len) = 0;

    virtual bool readable() const = 0;
    virtual bool writable() const = 0;
    virtual bool valid() const = 0;

    virtual bool is_datagram() const { return false; }
    virtual int native_fd() const { return -1; }
};

/**
 * Stable handle for an event-loop callback slot.
 *
 * The generation field prevents stale handles from controlling a later callback
 * after a removed slot has been reused.
 */
struct EventHandle {
    static constexpr uint16_t invalid_slot = 0xffffu;

    constexpr EventHandle() = default;
    constexpr EventHandle(uint16_t slot_value, uint16_t generation_value)
        : slot(slot_value), generation(generation_value) {}

    uint16_t slot = invalid_slot;
    uint16_t generation = 0;

    bool assigned() const { return slot != invalid_slot; }
    explicit operator bool() const { return assigned(); }
};

/**
 * Application-facing event-loop facade.
 *
 * EventLoop owns the native clock and scheduler selected by native.hpp. On
 * Linux this is the libevent scheduler. On Arduino this is the cooperative
 * scheduler. Callback storage is fixed-size to avoid std::function and heap
 * allocation in small targets.
 */
template<size_t MaxCallbacks = ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS,
         size_t CallbackStorageSize = ASYNC_EVENT_LOOP_DEFAULT_CALLBACK_STORAGE_SIZE>
class EventLoop {
public:
    EventLoop() : scheduler_(clock_) {
        static_assert(MaxCallbacks <= 0xfffeu, "EventLoop supports at most 65534 callback slots");
    }

    /** Default cooperative readiness check period for streams or pins without fd readiness. */
    static constexpr uint64_t default_readiness_poll_us = 1000ULL;

    /** Return true when the selected platform backend initialized correctly. */
    bool valid() const { return scheduler_.valid(); }

    /** Return true when an event handle still names an active slot in this loop. */
    bool valid(EventHandle handle) const {
        return handle.assigned() &&
               handle.slot < MaxCallbacks &&
               callback_used_[handle.slot] &&
               callback_tasks_[handle.slot].active() &&
               callback_generations_[handle.slot] == handle.generation;
    }

    /** Enable a previously registered callback slot. */
    bool enable(EventHandle handle) {
        if (!valid(handle)) {
            return false;
        }
        callback_tasks_[handle.slot].set_enabled(true);
        return true;
    }

    /** Disable a callback slot without destroying its stored callable. */
    bool disable(EventHandle handle) {
        if (!valid(handle)) {
            return false;
        }
        callback_tasks_[handle.slot].set_enabled(false);
        return true;
    }

    /** Remove a callback slot, unschedule its backend task, and make the slot reusable. */
    bool remove(EventHandle handle) {
        if (!valid(handle)) {
            return false;
        }
        if (!scheduler_.remove(callback_tasks_[handle.slot])) {
            return false;
        }
        release_registered(handle);
        return true;
    }

    /** Register a runtime task that repeats every period_us microseconds. */
    bool add_periodic(IRuntimeTask& task, uint64_t period_us) {
        return scheduler_.add_periodic(task, period_us);
    }

    /** Register a runtime task that runs once at absolute due_us. */
    bool add_one_shot(IRuntimeTask& task, uint64_t due_us) {
        return scheduler_.add_one_shot(task, due_us);
    }

    /** Register a callable that repeats every period_us microseconds. */
    template<typename Callable>
    EventHandle on_repeat_us(uint64_t period_us, Callable callable) {
        const EventHandle handle = allocate_callback(callable);
        if (!handle.assigned()) {
            return EventHandle{};
        }
        if (!scheduler_.add_periodic(callback_tasks_[handle.slot], period_us)) {
            release_unregistered(handle);
            return EventHandle{};
        }
        return handle;
    }

    /** Register a callable that runs once after delay_us microseconds. */
    template<typename Callable>
    EventHandle on_delay_us(uint64_t delay_us, Callable callable) {
        for (size_t i = 0; i < MaxCallbacks; ++i) {
            if (!callback_used_[i]) {
                const EventHandle handle{static_cast<uint16_t>(i), callback_generations_[i]};
                const bool set_ok = callback_tasks_[i].set(
                    OneShotCallback<Callable>(this, handle, static_cast<Callable&&>(callable)));
                if (!set_ok) {
                    return EventHandle{};
                }
                callback_used_[i] = true;
                const uint64_t now_us = clock_.micros();
                const uint64_t due_us = delay_us > UINT64_MAX - now_us ? UINT64_MAX : now_us + delay_us;
                if (!scheduler_.add_one_shot(callback_tasks_[i], due_us)) {
                    release_unregistered(handle);
                    return EventHandle{};
                }
                return handle;
            }
        }
        return EventHandle{};
    }

    /** Register a callable that repeats every interval_ms milliseconds. */
    template<typename Callable>
    EventHandle on_repeat(uint32_t interval_ms, Callable callable) {
        return on_repeat_us(static_cast<uint64_t>(interval_ms) * 1000ULL, callable);
    }

    /** Register a callable that runs once after delay_ms milliseconds. */
    template<typename Callable>
    EventHandle on_delay(uint32_t delay_ms, Callable callable) {
        return on_delay_us(static_cast<uint64_t>(delay_ms) * 1000ULL, callable);
    }

    /**
     * Register a stream data-ready callback.
     *
     * The same path handles normal byte streams and datagram streams. The stream
     * object is captured by reference and must outlive the returned EventHandle
     * or be removed before the stream is destroyed.
     */
    template<typename Callable>
    EventHandle on_bytes_ready(IByteStream& stream,
                               Callable callable,
                               uint64_t cooperative_poll_us = default_readiness_poll_us) {
        const EventHandle handle = allocate_callback([&stream, callable]() mutable {
            if (stream.readable()) {
                callable();
            }
        });
        if (!handle.assigned()) {
            return EventHandle{};
        }
        if (!register_readiness_or_poll(stream.native_fd(), callback_tasks_[handle.slot], cooperative_poll_us)) {
            release_unregistered(handle);
            return EventHandle{};
        }
        return handle;
    }

    /**
     * Register a GPIO/pin event source callback.
     *
     * Linux fd-backed sources use native fd readiness. Cooperative sources are
     * polled from tick()/run_once(). The callable receives each PinEvent.
     */
    template<typename Callable>
    EventHandle on_pin_event(IPinEventSource& source,
                             Callable callable,
                             uint64_t cooperative_poll_us = default_readiness_poll_us) {
        const EventHandle handle = allocate_callback([&source, callable]() mutable {
            PinEvent event;
            while (source.read_event(event)) {
                callable(event);
            }
        });
        if (!handle.assigned()) {
            return EventHandle{};
        }
        if (!register_readiness_or_poll(source.native_fd(), callback_tasks_[handle.slot], cooperative_poll_us)) {
            release_unregistered(handle);
            return EventHandle{};
        }
        return handle;
    }

    /** Poll one loop iteration. This is the method normally called by Arduino loop(). */
    void tick() { scheduler_.run_once(); }

    /** Poll one loop iteration. Same as tick(), but clearer in Linux code. */
    void run_once() { scheduler_.run_once(); }

    /** Run until request_exit() is called. Normal Linux daemon/app entry point. */
    void run_forever() { scheduler_.run_forever(); }

    /** Request exit from run_forever(). */
    void request_exit() { scheduler_.request_exit(); }

    /** Access the native clock for advanced integrations/tests. */
    NativeClock& clock() { return clock_; }

    /** Access the native scheduler for fd readiness or lower-level integrations. */
    NativeSchedulerFor<MaxCallbacks>& scheduler() { return scheduler_; }

private:
    template<typename Callable>
    struct OneShotCallback {
        OneShotCallback(EventLoop* loop_value, EventHandle handle_value, Callable callable_value)
            : loop(loop_value), handle(handle_value), callable(static_cast<Callable&&>(callable_value)) {}

        void operator()() {
            callable();
            loop->complete_one_shot(handle);
        }

        EventLoop* loop = nullptr;
        EventHandle handle;
        Callable callable;
    };

    template<typename Callable>
    EventHandle allocate_callback(Callable callable) {
        for (size_t i = 0; i < MaxCallbacks; ++i) {
            if (!callback_used_[i]) {
                if (!callback_tasks_[i].set(static_cast<Callable&&>(callable))) {
                    return EventHandle{};
                }
                callback_used_[i] = true;
                return EventHandle{static_cast<uint16_t>(i), callback_generations_[i]};
            }
        }
        return EventHandle{};
    }

    void release_registered(EventHandle handle) {
        if (!handle.assigned() || handle.slot >= MaxCallbacks) {
            return;
        }
        if (!callback_used_[handle.slot] || callback_generations_[handle.slot] != handle.generation) {
            return;
        }
        callback_tasks_[handle.slot].clear();
        callback_used_[handle.slot] = false;
        ++callback_generations_[handle.slot];
    }

    void complete_one_shot(EventHandle handle) {
        release_registered(handle);
    }

    void release_unregistered(EventHandle handle) {
        if (!handle.assigned() || handle.slot >= MaxCallbacks) {
            return;
        }
        callback_tasks_[handle.slot].clear();
        callback_used_[handle.slot] = false;
        ++callback_generations_[handle.slot];
    }

    bool register_readiness_or_poll(int fd, IRuntimeTask& task, uint64_t cooperative_poll_us) {
#if defined(__linux__)
        if (fd >= 0) {
            return scheduler_.add_readable_fd(fd, task);
        }
#else
        (void)fd;
#endif
        if (cooperative_poll_us == 0) {
            cooperative_poll_us = default_readiness_poll_us;
        }
        return scheduler_.add_periodic(task, cooperative_poll_us);
    }

    NativeClock clock_;
    NativeSchedulerFor<MaxCallbacks> scheduler_;
    CallbackTask<CallbackStorageSize> callback_tasks_[MaxCallbacks];
    uint16_t callback_generations_[MaxCallbacks]{};
    bool callback_used_[MaxCallbacks]{};
};

} // namespace async_event_loop
