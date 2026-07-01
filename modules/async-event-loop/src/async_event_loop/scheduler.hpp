#pragma once

#include <new>
#include <stddef.h>
#include <stdint.h>

namespace async_event_loop {

/**
 * Monotonic microsecond clock used by schedulers and event-loop users.
 *
 * Implementations must return a steadily increasing time base suitable for
 * relative scheduling. Wall-clock time, time zones, and calendar corrections do
 * not belong here.
 */
class IClock {
public:
    virtual ~IClock() = default;

    /** Return monotonic time in microseconds. */
    virtual uint64_t micros() const = 0;
};

/**
 * Minimal task interface scheduled by periodic and one-shot backends.
 *
 * A task is called from the scheduler with the scheduler's current monotonic
 * timestamp. Tasks should return quickly and must not assume they run on a
 * separate thread.
 */
class IRuntimeTask {
public:
    virtual ~IRuntimeTask() = default;

    /** Execute one scheduler callback at now_us. */
    virtual void poll(uint64_t now_us) = 0;
};

/**
 * Fixed-storage callback runtime task.
 *
 * CallbackTask owns one callable inside static storage. It can be disabled or
 * removed after it has been registered with a scheduler. Destruction is deferred
 * when a callback removes itself while its callable is still executing.
 */
template<size_t StorageSize = 48>
class CallbackTask final : public IRuntimeTask {
public:
    CallbackTask() = default;

    CallbackTask(const CallbackTask&) = delete;
    CallbackTask& operator=(const CallbackTask&) = delete;

    ~CallbackTask() override { destroy_now(); }

    /**
     * Store a callable in-place and make this task active and enabled.
     *
     * The callable must fit in StorageSize bytes and use no stricter alignment
     * than StorageAlignment. The callable is invoked as callable() even though
     * the scheduler passes now_us through IRuntimeTask::poll().
     */
    template<typename Callable>
    bool set(Callable callable) {
        static_assert(sizeof(Callable) <= StorageSize, "callback object is too large for CallbackTask storage");
        static_assert(alignof(Callable) <= alignof(StorageAlignment), "callback object alignment is too large for CallbackTask storage");
        clear();
        if (invoking_) {
            return false;
        }
        new (storage_) Callable(static_cast<Callable&&>(callable));
        invoke_ = [](void* storage, uint64_t) {
            (*static_cast<Callable*>(storage))();
        };
        destroy_ = [](void* storage) {
            static_cast<Callable*>(storage)->~Callable();
        };
        active_ = true;
        enabled_ = true;
        clear_requested_ = false;
        return true;
    }

    /** Invoke the stored callable when active and enabled. */
    void poll(uint64_t now_us) override {
        if (active_ && enabled_ && invoke_) {
            invoking_ = true;
            void (*invoke)(void*, uint64_t) = invoke_;
            invoke(storage_, now_us);
            invoking_ = false;
            if (clear_requested_) {
                destroy_now();
            }
        }
    }

    /** True after set() succeeds and before clear()/destruction completes. */
    bool active() const { return active_; }

    /** True when active and not disabled. */
    bool enabled() const { return enabled_; }

    /** Enable or disable an active callback without freeing its storage. */
    void set_enabled(bool enabled) {
        if (active_) {
            enabled_ = enabled;
        }
    }

    /**
     * Destroy the stored callable and make the task inactive.
     *
     * If called from inside the callback, destruction is deferred until the
     * callback returns so self-removal is safe.
     */
    void clear() {
        if (invoking_) {
            active_ = false;
            enabled_ = false;
            clear_requested_ = true;
            return;
        }
        destroy_now();
    }

private:
    union StorageAlignment {
        void* pointer;
        long long integer;
        double floating;
        long double long_floating;
    };

    void destroy_now() {
        if ((active_ || clear_requested_) && destroy_) {
            destroy_(storage_);
        }
        active_ = false;
        enabled_ = false;
        clear_requested_ = false;
        invoke_ = nullptr;
        destroy_ = nullptr;
    }

    alignas(StorageAlignment) unsigned char storage_[StorageSize]{};
    void (*invoke_)(void*, uint64_t) = nullptr;
    void (*destroy_)(void*) = nullptr;
    bool active_ = false;
    bool enabled_ = false;
    bool invoking_ = false;
    bool clear_requested_ = false;
};

/**
 * Periodic timer helper for cooperative schedulers.
 *
 * The timer is drift-resistant: when ready() observes that time has jumped past
 * one or more periods, it advances next_due_us_ until the next future deadline
 * rather than firing repeatedly to catch up.
 */
class PeriodicTimer {
public:
    explicit PeriodicTimer(uint64_t period_us = 0) : period_us_(period_us) {}

    /** Set the period in microseconds. A zero period never becomes ready. */
    void set_period_us(uint64_t period_us) { period_us_ = period_us; }

    /** Return the current period in microseconds. */
    uint64_t period_us() const { return period_us_; }

    /** Arm/re-arm the timer so the next due time starts at now_us. */
    void reset(uint64_t now_us = 0) {
        next_due_us_ = now_us;
        armed_ = true;
    }

    /**
     * Return true once per elapsed period.
     *
     * The first call arms the timer. After a ready event, the next due time is
     * moved forward by whole periods until it is in the future.
     */
    bool ready(uint64_t now_us) {
        if (!armed_) {
            reset(now_us);
        }
        if (period_us_ == 0 || now_us < next_due_us_) {
            return false;
        }
        do {
            next_due_us_ += period_us_;
        } while (now_us >= next_due_us_);
        return true;
    }

    /** Return the next absolute due time in microseconds. */
    uint64_t next_due_us() const { return next_due_us_; }

private:
    uint64_t period_us_ = 0;
    uint64_t next_due_us_ = 0;
    bool armed_ = false;
};

/** One-shot timer helper for cooperative schedulers. */
class OneShotTimer {
public:
    /** Arm the timer to fire once at absolute due_us. */
    void arm(uint64_t due_us) {
        due_us_ = due_us;
        armed_ = true;
        fired_ = false;
    }

    /** Cancel the pending one-shot. fired() is not changed. */
    void disarm() { armed_ = false; }

    /** Return true when the one-shot is waiting for its due time. */
    bool armed() const { return armed_; }

    /** Return true after the current/last arm has fired. */
    bool fired() const { return fired_; }

    /** Return the absolute due time in microseconds. */
    uint64_t due_us() const { return due_us_; }

    /** Return true exactly once when now_us reaches due_us. */
    bool ready(uint64_t now_us) {
        if (!armed_ || fired_ || now_us < due_us_) {
            return false;
        }
        fired_ = true;
        armed_ = false;
        return true;
    }

private:
    uint64_t due_us_ = 0;
    bool armed_ = false;
    bool fired_ = false;
};

/**
 * Scheduler backend interface used by EventLoop.
 *
 * Implementations may be cooperative polling schedulers, libevent-backed
 * schedulers, or test schedulers. They own scheduling policy, not task storage;
 * tasks must outlive their registration or be removed before destruction.
 */
class IScheduler {
public:
    virtual ~IScheduler() = default;

    /** Return true when the backend initialized successfully. */
    virtual bool valid() const = 0;

    /** Schedule a task to run repeatedly every period_us microseconds. */
    virtual bool add_periodic(IRuntimeTask& task, uint64_t period_us) = 0;

    /** Schedule a task to run once at absolute due_us. */
    virtual bool add_one_shot(IRuntimeTask& task, uint64_t due_us) = 0;

    /** Unschedule a previously registered task. */
    virtual bool remove(IRuntimeTask& task) = 0;

    /** Run one scheduler iteration. Cooperative backends use this from loop(). */
    virtual void run_once() = 0;

    /** Run scheduler iterations until request_exit() is observed. */
    virtual void run_forever() = 0;

    /** Ask run_forever() to stop. */
    virtual void request_exit() = 0;
};

} // namespace async_event_loop
