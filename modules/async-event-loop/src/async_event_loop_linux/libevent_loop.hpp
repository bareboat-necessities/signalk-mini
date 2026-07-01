#pragma once

#include <assert.h>
#include <stdint.h>
#include <sys/time.h>
#include <event2/event.h>

#include <memory>
#include <utility>
#include <vector>

#include "async_event_loop/scheduler.hpp"

namespace async_event_loop {
namespace detail {

inline uint64_t periodic_timer_next_delay_us(uint64_t& next_due_us,
                                             uint64_t period_us,
                                             uint64_t now_us) {
    const uint64_t overdue_us = now_us > next_due_us ? now_us - next_due_us : 0;
    next_due_us += period_us;
    return period_us > overdue_us ? period_us - overdue_us : 0;
}

} // namespace detail

class LinuxLibeventLoop final : public IScheduler {
public:
    explicit LinuxLibeventLoop(IClock& clock)
        : clock_(clock), base_(event_base_new()) {}

    ~LinuxLibeventLoop() override {
        free_timer_events(periodic_events_);
        free_timer_events(one_shot_events_);
        free_fd_events(fd_events_);
        if (base_) {
            event_base_free(base_);
            base_ = nullptr;
        }
    }

    bool valid() const { return base_ != nullptr; }
    event_base* base() { return base_; }
    const event_base* base() const { return base_; }

    bool add_periodic(IRuntimeTask& task, uint64_t period_us) override {
        compact_removed_events();
        if (!base_ || period_us == 0) {
            return false;
        }
        auto item = std::unique_ptr<TimerEvent>(new TimerEvent());
        item->loop = this;
        item->task = &task;
        item->period_us = period_us;
        item->next_due_us = clock_.micros() + period_us;
        item->periodic = true;
        item->ev = evtimer_new(base_, &LinuxLibeventLoop::timer_callback, item.get());
        if (!item->ev) {
            return false;
        }
        timeval tv = timeval_from_us(period_us);
        if (evtimer_add(item->ev, &tv) != 0) {
            event_free(item->ev);
            item->ev = nullptr;
            return false;
        }
        periodic_events_.push_back(std::move(item));
        return true;
    }

    bool add_one_shot(IRuntimeTask& task, uint64_t due_us) override {
        compact_removed_events();
        if (!base_) {
            return false;
        }
        auto item = std::unique_ptr<TimerEvent>(new TimerEvent());
        item->loop = this;
        item->task = &task;
        item->period_us = 0;
        item->next_due_us = due_us;
        item->periodic = false;
        item->ev = evtimer_new(base_, &LinuxLibeventLoop::timer_callback, item.get());
        if (!item->ev) {
            return false;
        }
        const uint64_t now_us = clock_.micros();
        const uint64_t delta_us = due_us > now_us ? due_us - now_us : 0;
        timeval tv = timeval_from_us(delta_us);
        if (evtimer_add(item->ev, &tv) != 0) {
            event_free(item->ev);
            item->ev = nullptr;
            return false;
        }
        one_shot_events_.push_back(std::move(item));
        return true;
    }

    bool remove(IRuntimeTask& task) override {
        bool removed = false;
        removed = mark_timer_events_removed(periodic_events_, task) || removed;
        removed = mark_timer_events_removed(one_shot_events_, task) || removed;
        removed = mark_fd_events_removed(fd_events_, task) || removed;
        compact_removed_events();
        return removed;
    }

    bool add_readable_fd(int fd, IRuntimeTask& task) {
        compact_removed_events();
        return add_fd(fd, EV_READ | EV_PERSIST, task);
    }

    bool add_writable_fd(int fd, IRuntimeTask& task) {
        compact_removed_events();
        return add_fd(fd, EV_WRITE | EV_PERSIST, task);
    }

    bool remove_fd(int fd, IRuntimeTask& task) {
        bool removed = false;
        for (auto& entry : fd_events_) {
            FdEvent* item = entry.get();
            if (item && item->fd == fd && item->task == &task) {
                item->removed = true;
                item->task = nullptr;
                removed = true;
            }
        }
        compact_removed_events();
        return removed;
    }

    void run_once() override {
        if (base_) {
            event_base_loop(base_, EVLOOP_NONBLOCK);
            compact_removed_events();
        }
    }

    void run_forever() override {
        if (base_) {
            event_base_dispatch(base_);
            compact_removed_events();
        }
    }

    void request_exit() override {
        if (base_) {
            event_base_loopbreak(base_);
        }
    }

private:
    struct TimerEvent {
        LinuxLibeventLoop* loop = nullptr;
        IRuntimeTask* task = nullptr;
        event* ev = nullptr;
        uint64_t period_us = 0;
        uint64_t next_due_us = 0;
        bool periodic = false;
        bool removed = false;
    };

    struct FdEvent {
        LinuxLibeventLoop* loop = nullptr;
        IRuntimeTask* task = nullptr;
        event* ev = nullptr;
        int fd = -1;
        bool removed = false;
    };

    bool add_fd(int fd, short flags, IRuntimeTask& task) {
        if (!base_ || fd < 0) {
            return false;
        }
        auto item = std::unique_ptr<FdEvent>(new FdEvent());
        item->loop = this;
        item->task = &task;
        item->fd = fd;
        item->ev = event_new(base_, fd, flags, &LinuxLibeventLoop::fd_callback, item.get());
        if (!item->ev) {
            return false;
        }
        if (event_add(item->ev, nullptr) != 0) {
            event_free(item->ev);
            item->ev = nullptr;
            return false;
        }
        fd_events_.push_back(std::move(item));
        return true;
    }

    static void free_event(event* ev) {
        if (ev) {
            event_free(ev);
        }
    }

    static void free_timer_events(std::vector<std::unique_ptr<TimerEvent>>& events) {
        for (auto& e : events) {
            if (e && e->ev) {
                event_free(e->ev);
                e->ev = nullptr;
            }
        }
        events.clear();
    }

    static void free_fd_events(std::vector<std::unique_ptr<FdEvent>>& events) {
        for (auto& e : events) {
            if (e && e->ev) {
                event_free(e->ev);
                e->ev = nullptr;
            }
        }
        events.clear();
    }

    static bool mark_timer_events_removed(std::vector<std::unique_ptr<TimerEvent>>& events, IRuntimeTask& task) {
        bool removed = false;
        for (auto& entry : events) {
            TimerEvent* item = entry.get();
            if (item && item->task == &task) {
                item->removed = true;
                item->task = nullptr;
                removed = true;
            }
        }
        return removed;
    }

    static bool mark_fd_events_removed(std::vector<std::unique_ptr<FdEvent>>& events, IRuntimeTask& task) {
        bool removed = false;
        for (auto& entry : events) {
            FdEvent* item = entry.get();
            if (item && item->task == &task) {
                item->removed = true;
                item->task = nullptr;
                removed = true;
            }
        }
        return removed;
    }

    void compact_removed_events() {
        if (callback_depth_ != 0) {
            return;
        }
        compact_removed_timer_events(periodic_events_);
        compact_removed_timer_events(one_shot_events_);
        compact_removed_fd_events(fd_events_);
    }

    static void compact_removed_timer_events(std::vector<std::unique_ptr<TimerEvent>>& events) {
        for (auto it = events.begin(); it != events.end();) {
            TimerEvent* item = it->get();
            if (item && item->removed) {
                free_event(item->ev);
                item->ev = nullptr;
                it = events.erase(it);
            } else {
                ++it;
            }
        }
    }

    static void compact_removed_fd_events(std::vector<std::unique_ptr<FdEvent>>& events) {
        for (auto it = events.begin(); it != events.end();) {
            FdEvent* item = it->get();
            if (item && item->removed) {
                free_event(item->ev);
                item->ev = nullptr;
                it = events.erase(it);
            } else {
                ++it;
            }
        }
    }

    static timeval timeval_from_us(uint64_t us) {
        timeval tv;
        tv.tv_sec = static_cast<time_t>(us / 1000000ULL);
        tv.tv_usec = static_cast<suseconds_t>(us % 1000000ULL);
        return tv;
    }

    void enter_callback() { ++callback_depth_; }

    void leave_callback() {
        assert(callback_depth_ > 0);
        if (callback_depth_ > 0) {
            --callback_depth_;
        }
    }

    static void timer_callback(evutil_socket_t, short, void* arg) {
        auto* item = static_cast<TimerEvent*>(arg);
        if (!item || !item->loop || item->removed || !item->task) {
            return;
        }
        LinuxLibeventLoop* loop = item->loop;
        loop->enter_callback();
        item->task->poll(loop->clock_.micros());
        loop->leave_callback();
        if (!item->removed && item->periodic && item->ev) {
            const uint64_t now_us = loop->clock_.micros();
            const uint64_t next_delay_us = detail::periodic_timer_next_delay_us(
                item->next_due_us,
                item->period_us,
                now_us);
            timeval tv = timeval_from_us(next_delay_us);
            evtimer_add(item->ev, &tv);
        } else if (!item->periodic) {
            item->removed = true;
            item->task = nullptr;
        }
        loop->compact_removed_events();
    }

    static void fd_callback(evutil_socket_t, short, void* arg) {
        auto* item = static_cast<FdEvent*>(arg);
        if (!item || !item->loop || item->removed || !item->task) {
            return;
        }
        LinuxLibeventLoop* loop = item->loop;
        loop->enter_callback();
        item->task->poll(loop->clock_.micros());
        loop->leave_callback();
        loop->compact_removed_events();
    }

    IClock& clock_;
    event_base* base_ = nullptr;
    std::vector<std::unique_ptr<TimerEvent> > periodic_events_;
    std::vector<std::unique_ptr<TimerEvent> > one_shot_events_;
    std::vector<std::unique_ptr<FdEvent> > fd_events_;
    int callback_depth_ = 0;
};

} // namespace async_event_loop
