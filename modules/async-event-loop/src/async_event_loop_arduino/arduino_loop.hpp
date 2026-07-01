#pragma once

#include <stddef.h>
#if defined(ARDUINO)
#include <Arduino.h>
#endif
#include "async_event_loop/scheduler.hpp"

#ifndef ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS
#define ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS 32
#endif

namespace async_event_loop {

template<size_t MaxTasks>
class ArduinoLoopImpl final : public IScheduler {
public:
    explicit ArduinoLoopImpl(IClock& clock) : clock_(clock) {}

    bool valid() const override { return true; }

    bool add_periodic(IRuntimeTask& task, uint64_t period_us) override {
        compact_inactive();
        if (period_us == 0 || task_count_ >= MaxTasks) {
            return false;
        }
        TaskSlot slot;
        slot.task = &task;
        slot.period_us = period_us;
        slot.next_due_us = clock_.micros();
        slot.periodic = true;
        slot.active = true;
        tasks_[task_count_++] = slot;
        return true;
    }

    bool add_one_shot(IRuntimeTask& task, uint64_t due_us) override {
        compact_inactive();
        if (task_count_ >= MaxTasks) {
            return false;
        }
        TaskSlot slot;
        slot.task = &task;
        slot.period_us = 0;
        slot.next_due_us = due_us;
        slot.periodic = false;
        slot.active = true;
        tasks_[task_count_++] = slot;
        return true;
    }

    bool remove(IRuntimeTask& task) override {
        bool removed = false;
        for (size_t i = 0; i < task_count_; ++i) {
            TaskSlot& slot = tasks_[i];
            if (slot.task == &task) {
                slot.task = nullptr;
                slot.active = false;
                removed = true;
            }
        }
        if (!running_) {
            compact_inactive();
        }
        return removed;
    }

    void run_once() override {
        compact_inactive();
        running_ = true;
        const uint64_t now_us = clock_.micros();
        for (size_t i = 0; i < task_count_; ++i) {
            TaskSlot& slot = tasks_[i];
            if (!slot.active || !slot.task || now_us < slot.next_due_us) {
                continue;
            }
            slot.task->poll(now_us);
            if (!slot.active || !slot.task) {
                continue;
            }
            if (slot.periodic) {
                do {
                    slot.next_due_us += slot.period_us;
                } while (now_us >= slot.next_due_us);
            } else {
                slot.active = false;
                slot.task = nullptr;
            }
        }
        running_ = false;
        compact_inactive();
    }

    void run_forever() override {
        while (!exit_requested_) {
            run_once();
            yield_if_available();
        }
    }

    void request_exit() override { exit_requested_ = true; }
    void tick() { run_once(); }

    static constexpr size_t capacity() { return MaxTasks; }

private:
    struct TaskSlot {
        IRuntimeTask* task = nullptr;
        uint64_t period_us = 0;
        uint64_t next_due_us = 0;
        bool periodic = false;
        bool active = false;
    };

    void compact_inactive() {
        for (size_t i = 0; i < task_count_;) {
            const TaskSlot& slot = tasks_[i];
            if (!slot.active || !slot.task) {
                erase_task(i);
            } else {
                ++i;
            }
        }
    }

    void erase_task(size_t index) {
        if (index >= task_count_) {
            return;
        }
        for (size_t i = index + 1; i < task_count_; ++i) {
            tasks_[i - 1] = tasks_[i];
        }
        --task_count_;
        tasks_[task_count_] = TaskSlot{};
    }

    static void yield_if_available() {
#if defined(ARDUINO)
        yield();
#endif
    }

    IClock& clock_;
    TaskSlot tasks_[MaxTasks]{};
    size_t task_count_ = 0;
    bool exit_requested_ = false;
    bool running_ = false;
};

using ArduinoLoop = ArduinoLoopImpl<ASYNC_EVENT_LOOP_DEFAULT_MAX_CALLBACKS>;

} // namespace async_event_loop
