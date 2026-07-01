#pragma once

#include <stddef.h>
#include <stdint.h>
#include <ship_data_model.hpp>
#include "types.hpp"

namespace signalk_mini {

struct ModelChange {
    ModelField field = ModelField::None;
    SourceId source_id = 0;
    uint64_t timestamp_us = 0;
    uint64_t sequence = 0;
};

template<size_t Capacity>
class ModelChangeQueue {
public:
    bool push(const ModelChange& change) {
        if (count_ == Capacity) {
            tail_ = (tail_ + 1) % Capacity;
            --count_;
        }
        items_[head_] = change;
        head_ = (head_ + 1) % Capacity;
        ++count_;
        return true;
    }

    bool pop(ModelChange& change) {
        if (count_ == 0) return false;
        change = items_[tail_];
        tail_ = (tail_ + 1) % Capacity;
        --count_;
        return true;
    }

    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }

private:
    ModelChange items_[Capacity]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
};

template<typename Real, size_t QueueCapacity = 128>
class ModelStore {
public:
    using Model = ship_data_model::DataModel<Real>;

    Model& model() { return model_; }
    const Model& model() const { return model_; }
    ModelChangeQueue<QueueCapacity>& changes() { return changes_; }
    uint64_t sequence() const { return sequence_; }

    void mark_changed(ModelField field, SourceId source_id, uint64_t now_us) {
        ModelChange change{field, source_id, now_us, ++sequence_};
        changes_.push(change);
    }

private:
    Model model_{};
    ModelChangeQueue<QueueCapacity> changes_{};
    uint64_t sequence_ = 0;
};

} // namespace signalk_mini
