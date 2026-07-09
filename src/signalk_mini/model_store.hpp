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
    static_assert(Capacity > 0, "ModelChangeQueue capacity must be greater than zero");

    bool push(const ModelChange& change) {
        if (coalesce(change)) return true;

        if (count_ == Capacity) {
            tail_ = (tail_ + 1) % Capacity;
            --count_;
            ++dropped_count_;
        }
        items_[head_] = change;
        head_ = (head_ + 1) % Capacity;
        ++count_;
        ++enqueued_count_;
        if (count_ > high_watermark_) high_watermark_ = count_;
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
    size_t capacity() const { return Capacity; }
    size_t high_watermark() const { return high_watermark_; }
    bool empty() const { return count_ == 0; }
    uint64_t dropped_count() const { return dropped_count_; }
    uint64_t enqueued_count() const { return enqueued_count_; }
    uint64_t coalesced_count() const { return coalesced_count_; }

private:
    bool coalesce(const ModelChange& change) {
        for (size_t offset = 0; offset < count_; ++offset) {
            const size_t reverse_offset = count_ - 1 - offset;
            const size_t index = (tail_ + reverse_offset) % Capacity;
            ModelChange& existing = items_[index];
            if (existing.field == change.field && existing.source_id == change.source_id) {
                existing.timestamp_us = change.timestamp_us;
                existing.sequence = change.sequence;
                ++coalesced_count_;
                return true;
            }
        }
        return false;
    }

    ModelChange items_[Capacity]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
    size_t high_watermark_ = 0;
    uint64_t dropped_count_ = 0;
    uint64_t enqueued_count_ = 0;
    uint64_t coalesced_count_ = 0;
};

template<typename Real, size_t QueueCapacity = 512>
class ModelStore {
public:
    using Model = ship_data_model::DataModel<Real>;

    Model& model() { return model_; }
    const Model& model() const { return model_; }
    ModelChangeQueue<QueueCapacity>& changes() { return changes_; }
    const ModelChangeQueue<QueueCapacity>& changes() const { return changes_; }
    uint64_t sequence() const { return sequence_; }
    uint64_t marked_change_count() const { return marked_change_count_; }
    uint64_t enqueued_change_count() const { return changes_.enqueued_count(); }
    uint64_t coalesced_change_count() const { return changes_.coalesced_count(); }
    uint64_t dropped_change_count() const { return changes_.dropped_count(); }
    size_t pending_change_count() const { return changes_.size(); }
    size_t change_queue_capacity() const { return changes_.capacity(); }
    size_t change_queue_high_watermark() const { return changes_.high_watermark(); }

    void mark_changed(ModelField field, SourceId source_id, uint64_t now_us) {
        ++marked_change_count_;
        ModelChange change{field, source_id, now_us, ++sequence_};
        changes_.push(change);
    }

private:
    Model model_{};
    ModelChangeQueue<QueueCapacity> changes_{};
    uint64_t sequence_ = 0;
    uint64_t marked_change_count_ = 0;
};

} // namespace signalk_mini
