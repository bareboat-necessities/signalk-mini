#pragma once

#include <stddef.h>
#include <stdint.h>
#include <ship_data_model.hpp>
#include "memory_profile.hpp"
#include "types.hpp"

namespace signalk_mini {

inline uint32_t model_change_timestamp_ms(uint64_t now_us) {
    return static_cast<uint32_t>(now_us / 1000ULL);
}

struct ModelChange {
    ModelField field = ModelField::None;
    SourceId source_id = 0;
    uint32_t timestamp_ms = 0;
    uint32_t sequence = 0;
};

template<size_t Capacity>
class ModelChangeQueue {
public:
    static_assert(Capacity > 0, "ModelChangeQueue capacity must be greater than zero");
    static_assert(Capacity <= MaxSupportedModelChangeQueueCapacity, "ModelChangeQueue capacity exceeds compact index range");

    bool push(const ModelChange& change) {
        if (coalesce(change)) return true;

        if (count_ == Capacity) {
            tail_ = advance(tail_);
            --count_;
            ++dropped_count_;
        }
        items_[head_] = change;
        head_ = advance(head_);
        ++count_;
        ++enqueued_count_;
        if (count_ > high_watermark_) high_watermark_ = count_;
        return true;
    }

    bool pop(ModelChange& change) {
        if (count_ == 0) return false;
        change = items_[tail_];
        tail_ = advance(tail_);
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
    static uint16_t advance(uint16_t value) {
        const uint16_t next = static_cast<uint16_t>(value + 1u);
        return next == Capacity ? 0 : next;
    }

    bool coalesce(const ModelChange& change) {
        for (uint16_t offset = 0; offset < count_; ++offset) {
            const uint16_t reverse_offset = static_cast<uint16_t>(count_ - 1u - offset);
            const uint16_t index = static_cast<uint16_t>((tail_ + reverse_offset) % Capacity);
            ModelChange& existing = items_[index];
            if (existing.field == change.field && existing.source_id == change.source_id) {
                existing.timestamp_ms = change.timestamp_ms;
                existing.sequence = change.sequence;
                ++coalesced_count_;
                return true;
            }
        }
        return false;
    }

    ModelChange items_[Capacity]{};
    uint16_t head_ = 0;
    uint16_t tail_ = 0;
    uint16_t count_ = 0;
    uint16_t high_watermark_ = 0;
    uint64_t dropped_count_ = 0;
    uint64_t enqueued_count_ = 0;
    uint64_t coalesced_count_ = 0;
};

// Per-source values for Signal K paths reported by more than one source.
// The typed model keeps one value per field (the latest from any source);
// this bounded table keeps each source's latest value for multi-source paths
// so they can be published as Signal K `values`. Entries are keyed by the
// static Signal K path string, so fields that map to the same path share them.
enum class SourceValueKind : uint8_t { Empty, Number, Bool, Position };

struct SourceValueEntry {
    uint64_t last_update_us = 0;
    double a = 0.0; // number, bool (0/1) or latitude
    double b = 0.0; // longitude
    const char* path = nullptr;
    SourceId source_id = 0;
    SourceValueKind kind = SourceValueKind::Empty;
};

template<size_t Capacity>
class SourceValueTable {
public:
    // Inserts or replaces (path, source); evicts the oldest entry when full.
    void record(const SourceValueEntry& entry) {
        SourceValueEntry* slot = nullptr;
        for (SourceValueEntry& existing : entries_) {
            if (existing.kind != SourceValueKind::Empty && existing.path == entry.path && existing.source_id == entry.source_id) {
                slot = &existing;
                break;
            }
        }
        if (!slot) {
            for (SourceValueEntry& existing : entries_) {
                if (existing.kind == SourceValueKind::Empty) {
                    slot = &existing;
                    break;
                }
            }
        }
        if (!slot) {
            slot = &entries_[0];
            for (SourceValueEntry& existing : entries_) {
                if (existing.last_update_us < slot->last_update_us) slot = &existing;
            }
            ++evicted_count_;
        }
        *slot = entry;
    }

    const SourceValueEntry* find(const char* path, SourceId source_id) const {
        for (const SourceValueEntry& entry : entries_) {
            if (entry.kind != SourceValueKind::Empty && entry.path == path && entry.source_id == source_id) return &entry;
        }
        return nullptr;
    }

    size_t count_for(const char* path) const {
        size_t count = 0;
        for (const SourceValueEntry& entry : entries_) {
            if (entry.kind != SourceValueKind::Empty && entry.path == path) ++count;
        }
        return count;
    }

    template<typename Callback>
    void for_each_for(const char* path, Callback callback) const {
        for (const SourceValueEntry& entry : entries_) {
            if (entry.kind != SourceValueKind::Empty && entry.path == path) callback(entry);
        }
    }

    uint64_t evicted_count() const { return evicted_count_; }

private:
    SourceValueEntry entries_[Capacity]{};
    uint64_t evicted_count_ = 0;
};

// Notified on every mark_changed, after the model value has been written and
// before the change is queued.
struct IModelMarkObserver {
    virtual ~IModelMarkObserver() = default;
    virtual void on_model_marked(ModelField field, SourceId source_id, SourceId previous_source_id,
                                 bool had_value, uint64_t now_us) = 0;
};

template<typename Real, size_t QueueCapacity = DefaultModelChangeQueueCapacity>
class ModelStore {
public:
    using Model = ship_data_model::DataModel<Real>;
    static constexpr size_t FieldCount = static_cast<size_t>(ModelField::Count);
    static constexpr size_t PresenceBytes = (FieldCount + 7u) / 8u;

    Model& model() { return model_; }
    const Model& model() const { return model_; }
    ModelChangeQueue<QueueCapacity>& changes() { return changes_; }
    const ModelChangeQueue<QueueCapacity>& changes() const { return changes_; }
    uint32_t sequence() const { return sequence_; }
    uint64_t marked_change_count() const { return marked_change_count_; }
    uint64_t enqueued_change_count() const { return changes_.enqueued_count(); }
    uint64_t coalesced_change_count() const { return changes_.coalesced_count(); }
    uint64_t dropped_change_count() const { return changes_.dropped_count(); }
    SourceId source_id_for(ModelField field) const {
        const size_t index = static_cast<size_t>(field);
        return index < FieldCount ? latest_source_id_[index] : 0;
    }
    bool has_value(ModelField field) const {
        const size_t index = static_cast<size_t>(field);
        return index < FieldCount && (present_bits_[index >> 3u] & static_cast<uint8_t>(1u << (index & 7u))) != 0;
    }
    size_t pending_change_count() const { return changes_.size(); }
    size_t change_queue_capacity() const { return changes_.capacity(); }
    size_t change_queue_high_watermark() const { return changes_.high_watermark(); }
    uint64_t autopilot_state_last_update_us() const { return autopilot_state_last_update_us_; }
    SourceId autopilot_state_source_id() const { return autopilot_state_source_id_; }

    void record_current(ModelField field, SourceId source_id) {
        const size_t field_index = static_cast<size_t>(field);
        if (field_index < FieldCount) {
            latest_source_id_[field_index] = source_id;
            present_bits_[field_index >> 3u] |= static_cast<uint8_t>(1u << (field_index & 7u));
        }
    }

    using SourceValues = SourceValueTable<DefaultSourceValueCapacity>;
    SourceValues& source_values() { return source_values_; }
    const SourceValues& source_values() const { return source_values_; }
    void set_mark_observer(IModelMarkObserver* observer) { mark_observer_ = observer; }

    void mark_changed(ModelField field, SourceId source_id, uint64_t now_us) {
        ++marked_change_count_;
        if (mark_observer_) {
            const size_t index = static_cast<size_t>(field);
            const bool had_value = has_value(field);
            const SourceId previous = index < FieldCount ? latest_source_id_[index] : 0;
            mark_observer_->on_model_marked(field, source_id, previous, had_value, now_us);
        }
        record_current(field, source_id);
        if (field == ModelField::AutopilotMode || field == ModelField::AutopilotEnabled) {
            autopilot_state_last_update_us_ = now_us;
            autopilot_state_source_id_ = source_id;
        }
        ModelChange change{field, source_id, model_change_timestamp_ms(now_us), ++sequence_};
        changes_.push(change);
    }

private:
    Model model_{};
    ModelChangeQueue<QueueCapacity> changes_{};
    uint32_t sequence_ = 0;
    uint64_t marked_change_count_ = 0;
    uint64_t autopilot_state_last_update_us_ = 0;
    SourceId autopilot_state_source_id_ = 0;
    SourceId latest_source_id_[FieldCount]{};
    uint8_t present_bits_[PresenceBytes]{};
    SourceValues source_values_{};
    IModelMarkObserver* mark_observer_ = nullptr;
};

} // namespace signalk_mini
