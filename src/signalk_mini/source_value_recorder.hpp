#pragma once

#include <stddef.h>
#include <stdint.h>

#include "model_store.hpp"
#include "signalk_json_stream_writer.hpp"
#include "signalk_mapper.hpp"

namespace signalk_mini {

// Fills ModelStore::source_values() for fields that more than one source
// updates. A field becomes multi-source the first time a different source
// overwrites it; from then on each source's mapped value is recorded as it
// arrives (the value the previous source had at that moment is already gone,
// so its entry appears with its next update). Numbers, booleans and positions
// are kept; text and other objects only keep the latest value in the model.
template<typename Real>
class SourceValueRecorder final : public IModelMarkObserver {
public:
    explicit SourceValueRecorder(ModelStore<Real>& store) : store_(store) {}

    SourceValueRecorder(const SourceValueRecorder&) = delete;
    SourceValueRecorder& operator=(const SourceValueRecorder&) = delete;

    void on_model_marked(ModelField field, SourceId source_id, SourceId previous_source_id,
                         bool had_value, uint64_t now_us) override {
        const size_t index = static_cast<size_t>(field);
        if (index >= FieldCount) return;
        const uint8_t bit = static_cast<uint8_t>(1u << (index & 7u));
        if (had_value && previous_source_id != source_id) multi_source_[index >> 3u] |= bit;
        if ((multi_source_[index >> 3u] & bit) == 0) return;

        SignalKMappedValue<Real> mapped;
        if (!mapper_.map_field(store_.model(), field, mapped) || !mapped.path) return;
        SourceValueEntry entry;
        entry.path = mapped.path;
        entry.source_id = source_id;
        entry.last_update_us = now_us;
        if (mapped.kind == SignalKMappedValueKind::Number) {
            entry.kind = SourceValueKind::Number;
            entry.a = mapped.has_coordinate ? mapped.coordinate : static_cast<double>(mapped.number);
        } else if (mapped.kind == SignalKMappedValueKind::Bool) {
            entry.kind = SourceValueKind::Bool;
            entry.a = mapped.boolean ? 1.0 : 0.0;
        } else if (mapped.kind == SignalKMappedValueKind::Object && mapped.object_kind == SignalKObjectKind::Position) {
            const auto& fix = store_.model().gnss.fix;
            entry.kind = SourceValueKind::Position;
            entry.a = fix.fix_lat_deg.value;
            entry.b = fix.fix_lon_deg.value;
        } else {
            return;
        }
        store_.source_values().record(entry);
    }

    bool multi_source(ModelField field) const {
        const size_t index = static_cast<size_t>(field);
        return index < FieldCount && (multi_source_[index >> 3u] & static_cast<uint8_t>(1u << (index & 7u))) != 0;
    }

private:
    static constexpr size_t FieldCount = static_cast<size_t>(ModelField::Count);

    ModelStore<Real>& store_;
    SignalKMapper<Real> mapper_;
    uint8_t multi_source_[(FieldCount + 7u) / 8u]{};
};

// Writes one source value as JSON (the `value` member of a Signal K entry).
inline bool signalk_write_source_value(SignalKJsonStreamWriter& out, const SourceValueEntry& entry) {
    switch (entry.kind) {
    case SourceValueKind::Number: return out.append_real(entry.a);
    case SourceValueKind::Bool: return out.append_bool(entry.a != 0.0);
    case SourceValueKind::Position:
        return out.append_raw("{\"latitude\":") && out.append_real(entry.a) &&
               out.append_raw(",\"longitude\":") && out.append_real(entry.b) && out.append_char('}');
    case SourceValueKind::Empty:
    default: return false;
    }
}

} // namespace signalk_mini
