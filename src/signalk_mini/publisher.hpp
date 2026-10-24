#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <type_traits>

#include "config.hpp"
#include "memory_profile.hpp"
#include "signalk_delta_writer.hpp"
#include "signalk_json_stream_writer.hpp"
#include "units.hpp"
#include "model_store.hpp"
#include "signalk_mapper.hpp"
#include "signalk_typed_view.hpp"

namespace signalk_mini {

// Builds one Signal K delta in a caller-owned fixed buffer:
// {"context":C,"updates":[{"timestamp":T,"source":{"label":L},"values":[V,...]}]}\r\n
// Room for the closing bytes is reserved up front, so every accepted value
// leaves a complete delta. After finish() the header is still in the buffer,
// so reopen() starts the next delta of a split without rebuilding it.
class SignalKDeltaFrame {
public:
    enum class AddResult : uint8_t { Added, Full, TooLarge };

    SignalKDeltaFrame(char* buffer, size_t capacity)
        : buffer_(buffer), out_(buffer, capacity > TrailerLen ? capacity - TrailerLen : 0) {}

    bool open() const { return open_; }
    size_t count() const { return count_; }

    bool begin(const char* context, const char* timestamp, const char* source_label) {
        out_.rewind(0);
        open_ = out_.append_raw("{\"context\":") && out_.append_quoted(context) &&
                out_.append_raw(",\"updates\":[{\"timestamp\":") && out_.append_quoted(timestamp) &&
                out_.append_raw(",\"source\":{\"label\":") &&
                out_.append_quoted(source_label ? source_label : "signalk-mini") &&
                out_.append_raw("},\"values\":[") && out_.ok();
        header_len_ = open_ ? out_.size() : 0;
        count_ = 0;
        return open_;
    }

    void reopen() {
        if (header_len_ == 0 || !out_.rewind(header_len_)) return;
        open_ = true;
        count_ = 0;
    }

    // write_value appends one JSON value for `path`.
    template<typename WriteValue>
    AddResult add(const char* path, WriteValue&& write_value) {
        const size_t mark = out_.size();
        const bool written = (count_ == 0 || out_.append_char(',')) &&
                             out_.append_raw("{\"path\":") && out_.append_quoted(path ? path : "") &&
                             out_.append_raw(",\"value\":") && write_value(out_) &&
                             out_.append_char('}') && out_.ok();
        if (written) {
            ++count_;
            return AddResult::Added;
        }
        out_.rewind(mark);
        return count_ == 0 ? AddResult::TooLarge : AddResult::Full;
    }

    size_t finish() {
        const bool has_values = open_ && count_ != 0;
        open_ = false;
        if (!has_values) return 0;
        size_t len = out_.size();
        memcpy(buffer_ + len, Trailer, TrailerLen);
        len += TrailerLen;
        buffer_[len] = '\0';
        return len;
    }

private:
    static constexpr char Trailer[] = "]}]}\r\n";
    static constexpr size_t TrailerLen = sizeof(Trailer) - 1;

    char* buffer_;
    SignalKJsonStreamWriter out_;
    size_t header_len_ = 0;
    size_t count_ = 0;
    bool open_ = false;
};

template<typename Real, size_t MaxJsonBufferSize = DefaultSignalKJsonBufferSize, size_t MaxBatchValues = DefaultSignalKBatchValues>
class SignalKPublisher {
public:
    // Resumable snapshot cursor: model fields first, then one slot per AIS target.
    static constexpr uint16_t SnapshotAisCursorBase = static_cast<uint16_t>(ModelField::Count);
    static constexpr uint16_t SnapshotCursorEnd =
        static_cast<uint16_t>(SnapshotAisCursorBase + ship_data_model::AIS_TARGET_TABLE_CAPACITY);

    SignalKPublisher(ModelStore<Real>& store, const SignalKMiniConfig& config)
        : store_(store), config_(config) {
        static_assert(MaxBatchValues > 0, "SignalKPublisher batch size must be greater than zero");
    }

    // Sends queued model changes, batched per source. A delta is closed and sent
    // when the next value does not fit the JSON buffer, so nothing is dropped
    // unless a single value is larger than the whole buffer.
    template<typename DeltaSink>
    void publish_pending(DeltaSink& sink) {
        const size_t json_capacity = effective_json_buffer_size();
        if (json_capacity == 0) return;

        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return;

        SignalKDeltaFrame frame(json_, json_capacity);
        SourceId frame_source_id = 0;
        ModelChange change;
        size_t processed = 0;
        const size_t max_changes = static_cast<size_t>(config_.publisher.max_changes_per_tick);

        while (processed < max_changes && store_.changes().pop(change)) {
            ++processed;

            const char* source_label = label_for_source(change.source_id);
            SignalKMappedValue<Real> mapped;
            const bool mapped_ok = change.field != ModelField::AisTargetsObject &&
                                   mapper_.map_change(store_.model(), change, mapped) && mapped.path;
            if (change.field == ModelField::AisTargetsObject ||
                (mapped_ok && mapped.kind == SignalKMappedValueKind::Object && mapped.object_kind == SignalKObjectKind::AisTargets)) {
                flush(sink, frame);
                for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
                    publish_ais_target(sink, frame, timestamp, source_label, i, nullptr);
                }
                continue;
            }
            if (!mapped_ok) continue;

            if (frame.open() && (change.source_id != frame_source_id || frame.count() >= MaxBatchValues)) flush(sink, frame);
            if (!frame.open()) {
                if (!frame.begin(context(), timestamp, source_label)) {
                    ++dropped_publish_count_;
                    continue;
                }
                frame_source_id = change.source_id;
            }
            add_value(sink, frame, mapped);
        }

        flush(sink, frame);
    }

    // Sends the current typed model as deltas. With a cursor the snapshot is
    // resumable: if the sink refuses a delta (its write_signal_k_delta returns
    // false), *cursor is left at the first value of that delta and false is
    // returned; calling again with the same cursor continues from there.
    template<typename DeltaSink>
    bool publish_current(DeltaSink& sink, uint64_t now_us = 0, bool include_other_contexts = false, uint16_t* cursor = nullptr) {
        const size_t json_capacity = effective_json_buffer_size();
        if (json_capacity == 0) return true;
        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return true;

        uint16_t local_cursor = 0;
        uint16_t& position = cursor ? *cursor : local_cursor;
        if (position == 0) position = 1;

        SignalKTypedModelView<Real> view(store_, config_, now_us);
        SignalKDeltaFrame frame(json_, json_capacity);
        SourceId frame_source_id = 0;
        uint16_t frame_first = position;

        for (; position < SnapshotAisCursorBase; ++position) {
            SignalKMappedValue<Real> mapped;
            if (!view.selected_current_value(static_cast<ModelField>(position), mapped)) continue;
            if (frame.open() && (mapped.source_id != frame_source_id || frame.count() >= MaxBatchValues)) {
                if (!flush(sink, frame)) {
                    position = frame_first;
                    return false;
                }
            }
            if (!frame.open()) {
                if (!frame.begin(context(), timestamp, view.source_label(mapped.source_id))) {
                    ++dropped_publish_count_;
                    continue;
                }
                frame_source_id = mapped.source_id;
                frame_first = position;
            }
            if (!add_value(sink, frame, mapped, &frame_first, position)) {
                position = frame_first;
                return false;
            }
        }
        if (!flush(sink, frame)) {
            position = frame_first;
            return false;
        }

        const auto& ais_table = store_.model().ais.targets;
        const bool send_ais = include_other_contexts && ais_table.target_count.valid &&
            view.field_is_current(ModelField::AisTargetsObject, ais_table.target_count.last_update_us);
        if (send_ais) {
            const char* source_label = view.source_label(store_.source_id_for(ModelField::AisTargetsObject));
            for (; position < SnapshotCursorEnd; ++position) {
                const uint8_t slot = static_cast<uint8_t>(position - SnapshotAisCursorBase);
                if (!publish_ais_target(sink, frame, timestamp, source_label, slot, &view)) return false;
            }
        }
        position = 0;
        ++published_snapshot_count_;
        return true;
    }

    uint64_t dropped_publish_count() const { return dropped_publish_count_; }
    uint64_t published_delta_count() const { return published_delta_count_; }
    uint64_t published_value_count() const { return published_value_count_; }
    uint64_t published_snapshot_count() const { return published_snapshot_count_; }
    // Text values shortened because they could not fit an empty delta.
    uint64_t truncated_value_count() const { return truncated_value_count_; }

private:
    size_t effective_json_buffer_size() const {
        const size_t configured = static_cast<size_t>(config_.publisher.json_buffer_size);
        if (configured == 0) return 0;
        return configured < MaxJsonBufferSize ? configured : MaxJsonBufferSize;
    }

    const char* context() const {
        return config_.identity.self && config_.identity.self[0]
            ? config_.identity.self
            : "vessels.urn:mrn:signalk:uuid:00000000-0000-4000-8000-000000000001";
    }

    const char* label_for_source(SourceId source_id) const {
        if (source_id >= FirstConnectorSourceId && config_.connectors) {
            const size_t index = static_cast<size_t>(source_id - FirstConnectorSourceId);
            if (index < config_.connector_count) {
                const char* label = config_.connectors[index].label;
                if (label && label[0]) return label;
            }
        }
        return config_.publisher.source_label ? config_.publisher.source_label : config_.identity.server_name;
    }

    // A sink may return bool to refuse a delta (used by resumable snapshots);
    // sinks returning void always accept.
    template<typename DeltaSink>
    static bool deliver(DeltaSink& sink, const char* json, size_t len) {
        using Result = decltype(sink.write_signal_k_delta(json, len));
        if constexpr (std::is_same<Result, bool>::value) {
            return sink.write_signal_k_delta(json, len);
        } else {
            sink.write_signal_k_delta(json, len);
            return true;
        }
    }

    template<typename DeltaSink>
    bool flush(DeltaSink& sink, SignalKDeltaFrame& frame) {
        const size_t values = frame.count();
        const size_t len = frame.finish();
        if (len == 0) return true;
        if (!deliver(sink, json_, len)) return false;
        ++published_delta_count_;
        published_value_count_ += values;
        return true;
    }

    // Adds one path/value, sending the current delta and continuing in a new one
    // when it does not fit. Returns false only if the sink refused that delta.
    template<typename DeltaSink, typename WriteValue>
    bool add_or_split(DeltaSink& sink,
                      SignalKDeltaFrame& frame,
                      const char* path,
                      WriteValue&& write_value) {
        SignalKDeltaFrame::AddResult result = SignalKDeltaFrame::AddResult::Added;
        if (!add_or_split_result(sink, frame, path, write_value, result)) return false;
        if (result == SignalKDeltaFrame::AddResult::TooLarge) ++dropped_publish_count_;
        return true;
    }

    // `result` is TooLarge when the value does not fit even an empty delta.
    template<typename DeltaSink, typename WriteValue>
    bool add_or_split_result(DeltaSink& sink,
                             SignalKDeltaFrame& frame,
                             const char* path,
                             WriteValue&& write_value,
                             SignalKDeltaFrame::AddResult& result,
                             uint16_t* frame_first = nullptr,
                             uint16_t position = 0) {
        result = frame.add(path, write_value);
        if (result == SignalKDeltaFrame::AddResult::Full) {
            if (!flush(sink, frame)) return false;
            frame.reopen();
            if (frame_first) *frame_first = position;
            result = frame.add(path, write_value);
        }
        return true;
    }

    // Adds one mapped value. If it cannot fit even an empty delta, an object is
    // sent member by member under sub-paths (path.member, same Signal K tree),
    // and text is shortened to fit; only what still cannot fit is dropped.
    template<typename DeltaSink>
    bool add_value(DeltaSink& sink,
                   SignalKDeltaFrame& frame,
                   const SignalKMappedValue<Real>& mapped,
                   uint16_t* frame_first = nullptr,
                   uint16_t position = 0) {
        SignalKDeltaFrame::AddResult result = SignalKDeltaFrame::AddResult::Added;
        if (!add_or_split_result(sink, frame, mapped.path, [&](SignalKJsonStreamWriter& out) {
                return writer_.write_value(out, store_.model(), mapped);
            }, result, frame_first, position)) return false;
        if (result != SignalKDeltaFrame::AddResult::TooLarge) return true;

        if (mapped.kind == SignalKMappedValueKind::Text) {
            return add_truncated_text(sink, frame, mapped.path, mapped.text, mapped.text ? strlen(mapped.text) : 0);
        }
        if (mapped.kind != SignalKMappedValueKind::Object) {
            ++dropped_publish_count_;
            return true;
        }
        JsonDocument doc;
        JsonVariant value = doc.to<JsonVariant>();
        // An object with no members present has no value to send; that is not a drop.
        if (!writer_.build_object_value(store_.model(), mapped, value)) return true;
        return add_json_split(sink, frame, mapped.path, value, 0);
    }

    static constexpr uint8_t MaxObjectSplitDepth = 3;

    template<typename DeltaSink>
    bool add_json_split(DeltaSink& sink, SignalKDeltaFrame& frame, const char* path, JsonVariantConst value, uint8_t depth) {
        SignalKDeltaFrame::AddResult result = SignalKDeltaFrame::AddResult::Added;
        if (!add_or_split_result(sink, frame, path, [&](SignalKJsonStreamWriter& out) {
                return out.append_json(value);
            }, result)) return false;
        if (result != SignalKDeltaFrame::AddResult::TooLarge) return true;

        if (value.is<const char*>()) {
            const char* text = value.as<const char*>();
            return add_truncated_text(sink, frame, path, text, text ? strlen(text) : 0);
        }
        if (!value.is<JsonObjectConst>() || depth >= MaxObjectSplitDepth) {
            ++dropped_publish_count_;
            return true;
        }
        for (JsonPairConst member : value.as<JsonObjectConst>()) {
            char child_path[160];
            const int len = path && path[0]
                ? snprintf(child_path, sizeof(child_path), "%s.%s", path, member.key().c_str())
                : snprintf(child_path, sizeof(child_path), "%s", member.key().c_str());
            if (len <= 0 || static_cast<size_t>(len) >= sizeof(child_path)) {
                ++dropped_publish_count_;
                continue;
            }
            if (!add_json_split(sink, frame, child_path, member.value(), static_cast<uint8_t>(depth + 1))) return false;
        }
        return true;
    }

    // Sends the longest prefix of `text` that fits an empty delta.
    template<typename DeltaSink>
    bool add_truncated_text(DeltaSink& sink, SignalKDeltaFrame& frame, const char* path, const char* text, size_t len) {
        while (len > 0) {
            len = len > 16 ? len - len / 4 : len - 1;
            while (len > 0 && (static_cast<unsigned char>(text[len]) & 0xC0u) == 0x80u) --len;
            SignalKDeltaFrame::AddResult result = SignalKDeltaFrame::AddResult::Added;
            if (!add_or_split_result(sink, frame, path, [&](SignalKJsonStreamWriter& out) {
                    return out.append_quoted(text, len);
                }, result)) return false;
            if (result != SignalKDeltaFrame::AddResult::TooLarge) {
                ++truncated_value_count_;
                return true;
            }
        }
        ++dropped_publish_count_;
        return true;
    }

    // Publishes one AIS target table slot in its own vessel context. Large
    // targets are split across deltas like any other batch.
    template<typename DeltaSink>
    bool publish_ais_target(DeltaSink& sink,
                            SignalKDeltaFrame& frame,
                            const char* timestamp,
                            const char* source_label,
                            uint8_t slot,
                            const SignalKTypedModelView<Real>* current_view) {
        const auto& target = store_.model().ais.targets.targets[slot];
        if (!target.occupied || (current_view && !current_view->ais_target_is_current(target)) ||
            !target.mmsi.valid || !is_vessel_mmsi(target.mmsi.value)) return true;

        char mmsi[10];
        if (!format_mmsi(mmsi, sizeof(mmsi), target.mmsi.value)) return true;
        char target_context[64];
        const int context_len = snprintf(target_context, sizeof(target_context), "vessels.urn:mrn:imo:mmsi:%s", mmsi);
        if (context_len <= 0 || static_cast<size_t>(context_len) >= sizeof(target_context)) return true;
        if (!frame.begin(target_context, timestamp, source_label)) {
            ++dropped_publish_count_;
            return true;
        }

        auto add = [&](const char* path, auto&& write) { return add_or_split(sink, frame, path, write); };
        if (!add("", [&](SignalKJsonStreamWriter& out) {
                if (!out.append_char('{')) return false;
                if (target.vessel_name[0] &&
                    (!out.append_raw("\"name\":") || !out.append_quoted(target.vessel_name) || !out.append_char(','))) return false;
                return out.append_raw("\"mmsi\":") && out.append_quoted(mmsi) && out.append_char('}');
            })) return false;
        if (target.latitude_deg.valid && target.longitude_deg.valid &&
            !add("navigation.position", [&](SignalKJsonStreamWriter& out) {
                return out.append_raw("{\"latitude\":") && out.append_real(target.latitude_deg.value) &&
                       out.append_raw(",\"longitude\":") && out.append_real(target.longitude_deg.value) &&
                       out.append_char('}');
            })) return false;
        if (target.speed_over_ground_kn.valid &&
            !add("navigation.speedOverGround", [&](SignalKJsonStreamWriter& out) {
                return out.append_real(knots_to_mps<Real>(target.speed_over_ground_kn.value));
            })) return false;
        if (target.course_over_ground_deg.valid &&
            !add("navigation.courseOverGroundTrue", [&](SignalKJsonStreamWriter& out) {
                return out.append_real(deg_to_rad<Real>(target.course_over_ground_deg.value));
            })) return false;
        if (target.true_heading_deg.valid &&
            !add("navigation.headingTrue", [&](SignalKJsonStreamWriter& out) {
                return out.append_real(deg_to_rad<Real>(target.true_heading_deg.value));
            })) return false;
        return flush(sink, frame);
    }

    static bool is_vessel_mmsi(int32_t mmsi) {
        return mmsi >= 200000000 && mmsi <= 799999999;
    }

    static bool format_mmsi(char* dst, size_t dst_size, int32_t mmsi) {
        if (!dst || dst_size < 10 || !is_vessel_mmsi(mmsi)) return false;
        return snprintf(dst, dst_size, "%09ld", static_cast<long>(mmsi)) == 9;
    }

    ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
    SignalKMapper<Real> mapper_;
    SignalKDeltaWriter<Real> writer_;
    char json_[MaxJsonBufferSize]{};
    uint64_t dropped_publish_count_ = 0;
    uint64_t published_delta_count_ = 0;
    uint64_t published_value_count_ = 0;
    uint64_t published_snapshot_count_ = 0;
    uint64_t truncated_value_count_ = 0;
};

} // namespace signalk_mini
