#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.hpp"
#include "memory_profile.hpp"
#include "signalk_delta_writer.hpp"
#include "signalk_json_stream_writer.hpp"
#include "units.hpp"
#include "model_store.hpp"
#include "signalk_mapper.hpp"
#include "signalk_typed_view.hpp"

namespace signalk_mini {

template<typename Real, size_t MaxJsonBufferSize = DefaultSignalKJsonBufferSize, size_t MaxBatchValues = DefaultSignalKBatchValues>
class SignalKPublisher {
public:
    SignalKPublisher(ModelStore<Real>& store, const SignalKMiniConfig& config)
        : store_(store), config_(config) {
        static_assert(MaxBatchValues > 0, "SignalKPublisher batch size must be greater than zero");
    }

    template<typename DeltaSink>
    void publish_pending(DeltaSink& sink) {
        const size_t json_capacity = effective_json_buffer_size();
        if (json_capacity == 0) return;

        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return;

        ModelChange change;
        SourceId batch_source_id = 0;
        const char* batch_source_label = nullptr;
        size_t batch_count = 0;
        size_t processed = 0;
        const size_t max_changes = static_cast<size_t>(config_.publisher.max_changes_per_tick);

        while (processed < max_changes && store_.changes().pop(change)) {
            ++processed;

            const char* source_label = label_for_source(change.source_id);
            if (change.field == ModelField::AisTargetsObject) {
                flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
                publish_ais_targets(sink, json_capacity, timestamp, source_label);
                continue;
            }

            SignalKMappedValue<Real> mapped;
            if (!mapper_.map_change(store_.model(), change, mapped) || !mapped.path) continue;
            if (mapped.kind == SignalKMappedValueKind::Object) {
                flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
                if (mapped.object_kind == SignalKObjectKind::AisTargets) {
                    publish_ais_targets(sink, json_capacity, timestamp, source_label);
                } else {
                    publish_single(sink, json_capacity, timestamp, source_label, mapped);
                }
                continue;
            }

            if (batch_count != 0 && (change.source_id != batch_source_id || batch_count >= MaxBatchValues)) {
                flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
            }
            if (batch_count == 0) {
                batch_source_id = change.source_id;
                batch_source_label = source_label;
            }
            scalar_batch_[batch_count++] = mapped;
        }

        flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
    }

    template<typename DeltaSink>
    void publish_current(DeltaSink& sink, uint64_t now_us = 0, bool include_other_contexts = false) {
        const size_t json_capacity = effective_json_buffer_size();
        if (json_capacity == 0) return;
        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return;

        SignalKTypedModelView<Real> view(store_, config_, now_us);
        SourceId batch_source_id = 0;
        const char* batch_source_label = nullptr;
        size_t batch_count = 0;
        view.for_each_current_value([&](ModelField, const SignalKMappedValue<Real>& mapped) {
            const char* source_label = view.source_label(mapped.source_id);
            if (mapped.kind == SignalKMappedValueKind::Object) {
                flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
                publish_single(sink, json_capacity, timestamp, source_label, mapped);
                return;
            }
            if (batch_count != 0 && (mapped.source_id != batch_source_id || batch_count >= MaxBatchValues)) {
                flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
            }
            if (batch_count == 0) {
                batch_source_id = mapped.source_id;
                batch_source_label = source_label;
            }
            scalar_batch_[batch_count++] = mapped;
        });
        flush_scalar_batch(sink, json_capacity, timestamp, batch_source_label, scalar_batch_, batch_count);
        const auto& ais_table = store_.model().ais.targets;
        if (include_other_contexts && ais_table.target_count.valid &&
            view.field_is_current(ModelField::AisTargetsObject, ais_table.target_count.last_update_us)) {
            publish_ais_targets(sink, json_capacity, timestamp,
                                view.source_label(store_.source_id_for(ModelField::AisTargetsObject)), &view);
        }
        ++published_snapshot_count_;
    }

    uint64_t dropped_publish_count() const { return dropped_publish_count_; }
    uint64_t published_delta_count() const { return published_delta_count_; }
    uint64_t published_value_count() const { return published_value_count_; }
    uint64_t published_snapshot_count() const { return published_snapshot_count_; }

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

    int write_scalar_batch(char* dst,
                           size_t dst_size,
                           const char* timestamp,
                           const char* source_label,
                           const SignalKMappedValue<Real>* values,
                           size_t value_count) const {
        if (!dst || dst_size == 0 || !timestamp || !values || value_count == 0) return 0;

        SignalKJsonStreamWriter out(dst, dst_size);
        if (!out.append_raw("{\"context\":")) return 0;
        if (!out.append_quoted(context())) return 0;
        if (!out.append_raw(",\"updates\":[{\"timestamp\":")) return 0;
        if (!out.append_quoted(timestamp)) return 0;
        if (!out.append_raw(",\"source\":{\"label\":")) return 0;
        if (!out.append_quoted(source_label ? source_label : "signalk-mini")) return 0;
        if (!out.append_raw("},\"values\":[")) return 0;

        for (size_t i = 0; i < value_count; ++i) {
            const SignalKMappedValue<Real>& value = values[i];
            if (!value.path || value.kind == SignalKMappedValueKind::Object) return 0;
            if (i != 0 && !out.append_char(',')) return 0;
            if (!out.append_raw("{\"path\":")) return 0;
            if (!out.append_quoted(value.path)) return 0;
            if (!out.append_raw(",\"value\":")) return 0;
            if (!signalk_json_write_scalar_value(out, value.kind, value.number, value.boolean, value.text)) return 0;
            if (!out.append_char('}')) return 0;
        }

        if (!out.append_raw("]}]}")) return 0;
        if (!out.finish_crlf()) return 0;
        return out.ok() ? static_cast<int>(out.size()) : 0;
    }

    static bool insert_bytes(char* json,
                             size_t& len,
                             size_t capacity,
                             size_t offset,
                             const char* text,
                             size_t text_len) {
        if (!json || !text || offset > len || len + text_len >= capacity) return false;
        memmove(json + offset + text_len, json + offset, len - offset + 1);
        memcpy(json + offset, text, text_len);
        len += text_len;
        return true;
    }

    bool add_object_delta_envelope(char* json,
                                   size_t& len,
                                   size_t capacity,
                                   const char* timestamp) const {
        if (!json || !timestamp || len < 13 || strncmp(json, "{\"updates\":[{", 13) != 0) return false;

        char root_prefix[160];
        const int root_len = snprintf(root_prefix,
                                      sizeof(root_prefix),
                                      "\"context\":\"%s\",",
                                      context());
        if (root_len <= 0 || static_cast<size_t>(root_len) >= sizeof(root_prefix)) return false;
        if (!insert_bytes(json, len, capacity, 1, root_prefix, static_cast<size_t>(root_len))) return false;

        const char* updates = strstr(json, "\"updates\":[{");
        if (!updates) return false;
        const size_t update_offset = static_cast<size_t>(updates - json) + strlen("\"updates\":[{");

        char update_prefix[96];
        const int update_len = snprintf(update_prefix,
                                        sizeof(update_prefix),
                                        "\"timestamp\":\"%s\",",
                                        timestamp);
        if (update_len <= 0 || static_cast<size_t>(update_len) >= sizeof(update_prefix)) return false;
        return insert_bytes(json,
                            len,
                            capacity,
                            update_offset,
                            update_prefix,
                            static_cast<size_t>(update_len));
    }

    template<typename DeltaSink>
    void write_to_sink(DeltaSink& sink, const char* json, size_t len) {
        sink.write_signal_k_delta(json, len);
    }

    template<typename DeltaSink>
    void flush_scalar_batch(DeltaSink& sink,
                            size_t json_capacity,
                            const char* timestamp,
                            const char* source_label,
                            SignalKMappedValue<Real>* values,
                            size_t& value_count) {
        if (value_count == 0) return;
        const int len = write_scalar_batch(json_, json_capacity, timestamp, source_label, values, value_count);
        if (len <= 0 || static_cast<size_t>(len) >= json_capacity) {
            ++dropped_publish_count_;
            value_count = 0;
            return;
        }
        write_to_sink(sink, json_, static_cast<size_t>(len));
        ++published_delta_count_;
        published_value_count_ += value_count;
        value_count = 0;
    }

    template<typename DeltaSink>
    void publish_ais_targets(DeltaSink& sink,
                             size_t json_capacity,
                             const char* timestamp,
                             const char* source_label,
                             const SignalKTypedModelView<Real>* current_view = nullptr) {
        const auto& table = store_.model().ais.targets;
        for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
            const auto& target = table.targets[i];
            if (!target.occupied || (current_view && !current_view->ais_target_is_current(target)) ||
                !target.mmsi.valid || !is_vessel_mmsi(target.mmsi.value)) continue;
            const int len = write_ais_target_delta(json_, json_capacity, timestamp, source_label, target);
            if (len <= 0 || static_cast<size_t>(len) >= json_capacity) {
                ++dropped_publish_count_;
                continue;
            }
            write_to_sink(sink, json_, static_cast<size_t>(len));
            ++published_delta_count_;
        }
    }

    template<typename Target>
    int write_ais_target_delta(char* dst,
                               size_t capacity,
                               const char* timestamp,
                               const char* source_label,
                               const Target& target) const {
        char mmsi[10];
        if (!format_mmsi(mmsi, sizeof(mmsi), target.mmsi.value)) return 0;
        char context[64];
        const int context_len = snprintf(context, sizeof(context), "vessels.urn:mrn:imo:mmsi:%s", mmsi);
        if (context_len <= 0 || static_cast<size_t>(context_len) >= sizeof(context)) return 0;
        SignalKJsonStreamWriter out(dst, capacity);
        if (!out.append_raw("{\"context\":")) return 0;
        if (!out.append_quoted(context)) return 0;
        if (!out.append_raw(",\"updates\":[{\"timestamp\":")) return 0;
        if (!out.append_quoted(timestamp)) return 0;
        if (!out.append_raw(",\"source\":{\"label\":")) return 0;
        if (!out.append_quoted(source_label ? source_label : "signalk-mini")) return 0;
        if (!out.append_raw("},\"values\":[")) return 0;
        bool first = true;
        auto start_value = [&](const char* path) -> bool {
            if (!first && !out.append_char(',')) return false;
            first = false;
            return out.append_raw("{\"path\":") && out.append_quoted(path) && out.append_raw(",\"value\":");
        };
        if (target.vessel_name[0] || target.mmsi.valid) {
            if (!start_value("") || !out.append_char('{')) return 0;
            bool static_first = true;
            if (target.vessel_name[0]) {
                if (!out.append_raw("\"name\":") || !out.append_quoted(target.vessel_name)) return 0;
                static_first = false;
            }
            if (target.mmsi.valid) {
                if (!static_first && !out.append_char(',')) return 0;
                if (!out.append_raw("\"mmsi\":") || !out.append_quoted(mmsi)) return 0;
            }
            if (!out.append_raw("}}")) return 0;
        }
        if (target.latitude_deg.valid && target.longitude_deg.valid) {
            if (!start_value("navigation.position") || !out.append_char('{')) return 0;
            bool position_first = true;
            if (target.latitude_deg.valid) {
                if (!out.append_raw("\"latitude\":" ) || !out.append_real(target.latitude_deg.value)) return 0;
                position_first = false;
            }
            if (target.longitude_deg.valid) {
                if (!position_first && !out.append_char(',')) return 0;
                if (!out.append_raw("\"longitude\":" ) || !out.append_real(target.longitude_deg.value)) return 0;
            }
            if (!out.append_raw("}}")) return 0;
        }
        if (target.speed_over_ground_kn.valid) {
            if (!start_value("navigation.speedOverGround") || !out.append_real(knots_to_mps<Real>(target.speed_over_ground_kn.value)) || !out.append_char('}')) return 0;
        }
        if (target.course_over_ground_deg.valid) {
            if (!start_value("navigation.courseOverGroundTrue") || !out.append_real(deg_to_rad<Real>(target.course_over_ground_deg.value)) || !out.append_char('}')) return 0;
        }
        if (target.true_heading_deg.valid) {
            if (!start_value("navigation.headingTrue") || !out.append_real(deg_to_rad<Real>(target.true_heading_deg.value)) || !out.append_char('}')) return 0;
        }
        if (!out.append_raw("]}]}")) return 0;
        if (!out.finish_crlf()) return 0;
        return out.ok() ? static_cast<int>(out.size()) : 0;
    }

    static bool is_vessel_mmsi(int32_t mmsi) {
        return mmsi >= 200000000 && mmsi <= 799999999;
    }

    static bool format_mmsi(char* dst, size_t dst_size, int32_t mmsi) {
        if (!dst || dst_size < 10 || !is_vessel_mmsi(mmsi)) return false;
        return snprintf(dst, dst_size, "%09ld", static_cast<long>(mmsi)) == 9;
    }

    template<typename DeltaSink>
    void publish_single(DeltaSink& sink,
                        size_t json_capacity,
                        const char* timestamp,
                        const char* source_label,
                        const SignalKMappedValue<Real>& mapped) {
        int len = writer_.write_mapped(json_, json_capacity, source_label, store_.model(), mapped);
        size_t object_len = len > 0 ? static_cast<size_t>(len) : 0;
        if (object_len >= 2 && json_[object_len - 2] == '\r' && json_[object_len - 1] == '\n') {
            object_len -= 2;
            json_[object_len] = '\0';
        }
        if (len <= 0 || !add_object_delta_envelope(json_, object_len, json_capacity, timestamp) || object_len + 2 >= json_capacity) {
            ++dropped_publish_count_;
            return;
        }
        json_[object_len++] = '\r';
        json_[object_len++] = '\n';
        json_[object_len] = '\0';
        write_to_sink(sink, json_, object_len);
        ++published_delta_count_;
        ++published_value_count_;
    }

    ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
    SignalKMapper<Real> mapper_;
    SignalKDeltaWriter<Real> writer_;
    SignalKMappedValue<Real> scalar_batch_[MaxBatchValues]{};
    char json_[MaxJsonBufferSize]{};
    uint64_t dropped_publish_count_ = 0;
    uint64_t published_delta_count_ = 0;
    uint64_t published_value_count_ = 0;
    uint64_t published_snapshot_count_ = 0;
};

} // namespace signalk_mini
