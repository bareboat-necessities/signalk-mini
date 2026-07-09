#pragma once

#include <stddef.h>
#include <stdint.h>
#include "config.hpp"
#include "memory_profile.hpp"
#include "signalk_delta_writer.hpp"
#include "signalk_json_stream_writer.hpp"
#include "model_store.hpp"
#include "signalk_mapper.hpp"

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

        ModelChange change;
        SourceId batch_source_id = 0;
        const char* batch_source_label = nullptr;
        size_t batch_count = 0;
        size_t processed = 0;
        const size_t max_changes = static_cast<size_t>(config_.publisher.max_changes_per_tick);

        while (processed < max_changes && store_.changes().pop(change)) {
            ++processed;

            SignalKMappedValue<Real> mapped;
            if (!mapper_.map_change(store_.model(), change, mapped) || !mapped.path) continue;

            const char* source_label = label_for_source(change.source_id);
            if (mapped.kind == SignalKMappedValueKind::Object) {
                flush_scalar_batch(sink, json_capacity, batch_source_label, scalar_batch_, batch_count);
                publish_single(sink, json_capacity, source_label, mapped);
                continue;
            }

            if (batch_count != 0 && (change.source_id != batch_source_id || batch_count >= MaxBatchValues)) {
                flush_scalar_batch(sink, json_capacity, batch_source_label, scalar_batch_, batch_count);
            }
            if (batch_count == 0) {
                batch_source_id = change.source_id;
                batch_source_label = source_label;
            }
            scalar_batch_[batch_count++] = mapped;
        }

        flush_scalar_batch(sink, json_capacity, batch_source_label, scalar_batch_, batch_count);
    }

    uint64_t dropped_publish_count() const { return dropped_publish_count_; }
    uint64_t published_delta_count() const { return published_delta_count_; }
    uint64_t published_value_count() const { return published_value_count_; }

private:
    size_t effective_json_buffer_size() const {
        const size_t configured = static_cast<size_t>(config_.publisher.json_buffer_size);
        if (configured == 0) return 0;
        return configured < MaxJsonBufferSize ? configured : MaxJsonBufferSize;
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
                           const char* source_label,
                           const SignalKMappedValue<Real>* values,
                           size_t value_count) const {
        if (!dst || dst_size == 0 || !values || value_count == 0) return 0;

        SignalKJsonStreamWriter out(dst, dst_size);
        if (!out.append_raw("{\"updates\":[{\"source\":{\"label\":")) return 0;
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

    template<typename DeltaSink>
    void write_to_sink(DeltaSink& sink, const char* json, size_t len) {
        sink.write_signal_k_delta(json, len);
    }

    template<typename DeltaSink>
    void flush_scalar_batch(DeltaSink& sink,
                            size_t json_capacity,
                            const char* source_label,
                            SignalKMappedValue<Real>* values,
                            size_t& value_count) {
        if (value_count == 0) return;
        const int len = write_scalar_batch(json_, json_capacity, source_label, values, value_count);
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
    void publish_single(DeltaSink& sink,
                        size_t json_capacity,
                        const char* source_label,
                        const SignalKMappedValue<Real>& mapped) {
        const int len = writer_.write_mapped(json_, json_capacity, source_label, store_.model(), mapped);
        if (len <= 0 || static_cast<size_t>(len) >= json_capacity) {
            ++dropped_publish_count_;
            return;
        }
        write_to_sink(sink, json_, static_cast<size_t>(len));
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
};

} // namespace signalk_mini
