#pragma once

#include <stddef.h>
#include <stdint.h>
#include <ArduinoJson.h>
#include <async_event_loop.hpp>
#include "config.hpp"
#include "signalk_delta_writer.hpp"
#include "model_store.hpp"
#include "signalk_mapper.hpp"

namespace signalk_mini {

template<typename Real, size_t MaxJsonBufferSize = 1024, size_t MaxBatchValues = 32>
class SignalKPublisher {
public:
    SignalKPublisher(ModelStore<Real>& store, const SignalKMiniConfig& config)
        : store_(store), config_(config) {
        static_assert(MaxBatchValues > 0, "SignalKPublisher batch size must be greater than zero");
    }

    template<typename RuntimeConnectionRegistry>
    void publish_pending(RuntimeConnectionRegistry& connections) {
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
                flush_scalar_batch(connections, json_capacity, batch_source_label, scalar_batch_, batch_count);
                publish_single(connections, json_capacity, source_label, mapped);
                continue;
            }

            if (batch_count != 0 && (change.source_id != batch_source_id || batch_count >= MaxBatchValues)) {
                flush_scalar_batch(connections, json_capacity, batch_source_label, scalar_batch_, batch_count);
            }
            if (batch_count == 0) {
                batch_source_id = change.source_id;
                batch_source_label = source_label;
            }
            scalar_batch_[batch_count++] = mapped;
        }

        flush_scalar_batch(connections, json_capacity, batch_source_label, scalar_batch_, batch_count);
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
        if (source_id >= 10 && config_.connectors) {
            const size_t index = static_cast<size_t>(source_id - 10);
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

        JsonDocument doc;
        JsonArray updates = doc["updates"].to<JsonArray>();
        JsonObject update = updates.add<JsonObject>();
        JsonObject source = update["source"].to<JsonObject>();
        source["label"] = source_label ? source_label : "signalk-mini";
        JsonArray items = update["values"].to<JsonArray>();

        for (size_t i = 0; i < value_count; ++i) {
            const SignalKMappedValue<Real>& value = values[i];
            if (!value.path || value.kind == SignalKMappedValueKind::Object) return 0;
            JsonObject item = items.add<JsonObject>();
            item["path"] = value.path;
            if (value.kind == SignalKMappedValueKind::Number) item["value"] = value.number;
            else if (value.kind == SignalKMappedValueKind::Bool) item["value"] = value.boolean;
            else if (value.kind == SignalKMappedValueKind::Text) item["value"] = value.text ? value.text : "";
            else return 0;
        }

        const size_t len = serializeJson(doc, dst, dst_size);
        if (len + 2 >= dst_size) return 0;
        dst[len] = '\r';
        dst[len + 1] = '\n';
        dst[len + 2] = '\0';
        return static_cast<int>(len + 2);
    }

    template<typename RuntimeConnectionRegistry>
    void write_to_connections(RuntimeConnectionRegistry& connections, const char* json, size_t len) {
        connections.for_each_tx([&](async_event_loop::ITcpConnection& connection) {
            connection.write(reinterpret_cast<const uint8_t*>(json), len);
        });
    }

    template<typename RuntimeConnectionRegistry>
    void flush_scalar_batch(RuntimeConnectionRegistry& connections,
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
        write_to_connections(connections, json_, static_cast<size_t>(len));
        ++published_delta_count_;
        published_value_count_ += value_count;
        value_count = 0;
    }

    template<typename RuntimeConnectionRegistry>
    void publish_single(RuntimeConnectionRegistry& connections,
                        size_t json_capacity,
                        const char* source_label,
                        const SignalKMappedValue<Real>& mapped) {
        const int len = writer_.write_mapped(json_, json_capacity, source_label, store_.model(), mapped);
        if (len <= 0 || static_cast<size_t>(len) >= json_capacity) {
            ++dropped_publish_count_;
            return;
        }
        write_to_connections(connections, json_, static_cast<size_t>(len));
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
