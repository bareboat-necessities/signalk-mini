#pragma once

#include <stddef.h>
#include <stdint.h>
#include <async_event_loop.hpp>
#include "config.hpp"
#include "signalk_delta_writer.hpp"
#include "model_store.hpp"
#include "signalk_mapper.hpp"

namespace signalk_mini {

template<typename Real, size_t MaxJsonBufferSize = 1024>
class SignalKPublisher {
public:
    SignalKPublisher(ModelStore<Real>& store, const SignalKMiniConfig& config)
        : store_(store), config_(config) {}

    template<typename RuntimeConnectionRegistry>
    void publish_pending(RuntimeConnectionRegistry& connections) {
        const size_t json_capacity = effective_json_buffer_size();
        if (json_capacity == 0) return;

        SignalKMapper<Real> mapper;
        SignalKDeltaWriter<Real> writer;
        ModelChange change;
        size_t emitted = 0;
        while (emitted < config_.publisher.max_changes_per_tick && store_.changes().pop(change)) {
            const char* source_label = label_for_source(change.source_id);
            int len = 0;

            if (change.field == ModelField::CommServerClockS) {
                const auto& clock = store_.model().comm.server.clock_s;
                if (!clock.valid) continue;
                len = writer.write_number(json_,
                                          json_capacity,
                                          source_label,
                                          "communication.server.clock",
                                          static_cast<Real>(clock.value));
            } else {
                SignalKMappedValue<Real> mapped;
                if (!mapper.map_change(store_.model(), change, mapped) || !mapped.path) continue;
                len = writer.write_mapped(json_, json_capacity, source_label, store_.model(), mapped);
            }

            if (len <= 0 || static_cast<size_t>(len) >= json_capacity) {
                ++dropped_publish_count_;
                continue;
            }
            connections.for_each_tx([&](async_event_loop::ITcpConnection& connection) {
                connection.write(reinterpret_cast<const uint8_t*>(json_), static_cast<size_t>(len));
            });
            ++published_delta_count_;
            ++emitted;
        }
    }

    uint64_t dropped_publish_count() const { return dropped_publish_count_; }
    uint64_t published_delta_count() const { return published_delta_count_; }

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

    ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
    char json_[MaxJsonBufferSize]{};
    uint64_t dropped_publish_count_ = 0;
    uint64_t published_delta_count_ = 0;
};

} // namespace signalk_mini
