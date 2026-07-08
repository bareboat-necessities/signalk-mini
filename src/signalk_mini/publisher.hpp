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
            SignalKMappedValue<Real> mapped;
            if (!mapper.map_change(store_.model(), change, mapped) || !mapped.path) continue;
            const int len = writer.write_mapped(json_, json_capacity, config_.publisher.source_label, store_.model(), mapped);
            if (len <= 0 || static_cast<size_t>(len) >= json_capacity) continue;
            connections.for_each_tx([&](async_event_loop::ITcpConnection& connection) {
                connection.write(reinterpret_cast<const uint8_t*>(json_), static_cast<size_t>(len));
            });
            ++emitted;
        }
    }

private:
    size_t effective_json_buffer_size() const {
        const size_t configured = static_cast<size_t>(config_.publisher.json_buffer_size);
        if (configured == 0) return 0;
        return configured < MaxJsonBufferSize ? configured : MaxJsonBufferSize;
    }

    ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
    char json_[MaxJsonBufferSize]{};
};

} // namespace signalk_mini
