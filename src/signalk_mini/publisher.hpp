#pragma once

#include <stddef.h>
#include <stdint.h>
#include <async_event_loop.hpp>
#include "config.hpp"
#include "signalk_delta_writer.hpp"
#include "model_store.hpp"
#include "signalk_mapper.hpp"

namespace signalk_mini {

template<typename Real>
class SignalKPublisher {
public:
    SignalKPublisher(ModelStore<Real>& store, const SignalKMiniConfig& config)
        : store_(store), config_(config) {}

    template<typename RuntimeConnectionRegistry>
    void publish_pending(RuntimeConnectionRegistry& connections) {
        SignalKMapper<Real> mapper;
        SignalKDeltaWriter<Real> writer;
        ModelChange change;
        size_t emitted = 0;
        while (emitted < config_.publisher.max_changes_per_tick && store_.changes().pop(change)) {
            SignalKMappedValue<Real> mapped;
            if (!mapper.map_change(store_.model(), change, mapped) || !mapped.path) continue;
            char json[512];
            const int len = writer.write_mapped(json, sizeof(json), config_.publisher.source_label, mapped);
            if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) continue;
            connections.for_each_tx([&](async_event_loop::ITcpConnection& connection) {
                connection.write(reinterpret_cast<const uint8_t*>(json), static_cast<size_t>(len));
            });
            ++emitted;
        }
    }

private:
    ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
};

} // namespace signalk_mini
