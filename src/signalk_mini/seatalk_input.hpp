#pragma once

#include <stddef.h>
#include <stdint.h>

#include <seatalk.hpp>

#include "model_store.hpp"
#include "seatalk_change_mapper.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real, size_t QueueCapacity = 128>
class SeaTalkInput {
public:
    explicit SeaTalkInput(ModelStore<Real, QueueCapacity>& store) : store_(store) {}

    void set_stream_gap_timeout_us(uint64_t timeout_us) { stream_gap_timeout_us_ = timeout_us; }
    void reset_stream() { receiver_.reset_stream(); }

    bool feed_datagram(const uint8_t* bytes, size_t length, SourceId source_id, uint64_t now_us) {
        const uint32_t before = receiver_.decoded_count();
        if (!receiver_.accept_datagram(bytes, length, store_.model(), now_us, ship_data_model::SensorSource::serial)) return false;
        mark_changed_if_decoded(before, source_id, now_us);
        return receiver_.decoded_count() != before;
    }

    bool feed_octets(const uint8_t* bytes, size_t length, SourceId source_id, uint64_t now_us) {
        reset_stream_after_gap(now_us);
        last_stream_octets_us_ = now_us;

        const uint32_t before = receiver_.decoded_count();
        if (!receiver_.accept_octets(bytes, length, store_.model(), now_us, ship_data_model::SensorSource::serial)) return false;
        mark_changed_if_decoded(before, source_id, now_us);
        return receiver_.decoded_count() != before;
    }

    const seatalk::SeaTalkReceiver<Real>& receiver() const { return receiver_; }

private:
    void reset_stream_after_gap(uint64_t now_us) {
        if (stream_gap_timeout_us_ == 0 || last_stream_octets_us_ == 0) return;
        if (now_us >= last_stream_octets_us_ && now_us - last_stream_octets_us_ > stream_gap_timeout_us_) receiver_.reset_stream();
    }

    void mark_changed_if_decoded(uint32_t before, SourceId source_id, uint64_t now_us) {
        if (receiver_.decoded_count() == before) return;
        mark_seatalk_changes(store_, receiver_.last_decoded(), source_id, now_us);
    }

    ModelStore<Real, QueueCapacity>& store_;
    seatalk::SeaTalkReceiver<Real> receiver_;
    uint64_t last_stream_octets_us_ = 0;
    uint64_t stream_gap_timeout_us_ = 500000;
};

} // namespace signalk_mini
