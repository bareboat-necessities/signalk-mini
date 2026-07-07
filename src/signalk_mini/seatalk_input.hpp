#pragma once

#include <stddef.h>
#include <stdint.h>

#include <seatalk.hpp>

#include "model_store.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real>
class SeaTalkInput {
public:
    explicit SeaTalkInput(ModelStore<Real>& store) : store_(store) {}

    bool feed_datagram(const uint8_t* bytes, size_t length, SourceId source_id, uint64_t now_us) {
        const uint32_t before = receiver_.decoded_count();
        if (!receiver_.accept_datagram(bytes, length, store_.model(), now_us, ship_data_model::SensorSource::serial)) return false;
        mark_changed_if_decoded(before, source_id, now_us);
        return receiver_.decoded_count() != before;
    }

    bool feed_octets(const uint8_t* bytes, size_t length, SourceId source_id, uint64_t now_us) {
        const uint32_t before = receiver_.decoded_count();
        if (!receiver_.accept_octets(bytes, length, store_.model(), now_us, ship_data_model::SensorSource::serial)) return false;
        mark_changed_if_decoded(before, source_id, now_us);
        return receiver_.decoded_count() != before;
    }

    const seatalk::SeaTalkReceiver<Real>& receiver() const { return receiver_; }

private:
    void mark_changed_if_decoded(uint32_t before, SourceId source_id, uint64_t now_us) {
        if (receiver_.decoded_count() == before) return;
        mark_changed_from_decoded(receiver_.last_decoded(), source_id, now_us);
    }

    void mark_changed_from_decoded(const seatalk::SeaTalkDecoded<Real>& decoded, SourceId source_id, uint64_t now_us) {
        using Kind = seatalk::SeaTalkDecodedKind;
        switch (decoded.kind) {
        case Kind::depth:
            store_.mark_changed(ModelField::SeaDepthM, source_id, now_us);
            break;
        case Kind::apparent_wind_angle:
            store_.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
            break;
        case Kind::apparent_wind_speed:
            store_.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
            break;
        case Kind::heading_magnetic:
        case Kind::rudder_angle:
        case Kind::autopilot_state:
            store_.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
            break;
        case Kind::position_latitude:
            store_.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
            break;
        case Kind::position_longitude:
            store_.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
            break;
        case Kind::position_lat_lon:
            store_.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
            store_.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
            break;
        case Kind::speed_over_ground:
            store_.mark_changed(ModelField::GnssSpeedKn, source_id, now_us);
            break;
        case Kind::course_over_ground:
            store_.mark_changed(ModelField::GnssTrackDeg, source_id, now_us);
            break;
        default:
            break;
        }
    }

    ModelStore<Real>& store_;
    seatalk::SeaTalkReceiver<Real> receiver_;
};

} // namespace signalk_mini
