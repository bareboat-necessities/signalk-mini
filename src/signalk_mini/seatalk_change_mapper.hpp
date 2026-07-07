#pragma once

#include <seatalk.hpp>

#include "model_store.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real, size_t QueueCapacity>
void mark_seatalk_changes(ModelStore<Real, QueueCapacity>& store,
                          const seatalk::SeaTalkDecoded<Real>& decoded,
                          SourceId source_id,
                          uint64_t now_us) {
    using Kind = seatalk::SeaTalkDecodedKind;

    switch (decoded.kind) {
    case Kind::depth:
        store.mark_changed(ModelField::SeaDepthM, source_id, now_us);
        break;

    case Kind::apparent_wind_angle:
        store.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
        break;

    case Kind::apparent_wind_speed:
        store.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
        break;

    case Kind::heading_magnetic:
    case Kind::rudder_angle:
    case Kind::autopilot_state:
        store.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        break;

    case Kind::position_latitude:
        store.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
        break;

    case Kind::position_longitude:
        store.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
        break;

    case Kind::position_lat_lon:
        store.mark_changed(ModelField::GnssFixLatDeg, source_id, now_us);
        store.mark_changed(ModelField::GnssFixLonDeg, source_id, now_us);
        break;

    case Kind::speed_over_ground:
        store.mark_changed(ModelField::GnssSpeedKn, source_id, now_us);
        break;

    case Kind::course_over_ground:
        store.mark_changed(ModelField::GnssTrackDeg, source_id, now_us);
        break;

    default:
        break;
    }
}

} // namespace signalk_mini
