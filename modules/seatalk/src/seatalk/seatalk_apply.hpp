#pragma once

#include <ship_data_model.hpp>

#include "seatalk_decoder.hpp"

namespace seatalk {

template<typename Real>
bool apply_seatalk_decoded(const SeaTalkDecoded<Real>& decoded,
                           ship_data_model::DataModel<Real>& model,
                           uint64_t now_us,
                           ship_data_model::SensorSource source) {
    switch (decoded.kind) {
    case SeaTalkDecodedKind::depth:
        model.sea.depth_m.set(decoded.value, now_us);
        model.sea.depth_below_surface_m.set(decoded.value, now_us);
        model.sea.depth_source.value = source;
        model.sea.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::apparent_wind_angle:
        model.wind.apparent.direction_deg.set(decoded.value, now_us);
        model.wind.apparent.source.value = source;
        model.wind.apparent.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::apparent_wind_speed:
        model.wind.apparent.speed_kn.set(decoded.value, now_us);
        if (decoded.secondary_valid) model.wind.apparent.speed_m_s.set(decoded.secondary_value, now_us);
        model.wind.apparent.source.value = source;
        model.wind.apparent.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::speed_through_water:
        model.sea.speed_kn.set(decoded.value, now_us);
        model.sea.longitudinal_water_speed_kn.set(decoded.value, now_us);
        model.sea.water_speed_status = 'A';
        model.sea.source.value = source;
        model.sea.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::water_temperature:
        model.sea.temperature_c.set(decoded.value, now_us);
        model.sea.source.value = source;
        model.sea.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::heading_magnetic:
        model.ins.imu.heading_deg.set(decoded.value, now_us);
        model.ins.imu.heading_magnetic_deg.set(decoded.value, now_us);
        return true;

    case SeaTalkDecodedKind::rudder_angle:
        model.steering.rudder.angle_deg.set(decoded.value, now_us);
        model.steering.rudder.source.value = source;
        model.steering.rudder.last_update_us = now_us;
        if (decoded.secondary_valid) {
            model.ins.imu.heading_deg.set(decoded.secondary_value, now_us);
            model.ins.imu.heading_magnetic_deg.set(decoded.secondary_value, now_us);
        }
        return true;

    case SeaTalkDecodedKind::trip_distance:
        model.sea.trip_distance_nmi.set(decoded.value, now_us);
        model.sea.source.value = source;
        model.sea.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::total_distance:
        model.sea.total_distance_nmi.set(decoded.value, now_us);
        model.sea.source.value = source;
        model.sea.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::none:
    default:
        return false;
    }
}

} // namespace seatalk
