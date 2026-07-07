#pragma once

#include <string.h>
#include <ship_data_model.hpp>

#include "seatalk_decoder.hpp"

namespace seatalk {

inline ship_data_model::AutopilotMode seatalk_model_autopilot_mode(SeaTalkAutopilotMode mode) {
    switch (mode) {
    case SeaTalkAutopilotMode::standby: return ship_data_model::AutopilotMode::compass;
    case SeaTalkAutopilotMode::auto_heading: return ship_data_model::AutopilotMode::compass;
    case SeaTalkAutopilotMode::vane: return ship_data_model::AutopilotMode::wind;
    case SeaTalkAutopilotMode::track: return ship_data_model::AutopilotMode::nav;
    }
    return ship_data_model::AutopilotMode::compass;
}

inline void seatalk_copy_text(char* dst, size_t dst_len, const char* src) {
    if (!dst || dst_len == 0) return;
    dst[0] = '\0';
    if (!src) return;
    strncpy(dst, src, dst_len - 1);
    dst[dst_len - 1] = '\0';
}

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

    case SeaTalkDecodedKind::autopilot_state: {
        auto& ap = model.autopilot.controller;
        ap.mode.value = seatalk_model_autopilot_mode(decoded.autopilot_mode);
        ap.enabled.value = decoded.autopilot_mode != SeaTalkAutopilotMode::standby;
        ap.heading_deg.set(decoded.value, now_us);
        if (decoded.secondary_valid) ap.heading_command_deg.set(decoded.secondary_value, now_us);
        model.ins.imu.heading_deg.set(decoded.value, now_us);
        model.ins.imu.heading_magnetic_deg.set(decoded.value, now_us);
        if (decoded.third_valid) {
            model.steering.rudder.angle_deg.set(decoded.third_value, now_us);
            model.steering.rudder.source.value = source;
            model.steering.rudder.last_update_us = now_us;
        }
        if (decoded.alarm) {
            model.autopilot.status.warnings.set(decoded.code, now_us);
            seatalk_copy_text(model.notifications.messages.event.event_id, sizeof(model.notifications.messages.event.event_id), "seatalk_ap_alarm");
            seatalk_copy_text(model.notifications.messages.event.event_source, sizeof(model.notifications.messages.event.event_source), "seatalk");
            model.notifications.messages.event.event_state = 'A';
            seatalk_copy_text(model.notifications.messages.event.event_text, sizeof(model.notifications.messages.event.event_text), decoded.label);
            model.notifications.messages.event.field_count.set(1, now_us);
            model.notifications.messages.event.source.value = source;
            model.notifications.messages.event.last_update_us = now_us;
        }
        return true;
    }

    case SeaTalkDecodedKind::autopilot_key:
        seatalk_copy_text(model.notifications.messages.event.event_id, sizeof(model.notifications.messages.event.event_id), "seatalk_ap_key");
        seatalk_copy_text(model.notifications.messages.event.event_source, sizeof(model.notifications.messages.event.event_source), "seatalk");
        model.notifications.messages.event.event_state = decoded.long_press ? 'L' : 'S';
        seatalk_copy_text(model.notifications.messages.event.event_text, sizeof(model.notifications.messages.event.event_text), decoded.label);
        model.notifications.messages.event.field_count.set(1, now_us);
        model.notifications.messages.event.source.value = source;
        model.notifications.messages.event.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::navigation_to_waypoint:
        model.route.apb.xte_nmi.set(decoded.value, now_us);
        if (decoded.secondary_valid) {
            model.route.apb.present_to_destination_bearing_deg.set(decoded.secondary_value, now_us);
            model.route.apb.heading_to_steer_deg.set(decoded.secondary_value, now_us);
            model.route.waypoint.bearing_magnetic_deg.set(decoded.secondary_value, now_us);
        }
        if (decoded.third_valid) {
            model.route.apb.origin_to_destination_bearing_deg.set(decoded.secondary_value, now_us);
            model.route.waypoint.distance_nmi.set(decoded.third_value, now_us);
            model.route.rmb.range_nmi.set(decoded.third_value, now_us);
        }
        model.route.apb.mode_hint.value = ship_data_model::AutopilotMode::nav;
        model.route.apb.source.value = source;
        model.route.apb.last_update_us = now_us;
        model.route.waypoint.source.value = source;
        model.route.waypoint.last_update_us = now_us;
        return true;

    case SeaTalkDecodedKind::compass_variation:
        model.ins.imu.magnetic_variation_deg.set(decoded.value, now_us);
        model.gnss.fix.declination_deg.set(decoded.value, now_us);
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
