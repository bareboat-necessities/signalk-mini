#pragma once

#include <stdio.h>
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

template<typename Model>
void seatalk_apply_message_text(Model& model, const char* id, const char* code, const char* text,
                                uint64_t now_us, ship_data_model::SensorSource source) {
    auto& msg = model.notifications.messages.text;
    seatalk_copy_text(msg.id, sizeof(msg.id), id);
    seatalk_copy_text(msg.code, sizeof(msg.code), code);
    seatalk_copy_text(msg.text, sizeof(msg.text), text);
    msg.field_count.set(1, now_us);
    msg.source.value = source;
    msg.last_update_us = now_us;
}

template<typename Model>
void seatalk_apply_waypoint_id(Model& model, const char* waypoint_id, uint64_t now_us, ship_data_model::SensorSource source) {
    if (!waypoint_id || waypoint_id[0] == '\0') return;
    seatalk_copy_text(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), waypoint_id);
    seatalk_copy_text(model.route.apb.destination_id, sizeof(model.route.apb.destination_id), waypoint_id);
    seatalk_copy_text(model.route.rmb.destination_id, sizeof(model.route.rmb.destination_id), waypoint_id);
    model.route.waypoint.source.value = source;
    model.route.waypoint.last_update_us = now_us;
    model.route.apb.source.value = source;
    model.route.apb.last_update_us = now_us;
    model.route.rmb.source.value = source;
    model.route.rmb.last_update_us = now_us;
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
    case SeaTalkDecodedKind::engine_rpm_pitch:
        model.propulsion.revolutions.speed_rpm.set(decoded.value, now_us);
        if (decoded.secondary_valid) model.propulsion.revolutions.propeller_pitch_percent.set(decoded.secondary_value, now_us);
        model.propulsion.revolutions.number.set(decoded.code, now_us);
        model.propulsion.revolutions.source_type = 'S';
        model.propulsion.revolutions.status = 'A';
        model.propulsion.revolutions.source.value = source;
        model.propulsion.revolutions.last_update_us = now_us;
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
        model.route.log.trip_distance_nmi.set(decoded.value, now_us);
        model.route.log.source.value = source;
        model.route.log.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::total_distance:
        model.route.log.total_distance_nmi.set(decoded.value, now_us);
        model.route.log.source.value = source;
        model.route.log.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::trip_total:
        model.route.log.total_distance_nmi.set(decoded.value, now_us);
        if (decoded.secondary_valid) model.route.log.trip_distance_nmi.set(decoded.secondary_value, now_us);
        model.route.log.source.value = source;
        model.route.log.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::display_units:
        seatalk_apply_message_text(model, "seatalk_display_units", "units", decoded.label, now_us, source);
        return true;
    case SeaTalkDecodedKind::lamp_intensity:
        seatalk_apply_message_text(model, "seatalk_lamp", "intensity", decoded.label, now_us, source);
        return true;
    case SeaTalkDecodedKind::position_latitude:
        model.gnss.fix.fix_lat_deg.set(decoded.value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::position_longitude:
        model.gnss.fix.fix_lon_deg.set(decoded.value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::position_lat_lon:
        model.gnss.fix.fix_lat_deg.set(decoded.value, now_us);
        if (decoded.secondary_valid) model.gnss.fix.fix_lon_deg.set(decoded.secondary_value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::speed_over_ground:
        model.gnss.fix.speed_kn.set(decoded.value, now_us);
        model.sea.longitudinal_ground_speed_kn.set(decoded.value, now_us);
        model.sea.ground_speed_status = 'A';
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        model.sea.source.value = source;
        model.sea.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::course_over_ground:
        model.gnss.fix.track_deg.set(decoded.value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::time_utc:
        model.gnss.fix.timestamp_s.set(decoded.value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::date_utc:
        model.gnss.fix.date_day.set(decoded.int_value, now_us);
        model.gnss.fix.date_month.set(decoded.int_secondary_value, now_us);
        model.gnss.fix.date_year.set(decoded.int_third_value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::satellite_info:
        model.gnss.fix.satellites_used.set(decoded.int_value, now_us);
        model.gnss.satellites_in_view.satellites_in_view.set(decoded.int_value, now_us);
        if (decoded.secondary_valid) model.gnss.fix.hdop.set(decoded.secondary_value, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        model.gnss.satellites_in_view.source.value = source;
        model.gnss.satellites_in_view.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::satellite_detail:
        if (decoded.code == 0x57) {
            model.gnss.fix.fix_quality.set(decoded.int_value, now_us);
            model.gnss.fix.satellites_used.set(decoded.int_secondary_value, now_us);
            if (decoded.secondary_valid) model.gnss.fix.hdop.set(decoded.value, now_us);
            if (decoded.int_third_value != 0) model.gnss.fix.dgps_station_id.set(decoded.int_third_value, now_us);
            model.gnss.fix.source.value = source;
            model.gnss.fix.last_update_us = now_us;
        } else if (decoded.int_value != 0) {
            model.gnss.satellites_in_view.satellite_prn[0].set(decoded.int_value, now_us);
            model.gnss.satellites_in_view.azimuth_true_deg[0].set(decoded.value, now_us);
            if (decoded.secondary_valid) model.gnss.satellites_in_view.elevation_deg[0].set(decoded.secondary_value, now_us);
            if (decoded.third_valid) model.gnss.satellites_in_view.snr_db[0].set(decoded.third_value, now_us);
            model.gnss.satellites_in_view.source.value = source;
            model.gnss.satellites_in_view.last_update_us = now_us;
        } else {
            seatalk_apply_message_text(model, "seatalk_satellite", "detail", decoded.label, now_us, source);
        }
        return true;
    case SeaTalkDecodedKind::differential_detail:
        model.gnss.satellites_in_view.satellite_prn[0].set(decoded.int_value, now_us);
        model.gnss.satellites_in_view.azimuth_true_deg[0].set(decoded.value, now_us);
        if (decoded.secondary_valid) model.gnss.satellites_in_view.elevation_deg[0].set(decoded.secondary_value, now_us);
        if (decoded.third_valid) model.gnss.satellites_in_view.snr_db[0].set(decoded.third_value, now_us);
        model.gnss.satellites_in_view.source.value = source;
        model.gnss.satellites_in_view.last_update_us = now_us;
        seatalk_apply_message_text(model, "seatalk_differential", "detail", decoded.label, now_us, source);
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
    case SeaTalkDecodedKind::waypoint_id:
    case SeaTalkDecodedKind::waypoint_name:
        seatalk_apply_waypoint_id(model, decoded.label, now_us, source);
        return true;
    case SeaTalkDecodedKind::waypoint_definition:
        seatalk_apply_message_text(model, "seatalk_waypoint_definition", "definition", decoded.label, now_us, source);
        return true;
    case SeaTalkDecodedKind::arrival_info:
        seatalk_copy_text(model.route.waypoint_arrival.waypoint_id, sizeof(model.route.waypoint_arrival.waypoint_id), decoded.label);
        seatalk_copy_text(model.route.apb.destination_id, sizeof(model.route.apb.destination_id), decoded.label);
        model.route.waypoint_arrival.perpendicular_passed.value = decoded.secondary_valid;
        model.route.waypoint_arrival.arrival_circle_entered.value = decoded.third_valid;
        model.route.apb.perpendicular_passed.value = decoded.secondary_valid;
        model.route.apb.arrival_circle_entered.value = decoded.third_valid;
        model.route.rmb.arrived.value = decoded.secondary_valid || decoded.third_valid;
        model.route.waypoint_arrival.source.value = source;
        model.route.waypoint_arrival.last_update_us = now_us;
        model.route.apb.source.value = source;
        model.route.apb.last_update_us = now_us;
        model.route.rmb.source.value = source;
        model.route.rmb.last_update_us = now_us;
        return true;
    case SeaTalkDecodedKind::device_status:
        seatalk_apply_message_text(model, "seatalk_device", "status", decoded.label, now_us, source);
        return true;
    case SeaTalkDecodedKind::observed_unknown:
        seatalk_apply_message_text(model, "seatalk_observed", "unknown", decoded.label, now_us, source);
        return true;
    case SeaTalkDecodedKind::none:
    default:
        return false;
    }
}

} // namespace seatalk
