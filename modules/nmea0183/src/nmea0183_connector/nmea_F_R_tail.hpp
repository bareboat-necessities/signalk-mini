#pragma once

// Included inside Nmea0183RxConnector through nmea_S_T.hpp.
// Restored continuation for NMEA sentence group F-R.

template<typename Model>
bool apply_nrx(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short NRX"; return false; }
    auto& received = model.notifications.navtex.received; int32_t value = 0;
    if (parse_int32(sentence.field(0), value)) received.total_fragments.set(value, now_us);
    if (parse_int32(sentence.field(1), value)) received.fragment_number.set(value, now_us);
    nmea_copy_span(received.sentence_message_id, sizeof(received.sentence_message_id), sentence.field(2));
    const char* text = nullptr; size_t text_len = 0;
    if (sentence.fragment.is_fragmented) { if (!state_.navtex_message.complete) return true; text = state_.navtex_message.text; text_len = strlen(state_.navtex_message.text); received.complete = state_.navtex_message.complete; received.overflow = state_.navtex_message.overflow; } else { text = sentence.field(3).data; text_len = sentence.field(3).length; received.complete = true; received.overflow = false; }
    nmea_copy_span(received.message_text, sizeof(received.message_text), NmeaSpan(text, text_len));
    received.text_length.set(static_cast<int32_t>(strlen(received.message_text)), now_us);
    received.end_of_message = navtex_text_has_eom(received.message_text);
    received.framing_valid = strstr(received.message_text, "ZCZC") != nullptr && received.end_of_message;
    navtex_copy_body_text(received.body_text, sizeof(received.body_text), received.message_text);
    received.body_length.set(static_cast<int32_t>(strlen(received.body_text)), now_us);
    int32_t serial = 0; bool has_serial = false;
    parse_navtex_message_id(received.message_text, received.navtex_message_id, received.transmitter_id, received.subject_indicator, serial, has_serial);
    if (has_serial) { received.serial_number.set(serial, now_us); received.subject_category.set(navtex_subject_category(received.subject_indicator), now_us); nmea_copy_cstr(received.subject_label, sizeof(received.subject_label), navtex_subject_label(received.subject_indicator)); received.subject_is_service = received.subject_category.value == 20; }
    set_source(received.source, source); received.last_update_us = now_us; navtex_store_received_message(model.notifications.navtex, received, now_us); return true;
}

template<typename Model>
bool apply_nrm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short NRM"; return false; }
    auto& mask = model.notifications.navtex.receiver_mask; int32_t value = 0;
    if (parse_int32(sentence.field(0), value)) mask.total_fragments.set(value, now_us);
    if (parse_int32(sentence.field(1), value)) mask.fragment_number.set(value, now_us);
    nmea_copy_span(mask.sentence_message_id, sizeof(mask.sentence_message_id), sentence.field(2));
    nmea_copy_span(mask.receiver_id, sizeof(mask.receiver_id), sentence.field(3));
    if (sentence.field_count > 4) nmea_copy_span(mask.station_mask, sizeof(mask.station_mask), sentence.field(4));
    if (sentence.field_count > 5) nmea_copy_span(mask.subject_mask, sizeof(mask.subject_mask), sentence.field(5));
    int32_t enabled = 0; mask.station_mask_bits = navtex_letter_mask_bits(mask.station_mask, enabled); mask.enabled_station_count.set(enabled, now_us); mask.subject_mask_bits = navtex_letter_mask_bits(mask.subject_mask, enabled); mask.enabled_subject_count.set(enabled, now_us);
    if (sentence.field_count > 6) nmea_copy_span(mask.status_text, sizeof(mask.status_text), sentence.field(6));
    mask.complete = !sentence.fragment.is_fragmented || state_.navtex_message.complete;
    mask.overflow = sentence.fragment.is_fragmented && state_.navtex_message.overflow;
    set_source(mask.source, source); mask.last_update_us = now_us; return true;
}

template<typename Model>
bool apply_oln(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    const uint8_t count = sentence.field_count < 3 ? sentence.field_count : 3;
    if (count == 0) {
        last_error_ = "short OLN";
        return false;
    }

    for (uint8_t i = 0; i < count; ++i) {
        nmea_copy_span(model.nav.omega_lane_numbers.pair[i],
                       sizeof(model.nav.omega_lane_numbers.pair[i]),
                       sentence.field(i));
    }
    set_source(model.nav.omega_lane_numbers.source, source);
    model.nav.omega_lane_numbers.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_r00(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    const uint8_t count = sentence.field_count < 16 ? sentence.field_count : 16;
    if (count == 0) {
        last_error_ = "short R00";
        return false;
    }

    for (uint8_t i = 0; i < count; ++i) {
        nmea_copy_span(model.route.active.waypoint_id[i],
                       sizeof(model.route.active.waypoint_id[i]),
                       sentence.field(i));
    }
    model.route.active.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.route.active.source, source);
    model.route.active.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13 || sentence.field(0)[0] != 'A') {
        last_error_ = "bad RMB";
        return false;
    }

    float v = 0.0f;
    if (parse_left_right_signed(sentence.field(1), sentence.field(2), v)) {
        model.route.rmb.xte_nmi.set(static_cast<Real>(v), now_us);
        model.route.apb.xte_nmi.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.route.rmb.origin_id, sizeof(model.route.rmb.origin_id), sentence.field(3));
    nmea_copy_span(model.route.rmb.destination_id, sizeof(model.route.rmb.destination_id), sentence.field(4));
    if (parse_lat_lon(sentence.field(5), sentence.field(6), v)) model.route.rmb.destination_lat_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(7), sentence.field(8), v)) model.route.rmb.destination_lon_deg.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(9), v)) model.route.rmb.range_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(10), v)) {
        const Real bearing = static_cast<Real>(wrap_360_deg(v));
        model.route.rmb.bearing_deg.set(bearing, now_us);
        model.route.apb.track_deg.set(bearing, now_us);
    }
    if (parse_real(sentence.field(11), v)) model.route.rmb.closing_velocity_kn.set(static_cast<Real>(v), now_us);
    model.route.rmb.arrived.value = sentence.field(12)[0] == 'A';
    set_source(model.route.rmb.source, source);
    set_source(model.route.apb.source, source);
    model.route.rmb.last_update_us = now_us;
    model.route.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rlm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short RLM"; return false; }
    float v = 0.0f;
    nmea_copy_span(model.comm.return_link_message.beacon_id, sizeof(model.comm.return_link_message.beacon_id), sentence.field(0));
    if (parse_utc_time_of_day_s(sentence.field(1), v)) model.comm.return_link_message.reception_time_s.set(static_cast<Real>(v), now_us);
    model.comm.return_link_message.message_code = sentence.field(2)[0];
    nmea_copy_span(model.comm.return_link_message.message_body, sizeof(model.comm.return_link_message.message_body), sentence.field(3));
    set_source(model.comm.return_link_message.source, source);
    model.comm.return_link_message.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rot(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float v = 0.0f;
    if (sentence.field_count < 2 || sentence.field(1)[0] != 'A' || !parse_real(sentence.field(0), v)) { last_error_ = "bad ROT"; return false; }
    model.ins.imu.heading_rate_lowpass_deg_s.set(static_cast<Real>(v / 60.0f), now_us);
    return true;
}

template<typename Model>
bool apply_rpm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5 || sentence.field(4)[0] != 'A') { last_error_ = "bad RPM"; return false; }
    int32_t n = 0; float v = 0.0f;
    model.propulsion.revolutions.source_type = sentence.field(0)[0];
    model.propulsion.revolutions.status = sentence.field(4)[0];
    if (parse_int32(sentence.field(1), n)) model.propulsion.revolutions.number.set(n, now_us);
    if (parse_real(sentence.field(2), v)) model.propulsion.revolutions.speed_rpm.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(3), v)) model.propulsion.revolutions.propeller_pitch_percent.set(static_cast<Real>(v), now_us);
    set_source(model.propulsion.revolutions.source, source);
    model.propulsion.revolutions.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rsa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float v = 0.0f;
    if (sentence.field_count >= 2 && sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), v)) {
        model.steering.rudder.angle_deg.set(static_cast<Real>(-v), now_us);
    } else if (sentence.field_count >= 4 && sentence.field(3)[0] == 'A' && parse_real(sentence.field(2), v)) {
        model.steering.rudder.angle_deg.set(static_cast<Real>(-v), now_us);
    } else { last_error_ = "bad RSA"; return false; }
    set_source(model.steering.rudder.source, source);
    model.steering.rudder.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rsd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) { last_error_ = "short RSD"; return false; }
    float v = 0.0f;
    model.nav.radar_system.range_units = sentence.field(11)[0];
    if (parse_distance_nmi(sentence.field(0), sentence.field(11), v)) model.nav.radar_system.origin_range_nmi[0].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(1), v)) model.nav.radar_system.origin_bearing_deg[0].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(2), sentence.field(11), v)) model.nav.radar_system.variable_range_marker_nmi[0].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(3), v)) model.nav.radar_system.electronic_bearing_line_deg[0].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(4), sentence.field(11), v)) model.nav.radar_system.origin_range_nmi[1].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(5), v)) model.nav.radar_system.origin_bearing_deg[1].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(6), sentence.field(11), v)) model.nav.radar_system.variable_range_marker_nmi[1].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) model.nav.radar_system.electronic_bearing_line_deg[1].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(8), sentence.field(11), v)) model.nav.radar_system.cursor_range_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(9), v)) model.nav.radar_system.cursor_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(10), sentence.field(11), v)) model.nav.radar_system.range_scale_nmi.set(static_cast<Real>(v), now_us);
    set_source(model.nav.radar_system.source, source);
    model.nav.radar_system.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rte(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short RTE"; return false; }
    int32_t n = 0;
    if (parse_int32(sentence.field(0), n)) model.route.active.total_messages.set(n, now_us);
    if (parse_int32(sentence.field(1), n)) model.route.active.message_number.set(n, now_us);
    model.route.active.mode = sentence.field(2)[0];
    const uint8_t count = static_cast<uint8_t>((sentence.field_count - 3) < 16 ? (sentence.field_count - 3) : 16);
    for (uint8_t i = 0; i < count; ++i) {
        nmea_copy_span(model.route.active.waypoint_id[i], sizeof(model.route.active.waypoint_id[i]), sentence.field(static_cast<uint8_t>(3 + i)));
    }
    model.route.active.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.route.active.source, source);
    model.route.active.last_update_us = now_us;
    return true;
}
