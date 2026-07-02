#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_oln(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    const uint8_t count = sentence.field_count < 3 ? sentence.field_count : 3;
    if (count == 0) { last_error_ = "short OLN"; return false; }
    for (uint8_t i = 0; i < count; ++i) nmea_copy_span(model.navigation.omega_lane_numbers.pair[i], sizeof(model.navigation.omega_lane_numbers.pair[i]), sentence.field(i));
    set_source(model.navigation.omega_lane_numbers.source, source);
    model.navigation.omega_lane_numbers.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_osd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) { last_error_ = "short OSD"; return false; }
    float v = 0.0f;
    model.navigation.own_ship.heading_status = sentence.field(1)[0];
    model.navigation.own_ship.course_reference = sentence.field(3)[0];
    model.navigation.own_ship.speed_reference = sentence.field(5)[0];
    if (sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), v)) {
        const Real h = static_cast<Real>(wrap_360_deg(v));
        model.navigation.own_ship.heading_true_deg.set(h, now_us);
        model.imu.heading_deg.set(h, now_us);
        model.imu.heading_true_deg.set(h, now_us);
    }
    if (parse_real(sentence.field(2), v)) {
        model.navigation.own_ship.course_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
        if (sentence.field(3)[0] == 'T') model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_knots(sentence.field(4), sentence.field(8), v)) {
        model.navigation.own_ship.speed_kn.set(static_cast<Real>(v), now_us);
        model.navigation.gps.speed_kn.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(6), v)) {
        model.navigation.own_ship.set_true_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
        model.water.current_direction_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_knots(sentence.field(7), sentence.field(8), v)) {
        model.navigation.own_ship.drift_speed_kn.set(static_cast<Real>(v), now_us);
        model.water.current_speed_kn.set(static_cast<Real>(v), now_us);
    }
    set_source(model.navigation.own_ship.source, source);
    set_source(model.navigation.gps.source, source);
    set_source(model.water.source, source);
    model.navigation.own_ship.last_update_us = now_us;
    model.navigation.gps.last_update_us = now_us;
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_r00(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    const uint8_t count = sentence.field_count < 16 ? sentence.field_count : 16;
    if (count == 0) { last_error_ = "short R00"; return false; }
    for (uint8_t i = 0; i < count; ++i) nmea_copy_span(model.navigation.active_route.waypoint_id[i], sizeof(model.navigation.active_route.waypoint_id[i]), sentence.field(i));
    model.navigation.active_route.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.navigation.active_route.source, source);
    model.navigation.active_route.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rma(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 11 || sentence.field(0)[0] != 'A') { last_error_ = "bad RMA"; return false; }
    float v = 0.0f;
    model.navigation.rma.status = sentence.field(0)[0];
    if (parse_lat_lon(sentence.field(1), sentence.field(2), v)) { model.navigation.rma.latitude_deg.set(static_cast<Real>(v), now_us); model.navigation.gps.fix_lat_deg.set(static_cast<Real>(v), now_us); }
    if (parse_lat_lon(sentence.field(3), sentence.field(4), v)) { model.navigation.rma.longitude_deg.set(static_cast<Real>(v), now_us); model.navigation.gps.fix_lon_deg.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(5), v)) model.navigation.rma.time_difference_a_us.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(6), v)) model.navigation.rma.time_difference_b_us.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) { model.navigation.rma.speed_kn.set(static_cast<Real>(v), now_us); model.navigation.gps.speed_kn.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(8), v)) { model.navigation.rma.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_east_west_signed(sentence.field(9), sentence.field(10), v)) { model.navigation.rma.magnetic_variation_deg.set(static_cast<Real>(v), now_us); model.navigation.gps.declination_deg.set(static_cast<Real>(v), now_us); model.imu.magnetic_variation_deg.set(static_cast<Real>(v), now_us); }
    set_source(model.navigation.rma.source, source);
    set_source(model.navigation.gps.source, source);
    model.navigation.rma.last_update_us = now_us;
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13 || sentence.field(0)[0] != 'A') { last_error_ = "bad RMB"; return false; }
    float v = 0.0f;
    if (parse_left_right_signed(sentence.field(1), sentence.field(2), v)) { model.navigation.rmb.xte_nmi.set(static_cast<Real>(v), now_us); model.navigation.apb.xte_nmi.set(static_cast<Real>(v), now_us); }
    nmea_copy_span(model.navigation.rmb.origin_id, sizeof(model.navigation.rmb.origin_id), sentence.field(3));
    nmea_copy_span(model.navigation.rmb.destination_id, sizeof(model.navigation.rmb.destination_id), sentence.field(4));
    if (parse_lat_lon(sentence.field(5), sentence.field(6), v)) model.navigation.rmb.destination_lat_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(7), sentence.field(8), v)) model.navigation.rmb.destination_lon_deg.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(9), v)) model.navigation.rmb.range_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(10), v)) { model.navigation.rmb.bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); model.navigation.apb.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_real(sentence.field(11), v)) model.navigation.rmb.closing_velocity_kn.set(static_cast<Real>(v), now_us);
    model.navigation.rmb.arrived.value = sentence.field(12)[0] == 'A';
    set_source(model.navigation.rmb.source, source);
    set_source(model.navigation.apb.source, source);
    model.navigation.rmb.last_update_us = now_us;
    model.navigation.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8 || sentence.field(1)[0] != 'A') { last_error_ = "invalid RMC"; return false; }
    float v = 0.0f;
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(4), sentence.field(5), v)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(6), v)) model.navigation.gps.speed_kn.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field_count >= 9 && parse_rmc_timestamp_s(sentence.field(0), sentence.field(8), v)) model.navigation.gps.timestamp_s.set(static_cast<Real>(v), now_us);
    if (sentence.field_count >= 11 && parse_east_west_signed(sentence.field(9), sentence.field(10), v)) model.navigation.gps.declination_deg.set(static_cast<Real>(v), now_us);
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rlm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short RLM"; return false; }
    float v = 0.0f;
    nmea_copy_span(model.navigation.return_link_message.beacon_id, sizeof(model.navigation.return_link_message.beacon_id), sentence.field(0));
    if (parse_utc_time_of_day_s(sentence.field(1), v)) model.navigation.return_link_message.reception_time_s.set(static_cast<Real>(v), now_us);
    model.navigation.return_link_message.message_code = sentence.field(2)[0];
    nmea_copy_span(model.navigation.return_link_message.message_body, sizeof(model.navigation.return_link_message.message_body), sentence.field(3));
    set_source(model.navigation.return_link_message.source, source);
    model.navigation.return_link_message.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rot(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float rate_deg_min = 0.0f;
    if (sentence.field_count < 2 || sentence.field(1)[0] != 'A' || !parse_real(sentence.field(0), rate_deg_min)) { last_error_ = "bad ROT"; return false; }
    model.imu.heading_rate_lowpass_deg_s.set(static_cast<Real>(rate_deg_min / 60.0f), now_us);
    return true;
}

template<typename Model>
bool apply_rpm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5 || sentence.field(4)[0] != 'A') { last_error_ = "bad RPM"; return false; }
    int32_t n = 0; float v = 0.0f;
    model.navigation.revolutions.source_type = sentence.field(0)[0];
    model.navigation.revolutions.status = sentence.field(4)[0];
    if (parse_int32(sentence.field(1), n)) model.navigation.revolutions.number.set(n, now_us);
    if (parse_real(sentence.field(2), v)) model.navigation.revolutions.speed_rpm.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(3), v)) model.navigation.revolutions.propeller_pitch_percent.set(static_cast<Real>(v), now_us);
    set_source(model.navigation.revolutions.source, source);
    model.navigation.revolutions.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rsa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float angle_deg = 0.0f;
    if (sentence.field_count >= 2 && sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), angle_deg)) model.rudder.angle_deg.set(static_cast<Real>(-angle_deg), now_us);
    else if (sentence.field_count >= 4 && sentence.field(3)[0] == 'A' && parse_real(sentence.field(2), angle_deg)) model.rudder.angle_deg.set(static_cast<Real>(-angle_deg), now_us);
    else { last_error_ = "bad RSA"; return false; }
    set_source(model.rudder.source, source);
    model.rudder.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rsd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) { last_error_ = "short RSD"; return false; }
    float v = 0.0f;
    model.navigation.radar_system.range_units = sentence.field(11)[0];
    if (parse_distance_nmi(sentence.field(0), sentence.field(11), v)) model.navigation.radar_system.origin_range_nmi[0].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(1), v)) model.navigation.radar_system.origin_bearing_deg[0].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(2), sentence.field(11), v)) model.navigation.radar_system.variable_range_marker_nmi[0].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(3), v)) model.navigation.radar_system.electronic_bearing_line_deg[0].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(4), sentence.field(11), v)) model.navigation.radar_system.origin_range_nmi[1].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(5), v)) model.navigation.radar_system.origin_bearing_deg[1].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(6), sentence.field(11), v)) model.navigation.radar_system.variable_range_marker_nmi[1].set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) model.navigation.radar_system.electronic_bearing_line_deg[1].set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(8), sentence.field(11), v)) model.navigation.radar_system.cursor_range_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(9), v)) model.navigation.radar_system.cursor_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(10), sentence.field(11), v)) model.navigation.radar_system.range_scale_nmi.set(static_cast<Real>(v), now_us);
    set_source(model.navigation.radar_system.source, source);
    model.navigation.radar_system.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rte(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short RTE"; return false; }
    int32_t n = 0;
    if (parse_int32(sentence.field(0), n)) model.navigation.active_route.total_messages.set(n, now_us);
    if (parse_int32(sentence.field(1), n)) model.navigation.active_route.message_number.set(n, now_us);
    model.navigation.active_route.mode = sentence.field(2)[0];
    const uint8_t count = static_cast<uint8_t>((sentence.field_count - 3) < 16 ? (sentence.field_count - 3) : 16);
    for (uint8_t i = 0; i < count; ++i) nmea_copy_span(model.navigation.active_route.waypoint_id[i], sizeof(model.navigation.active_route.waypoint_id[i]), sentence.field(static_cast<uint8_t>(3 + i)));
    model.navigation.active_route.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.navigation.active_route.source, source);
    model.navigation.active_route.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_sfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short SFI"; return false; }
    int32_t n = 0; float v = 0.0f;
    if (parse_int32(sentence.field(0), n)) model.navigation.scanning_frequency.total_messages.set(n, now_us);
    if (parse_int32(sentence.field(1), n)) model.navigation.scanning_frequency.message_number.set(n, now_us);
    for (uint8_t i = 0; i < 6; ++i) {
        const uint8_t base = static_cast<uint8_t>(2 + i * 2);
        if (base >= sentence.field_count) break;
        if (parse_real(sentence.field(base), v)) model.navigation.scanning_frequency.frequency_hz[i].set(static_cast<Real>(v), now_us);
        if (base + 1 < sentence.field_count) model.navigation.scanning_frequency.mode[i] = sentence.field(base + 1)[0];
    }
    set_source(model.navigation.scanning_frequency.source, source);
    model.navigation.scanning_frequency.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_stn(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    int32_t n = 0;
    if (sentence.field_count < 1 || !parse_int32(sentence.field(0), n)) { last_error_ = "bad STN"; return false; }
    model.navigation.multiple_data_id.talker_id_number.set(n, now_us);
    set_source(model.navigation.multiple_data_id.source, source);
    model.navigation.multiple_data_id.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tds(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TDS"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.water.trawl_door_centerline_offset_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.water.trawl_door_along_centerline_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.water.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short TFI"; return false; }
    int32_t n = 0;
    for (uint8_t i = 0; i < 3; ++i) if (parse_int32(sentence.field(i), n)) model.water.trawl_catch_sensor_status[i].set(n, now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tlb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) { last_error_ = "short TLB"; return false; }
    int32_t n = 0; uint8_t count = 0;
    for (uint8_t i = 0; i + 1 < sentence.field_count && count < 8; i = static_cast<uint8_t>(i + 2)) {
        if (parse_int32(sentence.field(i), n)) {
            model.navigation.tracked_target.label_target_number[count].set(n, now_us);
            nmea_copy_span(model.navigation.tracked_target.label[count], sizeof(model.navigation.tracked_target.label[count]), sentence.field(static_cast<uint8_t>(i + 1)));
            ++count;
        }
    }
    if (count == 0) { last_error_ = "bad TLB"; return false; }
    model.navigation.tracked_target.label_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.navigation.tracked_target.source, source);
    model.navigation.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tll(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) { last_error_ = "short TLL"; return false; }
    int32_t n = 0; float v = 0.0f;
    if (parse_int32(sentence.field(0), n)) model.navigation.tracked_target.target_number.set(n, now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), v)) model.navigation.tracked_target.latitude_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), v)) model.navigation.tracked_target.longitude_deg.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.tracked_target.target_name, sizeof(model.navigation.tracked_target.target_name), sentence.field(5));
    if (parse_utc_time_of_day_s(sentence.field(6), v)) model.navigation.tracked_target.utc_time_s.set(static_cast<Real>(v), now_us);
    model.navigation.tracked_target.target_status = sentence.field(7)[0];
    model.navigation.tracked_target.reference_target = sentence.field(8)[0];
    set_source(model.navigation.tracked_target.source, source);
    model.navigation.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TPC"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.water.trawl_cartesian_centerline_offset_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.water.trawl_cartesian_along_centerline_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.water.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TPR"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.water.trawl_relative_range_m.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(2), v)) model.water.trawl_relative_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.water.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TPT"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.water.trawl_true_range_m.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(2), v)) model.water.trawl_true_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.water.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_trf(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) { last_error_ = "short TRF"; return false; }
    int32_t n = 0; float v = 0.0f;
    model.navigation.transit_fix.data_validity = sentence.field(11)[0];
    if (parse_utc_time_of_day_s(sentence.field(0), v)) model.navigation.transit_fix.utc_time_s.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.transit_fix.date_ddmmyy, sizeof(model.navigation.transit_fix.date_ddmmyy), sentence.field(1));
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) { model.navigation.transit_fix.latitude_deg.set(static_cast<Real>(v), now_us); if (sentence.field(11)[0] == 'A') model.navigation.gps.fix_lat_deg.set(static_cast<Real>(v), now_us); }
    if (parse_lat_lon(sentence.field(4), sentence.field(5), v)) { model.navigation.transit_fix.longitude_deg.set(static_cast<Real>(v), now_us); if (sentence.field(11)[0] == 'A') model.navigation.gps.fix_lon_deg.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(6), v)) model.navigation.transit_fix.elevation_angle_deg.set(static_cast<Real>(v), now_us);
    if (parse_int32(sentence.field(7), n)) model.navigation.transit_fix.iterations.set(n, now_us);
    if (parse_int32(sentence.field(8), n)) model.navigation.transit_fix.doppler_intervals.set(n, now_us);
    if (parse_real(sentence.field(9), v)) model.navigation.transit_fix.update_distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_int32(sentence.field(10), n)) model.navigation.transit_fix.satellite_number.set(n, now_us);
    set_source(model.navigation.transit_fix.source, source);
    set_source(model.navigation.gps.source, source);
    model.navigation.transit_fix.last_update_us = now_us;
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ttm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) { last_error_ = "short TTM"; return false; }
    int32_t n = 0; float v = 0.0f;
    model.navigation.tracked_target.bearing_reference = sentence.field(3)[0];
    model.navigation.tracked_target.course_reference = sentence.field(6)[0];
    model.navigation.tracked_target.distance_units = sentence.field(9)[0];
    model.navigation.tracked_target.target_status = sentence.field(11)[0];
    model.navigation.tracked_target.reference_target = sentence.field(12)[0];
    if (parse_int32(sentence.field(0), n)) model.navigation.tracked_target.target_number.set(n, now_us);
    if (parse_distance_nmi(sentence.field(1), sentence.field(9), v)) model.navigation.tracked_target.distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(2), v)) model.navigation.tracked_target.bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_real(sentence.field(4), v)) model.navigation.tracked_target.speed_kn.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(5), v)) model.navigation.tracked_target.course_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(7), sentence.field(9), v)) model.navigation.tracked_target.cpa_distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(8), v)) model.navigation.tracked_target.tcpa_min.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.tracked_target.target_name, sizeof(model.navigation.tracked_target.target_name), sentence.field(10));
    set_source(model.navigation.tracked_target.source, source);
    model.navigation.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_txt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    return apply_raw_sentence_record(sentence, state_.text_sentence, "TXT", now_us, source);
}

template<typename Model>
bool apply_vbw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short VBW"; return false; }
    float v = 0.0f;
    model.water.water_speed_status = sentence.field(2)[0];
    model.water.ground_speed_status = sentence.field(5)[0];
    if (sentence.field(2)[0] == 'A') { if (parse_real(sentence.field(0), v)) { model.water.longitudinal_water_speed_kn.set(static_cast<Real>(v), now_us); model.water.speed_kn.set(static_cast<Real>(v), now_us); } if (parse_real(sentence.field(1), v)) model.water.transverse_water_speed_kn.set(static_cast<Real>(v), now_us); }
    if (sentence.field(5)[0] == 'A') { if (parse_real(sentence.field(3), v)) model.water.longitudinal_ground_speed_kn.set(static_cast<Real>(v), now_us); if (parse_real(sentence.field(4), v)) model.water.transverse_ground_speed_kn.set(static_cast<Real>(v), now_us); }
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short VDR"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'T' && parse_real(sentence.field(0), v)) model.water.current_direction_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.water.current_direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_knots(sentence.field(4), sentence.field(5), v)) model.water.current_speed_kn.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vhw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float speed_kn = 0.0f;
    if (!parse_knots(sentence.field(4), sentence.field(5), speed_kn)) { last_error_ = "bad VHW"; return false; }
    model.water.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vlw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short VLW"; return false; }
    float v = 0.0f;
    if (parse_distance_nmi(sentence.field(0), sentence.field(1), v)) model.water.total_distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_distance_nmi(sentence.field(2), sentence.field(3), v)) model.water.trip_distance_nmi.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vpw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short VPW"; return false; }
    float v = 0.0f;
    if (parse_knots(sentence.field(0), sentence.field(1), v)) model.water.speed_parallel_to_wind_kn.set(static_cast<Real>(v), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.water.speed_parallel_to_wind_m_s.set(static_cast<Real>(v), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vtg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 7) { last_error_ = "short VTG"; return false; }
    float v = 0.0f;
    if (parse_real(sentence.field(0), v)) model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_knots(sentence.field(4), sentence.field(5), v) || parse_knots(sentence.field(6), sentence.field(7), v)) model.navigation.gps.speed_kn.set(static_cast<Real>(v), now_us);
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vwr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source, bool true_wind) {
    float angle_deg = 0.0f, speed_kn = 0.0f;
    if (!parse_left_right_signed(sentence.field(0), sentence.field(1), angle_deg) || !parse_knots(sentence.field(2), sentence.field(3), speed_kn)) { last_error_ = true_wind ? "bad VWT" : "bad VWR"; return false; }
    return true_wind ? set_wind(model.wind.truewind, angle_deg, speed_kn, now_us, source) : set_wind(model.wind.apparent, angle_deg, speed_kn, now_us, source);
}

template<typename Model>
bool apply_wcv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short WCV"; return false; }
    float v = 0.0f;
    if (!parse_knots(sentence.field(0), sentence.field(1), v)) { last_error_ = "bad WCV"; return false; }
    model.navigation.rmb.closing_velocity_kn.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.rmb.destination_id, sizeof(model.navigation.rmb.destination_id), sentence.field(2));
    nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), sentence.field(2));
    set_source(model.navigation.rmb.source, source);
    set_source(model.navigation.waypoint.source, source);
    model.navigation.rmb.last_update_us = now_us;
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_wdc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    return apply_raw_sentence_record(sentence, state_.waypoint_distance_great_circle, "WDC", now_us, source);
}

template<typename Model>
bool apply_wdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    return apply_raw_sentence_record(sentence, state_.waypoint_distance_rhumb, "WDR", now_us, source);
}

template<typename Model>
bool apply_wnc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short WNC"; return false; }
    float v = 0.0f;
    if (parse_distance_nmi(sentence.field(0), sentence.field(1), v) || parse_distance_nmi(sentence.field(2), sentence.field(3), v)) model.navigation.waypoint.distance_nmi.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), sentence.field(4));
    nmea_copy_span(model.navigation.waypoint.from_waypoint_id, sizeof(model.navigation.waypoint.from_waypoint_id), sentence.field(5));
    set_source(model.navigation.waypoint.source, source);
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_wpl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) { last_error_ = "short WPL"; return false; }
    float v = 0.0f;
    if (parse_lat_lon(sentence.field(0), sentence.field(1), v)) model.navigation.waypoint.latitude_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) model.navigation.waypoint.longitude_deg.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), sentence.field(4));
    set_source(model.navigation.waypoint.source, source);
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_xdr(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    bool any = false; float v = 0.0f;
    for (uint8_t i = 0; i + 3 < sentence.field_count; i = static_cast<uint8_t>(i + 4)) {
        if (sentence.field(i)[0] == 'A' && parse_real(sentence.field(i + 1), v) && sentence.field(i + 2)[0] == 'D') {
            if (nmea_span_equals(sentence.field(i + 3), "PTCH")) { model.imu.pitch_deg.set(static_cast<Real>(v), now_us); any = true; }
            else if (nmea_span_equals(sentence.field(i + 3), "ROLL")) { model.imu.roll_deg.set(static_cast<Real>(v), now_us); any = true; }
        }
    }
    if (!any) last_error_ = "bad XDR";
    return any;
}

template<typename Model>
bool apply_xte(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float xte = 0.0f;
    if (sentence.field_count < 5 || sentence.field(0)[0] != 'A' || sentence.field(1)[0] != 'A' || !parse_left_right_signed(sentence.field(2), sentence.field(3), xte)) { last_error_ = "bad XTE"; return false; }
    model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
    set_source(model.navigation.apb.source, source);
    model.navigation.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_xtr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float xte = 0.0f;
    if (sentence.field_count < 3 || sentence.field(2)[0] != 'N' || !parse_left_right_signed(sentence.field(0), sentence.field(1), xte)) { last_error_ = "bad XTR"; return false; }
    model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
    model.navigation.rmb.xte_nmi.set(static_cast<Real>(xte), now_us);
    set_source(model.navigation.apb.source, source);
    set_source(model.navigation.rmb.source, source);
    model.navigation.apb.last_update_us = now_us;
    model.navigation.rmb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short ZDA"; return false; }
    float t = 0.0f; int32_t n = 0;
    if (!parse_utc_time_of_day_s(sentence.field(0), t)) { last_error_ = "bad ZDA"; return false; }
    model.navigation.gps.timestamp_s.set(static_cast<Real>(t), now_us);
    if (parse_int32(sentence.field(1), n)) model.navigation.gps.date_day.set(n, now_us);
    if (parse_int32(sentence.field(2), n)) model.navigation.gps.date_month.set(n, now_us);
    if (parse_int32(sentence.field(3), n)) model.navigation.gps.date_year.set(n, now_us);
    if (parse_int32(sentence.field(4), n)) model.navigation.gps.local_zone_hours.set(n, now_us);
    if (parse_int32(sentence.field(5), n)) model.navigation.gps.local_zone_minutes.set(n, now_us);
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zdl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    if (sentence.field_count < 3) { last_error_ = "short ZDL"; return false; }
    float v = 0.0f;
    if (parse_utc_time_of_day_s(sentence.field(0), v) || parse_real(sentence.field(0), v)) { state_.variable_point.time_to_point_s = v; state_.variable_point.has_time = true; }
    if (parse_distance_nmi(sentence.field(1), sentence.field_count > 2 ? sentence.field(2) : NmeaSpan(), v)) { state_.variable_point.distance_to_point_nmi = v; state_.variable_point.has_distance = true; }
    if (sentence.field_count > 3) nmea_copy_span(state_.variable_point.point_id, sizeof(state_.variable_point.point_id), sentence.field(3));
    set_source(state_.variable_point.source, source);
    state_.variable_point.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zfo(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short ZFO"; return false; }
    float v = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) { last_error_ = "bad ZFO"; return false; }
    model.navigation.waypoint.origin_utc_time_s.set(static_cast<Real>(v), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), v) || parse_real(sentence.field(1), v)) model.navigation.waypoint.origin_elapsed_time_s.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.waypoint.from_waypoint_id, sizeof(model.navigation.waypoint.from_waypoint_id), sentence.field(2));
    set_source(model.navigation.waypoint.source, source);
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ztg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short ZTG"; return false; }
    float v = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) { last_error_ = "bad ZTG"; return false; }
    model.navigation.waypoint.destination_utc_time_s.set(static_cast<Real>(v), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), v) || parse_real(sentence.field(1), v)) model.navigation.waypoint.destination_time_remaining_s.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), sentence.field(2));
    set_source(model.navigation.waypoint.source, source);
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}
