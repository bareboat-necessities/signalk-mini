#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_oln(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    const uint8_t count = sentence.field_count < 3 ? sentence.field_count : 3;
    if (count == 0) { last_error_ = "short OLN"; return false; }
    for (uint8_t i = 0; i < count; ++i) nmea_copy_span(model.nav.omega_lane_numbers.pair[i], sizeof(model.nav.omega_lane_numbers.pair[i]), sentence.field(i));
    set_source(model.nav.omega_lane_numbers.source, source);
    model.nav.omega_lane_numbers.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_osd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) { last_error_ = "short OSD"; return false; }
    float v = 0.0f;
    model.nav.own_ship.heading_status = sentence.field(1)[0];
    model.nav.own_ship.course_reference = sentence.field(3)[0];
    model.nav.own_ship.speed_reference = sentence.field(5)[0];
    if (sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), v)) {
        const Real h = static_cast<Real>(wrap_360_deg(v));
        model.nav.own_ship.heading_true_deg.set(h, now_us);
        model.imu.heading_deg.set(h, now_us);
        model.imu.heading_true_deg.set(h, now_us);
    }
    if (parse_real(sentence.field(2), v)) {
        model.nav.own_ship.course_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
        if (sentence.field(3)[0] == 'T') model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_knots(sentence.field(4), sentence.field(8), v)) {
        model.nav.own_ship.speed_kn.set(static_cast<Real>(v), now_us);
        model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(6), v)) {
        model.nav.own_ship.set_true_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
        model.sea.current_direction_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_knots(sentence.field(7), sentence.field(8), v)) {
        model.nav.own_ship.drift_speed_kn.set(static_cast<Real>(v), now_us);
        model.sea.current_speed_kn.set(static_cast<Real>(v), now_us);
    }
    set_source(model.nav.own_ship.source, source);
    set_source(model.gnss.fix.source, source);
    set_source(model.sea.source, source);
    model.nav.own_ship.last_update_us = now_us;
    model.gnss.fix.last_update_us = now_us;
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_r00(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    const uint8_t count = sentence.field_count < 16 ? sentence.field_count : 16;
    if (count == 0) { last_error_ = "short R00"; return false; }
    for (uint8_t i = 0; i < count; ++i) nmea_copy_span(model.route.active.waypoint_id[i], sizeof(model.route.active.waypoint_id[i]), sentence.field(i));
    model.route.active.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.route.active.source, source);
    model.route.active.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rma(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 11 || sentence.field(0)[0] != 'A') { last_error_ = "bad RMA"; return false; }
    float v = 0.0f;
    model.nav.rma.status = sentence.field(0)[0];
    if (parse_lat_lon(sentence.field(1), sentence.field(2), v)) { model.nav.rma.latitude_deg.set(static_cast<Real>(v), now_us); model.gnss.fix.fix_lat_deg.set(static_cast<Real>(v), now_us); }
    if (parse_lat_lon(sentence.field(3), sentence.field(4), v)) { model.nav.rma.longitude_deg.set(static_cast<Real>(v), now_us); model.gnss.fix.fix_lon_deg.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(5), v)) model.nav.rma.time_difference_a_us.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(6), v)) model.nav.rma.time_difference_b_us.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) { model.nav.rma.speed_kn.set(static_cast<Real>(v), now_us); model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(8), v)) { model.nav.rma.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_east_west_signed(sentence.field(9), sentence.field(10), v)) { model.nav.rma.magnetic_variation_deg.set(static_cast<Real>(v), now_us); model.gnss.fix.declination_deg.set(static_cast<Real>(v), now_us); model.imu.magnetic_variation_deg.set(static_cast<Real>(v), now_us); }
    set_source(model.nav.rma.source, source);
    set_source(model.gnss.fix.source, source);
    model.nav.rma.last_update_us = now_us;
    model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13 || sentence.field(0)[0] != 'A') { last_error_ = "bad RMB"; return false; }
    float v = 0.0f;
    if (parse_left_right_signed(sentence.field(1), sentence.field(2), v)) { model.route.rmb.xte_nmi.set(static_cast<Real>(v), now_us); model.route.apb.xte_nmi.set(static_cast<Real>(v), now_us); }
    nmea_copy_span(model.route.rmb.origin_id, sizeof(model.route.rmb.origin_id), sentence.field(3));
    nmea_copy_span(model.route.rmb.destination_id, sizeof(model.route.rmb.destination_id), sentence.field(4));
    if (parse_lat_lon(sentence.field(5), sentence.field(6), v)) model.route.rmb.destination_lat_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(7), sentence.field(8), v)) model.route.rmb.destination_lon_deg.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(9), v)) model.route.rmb.range_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(10), v)) { model.route.rmb.bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); model.route.apb.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_real(sentence.field(11), v)) model.route.rmb.closing_velocity_kn.set(static_cast<Real>(v), now_us);
    model.route.rmb.arrived.value = sentence.field(12)[0] == 'A';
    set_source(model.route.rmb.source, source);
    set_source(model.route.apb.source, source);
    model.route.rmb.last_update_us = now_us;
    model.route.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8 || sentence.field(1)[0] != 'A') { last_error_ = "invalid RMC"; return false; }
    float v = 0.0f;
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) model.gnss.fix.fix_lat_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(4), sentence.field(5), v)) model.gnss.fix.fix_lon_deg.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(6), v)) model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field_count >= 9 && parse_rmc_timestamp_s(sentence.field(0), sentence.field(8), v)) model.gnss.fix.timestamp_s.set(static_cast<Real>(v), now_us);
    if (sentence.field_count >= 11 && parse_east_west_signed(sentence.field(9), sentence.field(10), v)) model.gnss.fix.declination_deg.set(static_cast<Real>(v), now_us);
    set_source(model.gnss.fix.source, source);
    model.gnss.fix.last_update_us = now_us;
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
    float rate_deg_min = 0.0f;
    if (sentence.field_count < 2 || sentence.field(1)[0] != 'A' || !parse_real(sentence.field(0), rate_deg_min)) { last_error_ = "bad ROT"; return false; }
    model.imu.heading_rate_lowpass_deg_s.set(static_cast<Real>(rate_deg_min / 60.0f), now_us);
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
    float angle_deg = 0.0f;
    if (sentence.field_count >= 2 && sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), angle_deg)) model.steering.rudder.angle_deg.set(static_cast<Real>(-angle_deg), now_us);
    else if (sentence.field_count >= 4 && sentence.field(3)[0] == 'A' && parse_real(sentence.field(2), angle_deg)) model.steering.rudder.angle_deg.set(static_cast<Real>(-angle_deg), now_us);
    else { last_error_ = "bad RSA"; return false; }
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
    for (uint8_t i = 0; i < count; ++i) nmea_copy_span(model.route.active.waypoint_id[i], sizeof(model.route.active.waypoint_id[i]), sentence.field(static_cast<uint8_t>(3 + i)));
    model.route.active.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.route.active.source, source);
    model.route.active.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_sfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short SFI"; return false; }
    int32_t n = 0; float v = 0.0f;
    if (parse_int32(sentence.field(0), n)) model.nav.scanning_frequency.total_messages.set(n, now_us);
    if (parse_int32(sentence.field(1), n)) model.nav.scanning_frequency.message_number.set(n, now_us);
    for (uint8_t i = 0; i < 6; ++i) {
        const uint8_t base = static_cast<uint8_t>(2 + i * 2);
        if (base >= sentence.field_count) break;
        if (parse_real(sentence.field(base), v)) model.nav.scanning_frequency.frequency_hz[i].set(static_cast<Real>(v), now_us);
        if (base + 1 < sentence.field_count) model.nav.scanning_frequency.mode[i] = sentence.field(base + 1)[0];
    }
    set_source(model.nav.scanning_frequency.source, source);
    model.nav.scanning_frequency.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_stn(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    int32_t n = 0;
    if (sentence.field_count < 1 || !parse_int32(sentence.field(0), n)) { last_error_ = "bad STN"; return false; }
    model.nav.multiple_data_id.talker_id_number.set(n, now_us);
    set_source(model.nav.multiple_data_id.source, source);
    model.nav.multiple_data_id.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tds(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TDS"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.sea.trawl_door_centerline_offset_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.sea.trawl_door_along_centerline_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.sea.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short TFI"; return false; }
    int32_t n = 0;
    for (uint8_t i = 0; i < 3; ++i) if (parse_int32(sentence.field(i), n)) model.sea.trawl_catch_sensor_status[i].set(n, now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tlb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) { last_error_ = "short TLB"; return false; }
    int32_t n = 0; uint8_t count = 0;
    for (uint8_t i = 0; i + 1 < sentence.field_count && count < 8; i = static_cast<uint8_t>(i + 2)) {
        if (parse_int32(sentence.field(i), n)) {
            model.ais.tracked_target.label_target_number[count].set(n, now_us);
            nmea_copy_span(model.ais.tracked_target.label[count], sizeof(model.ais.tracked_target.label[count]), sentence.field(static_cast<uint8_t>(i + 1)));
            ++count;
        }
    }
    if (count == 0) { last_error_ = "bad TLB"; return false; }
    model.ais.tracked_target.label_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.ais.tracked_target.source, source);
    model.ais.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tll(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) { last_error_ = "short TLL"; return false; }
    int32_t n = 0; float v = 0.0f;
    if (parse_int32(sentence.field(0), n)) model.ais.tracked_target.target_number.set(n, now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), v)) model.ais.tracked_target.latitude_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), v)) model.ais.tracked_target.longitude_deg.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.ais.tracked_target.target_name, sizeof(model.ais.tracked_target.target_name), sentence.field(5));
    if (parse_utc_time_of_day_s(sentence.field(6), v)) model.ais.tracked_target.utc_time_s.set(static_cast<Real>(v), now_us);
    model.ais.tracked_target.target_status = sentence.field(7)[0];
    model.ais.tracked_target.reference_target = sentence.field(8)[0];
    set_source(model.ais.tracked_target.source, source);
    model.ais.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TPC"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.sea.trawl_cartesian_centerline_offset_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.sea.trawl_cartesian_along_centerline_m.set(static_cast<Real>(v), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.sea.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TPR"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.sea.trawl_relative_range_m.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(2), v)) model.sea.trawl_relative_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.sea.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short TPT"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) model.sea.trawl_true_range_m.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(2), v)) model.sea.trawl_true_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) model.sea.trawl_depth_below_surface_m.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_trf(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) { last_error_ = "short TRF"; return false; }
    int32_t n = 0; float v = 0.0f;
    model.nav.transit_fix.data_validity = sentence.field(11)[0];
    if (parse_utc_time_of_day_s(sentence.field(0), v)) model.nav.transit_fix.utc_time_s.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.nav.transit_fix.date_ddmmyy, sizeof(model.nav.transit_fix.date_ddmmyy), sentence.field(1));
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) { model.nav.transit_fix.latitude_deg.set(static_cast<Real>(v), now_us); if (sentence.field(11)[0] == 'A') model.gnss.fix.fix_lat_deg.set(static_cast<Real>(v), now_us); }
    if (parse_lat_lon(sentence.field(4), sentence.field(5), v)) { model.nav.transit_fix.longitude_deg.set(static_cast<Real>(v), now_us); if (sentence.field(11)[0] == 'A') model.gnss.fix.fix_lon_deg.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(6), v)) model.nav.transit_fix.elevation_angle_deg.set(static_cast<Real>(v), now_us);
    if (parse_int32(sentence.field(7), n)) model.nav.transit_fix.iterations.set(n, now_us);
    if (parse_int32(sentence.field(8), n)) model.nav.transit_fix.doppler_intervals.set(n, now_us);
    if (parse_real(sentence.field(9), v)) model.nav.transit_fix.update_distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_int32(sentence.field(10), n)) model.nav.transit_fix.satellite_number.set(n, now_us);
    set_source(model.nav.transit_fix.source, source);
    set_source(model.gnss.fix.source, source);
    model.nav.transit_fix.last_update_us = now_us;
    model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ttm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) { last_error_ = "short TTM"; return false; }
    int32_t n = 0; float v = 0.0f;
    model.ais.tracked_target.bearing_reference = sentence.field(3)[0];
    model.ais.tracked_target.course_reference = sentence.field(6)[0];
    model.ais.tracked_target.distance_units = sentence.field(9)[0];
    model.ais.tracked_target.target_status = sentence.field(11)[0];
    model.ais.tracked_target.reference_target = sentence.field(12)[0];
    if (parse_int32(sentence.field(0), n)) model.ais.tracked_target.target_number.set(n, now_us);
    if (parse_distance_nmi(sentence.field(1), sentence.field(9), v)) model.ais.tracked_target.distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(2), v)) model.ais.tracked_target.bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_real(sentence.field(4), v)) model.ais.tracked_target.speed_kn.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(5), v)) model.ais.tracked_target.course_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_distance_nmi(sentence.field(7), sentence.field(9), v)) model.ais.tracked_target.cpa_distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(8), v)) model.ais.tracked_target.tcpa_min.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.ais.tracked_target.target_name, sizeof(model.ais.tracked_target.target_name), sentence.field(10));
    set_source(model.ais.tracked_target.source, source);
    model.ais.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_txt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model; (void)now_us; (void)source;
    return accept_unmodeled_sentence(sentence);
}

template<typename Model>
bool apply_vbw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short VBW"; return false; }
    float v = 0.0f;
    model.sea.water_speed_status = sentence.field(2)[0];
    model.sea.ground_speed_status = sentence.field(5)[0];
    if (sentence.field(2)[0] == 'A') { if (parse_real(sentence.field(0), v)) { model.sea.longitudinal_water_speed_kn.set(static_cast<Real>(v), now_us); model.sea.speed_kn.set(static_cast<Real>(v), now_us); } if (parse_real(sentence.field(1), v)) model.sea.transverse_water_speed_kn.set(static_cast<Real>(v), now_us); }
    if (sentence.field(5)[0] == 'A') { if (parse_real(sentence.field(3), v)) model.sea.longitudinal_ground_speed_kn.set(static_cast<Real>(v), now_us); if (parse_real(sentence.field(4), v)) model.sea.transverse_ground_speed_kn.set(static_cast<Real>(v), now_us); }
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short VDR"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'T' && parse_real(sentence.field(0), v)) model.sea.current_direction_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.sea.current_direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_knots(sentence.field(4), sentence.field(5), v)) model.sea.current_speed_kn.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vhw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float speed_kn = 0.0f;
    if (!parse_knots(sentence.field(4), sentence.field(5), speed_kn)) { last_error_ = "bad VHW"; return false; }
    model.sea.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vlw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short VLW"; return false; }
    float v = 0.0f;
    if (parse_distance_nmi(sentence.field(0), sentence.field(1), v)) model.sea.total_distance_nmi.set(static_cast<Real>(v), now_us);
    if (parse_distance_nmi(sentence.field(2), sentence.field(3), v)) model.sea.trip_distance_nmi.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vpw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short VPW"; return false; }
    float v = 0.0f;
    if (parse_knots(sentence.field(0), sentence.field(1), v)) model.sea.speed_parallel_to_wind_kn.set(static_cast<Real>(v), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) model.sea.speed_parallel_to_wind_m_s.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vtg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 7) { last_error_ = "short VTG"; return false; }
    float v = 0.0f;
    if (parse_real(sentence.field(0), v)) model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_knots(sentence.field(4), sentence.field(5), v) || parse_knots(sentence.field(6), sentence.field(7), v)) model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us);
    set_source(model.gnss.fix.source, source);
    model.gnss.fix.last_update_us = now_us;
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
    model.route.rmb.closing_velocity_kn.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.route.rmb.destination_id, sizeof(model.route.rmb.destination_id), sentence.field(2));
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(2));
    set_source(model.route.rmb.source, source);
    set_source(model.route.waypoint.source, source);
    model.route.rmb.last_update_us = now_us;
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_wdc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model; (void)now_us; (void)source;
    return accept_unmodeled_sentence(sentence);
}

template<typename Model>
bool apply_wdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model; (void)now_us; (void)source;
    return accept_unmodeled_sentence(sentence);
}

template<typename Model>
bool apply_wnc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short WNC"; return false; }
    float v = 0.0f;
    if (parse_distance_nmi(sentence.field(0), sentence.field(1), v) || parse_distance_nmi(sentence.field(2), sentence.field(3), v)) model.route.waypoint.distance_nmi.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(4));
    nmea_copy_span(model.route.waypoint.from_waypoint_id, sizeof(model.route.waypoint.from_waypoint_id), sentence.field(5));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_wpl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) { last_error_ = "short WPL"; return false; }
    float v = 0.0f;
    if (parse_lat_lon(sentence.field(0), sentence.field(1), v)) model.route.waypoint.latitude_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) model.route.waypoint.longitude_deg.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(4));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
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
    model.route.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
    set_source(model.route.apb.source, source);
    model.route.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_xtr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float xte = 0.0f;
    if (sentence.field_count < 3 || sentence.field(2)[0] != 'N' || !parse_left_right_signed(sentence.field(0), sentence.field(1), xte)) { last_error_ = "bad XTR"; return false; }
    model.route.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
    model.route.rmb.xte_nmi.set(static_cast<Real>(xte), now_us);
    set_source(model.route.apb.source, source);
    set_source(model.route.rmb.source, source);
    model.route.apb.last_update_us = now_us;
    model.route.rmb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short ZDA"; return false; }
    float t = 0.0f; int32_t n = 0;
    if (!parse_utc_time_of_day_s(sentence.field(0), t)) { last_error_ = "bad ZDA"; return false; }
    model.gnss.fix.timestamp_s.set(static_cast<Real>(t), now_us);
    if (parse_int32(sentence.field(1), n)) model.gnss.fix.date_day.set(n, now_us);
    if (parse_int32(sentence.field(2), n)) model.gnss.fix.date_month.set(n, now_us);
    if (parse_int32(sentence.field(3), n)) model.gnss.fix.date_year.set(n, now_us);
    if (parse_int32(sentence.field(4), n)) model.gnss.fix.local_zone_hours.set(n, now_us);
    if (parse_int32(sentence.field(5), n)) model.gnss.fix.local_zone_minutes.set(n, now_us);
    set_source(model.gnss.fix.source, source);
    model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zdl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model; (void)now_us; (void)source;
    return sentence.field_count >= 3 ? accept_unmodeled_sentence(sentence) : (last_error_ = "short ZDL", false);
}

template<typename Model>
bool apply_zfo(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short ZFO"; return false; }
    float v = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) { last_error_ = "bad ZFO"; return false; }
    model.route.waypoint.origin_utc_time_s.set(static_cast<Real>(v), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), v) || parse_real(sentence.field(1), v)) model.route.waypoint.origin_elapsed_time_s.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.route.waypoint.from_waypoint_id, sizeof(model.route.waypoint.from_waypoint_id), sentence.field(2));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ztg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short ZTG"; return false; }
    float v = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) { last_error_ = "bad ZTG"; return false; }
    model.route.waypoint.destination_utc_time_s.set(static_cast<Real>(v), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), v) || parse_real(sentence.field(1), v)) model.route.waypoint.destination_time_remaining_s.set(static_cast<Real>(v), now_us);
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(2));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}
