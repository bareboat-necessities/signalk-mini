#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_oln(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) {
        last_error_ = "short OLN";
        return false;
    }

    const uint8_t count = sentence.field_count < 3 ? sentence.field_count : 3;
    for (uint8_t index = 0; index < count; ++index) {
        nmea_copy_span(model.navigation.omega_lane_numbers.pair[index],
                       sizeof(model.navigation.omega_lane_numbers.pair[index]),
                       sentence.field(index));
    }
    set_source(model.navigation.omega_lane_numbers.source, source);
    model.navigation.omega_lane_numbers.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_osd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) {
        last_error_ = "short OSD";
        return false;
    }

    float value = 0.0f;
    model.navigation.own_ship.heading_status = sentence.field(1)[0];
    model.navigation.own_ship.course_reference = sentence.field(3)[0];
    model.navigation.own_ship.speed_reference = sentence.field(5)[0];

    if (sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), value)) {
        const Real heading = static_cast<Real>(wrap_360_deg(value));
        model.navigation.own_ship.heading_true_deg.set(heading, now_us);
        model.imu.heading_deg.set(heading, now_us);
        model.imu.heading_true_deg.set(heading, now_us);
    }
    if (parse_real(sentence.field(2), value)) {
        model.navigation.own_ship.course_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
        if (sentence.field(3)[0] == 'T') model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    }
    if (parse_knots(sentence.field(4), sentence.field(8), value)) {
        model.navigation.own_ship.speed_kn.set(static_cast<Real>(value), now_us);
        model.navigation.gps.speed_kn.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(6), value)) {
        model.navigation.own_ship.set_true_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
        model.water.current_direction_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    }
    if (parse_knots(sentence.field(7), sentence.field(8), value)) {
        model.navigation.own_ship.drift_speed_kn.set(static_cast<Real>(value), now_us);
        model.water.current_speed_kn.set(static_cast<Real>(value), now_us);
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
    if (sentence.field_count < 1) {
        last_error_ = "short R00";
        return false;
    }

    const uint8_t count = sentence.field_count < 16 ? sentence.field_count : 16;
    for (uint8_t index = 0; index < count; ++index) {
        nmea_copy_span(model.navigation.active_route.waypoint_id[index],
                       sizeof(model.navigation.active_route.waypoint_id[index]),
                       sentence.field(index));
    }
    model.navigation.active_route.waypoint_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.navigation.active_route.source, source);
    model.navigation.active_route.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rma(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 11 || sentence.field(0)[0] != 'A') {
        last_error_ = "bad RMA";
        return false;
    }

    float value = 0.0f;
    model.navigation.rma.status = sentence.field(0)[0];
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) {
        model.navigation.rma.latitude_deg.set(static_cast<Real>(value), now_us);
        model.navigation.gps.fix_lat_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) {
        model.navigation.rma.longitude_deg.set(static_cast<Real>(value), now_us);
        model.navigation.gps.fix_lon_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(5), value)) model.navigation.rma.time_difference_a_us.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.navigation.rma.time_difference_b_us.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) {
        model.navigation.rma.speed_kn.set(static_cast<Real>(value), now_us);
        model.navigation.gps.speed_kn.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(8), value)) {
        model.navigation.rma.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
        model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    }
    if (parse_east_west_signed(sentence.field(9), sentence.field(10), value)) {
        model.navigation.rma.magnetic_variation_deg.set(static_cast<Real>(value), now_us);
        model.navigation.gps.declination_deg.set(static_cast<Real>(value), now_us);
        model.imu.magnetic_variation_deg.set(static_cast<Real>(value), now_us);
    }

    set_source(model.navigation.rma.source, source);
    set_source(model.navigation.gps.source, source);
    model.navigation.rma.last_update_us = now_us;
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13 || sentence.field(0)[0] != 'A') {
        last_error_ = "bad RMB";
        return false;
    }

    float value = 0.0f;
    if (parse_left_right_signed(sentence.field(1), sentence.field(2), value)) {
        model.navigation.rmb.xte_nmi.set(static_cast<Real>(value), now_us);
        model.navigation.apb.xte_nmi.set(static_cast<Real>(value), now_us);
    }
    nmea_copy_span(model.navigation.rmb.origin_id, sizeof(model.navigation.rmb.origin_id), sentence.field(3));
    nmea_copy_span(model.navigation.rmb.destination_id, sizeof(model.navigation.rmb.destination_id), sentence.field(4));
    if (parse_lat_lon(sentence.field(5), sentence.field(6), value)) model.navigation.rmb.destination_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(7), sentence.field(8), value)) model.navigation.rmb.destination_lon_deg.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.navigation.rmb.range_nmi.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(10), value)) {
        model.navigation.rmb.bearing_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
        model.navigation.apb.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    }
    if (parse_real(sentence.field(11), value)) model.navigation.rmb.closing_velocity_kn.set(static_cast<Real>(value), now_us);
    model.navigation.rmb.arrived.value = sentence.field(12)[0] == 'A';
    set_source(model.navigation.rmb.source, source);
    set_source(model.navigation.apb.source, source);
    model.navigation.rmb.last_update_us = now_us;
    model.navigation.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8 || sentence.field(1)[0] != 'A') {
        last_error_ = "invalid RMC";
        return false;
    }

    float value = 0.0f;
    if (parse_lat_lon(sentence.field(2), sentence.field(3), value)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(4), sentence.field(5), value)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.navigation.gps.speed_kn.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (sentence.field_count >= 9 && parse_rmc_timestamp_s(sentence.field(0), sentence.field(8), value)) {
        model.navigation.gps.timestamp_s.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count >= 11 && parse_east_west_signed(sentence.field(9), sentence.field(10), value)) {
        model.navigation.gps.declination_deg.set(static_cast<Real>(value), now_us);
    }
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rot(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float rate_deg_min = 0.0f;
    if (sentence.field_count < 2 || sentence.field(1)[0] != 'A' || !parse_real(sentence.field(0), rate_deg_min)) {
        last_error_ = "bad ROT";
        return false;
    }

    model.imu.heading_rate_lowpass_deg_s.set(static_cast<Real>(rate_deg_min / 60.0f), now_us);
    return true;
}

template<typename Model>
bool apply_rsa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float angle_deg = 0.0f;
    if (sentence.field_count >= 2 && sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), angle_deg)) {
        model.rudder.angle_deg.set(static_cast<Real>(-angle_deg), now_us);
    } else if (sentence.field_count >= 4 && sentence.field(3)[0] == 'A' && parse_real(sentence.field(2), angle_deg)) {
        model.rudder.angle_deg.set(static_cast<Real>(-angle_deg), now_us);
    } else {
        last_error_ = "bad RSA";
        return false;
    }

    set_source(model.rudder.source, source);
    model.rudder.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vhw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float speed_kn = 0.0f;
    if (!parse_knots(sentence.field(4), sentence.field(5), speed_kn)) {
        last_error_ = "bad VHW";
        return false;
    }

    model.water.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vtg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 7) {
        last_error_ = "short VTG";
        return false;
    }

    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_knots(sentence.field(4), sentence.field(5), value) || parse_knots(sentence.field(6), sentence.field(7), value)) {
        model.navigation.gps.speed_kn.set(static_cast<Real>(value), now_us);
    }
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vwr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source, bool true_wind) {
    float angle_deg = 0.0f;
    float speed_kn = 0.0f;
    if (!parse_left_right_signed(sentence.field(0), sentence.field(1), angle_deg) ||
        !parse_knots(sentence.field(2), sentence.field(3), speed_kn)) {
        last_error_ = true_wind ? "bad VWT" : "bad VWR";
        return false;
    }

    if (true_wind) return set_wind(model.wind.truewind, angle_deg, speed_kn, now_us, source);
    return set_wind(model.wind.apparent, angle_deg, speed_kn, now_us, source);
}

template<typename Model>
bool apply_xdr(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    bool any = false;
    float value = 0.0f;
    for (uint8_t index = 0; index + 3 < sentence.field_count; index += 4) {
        if (sentence.field(index)[0] == 'A' &&
            parse_real(sentence.field(index + 1), value) &&
            sentence.field(index + 2)[0] == 'D') {
            if (nmea_span_equals(sentence.field(index + 3), "PTCH")) {
                model.imu.pitch_deg.set(static_cast<Real>(value), now_us);
                any = true;
            } else if (nmea_span_equals(sentence.field(index + 3), "ROLL")) {
                model.imu.roll_deg.set(static_cast<Real>(value), now_us);
                any = true;
            }
        }
    }
    if (!any) last_error_ = "bad XDR";
    return any;
}

template<typename Model>
bool apply_xte(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float xte_nmi = 0.0f;
    if (sentence.field_count < 5 ||
        sentence.field(0)[0] != 'A' ||
        sentence.field(1)[0] != 'A' ||
        !parse_left_right_signed(sentence.field(2), sentence.field(3), xte_nmi)) {
        last_error_ = "bad XTE";
        return false;
    }

    model.navigation.apb.xte_nmi.set(static_cast<Real>(xte_nmi), now_us);
    set_source(model.navigation.apb.source, source);
    model.navigation.apb.last_update_us = now_us;
    return true;
}
