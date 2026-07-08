#pragma once

// Included inside Nmea0183RxConnector.
// Balanced NMEA sentence group: V-Z.

template<typename Model>
bool apply_vbw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short VBW";
        return false;
    }

    float v = 0.0f;
    model.sea.water_speed_status = sentence.field(2)[0];
    model.sea.ground_speed_status = sentence.field(5)[0];

    if (sentence.field(2)[0] == 'A') {
        if (parse_real(sentence.field(0), v)) {
            model.sea.longitudinal_water_speed_kn.set(static_cast<Real>(v), now_us);
            model.sea.speed_kn.set(static_cast<Real>(v), now_us);
        }
        if (parse_real(sentence.field(1), v)) {
            model.sea.transverse_water_speed_kn.set(static_cast<Real>(v), now_us);
        }
    }

    if (sentence.field(5)[0] == 'A') {
        if (parse_real(sentence.field(3), v)) {
            model.sea.longitudinal_ground_speed_kn.set(static_cast<Real>(v), now_us);
        }
        if (parse_real(sentence.field(4), v)) {
            model.sea.transverse_ground_speed_kn.set(static_cast<Real>(v), now_us);
        }
    }

    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short VDR";
        return false;
    }

    float v = 0.0f;
    if (sentence.field(1)[0] == 'T' && parse_real(sentence.field(0), v)) {
        model.sea.current_direction_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) {
        model.sea.current_direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_knots(sentence.field(4), sentence.field(5), v)) {
        model.sea.current_speed_kn.set(static_cast<Real>(v), now_us);
    }
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vhw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float v = 0.0f;
    if (!parse_knots(sentence.field(4), sentence.field(5), v)) {
        last_error_ = "bad VHW";
        return false;
    }
    model.sea.speed_kn.set(static_cast<Real>(v), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vlw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short VLW";
        return false;
    }

    float v = 0.0f;
    if (parse_distance_nmi(sentence.field(0), sentence.field(1), v)) {
        model.route.log.total_distance_nmi.set(static_cast<Real>(v), now_us);
    }
    if (parse_distance_nmi(sentence.field(2), sentence.field(3), v)) {
        model.route.log.trip_distance_nmi.set(static_cast<Real>(v), now_us);
    }
    set_source(model.route.log.source, source);
    model.route.log.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vpw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short VPW";
        return false;
    }

    float v = 0.0f;
    if (parse_knots(sentence.field(0), sentence.field(1), v)) {
        model.sea.speed_parallel_to_wind_kn.set(static_cast<Real>(v), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) {
        model.sea.speed_parallel_to_wind_m_s.set(static_cast<Real>(v), now_us);
    }
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vwr(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source,
               bool true_wind) {
    float a = 0.0f;
    float s = 0.0f;
    if (!parse_left_right_signed(sentence.field(0), sentence.field(1), a) ||
        !parse_knots(sentence.field(2), sentence.field(3), s)) {
        last_error_ = true_wind ? "bad VWT" : "bad VWR";
        return false;
    }
    return true_wind ? set_wind(model.wind.truewind, a, s, now_us, source)
                     : set_wind(model.wind.apparent, a, s, now_us, source);
}

template<typename Model>
bool apply_wcv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short WCV";
        return false;
    }

    float v = 0.0f;
    if (!parse_knots(sentence.field(0), sentence.field(1), v)) {
        last_error_ = "bad WCV";
        return false;
    }
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
    if (sentence.field_count < 6) {
        last_error_ = "short WDC";
        return false;
    }

    float v = 0.0f;
    if (parse_distance_nmi(sentence.field(0), sentence.field(1), v) ||
        parse_distance_nmi(sentence.field(2), sentence.field(3), v)) {
        model.route.waypoint.distance_nmi.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.route.waypoint.to_waypoint_id,
                   sizeof(model.route.waypoint.to_waypoint_id),
                   sentence.field(4));
    nmea_copy_span(model.route.waypoint.from_waypoint_id,
                   sizeof(model.route.waypoint.from_waypoint_id),
                   sentence.field(5));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_wdr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    return apply_wdc(sentence, model, now_us, source);
}

template<typename Model>
bool apply_wnc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    return apply_wdc(sentence, model, now_us, source);
}

template<typename Model>
bool apply_wpl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) {
        last_error_ = "short WPL";
        return false;
    }

    float v = 0.0f;
    if (parse_lat_lon(sentence.field(0), sentence.field(1), v)) {
        model.route.waypoint.latitude_deg.set(static_cast<Real>(v), now_us);
    }
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) {
        model.route.waypoint.longitude_deg.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(4));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_xte(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float v = 0.0f;
    if (sentence.field_count < 5 ||
        sentence.field(0)[0] != 'A' ||
        sentence.field(1)[0] != 'A' ||
        !parse_left_right_signed(sentence.field(2), sentence.field(3), v)) {
        last_error_ = "bad XTE";
        return false;
    }
    model.route.apb.xte_nmi.set(static_cast<Real>(v), now_us);
    set_source(model.route.apb.source, source);
    model.route.apb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_xtr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float v = 0.0f;
    if (sentence.field_count < 3 || sentence.field(2)[0] != 'N' || !parse_left_right_signed(sentence.field(0), sentence.field(1), v)) {
        last_error_ = "bad XTR";
        return false;
    }
    model.route.apb.xte_nmi.set(static_cast<Real>(v), now_us);
    model.route.rmb.xte_nmi.set(static_cast<Real>(v), now_us);
    set_source(model.route.apb.source, source);
    set_source(model.route.rmb.source, source);
    model.route.apb.last_update_us = now_us;
    model.route.rmb.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zdl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short ZDL";
        return false;
    }

    float v = 0.0f;
    if (parse_utc_time_of_day_s(sentence.field(0), v) || parse_real(sentence.field(0), v)) {
        model.route.waypoint.destination_time_remaining_s.set(static_cast<Real>(v), now_us);
    }
    if (parse_distance_nmi(sentence.field(1), sentence.field(2), v)) {
        model.route.waypoint.distance_nmi.set(static_cast<Real>(v), now_us);
    }
    if (sentence.field_count > 3) {
        nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(3));
    }
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zfo(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short ZFO";
        return false;
    }

    float v = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) {
        last_error_ = "bad ZFO";
        return false;
    }
    model.route.waypoint.origin_utc_time_s.set(static_cast<Real>(v), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), v) || parse_real(sentence.field(1), v)) {
        model.route.waypoint.origin_elapsed_time_s.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.route.waypoint.from_waypoint_id, sizeof(model.route.waypoint.from_waypoint_id), sentence.field(2));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ztg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short ZTG";
        return false;
    }

    float v = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) {
        last_error_ = "bad ZTG";
        return false;
    }
    model.route.waypoint.destination_utc_time_s.set(static_cast<Real>(v), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), v) || parse_real(sentence.field(1), v)) {
        model.route.waypoint.destination_time_remaining_s.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.route.waypoint.to_waypoint_id, sizeof(model.route.waypoint.to_waypoint_id), sentence.field(2));
    set_source(model.route.waypoint.source, source);
    model.route.waypoint.last_update_us = now_us;
    return true;
}
