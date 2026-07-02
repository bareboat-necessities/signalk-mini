#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_hdt(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float heading_deg = 0.0f;
    if (!parse_real(sentence.field(0), heading_deg)) {
        last_error_ = "bad HDT";
        return false;
    }

    const Real value = static_cast<Real>(wrap_360_deg(heading_deg));
    model.imu.heading_deg.set(value, now_us);
    model.imu.heading_true_deg.set(value, now_us);
    return true;
}

template<typename Model>
bool apply_hdm(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float heading_deg = 0.0f;
    if (!parse_real(sentence.field(0), heading_deg)) {
        last_error_ = "bad HDM";
        return false;
    }

    const Real value = static_cast<Real>(wrap_360_deg(heading_deg));
    model.imu.heading_deg.set(value, now_us);
    model.imu.heading_magnetic_deg.set(value, now_us);
    return true;
}

template<typename Model>
bool apply_hdg(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    if (!apply_hdm(sentence, model, now_us)) {
        last_error_ = "bad HDG";
        return false;
    }

    float value = 0.0f;
    if (sentence.field_count >= 3 && parse_east_west_signed(sentence.field(1), sentence.field(2), value)) {
        model.imu.magnetic_deviation_deg.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count >= 5 && parse_east_west_signed(sentence.field(3), sentence.field(4), value)) {
        model.imu.magnetic_variation_deg.set(static_cast<Real>(value), now_us);
        model.gnss.fix.declination_deg.set(static_cast<Real>(value), now_us);
    }
    return true;
}

template<typename Model>
bool apply_hfb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short HFB";
        return false;
    }

    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) {
        model.sea.trawl_headrope_to_footrope_m.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(2), value)) {
        model.sea.trawl_headrope_to_bottom_m.set(static_cast<Real>(value), now_us);
    }
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_hsc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short HSC";
        return false;
    }

    float heading_deg = 0.0f;
    if (sentence.field(1)[0] == 'T' && parse_real(sentence.field(0), heading_deg)) {
        model.route.heading_steering_command.heading_true_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
        model.route.apb.heading_to_steer_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), heading_deg)) {
        model.route.heading_steering_command.heading_magnetic_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
    }
    set_source(model.route.heading_steering_command.source, source);
    model.route.heading_steering_command.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_its(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) {
        last_error_ = "short ITS";
        return false;
    }

    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.nav.legacy_timing.gri_us_div_10.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.nav.legacy_timing.master_relative_snr_db.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.nav.legacy_timing.master_relative_ecd.set(static_cast<Real>(value), now_us);
    for (uint8_t index = 0; index < 5; ++index) {
        const uint8_t base = static_cast<uint8_t>(3 + index * 2);
        if (parse_real(sentence.field(base), value)) {
            model.nav.legacy_timing.delta_us[index].set(static_cast<Real>(value), now_us);
        }
        model.nav.legacy_timing.delta_status[index] = sentence.field(base + 1)[0];
    }
    set_source(model.nav.legacy_timing.source, source);
    model.nav.legacy_timing.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_lwy(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float leeway_deg = 0.0f;
    if (sentence.field_count < 2 || sentence.field(0)[0] != 'A' || !parse_real(sentence.field(1), leeway_deg)) {
        last_error_ = "bad LWY";
        return false;
    }

    model.sea.leeway_deg.set(static_cast<Real>(leeway_deg), now_us);
    set_source(model.sea.leeway_source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 20) {
        last_error_ = "short MDA";
        return false;
    }

    float value = 0.0f;
    if (sentence.field(1)[0] == 'I' && parse_real(sentence.field(0), value)) model.env.barometric_pressure_inhg.set(static_cast<Real>(value), now_us);
    if (sentence.field(3)[0] == 'B' && parse_real(sentence.field(2), value)) model.env.barometric_pressure_bar.set(static_cast<Real>(value), now_us);
    if (sentence.field(5)[0] == 'C' && parse_real(sentence.field(4), value)) model.env.air_temperature_c.set(static_cast<Real>(value), now_us);
    if (sentence.field(7)[0] == 'C' && parse_real(sentence.field(6), value)) model.sea.temperature_c.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.env.relative_humidity_percent.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.env.absolute_humidity_percent.set(static_cast<Real>(value), now_us);
    if (sentence.field(11)[0] == 'C' && parse_real(sentence.field(10), value)) model.env.dew_point_c.set(static_cast<Real>(value), now_us);
    if (sentence.field(13)[0] == 'T' && parse_real(sentence.field(12), value)) {
        const Real direction = static_cast<Real>(wrap_360_deg(value));
        model.wind.surface.direction_deg.set(direction, now_us);
        model.wind.truewind.direction_deg.set(direction, now_us);
    }
    if (sentence.field(15)[0] == 'M' && parse_real(sentence.field(14), value)) model.wind.surface.direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_knots(sentence.field(16), sentence.field(17), value)) {
        model.wind.surface.speed_kn.set(static_cast<Real>(value), now_us);
        model.wind.truewind.speed_kn.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field(19)[0] == 'M' && parse_real(sentence.field(18), value)) {
        model.wind.surface.speed_m_s.set(static_cast<Real>(value), now_us);
        model.wind.truewind.speed_m_s.set(static_cast<Real>(value), now_us);
    }
    set_source(model.sea.source, source);
    set_source(model.wind.surface.source, source);
    set_source(model.wind.truewind.source, source);
    model.sea.last_update_us = now_us;
    model.wind.surface.last_update_us = now_us;
    model.wind.truewind.last_update_us = now_us;
    return true;
}

template<typename Wind>
bool set_wind(Wind& wind, float angle_deg, float speed_kn, uint64_t now_us, ship_data_model::SensorSource source) {
    wind.direction_deg.set(static_cast<Real>(wrap_180_deg(angle_deg)), now_us);
    wind.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(wind.source, source);
    wind.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_msk(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) {
        last_error_ = "short MSK";
        return false;
    }

    float value = 0.0f;
    int32_t bit_rate = 0;
    if (parse_real(sentence.field(0), value)) model.comm.beacon_control.frequency_khz.set(static_cast<Real>(value), now_us);
    model.comm.beacon_control.frequency_mode = sentence.field(1)[0];
    if (parse_int32(sentence.field(2), bit_rate)) model.comm.beacon_control.bit_rate_bps.set(bit_rate, now_us);
    model.comm.beacon_control.bit_rate_mode = sentence.field(3)[0];
    if (parse_real(sentence.field(4), value)) model.comm.beacon_control.status_frequency_khz.set(static_cast<Real>(value), now_us);
    set_source(model.comm.beacon_control.source, source);
    model.comm.beacon_control.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mss(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) {
        last_error_ = "short MSS";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_real(sentence.field(0), value)) model.comm.beacon_status.signal_strength_db_uv.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.comm.beacon_status.signal_to_noise_ratio_db.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.comm.beacon_status.beacon_frequency_khz.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(3), integer_value)) model.comm.beacon_status.beacon_bit_rate_bps.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.comm.beacon_status.status.set(integer_value, now_us);
    set_source(model.comm.beacon_status.source, source);
    model.comm.beacon_status.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mtw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2 || sentence.field(1)[0] != 'C') {
        last_error_ = "bad MTW";
        return false;
    }

    float temperature_c = 0.0f;
    if (!parse_real(sentence.field(0), temperature_c)) {
        last_error_ = "bad MTW";
        return false;
    }

    model.sea.temperature_c.set(static_cast<Real>(temperature_c), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mwd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float direction_deg = 0.0f;
    float speed_kn = 0.0f;
    if (sentence.field_count < 7 ||
        !parse_real(sentence.field(0), direction_deg) ||
        !parse_knots(sentence.field(4), sentence.field(5), speed_kn)) {
        last_error_ = "bad MWD";
        return false;
    }

    model.wind.truewind.direction_deg.set(static_cast<Real>(wrap_180_deg(direction_deg)), now_us);
    model.wind.truewind.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(model.wind.truewind.source, source);
    model.wind.truewind.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mwv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5 || sentence.field(4)[0] != 'A') {
        last_error_ = "invalid MWV";
        return false;
    }

    float angle_deg = 0.0f;
    float speed_kn = 0.0f;
    if (!parse_real(sentence.field(0), angle_deg) || !parse_knots(sentence.field(2), sentence.field(3), speed_kn)) {
        last_error_ = "bad MWV";
        return false;
    }

    if (sentence.field(1)[0] == 'T') return set_wind(model.wind.truewind, angle_deg, speed_kn, now_us, source);
    return set_wind(model.wind.apparent, angle_deg, speed_kn, now_us, source);
}
