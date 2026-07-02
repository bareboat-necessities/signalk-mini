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
        model.navigation.gps.declination_deg.set(static_cast<Real>(value), now_us);
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
        model.water.trawl_headrope_to_footrope_m.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(2), value)) {
        model.water.trawl_headrope_to_bottom_m.set(static_cast<Real>(value), now_us);
    }
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
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
        model.navigation.heading_steering_command.heading_true_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
        model.navigation.apb.heading_to_steer_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), heading_deg)) {
        model.navigation.heading_steering_command.heading_magnetic_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
    }
    set_source(model.navigation.heading_steering_command.source, source);
    model.navigation.heading_steering_command.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_its(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) {
        last_error_ = "short ITS";
        return false;
    }

    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.navigation.legacy_timing.gri_us_div_10.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.navigation.legacy_timing.master_relative_snr_db.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.navigation.legacy_timing.master_relative_ecd.set(static_cast<Real>(value), now_us);
    for (uint8_t index = 0; index < 5; ++index) {
        const uint8_t base = static_cast<uint8_t>(3 + index * 2);
        if (parse_real(sentence.field(base), value)) {
            model.navigation.legacy_timing.delta_us[index].set(static_cast<Real>(value), now_us);
        }
        model.navigation.legacy_timing.delta_status[index] = sentence.field(base + 1)[0];
    }
    set_source(model.navigation.legacy_timing.source, source);
    model.navigation.legacy_timing.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_lwy(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float leeway_deg = 0.0f;
    if (sentence.field_count < 2 || sentence.field(0)[0] != 'A' || !parse_real(sentence.field(1), leeway_deg)) {
        last_error_ = "bad LWY";
        return false;
    }

    model.water.leeway_deg.set(static_cast<Real>(leeway_deg), now_us);
    set_source(model.water.leeway_source, source);
    model.water.last_update_us = now_us;
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
    if (parse_real(sentence.field(0), value)) model.navigation.beacon_control.frequency_khz.set(static_cast<Real>(value), now_us);
    model.navigation.beacon_control.frequency_mode = sentence.field(1)[0];
    if (parse_int32(sentence.field(2), bit_rate)) model.navigation.beacon_control.bit_rate_bps.set(bit_rate, now_us);
    model.navigation.beacon_control.bit_rate_mode = sentence.field(3)[0];
    if (parse_real(sentence.field(4), value)) model.navigation.beacon_control.status_frequency_khz.set(static_cast<Real>(value), now_us);
    set_source(model.navigation.beacon_control.source, source);
    model.navigation.beacon_control.last_update_us = now_us;
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
    if (parse_real(sentence.field(0), value)) model.navigation.beacon_status.signal_strength_db_uv.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.navigation.beacon_status.signal_to_noise_ratio_db.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.navigation.beacon_status.beacon_frequency_khz.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(3), integer_value)) model.navigation.beacon_status.beacon_bit_rate_bps.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.navigation.beacon_status.status.set(integer_value, now_us);
    set_source(model.navigation.beacon_status.source, source);
    model.navigation.beacon_status.last_update_us = now_us;
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

    model.water.temperature_c.set(static_cast<Real>(temperature_c), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
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
