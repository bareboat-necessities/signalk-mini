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
