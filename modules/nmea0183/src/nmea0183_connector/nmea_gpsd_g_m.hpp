#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_gns(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) {
        last_error_ = "short GNS";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.gps.timestamp_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.navigation.gps.fix_mode_indicator, sizeof(model.navigation.gps.fix_mode_indicator), sentence.field(5));
    if (sentence.field(5).length > 0) model.navigation.gps.fix_quality.set(sentence.field(5)[0] == 'N' ? 0 : 1, now_us);
    if (parse_int32(sentence.field(6), integer_value)) model.navigation.gps.satellites_used.set(integer_value, now_us);
    if (parse_real(sentence.field(7), value)) model.navigation.gps.hdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.navigation.gps.fix_alt_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.navigation.gps.geoidal_separation_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(10), value)) model.navigation.gps.dgps_age_s.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(11), integer_value)) model.navigation.gps.dgps_station_id.set(integer_value, now_us);
    if (sentence.field_count >= 13) model.navigation.gps.navigational_status = sentence.field(12)[0];
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 20) {
        last_error_ = "short MDA";
        return false;
    }

    float value = 0.0f;
    if (sentence.field(1)[0] == 'I' && parse_real(sentence.field(0), value)) model.water.barometric_pressure_inhg.set(static_cast<Real>(value), now_us);
    if (sentence.field(3)[0] == 'B' && parse_real(sentence.field(2), value)) model.water.barometric_pressure_bar.set(static_cast<Real>(value), now_us);
    if (sentence.field(5)[0] == 'C' && parse_real(sentence.field(4), value)) model.water.air_temperature_c.set(static_cast<Real>(value), now_us);
    if (sentence.field(7)[0] == 'C' && parse_real(sentence.field(6), value)) model.water.temperature_c.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.water.relative_humidity_percent.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.water.absolute_humidity_percent.set(static_cast<Real>(value), now_us);
    if (sentence.field(11)[0] == 'C' && parse_real(sentence.field(10), value)) model.water.dew_point_c.set(static_cast<Real>(value), now_us);
    if (sentence.field(13)[0] == 'T' && parse_real(sentence.field(12), value)) {
        const Real direction = static_cast<Real>(wrap_360_deg(value));
        model.water.wind_direction_deg.set(direction, now_us);
        model.wind.truewind.direction_deg.set(direction, now_us);
    }
    if (sentence.field(15)[0] == 'M' && parse_real(sentence.field(14), value)) model.water.wind_direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_knots(sentence.field(16), sentence.field(17), value)) {
        model.water.wind_speed_kn.set(static_cast<Real>(value), now_us);
        model.wind.truewind.speed_kn.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field(19)[0] == 'M' && parse_real(sentence.field(18), value)) {
        model.water.wind_speed_m_s.set(static_cast<Real>(value), now_us);
        model.wind.truewind.speed_m_s.set(static_cast<Real>(value), now_us);
    }
    set_source(model.water.source, source);
    set_source(model.wind.truewind.source, source);
    model.water.last_update_us = now_us;
    model.wind.truewind.last_update_us = now_us;
    return true;
}
