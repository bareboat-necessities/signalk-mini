#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_zda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short ZDA";
        return false;
    }

    float time_s = 0.0f;
    int32_t integer_value = 0;
    if (!parse_utc_time_of_day_s(sentence.field(0), time_s)) {
        last_error_ = "bad ZDA";
        return false;
    }

    model.navigation.gps.timestamp_s.set(static_cast<Real>(time_s), now_us);
    if (parse_int32(sentence.field(1), integer_value)) model.navigation.gps.date_day.set(integer_value, now_us);
    if (parse_int32(sentence.field(2), integer_value)) model.navigation.gps.date_month.set(integer_value, now_us);
    if (parse_int32(sentence.field(3), integer_value)) model.navigation.gps.date_year.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.navigation.gps.local_zone_hours.set(integer_value, now_us);
    if (parse_int32(sentence.field(5), integer_value)) model.navigation.gps.local_zone_minutes.set(integer_value, now_us);
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zfo(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short ZFO";
        return false;
    }

    float value = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), value)) {
        last_error_ = "bad ZFO";
        return false;
    }
    model.navigation.waypoint.origin_utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), value) || parse_real(sentence.field(1), value)) {
        model.navigation.waypoint.origin_elapsed_time_s.set(static_cast<Real>(value), now_us);
    }
    nmea_copy_span(model.navigation.waypoint.from_waypoint_id, sizeof(model.navigation.waypoint.from_waypoint_id), sentence.field(2));
    set_source(model.navigation.waypoint.source, source);
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ztg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short ZTG";
        return false;
    }

    float value = 0.0f;
    if (!parse_utc_time_of_day_s(sentence.field(0), value)) {
        last_error_ = "bad ZTG";
        return false;
    }
    model.navigation.waypoint.destination_utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_utc_time_of_day_s(sentence.field(1), value) || parse_real(sentence.field(1), value)) {
        model.navigation.waypoint.destination_time_remaining_s.set(static_cast<Real>(value), now_us);
    }
    nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), sentence.field(2));
    set_source(model.navigation.waypoint.source, source);
    model.navigation.waypoint.last_update_us = now_us;
    return true;
}
