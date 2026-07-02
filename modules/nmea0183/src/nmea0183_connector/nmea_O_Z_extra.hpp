#pragma once

// Included inside Nmea0183RxConnector after nmea_O_Z.hpp.

template<typename Model>
bool apply_txt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    return apply_raw_sentence_record(sentence, state_.text_sentence, "TXT", now_us, source);
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
bool apply_zdl(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    if (sentence.field_count < 3) {
        last_error_ = "short ZDL";
        return false;
    }
    float value = 0.0f;
    if (parse_utc_time_of_day_s(sentence.field(0), value) || parse_real(sentence.field(0), value)) {
        state_.variable_point.time_to_point_s = value;
        state_.variable_point.has_time = true;
    }
    if (parse_distance_nmi(sentence.field(1), sentence.field_count > 2 ? sentence.field(2) : NmeaSpan(), value)) {
        state_.variable_point.distance_to_point_nmi = value;
        state_.variable_point.has_distance = true;
    }
    if (sentence.field_count > 3) nmea_copy_span(state_.variable_point.point_id,
                                                 sizeof(state_.variable_point.point_id),
                                                 sentence.field(3));
    set_source(state_.variable_point.source, source);
    state_.variable_point.last_update_us = now_us;
    return true;
}
