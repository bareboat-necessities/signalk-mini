#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_alc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short ALC"; return false; }
    auto& rec = model.notifications.alerts.cyclic_list;
    int32_t parsed = 0;
    if (parse_int32(sentence.field(0), parsed)) rec.total_messages.set(parsed, now_us);
    if (parse_int32(sentence.field(1), parsed)) rec.message_number.set(parsed, now_us);
    nmea_copy_span(rec.sequential_message_id, sizeof(rec.sequential_message_id), sentence.field(2));
    uint8_t count = 0;
    for (uint8_t i = 3; i < sentence.field_count && count < ship_data_model::ALERT_LIST_ENTRY_CAPACITY; ++i) {
        if (sentence.field(i).length == 0) continue;
        nmea_copy_span(rec.alert_identifier[count], sizeof(rec.alert_identifier[count]), sentence.field(i));
        ++count;
    }
    rec.alert_count.set(static_cast<int32_t>(count), now_us);
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_alf(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short ALF"; return false; }
    auto& rec = model.notifications.alerts.alert_report;
    int32_t parsed = 0;
    if (parse_int32(sentence.field(0), parsed)) rec.total_fragments.set(parsed, now_us);
    if (sentence.field_count > 1 && parse_int32(sentence.field(1), parsed)) rec.fragment_number.set(parsed, now_us);
    if (sentence.field_count > 2) nmea_copy_span(rec.sequential_message_id, sizeof(rec.sequential_message_id), sentence.field(2));
    if (sentence.field_count > 3) nmea_copy_span(rec.alert_identifier, sizeof(rec.alert_identifier), sentence.field(3));
    if (sentence.field_count > 4 && parse_int32(sentence.field(4), parsed)) rec.alert_instance.set(parsed, now_us);
    if (sentence.field_count > 5 && parse_int32(sentence.field(5), parsed)) rec.revision_counter.set(parsed, now_us);
    if (sentence.field_count > 6 && parse_int32(sentence.field(6), parsed)) rec.escalation_counter.set(parsed, now_us);
    if (sentence.field_count > 7) rec.category = sentence.field(7)[0];
    if (sentence.field_count > 8) rec.priority = sentence.field(8)[0];
    if (sentence.field_count > 9) rec.alert_state = sentence.field(9)[0];
    if (sentence.field_count > 10) nmea_copy_span(rec.alert_text, sizeof(rec.alert_text), sentence.field(10));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_alr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short ALR"; return false; }
    auto& rec = model.notifications.alerts.alarm_state;
    float seconds = 0.0f;
    int32_t parsed = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), seconds)) rec.utc_time_s.set(static_cast<Real>(seconds), now_us);
    nmea_copy_span(rec.alarm_identifier, sizeof(rec.alarm_identifier), sentence.field(1));
    if (parse_int32(sentence.field(1), parsed)) rec.local_alarm_number.set(parsed, now_us);
    rec.condition_state = sentence.field(2)[0];
    rec.acknowledgement_state = sentence.field(3)[0];
    if (sentence.field_count > 4) nmea_copy_span(rec.description, sizeof(rec.description), sentence.field(4));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_arc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short ARC"; return false; }
    auto& rec = model.notifications.alerts.command_refused;
    int32_t parsed = 0;
    nmea_copy_span(rec.alert_identifier, sizeof(rec.alert_identifier), sentence.field(0));
    if (sentence.field_count > 1 && parse_int32(sentence.field(1), parsed)) rec.alert_instance.set(parsed, now_us);
    if (sentence.field_count > 2) nmea_copy_span(rec.refused_command, sizeof(rec.refused_command), sentence.field(2));
    if (sentence.field_count > 3) nmea_copy_span(rec.reason_code, sizeof(rec.reason_code), sentence.field(3));
    if (sentence.field_count > 4) nmea_copy_span(rec.reason_text, sizeof(rec.reason_text), sentence.field(4));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_hbt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) { last_error_ = "short HBT"; return false; }
    auto& rec = model.notifications.alerts.heartbeat;
    float interval = 0.0f;
    rec.status = sentence.field(0)[0];
    if (parse_real(sentence.field(1), interval)) rec.interval_s.set(static_cast<Real>(interval), now_us);
    if (sentence.field_count > 2) nmea_copy_span(rec.sequential_message_id, sizeof(rec.sequential_message_id), sentence.field(2));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_smv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) { last_error_ = "short SMV"; return false; }
    auto& rec = model.notifications.special.smv;
    float value = 0.0f;
    int32_t parsed = 0;
    nmea_copy_span(rec.message_id, sizeof(rec.message_id), sentence.field(0));
    if (parse_int32(sentence.field(1), parsed)) rec.mmsi.set(parsed, now_us);
    else nmea_copy_span(rec.mmsi_text, sizeof(rec.mmsi_text), sentence.field(1));
    if (parse_utc_time_of_day_s(sentence.field(2), value)) rec.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) rec.latitude_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(5), sentence.field(6), value)) rec.longitude_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(rec.event_type, sizeof(rec.event_type), sentence.field(7));
    if (sentence.field_count > 8) nmea_copy_span(rec.sar_capability, sizeof(rec.sar_capability), sentence.field(8));
    if (sentence.field_count > 9) nmea_copy_span(rec.route_id, sizeof(rec.route_id), sentence.field(9));
    if (sentence.field_count > 10) rec.status = sentence.field(10)[0];
    if (sentence.field_count > 11) nmea_copy_span(rec.description, sizeof(rec.description), sentence.field(11));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}
