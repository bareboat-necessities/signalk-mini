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
