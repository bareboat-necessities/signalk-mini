#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_dsc(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    (void)model;
    if (sentence.field_count < 10) {
        last_error_ = "short DSC";
        return false;
    }

    auto& dsc = dsc_state_.message;
    int32_t integer_value = 0;
    float value = 0.0f;
    if (parse_int32(sentence.field(0), integer_value)) {
        dsc.format_specifier.set(integer_value, now_us);
    }
    nmea_copy_span(dsc.sender_mmsi, sizeof(dsc.sender_mmsi), sentence.field(1));
    if (parse_int32(sentence.field(2), integer_value)) {
        dsc.category.set(integer_value, now_us);
    }
    if (parse_int32(sentence.field(3), integer_value)) {
        dsc.nature_or_first_telecommand.set(integer_value, now_us);
    }
    if (parse_int32(sentence.field(4), integer_value)) {
        dsc.communication_or_second_telecommand.set(integer_value, now_us);
    }
    nmea_copy_span(dsc.position_code, sizeof(dsc.position_code), sentence.field(5));
    float lat = 0.0f;
    float lon = 0.0f;
    if (parse_dsc_compact_position(sentence.field(5), lat, lon)) {
        dsc.latitude_deg = lat;
        dsc.longitude_deg = lon;
        dsc.has_position = true;
    }
    if (parse_nmea_hhmm_time_s(sentence.field(6), value)) {
        dsc.utc_time_s = value;
        dsc.has_utc_time = true;
    }
    nmea_copy_span(dsc.address_or_distress_mmsi,
                   sizeof(dsc.address_or_distress_mmsi),
                   sentence.field(7));
    nmea_copy_span(dsc.field10, sizeof(dsc.field10), sentence.field(8));
    dsc.end_of_sequence = sentence.field(9)[0];
    if (sentence.field_count > 10) {
        dsc.expansion_flag = sentence.field(10)[0];
    }
    set_source(dsc.source, source);
    dsc.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dse(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    (void)model;
    if (sentence.field_count < 6) {
        last_error_ = "short DSE";
        return false;
    }

    auto& dse = dsc_state_.expansion;
    int32_t integer_value = 0;
    if (parse_int32(sentence.field(0), integer_value)) {
        dse.total_messages.set(integer_value, now_us);
    }
    if (parse_int32(sentence.field(1), integer_value)) {
        dse.message_number.set(integer_value, now_us);
    }
    dse.query_flag = sentence.field(2)[0];
    nmea_copy_span(dse.sender_mmsi, sizeof(dse.sender_mmsi), sentence.field(3));
    if (parse_int32(sentence.field(4), integer_value)) {
        dse.expansion_specifier.set(integer_value, now_us);
    }
    nmea_copy_span(dse.payload, sizeof(dse.payload), sentence.field(5));
    set_source(dse.source, source);
    dse.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dsi(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) {
        last_error_ = "short DSI";
        return false;
    }

    auto& interrogation = model.notifications.dsc.interrogation;
    nmea_copy_span(interrogation.request_id,
                   sizeof(interrogation.request_id),
                   sentence.field(0));
    if (sentence.field_count > 1) {
        nmea_copy_span(interrogation.remote_mmsi,
                       sizeof(interrogation.remote_mmsi),
                       sentence.field(1));
    }
    interrogation.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(interrogation.source, source);
    interrogation.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dsr(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) {
        last_error_ = "short DSR";
        return false;
    }

    auto& response = model.notifications.dsc.response;
    nmea_copy_span(response.response_id,
                   sizeof(response.response_id),
                   sentence.field(0));
    if (sentence.field_count > 1) {
        nmea_copy_span(response.remote_mmsi,
                       sizeof(response.remote_mmsi),
                       sentence.field(1));
    }
    response.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(response.source, source);
    response.last_update_us = now_us;
    return true;
}
