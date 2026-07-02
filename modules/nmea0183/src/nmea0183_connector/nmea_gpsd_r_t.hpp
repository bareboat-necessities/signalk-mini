#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_rlm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short RLM";
        return false;
    }

    float value = 0.0f;
    nmea_copy_span(model.navigation.return_link_message.beacon_id,
                   sizeof(model.navigation.return_link_message.beacon_id),
                   sentence.field(0));
    if (parse_utc_time_of_day_s(sentence.field(1), value)) model.navigation.return_link_message.reception_time_s.set(static_cast<Real>(value), now_us);
    model.navigation.return_link_message.message_code = sentence.field(2)[0];
    nmea_copy_span(model.navigation.return_link_message.message_body,
                   sizeof(model.navigation.return_link_message.message_body),
                   sentence.field(3));
    set_source(model.navigation.return_link_message.source, source);
    model.navigation.return_link_message.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short TFI";
        return false;
    }

    int32_t value = 0;
    for (uint8_t index = 0; index < 3; ++index) {
        if (parse_int32(sentence.field(index), value)) model.water.trawl_catch_sensor_status[index].set(value, now_us);
    }
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tlb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) {
        last_error_ = "short TLB";
        return false;
    }

    int32_t value = 0;
    uint8_t label_count = 0;
    for (uint8_t index = 0; index + 1 < sentence.field_count && label_count < 8; index = static_cast<uint8_t>(index + 2)) {
        if (parse_int32(sentence.field(index), value)) {
            model.navigation.tracked_target.label_target_number[label_count].set(value, now_us);
            nmea_copy_span(model.navigation.tracked_target.label[label_count],
                           sizeof(model.navigation.tracked_target.label[label_count]),
                           sentence.field(static_cast<uint8_t>(index + 1)));
            ++label_count;
        }
    }
    if (label_count == 0) {
        last_error_ = "bad TLB";
        return false;
    }
    model.navigation.tracked_target.label_count.set(static_cast<int32_t>(label_count), now_us);
    set_source(model.navigation.tracked_target.source, source);
    model.navigation.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tll(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) {
        last_error_ = "short TLL";
        return false;
    }

    int32_t integer_value = 0;
    float value = 0.0f;
    if (parse_int32(sentence.field(0), integer_value)) model.navigation.tracked_target.target_number.set(integer_value, now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.navigation.tracked_target.latitude_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.navigation.tracked_target.longitude_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.navigation.tracked_target.target_name, sizeof(model.navigation.tracked_target.target_name), sentence.field(5));
    if (parse_utc_time_of_day_s(sentence.field(6), value)) model.navigation.tracked_target.utc_time_s.set(static_cast<Real>(value), now_us);
    model.navigation.tracked_target.target_status = sentence.field(7)[0];
    model.navigation.tracked_target.reference_target = sentence.field(8)[0];
    set_source(model.navigation.tracked_target.source, source);
    model.navigation.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short TPC";
        return false;
    }

    float value = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), value)) model.water.trawl_cartesian_centerline_offset_m.set(static_cast<Real>(value), now_us);
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), value)) model.water.trawl_cartesian_along_centerline_m.set(static_cast<Real>(value), now_us);
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), value)) model.water.trawl_depth_below_surface_m.set(static_cast<Real>(value), now_us);
    set_source(model.water.source, source);
    model.water.last_update_us = now_us;
    return true;
}
