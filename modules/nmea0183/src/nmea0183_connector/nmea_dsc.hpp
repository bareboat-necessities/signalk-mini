#pragma once

// Included inside Nmea0183RxConnector.

static constexpr uint64_t DSC_DSE_TIMEOUT_US = 500000ULL;

static bool dsc_format_is_distress(int32_t code) {
    return code == 112;
}

static bool dsc_category_is_distress(int32_t code) {
    return code == 112;
}

static bool dsc_category_is_urgency(int32_t code) {
    return code == 110;
}

static bool dsc_category_is_safety(int32_t code) {
    return code == 108;
}

static bool dsc_end_signal_is_ack(char eos) {
    return eos == 'A' || eos == 'R';
}

bool dsc_expansion_expected() const {
    const char flag = dsc_state_.message.expansion_flag;
    return flag == 'E' || flag == 'e';
}

bool dse_matches_pending_dsc(ship_data_model::SensorSource source) const {
    if (!dsc_pending_commit_) return false;
    if (dsc_pending_source_ != ship_data_model::SensorSource::none &&
        source != ship_data_model::SensorSource::none &&
        source != dsc_pending_source_) {
        return false;
    }
    if (dsc_state_.message.sender_mmsi[0] != '\0' &&
        dsc_state_.expansion.sender_mmsi[0] != '\0' &&
        strcmp(dsc_state_.message.sender_mmsi, dsc_state_.expansion.sender_mmsi) != 0) {
        return false;
    }
    return true;
}

template<typename Model, typename Alert>
void commit_dsc_alert(Model& model,
                      Alert& alert,
                      const char* alert_text,
                      uint64_t now_us,
                      ship_data_model::SensorSource source) {
    const auto& dsc = dsc_state_.message;
    alert = Alert{};
    set_source(alert.source, source);
    nmea_copy_cstr(alert.sender_mmsi, sizeof(alert.sender_mmsi), dsc.sender_mmsi);
    nmea_copy_cstr(alert.address_or_distress_mmsi,
                   sizeof(alert.address_or_distress_mmsi),
                   dsc.address_or_distress_mmsi);
    if (dsc.category.last_update_us) alert.category.set(dsc.category.value, now_us);
    if (dsc.nature_or_first_telecommand.last_update_us) {
        alert.nature_or_first_telecommand.set(dsc.nature_or_first_telecommand.value, now_us);
    }
    if (dsc.has_position) {
        alert.latitude_deg.set(static_cast<Real>(dsc.latitude_deg), now_us);
        alert.longitude_deg.set(static_cast<Real>(dsc.longitude_deg), now_us);
    }
    if (dsc.has_utc_time) alert.utc_time_s.set(static_cast<Real>(dsc.utc_time_s), now_us);
    nmea_copy_cstr(alert.alert_text, sizeof(alert.alert_text), alert_text);
    alert.active = true;
    alert.last_update_us = now_us;
    (void)model;
}

template<typename Model>
void promote_dsc_notifications(Model& model,
                               uint64_t now_us,
                               ship_data_model::SensorSource source) {
    const auto& dsc = dsc_state_.message;
    const int32_t format = dsc.format_specifier.last_update_us ? dsc.format_specifier.value : 0;
    const int32_t category = dsc.category.last_update_us ? dsc.category.value : 0;

    if (dsc_format_is_distress(format) || dsc_category_is_distress(category)) {
        commit_dsc_alert(model, model.notifications.dsc.distress, "DSC distress", now_us, source);
        model.comm.dsc.distress_count.set(model.comm.dsc.distress_count.value + 1, now_us);
    } else if (dsc_category_is_urgency(category)) {
        commit_dsc_alert(model, model.notifications.dsc.urgency, "DSC urgency", now_us, source);
        model.comm.dsc.urgency_count.set(model.comm.dsc.urgency_count.value + 1, now_us);
    } else if (dsc_category_is_safety(category)) {
        commit_dsc_alert(model, model.notifications.dsc.safety, "DSC safety", now_us, source);
        model.comm.dsc.safety_count.set(model.comm.dsc.safety_count.value + 1, now_us);
    }
}

template<typename Model>
bool commit_dsc_message_to_model(Model& model,
                                 uint64_t now_us,
                                 ship_data_model::SensorSource source,
                                 bool expansion_received,
                                 bool expansion_timeout) {
    const auto& dsc = dsc_state_.message;
    const auto& dse = dsc_state_.expansion;

    auto& comm = model.comm.dsc;
    auto& dst = comm.latest_call;
    dst = ship_data_model::DscCallData<Real>{};

    set_source(dst.source, source);
    if (dsc.format_specifier.last_update_us) dst.format_specifier.set(dsc.format_specifier.value, now_us);
    nmea_copy_cstr(dst.sender_mmsi, sizeof(dst.sender_mmsi), dsc.sender_mmsi);
    if (dsc.category.last_update_us) dst.category.set(dsc.category.value, now_us);
    if (dsc.nature_or_first_telecommand.last_update_us) {
        dst.nature_or_first_telecommand.set(dsc.nature_or_first_telecommand.value, now_us);
    }
    if (dsc.communication_or_second_telecommand.last_update_us) {
        dst.communication_or_second_telecommand.set(dsc.communication_or_second_telecommand.value, now_us);
    }

    nmea_copy_cstr(dst.position_code, sizeof(dst.position_code), dsc.position_code);
    if (dsc.has_position) {
        dst.latitude_deg.set(static_cast<Real>(dsc.latitude_deg), now_us);
        dst.longitude_deg.set(static_cast<Real>(dsc.longitude_deg), now_us);
    }
    if (dsc.has_utc_time) dst.utc_time_s.set(static_cast<Real>(dsc.utc_time_s), now_us);

    nmea_copy_cstr(dst.address_or_distress_mmsi,
                   sizeof(dst.address_or_distress_mmsi),
                   dsc.address_or_distress_mmsi);
    nmea_copy_cstr(dst.field10, sizeof(dst.field10), dsc.field10);
    dst.end_of_sequence = dsc.end_of_sequence;
    dst.expansion_expected = dsc_expansion_expected();
    dst.expansion_received = expansion_received;
    dst.expansion_timeout = expansion_timeout;

    if (expansion_received) {
        if (dse.total_messages.last_update_us) dst.dse_total_messages.set(dse.total_messages.value, now_us);
        if (dse.message_number.last_update_us) dst.dse_message_number.set(dse.message_number.value, now_us);
        dst.dse_query_flag = dse.query_flag;
        if (dse.expansion_specifier.last_update_us) {
            dst.dse_expansion_specifier.set(dse.expansion_specifier.value, now_us);
        }
        nmea_copy_cstr(dst.dse_payload, sizeof(dst.dse_payload), dse.payload);
    }

    dst.field_count.set(dst.expansion_expected ? 11 : 10, now_us);
    dst.first_seen_us = dsc.last_update_us != 0 ? dsc.last_update_us : now_us;
    dst.last_update_us = now_us;

    comm.call_count.set(comm.call_count.value + 1, now_us);
    if (expansion_timeout) {
        comm.expansion_timeout_count.set(comm.expansion_timeout_count.value + 1, now_us);
    }

    promote_dsc_notifications(model, now_us, source);
    return true;
}

template<typename Model>
void expire_pending_dsc_if_needed(Model& model, uint64_t now_us) {
    if (!dsc_pending_commit_) return;
    if (dsc_pending_started_us_ == 0 || now_us < dsc_pending_started_us_) return;
    if (now_us - dsc_pending_started_us_ <= DSC_DSE_TIMEOUT_US) return;

    const auto source = dsc_pending_source_;
    dsc_pending_commit_ = false;
    dsc_pending_started_us_ = 0;
    dsc_pending_source_ = ship_data_model::SensorSource::none;
    commit_dsc_message_to_model(model, now_us, source, false, true);
}

template<typename Model>
bool apply_dsc(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 10) {
        last_error_ = "short DSC";
        return false;
    }

    auto& dsc = dsc_state_.message;
    dsc = NmeaDscMessageRecord{};
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

    if (dsc_expansion_expected()) {
        dsc_pending_commit_ = true;
        dsc_pending_started_us_ = now_us;
        dsc_pending_source_ = source;
        return true;
    }

    dsc_pending_commit_ = false;
    dsc_pending_started_us_ = 0;
    dsc_pending_source_ = ship_data_model::SensorSource::none;
    return commit_dsc_message_to_model(model, now_us, source, false, false);
}

template<typename Model>
bool apply_dse(const NmeaSentence& sentence,
               Model& model,
               uint64_t now_us,
               ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short DSE";
        return false;
    }

    auto& dse = dsc_state_.expansion;
    dse = NmeaDscExpansionRecord{};
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

    if (dse_matches_pending_dsc(source)) {
        if (sentence.fragment.is_fragmented && !dsc_state_.multipart.complete) return true;
        if (sentence.fragment.is_fragmented && dsc_state_.multipart.complete) {
            nmea_copy_cstr(dse.payload, sizeof(dse.payload), dsc_state_.multipart.text);
        }
        dsc_pending_commit_ = false;
        dsc_pending_started_us_ = 0;
        dsc_pending_source_ = ship_data_model::SensorSource::none;
        return commit_dsc_message_to_model(model, now_us, source, true, false);
    }

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
