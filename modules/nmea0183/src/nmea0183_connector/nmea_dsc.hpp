#pragma once

// Included inside Nmea0183RxConnector.

static constexpr uint64_t DSC_DSE_TIMEOUT_US = 500000ULL;
static constexpr uint64_t DSC_DUPLICATE_WINDOW_US = 10000000ULL;

enum DscFormatSpecifier : int32_t {
    DSC_FORMAT_GEOGRAPHIC_AREA = 102,
    DSC_FORMAT_DISTRESS = 112,
    DSC_FORMAT_GROUP = 114,
    DSC_FORMAT_ALL_SHIPS = 116,
    DSC_FORMAT_INDIVIDUAL = 120
};

enum DscCategory : int32_t {
    DSC_CATEGORY_ROUTINE = 100,
    DSC_CATEGORY_SAFETY = 108,
    DSC_CATEGORY_URGENCY = 110,
    DSC_CATEGORY_DISTRESS = 112
};

enum DscEndSignal : char {
    DSC_END_SIGNAL_ACKNOWLEDGE = 'A',
    DSC_END_SIGNAL_RETRANSMIT_ACK = 'R',
    DSC_END_SIGNAL_END_OF_SEQUENCE = 'E'
};

static bool dsc_format_is_distress(int32_t code) {
    return code == DSC_FORMAT_DISTRESS;
}

static bool dsc_category_is_distress(int32_t code) {
    return code == DSC_CATEGORY_DISTRESS;
}

static bool dsc_category_is_urgency(int32_t code) {
    return code == DSC_CATEGORY_URGENCY;
}

static bool dsc_category_is_safety(int32_t code) {
    return code == DSC_CATEGORY_SAFETY;
}

static bool dsc_category_is_routine(int32_t code) {
    return code == DSC_CATEGORY_ROUTINE;
}

static bool dsc_end_signal_is_ack(char eos) {
    const auto signal = static_cast<DscEndSignal>(eos);
    return signal == DSC_END_SIGNAL_ACKNOWLEDGE || signal == DSC_END_SIGNAL_RETRANSMIT_ACK;
}

static bool dsc_priority_is_alert(ship_data_model::DscPriority priority) {
    return priority == ship_data_model::DscPriority::distress ||
           priority == ship_data_model::DscPriority::urgency ||
           priority == ship_data_model::DscPriority::safety;
}

static bool dsc_char_is_digit(char c) {
    return c >= '0' && c <= '9';
}

static bool dsc_char_is_alpha(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

static ship_data_model::DscPriority dsc_priority_from_codes(
    bool has_format,
    int32_t format,
    bool has_category,
    int32_t category) {
    if ((has_format && dsc_format_is_distress(format)) ||
        (has_category && dsc_category_is_distress(category))) {
        return ship_data_model::DscPriority::distress;
    }
    if (has_category && dsc_category_is_urgency(category)) return ship_data_model::DscPriority::urgency;
    if (has_category && dsc_category_is_safety(category)) return ship_data_model::DscPriority::safety;
    if (has_category && dsc_category_is_routine(category)) return ship_data_model::DscPriority::routine;
    return ship_data_model::DscPriority::unknown;
}

static ship_data_model::DscAddressType dsc_address_type_from_format(bool has_format, int32_t format) {
    if (!has_format) return ship_data_model::DscAddressType::unknown;
    switch (format) {
    case DSC_FORMAT_DISTRESS: return ship_data_model::DscAddressType::distress;
    case DSC_FORMAT_INDIVIDUAL: return ship_data_model::DscAddressType::individual;
    case DSC_FORMAT_ALL_SHIPS: return ship_data_model::DscAddressType::all_ships;
    case DSC_FORMAT_GROUP: return ship_data_model::DscAddressType::group;
    case DSC_FORMAT_GEOGRAPHIC_AREA: return ship_data_model::DscAddressType::geographic_area;
    default: return ship_data_model::DscAddressType::unknown;
    }
}

static ship_data_model::DscEndSignalType dsc_end_signal_type_from_char(char eos) {
    if (eos == 0) return ship_data_model::DscEndSignalType::unknown;
    if (dsc_end_signal_is_ack(eos)) return ship_data_model::DscEndSignalType::ack;
    return ship_data_model::DscEndSignalType::end_of_sequence;
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

static bool dsc_stamped_i32_equal(const ship_data_model::Stamped<int32_t>& a,
                                  const ship_data_model::Stamped<int32_t>& b) {
    if (a.valid != b.valid) return false;
    return !a.valid || a.value == b.value;
}

bool dsc_call_matches_duplicate_key(const ship_data_model::DscCallData<Real>& a,
                                    const ship_data_model::DscCallData<Real>& b) const {
    if (b.last_update_us == 0) return false;
    if (strcmp(a.sender_mmsi, b.sender_mmsi) != 0) return false;
    if (!dsc_stamped_i32_equal(a.format_specifier, b.format_specifier)) return false;
    if (!dsc_stamped_i32_equal(a.category, b.category)) return false;
    if (!dsc_stamped_i32_equal(a.nature_or_first_telecommand, b.nature_or_first_telecommand)) return false;
    if (!dsc_stamped_i32_equal(a.communication_or_second_telecommand, b.communication_or_second_telecommand)) return false;
    if (strcmp(a.position_code, b.position_code) != 0) return false;
    if (strcmp(a.address_or_distress_mmsi, b.address_or_distress_mmsi) != 0) return false;
    if (strcmp(a.field10, b.field10) != 0) return false;
    if (a.end_of_sequence != b.end_of_sequence) return false;
    if (a.priority != b.priority) return false;
    if (a.address_type != b.address_type) return false;
    return true;
}

int32_t find_recent_dsc_duplicate(const ship_data_model::DscCommData<Real>& comm,
                                  const ship_data_model::DscCallData<Real>& call,
                                  uint64_t now_us) const {
    for (uint8_t i = 0; i < ship_data_model::DSC_CALL_HISTORY_CAPACITY; ++i) {
        const auto& candidate = comm.recent_calls[i];
        if (candidate.last_update_us == 0) continue;
        if (now_us >= candidate.last_update_us &&
            now_us - candidate.last_update_us > DSC_DUPLICATE_WINDOW_US) {
            continue;
        }
        if (dsc_call_matches_duplicate_key(call, candidate)) return static_cast<int32_t>(i);
    }
    return -1;
}

void push_recent_dsc_call(ship_data_model::DscCommData<Real>& comm,
                          const ship_data_model::DscCallData<Real>& call,
                          uint64_t now_us) {
    int32_t next = comm.recent_call_next_index.valid ? comm.recent_call_next_index.value : 0;
    if (next < 0 || next >= ship_data_model::DSC_CALL_HISTORY_CAPACITY) next = 0;

    comm.recent_calls[static_cast<uint8_t>(next)] = call;

    int32_t count = comm.recent_call_count.valid ? comm.recent_call_count.value : 0;
    if (count < ship_data_model::DSC_CALL_HISTORY_CAPACITY) ++count;

    const int32_t next_after = (next + 1) % ship_data_model::DSC_CALL_HISTORY_CAPACITY;
    comm.recent_call_count.set(count, now_us);
    comm.recent_call_next_index.set(next_after, now_us);
}

template<typename Call>
void decode_dse_payload(Call& dst, const char* payload, uint64_t now_us) const {
    dst.dse_payload_type = ship_data_model::DscDsePayloadType::none;
    dst.dse_ascii_valid = true;
    dst.dse_payload_length.set(0, now_us);
    dst.dse_digit_count.set(0, now_us);
    dst.dse_decoded_text[0] = '\0';

    if (!payload || payload[0] == '\0') return;

    bool ascii_valid = true;
    bool all_digits = true;
    bool has_alpha = false;
    bool has_digit = false;
    bool has_other = false;
    int32_t length = 0;
    int32_t digit_count = 0;

    for (const char* p = payload; *p != '\0'; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        if (c < 0x20 || c > 0x7e) ascii_valid = false;
        if (dsc_char_is_digit(*p)) {
            has_digit = true;
            ++digit_count;
        } else {
            all_digits = false;
            if (dsc_char_is_alpha(*p)) {
                has_alpha = true;
            } else if (*p != ' ') {
                has_other = true;
            }
        }
        ++length;
    }

    dst.dse_ascii_valid = ascii_valid;
    dst.dse_payload_length.set(length, now_us);
    dst.dse_digit_count.set(digit_count, now_us);

    if (!ascii_valid) {
        dst.dse_payload_type = ship_data_model::DscDsePayloadType::invalid;
        return;
    }

    nmea_copy_cstr(dst.dse_decoded_text, sizeof(dst.dse_decoded_text), payload);
    if (all_digits && has_digit) {
        dst.dse_payload_type = ship_data_model::DscDsePayloadType::digits;
    } else if (has_other || (has_alpha && has_digit)) {
        dst.dse_payload_type = ship_data_model::DscDsePayloadType::mixed_ascii;
    } else {
        dst.dse_payload_type = ship_data_model::DscDsePayloadType::text;
    }
}

template<typename Alert>
bool dsc_alert_matches_current(const Alert& alert) const {
    const auto& dsc = dsc_state_.message;
    if (alert.first_seen_us == 0) return false;
    if (strcmp(alert.sender_mmsi, dsc.sender_mmsi) != 0) return false;
    if (dsc.category.last_update_us) {
        if (!alert.category.valid || alert.category.value != dsc.category.value) return false;
    }
    if (dsc.nature_or_first_telecommand.last_update_us) {
        if (!alert.nature_or_first_telecommand.valid ||
            alert.nature_or_first_telecommand.value != dsc.nature_or_first_telecommand.value) {
            return false;
        }
    }
    if (dsc.has_position) {
        if (!alert.latitude_deg.valid || !alert.longitude_deg.valid) return false;
        if (alert.latitude_deg.value != static_cast<Real>(dsc.latitude_deg)) return false;
        if (alert.longitude_deg.value != static_cast<Real>(dsc.longitude_deg)) return false;
    }
    return true;
}

template<typename Model, typename Alert>
bool commit_dsc_alert(Model& model,
                      Alert& alert,
                      const char* alert_text,
                      uint64_t now_us,
                      ship_data_model::SensorSource source,
                      bool duplicate) {
    const auto& dsc = dsc_state_.message;
    const bool same_alert = dsc_alert_matches_current(alert);
    const bool acknowledged = dsc_end_signal_is_ack(dsc.end_of_sequence);
    const int32_t previous_repeats = alert.repeat_count.valid ? alert.repeat_count.value : 0;
    const bool previously_acknowledged = same_alert && alert.acknowledged;

    if (acknowledged && !same_alert) return false;

    if (!same_alert) {
        alert = Alert{};
        alert.first_seen_us = now_us;
    }

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
    alert.acknowledged = previously_acknowledged || acknowledged;
    alert.resolved = false;
    alert.duplicate = duplicate;
    alert.repeat_count.set(same_alert ? previous_repeats + 1 : 1, now_us);
    alert.last_update_us = now_us;
    (void)model;
    return true;
}

template<typename Model>
bool promote_dsc_notifications(Model& model,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               bool duplicate) {
    const auto& dsc = dsc_state_.message;
    const int32_t format = dsc.format_specifier.last_update_us ? dsc.format_specifier.value : 0;
    const int32_t category = dsc.category.last_update_us ? dsc.category.value : 0;
    const bool acknowledged = dsc_end_signal_is_ack(dsc.end_of_sequence);

    if (dsc_format_is_distress(format) || dsc_category_is_distress(category)) {
        if (!commit_dsc_alert(model, model.notifications.dsc.distress, "DSC distress", now_us, source, duplicate)) return false;
        if (!duplicate && !acknowledged) model.comm.dsc.distress_count.set(model.comm.dsc.distress_count.value + 1, now_us);
        return true;
    } else if (dsc_category_is_urgency(category)) {
        if (!commit_dsc_alert(model, model.notifications.dsc.urgency, "DSC urgency", now_us, source, duplicate)) return false;
        if (!duplicate && !acknowledged) model.comm.dsc.urgency_count.set(model.comm.dsc.urgency_count.value + 1, now_us);
        return true;
    } else if (dsc_category_is_safety(category)) {
        if (!commit_dsc_alert(model, model.notifications.dsc.safety, "DSC safety", now_us, source, duplicate)) return false;
        if (!duplicate && !acknowledged) model.comm.dsc.safety_count.set(model.comm.dsc.safety_count.value + 1, now_us);
        return true;
    }

    return false;
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

    const bool has_format = dsc.format_specifier.last_update_us != 0;
    const bool has_category = dsc.category.last_update_us != 0;
    const int32_t format = has_format ? dsc.format_specifier.value : 0;
    const int32_t category = has_category ? dsc.category.value : 0;

    set_source(dst.source, source);
    if (has_format) dst.format_specifier.set(format, now_us);
    nmea_copy_cstr(dst.sender_mmsi, sizeof(dst.sender_mmsi), dsc.sender_mmsi);
    if (has_category) dst.category.set(category, now_us);
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
    dst.priority = dsc_priority_from_codes(has_format, format, has_category, category);
    dst.address_type = dsc_address_type_from_format(has_format, format);
    dst.end_signal_type = dsc_end_signal_type_from_char(dsc.end_of_sequence);
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
        decode_dse_payload(dst, dst.dse_payload, now_us);
    }

    dst.field_count.set(dst.expansion_expected ? 11 : 10, now_us);
    dst.first_seen_us = dsc.last_update_us != 0 ? dsc.last_update_us : now_us;
    dst.last_update_us = now_us;
    dst.repeat_count.set(1, now_us);

    const bool acknowledged = dsc_end_signal_is_ack(dst.end_of_sequence);
    const bool alert_ack = acknowledged && dsc_priority_is_alert(dst.priority);
    const int32_t duplicate_index = find_recent_dsc_duplicate(comm, dst, now_us);
    if (duplicate_index >= 0) {
        const auto& previous = comm.recent_calls[static_cast<uint8_t>(duplicate_index)];
        const int32_t previous_repeats = previous.repeat_count.valid ? previous.repeat_count.value : 1;
        dst.duplicate = true;
        dst.repeat_count.set(previous_repeats + 1, now_us);
        if (previous.first_seen_us != 0) dst.first_seen_us = previous.first_seen_us;
        const bool promoted = promote_dsc_notifications(model, now_us, source, true);
        if (alert_ack && !promoted) dst.orphan_ack = true;
        comm.recent_calls[static_cast<uint8_t>(duplicate_index)] = dst;
        comm.duplicate_count.set(comm.duplicate_count.value + 1, now_us);
        return true;
    }

    const bool promoted = promote_dsc_notifications(model, now_us, source, false);
    if (alert_ack && promoted) {
        comm.acknowledged_count.set(comm.acknowledged_count.value + 1, now_us);
    } else if (alert_ack && !promoted) {
        dst.orphan_ack = true;
        comm.orphan_ack_count.set(comm.orphan_ack_count.value + 1, now_us);
    }

    push_recent_dsc_call(comm, dst, now_us);
    comm.call_count.set(comm.call_count.value + 1, now_us);
    if (expansion_timeout) {
        comm.expansion_timeout_count.set(comm.expansion_timeout_count.value + 1, now_us);
    }

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
