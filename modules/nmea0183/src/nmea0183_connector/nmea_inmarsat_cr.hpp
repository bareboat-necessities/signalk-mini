#pragma once

// Included inside Nmea0183RxConnector after nmea_inmarsat.hpp.

bool inmarsat_cr_sentence_is_from_terminal(const NmeaSentence& sentence) {
    if (!talker_is(sentence, "CR")) {
        last_error_ = "unsupported CR terminal sentence talker";
        return false;
    }
    return true;
}

template<typename Message>
bool inmarsat_cr_message_matches_id(const Message& message, const char* message_id) const {
    return message_id && message_id[0] != '\0' && message.message_id[0] != '\0' && strcmp(message.message_id, message_id) == 0;
}

template<typename SafetyNetData>
void inmarsat_cr_mark_message_acknowledged(SafetyNetData& safetynet, const char* message_id, uint64_t now_us) const {
    if (!message_id || message_id[0] == '\0') return;
    if (inmarsat_cr_message_matches_id(safetynet.latest_message, message_id)) {
        safetynet.latest_message.acknowledged = true;
        safetynet.latest_message.last_update_us = now_us;
    }
    for (uint8_t i = 0; i < ship_data_model::INMARSAT_SAFETYNET_MESSAGE_HISTORY_CAPACITY; ++i) {
        auto& slot = safetynet.recent_messages[i];
        if (inmarsat_cr_message_matches_id(slot, message_id)) {
            slot.acknowledged = true;
            slot.last_update_us = now_us;
        }
    }
}

template<typename Model>
bool apply_inmarsat_can(const NmeaSentence& sentence,
                        Model& model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
    if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
    if (sentence.field_count < 1) { last_error_ = "short CRCAN"; return false; }

    char message_id[16] = {0};
    nmea_copy_span(message_id, sizeof(message_id), sentence.field(0));
    inmarsat_cr_mark_message_acknowledged(model.notifications.inmarsat.safetynet, message_id, now_us);

    auto& event = model.notifications.messages.event;
    nmea_copy_span(event.event_id, sizeof(event.event_id), sentence.field(0));
    nmea_copy_cstr(event.event_source, sizeof(event.event_source), "inmarsat");
    event.event_state = sentence.field_count > 1 && !sentence.field(1).empty() ? sentence.field(1)[0] : 'C';
    if (sentence.field_count > 2) nmea_copy_span(event.event_text, sizeof(event.event_text), sentence.field(2));
    else nmea_copy_cstr(event.event_text, sizeof(event.event_text), "message_cancelled");
    event.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(event.source, source);
    event.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_inmarsat_crq(const NmeaSentence& sentence,
                        Model& model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
    if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
    if (sentence.field_count < 1) { last_error_ = "short CRCRQ"; return false; }

    auto& rec = model.notifications.messages.text;
    if (sentence.field_count > 0) nmea_copy_span(rec.id, sizeof(rec.id), sentence.field(0));
    else nmea_copy_cstr(rec.id, sizeof(rec.id), "CRQ");
    if (sentence.field_count > 1) nmea_copy_span(rec.code, sizeof(rec.code), sentence.field(1));
    else nmea_copy_cstr(rec.code, sizeof(rec.code), "query");
    if (sentence.field_count > 2) nmea_copy_span(rec.value, sizeof(rec.value), sentence.field(2));
    if (sentence.field_count > 3) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(3));
    else if (sentence.field_count > 2) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(2));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_inmarsat_dsm(const NmeaSentence& sentence,
                        Model& model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
    if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
    if (sentence.field_count < 1) { last_error_ = "short CRDSM"; return false; }

    auto& rec = model.notifications.messages.text;
    nmea_copy_cstr(rec.id, sizeof(rec.id), "DSM");
    if (sentence.field_count > 0) nmea_copy_span(rec.code, sizeof(rec.code), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(rec.value, sizeof(rec.value), sentence.field(1));
    if (sentence.field_count > 2) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(2));
    else if (sentence.field_count > 1) nmea_copy_span(rec.text, sizeof(rec.text), sentence.field(1));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;

    auto& latest = model.notifications.inmarsat.safetynet.latest_message;
    set_source(latest.source, source);
    nmea_copy_span(latest.terminal_id, sizeof(latest.terminal_id), sentence.talker);
    if (sentence.field_count > 0 && !sentence.field(0).empty()) {
        latest.ocean_region_code = sentence.field(0)[0];
        nmea_copy_cstr(latest.ocean_region_label, sizeof(latest.ocean_region_label), inmarsat_ocean_region_label(latest.ocean_region_code));
    }
    if (sentence.field_count > 1 && !sentence.field(1).empty()) {
        latest.priority_code = sentence.field(1)[0];
        nmea_copy_cstr(latest.priority_label, sizeof(latest.priority_label), inmarsat_priority_label(latest.priority_code));
    }
    if (sentence.field_count > 2) {
        nmea_copy_span(latest.service_code, sizeof(latest.service_code), sentence.field(2));
        nmea_copy_cstr(latest.service_label, sizeof(latest.service_label), inmarsat_service_label(latest.service_code));
    }
    latest.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_inmarsat_tmd(const NmeaSentence& sentence,
                        Model& model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
    if (!inmarsat_cr_sentence_is_from_terminal(sentence)) return false;
    if (sentence.field_count < 1) { last_error_ = "short CRTMD"; return false; }

    NmeaSentence multipart = sentence;
    nmea_set_fragment_info(multipart, 0, 1, 2, 3);
    if (multipart.fragment.is_fragmented) {
        update_inmarsat_multipart_message_state(multipart, now_us, source);
        if (!state_.inmarsat_message.complete) return true;
        if (!inmarsat_cstr_printable_ascii(state_.inmarsat_message.text)) {
            auto& safetynet = model.notifications.inmarsat.safetynet;
            safetynet.unsupported_count.set(safetynet.unsupported_count.value + 1, now_us);
            return true;
        }
        return commit_inmarsat_multipart_message(multipart, model, now_us, source);
    }

    auto& safetynet = model.notifications.inmarsat.safetynet;
    auto& dst = safetynet.latest_message;
    dst = ship_data_model::InmarsatSafetyNetMessageData<Real>{};
    set_source(dst.source, source);
    nmea_copy_span(dst.terminal_id, sizeof(dst.terminal_id), sentence.talker);
    if (sentence.field_count > 0) nmea_copy_span(dst.message_id, sizeof(dst.message_id), sentence.field(0));
    else nmea_copy_cstr(dst.message_id, sizeof(dst.message_id), "TMD");
    if (sentence.field_count > 1 && !sentence.field(1).empty()) dst.msi_status = sentence.field(1)[0];
    NmeaSpan payload = inmarsat_best_payload_field(sentence);
    if (!payload.empty()) nmea_copy_span(dst.message_text, sizeof(dst.message_text), payload);
    dst.total_fragments.set(1, now_us);
    dst.fragment_number.set(1, now_us);
    dst.text_length.set(static_cast<int32_t>(strlen(dst.message_text)), now_us);
    dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    dst.body_complete = true;
    dst.complete = true;
    dst.overflow = payload.length >= sizeof(dst.message_text);
    dst.body_has_unknown_char = inmarsat_span_has_unknown_char(payload);
    dst.first_seen_us = now_us;
    dst.last_update_us = now_us;
    commit_inmarsat_notification_message(safetynet, dst, now_us);
    return true;
}
