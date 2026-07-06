#pragma once

// Included inside Nmea0183RxConnector.

static bool inmarsat_body_starts_with(const NmeaSentence& sentence, const char* prefix) {
    return nmea_span_starts_with(sentence.body, prefix);
}

static bool inmarsat_span_printable_ascii(NmeaSpan span) {
    for (uint8_t i = 0; i < span.length; ++i) {
        const unsigned char c = static_cast<unsigned char>(span[i]);
        if (c < 0x20 || c > 0x7e) return false;
    }
    return true;
}

static bool inmarsat_cstr_printable_ascii(const char* text) {
    if (!text) return false;
    while (*text) {
        const unsigned char c = static_cast<unsigned char>(*text++);
        if (c < 0x20 || c > 0x7e) return false;
    }
    return true;
}

static NmeaSpan inmarsat_best_payload_field(const NmeaSentence& sentence) {
    for (int index = static_cast<int>(sentence.field_count) - 1; index >= 0; --index) {
        const NmeaSpan field = sentence.field(static_cast<uint8_t>(index));
        if (!field.empty()) return field;
    }
    return NmeaSpan();
}

enum class InmarsatConnectorSentenceKind : uint8_t {
    unknown,
    imk,
    imn,
    imr,
    sm1,
    sm2,
    sm3,
    sm4,
    smb,
    pinm,
    inm
};

enum class InmarsatSafetyNetHeaderKind : uint8_t {
    none,
    all_ships_or_navarea,
    coastal_warning_area,
    circular_area,
    rectangular_area
};

InmarsatConnectorSentenceKind inmarsat_sentence_kind_from_sentence(const NmeaSentence& sentence) const {
    if (sentence_is(sentence, "IMK")) return InmarsatConnectorSentenceKind::imk;
    if (sentence_is(sentence, "IMN")) return InmarsatConnectorSentenceKind::imn;
    if (sentence_is(sentence, "IMR")) return InmarsatConnectorSentenceKind::imr;
    if (sentence_is(sentence, "SM1")) return InmarsatConnectorSentenceKind::sm1;
    if (sentence_is(sentence, "SM2")) return InmarsatConnectorSentenceKind::sm2;
    if (sentence_is(sentence, "SM3")) return InmarsatConnectorSentenceKind::sm3;
    if (sentence_is(sentence, "SM4")) return InmarsatConnectorSentenceKind::sm4;
    if (sentence_is(sentence, "SMB")) return InmarsatConnectorSentenceKind::smb;
    if (inmarsat_body_starts_with(sentence, "PINM")) return InmarsatConnectorSentenceKind::pinm;
    if (inmarsat_body_starts_with(sentence, "INM")) return InmarsatConnectorSentenceKind::inm;
    return InmarsatConnectorSentenceKind::unknown;
}

InmarsatSafetyNetHeaderKind inmarsat_safetynet_header_kind_from_sentence(const NmeaSentence& sentence) const {
    if (sentence_is(sentence, "SM1")) return InmarsatSafetyNetHeaderKind::all_ships_or_navarea;
    if (sentence_is(sentence, "SM2")) return InmarsatSafetyNetHeaderKind::coastal_warning_area;
    if (sentence_is(sentence, "SM3")) return InmarsatSafetyNetHeaderKind::circular_area;
    if (sentence_is(sentence, "SM4")) return InmarsatSafetyNetHeaderKind::rectangular_area;
    return InmarsatSafetyNetHeaderKind::none;
}

bool inmarsat_fragment_matches_active_record(const NmeaSentence& sentence) const {
    const auto& record = state_.inmarsat_message;
    return (record.in_progress || record.complete) &&
           nmea_span_equals(sentence.sentence, record.sentence_id) &&
           nmea_span_equals(sentence.talker, record.talker_id) &&
           nmea_span_equals(sentence.fragment.message_id, record.message_id);
}

bool inmarsat_bad_non_first_fragment_before_update(const NmeaSentence& sentence) const {
    return sentence.family == NmeaSentenceFamily::Inmarsat &&
           sentence.fragment.is_fragmented &&
           !sentence.fragment.is_first() &&
           !inmarsat_fragment_matches_active_record(sentence);
}

template<typename Message>
bool inmarsat_same_notification_message(const Message& a, const Message& b) const {
    return strcmp(a.message_id, b.message_id) == 0 &&
           strcmp(a.terminal_id, b.terminal_id) == 0 &&
           strcmp(a.status, b.status) == 0 &&
           strcmp(a.message_text, b.message_text) == 0;
}

template<typename SafetyNetData, typename Message>
void store_inmarsat_recent_message(SafetyNetData& safetynet, Message& message, uint64_t now_us) const {
    const uint8_t capacity = ship_data_model::INMARSAT_SAFETYNET_MESSAGE_HISTORY_CAPACITY;
    for (uint8_t i = 0; i < capacity; ++i) {
        auto& slot = safetynet.recent_messages[i];
        if (slot.first_seen_us != 0 && inmarsat_same_notification_message(slot, message)) {
            const int32_t repeat = slot.repeat_count.valid ? slot.repeat_count.value + 1 : 2;
            const uint64_t first_seen = slot.first_seen_us;
            slot = message;
            slot.first_seen_us = first_seen;
            slot.last_update_us = now_us;
            slot.duplicate = true;
            slot.repeat_count.set(repeat, now_us);
            message = slot;
            safetynet.duplicate_count.set(safetynet.duplicate_count.value + 1, now_us);
            return;
        }
    }

    int32_t next = safetynet.recent_message_next_index.valid ? safetynet.recent_message_next_index.value : 0;
    if (next < 0 || next >= static_cast<int32_t>(capacity)) next = 0;
    auto& slot = safetynet.recent_messages[static_cast<uint8_t>(next)];
    const bool overwriting = slot.first_seen_us != 0;
    if (message.first_seen_us == 0) message.first_seen_us = now_us;
    message.last_update_us = now_us;
    message.duplicate = false;
    message.repeat_count.set(1, now_us);
    slot = message;

    const int32_t count = safetynet.recent_message_count.valid ? safetynet.recent_message_count.value : 0;
    if (count < static_cast<int32_t>(capacity)) safetynet.recent_message_count.set(count + 1, now_us);
    if (overwriting) safetynet.overwrite_count.set(safetynet.overwrite_count.value + 1, now_us);
    safetynet.recent_message_next_index.set((next + 1) % static_cast<int32_t>(capacity), now_us);
}

template<typename SafetyNetData, typename Message>
void commit_inmarsat_notification_message(SafetyNetData& safetynet, Message& message, uint64_t now_us) const {
    store_inmarsat_recent_message(safetynet, message, now_us);
    safetynet.message_count.set(safetynet.message_count.value + 1, now_us);
}

template<typename Model>
bool commit_inmarsat_multipart_message(const NmeaSentence& sentence,
                                       Model& model,
                                       uint64_t now_us,
                                       ship_data_model::SensorSource source) {
    const auto& assembled = state_.inmarsat_message;
    auto& safetynet = model.notifications.inmarsat.safetynet;
    auto& dst = safetynet.latest_message;
    dst = ship_data_model::InmarsatSafetyNetMessageData<Real>{};

    set_source(dst.source, source);
    nmea_copy_cstr(dst.message_id, sizeof(dst.message_id), assembled.message_id);
    nmea_copy_cstr(dst.terminal_id, sizeof(dst.terminal_id), assembled.talker_id);
    nmea_copy_cstr(dst.message_text, sizeof(dst.message_text), assembled.text);

    if (assembled.total_fragments.last_update_us) {
        dst.total_fragments.set(assembled.total_fragments.value, now_us);
    }
    if (assembled.last_fragment_number.last_update_us) {
        dst.fragment_number.set(assembled.last_fragment_number.value, now_us);
    }
    if (assembled.text_length.last_update_us) {
        dst.text_length.set(assembled.text_length.value, now_us);
    } else {
        dst.text_length.set(static_cast<int32_t>(strlen(assembled.text)), now_us);
    }

    dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    dst.complete = assembled.complete;
    dst.overflow = assembled.overflow || strlen(assembled.text) >= sizeof(dst.message_text);
    dst.first_seen_us = now_us;
    dst.last_update_us = now_us;
    commit_inmarsat_notification_message(safetynet, dst, now_us);
    return true;
}

template<typename Message>
void set_inmarsat_single_identity(const NmeaSentence& sentence, Message& dst) const {
    if (sentence.field_count > 0) {
        nmea_copy_span(dst.message_id, sizeof(dst.message_id), sentence.field(0));
    } else {
        nmea_copy_span(dst.message_id, sizeof(dst.message_id), sentence.sentence);
    }
    nmea_copy_span(dst.terminal_id, sizeof(dst.terminal_id), sentence.talker);
    if (sentence.field_count > 1) {
        nmea_copy_span(dst.status, sizeof(dst.status), sentence.field(1));
    }
}

template<typename Model>
bool commit_inmarsat_single_message(const NmeaSentence& sentence,
                                    Model& model,
                                    uint64_t now_us,
                                    ship_data_model::SensorSource source) {
    auto& safetynet = model.notifications.inmarsat.safetynet;
    auto& dst = safetynet.latest_message;
    dst = ship_data_model::InmarsatSafetyNetMessageData<Real>{};

    const NmeaSpan payload = inmarsat_best_payload_field(sentence);
    if (payload.empty()) {
        safetynet.unsupported_count.set(safetynet.unsupported_count.value + 1, now_us);
        return true;
    }
    if (!inmarsat_span_printable_ascii(payload)) {
        safetynet.unsupported_count.set(safetynet.unsupported_count.value + 1, now_us);
        return true;
    }

    set_source(dst.source, source);
    set_inmarsat_single_identity(sentence, dst);
    nmea_copy_span(dst.message_text, sizeof(dst.message_text), payload);

    dst.total_fragments.set(1, now_us);
    dst.fragment_number.set(1, now_us);
    dst.text_length.set(static_cast<int32_t>(payload.length), now_us);
    dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    dst.complete = true;
    dst.overflow = payload.length >= sizeof(dst.message_text);
    dst.first_seen_us = now_us;
    dst.last_update_us = now_us;
    commit_inmarsat_notification_message(safetynet, dst, now_us);
    return true;
}

template<typename Model>
bool apply_inmarsat(const NmeaSentence& sentence,
                    Model& model,
                    uint64_t now_us,
                    ship_data_model::SensorSource source,
                    bool bad_fragment_before_update = false) {
    (void)inmarsat_sentence_kind_from_sentence(sentence);
    (void)inmarsat_safetynet_header_kind_from_sentence(sentence);

    auto& safetynet = model.notifications.inmarsat.safetynet;
    if (sentence.fragment.is_fragmented) {
        const auto& assembled = state_.inmarsat_message;
        if (bad_fragment_before_update || assembled.bad_fragment_count.last_update_us == now_us) {
            safetynet.bad_fragment_count.set(safetynet.bad_fragment_count.value + 1, now_us);
            return true;
        }
        if (!assembled.complete) return true;
        if (!inmarsat_cstr_printable_ascii(assembled.text)) {
            safetynet.unsupported_count.set(safetynet.unsupported_count.value + 1, now_us);
            return true;
        }
        return commit_inmarsat_multipart_message(sentence, model, now_us, source);
    }

    return commit_inmarsat_single_message(sentence, model, now_us, source);
}
