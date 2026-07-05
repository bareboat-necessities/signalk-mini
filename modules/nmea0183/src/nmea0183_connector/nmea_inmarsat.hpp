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

bool inmarsat_is_standard_message(const NmeaSentence& sentence) const {
    return sentence_is(sentence, "IMK") || sentence_is(sentence, "IMN") || sentence_is(sentence, "IMR");
}

bool inmarsat_is_proprietary_message(const NmeaSentence& sentence) const {
    return inmarsat_body_starts_with(sentence, "PINM") || inmarsat_body_starts_with(sentence, "INM");
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
void set_inmarsat_message_type(const NmeaSentence& sentence, Message& message) const {
    if (inmarsat_body_starts_with(sentence, "PINM")) {
        nmea_copy_cstr(message.message_type, sizeof(message.message_type), "PINM");
    } else if (inmarsat_body_starts_with(sentence, "INM")) {
        nmea_copy_cstr(message.message_type, sizeof(message.message_type), "INM");
    } else {
        nmea_copy_span(message.message_type, sizeof(message.message_type), sentence.sentence);
    }
}

template<typename Model>
bool commit_inmarsat_multipart_message(const NmeaSentence& sentence,
                                       Model& model,
                                       uint64_t now_us,
                                       ship_data_model::SensorSource source) {
    const auto& assembled = state_.inmarsat_message;
    auto& inmarsat = model.comm.inmarsat;
    auto& dst = inmarsat.latest_message;
    dst = ship_data_model::InmarsatMessageData<Real>{};

    set_source(dst.source, source);
    nmea_copy_cstr(dst.message_id, sizeof(dst.message_id), assembled.message_id);
    nmea_copy_cstr(dst.terminal_id, sizeof(dst.terminal_id), assembled.talker_id);
    nmea_copy_cstr(dst.message_type, sizeof(dst.message_type), assembled.sentence_id);
    nmea_copy_cstr(dst.decoded_text, sizeof(dst.decoded_text), assembled.text);

    if (assembled.total_fragments.last_update_us) {
        dst.total_fragments.set(assembled.total_fragments.value, now_us);
    }
    if (assembled.last_fragment_number.last_update_us) {
        dst.last_fragment_number.set(assembled.last_fragment_number.value, now_us);
    }
    if (assembled.text_length.last_update_us) {
        dst.text_length.set(assembled.text_length.value, now_us);
    } else {
        dst.text_length.set(static_cast<int32_t>(strlen(assembled.text)), now_us);
    }

    dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    dst.complete = assembled.complete;
    dst.overflow = assembled.overflow || strlen(assembled.text) >= sizeof(dst.decoded_text);
    dst.first_seen_us = now_us;
    dst.last_update_us = now_us;
    inmarsat.message_count.set(inmarsat.message_count.value + 1, now_us);
    return true;
}

template<typename Message>
void set_inmarsat_single_identity(const NmeaSentence& sentence, Message& dst) const {
    set_inmarsat_message_type(sentence, dst);
    if (sentence.field_count > 0) {
        nmea_copy_span(dst.message_id, sizeof(dst.message_id), sentence.field(0));
    } else {
        nmea_copy_span(dst.message_id, sizeof(dst.message_id), sentence.sentence);
    }
    nmea_copy_span(dst.terminal_id, sizeof(dst.terminal_id), sentence.talker);
    if ((inmarsat_is_standard_message(sentence) || inmarsat_is_proprietary_message(sentence)) && sentence.field_count > 1) {
        nmea_copy_span(dst.message_status, sizeof(dst.message_status), sentence.field(1));
    }
}

template<typename Model>
bool commit_inmarsat_single_message(const NmeaSentence& sentence,
                                    Model& model,
                                    uint64_t now_us,
                                    ship_data_model::SensorSource source) {
    auto& inmarsat = model.comm.inmarsat;
    auto& dst = inmarsat.latest_message;
    dst = ship_data_model::InmarsatMessageData<Real>{};

    const NmeaSpan payload = inmarsat_best_payload_field(sentence);
    if (payload.empty()) {
        inmarsat.unsupported_count.set(inmarsat.unsupported_count.value + 1, now_us);
        return true;
    }
    if (!inmarsat_span_printable_ascii(payload)) {
        inmarsat.unsupported_count.set(inmarsat.unsupported_count.value + 1, now_us);
        return true;
    }

    set_source(dst.source, source);
    set_inmarsat_single_identity(sentence, dst);
    nmea_copy_span(dst.decoded_text, sizeof(dst.decoded_text), payload);

    dst.total_fragments.set(1, now_us);
    dst.last_fragment_number.set(1, now_us);
    dst.text_length.set(static_cast<int32_t>(payload.length), now_us);
    dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    dst.complete = true;
    dst.overflow = payload.length >= sizeof(dst.decoded_text);
    dst.first_seen_us = now_us;
    dst.last_update_us = now_us;
    inmarsat.message_count.set(inmarsat.message_count.value + 1, now_us);
    return true;
}

template<typename Model>
bool apply_inmarsat(const NmeaSentence& sentence,
                    Model& model,
                    uint64_t now_us,
                    ship_data_model::SensorSource source,
                    bool bad_fragment_before_update = false) {
    if (sentence.fragment.is_fragmented) {
        const auto& assembled = state_.inmarsat_message;
        if (bad_fragment_before_update || assembled.bad_fragment_count.last_update_us == now_us) {
            model.comm.inmarsat.bad_fragment_count.set(model.comm.inmarsat.bad_fragment_count.value + 1, now_us);
            return true;
        }
        if (!assembled.complete) return true;
        if (!inmarsat_cstr_printable_ascii(assembled.text)) {
            model.comm.inmarsat.unsupported_count.set(model.comm.inmarsat.unsupported_count.value + 1, now_us);
            return true;
        }
        return commit_inmarsat_multipart_message(sentence, model, now_us, source);
    }

    return commit_inmarsat_single_message(sentence, model, now_us, source);
}
