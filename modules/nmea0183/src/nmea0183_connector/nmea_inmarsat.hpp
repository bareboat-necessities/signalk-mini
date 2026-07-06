#pragma once

// Included inside Nmea0183RxConnector.

static bool inmarsat_body_starts_with(const NmeaSentence& sentence, const char* prefix) {
    return nmea_span_starts_with(sentence.body, prefix);
}

static bool inmarsat_char_is_digit(char c) {
    return c >= '0' && c <= '9';
}

static bool inmarsat_char_is_alpha(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

static bool inmarsat_char_is_token_separator(char c) {
    return c == ' ' || c == ';' || c == '|' || c == '/' || c == ':' || c == '=';
}

static bool inmarsat_char_is_structured_separator(char c) {
    return c == ';' || c == '|' || c == '/' || c == ':';
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

ship_data_model::InmarsatSentenceType inmarsat_sentence_type_from_sentence(const NmeaSentence& sentence) const {
    if (sentence_is(sentence, "IMK")) return ship_data_model::InmarsatSentenceType::imk;
    if (sentence_is(sentence, "IMN")) return ship_data_model::InmarsatSentenceType::imn;
    if (sentence_is(sentence, "IMR")) return ship_data_model::InmarsatSentenceType::imr;
    if (inmarsat_body_starts_with(sentence, "PINM")) return ship_data_model::InmarsatSentenceType::pinm;
    if (inmarsat_body_starts_with(sentence, "INM")) return ship_data_model::InmarsatSentenceType::inm;
    return ship_data_model::InmarsatSentenceType::unknown;
}

ship_data_model::InmarsatSentenceType inmarsat_sentence_type_from_id(const char* sentence_id) const {
    if (!sentence_id) return ship_data_model::InmarsatSentenceType::unknown;
    if (strcmp(sentence_id, "IMK") == 0) return ship_data_model::InmarsatSentenceType::imk;
    if (strcmp(sentence_id, "IMN") == 0) return ship_data_model::InmarsatSentenceType::imn;
    if (strcmp(sentence_id, "IMR") == 0) return ship_data_model::InmarsatSentenceType::imr;
    if (strcmp(sentence_id, "PINM") == 0) return ship_data_model::InmarsatSentenceType::pinm;
    if (strcmp(sentence_id, "INM") == 0) return ship_data_model::InmarsatSentenceType::inm;
    return ship_data_model::InmarsatSentenceType::unknown;
}

static void inmarsat_copy_cstr_part(char* out, size_t out_size, const char* text, size_t begin, size_t end) {
    if (!out || out_size == 0) return;
    size_t n = end > begin ? end - begin : 0u;
    if (n + 1u > out_size) n = out_size - 1u;
    if (n && text) memcpy(out, text + begin, n);
    out[n] = '\0';
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

template<typename Message>
void set_inmarsat_sentence_decoding(Message& message, ship_data_model::InmarsatSentenceType sentence_type) const {
    message.sentence_type = sentence_type;
    message.standard_sentence = sentence_type == ship_data_model::InmarsatSentenceType::imk ||
                                sentence_type == ship_data_model::InmarsatSentenceType::imn ||
                                sentence_type == ship_data_model::InmarsatSentenceType::imr;
    message.proprietary_sentence = sentence_type == ship_data_model::InmarsatSentenceType::pinm ||
                                   sentence_type == ship_data_model::InmarsatSentenceType::inm;
}

template<typename Message>
bool inmarsat_same_decoded_message(const Message& a, const Message& b) const {
    return strcmp(a.message_id, b.message_id) == 0 &&
           strcmp(a.terminal_id, b.terminal_id) == 0 &&
           strcmp(a.message_type, b.message_type) == 0 &&
           strcmp(a.message_status, b.message_status) == 0 &&
           strcmp(a.decoded_text, b.decoded_text) == 0;
}

template<typename InmarsatData, typename Message>
void store_inmarsat_recent_message(InmarsatData& inmarsat, Message& message, uint64_t now_us) const {
    const uint8_t capacity = ship_data_model::INMARSAT_MESSAGE_HISTORY_CAPACITY;
    for (uint8_t i = 0; i < capacity; ++i) {
        auto& slot = inmarsat.recent_messages[i];
        if (slot.first_seen_us != 0 && inmarsat_same_decoded_message(slot, message)) {
            const int32_t repeat = slot.repeat_count.valid ? slot.repeat_count.value + 1 : 2;
            const uint64_t first_seen = slot.first_seen_us;
            slot = message;
            slot.first_seen_us = first_seen;
            slot.last_update_us = now_us;
            slot.duplicate = true;
            slot.repeat_count.set(repeat, now_us);
            message = slot;
            inmarsat.duplicate_count.set(inmarsat.duplicate_count.value + 1, now_us);
            return;
        }
    }

    int32_t next = inmarsat.recent_message_next_index.valid ? inmarsat.recent_message_next_index.value : 0;
    if (next < 0 || next >= static_cast<int32_t>(capacity)) next = 0;
    auto& slot = inmarsat.recent_messages[static_cast<uint8_t>(next)];
    const bool overwriting = slot.first_seen_us != 0;
    if (message.first_seen_us == 0) message.first_seen_us = now_us;
    message.last_update_us = now_us;
    message.duplicate = false;
    message.repeat_count.set(1, now_us);
    slot = message;

    const int32_t count = inmarsat.recent_message_count.valid ? inmarsat.recent_message_count.value : 0;
    if (count < static_cast<int32_t>(capacity)) inmarsat.recent_message_count.set(count + 1, now_us);
    if (overwriting) inmarsat.overwrite_count.set(inmarsat.overwrite_count.value + 1, now_us);
    inmarsat.recent_message_next_index.set((next + 1) % static_cast<int32_t>(capacity), now_us);
}

template<typename InmarsatData, typename Message>
void commit_inmarsat_decoded_message(InmarsatData& inmarsat, Message& message, uint64_t now_us) const {
    store_inmarsat_recent_message(inmarsat, message, now_us);
    inmarsat.message_count.set(inmarsat.message_count.value + 1, now_us);
}

template<typename Message>
void decode_inmarsat_payload(Message& dst, const char* text, uint64_t now_us) const {
    dst.payload_type = ship_data_model::InmarsatPayloadType::none;
    dst.ascii_valid = true;
    dst.payload_length_chars.set(0, now_us);
    dst.digit_count.set(0, now_us);
    dst.alpha_count.set(0, now_us);
    dst.separator_count.set(0, now_us);
    dst.token_count.set(0, now_us);
    dst.key_value_count.set(0, now_us);
    dst.first_token[0] = '\0';
    dst.second_token[0] = '\0';
    dst.first_key[0] = '\0';
    dst.first_value[0] = '\0';

    if (!text || text[0] == '\0') return;

    bool ascii_valid = true;
    bool all_digits = true;
    bool has_digit = false;
    bool has_alpha = false;
    bool has_other = false;
    bool has_structured_separator = false;
    int32_t length = 0;
    int32_t digit_count = 0;
    int32_t alpha_count = 0;
    int32_t separator_count = 0;
    int32_t token_count = 0;
    int32_t key_value_count = 0;
    size_t token_start = 0u;
    bool in_token = false;
    bool copied_first_token = false;
    bool copied_second_token = false;

    for (size_t i = 0u; text[i] != '\0'; ++i) {
        const char c = text[i];
        const unsigned char uc = static_cast<unsigned char>(c);
        const bool separator = inmarsat_char_is_token_separator(c);
        if (uc < 0x20 || uc > 0x7e) ascii_valid = false;
        if (inmarsat_char_is_digit(c)) {
            has_digit = true;
            ++digit_count;
        } else {
            all_digits = false;
            if (inmarsat_char_is_alpha(c)) {
                has_alpha = true;
                ++alpha_count;
            } else if (separator) {
                ++separator_count;
                if (inmarsat_char_is_structured_separator(c)) has_structured_separator = true;
            } else {
                has_other = true;
            }
        }

        if (!separator && !in_token) {
            in_token = true;
            token_start = i;
        }
        if ((separator || text[i + 1u] == '\0') && in_token) {
            const size_t token_end = separator ? i : i + 1u;
            if (token_end > token_start) {
                ++token_count;
                if (!copied_first_token) {
                    inmarsat_copy_cstr_part(dst.first_token, sizeof(dst.first_token), text, token_start, token_end);
                    copied_first_token = true;
                } else if (!copied_second_token) {
                    inmarsat_copy_cstr_part(dst.second_token, sizeof(dst.second_token), text, token_start, token_end);
                    copied_second_token = true;
                }
            }
            in_token = false;
        }
        ++length;
    }

    const char* eq = strchr(text, '=');
    if (eq && eq != text && eq[1] != '\0') {
        const size_t key_end = static_cast<size_t>(eq - text);
        size_t value_end = key_end + 1u;
        while (text[value_end] != '\0' && !inmarsat_char_is_token_separator(text[value_end])) ++value_end;
        inmarsat_copy_cstr_part(dst.first_key, sizeof(dst.first_key), text, 0u, key_end);
        inmarsat_copy_cstr_part(dst.first_value, sizeof(dst.first_value), text, key_end + 1u, value_end);
    }
    for (const char* p = text; *p != '\0'; ++p) {
        if (*p == '=' && p != text && p[1] != '\0') ++key_value_count;
    }

    dst.ascii_valid = ascii_valid;
    dst.payload_length_chars.set(length, now_us);
    dst.digit_count.set(digit_count, now_us);
    dst.alpha_count.set(alpha_count, now_us);
    dst.separator_count.set(separator_count, now_us);
    dst.token_count.set(token_count, now_us);
    dst.key_value_count.set(key_value_count, now_us);

    if (!ascii_valid) {
        dst.payload_type = ship_data_model::InmarsatPayloadType::invalid;
    } else if (all_digits && has_digit) {
        dst.payload_type = ship_data_model::InmarsatPayloadType::digits;
    } else if (key_value_count > 0) {
        dst.payload_type = ship_data_model::InmarsatPayloadType::key_value;
    } else if (has_structured_separator && token_count > 1) {
        dst.payload_type = ship_data_model::InmarsatPayloadType::delimited;
    } else if (has_other || (has_alpha && has_digit)) {
        dst.payload_type = ship_data_model::InmarsatPayloadType::mixed_ascii;
    } else {
        dst.payload_type = ship_data_model::InmarsatPayloadType::text;
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
    set_inmarsat_sentence_decoding(dst, inmarsat_sentence_type_from_id(assembled.sentence_id));
    decode_inmarsat_payload(dst, dst.decoded_text, now_us);

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
    commit_inmarsat_decoded_message(inmarsat, dst, now_us);
    return true;
}

template<typename Message>
void set_inmarsat_single_identity(const NmeaSentence& sentence, Message& dst) const {
    set_inmarsat_message_type(sentence, dst);
    set_inmarsat_sentence_decoding(dst, inmarsat_sentence_type_from_sentence(sentence));
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
    decode_inmarsat_payload(dst, dst.decoded_text, now_us);

    dst.total_fragments.set(1, now_us);
    dst.last_fragment_number.set(1, now_us);
    dst.text_length.set(static_cast<int32_t>(payload.length), now_us);
    dst.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    dst.complete = true;
    dst.overflow = payload.length >= sizeof(dst.decoded_text);
    dst.first_seen_us = now_us;
    dst.last_update_us = now_us;
    commit_inmarsat_decoded_message(inmarsat, dst, now_us);
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
