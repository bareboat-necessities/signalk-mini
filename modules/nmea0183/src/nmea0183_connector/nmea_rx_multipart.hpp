#pragma once

// Included inside Nmea0183RxConnector private section.

template<typename Setting>
void set_source(Setting& setting, ship_data_model::SensorSource source) {
    if (source != ship_data_model::SensorSource::none) setting.value = source;
}

template<typename Record>
bool apply_nmea_text_record(const NmeaSentence& sentence, Record& record, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count > 0) nmea_copy_span(record.id, sizeof(record.id), sentence.field(0));
    if (sentence.field_count > 1) nmea_copy_span(record.code, sizeof(record.code), sentence.field(1));
    if (sentence.field_count > 2) nmea_copy_span(record.value, sizeof(record.value), sentence.field(2));
    if (sentence.field_count > 3) nmea_copy_span(record.text, sizeof(record.text), sentence.field(3));
    else if (sentence.field_count > 2) nmea_copy_span(record.text, sizeof(record.text), sentence.field(2));
    record.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(record.source, source);
    record.last_update_us = now_us;
    return true;
}

bool accepts_fragment_only_family(NmeaSentenceFamily family) const {
    return family == NmeaSentenceFamily::Ais ||
           family == NmeaSentenceFamily::NavTex ||
           family == NmeaSentenceFamily::SeaTalk ||
           family == NmeaSentenceFamily::Inmarsat;
}

char ais_radio_channel(const NmeaSentence& sentence) const {
    return sentence.field_count > 3 ? sentence.field(3)[0] : 0;
}

template<typename Multipart>
void reset_multipart_record(Multipart& record) {
    record.received_mask = 0;
    record.in_progress = false;
    record.complete = false;
    record.overflow = false;
    record.talker_id[0] = '\0';
    record.sentence_id[0] = '\0';
    record.message_id[0] = '\0';
    record.radio_channel = 0;
    record.text[0] = '\0';
    record.text_length = NmeaMessageStampedInt{};
    record.duplicate_fragment_count = NmeaMessageStampedInt{};
    record.bad_fragment_count = NmeaMessageStampedInt{};
}

template<typename Multipart>
void copy_multipart_identity(const NmeaSentence& sentence, Multipart& record) {
    nmea_copy_span(record.sentence_id, sizeof(record.sentence_id), sentence.sentence);
    nmea_copy_span(record.talker_id, sizeof(record.talker_id), sentence.talker);
    nmea_copy_span(record.message_id, sizeof(record.message_id), sentence.fragment.message_id);
    record.radio_channel = sentence.family == NmeaSentenceFamily::Ais ? ais_radio_channel(sentence) : 0;
}

template<typename Multipart>
bool multipart_record_matches(const NmeaSentence& sentence, const Multipart& record) const {
    if (!record.in_progress && !record.complete) return false;
    if (!nmea_span_equals(sentence.sentence, record.sentence_id)) return false;
    if (!nmea_span_equals(sentence.talker, record.talker_id)) return false;
    if (!nmea_span_equals(sentence.fragment.message_id, record.message_id)) return false;
    if (sentence.family == NmeaSentenceFamily::Ais && ais_radio_channel(sentence) != record.radio_channel) return false;
    return true;
}

void cleanup_ais_multipart_slots(uint64_t now_us, uint64_t timeout_us = NMEA_AIS_MULTIPART_STALE_TIMEOUT_US) {
    if (timeout_us == 0u) return;
    for (uint8_t i = 0; i < NMEA_AIS_MULTIPART_SLOT_COUNT; ++i) {
        auto& slot = state_.ais_messages[i];
        if (!slot.in_progress || slot.last_update_us == 0u) continue;
        if (now_us < slot.last_update_us || now_us - slot.last_update_us <= timeout_us) continue;
        reset_multipart_record(slot);
        state_.ais_multipart_stale_count.set(state_.ais_multipart_stale_count.value + 1, now_us);
    }
}

NmeaMultipartMessageRecord& select_ais_multipart_record(const NmeaSentence& sentence, uint64_t now_us, bool& matched_existing) {
    cleanup_ais_multipart_slots(now_us);
    matched_existing = false;
    for (uint8_t i = 0; i < NMEA_AIS_MULTIPART_SLOT_COUNT; ++i) {
        if (multipart_record_matches(sentence, state_.ais_messages[i])) {
            matched_existing = true;
            state_.active_ais_slot.set(static_cast<int32_t>(i), now_us);
            return state_.ais_messages[i];
        }
    }
    uint8_t slot = NMEA_AIS_MULTIPART_SLOT_COUNT;
    if (sentence.fragment.is_first()) {
        for (uint8_t i = 0; i < NMEA_AIS_MULTIPART_SLOT_COUNT; ++i) {
            if (!state_.ais_messages[i].in_progress) { slot = i; break; }
        }
    }
    if (slot == NMEA_AIS_MULTIPART_SLOT_COUNT) {
        uint64_t oldest = state_.ais_messages[0].last_update_us;
        slot = 0;
        for (uint8_t i = 1; i < NMEA_AIS_MULTIPART_SLOT_COUNT; ++i) {
            if (state_.ais_messages[i].last_update_us < oldest) { oldest = state_.ais_messages[i].last_update_us; slot = i; }
        }
        state_.ais_multipart_replacement_count.set(state_.ais_multipart_replacement_count.value + 1, now_us);
    }
    state_.active_ais_slot.set(static_cast<int32_t>(slot), now_us);
    return state_.ais_messages[slot];
}

void cleanup_navtex_multipart_slots(uint64_t now_us, uint64_t timeout_us = NMEA_NAVTEX_MULTIPART_STALE_TIMEOUT_US) {
    if (timeout_us == 0u) return;
    for (uint8_t i = 0; i < NMEA_NAVTEX_MULTIPART_SLOT_COUNT; ++i) {
        auto& slot = state_.navtex_messages[i];
        if (!slot.in_progress || slot.last_update_us == 0u) continue;
        if (now_us < slot.last_update_us || now_us - slot.last_update_us <= timeout_us) continue;
        reset_multipart_record(slot);
        state_.navtex_multipart_stale_count.set(state_.navtex_multipart_stale_count.value + 1, now_us);
    }
}

NmeaNavtexMultipartMessageRecord& select_navtex_multipart_record(const NmeaSentence& sentence, uint64_t now_us, bool& matched_existing) {
    cleanup_navtex_multipart_slots(now_us);
    matched_existing = false;
    for (uint8_t i = 0; i < NMEA_NAVTEX_MULTIPART_SLOT_COUNT; ++i) {
        if (multipart_record_matches(sentence, state_.navtex_messages[i])) {
            matched_existing = true;
            state_.active_navtex_slot.set(static_cast<int32_t>(i), now_us);
            return state_.navtex_messages[i];
        }
    }
    uint8_t slot = NMEA_NAVTEX_MULTIPART_SLOT_COUNT;
    if (sentence.fragment.is_first()) {
        for (uint8_t i = 0; i < NMEA_NAVTEX_MULTIPART_SLOT_COUNT; ++i) {
            if (!state_.navtex_messages[i].in_progress) { slot = i; break; }
        }
    }
    if (slot == NMEA_NAVTEX_MULTIPART_SLOT_COUNT) {
        uint64_t oldest = state_.navtex_messages[0].last_update_us;
        slot = 0;
        for (uint8_t i = 1; i < NMEA_NAVTEX_MULTIPART_SLOT_COUNT; ++i) {
            if (state_.navtex_messages[i].last_update_us < oldest) { oldest = state_.navtex_messages[i].last_update_us; slot = i; }
        }
        state_.navtex_multipart_replacement_count.set(state_.navtex_multipart_replacement_count.value + 1, now_us);
    }
    state_.active_navtex_slot.set(static_cast<int32_t>(slot), now_us);
    return state_.navtex_messages[slot];
}

template<typename Multipart>
void mark_bad_multipart_fragment(const NmeaSentence& sentence, Multipart& record, uint64_t now_us, ship_data_model::SensorSource source) {
    reset_multipart_record(record);
    copy_multipart_identity(sentence, record);
    record.total_fragments.set(static_cast<int32_t>(sentence.fragment.total), now_us);
    record.last_fragment_number.set(static_cast<int32_t>(sentence.fragment.number), now_us);
    record.bad_fragment_count.set(record.bad_fragment_count.value + 1, now_us);
    set_source(record.source, source);
    record.last_update_us = now_us;
}

template<typename Multipart>
void update_multipart_record(const NmeaSentence& sentence, Multipart& record, uint64_t now_us, ship_data_model::SensorSource source, bool matched_existing = true) {
    if (!sentence.fragment.is_fragmented) return;
    if (sentence.fragment.total == 0 || sentence.fragment.number == 0 || sentence.fragment.number > sentence.fragment.total || sentence.fragment.number > 16) {
        mark_bad_multipart_fragment(sentence, record, now_us, source);
        return;
    }
    if (!sentence.fragment.is_first() && !matched_existing) {
        mark_bad_multipart_fragment(sentence, record, now_us, source);
        return;
    }
    if (sentence.fragment.is_first() || !record.in_progress || record.complete) {
        reset_multipart_record(record);
        copy_multipart_identity(sentence, record);
        record.in_progress = true;
    }
    record.total_fragments.set(static_cast<int32_t>(sentence.fragment.total), now_us);
    record.last_fragment_number.set(static_cast<int32_t>(sentence.fragment.number), now_us);
    const uint16_t bit = static_cast<uint16_t>(1u << (sentence.fragment.number - 1u));
    if ((record.received_mask & bit) != 0u) {
        record.duplicate_fragment_count.set(record.duplicate_fragment_count.value + 1, now_us);
        set_source(record.source, source);
        record.last_update_us = now_us;
        return;
    }
    record.received_mask = static_cast<uint16_t>(record.received_mask | bit);
    size_t current = strlen(record.text);
    const size_t capacity = sizeof(record.text);
    size_t append = sentence.fragment.payload.length;
    if (current + append + 1u > capacity) { append = current < capacity ? capacity - current - 1u : 0u; record.overflow = true; }
    if (append > 0u && sentence.fragment.payload.data) { memcpy(record.text + current, sentence.fragment.payload.data, append); record.text[current + append] = '\0'; current += append; }
    record.text_length.set(static_cast<int32_t>(current), now_us);
    const uint16_t expected = sentence.fragment.total >= 16 ? 0xffffu : static_cast<uint16_t>((1u << sentence.fragment.total) - 1u);
    record.complete = sentence.fragment.total > 0 && (record.received_mask & expected) == expected;
    if (record.complete) record.in_progress = false;
    set_source(record.source, source);
    record.last_update_us = now_us;
}

void update_ais_multipart_message_state(const NmeaSentence& sentence, uint64_t now_us, ship_data_model::SensorSource source) {
    bool matched_existing = false;
    auto& selected = select_ais_multipart_record(sentence, now_us, matched_existing);
    update_multipart_record(sentence, selected, now_us, source, matched_existing);
    state_.ais_message = selected;
}

void update_navtex_multipart_message_state(const NmeaSentence& sentence, uint64_t now_us, ship_data_model::SensorSource source) {
    bool matched_existing = false;
    auto& selected = select_navtex_multipart_record(sentence, now_us, matched_existing);
    update_multipart_record(sentence, selected, now_us, source, matched_existing);
    state_.navtex_message = selected;
}

void update_multipart_message_state(const NmeaSentence& sentence, uint64_t now_us, ship_data_model::SensorSource source) {
    if (!sentence.fragment.is_fragmented) return;
    if (sentence_is(sentence, "TXT")) update_multipart_record(sentence, state_.text_message, now_us, source);
    else if (sentence.family == NmeaSentenceFamily::Ais) update_ais_multipart_message_state(sentence, now_us, source);
    else if (sentence.family == NmeaSentenceFamily::NavTex) update_navtex_multipart_message_state(sentence, now_us, source);
    else if (sentence.family == NmeaSentenceFamily::Dsc) update_multipart_record(sentence, dsc_state_.multipart, now_us, source);
    else if (sentence.family == NmeaSentenceFamily::SeaTalk) update_multipart_record(sentence, state_.seatalk_message, now_us, source);
    else if (sentence.family == NmeaSentenceFamily::Inmarsat) update_multipart_record(sentence, state_.inmarsat_message, now_us, source);
    else update_multipart_record(sentence, state_.generic_multipart_message, now_us, source);
}
