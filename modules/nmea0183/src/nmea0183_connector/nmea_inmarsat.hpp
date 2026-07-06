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

static const char* inmarsat_ocean_region_label(char code) {
    switch (code) {
    case '0': return "AOR-W";
    case '1': return "AOR-E";
    case '2': return "POR";
    case '3': return "IOR";
    case '8': return "Unknown";
    case '9': return "All ocean regions";
    default: return "Reserved";
    }
}

static const char* inmarsat_priority_label(char code) {
    switch (code) {
    case '1': return "Safety";
    case '2': return "Urgency";
    case '3': return "Distress";
    case '9': return "Unknown";
    default: return "Reserved";
    }
}

static const char* inmarsat_presentation_label(const char* code) {
    return code && strcmp(code, "00") == 0 ? "IA5" : "Unknown";
}

static const char* inmarsat_service_label(const char* code) {
    if (!code) return "Unknown";
    if (strcmp(code, "00") == 0) return "All ships";
    if (strcmp(code, "04") == 0) return "Warning to rectangular area";
    if (strcmp(code, "13") == 0) return "Coastal warning";
    if (strcmp(code, "14") == 0) return "Shore-to-ship distress alert to circular area";
    if (strcmp(code, "24") == 0) return "Warning to circular area";
    if (strcmp(code, "31") == 0) return "NAVAREA/METAREA warning or forecast";
    if (strcmp(code, "34") == 0) return "SAR coordination to rectangular area";
    if (strcmp(code, "44") == 0) return "SAR coordination to circular area";
    return "Unknown";
}

static const char* inmarsat_coastal_subject_label(char code) {
    switch (code) {
    case 'A': return "Navigational warnings";
    case 'B': return "Meteorological warnings";
    case 'C': return "Ice reports";
    case 'D': return "Search and rescue information and piracy warnings";
    case 'E': return "Meteorological forecasts";
    case 'F': return "Pilot service messages";
    case 'G': return "AIS";
    case 'H': return "LORAN messages";
    case 'I': return "Not used";
    case 'J': return "SATNAV messages";
    case 'K': return "Other electronic navaid messages";
    case 'L': return "Other navigational warnings";
    case 'V':
    case 'W':
    case 'X':
    case 'Y': return "Special services allocation";
    case 'Z': return "No messages on hand";
    default: return "Unknown";
    }
}

static bool inmarsat_service_is_mandatory(const char* service_code, char subject) {
    if (!service_code) return false;
    if (strcmp(service_code, "00") == 0 || strcmp(service_code, "04") == 0 ||
        strcmp(service_code, "14") == 0 || strcmp(service_code, "24") == 0 ||
        strcmp(service_code, "31") == 0 || strcmp(service_code, "34") == 0 ||
        strcmp(service_code, "44") == 0) {
        return true;
    }
    if (strcmp(service_code, "13") == 0) return subject == 'A' || subject == 'B' || subject == 'D';
    return false;
}

static bool inmarsat_span_has_unknown_char(NmeaSpan span) {
    for (uint8_t i = 0; i < span.length; ++i) {
        if (span[i] == '_') return true;
    }
    return false;
}

static bool inmarsat_cstr_has_unknown_char(const char* text) {
    if (!text) return false;
    while (*text) {
        if (*text++ == '_') return true;
    }
    return false;
}

static bool inmarsat_parse_safetynet_lat_lon(NmeaSpan magnitude, NmeaSpan hemisphere, float& out_deg) {
    if (magnitude.empty() || hemisphere.empty()) return false;
    const char h = hemisphere[0];
    const bool lon = h == 'E' || h == 'W';
    const bool lat = h == 'N' || h == 'S';
    if (!lat && !lon) return false;
    const uint8_t deg_digits = lon ? 3 : 2;
    if (magnitude.length <= deg_digits) {
        int32_t deg = 0;
        if (!parse_int32(magnitude, deg)) return false;
        out_deg = static_cast<float>(deg);
        if (h == 'S' || h == 'W') out_deg = -out_deg;
        return true;
    }
    return parse_lat_lon(magnitude, hemisphere, out_deg);
}

void reset_inmarsat_safetynet_body(NmeaSafetyNetAssemblyRecord& rec) const {
    rec.body_in_progress = false;
    rec.body_complete = false;
    rec.overflow = false;
    rec.body_has_unknown_char = false;
    rec.total_body_sentences = 0;
    rec.received_body_sentence_count = 0;
    rec.last_body_sentence_number = 0;
    rec.sequential_message_id[0] = '\0';
    rec.received_mask = 0;
    rec.body_text[0] = '\0';
    rec.body_text_length = 0;
}

void reset_inmarsat_safetynet_record(NmeaSafetyNetAssemblyRecord& rec) const {
    rec = NmeaSafetyNetAssemblyRecord{};
}

bool append_inmarsat_safetynet_char(NmeaSafetyNetAssemblyRecord& rec, char c) const {
    const size_t capacity = sizeof(rec.body_text);
    if (rec.body_text_length + 1 >= static_cast<int32_t>(capacity)) {
        rec.overflow = true;
        return false;
    }
    rec.body_text[rec.body_text_length++] = c;
    rec.body_text[rec.body_text_length] = '\0';
    return true;
}

void append_inmarsat_safetynet_body(NmeaSafetyNetAssemblyRecord& rec, NmeaSpan body) const {
    for (uint8_t i = 0; i < body.length; ++i) {
        const char c = body[i];
        if (c == '_') rec.body_has_unknown_char = true;
        if (c == '^' && i + 2 < body.length) {
            const uint8_t hi = from_hex(body[i + 1]);
            const uint8_t lo = from_hex(body[i + 2]);
            if (hi <= 0x0f && lo <= 0x0f) {
                const char decoded = static_cast<char>((hi << 4) | lo);
                if (decoded != '\0') append_inmarsat_safetynet_char(rec, decoded);
                i = static_cast<uint8_t>(i + 2);
                continue;
            }
        }
        append_inmarsat_safetynet_char(rec, c);
    }
}

template<typename Message>
bool inmarsat_same_notification_message(const Message& a, const Message& b) const {
    if (a.les_sequence_number[0] != '\0' && b.les_sequence_number[0] != '\0') {
        return strcmp(a.les_sequence_number, b.les_sequence_number) == 0 &&
               strcmp(a.les_id, b.les_id) == 0 &&
               strcmp(a.service_code, b.service_code) == 0;
    }
    return strcmp(a.message_id, b.message_id) == 0 &&
           strcmp(a.terminal_id, b.terminal_id) == 0 &&
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

bool parse_inmarsat_safetynet_common_header(const NmeaSentence& sentence,
                                            NmeaSafetyNetAssemblyRecord& rec,
                                            uint64_t now_us,
                                            ship_data_model::SensorSource source,
                                            const char* address_kind) {
    if (sentence.field_count < 13) { last_error_ = "short SafetyNET header"; return false; }

    reset_inmarsat_safetynet_record(rec);
    rec.received_year = -1;
    rec.received_month = -1;
    rec.received_day = -1;
    rec.received_hour = -1;
    rec.received_minute = -1;
    rec.header_received = true;
    rec.first_seen_us = now_us;
    rec.last_update_us = now_us;
    rec.source.value = source;
    nmea_copy_span(rec.talker_id, sizeof(rec.talker_id), sentence.talker);
    rec.msi_status = sentence.field(0)[0];
    nmea_copy_span(rec.message_id, sizeof(rec.message_id), sentence.field(1));
    nmea_copy_span(rec.les_sequence_number, sizeof(rec.les_sequence_number), sentence.field(2));
    nmea_copy_span(rec.les_id, sizeof(rec.les_id), sentence.field(3));
    rec.ocean_region_code = sentence.field(4)[0];
    rec.priority_code = sentence.field(5)[0];
    nmea_copy_span(rec.service_code, sizeof(rec.service_code), sentence.field(6));
    nmea_copy_span(rec.presentation_code, sizeof(rec.presentation_code), sentence.field(7));
    parse_int32(sentence.field(8), rec.received_year);
    parse_int32(sentence.field(9), rec.received_month);
    parse_int32(sentence.field(10), rec.received_day);
    parse_int32(sentence.field(11), rec.received_hour);
    parse_int32(sentence.field(12), rec.received_minute);
    nmea_copy_cstr(rec.address_kind, sizeof(rec.address_kind), address_kind);
    return rec.message_id[0] != '\0';
}

bool apply_safetynet_sm1_header(const NmeaSentence& sentence,
                                NmeaSafetyNetAssemblyRecord& rec,
                                uint64_t now_us,
                                ship_data_model::SensorSource source) {
    if (sentence.field_count < 14) { last_error_ = "short SM1"; return false; }
    if (!parse_inmarsat_safetynet_common_header(sentence, rec, now_us, source, "navarea_metarea")) return false;
    int32_t area = 0;
    if (parse_int32(sentence.field(13), area)) {
        rec.navarea_metarea_code = area;
        rec.has_navarea_metarea_code = true;
        if (strcmp(rec.service_code, "00") == 0) nmea_copy_cstr(rec.address_kind, sizeof(rec.address_kind), "all_ships");
    }
    return true;
}

bool apply_safetynet_sm2_header(const NmeaSentence& sentence,
                                NmeaSafetyNetAssemblyRecord& rec,
                                uint64_t now_us,
                                ship_data_model::SensorSource source) {
    if (sentence.field_count < 16) { last_error_ = "short SM2"; return false; }
    if (!parse_inmarsat_safetynet_common_header(sentence, rec, now_us, source, "coastal_warning_area")) return false;
    int32_t area = 0;
    if (parse_int32(sentence.field(13), area)) {
        rec.coastal_warning_navarea_metarea = area;
        rec.has_coastal_warning_navarea_metarea = true;
    }
    rec.coastal_warning_area = sentence.field(14)[0];
    rec.coastal_warning_subject = sentence.field(15)[0];
    return true;
}

bool apply_safetynet_sm3_header(const NmeaSentence& sentence,
                                NmeaSafetyNetAssemblyRecord& rec,
                                uint64_t now_us,
                                ship_data_model::SensorSource source) {
    if (sentence.field_count < 18) { last_error_ = "short SM3"; return false; }
    if (!parse_inmarsat_safetynet_common_header(sentence, rec, now_us, source, "circular_area")) return false;
    float value = 0.0f;
    if (inmarsat_parse_safetynet_lat_lon(sentence.field(13), sentence.field(14), value)) {
        rec.circular_center_lat_deg = value;
        rec.has_circular_center = true;
    }
    if (inmarsat_parse_safetynet_lat_lon(sentence.field(15), sentence.field(16), value)) {
        rec.circular_center_lon_deg = value;
        rec.has_circular_center = rec.has_circular_center && true;
    } else {
        rec.has_circular_center = false;
    }
    int32_t radius = 0;
    if (parse_int32(sentence.field(17), radius)) {
        rec.circular_radius_nmi = radius;
        rec.has_circular_radius = true;
    }
    return true;
}

bool apply_safetynet_sm4_header(const NmeaSentence& sentence,
                                NmeaSafetyNetAssemblyRecord& rec,
                                uint64_t now_us,
                                ship_data_model::SensorSource source) {
    if (sentence.field_count < 19) { last_error_ = "short SM4"; return false; }
    if (!parse_inmarsat_safetynet_common_header(sentence, rec, now_us, source, "rectangular_area")) return false;
    float value = 0.0f;
    if (inmarsat_parse_safetynet_lat_lon(sentence.field(13), sentence.field(14), value)) {
        rec.rectangle_sw_lat_deg = value;
        rec.has_rectangle_sw = true;
    }
    if (inmarsat_parse_safetynet_lat_lon(sentence.field(15), sentence.field(16), value)) {
        rec.rectangle_sw_lon_deg = value;
        rec.has_rectangle_sw = rec.has_rectangle_sw && true;
    } else {
        rec.has_rectangle_sw = false;
    }
    int32_t extent = 0;
    if (parse_int32(sentence.field(17), extent)) {
        rec.rectangle_extent_lat_deg = extent;
        rec.has_rectangle_extent = true;
    }
    if (parse_int32(sentence.field(18), extent)) {
        rec.rectangle_extent_lon_deg = extent;
        rec.has_rectangle_extent = rec.has_rectangle_extent && true;
    } else {
        rec.has_rectangle_extent = false;
    }
    return true;
}

template<typename Model>
bool commit_inmarsat_safetynet_message(Model& model, uint64_t now_us) {
    auto& rec = state_.inmarsat_safetynet;
    auto& safetynet = model.notifications.inmarsat.safetynet;
    auto& dst = safetynet.latest_message;
    dst = ship_data_model::InmarsatSafetyNetMessageData<Real>{};

    set_source(dst.source, rec.source.value);
    nmea_copy_cstr(dst.message_id, sizeof(dst.message_id), rec.message_id);
    nmea_copy_cstr(dst.terminal_id, sizeof(dst.terminal_id), rec.talker_id);
    dst.msi_status = rec.msi_status;
    nmea_copy_cstr(dst.les_sequence_number, sizeof(dst.les_sequence_number), rec.les_sequence_number);
    nmea_copy_cstr(dst.les_id, sizeof(dst.les_id), rec.les_id);
    dst.ocean_region_code = rec.ocean_region_code;
    nmea_copy_cstr(dst.ocean_region_label, sizeof(dst.ocean_region_label), inmarsat_ocean_region_label(rec.ocean_region_code));
    dst.priority_code = rec.priority_code;
    nmea_copy_cstr(dst.priority_label, sizeof(dst.priority_label), inmarsat_priority_label(rec.priority_code));
    nmea_copy_cstr(dst.service_code, sizeof(dst.service_code), rec.service_code);
    nmea_copy_cstr(dst.service_label, sizeof(dst.service_label), inmarsat_service_label(rec.service_code));
    nmea_copy_cstr(dst.presentation_code, sizeof(dst.presentation_code), rec.presentation_code);
    nmea_copy_cstr(dst.presentation_label, sizeof(dst.presentation_label), inmarsat_presentation_label(rec.presentation_code));
    if (rec.received_year > 0) dst.received_year.set(rec.received_year, now_us);
    if (rec.received_month > 0) dst.received_month.set(rec.received_month, now_us);
    if (rec.received_day > 0) dst.received_day.set(rec.received_day, now_us);
    if (rec.received_hour >= 0) dst.received_hour.set(rec.received_hour, now_us);
    if (rec.received_minute >= 0) dst.received_minute.set(rec.received_minute, now_us);
    nmea_copy_cstr(dst.address_kind, sizeof(dst.address_kind), rec.address_kind);
    if (rec.has_navarea_metarea_code) dst.navarea_metarea_code.set(rec.navarea_metarea_code, now_us);
    if (rec.has_coastal_warning_navarea_metarea) dst.coastal_warning_navarea_metarea.set(rec.coastal_warning_navarea_metarea, now_us);
    dst.coastal_warning_area = rec.coastal_warning_area;
    dst.coastal_warning_subject = rec.coastal_warning_subject;
    nmea_copy_cstr(dst.coastal_warning_subject_label, sizeof(dst.coastal_warning_subject_label), inmarsat_coastal_subject_label(rec.coastal_warning_subject));
    dst.coastal_warning_subject_mandatory = rec.coastal_warning_subject == 'A' || rec.coastal_warning_subject == 'B' || rec.coastal_warning_subject == 'D';
    if (rec.has_circular_center) {
        dst.circular_center_lat_deg.set(static_cast<Real>(rec.circular_center_lat_deg), now_us);
        dst.circular_center_lon_deg.set(static_cast<Real>(rec.circular_center_lon_deg), now_us);
    }
    if (rec.has_circular_radius) dst.circular_radius_nmi.set(rec.circular_radius_nmi, now_us);
    if (rec.has_rectangle_sw) {
        dst.rectangle_sw_lat_deg.set(static_cast<Real>(rec.rectangle_sw_lat_deg), now_us);
        dst.rectangle_sw_lon_deg.set(static_cast<Real>(rec.rectangle_sw_lon_deg), now_us);
    }
    if (rec.has_rectangle_extent) {
        dst.rectangle_extent_lat_deg.set(rec.rectangle_extent_lat_deg, now_us);
        dst.rectangle_extent_lon_deg.set(rec.rectangle_extent_lon_deg, now_us);
    }
    nmea_copy_cstr(dst.sequential_message_id, sizeof(dst.sequential_message_id), rec.sequential_message_id);
    nmea_copy_cstr(dst.message_text, sizeof(dst.message_text), rec.body_text);
    dst.total_fragments.set(rec.total_body_sentences, now_us);
    dst.fragment_number.set(rec.last_body_sentence_number, now_us);
    dst.text_length.set(rec.body_text_length, now_us);
    dst.field_count.set(rec.received_body_sentence_count, now_us);
    dst.body_complete = rec.body_complete;
    dst.complete = rec.body_complete && rec.msi_status == 'A';
    dst.overflow = rec.overflow;
    dst.body_has_unknown_char = rec.body_has_unknown_char || inmarsat_cstr_has_unknown_char(rec.body_text);
    dst.requires_alarm = rec.priority_code == '2' || rec.priority_code == '3';
    dst.mandatory_reception = inmarsat_service_is_mandatory(rec.service_code, rec.coastal_warning_subject);
    dst.suppressible_by_receiver = !dst.mandatory_reception;
    dst.first_seen_us = rec.first_seen_us != 0 ? rec.first_seen_us : now_us;
    dst.last_update_us = now_us;
    commit_inmarsat_notification_message(safetynet, dst, now_us);
    return true;
}

template<typename Model>
bool apply_safetynet_header(const NmeaSentence& sentence,
                            Model& model,
                            uint64_t now_us,
                            ship_data_model::SensorSource source,
                            InmarsatConnectorSentenceKind kind) {
    (void)model;
    auto& rec = state_.inmarsat_safetynet;
    bool ok = false;
    if (kind == InmarsatConnectorSentenceKind::sm1) ok = apply_safetynet_sm1_header(sentence, rec, now_us, source);
    else if (kind == InmarsatConnectorSentenceKind::sm2) ok = apply_safetynet_sm2_header(sentence, rec, now_us, source);
    else if (kind == InmarsatConnectorSentenceKind::sm3) ok = apply_safetynet_sm3_header(sentence, rec, now_us, source);
    else if (kind == InmarsatConnectorSentenceKind::sm4) ok = apply_safetynet_sm4_header(sentence, rec, now_us, source);
    if (!ok) return false;
    reset_inmarsat_safetynet_body(rec);
    return true;
}

template<typename Model>
bool apply_safetynet_smb(const NmeaSentence& sentence,
                         Model& model,
                         uint64_t now_us,
                         ship_data_model::SensorSource source) {
    auto& rec = state_.inmarsat_safetynet;
    auto& safetynet = model.notifications.inmarsat.safetynet;
    if (sentence.field_count < 5) { last_error_ = "short SMB"; return false; }

    int32_t total = 0;
    if (!parse_int32(sentence.field(0), total) || total <= 0) { last_error_ = "bad SMB total"; return false; }
    int32_t number = 0;
    if (!parse_int32(sentence.field(1), number)) {
        if (total == 1 && sentence.field(1).empty()) number = 1;
        else { last_error_ = "bad SMB number"; return false; }
    }
    if (number <= 0 || number > total) { last_error_ = "bad SMB number"; return false; }

    char body_message_id[16] = {0};
    nmea_copy_span(body_message_id, sizeof(body_message_id), sentence.field(3));
    if (!rec.header_received || strcmp(rec.message_id, body_message_id) != 0) {
        safetynet.bad_fragment_count.set(safetynet.bad_fragment_count.value + 1, now_us);
        return true;
    }

    if (rec.total_body_sentences == 0) rec.total_body_sentences = total;
    if (rec.total_body_sentences != total) {
        rec.overflow = true;
        safetynet.bad_fragment_count.set(safetynet.bad_fragment_count.value + 1, now_us);
        return true;
    }

    rec.source.value = source;
    rec.body_in_progress = true;
    rec.last_update_us = now_us;
    if (rec.first_seen_us == 0) rec.first_seen_us = now_us;
    nmea_copy_span(rec.sequential_message_id, sizeof(rec.sequential_message_id), sentence.field(2));

    if (number <= 64) {
        const uint64_t bit = 1ULL << static_cast<unsigned>(number - 1);
        if ((rec.received_mask & bit) != 0ULL) {
            return true;
        }
        rec.received_mask |= bit;
    } else {
        rec.overflow = true;
    }

    if (number != rec.last_body_sentence_number + 1) {
        rec.overflow = true;
    }

    append_inmarsat_safetynet_body(rec, sentence.field(4));
    ++rec.received_body_sentence_count;
    rec.last_body_sentence_number = number;
    if (inmarsat_span_has_unknown_char(sentence.field(4))) rec.body_has_unknown_char = true;

    if (total <= 64) {
        const uint64_t expected = total == 64 ? 0xffffffffffffffffULL : ((1ULL << static_cast<unsigned>(total)) - 1ULL);
        rec.body_complete = (rec.received_mask & expected) == expected;
    } else {
        rec.body_complete = rec.received_body_sentence_count >= total;
    }
    if (rec.body_complete) {
        rec.body_in_progress = false;
        return commit_inmarsat_safetynet_message(model, now_us);
    }
    return true;
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
    dst.body_complete = assembled.complete;
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
    if (sentence.field_count > 1 && !sentence.field(1).empty()) {
        dst.msi_status = sentence.field(1)[0];
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
    dst.body_complete = true;
    dst.complete = true;
    dst.overflow = payload.length >= sizeof(dst.message_text);
    dst.body_has_unknown_char = inmarsat_span_has_unknown_char(payload);
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
    const InmarsatConnectorSentenceKind kind = inmarsat_sentence_kind_from_sentence(sentence);

    if (kind == InmarsatConnectorSentenceKind::sm1 || kind == InmarsatConnectorSentenceKind::sm2 ||
        kind == InmarsatConnectorSentenceKind::sm3 || kind == InmarsatConnectorSentenceKind::sm4) {
        return apply_safetynet_header(sentence, model, now_us, source, kind);
    }
    if (kind == InmarsatConnectorSentenceKind::smb) {
        return apply_safetynet_smb(sentence, model, now_us, source);
    }

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
