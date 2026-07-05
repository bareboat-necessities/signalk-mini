#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_hbt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) { last_error_ = "short HBT"; return false; }
    auto& rec = model.notifications.alerts.heartbeat;
    float interval = 0.0f;
    rec.status = sentence.field(0)[0];
    if (parse_real(sentence.field(1), interval)) rec.interval_s.set(static_cast<Real>(interval), now_us);
    if (sentence.field_count > 2) nmea_copy_span(rec.sequential_message_id, sizeof(rec.sequential_message_id), sentence.field(2));
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_hdt(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float heading_deg = 0.0f;
    if (!parse_real(sentence.field(0), heading_deg)) { last_error_ = "bad HDT"; return false; }
    const Real value = static_cast<Real>(wrap_360_deg(heading_deg));
    model.ins.imu.heading_deg.set(value, now_us);
    model.ins.imu.heading_true_deg.set(value, now_us);
    return true;
}

template<typename Model>
bool apply_hdm(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    float heading_deg = 0.0f;
    if (!parse_real(sentence.field(0), heading_deg)) { last_error_ = "bad HDM"; return false; }
    const Real value = static_cast<Real>(wrap_360_deg(heading_deg));
    model.ins.imu.heading_deg.set(value, now_us);
    model.ins.imu.heading_magnetic_deg.set(value, now_us);
    return true;
}

template<typename Model>
bool apply_hdg(const NmeaSentence& sentence, Model& model, uint64_t now_us) {
    if (!apply_hdm(sentence, model, now_us)) { last_error_ = "bad HDG"; return false; }
    float value = 0.0f;
    if (sentence.field_count >= 3 && parse_east_west_signed(sentence.field(1), sentence.field(2), value)) model.ins.imu.magnetic_deviation_deg.set(static_cast<Real>(value), now_us);
    if (sentence.field_count >= 5 && parse_east_west_signed(sentence.field(3), sentence.field(4), value)) {
        model.ins.imu.magnetic_variation_deg.set(static_cast<Real>(value), now_us);
        model.gnss.fix.declination_deg.set(static_cast<Real>(value), now_us);
    }
    return true;
}

template<typename Model>
bool apply_hfb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short HFB"; return false; }
    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.sea.trawl_headrope_to_footrope_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.sea.trawl_headrope_to_bottom_m.set(static_cast<Real>(value), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_hsc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short HSC"; return false; }
    float heading_deg = 0.0f;
    if (sentence.field(1)[0] == 'T' && parse_real(sentence.field(0), heading_deg)) {
        model.route.heading_steering_command.heading_true_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
        model.route.apb.heading_to_steer_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), heading_deg)) model.route.heading_steering_command.heading_magnetic_deg.set(static_cast<Real>(wrap_360_deg(heading_deg)), now_us);
    set_source(model.route.heading_steering_command.source, source);
    model.route.heading_steering_command.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_its(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) { last_error_ = "short ITS"; return false; }
    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.nav.legacy_timing.gri_us_div_10.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.nav.legacy_timing.master_relative_snr_db.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.nav.legacy_timing.master_relative_ecd.set(static_cast<Real>(value), now_us);
    for (uint8_t index = 0; index < 5; ++index) {
        const uint8_t base = static_cast<uint8_t>(3 + index * 2);
        if (parse_real(sentence.field(base), value)) model.nav.legacy_timing.delta_us[index].set(static_cast<Real>(value), now_us);
        model.nav.legacy_timing.delta_status[index] = sentence.field(base + 1)[0];
    }
    set_source(model.nav.legacy_timing.source, source);
    model.nav.legacy_timing.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_lwy(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float leeway_deg = 0.0f;
    if (sentence.field_count < 2 || sentence.field(0)[0] != 'A' || !parse_real(sentence.field(1), leeway_deg)) { last_error_ = "bad LWY"; return false; }
    model.sea.leeway_deg.set(static_cast<Real>(leeway_deg), now_us);
    set_source(model.sea.leeway_source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 20) { last_error_ = "short MDA"; return false; }
    float value = 0.0f;
    if (sentence.field(1)[0] == 'I' && parse_real(sentence.field(0), value)) model.env.barometric_pressure_inhg.set(static_cast<Real>(value), now_us);
    if (sentence.field(3)[0] == 'B' && parse_real(sentence.field(2), value)) model.env.barometric_pressure_bar.set(static_cast<Real>(value), now_us);
    if (sentence.field(5)[0] == 'C' && parse_real(sentence.field(4), value)) model.env.air_temperature_c.set(static_cast<Real>(value), now_us);
    if (sentence.field(7)[0] == 'C' && parse_real(sentence.field(6), value)) model.sea.temperature_c.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.env.relative_humidity_percent.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.env.absolute_humidity_percent.set(static_cast<Real>(value), now_us);
    if (sentence.field(11)[0] == 'C' && parse_real(sentence.field(10), value)) model.env.dew_point_c.set(static_cast<Real>(value), now_us);
    if (sentence.field(13)[0] == 'T' && parse_real(sentence.field(12), value)) {
        const Real direction = static_cast<Real>(wrap_360_deg(value));
        model.wind.surface.direction_deg.set(direction, now_us);
        model.wind.truewind.direction_deg.set(direction, now_us);
    }
    if (sentence.field(15)[0] == 'M' && parse_real(sentence.field(14), value)) model.wind.surface.direction_magnetic_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_knots(sentence.field(16), sentence.field(17), value)) { model.wind.surface.speed_kn.set(static_cast<Real>(value), now_us); model.wind.truewind.speed_kn.set(static_cast<Real>(value), now_us); }
    if (sentence.field(19)[0] == 'M' && parse_real(sentence.field(18), value)) { model.wind.surface.speed_m_s.set(static_cast<Real>(value), now_us); model.wind.truewind.speed_m_s.set(static_cast<Real>(value), now_us); }
    set_source(model.env.source, source); set_source(model.sea.source, source); set_source(model.wind.surface.source, source); set_source(model.wind.truewind.source, source);
    model.env.last_update_us = now_us; model.sea.last_update_us = now_us; model.wind.surface.last_update_us = now_us; model.wind.truewind.last_update_us = now_us;
    return true;
}

template<typename Wind>
bool set_wind(Wind& wind, float angle_deg, float speed_kn, uint64_t now_us, ship_data_model::SensorSource source) {
    wind.direction_deg.set(static_cast<Real>(wrap_180_deg(angle_deg)), now_us);
    wind.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(wind.source, source);
    wind.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_msk(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) { last_error_ = "short MSK"; return false; }
    float value = 0.0f; int32_t bit_rate = 0;
    if (parse_real(sentence.field(0), value)) model.comm.beacon_control.frequency_khz.set(static_cast<Real>(value), now_us);
    model.comm.beacon_control.frequency_mode = sentence.field(1)[0];
    if (parse_int32(sentence.field(2), bit_rate)) model.comm.beacon_control.bit_rate_bps.set(bit_rate, now_us);
    model.comm.beacon_control.bit_rate_mode = sentence.field(3)[0];
    if (parse_real(sentence.field(4), value)) model.comm.beacon_control.status_frequency_khz.set(static_cast<Real>(value), now_us);
    set_source(model.comm.beacon_control.source, source);
    model.comm.beacon_control.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mss(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5) { last_error_ = "short MSS"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    if (parse_real(sentence.field(0), value)) model.comm.beacon_status.signal_strength_db_uv.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.comm.beacon_status.signal_to_noise_ratio_db.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.comm.beacon_status.beacon_frequency_khz.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(3), integer_value)) model.comm.beacon_status.beacon_bit_rate_bps.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.comm.beacon_status.status.set(integer_value, now_us);
    set_source(model.comm.beacon_status.source, source);
    model.comm.beacon_status.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mtw(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2 || sentence.field(1)[0] != 'C') { last_error_ = "bad MTW"; return false; }
    float temperature_c = 0.0f;
    if (!parse_real(sentence.field(0), temperature_c)) { last_error_ = "bad MTW"; return false; }
    model.sea.temperature_c.set(static_cast<Real>(temperature_c), now_us);
    set_source(model.sea.source, source);
    model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mwd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float direction_deg = 0.0f; float speed_kn = 0.0f;
    if (sentence.field_count < 7 || !parse_real(sentence.field(0), direction_deg) || !parse_knots(sentence.field(4), sentence.field(5), speed_kn)) { last_error_ = "bad MWD"; return false; }
    model.wind.truewind.direction_deg.set(static_cast<Real>(wrap_180_deg(direction_deg)), now_us);
    model.wind.truewind.speed_kn.set(static_cast<Real>(speed_kn), now_us);
    set_source(model.wind.truewind.source, source);
    model.wind.truewind.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_mwv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 5 || sentence.field(4)[0] != 'A') { last_error_ = "invalid MWV"; return false; }
    float angle_deg = 0.0f; float speed_kn = 0.0f;
    if (!parse_real(sentence.field(0), angle_deg) || !parse_knots(sentence.field(2), sentence.field(3), speed_kn)) { last_error_ = "bad MWV"; return false; }
    if (sentence.field(1)[0] == 'T') return set_wind(model.wind.truewind, angle_deg, speed_kn, now_us, source);
    return set_wind(model.wind.apparent, angle_deg, speed_kn, now_us, source);
}

bool navtex_text_has_eom(const char* text) const { return text && strstr(text, "NNNN") != nullptr; }
bool navtex_is_id_char(char c) const { return (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'); }

int32_t navtex_subject_category(char subject) const {
    switch (subject) {
    case 'A': return 1; case 'B': return 2; case 'C': return 3; case 'D': return 4; case 'E': return 5; case 'F': return 6; case 'G': return 7; case 'H': return 8; case 'J': return 9; case 'K': return 10; case 'L': return 11; case 'V': case 'W': case 'X': case 'Y': return 20; case 'Z': return 30; default: return 0;
    }
}

const char* navtex_subject_label(char subject) const {
    switch (subject) {
    case 'A': return "navigation"; case 'B': return "meteorology"; case 'C': return "ice"; case 'D': return "search_rescue"; case 'E': return "forecast"; case 'F': return "pilot_service"; case 'G': return "ais"; case 'H': return "loran"; case 'J': return "satnav"; case 'K': return "other_enav"; case 'L': return "navigation_extra"; case 'V': case 'W': case 'X': case 'Y': return "special_service"; case 'Z': return "no_messages"; default: return "unknown";
    }
}

uint32_t navtex_letter_mask_bits(const char* text, int32_t& count) const {
    uint32_t bits = 0; count = 0; if (!text) return bits;
    for (const char* p = text; *p; ++p) { char c = *p; if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A'); if (c < 'A' || c > 'Z') continue; const uint32_t bit = static_cast<uint32_t>(1u << (c - 'A')); if ((bits & bit) == 0u) { bits |= bit; ++count; } }
    return bits;
}

void parse_navtex_message_id(const char* text, char out[8], char& transmitter_id, char& subject_indicator, int32_t& serial_number, bool& has_serial) const {
    out[0] = '\0'; transmitter_id = 0; subject_indicator = 0; serial_number = 0; has_serial = false; if (!text) return;
    const char* p = strstr(text, "ZCZC"); if (p) p += 4; else p = text; while (*p == ' ') ++p;
    if (!navtex_is_id_char(p[0]) || !navtex_is_id_char(p[1]) || p[2] < '0' || p[2] > '9' || p[3] < '0' || p[3] > '9') return;
    out[0] = p[0]; out[1] = p[1]; out[2] = p[2]; out[3] = p[3]; out[4] = '\0'; transmitter_id = p[0]; subject_indicator = p[1]; serial_number = static_cast<int32_t>((p[2] - '0') * 10 + (p[3] - '0')); has_serial = true;
}

void navtex_copy_body_text(char* out, size_t out_size, const char* text) const {
    if (!out || out_size == 0u) return; out[0] = '\0'; if (!text) return;
    const char* begin = strstr(text, "ZCZC");
    if (begin) { begin += 4; while (*begin == ' ') ++begin; if (navtex_is_id_char(begin[0]) && navtex_is_id_char(begin[1]) && begin[2] >= '0' && begin[2] <= '9' && begin[3] >= '0' && begin[3] <= '9') begin += 4; } else begin = text;
    while (*begin == ' ') ++begin;
    const char* end = strstr(begin, "NNNN"); if (!end) end = begin + strlen(begin); while (end > begin && (end[-1] == ' ' || end[-1] == '\r' || end[-1] == '\n')) --end;
    nmea_copy_span(out, out_size, NmeaSpan(begin, static_cast<size_t>(end - begin)));
}

template<typename Message>
bool navtex_same_message_id(const Message& a, const Message& b) const {
    if (a.navtex_message_id[0] && b.navtex_message_id[0]) return strcmp(a.navtex_message_id, b.navtex_message_id) == 0;
    if (a.transmitter_id && b.transmitter_id && a.subject_indicator && b.subject_indicator && a.serial_number.valid && b.serial_number.valid) return a.transmitter_id == b.transmitter_id && a.subject_indicator == b.subject_indicator && a.serial_number.value == b.serial_number.value;
    return false;
}

template<typename NavtexData, typename ReceivedMessage>
void navtex_store_received_message(NavtexData& navtex, ReceivedMessage& received, uint64_t now_us) {
    auto& history = navtex.history; const uint8_t capacity = ship_data_model::NAVTEX_MESSAGE_HISTORY_CAPACITY;
    for (uint8_t i = 0; i < capacity; ++i) {
        auto& slot = history.messages[i];
        if (slot.first_seen_us != 0 && navtex_same_message_id(slot, received)) {
            const int32_t repeat = slot.repeat_count.valid ? slot.repeat_count.value + 1 : 2; const uint64_t first_seen = slot.first_seen_us; slot = received; slot.first_seen_us = first_seen; slot.repeat_count.set(repeat, now_us); slot.duplicate = true; slot.last_update_us = now_us; received = slot; const int32_t duplicate_count = history.duplicate_count.valid ? history.duplicate_count.value : 0; history.duplicate_count.set(duplicate_count + 1, now_us); return;
        }
    }
    int32_t next = history.next_index.valid ? history.next_index.value : 0; if (next < 0 || next >= static_cast<int32_t>(capacity)) next = 0; auto& slot = history.messages[static_cast<uint8_t>(next)]; const bool overwriting = slot.first_seen_us != 0; received.first_seen_us = now_us; received.repeat_count.set(1, now_us); received.duplicate = false; slot = received; const int32_t count = history.count.valid ? history.count.value : 0; if (count < static_cast<int32_t>(capacity)) history.count.set(count + 1, now_us); if (overwriting) { const int32_t overwrite_count = history.overwrite_count.valid ? history.overwrite_count.value : 0; history.overwrite_count.set(overwrite_count + 1, now_us); } history.next_index.set((next + 1) % static_cast<int32_t>(capacity), now_us);
}

template<typename Model>
bool apply_nrx(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short NRX"; return false; }
    auto& received = model.notifications.navtex.received; int32_t value = 0;
    if (parse_int32(sentence.field(0), value)) received.total_fragments.set(value, now_us);
    if (parse_int32(sentence.field(1), value)) received.fragment_number.set(value, now_us);
    nmea_copy_span(received.sentence_message_id, sizeof(received.sentence_message_id), sentence.field(2));
    const char* text = nullptr; size_t text_len = 0;
    if (sentence.fragment.is_fragmented) { if (!state_.navtex_message.complete) return true; text = state_.navtex_message.text; text_len = strlen(state_.navtex_message.text); received.complete = state_.navtex_message.complete; received.overflow = state_.navtex_message.overflow; } else { text = sentence.field(3).data; text_len = sentence.field(3).length; received.complete = true; received.overflow = false; }
    nmea_copy_span(received.message_text, sizeof(received.message_text), NmeaSpan(text, text_len));
    received.text_length.set(static_cast<int32_t>(strlen(received.message_text)), now_us);
    received.end_of_message = navtex_text_has_eom(received.message_text);
    received.framing_valid = strstr(received.message_text, "ZCZC") != nullptr && received.end_of_message;
    navtex_copy_body_text(received.body_text, sizeof(received.body_text), received.message_text);
    received.body_length.set(static_cast<int32_t>(strlen(received.body_text)), now_us);
    int32_t serial = 0; bool has_serial = false;
    parse_navtex_message_id(received.message_text, received.navtex_message_id, received.transmitter_id, received.subject_indicator, serial, has_serial);
    if (has_serial) { received.serial_number.set(serial, now_us); received.subject_category.set(navtex_subject_category(received.subject_indicator), now_us); nmea_copy_cstr(received.subject_label, sizeof(received.subject_label), navtex_subject_label(received.subject_indicator)); received.subject_is_service = received.subject_category.value == 20; }
    set_source(received.source, source); received.last_update_us = now_us; navtex_store_received_message(model.notifications.navtex, received, now_us); return true;
}

template<typename Model>
bool apply_nrm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) { last_error_ = "short NRM"; return false; }
    auto& mask = model.notifications.navtex.receiver_mask; int32_t value = 0;
    if (parse_int32(sentence.field(0), value)) mask.total_fragments.set(value, now_us);
    if (parse_int32(sentence.field(1), value)) mask.fragment_number.set(value, now_us);
    nmea_copy_span(mask.sentence_message_id, sizeof(mask.sentence_message_id), sentence.field(2));
    nmea_copy_span(mask.receiver_id, sizeof(mask.receiver_id), sentence.field(3));
    if (sentence.field_count > 4) nmea_copy_span(mask.station_mask, sizeof(mask.station_mask), sentence.field(4));
    if (sentence.field_count > 5) nmea_copy_span(mask.subject_mask, sizeof(mask.subject_mask), sentence.field(5));
    int32_t enabled = 0; mask.station_mask_bits = navtex_letter_mask_bits(mask.station_mask, enabled); mask.enabled_station_count.set(enabled, now_us); mask.subject_mask_bits = navtex_letter_mask_bits(mask.subject_mask, enabled); mask.enabled_subject_count.set(enabled, now_us);
    if (sentence.field_count > 6) nmea_copy_span(mask.status_text, sizeof(mask.status_text), sentence.field(6));
    mask.complete = !sentence.fragment.is_fragmented || state_.navtex_message.complete;
    mask.overflow = sentence.fragment.is_fragmented && state_.navtex_message.overflow;
    set_source(mask.source, source); mask.last_update_us = now_us; return true;
}
