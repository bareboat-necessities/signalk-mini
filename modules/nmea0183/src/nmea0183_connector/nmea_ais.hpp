#pragma once

// Included inside Nmea0183RxConnector.

int ais_sixbit_value(char c) const {
    if (c >= '0' && c <= 'W') return c - '0';
    if (c >= '`' && c <= 'w') return c - '8';
    return -1;
}

bool ais_payload_valid(const char* payload, size_t payload_len) const {
    if (!payload || payload_len == 0) return false;
    for (size_t i = 0; i < payload_len; ++i) {
        if (ais_sixbit_value(payload[i]) < 0) return false;
    }
    return true;
}

uint32_t ais_get_u(const char* payload, size_t payload_len, size_t bit_offset, uint8_t bit_count, bool& ok) const {
    if (!ok || !payload || bit_count > 32u) {
        ok = false;
        return 0;
    }
    const size_t total_bits = payload_len * 6u;
    if (bit_offset + bit_count > total_bits) {
        ok = false;
        return 0;
    }

    uint32_t out = 0;
    for (uint8_t i = 0; i < bit_count; ++i) {
        const size_t bit_index = bit_offset + i;
        const int value = ais_sixbit_value(payload[bit_index / 6u]);
        if (value < 0) {
            ok = false;
            return 0;
        }
        const uint8_t bit = static_cast<uint8_t>((value >> (5u - (bit_index % 6u))) & 1u);
        out = static_cast<uint32_t>((out << 1u) | bit);
    }
    return out;
}

int32_t ais_get_s(const char* payload, size_t payload_len, size_t bit_offset, uint8_t bit_count, bool& ok) const {
    const uint32_t raw = ais_get_u(payload, payload_len, bit_offset, bit_count, ok);
    if (!ok || bit_count == 0u || bit_count >= 32u) return static_cast<int32_t>(raw);
    const uint32_t sign = 1u << (bit_count - 1u);
    if ((raw & sign) == 0u) return static_cast<int32_t>(raw);
    const uint32_t mask = (1u << bit_count) - 1u;
    return -static_cast<int32_t>(((~raw) & mask) + 1u);
}

char ais_text_char(uint32_t value) const {
    value &= 0x3fu;
    if (value < 32u) return static_cast<char>('@' + value);
    return static_cast<char>(' ' + (value - 32u));
}

void ais_copy_text(char* out, size_t out_size, const char* payload, size_t payload_len, size_t bit_offset, uint8_t char_count) const {
    if (!out || out_size == 0u) return;
    size_t n = char_count;
    if (n + 1u > out_size) n = out_size - 1u;
    bool ok = true;
    for (size_t i = 0; i < n; ++i) {
        out[i] = ais_text_char(ais_get_u(payload, payload_len, bit_offset + i * 6u, 6, ok));
        if (!ok) {
            out[i] = '\0';
            return;
        }
    }
    while (n > 0u && (out[n - 1u] == '@' || out[n - 1u] == ' ')) --n;
    out[n] = '\0';
}

bool ais_position_deg(int32_t raw, bool longitude, Real& out_deg) const {
    const int32_t unavailable = longitude ? 108600000 : 54600000;
    const int32_t max_abs = longitude ? 108000000 : 54000000;
    if (raw == unavailable || raw > max_abs || raw < -max_abs) return false;
    out_deg = static_cast<Real>(static_cast<float>(raw) / 600000.0f);
    return true;
}

void ais_set_common_raw(const NmeaSentence& sentence,
                        ship_data_model::AisRawMessageData<Real>& raw,
                        const char* payload,
                        size_t payload_len,
                        int32_t fill_bits,
                        int32_t message_type,
                        int32_t repeat_indicator,
                        int32_t mmsi,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
    set_source(raw.source, source);
    nmea_copy_span(raw.sentence_id, sizeof(raw.sentence_id), sentence.sentence);
    raw.radio_channel = sentence.field_count > 3 ? sentence.field(3)[0] : 0;
    raw.own_vessel = sentence_is(sentence, "VDO");
    raw.complete = true;
    raw.overflow = sentence.fragment.is_fragmented ? state_.ais_message.overflow : false;
    nmea_copy_span(raw.payload, sizeof(raw.payload), NmeaSpan(payload, payload_len));
    raw.payload_bit_count.set(static_cast<int32_t>(payload_len * 6u - static_cast<size_t>(fill_bits)), now_us);
    raw.fill_bits.set(fill_bits, now_us);
    raw.message_type.set(message_type, now_us);
    raw.repeat_indicator.set(repeat_indicator, now_us);
    raw.mmsi.set(mmsi, now_us);
    raw.last_update_us = now_us;
}

void ais_update_legacy_target_from_position(const ship_data_model::AisPositionReportData<Real>& pos,
                                            ship_data_model::TrackedTargetData<Real>& target,
                                            uint64_t now_us,
                                            ship_data_model::SensorSource source) {
    set_source(target.source, source);
    if (pos.mmsi.valid) target.target_number.set(pos.mmsi.value, now_us);
    if (pos.latitude_deg.valid) target.latitude_deg.set(pos.latitude_deg.value, now_us);
    if (pos.longitude_deg.valid) target.longitude_deg.set(pos.longitude_deg.value, now_us);
    if (pos.speed_over_ground_kn.valid) target.speed_kn.set(pos.speed_over_ground_kn.value, now_us);
    if (pos.course_over_ground_deg.valid) {
        target.course_deg.set(pos.course_over_ground_deg.value, now_us);
        target.course_reference = 'T';
    }
    target.last_update_us = now_us;
}

bool ais_parse_position_report(const char* payload,
                               size_t payload_len,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               ship_data_model::AisPositionReportData<Real>& out,
                               ship_data_model::TrackedTargetData<Real>& legacy_target) {
    bool ok = true;
    const int32_t message_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 0, 6, ok));
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    if (!ok) return false;

    set_source(out.source, source);
    out.message_type.set(message_type, now_us);
    out.mmsi.set(mmsi, now_us);

    if (message_type == 1 || message_type == 2 || message_type == 3) {
        const int32_t nav_status = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 4, ok));
        const int32_t rot_raw = ais_get_s(payload, payload_len, 42, 8, ok);
        const int32_t sog_raw = static_cast<int32_t>(ais_get_u(payload, payload_len, 50, 10, ok));
        const bool accuracy = ais_get_u(payload, payload_len, 60, 1, ok) != 0u;
        const int32_t lon_raw = ais_get_s(payload, payload_len, 61, 28, ok);
        const int32_t lat_raw = ais_get_s(payload, payload_len, 89, 27, ok);
        const int32_t cog_raw = static_cast<int32_t>(ais_get_u(payload, payload_len, 116, 12, ok));
        const int32_t heading_raw = static_cast<int32_t>(ais_get_u(payload, payload_len, 128, 9, ok));
        const int32_t timestamp = static_cast<int32_t>(ais_get_u(payload, payload_len, 137, 6, ok));
        const int32_t maneuver = static_cast<int32_t>(ais_get_u(payload, payload_len, 143, 2, ok));
        out.raim = ais_get_u(payload, payload_len, 148, 1, ok) != 0u;
        if (!ok) return false;
        out.navigation_status.set(nav_status, now_us);
        if (rot_raw != -128) {
            const float signed_rot = static_cast<float>(rot_raw) / 4.733f;
            const float rot_deg_min = signed_rot < 0.0f ? -(signed_rot * signed_rot) : signed_rot * signed_rot;
            out.rate_of_turn_deg_min.set(static_cast<Real>(rot_deg_min), now_us);
        }
        if (sog_raw < 1023) out.speed_over_ground_kn.set(static_cast<Real>(static_cast<float>(sog_raw) / 10.0f), now_us);
        out.position_accuracy = accuracy;
        Real lon = Real{};
        Real lat = Real{};
        if (ais_position_deg(lon_raw, true, lon)) out.longitude_deg.set(lon, now_us);
        if (ais_position_deg(lat_raw, false, lat)) out.latitude_deg.set(lat, now_us);
        if (cog_raw < 3600) out.course_over_ground_deg.set(static_cast<Real>(static_cast<float>(cog_raw) / 10.0f), now_us);
        if (heading_raw < 511) out.true_heading_deg.set(static_cast<Real>(heading_raw), now_us);
        if (timestamp < 60) out.timestamp_s.set(timestamp, now_us);
        out.maneuver_indicator.set(maneuver, now_us);
    } else if (message_type == 18 || message_type == 19) {
        const int32_t sog_raw = static_cast<int32_t>(ais_get_u(payload, payload_len, 46, 10, ok));
        const bool accuracy = ais_get_u(payload, payload_len, 56, 1, ok) != 0u;
        const int32_t lon_raw = ais_get_s(payload, payload_len, 57, 28, ok);
        const int32_t lat_raw = ais_get_s(payload, payload_len, 85, 27, ok);
        const int32_t cog_raw = static_cast<int32_t>(ais_get_u(payload, payload_len, 112, 12, ok));
        const int32_t heading_raw = static_cast<int32_t>(ais_get_u(payload, payload_len, 124, 9, ok));
        const int32_t timestamp = static_cast<int32_t>(ais_get_u(payload, payload_len, 133, 6, ok));
        out.raim = ais_get_u(payload, payload_len, 147, 1, ok) != 0u;
        if (!ok) return false;
        if (sog_raw < 1023) out.speed_over_ground_kn.set(static_cast<Real>(static_cast<float>(sog_raw) / 10.0f), now_us);
        out.position_accuracy = accuracy;
        Real lon = Real{};
        Real lat = Real{};
        if (ais_position_deg(lon_raw, true, lon)) out.longitude_deg.set(lon, now_us);
        if (ais_position_deg(lat_raw, false, lat)) out.latitude_deg.set(lat, now_us);
        if (cog_raw < 3600) out.course_over_ground_deg.set(static_cast<Real>(static_cast<float>(cog_raw) / 10.0f), now_us);
        if (heading_raw < 511) out.true_heading_deg.set(static_cast<Real>(heading_raw), now_us);
        if (timestamp < 60) out.timestamp_s.set(timestamp, now_us);
    } else {
        return false;
    }
    out.last_update_us = now_us;
    ais_update_legacy_target_from_position(out, legacy_target, now_us, source);
    return true;
}

bool ais_parse_base_station(const char* payload,
                            size_t payload_len,
                            uint64_t now_us,
                            ship_data_model::SensorSource source,
                            ship_data_model::AisBaseStationData<Real>& out) {
    bool ok = true;
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    const int32_t year = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 14, ok));
    const int32_t month = static_cast<int32_t>(ais_get_u(payload, payload_len, 52, 4, ok));
    const int32_t day = static_cast<int32_t>(ais_get_u(payload, payload_len, 56, 5, ok));
    const int32_t hour = static_cast<int32_t>(ais_get_u(payload, payload_len, 61, 5, ok));
    const int32_t minute = static_cast<int32_t>(ais_get_u(payload, payload_len, 66, 6, ok));
    const int32_t second = static_cast<int32_t>(ais_get_u(payload, payload_len, 72, 6, ok));
    const bool accuracy = ais_get_u(payload, payload_len, 78, 1, ok) != 0u;
    const int32_t lon_raw = ais_get_s(payload, payload_len, 79, 28, ok);
    const int32_t lat_raw = ais_get_s(payload, payload_len, 107, 27, ok);
    const int32_t epfd = static_cast<int32_t>(ais_get_u(payload, payload_len, 134, 4, ok));
    const bool raim = ais_get_u(payload, payload_len, 148, 1, ok) != 0u;
    if (!ok) return false;
    set_source(out.source, source);
    out.mmsi.set(mmsi, now_us);
    out.year.set(year, now_us);
    out.month.set(month, now_us);
    out.day.set(day, now_us);
    out.hour.set(hour, now_us);
    out.minute.set(minute, now_us);
    out.second.set(second, now_us);
    out.position_accuracy = accuracy;
    Real lon = Real{};
    Real lat = Real{};
    if (ais_position_deg(lon_raw, true, lon)) out.longitude_deg.set(lon, now_us);
    if (ais_position_deg(lat_raw, false, lat)) out.latitude_deg.set(lat, now_us);
    out.epfd_type.set(epfd, now_us);
    out.raim = raim;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_static_voyage(const char* payload,
                             size_t payload_len,
                             uint64_t now_us,
                             ship_data_model::SensorSource source,
                             ship_data_model::AisStaticVoyageData<Real>& out,
                             ship_data_model::TrackedTargetData<Real>& legacy_target) {
    bool ok = true;
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    const int32_t ais_version = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 2, ok));
    const int32_t imo = static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok));
    const int32_t ship_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 232, 8, ok));
    const int32_t bow = static_cast<int32_t>(ais_get_u(payload, payload_len, 240, 9, ok));
    const int32_t stern = static_cast<int32_t>(ais_get_u(payload, payload_len, 249, 9, ok));
    const int32_t port = static_cast<int32_t>(ais_get_u(payload, payload_len, 258, 6, ok));
    const int32_t starboard = static_cast<int32_t>(ais_get_u(payload, payload_len, 264, 6, ok));
    const int32_t epfd = static_cast<int32_t>(ais_get_u(payload, payload_len, 270, 4, ok));
    const int32_t eta_month = static_cast<int32_t>(ais_get_u(payload, payload_len, 274, 4, ok));
    const int32_t eta_day = static_cast<int32_t>(ais_get_u(payload, payload_len, 278, 5, ok));
    const int32_t eta_hour = static_cast<int32_t>(ais_get_u(payload, payload_len, 283, 5, ok));
    const int32_t eta_minute = static_cast<int32_t>(ais_get_u(payload, payload_len, 288, 6, ok));
    const int32_t draught_dm = static_cast<int32_t>(ais_get_u(payload, payload_len, 294, 8, ok));
    const bool dte_not_ready = ais_get_u(payload, payload_len, 422, 1, ok) != 0u;
    if (!ok) return false;
    set_source(out.source, source);
    out.mmsi.set(mmsi, now_us);
    out.ais_version.set(ais_version, now_us);
    out.imo_number.set(imo, now_us);
    ais_copy_text(out.call_sign, sizeof(out.call_sign), payload, payload_len, 70, 7);
    ais_copy_text(out.vessel_name, sizeof(out.vessel_name), payload, payload_len, 112, 20);
    out.ship_type.set(ship_type, now_us);
    out.dimension_to_bow_m.set(bow, now_us);
    out.dimension_to_stern_m.set(stern, now_us);
    out.dimension_to_port_m.set(port, now_us);
    out.dimension_to_starboard_m.set(starboard, now_us);
    out.epfd_type.set(epfd, now_us);
    out.eta_month.set(eta_month, now_us);
    out.eta_day.set(eta_day, now_us);
    out.eta_hour.set(eta_hour, now_us);
    out.eta_minute.set(eta_minute, now_us);
    out.draught_m.set(static_cast<Real>(static_cast<float>(draught_dm) / 10.0f), now_us);
    ais_copy_text(out.destination, sizeof(out.destination), payload, payload_len, 302, 20);
    out.dte_ready = !dte_not_ready;
    out.last_update_us = now_us;
    set_source(legacy_target.source, source);
    legacy_target.target_number.set(mmsi, now_us);
    nmea_copy_cstr(legacy_target.target_name, sizeof(legacy_target.target_name), out.vessel_name);
    legacy_target.last_update_us = now_us;
    return true;
}

bool ais_parse_class_b_static(const char* payload,
                              size_t payload_len,
                              uint64_t now_us,
                              ship_data_model::SensorSource source,
                              ship_data_model::AisClassBStaticData<Real>& out,
                              ship_data_model::TrackedTargetData<Real>& legacy_target) {
    bool ok = true;
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    const int32_t part = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 2, ok));
    if (!ok) return false;
    set_source(out.source, source);
    out.mmsi.set(mmsi, now_us);
    out.part_number.set(part, now_us);
    if (part == 0) {
        ais_copy_text(out.vessel_name, sizeof(out.vessel_name), payload, payload_len, 40, 20);
        nmea_copy_cstr(legacy_target.target_name, sizeof(legacy_target.target_name), out.vessel_name);
    } else if (part == 1) {
        const int32_t ship_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 8, ok));
        const int32_t bow = static_cast<int32_t>(ais_get_u(payload, payload_len, 132, 9, ok));
        const int32_t stern = static_cast<int32_t>(ais_get_u(payload, payload_len, 141, 9, ok));
        const int32_t port = static_cast<int32_t>(ais_get_u(payload, payload_len, 150, 6, ok));
        const int32_t starboard = static_cast<int32_t>(ais_get_u(payload, payload_len, 156, 6, ok));
        if (!ok) return false;
        out.ship_type.set(ship_type, now_us);
        ais_copy_text(out.vendor_id, sizeof(out.vendor_id), payload, payload_len, 48, 7);
        ais_copy_text(out.call_sign, sizeof(out.call_sign), payload, payload_len, 90, 7);
        out.dimension_to_bow_m.set(bow, now_us);
        out.dimension_to_stern_m.set(stern, now_us);
        out.dimension_to_port_m.set(port, now_us);
        out.dimension_to_starboard_m.set(starboard, now_us);
    }
    legacy_target.target_number.set(mmsi, now_us);
    set_source(legacy_target.source, source);
    legacy_target.last_update_us = now_us;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_aid_to_navigation(const char* payload,
                                 size_t payload_len,
                                 uint64_t now_us,
                                 ship_data_model::SensorSource source,
                                 ship_data_model::AisAidToNavigationData<Real>& out) {
    bool ok = true;
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    const int32_t aid_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 5, ok));
    const bool accuracy = ais_get_u(payload, payload_len, 163, 1, ok) != 0u;
    const int32_t lon_raw = ais_get_s(payload, payload_len, 164, 28, ok);
    const int32_t lat_raw = ais_get_s(payload, payload_len, 192, 27, ok);
    const int32_t bow = static_cast<int32_t>(ais_get_u(payload, payload_len, 219, 9, ok));
    const int32_t stern = static_cast<int32_t>(ais_get_u(payload, payload_len, 228, 9, ok));
    const int32_t port = static_cast<int32_t>(ais_get_u(payload, payload_len, 237, 6, ok));
    const int32_t starboard = static_cast<int32_t>(ais_get_u(payload, payload_len, 243, 6, ok));
    const int32_t epfd = static_cast<int32_t>(ais_get_u(payload, payload_len, 249, 4, ok));
    const int32_t timestamp = static_cast<int32_t>(ais_get_u(payload, payload_len, 253, 6, ok));
    const bool off_position = ais_get_u(payload, payload_len, 259, 1, ok) != 0u;
    const bool raim = ais_get_u(payload, payload_len, 268, 1, ok) != 0u;
    const bool virtual_aid = ais_get_u(payload, payload_len, 269, 1, ok) != 0u;
    const bool assigned_mode = ais_get_u(payload, payload_len, 270, 1, ok) != 0u;
    if (!ok) return false;
    set_source(out.source, source);
    out.mmsi.set(mmsi, now_us);
    out.aid_type.set(aid_type, now_us);
    ais_copy_text(out.name, sizeof(out.name), payload, payload_len, 43, 20);
    out.position_accuracy = accuracy;
    Real lon = Real{};
    Real lat = Real{};
    if (ais_position_deg(lon_raw, true, lon)) out.longitude_deg.set(lon, now_us);
    if (ais_position_deg(lat_raw, false, lat)) out.latitude_deg.set(lat, now_us);
    out.dimension_to_bow_m.set(bow, now_us);
    out.dimension_to_stern_m.set(stern, now_us);
    out.dimension_to_port_m.set(port, now_us);
    out.dimension_to_starboard_m.set(starboard, now_us);
    out.epfd_type.set(epfd, now_us);
    if (timestamp < 60) out.timestamp_s.set(timestamp, now_us);
    out.off_position = off_position;
    out.raim = raim;
    out.virtual_aid = virtual_aid;
    out.assigned_mode = assigned_mode;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_safety_text(const char* payload,
                           size_t payload_len,
                           uint64_t now_us,
                           ship_data_model::SensorSource source,
                           ship_data_model::AisSafetyTextData<Real>& out) {
    bool ok = true;
    const int32_t message_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 0, 6, ok));
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    set_source(out.source, source);
    out.message_type.set(message_type, now_us);
    out.mmsi.set(mmsi, now_us);
    if (message_type == 12) {
        const int32_t sequence = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 2, ok));
        const int32_t destination = static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok));
        const bool retransmit = ais_get_u(payload, payload_len, 70, 1, ok) != 0u;
        if (!ok) return false;
        out.sequence_number.set(sequence, now_us);
        out.destination_mmsi.set(destination, now_us);
        out.retransmit = retransmit;
        ais_copy_text(out.text, sizeof(out.text), payload, payload_len, 72, static_cast<uint8_t>((payload_len * 6u - 72u) / 6u));
    } else if (message_type == 14) {
        if (!ok) return false;
        ais_copy_text(out.text, sizeof(out.text), payload, payload_len, 40, static_cast<uint8_t>((payload_len * 6u - 40u) / 6u));
    } else {
        return false;
    }
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_binary_metadata(const char* payload,
                               size_t payload_len,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               ship_data_model::AisBinaryData<Real>& out) {
    bool ok = true;
    const int32_t message_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 0, 6, ok));
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    set_source(out.source, source);
    out.message_type.set(message_type, now_us);
    out.mmsi.set(mmsi, now_us);
    if (message_type == 6) {
        const int32_t sequence = static_cast<int32_t>(ais_get_u(payload, payload_len, 38, 2, ok));
        const int32_t destination = static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok));
        const bool retransmit = ais_get_u(payload, payload_len, 70, 1, ok) != 0u;
        const int32_t dac = static_cast<int32_t>(ais_get_u(payload, payload_len, 72, 10, ok));
        const int32_t function_id = static_cast<int32_t>(ais_get_u(payload, payload_len, 82, 6, ok));
        if (!ok) return false;
        out.sequence_number.set(sequence, now_us);
        out.destination_mmsi.set(destination, now_us);
        out.retransmit = retransmit;
        out.dac.set(dac, now_us);
        out.function_id.set(function_id, now_us);
        out.payload_bit_count.set(static_cast<int32_t>(payload_len * 6u - 88u), now_us);
    } else if (message_type == 8) {
        const int32_t dac = static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 10, ok));
        const int32_t function_id = static_cast<int32_t>(ais_get_u(payload, payload_len, 50, 6, ok));
        if (!ok) return false;
        out.dac.set(dac, now_us);
        out.function_id.set(function_id, now_us);
        out.payload_bit_count.set(static_cast<int32_t>(payload_len * 6u - 56u), now_us);
    } else {
        return false;
    }
    out.last_update_us = now_us;
    return true;
}

bool apply_ais_vdm_vdo(const NmeaSentence& sentence,
                       ship_data_model::DataModel<Real>& model,
                       uint64_t now_us,
                       ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short AIS";
        return false;
    }

    int32_t fill_bits = 0;
    if (!parse_int32(sentence.field(5), fill_bits) || fill_bits < 0 || fill_bits > 5) {
        last_error_ = "bad AIS fill bits";
        return false;
    }

    const char* payload = nullptr;
    size_t payload_len = 0;
    if (sentence.fragment.is_fragmented) {
        if (!state_.ais_message.complete) return true;
        payload = state_.ais_message.text;
        payload_len = strlen(state_.ais_message.text);
    } else {
        payload = sentence.field(4).data;
        payload_len = sentence.field(4).length;
    }

    if (!ais_payload_valid(payload, payload_len)) {
        last_error_ = "bad AIS payload";
        return false;
    }
    const size_t payload_bits = payload_len * 6u;
    if (payload_bits <= static_cast<size_t>(fill_bits) || payload_bits - static_cast<size_t>(fill_bits) < 38u) {
        last_error_ = "short AIS payload";
        return false;
    }

    bool ok = true;
    const int32_t message_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 0, 6, ok));
    const int32_t repeat_indicator = static_cast<int32_t>(ais_get_u(payload, payload_len, 6, 2, ok));
    const int32_t mmsi = static_cast<int32_t>(ais_get_u(payload, payload_len, 8, 30, ok));
    if (!ok || message_type < 1 || message_type > 27) {
        last_error_ = "bad AIS header";
        return false;
    }

    ais_set_common_raw(sentence, model.ais.raw, payload, payload_len, fill_bits, message_type, repeat_indicator, mmsi, now_us, source);

    bool parsed = true;
    switch (message_type) {
    case 1:
    case 2:
    case 3:
        parsed = ais_parse_position_report(payload, payload_len, now_us, source, model.ais.position_report, model.ais.tracked_target);
        break;
    case 4:
        parsed = ais_parse_base_station(payload, payload_len, now_us, source, model.ais.base_station);
        break;
    case 5:
        parsed = ais_parse_static_voyage(payload, payload_len, now_us, source, model.ais.static_voyage, model.ais.tracked_target);
        break;
    case 6:
    case 8:
        parsed = ais_parse_binary_metadata(payload, payload_len, now_us, source, model.ais.binary);
        break;
    case 12:
    case 14:
        parsed = ais_parse_safety_text(payload, payload_len, now_us, source, model.ais.safety_text);
        break;
    case 18:
    case 19:
        parsed = ais_parse_position_report(payload, payload_len, now_us, source, model.ais.class_b_position_report, model.ais.tracked_target);
        if (parsed && message_type == 19) {
            ais_copy_text(model.ais.class_b_static.vessel_name, sizeof(model.ais.class_b_static.vessel_name), payload, payload_len, 143, 20);
            model.ais.class_b_static.mmsi.set(mmsi, now_us);
            model.ais.class_b_static.last_update_us = now_us;
        }
        break;
    case 21:
        parsed = ais_parse_aid_to_navigation(payload, payload_len, now_us, source, model.ais.aid_to_navigation);
        break;
    case 24:
        parsed = ais_parse_class_b_static(payload, payload_len, now_us, source, model.ais.class_b_static, model.ais.tracked_target);
        break;
    default:
        parsed = true;
        break;
    }

    if (!parsed) {
        last_error_ = "bad AIS message fields";
        return false;
    }
    return true;
}
