#pragma once

// Included inside Nmea0183RxConnector after nmea_ais.hpp.

bool ais_control_message_type(int32_t type) const {
    return type == 7 || type == 10 || type == 13 || type == 15 ||
           type == 16 || type == 17 || type == 20 || type == 22 || type == 23;
}

bool ais_control_payload_from_sentence(const NmeaSentence& sentence,
                                       const char*& payload,
                                       size_t& payload_len,
                                       int32_t& fill_bits) {
    if (sentence.field_count < 6) {
        last_error_ = "short AIS";
        return false;
    }
    if (!parse_int32(sentence.field(5), fill_bits) || fill_bits < 0 || fill_bits > 5) {
        last_error_ = "bad AIS fill bits";
        return false;
    }
    if (sentence.fragment.is_fragmented) {
        if (!state_.ais_message.complete) {
            payload = nullptr;
            payload_len = 0;
            return true;
        }
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
    return true;
}

bool ais_sentence_is_control(const NmeaSentence& sentence) {
    const char* payload = nullptr;
    size_t payload_len = 0;
    int32_t fill_bits = 0;
    const char* saved_error = last_error_;
    if (!ais_control_payload_from_sentence(sentence, payload, payload_len, fill_bits) || !payload) {
        last_error_ = saved_error;
        return false;
    }
    AisDecodedHeader header;
    const bool ok = ais_decode_header(payload, payload_len, header) && ais_control_message_type(header.message_type);
    last_error_ = saved_error;
    return ok;
}

template<typename Record>
void ais_set_control_header(Record& out,
                            const AisDecodedHeader& header,
                            uint64_t now_us,
                            ship_data_model::SensorSource source,
                            ship_data_model::AisTargetTableData<Real>& target_table) {
    ais_set_header_record(out, header, now_us, source);
    ais_update_target_header(target_table, header, now_us, source);
}

bool ais_parse_acknowledgement(const char* payload,
                               size_t payload_len,
                               const AisDecodedHeader& header,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               ship_data_model::AisAcknowledgementData<Real>& out,
                               ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    ais_set_control_header(out, header, now_us, source, target_table);
    uint8_t count = 0;
    size_t offset = 40;
    while (count < 4 && offset + 32u <= payload_len * 6u) {
        const int32_t dest = static_cast<int32_t>(ais_get_u(payload, payload_len, offset, 30, ok));
        const int32_t seq = static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 30u, 2, ok));
        if (!ok) return false;
        out.destination_mmsi[count].set(dest, now_us);
        out.sequence_number[count].set(seq, now_us);
        ++count;
        offset += 32u;
    }
    out.acknowledgement_count.set(static_cast<int32_t>(count), now_us);
    out.last_update_us = now_us;
    return count > 0;
}

bool ais_parse_utc_inquiry(const char* payload,
                           size_t payload_len,
                           const AisDecodedHeader& header,
                           uint64_t now_us,
                           ship_data_model::SensorSource source,
                           ship_data_model::AisUtcInquiryData<Real>& out,
                           ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 72u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    out.destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok)), now_us);
    if (!ok) return false;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_interrogation(const char* payload,
                             size_t payload_len,
                             const AisDecodedHeader& header,
                             uint64_t now_us,
                             ship_data_model::SensorSource source,
                             ship_data_model::AisInterrogationData<Real>& out,
                             ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    const size_t bits = payload_len * 6u;
    if (bits < 88u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    out.first_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok)), now_us);
    out.first_message_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 70, 6, ok)), now_us);
    out.first_slot_offset.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 76, 12, ok)), now_us);
    if (!ok) return false;
    if (bits >= 110u) {
        out.second_message_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 90, 6, ok)), now_us);
        out.second_slot_offset.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 96, 12, ok)), now_us);
        if (!ok) return false;
    }
    if (bits >= 160u) {
        out.second_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 110, 30, ok)), now_us);
        out.third_message_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 140, 6, ok)), now_us);
        out.third_slot_offset.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 146, 12, ok)), now_us);
        if (!ok) return false;
    }
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_assignment_command(const char* payload,
                                  size_t payload_len,
                                  const AisDecodedHeader& header,
                                  uint64_t now_us,
                                  ship_data_model::SensorSource source,
                                  ship_data_model::AisAssignmentCommandData<Real>& out,
                                  ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    const size_t bits = payload_len * 6u;
    if (bits < 92u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    uint8_t count = 0;
    out.destination_mmsi[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 30, ok)), now_us);
    out.offset[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 70, 12, ok)), now_us);
    out.increment[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 82, 10, ok)), now_us);
    if (!ok) return false;
    ++count;
    if (bits >= 144u) {
        out.destination_mmsi[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 92, 30, ok)), now_us);
        out.offset[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 122, 12, ok)), now_us);
        out.increment[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, 134, 10, ok)), now_us);
        if (!ok) return false;
        ++count;
    }
    out.assignment_count.set(static_cast<int32_t>(count), now_us);
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_data_link_management(const char* payload,
                                    size_t payload_len,
                                    const AisDecodedHeader& header,
                                    uint64_t now_us,
                                    ship_data_model::SensorSource source,
                                    ship_data_model::AisDataLinkManagementData<Real>& out,
                                    ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    ais_set_control_header(out, header, now_us, source, target_table);
    uint8_t count = 0;
    size_t offset = 40;
    while (count < 4 && offset + 30u <= payload_len * 6u) {
        out.slot_offset[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset, 12, ok)), now_us);
        out.slot_count[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 12u, 4, ok)), now_us);
        out.timeout_min[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 16u, 3, ok)), now_us);
        out.increment[count].set(static_cast<int32_t>(ais_get_u(payload, payload_len, offset + 19u, 11, ok)), now_us);
        if (!ok) return false;
        ++count;
        offset += 30u;
    }
    out.reservation_count.set(static_cast<int32_t>(count), now_us);
    out.last_update_us = now_us;
    return count > 0;
}

bool ais_parse_dgnss_broadcast(const char* payload,
                               size_t payload_len,
                               const AisDecodedHeader& header,
                               uint64_t now_us,
                               ship_data_model::SensorSource source,
                               ship_data_model::AisDgnssBroadcastData<Real>& out,
                               ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 80u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    Real lon = Real{};
    Real lat = Real{};
    const int32_t lon_raw = ais_get_s(payload, payload_len, 40, 18, ok);
    const int32_t lat_raw = ais_get_s(payload, payload_len, 58, 17, ok);
    if (!ok) return false;
    if (ais_long_range_position_deg(lon_raw, true, lon)) out.longitude_deg.set(lon, now_us);
    if (ais_long_range_position_deg(lat_raw, false, lat)) out.latitude_deg.set(lat, now_us);
    const int32_t bits = static_cast<int32_t>(payload_len * 6u);
    out.payload_bit_count.set(bits > 80 ? bits - 80 : 0, now_us);
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_channel_management(const char* payload,
                                  size_t payload_len,
                                  const AisDecodedHeader& header,
                                  uint64_t now_us,
                                  ship_data_model::SensorSource source,
                                  ship_data_model::AisChannelManagementData<Real>& out,
                                  ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 145u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    out.channel_a.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 12, ok)), now_us);
    out.channel_b.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 52, 12, ok)), now_us);
    out.txrx_mode.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 64, 4, ok)), now_us);
    out.high_power = ais_get_u(payload, payload_len, 68, 1, ok) != 0u;
    const bool addressed = ais_get_u(payload, payload_len, 139, 1, ok) != 0u;
    out.addressed = addressed;
    if (addressed) {
        out.first_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 69, 30, ok)), now_us);
        out.second_destination_mmsi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 104, 30, ok)), now_us);
    } else {
        Real ne_lon = Real{};
        Real ne_lat = Real{};
        Real sw_lon = Real{};
        Real sw_lat = Real{};
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 69, 18, ok), true, ne_lon)) out.northeast_lon_deg.set(ne_lon, now_us);
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 87, 17, ok), false, ne_lat)) out.northeast_lat_deg.set(ne_lat, now_us);
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 104, 18, ok), true, sw_lon)) out.southwest_lon_deg.set(sw_lon, now_us);
        if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 122, 17, ok), false, sw_lat)) out.southwest_lat_deg.set(sw_lat, now_us);
    }
    out.bandwidth_a_12_5khz = ais_get_u(payload, payload_len, 140, 1, ok) != 0u;
    out.bandwidth_b_12_5khz = ais_get_u(payload, payload_len, 141, 1, ok) != 0u;
    out.transitional_zone_size_nmi.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 142, 3, ok)), now_us);
    if (!ok) return false;
    out.last_update_us = now_us;
    return true;
}

bool ais_parse_group_assignment(const char* payload,
                                size_t payload_len,
                                const AisDecodedHeader& header,
                                uint64_t now_us,
                                ship_data_model::SensorSource source,
                                ship_data_model::AisGroupAssignmentCommandData<Real>& out,
                                ship_data_model::AisTargetTableData<Real>& target_table) {
    bool ok = true;
    if (payload_len * 6u < 154u) return false;
    ais_set_control_header(out, header, now_us, source, target_table);
    Real ne_lon = Real{};
    Real ne_lat = Real{};
    Real sw_lon = Real{};
    Real sw_lat = Real{};
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 40, 18, ok), true, ne_lon)) out.northeast_lon_deg.set(ne_lon, now_us);
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 58, 17, ok), false, ne_lat)) out.northeast_lat_deg.set(ne_lat, now_us);
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 75, 18, ok), true, sw_lon)) out.southwest_lon_deg.set(sw_lon, now_us);
    if (ais_long_range_position_deg(ais_get_s(payload, payload_len, 93, 17, ok), false, sw_lat)) out.southwest_lat_deg.set(sw_lat, now_us);
    out.station_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 110, 4, ok)), now_us);
    out.ship_type.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 114, 8, ok)), now_us);
    out.txrx_mode.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 144, 2, ok)), now_us);
    out.report_interval.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 146, 4, ok)), now_us);
    out.quiet_time_min.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 150, 4, ok)), now_us);
    if (!ok) return false;
    out.last_update_us = now_us;
    return true;
}

bool apply_ais_control_vdm_vdo(const NmeaSentence& sentence,
                               ship_data_model::DataModel<Real>& model,
                               uint64_t now_us,
                               ship_data_model::SensorSource source) {
    const char* payload = nullptr;
    size_t payload_len = 0;
    int32_t fill_bits = 0;
    if (!ais_control_payload_from_sentence(sentence, payload, payload_len, fill_bits)) return false;
    if (!payload) return true;

    AisDecodedHeader header;
    if (!ais_decode_header(payload, payload_len, header)) {
        last_error_ = "bad AIS header";
        return false;
    }
    if (!ais_control_message_type(header.message_type)) {
        last_error_ = "unsupported AIS control message type";
        return false;
    }

    bool parsed = false;
    switch (header.message_type) {
    case 7:
    case 13:
        parsed = ais_parse_acknowledgement(payload, payload_len, header, now_us, source, model.ais.acknowledgement, model.ais.targets);
        break;
    case 10:
        parsed = ais_parse_utc_inquiry(payload, payload_len, header, now_us, source, model.ais.utc_inquiry, model.ais.targets);
        break;
    case 15:
        parsed = ais_parse_interrogation(payload, payload_len, header, now_us, source, model.ais.interrogation, model.ais.targets);
        break;
    case 16:
        parsed = ais_parse_assignment_command(payload, payload_len, header, now_us, source, model.ais.assignment_command, model.ais.targets);
        break;
    case 17:
        parsed = ais_parse_dgnss_broadcast(payload, payload_len, header, now_us, source, model.ais.dgnss_broadcast, model.ais.targets);
        break;
    case 20:
        parsed = ais_parse_data_link_management(payload, payload_len, header, now_us, source, model.ais.data_link_management, model.ais.targets);
        break;
    case 22:
        parsed = ais_parse_channel_management(payload, payload_len, header, now_us, source, model.ais.channel_management, model.ais.targets);
        break;
    case 23:
        parsed = ais_parse_group_assignment(payload, payload_len, header, now_us, source, model.ais.group_assignment, model.ais.targets);
        break;
    default:
        break;
    }
    if (!parsed) {
        last_error_ = "bad AIS control fields";
        return false;
    }
    return true;
}
