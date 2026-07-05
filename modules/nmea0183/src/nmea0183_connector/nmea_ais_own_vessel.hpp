#pragma once

// Included inside Nmea0183RxConnector after nmea_ais.hpp and nmea_ais_control.hpp.

void ais_remove_target_by_mmsi(ship_data_model::AisTargetTableData<Real>& table,
                               int32_t mmsi,
                               uint64_t now_us) {
    for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
        auto& target = table.targets[i];
        if (target.occupied && target.mmsi.valid && target.mmsi.value == mmsi) {
            target = ship_data_model::AisTargetData<Real>{};
            if (table.target_count.valid && table.target_count.value > 0) {
                table.target_count.set(table.target_count.value - 1, now_us);
            }
            return;
        }
    }
}

bool ais_enough_bits(size_t payload_len, int32_t fill_bits, size_t bit_count) const {
    const size_t total = payload_len * 6u;
    if (fill_bits < 0 || static_cast<size_t>(fill_bits) > total) return false;
    return total - static_cast<size_t>(fill_bits) >= bit_count;
}

const char* ais_binary_application_label(int32_t dac, int32_t fi) const {
    if (dac == 1 && fi == 31) return "imo_met_hydro";
    if (dac == 1 && fi == 12) return "imo_dangerous_cargo";
    if (dac == 1 && fi == 16) return "imo_persons_on_board";
    if (dac == 1 && fi == 17) return "imo_vts_generated_text";
    if (dac == 1 && fi == 22) return "imo_area_notice";
    if (dac == 200 && fi == 10) return "regional_weather";
    return "unknown";
}

void ais_update_binary_application(const char* payload,
                                   size_t payload_len,
                                   int32_t fill_bits,
                                   const AisDecodedHeader& header,
                                   uint64_t now_us,
                                   ship_data_model::SensorSource source,
                                   ship_data_model::AisData<Real>& ais) {
    if (header.message_type != 6 && header.message_type != 8 && header.message_type != 25 && header.message_type != 26) return;

    bool ok = true;
    size_t payload_start = 0;
    bool addressed = false;
    bool structured = true;
    int32_t dac = -1;
    int32_t fi = -1;

    if (header.message_type == 6) {
        if (!ais_enough_bits(payload_len, fill_bits, 88)) return;
        addressed = true;
        dac = static_cast<int32_t>(ais_get_u(payload, payload_len, 72, 10, ok));
        fi = static_cast<int32_t>(ais_get_u(payload, payload_len, 82, 6, ok));
        payload_start = 88;
    } else if (header.message_type == 8) {
        if (!ais_enough_bits(payload_len, fill_bits, 56)) return;
        addressed = false;
        dac = static_cast<int32_t>(ais_get_u(payload, payload_len, 40, 10, ok));
        fi = static_cast<int32_t>(ais_get_u(payload, payload_len, 50, 6, ok));
        payload_start = 56;
    } else if (header.message_type == 25 || header.message_type == 26) {
        if (!ais_enough_bits(payload_len, fill_bits, 40)) return;
        addressed = ais_get_u(payload, payload_len, 38, 1, ok) != 0u;
        structured = ais_get_u(payload, payload_len, 39, 1, ok) != 0u;
        payload_start = 40;
        if (addressed) payload_start += 30;
        if (!structured) return;
        if (!ais_enough_bits(payload_len, fill_bits, payload_start + 16)) return;
        dac = static_cast<int32_t>(ais_get_u(payload, payload_len, payload_start, 10, ok));
        fi = static_cast<int32_t>(ais_get_u(payload, payload_len, payload_start + 10, 6, ok));
        payload_start += 16;
        if (header.message_type == 26) payload_start += 20;
    }
    if (!ok) return;

    auto& app = ais.binary_application;
    ais_set_header_record(app, header, now_us, source);
    app.dac.set(dac, now_us);
    app.function_id.set(fi, now_us);
    app.addressed = addressed;
    app.structured = structured;
    app.payload_start_bit.set(static_cast<int32_t>(payload_start), now_us);

    const size_t usable_bits = payload_len * 6u - static_cast<size_t>(fill_bits);
    const int32_t remaining = usable_bits > payload_start ? static_cast<int32_t>(usable_bits - payload_start) : 0;
    app.payload_bit_count.set(remaining, now_us);
    const uint8_t first_bits = remaining >= 32 ? 32 : static_cast<uint8_t>(remaining);
    if (first_bits > 0) {
        bool first_ok = true;
        app.first_payload_bits.set(static_cast<int32_t>(ais_get_u(payload, payload_len, payload_start, first_bits, first_ok)), now_us);
    }

    const char* label = ais_binary_application_label(dac, fi);
    nmea_copy_cstr(app.application_label, sizeof(app.application_label), label);
    app.known_application = strcmp(label, "unknown") != 0;
    app.last_update_us = now_us;
}

void ais_update_class_b_enhancements(const char* payload,
                                     size_t payload_len,
                                     int32_t fill_bits,
                                     const AisDecodedHeader& header,
                                     uint64_t now_us,
                                     ship_data_model::AisData<Real>& ais) {
    if (header.message_type != 18 && header.message_type != 19) return;
    bool ok = true;
    auto& pos = ais.class_b_position_report;

    if (header.message_type == 18 && ais_enough_bits(payload_len, fill_bits, 168)) {
        pos.cs_unit = ais_get_u(payload, payload_len, 141, 1, ok) != 0u;
        pos.display = ais_get_u(payload, payload_len, 142, 1, ok) != 0u;
        pos.dsc = ais_get_u(payload, payload_len, 143, 1, ok) != 0u;
        pos.band = ais_get_u(payload, payload_len, 144, 1, ok) != 0u;
        pos.accepts_message_22 = ais_get_u(payload, payload_len, 145, 1, ok) != 0u;
        pos.assigned_mode = ais_get_u(payload, payload_len, 146, 1, ok) != 0u;
        pos.raim = ais_get_u(payload, payload_len, 147, 1, ok) != 0u;
        pos.radio_status.set(static_cast<int32_t>(ais_get_u(payload, payload_len, 149, 19, ok)), now_us);
    } else if (header.message_type == 19 && ais_enough_bits(payload_len, fill_bits, 312)) {
        auto& stat = ais.class_b_static;
        const int32_t ship_type = static_cast<int32_t>(ais_get_u(payload, payload_len, 263, 8, ok));
        const int32_t bow = static_cast<int32_t>(ais_get_u(payload, payload_len, 271, 9, ok));
        const int32_t stern = static_cast<int32_t>(ais_get_u(payload, payload_len, 280, 9, ok));
        const int32_t port = static_cast<int32_t>(ais_get_u(payload, payload_len, 289, 6, ok));
        const int32_t starboard = static_cast<int32_t>(ais_get_u(payload, payload_len, 295, 6, ok));
        const int32_t epfd = static_cast<int32_t>(ais_get_u(payload, payload_len, 301, 4, ok));
        const bool dte_not_ready = ais_get_u(payload, payload_len, 305, 1, ok) != 0u;
        const bool assigned = ais_get_u(payload, payload_len, 306, 1, ok) != 0u;
        if (!ok) return;
        stat.ship_type.set(ship_type, now_us);
        stat.dimension_to_bow_m.set(bow, now_us);
        stat.dimension_to_stern_m.set(stern, now_us);
        stat.dimension_to_port_m.set(port, now_us);
        stat.dimension_to_starboard_m.set(starboard, now_us);
        stat.epfd_type.set(epfd, now_us);
        stat.dte_ready = !dte_not_ready;
        stat.assigned_mode = assigned;
        pos.dte_ready = !dte_not_ready;
        pos.assigned_mode = assigned;
    }
}

void ais_update_aton_extension(const char* payload,
                               size_t payload_len,
                               int32_t fill_bits,
                               const AisDecodedHeader& header,
                               uint64_t now_us,
                               ship_data_model::AisData<Real>& ais) {
    if (header.message_type != 21) return;
    if (!ais_enough_bits(payload_len, fill_bits, 278)) return;
    const size_t usable_bits = payload_len * 6u - static_cast<size_t>(fill_bits);
    const size_t extension_bits = usable_bits > 272u ? usable_bits - 272u : 0u;
    if (extension_bits < 6u) return;
    uint8_t chars = static_cast<uint8_t>(extension_bits / 6u);
    if (chars > 14u) chars = 14u;
    ais_copy_text(ais.aid_to_navigation.name_extension,
                  sizeof(ais.aid_to_navigation.name_extension),
                  payload,
                  payload_len,
                  272,
                  chars);
    ais.aid_to_navigation.name_extension_available = ais.aid_to_navigation.name_extension[0] != '\0';
    ais.aid_to_navigation.last_update_us = now_us;
}

void ais_update_enhanced_fields(const char* payload,
                                size_t payload_len,
                                int32_t fill_bits,
                                const AisDecodedHeader& header,
                                uint64_t now_us,
                                ship_data_model::SensorSource source,
                                ship_data_model::AisData<Real>& ais) {
    ais_update_class_b_enhancements(payload, payload_len, fill_bits, header, now_us, ais);
    ais_update_aton_extension(payload, payload_len, fill_bits, header, now_us, ais);
    ais_update_binary_application(payload, payload_len, fill_bits, header, now_us, source, ais);
}

void ais_set_own_vessel_header(ship_data_model::AisOwnVesselData<Real>& own,
                               const AisDecodedHeader& header,
                               uint64_t now_us,
                               ship_data_model::SensorSource source) {
    set_source(own.source, source);
    own.message_type.set(header.message_type, now_us);
    own.repeat_indicator.set(header.repeat_indicator, now_us);
    own.mmsi.set(header.mmsi, now_us);
    if (!own.valid) own.first_seen_us = now_us;
    own.valid = true;
    own.last_seen_us = now_us;
}

void ais_copy_own_position_from_position_report(ship_data_model::AisOwnVesselData<Real>& own,
                                                const ship_data_model::AisPositionReportData<Real>& pos,
                                                bool class_b,
                                                uint64_t now_us) {
    if (pos.navigation_status.valid) own.navigation_status.set(pos.navigation_status.value, now_us);
    if (pos.latitude_deg.valid) own.latitude_deg.set(pos.latitude_deg.value, now_us);
    if (pos.longitude_deg.valid) own.longitude_deg.set(pos.longitude_deg.value, now_us);
    if (pos.speed_over_ground_kn.valid) own.speed_over_ground_kn.set(pos.speed_over_ground_kn.value, now_us);
    if (pos.course_over_ground_deg.valid) own.course_over_ground_deg.set(pos.course_over_ground_deg.value, now_us);
    if (pos.true_heading_deg.valid) own.true_heading_deg.set(pos.true_heading_deg.value, now_us);
    if (pos.timestamp_s.valid) own.timestamp_s.set(pos.timestamp_s.value, now_us);
    own.position_accuracy = pos.position_accuracy;
    own.raim = pos.raim;
    own.class_b = class_b;
    own.last_position_update_us = now_us;
}

void ais_copy_own_position_from_sar(ship_data_model::AisOwnVesselData<Real>& own,
                                    const ship_data_model::AisSarAircraftPositionData<Real>& sar,
                                    uint64_t now_us) {
    if (sar.latitude_deg.valid) own.latitude_deg.set(sar.latitude_deg.value, now_us);
    if (sar.longitude_deg.valid) own.longitude_deg.set(sar.longitude_deg.value, now_us);
    if (sar.speed_over_ground_kn.valid) own.speed_over_ground_kn.set(sar.speed_over_ground_kn.value, now_us);
    if (sar.course_over_ground_deg.valid) own.course_over_ground_deg.set(sar.course_over_ground_deg.value, now_us);
    if (sar.timestamp_s.valid) own.timestamp_s.set(sar.timestamp_s.value, now_us);
    own.position_accuracy = sar.position_accuracy;
    own.raim = sar.raim;
    own.sar_aircraft = true;
    own.last_position_update_us = now_us;
}

void ais_copy_own_position_from_base_station(ship_data_model::AisOwnVesselData<Real>& own,
                                             const ship_data_model::AisBaseStationData<Real>& base,
                                             uint64_t now_us) {
    if (base.latitude_deg.valid) own.latitude_deg.set(base.latitude_deg.value, now_us);
    if (base.longitude_deg.valid) own.longitude_deg.set(base.longitude_deg.value, now_us);
    own.position_accuracy = base.position_accuracy;
    own.raim = base.raim;
    own.base_station = true;
    own.last_position_update_us = now_us;
}

void ais_copy_own_position_from_aton(ship_data_model::AisOwnVesselData<Real>& own,
                                     const ship_data_model::AisAidToNavigationData<Real>& aton,
                                     uint64_t now_us) {
    if (aton.latitude_deg.valid) own.latitude_deg.set(aton.latitude_deg.value, now_us);
    if (aton.longitude_deg.valid) own.longitude_deg.set(aton.longitude_deg.value, now_us);
    if (aton.timestamp_s.valid) own.timestamp_s.set(aton.timestamp_s.value, now_us);
    own.position_accuracy = aton.position_accuracy;
    own.raim = aton.raim;
    own.aid_to_navigation = true;
    own.last_position_update_us = now_us;
}

void ais_copy_own_position_from_long_range(ship_data_model::AisOwnVesselData<Real>& own,
                                           const ship_data_model::AisLongRangeBroadcastData<Real>& lr,
                                           uint64_t now_us) {
    if (lr.navigation_status.valid) own.navigation_status.set(lr.navigation_status.value, now_us);
    if (lr.latitude_deg.valid) own.latitude_deg.set(lr.latitude_deg.value, now_us);
    if (lr.longitude_deg.valid) own.longitude_deg.set(lr.longitude_deg.value, now_us);
    if (lr.speed_over_ground_kn.valid) own.speed_over_ground_kn.set(lr.speed_over_ground_kn.value, now_us);
    if (lr.course_over_ground_deg.valid) own.course_over_ground_deg.set(lr.course_over_ground_deg.value, now_us);
    own.position_accuracy = lr.position_accuracy;
    own.raim = lr.raim;
    own.last_position_update_us = now_us;
}

void ais_copy_own_static_from_voyage(ship_data_model::AisOwnVesselData<Real>& own,
                                     const ship_data_model::AisStaticVoyageData<Real>& stat,
                                     uint64_t now_us) {
    if (stat.ship_type.valid) own.ship_type.set(stat.ship_type.value, now_us);
    if (stat.imo_number.valid) own.imo_number.set(stat.imo_number.value, now_us);
    nmea_copy_cstr(own.vessel_name, sizeof(own.vessel_name), stat.vessel_name);
    nmea_copy_cstr(own.call_sign, sizeof(own.call_sign), stat.call_sign);
    nmea_copy_cstr(own.destination, sizeof(own.destination), stat.destination);
    own.last_static_update_us = now_us;
}

void ais_copy_own_static_from_class_b(ship_data_model::AisOwnVesselData<Real>& own,
                                      const ship_data_model::AisClassBStaticData<Real>& stat,
                                      uint64_t now_us) {
    if (stat.ship_type.valid) own.ship_type.set(stat.ship_type.value, now_us);
    if (stat.vessel_name[0]) nmea_copy_cstr(own.vessel_name, sizeof(own.vessel_name), stat.vessel_name);
    if (stat.call_sign[0]) nmea_copy_cstr(own.call_sign, sizeof(own.call_sign), stat.call_sign);
    own.class_b = true;
    own.last_static_update_us = now_us;
}

void ais_copy_own_static_from_aton(ship_data_model::AisOwnVesselData<Real>& own,
                                   const ship_data_model::AisAidToNavigationData<Real>& aton,
                                   uint64_t now_us) {
    if (aton.aid_type.valid) own.aid_type.set(aton.aid_type.value, now_us);
    if (aton.name[0]) nmea_copy_cstr(own.vessel_name, sizeof(own.vessel_name), aton.name);
    own.aid_to_navigation = true;
    own.last_static_update_us = now_us;
}

void ais_update_own_vessel_from_latest(ship_data_model::AisData<Real>& ais,
                                       const AisDecodedHeader& header,
                                       uint64_t now_us,
                                       ship_data_model::SensorSource source) {
    auto& own = ais.own_vessel;
    ais_set_own_vessel_header(own, header, now_us, source);

    switch (header.message_type) {
    case 1:
    case 2:
    case 3:
        ais_copy_own_position_from_position_report(own, ais.position_report, false, now_us);
        break;
    case 4:
    case 11:
        ais_copy_own_position_from_base_station(own, ais.base_station, now_us);
        break;
    case 5:
        ais_copy_own_static_from_voyage(own, ais.static_voyage, now_us);
        break;
    case 9:
        ais_copy_own_position_from_sar(own, ais.sar_aircraft_position, now_us);
        break;
    case 18:
    case 19:
        ais_copy_own_position_from_position_report(own, ais.class_b_position_report, true, now_us);
        if (header.message_type == 19) ais_copy_own_static_from_class_b(own, ais.class_b_static, now_us);
        break;
    case 21:
        ais_copy_own_position_from_aton(own, ais.aid_to_navigation, now_us);
        ais_copy_own_static_from_aton(own, ais.aid_to_navigation, now_us);
        break;
    case 24:
        ais_copy_own_static_from_class_b(own, ais.class_b_static, now_us);
        break;
    case 27:
        ais_copy_own_position_from_long_range(own, ais.long_range_broadcast, now_us);
        break;
    default:
        break;
    }
}

bool apply_ais_vdm_vdo_with_own_vessel(const NmeaSentence& sentence,
                                       ship_data_model::DataModel<Real>& model,
                                       uint64_t now_us,
                                       ship_data_model::SensorSource source) {
    const bool own_sentence = sentence_is(sentence, "VDO");
    const bool control = ais_sentence_is_control(sentence);
    const auto previous_tracked_target = model.ais.tracked_target;

    const bool parsed = control
        ? apply_ais_control_vdm_vdo(sentence, model, now_us, source)
        : apply_ais_vdm_vdo(sentence, model, now_us, source);
    if (!parsed) return false;

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

    if (!control) ais_update_enhanced_fields(payload, payload_len, fill_bits, header, now_us, source, model.ais);
    if (!own_sentence) return true;

    ais_update_own_vessel_from_latest(model.ais, header, now_us, source);
    ais_remove_target_by_mmsi(model.ais.targets, header.mmsi, now_us);
    model.ais.tracked_target = previous_tracked_target;
    return true;
}
