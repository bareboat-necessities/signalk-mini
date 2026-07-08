#pragma once

// Included inside Nmea0183RxConnector.
// Balanced NMEA sentence group: S-T.

template<typename Model>
bool apply_sfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short SFI";
        return false;
    }

    int32_t n = 0;
    float v = 0.0f;
    if (parse_int32(sentence.field(0), n)) {
        model.nav.scanning_frequency.total_messages.set(n, now_us);
    }
    if (parse_int32(sentence.field(1), n)) {
        model.nav.scanning_frequency.message_number.set(n, now_us);
    }
    for (uint8_t i = 0; i < 6; ++i) {
        const uint8_t base = static_cast<uint8_t>(2 + i * 2);
        if (base >= sentence.field_count) break;
        if (parse_real(sentence.field(base), v)) {
            model.nav.scanning_frequency.frequency_hz[i].set(static_cast<Real>(v), now_us);
        }
        if (base + 1 < sentence.field_count) {
            model.nav.scanning_frequency.mode[i] = sentence.field(base + 1)[0];
        }
    }
    set_source(model.nav.scanning_frequency.source, source);
    model.nav.scanning_frequency.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_smv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) {
        last_error_ = "short SMV";
        return false;
    }

    auto& rec = model.notifications.special.smv;
    float value = 0.0f;
    int32_t parsed = 0;
    nmea_copy_span(rec.message_id, sizeof(rec.message_id), sentence.field(0));
    if (parse_int32(sentence.field(1), parsed)) {
        rec.mmsi.set(parsed, now_us);
    } else {
        nmea_copy_span(rec.mmsi_text, sizeof(rec.mmsi_text), sentence.field(1));
    }
    if (parse_utc_time_of_day_s(sentence.field(2), value)) {
        rec.utc_time_s.set(static_cast<Real>(value), now_us);
    }
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) {
        rec.latitude_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_lat_lon(sentence.field(5), sentence.field(6), value)) {
        rec.longitude_deg.set(static_cast<Real>(value), now_us);
    }
    nmea_copy_span(rec.event_type, sizeof(rec.event_type), sentence.field(7));
    if (sentence.field_count > 8) {
        nmea_copy_span(rec.sar_capability, sizeof(rec.sar_capability), sentence.field(8));
    }
    if (sentence.field_count > 9) {
        nmea_copy_span(rec.route_id, sizeof(rec.route_id), sentence.field(9));
    }
    if (sentence.field_count > 10) {
        rec.status = sentence.field(10)[0];
    }
    if (sentence.field_count > 11) {
        nmea_copy_span(rec.description, sizeof(rec.description), sentence.field(11));
    }
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source);
    rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_stn(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    int32_t n = 0;
    if (sentence.field_count < 1 || !parse_int32(sentence.field(0), n)) {
        last_error_ = "bad STN";
        return false;
    }
    model.nav.multiple_data_id.talker_id_number.set(n, now_us);
    set_source(model.nav.multiple_data_id.source, source);
    model.nav.multiple_data_id.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tds(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short TDS";
        return false;
    }

    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) {
        model.trawl.door_centerline_offset_m.set(static_cast<Real>(v), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) {
        model.trawl.door_along_centerline_m.set(static_cast<Real>(v), now_us);
    }
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) {
        model.trawl.depth_below_surface_m.set(static_cast<Real>(v), now_us);
    }
    set_source(model.trawl.source, source);
    model.trawl.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tfi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short TFI";
        return false;
    }

    int32_t n = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        if (parse_int32(sentence.field(i), n)) {
            model.trawl.catch_sensor_status[i].set(n, now_us);
        }
    }
    set_source(model.trawl.source, source);
    model.trawl.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tlb(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 2) {
        last_error_ = "short TLB";
        return false;
    }

    int32_t n = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i + 1 < sentence.field_count && count < 8; i = static_cast<uint8_t>(i + 2)) {
        if (!parse_int32(sentence.field(i), n)) continue;
        model.ais.tracked_target.label_target_number[count].set(n, now_us);
        nmea_copy_span(model.ais.tracked_target.label[count],
                       sizeof(model.ais.tracked_target.label[count]),
                       sentence.field(static_cast<uint8_t>(i + 1)));
        ++count;
    }
    if (count == 0) {
        last_error_ = "bad TLB";
        return false;
    }
    model.ais.tracked_target.label_count.set(static_cast<int32_t>(count), now_us);
    set_source(model.ais.tracked_target.source, source);
    model.ais.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tll(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) {
        last_error_ = "short TLL";
        return false;
    }

    int32_t n = 0;
    float v = 0.0f;
    if (parse_int32(sentence.field(0), n)) {
        model.ais.tracked_target.target_number.set(n, now_us);
    }
    if (parse_lat_lon(sentence.field(1), sentence.field(2), v)) {
        model.ais.tracked_target.latitude_deg.set(static_cast<Real>(v), now_us);
    }
    if (parse_lat_lon(sentence.field(3), sentence.field(4), v)) {
        model.ais.tracked_target.longitude_deg.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.ais.tracked_target.target_name, sizeof(model.ais.tracked_target.target_name), sentence.field(5));
    if (parse_utc_time_of_day_s(sentence.field(6), v)) {
        model.ais.tracked_target.utc_time_s.set(static_cast<Real>(v), now_us);
    }
    model.ais.tracked_target.target_status = sentence.field(7)[0];
    model.ais.tracked_target.reference_target = sentence.field(8)[0];
    set_source(model.ais.tracked_target.source, source);
    model.ais.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short TPC";
        return false;
    }

    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) {
        model.trawl.cartesian_centerline_offset_m.set(static_cast<Real>(v), now_us);
    }
    if (sentence.field(3)[0] == 'M' && parse_real(sentence.field(2), v)) {
        model.trawl.cartesian_along_centerline_m.set(static_cast<Real>(v), now_us);
    }
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) {
        model.trawl.depth_below_surface_m.set(static_cast<Real>(v), now_us);
    }
    set_source(model.trawl.source, source);
    model.trawl.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpr(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short TPR";
        return false;
    }

    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) {
        model.trawl.relative_range_m.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(2), v)) {
        model.trawl.relative_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) {
        model.trawl.depth_below_surface_m.set(static_cast<Real>(v), now_us);
    }
    set_source(model.trawl.source, source);
    model.trawl.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_tpt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) {
        last_error_ = "short TPT";
        return false;
    }

    float v = 0.0f;
    if (sentence.field(1)[0] == 'M' && parse_real(sentence.field(0), v)) {
        model.trawl.true_range_m.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(2), v)) {
        model.trawl.true_bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (sentence.field(5)[0] == 'M' && parse_real(sentence.field(4), v)) {
        model.trawl.depth_below_surface_m.set(static_cast<Real>(v), now_us);
    }
    set_source(model.trawl.source, source);
    model.trawl.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_trf(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) {
        last_error_ = "short TRF";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) {
        model.nav.transit_fix.utc_time_s.set(static_cast<Real>(value), now_us);
    }
    nmea_copy_span(model.nav.transit_fix.date_ddmmyy, sizeof(model.nav.transit_fix.date_ddmmyy), sentence.field(1));
    if (parse_lat_lon(sentence.field(2), sentence.field(3), value)) {
        model.nav.transit_fix.latitude_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_lat_lon(sentence.field(4), sentence.field(5), value)) {
        model.nav.transit_fix.longitude_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(6), value)) {
        model.nav.transit_fix.elevation_angle_deg.set(static_cast<Real>(value), now_us);
    }
    if (parse_int32(sentence.field(7), integer_value)) {
        model.nav.transit_fix.iterations.set(integer_value, now_us);
    }
    if (parse_int32(sentence.field(8), integer_value)) {
        model.nav.transit_fix.doppler_intervals.set(integer_value, now_us);
    }
    if (parse_real(sentence.field(9), value)) {
        model.nav.transit_fix.update_distance_nmi.set(static_cast<Real>(value), now_us);
    }
    if (parse_int32(sentence.field(10), integer_value)) {
        model.nav.transit_fix.satellite_number.set(integer_value, now_us);
    }
    model.nav.transit_fix.data_validity = sentence.field(11)[0];
    set_source(model.nav.transit_fix.source, source);
    model.nav.transit_fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_ttm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) {
        last_error_ = "short TTM";
        return false;
    }

    int32_t n = 0;
    float v = 0.0f;
    model.ais.tracked_target.bearing_reference = sentence.field(3)[0];
    model.ais.tracked_target.course_reference = sentence.field(6)[0];
    model.ais.tracked_target.distance_units = sentence.field(9)[0];
    model.ais.tracked_target.target_status = sentence.field(11)[0];
    model.ais.tracked_target.reference_target = sentence.field(12)[0];

    if (parse_int32(sentence.field(0), n)) {
        model.ais.tracked_target.target_number.set(n, now_us);
    }
    if (parse_distance_nmi(sentence.field(1), sentence.field(9), v)) {
        model.ais.tracked_target.distance_nmi.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(2), v)) {
        model.ais.tracked_target.bearing_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_real(sentence.field(4), v)) {
        model.ais.tracked_target.speed_kn.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(5), v)) {
        model.ais.tracked_target.course_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    }
    if (parse_distance_nmi(sentence.field(7), sentence.field(9), v)) {
        model.ais.tracked_target.cpa_distance_nmi.set(static_cast<Real>(v), now_us);
    }
    if (parse_real(sentence.field(8), v)) {
        model.ais.tracked_target.tcpa_min.set(static_cast<Real>(v), now_us);
    }
    nmea_copy_span(model.ais.tracked_target.target_name, sizeof(model.ais.tracked_target.target_name), sentence.field(10));
    set_source(model.ais.tracked_target.source, source);
    model.ais.tracked_target.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_txt(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    return apply_nmea_text_record(sentence, model.notifications.messages.text, now_us, source);
}
