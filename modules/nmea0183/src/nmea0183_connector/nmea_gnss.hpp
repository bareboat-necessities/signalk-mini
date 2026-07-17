#pragma once

// Included inside Nmea0183RxConnector.

void copy_gnss_talker_id(char* out, size_t out_size, const NmeaSentence& sentence) {
    nmea_copy_span(out, out_size, sentence.talker);
}

bool gnss_mode_has_fix(NmeaSpan mode) {
    for (size_t i = 0; i < mode.length; ++i) {
        if (mode[i] != '\0' && mode[i] != 'N') return true;
    }
    return false;
}

template<typename Fix>
void update_gnss_hae_altitude(Fix& fix, uint64_t now_us) {
    if (!fix.fix_alt_msl_m.valid || !fix.geoidal_separation_m.valid) return;
    fix.fix_alt_hae_m.set(static_cast<Real>(fix.fix_alt_msl_m.value + fix.geoidal_separation_m.value), now_us);
}

template<typename Model>
bool gnss_satellite_is_used(const Model& model, int32_t satellite_id) {
    for (size_t i = 0; i < 12; ++i) {
        const auto& active = model.gnss.dop_active_satellites.satellite_prn[i];
        if (active.valid && active.value == satellite_id) return true;
    }
    return false;
}

template<typename Model>
bool apply_alm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 15) { last_error_ = "short ALM"; return false; }
    int32_t integer_value = 0; float value = 0.0f;
    copy_gnss_talker_id(model.gnss.almanac.talker_id, sizeof(model.gnss.almanac.talker_id), sentence);
    if (parse_int32(sentence.field(0), integer_value)) model.gnss.almanac.total_messages.set(integer_value, now_us);
    if (parse_int32(sentence.field(1), integer_value)) model.gnss.almanac.message_number.set(integer_value, now_us);
    if (parse_int32(sentence.field(2), integer_value)) model.gnss.almanac.satellite_prn.set(integer_value, now_us);
    if (parse_int32(sentence.field(3), integer_value)) model.gnss.almanac.gnss_week.set(integer_value, now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.gnss.almanac.sv_health.set(integer_value, now_us);
    if (parse_real(sentence.field(5), value)) model.gnss.almanac.eccentricity.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.gnss.almanac.reference_time_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) model.gnss.almanac.inclination_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.gnss.almanac.right_ascension_rate_rad_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.gnss.almanac.sqrt_semi_major_axis.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(10), value)) model.gnss.almanac.argument_of_perigee_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(11), value)) model.gnss.almanac.longitude_ascension_node_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(12), value)) model.gnss.almanac.mean_anomaly_rad.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(13), value)) model.gnss.almanac.clock_f0_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(14), value)) model.gnss.almanac.clock_f1_s_s.set(static_cast<Real>(value), now_us);
    set_source(model.gnss.almanac.source, source); model.gnss.almanac.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gbs(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) { last_error_ = "short GBS"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    copy_gnss_talker_id(model.gnss.fault_detection.talker_id, sizeof(model.gnss.fault_detection.talker_id), sentence);
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.gnss.fault_detection.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.gnss.fault_detection.expected_error_lat_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.gnss.fault_detection.expected_error_lon_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(3), value)) model.gnss.fault_detection.expected_error_alt_m.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.gnss.fault_detection.failed_satellite_prn.set(integer_value, now_us);
    if (parse_real(sentence.field(5), value)) model.gnss.fault_detection.missed_detection_probability.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.gnss.fault_detection.failed_satellite_bias_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) model.gnss.fault_detection.failed_satellite_bias_stddev_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 8 && parse_int32(sentence.field(8), integer_value)) model.gnss.fault_detection.system_id.set(integer_value, now_us);
    if (sentence.field_count > 9 && parse_int32(sentence.field(9), integer_value)) model.gnss.fault_detection.signal_id.set(integer_value, now_us);
    set_source(model.gnss.fault_detection.source, source); model.gnss.fault_detection.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_dtm(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) { last_error_ = "short DTM"; return false; }
    float value = 0.0f;
    nmea_copy_span(model.nav.datum.local_datum_code, sizeof(model.nav.datum.local_datum_code), sentence.field(0));
    nmea_copy_span(model.nav.datum.local_datum_subcode, sizeof(model.nav.datum.local_datum_subcode), sentence.field(1));
    if (parse_north_south_signed(sentence.field(2), sentence.field(3), value)) model.nav.datum.latitude_offset_min.set(static_cast<Real>(value), now_us);
    if (parse_east_west_signed(sentence.field(4), sentence.field(5), value)) model.nav.datum.longitude_offset_min.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.nav.datum.altitude_offset_m.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.nav.datum.reference_datum_code, sizeof(model.nav.datum.reference_datum_code), sentence.field(7));
    set_source(model.nav.datum.source, source); model.nav.datum.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gfa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 1) { last_error_ = "short GFA"; return false; }
    auto& rec = model.gnss.fix_accuracy;
    float value = 0.0f; int32_t integer_value = 0;
    copy_gnss_talker_id(rec.talker_id, sizeof(rec.talker_id), sentence);
    if (parse_utc_time_of_day_s(sentence.field(0), value)) rec.utc_time_s.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 1) rec.mode_indicator = sentence.field(1)[0];
    if (sentence.field_count > 2) rec.status = sentence.field(2)[0];
    if (sentence.field_count > 3 && parse_real(sentence.field(3), value)) rec.horizontal_accuracy_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 4 && parse_real(sentence.field(4), value)) rec.vertical_accuracy_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 5 && parse_real(sentence.field(5), value)) rec.semi_major_accuracy_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 6 && parse_real(sentence.field(6), value)) rec.semi_minor_accuracy_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 7 && parse_real(sentence.field(7), value)) rec.semi_major_orientation_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (sentence.field_count > 8 && parse_real(sentence.field(8), value)) rec.pdop.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 9 && parse_real(sentence.field(9), value)) rec.hdop.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 10 && parse_real(sentence.field(10), value)) rec.vdop.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 11 && parse_int32(sentence.field(11), integer_value)) rec.system_id.set(integer_value, now_us);
    if (sentence.field_count > 12 && parse_int32(sentence.field(12), integer_value)) rec.signal_id.set(integer_value, now_us);
    rec.field_count.set(static_cast<int32_t>(sentence.field_count), now_us);
    set_source(rec.source, source); rec.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gga(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9 || sentence.field(5)[0] == '0') { last_error_ = "invalid GGA"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence);
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.gnss.fix.timestamp_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.gnss.fix.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.gnss.fix.fix_lon_deg.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(5), integer_value)) {
        model.gnss.fix.fix_quality.set(integer_value, now_us);
        model.gnss.fix.fix_valid.set(integer_value > 0, now_us);
    }
    if (parse_int32(sentence.field(6), integer_value)) model.gnss.fix.satellites_used.set(integer_value, now_us);
    if (parse_real(sentence.field(7), value)) model.gnss.fix.hdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.gnss.fix.fix_alt_msl_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count >= 11 && parse_real(sentence.field(10), value)) model.gnss.fix.geoidal_separation_m.set(static_cast<Real>(value), now_us);
    update_gnss_hae_altitude(model.gnss.fix, now_us);
    if (sentence.field_count >= 13 && parse_real(sentence.field(12), value)) model.gnss.fix.dgps_age_s.set(static_cast<Real>(value), now_us);
    if (sentence.field_count >= 14 && parse_int32(sentence.field(13), integer_value)) model.gnss.fix.dgps_station_id.set(integer_value, now_us);
    set_source(model.gnss.fix.source, source); model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_glc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) { last_error_ = "short GLC"; return false; }
    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.nav.legacy_timing.gri_us_div_10.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.nav.legacy_timing.master_toa_us.set(static_cast<Real>(value), now_us);
    model.nav.legacy_timing.master_toa_status = sentence.field(2)[0];
    for (uint8_t index = 0; index < 5; ++index) { const uint8_t base = static_cast<uint8_t>(3 + index * 2); if (parse_real(sentence.field(base), value)) model.nav.legacy_timing.delta_us[index].set(static_cast<Real>(value), now_us); model.nav.legacy_timing.delta_status[index] = sentence.field(base + 1)[0]; }
    set_source(model.nav.legacy_timing.source, source); model.nav.legacy_timing.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gll(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6 || sentence.field(5)[0] != 'A') { last_error_ = "bad GLL"; return false; }
    float value = 0.0f;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence);
    if (parse_lat_lon(sentence.field(0), sentence.field(1), value)) model.gnss.fix.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(2), sentence.field(3), value)) model.gnss.fix.fix_lon_deg.set(static_cast<Real>(value), now_us);
    if (parse_utc_time_of_day_s(sentence.field(4), value)) model.gnss.fix.timestamp_s.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 6) model.gnss.fix.mode_indicator = sentence.field(6)[0];
    model.gnss.fix.fix_valid.set(true, now_us);
    set_source(model.gnss.fix.source, source); model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gns(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) { last_error_ = "short GNS"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence);
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.gnss.fix.timestamp_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.gnss.fix.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.gnss.fix.fix_lon_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.gnss.fix.fix_mode_indicator, sizeof(model.gnss.fix.fix_mode_indicator), sentence.field(5));
    const bool has_fix = gnss_mode_has_fix(sentence.field(5));
    model.gnss.fix.fix_quality.set(has_fix ? 1 : 0, now_us);
    model.gnss.fix.fix_valid.set(has_fix, now_us);
    if (parse_int32(sentence.field(6), integer_value)) model.gnss.fix.satellites_used.set(integer_value, now_us);
    if (parse_real(sentence.field(7), value)) model.gnss.fix.hdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.gnss.fix.fix_alt_msl_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.gnss.fix.geoidal_separation_m.set(static_cast<Real>(value), now_us);
    update_gnss_hae_altitude(model.gnss.fix, now_us);
    if (parse_real(sentence.field(10), value)) model.gnss.fix.dgps_age_s.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(11), integer_value)) model.gnss.fix.dgps_station_id.set(integer_value, now_us);
    if (sentence.field_count >= 13) model.gnss.fix.navigational_status = sentence.field(12)[0];
    if (sentence.field_count >= 14 && parse_int32(sentence.field(13), integer_value)) model.gnss.fix.system_id.set(integer_value, now_us);
    if (sentence.field_count >= 15 && parse_int32(sentence.field(14), integer_value)) model.gnss.fix.signal_id.set(integer_value, now_us);
    set_source(model.gnss.fix.source, source); model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_grs(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 14) { last_error_ = "short GRS"; return false; }
    float value = 0.0f; int32_t mode = 0;
    copy_gnss_talker_id(model.gnss.range_residual.talker_id, sizeof(model.gnss.range_residual.talker_id), sentence);
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.gnss.range_residual.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(1), mode)) model.gnss.range_residual.mode.set(mode, now_us);
    for (uint8_t index = 0; index < 12; ++index) if (parse_real(sentence.field(static_cast<uint8_t>(2 + index)), value)) model.gnss.range_residual.satellite_residual_m[index].set(static_cast<Real>(value), now_us);
    set_source(model.gnss.range_residual.source, source); model.gnss.range_residual.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gsa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 17) { last_error_ = "short GSA"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    copy_gnss_talker_id(model.gnss.dop_active_satellites.talker_id, sizeof(model.gnss.dop_active_satellites.talker_id), sentence);
    model.gnss.dop_active_satellites.selection_mode = sentence.field(0)[0];
    if (parse_int32(sentence.field(1), integer_value)) {
        model.gnss.dop_active_satellites.fix_mode.set(integer_value, now_us);
        model.gnss.fix.fix_type.set(integer_value, now_us);
        model.gnss.fix.fix_valid.set(integer_value > 1, now_us);
    }
    for (uint8_t index = 0; index < 12; ++index) if (parse_int32(sentence.field(static_cast<uint8_t>(2 + index)), integer_value)) model.gnss.dop_active_satellites.satellite_prn[index].set(integer_value, now_us);
    if (parse_real(sentence.field(14), value)) model.gnss.dop_active_satellites.pdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(15), value)) { model.gnss.dop_active_satellites.hdop.set(static_cast<Real>(value), now_us); model.gnss.fix.hdop.set(static_cast<Real>(value), now_us); }
    if (parse_real(sentence.field(16), value)) model.gnss.dop_active_satellites.vdop.set(static_cast<Real>(value), now_us);
    if (sentence.field_count > 17 && parse_int32(sentence.field(17), integer_value)) model.gnss.dop_active_satellites.system_id.set(integer_value, now_us);
    if (sentence.field_count > 18 && parse_int32(sentence.field(18), integer_value)) model.gnss.dop_active_satellites.signal_id.set(integer_value, now_us);
    set_source(model.gnss.dop_active_satellites.source, source); set_source(model.gnss.fix.source, source); model.gnss.dop_active_satellites.last_update_us = now_us; model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gst(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) { last_error_ = "short GST"; return false; }
    float value = 0.0f;
    copy_gnss_talker_id(model.gnss.noise_statistics.talker_id, sizeof(model.gnss.noise_statistics.talker_id), sentence);
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.gnss.noise_statistics.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.gnss.noise_statistics.rms_range_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.gnss.noise_statistics.semi_major_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(3), value)) model.gnss.noise_statistics.semi_minor_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(4), value)) model.gnss.noise_statistics.semi_major_orientation_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_real(sentence.field(5), value)) model.gnss.noise_statistics.latitude_error_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.gnss.noise_statistics.longitude_error_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) {
        model.gnss.noise_statistics.altitude_error_stddev_m.set(static_cast<Real>(value), now_us);
        model.gnss.fix_accuracy.vertical_accuracy_m.set(static_cast<Real>(value), now_us);
    }
    set_source(model.gnss.noise_statistics.source, source); set_source(model.gnss.fix_accuracy.source, source);
    model.gnss.noise_statistics.last_update_us = now_us; model.gnss.fix_accuracy.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gsv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) { last_error_ = "short GSV"; return false; }
    float value = 0.0f;
    int32_t total_messages = 0;
    int32_t message_number = 0;
    int32_t satellites_in_view = 0;
    if (!parse_int32(sentence.field(0), total_messages) || total_messages <= 0 ||
        !parse_int32(sentence.field(1), message_number) || message_number <= 0 || message_number > total_messages) {
        last_error_ = "bad GSV sequence";
        return false;
    }
    parse_int32(sentence.field(2), satellites_in_view);

    auto& rec = model.gnss.satellites_in_view;
    copy_gnss_talker_id(rec.talker_id, sizeof(rec.talker_id), sentence);
    rec.total_messages.set(total_messages, now_us);
    rec.message_number.set(message_number, now_us);
    rec.satellites_in_view.set(satellites_in_view, now_us);

    uint8_t used_fields = 3;
    uint8_t parsed_observations = 0;
    for (uint8_t index = 0; index < 4; ++index) {
        const uint8_t base = static_cast<uint8_t>(3 + index * 4);
        if (base >= sentence.field_count) break;
        int32_t integer_value = 0;
        if (parse_int32(sentence.field(base), integer_value)) rec.satellite_prn[index].set(integer_value, now_us);
        if (base + 1 < sentence.field_count && parse_real(sentence.field(base + 1), value)) rec.elevation_deg[index].set(static_cast<Real>(value), now_us);
        if (base + 2 < sentence.field_count && parse_real(sentence.field(base + 2), value)) rec.azimuth_true_deg[index].set(static_cast<Real>(wrap_360_deg(value)), now_us);
        if (base + 3 < sentence.field_count && parse_real(sentence.field(base + 3), value)) rec.snr_db[index].set(static_cast<Real>(value), now_us);
        used_fields = static_cast<uint8_t>(base + 4);
        ++parsed_observations;
    }

    int32_t signal_id = -1;
    int32_t system_id = -1;
    if (used_fields < sentence.field_count && parse_int32(sentence.field(used_fields), signal_id)) rec.signal_id.set(signal_id, now_us);
    if (used_fields + 1 < sentence.field_count && parse_int32(sentence.field(static_cast<uint8_t>(used_fields + 1)), system_id)) rec.system_id.set(system_id, now_us);

    auto& sky = model.gnss.sky_view;
    if (message_number == 1 || std::strncmp(sky.talker_id, rec.talker_id, sizeof(sky.talker_id)) != 0) {
        sky.clear_observations();
        nmea_copy_cstr(sky.talker_id, sizeof(sky.talker_id), rec.talker_id);
    }
    sky.satellites_in_view.set(satellites_in_view, now_us);
    sky.complete = false;

    const size_t fragment_start = static_cast<size_t>(message_number - 1) * 4u;
    for (uint8_t index = 0; index < parsed_observations; ++index) {
        const size_t target_index = fragment_start + index;
        if (target_index >= sky.capacity()) break;
        auto& observation = sky.observations[target_index];
        if (rec.satellite_prn[index].valid) {
            observation.satellite_id = static_cast<int16_t>(rec.satellite_prn[index].value);
            observation.satellite_id_valid = true;
            observation.used = gnss_satellite_is_used(model, rec.satellite_prn[index].value);
        }
        if (rec.elevation_deg[index].valid) {
            observation.elevation_deg = rec.elevation_deg[index].value;
            observation.elevation_valid = true;
        }
        if (rec.azimuth_true_deg[index].valid) {
            observation.azimuth_true_deg = rec.azimuth_true_deg[index].value;
            observation.azimuth_valid = true;
        }
        if (rec.snr_db[index].valid) {
            observation.cn0_db_hz = rec.snr_db[index].value;
            observation.cn0_valid = true;
        }
        observation.signal_id = static_cast<int16_t>(signal_id);
        observation.system_id = static_cast<int16_t>(system_id);
        const uint16_t count = static_cast<uint16_t>(target_index + 1);
        if (count > sky.observation_count) sky.observation_count = count;
    }

    if (message_number == total_messages) {
        int32_t used_count = 0;
        for (uint16_t i = 0; i < sky.observation_count; ++i) if (sky.observations[i].used) ++used_count;
        sky.satellites_used.set(used_count, now_us);
        sky.complete = true;
        ++sky.sequence;
    }

    set_source(rec.source, source); set_source(sky.source, source);
    rec.last_update_us = now_us; sky.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gtd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float value = 0.0f; bool any = false;
    for (uint8_t index = 0; index < 5 && index < sentence.field_count; ++index) if (parse_real(sentence.field(index), value)) { model.nav.legacy_delta.value[index].set(static_cast<Real>(value), now_us); any = true; }
    set_source(model.nav.legacy_delta.source, source); model.nav.legacy_delta.last_update_us = now_us;
    if (!any) last_error_ = "bad GTD";
    return any;
}

template<typename Model>
bool apply_gxa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 7) { last_error_ = "short GXA"; return false; }
    float value = 0.0f; int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.nav.transit_fix.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.nav.transit_fix.latitude_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.nav.transit_fix.longitude_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.nav.transit_fix.waypoint_id, sizeof(model.nav.transit_fix.waypoint_id), sentence.field(5));
    if (parse_int32(sentence.field(6), integer_value)) model.nav.transit_fix.satellite_number.set(integer_value, now_us);
    set_source(model.nav.transit_fix.source, source); model.nav.transit_fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_osd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9) { last_error_ = "short OSD"; return false; }
    float v = 0.0f;
    if (sentence.field(1)[0] == 'A' && parse_real(sentence.field(0), v)) { const Real h = static_cast<Real>(wrap_360_deg(v)); model.nav.own_ship.heading_true_deg.set(h, now_us); model.ins.imu.heading_deg.set(h, now_us); model.ins.imu.heading_true_deg.set(h, now_us); }
    model.nav.own_ship.heading_status = sentence.field(1)[0]; model.nav.own_ship.course_reference = sentence.field(3)[0]; model.nav.own_ship.speed_reference = sentence.field(5)[0];
    if (parse_real(sentence.field(2), v)) { model.nav.own_ship.course_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); if (sentence.field(3)[0] == 'T') model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_knots(sentence.field(4), sentence.field(8), v)) { model.nav.own_ship.speed_kn.set(static_cast<Real>(v), now_us); model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(6), v)) { model.nav.own_ship.set_true_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); model.sea.current_direction_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_knots(sentence.field(7), sentence.field(8), v)) { model.nav.own_ship.drift_speed_kn.set(static_cast<Real>(v), now_us); model.sea.current_speed_kn.set(static_cast<Real>(v), now_us); }
    set_source(model.nav.own_ship.source, source); set_source(model.gnss.fix.source, source); set_source(model.sea.source, source); model.nav.own_ship.last_update_us = now_us; model.gnss.fix.last_update_us = now_us; model.sea.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rma(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 11 || sentence.field(0)[0] != 'A') { last_error_ = "bad RMA"; return false; }
    float v = 0.0f;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence); model.nav.rma.status = sentence.field(0)[0];
    if (parse_lat_lon(sentence.field(1), sentence.field(2), v)) { model.nav.rma.latitude_deg.set(static_cast<Real>(v), now_us); model.gnss.fix.fix_lat_deg.set(static_cast<Real>(v), now_us); }
    if (parse_lat_lon(sentence.field(3), sentence.field(4), v)) { model.nav.rma.longitude_deg.set(static_cast<Real>(v), now_us); model.gnss.fix.fix_lon_deg.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(5), v)) model.nav.rma.time_difference_a_us.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(6), v)) model.nav.rma.time_difference_b_us.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) { model.nav.rma.speed_kn.set(static_cast<Real>(v), now_us); model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us); }
    if (parse_real(sentence.field(8), v)) { model.nav.rma.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us); }
    if (parse_east_west_signed(sentence.field(9), sentence.field(10), v)) { model.nav.rma.magnetic_variation_deg.set(static_cast<Real>(v), now_us); model.gnss.fix.declination_deg.set(static_cast<Real>(v), now_us); model.ins.imu.magnetic_variation_deg.set(static_cast<Real>(v), now_us); }
    model.gnss.fix.fix_valid.set(true, now_us);
    set_source(model.nav.rma.source, source); set_source(model.gnss.fix.source, source); model.nav.rma.last_update_us = now_us; model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_rmc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8 || sentence.field(1)[0] != 'A') { last_error_ = "invalid RMC"; return false; }
    float v = 0.0f;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence);
    if (parse_lat_lon(sentence.field(2), sentence.field(3), v)) model.gnss.fix.fix_lat_deg.set(static_cast<Real>(v), now_us);
    if (parse_lat_lon(sentence.field(4), sentence.field(5), v)) model.gnss.fix.fix_lon_deg.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(6), v)) model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us);
    if (parse_real(sentence.field(7), v)) model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (sentence.field_count >= 9 && parse_rmc_timestamp_s(sentence.field(0), sentence.field(8), v)) model.gnss.fix.timestamp_s.set(static_cast<Real>(v), now_us);
    if (sentence.field_count >= 11 && parse_east_west_signed(sentence.field(9), sentence.field(10), v)) model.gnss.fix.declination_deg.set(static_cast<Real>(v), now_us);
    if (sentence.field_count > 11) model.gnss.fix.mode_indicator = sentence.field(11)[0];
    if (sentence.field_count > 12) model.gnss.fix.navigational_status = sentence.field(12)[0];
    model.gnss.fix.fix_valid.set(true, now_us);
    set_source(model.gnss.fix.source, source); model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_vtg(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 7) { last_error_ = "short VTG"; return false; }
    float v = 0.0f;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence);
    if (parse_real(sentence.field(0), v)) model.gnss.fix.track_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
    if (parse_knots(sentence.field(4), sentence.field(5), v) || parse_knots(sentence.field(6), sentence.field(7), v)) model.gnss.fix.speed_kn.set(static_cast<Real>(v), now_us);
    if (sentence.field_count > 8) model.gnss.fix.mode_indicator = sentence.field(8)[0];
    set_source(model.gnss.fix.source, source); model.gnss.fix.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_zda(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6) { last_error_ = "short ZDA"; return false; }
    float v = 0.0f; int32_t n = 0;
    copy_gnss_talker_id(model.gnss.fix.talker_id, sizeof(model.gnss.fix.talker_id), sentence);
    if (!parse_utc_time_of_day_s(sentence.field(0), v)) { last_error_ = "bad ZDA"; return false; }
    model.gnss.fix.timestamp_s.set(static_cast<Real>(v), now_us);
    if (parse_int32(sentence.field(1), n)) model.gnss.fix.date_day.set(n, now_us);
    if (parse_int32(sentence.field(2), n)) model.gnss.fix.date_month.set(n, now_us);
    if (parse_int32(sentence.field(3), n)) model.gnss.fix.date_year.set(n, now_us);
    if (parse_int32(sentence.field(4), n)) model.gnss.fix.local_zone_hours.set(n, now_us);
    if (parse_int32(sentence.field(5), n)) model.gnss.fix.local_zone_minutes.set(n, now_us);
    set_source(model.gnss.fix.source, source); model.gnss.fix.last_update_us = now_us;
    return true;
}
