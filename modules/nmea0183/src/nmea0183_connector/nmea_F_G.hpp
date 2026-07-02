#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_fir(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    (void)model;
    return apply_raw_sentence_record(sentence, state_.fire_detection, "FIR", now_us, source);
}

template<typename Model>
bool apply_fsi(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 4) {
        last_error_ = "short FSI";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_real(sentence.field(0), value)) {
        model.navigation.radio_frequency_set.transmitting_frequency_hz.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(1), value)) {
        model.navigation.radio_frequency_set.receiving_frequency_hz.set(static_cast<Real>(value), now_us);
    }
    model.navigation.radio_frequency_set.communication_mode = sentence.field(2)[0];
    if (parse_int32(sentence.field(3), integer_value)) {
        model.navigation.radio_frequency_set.power_level.set(integer_value, now_us);
    }
    set_source(model.navigation.radio_frequency_set.source, source);
    model.navigation.radio_frequency_set.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gbs(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) {
        last_error_ = "short GBS";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.gps_fault.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.navigation.gps_fault.expected_error_lat_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.navigation.gps_fault.expected_error_lon_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(3), value)) model.navigation.gps_fault.expected_error_alt_m.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(4), integer_value)) model.navigation.gps_fault.failed_satellite_prn.set(integer_value, now_us);
    if (parse_real(sentence.field(5), value)) model.navigation.gps_fault.missed_detection_probability.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.navigation.gps_fault.failed_satellite_bias_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) model.navigation.gps_fault.failed_satellite_bias_stddev_m.set(static_cast<Real>(value), now_us);
    set_source(model.navigation.gps_fault.source, source);
    model.navigation.gps_fault.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gga(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 9 || sentence.field(5)[0] == '0') {
        last_error_ = "invalid GGA";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.gps.timestamp_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(5), integer_value)) model.navigation.gps.fix_quality.set(integer_value, now_us);
    if (parse_int32(sentence.field(6), integer_value)) model.navigation.gps.satellites_used.set(integer_value, now_us);
    if (parse_real(sentence.field(7), value)) model.navigation.gps.hdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.navigation.gps.fix_alt_m.set(static_cast<Real>(value), now_us);
    if (sentence.field_count >= 11 && parse_real(sentence.field(10), value)) {
        model.navigation.gps.geoidal_separation_m.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count >= 13 && parse_real(sentence.field(12), value)) {
        model.navigation.gps.dgps_age_s.set(static_cast<Real>(value), now_us);
    }
    if (sentence.field_count >= 14 && parse_int32(sentence.field(13), integer_value)) {
        model.navigation.gps.dgps_station_id.set(integer_value, now_us);
    }
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_glc(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 13) {
        last_error_ = "short GLC";
        return false;
    }

    float value = 0.0f;
    if (parse_real(sentence.field(0), value)) model.navigation.legacy_timing.gri_us_div_10.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.navigation.legacy_timing.master_toa_us.set(static_cast<Real>(value), now_us);
    model.navigation.legacy_timing.master_toa_status = sentence.field(2)[0];
    for (uint8_t index = 0; index < 5; ++index) {
        const uint8_t base = static_cast<uint8_t>(3 + index * 2);
        if (parse_real(sentence.field(base), value)) model.navigation.legacy_timing.delta_us[index].set(static_cast<Real>(value), now_us);
        model.navigation.legacy_timing.delta_status[index] = sentence.field(base + 1)[0];
    }
    set_source(model.navigation.legacy_timing.source, source);
    model.navigation.legacy_timing.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gll(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 6 || sentence.field(5)[0] != 'A') {
        last_error_ = "bad GLL";
        return false;
    }

    float value = 0.0f;
    if (parse_lat_lon(sentence.field(0), sentence.field(1), value)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(2), sentence.field(3), value)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(value), now_us);
    if (parse_utc_time_of_day_s(sentence.field(4), value)) model.navigation.gps.timestamp_s.set(static_cast<Real>(value), now_us);
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gns(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) {
        last_error_ = "short GNS";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.gps.timestamp_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.navigation.gps.fix_mode_indicator, sizeof(model.navigation.gps.fix_mode_indicator), sentence.field(5));
    if (sentence.field(5).length > 0) model.navigation.gps.fix_quality.set(sentence.field(5)[0] == 'N' ? 0 : 1, now_us);
    if (parse_int32(sentence.field(6), integer_value)) model.navigation.gps.satellites_used.set(integer_value, now_us);
    if (parse_real(sentence.field(7), value)) model.navigation.gps.hdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(8), value)) model.navigation.gps.fix_alt_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(9), value)) model.navigation.gps.geoidal_separation_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(10), value)) model.navigation.gps.dgps_age_s.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(11), integer_value)) model.navigation.gps.dgps_station_id.set(integer_value, now_us);
    if (sentence.field_count >= 13) model.navigation.gps.navigational_status = sentence.field(12)[0];
    set_source(model.navigation.gps.source, source);
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_grs(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 14) {
        last_error_ = "short GRS";
        return false;
    }

    float value = 0.0f;
    int32_t mode = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.gps_range_residual.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_int32(sentence.field(1), mode)) model.navigation.gps_range_residual.mode.set(mode, now_us);
    for (uint8_t index = 0; index < 12; ++index) {
        if (parse_real(sentence.field(static_cast<uint8_t>(2 + index)), value)) {
            model.navigation.gps_range_residual.satellite_residual_m[index].set(static_cast<Real>(value), now_us);
        }
    }
    set_source(model.navigation.gps_range_residual.source, source);
    model.navigation.gps_range_residual.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gsa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 17) {
        last_error_ = "short GSA";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    model.navigation.gps_dop.selection_mode = sentence.field(0)[0];
    if (parse_int32(sentence.field(1), integer_value)) model.navigation.gps_dop.fix_mode.set(integer_value, now_us);
    for (uint8_t index = 0; index < 12; ++index) {
        if (parse_int32(sentence.field(static_cast<uint8_t>(2 + index)), integer_value)) {
            model.navigation.gps_dop.satellite_prn[index].set(integer_value, now_us);
        }
    }
    if (parse_real(sentence.field(14), value)) model.navigation.gps_dop.pdop.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(15), value)) {
        model.navigation.gps_dop.hdop.set(static_cast<Real>(value), now_us);
        model.navigation.gps.hdop.set(static_cast<Real>(value), now_us);
    }
    if (parse_real(sentence.field(16), value)) model.navigation.gps_dop.vdop.set(static_cast<Real>(value), now_us);
    set_source(model.navigation.gps_dop.source, source);
    set_source(model.navigation.gps.source, source);
    model.navigation.gps_dop.last_update_us = now_us;
    model.navigation.gps.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gst(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 8) {
        last_error_ = "short GST";
        return false;
    }

    float value = 0.0f;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.gps_noise.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(1), value)) model.navigation.gps_noise.rms_range_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(2), value)) model.navigation.gps_noise.semi_major_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(3), value)) model.navigation.gps_noise.semi_minor_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(4), value)) model.navigation.gps_noise.semi_major_orientation_deg.set(static_cast<Real>(wrap_360_deg(value)), now_us);
    if (parse_real(sentence.field(5), value)) model.navigation.gps_noise.latitude_error_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(6), value)) model.navigation.gps_noise.longitude_error_stddev_m.set(static_cast<Real>(value), now_us);
    if (parse_real(sentence.field(7), value)) model.navigation.gps_noise.altitude_error_stddev_m.set(static_cast<Real>(value), now_us);
    set_source(model.navigation.gps_noise.source, source);
    model.navigation.gps_noise.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gsv(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 3) {
        last_error_ = "short GSV";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_int32(sentence.field(0), integer_value)) model.navigation.gps_satellites_in_view.total_messages.set(integer_value, now_us);
    if (parse_int32(sentence.field(1), integer_value)) model.navigation.gps_satellites_in_view.message_number.set(integer_value, now_us);
    if (parse_int32(sentence.field(2), integer_value)) model.navigation.gps_satellites_in_view.satellites_in_view.set(integer_value, now_us);
    for (uint8_t index = 0; index < 4; ++index) {
        const uint8_t base = static_cast<uint8_t>(3 + index * 4);
        if (base >= sentence.field_count) break;
        if (parse_int32(sentence.field(base), integer_value)) model.navigation.gps_satellites_in_view.satellite_prn[index].set(integer_value, now_us);
        if (base + 1 < sentence.field_count && parse_real(sentence.field(base + 1), value)) {
            model.navigation.gps_satellites_in_view.elevation_deg[index].set(static_cast<Real>(value), now_us);
        }
        if (base + 2 < sentence.field_count && parse_real(sentence.field(base + 2), value)) {
            model.navigation.gps_satellites_in_view.azimuth_true_deg[index].set(static_cast<Real>(wrap_360_deg(value)), now_us);
        }
        if (base + 3 < sentence.field_count && parse_real(sentence.field(base + 3), value)) {
            model.navigation.gps_satellites_in_view.snr_db[index].set(static_cast<Real>(value), now_us);
        }
    }
    set_source(model.navigation.gps_satellites_in_view.source, source);
    model.navigation.gps_satellites_in_view.last_update_us = now_us;
    return true;
}

template<typename Model>
bool apply_gtd(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    float value = 0.0f;
    bool any = false;
    for (uint8_t index = 0; index < 5 && index < sentence.field_count; ++index) {
        if (parse_real(sentence.field(index), value)) {
            model.navigation.legacy_delta.value[index].set(static_cast<Real>(value), now_us);
            any = true;
        }
    }
    set_source(model.navigation.legacy_delta.source, source);
    model.navigation.legacy_delta.last_update_us = now_us;
    if (!any) last_error_ = "bad GTD";
    return any;
}

template<typename Model>
bool apply_gxa(const NmeaSentence& sentence, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
    if (sentence.field_count < 7) {
        last_error_ = "short GXA";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;
    if (parse_utc_time_of_day_s(sentence.field(0), value)) model.navigation.transit_fix.utc_time_s.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(1), sentence.field(2), value)) model.navigation.transit_fix.latitude_deg.set(static_cast<Real>(value), now_us);
    if (parse_lat_lon(sentence.field(3), sentence.field(4), value)) model.navigation.transit_fix.longitude_deg.set(static_cast<Real>(value), now_us);
    nmea_copy_span(model.navigation.transit_fix.waypoint_id, sizeof(model.navigation.transit_fix.waypoint_id), sentence.field(5));
    if (parse_int32(sentence.field(6), integer_value)) model.navigation.transit_fix.satellite_number.set(integer_value, now_us);
    set_source(model.navigation.transit_fix.source, source);
    model.navigation.transit_fix.last_update_us = now_us;
    return true;
}
