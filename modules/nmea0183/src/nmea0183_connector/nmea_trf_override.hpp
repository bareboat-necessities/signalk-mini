#pragma once

// Included inside Nmea0183RxConnector.

template<typename Model>
bool apply_trf_full(const NmeaSentence& sentence,
                    Model& model,
                    uint64_t now_us,
                    ship_data_model::SensorSource source) {
    if (sentence.field_count < 12) {
        last_error_ = "short TRF";
        return false;
    }

    float value = 0.0f;
    int32_t integer_value = 0;

    if (parse_utc_time_of_day_s(sentence.field(0), value)) {
        model.nav.transit_fix.utc_time_s.set(static_cast<Real>(value), now_us);
    }
    nmea_copy_span(model.nav.transit_fix.date_ddmmyy,
                   sizeof(model.nav.transit_fix.date_ddmmyy),
                   sentence.field(1));
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
