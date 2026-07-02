#pragma once

#include <string.h>
#include <stdint.h>
#include <ship_data_model.hpp>
#include "sentence_parser.hpp"

namespace nmea0183_connector {

inline float wrap_360_deg(float v) {
    while (v >= 360.0f) v -= 360.0f;
    while (v < 0.0f) v += 360.0f;
    return v;
}

inline float wrap_180_deg(float v) {
    v = wrap_360_deg(v + 180.0f) - 180.0f;
    if (v <= -180.0f) v += 360.0f;
    return v;
}

inline bool parse_int32(NmeaSpan field, int32_t& out) {
    float value = 0;
    if (!parse_real(field, value)) return false;
    out = static_cast<int32_t>(value);
    return true;
}

inline bool parse_north_south_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float v = 0.0f;
    if (!parse_real(magnitude, v) || side.empty()) return false;
    if (side[0] == 'N') out = v;
    else if (side[0] == 'S') out = -v;
    else return false;
    return true;
}

inline bool parse_utc_time_of_day_s(NmeaSpan utc_time, float& out_s) {
    if (utc_time.length < 6) return false;
    int hour = (utc_time[0]-'0')*10 + (utc_time[1]-'0');
    int minute = (utc_time[2]-'0')*10 + (utc_time[3]-'0');
    int second = (utc_time[4]-'0')*10 + (utc_time[5]-'0');
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) return false;
    out_s = static_cast<float>(hour * 3600 + minute * 60 + second);
    if (utc_time.length > 7 && utc_time[6] == '.') {
        float fraction = 0.0f;
        float scale = 0.1f;
        for (uint8_t i = 7; i < utc_time.length; ++i) {
            char c = utc_time[i];
            if (c < '0' || c > '9') break;
            fraction += static_cast<float>(c - '0') * scale;
            scale *= 0.1f;
        }
        out_s += fraction;
    }
    return true;
}

inline bool parse_rmc_timestamp_s(NmeaSpan utc_time, NmeaSpan date_ddmmyy, float& out_s) {
    if (utc_time.length < 6 || date_ddmmyy.length < 6) return false;
    int hour = (utc_time[0]-'0')*10 + (utc_time[1]-'0');
    int minute = (utc_time[2]-'0')*10 + (utc_time[3]-'0');
    int second = (utc_time[4]-'0')*10 + (utc_time[5]-'0');
    int day = (date_ddmmyy[0]-'0')*10 + (date_ddmmyy[1]-'0');
    int month = (date_ddmmyy[2]-'0')*10 + (date_ddmmyy[3]-'0');
    int yy = (date_ddmmyy[4]-'0')*10 + (date_ddmmyy[5]-'0');
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) return false;
    if (day < 1 || day > 31 || month < 1 || month > 12) return false;
    int year = yy >= 70 ? 1900 + yy : 2000 + yy;
    int y = year - (month <= 2);
    int era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = static_cast<unsigned>(y - era * 400);
    unsigned m = static_cast<unsigned>(month + (month > 2 ? -3 : 9));
    unsigned doy = (153 * m + 2) / 5 + static_cast<unsigned>(day) - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = era * 146097L + static_cast<long>(doe) - 719468L;
    out_s = static_cast<float>(days * 86400L + hour * 3600 + minute * 60 + second);
    return true;
}

inline bool parse_distance_nmi(NmeaSpan value, NmeaSpan unit, float& out_nmi) {
    float v = 0;
    if (!parse_real(value, v)) return false;
    const char u = unit.empty() ? 'N' : unit[0];
    if (u == 'N') out_nmi = v;
    else if (u == 'K') out_nmi = v * 0.539956803f;
    else return false;
    return true;
}

inline bool parse_depth_m_from_triplet(const NmeaSentence& s, float& out_m) {
    if (s.field_count >= 4 && parse_real(s.field(2), out_m)) return true;
    float v = 0;
    if (s.field_count >= 2 && parse_real(s.field(0), v)) { out_m = v * 0.3048f; return true; }
    if (s.field_count >= 6 && parse_real(s.field(4), v)) { out_m = v * 1.8288f; return true; }
    return false;
}

template<typename Real = float>
class Nmea0183RxConnector {
public:
    Nmea0183RxConnector() : last_error_(""), last_apb_mode_(ship_data_model::AutopilotMode::gps) {
        last_apb_sender_id_[0] = last_apb_sender_id_[1] = last_apb_sender_id_[2] = '\0';
    }

    const char* last_error() const { return last_error_; }
    ship_data_model::AutopilotMode last_apb_mode() const { return last_apb_mode_; }
    const char* last_apb_sender_id() const { return last_apb_sender_id_; }

    bool apply_sentence(const NmeaSentence& s, ship_data_model::DataModel<Real>& model, uint64_t now_us) {
        return apply_sentence(s, model, now_us, ship_data_model::SensorSource::none);
    }

    bool apply_sentence(const NmeaSentence& s, ship_data_model::DataModel<Real>& model, uint64_t now_us, ship_data_model::SensorSource source) {
        last_error_ = "";
        if (!s.valid_checksum) { last_error_ = "invalid checksum"; return false; }
        if (sentence_is(s, "AAM")) return apply_aam(s, model, now_us, source);
        if (sentence_is(s, "ALM")) return apply_alm(s, model, now_us, source);
        if (sentence_is(s, "APA")) return apply_apa(s, model, now_us, source);
        if (sentence_is(s, "APB")) return apply_apb(s, model, now_us, source);
        if (sentence_is(s, "BOD")) return apply_bod_bww(s, model, now_us, source);
        if (sentence_is(s, "BWC")) return apply_bwc_bwr(s, model, now_us, source);
        if (sentence_is(s, "BWR")) return apply_bwc_bwr(s, model, now_us, source);
        if (sentence_is(s, "BWW")) return apply_bod_bww(s, model, now_us, source);
        if (sentence_is(s, "DBK")) return apply_depth_below_keel(s, model, now_us, source);
        if (sentence_is(s, "DBS")) return apply_depth_below_surface(s, model, now_us, source);
        if (sentence_is(s, "DBT")) return apply_dbt(s, model, now_us, source);
        if (sentence_is(s, "DCN")) return apply_dcn(s, model, now_us, source);
        if (sentence_is(s, "DPT")) return apply_dpt(s, model, now_us, source);
        if (sentence_is(s, "DTM")) return apply_dtm(s, model, now_us, source);
        if (sentence_is(s, "FSI")) return apply_fsi(s, model, now_us, source);
        if (sentence_is(s, "GBS")) return apply_gbs(s, model, now_us, source);
        if (sentence_is(s, "RMC")) return apply_rmc(s, model, now_us, source);
        if (sentence_is(s, "GGA")) return apply_gga(s, model, now_us, source);
        if (sentence_is(s, "GLL")) return apply_gll(s, model, now_us, source);
        if (sentence_is(s, "VTG")) return apply_vtg(s, model, now_us, source);
        if (sentence_is(s, "HDT")) return apply_heading(s, model, now_us, "bad HDT");
        if (sentence_is(s, "HDM")) return apply_heading(s, model, now_us, "bad HDM");
        if (sentence_is(s, "HDG")) return apply_hdg(s, model, now_us);
        if (sentence_is(s, "MWV")) return apply_mwv(s, model, now_us, source);
        if (sentence_is(s, "MWD")) return apply_mwd(s, model, now_us, source);
        if (sentence_is(s, "VWR")) return apply_vwr(s, model, now_us, source, false);
        if (sentence_is(s, "VWT")) return apply_vwr(s, model, now_us, source, true);
        if (sentence_is(s, "VHW")) return apply_vhw(s, model, now_us, source);
        if (sentence_is(s, "LWY")) return apply_lwy(s, model, now_us, source);
        if (sentence_is(s, "RSA")) return apply_rsa(s, model, now_us, source);
        if (sentence_is(s, "RMB")) return apply_rmb(s, model, now_us, source);
        if (sentence_is(s, "XTE")) return apply_xte(s, model, now_us, source);
        if (sentence_is(s, "XDR")) return apply_xdr(s, model, now_us);
        if (sentence_is(s, "ROT")) return apply_rot(s, model, now_us);
        last_error_ = "unsupported sentence";
        return false;
    }

private:
    const char* last_error_;
    ship_data_model::AutopilotMode last_apb_mode_;
    char last_apb_sender_id_[3];

    template<typename Setting>
    void set_source(Setting& setting, ship_data_model::SensorSource source) {
        if (source != ship_data_model::SensorSource::none) setting.value = source;
    }

    template<typename Model>
    bool apply_aam(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 5) { last_error_ = "short AAM"; return false; }
        float radius = 0;
        model.navigation.waypoint_arrival.arrival_circle_entered.value = s.field(0)[0] == 'A';
        model.navigation.waypoint_arrival.perpendicular_passed.value = s.field(1)[0] == 'A';
        if (parse_distance_nmi(s.field(2), s.field(3), radius)) model.navigation.waypoint_arrival.arrival_radius_nmi.set(static_cast<Real>(radius), now_us);
        nmea_copy_span(model.navigation.waypoint_arrival.waypoint_id, sizeof(model.navigation.waypoint_arrival.waypoint_id), s.field(4));
        set_source(model.navigation.waypoint_arrival.source, source);
        model.navigation.waypoint_arrival.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_alm(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 15) { last_error_ = "short ALM"; return false; }
        int32_t i = 0;
        float v = 0;
        if (parse_int32(s.field(0), i)) model.navigation.gps_almanac.total_messages.set(i, now_us);
        if (parse_int32(s.field(1), i)) model.navigation.gps_almanac.message_number.set(i, now_us);
        if (parse_int32(s.field(2), i)) model.navigation.gps_almanac.satellite_prn.set(i, now_us);
        if (parse_int32(s.field(3), i)) model.navigation.gps_almanac.gps_week.set(i, now_us);
        if (parse_int32(s.field(4), i)) model.navigation.gps_almanac.sv_health.set(i, now_us);
        if (parse_real(s.field(5), v)) model.navigation.gps_almanac.eccentricity.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(6), v)) model.navigation.gps_almanac.reference_time_s.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(7), v)) model.navigation.gps_almanac.inclination_rad.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(8), v)) model.navigation.gps_almanac.right_ascension_rate_rad_s.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(9), v)) model.navigation.gps_almanac.sqrt_semi_major_axis.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(10), v)) model.navigation.gps_almanac.argument_of_perigee_rad.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(11), v)) model.navigation.gps_almanac.longitude_ascension_node_rad.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(12), v)) model.navigation.gps_almanac.mean_anomaly_rad.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(13), v)) model.navigation.gps_almanac.clock_f0_s.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(14), v)) model.navigation.gps_almanac.clock_f1_s_s.set(static_cast<Real>(v), now_us);
        set_source(model.navigation.gps_almanac.source, source);
        model.navigation.gps_almanac.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_dcn(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 16) { last_error_ = "short DCN"; return false; }
        float v = 0;
        int32_t i = 0;
        nmea_copy_span(model.navigation.decca.chain_id, sizeof(model.navigation.decca.chain_id), s.field(0));
        nmea_copy_span(model.navigation.decca.red_zone, sizeof(model.navigation.decca.red_zone), s.field(1));
        if (parse_real(s.field(2), v)) model.navigation.decca.red_line_of_position.set(static_cast<Real>(v), now_us);
        model.navigation.decca.red_master_status = s.field(3)[0];
        nmea_copy_span(model.navigation.decca.green_zone, sizeof(model.navigation.decca.green_zone), s.field(4));
        if (parse_real(s.field(5), v)) model.navigation.decca.green_line_of_position.set(static_cast<Real>(v), now_us);
        model.navigation.decca.green_master_status = s.field(6)[0];
        nmea_copy_span(model.navigation.decca.purple_zone, sizeof(model.navigation.decca.purple_zone), s.field(7));
        if (parse_real(s.field(8), v)) model.navigation.decca.purple_line_of_position.set(static_cast<Real>(v), now_us);
        model.navigation.decca.purple_master_status = s.field(9)[0];
        model.navigation.decca.red_line_navigation_use = s.field(10)[0];
        model.navigation.decca.green_line_navigation_use = s.field(11)[0];
        model.navigation.decca.purple_line_navigation_use = s.field(12)[0];
        if (parse_distance_nmi(s.field(13), s.field(14), v)) model.navigation.decca.position_uncertainty_nmi.set(static_cast<Real>(v), now_us);
        if (parse_int32(s.field(15), i)) model.navigation.decca.fix_data_basis.set(i, now_us);
        set_source(model.navigation.decca.source, source);
        model.navigation.decca.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_dtm(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 8) { last_error_ = "short DTM"; return false; }
        float v = 0;
        nmea_copy_span(model.navigation.datum.local_datum_code, sizeof(model.navigation.datum.local_datum_code), s.field(0));
        nmea_copy_span(model.navigation.datum.local_datum_subcode, sizeof(model.navigation.datum.local_datum_subcode), s.field(1));
        if (parse_north_south_signed(s.field(2), s.field(3), v)) model.navigation.datum.latitude_offset_min.set(static_cast<Real>(v), now_us);
        if (parse_east_west_signed(s.field(4), s.field(5), v)) model.navigation.datum.longitude_offset_min.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(6), v)) model.navigation.datum.altitude_offset_m.set(static_cast<Real>(v), now_us);
        nmea_copy_span(model.navigation.datum.reference_datum_code, sizeof(model.navigation.datum.reference_datum_code), s.field(7));
        set_source(model.navigation.datum.source, source);
        model.navigation.datum.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_fsi(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 4) { last_error_ = "short FSI"; return false; }
        float v = 0;
        int32_t i = 0;
        if (parse_real(s.field(0), v)) model.navigation.radio_frequency_set.transmitting_frequency_hz.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(1), v)) model.navigation.radio_frequency_set.receiving_frequency_hz.set(static_cast<Real>(v), now_us);
        model.navigation.radio_frequency_set.communication_mode = s.field(2)[0];
        if (parse_int32(s.field(3), i)) model.navigation.radio_frequency_set.power_level.set(i, now_us);
        set_source(model.navigation.radio_frequency_set.source, source);
        model.navigation.radio_frequency_set.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_gbs(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 8) { last_error_ = "short GBS"; return false; }
        float v = 0;
        int32_t i = 0;
        if (parse_utc_time_of_day_s(s.field(0), v)) model.navigation.gps_fault.utc_time_s.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(1), v)) model.navigation.gps_fault.expected_error_lat_m.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(2), v)) model.navigation.gps_fault.expected_error_lon_m.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(3), v)) model.navigation.gps_fault.expected_error_alt_m.set(static_cast<Real>(v), now_us);
        if (parse_int32(s.field(4), i)) model.navigation.gps_fault.failed_satellite_prn.set(i, now_us);
        if (parse_real(s.field(5), v)) model.navigation.gps_fault.missed_detection_probability.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(6), v)) model.navigation.gps_fault.failed_satellite_bias_m.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(7), v)) model.navigation.gps_fault.failed_satellite_bias_stddev_m.set(static_cast<Real>(v), now_us);
        set_source(model.navigation.gps_fault.source, source);
        model.navigation.gps_fault.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_apa(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 10) { last_error_ = "short APA"; return false; }
        float xte = 0, bearing = 0;
        if (parse_distance_nmi(s.field(2), s.field(4), xte)) {
            if (s.field(3)[0] == 'L') xte = -xte;
            else if (s.field(3)[0] != 'R') { last_error_ = "bad APA steer direction"; return false; }
            model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
        }
        model.navigation.apb.arrival_circle_entered.value = s.field(5)[0] == 'A';
        model.navigation.apb.perpendicular_passed.value = s.field(6)[0] == 'A';
        if (parse_real(s.field(7), bearing)) model.navigation.apb.origin_to_destination_bearing_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us);
        nmea_copy_span(model.navigation.apb.destination_id, sizeof(model.navigation.apb.destination_id), s.field(9));
        set_source(model.navigation.apb.source, source);
        model.navigation.apb.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_bod_bww(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 6) { last_error_ = sentence_is(s, "BOD") ? "short BOD" : "short BWW"; return false; }
        float bearing = 0;
        if (parse_real(s.field(0), bearing)) model.navigation.waypoint.bearing_true_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us);
        if (parse_real(s.field(2), bearing)) model.navigation.waypoint.bearing_magnetic_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us);
        nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), s.field(4));
        nmea_copy_span(model.navigation.waypoint.from_waypoint_id, sizeof(model.navigation.waypoint.from_waypoint_id), s.field(5));
        set_source(model.navigation.waypoint.source, source);
        model.navigation.waypoint.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_bwc_bwr(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 12) { last_error_ = sentence_is(s, "BWC") ? "short BWC" : "short BWR"; return false; }
        float v = 0;
        if (parse_utc_time_of_day_s(s.field(0), v)) model.navigation.waypoint.utc_time_s.set(static_cast<Real>(v), now_us);
        if (parse_lat_lon(s.field(1), s.field(2), v)) model.navigation.waypoint.latitude_deg.set(static_cast<Real>(v), now_us);
        if (parse_lat_lon(s.field(3), s.field(4), v)) model.navigation.waypoint.longitude_deg.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(5), v)) model.navigation.waypoint.bearing_true_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
        if (parse_real(s.field(7), v)) model.navigation.waypoint.bearing_magnetic_deg.set(static_cast<Real>(wrap_360_deg(v)), now_us);
        if (parse_distance_nmi(s.field(9), s.field(10), v)) model.navigation.waypoint.distance_nmi.set(static_cast<Real>(v), now_us);
        nmea_copy_span(model.navigation.waypoint.to_waypoint_id, sizeof(model.navigation.waypoint.to_waypoint_id), s.field(11));
        set_source(model.navigation.waypoint.source, source);
        model.navigation.waypoint.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_rmc(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 8 || s.field(1)[0] != 'A') { last_error_ = "invalid RMC"; return false; }
        float lat = 0, lon = 0, speed = 0, track = 0, timestamp = 0;
        if (parse_lat_lon(s.field(2), s.field(3), lat)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(lat), now_us);
        if (parse_lat_lon(s.field(4), s.field(5), lon)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(lon), now_us);
        if (parse_real(s.field(6), speed)) model.navigation.gps.speed_kn.set(static_cast<Real>(speed), now_us);
        if (parse_real(s.field(7), track)) model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(track)), now_us);
        if (s.field_count >= 9 && parse_rmc_timestamp_s(s.field(0), s.field(8), timestamp)) model.navigation.gps.timestamp_s.set(static_cast<Real>(timestamp), now_us);
        if (s.field_count >= 11) {
            float declination = 0;
            if (parse_east_west_signed(s.field(9), s.field(10), declination)) model.navigation.gps.declination_deg.set(static_cast<Real>(declination), now_us);
        }
        set_source(model.navigation.gps.source, source);
        model.navigation.gps.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_gga(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 9 || s.field(5)[0] == '0') { last_error_ = "invalid GGA"; return false; }
        float v = 0;
        int32_t i = 0;
        if (parse_utc_time_of_day_s(s.field(0), v)) model.navigation.gps.timestamp_s.set(static_cast<Real>(v), now_us);
        if (parse_lat_lon(s.field(1), s.field(2), v)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(v), now_us);
        if (parse_lat_lon(s.field(3), s.field(4), v)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(v), now_us);
        if (parse_int32(s.field(5), i)) model.navigation.gps.fix_quality.set(i, now_us);
        if (parse_int32(s.field(6), i)) model.navigation.gps.satellites_used.set(i, now_us);
        if (parse_real(s.field(7), v)) model.navigation.gps.hdop.set(static_cast<Real>(v), now_us);
        if (parse_real(s.field(8), v)) model.navigation.gps.fix_alt_m.set(static_cast<Real>(v), now_us);
        if (s.field_count >= 11 && parse_real(s.field(10), v)) model.navigation.gps.geoidal_separation_m.set(static_cast<Real>(v), now_us);
        if (s.field_count >= 13 && parse_real(s.field(12), v)) model.navigation.gps.dgps_age_s.set(static_cast<Real>(v), now_us);
        if (s.field_count >= 14 && parse_int32(s.field(13), i)) model.navigation.gps.dgps_station_id.set(i, now_us);
        set_source(model.navigation.gps.source, source);
        model.navigation.gps.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_gll(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 6 || s.field(5)[0] != 'A') { last_error_ = "bad GLL"; return false; }
        float lat = 0, lon = 0;
        if (parse_lat_lon(s.field(0), s.field(1), lat)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(lat), now_us);
        if (parse_lat_lon(s.field(2), s.field(3), lon)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(lon), now_us);
        set_source(model.navigation.gps.source, source);
        model.navigation.gps.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_vtg(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 7) { last_error_ = "short VTG"; return false; }
        float track = 0, speed = 0;
        if (parse_real(s.field(0), track)) model.navigation.gps.track_deg.set(static_cast<Real>(wrap_360_deg(track)), now_us);
        if (parse_knots(s.field(4), s.field(5), speed)) model.navigation.gps.speed_kn.set(static_cast<Real>(speed), now_us);
        else if (parse_knots(s.field(6), s.field(7), speed)) model.navigation.gps.speed_kn.set(static_cast<Real>(speed), now_us);
        set_source(model.navigation.gps.source, source);
        model.navigation.gps.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_heading(const NmeaSentence& s, Model& model, uint64_t now_us, const char* error) {
        float heading = 0;
        if (!parse_real(s.field(0), heading)) { last_error_ = error; return false; }
        model.imu.heading_deg.set(static_cast<Real>(wrap_360_deg(heading)), now_us);
        return true;
    }

    template<typename Model>
    bool apply_hdg(const NmeaSentence& s, Model& model, uint64_t now_us) {
        if (!apply_heading(s, model, now_us, "bad HDG")) return false;
        float variation = 0;
        if (s.field_count >= 5 && parse_east_west_signed(s.field(3), s.field(4), variation)) model.navigation.gps.declination_deg.set(static_cast<Real>(variation), now_us);
        return true;
    }

    template<typename Model>
    bool apply_mwv(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 5 || s.field(4)[0] != 'A') { last_error_ = "invalid MWV"; return false; }
        float angle = 0, speed = 0;
        if (!parse_real(s.field(0), angle) || !parse_knots(s.field(2), s.field(3), speed)) { last_error_ = "bad MWV"; return false; }
        if (s.field(1)[0] == 'T') return set_wind(model.wind.truewind, angle, speed, now_us, source);
        return set_wind(model.wind.apparent, angle, speed, now_us, source);
    }

    template<typename Wind>
    bool set_wind(Wind& wind, float angle, float speed, uint64_t now_us, ship_data_model::SensorSource source) {
        wind.direction_deg.set(wrap_180_deg(angle), now_us);
        wind.speed_kn.set(speed, now_us);
        set_source(wind.source, source);
        wind.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_mwd(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float direction = 0, speed = 0;
        if (s.field_count < 7 || !parse_real(s.field(0), direction) || !parse_knots(s.field(4), s.field(5), speed)) { last_error_ = "bad MWD"; return false; }
        model.wind.truewind.direction_deg.set(static_cast<Real>(wrap_180_deg(direction)), now_us);
        model.wind.truewind.speed_kn.set(static_cast<Real>(speed), now_us);
        set_source(model.wind.truewind.source, source);
        model.wind.truewind.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_vwr(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source, bool true_wind) {
        float angle = 0, speed = 0;
        if (!parse_left_right_signed(s.field(0), s.field(1), angle) || !parse_knots(s.field(2), s.field(3), speed)) { last_error_ = true_wind ? "bad VWT" : "bad VWR"; return false; }
        if (true_wind) return set_wind(model.wind.truewind, angle, speed, now_us, source);
        return set_wind(model.wind.apparent, angle, speed, now_us, source);
    }

    template<typename Model>
    bool apply_vhw(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float speed = 0;
        if (!parse_knots(s.field(4), s.field(5), speed)) { last_error_ = "bad VHW"; return false; }
        model.water.speed_kn.set(static_cast<Real>(speed), now_us);
        set_source(model.water.source, source);
        model.water.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_lwy(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float leeway = 0;
        if (s.field_count < 2 || s.field(0)[0] != 'A' || !parse_real(s.field(1), leeway)) { last_error_ = "bad LWY"; return false; }
        model.water.leeway_deg.set(static_cast<Real>(leeway), now_us);
        set_source(model.water.leeway_source, source);
        model.water.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_dbt(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float depth = 0;
        if (parse_depth_m_from_triplet(s, depth)) {
            model.water.depth_m.set(static_cast<Real>(depth), now_us);
            set_source(model.water.depth_source, source);
            model.water.last_update_us = now_us;
            return true;
        }
        last_error_ = "bad DBT";
        return false;
    }

    template<typename Model>
    bool apply_depth_below_keel(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float depth = 0;
        if (!parse_depth_m_from_triplet(s, depth)) { last_error_ = "bad DBK"; return false; }
        model.water.depth_below_keel_m.set(static_cast<Real>(depth), now_us);
        set_source(model.water.depth_source, source);
        model.water.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_depth_below_surface(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float depth = 0;
        if (!parse_depth_m_from_triplet(s, depth)) { last_error_ = "bad DBS"; return false; }
        model.water.depth_below_surface_m.set(static_cast<Real>(depth), now_us);
        set_source(model.water.depth_source, source);
        model.water.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_dpt(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float depth = 0, offset = 0;
        if (!parse_real(s.field(0), depth)) { last_error_ = "bad DPT"; return false; }
        model.water.depth_m.set(static_cast<Real>(depth), now_us);
        if (parse_real(s.field(1), offset)) model.water.depth_offset_m.set(static_cast<Real>(offset), now_us);
        set_source(model.water.depth_source, source);
        model.water.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_rsa(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float angle = 0;
        if (s.field_count >= 2 && s.field(1)[0] == 'A' && parse_real(s.field(0), angle)) {
            model.rudder.angle_deg.set(static_cast<Real>(-angle), now_us);
        } else if (s.field_count >= 4 && s.field(3)[0] == 'A' && parse_real(s.field(2), angle)) {
            model.rudder.angle_deg.set(static_cast<Real>(-angle), now_us);
        } else { last_error_ = "bad RSA"; return false; }
        set_source(model.rudder.source, source);
        model.rudder.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_xte(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        float xte = 0;
        if (s.field_count < 5 || s.field(0)[0] != 'A' || s.field(1)[0] != 'A' || !parse_left_right_signed(s.field(2), s.field(3), xte)) { last_error_ = "bad XTE"; return false; }
        model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
        set_source(model.navigation.apb.source, source);
        model.navigation.apb.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_apb(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 14) { last_error_ = "short APB"; return false; }
        bool any = false;
        float xte = 0, bearing = 0, track = 0;
        if (parse_left_right_signed(s.field(2), s.field(3), xte)) {
            if (xte > 0.15f) xte = 0.15f;
            if (xte < -0.15f) xte = -0.15f;
            model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
            any = true;
        }
        model.navigation.apb.arrival_circle_entered.value = s.field(5)[0] == 'A';
        model.navigation.apb.perpendicular_passed.value = s.field(6)[0] == 'A';
        if (parse_real(s.field(7), bearing)) { model.navigation.apb.origin_to_destination_bearing_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us); any = true; }
        nmea_copy_span(model.navigation.apb.destination_id, sizeof(model.navigation.apb.destination_id), s.field(9));
        if (parse_real(s.field(10), bearing)) { model.navigation.apb.present_to_destination_bearing_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us); any = true; }
        if (parse_real(s.field(12), track)) {
            model.navigation.apb.track_deg.set(static_cast<Real>(wrap_360_deg(track)), now_us);
            model.navigation.apb.heading_to_steer_deg.set(static_cast<Real>(wrap_360_deg(track)), now_us);
            any = true;
        }
        last_apb_mode_ = s.field(13)[0] == 'M' ? ship_data_model::AutopilotMode::compass : ship_data_model::AutopilotMode::gps;
        model.navigation.apb.mode_hint.value = last_apb_mode_;
        last_apb_sender_id_[0] = s.talker[0];
        last_apb_sender_id_[1] = s.talker[1];
        last_apb_sender_id_[2] = '\0';
        model.navigation.apb.sender_id[0] = last_apb_sender_id_[0];
        model.navigation.apb.sender_id[1] = last_apb_sender_id_[1];
        model.navigation.apb.sender_id[2] = '\0';
        set_source(model.navigation.apb.source, source);
        model.navigation.apb.last_update_us = now_us;
        if (!any) last_error_ = "bad APB";
        return any;
    }

    template<typename Model>
    bool apply_rmb(const NmeaSentence& s, Model& model, uint64_t now_us, ship_data_model::SensorSource source) {
        if (s.field_count < 13 || s.field(0)[0] != 'A') { last_error_ = "bad RMB"; return false; }
        float xte = 0, lat = 0, lon = 0, range = 0, bearing = 0, vmg = 0;
        if (parse_left_right_signed(s.field(1), s.field(2), xte)) {
            model.navigation.rmb.xte_nmi.set(static_cast<Real>(xte), now_us);
            model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
        }
        nmea_copy_span(model.navigation.rmb.origin_id, sizeof(model.navigation.rmb.origin_id), s.field(3));
        nmea_copy_span(model.navigation.rmb.destination_id, sizeof(model.navigation.rmb.destination_id), s.field(4));
        if (parse_lat_lon(s.field(5), s.field(6), lat)) model.navigation.rmb.destination_lat_deg.set(static_cast<Real>(lat), now_us);
        if (parse_lat_lon(s.field(7), s.field(8), lon)) model.navigation.rmb.destination_lon_deg.set(static_cast<Real>(lon), now_us);
        if (parse_real(s.field(9), range)) model.navigation.rmb.range_nmi.set(static_cast<Real>(range), now_us);
        if (parse_real(s.field(10), bearing)) {
            model.navigation.rmb.bearing_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us);
            model.navigation.apb.track_deg.set(static_cast<Real>(wrap_360_deg(bearing)), now_us);
        }
        if (parse_real(s.field(11), vmg)) model.navigation.rmb.closing_velocity_kn.set(static_cast<Real>(vmg), now_us);
        model.navigation.rmb.arrived.value = s.field(12)[0] == 'A';
        set_source(model.navigation.rmb.source, source);
        set_source(model.navigation.apb.source, source);
        model.navigation.rmb.last_update_us = now_us;
        model.navigation.apb.last_update_us = now_us;
        return true;
    }

    template<typename Model>
    bool apply_xdr(const NmeaSentence& s, Model& model, uint64_t now_us) {
        bool any = false;
        for (uint8_t i = 0; i + 3 < s.field_count; i += 4) {
            float v = 0;
            if (s.field(i)[0] == 'A' && parse_real(s.field(i + 1), v) && s.field(i + 2)[0] == 'D') {
                if (nmea_span_equals(s.field(i + 3), "PTCH")) { model.imu.pitch_deg.set(static_cast<Real>(v), now_us); any = true; }
                else if (nmea_span_equals(s.field(i + 3), "ROLL")) { model.imu.roll_deg.set(static_cast<Real>(v), now_us); any = true; }
            }
        }
        if (!any) last_error_ = "bad XDR";
        return any;
    }

    template<typename Model>
    bool apply_rot(const NmeaSentence& s, Model& model, uint64_t now_us) {
        float rate_min = 0;
        if (s.field_count < 2 || s.field(1)[0] != 'A' || !parse_real(s.field(0), rate_min)) { last_error_ = "bad ROT"; return false; }
        model.imu.heading_rate_lowpass_deg_s.set(static_cast<Real>(rate_min / 60.0f), now_us);
        return true;
    }
};

} // namespace nmea0183_connector
