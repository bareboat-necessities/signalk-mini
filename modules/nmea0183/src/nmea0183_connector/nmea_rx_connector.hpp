#pragma once

#include <string.h>
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
        if (sentence_is(s, "DBT")) return apply_dbt(s, model, now_us, source);
        if (sentence_is(s, "DPT")) return apply_dpt(s, model, now_us, source);
        if (sentence_is(s, "RSA")) return apply_rsa(s, model, now_us, source);
        if (sentence_is(s, "APB")) return apply_apb(s, model, now_us, source);
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
        float lat = 0, lon = 0, alt = 0;
        if (parse_lat_lon(s.field(1), s.field(2), lat)) model.navigation.gps.fix_lat_deg.set(static_cast<Real>(lat), now_us);
        if (parse_lat_lon(s.field(3), s.field(4), lon)) model.navigation.gps.fix_lon_deg.set(static_cast<Real>(lon), now_us);
        if (parse_real(s.field(8), alt)) model.navigation.gps.fix_alt_m.set(static_cast<Real>(alt), now_us);
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
        if (s.field_count >= 4 && parse_real(s.field(2), depth)) {
            model.water.depth_m.set(static_cast<Real>(depth), now_us);
            set_source(model.water.depth_source, source);
            model.water.last_update_us = now_us;
            return true;
        }
        last_error_ = "bad DBT";
        return false;
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
        float xte = 0, track = 0;
        if (parse_left_right_signed(s.field(2), s.field(3), xte)) {
            if (xte > 0.15f) xte = 0.15f;
            if (xte < -0.15f) xte = -0.15f;
            model.navigation.apb.xte_nmi.set(static_cast<Real>(xte), now_us);
            any = true;
        }
        if (parse_real(s.field(12), track)) {
            model.navigation.apb.track_deg.set(static_cast<Real>(wrap_360_deg(track)), now_us);
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
