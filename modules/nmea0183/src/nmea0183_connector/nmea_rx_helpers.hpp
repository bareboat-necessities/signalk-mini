#pragma once

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
    float value = 0.0f;
    if (!parse_real(field, value)) return false;
    out = static_cast<int32_t>(value);
    return true;
}

inline bool parse_nmea_hhmm_time_s(NmeaSpan field, float& out_s) {
    if (field.length < 4 || !field.data) return false;
    const int hour = (field[0] - '0') * 10 + (field[1] - '0');
    const int minute = (field[2] - '0') * 10 + (field[3] - '0');
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59) return false;
    out_s = static_cast<float>(hour * 3600 + minute * 60);
    return true;
}

inline bool parse_north_south_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float value = 0.0f;
    if (!parse_real(magnitude, value) || side.empty()) return false;

    if (side[0] == 'N') out = value;
    else if (side[0] == 'S') out = -value;
    else return false;
    return true;
}

inline bool parse_east_west_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float value = 0.0f;
    if (!parse_real(magnitude, value) || side.empty()) return false;

    if (side[0] == 'E') out = value;
    else if (side[0] == 'W') out = -value;
    else return false;
    return true;
}

inline bool parse_left_right_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float value = 0.0f;
    if (!parse_real(magnitude, value) || side.empty()) return false;

    if (side[0] == 'R') out = value;
    else if (side[0] == 'L') out = -value;
    else return false;
    return true;
}

inline bool parse_lat_lon(NmeaSpan ddmm, NmeaSpan hemisphere, float& out_deg) {
    if (!ddmm.data || ddmm.length < 4 || hemisphere.empty()) return false;
    const char h = hemisphere[0];
    const uint8_t deg_digits = (h == 'E' || h == 'W') ? 3 : 2;
    if (ddmm.length <= deg_digits + 1) return false;

    int degrees = 0;
    for (uint8_t i = 0; i < deg_digits; ++i) {
        if (ddmm[i] < '0' || ddmm[i] > '9') return false;
        degrees = degrees * 10 + (ddmm[i] - '0');
    }
    float minutes = 0.0f;
    if (!parse_real(NmeaSpan(ddmm.data + deg_digits, ddmm.length - deg_digits), minutes)) return false;
    out_deg = static_cast<float>(degrees) + minutes / 60.0f;
    if (h == 'S' || h == 'W') out_deg = -out_deg;
    else if (h != 'N' && h != 'E') return false;
    return true;
}

inline bool parse_knots(NmeaSpan value, NmeaSpan unit, float& out_kn) {
    float v = 0.0f;
    if (!parse_real(value, v)) return false;
    const char u = unit.empty() ? 'N' : unit[0];
    if (u == 'N') out_kn = v;
    else if (u == 'K') out_kn = v * 0.539956803f;
    else if (u == 'M') out_kn = v * 1.94384449f;
    else return false;
    return true;
}

inline bool parse_utc_time_of_day_s(NmeaSpan utc_time, float& out_s) {
    if (utc_time.length < 6) return false;

    const int hour = (utc_time[0] - '0') * 10 + (utc_time[1] - '0');
    const int minute = (utc_time[2] - '0') * 10 + (utc_time[3] - '0');
    const int second = (utc_time[4] - '0') * 10 + (utc_time[5] - '0');
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) return false;

    out_s = static_cast<float>(hour * 3600 + minute * 60 + second);
    if (utc_time.length > 7 && utc_time[6] == '.') {
        float fraction = 0.0f;
        float scale = 0.1f;
        for (uint8_t i = 7; i < utc_time.length; ++i) {
            const char c = utc_time[i];
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

    const int hour = (utc_time[0] - '0') * 10 + (utc_time[1] - '0');
    const int minute = (utc_time[2] - '0') * 10 + (utc_time[3] - '0');
    const int second = (utc_time[4] - '0') * 10 + (utc_time[5] - '0');
    const int day = (date_ddmmyy[0] - '0') * 10 + (date_ddmmyy[1] - '0');
    const int month = (date_ddmmyy[2] - '0') * 10 + (date_ddmmyy[3] - '0');
    const int yy = (date_ddmmyy[4] - '0') * 10 + (date_ddmmyy[5] - '0');
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 60) return false;
    if (day < 1 || day > 31 || month < 1 || month > 12) return false;

    const int year = yy >= 70 ? 1900 + yy : 2000 + yy;
    const int y = year - (month <= 2);
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned m = static_cast<unsigned>(month + (month > 2 ? -3 : 9));
    const unsigned doy = (153 * m + 2) / 5 + static_cast<unsigned>(day) - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long days = era * 146097L + static_cast<long>(doe) - 719468L;
    out_s = static_cast<float>(days * 86400L + hour * 3600 + minute * 60 + second);
    return true;
}

inline bool parse_distance_nmi(NmeaSpan value, NmeaSpan unit, float& out_nmi) {
    float v = 0.0f;
    if (!parse_real(value, v)) return false;

    const char u = unit.empty() ? 'N' : unit[0];
    if (u == 'N') out_nmi = v;
    else if (u == 'K') out_nmi = v * 0.539956803f;
    else return false;
    return true;
}

inline bool parse_depth_m(NmeaSpan value, NmeaSpan unit, float& out_m) {
    float v = 0.0f;
    if (!parse_real(value, v)) return false;
    const char u = unit.empty() ? 'M' : unit[0];
    if (u == 'M') out_m = v;
    else if (u == 'f' || u == 'F') out_m = v * 0.3048f;
    else if (u == 'm') out_m = v * 1852.0f;
    else return false;
    return true;
}

inline bool parse_depth_m_from_triplet(const NmeaSentence& sentence, float& out_m) {
    if (sentence.field_count >= 4 && parse_real(sentence.field(2), out_m)) return true;

    float value = 0.0f;
    if (sentence.field_count >= 2 && parse_real(sentence.field(0), value)) {
        out_m = value * 0.3048f;
        return true;
    }
    if (sentence.field_count >= 6 && parse_real(sentence.field(4), value)) {
        out_m = value * 1.8288f;
        return true;
    }
    return false;
}

inline bool parse_dsc_compact_position(NmeaSpan code, float& lat_deg, float& lon_deg) {
    if (code.length < 10 || !code.data) return false;
    for (uint8_t i = 0; i < 10; ++i) if (code[i] < '0' || code[i] > '9') return false;
    const int quadrant = code[0] - '0';
    const int lat_d = (code[1] - '0') * 10 + (code[2] - '0');
    const int lat_m = (code[3] - '0') * 10 + (code[4] - '0');
    const int lon_d = (code[5] - '0') * 100 + (code[6] - '0') * 10 + (code[7] - '0');
    const int lon_m = (code[8] - '0') * 10 + (code[9] - '0');
    if (quadrant < 0 || quadrant > 3 || lat_d > 90 || lat_m > 59 || lon_d > 180 || lon_m > 59) return false;
    lat_deg = static_cast<float>(lat_d) + static_cast<float>(lat_m) / 60.0f;
    lon_deg = static_cast<float>(lon_d) + static_cast<float>(lon_m) / 60.0f;
    if (quadrant == 2 || quadrant == 3) lat_deg = -lat_deg;
    if (quadrant == 1 || quadrant == 3) lon_deg = -lon_deg;
    return true;
}

} // namespace nmea0183_connector
