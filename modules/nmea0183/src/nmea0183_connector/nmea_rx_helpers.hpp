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

inline bool parse_north_south_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float value = 0.0f;
    if (!parse_real(magnitude, value) || side.empty()) return false;

    if (side[0] == 'N') out = value;
    else if (side[0] == 'S') out = -value;
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

} // namespace nmea0183_connector
