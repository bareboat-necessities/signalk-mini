#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ship_data_model.hpp>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "nmea0183_helpers.hpp"

namespace nmea0183_connector {

inline void format_real(char* out, size_t out_size, float value, unsigned decimals = 1) {
    if (!out || out_size == 0) return;
#ifdef ARDUINO
    dtostrf(value, 0, decimals, out);
    char* first = out;
    while (*first == ' ') ++first;
    if (first != out) memmove(out, first, strlen(first) + 1);
#else
    snprintf(out, out_size, "%.*f", static_cast<int>(decimals), static_cast<double>(value));
#endif
}

inline float format_wrap_360_deg(float angle_deg) {
    while (angle_deg >= 360.0f) angle_deg -= 360.0f;
    while (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg;
}

inline size_t make_sentence(char* out, size_t out_size, const char* body, char start_char = '$') {
    if (!out || out_size == 0 || !body) return 0;
    uint8_t cs = nmea_checksum_body(body);
    int n = snprintf(out, out_size, "%c%s*%c%c\r\n", start_char, body, to_hex(cs >> 4), to_hex(cs));
    if (n < 0) return 0;
    return static_cast<size_t>(n) < out_size ? static_cast<size_t>(n) : out_size - 1;
}

inline size_t make_hdt(char* out, size_t out_size, float heading_deg, const char* talker = "II") {
    char h[24];
    char body[64];
    format_real(h, sizeof(h), heading_deg, 1);
    snprintf(body, sizeof(body), "%sHDT,%s,T", talker, h);
    return make_sentence(out, out_size, body);
}

inline size_t make_hdm(char* out, size_t out_size, float heading_deg, const char* talker = "II") {
    char h[24];
    char body[64];
    format_real(h, sizeof(h), heading_deg, 1);
    snprintf(body, sizeof(body), "%sHDM,%s,M", talker, h);
    return make_sentence(out, out_size, body);
}

inline size_t make_mwv(char* out, size_t out_size, float angle_deg, float speed_kn, bool apparent = true, const char* talker = "II") {
    char a[24];
    char s[24];
    char body[80];
    format_real(a, sizeof(a), angle_deg, 1);
    format_real(s, sizeof(s), speed_kn, 1);
    snprintf(body, sizeof(body), "%sMWV,%s,%c,%s,N,A", talker, a, apparent ? 'R' : 'T', s);
    return make_sentence(out, out_size, body);
}

inline size_t make_rsa(char* out, size_t out_size, float nmea_rudder_deg, const char* talker = "II") {
    char r[24];
    char body[80];
    format_real(r, sizeof(r), nmea_rudder_deg, 1);
    snprintf(body, sizeof(body), "%sRSA,%s,A,,V", talker, r);
    return make_sentence(out, out_size, body);
}

inline size_t make_vhw(char* out, size_t out_size, float water_speed_kn, const char* talker = "II") {
    char s[24];
    char body[80];
    format_real(s, sizeof(s), water_speed_kn, 1);
    snprintf(body, sizeof(body), "%sVHW,,,,,%s,N,,K", talker, s);
    return make_sentence(out, out_size, body);
}

inline size_t make_xdr_pitch(char* out, size_t out_size, float pitch_deg, const char* talker = "AP") {
    char v[24];
    char body[80];
    format_real(v, sizeof(v), pitch_deg, 3);
    snprintf(body, sizeof(body), "%sXDR,A,%s,D,PTCH", talker, v);
    return make_sentence(out, out_size, body);
}

inline size_t make_xdr_roll(char* out, size_t out_size, float roll_deg, const char* talker = "AP") {
    char v[24];
    char body[80];
    format_real(v, sizeof(v), roll_deg, 3);
    snprintf(body, sizeof(body), "%sXDR,A,%s,D,ROLL", talker, v);
    return make_sentence(out, out_size, body);
}

inline size_t make_rot(char* out, size_t out_size, float rate_deg_min, const char* talker = "AP") {
    char v[24];
    char body[80];
    format_real(v, sizeof(v), rate_deg_min, 3);
    snprintf(body, sizeof(body), "%sROT,%s,A", talker, v);
    return make_sentence(out, out_size, body);
}

inline void format_lat_lon_ddmm(char* out, size_t out_size, float degrees_value, char positive_hemi, char negative_hemi, char& hemi) {
    if (!out || out_size == 0) return;
    float a = degrees_value < 0.0f ? -degrees_value : degrees_value;
    int deg_i = static_cast<int>(a);
    float minutes = (a - static_cast<float>(deg_i)) * 60.0f;
    hemi = degrees_value >= 0.0f ? positive_hemi : negative_hemi;
    snprintf(out, out_size, "%02d%07.4f", deg_i, static_cast<double>(minutes));
}

inline size_t make_rmc(char* out,
                       size_t out_size,
                       const char* sentence_id,
                       const char* utc_time,
                       bool valid,
                       float lat_deg,
                       float lon_deg,
                       float speed_kn,
                       float track_deg,
                       const char* date_ddmmyy) {
    char lat[20];
    char lon[20];
    char speed[20];
    char track[20];
    char body[128];
    char ns = 'N';
    char ew = 'E';
    format_lat_lon_ddmm(lat, sizeof(lat), lat_deg, 'N', 'S', ns);
    format_lat_lon_ddmm(lon, sizeof(lon), lon_deg, 'E', 'W', ew);
    format_real(speed, sizeof(speed), speed_kn, 2);
    float tr = track_deg > 0.0f ? track_deg : 360.0f + track_deg;
    format_real(track, sizeof(track), tr, 2);
    snprintf(body, sizeof(body), "%s,%s,%c,%s,%c,%s,%c,%s,%s,%s,,,A",
             sentence_id ? sentence_id : "GPRMC",
             utc_time ? utc_time : "000000.000",
             valid ? 'A' : 'V',
             lat, ns,
             lon, ew,
             speed,
             track,
             date_ddmmyy ? date_ddmmyy : "010180");
    return make_sentence(out, out_size, body);
}

template<typename Real>
inline size_t make_ap_rmc(char* out,
                          size_t out_size,
                          const ship_data_model::DataModel<Real>& model,
                          const char* utc_time,
                          const char* date_ddmmyy,
                          const char* gps_id = "APRMC") {
    bool valid = model.navigation.gps.fix_lat_deg.valid && model.navigation.gps.fix_lon_deg.valid;
    float lat = static_cast<float>(model.navigation.gps.fix_lat_deg.value);
    float lon = static_cast<float>(model.navigation.gps.fix_lon_deg.value);
    float speed = model.navigation.gps.speed_kn.valid ? static_cast<float>(model.navigation.gps.speed_kn.value) : 0.0f;
    float track = model.navigation.gps.track_deg.valid ? static_cast<float>(model.navigation.gps.track_deg.value) : 0.0f;
    return make_rmc(out, out_size, gps_id, utc_time, valid, lat, lon, speed, track, date_ddmmyy);
}

inline size_t make_ap_hdm(char* out, size_t out_size, float heading_lowpass_deg) {
    return make_hdm(out, out_size, heading_lowpass_deg, "AP");
}

inline size_t make_ap_mwv(char* out, size_t out_size, float direction_deg, float speed_kn, bool apparent = true) {
    return make_mwv(out, out_size, format_wrap_360_deg(direction_deg), speed_kn, apparent, "AP");
}

inline size_t make_ap_rsa(char* out, size_t out_size, float rudder_deg) {
    return make_rsa(out, out_size, -rudder_deg, "AP");
}

} // namespace nmea0183_connector
