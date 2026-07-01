#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace nmea0183_connector {

struct NmeaSpan {
    const char* data;
    uint8_t length;

    NmeaSpan() : data(nullptr), length(0) {}
    NmeaSpan(const char* p, size_t n)
        : data(p), length(n > 255u ? 255u : static_cast<uint8_t>(n)) {}

    bool empty() const { return length == 0; }
    char operator[](size_t index) const { return index < length && data ? data[index] : '\0'; }
};

inline bool nmea_span_equals(NmeaSpan span, const char* text) {
    if (!text) return false;
    size_t i = 0;
    for (; text[i] && i < span.length; ++i) {
        if (!span.data || span.data[i] != text[i]) return false;
    }
    return i == span.length && text[i] == '\0';
}

inline void nmea_copy_span(char* out, size_t out_size, NmeaSpan span) {
    if (!out || out_size == 0) return;
    size_t n = span.length;
    if (n + 1 > out_size) n = out_size - 1;
    if (n && span.data) memcpy(out, span.data, n);
    out[n] = '\0';
}

inline uint8_t from_hex(char c) {
    if (c >= '0' && c <= '9') return static_cast<uint8_t>(c - '0');
    if (c >= 'A' && c <= 'F') return static_cast<uint8_t>(10 + c - 'A');
    if (c >= 'a' && c <= 'f') return static_cast<uint8_t>(10 + c - 'a');
    return 0xff;
}

inline char to_hex(uint8_t v) {
    static const char* hex = "0123456789ABCDEF";
    return hex[v & 0x0f];
}

inline uint8_t nmea_checksum_body(const char* body) {
    uint8_t cs = 0;
    if (!body) return cs;
    while (*body) cs ^= static_cast<uint8_t>(*body++);
    return cs;
}

inline uint8_t nmea_checksum_range(const char* begin, const char* end) {
    uint8_t cs = 0;
    while (begin && begin < end) cs ^= static_cast<uint8_t>(*begin++);
    return cs;
}

inline bool parse_checksum_hex(const char* two_hex, uint8_t& out) {
    if (!two_hex || !two_hex[0] || !two_hex[1]) return false;
    uint8_t hi = from_hex(two_hex[0]);
    uint8_t lo = from_hex(two_hex[1]);
    if (hi > 0x0f || lo > 0x0f) return false;
    out = static_cast<uint8_t>((hi << 4) | lo);
    return true;
}

inline bool verify_nmea_checksum(const char* line) {
    if (!line || (*line != '$' && *line != '!')) return false;
    const char* star = strchr(line, '*');
    if (!star) return false;
    uint8_t expected = 0;
    if (!parse_checksum_hex(star + 1, expected)) return false;
    return nmea_checksum_range(line + 1, star) == expected;
}

inline bool parse_real(const char* field, size_t length, float& out) {
    if (!field || length == 0) return false;
    char local[32];
    size_t n = length;
    if (n >= sizeof(local)) n = sizeof(local) - 1;
    memcpy(local, field, n);
    local[n] = '\0';
    char* end = nullptr;
    double v = strtod(local, &end);
    if (end == local) return false;
    out = static_cast<float>(v);
    return true;
}

inline bool parse_real(NmeaSpan field, float& out) {
    return parse_real(field.data, field.length, out);
}

inline bool parse_real(const char* field, float& out) {
    return field ? parse_real(field, strlen(field), out) : false;
}

inline bool parse_char_field(NmeaSpan field, char& out) {
    if (field.empty()) return false;
    out = field[0];
    return true;
}

inline bool parse_char_field(const char* field, char& out) {
    if (!field || !field[0]) return false;
    out = field[0];
    return true;
}

inline bool parse_degrees(NmeaSpan field, float& out_deg) {
    return parse_real(field, out_deg);
}

inline bool parse_degrees(const char* field, float& out_deg) {
    return parse_real(field, out_deg);
}

inline bool parse_knots(NmeaSpan value, NmeaSpan unit, float& out_kn) {
    float v = 0.0f;
    if (!parse_real(value, v)) return false;
    char u = unit.empty() ? 'N' : unit[0];
    if (u == 'N') out_kn = v;
    else if (u == 'K') out_kn = v * 0.539956803f;
    else if (u == 'M') out_kn = v * 1.94384449f;
    else return false;
    return true;
}

inline bool parse_knots(const char* value, const char* unit, float& out_kn) {
    return parse_knots(NmeaSpan(value, value ? strlen(value) : 0), NmeaSpan(unit, unit ? strlen(unit) : 0), out_kn);
}

inline bool parse_lat_lon(NmeaSpan value, NmeaSpan hemi, float& out_deg) {
    if (value.empty() || hemi.empty()) return false;
    float raw = 0.0f;
    if (!parse_real(value, raw)) return false;
    int degrees = static_cast<int>(raw / 100.0f);
    float minutes = raw - static_cast<float>(degrees * 100);
    float deg = static_cast<float>(degrees) + minutes / 60.0f;
    char h = hemi[0];
    if (h == 'S' || h == 'W') deg = -deg;
    else if (!(h == 'N' || h == 'E')) return false;
    out_deg = deg;
    return true;
}

inline bool parse_lat_lon(const char* value, const char* hemi, float& out_deg) {
    return parse_lat_lon(NmeaSpan(value, value ? strlen(value) : 0), NmeaSpan(hemi, hemi ? strlen(hemi) : 0), out_deg);
}

inline bool parse_left_right_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float v = 0.0f;
    if (!parse_real(magnitude, v) || side.empty()) return false;
    if (side[0] == 'L') out = -v;
    else if (side[0] == 'R') out = v;
    else return false;
    return true;
}

inline bool parse_left_right_signed(const char* magnitude, const char* side, float& out) {
    return parse_left_right_signed(NmeaSpan(magnitude, magnitude ? strlen(magnitude) : 0), NmeaSpan(side, side ? strlen(side) : 0), out);
}

inline bool parse_east_west_signed(NmeaSpan magnitude, NmeaSpan side, float& out) {
    float v = 0.0f;
    if (!parse_real(magnitude, v) || side.empty()) return false;
    if (side[0] == 'E') out = v;
    else if (side[0] == 'W') out = -v;
    else return false;
    return true;
}

inline bool parse_east_west_signed(const char* magnitude, const char* side, float& out) {
    return parse_east_west_signed(NmeaSpan(magnitude, magnitude ? strlen(magnitude) : 0), NmeaSpan(side, side ? strlen(side) : 0), out);
}

} // namespace nmea0183_connector
