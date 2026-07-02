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

inline void nmea_copy_cstr(char* out, size_t out_size, const char* text) {
    if (!out || out_size == 0) return;
    size_t n = text ? strlen(text) : 0;
    if (n + 1 > out_size) n = out_size - 1;
    if (n && text) memcpy(out, text, n);
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

} // namespace nmea0183_connector