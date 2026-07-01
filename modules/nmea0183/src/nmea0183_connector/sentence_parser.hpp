#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "nmea0183_helpers.hpp"

namespace nmea0183_connector {

static const uint8_t NMEA_MAX_SENTENCE_LEN = 96;
static const uint8_t NMEA_MAX_FIELDS = 32;

struct NmeaSentence {
    char raw[NMEA_MAX_SENTENCE_LEN];
    uint8_t raw_length;
    NmeaSpan body;
    NmeaSpan talker;
    NmeaSpan sentence;
    NmeaSpan fields[NMEA_MAX_FIELDS];
    uint8_t field_count;
    bool valid_checksum;
    char start_char;

    void clear() {
        raw[0] = '\0';
        raw_length = 0;
        body = NmeaSpan();
        talker = NmeaSpan();
        sentence = NmeaSpan();
        field_count = 0;
        valid_checksum = false;
        start_char = '$';
        for (uint8_t i = 0; i < NMEA_MAX_FIELDS; ++i) fields[i] = NmeaSpan();
    }

    NmeaSpan field(uint8_t index) const {
        return index < field_count ? fields[index] : NmeaSpan();
    }
};

inline bool sentence_is(const NmeaSentence& s, const char* sentence) {
    return nmea_span_equals(s.sentence, sentence);
}

inline bool talker_is(const NmeaSentence& s, const char* talker) {
    return nmea_span_equals(s.talker, talker);
}

class Nmea0183StreamParser {
public:
    Nmea0183StreamParser() { reset(); }

    void reset() {
        pos_ = 0;
        collecting_ = false;
        last_error_ = "";
        buffer_[0] = '\0';
    }

    const char* last_error() const { return last_error_; }

    bool push(char c, NmeaSentence& out) {
        if (c == '$' || c == '!') {
            collecting_ = true;
            pos_ = 0;
            buffer_[pos_++] = c;
            buffer_[pos_] = '\0';
            return false;
        }
        if (!collecting_) return false;
        if (c == '\r') return false;
        if (c == '\n') {
            buffer_[pos_] = '\0';
            collecting_ = false;
            return parse_line(buffer_, out);
        }
        if (pos_ + 1 >= NMEA_MAX_SENTENCE_LEN) {
            reset();
            last_error_ = "sentence too long";
            return false;
        }
        buffer_[pos_++] = c;
        buffer_[pos_] = '\0';
        return false;
    }

    bool parse_line(const char* line, NmeaSentence& out) {
        out.clear();
        if (!line || (line[0] != '$' && line[0] != '!')) { last_error_ = "bad start"; return false; }
        const char* star = strchr(line, '*');
        if (!star) { last_error_ = "missing checksum"; return false; }
        uint8_t supplied = 0;
        if (!parse_checksum_hex(star + 1, supplied)) { last_error_ = "bad checksum field"; return false; }
        uint8_t computed = nmea_checksum_range(line + 1, star);
        if (computed != supplied) { last_error_ = "checksum mismatch"; return false; }

        size_t raw_len = strlen(line);
        if (raw_len >= NMEA_MAX_SENTENCE_LEN) raw_len = NMEA_MAX_SENTENCE_LEN - 1;
        memcpy(out.raw, line, raw_len);
        out.raw[raw_len] = '\0';
        out.raw_length = static_cast<uint8_t>(raw_len);
        out.start_char = out.raw[0];
        out.valid_checksum = true;

        const char* raw_begin = out.raw;
        const size_t body_offset = 1;
        const size_t body_len = static_cast<size_t>(star - (line + 1));
        if (body_len < 5 || body_offset + body_len > raw_len) { last_error_ = "short body"; return false; }
        out.body = NmeaSpan(raw_begin + body_offset, body_len);
        out.talker = NmeaSpan(out.body.data, 2);
        out.sentence = NmeaSpan(out.body.data + 2, 3);

        const char* field_begin = out.body.data;
        const char* body_end = out.body.data + out.body.length;
        const char* comma = static_cast<const char*>(memchr(field_begin, ',', static_cast<size_t>(body_end - field_begin)));
        if (!comma) {
            last_error_ = "";
            return true;
        }
        field_begin = comma + 1;
        while (out.field_count < NMEA_MAX_FIELDS && field_begin <= body_end) {
            const char* next = static_cast<const char*>(memchr(field_begin, ',', static_cast<size_t>(body_end - field_begin)));
            const char* field_end = next ? next : body_end;
            out.fields[out.field_count++] = NmeaSpan(field_begin, static_cast<size_t>(field_end - field_begin));
            if (!next) break;
            field_begin = next + 1;
        }
        last_error_ = "";
        return true;
    }

private:
    char buffer_[NMEA_MAX_SENTENCE_LEN];
    size_t pos_;
    bool collecting_;
    const char* last_error_;
};

} // namespace nmea0183_connector
