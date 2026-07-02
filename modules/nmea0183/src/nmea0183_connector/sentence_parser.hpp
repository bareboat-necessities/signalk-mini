#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "nmea0183_helpers.hpp"
#include "special_sentence_hooks.hpp"

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
    NmeaSentenceFamily family;

    void clear() {
        raw[0] = '\0';
        raw_length = 0;
        body = NmeaSpan();
        talker = NmeaSpan();
        sentence = NmeaSpan();
        field_count = 0;
        valid_checksum = false;
        start_char = '$';
        family = NmeaSentenceFamily::Standard;
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

inline bool sentence_is_any(const NmeaSentence& s, const char* a, const char* b) {
    return sentence_is(s, a) || sentence_is(s, b);
}

inline bool sentence_is_any(const NmeaSentence& s, const char* a, const char* b, const char* c) {
    return sentence_is(s, a) || sentence_is(s, b) || sentence_is(s, c);
}

inline bool nmea_is_ais_sentence(const NmeaSentence& s) {
    return s.start_char == '!' && sentence_is_any(s, "VDM", "VDO", "VSD");
}

inline bool nmea_is_seatalk_sentence(const NmeaSentence& s) {
    return nmea_span_starts_with(s.body, "STALK") || nmea_span_starts_with(s.body, "PSTALK") ||
           (talker_is(s, "ST") && sentence_is(s, "ALK"));
}

inline bool nmea_is_dsc_sentence(const NmeaSentence& s) {
    return sentence_is_any(s, "DSC", "DSE", "DSI");
}

inline bool nmea_is_inmarsat_sentence(const NmeaSentence& s) {
    return nmea_span_starts_with(s.body, "PINM") || nmea_span_starts_with(s.body, "INM") ||
           sentence_is_any(s, "IMK", "IMN", "IMR") || talker_is(s, "IC") || talker_is(s, "CS");
}

inline bool nmea_is_proprietary_sentence(const NmeaSentence& s) {
    return s.body.length > 0 && s.body[0] == 'P';
}

inline NmeaSentenceFamily classify_nmea_sentence(const NmeaSentence& s) {
    if (nmea_is_ais_sentence(s)) return NmeaSentenceFamily::Ais;
    if (nmea_is_seatalk_sentence(s)) return NmeaSentenceFamily::SeaTalk;
    if (nmea_is_dsc_sentence(s)) return NmeaSentenceFamily::Dsc;
    if (nmea_is_inmarsat_sentence(s)) return NmeaSentenceFamily::Inmarsat;
    if (nmea_is_proprietary_sentence(s)) return NmeaSentenceFamily::Proprietary;
    if (s.start_char == '!') return NmeaSentenceFamily::UnknownEncapsulation;
    return NmeaSentenceFamily::Standard;
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
    void set_hooks(const NmeaParserHooks& hooks) { hooks_ = hooks; }
    void clear_hooks() { hooks_ = NmeaParserHooks{}; }

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
        return parse_line(line, out, true);
    }

    bool parse_line(const char* line, NmeaSentence& out, bool validate_checksum) {
        out.clear();
        if (!line || (line[0] != '$' && line[0] != '!')) { last_error_ = "bad start"; return false; }

        const char* star = strchr(line, '*');
        const char* body_end_in = star ? star : (line + strlen(line));

        if (validate_checksum) {
            if (!star) { last_error_ = "missing checksum"; return false; }
            uint8_t supplied = 0;
            if (!parse_checksum_hex(star + 1, supplied)) { last_error_ = "bad checksum field"; return false; }
            uint8_t computed = nmea_checksum_range(line + 1, star);
            if (computed != supplied) { last_error_ = "checksum mismatch"; return false; }
            out.valid_checksum = true;
        } else if (star) {
            uint8_t supplied = 0;
            out.valid_checksum = parse_checksum_hex(star + 1, supplied) && nmea_checksum_range(line + 1, star) == supplied;
        }

        size_t raw_len = strlen(line);
        if (raw_len >= NMEA_MAX_SENTENCE_LEN) raw_len = NMEA_MAX_SENTENCE_LEN - 1;
        memcpy(out.raw, line, raw_len);
        out.raw[raw_len] = '\0';
        out.raw_length = static_cast<uint8_t>(raw_len);
        out.start_char = out.raw[0];

        const char* raw_begin = out.raw;
        const size_t body_offset = 1;
        const size_t body_len = static_cast<size_t>(body_end_in - (line + 1));
        if (body_len < 5 || body_offset + body_len > raw_len) { last_error_ = "short body"; return false; }
        out.body = NmeaSpan(raw_begin + body_offset, body_len);
        out.talker = NmeaSpan(out.body.data, 2);
        out.sentence = NmeaSpan(out.body.data + 2, 3);

        const char* field_begin = out.body.data;
        const char* body_end = out.body.data + out.body.length;
        const char* comma = static_cast<const char*>(memchr(field_begin, ',', static_cast<size_t>(body_end - field_begin)));
        if (!comma) return finish_success(out);
        field_begin = comma + 1;
        while (out.field_count < NMEA_MAX_FIELDS && field_begin <= body_end) {
            const char* next = static_cast<const char*>(memchr(field_begin, ',', static_cast<size_t>(body_end - field_begin)));
            const char* field_end = next ? next : body_end;
            out.fields[out.field_count++] = NmeaSpan(field_begin, static_cast<size_t>(field_end - field_begin));
            if (!next) break;
            field_begin = next + 1;
        }
        return finish_success(out);
    }

private:
    bool finish_success(NmeaSentence& out) {
        out.family = classify_nmea_sentence(out);
        dispatch_hook(out);
        last_error_ = "";
        return true;
    }

    void dispatch_hook(const NmeaSentence& out) const {
        switch (out.family) {
        case NmeaSentenceFamily::Ais:
            if (hooks_.on_ais_sentence) hooks_.on_ais_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::SeaTalk:
            if (hooks_.on_seatalk_sentence) hooks_.on_seatalk_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::Dsc:
            if (hooks_.on_dsc_sentence) hooks_.on_dsc_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::Inmarsat:
            if (hooks_.on_inmarsat_sentence) hooks_.on_inmarsat_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::Proprietary:
            if (hooks_.on_proprietary_sentence) hooks_.on_proprietary_sentence(out, hooks_.user);
            break;
        default:
            break;
        }
    }

    char buffer_[NMEA_MAX_SENTENCE_LEN];
    size_t pos_;
    bool collecting_;
    const char* last_error_;
    NmeaParserHooks hooks_{};
};

} // namespace nmea0183_connector
