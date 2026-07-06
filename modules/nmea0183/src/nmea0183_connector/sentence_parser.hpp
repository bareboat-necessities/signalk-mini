#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "nmea0183_helpers.hpp"
#include "special_sentence_hooks.hpp"

namespace nmea0183_connector {

static const uint8_t NMEA_MAX_SENTENCE_LEN = 192;
static const uint8_t NMEA_MAX_FIELDS = 32;

struct NmeaFragmentInfo {
    bool is_fragmented = false;
    uint8_t total = 0;
    uint8_t number = 0;
    NmeaSpan message_id;
    NmeaSpan payload;

    bool is_first() const { return is_fragmented && number == 1; }
    bool is_last() const { return is_fragmented && total > 0 && number == total; }
};

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
    NmeaTalkerId talker_id;
    NmeaGnssSystem gnss_system;
    NmeaSpan query_requester_talker;
    NmeaSpan query_target_talker;
    NmeaSpan query_requested_sentence;
    NmeaSpan proprietary_vendor_code;
    NmeaProprietaryVendor proprietary_vendor;
    NmeaFragmentInfo fragment;

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
        talker_id = NmeaTalkerId::Unknown;
        gnss_system = NmeaGnssSystem::Unknown;
        query_requester_talker = NmeaSpan();
        query_target_talker = NmeaSpan();
        query_requested_sentence = NmeaSpan();
        proprietary_vendor_code = NmeaSpan();
        proprietary_vendor = NmeaProprietaryVendor::Unknown;
        fragment = NmeaFragmentInfo{};
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

inline bool nmea_parse_uint8_span(NmeaSpan field, uint8_t& out) {
    if (field.empty() || !field.data) return false;
    unsigned value = 0;
    for (uint8_t i = 0; i < field.length; ++i) {
        const char c = field[i];
        if (c < '0' || c > '9') return false;
        value = value * 10u + static_cast<unsigned>(c - '0');
        if (value > 255u) return false;
    }
    out = static_cast<uint8_t>(value);
    return true;
}

inline bool nmea_is_query_sentence(const NmeaSentence& s) {
    return s.start_char == '$' && s.sentence.length == 3 && s.sentence.data &&
           s.sentence.data[2] == 'Q' && s.query_requested_sentence.length == 3;
}

inline bool nmea_is_ais_sentence(const NmeaSentence& s) {
    return s.start_char == '!' && sentence_is_any(s, "VDM", "VDO", "VSD");
}

inline bool nmea_is_seatalk_sentence(const NmeaSentence& s) {
    return nmea_span_starts_with(s.body, "STALK") || nmea_span_starts_with(s.body, "PSTALK") ||
           (talker_is(s, "ST") && sentence_is(s, "ALK"));
}

inline bool nmea_is_dsc_sentence(const NmeaSentence& s) {
    return sentence_is_any(s, "DSC", "DSE", "DSI") || sentence_is(s, "DSR");
}

inline bool nmea_is_inmarsat_sm_sentence(const NmeaSentence& s) {
    return sentence_is_any(s, "SM1", "SM2", "SM3") || sentence_is(s, "SM4") || sentence_is(s, "SMB");
}

inline bool nmea_is_inmarsat_sentence(const NmeaSentence& s) {
    return nmea_span_starts_with(s.body, "PINM") || nmea_span_starts_with(s.body, "INM") ||
           sentence_is_any(s, "IMK", "IMN", "IMR") || nmea_is_inmarsat_sm_sentence(s) ||
           talker_is(s, "IC") || talker_is(s, "CS");
}

inline bool nmea_is_proprietary_sentence(const NmeaSentence& s) {
    return s.body.length > 0 && s.body[0] == 'P';
}

inline void nmea_set_fragment_info(NmeaSentence& s, uint8_t total_index, uint8_t number_index, uint8_t id_index, uint8_t payload_index) {
    uint8_t total = 0;
    uint8_t number = 0;
    if (s.field_count <= total_index || s.field_count <= number_index) return;
    if (!nmea_parse_uint8_span(s.field(total_index), total) || !nmea_parse_uint8_span(s.field(number_index), number)) return;
    if (total == 0 || number == 0 || number > total || total > 16) return;

    s.fragment.is_fragmented = true;
    s.fragment.total = total;
    s.fragment.number = number;
    if (id_index < s.field_count) s.fragment.message_id = s.field(id_index);
    if (payload_index < s.field_count) s.fragment.payload = s.field(payload_index);
}

inline void nmea_detect_fragment_info(NmeaSentence& s) {
    if (sentence_is(s, "TXT")) {
        nmea_set_fragment_info(s, 0, 1, 2, 3);
    } else if (nmea_is_ais_sentence(s)) {
        nmea_set_fragment_info(s, 0, 1, 2, 4);
    } else if (sentence_is(s, "DSE")) {
        nmea_set_fragment_info(s, 0, 1, 3, 5);
    } else if (sentence_is_any(s, "NRM", "NRX")) {
        nmea_set_fragment_info(s, 0, 1, 2, 3);
    } else if (sentence_is(s, "SMB")) {
        nmea_set_fragment_info(s, 0, 1, 3, 4);
    } else if (nmea_is_seatalk_sentence(s) || nmea_is_inmarsat_sentence(s)) {
        nmea_set_fragment_info(s, 0, 1, 2, 3);
    }
}

inline NmeaSentenceFamily classify_nmea_sentence(const NmeaSentence& s) {
    if (nmea_is_query_sentence(s)) return NmeaSentenceFamily::Query;
    if (nmea_is_ais_sentence(s)) return NmeaSentenceFamily::Ais;
    if (nmea_is_seatalk_sentence(s)) return NmeaSentenceFamily::SeaTalk;
    if (nmea_is_dsc_sentence(s)) return NmeaSentenceFamily::Dsc;
    if (sentence_is_any(s, "NRM", "NRX")) return NmeaSentenceFamily::NavTex;
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
        if (!line) { last_error_ = "bad start"; return false; }

        const char* sentence_line = line;
        if (sentence_line[0] == '/') {
            const char* tag_end = strchr(sentence_line + 1, '/');
            if (!tag_end || (tag_end[1] != '$' && tag_end[1] != '!')) { last_error_ = "bad tag block"; return false; }
            sentence_line = tag_end + 1;
        }

        if (sentence_line[0] != '$' && sentence_line[0] != '!') { last_error_ = "bad start"; return false; }

        const char* star = strchr(sentence_line, '*');
        const char* body_end_in = star ? star : (sentence_line + strlen(sentence_line));

        if (validate_checksum) {
            if (!star) { last_error_ = "missing checksum"; return false; }
            uint8_t supplied = 0;
            if (!parse_checksum_hex(star + 1, supplied)) { last_error_ = "bad checksum field"; return false; }
            uint8_t computed = nmea_checksum_range(sentence_line + 1, star);
            if (computed != supplied) { last_error_ = "checksum mismatch"; return false; }
            out.valid_checksum = true;
        } else if (star) {
            uint8_t supplied = 0;
            out.valid_checksum = parse_checksum_hex(star + 1, supplied) && nmea_checksum_range(sentence_line + 1, star) == supplied;
        }

        size_t raw_len = strlen(sentence_line);
        if (raw_len >= NMEA_MAX_SENTENCE_LEN) raw_len = NMEA_MAX_SENTENCE_LEN - 1;
        memcpy(out.raw, sentence_line, raw_len);
        out.raw[raw_len] = '\0';
        out.raw_length = static_cast<uint8_t>(raw_len);
        out.start_char = out.raw[0];

        const char* raw_begin = out.raw;
        const size_t body_offset = 1;
        const size_t body_len = static_cast<size_t>(body_end_in - (sentence_line + 1));
        if (body_len < 5 || body_offset + body_len > raw_len) { last_error_ = "short body"; return false; }
        out.body = NmeaSpan(raw_begin + body_offset, body_len);
        out.talker = NmeaSpan(out.body.data, 2);
        out.sentence = NmeaSpan(out.body.data + 2, 3);
        out.talker_id = nmea_talker_id_from_span(out.talker);
        out.gnss_system = nmea_gnss_system_from_talker(out.talker_id);
        if (out.sentence.length == 3 && out.sentence.data && out.sentence.data[2] == 'Q') {
            out.query_requester_talker = NmeaSpan(out.body.data, 2);
            out.query_target_talker = NmeaSpan(out.body.data + 2, 2);
        }
        out.proprietary_vendor_code = nmea_proprietary_vendor_code_from_body(out.body);
        out.proprietary_vendor = nmea_proprietary_vendor_from_code(out.proprietary_vendor_code);

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
        if (out.query_requester_talker.length == 2 && out.field_count >= 1 && out.field(0).length == 3) {
            out.query_requested_sentence = out.field(0);
        }
        return finish_success(out);
    }

private:
    bool finish_success(NmeaSentence& out) {
        out.family = classify_nmea_sentence(out);
        nmea_detect_fragment_info(out);
        dispatch_hook(out);
        last_error_ = "";
        return true;
    }

    void dispatch_hook(const NmeaSentence& out) const {
        switch (out.family) {
        case NmeaSentenceFamily::Query:
            if (hooks_.on_query_sentence) hooks_.on_query_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::Ais:
            if (hooks_.on_ais_sentence) hooks_.on_ais_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::SeaTalk:
            if (hooks_.on_seatalk_sentence) hooks_.on_seatalk_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::Dsc:
            if (hooks_.on_dsc_sentence) hooks_.on_dsc_sentence(out, hooks_.user);
            break;
        case NmeaSentenceFamily::NavTex:
            if (hooks_.on_navtex_sentence) hooks_.on_navtex_sentence(out, hooks_.user);
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

    NmeaParserHooks hooks_;
    char buffer_[NMEA_MAX_SENTENCE_LEN];
    uint8_t pos_ = 0;
    bool collecting_ = false;
    const char* last_error_ = "";
};

} // namespace nmea0183_connector
