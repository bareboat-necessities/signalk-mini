#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nmea0183_helpers.hpp"
#include "special_sentence_hooks.hpp"

namespace nmea0183_connector {

static const uint8_t NMEA_TOKEN_MAX_TOKENS = 2;

struct NmeaTokenFragmentInfo {
    bool is_fragmented = false;
    uint8_t total = 0;
    uint8_t number = 0;
    NmeaSpan message_id;
    NmeaSpan payload;
};

struct NmeaToken {
    NmeaSpan text;
    NmeaSentenceFamily family = NmeaSentenceFamily::Standard;
    NmeaSpan proprietary_vendor_code;
    NmeaProprietaryVendor proprietary_vendor = NmeaProprietaryVendor::Unknown;
    NmeaTokenFragmentInfo fragment;
    bool is_sentence = false;
    bool has_checksum = false;
};

struct NmeaTokenizeResult {
    NmeaToken tokens[NMEA_TOKEN_MAX_TOKENS];
    uint8_t count = 0;

    const NmeaToken* first_sentence() const {
        for (uint8_t i = 0; i < count; ++i) {
            if (tokens[i].is_sentence) return &tokens[i];
        }
        return nullptr;
    }
};

inline bool nmea_is_line_end(char c) {
    return c == '\0' || c == '\r' || c == '\n';
}

inline bool nmea_is_token_start(char c) {
    return c == '$' || c == '!';
}

inline NmeaSpan nmea_trim_line(const char* line) {
    if (!line) return NmeaSpan();
    const char* begin = line;
    while (*begin == ' ' || *begin == '\t') ++begin;
    const char* end = begin;
    while (!nmea_is_line_end(*end)) ++end;
    while (end > begin && (end[-1] == ' ' || end[-1] == '\t')) --end;
    return NmeaSpan(begin, static_cast<size_t>(end - begin));
}

inline bool nmea_token_has_checksum(NmeaSpan token) {
    if (!token.data) return false;
    for (uint8_t i = 0; i < token.length; ++i) {
        if (token.data[i] == '*') return true;
    }
    return false;
}

inline NmeaSpan nmea_token_until_line_end(const char* start) {
    if (!start) return NmeaSpan();
    const char* end = start;
    while (!nmea_is_line_end(*end)) ++end;
    return NmeaSpan(start, static_cast<size_t>(end - start));
}

inline const char* nmea_find_sentence_start(NmeaSpan line) {
    if (!line.data) return nullptr;
    for (uint8_t i = 0; i < line.length; ++i) {
        if (nmea_is_token_start(line.data[i])) return line.data + i;
    }
    return nullptr;
}

inline bool nmea_looks_like_inmarsat_envelope(NmeaSpan line) {
    return line.length > 0 && line.data && line.data[0] == '/';
}

inline bool nmea_token_body_starts_with(NmeaSpan token, const char* prefix) {
    if (!token.data || token.length < 1 || !prefix) return false;
    const char* body = token.data + 1;
    const size_t body_len = token.length - 1;
    for (size_t i = 0; prefix[i]; ++i) {
        if (i >= body_len || body[i] != prefix[i]) return false;
    }
    return true;
}

inline bool nmea_token_sentence_is(NmeaSpan token, const char* sentence) {
    if (!token.data || !sentence || token.length < 6) return false;
    return token.data[3] == sentence[0] && token.data[4] == sentence[1] && token.data[5] == sentence[2];
}

inline bool nmea_token_is_query(NmeaSpan token) {
    return token.data && token.length > 6 && token[0] == '$' && token.data[5] == 'Q';
}

inline NmeaSpan nmea_token_body_without_checksum(NmeaSpan token) {
    if (!token.data || token.length < 2) return NmeaSpan();
    const char* begin = token.data + 1;
    const char* end = token.data + token.length;
    for (const char* p = begin; p < end; ++p) {
        if (*p == '*') { end = p; break; }
    }
    return NmeaSpan(begin, static_cast<size_t>(end - begin));
}

inline NmeaSpan nmea_token_field(NmeaSpan token, uint8_t wanted_index) {
    const NmeaSpan body = nmea_token_body_without_checksum(token);
    if (body.length < 5 || !body.data) return NmeaSpan();
    const char* begin = body.data;
    const char* end = body.data + body.length;
    const char* field_begin = begin;
    const char* comma = static_cast<const char*>(memchr(field_begin, ',', static_cast<size_t>(end - field_begin)));
    if (!comma) return NmeaSpan();
    field_begin = comma + 1;
    uint8_t index = 0;
    while (field_begin <= end) {
        const char* next = static_cast<const char*>(memchr(field_begin, ',', static_cast<size_t>(end - field_begin)));
        const char* field_end = next ? next : end;
        if (index == wanted_index) return NmeaSpan(field_begin, static_cast<size_t>(field_end - field_begin));
        if (!next) break;
        field_begin = next + 1;
        ++index;
    }
    return NmeaSpan();
}

inline bool nmea_token_parse_uint8(NmeaSpan field, uint8_t& out) {
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

inline void nmea_token_set_fragment(NmeaTokenFragmentInfo& fragment, NmeaSpan token, uint8_t total_index, uint8_t number_index, uint8_t id_index, uint8_t payload_index) {
    uint8_t total = 0;
    uint8_t number = 0;
    if (!nmea_token_parse_uint8(nmea_token_field(token, total_index), total)) return;
    if (!nmea_token_parse_uint8(nmea_token_field(token, number_index), number)) return;
    if (total == 0 || number == 0 || number > total || total > 16) return;
    fragment.is_fragmented = true;
    fragment.total = total;
    fragment.number = number;
    fragment.message_id = nmea_token_field(token, id_index);
    fragment.payload = nmea_token_field(token, payload_index);
}

inline void nmea_detect_token_fragment(NmeaToken& token) {
    if (!token.is_sentence) return;
    if (nmea_token_sentence_is(token.text, "TXT")) nmea_token_set_fragment(token.fragment, token.text, 0, 1, 2, 3);
    else if (token.family == NmeaSentenceFamily::Ais) nmea_token_set_fragment(token.fragment, token.text, 0, 1, 2, 4);
    else if (nmea_token_sentence_is(token.text, "DSE")) nmea_token_set_fragment(token.fragment, token.text, 0, 1, 3, 5);
    else if (nmea_token_sentence_is(token.text, "NRM") || nmea_token_sentence_is(token.text, "NRX")) nmea_token_set_fragment(token.fragment, token.text, 0, 1, 2, 3);
    else if (token.family == NmeaSentenceFamily::SeaTalk || token.family == NmeaSentenceFamily::Inmarsat) nmea_token_set_fragment(token.fragment, token.text, 0, 1, 2, 3);
}

inline NmeaSentenceFamily nmea_classify_token_span(NmeaSpan token) {
    if (!token.data || token.length == 0) return NmeaSentenceFamily::Standard;
    if (token[0] == '!') {
        if (nmea_token_body_starts_with(token, "AIVDM") || nmea_token_body_starts_with(token, "AIVDO") ||
            nmea_token_body_starts_with(token, "BSVDM") || nmea_token_body_starts_with(token, "BSVDO")) {
            return NmeaSentenceFamily::Ais;
        }
        return NmeaSentenceFamily::UnknownEncapsulation;
    }
    if (nmea_token_is_query(token)) return NmeaSentenceFamily::Query;
    if (nmea_token_body_starts_with(token, "STALK") || nmea_token_body_starts_with(token, "PSTALK")) {
        return NmeaSentenceFamily::SeaTalk;
    }
    if (nmea_token_body_starts_with(token, "DSC") || nmea_token_body_starts_with(token, "DSE") ||
        nmea_token_body_starts_with(token, "DSI") || nmea_token_body_starts_with(token, "DSR") ||
        nmea_token_body_starts_with(token, "CDDSC")) {
        return NmeaSentenceFamily::Dsc;
    }
    if (nmea_token_sentence_is(token, "NRM") || nmea_token_sentence_is(token, "NRX")) {
        return NmeaSentenceFamily::NavTex;
    }
    if (nmea_token_body_starts_with(token, "CSSM") || nmea_token_body_starts_with(token, "PINM") ||
        nmea_token_body_starts_with(token, "INM")) {
        return NmeaSentenceFamily::Inmarsat;
    }
    if (token.length > 1 && token[1] == 'P') return NmeaSentenceFamily::Proprietary;
    return NmeaSentenceFamily::Standard;
}

inline void nmea_add_token(NmeaTokenizeResult& result, NmeaSpan text, NmeaSentenceFamily family, bool is_sentence) {
    if (result.count >= NMEA_TOKEN_MAX_TOKENS) return;
    NmeaToken& token = result.tokens[result.count++];
    token.text = text;
    token.family = family;
    token.proprietary_vendor_code = nmea_proprietary_vendor_code_from_token(text);
    token.proprietary_vendor = nmea_proprietary_vendor_from_code(token.proprietary_vendor_code);
    token.fragment = NmeaTokenFragmentInfo{};
    token.is_sentence = is_sentence;
    token.has_checksum = nmea_token_has_checksum(text);
    nmea_detect_token_fragment(token);
}

inline NmeaTokenizeResult tokenize_nmea_line(const char* raw_line) {
    NmeaTokenizeResult result;
    const NmeaSpan line = nmea_trim_line(raw_line);
    if (line.empty()) return result;

    if (nmea_is_token_start(line[0])) {
        nmea_add_token(result, line, nmea_classify_token_span(line), true);
        return result;
    }

    if (nmea_looks_like_inmarsat_envelope(line)) {
        const char* embedded = nmea_find_sentence_start(line);
        if (embedded && embedded > line.data) {
            nmea_add_token(result, NmeaSpan(line.data, static_cast<size_t>(embedded - line.data)), NmeaSentenceFamily::Inmarsat, false);
            nmea_add_token(result, nmea_token_until_line_end(embedded), NmeaSentenceFamily::Inmarsat, true);
            return result;
        }
        nmea_add_token(result, line, NmeaSentenceFamily::Inmarsat, false);
        return result;
    }

    const char* embedded = nmea_find_sentence_start(line);
    if (embedded) {
        const NmeaSpan token = nmea_token_until_line_end(embedded);
        nmea_add_token(result, token, nmea_classify_token_span(token), true);
    }
    return result;
}

} // namespace nmea0183_connector
