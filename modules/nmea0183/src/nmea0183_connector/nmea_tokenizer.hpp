#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "nmea0183_helpers.hpp"
#include "special_sentence_hooks.hpp"

namespace nmea0183_connector {

static const uint8_t NMEA_TOKEN_MAX_TOKENS = 2;

struct NmeaToken {
    NmeaSpan text;
    NmeaSentenceFamily family = NmeaSentenceFamily::Standard;
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

inline NmeaSentenceFamily nmea_classify_token_span(NmeaSpan token) {
    if (!token.data || token.length == 0) return NmeaSentenceFamily::Standard;
    if (token[0] == '!') {
        if (nmea_token_body_starts_with(token, "AIVDM") || nmea_token_body_starts_with(token, "AIVDO") ||
            nmea_token_body_starts_with(token, "BSVDM") || nmea_token_body_starts_with(token, "BSVDO")) {
            return NmeaSentenceFamily::Ais;
        }
        return NmeaSentenceFamily::UnknownEncapsulation;
    }
    if (nmea_token_body_starts_with(token, "STALK") || nmea_token_body_starts_with(token, "PSTALK")) {
        return NmeaSentenceFamily::SeaTalk;
    }
    if (nmea_token_body_starts_with(token, "DSC") || nmea_token_body_starts_with(token, "DSE") ||
        nmea_token_body_starts_with(token, "DSI") || nmea_token_body_starts_with(token, "CDDSC")) {
        return NmeaSentenceFamily::Dsc;
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
    token.is_sentence = is_sentence;
    token.has_checksum = nmea_token_has_checksum(text);
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
