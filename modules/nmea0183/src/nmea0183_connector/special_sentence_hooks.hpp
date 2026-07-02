#pragma once

#include <stdint.h>
#include "nmea0183_helpers.hpp"
#include "proprietary_vendor.hpp"

namespace nmea0183_connector {

struct NmeaSentence;

enum class NmeaSentenceFamily : uint8_t {
    Standard = 0,
    Query,
    Ais,
    SeaTalk,
    Dsc,
    NavTex,
    Inmarsat,
    Proprietary,
    UnknownEncapsulation,
};

inline const char* nmea_sentence_family_name(NmeaSentenceFamily family) {
    switch (family) {
    case NmeaSentenceFamily::Standard: return "standard";
    case NmeaSentenceFamily::Query: return "query";
    case NmeaSentenceFamily::Ais: return "ais";
    case NmeaSentenceFamily::SeaTalk: return "seatalk";
    case NmeaSentenceFamily::Dsc: return "dsc";
    case NmeaSentenceFamily::NavTex: return "navtex";
    case NmeaSentenceFamily::Inmarsat: return "inmarsat";
    case NmeaSentenceFamily::Proprietary: return "proprietary";
    case NmeaSentenceFamily::UnknownEncapsulation: return "unknown_encapsulation";
    }
    return "unknown";
}

struct NmeaParserHooks {
    void* user = nullptr;
    void (*on_query_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
    void (*on_ais_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
    void (*on_seatalk_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
    void (*on_dsc_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
    void (*on_navtex_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
    void (*on_inmarsat_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
    void (*on_proprietary_sentence)(const NmeaSentence& sentence, void* user) = nullptr;
};

inline bool nmea_span_starts_with(NmeaSpan span, const char* prefix) {
    if (!prefix) return false;
    for (size_t i = 0; prefix[i]; ++i) {
        if (i >= span.length || !span.data || span.data[i] != prefix[i]) return false;
    }
    return true;
}

} // namespace nmea0183_connector
