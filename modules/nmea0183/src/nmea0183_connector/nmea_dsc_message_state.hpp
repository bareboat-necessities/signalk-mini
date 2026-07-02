#pragma once

#include <stdint.h>
#include "nmea_message_state.hpp"

namespace nmea0183_connector {

struct NmeaDscMessageRecord {
    NmeaMessageSource source;
    NmeaMessageStampedInt format_specifier;
    char sender_mmsi[16] = {0};
    NmeaMessageStampedInt category;
    NmeaMessageStampedInt nature_or_first_telecommand;
    NmeaMessageStampedInt communication_or_second_telecommand;
    char position_code[16] = {0};
    float latitude_deg = 0.0f;
    float longitude_deg = 0.0f;
    float utc_time_s = 0.0f;
    bool has_position = false;
    bool has_utc_time = false;
    char address_or_distress_mmsi[16] = {0};
    char field10[16] = {0};
    char end_of_sequence = 0;
    char expansion_flag = 0;
    NmeaMessageStampedInt field_count;
    bool truncated = false;
    char raw_field[NMEA_MESSAGE_RAW_MAX_FIELDS][NMEA_MESSAGE_RAW_FIELD_BYTES] = {{0}};
    uint64_t last_update_us = 0;
};

struct NmeaDscExpansionRecord {
    NmeaMessageSource source;
    NmeaMessageStampedInt total_messages;
    NmeaMessageStampedInt message_number;
    char query_flag = 0;
    char sender_mmsi[16] = {0};
    NmeaMessageStampedInt expansion_specifier;
    char payload[64] = {0};
    NmeaMessageStampedInt field_count;
    bool truncated = false;
    char raw_field[NMEA_MESSAGE_RAW_MAX_FIELDS][NMEA_MESSAGE_RAW_FIELD_BYTES] = {{0}};
    uint64_t last_update_us = 0;
};

struct NmeaDscMessageState {
    NmeaDscMessageRecord message;
    NmeaDscExpansionRecord expansion;
    NmeaRawMessageRecord initiate;
    NmeaRawMessageRecord response;
    NmeaMultipartMessageRecord multipart;
};

} // namespace nmea0183_connector
