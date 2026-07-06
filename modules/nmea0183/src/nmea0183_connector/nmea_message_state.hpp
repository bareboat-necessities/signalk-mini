#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>

namespace nmea0183_connector {

static const uint8_t NMEA_MESSAGE_MULTIPART_TEXT_BYTES = 96;
static const uint16_t NMEA_NAVTEX_MULTIPART_TEXT_BYTES = 256;
static const uint16_t NMEA_INMARSAT_SAFETYNET_TEXT_BYTES = 512;
static const uint64_t NMEA_AIS_MULTIPART_STALE_TIMEOUT_US = 30000000ULL;
static const uint64_t NMEA_NAVTEX_MULTIPART_STALE_TIMEOUT_US = 30000000ULL;
static const uint8_t NMEA_MESSAGE_MULTIPART_ID_BYTES = 16;
static const uint8_t NMEA_AIS_MULTIPART_SLOT_COUNT = 4;
static const uint8_t NMEA_NAVTEX_MULTIPART_SLOT_COUNT = 4;

struct NmeaMessageSource {
    ship_data_model::SensorSource value = ship_data_model::SensorSource::none;
};

struct NmeaMessageStampedInt {
    int32_t value = 0;
    uint64_t last_update_us = 0;

    void set(int32_t next, uint64_t now_us) {
        value = next;
        last_update_us = now_us;
    }
};

template<uint16_t TextBytes>
struct NmeaMultipartMessageRecordT {
    NmeaMessageSource source;
    char sentence_id[4] = {0};
    char talker_id[3] = {0};
    char message_id[NMEA_MESSAGE_MULTIPART_ID_BYTES] = {0};
    char radio_channel = 0;
    NmeaMessageStampedInt total_fragments;
    NmeaMessageStampedInt last_fragment_number;
    uint16_t received_mask = 0;
    bool in_progress = false;
    bool complete = false;
    bool overflow = false;
    char text[TextBytes] = {0};
    NmeaMessageStampedInt text_length;
    NmeaMessageStampedInt duplicate_fragment_count;
    NmeaMessageStampedInt bad_fragment_count;
    uint64_t last_update_us = 0;
};

struct NmeaSafetyNetAssemblyRecord {
    NmeaMessageSource source;
    char talker_id[3] = {0};
    char message_id[16] = {0};
    char msi_status = 0;
    char les_sequence_number[8] = {0};
    char les_id[4] = {0};
    char ocean_region_code = 0;
    char priority_code = 0;
    char service_code[4] = {0};
    char presentation_code[4] = {0};
    int32_t received_year = 0;
    int32_t received_month = 0;
    int32_t received_day = 0;
    int32_t received_hour = 0;
    int32_t received_minute = 0;
    char address_kind[24] = {0};
    int32_t navarea_metarea_code = 0;
    int32_t coastal_warning_navarea_metarea = 0;
    char coastal_warning_area = 0;
    char coastal_warning_subject = 0;
    float circular_center_lat_deg = 0.0f;
    float circular_center_lon_deg = 0.0f;
    int32_t circular_radius_nmi = 0;
    float rectangle_sw_lat_deg = 0.0f;
    float rectangle_sw_lon_deg = 0.0f;
    int32_t rectangle_extent_lat_deg = 0;
    int32_t rectangle_extent_lon_deg = 0;
    bool has_navarea_metarea_code = false;
    bool has_coastal_warning_navarea_metarea = false;
    bool has_circular_center = false;
    bool has_circular_radius = false;
    bool has_rectangle_sw = false;
    bool has_rectangle_extent = false;
    bool header_received = false;
    bool body_in_progress = false;
    bool body_complete = false;
    bool overflow = false;
    bool body_has_unknown_char = false;
    int32_t total_body_sentences = 0;
    int32_t received_body_sentence_count = 0;
    int32_t last_body_sentence_number = 0;
    char sequential_message_id[8] = {0};
    uint64_t received_mask = 0;
    char body_text[NMEA_INMARSAT_SAFETYNET_TEXT_BYTES] = {0};
    int32_t body_text_length = 0;
    uint64_t first_seen_us = 0;
    uint64_t last_update_us = 0;
};

using NmeaMultipartMessageRecord = NmeaMultipartMessageRecordT<NMEA_MESSAGE_MULTIPART_TEXT_BYTES>;
using NmeaNavtexMultipartMessageRecord = NmeaMultipartMessageRecordT<NMEA_NAVTEX_MULTIPART_TEXT_BYTES>;

struct NmeaMessageState {
    NmeaMultipartMessageRecord text_message;
    NmeaMultipartMessageRecord ais_message;
    NmeaMultipartMessageRecord ais_messages[NMEA_AIS_MULTIPART_SLOT_COUNT];
    NmeaMessageStampedInt active_ais_slot;
    NmeaMessageStampedInt ais_multipart_replacement_count;
    NmeaMessageStampedInt ais_multipart_stale_count;
    NmeaNavtexMultipartMessageRecord navtex_message;
    NmeaNavtexMultipartMessageRecord navtex_messages[NMEA_NAVTEX_MULTIPART_SLOT_COUNT];
    NmeaMessageStampedInt active_navtex_slot;
    NmeaMessageStampedInt navtex_multipart_replacement_count;
    NmeaMessageStampedInt navtex_multipart_stale_count;
    NmeaMultipartMessageRecord seatalk_message;
    NmeaMultipartMessageRecord inmarsat_message;
    NmeaSafetyNetAssemblyRecord inmarsat_safetynet;
    NmeaMultipartMessageRecord generic_multipart_message;
};

} // namespace nmea0183_connector
