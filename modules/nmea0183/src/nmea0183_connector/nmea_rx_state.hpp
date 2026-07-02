#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>

namespace nmea0183_connector {

static const uint8_t NMEA_RX_RAW_MAX_FIELDS = 16;
static const uint8_t NMEA_RX_RAW_FIELD_BYTES = 16;
static const uint8_t NMEA_RX_MULTIPART_TEXT_BYTES = 96;
static const uint8_t NMEA_RX_MULTIPART_ID_BYTES = 16;

struct NmeaRxSource {
    ship_data_model::SensorSource value = ship_data_model::SensorSource::none;
};

struct NmeaRxStampedInt {
    int32_t value = 0;
    uint64_t last_update_us = 0;

    void set(int32_t next, uint64_t now_us) {
        value = next;
        last_update_us = now_us;
    }
};

struct NmeaRxRawSentenceRecord {
    NmeaRxSource source;
    char sentence_id[4] = {0};
    NmeaRxStampedInt field_count;
    bool truncated = false;
    char field[NMEA_RX_RAW_MAX_FIELDS][NMEA_RX_RAW_FIELD_BYTES] = {{0}};
    uint64_t last_update_us = 0;
};

struct NmeaRxMultipartRecord {
    NmeaRxSource source;
    char sentence_id[4] = {0};
    char message_id[NMEA_RX_MULTIPART_ID_BYTES] = {0};
    NmeaRxStampedInt total_fragments;
    NmeaRxStampedInt last_fragment_number;
    uint16_t received_mask = 0;
    bool in_progress = false;
    bool complete = false;
    bool overflow = false;
    char text[NMEA_RX_MULTIPART_TEXT_BYTES] = {0};
    NmeaRxStampedInt text_length;
    uint64_t last_update_us = 0;
};

struct NmeaRxDscMessageRecord {
    NmeaRxSource source;
    NmeaRxStampedInt format_specifier;
    char sender_mmsi[16] = {0};
    NmeaRxStampedInt category;
    NmeaRxStampedInt nature_or_first_telecommand;
    NmeaRxStampedInt communication_or_second_telecommand;
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
    NmeaRxStampedInt field_count;
    bool truncated = false;
    char raw_field[NMEA_RX_RAW_MAX_FIELDS][NMEA_RX_RAW_FIELD_BYTES] = {{0}};
    uint64_t last_update_us = 0;
};

struct NmeaRxDscExpansionRecord {
    NmeaRxSource source;
    NmeaRxStampedInt total_messages;
    NmeaRxStampedInt message_number;
    char query_flag = 0;
    char sender_mmsi[16] = {0};
    NmeaRxStampedInt expansion_specifier;
    char payload[64] = {0};
    NmeaRxStampedInt field_count;
    bool truncated = false;
    char raw_field[NMEA_RX_RAW_MAX_FIELDS][NMEA_RX_RAW_FIELD_BYTES] = {{0}};
    uint64_t last_update_us = 0;
};

struct NmeaRxCurrentLayerRecord {
    NmeaRxSource source;
    NmeaRxStampedInt layer_number;
    float current_direction_deg = 0.0f;
    char direction_reference = 0;
    float current_speed_kn = 0.0f;
    float layer_depth_m = 0.0f;
    bool has_direction = false;
    bool has_speed = false;
    bool has_depth = false;
    uint64_t last_update_us = 0;
};

struct NmeaRxVariablePointRecord {
    NmeaRxSource source;
    float time_to_point_s = 0.0f;
    float distance_to_point_nmi = 0.0f;
    char point_id[16] = {0};
    bool has_time = false;
    bool has_distance = false;
    uint64_t last_update_us = 0;
};

struct NmeaRxDscState {
    NmeaRxDscMessageRecord message;
    NmeaRxDscExpansionRecord expansion;
    NmeaRxRawSentenceRecord initiate;
    NmeaRxRawSentenceRecord response;
    NmeaRxMultipartRecord multipart;
};

struct Nmea0183RxState {
    NmeaRxRawSentenceRecord alarm_acknowledgement;
    NmeaRxRawSentenceRecord automatic_device_status;
    NmeaRxRawSentenceRecord acknowledge_detail_alarm;
    NmeaRxRawSentenceRecord set_detail_alarm;
    NmeaRxRawSentenceRecord autopilot_system;
    NmeaRxRawSentenceRecord bearing_distance_dead_reckoning;
    NmeaRxRawSentenceRecord encryption_key_command;
    NmeaRxRawSentenceRecord operational_period_command;
    NmeaRxCurrentLayerRecord current_layer;
    NmeaRxRawSentenceRecord device_capability_report;
    NmeaRxRawSentenceRecord display_dimming_control;
    NmeaRxRawSentenceRecord door_status;
    NmeaRxDscState dsc;
    NmeaRxRawSentenceRecord engine_telegraph;
    NmeaRxRawSentenceRecord event_message;
    NmeaRxRawSentenceRecord fire_detection;
    NmeaRxRawSentenceRecord waypoint_distance_rhumb;
    NmeaRxRawSentenceRecord waypoint_distance_great_circle;
    NmeaRxVariablePointRecord variable_point;

    NmeaRxRawSentenceRecord text_sentence;
    NmeaRxMultipartRecord text_message;
    NmeaRxMultipartRecord ais_message;
    NmeaRxMultipartRecord navtex_message;
    NmeaRxMultipartRecord seatalk_message;
    NmeaRxMultipartRecord inmarsat_message;
    NmeaRxMultipartRecord generic_multipart_message;
};

} // namespace nmea0183_connector
