#pragma once

#include <stdint.h>
#include <ship_data_model.hpp>

namespace nmea0183_connector {

static const uint8_t NMEA_MESSAGE_RAW_MAX_FIELDS = 16;
static const uint8_t NMEA_MESSAGE_RAW_FIELD_BYTES = 16;
static const uint8_t NMEA_MESSAGE_MULTIPART_TEXT_BYTES = 96;
static const uint8_t NMEA_MESSAGE_MULTIPART_ID_BYTES = 16;

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

struct NmeaRawMessageRecord {
    NmeaMessageSource source;
    char sentence_id[4] = {0};
    NmeaMessageStampedInt field_count;
    bool truncated = false;
    char field[NMEA_MESSAGE_RAW_MAX_FIELDS][NMEA_MESSAGE_RAW_FIELD_BYTES] = {{0}};
    uint64_t last_update_us = 0;
};

struct NmeaMultipartMessageRecord {
    NmeaMessageSource source;
    char sentence_id[4] = {0};
    char message_id[NMEA_MESSAGE_MULTIPART_ID_BYTES] = {0};
    NmeaMessageStampedInt total_fragments;
    NmeaMessageStampedInt last_fragment_number;
    uint16_t received_mask = 0;
    bool in_progress = false;
    bool complete = false;
    bool overflow = false;
    char text[NMEA_MESSAGE_MULTIPART_TEXT_BYTES] = {0};
    NmeaMessageStampedInt text_length;
    uint64_t last_update_us = 0;
};

struct NmeaCurrentLayerMessageRecord {
    NmeaMessageSource source;
    NmeaMessageStampedInt layer_number;
    float current_direction_deg = 0.0f;
    char direction_reference = 0;
    float current_speed_kn = 0.0f;
    float layer_depth_m = 0.0f;
    bool has_direction = false;
    bool has_speed = false;
    bool has_depth = false;
    uint64_t last_update_us = 0;
};

struct NmeaVariablePointMessageRecord {
    NmeaMessageSource source;
    float time_to_point_s = 0.0f;
    float distance_to_point_nmi = 0.0f;
    char point_id[16] = {0};
    bool has_time = false;
    bool has_distance = false;
    uint64_t last_update_us = 0;
};

struct NmeaMessageState {
    NmeaRawMessageRecord alarm_acknowledgement;
    NmeaRawMessageRecord automatic_device_status;
    NmeaRawMessageRecord acknowledge_detail_alarm;
    NmeaRawMessageRecord set_detail_alarm;
    NmeaRawMessageRecord autopilot_system;
    NmeaRawMessageRecord bearing_distance_dead_reckoning;
    NmeaRawMessageRecord encryption_key_command;
    NmeaRawMessageRecord operational_period_command;
    NmeaCurrentLayerMessageRecord current_layer;
    NmeaRawMessageRecord device_capability_report;
    NmeaRawMessageRecord display_dimming_control;
    NmeaRawMessageRecord door_status;
    NmeaRawMessageRecord engine_telegraph;
    NmeaRawMessageRecord event_message;
    NmeaRawMessageRecord fire_detection;
    NmeaRawMessageRecord waypoint_distance_rhumb;
    NmeaRawMessageRecord waypoint_distance_great_circle;
    NmeaVariablePointMessageRecord variable_point;

    NmeaRawMessageRecord text_sentence;
    NmeaMultipartMessageRecord text_message;
    NmeaMultipartMessageRecord ais_message;
    NmeaMultipartMessageRecord navtex_message;
    NmeaMultipartMessageRecord seatalk_message;
    NmeaMultipartMessageRecord inmarsat_message;
    NmeaMultipartMessageRecord generic_multipart_message;
};

} // namespace nmea0183_connector
