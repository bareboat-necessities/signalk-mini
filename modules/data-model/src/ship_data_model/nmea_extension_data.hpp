#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct NmeaRawSentenceData {
    Setting<SensorSource> source;
    char sentence_id[4] = {0};
    Stamped<int32_t> field_count;
    char field[16][32] = {{0}};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NmeaDscMessageData {
    Setting<SensorSource> source;
    Stamped<int32_t> format_specifier;
    char sender_mmsi[16] = {0};
    Stamped<int32_t> category;
    Stamped<int32_t> nature_or_first_telecommand;
    Stamped<int32_t> communication_or_second_telecommand;
    char position_code[16] = {0};
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> utc_time_s;
    char address_or_distress_mmsi[16] = {0};
    char field10[32] = {0};
    char end_of_sequence = 0;
    char expansion_flag = 0;
    Stamped<int32_t> field_count;
    char raw_field[16][32] = {{0}};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NmeaDscExpansionData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    char query_flag = 0;
    char sender_mmsi[16] = {0};
    Stamped<int32_t> expansion_specifier;
    char payload[64] = {0};
    Stamped<int32_t> field_count;
    char raw_field[16][32] = {{0}};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NmeaDscData {
    NmeaDscMessageData<Real> message;
    NmeaDscExpansionData<Real> expansion;
    NmeaRawSentenceData<Real> initiate;
    NmeaRawSentenceData<Real> response;
};

template<typename Real = float>
struct NmeaCurrentLayerData {
    Setting<SensorSource> source;
    Stamped<int32_t> layer_number;
    Stamped<Real> current_direction_deg;
    char direction_reference = 0;
    Stamped<Real> current_speed_kn;
    Stamped<Real> layer_depth_m;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NmeaVariablePointData {
    Setting<SensorSource> source;
    Stamped<Real> time_to_point_s;
    Stamped<Real> distance_to_point_nmi;
    char point_id[16] = {0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NmeaExtensionsData {
    NmeaRawSentenceData<Real> alarm_acknowledgement;
    NmeaRawSentenceData<Real> automatic_device_status;
    NmeaRawSentenceData<Real> acknowledge_detail_alarm;
    NmeaRawSentenceData<Real> set_detail_alarm;
    NmeaRawSentenceData<Real> autopilot_system;
    NmeaRawSentenceData<Real> bearing_distance_dead_reckoning;
    NmeaRawSentenceData<Real> encryption_key_command;
    NmeaRawSentenceData<Real> operational_period_command;
    NmeaCurrentLayerData<Real> current_layer;
    NmeaRawSentenceData<Real> device_capability_report;
    NmeaRawSentenceData<Real> display_dimming_control;
    NmeaRawSentenceData<Real> door_status;
    NmeaDscData<Real> dsc;
    NmeaRawSentenceData<Real> engine_telegraph;
    NmeaRawSentenceData<Real> event_message;
    NmeaRawSentenceData<Real> fire_detection;
    NmeaRawSentenceData<Real> waypoint_distance_rhumb;
    NmeaRawSentenceData<Real> waypoint_distance_great_circle;
    NmeaVariablePointData<Real> variable_point;
};

} // namespace ship_data_model
