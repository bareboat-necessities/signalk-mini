#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

static const uint8_t DSC_CALL_HISTORY_CAPACITY = 4;

template<typename Real = float>
struct RadioFrequencySetData {
    Setting<SensorSource> source;
    Stamped<Real> transmitting_frequency_hz;
    Stamped<Real> receiving_frequency_hz;
    char communication_mode = 0;
    Stamped<int32_t> power_level;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct BeaconReceiverControlData {
    Setting<SensorSource> source;
    Stamped<Real> frequency_khz;
    char frequency_mode = 0;
    Stamped<int32_t> bit_rate_bps;
    char bit_rate_mode = 0;
    Stamped<Real> status_frequency_khz;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct BeaconReceiverStatusData {
    Setting<SensorSource> source;
    Stamped<Real> signal_strength_db_uv;
    Stamped<Real> signal_to_noise_ratio_db;
    Stamped<Real> beacon_frequency_khz;
    Stamped<int32_t> beacon_bit_rate_bps;
    Stamped<int32_t> status;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct ReturnLinkMessageData {
    Setting<SensorSource> source;
    char beacon_id[24] = {0};
    Stamped<Real> reception_time_s;
    char message_code = 0;
    char message_body[64] = {0};
    uint64_t last_update_us = 0;
};

enum class DscPriority : uint8_t {
    unknown,
    routine,
    safety,
    urgency,
    distress
};

enum class DscAddressType : uint8_t {
    unknown,
    distress,
    individual,
    all_ships,
    group,
    geographic_area
};

enum class DscEndSignalType : uint8_t {
    unknown,
    ack,
    end_of_sequence
};

template<typename Real = float>
struct DscCallData {
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
    char field10[16] = {0};
    char end_of_sequence = 0;
    DscPriority priority = DscPriority::unknown;
    DscAddressType address_type = DscAddressType::unknown;
    DscEndSignalType end_signal_type = DscEndSignalType::unknown;
    bool expansion_expected = false;
    bool expansion_received = false;
    bool expansion_timeout = false;
    Stamped<int32_t> dse_total_messages;
    Stamped<int32_t> dse_message_number;
    char dse_query_flag = 0;
    Stamped<int32_t> dse_expansion_specifier;
    char dse_payload[64] = {0};
    bool duplicate = false;
    Stamped<int32_t> repeat_count;
    Stamped<int32_t> field_count;
    uint64_t first_seen_us = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct DscCommData {
    DscCallData<Real> latest_call;
    DscCallData<Real> recent_calls[DSC_CALL_HISTORY_CAPACITY];
    Stamped<int32_t> recent_call_count;
    Stamped<int32_t> recent_call_next_index;
    Stamped<int32_t> call_count;
    Stamped<int32_t> duplicate_count;
    Stamped<int32_t> distress_count;
    Stamped<int32_t> urgency_count;
    Stamped<int32_t> safety_count;
    Stamped<int32_t> expansion_timeout_count;
};

template<typename Real = float>
struct InmarsatMessageData {
    Setting<SensorSource> source;
    char message_id[16] = {0};
    char terminal_id[24] = {0};
    char message_type[16] = {0};
    char message_status[16] = {0};
    char decoded_text[160] = {0};
    Stamped<int32_t> total_fragments;
    Stamped<int32_t> last_fragment_number;
    Stamped<int32_t> text_length;
    Stamped<int32_t> field_count;
    bool complete = false;
    bool overflow = false;
    uint64_t first_seen_us = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct InmarsatData {
    InmarsatMessageData<Real> latest_message;
    Stamped<int32_t> message_count;
    Stamped<int32_t> unsupported_count;
    Stamped<int32_t> bad_fragment_count;
};

template<typename Real = float>
struct CommEquipmentControlCommandData {
    Setting<SensorSource> source;
    char equipment_id[24] = {0};
    char command_id[24] = {0};
    char command_state = 0;
    char parameter[48] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct CommEquipmentControlOperationData {
    Setting<SensorSource> source;
    char equipment_id[24] = {0};
    char operation_id[24] = {0};
    char operation_state = 0;
    char value[48] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct CommEquipmentControlResponseData {
    Setting<SensorSource> source;
    char equipment_id[24] = {0};
    char command_id[24] = {0};
    char response_state = 0;
    char result[48] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct CommEquipmentDisplayControlData {
    Setting<SensorSource> source;
    char display_id[24] = {0};
    char page_id[24] = {0};
    char control_state = 0;
    char value[48] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct CommEquipmentDoorStatusData {
    Setting<SensorSource> source;
    char door_id[24] = {0};
    char door_state = 0;
    char lock_state = 0;
    char description[48] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct CommEquipmentData {
    CommEquipmentControlCommandData<Real> control_command;
    CommEquipmentControlOperationData<Real> control_operation;
    CommEquipmentControlResponseData<Real> control_response;
    CommEquipmentDisplayControlData<Real> display_control;
    CommEquipmentDoorStatusData<Real> door_status;
};

template<typename Real = float>
struct CommData {
    RadioFrequencySetData<Real> radio_frequency_set;
    BeaconReceiverControlData<Real> beacon_control;
    BeaconReceiverStatusData<Real> beacon_status;
    ReturnLinkMessageData<Real> return_link_message;
    DscCommData<Real> dsc;
    InmarsatData<Real> inmarsat;
    CommEquipmentData<Real> equipment;
};

} // namespace ship_data_model
