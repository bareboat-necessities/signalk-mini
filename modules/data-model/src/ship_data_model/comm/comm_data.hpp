#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

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
    CommEquipmentData<Real> equipment;
};

} // namespace ship_data_model
