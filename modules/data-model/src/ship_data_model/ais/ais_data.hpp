#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct TrackedTargetData {
    Setting<SensorSource> source;
    Stamped<int32_t> target_number;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> utc_time_s;
    Stamped<Real> distance_nmi;
    Stamped<Real> bearing_deg;
    char bearing_reference = 0;
    Stamped<Real> speed_kn;
    Stamped<Real> course_deg;
    char course_reference = 0;
    Stamped<Real> cpa_distance_nmi;
    Stamped<Real> tcpa_min;
    char distance_units = 0;
    char target_name[24] = {0};
    char target_status = 0;
    char reference_target = 0;
    Stamped<int32_t> label_target_number[8];
    char label[8][24] = {{0}};
    Stamped<int32_t> label_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisDataLinkStatusData {
    Setting<SensorSource> source;
    char station_id[24] = {0};
    char slot_status[16] = {0};
    char status_text[48] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisAddressedSafetyData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    char sequential_message_id[16] = {0};
    char destination_mmsi[16] = {0};
    char retransmit_flag = 0;
    char safety_text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisData {
    TrackedTargetData<Real> tracked_target;
    AisDataLinkStatusData<Real> data_link_status;
    AisAddressedSafetyData<Real> addressed_safety;
};

} // namespace ship_data_model
