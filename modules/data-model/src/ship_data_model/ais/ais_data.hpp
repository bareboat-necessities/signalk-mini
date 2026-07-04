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
struct AisPositionReportData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> navigation_status;
    Stamped<Real> rate_of_turn_deg_min;
    Stamped<Real> speed_over_ground_kn;
    bool position_accuracy = false;
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    Stamped<Real> course_over_ground_deg;
    Stamped<Real> true_heading_deg;
    Stamped<int32_t> timestamp_s;
    Stamped<int32_t> maneuver_indicator;
    bool raim = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisBaseStationData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> year;
    Stamped<int32_t> month;
    Stamped<int32_t> day;
    Stamped<int32_t> hour;
    Stamped<int32_t> minute;
    Stamped<int32_t> second;
    bool position_accuracy = false;
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    Stamped<int32_t> epfd_type;
    bool raim = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisStaticVoyageData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> ais_version;
    Stamped<int32_t> imo_number;
    char call_sign[8] = {0};
    char vessel_name[24] = {0};
    Stamped<int32_t> ship_type;
    Stamped<int32_t> dimension_to_bow_m;
    Stamped<int32_t> dimension_to_stern_m;
    Stamped<int32_t> dimension_to_port_m;
    Stamped<int32_t> dimension_to_starboard_m;
    Stamped<int32_t> epfd_type;
    Stamped<int32_t> eta_month;
    Stamped<int32_t> eta_day;
    Stamped<int32_t> eta_hour;
    Stamped<int32_t> eta_minute;
    Stamped<Real> draught_m;
    char destination[24] = {0};
    bool dte_ready = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisClassBStaticData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> part_number;
    char vendor_id[8] = {0};
    char call_sign[8] = {0};
    char vessel_name[24] = {0};
    Stamped<int32_t> ship_type;
    Stamped<int32_t> dimension_to_bow_m;
    Stamped<int32_t> dimension_to_stern_m;
    Stamped<int32_t> dimension_to_port_m;
    Stamped<int32_t> dimension_to_starboard_m;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisAidToNavigationData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> aid_type;
    char name[40] = {0};
    bool position_accuracy = false;
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    Stamped<int32_t> dimension_to_bow_m;
    Stamped<int32_t> dimension_to_stern_m;
    Stamped<int32_t> dimension_to_port_m;
    Stamped<int32_t> dimension_to_starboard_m;
    Stamped<int32_t> epfd_type;
    Stamped<int32_t> timestamp_s;
    bool off_position = false;
    bool raim = false;
    bool virtual_aid = false;
    bool assigned_mode = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisSafetyTextData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> destination_mmsi;
    Stamped<int32_t> sequence_number;
    bool retransmit = false;
    char text[96] = {0};
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
    AisPositionReportData<Real> position_report;
    AisPositionReportData<Real> class_b_position_report;
    AisBaseStationData<Real> base_station;
    AisStaticVoyageData<Real> static_voyage;
    AisClassBStaticData<Real> class_b_static;
    AisAidToNavigationData<Real> aid_to_navigation;
    AisSafetyTextData<Real> safety_text;
    AisDataLinkStatusData<Real> data_link_status;
    AisAddressedSafetyData<Real> addressed_safety;
};

} // namespace ship_data_model
