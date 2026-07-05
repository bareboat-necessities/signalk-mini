#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

static const uint8_t AIS_TARGET_TABLE_CAPACITY = 16;

template<typename Real = float>
struct AisMessageHeaderData {
    Setting<SensorSource> source;
    Stamped<int32_t> message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> mmsi;
};

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
struct AisTargetData {
    Setting<SensorSource> source;
    bool occupied = false;
    Stamped<int32_t> mmsi;
    Stamped<int32_t> last_message_type;
    Stamped<int32_t> repeat_indicator;
    Stamped<int32_t> navigation_status;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> speed_over_ground_kn;
    Stamped<Real> course_over_ground_deg;
    Stamped<Real> true_heading_deg;
    Stamped<int32_t> timestamp_s;
    Stamped<int32_t> ship_type;
    Stamped<int32_t> imo_number;
    Stamped<int32_t> aid_type;
    char vessel_name[24] = {0};
    char call_sign[8] = {0};
    char destination[24] = {0};
    bool position_accuracy = false;
    bool raim = false;
    bool class_b = false;
    bool base_station = false;
    bool aid_to_navigation = false;
    bool sar_aircraft = false;
    bool stale = false;
    uint64_t first_seen_us = 0;
    uint64_t last_seen_us = 0;
    uint64_t last_position_update_us = 0;
    uint64_t last_static_update_us = 0;
};

template<typename Real = float, uint8_t MaxTargets = AIS_TARGET_TABLE_CAPACITY>
struct AisTargetTableData {
    AisTargetData<Real> targets[MaxTargets];
    Stamped<int32_t> target_count;
    Stamped<int32_t> replacement_count;
    Stamped<int32_t> overflow_count;
};

template<typename Real = float>
struct AisOwnVesselData : AisMessageHeaderData<Real> {
    bool valid = false;
    Stamped<int32_t> navigation_status;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> speed_over_ground_kn;
    Stamped<Real> course_over_ground_deg;
    Stamped<Real> true_heading_deg;
    Stamped<int32_t> timestamp_s;
    Stamped<int32_t> ship_type;
    Stamped<int32_t> imo_number;
    Stamped<int32_t> aid_type;
    char vessel_name[24] = {0};
    char call_sign[8] = {0};
    char destination[24] = {0};
    bool position_accuracy = false;
    bool raim = false;
    bool class_b = false;
    bool base_station = false;
    bool aid_to_navigation = false;
    bool sar_aircraft = false;
    uint64_t first_seen_us = 0;
    uint64_t last_seen_us = 0;
    uint64_t last_position_update_us = 0;
    uint64_t last_static_update_us = 0;
};

template<typename Real = float>
struct AisPositionReportData : AisMessageHeaderData<Real> {
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
    bool cs_unit = false;
    bool display = false;
    bool dsc = false;
    bool band = false;
    bool accepts_message_22 = false;
    bool assigned_mode = false;
    bool dte_ready = false;
    Stamped<int32_t> radio_status;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisSarAircraftPositionData : AisMessageHeaderData<Real> {
    Stamped<int32_t> altitude_m;
    Stamped<Real> speed_over_ground_kn;
    bool position_accuracy = false;
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    Stamped<Real> course_over_ground_deg;
    Stamped<int32_t> timestamp_s;
    bool altitude_sensor_barometric = false;
    bool dte_ready = false;
    bool assigned_mode = false;
    bool raim = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisBaseStationData : AisMessageHeaderData<Real> {
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
struct AisStaticVoyageData : AisMessageHeaderData<Real> {
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
struct AisClassBStaticData : AisMessageHeaderData<Real> {
    Stamped<int32_t> part_number;
    char vendor_id[8] = {0};
    char call_sign[8] = {0};
    char vessel_name[24] = {0};
    Stamped<int32_t> ship_type;
    Stamped<int32_t> dimension_to_bow_m;
    Stamped<int32_t> dimension_to_stern_m;
    Stamped<int32_t> dimension_to_port_m;
    Stamped<int32_t> dimension_to_starboard_m;
    Stamped<int32_t> epfd_type;
    bool dte_ready = false;
    bool assigned_mode = false;
    Stamped<int32_t> merge_mmsi;
    bool part_a_received = false;
    bool part_b_received = false;
    bool complete = false;
    uint64_t part_a_update_us = 0;
    uint64_t part_b_update_us = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisAidToNavigationData : AisMessageHeaderData<Real> {
    Stamped<int32_t> aid_type;
    char name[40] = {0};
    char name_extension[16] = {0};
    Stamped<int32_t> name_extension_char_count;
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
    bool name_extension_available = false;
    bool name_extension_valid = false;
    bool name_extension_truncated = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisLongRangeBroadcastData : AisMessageHeaderData<Real> {
    Stamped<int32_t> navigation_status;
    bool position_accuracy = false;
    bool raim = false;
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    Stamped<Real> speed_over_ground_kn;
    Stamped<Real> course_over_ground_deg;
    bool gnss_position_status = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisSafetyTextData : AisMessageHeaderData<Real> {
    Stamped<int32_t> destination_mmsi;
    Stamped<int32_t> sequence_number;
    bool retransmit = false;
    char text[96] = {0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisBinaryEnvelopeData : AisMessageHeaderData<Real> {
    Stamped<int32_t> destination_mmsi;
    Stamped<int32_t> sequence_number;
    Stamped<int32_t> dac;
    Stamped<int32_t> function_id;
    Stamped<int32_t> payload_bit_count;
    bool addressed = false;
    bool structured = false;
    bool retransmit = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisBinaryApplicationData : AisMessageHeaderData<Real> {
    Stamped<int32_t> dac;
    Stamped<int32_t> function_id;
    Stamped<int32_t> payload_bit_count;
    Stamped<int32_t> payload_start_bit;
    Stamped<int32_t> first_payload_bits;
    Stamped<int32_t> decoded_field_count;
    Stamped<int32_t> quantity;
    Stamped<int32_t> link_id;
    Stamped<int32_t> notice_type;
    Stamped<int32_t> day;
    Stamped<int32_t> hour;
    Stamped<int32_t> minute;
    Stamped<int32_t> duration_min;
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    char text[96] = {0};
    char application_label[32] = {0};
    bool known_application = false;
    bool addressed = false;
    bool structured = false;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisAcknowledgementData : AisMessageHeaderData<Real> {
    Stamped<int32_t> acknowledgement_count;
    Stamped<int32_t> destination_mmsi[4];
    Stamped<int32_t> sequence_number[4];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisUtcInquiryData : AisMessageHeaderData<Real> {
    Stamped<int32_t> destination_mmsi;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisInterrogationData : AisMessageHeaderData<Real> {
    Stamped<int32_t> first_destination_mmsi;
    Stamped<int32_t> first_message_type;
    Stamped<int32_t> first_slot_offset;
    Stamped<int32_t> second_message_type;
    Stamped<int32_t> second_slot_offset;
    Stamped<int32_t> second_destination_mmsi;
    Stamped<int32_t> third_message_type;
    Stamped<int32_t> third_slot_offset;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisAssignmentCommandData : AisMessageHeaderData<Real> {
    Stamped<int32_t> assignment_count;
    Stamped<int32_t> destination_mmsi[2];
    Stamped<int32_t> offset[2];
    Stamped<int32_t> increment[2];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisDataLinkManagementData : AisMessageHeaderData<Real> {
    Stamped<int32_t> reservation_count;
    Stamped<int32_t> slot_offset[4];
    Stamped<int32_t> slot_count[4];
    Stamped<int32_t> timeout_min[4];
    Stamped<int32_t> increment[4];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisDgnssBroadcastData : AisMessageHeaderData<Real> {
    Stamped<Real> longitude_deg;
    Stamped<Real> latitude_deg;
    Stamped<int32_t> payload_bit_count;
    Stamped<int32_t> payload_start_bit;
    Stamped<int32_t> payload_byte_count;
    Stamped<int32_t> first_payload_bits;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisChannelManagementData : AisMessageHeaderData<Real> {
    Stamped<int32_t> channel_a;
    Stamped<int32_t> channel_b;
    Stamped<int32_t> txrx_mode;
    bool high_power = false;
    bool addressed = false;
    Stamped<Real> northeast_lon_deg;
    Stamped<Real> northeast_lat_deg;
    Stamped<Real> southwest_lon_deg;
    Stamped<Real> southwest_lat_deg;
    Stamped<int32_t> first_destination_mmsi;
    Stamped<int32_t> second_destination_mmsi;
    bool bandwidth_a_12_5khz = false;
    bool bandwidth_b_12_5khz = false;
    Stamped<int32_t> transitional_zone_size_nmi;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AisGroupAssignmentCommandData : AisMessageHeaderData<Real> {
    Stamped<Real> northeast_lon_deg;
    Stamped<Real> northeast_lat_deg;
    Stamped<Real> southwest_lon_deg;
    Stamped<Real> southwest_lat_deg;
    Stamped<int32_t> station_type;
    Stamped<int32_t> ship_type;
    Stamped<int32_t> txrx_mode;
    Stamped<int32_t> report_interval;
    Stamped<int32_t> quiet_time_min;
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
    AisOwnVesselData<Real> own_vessel;
    AisTargetTableData<Real> targets;
    AisPositionReportData<Real> position_report;
    AisPositionReportData<Real> class_b_position_report;
    AisSarAircraftPositionData<Real> sar_aircraft_position;
    AisBaseStationData<Real> base_station;
    AisStaticVoyageData<Real> static_voyage;
    AisClassBStaticData<Real> class_b_static;
    AisAidToNavigationData<Real> aid_to_navigation;
    AisLongRangeBroadcastData<Real> long_range_broadcast;
    AisSafetyTextData<Real> safety_text;
    AisBinaryEnvelopeData<Real> binary_envelope;
    AisBinaryApplicationData<Real> binary_application;
    AisAcknowledgementData<Real> acknowledgement;
    AisUtcInquiryData<Real> utc_inquiry;
    AisInterrogationData<Real> interrogation;
    AisAssignmentCommandData<Real> assignment_command;
    AisDataLinkManagementData<Real> data_link_management;
    AisDgnssBroadcastData<Real> dgnss_broadcast;
    AisChannelManagementData<Real> channel_management;
    AisGroupAssignmentCommandData<Real> group_assignment;
    AisDataLinkStatusData<Real> data_link_status;
    AisAddressedSafetyData<Real> addressed_safety;
};

template<typename Real, uint8_t MaxTargets>
int32_t ais_find_target_index(const AisTargetTableData<Real, MaxTargets>& table, int32_t mmsi) {
    for (uint8_t i = 0; i < MaxTargets; ++i) {
        const auto& target = table.targets[i];
        if (target.occupied && target.mmsi.valid && target.mmsi.value == mmsi) return static_cast<int32_t>(i);
    }
    return -1;
}

template<typename Real, uint8_t MaxTargets>
bool ais_clear_target(AisTargetTableData<Real, MaxTargets>& table, int32_t mmsi, uint64_t now_us) {
    const int32_t index = ais_find_target_index(table, mmsi);
    if (index < 0) return false;
    table.targets[static_cast<uint8_t>(index)] = AisTargetData<Real>{};
    const int32_t count = table.target_count.valid ? table.target_count.value : 0;
    if (count > 0) table.target_count.set(count - 1, now_us);
    return true;
}

template<typename Real, uint8_t MaxTargets>
void ais_clear_targets(AisTargetTableData<Real, MaxTargets>& table) {
    table = AisTargetTableData<Real, MaxTargets>{};
}

template<typename Real, uint8_t MaxTargets>
int32_t ais_expire_targets_older_than(AisTargetTableData<Real, MaxTargets>& table,
                                     uint64_t cutoff_us,
                                     uint64_t now_us) {
    int32_t expired = 0;
    int32_t remaining = 0;
    for (uint8_t i = 0; i < MaxTargets; ++i) {
        auto& target = table.targets[i];
        if (!target.occupied) continue;
        if (target.last_seen_us != 0 && target.last_seen_us < cutoff_us) {
            target = AisTargetData<Real>{};
            ++expired;
        } else {
            ++remaining;
        }
    }
    table.target_count.set(remaining, now_us);
    return expired;
}

template<typename Real, uint8_t MaxTargets>
int32_t ais_mark_targets_stale_older_than(AisTargetTableData<Real, MaxTargets>& table,
                                         uint64_t cutoff_us,
                                         uint64_t now_us) {
    int32_t marked = 0;
    for (uint8_t i = 0; i < MaxTargets; ++i) {
        auto& target = table.targets[i];
        if (!target.occupied || target.stale) continue;
        if (target.last_seen_us != 0 && target.last_seen_us < cutoff_us) {
            target.stale = true;
            target.last_seen_us = target.last_seen_us == 0 ? now_us : target.last_seen_us;
            ++marked;
        }
    }
    return marked;
}

template<typename Real>
int32_t ais_find_target_index(const AisTargetTableData<Real>& table, int32_t mmsi) {
    return ais_find_target_index<Real, AIS_TARGET_TABLE_CAPACITY>(table, mmsi);
}

template<typename Real>
bool ais_clear_target(AisTargetTableData<Real>& table, int32_t mmsi, uint64_t now_us) {
    return ais_clear_target<Real, AIS_TARGET_TABLE_CAPACITY>(table, mmsi, now_us);
}

template<typename Real>
void ais_clear_targets(AisTargetTableData<Real>& table) {
    ais_clear_targets<Real, AIS_TARGET_TABLE_CAPACITY>(table);
}

template<typename Real>
int32_t ais_expire_targets_older_than(AisTargetTableData<Real>& table, uint64_t cutoff_us, uint64_t now_us) {
    return ais_expire_targets_older_than<Real, AIS_TARGET_TABLE_CAPACITY>(table, cutoff_us, now_us);
}

template<typename Real>
int32_t ais_mark_targets_stale_older_than(AisTargetTableData<Real>& table, uint64_t cutoff_us, uint64_t now_us) {
    return ais_mark_targets_stale_older_than<Real, AIS_TARGET_TABLE_CAPACITY>(table, cutoff_us, now_us);
}

} // namespace ship_data_model
