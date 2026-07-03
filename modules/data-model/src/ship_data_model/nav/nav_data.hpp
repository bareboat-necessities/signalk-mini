#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct DatumReferenceData {
    Setting<SensorSource> source;
    char local_datum_code[8] = {0};
    char local_datum_subcode[8] = {0};
    Stamped<Real> latitude_offset_min;
    Stamped<Real> longitude_offset_min;
    Stamped<Real> altitude_offset_m;
    char reference_datum_code[8] = {0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct DeccaPositionData {
    Setting<SensorSource> source;
    char chain_id[8] = {0};
    char red_zone[8] = {0};
    char green_zone[8] = {0};
    char purple_zone[8] = {0};
    Stamped<Real> red_line_of_position;
    Stamped<Real> green_line_of_position;
    Stamped<Real> purple_line_of_position;
    char red_master_status = 0;
    char green_master_status = 0;
    char purple_master_status = 0;
    char red_line_navigation_use = 0;
    char green_line_navigation_use = 0;
    char purple_line_navigation_use = 0;
    Stamped<Real> position_uncertainty_nmi;
    Stamped<int32_t> fix_data_basis;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct LegacyTimingData {
    Setting<SensorSource> source;
    Stamped<Real> gri_us_div_10;
    Stamped<Real> master_toa_us;
    Stamped<Real> master_relative_snr_db;
    Stamped<Real> master_relative_ecd;
    char master_toa_status = 0;
    Stamped<Real> delta_us[5];
    char delta_status[5] = {0, 0, 0, 0, 0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct LegacyDeltaData {
    Setting<SensorSource> source;
    Stamped<Real> value[5];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct TransitFixData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    char date_ddmmyy[8] = {0};
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    char waypoint_id[16] = {0};
    Stamped<Real> elevation_angle_deg;
    Stamped<int32_t> iterations;
    Stamped<int32_t> doppler_intervals;
    Stamped<Real> update_distance_nmi;
    Stamped<int32_t> satellite_number;
    char data_validity = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct OmegaLaneNumbersData {
    Setting<SensorSource> source;
    char pair[3][16] = {{0}, {0}, {0}};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct OwnShipData {
    Setting<SensorSource> source;
    Stamped<Real> heading_true_deg;
    char heading_status = 0;
    Stamped<Real> course_deg;
    char course_reference = 0;
    Stamped<Real> speed_kn;
    char speed_reference = 0;
    Stamped<Real> set_true_deg;
    Stamped<Real> drift_speed_kn;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct RadarSystemData {
    Setting<SensorSource> source;
    Stamped<Real> origin_range_nmi[2];
    Stamped<Real> origin_bearing_deg[2];
    Stamped<Real> variable_range_marker_nmi[2];
    Stamped<Real> electronic_bearing_line_deg[2];
    Stamped<Real> cursor_range_nmi;
    Stamped<Real> cursor_bearing_deg;
    Stamped<Real> range_scale_nmi;
    char range_units = 0;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct ScanningFrequencyData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    Stamped<Real> frequency_hz[6];
    char mode[6] = {0, 0, 0, 0, 0, 0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct MultipleDataIdData {
    Setting<SensorSource> source;
    Stamped<int32_t> talker_id_number;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct RmaData {
    Setting<SensorSource> source;
    char status = 0;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> time_difference_a_us;
    Stamped<Real> time_difference_b_us;
    Stamped<Real> speed_kn;
    Stamped<Real> track_deg;
    Stamped<Real> magnetic_variation_deg;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NavData {
    DatumReferenceData<Real> datum;
    DeccaPositionData<Real> decca;
    LegacyTimingData<Real> legacy_timing;
    LegacyDeltaData<Real> legacy_delta;
    TransitFixData<Real> transit_fix;
    OmegaLaneNumbersData<Real> omega_lane_numbers;
    OwnShipData<Real> own_ship;
    RadarSystemData<Real> radar_system;
    ScanningFrequencyData<Real> scanning_frequency;
    MultipleDataIdData<Real> multiple_data_id;
    RmaData<Real> rma;
};

} // namespace ship_data_model
