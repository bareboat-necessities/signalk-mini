#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct NavigationData;

template<typename Real = float>
struct GpsData {
    Setting<SensorSource> source;
    RangeSetting<Real> rate_hz;

    Stamped<Real> timestamp_s;
    Stamped<int32_t> date_day;
    Stamped<int32_t> date_month;
    Stamped<int32_t> date_year;
    Stamped<int32_t> local_zone_hours;
    Stamped<int32_t> local_zone_minutes;
    Stamped<Real> track_deg;
    Stamped<Real> speed_kn;

    Stamped<Real> fix_lat_deg;
    Stamped<Real> fix_lon_deg;
    Stamped<Real> fix_alt_m;
    Stamped<int32_t> fix_quality;
    Stamped<int32_t> satellites_used;
    Stamped<Real> hdop;
    Stamped<Real> geoidal_separation_m;
    Stamped<Real> dgps_age_s;
    Stamped<int32_t> dgps_station_id;
    bool has_fix_json = false;

    Stamped<Real> leeway_ground_deg;
    Stamped<Real> compass_error_deg;
    Stamped<Real> declination_deg;
    Setting<int32_t> alignment_counter;

    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GpsAlmanacData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    Stamped<int32_t> satellite_prn;
    Stamped<int32_t> gps_week;
    Stamped<int32_t> sv_health;
    Stamped<Real> eccentricity;
    Stamped<Real> reference_time_s;
    Stamped<Real> inclination_rad;
    Stamped<Real> right_ascension_rate_rad_s;
    Stamped<Real> sqrt_semi_major_axis;
    Stamped<Real> argument_of_perigee_rad;
    Stamped<Real> longitude_ascension_node_rad;
    Stamped<Real> mean_anomaly_rad;
    Stamped<Real> clock_f0_s;
    Stamped<Real> clock_f1_s_s;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GpsFaultDetectionData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    Stamped<Real> expected_error_lat_m;
    Stamped<Real> expected_error_lon_m;
    Stamped<Real> expected_error_alt_m;
    Stamped<int32_t> failed_satellite_prn;
    Stamped<Real> missed_detection_probability;
    Stamped<Real> failed_satellite_bias_m;
    Stamped<Real> failed_satellite_bias_stddev_m;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GpsRangeResidualData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    Stamped<int32_t> mode;
    Stamped<Real> satellite_residual_m[12];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GpsNoiseStatisticsData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    Stamped<Real> rms_range_stddev_m;
    Stamped<Real> semi_major_stddev_m;
    Stamped<Real> semi_minor_stddev_m;
    Stamped<Real> semi_major_orientation_deg;
    Stamped<Real> latitude_error_stddev_m;
    Stamped<Real> longitude_error_stddev_m;
    Stamped<Real> altitude_error_stddev_m;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GpsDopActiveSatellitesData {
    Setting<SensorSource> source;
    char selection_mode = 0;
    Stamped<int32_t> fix_mode;
    Stamped<int32_t> satellite_prn[12];
    Stamped<Real> pdop;
    Stamped<Real> hdop;
    Stamped<Real> vdop;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GpsSatellitesInViewData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    Stamped<int32_t> satellites_in_view;
    Stamped<int32_t> satellite_prn[4];
    Stamped<Real> elevation_deg[4];
    Stamped<Real> azimuth_true_deg[4];
    Stamped<Real> snr_db[4];
    uint64_t last_update_us = 0;
};

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
struct RadioFrequencySetData {
    Setting<SensorSource> source;
    Stamped<Real> transmitting_frequency_hz;
    Stamped<Real> receiving_frequency_hz;
    char communication_mode = 0;
    Stamped<int32_t> power_level;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct HeadingSteeringCommandData {
    Setting<SensorSource> source;
    Stamped<Real> heading_true_deg;
    Stamped<Real> heading_magnetic_deg;
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
struct ActiveRouteData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    char mode = 0;
    char waypoint_id[16][16] = {{0}};
    Stamped<int32_t> waypoint_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct RevolutionsData {
    Setting<SensorSource> source;
    char source_type = 0;
    Stamped<int32_t> number;
    Stamped<Real> speed_rpm;
    Stamped<Real> propeller_pitch_percent;
    char status = 0;
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
struct TrackedTargetData {
    Setting<SensorSource> source;
    Stamped<int32_t> target_number;
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
struct WaypointArrivalData {
    Setting<SensorSource> source;
    Setting<bool> arrival_circle_entered;
    Setting<bool> perpendicular_passed;
    Stamped<Real> arrival_radius_nmi;
    char waypoint_id[16] = {0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct WaypointNavigationData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    Stamped<Real> origin_utc_time_s;
    Stamped<Real> origin_elapsed_time_s;
    Stamped<Real> destination_utc_time_s;
    Stamped<Real> destination_time_remaining_s;
    Stamped<Real> latitude_deg;
    Stamped<Real> longitude_deg;
    Stamped<Real> bearing_true_deg;
    Stamped<Real> bearing_magnetic_deg;
    Stamped<Real> distance_nmi;
    char to_waypoint_id[16] = {0};
    char from_waypoint_id[16] = {0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct ApbData {
    Setting<SensorSource> source;
    Stamped<Real> track_deg;
    Stamped<Real> xte_nmi;
    Stamped<Real> origin_to_destination_bearing_deg;
    Stamped<Real> present_to_destination_bearing_deg;
    Stamped<Real> heading_to_steer_deg;
    RangeSetting<Real> xte_gain_deg_per_nmi;
    Setting<bool> arrival_circle_entered;
    Setting<bool> perpendicular_passed;
    Setting<AutopilotMode> mode_hint;
    char destination_id[16] = {0};
    char sender_id[3] = {0, 0, 0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct RmbData {
    Setting<SensorSource> source;
    char origin_id[16] = {0};
    char destination_id[16] = {0};
    Stamped<Real> xte_nmi;
    Stamped<Real> destination_lat_deg;
    Stamped<Real> destination_lon_deg;
    Stamped<Real> range_nmi;
    Stamped<Real> bearing_deg;
    Stamped<Real> closing_velocity_kn;
    Setting<bool> arrived;
    uint64_t last_update_us = 0;
};

template<typename Real>
struct NavigationData {
    GpsData<Real> gps;
    GpsAlmanacData<Real> gps_almanac;
    GpsFaultDetectionData<Real> gps_fault;
    GpsRangeResidualData<Real> gps_range_residual;
    GpsNoiseStatisticsData<Real> gps_noise;
    GpsDopActiveSatellitesData<Real> gps_dop;
    GpsSatellitesInViewData<Real> gps_satellites_in_view;
    DatumReferenceData<Real> datum;
    DeccaPositionData<Real> decca;
    LegacyTimingData<Real> legacy_timing;
    LegacyDeltaData<Real> legacy_delta;
    TransitFixData<Real> transit_fix;
    RadioFrequencySetData<Real> radio_frequency_set;
    HeadingSteeringCommandData<Real> heading_steering_command;
    BeaconReceiverControlData<Real> beacon_control;
    BeaconReceiverStatusData<Real> beacon_status;
    OmegaLaneNumbersData<Real> omega_lane_numbers;
    OwnShipData<Real> own_ship;
    ActiveRouteData<Real> active_route;
    RevolutionsData<Real> revolutions;
    RadarSystemData<Real> radar_system;
    ScanningFrequencyData<Real> scanning_frequency;
    MultipleDataIdData<Real> multiple_data_id;
    TrackedTargetData<Real> tracked_target;
    RmaData<Real> rma;
    WaypointArrivalData<Real> waypoint_arrival;
    WaypointNavigationData<Real> waypoint;
    ApbData<Real> apb;
    RmbData<Real> rmb;
};

} // namespace ship_data_model
