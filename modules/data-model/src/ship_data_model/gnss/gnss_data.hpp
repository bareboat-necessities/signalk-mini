#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct GnssData {
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
    char fix_mode_indicator[8] = {0};
    char navigational_status = 0;
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
struct GnssAlmanacData {
    Setting<SensorSource> source;
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    Stamped<int32_t> satellite_prn;
    Stamped<int32_t> gnss_week;
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
struct GnssFaultDetectionData {
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
struct GnssRangeResidualData {
    Setting<SensorSource> source;
    Stamped<Real> utc_time_s;
    Stamped<int32_t> mode;
    Stamped<Real> satellite_residual_m[12];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GnssNoiseStatisticsData {
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
struct GnssDopActiveSatellitesData {
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
struct GnssSatellitesInViewData {
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
struct GnssModelData {
    GnssData<Real> fix;
    GnssAlmanacData<Real> almanac;
    GnssFaultDetectionData<Real> fault_detection;
    GnssRangeResidualData<Real> range_residual;
    GnssNoiseStatisticsData<Real> noise_statistics;
    GnssDopActiveSatellitesData<Real> dop_active_satellites;
    GnssSatellitesInViewData<Real> satellites_in_view;
};

} // namespace ship_data_model
