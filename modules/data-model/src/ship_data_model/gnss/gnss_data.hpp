#pragma once

#include <stddef.h>
#include "../core_data_types.hpp"

namespace ship_data_model {

#if defined(ARDUINO)
inline constexpr size_t DefaultGnssSkyViewCapacity = 24;
#else
inline constexpr size_t DefaultGnssSkyViewCapacity = 64;
#endif

template<typename Real = float>
struct GnssSatelliteObservation {
    int16_t satellite_id = -1;
    int16_t system_id = -1;
    int16_t signal_id = -1;

    Real elevation_deg{};
    Real azimuth_true_deg{};
    Real cn0_db_hz{};

    bool satellite_id_valid = false;
    bool elevation_valid = false;
    bool azimuth_valid = false;
    bool cn0_valid = false;
    bool used = false;
    bool healthy = false;
    bool differential_corrections = false;
};

template<typename Real = float, size_t Capacity = DefaultGnssSkyViewCapacity>
struct GnssSkyViewData {
    Setting<SensorSource> source;
    char talker_id[3] = {0};

    Stamped<int32_t> satellites_in_view;
    Stamped<int32_t> satellites_used;
    GnssSatelliteObservation<Real> observations[Capacity]{};
    uint16_t observation_count = 0;
    bool complete = false;
    uint32_t sequence = 0;
    uint64_t last_update_us = 0;

    static constexpr size_t capacity() { return Capacity; }

    void clear_observations() {
        for (size_t i = 0; i < Capacity; ++i) observations[i] = GnssSatelliteObservation<Real>{};
        observation_count = 0;
        complete = false;
    }
};

template<typename Real = float>
struct GnssData {
    Setting<SensorSource> source;
    char talker_id[3] = {0};
    RangeSetting<Real> rate_hz;

    Stamped<Real> timestamp_s;
    Stamped<int32_t> date_day;
    Stamped<int32_t> date_month;
    Stamped<int32_t> date_year;
    Stamped<int32_t> local_zone_hours;
    Stamped<int32_t> local_zone_minutes;
    Stamped<Real> track_deg;
    Stamped<Real> speed_kn;
    Stamped<Real> vertical_speed_m_s;

    Stamped<Real> fix_lat_deg;
    Stamped<Real> fix_lon_deg;
    Stamped<Real> fix_alt_msl_m;
    Stamped<Real> fix_alt_hae_m;
    Stamped<int32_t> fix_quality;
    Stamped<int32_t> fix_type;
    Stamped<bool> fix_valid;
    char fix_mode_indicator[8] = {0};
    char navigational_status = 0;
    char mode_indicator = 0;
    Stamped<int32_t> system_id;
    Stamped<int32_t> signal_id;
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
    char talker_id[3] = {0};
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
    char talker_id[3] = {0};
    Stamped<Real> utc_time_s;
    Stamped<Real> expected_error_lat_m;
    Stamped<Real> expected_error_lon_m;
    Stamped<Real> expected_error_alt_m;
    Stamped<int32_t> failed_satellite_prn;
    Stamped<Real> missed_detection_probability;
    Stamped<Real> failed_satellite_bias_m;
    Stamped<Real> failed_satellite_bias_stddev_m;
    Stamped<int32_t> system_id;
    Stamped<int32_t> signal_id;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GnssFixAccuracyData {
    Setting<SensorSource> source;
    char talker_id[3] = {0};
    Stamped<Real> utc_time_s;
    char mode_indicator = 0;
    char status = 0;
    Stamped<Real> horizontal_accuracy_m;
    Stamped<Real> vertical_accuracy_m;
    Stamped<Real> speed_accuracy_m_s;
    Stamped<Real> track_accuracy_deg;
    Stamped<Real> time_accuracy_s;
    Stamped<Real> semi_major_accuracy_m;
    Stamped<Real> semi_minor_accuracy_m;
    Stamped<Real> semi_major_orientation_deg;
    Stamped<Real> pdop;
    Stamped<Real> hdop;
    Stamped<Real> vdop;
    Stamped<int32_t> system_id;
    Stamped<int32_t> signal_id;
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GnssRangeResidualData {
    Setting<SensorSource> source;
    char talker_id[3] = {0};
    Stamped<Real> utc_time_s;
    Stamped<int32_t> mode;
    Stamped<Real> satellite_residual_m[12];
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GnssNoiseStatisticsData {
    Setting<SensorSource> source;
    char talker_id[3] = {0};
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
    char talker_id[3] = {0};
    char selection_mode = 0;
    Stamped<int32_t> fix_mode;
    Stamped<int32_t> satellite_prn[12];
    Stamped<Real> pdop;
    Stamped<Real> hdop;
    Stamped<Real> vdop;
    Stamped<int32_t> system_id;
    Stamped<int32_t> signal_id;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GnssSatellitesInViewData {
    Setting<SensorSource> source;
    char talker_id[3] = {0};
    Stamped<int32_t> total_messages;
    Stamped<int32_t> message_number;
    Stamped<int32_t> satellites_in_view;
    Stamped<int32_t> satellite_prn[4];
    Stamped<Real> elevation_deg[4];
    Stamped<Real> azimuth_true_deg[4];
    Stamped<Real> snr_db[4];
    Stamped<int32_t> system_id;
    Stamped<int32_t> signal_id;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct GnssModelData {
    GnssData<Real> fix;
    GnssAlmanacData<Real> almanac;
    GnssFaultDetectionData<Real> fault_detection;
    GnssFixAccuracyData<Real> fix_accuracy;
    GnssRangeResidualData<Real> range_residual;
    GnssNoiseStatisticsData<Real> noise_statistics;
    GnssDopActiveSatellitesData<Real> dop_active_satellites;
    GnssSatellitesInViewData<Real> satellites_in_view;
    GnssSkyViewData<Real> sky_view;
};

} // namespace ship_data_model
