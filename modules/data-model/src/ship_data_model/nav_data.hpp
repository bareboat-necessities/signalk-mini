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
    Stamped<Real> track_deg;
    Stamped<Real> speed_kn;

    Stamped<Real> fix_lat_deg;
    Stamped<Real> fix_lon_deg;
    Stamped<Real> fix_alt_m;
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
    WaypointArrivalData<Real> waypoint_arrival;
    WaypointNavigationData<Real> waypoint;
    ApbData<Real> apb;
    RmbData<Real> rmb;
};

} // namespace ship_data_model
