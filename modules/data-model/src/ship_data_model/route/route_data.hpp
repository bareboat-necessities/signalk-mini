#pragma once

#include "../core_data_types.hpp"
#include "../autopilot/autopilot_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct HeadingSteeringCommandData {
    Setting<SensorSource> source;
    Stamped<Real> heading_true_deg;
    Stamped<Real> heading_magnetic_deg;
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
struct RouteLogData {
    Setting<SensorSource> source;
    Stamped<Real> total_distance_nmi;
    Stamped<Real> trip_distance_nmi;
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

template<typename Real = float>
struct RouteData {
    ActiveRouteData<Real> active;
    RouteLogData<Real> log;
    WaypointArrivalData<Real> waypoint_arrival;
    WaypointNavigationData<Real> waypoint;
    ApbData<Real> apb;
    RmbData<Real> rmb;
    HeadingSteeringCommandData<Real> heading_steering_command;
};

} // namespace ship_data_model
