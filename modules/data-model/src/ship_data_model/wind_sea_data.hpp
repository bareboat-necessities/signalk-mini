#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct WindSensorData {
    Setting<SensorSource> source;
    Stamped<Real> direction_deg;
    Stamped<Real> filtered_direction_deg;
    Stamped<Real> speed_kn;
    Stamped<Real> speed_m_s;
    Stamped<Real> filtered_speed_kn;
    RangeSetting<Real> filter_constant_0_1;
    Stamped<Real> filter_factor_0_1;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct WindData {
    WindSensorData<Real> apparent;
    WindSensorData<Real> truewind;
};

template<typename Real = float>
struct WaterData {
    Setting<SensorSource> source;
    Setting<SensorSource> leeway_source;
    Setting<SensorSource> depth_source;
    Stamped<Real> speed_kn;
    Stamped<Real> longitudinal_water_speed_kn;
    Stamped<Real> transverse_water_speed_kn;
    char water_speed_status = 0;
    Stamped<Real> longitudinal_ground_speed_kn;
    Stamped<Real> transverse_ground_speed_kn;
    char ground_speed_status = 0;
    Stamped<Real> speed_parallel_to_wind_kn;
    Stamped<Real> speed_parallel_to_wind_m_s;
    Stamped<Real> total_distance_nmi;
    Stamped<Real> trip_distance_nmi;
    Stamped<Real> leeway_deg;
    Stamped<Real> current_speed_kn;
    Stamped<Real> current_direction_deg;
    Stamped<Real> current_direction_magnetic_deg;
    Stamped<Real> wind_speed_kn;
    Stamped<Real> wind_speed_m_s;
    Stamped<Real> wind_direction_deg;
    Stamped<Real> wind_direction_magnetic_deg;
    Stamped<Real> barometric_pressure_inhg;
    Stamped<Real> barometric_pressure_bar;
    Stamped<Real> air_temperature_c;
    Stamped<Real> relative_humidity_percent;
    Stamped<Real> absolute_humidity_percent;
    Stamped<Real> dew_point_c;
    Stamped<Real> depth_m;
    Stamped<Real> depth_below_keel_m;
    Stamped<Real> depth_below_surface_m;
    Stamped<Real> depth_offset_m;
    Stamped<Real> temperature_c;
    Stamped<Real> trawl_headrope_to_footrope_m;
    Stamped<Real> trawl_headrope_to_bottom_m;
    Stamped<Real> trawl_door_centerline_offset_m;
    Stamped<Real> trawl_door_along_centerline_m;
    Stamped<Real> trawl_cartesian_centerline_offset_m;
    Stamped<Real> trawl_cartesian_along_centerline_m;
    Stamped<int32_t> trawl_catch_sensor_status[3];
    Stamped<Real> trawl_depth_below_surface_m;
    Stamped<Real> trawl_relative_range_m;
    Stamped<Real> trawl_relative_bearing_deg;
    Stamped<Real> trawl_true_range_m;
    Stamped<Real> trawl_true_bearing_deg;
    uint64_t last_update_us = 0;
};

} // namespace ship_data_model
