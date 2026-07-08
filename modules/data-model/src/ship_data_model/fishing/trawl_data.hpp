#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct TrawlData {
    Setting<SensorSource> source;
    Stamped<Real> headrope_to_footrope_m;
    Stamped<Real> headrope_to_bottom_m;
    Stamped<Real> door_centerline_offset_m;
    Stamped<Real> door_along_centerline_m;
    Stamped<Real> cartesian_centerline_offset_m;
    Stamped<Real> cartesian_along_centerline_m;
    Stamped<int32_t> catch_sensor_status[3];
    Stamped<Real> depth_below_surface_m;
    Stamped<Real> relative_range_m;
    Stamped<Real> relative_bearing_deg;
    Stamped<Real> true_range_m;
    Stamped<Real> true_bearing_deg;
    uint64_t last_update_us = 0;
};

} // namespace ship_data_model
