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
    Stamped<Real> leeway_deg;
    Stamped<Real> current_speed_kn;
    Stamped<Real> current_direction_deg;
    Stamped<Real> wind_speed_kn;
    Stamped<Real> wind_direction_deg;
    Stamped<Real> depth_m;
    Stamped<Real> depth_offset_m;
    uint64_t last_update_us = 0;
};

} // namespace ship_data_model
