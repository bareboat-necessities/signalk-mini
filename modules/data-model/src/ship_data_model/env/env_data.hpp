#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct EnvData {
    Setting<SensorSource> source;
    Stamped<Real> barometric_pressure_inhg;
    Stamped<Real> barometric_pressure_bar;
    Stamped<Real> air_temperature_c;
    Stamped<Real> relative_humidity_percent;
    Stamped<Real> absolute_humidity_percent;
    Stamped<Real> dew_point_c;
    uint64_t last_update_us = 0;
};

} // namespace ship_data_model
