#pragma once

#include "../core_data_types.hpp"

namespace ship_data_model {

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
struct PropulsionData {
    RevolutionsData<Real> revolutions;
};

} // namespace ship_data_model
