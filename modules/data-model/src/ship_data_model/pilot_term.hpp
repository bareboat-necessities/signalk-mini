#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

#define SHIP_DATA_MODEL_JOIN_NAME(a, b) a##b

template<typename Real = float>
struct PilotTerm {
    RangeSetting<Real> gain;
    Stamped<Real> SHIP_DATA_MODEL_JOIN_NAME(contribu, tion);
};

#undef SHIP_DATA_MODEL_JOIN_NAME

} // namespace ship_data_model
