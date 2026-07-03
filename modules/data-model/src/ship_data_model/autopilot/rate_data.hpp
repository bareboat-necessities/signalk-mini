#pragma once

#include "pilot_term.hpp"

namespace ship_data_model {

#define SHIP_DATA_MODEL_JOIN_FIELD(a, b) a##b

template<typename Real = float>
struct RatePilotData {
    PilotTerm<Real> D;
    PilotTerm<Real> DD;
    PilotTerm<Real> SHIP_DATA_MODEL_JOIN_FIELD(F, F);
    RangeSetting<Real> max_turn_rate_deg_s;
    RangeSetting<Real> turn_rate_rate_deg_s2;
};

#undef SHIP_DATA_MODEL_JOIN_FIELD

} // namespace ship_data_model
