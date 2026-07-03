#pragma once

#include "pilot_term.hpp"

namespace ship_data_model {

#define SHIP_DATA_MODEL_JOIN_FIELD(a, b) a##b

template<typename Real = float>
struct WindPilotData {
    PilotTerm<Real> P;
    PilotTerm<Real> D;
    PilotTerm<Real> DD;
    PilotTerm<Real> SHIP_DATA_MODEL_JOIN_FIELD(W, G);
};

#undef SHIP_DATA_MODEL_JOIN_FIELD

} // namespace ship_data_model
