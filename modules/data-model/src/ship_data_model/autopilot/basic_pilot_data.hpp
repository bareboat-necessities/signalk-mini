#pragma once

#include "pilot_term.hpp"

namespace ship_data_model {

#define SHIP_DATA_MODEL_JOIN_FIELD(a, b) a##b

template<typename Real = float>
struct BasicPilotData {
    PilotTerm<Real> P;
    PilotTerm<Real> I;
    PilotTerm<Real> D;
    PilotTerm<Real> DD;
    PilotTerm<Real> PR;
    PilotTerm<Real> SHIP_DATA_MODEL_JOIN_FIELD(F, F);
};

#undef SHIP_DATA_MODEL_JOIN_FIELD

} // namespace ship_data_model
