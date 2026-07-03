#pragma once

#include "pilot_term.hpp"

namespace ship_data_model {

template<typename Real = float>
struct SimplePilotData {
    PilotTerm<Real> P;
    PilotTerm<Real> I;
    PilotTerm<Real> D;
};

} // namespace ship_data_model
