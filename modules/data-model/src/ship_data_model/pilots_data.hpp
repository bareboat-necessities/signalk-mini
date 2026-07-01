#pragma once

#include "basic_pilot_data.hpp"
#include "wind_pilot_data.hpp"
#include "rate_data.hpp"

namespace ship_data_model {

#define SHIP_DATA_MODEL_JOIN_FIELD(a, b) a##b

template<typename Real = float>
struct PilotsData {
    BasicPilotData<Real> basic;
    BasicPilotData<Real> absolute;
    WindPilotData<Real> wind;
    BasicPilotData<Real> gps;
    BasicPilotData<Real> simple;
    RatePilotData<Real> rate;
    BasicPilotData<Real> SHIP_DATA_MODEL_JOIN_FIELD(v, mg);
};

#undef SHIP_DATA_MODEL_JOIN_FIELD

} // namespace ship_data_model
