#pragma once

#include "autopilot_data.hpp"
#include "../steering/steering_data.hpp"

namespace ship_data_model {

template<typename Real = float>
struct AutopilotModelData {
    AutopilotServerData<Real> server;
    AutopilotStatusData<Real> status;
    AutopilotData<Real> controller;
    PilotsData<Real> pilots;
    PilotCommandData<Real> pilot_command;
    PilotOutputData<Real> pilot_output;
    TackData<Real> tack;
    AutopilotProfileData<Real> profile;
    ValuePublicationState<Real> runtime_publication;
};

} // namespace ship_data_model
