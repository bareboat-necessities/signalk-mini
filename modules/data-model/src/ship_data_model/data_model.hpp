#pragma once

#include "autopilot_data.hpp"
#include "ship_imu_data.hpp"
#include "nav_data.hpp"
#include "wind_sea_data.hpp"
#include "steering_data.hpp"

namespace ship_data_model {

template<typename Real = float>
struct DataModel {
    AutopilotServerData<Real> server;
    AutopilotStatusData<Real> status;

    AutopilotData<Real> ap;
    NavigationData<Real> navigation;
    WindData<Real> wind;
    WaterData<Real> water;
    ShipImuData<Real> imu;
    ImuStateData<Real> imu_state;
    ImuCalibrationData<Real> imu_calibration;

    RudderData<Real> rudder;
    ServoData<Real> servo;
    ServoTelemetryData<Real> servo_telemetry;
    ServoCalibrationData<Real> servo_calibration;

    PilotsData<Real> pilots;
    PilotCommandData<Real> pilot_command;
    PilotOutputData<Real> pilot_output;
    TackData<Real> tack;

    AutopilotProfileData<Real> profile;
    ValuePublicationState<Real> runtime_publication;
};

} // namespace ship_data_model

#include "data_model_mapping.hpp"
