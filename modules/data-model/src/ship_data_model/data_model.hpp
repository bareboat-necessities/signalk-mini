#pragma once

#include "autopilot_data.hpp"
#include "ship_imu_data.hpp"
#include "nav_data.hpp"
#include "wind_sea_data.hpp"
#include "steering_data.hpp"

namespace ship_data_model {

template<typename Real = float>
struct SteeringModelData {
    RudderData<Real> rudder;
    ServoData<Real> servo;
    ServoTelemetryData<Real> servo_telemetry;
    ServoCalibrationData<Real> servo_calibration;
};

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

template<typename Real = float>
struct FluidsData {
};

template<typename Real = float>
struct BatteriesData {
};

template<typename Real = float>
struct ElectricalData {
};

template<typename Real = float>
struct AlarmNotificationsData {
};

template<typename Real = float>
struct MessageNotificationsData {
};

template<typename Real = float>
struct NotificationsData {
    AlarmNotificationsData<Real> alarms;
    MessageNotificationsData<Real> messages;
};

template<typename Real = float>
struct DataModel {
    NavData<Real> nav;
    RouteData<Real> route;
    GnssModelData<Real> gnss;
    AisData<Real> ais;

    WindData<Real> wind;
    SeaData<Real> sea;
    EnvData<Real> env;

    PropulsionData<Real> propulsion;
    SteeringModelData<Real> steering;
    AutopilotModelData<Real> autopilot;

    FluidsData<Real> fluids;
    BatteriesData<Real> batteries;
    ElectricalData<Real> electrical;
    CommData<Real> comm;
    NotificationsData<Real> notifications;

    // IMU remains in top-level storage until the inertial/navigation split is modeled separately.
    ShipImuData<Real> imu;
    ImuStateData<Real> imu_state;
    ImuCalibrationData<Real> imu_calibration;

    // Compatibility references for older modules during the reorganization.
    AutopilotServerData<Real>& server;
    AutopilotStatusData<Real>& status;
    AutopilotData<Real>& ap;
    NavigationData<Real> navigation;
    SeaData<Real>& water;
    RudderData<Real>& rudder;
    ServoData<Real>& servo;
    ServoTelemetryData<Real>& servo_telemetry;
    ServoCalibrationData<Real>& servo_calibration;
    PilotsData<Real>& pilots;
    PilotCommandData<Real>& pilot_command;
    PilotOutputData<Real>& pilot_output;
    TackData<Real>& tack;
    AutopilotProfileData<Real>& profile;
    ValuePublicationState<Real>& runtime_publication;

    DataModel()
        : env(sea),
          server(autopilot.server),
          status(autopilot.status),
          ap(autopilot.controller),
          navigation(nav, route, gnss, ais, propulsion, comm),
          water(sea),
          rudder(steering.rudder),
          servo(steering.servo),
          servo_telemetry(steering.servo_telemetry),
          servo_calibration(steering.servo_calibration),
          pilots(autopilot.pilots),
          pilot_command(autopilot.pilot_command),
          pilot_output(autopilot.pilot_output),
          tack(autopilot.tack),
          profile(autopilot.profile),
          runtime_publication(autopilot.runtime_publication) {}

    DataModel(const DataModel&) = delete;
    DataModel& operator=(const DataModel&) = delete;
};

} // namespace ship_data_model

#include "data_model_mapping.hpp"
