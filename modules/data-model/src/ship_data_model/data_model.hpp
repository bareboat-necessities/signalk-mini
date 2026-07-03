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
struct NotificationTextData {
    Setting<SensorSource> source;
    char id[24] = {0};
    char code[16] = {0};
    char value[16] = {0};
    char text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlarmNotificationsData {
    NotificationTextData<Real> acknowledgement;
    NotificationTextData<Real> alarm;
    NotificationTextData<Real> equipment_status;
    NotificationTextData<Real> event_log;
    NotificationTextData<Real> event;
    NotificationTextData<Real> fire;
};

template<typename Real = float>
struct MessageNotificationsData {
    NotificationTextData<Real> text;
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
};

} // namespace ship_data_model

#include "data_model_mapping.hpp"
