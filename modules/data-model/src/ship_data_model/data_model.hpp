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
struct InsData {
    ShipImuData<Real> imu;
    ImuStateData<Real> imu_state;
    ImuCalibrationData<Real> imu_calibration;
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
struct AlertAcknowledgementData {
    Setting<SensorSource> source;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertAcknowledgementDetailData {
    Setting<SensorSource> source;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    Stamped<int32_t> alert_instance;
    char acknowledgement_state = 0;
    char operator_id[24] = {0};
    char detail[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct AlertConditionData {
    Setting<SensorSource> source;
    char alarm_identifier[24] = {0};
    Stamped<int32_t> local_alarm_number;
    Stamped<int32_t> alert_instance;
    char condition_state = 0;
    char priority = 0;
    char description[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct NotificationAlertsData {
    AlertAcknowledgementData<Real> acknowledgement;
    AlertAcknowledgementDetailData<Real> acknowledgement_detail;
    AlertConditionData<Real> condition;
    NmeaTextRecordData<Real> fire;
};

template<typename Real = float>
struct NotificationMessagesData {
    NmeaTextRecordData<Real> text;
    NmeaTextRecordData<Real> event_log;
    NmeaTextRecordData<Real> event;
};

template<typename Real = float>
struct NotificationDscData {
    NmeaTextRecordData<Real> interrogation;
    NmeaTextRecordData<Real> response;
};

template<typename Real = float>
struct NotificationsData {
    NotificationAlertsData<Real> alerts;
    NotificationMessagesData<Real> messages;
    NotificationDscData<Real> dsc;
};

template<typename Real = float>
struct DataModel {
    NavData<Real> nav;
    RouteData<Real> route;
    GnssModelData<Real> gnss;
    AisData<Real> ais;
    InsData<Real> ins;

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
};

} // namespace ship_data_model

#include "data_model_mapping.hpp"
