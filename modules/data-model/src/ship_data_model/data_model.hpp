#pragma once

#include "autopilot/autopilot_model_data.hpp"
#include "ins/ins_data.hpp"
#include "nav/nav_data.hpp"
#include "route/route_data.hpp"
#include "gnss/gnss_data.hpp"
#include "ais/ais_data.hpp"
#include "wind_sea/wind_sea_data.hpp"
#include "env/env_data.hpp"
#include "propulsion/propulsion_data.hpp"
#include "steering/steering_model_data.hpp"
#include "fluids/fluids_data.hpp"
#include "batteries/batteries_data.hpp"
#include "electrical/electrical_data.hpp"
#include "comm/comm_data.hpp"
#include "notifications/notifications_data.hpp"

namespace ship_data_model {

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
