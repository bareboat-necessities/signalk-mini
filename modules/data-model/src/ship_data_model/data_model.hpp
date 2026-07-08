#pragma once

#include "autopilot/autopilot_model_data.hpp"
#include "ins/ins_data.hpp"
#include "nav/nav_data.hpp"
#include "route/route_data.hpp"
#include "gnss/gnss_data.hpp"
#include "ais/ais_data.hpp"
#include "wind_sea/wind_sea_data.hpp"
#include "env/env_data.hpp"
#include "fishing/trawl_data.hpp"
#include "propulsion/propulsion_data.hpp"
#include "steering/steering_model_data.hpp"
#include "fluids/fluids_data.hpp"
#include "batteries/batteries_data.hpp"
#include "electrical/electrical_data.hpp"
#include "comm/comm_data.hpp"
#include "notifications/notifications_data.hpp"
#include "appliances/appliances_data.hpp"
#include "audio_video/audio_video_data.hpp"
#include "anchors/anchors_data.hpp"
#include "dinghy_lifeboat/dinghy_lifeboat_data.hpp"
#include "charts/charts_data.hpp"
#include "lights/lights_data.hpp"
#include "sails/sails_data.hpp"
#include "bilges/bilges_data.hpp"

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
    TrawlData<Real> trawl;

    PropulsionData<Real> propulsion;
    SteeringModelData<Real> steering;
    AutopilotModelData<Real> autopilot;

    FluidsData<Real> fluids;
    BatteriesData<Real> batteries;
    ElectricalData<Real> electrical;
    CommData<Real> comm;
    NotificationsData<Real> notifications;

    AppliancesData<Real> appliances;
    AudioVideoData<Real> audio_video;
    AnchorsData<Real> anchors;
    DinghyLifeboatData<Real> dinghy_lifeboat;
    ChartsData<Real> charts;
    LightsData<Real> lights;
    SailsData<Real> sails;
    BilgesData<Real> bilges;
};

} // namespace ship_data_model

#include "data_model_mapping.hpp"
