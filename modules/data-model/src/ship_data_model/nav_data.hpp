#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct NavigationData;

template<typename Real = float>
struct GpsData {
    Setting<SensorSource> source;
    RangeSetting<Real> rate_hz;

    Stamped<Real> timestamp_s;
    Stamped<Real> track_deg;
    Stamped<Real> speed_kn;

    Stamped<Real> fix_lat_deg;
    Stamped<Real> fix_lon_deg;
    Stamped<Real> fix_alt_m;
    bool has_fix_json = false;

    Stamped<Real> leeway_ground_deg;
    Stamped<Real> compass_error_deg;
    Stamped<Real> declination_deg;
    Setting<int32_t> alignment_counter;

    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct ApbData {
    Setting<SensorSource> source;
    Stamped<Real> track_deg;
    Stamped<Real> xte_nmi;
    RangeSetting<Real> xte_gain_deg_per_nmi;
    Setting<AutopilotMode> mode_hint;
    char sender_id[3] = {0, 0, 0};
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct RmbData {
    Setting<SensorSource> source;
    char origin_id[16] = {0};
    char destination_id[16] = {0};
    Stamped<Real> xte_nmi;
    Stamped<Real> destination_lat_deg;
    Stamped<Real> destination_lon_deg;
    Stamped<Real> range_nmi;
    Stamped<Real> bearing_deg;
    Stamped<Real> closing_velocity_kn;
    Setting<bool> arrived;
    uint64_t last_update_us = 0;
};

template<typename Real>
struct NavigationData {
    GpsData<Real> gps;
    ApbData<Real> apb;
    RmbData<Real> rmb;
};

} // namespace ship_data_model
