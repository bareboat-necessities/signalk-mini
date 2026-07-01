#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct AutopilotServerData {
    char version[32] = {0};
    char hostname[64] = {0};
    char config_dir[128] = {0};
    char profile_name[64] = {0};
    Setting<bool> enabled;
    Setting<bool> persistent;
    Setting<bool> watchdog_enabled;
    Stamped<Real> timestamp_s;
    Stamped<Real> uptime_s;
    Stamped<Real> monotonic_s;
    Stamped<Real> poll_period_s;
    Stamped<uint32_t> client_count;
    Stamped<uint32_t> value_count;
    Stamped<uint32_t> watch_count;
};

template<typename Real = float>
struct AutopilotStatusData {
    Stamped<SystemHealth> health;
    Stamped<uint32_t> faults;
    Stamped<uint32_t> warnings;
    Stamped<Real> uptime_s;
    Stamped<Real> loop_period_s;
    Stamped<Real> cpu_load_0_1;
    Stamped<uint32_t> free_memory_bytes;
    char last_error[96] = {0};
    char last_warning[96] = {0};
};

template<typename Real = float>
struct AutopilotTimings {
    Real server_s = Real(0);
    Real sensors_s = Real(0);
    Real imu_s = Real(0);
    Real autopilot_s = Real(0);
    Real servo_s = Real(0);
    Real total_s = Real(0);
};

template<typename Real = float>
struct AutopilotData {
    bool has_version = false;
    Stamped<Real> timestamp_s;

    Setting<AutopilotMode> preferred_mode;
    Setting<AutopilotMode> mode;
    uint32_t available_modes_mask = mode_mask_compass;
    Setting<bool> gps_and_nav_modes;

    Stamped<Real> heading_command_deg;
    Setting<bool> enabled;
    Stamped<Real> heading_deg;
    Stamped<Real> heading_error_deg;
    Stamped<Real> heading_error_int_deg;
    Stamped<Real> heading_command_rate_deg_s;

    Setting<PilotName> pilot;

    Stamped<Real> gps_compass_offset_deg;
    Stamped<Real> wind_compass_offset_deg;
    Stamped<Real> true_wind_compass_offset_deg;
    RangeSetting<Real> wind_offset_filter_0_1;

    Stamped<Real> runtime_s;
    Stamped<AutopilotTimings<Real>> timings;
};

} // namespace ship_data_model
