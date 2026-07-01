#pragma once

#include "core_data_types.hpp"

namespace ship_data_model {

template<typename Real = float>
struct ShipImuData {
    Setting<uint8_t> source_kind;
    char device_id[32] = {0};
    uint64_t last_update_us = 0;

    Setting<uint8_t> rate_hz;
    Stamped<Real> frequency_hz;

    Setting<Quat<Real>> alignment_q;
    RangeSetting<Real> heading_offset_deg;
    Setting<int32_t> alignment_counter;

    Stamped<Real> uptime_s;
    bool has_warning = false;
    bool has_error = false;

    Stamped<Vec3<Real>> gyro_bias_deg_s;
    Stamped<Vec3<Real>> accel_g;
    Stamped<Vec3<Real>> gyro_deg_s;
    Stamped<Vec3<Real>> compass_raw;
    Stamped<Vec3<Real>> accel_residuals_g;

    Stamped<Real> pitch_deg;
    Stamped<Real> roll_deg;
    Stamped<Real> heel_deg;

    Stamped<Real> pitch_rate_deg_s;
    Stamped<Real> roll_rate_deg_s;
    Stamped<Real> heading_rate_deg_s;
    Stamped<Real> heading_rate_rate_deg_s2;

    Stamped<Real> heading_rate_lowpass_deg_s;
    Stamped<Real> heading_rate_rate_lowpass_deg_s2;

    Stamped<Real> heading_deg;
    Stamped<Real> heading_lowpass_deg;
    Stamped<Quat<Real>> fusion_q_pose;

    RangeSetting<Real> heading_lowpass_constant_0_1;
    RangeSetting<Real> headingrate_lowpass_constant_0_1;
    RangeSetting<Real> headingraterate_lowpass_constant_0_1;
};

template<typename Real = float>
struct ImuStateData {
    Setting<bool> enabled;
    Stamped<bool> present;
    Stamped<bool> healthy;
    char device[64] = {0};
    char driver[64] = {0};
    Stamped<Real> rate_hz;
};

template<typename Real = float>
struct ImuVectorCalibrationData {
    Setting<bool> valid;
    Stamped<Vec3<Real>> bias;
};

template<typename Real = float>
struct ImuCalibrationData {
    Setting<bool> locked;
    Setting<ImuCalibrationState> state;

    ImuVectorCalibrationData<Real> accel;
    ImuVectorCalibrationData<Real> gyro;
    ImuVectorCalibrationData<Real> compass;

    Stamped<Vec3<Real>> accel_bias_g;
    Stamped<Vec3<Real>> gyro_bias_deg_s;
    Stamped<Vec3<Real>> compass_offset;
    Stamped<Real> compass_field_strength;
};

} // namespace ship_data_model
