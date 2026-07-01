#pragma once

#include <string.h>
#include "core_data_types.hpp"

namespace ship_data_model {

inline bool rudder_calibration_state_from_name(const char* name, RudderCalibrationState& out) {
    if (!name) return false;
    if (strcmp(name, "idle") == 0) out = RudderCalibrationState::idle;
    else if (strcmp(name, "reset") == 0) out = RudderCalibrationState::reset;
    else if (strcmp(name, "centered") == 0) out = RudderCalibrationState::centered;
    else if (strcmp(name, "starboard range") == 0) out = RudderCalibrationState::starboard_range;
    else if (strcmp(name, "port range") == 0) out = RudderCalibrationState::port_range;
    else if (strcmp(name, "auto gain") == 0) out = RudderCalibrationState::auto_gain;
    else return false;
    return true;
}

inline bool servo_faulted(uint32_t flags) {
    const uint32_t fault_mask = servo_overtemp_fault |
                                servo_overcurrent_fault |
                                servo_port_pin_fault |
                                servo_starboard_pin_fault |
                                servo_bad_voltage_fault |
                                servo_min_rudder_fault |
                                servo_max_rudder_fault |
                                servo_port_overcurrent_fault |
                                servo_starboard_overcurrent_fault |
                                servo_driver_timeout;
    return (flags & fault_mask) != 0;
}

template<typename Real = float>
struct RudderData {
    Setting<SensorSource> source;
    Stamped<Real> angle_deg;
    Setting<RudderCalibrationState> calibration_state;
    RangeSetting<Real> range_deg;
    RangeSetting<Real> offset_deg;
    uint64_t last_update_us = 0;
};

template<typename Real = float>
struct ServoData {
    bool has_controller = false;
    Setting<bool> engaged;
    Setting<uint32_t> flags;
    TimedCommand<Real> command_norm;
    TimedCommand<Real> position_command_deg;
    Stamped<Real> current_a;
    Stamped<Real> voltage_v;
    Stamped<Real> controller_temp_c;
    Stamped<Real> motor_temp_c;
    Stamped<Real> rudder_position_deg;
    RangeSetting<Real> max_current_a;
    RangeSetting<Real> max_controller_temp_c;
    RangeSetting<Real> max_motor_temp_c;
};

enum class ServoControllerState : uint8_t {
    disconnected,
    idle,
    engaged,
    fault
};

template<typename Real = float>
struct ServoTelemetryData {
    Setting<ServoControllerState> controller_state;
    Stamped<uint32_t> flags;
    Stamped<Real> command_norm;
    Stamped<Real> raw_command_norm;
    Stamped<Real> position_deg;
    Stamped<Real> rudder_feedback_deg;
    Stamped<Real> voltage_v;
    Stamped<Real> current_a;
    Stamped<Real> controller_temp_c;
    Stamped<Real> motor_temp_c;
    Stamped<uint32_t> packet_count;
    Stamped<uint32_t> error_count;
};

template<typename Real = float>
struct ServoCalibrationData {
    char source[64] = {0};
    Setting<bool> valid;
    Setting<RudderCalibrationState> state;
    RangeSetting<Real> min_deg;
    RangeSetting<Real> max_deg;
    RangeSetting<Real> center_deg;
    RangeSetting<Real> rudder_min_deg;
    RangeSetting<Real> rudder_max_deg;
    RangeSetting<Real> rudder_center_deg;
};

template<typename Real = float>
struct TackData {
    Setting<bool> enabled;
    Setting<TackState> state;
    Setting<TackDirection> direction;
    RangeSetting<Real> timeout_s;
    RangeSetting<Real> delay_s;
    RangeSetting<Real> angle_deg;
    RangeSetting<Real> rate_deg_s;
    Stamped<Real> progress_0_1;
    Stamped<Real> command_heading_deg;
    Stamped<Real> start_heading_deg;
    Stamped<Real> target_heading_deg;
};

} // namespace ship_data_model
