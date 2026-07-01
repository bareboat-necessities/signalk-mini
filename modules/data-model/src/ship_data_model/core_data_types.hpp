#pragma once

#include <stdint.h>
#include <stddef.h>

namespace ship_data_model {

enum class SensorSource : uint8_t {
    gpsd,
    servo,
    serial,
    tcp,
    signalk,
    water_wind,
    gps_wind,
    none
};

inline int source_priority(SensorSource source) {
    switch (source) {
    case SensorSource::gpsd:       return 1;
    case SensorSource::servo:      return 1;
    case SensorSource::serial:     return 2;
    case SensorSource::tcp:        return 3;
    case SensorSource::signalk:    return 4;
    case SensorSource::water_wind: return 5;
    case SensorSource::gps_wind:   return 6;
    case SensorSource::none:       return 7;
    }
    return 7;
}

enum class AutopilotMode : uint8_t {
    compass,
    gps,
    nav,
    wind,
    true_wind
};

enum class PilotName : uint8_t {
    basic,
    absolute,
    wind,
    gps,
    rate,
    simple,
    vmg,
    deadzone,
    autotune,
    fuzzy,
    learning,
    intellect
};

enum ModeMask : uint32_t {
    mode_mask_compass   = 1u << 0,
    mode_mask_gps       = 1u << 1,
    mode_mask_nav       = 1u << 2,
    mode_mask_wind      = 1u << 3,
    mode_mask_true_wind = 1u << 4
};

enum class SystemHealth : uint8_t {
    unknown,
    ok,
    degraded,
    fault
};

enum SystemFault : uint32_t {
    system_fault_none  = 0u,
    system_fault_imu   = 1u << 0,
    system_fault_servo = 1u << 1,
    system_fault_gps   = 1u << 2,
    system_fault_wind  = 1u << 3
};

inline bool system_faulted(uint32_t faults) {
    return faults != system_fault_none;
}

enum class ImuCalibrationState : uint8_t {
    unknown,
    invalid,
    learning,
    valid
};

enum class TackState : int32_t {
    idle,
    waiting,
    tacking,
    complete
};

enum class TackDirection : int32_t {
    port = -1,
    none = 0,
    starboard = 1
};

enum class PilotCommandSource : uint32_t {
    internal,
    runtime,
    remote
};

enum class RudderCalibrationState : uint8_t {
    idle,
    reset,
    centered,
    starboard_range,
    port_range,
    auto_gain
};

enum ServoFlag : uint32_t {
    servo_sync_flag                   = 1u,
    servo_overtemp_fault              = 2u,
    servo_overcurrent_fault           = 4u,
    servo_engaged_flag                = 8u,
    servo_invalid_packet_flag         = 16u,
    servo_port_pin_fault              = 32u,
    servo_starboard_pin_fault         = 64u,
    servo_bad_voltage_fault           = 128u,
    servo_min_rudder_fault            = 256u,
    servo_max_rudder_fault            = 512u,
    servo_current_range_flag          = 1024u,
    servo_bad_fuses_flag              = 2048u,
    servo_rebooted_flag               = 32768u,
    servo_port_overcurrent_fault      = 65536u,
    servo_starboard_overcurrent_fault = 131072u,
    servo_driver_timeout              = 262144u,
    servo_saturated                   = 524288u
};

template<typename Real = float>
struct Vec3 {
    Real x = Real(0);
    Real y = Real(0);
    Real z = Real(0);
};

template<typename Real = float>
struct Quat {
    Real w = Real(1);
    Real x = Real(0);
    Real y = Real(0);
    Real z = Real(0);
};

template<typename T>
struct Stamped {
    T value{};
    bool valid = false;
    uint64_t last_update_us = 0;

    void set(const T& v, uint64_t now_us) {
        value = v;
        valid = true;
        last_update_us = now_us;
    }

    bool stale(uint64_t now_us, uint64_t timeout_us) const {
        return !valid || now_us - last_update_us > timeout_us;
    }
};

template<typename T>
struct Setting {
    T value{};
};

template<typename T>
struct RangeSetting {
    T value{};
    T min{};
    T max{};
};

template<typename T>
struct TimedCommand {
    T value{};
    bool valid = false;
    uint64_t last_update_us = 0;
    uint64_t last_external_set_us = 0;
    bool use_period = true;

    void set_external(const T& v, uint64_t now_us) {
        value = v;
        valid = true;
        last_update_us = now_us;
        last_external_set_us = now_us;
        use_period = false;
    }

    void set_internal_command(const T& v, uint64_t now_us) {
        value = v;
        valid = true;
        last_update_us = now_us;
        use_period = true;
    }
};

template<typename Real = float>
struct ValuePublicationState {
    Stamped<uint32_t> published_value_count;
    Stamped<uint32_t> published_byte_count;
};

} // namespace ship_data_model
