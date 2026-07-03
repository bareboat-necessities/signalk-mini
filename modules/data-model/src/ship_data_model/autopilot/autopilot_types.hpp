#pragma once

#include <stdint.h>

namespace ship_data_model {

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

enum class PilotCommandSource : uint32_t {
    internal,
    runtime,
    remote
};

} // namespace ship_data_model
