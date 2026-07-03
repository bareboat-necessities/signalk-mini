#pragma once

#include <stdint.h>

namespace ship_data_model {

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

} // namespace ship_data_model
