#pragma once

#include <stdint.h>

namespace signalk_mini {

struct SignalKMiniConfig {
    const char* host = "0.0.0.0";
    uint16_t signalk_port = 20223;
    uint16_t nmea0183_tcp_port = 20224;
    const char* source_label = "signalk-mini";
};

} // namespace signalk_mini
