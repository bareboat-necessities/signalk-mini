#pragma once

#include <stdint.h>

namespace signalk_mini {

struct ServerIdentityConfig {
    const char* server_name = "signalk-mini";
    const char* server_version = "0.1.0";
    const char* self = "vessels.self";
};

struct SignalKMiniConfig {
    const char* host = "0.0.0.0";
    uint16_t signalk_port = 20223;
    uint16_t nmea0183_tcp_port = 20224;
    const char* source_label = "signalk-mini";
    ServerIdentityConfig identity;
    bool enable_signalk_tcp = true;
    bool enable_nmea0183_tcp = true;
    uint32_t publish_interval_us = 10000;
    uint16_t max_changes_per_tick = 32;
    uint16_t json_buffer_size = 512;
};

} // namespace signalk_mini
