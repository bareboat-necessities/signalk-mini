#pragma once

#include <stddef.h>
#include <stdint.h>

namespace signalk_mini {

struct SignalKTcpConfig {
    bool enabled = true;
    const char* host = "0.0.0.0";
    uint16_t port = 20223;
    uint16_t max_clients = 8;
};

struct Nmea0183TcpConfig {
    bool enabled = true;
    const char* host = "0.0.0.0";
    uint16_t port = 20224;
    uint16_t max_sources = 4;
};

struct PublisherConfig {
    uint32_t interval_us = 10000;
    uint16_t max_changes_per_tick = 32;
    uint16_t json_buffer_size = 512;
    const char* source_label = "signalk-mini";
    const char* source_type = "NMEA0183";
};

struct ServerIdentityConfig {
    const char* server_name = "signalk-mini";
    const char* server_version = "0.1.0";
    const char* signalk_version = "1.0.0";
    const char* self = "vessels.self";
};

struct SignalKMiniConfig {
    ServerIdentityConfig identity;
    SignalKTcpConfig signalk_tcp;
    Nmea0183TcpConfig nmea0183_tcp;
    PublisherConfig publisher;

    const char* host() const { return signalk_tcp.host; }
    uint16_t signalk_port() const { return signalk_tcp.port; }
    uint16_t nmea0183_tcp_port() const { return nmea0183_tcp.port; }
    const char* source_label() const { return publisher.source_label; }
};

} // namespace signalk_mini
