#pragma once

#include <stddef.h>
#include <stdint.h>

namespace signalk_mini {

static constexpr size_t max_connector_configs = 4;

struct ServerIdentityConfig {
    const char* server_name = "signalk-mini";
    const char* server_version = "0.1.0";
    const char* self = "vessels.self";
};

struct SignalKProtocolServerConfig {
    const char* host = "0.0.0.0";
    uint16_t port = 20223;
    uint16_t max_clients = 8;
    bool allow_rx = true;
    bool allow_tx = true;
};

struct PublisherConfig {
    uint32_t interval_us = 10000;
    uint16_t max_changes_per_tick = 32;
    uint16_t json_buffer_size = 512;
    const char* source_label = "signalk-mini";
};

enum class ConnectorKind : uint8_t {
    None = 0,
    Nmea0183TcpClient,
    Nmea0183TcpServer,
};

struct ConnectorConfig {
    bool enabled = false;
    ConnectorKind kind = ConnectorKind::None;
    const char* label = nullptr;
    const char* host = "127.0.0.1";
    uint16_t port = 0;
    bool allow_rx = true;
    bool allow_tx = false;
};

struct SignalKMiniConfig {
    ServerIdentityConfig identity;

    // The main Signal K protocol server is mandatory. It is always started.
    SignalKProtocolServerConfig signalk;

    PublisherConfig publisher;

    // Optional connectors are added or removed by configuration.
    ConnectorConfig connectors[max_connector_configs] = {
        {true, ConnectorKind::Nmea0183TcpClient, "nmea0183-tcp-client", "127.0.0.1", 10110, true, false},
        {},
        {},
        {}
    };
    size_t connector_count = 1;
};

} // namespace signalk_mini
