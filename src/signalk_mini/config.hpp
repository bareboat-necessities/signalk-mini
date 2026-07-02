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
    uint16_t max_connections = 8;
    bool allow_rx = true;
    bool allow_tx = true;
};

struct PublisherConfig {
    uint32_t interval_us = 10000;
    uint16_t max_changes_per_tick = 32;
    uint16_t json_buffer_size = 512;
    const char* source_label = "signalk-mini";
};

enum class ConnectorProtocol : uint8_t {
    None = 0,
    Nmea0183,
    Nmea2000,
    SignalK,
    GenericSensor,
};

enum class ConnectorTransport : uint8_t {
    None = 0,
    Serial,
    TcpClient,
    TcpServer,
    Udp,
    I2c,
    DigitalPin,
    AnalogPin,
};

struct ConnectorConfig {
    // ConnectorConfig is static configuration. It may create zero, one, or many runtime connections.
    bool enabled = false;
    ConnectorProtocol protocol = ConnectorProtocol::None;
    ConnectorTransport transport = ConnectorTransport::None;
    const char* label = nullptr;

    const char* host = "127.0.0.1";
    uint16_t port = 0;

    const char* device = nullptr;
    uint32_t baud = 4800;

    uint8_t pin = 0;
    uint8_t i2c_bus = 0;
    uint8_t i2c_address = 0;

    bool allow_rx = true;
    bool allow_tx = false;
};

struct SignalKMiniConfig {
    ServerIdentityConfig identity;

    // The main Signal K protocol server is mandatory. It is always started.
    SignalKProtocolServerConfig signalk;

    PublisherConfig publisher;

    // Connectors are configuration entries. Runtime TCP peers are connections.
    ConnectorConfig connectors[max_connector_configs] = {
        {true, ConnectorProtocol::Nmea0183, ConnectorTransport::TcpClient, "nmea0183-tcp-client", "127.0.0.1", 10110, nullptr, 4800, 0, 0, 0, true, false},
        {},
        {},
        {}
    };
    size_t connector_count = 1;
};

} // namespace signalk_mini
