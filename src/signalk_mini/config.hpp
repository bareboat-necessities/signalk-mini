#pragma once

#include <stddef.h>
#include <stdint.h>

namespace signalk_mini {

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

struct ConnectorAccessConfig {
    bool allow_rx = true;
    bool allow_tx = false;
};

struct Nmea0183ProtocolConfig {
    bool validate_checksum = false;
    bool validate_checksum_configured = false;
};

struct ConnectorProtocolConfig {
    ConnectorProtocol kind = ConnectorProtocol::None;
    Nmea0183ProtocolConfig nmea0183;
};

struct TcpClientTransportConfig {
    const char* host = "127.0.0.1";
    uint16_t port = 0;
};

struct TcpServerTransportConfig {
    const char* host = "127.0.0.1";
    uint16_t port = 0;
};

struct SerialTransportConfig {
    const char* device = nullptr;
    uint32_t baud = 4800;
};

struct UdpTransportConfig {
    const char* host = "127.0.0.1";
    uint16_t port = 0;
    uint16_t local_port = 0;
};

struct I2cTransportConfig {
    uint8_t bus = 0;
    uint8_t address = 0;
};

struct PinTransportConfig {
    uint8_t pin = 0;
};

struct ConnectorTransportConfig {
    ConnectorTransport kind = ConnectorTransport::None;
    TcpClientTransportConfig tcp_client;
    TcpServerTransportConfig tcp_server;
    SerialTransportConfig serial;
    UdpTransportConfig udp;
    I2cTransportConfig i2c;
    PinTransportConfig digital_pin;
    PinTransportConfig analog_pin;
};

inline bool default_nmea0183_validate_checksum(ConnectorTransport transport) {
    return transport == ConnectorTransport::Serial;
}

inline bool effective_nmea0183_validate_checksum(const Nmea0183ProtocolConfig& protocol, ConnectorTransport transport) {
    return protocol.validate_checksum_configured ? protocol.validate_checksum : default_nmea0183_validate_checksum(transport);
}

struct ConnectorConfig {
    bool enabled = false;
    const char* label = nullptr;
    ConnectorAccessConfig access;
    ConnectorProtocolConfig protocol;
    ConnectorTransportConfig transport;
};

inline ConnectorConfig make_default_nmea0183_tcp_client_connector() {
    ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "nmea0183-tcp-client";
    connector.access.allow_rx = true;
    connector.access.allow_tx = false;
    connector.protocol.kind = ConnectorProtocol::Nmea0183;
    connector.protocol.nmea0183.validate_checksum = false;
    connector.protocol.nmea0183.validate_checksum_configured = true;
    connector.transport.kind = ConnectorTransport::TcpClient;
    connector.transport.tcp_client.host = "127.0.0.1";
    connector.transport.tcp_client.port = 10110;
    return connector;
}

inline const ConnectorConfig default_connector_configs[] = {
    make_default_nmea0183_tcp_client_connector()
};

struct SignalKMiniConfig {
    ServerIdentityConfig identity;
    SignalKProtocolServerConfig signalk;
    PublisherConfig publisher;
    const ConnectorConfig* connectors = default_connector_configs;
    size_t connector_count = sizeof(default_connector_configs) / sizeof(default_connector_configs[0]);
};

} // namespace signalk_mini
