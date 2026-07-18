#pragma once

#include <stddef.h>
#include <stdint.h>

#include "memory_profile.hpp"

namespace signalk_mini {

struct ServerIdentityConfig {
    const char* server_name = "signalk-mini";
    const char* server_version = "0.1.0";
    const char* signalk_version = "1.8.2";
    const char* self = "vessels.urn:mrn:signalk:uuid:00000000-0000-4000-8000-000000000001";
};

struct SignalKWebSocketServerConfig {
#if defined(ARDUINO)
    bool enabled = false;
#else
    bool enabled = true;
#endif
    const char* host = "0.0.0.0";
    uint16_t port = 3001;
    uint16_t max_connections = DefaultSignalKMaxConnections;
    bool allow_rx = true;
    bool allow_tx = true;
};

struct SignalKProtocolServerConfig {
    const char* host = "0.0.0.0";
    uint16_t port = 20223;
    uint16_t max_connections = DefaultSignalKMaxConnections;
    bool allow_rx = true;
    bool allow_tx = true;
    SignalKWebSocketServerConfig websocket;
};

struct PublisherConfig {
    uint32_t interval_us = 10000;
    uint16_t max_changes_per_tick = DefaultMaxChangesPerTick;
    uint16_t json_buffer_size = static_cast<uint16_t>(DefaultSignalKJsonBufferSize);
    const char* source_label = "signalk-mini";
    bool send_current_values_on_connect = true;
    uint32_t current_value_timeout_ms = 0;
};

enum class ConnectorProtocol : uint8_t {
    None = 0,
    Nmea0183,
    SeaTalk1,
    Ubx,
    Gpsd,
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

struct UbxProtocolConfig {
    bool configure_receiver = false;
};

struct GpsdProtocolConfig {
    const char* device = nullptr;
    bool include_sky = true;
    bool include_gst = true;
};

struct ConnectorProtocolConfig {
    ConnectorProtocol kind = ConnectorProtocol::None;
    Nmea0183ProtocolConfig nmea0183;
    UbxProtocolConfig ubx;
    GpsdProtocolConfig gpsd;
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
    const char* listen_host = "0.0.0.0";
    uint16_t listen_port = 0;
    const char* remote_host = nullptr;
    uint16_t remote_port = 0;
    bool allow_broadcast = true;
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

struct ConnectorReconnectConfig {
    bool enabled = true;
    uint32_t initial_delay_ms = 1000;
    uint32_t maximum_delay_ms = 30000;
};

struct ConnectorConfig {
    bool enabled = false;
    const char* label = nullptr;
    ConnectorAccessConfig access;
    ConnectorProtocolConfig protocol;
    ConnectorTransportConfig transport;
    ConnectorReconnectConfig reconnect;
};

inline ConnectorConfig make_default_nmea0183_udp_listener_connector() {
    ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "nmea0183-udp-10110";
    connector.access.allow_rx = true;
    connector.access.allow_tx = false;
    connector.protocol.kind = ConnectorProtocol::Nmea0183;
    connector.protocol.nmea0183.validate_checksum = false;
    connector.protocol.nmea0183.validate_checksum_configured = true;
    connector.transport.kind = ConnectorTransport::Udp;
    connector.transport.udp.listen_host = "0.0.0.0";
    connector.transport.udp.listen_port = 10110;
    connector.transport.udp.allow_broadcast = true;
    return connector;
}

inline const ConnectorConfig default_connector_configs[] = {
    make_default_nmea0183_udp_listener_connector()
};

struct SignalKMiniConfig {
    ServerIdentityConfig identity;
    SignalKProtocolServerConfig signalk;
    PublisherConfig publisher;
    const ConnectorConfig* connectors = default_connector_configs;
    size_t connector_count = sizeof(default_connector_configs) / sizeof(default_connector_configs[0]);
};

// Standard MCU/sketch-owned input configuration.
// Use this when board-specific sketch code owns UART/I2C/pin polling and feeds
// parsed protocol facades directly, while the core server owns only Signal K TCP
// serving and delta publishing. This avoids enabling core UDP/TCP/serial
// connectors on MCU targets before those transports have board-specific support.
inline SignalKMiniConfig make_sketch_owned_io_config(const char* server_name,
                                                     const char* source_label,
                                                     uint16_t signalk_port = 20223,
                                                     uint32_t publisher_interval_us = 10000,
                                                     uint16_t max_changes_per_tick = DefaultMaxChangesPerTick,
                                                     uint16_t json_buffer_size = static_cast<uint16_t>(DefaultSignalKJsonBufferSize)) {
    SignalKMiniConfig cfg;
    cfg.identity.server_name = server_name ? server_name : "signalk-mini";
    cfg.identity.server_version = "0.1.0";
    cfg.signalk.host = "0.0.0.0";
    cfg.signalk.port = signalk_port;
    cfg.signalk.allow_rx = true;
    cfg.signalk.allow_tx = true;
    cfg.signalk.websocket.host = cfg.signalk.host;
    cfg.signalk.websocket.enabled = false;
    cfg.publisher.interval_us = publisher_interval_us;
    cfg.publisher.max_changes_per_tick = max_changes_per_tick;
    cfg.publisher.json_buffer_size = json_buffer_size;
    cfg.publisher.source_label = source_label ? source_label : cfg.identity.server_name;
    cfg.connectors = nullptr;
    cfg.connector_count = 0;
    return cfg;
}

} // namespace signalk_mini
