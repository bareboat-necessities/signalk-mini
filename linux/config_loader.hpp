#pragma once

#include <deque>
#include <string>
#include <vector>

#include <libconfig.h>
#include <signalk_mini/config.hpp>

namespace signalk_mini_linux {

class ConfigLoader {
public:
    signalk_mini::SignalKMiniConfig config;

    bool load_file(const char* path, std::string& error) {
        config_t file;
        config_init(&file);
        if (!config_read_file(&file, path)) {
            const int line = config_error_line(&file);
            const char* text = config_error_text(&file);
            error = line > 0
                ? std::string("config parse error at ") + path + ":" + std::to_string(line) + ": " + (text ? text : "unknown error")
                : std::string("cannot read config file: ") + path;
            config_destroy(&file);
            return false;
        }

        load_identity(&file);
        load_signalk(&file);
        load_publisher(&file);
        load_connectors(&file);

        config_destroy(&file);
        return true;
    }

private:
    const char* keep(const char* value) {
        if (!value) return nullptr;
        strings_.push_back(value);
        return strings_.back().c_str();
    }

    static bool read_bool(config_setting_t* setting, const char* name, bool& value) {
        int tmp = 0;
        if (!setting || !config_setting_lookup_bool(setting, name, &tmp)) return false;
        value = tmp != 0;
        return true;
    }

    static bool read_u16(config_setting_t* setting, const char* name, uint16_t& value) {
        int tmp = 0;
        if (!setting || !config_setting_lookup_int(setting, name, &tmp)) return false;
        if (tmp < 0) tmp = 0;
        if (tmp > 65535) tmp = 65535;
        value = static_cast<uint16_t>(tmp);
        return true;
    }

    static bool read_u32(config_setting_t* setting, const char* name, uint32_t& value) {
        int tmp = 0;
        if (!setting || !config_setting_lookup_int(setting, name, &tmp)) return false;
        if (tmp < 0) tmp = 0;
        value = static_cast<uint32_t>(tmp);
        return true;
    }

    static bool read_u8(config_setting_t* setting, const char* name, uint8_t& value) {
        int tmp = 0;
        if (!setting || !config_setting_lookup_int(setting, name, &tmp)) return false;
        if (tmp < 0) tmp = 0;
        if (tmp > 255) tmp = 255;
        value = static_cast<uint8_t>(tmp);
        return true;
    }

    static signalk_mini::ConnectorProtocol protocol_from_string(const char* value) {
        const std::string text = value ? value : "";
        if (text == "nmea0183") return signalk_mini::ConnectorProtocol::Nmea0183;
        if (text == "seatalk" || text == "seatalk1" || text == "seatalk_1") return signalk_mini::ConnectorProtocol::SeaTalk1;
        if (text == "nmea2000") return signalk_mini::ConnectorProtocol::Nmea2000;
        if (text == "signalk") return signalk_mini::ConnectorProtocol::SignalK;
        if (text == "generic_sensor") return signalk_mini::ConnectorProtocol::GenericSensor;
        return signalk_mini::ConnectorProtocol::None;
    }

    static signalk_mini::ConnectorTransport transport_from_string(const char* value) {
        const std::string text = value ? value : "";
        if (text == "serial") return signalk_mini::ConnectorTransport::Serial;
        if (text == "tcp_client") return signalk_mini::ConnectorTransport::TcpClient;
        if (text == "tcp_server") return signalk_mini::ConnectorTransport::TcpServer;
        if (text == "udp") return signalk_mini::ConnectorTransport::Udp;
        if (text == "i2c") return signalk_mini::ConnectorTransport::I2c;
        if (text == "digital_pin") return signalk_mini::ConnectorTransport::DigitalPin;
        if (text == "analog_pin") return signalk_mini::ConnectorTransport::AnalogPin;
        return signalk_mini::ConnectorTransport::None;
    }

    void load_identity(config_t* file) {
        const char* value = nullptr;
        if (config_lookup_string(file, "identity.server_name", &value)) config.identity.server_name = keep(value);
        if (config_lookup_string(file, "identity.server_version", &value)) config.identity.server_version = keep(value);
        if (config_lookup_string(file, "identity.self", &value)) config.identity.self = keep(value);
    }

    void load_signalk(config_t* file) {
        config_setting_t* s = config_lookup(file, "signalk");
        if (!s) return;
        const char* value = nullptr;
        if (config_setting_lookup_string(s, "host", &value)) config.signalk.host = keep(value);
        read_u16(s, "port", config.signalk.port);
        read_u16(s, "max_connections", config.signalk.max_connections);
        read_bool(s, "allow_rx", config.signalk.allow_rx);
        read_bool(s, "allow_tx", config.signalk.allow_tx);
        load_signalk_websocket(s);
    }

    void load_signalk_websocket(config_setting_t* signalk) {
        config_setting_t* s = config_setting_lookup(signalk, "websocket");
        if (!s) return;
        const char* value = nullptr;
        read_bool(s, "enabled", config.signalk.websocket.enabled);
        if (config_setting_lookup_string(s, "host", &value)) config.signalk.websocket.host = keep(value);
        read_u16(s, "port", config.signalk.websocket.port);
        read_u16(s, "fallback_port", config.signalk.websocket.fallback_port);
        read_u16(s, "max_connections", config.signalk.websocket.max_connections);
        read_bool(s, "allow_rx", config.signalk.websocket.allow_rx);
        read_bool(s, "allow_tx", config.signalk.websocket.allow_tx);
    }

    void load_publisher(config_t* file) {
        config_setting_t* s = config_lookup(file, "publisher");
        if (!s) return;
        const char* value = nullptr;
        read_u32(s, "interval_us", config.publisher.interval_us);
        read_u16(s, "max_changes_per_tick", config.publisher.max_changes_per_tick);
        read_u16(s, "json_buffer_size", config.publisher.json_buffer_size);
        if (config_setting_lookup_string(s, "source_label", &value)) config.publisher.source_label = keep(value);
    }

    void load_connectors(config_t* file) {
        config_setting_t* connector_list = config_lookup(file, "connectors");
        if (!connector_list) return;
        connectors_.clear();
        const int total = config_setting_length(connector_list);
        connectors_.reserve(total > 0 ? static_cast<size_t>(total) : 0);
        for (int i = 0; i < total; ++i) {
            config_setting_t* connector_setting = config_setting_get_elem(connector_list, i);
            if (!connector_setting) continue;
            signalk_mini::ConnectorConfig connector;
            load_connector(connector_setting, connector);
            connectors_.push_back(connector);
        }
        config.connectors = connectors_.empty() ? nullptr : connectors_.data();
        config.connector_count = connectors_.size();
    }

    void load_connector(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        const char* value = nullptr;
        read_bool(connector_setting, "enabled", connector.enabled);
        if (config_setting_lookup_string(connector_setting, "label", &value)) connector.label = keep(value);
        if (config_setting_lookup_string(connector_setting, "protocol", &value)) connector.protocol.kind = protocol_from_string(value);
        if (config_setting_lookup_string(connector_setting, "transport", &value)) connector.transport.kind = transport_from_string(value);
        load_connector_access(connector_setting, connector);
        load_nmea0183_protocol(connector_setting, connector);
        load_transport(connector_setting, connector);
    }

    void load_connector_access(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        config_setting_t* access = config_setting_lookup(connector_setting, "access");
        if (access) {
            read_bool(access, "allow_rx", connector.access.allow_rx);
            read_bool(access, "allow_tx", connector.access.allow_tx);
            return;
        }
        read_bool(connector_setting, "allow_rx", connector.access.allow_rx);
        read_bool(connector_setting, "allow_tx", connector.access.allow_tx);
    }

    void load_nmea0183_protocol(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        if (connector.protocol.kind != signalk_mini::ConnectorProtocol::Nmea0183) return;
        config_setting_t* nmea0183 = config_setting_lookup(connector_setting, "nmea0183");
        if (nmea0183) {
            if (read_bool(nmea0183, "validate_checksum", connector.protocol.nmea0183.validate_checksum)) connector.protocol.nmea0183.validate_checksum_configured = true;
        } else if (read_bool(connector_setting, "validate_checksum", connector.protocol.nmea0183.validate_checksum)) {
            connector.protocol.nmea0183.validate_checksum_configured = true;
        }
    }

    void load_transport(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        switch (connector.transport.kind) {
        case signalk_mini::ConnectorTransport::TcpClient:
            load_tcp_client(connector_setting, connector);
            break;
        case signalk_mini::ConnectorTransport::TcpServer:
            load_tcp_server(connector_setting, connector);
            break;
        case signalk_mini::ConnectorTransport::Serial:
            load_serial(connector_setting, connector);
            break;
        case signalk_mini::ConnectorTransport::Udp:
            load_udp(connector_setting, connector);
            break;
        case signalk_mini::ConnectorTransport::I2c:
            load_i2c(connector_setting, connector);
            break;
        case signalk_mini::ConnectorTransport::DigitalPin:
            load_pin(connector_setting, "digital_pin", connector.transport.digital_pin);
            break;
        case signalk_mini::ConnectorTransport::AnalogPin:
            load_pin(connector_setting, "analog_pin", connector.transport.analog_pin);
            break;
        default:
            break;
        }
    }

    void load_tcp_client(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        const char* value = nullptr;
        config_setting_t* s = config_setting_lookup(connector_setting, "tcp_client");
        if (!s) s = connector_setting;
        if (config_setting_lookup_string(s, "host", &value)) connector.transport.tcp_client.host = keep(value);
        read_u16(s, "port", connector.transport.tcp_client.port);
    }

    void load_tcp_server(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        const char* value = nullptr;
        config_setting_t* s = config_setting_lookup(connector_setting, "tcp_server");
        if (!s) s = connector_setting;
        if (config_setting_lookup_string(s, "host", &value)) connector.transport.tcp_server.host = keep(value);
        read_u16(s, "port", connector.transport.tcp_server.port);
    }

    void load_serial(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        const char* value = nullptr;
        config_setting_t* s = config_setting_lookup(connector_setting, "serial");
        if (!s) s = connector_setting;
        if (config_setting_lookup_string(s, "device", &value)) connector.transport.serial.device = keep(value);
        read_u32(s, "baud", connector.transport.serial.baud);
    }

    void load_udp(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        const char* value = nullptr;
        config_setting_t* s = config_setting_lookup(connector_setting, "udp");
        if (!s) s = connector_setting;
        if (config_setting_lookup_string(s, "listen_host", &value)) connector.transport.udp.listen_host = keep(value);
        else if (config_setting_lookup_string(s, "host", &value)) connector.transport.udp.listen_host = keep(value);
        if (!read_u16(s, "listen_port", connector.transport.udp.listen_port)) {
            read_u16(s, "local_port", connector.transport.udp.listen_port);
        }
        if (config_setting_lookup_string(s, "remote_host", &value)) connector.transport.udp.remote_host = keep(value);
        read_u16(s, "remote_port", connector.transport.udp.remote_port);
        read_bool(s, "allow_broadcast", connector.transport.udp.allow_broadcast);
    }

    void load_i2c(config_setting_t* connector_setting, signalk_mini::ConnectorConfig& connector) {
        config_setting_t* s = config_setting_lookup(connector_setting, "i2c");
        if (!s) s = connector_setting;
        read_u8(s, "bus", connector.transport.i2c.bus);
        read_u8(s, "address", connector.transport.i2c.address);
    }

    static void load_pin(config_setting_t* connector_setting, const char* name, signalk_mini::PinTransportConfig& pin) {
        config_setting_t* s = config_setting_lookup(connector_setting, name);
        if (!s) s = connector_setting;
        read_u8(s, "pin", pin.pin);
    }

    std::deque<std::string> strings_;
    std::vector<signalk_mini::ConnectorConfig> connectors_;
};

} // namespace signalk_mini_linux
