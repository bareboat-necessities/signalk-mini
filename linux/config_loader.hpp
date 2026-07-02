#pragma once

#include <deque>
#include <string>

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
        const int total = config_setting_length(connector_list);
        size_t out = 0;
        for (int i = 0; i < total && out < signalk_mini::max_connector_configs; ++i) {
            config_setting_t* connector_setting = config_setting_get_elem(connector_list, i);
            if (!connector_setting) continue;
            signalk_mini::ConnectorConfig connector;
            const char* value = nullptr;
            read_bool(connector_setting, "enabled", connector.enabled);
            if (config_setting_lookup_string(connector_setting, "protocol", &value)) connector.protocol = protocol_from_string(value);
            if (config_setting_lookup_string(connector_setting, "transport", &value)) connector.transport = transport_from_string(value);
            if (config_setting_lookup_string(connector_setting, "label", &value)) connector.label = keep(value);
            if (config_setting_lookup_string(connector_setting, "host", &value)) connector.host = keep(value);
            read_u16(connector_setting, "port", connector.port);
            if (config_setting_lookup_string(connector_setting, "device", &value)) connector.device = keep(value);
            read_u32(connector_setting, "baud", connector.baud);
            read_u8(connector_setting, "pin", connector.pin);
            read_u8(connector_setting, "i2c_bus", connector.i2c_bus);
            read_u8(connector_setting, "i2c_address", connector.i2c_address);
            read_bool(connector_setting, "allow_rx", connector.allow_rx);
            read_bool(connector_setting, "allow_tx", connector.allow_tx);
            config.connectors[out++] = connector;
        }
        config.connector_count = out;
    }

    std::deque<std::string> strings_;
};

} // namespace signalk_mini_linux
