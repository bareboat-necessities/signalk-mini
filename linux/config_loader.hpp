#pragma once

#include <deque>
#include <exception>
#include <string>

#include <libconfig.h++>
#include <signalk_mini/config.hpp>

namespace signalk_mini_linux {

class ConfigLoader {
public:
    signalk_mini::SignalKMiniConfig config;

    bool load_file(const char* path, std::string& error) {
        try {
            libconfig::Config file;
            file.readFile(path);
            load_identity(file);
            load_signalk(file);
            load_publisher(file);
            load_connectors(file);
            return true;
        } catch (const libconfig::FileIOException&) {
            error = std::string("cannot read config file: ") + path;
            return false;
        } catch (const libconfig::ParseException& e) {
            error = std::string("config parse error at ") + e.getFile() + ":" + std::to_string(e.getLine()) + ": " + e.getError();
            return false;
        } catch (const std::exception& e) {
            error = e.what();
            return false;
        }
    }

private:
    const char* keep(const std::string& value) {
        strings_.push_back(value);
        return strings_.back().c_str();
    }

    static bool read_string(const libconfig::Config& file, const char* path, std::string& value) {
        return file.lookupValue(path, value);
    }

    static bool read_bool(const libconfig::Setting& setting, const char* name, bool& value) {
        return setting.exists(name) && setting.lookupValue(name, value);
    }

    static bool read_u16(const libconfig::Setting& setting, const char* name, uint16_t& value) {
        int tmp = 0;
        if (!setting.exists(name) || !setting.lookupValue(name, tmp)) return false;
        if (tmp < 0) tmp = 0;
        if (tmp > 65535) tmp = 65535;
        value = static_cast<uint16_t>(tmp);
        return true;
    }

    static bool read_u32(const libconfig::Setting& setting, const char* name, uint32_t& value) {
        int tmp = 0;
        if (!setting.exists(name) || !setting.lookupValue(name, tmp)) return false;
        if (tmp < 0) tmp = 0;
        value = static_cast<uint32_t>(tmp);
        return true;
    }

    static signalk_mini::ConnectorProtocol protocol_from_string(const std::string& value) {
        if (value == "nmea0183") return signalk_mini::ConnectorProtocol::Nmea0183;
        if (value == "nmea2000") return signalk_mini::ConnectorProtocol::Nmea2000;
        if (value == "signalk") return signalk_mini::ConnectorProtocol::SignalK;
        if (value == "generic_sensor") return signalk_mini::ConnectorProtocol::GenericSensor;
        return signalk_mini::ConnectorProtocol::None;
    }

    static signalk_mini::ConnectorTransport transport_from_string(const std::string& value) {
        if (value == "serial") return signalk_mini::ConnectorTransport::Serial;
        if (value == "tcp_client") return signalk_mini::ConnectorTransport::TcpClient;
        if (value == "tcp_server") return signalk_mini::ConnectorTransport::TcpServer;
        if (value == "udp") return signalk_mini::ConnectorTransport::Udp;
        if (value == "i2c") return signalk_mini::ConnectorTransport::I2c;
        if (value == "digital_pin") return signalk_mini::ConnectorTransport::DigitalPin;
        if (value == "analog_pin") return signalk_mini::ConnectorTransport::AnalogPin;
        return signalk_mini::ConnectorTransport::None;
    }

    void load_identity(const libconfig::Config& file) {
        std::string value;
        if (read_string(file, "identity.server_name", value)) config.identity.server_name = keep(value);
        if (read_string(file, "identity.server_version", value)) config.identity.server_version = keep(value);
        if (read_string(file, "identity.self", value)) config.identity.self = keep(value);
    }

    void load_signalk(const libconfig::Config& file) {
        if (!file.exists("signalk")) return;
        const libconfig::Setting& s = file.lookup("signalk");
        std::string value;
        if (s.exists("host") && s.lookupValue("host", value)) config.signalk.host = keep(value);
        read_u16(s, "port", config.signalk.port);
        read_u16(s, "max_clients", config.signalk.max_clients);
        read_bool(s, "allow_rx", config.signalk.allow_rx);
        read_bool(s, "allow_tx", config.signalk.allow_tx);
    }

    void load_publisher(const libconfig::Config& file) {
        if (!file.exists("publisher")) return;
        const libconfig::Setting& s = file.lookup("publisher");
        std::string value;
        read_u32(s, "interval_us", config.publisher.interval_us);
        read_u16(s, "max_changes_per_tick", config.publisher.max_changes_per_tick);
        read_u16(s, "json_buffer_size", config.publisher.json_buffer_size);
        if (s.exists("source_label") && s.lookupValue("source_label", value)) config.publisher.source_label = keep(value);
    }

    void load_connectors(const libconfig::Config& file) {
        if (!file.exists("connectors")) return;
        const libconfig::Setting& list = file.lookup("connectors");
        const int total = list.getLength();
        size_t out = 0;
        for (int i = 0; i < total && out < signalk_mini::max_connector_configs; ++i) {
            const libconfig::Setting& s = list[i];
            signalk_mini::ConnectorConfig connector;
            std::string value;
            if (s.exists("enabled")) s.lookupValue("enabled", connector.enabled);
            if (s.exists("protocol") && s.lookupValue("protocol", value)) connector.protocol = protocol_from_string(value);
            if (s.exists("transport") && s.lookupValue("transport", value)) connector.transport = transport_from_string(value);
            if (s.exists("label") && s.lookupValue("label", value)) connector.label = keep(value);
            if (s.exists("host") && s.lookupValue("host", value)) connector.host = keep(value);
            read_u16(s, "port", connector.port);
            if (s.exists("device") && s.lookupValue("device", value)) connector.device = keep(value);
            read_u32(s, "baud", connector.baud);
            read_u8(s, "pin", connector.pin);
            read_u8(s, "i2c_bus", connector.i2c_bus);
            read_u8(s, "i2c_address", connector.i2c_address);
            read_bool(s, "allow_rx", connector.allow_rx);
            read_bool(s, "allow_tx", connector.allow_tx);
            config.connectors[out++] = connector;
        }
        config.connector_count = out;
    }

    static bool read_u8(const libconfig::Setting& setting, const char* name, uint8_t& value) {
        int tmp = 0;
        if (!setting.exists(name) || !setting.lookupValue(name, tmp)) return false;
        if (tmp < 0) tmp = 0;
        if (tmp > 255) tmp = 255;
        value = static_cast<uint8_t>(tmp);
        return true;
    }

    std::deque<std::string> strings_;
};

} // namespace signalk_mini_linux
