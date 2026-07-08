#include <cstring>
#include <iostream>
#include <string>

#include <signalk_mini.hpp>
#include "config_loader.hpp"

namespace {

constexpr const char* kDefaultConfigPath = "/etc/signalk-mini/signalk-mini.conf";

enum class CliAction {
    Run,
    ExitSuccess,
    ExitFailure,
};

const char* startup_error_to_string(signalk_mini::MiniSignalKServer<float>::StartupError error) {
    using StartupError = signalk_mini::MiniSignalKServer<float>::StartupError;
    switch (error) {
    case StartupError::None: return "none";
    case StartupError::InvalidSignalKPort: return "invalid Signal K port";
    case StartupError::InvalidSignalKMaxConnections: return "invalid Signal K max connections";
    case StartupError::InvalidPublisherInterval: return "invalid publisher interval";
    case StartupError::InvalidConnectorList: return "invalid connector list";
    case StartupError::UnsupportedConnectorProtocol: return "unsupported connector protocol";
    case StartupError::UnsupportedConnectorTransport: return "unsupported connector transport";
    case StartupError::InvalidConnectorTransportConfig: return "invalid connector transport config";
    case StartupError::ConnectorStartFailed: return "connector or listener start failed";
    }
    return "unknown startup error";
}

const char* protocol_to_string(signalk_mini::ConnectorProtocol protocol) {
    switch (protocol) {
    case signalk_mini::ConnectorProtocol::None: return "none";
    case signalk_mini::ConnectorProtocol::Nmea0183: return "nmea0183";
    case signalk_mini::ConnectorProtocol::SeaTalk1: return "seatalk1";
    case signalk_mini::ConnectorProtocol::Nmea2000: return "nmea2000";
    case signalk_mini::ConnectorProtocol::SignalK: return "signalk";
    case signalk_mini::ConnectorProtocol::GenericSensor: return "generic_sensor";
    }
    return "unknown";
}

const char* transport_to_string(signalk_mini::ConnectorTransport transport) {
    switch (transport) {
    case signalk_mini::ConnectorTransport::None: return "none";
    case signalk_mini::ConnectorTransport::Serial: return "serial";
    case signalk_mini::ConnectorTransport::TcpClient: return "tcp_client";
    case signalk_mini::ConnectorTransport::TcpServer: return "tcp_server";
    case signalk_mini::ConnectorTransport::Udp: return "udp";
    case signalk_mini::ConnectorTransport::I2c: return "i2c";
    case signalk_mini::ConnectorTransport::DigitalPin: return "digital_pin";
    case signalk_mini::ConnectorTransport::AnalogPin: return "analog_pin";
    }
    return "unknown";
}

void print_usage(const char* argv0) {
    const char* program = argv0 && argv0[0] ? argv0 : "signalk-mini";
    std::cout
        << "Usage: " << program << " [OPTIONS] [CONFIG_FILE]\n\n"
        << "Options:\n"
        << "  -c, --config PATH   Read libconfig configuration from PATH\n"
        << "  -h, --help          Show this help and exit\n"
        << "      --version       Show version and exit\n\n"
        << "Defaults when no config file is found:\n"
        << "  config:       " << kDefaultConfigPath << "\n"
        << "  Signal K TCP: 0.0.0.0:20223\n"
        << "  NMEA0183 UDP: 0.0.0.0:10110\n";
}

void print_connector_summary(const signalk_mini::ConnectorConfig& connector, size_t index) {
    std::cout << "connector[" << index << "]: "
              << (connector.enabled ? "enabled" : "disabled")
              << " label=" << (connector.label && connector.label[0] ? connector.label : "-")
              << " protocol=" << protocol_to_string(connector.protocol.kind)
              << " transport=" << transport_to_string(connector.transport.kind);

    switch (connector.transport.kind) {
    case signalk_mini::ConnectorTransport::Serial:
        std::cout << " device=" << (connector.transport.serial.device ? connector.transport.serial.device : "-")
                  << " baud=" << connector.transport.serial.baud;
        break;
    case signalk_mini::ConnectorTransport::TcpClient:
        std::cout << " remote=" << (connector.transport.tcp_client.host ? connector.transport.tcp_client.host : "-")
                  << ":" << connector.transport.tcp_client.port;
        break;
    case signalk_mini::ConnectorTransport::TcpServer:
        std::cout << " listen=" << (connector.transport.tcp_server.host ? connector.transport.tcp_server.host : "-")
                  << ":" << connector.transport.tcp_server.port;
        break;
    case signalk_mini::ConnectorTransport::Udp:
        std::cout << " listen=" << (connector.transport.udp.listen_host ? connector.transport.udp.listen_host : "-")
                  << ":" << connector.transport.udp.listen_port;
        if (connector.transport.udp.remote_host && connector.transport.udp.remote_host[0]) {
            std::cout << " remote=" << connector.transport.udp.remote_host << ":" << connector.transport.udp.remote_port;
        }
        break;
    default:
        break;
    }

    std::cout << " rx=" << (connector.access.allow_rx ? "yes" : "no")
              << " tx=" << (connector.access.allow_tx ? "yes" : "no")
              << "\n";
}

void print_startup_summary(const signalk_mini::SignalKMiniConfig& config, bool used_default_config) {
    std::cout << "signalk-mini " << (config.identity.server_version ? config.identity.server_version : "unknown") << " started\n";
    if (used_default_config) {
        std::cout << "config: built-in defaults\n";
    }
    std::cout << "Signal K TCP listening on "
              << (config.signalk.host ? config.signalk.host : "0.0.0.0")
              << ":" << config.signalk.port
              << " max_connections=" << config.signalk.max_connections
              << " rx=" << (config.signalk.allow_rx ? "yes" : "no")
              << " tx=" << (config.signalk.allow_tx ? "yes" : "no")
              << "\n";
    std::cout << "publisher interval_us=" << config.publisher.interval_us
              << " max_changes_per_tick=" << config.publisher.max_changes_per_tick
              << " json_buffer_size=" << config.publisher.json_buffer_size
              << "\n";
    if (!config.connectors || config.connector_count == 0) {
        std::cout << "connectors: none\n";
        return;
    }
    for (size_t i = 0; i < config.connector_count; ++i) {
        print_connector_summary(config.connectors[i], i);
    }
}

CliAction parse_args(int argc, char** argv, const char*& config_path, bool& explicit_config) {
    config_path = kDefaultConfigPath;
    explicit_config = false;
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if (std::strcmp(arg, "-h") == 0 || std::strcmp(arg, "--help") == 0) {
            print_usage(argv[0]);
            return CliAction::ExitSuccess;
        }
        if (std::strcmp(arg, "--version") == 0) {
            std::cout << "signalk-mini 0.1.0\n";
            return CliAction::ExitSuccess;
        }
        if (std::strcmp(arg, "-c") == 0 || std::strcmp(arg, "--config") == 0) {
            if (i + 1 >= argc) {
                std::cerr << arg << " requires a path\n";
                return CliAction::ExitFailure;
            }
            config_path = argv[++i];
            explicit_config = true;
            continue;
        }
        if (arg[0] == '-') {
            std::cerr << "unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return CliAction::ExitFailure;
        }
        if (explicit_config || std::strcmp(config_path, kDefaultConfigPath) != 0) {
            std::cerr << "multiple config paths provided\n";
            return CliAction::ExitFailure;
        }
        config_path = arg;
        explicit_config = true;
    }
    return CliAction::Run;
}

} // namespace

int main(int argc, char** argv) {
    const char* config_path = nullptr;
    bool explicit_config = false;
    const CliAction cli_action = parse_args(argc, argv, config_path, explicit_config);
    if (cli_action == CliAction::ExitSuccess) return 0;
    if (cli_action == CliAction::ExitFailure) return 1;

    signalk_mini_linux::ConfigLoader config_loader;
    std::string error;
    bool used_default_config = false;
    if (!config_loader.load_file(config_path, error)) {
        if (explicit_config) {
            std::cerr << error << "\n";
            return 1;
        }
        used_default_config = true;
        std::cerr << "using built-in defaults: " << error << "\n";
    }

    signalk_mini::SignalKMiniApp<float> app(config_loader.config);
    if (!app.begin()) {
        std::cerr << "failed to start signalk-mini: "
                  << startup_error_to_string(app.last_startup_error())
                  << "; connector_index=" << app.last_failed_connector_index()
                  << "; connector_start_failures=" << app.connector_start_failure_count()
                  << "\n";
        return 1;
    }
    print_startup_summary(config_loader.config, used_default_config);
    app.run_forever();
    return 0;
}
