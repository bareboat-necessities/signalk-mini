#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <signalk_mini.hpp>
#include "config_loader.hpp"

namespace {

constexpr const char* kConfigFileName = "signalk-mini.conf";
constexpr const char* kUserConfigDirName = ".signalk-mini";
constexpr const char* kEtcConfigPath = "/etc/signalk-mini/signalk-mini.conf";

enum class CliAction {
    Run,
    ExitSuccess,
    ExitFailure,
};

struct ConfigDiscoveryResult {
    std::string path;
    bool explicit_path = false;
    bool created_default = false;
    bool using_built_in_defaults = false;
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

bool file_exists(const std::string& path) {
    return !path.empty() && ::access(path.c_str(), F_OK) == 0;
}

bool directory_exists(const std::string& path) {
    struct stat st{};
    return !path.empty() && ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string user_config_dir() {
    const char* home = std::getenv("HOME");
    if (!home || !home[0]) return std::string();
    std::string path(home);
    if (!path.empty() && path.back() != '/') path += '/';
    path += kUserConfigDirName;
    return path;
}

std::string user_config_path() {
    std::string dir = user_config_dir();
    if (dir.empty()) return std::string();
    return dir + "/" + kConfigFileName;
}

const char* default_config_text() {
    return
        "identity = {\n"
        "  server_name = \"signalk-mini\";\n"
        "  server_version = \"0.1.0\";\n"
        "  self = \"vessels.self\";\n"
        "};\n"
        "\n"
        "signalk = {\n"
        "  host = \"0.0.0.0\";\n"
        "  port = 20223;\n"
        "  max_connections = 8;\n"
        "  allow_rx = true;\n"
        "  allow_tx = true;\n"
        "  websocket = {\n"
        "    enabled = true;\n"
        "    host = \"0.0.0.0\";\n"
        "    port = 3000;\n"
        "    max_connections = 8;\n"
        "    allow_rx = true;\n"
        "    allow_tx = true;\n"
        "  };\n"
        "};\n"
        "\n"
        "publisher = {\n"
        "  interval_us = 10000;\n"
        "  max_changes_per_tick = 32;\n"
        "  json_buffer_size = 512;\n"
        "  source_label = \"signalk-mini\";\n"
        "};\n"
        "\n"
        "connectors = (\n"
        "  {\n"
        "    enabled = true;\n"
        "    label = \"nmea0183-udp-10110\";\n"
        "    protocol = \"nmea0183\";\n"
        "    transport = \"udp\";\n"
        "    access = { allow_rx = true; allow_tx = false; };\n"
        "    nmea0183 = { validate_checksum = false; };\n"
        "    udp = {\n"
        "      listen_host = \"0.0.0.0\";\n"
        "      listen_port = 10110;\n"
        "      allow_broadcast = true;\n"
        "    };\n"
        "  }\n"
        ");\n";
}

bool ensure_user_config_dir(std::string& error) {
    const std::string dir = user_config_dir();
    if (dir.empty()) {
        error = "HOME is not set";
        return false;
    }
    if (directory_exists(dir)) return true;
    if (::mkdir(dir.c_str(), 0755) == 0) return true;
    if (errno == EEXIST && directory_exists(dir)) return true;
    error = std::string("cannot create ") + dir + ": " + std::strerror(errno);
    return false;
}

bool create_default_user_config(std::string& path, std::string& error) {
    if (!ensure_user_config_dir(error)) return false;
    path = user_config_path();
    if (path.empty()) {
        error = "HOME is not set";
        return false;
    }
    if (file_exists(path)) return true;

    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
        error = std::string("cannot create ") + path;
        return false;
    }
    out << default_config_text();
    if (!out.good()) {
        error = std::string("cannot write ") + path;
        return false;
    }
    return true;
}

ConfigDiscoveryResult discover_config(bool explicit_config, const char* explicit_path) {
    ConfigDiscoveryResult result;
    result.explicit_path = explicit_config;
    if (explicit_config) {
        result.path = explicit_path ? explicit_path : "";
        return result;
    }

    const std::string user_path = user_config_path();
    if (!user_path.empty() && file_exists(user_path)) {
        result.path = user_path;
        return result;
    }

    if (file_exists(kEtcConfigPath)) {
        result.path = kEtcConfigPath;
        return result;
    }

    std::string error;
    if (create_default_user_config(result.path, error)) {
        result.created_default = true;
    } else {
        result.using_built_in_defaults = true;
        std::cerr << "could not create default user config: " << error << "\n";
    }
    return result;
}

void print_usage(const char* argv0) {
    const char* program = argv0 && argv0[0] ? argv0 : "signalk-mini";
    std::cout
        << "Usage: " << program << " [OPTIONS] [CONFIG_FILE]\n\n"
        << "Options:\n"
        << "  -c, --config PATH   Read libconfig configuration from PATH\n"
        << "  -h, --help          Show this help and exit\n"
        << "      --version       Show version and exit\n\n"
        << "Config discovery when no explicit config is provided:\n"
        << "  1. ~/.signalk-mini/signalk-mini.conf\n"
        << "  2. " << kEtcConfigPath << "\n"
        << "  3. create ~/.signalk-mini/signalk-mini.conf with defaults\n\n"
        << "Default endpoints:\n"
        << "  Signal K TCP:       0.0.0.0:20223\n"
        << "  Signal K WebSocket: 0.0.0.0:3000\n"
        << "  NMEA0183 UDP:       0.0.0.0:10110\n";
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

void print_startup_summary(const signalk_mini::SignalKMiniConfig& config, const ConfigDiscoveryResult& discovery) {
    std::cout << "signalk-mini " << (config.identity.server_version ? config.identity.server_version : "unknown") << " started\n";
    if (discovery.using_built_in_defaults) {
        std::cout << "config: built-in defaults\n";
    } else {
        std::cout << "config: " << discovery.path << "\n";
        if (discovery.created_default) std::cout << "created default config: " << discovery.path << "\n";
    }
    std::cout << "Signal K TCP listening on "
              << (config.signalk.host ? config.signalk.host : "0.0.0.0")
              << ":" << config.signalk.port
              << " max_connections=" << config.signalk.max_connections
              << " rx=" << (config.signalk.allow_rx ? "yes" : "no")
              << " tx=" << (config.signalk.allow_tx ? "yes" : "no")
              << "\n";
    if (config.signalk.websocket.enabled) {
        std::cout << "Signal K WebSocket listening on "
                  << (config.signalk.websocket.host ? config.signalk.websocket.host : "0.0.0.0")
                  << ":" << config.signalk.websocket.port
                  << " max_connections=" << config.signalk.websocket.max_connections
                  << " rx=" << (config.signalk.websocket.allow_rx ? "yes" : "no")
                  << " tx=" << (config.signalk.websocket.allow_tx ? "yes" : "no")
                  << "\n";
    } else {
        std::cout << "Signal K WebSocket disabled\n";
    }
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

void print_startup_failure(const signalk_mini::SignalKMiniConfig& config,
                           signalk_mini::MiniSignalKServer<float>& app) {
    const auto error = app.last_startup_error();
    const size_t failed_index = app.last_failed_connector_index();
    std::cerr << "failed to start signalk-mini: " << startup_error_to_string(error);
    if (failed_index < config.connector_count) {
        std::cerr << "; connector_index=" << failed_index;
        const signalk_mini::ConnectorConfig& connector = config.connectors[failed_index];
        std::cerr << "; connector_label=" << (connector.label && connector.label[0] ? connector.label : "-")
                  << "; connector_transport=" << transport_to_string(connector.transport.kind);
    } else {
        std::cerr << "; listener_or_core_startup=true";
        if (error == signalk_mini::MiniSignalKServer<float>::StartupError::ConnectorStartFailed) {
            std::cerr << "; check Signal K TCP "
                      << (config.signalk.host ? config.signalk.host : "0.0.0.0")
                      << ":" << config.signalk.port;
            if (config.signalk.websocket.enabled) {
                std::cerr << " and Signal K WebSocket "
                          << (config.signalk.websocket.host ? config.signalk.websocket.host : "0.0.0.0")
                          << ":" << config.signalk.websocket.port;
            }
        }
    }
    std::cerr << "; connector_start_failures=" << app.connector_start_failure_count() << "\n";
}

CliAction parse_args(int argc, char** argv, const char*& config_path, bool& explicit_config) {
    config_path = nullptr;
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
        if (explicit_config || config_path) {
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
    const char* explicit_config_path = nullptr;
    bool explicit_config = false;
    const CliAction cli_action = parse_args(argc, argv, explicit_config_path, explicit_config);
    if (cli_action == CliAction::ExitSuccess) return 0;
    if (cli_action == CliAction::ExitFailure) return 1;

    const ConfigDiscoveryResult discovery = discover_config(explicit_config, explicit_config_path);

    signalk_mini_linux::ConfigLoader config_loader;
    std::string error;
    if (!discovery.using_built_in_defaults && !config_loader.load_file(discovery.path.c_str(), error)) {
        std::cerr << error << "\n";
        return 1;
    }

    signalk_mini::SignalKMiniApp<float> app(config_loader.config);
    if (!app.begin()) {
        print_startup_failure(config_loader.config, app);
        return 1;
    }
    print_startup_summary(config_loader.config, discovery);
    app.run_forever();
    return 0;
}
