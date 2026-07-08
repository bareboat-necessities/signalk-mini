#include <iostream>
#include <string>

#include <signalk_mini.hpp>
#include "config_loader.hpp"

namespace {

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

} // namespace

int main(int argc, char** argv) {
    const char* config_path = argc > 1 ? argv[1] : "/etc/signalk-mini/signalk-mini.conf";

    signalk_mini_linux::ConfigLoader config_loader;
    std::string error;
    if (!config_loader.load_file(config_path, error)) {
        if (argc > 1) {
            std::cerr << error << "\n";
            return 1;
        }
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
    app.run_forever();
    return 0;
}
