#include <iostream>
#include <string>

#include <signalk_mini.hpp>
#include "config_loader.hpp"

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
    if (!app.begin()) return 1;
    app.run_forever();
    return 0;
}
