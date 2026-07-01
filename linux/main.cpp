#include <iostream>
#include <string>
#include <signalk_mini.hpp>

#if defined(SIGNALK_MINI_ENABLE_NATIVE_TCP)
#include <signalk_mini/server.hpp>
#endif

int main() {
#if defined(SIGNALK_MINI_ENABLE_NATIVE_TCP)
    signalk_mini::SignalKMiniConfig config;
    signalk_mini::MiniSignalKServer<float> server(config);
    if (!server.begin()) return 2;
    server.run_forever();
    return 0;
#else
    signalk_mini::SignalKMiniApp<float> app;
    std::string line;
    uint64_t now_us = 0;
    while (std::getline(std::cin, line)) {
        now_us += 100000;
        app.nmea0183().feed_line(line.c_str(), 1, now_us);
    }
    std::cout << "changes=" << app.store().sequence() << "\n";
    return 0;
#endif
}
