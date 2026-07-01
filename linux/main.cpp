#include <iostream>
#include <string>
#include <signalk_mini.hpp>

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    std::string line;
    uint64_t now_us = 0;
    while (std::getline(std::cin, line)) {
        now_us += 100000;
        app.nmea0183().feed_line(line.c_str(), 1, now_us);
    }
    std::cout << "changes=" << app.store().sequence() << "\n";
    return 0;
}
