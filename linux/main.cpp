#include <iostream>
#include <string>
#include <signalk_mini.hpp>

int main() {
    using Real = float;
    signalk_mini::ModelStore<Real> store;
    signalk_mini::Nmea0183Input<Real> input(store);
    std::string line;
    uint64_t now_us = 0;
    while (std::getline(std::cin, line)) {
        now_us += 100000;
        input.feed_line(line.c_str(), 1, now_us);
    }
    std::cout << "changes=" << store.sequence() << "\n";
    const auto& model = store.model();
    if (model.navigation.gps.fix_lat_deg.valid && model.navigation.gps.fix_lon_deg.valid) {
        std::cout << "position_deg=" << model.navigation.gps.fix_lat_deg.value << "," << model.navigation.gps.fix_lon_deg.value << "\n";
    }
    return 0;
}
