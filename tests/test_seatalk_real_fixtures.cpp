#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>

#include <seatalk.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::filesystem::path test_source_dir() {
    const std::filesystem::path file(__FILE__);
    return file.has_parent_path() ? file.parent_path() : std::filesystem::path(".");
}

static std::filesystem::path fixture_dir() {
    return test_source_dir() / "fixtures" / "seatalk";
}

static int hex_value(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + c - 'a';
    if (c >= 'A' && c <= 'F') return 10 + c - 'A';
    return -1;
}

static bool parse_hex_line(const std::string& line, std::vector<uint8_t>& bytes) {
    bytes.clear();
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        if (i >= line.size() || line[i] == '#') break;
        if (i + 1 >= line.size()) return false;
        const int hi = hex_value(line[i]);
        const int lo = hex_value(line[i + 1]);
        if (hi < 0 || lo < 0) return false;
        bytes.push_back(static_cast<uint8_t>((hi << 4) | lo));
        i += 2;
        if (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i])) && line[i] != '#') return false;
    }
    return !bytes.empty();
}

static std::vector<std::filesystem::path> list_fixture_paths() {
    std::vector<std::filesystem::path> paths;
    const auto dir = fixture_dir();
    REQUIRE(std::filesystem::exists(dir));
    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".hex") paths.push_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    REQUIRE(!paths.empty());
    return paths;
}

static int feed_fixture(const std::filesystem::path& path,
                        seatalk::SeaTalkReceiver<float>& receiver,
                        ship_data_model::DataModel<float>& model) {
    std::ifstream input(path);
    REQUIRE(input.good());
    uint64_t now_us = 0;
    int record_count = 0;
    std::string line;
    std::vector<uint8_t> bytes;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        REQUIRE(parse_hex_line(line, bytes));
        now_us += 1000;
        REQUIRE(receiver.accept_datagram(bytes.data(), bytes.size(), model, now_us, ship_data_model::SensorSource::serial));
        ++record_count;
    }
    REQUIRE(record_count > 0);
    return record_count;
}

int main() {
    int fixture_count = 0;
    int record_count = 0;
    for (const auto& path : list_fixture_paths()) {
        seatalk::SeaTalkReceiver<float> receiver;
        ship_data_model::DataModel<float> model;
        record_count += feed_fixture(path, receiver, model);
        ++fixture_count;
        REQUIRE(receiver.decoded_count() > 0);
        REQUIRE(receiver.unsupported_count() == 0);
        REQUIRE(model.sea.depth_m.valid);
        REQUIRE(model.propulsion.revolutions.speed_rpm.valid);
        REQUIRE(model.propulsion.revolutions.propeller_pitch_percent.valid);
        REQUIRE(model.wind.apparent.direction_deg.valid);
        REQUIRE(model.wind.apparent.speed_kn.valid);
        REQUIRE(model.sea.speed_kn.valid);
        REQUIRE(model.sea.temperature_c.valid);
        REQUIRE(model.ins.imu.heading_magnetic_deg.valid);
        REQUIRE(model.steering.rudder.angle_deg.valid);
        REQUIRE(model.sea.total_distance_nmi.valid);
        REQUIRE(model.sea.trip_distance_nmi.valid);
        REQUIRE(model.gnss.fix.fix_lat_deg.valid);
        REQUIRE(model.gnss.fix.fix_lon_deg.valid);
        REQUIRE(model.gnss.fix.speed_kn.valid);
        REQUIRE(model.gnss.fix.track_deg.valid);
        REQUIRE(model.gnss.fix.timestamp_s.valid);
        REQUIRE(model.gnss.fix.date_year.valid);
        REQUIRE(model.gnss.fix.satellites_used.valid);
        REQUIRE(model.gnss.fix.fix_quality.valid);
        REQUIRE(model.gnss.satellites_in_view.satellite_prn[0].valid);
        REQUIRE(model.autopilot.controller.heading_deg.valid);
        REQUIRE(model.autopilot.controller.heading_command_deg.valid);
        REQUIRE(model.autopilot.controller.enabled.value);
        REQUIRE(model.route.apb.xte_nmi.valid);
        REQUIRE(model.route.waypoint.distance_nmi.valid);
        REQUIRE(model.route.waypoint_arrival.arrival_circle_entered.value);
        REQUIRE(std::strcmp(model.route.waypoint_arrival.waypoint_id, "WP01") == 0);
        REQUIRE(std::strcmp(model.notifications.messages.event.event_id, "seatalk_ap_key") == 0);
    }
    REQUIRE(fixture_count > 0);
    REQUIRE(record_count > 0);
    return 0;
}
