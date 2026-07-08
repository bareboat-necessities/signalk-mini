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

struct SeaTalkFixtureCoverage {
    bool depth = false;
    bool engine = false;
    bool wind_angle = false;
    bool wind_speed = false;
    bool water_speed = false;
    bool water_temperature = false;
    bool heading = false;
    bool rudder = false;
    bool log = false;
    bool gps_position = false;
    bool gps_speed_track = false;
    bool gps_time_date = false;
    bool gps_satellite = false;
    bool autopilot = false;
    bool waypoint_nav = false;
    bool arrival = false;
    bool autopilot_key = false;
};

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

static void update_coverage(const ship_data_model::DataModel<float>& model, SeaTalkFixtureCoverage& coverage) {
    coverage.depth = coverage.depth || model.sea.depth_m.valid;
    coverage.engine = coverage.engine || (model.propulsion.revolutions.speed_rpm.valid && model.propulsion.revolutions.propeller_pitch_percent.valid);
    coverage.wind_angle = coverage.wind_angle || model.wind.apparent.direction_deg.valid;
    coverage.wind_speed = coverage.wind_speed || model.wind.apparent.speed_kn.valid;
    coverage.water_speed = coverage.water_speed || model.sea.speed_kn.valid;
    coverage.water_temperature = coverage.water_temperature || model.sea.temperature_c.valid;
    coverage.heading = coverage.heading || model.ins.imu.heading_magnetic_deg.valid;
    coverage.rudder = coverage.rudder || model.steering.rudder.angle_deg.valid;
    coverage.log = coverage.log || (model.route.log.total_distance_nmi.valid && model.route.log.trip_distance_nmi.valid);
    coverage.gps_position = coverage.gps_position || (model.gnss.fix.fix_lat_deg.valid && model.gnss.fix.fix_lon_deg.valid);
    coverage.gps_speed_track = coverage.gps_speed_track || (model.gnss.fix.speed_kn.valid && model.gnss.fix.track_deg.valid);
    coverage.gps_time_date = coverage.gps_time_date || (model.gnss.fix.timestamp_s.valid && model.gnss.fix.date_year.valid);
    coverage.gps_satellite = coverage.gps_satellite || (model.gnss.fix.satellites_used.valid && model.gnss.satellites_in_view.satellite_prn[0].valid);
    coverage.autopilot = coverage.autopilot || (model.autopilot.controller.heading_deg.valid && model.autopilot.controller.heading_command_deg.valid);
    coverage.waypoint_nav = coverage.waypoint_nav || (model.route.apb.xte_nmi.valid && model.route.waypoint.distance_nmi.valid);
    coverage.arrival = coverage.arrival || (model.route.waypoint_arrival.arrival_circle_entered.value && std::strcmp(model.route.waypoint_arrival.waypoint_id, "WP01") == 0);
    coverage.autopilot_key = coverage.autopilot_key || std::strcmp(model.notifications.messages.event.event_id, "seatalk_ap_key") == 0;
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
    SeaTalkFixtureCoverage coverage;

    for (const auto& path : list_fixture_paths()) {
        seatalk::SeaTalkReceiver<float> receiver;
        ship_data_model::DataModel<float> model;
        record_count += feed_fixture(path, receiver, model);
        ++fixture_count;
        REQUIRE(receiver.decoded_count() > 0);
        REQUIRE(receiver.unsupported_count() == 0);
        update_coverage(model, coverage);
    }

    REQUIRE(fixture_count > 0);
    REQUIRE(record_count > 0);
    REQUIRE(coverage.depth);
    REQUIRE(coverage.engine);
    REQUIRE(coverage.wind_angle);
    REQUIRE(coverage.wind_speed);
    REQUIRE(coverage.water_speed);
    REQUIRE(coverage.water_temperature);
    REQUIRE(coverage.heading);
    REQUIRE(coverage.rudder);
    REQUIRE(coverage.log);
    REQUIRE(coverage.gps_position);
    REQUIRE(coverage.gps_speed_track);
    REQUIRE(coverage.gps_time_date);
    REQUIRE(coverage.gps_satellite);
    REQUIRE(coverage.autopilot);
    REQUIRE(coverage.waypoint_nav);
    REQUIRE(coverage.arrival);
    REQUIRE(coverage.autopilot_key);
    return 0;
}
