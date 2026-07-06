#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

struct FixtureCounts {
    int data_lines = 0;
    int accepted_lines = 0;
    int rejected_lines = 0;
};

static std::filesystem::path test_source_dir() {
    const std::filesystem::path file(__FILE__);
    return file.has_parent_path() ? file.parent_path() : std::filesystem::path(".");
}

static std::filesystem::path fixture_dir() {
    return test_source_dir() / "fixtures" / "nmea";
}

static bool is_expected_reject(const std::string& line, const char* last_error) {
    if (line.find("$GPRMC,,V") == 0 &&
        (std::strcmp(last_error, "bad RMC") == 0 || std::strcmp(last_error, "invalid RMC") == 0)) return true;
    if (line.find("$GNGLL,,,,") == 0 && std::strcmp(last_error, "bad GLL") == 0) return true;
    if (line.find("$GPGGA,,,,,,0") == 0 && std::strcmp(last_error, "invalid GGA") == 0) return true;
    if (line.find("$GNGGA,") == 0 && line.find(",0,") != std::string::npos && std::strcmp(last_error, "invalid GGA") == 0) return true;
    return false;
}

static FixtureCounts feed_fixture_path(signalk_mini::SignalKMiniApp<float>& app,
                                       const std::filesystem::path& path) {
    FixtureCounts counts;
    std::ifstream input(path);
    REQUIRE(input.good());

    uint64_t now_us = 0;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        REQUIRE(line[0] == '$' || line[0] == '!' || line[0] == '/');
        ++counts.data_lines;
        now_us += 1000000ULL;
        if (app.nmea0183().feed_line(line.c_str(), 1, now_us)) {
            ++counts.accepted_lines;
            continue;
        }
        if (is_expected_reject(line, app.nmea0183().last_error())) {
            ++counts.rejected_lines;
            continue;
        }
        std::fprintf(stderr, "Failed fixture %s line %d: %s\n",
                     path.filename().string().c_str(),
                     counts.data_lines,
                     line.c_str());
        std::fprintf(stderr, "last_error=%s\n", app.nmea0183().last_error());
        std::exit(3);
    }

    REQUIRE(counts.data_lines > 0);
    REQUIRE(counts.accepted_lines > 0);
    REQUIRE(counts.accepted_lines + counts.rejected_lines == counts.data_lines);
    return counts;
}

static std::vector<std::filesystem::path> list_nmea_fixture_paths() {
    std::vector<std::filesystem::path> fixtures;
    const auto dir = fixture_dir();
    REQUIRE(std::filesystem::exists(dir));
    REQUIRE(std::filesystem::is_directory(dir));

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() == ".nmea") fixtures.push_back(entry.path());
    }

    std::sort(fixtures.begin(), fixtures.end());
    REQUIRE(!fixtures.empty());
    return fixtures;
}

static void check_known_fixture_semantics(const std::string& name,
                                          const signalk_mini::SignalKMiniApp<float>& app,
                                          const FixtureCounts& counts) {
    const auto& model = app.store().model();

    if (name == "tripmate_850_gps_sample.nmea") {
        REQUIRE(counts.rejected_lines == 0);
        REQUIRE(model.gnss.fix.fix_lat_deg.valid);
        REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    } else if (name == "aerorust_nmea2.nmea") {
        REQUIRE(model.gnss.satellites_in_view.satellites_in_view.valid);
    } else if (name == "gpsbabel_nmea_ms.nmea") {
        REQUIRE(counts.rejected_lines == 0);
        REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    } else if (name == "gpsd_ais_18_27.nmea") {
        REQUIRE(counts.rejected_lines == 0);
    } else if (name == "dsc_signalk_examples.nmea") {
        REQUIRE(counts.rejected_lines == 0);
        REQUIRE(model.comm.dsc.call_count.valid);
    } else if (name == "navtex_swiftnmea_nrx.nmea") {
        REQUIRE(counts.rejected_lines == 0);
        REQUIRE(model.notifications.navtex.received.text_length.valid);
    } else if (name == "safetynet_sm_actual.nmea") {
        REQUIRE(counts.data_lines == 9);
        REQUIRE(counts.rejected_lines == 0);
        const auto& latest = model.notifications.inmarsat.safetynet.latest_message;
        REQUIRE(std::strcmp(latest.message_id, "300001") == 0);
        REQUIRE(std::strcmp(latest.address_kind, "rectangular_area") == 0);
        REQUIRE(latest.rectangle_sw_lat_deg.valid);
        REQUIRE(latest.rectangle_sw_lon_deg.valid);
        REQUIRE(latest.rectangle_extent_lat_deg.valid);
        REQUIRE(latest.rectangle_extent_lon_deg.valid);
    }
}

int main() {
    const auto fixtures = list_nmea_fixture_paths();
    int fixture_count = 0;
    int total_data_lines = 0;

    for (const auto& path : fixtures) {
        signalk_mini::SignalKMiniApp<float> app;
        const auto counts = feed_fixture_path(app, path);
        ++fixture_count;
        total_data_lines += counts.data_lines;
        check_known_fixture_semantics(path.filename().string(), app, counts);
    }

    REQUIRE(fixture_count == static_cast<int>(fixtures.size()));
    REQUIRE(total_data_lines > 0);
    return 0;
}
