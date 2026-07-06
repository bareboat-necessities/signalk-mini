#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static constexpr int MAX_FIXTURE_DATA_LINES = 500;

struct FixtureCounts {
    int data_lines = 0;
    int accepted_lines = 0;
    int rejected_lines = 0;
};

static std::string test_source_dir() {
    const std::string file = __FILE__;
    const std::string::size_type slash = file.find_last_of("/\\");
    return slash == std::string::npos ? "." : file.substr(0, slash);
}

static std::string fixture_path(const char* name) {
    return test_source_dir() + "/fixtures/nmea/" + name;
}

static bool is_expected_reject(const std::string& line, const char* last_error) {
    if (line.find("$GPRMC,,V") == 0 &&
        (std::strcmp(last_error, "bad RMC") == 0 || std::strcmp(last_error, "invalid RMC") == 0)) return true;
    if (line.find("$GNGLL,,,,") == 0 && std::strcmp(last_error, "bad GLL") == 0) return true;
    if (line.find("$GPGGA,,,,,,0") == 0 && std::strcmp(last_error, "invalid GGA") == 0) return true;
    if (line.find("$GNGGA,") == 0 && line.find(",0,") != std::string::npos && std::strcmp(last_error, "invalid GGA") == 0) return true;
    return false;
}

static FixtureCounts feed_fixture(signalk_mini::SignalKMiniApp<float>& app, const char* name) {
    FixtureCounts counts;
    std::ifstream input(fixture_path(name));
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
        std::fprintf(stderr, "Failed fixture %s line %d: %s\n", name, counts.data_lines, line.c_str());
        std::fprintf(stderr, "last_error=%s\n", app.nmea0183().last_error());
        std::exit(3);
    }

    REQUIRE(counts.data_lines > 0);
    REQUIRE(counts.data_lines <= MAX_FIXTURE_DATA_LINES);
    REQUIRE(counts.accepted_lines > 0);
    REQUIRE(counts.accepted_lines + counts.rejected_lines == counts.data_lines);
    return counts;
}

static void check_core_real_fixtures() {
    signalk_mini::SignalKMiniApp<float> app1;
    REQUIRE(feed_fixture(app1, "tripmate_850_gps_sample.nmea").rejected_lines == 0);
    REQUIRE(app1.store().model().gnss.fix.fix_lat_deg.valid);
    REQUIRE(app1.store().model().gnss.fix.fix_lon_deg.valid);

    signalk_mini::SignalKMiniApp<float> app2;
    REQUIRE(feed_fixture(app2, "aerorust_nmea2.nmea").data_lines == 36);
    REQUIRE(app2.store().model().gnss.satellites_in_view.satellites_in_view.valid);

    signalk_mini::SignalKMiniApp<float> app3;
    REQUIRE(feed_fixture(app3, "aerorust_nmea_with_sat_info.nmea").data_lines == 100);

    signalk_mini::SignalKMiniApp<float> app4;
    REQUIRE(feed_fixture(app4, "gpsbabel_nmea_ms.nmea").rejected_lines == 0);
    REQUIRE(app4.store().model().gnss.fix.fix_lat_deg.valid);

    signalk_mini::SignalKMiniApp<float> app5;
    REQUIRE(feed_fixture(app5, "gpsd_ais_18_27.nmea").rejected_lines == 0);

    signalk_mini::SignalKMiniApp<float> app6;
    REQUIRE(feed_fixture(app6, "dsc_signalk_examples.nmea").rejected_lines == 0);
    REQUIRE(app6.store().model().comm.dsc.call_count.valid);

    signalk_mini::SignalKMiniApp<float> app7;
    REQUIRE(feed_fixture(app7, "navtex_swiftnmea_nrx.nmea").rejected_lines == 0);
    REQUIRE(app7.store().model().notifications.navtex.received.text_length.valid);
}

static void check_safetynet_sm_fixture() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "safetynet_sm_actual.nmea");
    REQUIRE(counts.data_lines == 9);
    REQUIRE(counts.rejected_lines == 0);
    const auto& latest = app.store().model().notifications.inmarsat.safetynet.latest_message;
    REQUIRE(std::strcmp(latest.message_id, "300001") == 0);
    REQUIRE(std::strcmp(latest.address_kind, "rectangular_area") == 0);
    REQUIRE(latest.rectangle_sw_lat_deg.valid);
    REQUIRE(latest.rectangle_sw_lon_deg.valid);
    REQUIRE(latest.rectangle_extent_lat_deg.valid);
    REQUIRE(latest.rectangle_extent_lon_deg.valid);
}

int main() {
    check_core_real_fixtures();
    check_safetynet_sm_fixture();
    return 0;
}
