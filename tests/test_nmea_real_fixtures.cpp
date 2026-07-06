#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static constexpr int MAX_FIXTURE_DATA_LINES = 500;

struct FixtureCounts {
    int data_lines = 0;
    int accepted_lines = 0;
    int rejected_lines = 0;
};

static std::string test_source_dir() {
    const std::string file = __FILE__;
    const std::string::size_type slash = file.find_last_of("/\\");
    if (slash == std::string::npos) return ".";
    return file.substr(0, slash);
}

static std::string fixture_path(const char* name) {
    return test_source_dir() + "/fixtures/nmea/" + name;
}

static bool is_expected_reject(const std::string& line, const char* last_error) {
    if (line.find("$GPRMC,,V") == 0 && std::strcmp(last_error, "bad RMC") == 0) return true;
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
        REQUIRE(line[0] == '$' || line[0] == '!');
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

static void check_tripmate_sample() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "tripmate_850_gps_sample.nmea");
    REQUIRE(counts.data_lines == 12);
    REQUIRE(counts.rejected_lines == 0);
    const auto& model = app.store().model();

    REQUIRE(std::strcmp(model.gnss.fix.talker_id, "GP") == 0);
    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    NEAR(model.gnss.fix.fix_lat_deg.value, 53.361336f, 0.001f);
    NEAR(model.gnss.fix.fix_lon_deg.value, -6.505618f, 0.001f);
    REQUIRE(model.gnss.fix.satellites_used.value == 8);
    NEAR(model.gnss.fix.hdop.value, 1.03f, 0.001f);
    NEAR(model.gnss.fix.speed_kn.value, 0.06f, 0.001f);
    NEAR(model.gnss.fix.track_deg.value, 31.66f, 0.001f);
    REQUIRE(model.gnss.satellites_in_view.satellites_in_view.value == 11);
}

static void check_aerorust_nmea2() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "aerorust_nmea2.nmea");
    REQUIRE(counts.data_lines == 36);
    const auto& model = app.store().model();
    REQUIRE(model.gnss.satellites_in_view.satellites_in_view.valid);
    REQUIRE(model.notifications.messages.text.field_count.valid);
}

static void check_aerorust_satellite_fixture() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "aerorust_nmea_with_sat_info.nmea");
    REQUIRE(counts.data_lines == 100);
    const auto& model = app.store().model();
    REQUIRE(model.gnss.fix.fix_lat_deg.valid || model.gnss.satellites_in_view.satellites_in_view.valid);
}

static void check_gpsbabel_nmea_ms() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "gpsbabel_nmea_ms.nmea");
    REQUIRE(counts.data_lines > 100);
    REQUIRE(counts.rejected_lines == 0);
    const auto& model = app.store().model();
    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    REQUIRE(model.gnss.fix.speed_kn.valid);
    REQUIRE(model.gnss.fix.track_deg.valid);
}

static void check_gpsd_ais_18_27() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "gpsd_ais_18_27.nmea");
    REQUIRE(counts.data_lines > 200);
    REQUIRE(counts.rejected_lines == 0);
    const auto& model = app.store().model();
    REQUIRE(model.ais.position_report.mmsi.valid);
    REQUIRE(model.ais.long_range_broadcast.mmsi.valid);
}

static void check_dsc_signalk_examples() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "dsc_signalk_examples.nmea");
    REQUIRE(counts.data_lines == 19);
    REQUIRE(counts.rejected_lines == 0);
    // Most Signal K DSC examples set the DSE expansion flag. The one-second
    // fixture cadence commits the prior pending DSC on the next input line.
    REQUIRE(app.store().model().comm.dsc.call_count.valid);
    REQUIRE(app.store().model().comm.dsc.call_count.value > 0);
    REQUIRE(app.store().model().comm.dsc.latest_call.sender_mmsi[0] != '\0');
}

static void check_navtex_swiftnmea_nrx() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "navtex_swiftnmea_nrx.nmea");
    REQUIRE(counts.data_lines == 3);
    REQUIRE(counts.rejected_lines == 0);
}

int main() {
    check_tripmate_sample();
    check_aerorust_nmea2();
    check_aerorust_satellite_fixture();
    check_gpsbabel_nmea_ms();
    check_gpsd_ais_18_27();
    check_dsc_signalk_examples();
    check_navtex_swiftnmea_nrx();
    return 0;
}
