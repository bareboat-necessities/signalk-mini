#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

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

static bool is_expected_reject(const char* name, const std::string& line, const char* last_error) {
    if (std::strcmp(name, "aerorust_gnss_supported.nmea") == 0 &&
        line.find("$GPGGA,133605.0") == 0 &&
        std::strcmp(last_error, "invalid GGA") == 0) {
        return true;
    }
    if (std::strcmp(name, "gpsd_reference_edges.nmea") == 0 &&
        line.find("$GPRMC,,V") == 0 &&
        std::strcmp(last_error, "bad RMC") == 0) {
        return true;
    }
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
        ++counts.data_lines;
        now_us += 1000000ULL;
        if (app.nmea0183().feed_line(line.c_str(), 1, now_us)) {
            ++counts.accepted_lines;
            continue;
        }
        if (is_expected_reject(name, line, app.nmea0183().last_error())) {
            ++counts.rejected_lines;
            continue;
        }
        std::fprintf(stderr, "Failed fixture %s line %d: %s\n", name, counts.data_lines, line.c_str());
        std::fprintf(stderr, "last_error=%s\n", app.nmea0183().last_error());
        std::exit(3);
    }
    return counts;
}

static void check_aerorust_gnss_supported() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "aerorust_gnss_supported.nmea");
    REQUIRE(counts.data_lines == 9);
    REQUIRE(counts.accepted_lines == 8);
    REQUIRE(counts.rejected_lines == 1);
    const auto& model = app.store().model();

    REQUIRE(std::strcmp(model.gnss.fix.talker_id, "GP") == 0);
    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    NEAR(model.gnss.fix.fix_lat_deg.value, 49.274166f, 0.001f);
    NEAR(model.gnss.fix.fix_lon_deg.value, -123.185333f, 0.001f);

    REQUIRE(model.gnss.fix.speed_kn.valid);
    REQUIRE(model.gnss.fix.track_deg.valid);
    NEAR(model.gnss.fix.speed_kn.value, 0.0f, 0.001f);
    NEAR(model.gnss.fix.track_deg.value, 0.0f, 0.001f);

    REQUIRE(model.gnss.satellites_in_view.satellites_in_view.value == 12);
    REQUIRE(model.gnss.satellites_in_view.satellite_prn[0].value == 1);
    REQUIRE(model.notifications.messages.text.field_count.valid);
    REQUIRE(model.notifications.messages.text.field_count.value >= 4);
}

static void check_tripmate_sample() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "tripmate_850_gps_sample.nmea");
    REQUIRE(counts.data_lines == 12);
    REQUIRE(counts.accepted_lines == 12);
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

static void check_gpsd_reference_edges() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "gpsd_reference_edges.nmea");
    REQUIRE(counts.data_lines == 8);
    REQUIRE(counts.accepted_lines == 7);
    REQUIRE(counts.rejected_lines == 1);
    const auto& model = app.store().model();

    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    NEAR(model.gnss.fix.fix_lat_deg.value, 49.274166f, 0.001f);
    NEAR(model.gnss.fix.fix_lon_deg.value, -123.185333f, 0.001f);
    REQUIRE(model.gnss.fix.speed_kn.valid);
    REQUIRE(model.gnss.fix.track_deg.valid);
    NEAR(model.gnss.fix.speed_kn.value, 5.5f, 0.001f);
    NEAR(model.gnss.fix.track_deg.value, 54.7f, 0.001f);
    REQUIRE(model.gnss.fault_detection.failed_satellite_prn.value == 7);
    REQUIRE(model.route.waypoint_arrival.arrival_circle_entered.value);
}

static void check_ais_vdm_vdo_samples() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "ais_vdm_vdo_samples.nmea");
    REQUIRE(counts.data_lines == 9);
    REQUIRE(counts.accepted_lines == 9);
    REQUIRE(counts.rejected_lines == 0);
    const auto& model = app.store().model();

    REQUIRE(model.ais.own_vessel.valid);
    REQUIRE(model.ais.position_report.mmsi.valid);
    REQUIRE(model.ais.static_voyage.mmsi.valid);
    REQUIRE(model.ais.aid_to_navigation.mmsi.valid);
    REQUIRE(model.ais.channel_management.mmsi.valid);
    REQUIRE(model.ais.binary_envelope.mmsi.valid);
}

static void check_mixed_marine_stream() {
    signalk_mini::SignalKMiniApp<float> app;
    const auto counts = feed_fixture(app, "opencpn_signalk_mixed_stream.nmea");
    REQUIRE(counts.data_lines == 13);
    REQUIRE(counts.accepted_lines == 13);
    REQUIRE(counts.rejected_lines == 0);
    const auto& model = app.store().model();

    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    REQUIRE(model.wind.apparent.speed_kn.valid);
    REQUIRE(model.wind.apparent.direction_deg.valid);
    REQUIRE(model.sea.speed_kn.valid);
    REQUIRE(model.ins.imu.heading_deg.valid);
    REQUIRE(model.route.waypoint.distance_nmi.valid);
    REQUIRE(model.ais.position_report.mmsi.valid);
    NEAR(model.wind.apparent.speed_kn.value, 1.0f, 0.001f);
    NEAR(model.sea.speed_kn.value, 10.5f, 0.001f);
}

int main() {
    check_aerorust_gnss_supported();
    check_tripmate_sample();
    check_gpsd_reference_edges();
    check_ais_vdm_vdo_samples();
    check_mixed_marine_stream();
    return 0;
}
