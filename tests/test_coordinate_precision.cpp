#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>
#include "ubx_test_helpers.hpp"

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.12f expected %.12f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

// Float resolves only ~4e-6 deg near 48 deg and ~8e-6 deg near 74 deg, so every
// check below uses a tolerance that a float-backed coordinate cannot meet.
static constexpr double CoordinateTolerance = 1e-9;

static std::string sentence(char start, const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "%c%s*%c%c", start, body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, char start, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(start, body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static void test_coordinate_storage_type() {
    using Fix = decltype(ship_data_model::DataModel<float>{}.gnss.fix);
    static_assert(sizeof(Fix{}.fix_lat_deg.value) == sizeof(double), "latitude must be stored as double");
    static_assert(sizeof(Fix{}.fix_lon_deg.value) == sizeof(double), "longitude must be stored as double");
}

static void test_nmea_gga_keeps_high_resolution_minutes() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;
    feed(app, '$', "GPGGA,123519,4807.0381234,N,01131.0001234,W,4,12,0.6,10.0,M,46.9,M,,", now_us);

    const auto& fix = app.store().model().gnss.fix;
    NEAR(fix.fix_lat_deg.value, 48.0 + 7.0381234 / 60.0, CoordinateTolerance);
    NEAR(fix.fix_lon_deg.value, -(11.0 + 31.0001234 / 60.0), CoordinateTolerance);
}

static void test_ubx_nav_pvt_keeps_1e7_degree_resolution() {
    signalk_mini::ModelStore<float> store;
    signalk_mini::UbxInput<float> input(store);
    const auto pvt = ubx_test::nav_pvt(true);
    REQUIRE(input.feed_octets(pvt.data(), pvt.size(), 10, 1000));

    const auto& fix = store.model().gnss.fix;
    NEAR(fix.fix_lon_deg.value, -74.1234567, CoordinateTolerance);
    NEAR(fix.fix_lat_deg.value, 40.7654321, CoordinateTolerance);
}

static void test_ais_position_is_exact_1_10000_minute() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;
    feed(app, '!', "AIVDM,1,1,,A,11mg=5@P0oreDP8GD@44lRmD0000,0", now_us);

    const auto& pos = app.store().model().ais.position_report;
    REQUIRE(pos.longitude_deg.valid && pos.latitude_deg.valid);
    // AIS encodes 1/10000 minute integers, so the decoded degrees are exact multiples of 1/600000.
    const double lon_raw = pos.longitude_deg.value * 600000.0;
    const double lat_raw = pos.latitude_deg.value * 600000.0;
    NEAR(lon_raw, std::round(lon_raw), 1e-6);
    NEAR(lat_raw, std::round(lat_raw), 1e-6);
    NEAR(pos.longitude_deg.value, -73.9857, 1e-4);
    NEAR(pos.latitude_deg.value, 40.7484, 1e-4);
}

static void test_signalk_output_keeps_precision() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;
    feed(app, '$', "GPGGA,123519,4807.0381234,N,01131.0001234,W,4,12,0.6,10.0,M,46.9,M,,", now_us);
    feed(app, '$', "GPWPL,4807.0381234,N,01131.0001234,W,WP1", now_us);

    std::vector<char> json(64 * 1024);
    const int len = app.write_full_model(json.data(), json.size());
    REQUIRE(len > 0);
    const std::string text(json.data(), static_cast<size_t>(len));
    // 48 + 7.0381234/60 = 48.11730205666...; a float would serialize as 48.1173019 or 48.117302.
    REQUIRE(text.find("48.11730205") != std::string::npos);
    REQUIRE(text.find("-11.51666872") != std::string::npos);

    signalk_mini::SignalKMapper<float> mapper;
    signalk_mini::SignalKMappedValue<float> mapped;
    signalk_mini::ModelChange change;
    change.field = signalk_mini::ModelField::RouteWaypointLatDeg;
    REQUIRE(mapper.map_change(app.store().model(), change, mapped));
    REQUIRE(mapped.has_coordinate);
    NEAR(mapped.coordinate, 48.0 + 7.0381234 / 60.0, CoordinateTolerance);

    char value[64];
    signalk_mini::SignalKJsonStreamWriter out(value, sizeof(value));
    REQUIRE(signalk_mini::signalk_json_write_mapped_scalar(out, mapped));
    REQUIRE(std::string(value, out.size()).find("48.11730205") == 0);
}

static void test_rmc_output_round_trips_longitude() {
    ship_data_model::DataModel<float> model;
    model.gnss.fix.fix_lat_deg.set(-33.8567844, 1);
    model.gnss.fix.fix_lon_deg.set(5.2833333, 1);
    model.gnss.fix.speed_kn.set(1.0f, 1);
    model.gnss.fix.track_deg.set(2.0f, 1);

    char out[128];
    REQUIRE(nmea0183_connector::make_ap_rmc(out, sizeof(out), model, "123519", "230394") > 0);
    // Longitude uses three degree digits (dddmm.mmmm) and latitude two (ddmm.mmmm).
    REQUIRE(std::strstr(out, ",3351.4071,S,00517.0000,E,") != nullptr);

    char body[128];
    const char* star = std::strchr(out, '*');
    REQUIRE(star != nullptr);
    std::snprintf(body, sizeof(body), "%.*s", static_cast<int>(star - out - 1), out + 1);
    const std::string replayed = std::string("$") + body + std::string(star, 3);

    signalk_mini::SignalKMiniApp<float> app;
    REQUIRE(app.nmea0183().feed_line(replayed.c_str(), 1, 1000));
    NEAR(app.store().model().gnss.fix.fix_lat_deg.value, -33.8567844, 1e-5 / 6.0);
    NEAR(app.store().model().gnss.fix.fix_lon_deg.value, 5.2833333, 1e-5 / 6.0);

    model.gnss.fix.fix_lon_deg.set(-179.9999999, 1);
    REQUIRE(nmea0183_connector::make_ap_rmc(out, sizeof(out), model, "123519", "230394") > 0);
    REQUIRE(std::strstr(out, ",18000.0000,W,") != nullptr);
}

int main() {
    test_coordinate_storage_type();
    test_nmea_gga_keeps_high_resolution_minutes();
    test_ubx_nav_pvt_keeps_1e7_degree_resolution();
    test_ais_position_is_exact_1_10000_minute();
    test_signalk_output_keeps_precision();
    test_rmc_output_round_trips_longitude();
    std::puts("test_coordinate_precision passed");
    return 0;
}
