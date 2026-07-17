#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static std::string sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

static void feed_gga_for_talker(signalk_mini::SignalKMiniApp<float>& app, const char* talker, uint64_t& now_us) {
    char body[128];
    std::snprintf(body, sizeof(body), "%sGGA,123519,4807.038,N,01131.000,E,1,08,0.9,10.0,M,46.9,M,,", talker);
    feed(app, body, now_us);
    const auto& fix = app.store().model().gnss.fix;
    REQUIRE(std::strcmp(fix.talker_id, talker) == 0);
    REQUIRE(fix.fix_valid.valid && fix.fix_valid.value);
    NEAR(fix.fix_lat_deg.value, 48.1173f, 0.001f);
    NEAR(fix.fix_lon_deg.value, 11.516666f, 0.001f);
    NEAR(fix.fix_alt_msl_m.value, 10.0f, 0.001f);
    NEAR(fix.geoidal_separation_m.value, 46.9f, 0.001f);
    NEAR(fix.fix_alt_hae_m.value, 56.9f, 0.001f);
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "GNGNS,112257.00,3844.24011,N,00908.43828,W,AN,03,10.5,100.1,45.6,2.5,1234,S,1,5", now_us);
    const auto& gns_fix = app.store().model().gnss.fix;
    REQUIRE(std::strcmp(gns_fix.talker_id, "GN") == 0);
    REQUIRE(gns_fix.navigational_status == 'S');
    REQUIRE(gns_fix.system_id.value == 1);
    REQUIRE(gns_fix.signal_id.value == 5);
    REQUIRE(gns_fix.fix_valid.valid && gns_fix.fix_valid.value);
    NEAR(gns_fix.fix_lat_deg.value, 38.737335f, 0.001f);
    NEAR(gns_fix.fix_lon_deg.value, -9.140638f, 0.001f);
    NEAR(gns_fix.fix_alt_msl_m.value, 100.1f, 0.001f);
    NEAR(gns_fix.geoidal_separation_m.value, 45.6f, 0.001f);
    NEAR(gns_fix.fix_alt_hae_m.value, 145.7f, 0.001f);

    feed(app, "GNGBS,123519,1.1,2.2,3.3,07,0.1,4.4,5.5,1,7", now_us);
    REQUIRE(std::strcmp(app.store().model().gnss.fault_detection.talker_id, "GN") == 0);
    NEAR(app.store().model().gnss.fault_detection.expected_error_lat_m.value, 1.1f, 0.001f);
    REQUIRE(app.store().model().gnss.fault_detection.system_id.value == 1);
    REQUIRE(app.store().model().gnss.fault_detection.signal_id.value == 7);

    feed(app, "GNGFA,123519,A,A,0.9,1.8,2.0,1.0,33.0,1.2,0.8,1.5,1,7", now_us);
    REQUIRE(std::strcmp(app.store().model().gnss.fix_accuracy.talker_id, "GN") == 0);
    REQUIRE(app.store().model().gnss.fix_accuracy.mode_indicator == 'A');
    REQUIRE(app.store().model().gnss.fix_accuracy.status == 'A');
    NEAR(app.store().model().gnss.fix_accuracy.horizontal_accuracy_m.value, 0.9f, 0.001f);
    NEAR(app.store().model().gnss.fix_accuracy.vertical_accuracy_m.value, 1.8f, 0.001f);
    REQUIRE(app.store().model().gnss.fix_accuracy.system_id.value == 1);
    REQUIRE(app.store().model().gnss.fix_accuracy.signal_id.value == 7);

    feed(app, "GNGSA,A,3,04,05,09,12,24,25,29,31,,,,,1.8,1.0,1.5,1,7", now_us);
    REQUIRE(std::strcmp(app.store().model().gnss.dop_active_satellites.talker_id, "GN") == 0);
    REQUIRE(app.store().model().gnss.dop_active_satellites.system_id.value == 1);
    REQUIRE(app.store().model().gnss.dop_active_satellites.signal_id.value == 7);
    REQUIRE(app.store().model().gnss.fix.fix_type.valid);
    REQUIRE(app.store().model().gnss.fix.fix_type.value == 3);
    REQUIRE(app.store().model().gnss.fix.fix_valid.valid && app.store().model().gnss.fix.fix_valid.value);
    NEAR(app.store().model().gnss.dop_active_satellites.hdop.value, 1.0f, 0.001f);

    auto& sky = app.store().model().gnss.sky_view;
    const uint32_t initial_sequence = sky.sequence;
    feed(app, "GPGSV,2,1,08,04,40,083,45,05,17,123,40,09,11,233,30,12,09,300,28,1,1", now_us);
    REQUIRE(!sky.complete);
    REQUIRE(sky.sequence == initial_sequence);
    REQUIRE(sky.observation_count == 4);

    feed(app, "GPGSV,2,2,08,24,60,010,50,25,50,020,48,29,40,030,46,31,30,040,44,1,1", now_us);
    REQUIRE(std::strcmp(app.store().model().gnss.satellites_in_view.talker_id, "GP") == 0);
    REQUIRE(app.store().model().gnss.satellites_in_view.signal_id.value == 1);
    REQUIRE(app.store().model().gnss.satellites_in_view.system_id.value == 1);
    REQUIRE(app.store().model().gnss.satellites_in_view.satellite_prn[3].value == 31);
    REQUIRE(sky.complete);
    REQUIRE(sky.sequence == initial_sequence + 1);
    REQUIRE(sky.capacity() >= 8);
    REQUIRE(sky.observation_count == 8);
    REQUIRE(sky.satellites_in_view.value == 8);
    REQUIRE(sky.satellites_used.value == 8);
    REQUIRE(sky.observations[0].satellite_id_valid);
    REQUIRE(sky.observations[0].satellite_id == 4);
    REQUIRE(sky.observations[0].used);
    NEAR(sky.observations[0].elevation_deg, 40.0f, 0.001f);
    NEAR(sky.observations[0].azimuth_true_deg, 83.0f, 0.001f);
    NEAR(sky.observations[0].cn0_db_hz, 45.0f, 0.001f);
    REQUIRE(sky.observations[7].satellite_id == 31);
    REQUIRE(sky.observations[7].system_id == 1);
    REQUIRE(sky.observations[7].signal_id == 1);

    feed_gga_for_talker(app, "GP", now_us);
    feed_gga_for_talker(app, "GL", now_us);
    feed_gga_for_talker(app, "GA", now_us);
    feed_gga_for_talker(app, "GB", now_us);
    feed_gga_for_talker(app, "BD", now_us);
    feed_gga_for_talker(app, "GQ", now_us);
    feed_gga_for_talker(app, "GI", now_us);
    feed_gga_for_talker(app, "GN", now_us);

    feed(app, "GNDTM,W84,,0.10,N,0.20,E,1.5,W84", now_us);
    REQUIRE(std::strcmp(app.store().model().nav.datum.local_datum_code, "W84") == 0);
    REQUIRE(std::strcmp(app.store().model().nav.datum.reference_datum_code, "W84") == 0);
    NEAR(app.store().model().nav.datum.latitude_offset_min.value, 0.10f, 0.001f);
    NEAR(app.store().model().nav.datum.longitude_offset_min.value, 0.20f, 0.001f);
    NEAR(app.store().model().nav.datum.altitude_offset_m.value, 1.5f, 0.001f);

    return 0;
}
