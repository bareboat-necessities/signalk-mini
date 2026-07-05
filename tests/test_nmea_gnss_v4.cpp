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

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "GNGNS,112257.00,3844.24011,N,00908.43828,W,AN,03,10.5,100.1,45.6,2.5,1234,S,1,5", now_us);
    REQUIRE(std::strcmp(app.store().model().gnss.fix.talker_id, "GN") == 0);
    REQUIRE(app.store().model().gnss.fix.navigational_status == 'S');
    REQUIRE(app.store().model().gnss.fix.system_id.value == 1);
    REQUIRE(app.store().model().gnss.fix.signal_id.value == 5);
    NEAR(app.store().model().gnss.fix.fix_lat_deg.value, 38.737335f, 0.001f);
    NEAR(app.store().model().gnss.fix.fix_lon_deg.value, -9.140638f, 0.001f);

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
    NEAR(app.store().model().gnss.dop_active_satellites.hdop.value, 1.0f, 0.001f);

    feed(app, "GBGSV,1,1,04,01,40,083,45,02,17,123,40,03,11,233,30,04,09,300,28,6,3", now_us);
    REQUIRE(std::strcmp(app.store().model().gnss.satellites_in_view.talker_id, "GB") == 0);
    REQUIRE(app.store().model().gnss.satellites_in_view.signal_id.value == 6);
    REQUIRE(app.store().model().gnss.satellites_in_view.system_id.value == 3);
    REQUIRE(app.store().model().gnss.satellites_in_view.satellite_prn[3].value == 4);

    return 0;
}
