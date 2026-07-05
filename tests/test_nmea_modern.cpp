#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d\n", __FILE__, __LINE__); std::exit(2); } } while (0)

static std::string sentence(const std::string& body) {
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body.c_str());
    std::string out = "$";
    out += body;
    out += "*";
    out.push_back(nmea0183_connector::to_hex((cs >> 4) & 0x0f));
    out.push_back(nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const std::string& body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "IIHBT,A,30,HB01", now_us);
    const auto& hbt = app.store().model().notifications.alerts.heartbeat;
    REQUIRE(hbt.status == 'A');
    NEAR(hbt.interval_s.value, 30.0f, 0.001f);
    REQUIRE(std::strcmp(hbt.sequential_message_id, "HB01") == 0);

    feed(app, "IISMV,SM01,123456789,123519,4807.038,N,01131.000,E,TYPE1,CAP1,RT01,A,TEXT4", now_us);
    const auto& smv = app.store().model().notifications.special.smv;
    REQUIRE(std::strcmp(smv.message_id, "SM01") == 0);
    REQUIRE(smv.mmsi.value == 123456789);
    NEAR(smv.utc_time_s.value, 45319.0f, 0.001f);
    NEAR(smv.latitude_deg.value, 48.1173f, 0.001f);
    NEAR(smv.longitude_deg.value, 11.516666f, 0.001f);
    REQUIRE(std::strcmp(smv.event_type, "TYPE1") == 0);
    REQUIRE(std::strcmp(smv.sar_capability, "CAP1") == 0);
    REQUIRE(std::strcmp(smv.route_id, "RT01") == 0);
    REQUIRE(smv.status == 'A');
    REQUIRE(std::strcmp(smv.description, "TEXT4") == 0);

    return 0;
}
