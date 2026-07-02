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
    char out[160];
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

    feed(app, "GPWCV,6.7,N,DEST1", now_us);
    NEAR(app.store().model().navigation.rmb.closing_velocity_kn.value, 6.7f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.rmb.destination_id, "DEST1") == 0);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "DEST1") == 0);

    feed(app, "GPWNC,12.3,N,22.7796,K,TO1,FROM1", now_us);
    NEAR(app.store().model().navigation.waypoint.distance_nmi.value, 12.3f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "TO1") == 0);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.from_waypoint_id, "FROM1") == 0);

    feed(app, "GPWPL,4917.24,N,12309.57,W,WPLOC", now_us);
    NEAR(app.store().model().navigation.waypoint.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.longitude_deg.value, -123.1595f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "WPLOC") == 0);

    feed(app, "GPXTR,0.08,L,N", now_us);
    NEAR(app.store().model().navigation.apb.xte_nmi.value, -0.08f, 0.0001f);
    NEAR(app.store().model().navigation.rmb.xte_nmi.value, -0.08f, 0.0001f);

    feed(app, "GPZDA,123519.50,01,07,2026,-05,30", now_us);
    NEAR(app.store().model().navigation.gps.timestamp_s.value, 45319.5f, 0.001f);
    REQUIRE(app.store().model().navigation.gps.date_day.value == 1);
    REQUIRE(app.store().model().navigation.gps.date_month.value == 7);
    REQUIRE(app.store().model().navigation.gps.date_year.value == 2026);
    REQUIRE(app.store().model().navigation.gps.local_zone_hours.value == -5);
    REQUIRE(app.store().model().navigation.gps.local_zone_minutes.value == 30);

    feed(app, "GPZFO,123520,000315,ORIG1", now_us);
    NEAR(app.store().model().navigation.waypoint.origin_utc_time_s.value, 45320.0f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.origin_elapsed_time_s.value, 195.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.from_waypoint_id, "ORIG1") == 0);

    feed(app, "GPZTG,123521,000930,DEST2", now_us);
    NEAR(app.store().model().navigation.waypoint.destination_utc_time_s.value, 45321.0f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.destination_time_remaining_s.value, 570.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "DEST2") == 0);
    return 0;
}
