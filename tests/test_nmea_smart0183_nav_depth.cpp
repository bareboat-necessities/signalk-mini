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
    char out[256];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 1000;

    REQUIRE(app.nmea0183().feed_line(sentence("GPAAM,A,A,0.10,N,WP001").c_str(), 1, now_us += 1000));
    REQUIRE(app.store().model().navigation.waypoint_arrival.arrival_circle_entered.value);
    REQUIRE(app.store().model().navigation.waypoint_arrival.perpendicular_passed.value);
    NEAR(app.store().model().navigation.waypoint_arrival.arrival_radius_nmi.value, 0.10f, 0.0001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint_arrival.waypoint_id, "WP001") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("GPALM,1,1,12,2045,0,0.1,200.0,0.2,0.3,5153.5,0.4,0.5,0.6,0.7,0.8").c_str(), 1, now_us += 1000));
    REQUIRE(app.store().model().navigation.gps_almanac.satellite_prn.value == 12);
    REQUIRE(app.store().model().navigation.gps_almanac.gps_week.value == 2045);
    NEAR(app.store().model().navigation.gps_almanac.sqrt_semi_major_axis.value, 5153.5f, 0.001f);

    REQUIRE(app.nmea0183().feed_line(sentence("GPAPA,A,A,0.05,R,N,A,V,123.4,T,WP002").c_str(), 1, now_us += 1000));
    NEAR(app.store().model().navigation.apb.xte_nmi.value, 0.05f, 0.0001f);
    REQUIRE(app.store().model().navigation.apb.arrival_circle_entered.value);
    REQUIRE(!app.store().model().navigation.apb.perpendicular_passed.value);
    NEAR(app.store().model().navigation.apb.origin_to_destination_bearing_deg.value, 123.4f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.apb.destination_id, "WP002") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("GPAPB,A,A,0.02,L,N,A,A,100.0,T,WP003,110.0,T,120.0,T").c_str(), 1, now_us += 1000));
    NEAR(app.store().model().navigation.apb.xte_nmi.value, -0.02f, 0.0001f);
    REQUIRE(app.store().model().navigation.apb.arrival_circle_entered.value);
    REQUIRE(app.store().model().navigation.apb.perpendicular_passed.value);
    NEAR(app.store().model().navigation.apb.origin_to_destination_bearing_deg.value, 100.0f, 0.001f);
    NEAR(app.store().model().navigation.apb.present_to_destination_bearing_deg.value, 110.0f, 0.001f);
    NEAR(app.store().model().navigation.apb.heading_to_steer_deg.value, 120.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.apb.destination_id, "WP003") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("GPBOD,100.0,T,101.0,M,TO,FROM").c_str(), 1, now_us += 1000));
    NEAR(app.store().model().navigation.waypoint.bearing_true_deg.value, 100.0f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.bearing_magnetic_deg.value, 101.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "TO") == 0);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.from_waypoint_id, "FROM") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("GPBWC,225444,4917.24,N,12309.57,W,054.7,T,034.4,M,10.5,N,WP004").c_str(), 1, now_us += 1000));
    NEAR(app.store().model().navigation.waypoint.utc_time_s.value, 82484.0f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.longitude_deg.value, -123.1595f, 0.001f);
    NEAR(app.store().model().navigation.waypoint.distance_nmi.value, 10.5f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "WP004") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("SDDBK,32.8,f,10.0,M,5.5,F").c_str(), 1, now_us += 1000));
    REQUIRE(app.store().model().water.depth_below_keel_m.valid);
    NEAR(app.store().model().water.depth_below_keel_m.value, 10.0f, 0.001f);

    REQUIRE(app.nmea0183().feed_line(sentence("SDDBS,19.7,f,6.0,M,3.3,F").c_str(), 1, now_us += 1000));
    REQUIRE(app.store().model().water.depth_below_surface_m.valid);
    NEAR(app.store().model().water.depth_below_surface_m.value, 6.0f, 0.001f);

    return 0;
}
