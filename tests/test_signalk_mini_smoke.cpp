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
    std::snprintf(out, sizeof(out), "$%s*%c%c", body, nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void test_first_smart0183_group(signalk_mini::SignalKMiniApp<float>& app, uint64_t& now_us) {
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

    REQUIRE(app.nmea0183().feed_line(sentence("GPBWR,225445,4917.25,N,12309.58,W,055.7,T,035.4,M,11.5,N,WP005").c_str(), 1, now_us += 1000));
    NEAR(app.store().model().navigation.waypoint.distance_nmi.value, 11.5f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "WP005") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("GPBWW,200.0,T,201.0,M,TO2,FROM2").c_str(), 1, now_us += 1000));
    NEAR(app.store().model().navigation.waypoint.bearing_true_deg.value, 200.0f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.waypoint.to_waypoint_id, "TO2") == 0);

    REQUIRE(app.nmea0183().feed_line(sentence("SDDBK,32.8,f,10.0,M,5.5,F").c_str(), 1, now_us += 1000));
    REQUIRE(app.store().model().water.depth_below_keel_m.valid);
    NEAR(app.store().model().water.depth_below_keel_m.value, 10.0f, 0.001f);

    REQUIRE(app.nmea0183().feed_line(sentence("SDDBS,19.7,f,6.0,M,3.3,F").c_str(), 1, now_us += 1000));
    REQUIRE(app.store().model().water.depth_below_surface_m.valid);
    NEAR(app.store().model().water.depth_below_surface_m.value, 6.0f, 0.001f);
}

int main() {
    using Real = float;

    signalk_mini::SignalKMiniConfig default_config;
    REQUIRE(default_config.connector_count == 1);
    REQUIRE(default_config.connectors != nullptr);
    REQUIRE(default_config.connectors[0].enabled);
    REQUIRE(default_config.connectors[0].protocol.kind == signalk_mini::ConnectorProtocol::Nmea0183);
    REQUIRE(default_config.connectors[0].transport.kind == signalk_mini::ConnectorTransport::Udp);
    REQUIRE(std::strcmp(default_config.connectors[0].transport.udp.listen_host, "0.0.0.0") == 0);
    REQUIRE(default_config.connectors[0].transport.udp.listen_port == 10110);
    REQUIRE(default_config.connectors[0].protocol.nmea0183.validate_checksum == false);

    signalk_mini::SignalKMiniApp<Real> app;
    const char* bodies[] = {
        "GPRMC,142533.00,A,4042.6142,N,07400.4168,W,5.4,083.2,010726,13.1,W,A",
        "HCHDT,081.7,T",
        "IIMWV,045.0,R,12.8,N,A",
        "SDDPT,5.6,0.4"
    };
    uint64_t now_us = 0;
    for (const char* body : bodies) {
        const std::string line = sentence(body);
        now_us += 100000;
        REQUIRE(app.nmea0183().feed_line(line.c_str(), 7, now_us));
    }
    const auto& model = app.store().model();
    REQUIRE(model.navigation.gps.fix_lat_deg.valid);
    REQUIRE(model.navigation.gps.fix_lon_deg.valid);
    REQUIRE(model.navigation.gps.speed_kn.valid);
    REQUIRE(model.imu.heading_deg.valid);
    REQUIRE(model.wind.apparent.speed_kn.valid);
    REQUIRE(model.water.depth_m.valid);
    NEAR(model.navigation.gps.fix_lat_deg.value, 40.7102367f, 0.0002f);
    NEAR(model.navigation.gps.fix_lon_deg.value, -74.0069467f, 0.0002f);
    NEAR(model.navigation.gps.speed_kn.value, 5.4f, 0.001f);
    NEAR(model.wind.apparent.speed_kn.value, 12.8f, 0.001f);
    NEAR(model.water.depth_m.value, 5.6f, 0.001f);

    test_first_smart0183_group(app, now_us);

    signalk_mini::SignalKMapper<Real> mapper;
    signalk_mini::ModelChange change;
    bool saw_wind_speed = false;
    while (app.store().changes().pop(change)) {
        signalk_mini::SignalKMappedValue<Real> mapped;
        if (mapper.map_change(app.store().model(), change, mapped) && mapped.path && std::strcmp(mapped.path, "environment.wind.speedApparent") == 0) {
            saw_wind_speed = true;
            NEAR(mapped.number, 6.5848889f, 0.001f);
        }
    }
    REQUIRE(saw_wind_speed);
    return 0;
}
