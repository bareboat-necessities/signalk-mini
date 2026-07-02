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

    feed(app, "GNGNS,112257.00,3844.24011,N,00908.43828,W,AN,03,10.5,100.1,45.6,2.5,1234,S", now_us);
    NEAR(app.store().model().navigation.gps.timestamp_s.value, 40977.0f, 0.001f);
    NEAR(app.store().model().navigation.gps.fix_lat_deg.value, 38.737335f, 0.001f);
    NEAR(app.store().model().navigation.gps.fix_lon_deg.value, 9.140638f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.gps.fix_mode_indicator, "AN") == 0);
    REQUIRE(app.store().model().navigation.gps.satellites_used.value == 3);
    NEAR(app.store().model().navigation.gps.hdop.value, 10.5f, 0.001f);
    NEAR(app.store().model().navigation.gps.fix_alt_m.value, 100.1f, 0.001f);
    NEAR(app.store().model().navigation.gps.geoidal_separation_m.value, 45.6f, 0.001f);
    NEAR(app.store().model().navigation.gps.dgps_age_s.value, 2.5f, 0.001f);
    REQUIRE(app.store().model().navigation.gps.dgps_station_id.value == 1234);
    REQUIRE(app.store().model().navigation.gps.navigational_status == 'S');

    feed(app, "WIMDA,29.91,I,1.013,B,18.2,C,16.7,C,72.5,12.3,9.4,C,123.4,T,121.1,M,7.8,N,4.0,M", now_us);
    NEAR(app.store().model().water.barometric_pressure_inhg.value, 29.91f, 0.001f);
    NEAR(app.store().model().water.barometric_pressure_bar.value, 1.013f, 0.001f);
    NEAR(app.store().model().water.air_temperature_c.value, 18.2f, 0.001f);
    NEAR(app.store().model().water.temperature_c.value, 16.7f, 0.001f);
    NEAR(app.store().model().water.relative_humidity_percent.value, 72.5f, 0.001f);
    NEAR(app.store().model().water.dew_point_c.value, 9.4f, 0.001f);
    NEAR(app.store().model().water.wind_direction_deg.value, 123.4f, 0.001f);
    NEAR(app.store().model().water.wind_direction_magnetic_deg.value, 121.1f, 0.001f);
    NEAR(app.store().model().water.wind_speed_kn.value, 7.8f, 0.001f);
    NEAR(app.store().model().water.wind_speed_m_s.value, 4.0f, 0.001f);

    feed(app, "GPRLM,ABCDEF123456789,123519,4,HELLO", now_us);
    REQUIRE(std::strcmp(app.store().model().navigation.return_link_message.beacon_id, "ABCDEF123456789") == 0);
    NEAR(app.store().model().navigation.return_link_message.reception_time_s.value, 45319.0f, 0.001f);
    REQUIRE(app.store().model().navigation.return_link_message.message_code == '4');
    REQUIRE(std::strcmp(app.store().model().navigation.return_link_message.message_body, "HELLO") == 0);

    feed(app, "IITFI,0,1,2", now_us);
    REQUIRE(app.store().model().water.trawl_catch_sensor_status[0].value == 0);
    REQUIRE(app.store().model().water.trawl_catch_sensor_status[1].value == 1);
    REQUIRE(app.store().model().water.trawl_catch_sensor_status[2].value == 2);

    feed(app, "RATLB,7,TGT7,8,TGT8", now_us);
    REQUIRE(app.store().model().navigation.tracked_target.label_count.value == 2);
    REQUIRE(app.store().model().navigation.tracked_target.label_target_number[0].value == 7);
    REQUIRE(std::strcmp(app.store().model().navigation.tracked_target.label[1], "TGT8") == 0);

    feed(app, "RATLL,7,4917.24,N,12309.57,W,TARGET7,123520,T,R", now_us);
    REQUIRE(app.store().model().navigation.tracked_target.target_number.value == 7);
    NEAR(app.store().model().navigation.tracked_target.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().navigation.tracked_target.longitude_deg.value, -123.1595f, 0.001f);
    REQUIRE(std::strcmp(app.store().model().navigation.tracked_target.target_name, "TARGET7") == 0);
    NEAR(app.store().model().navigation.tracked_target.utc_time_s.value, 45320.0f, 0.001f);
    REQUIRE(app.store().model().navigation.tracked_target.target_status == 'T');
    REQUIRE(app.store().model().navigation.tracked_target.reference_target == 'R');

    feed(app, "IITPC,1.2,M,33.4,M,56.7,M", now_us);
    NEAR(app.store().model().water.trawl_cartesian_centerline_offset_m.value, 1.2f, 0.001f);
    NEAR(app.store().model().water.trawl_cartesian_along_centerline_m.value, 33.4f, 0.001f);
    NEAR(app.store().model().water.trawl_depth_below_surface_m.value, 56.7f, 0.001f);

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
