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

    feed(app, "ALACK,42", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().alarm_acknowledgement.sentence_id, "ACK") == 0);
    REQUIRE(std::strcmp(app.nmea0183().state().alarm_acknowledgement.field[0], "42") == 0);

    feed(app, "AIADS,DEV1,A,OK", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().automatic_device_status.field[0], "DEV1") == 0);

    feed(app, "ALAKD,ALM1,A", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().acknowledge_detail_alarm.field[0], "ALM1") == 0);

    feed(app, "ALALA,ALM2,SET", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().set_detail_alarm.field[1], "SET") == 0);

    feed(app, "APASD,MODE1,DATA", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().autopilot_system.field[0], "MODE1") == 0);

    feed(app, "GPBEC,112233,4917.24,N,12309.57,W,054.7,T,034.4,M,10.5,N,WPDR", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().bearing_distance_dead_reckoning.sentence_id, "BEC") == 0);

    feed(app, "CECEK,KEY1,ACTIVE", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().encryption_key_command.field[0], "KEY1") == 0);

    feed(app, "ECCOP,PERIOD,10", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().operational_period_command.field[1], "10") == 0);

    feed(app, "VCCUR,1,123.4,T,2.5,N,10.0,M", now_us);
    REQUIRE(app.nmea0183().state().current_layer.layer_number.value == 1);
    NEAR(app.nmea0183().state().current_layer.current_direction_deg, 123.4f, 0.001f);
    NEAR(app.nmea0183().state().current_layer.current_speed_kn, 2.5f, 0.001f);
    NEAR(app.nmea0183().state().current_layer.layer_depth_m, 10.0f, 0.001f);

    feed(app, "ECDCR,CAP,VALUE", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().device_capability_report.field[0], "CAP") == 0);

    feed(app, "INDDC,50,A", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().display_dimming_control.field[0], "50") == 0);

    feed(app, "FDDOR,DOOR1,OPEN", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().door_status.field[1], "OPEN") == 0);

    feed(app, "CDDSC,12,3380400790,12,06,00,1423108312,2019,,,S,E", now_us);
    REQUIRE(app.nmea0183().state().dsc.message.format_specifier.value == 12);
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.message.sender_mmsi, "3380400790") == 0);
    REQUIRE(app.nmea0183().state().dsc.message.category.value == 12);
    REQUIRE(app.nmea0183().state().dsc.message.nature_or_first_telecommand.value == 6);
    NEAR(app.nmea0183().state().dsc.message.latitude_deg, 42.516666f, 0.001f);
    NEAR(app.nmea0183().state().dsc.message.longitude_deg, -83.2f, 0.001f);
    NEAR(app.nmea0183().state().dsc.message.utc_time_s, 73140.0f, 0.001f);
    REQUIRE(app.nmea0183().state().dsc.message.end_of_sequence == 'S');
    REQUIRE(app.nmea0183().state().dsc.message.expansion_flag == 'E');

    feed(app, "CDDSE,1,1,A,3380400790,00,45894494", now_us);
    REQUIRE(app.nmea0183().state().dsc.expansion.total_messages.value == 1);
    REQUIRE(app.nmea0183().state().dsc.expansion.message_number.value == 1);
    REQUIRE(app.nmea0183().state().dsc.expansion.query_flag == 'A');
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.expansion.sender_mmsi, "3380400790") == 0);
    REQUIRE(app.nmea0183().state().dsc.expansion.expansion_specifier.value == 0);
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.expansion.payload, "45894494") == 0);

    feed(app, "CDDSI,3380400790,20", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.initiate.sentence_id, "DSI") == 0);

    feed(app, "CDDSR,3380400790,ACK", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().dsc.response.field[1], "ACK") == 0);

    feed(app, "ERETL,PORT,ASTERN", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().engine_telegraph.field[0], "PORT") == 0);

    feed(app, "ECEVE,EVT1,INFO", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().event_message.field[0], "EVT1") == 0);

    feed(app, "FDFIR,FIRE1,ALARM", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().fire_detection.field[1], "ALARM") == 0);

    feed(app, "GPWDC,12.3,N,22.7796,K,TO1,FROM1", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().waypoint_distance_great_circle.sentence_id, "WDC") == 0);

    feed(app, "GPWDR,12.4,N,22.9648,K,TO2,FROM2", now_us);
    REQUIRE(std::strcmp(app.nmea0183().state().waypoint_distance_rhumb.field[4], "FROM2") == 0);

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

    feed(app, "GPZDL,000930,12.3,N,VP1", now_us);
    NEAR(app.nmea0183().state().variable_point.time_to_point_s, 570.0f, 0.001f);
    NEAR(app.nmea0183().state().variable_point.distance_to_point_nmi, 12.3f, 0.001f);
    REQUIRE(std::strcmp(app.nmea0183().state().variable_point.point_id, "VP1") == 0);

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
