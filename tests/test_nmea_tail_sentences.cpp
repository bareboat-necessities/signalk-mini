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
    NEAR(app.store().model().gnss.fix.timestamp_s.value, 40977.0f, 0.001f);
    NEAR(app.store().model().gnss.fix.fix_lat_deg.value, 38.737335f, 0.001f);
    NEAR(app.store().model().gnss.fix.fix_lon_deg.value, -9.140638f, 0.001f);

    feed(app, "WIMDA,29.91,I,1.013,B,18.2,C,16.7,C,72.5,12.3,9.4,C,123.4,T,121.1,M,7.8,N,4.0,M", now_us);
    NEAR(app.store().model().env.barometric_pressure_inhg.value, 29.91f, 0.001f);
    NEAR(app.store().model().env.air_temperature_c.value, 18.2f, 0.001f);
    NEAR(app.store().model().sea.temperature_c.value, 16.7f, 0.001f);
    NEAR(app.store().model().wind.surface.speed_kn.value, 7.8f, 0.001f);

    feed(app, "IITFI,0,1,2", now_us);
    REQUIRE(app.store().model().sea.trawl_catch_sensor_status[0].value == 0);
    REQUIRE(app.store().model().sea.trawl_catch_sensor_status[1].value == 1);
    REQUIRE(app.store().model().sea.trawl_catch_sensor_status[2].value == 2);

    feed(app, "RATLL,7,4917.24,N,12309.57,W,TARGET7,123520,T,R", now_us);
    REQUIRE(app.store().model().ais.tracked_target.target_number.value == 7);
    NEAR(app.store().model().ais.tracked_target.latitude_deg.value, 49.287333f, 0.001f);
    NEAR(app.store().model().ais.tracked_target.longitude_deg.value, -123.1595f, 0.001f);

    feed(app, "ALACK,42", now_us);
    REQUIRE(app.store().model().alerting.acknowledgement.field_count.value == 1);
    REQUIRE(app.store().model().alerting.acknowledgement.last_update_us == now_us);

    feed(app, "AIADS,DEV1,A,OK", now_us);
    REQUIRE(app.store().model().equipment.ais_data_status.field_count.value == 3);

    feed(app, "VCCUR,1,123.4,T,2.5,N,10.0,M", now_us);
    NEAR(app.store().model().sea.current_direction_deg.value, 123.4f, 0.001f);
    NEAR(app.store().model().sea.current_speed_kn.value, 2.5f, 0.001f);

    feed(app, "CDDSC,12,3380400790,12,06,00,1423108312,2019,,,S,E", now_us);
    REQUIRE(app.nmea0183().dsc_state().message.format_specifier.value == 12);
    REQUIRE(std::strcmp(app.nmea0183().dsc_state().message.sender_mmsi, "3380400790") == 0);
    NEAR(app.nmea0183().dsc_state().message.latitude_deg, 42.516666f, 0.001f);
    NEAR(app.nmea0183().dsc_state().message.longitude_deg, -83.2f, 0.001f);

    feed(app, "CDDSE,1,1,A,3380400790,00,45894494", now_us);
    REQUIRE(app.nmea0183().dsc_state().expansion.total_messages.value == 1);
    REQUIRE(std::strcmp(app.nmea0183().dsc_state().expansion.payload, "45894494") == 0);

    feed(app, "CDDSI,REQ1,3380400790", now_us);
    REQUIRE(app.store().model().dsc.interrogation.field_count.value == 2);
    feed(app, "CDDSR,RSP1,3380400790", now_us);
    REQUIRE(app.store().model().dsc.response.field_count.value == 2);

    feed(app, "FDFIR,FIRE1,ALARM", now_us);
    REQUIRE(app.store().model().events.fire.field_count.value == 2);

    feed(app, "GPTXT,01,01,02,hello", now_us);
    REQUIRE(app.store().model().messages.text.field_count.value == 4);

    feed(app, "GPWDC,12.3,N,22.7796,K,TO1,FROM1", now_us);
    NEAR(app.store().model().route.waypoint.distance_nmi.value, 12.3f, 0.001f);
    feed(app, "GPWDR,12.4,N,22.9648,K,TO2,FROM2", now_us);
    NEAR(app.store().model().route.waypoint.distance_nmi.value, 12.4f, 0.001f);
    feed(app, "GPZDL,000930,12.3,N,VP1", now_us);
    NEAR(app.store().model().route.waypoint.distance_nmi.value, 12.3f, 0.001f);

    feed(app, "GPZFO,123520,000315,ORIG1", now_us);
    NEAR(app.store().model().route.waypoint.origin_utc_time_s.value, 45320.0f, 0.001f);

    feed(app, "GPZTG,123521,000930,DEST2", now_us);
    NEAR(app.store().model().route.waypoint.destination_utc_time_s.value, 45321.0f, 0.001f);
    return 0;
}
