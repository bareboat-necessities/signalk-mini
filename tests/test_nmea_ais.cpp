#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static std::string encapsulated_sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "!%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed_ais(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = encapsulated_sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed_ais(app, "AIVDM,1,1,,A,11mg=5@P0oreDP8GD@44lRmD0000,0", now_us);

    const auto& pos = app.store().model().ais.position_report;
    REQUIRE(pos.message_type.value == 1);
    REQUIRE(pos.repeat_indicator.value == 0);
    REQUIRE(pos.mmsi.value == 123456789);
    REQUIRE(pos.navigation_status.value == 0);
    NEAR(pos.speed_over_ground_kn.value, 5.5f, 0.001f);
    NEAR(pos.longitude_deg.value, -73.9857f, 0.0001f);
    NEAR(pos.latitude_deg.value, 40.7484f, 0.0001f);
    NEAR(pos.course_over_ground_deg.value, 123.4f, 0.001f);
    NEAR(pos.true_heading_deg.value, 90.0f, 0.001f);
    REQUIRE(pos.timestamp_s.value == 42);
    REQUIRE(app.store().model().ais.tracked_target.target_number.value == 123456789);
    NEAR(app.store().model().ais.tracked_target.latitude_deg.value, 40.7484f, 0.0001f);

    feed_ais(app, "AIVDM,2,1,7,B,51mg=5@2Fe3t<4hk7;=@E=B1HE=<Dh000000,0", now_us);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == false);
    feed_ais(app, "AIVDM,2,2,7,B,000U1PR347?os;B0DPSlP00000000000000,2", now_us);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == true);

    const auto& stat = app.store().model().ais.static_voyage;
    REQUIRE(stat.message_type.value == 5);
    REQUIRE(stat.repeat_indicator.value == 0);
    REQUIRE(stat.mmsi.value == 123456789);
    REQUIRE(stat.imo_number.value == 9876543);
    REQUIRE(std::strcmp(stat.call_sign, "CALL123") == 0);
    REQUIRE(std::strcmp(stat.vessel_name, "TEST VESSEL") == 0);
    REQUIRE(stat.ship_type.value == 37);
    REQUIRE(stat.dimension_to_bow_m.value == 12);
    REQUIRE(stat.dimension_to_stern_m.value == 34);
    REQUIRE(stat.dimension_to_port_m.value == 3);
    REQUIRE(stat.dimension_to_starboard_m.value == 4);
    REQUIRE(stat.epfd_type.value == 1);
    REQUIRE(stat.eta_month.value == 12);
    REQUIRE(stat.eta_day.value == 31);
    REQUIRE(stat.eta_hour.value == 23);
    REQUIRE(stat.eta_minute.value == 59);
    NEAR(stat.draught_m.value, 4.5f, 0.001f);
    REQUIRE(std::strcmp(stat.destination, "HARBOR") == 0);
    REQUIRE(stat.dte_ready == true);
    REQUIRE(std::strcmp(app.store().model().ais.tracked_target.target_name, "TEST VESSEL") == 0);

    return 0;
}
