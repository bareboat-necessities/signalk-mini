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
    char out[220];
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

static const ship_data_model::AisTargetData<float>* find_target(const ship_data_model::AisTargetTableData<float>& table, int32_t mmsi) {
    for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
        if (table.targets[i].occupied && table.targets[i].mmsi.valid && table.targets[i].mmsi.value == mmsi) return &table.targets[i];
    }
    return nullptr;
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed_ais(app, "AIVDO,1,1,,A,11mg=5@P0oreDP8GD@44lRmD0000,0", now_us);

    const auto& own = app.store().model().ais.own_vessel;
    REQUIRE(own.valid == true);
    REQUIRE(own.message_type.value == 1);
    REQUIRE(own.repeat_indicator.value == 0);
    REQUIRE(own.mmsi.value == 123456789);
    REQUIRE(own.navigation_status.value == 0);
    NEAR(own.speed_over_ground_kn.value, 5.5f, 0.001f);
    NEAR(own.longitude_deg.value, -73.9857f, 0.0001f);
    NEAR(own.latitude_deg.value, 40.7484f, 0.0001f);
    NEAR(own.course_over_ground_deg.value, 123.4f, 0.001f);
    NEAR(own.true_heading_deg.value, 90.0f, 0.001f);
    REQUIRE(own.timestamp_s.value == 42);
    REQUIRE(own.last_seen_us == now_us);
    REQUIRE(own.last_position_update_us == now_us);
    REQUIRE(find_target(app.store().model().ais.targets, 123456789) == nullptr);
    REQUIRE(!app.store().model().ais.targets.target_count.valid || app.store().model().ais.targets.target_count.value == 0);
    REQUIRE(app.store().model().ais.tracked_target.target_number.valid == false);

    feed_ais(app, "AIVDO,2,1,7,B,51mg=5@2Fe3t<4hk7;=@E=B1HE=<Dh000000,0", now_us);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == false);
    feed_ais(app, "AIVDO,2,2,7,B,000U1PR347?os;B0DPSlP00000000000000,2", now_us);
    REQUIRE(app.nmea0183().message_state().ais_message.complete == true);

    const auto& updated = app.store().model().ais.own_vessel;
    REQUIRE(updated.valid == true);
    REQUIRE(updated.message_type.value == 5);
    REQUIRE(updated.mmsi.value == 123456789);
    REQUIRE(updated.imo_number.value == 9876543);
    REQUIRE(updated.ship_type.value == 37);
    REQUIRE(std::strcmp(updated.call_sign, "CALL123") == 0);
    REQUIRE(std::strcmp(updated.vessel_name, "TEST VESSEL") == 0);
    REQUIRE(std::strcmp(updated.destination, "HARBOR") == 0);
    REQUIRE(updated.first_seen_us != 0);
    REQUIRE(updated.last_static_update_us == now_us);
    REQUIRE(find_target(app.store().model().ais.targets, 123456789) == nullptr);
    REQUIRE(!app.store().model().ais.targets.target_count.valid || app.store().model().ais.targets.target_count.value == 0);

    return 0;
}
