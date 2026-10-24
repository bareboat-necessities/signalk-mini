#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <ArduinoJson.h>
#include <nmea0183_connector.hpp>
#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

namespace {

constexpr signalk_mini::SourceId GpsA = signalk_mini::FirstConnectorSourceId;      // "connector0"
constexpr signalk_mini::SourceId GpsB = signalk_mini::FirstConnectorSourceId + 1;  // "connector1"

std::string sentence(const char* body) {
    char out[160];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

void feed(signalk_mini::SignalKMiniApp<float>& app, signalk_mini::SourceId source, const char* body, uint64_t& now_us) {
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), source, now_us += 1000, false));
}

struct CollectingSink {
    std::vector<std::string> deltas;
    void write_signal_k_delta(const char* json, size_t len) { deltas.emplace_back(json, len); }
};

JsonDocument full_model(signalk_mini::SignalKMiniApp<float>& app) {
    std::vector<char> buffer(64 * 1024);
    const int len = app.write_full_model(buffer.data(), buffer.size());
    REQUIRE(len > 0);
    JsonDocument doc;
    REQUIRE(!deserializeJson(doc, std::string(buffer.data(), static_cast<size_t>(len))));
    return doc;
}

JsonVariantConst self_vessel(const JsonDocument& doc) {
    const char* self = doc["self"] | "";
    return doc["vessels"][self];
}

void test_single_source_has_no_values() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 1000000;
    feed(app, GpsA, "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    feed(app, GpsA, "GPGGA,123520,4807.040,N,01131.002,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    const JsonDocument doc = full_model(app);
    const JsonVariantConst position = self_vessel(doc)["navigation"]["position"];
    REQUIRE(!position.isNull());
    REQUIRE(position["values"].isNull());
}

void test_two_gps_sources_are_both_reported() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 1000000;
    feed(app, GpsA, "GPGGA,123519,4807.000,N,01131.000,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    feed(app, GpsB, "GPGGA,123519,4808.000,N,01132.000,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    // A's second update, then B's: the model now holds B's position.
    feed(app, GpsA, "GPGGA,123520,4807.060,N,01131.060,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    feed(app, GpsB, "GPGGA,123520,4808.060,N,01132.060,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    // Speed becomes multi-source when B first overwrites A, so A's entry
    // appears with A's next update (as with a live 1 Hz feed).
    feed(app, GpsA, "GPRMC,123520,A,4807.060,N,01131.060,E,4.0,84.4,230394,3.1,W,A", now_us);
    feed(app, GpsB, "GPRMC,123520,A,4808.060,N,01132.060,E,6.0,84.4,230394,3.1,W,A", now_us);
    feed(app, GpsA, "GPRMC,123521,A,4807.060,N,01131.060,E,5.0,84.4,230394,3.1,W,A", now_us);
    feed(app, GpsB, "GPRMC,123521,A,4808.060,N,01132.060,E,7.0,84.4,230394,3.1,W,A", now_us);

    const JsonDocument doc = full_model(app);
    const JsonVariantConst position = self_vessel(doc)["navigation"]["position"];
    REQUIRE(std::strcmp(position["$source"] | "", "connector1") == 0);
    NEAR(position["values"]["connector0"]["value"]["latitude"].as<double>(), 48.0 + 7.06 / 60.0, 1e-9);
    NEAR(position["values"]["connector1"]["value"]["latitude"].as<double>(), 48.0 + 8.06 / 60.0, 1e-9);
    NEAR(position["values"]["connector0"]["value"]["longitude"].as<double>(), 11.0 + 31.06 / 60.0, 1e-9);

    const JsonVariantConst sog = self_vessel(doc)["navigation"]["speedOverGround"];
    NEAR(sog["values"]["connector0"]["value"].as<double>(), 5.0 * 0.514444, 1e-3);
    NEAR(sog["values"]["connector1"]["value"].as<double>(), 7.0 * 0.514444, 1e-3);
}

void test_deltas_carry_each_sources_own_value() {
    signalk_mini::SignalKMiniApp<float> app;
    signalk_mini::SignalKMiniConfig config;
    config.publisher.max_changes_per_tick = 256;
    signalk_mini::SignalKPublisher<float> publisher(app.store(), config);
    CollectingSink drained;
    uint64_t now_us = 1000000;
    feed(app, GpsA, "GPRMC,123519,A,4807.000,N,01131.000,E,5.0,84.4,230394,3.1,W,A", now_us);
    feed(app, GpsB, "GPRMC,123519,A,4808.000,N,01132.000,E,7.0,84.4,230394,3.1,W,A", now_us);
    publisher.publish_pending(drained);

    // Both sources update before the next publish: the model holds only B's
    // values, but A's delta must still carry A's speed and position.
    feed(app, GpsA, "GPRMC,123520,A,4807.100,N,01131.100,E,5.5,84.4,230394,3.1,W,A", now_us);
    feed(app, GpsB, "GPRMC,123520,A,4808.100,N,01132.100,E,7.5,84.4,230394,3.1,W,A", now_us);
    CollectingSink sink;
    publisher.publish_pending(sink);

    std::vector<double> speeds;
    std::vector<double> latitudes;
    for (const std::string& delta : sink.deltas) {
        JsonDocument doc;
        REQUIRE(!deserializeJson(doc, delta));
        const JsonArrayConst values = doc["updates"][0]["values"];
        for (JsonVariantConst value : values) {
            const char* path = value["path"] | "";
            if (std::strcmp(path, "navigation.speedOverGround") == 0) speeds.push_back(value["value"].as<double>());
            if (std::strcmp(path, "navigation.position") == 0) latitudes.push_back(value["value"]["latitude"].as<double>());
        }
    }
    REQUIRE(speeds.size() == 2);
    NEAR(speeds[0], 5.5 * 0.514444, 1e-3);
    NEAR(speeds[1], 7.5 * 0.514444, 1e-3);
    REQUIRE(latitudes.size() == 2);
    // ArduinoJson parses short numbers such as 48.135 as float, hence 1e-5.
    NEAR(latitudes[0], 48.0 + 7.1 / 60.0, 1e-5);
    NEAR(latitudes[1], 48.0 + 8.1 / 60.0, 1e-5);
}

void test_table_is_bounded() {
    signalk_mini::SourceValueTable<2> table;
    signalk_mini::SourceValueEntry entry;
    entry.path = "a";
    entry.kind = signalk_mini::SourceValueKind::Number;
    for (int i = 0; i < 5; ++i) {
        entry.source_id = static_cast<signalk_mini::SourceId>(10 + i);
        entry.last_update_us = static_cast<uint64_t>(100 + i);
        table.record(entry);
    }
    REQUIRE(table.count_for("a") == 2);
    REQUIRE(table.evicted_count() == 3);
    REQUIRE(table.find("a", 14) != nullptr);
    REQUIRE(table.find("a", 13) != nullptr);
    REQUIRE(table.find("a", 10) == nullptr);
}

} // namespace

int main() {
    test_single_source_has_no_values();
    test_two_gps_sources_are_both_reported();
    test_deltas_carry_each_sources_own_value();
    test_table_is_bounded();
    std::puts("test_multi_source passed");
    return 0;
}
