#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

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
    auto& safetynet = app.store().model().notifications.inmarsat.safetynet;

    feed(app, "ICIMK,A,OK,ONE", now_us);
    REQUIRE(safetynet.message_count.value == 1);
    REQUIRE(safetynet.recent_message_count.value == 1);
    REQUIRE(safetynet.latest_message.repeat_count.value == 1);
    REQUIRE(std::strcmp(safetynet.recent_messages[0].message_id, "A") == 0);

    const uint64_t first_seen = safetynet.latest_message.first_seen_us;
    feed(app, "ICIMK,A,OK,ONE", now_us);
    REQUIRE(safetynet.message_count.value == 2);
    REQUIRE(safetynet.recent_message_count.value == 1);
    REQUIRE(safetynet.duplicate_count.value == 1);
    REQUIRE(safetynet.latest_message.duplicate == true);
    REQUIRE(safetynet.latest_message.repeat_count.value == 2);
    REQUIRE(safetynet.latest_message.first_seen_us == first_seen);

    feed(app, "ICIMK,B,OK,TWO", now_us);
    feed(app, "ICIMK,C,OK,THREE", now_us);
    feed(app, "ICIMK,D,OK,FOUR", now_us);
    feed(app, "ICIMK,E,OK,FIVE", now_us);
    REQUIRE(safetynet.message_count.value == 6);
    REQUIRE(safetynet.recent_message_count.value == ship_data_model::INMARSAT_SAFETYNET_MESSAGE_HISTORY_CAPACITY);
    REQUIRE(safetynet.overwrite_count.value == 1);
    REQUIRE(std::strcmp(safetynet.latest_message.message_id, "E") == 0);
    return 0;
}
