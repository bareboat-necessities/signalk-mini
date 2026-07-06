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

    feed(app, "ICIMK,A,OK,ONE", now_us);
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 1);
    REQUIRE(app.store().model().comm.inmarsat.recent_message_count.value == 1);
    REQUIRE(app.store().model().comm.inmarsat.latest_message.repeat_count.value == 1);
    REQUIRE(std::strcmp(app.store().model().comm.inmarsat.recent_messages[0].message_id, "A") == 0);

    const uint64_t first_seen = app.store().model().comm.inmarsat.latest_message.first_seen_us;
    feed(app, "ICIMK,A,OK,ONE", now_us);
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 2);
    REQUIRE(app.store().model().comm.inmarsat.recent_message_count.value == 1);
    REQUIRE(app.store().model().comm.inmarsat.duplicate_count.value == 1);
    REQUIRE(app.store().model().comm.inmarsat.latest_message.duplicate == true);
    REQUIRE(app.store().model().comm.inmarsat.latest_message.repeat_count.value == 2);
    REQUIRE(app.store().model().comm.inmarsat.latest_message.first_seen_us == first_seen);

    feed(app, "ICIMK,B,OK,TWO", now_us);
    feed(app, "ICIMK,C,OK,THREE", now_us);
    feed(app, "ICIMK,D,OK,FOUR", now_us);
    feed(app, "ICIMK,E,OK,FIVE", now_us);
    REQUIRE(app.store().model().comm.inmarsat.message_count.value == 6);
    REQUIRE(app.store().model().comm.inmarsat.recent_message_count.value == ship_data_model::INMARSAT_MESSAGE_HISTORY_CAPACITY);
    REQUIRE(app.store().model().comm.inmarsat.overwrite_count.value == 1);
    REQUIRE(std::strcmp(app.store().model().comm.inmarsat.latest_message.message_id, "E") == 0);
    return 0;
}
