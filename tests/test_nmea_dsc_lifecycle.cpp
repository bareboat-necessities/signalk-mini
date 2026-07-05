#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static std::string sentence(const char* body) {
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::string out = "$";
    out += body;
    out += "*";
    out.push_back(nmea0183_connector::to_hex((cs >> 4) & 0x0f));
    out.push_back(nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

static void feed(signalk_mini::SignalKMiniApp<float>& app,
                 const char* body,
                 uint64_t& now_us,
                 uint64_t delta_us = 1000) {
    now_us += delta_us;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 1, now_us));
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;
    const char* call = "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,B";
    const char* ack = "CDDSC,112,338040079,112,107,126,0123407356,1234,338040079,00,A";

    feed(app, call, now_us);
    auto& model = app.store().model();
    const uint64_t first_seen_us = model.notifications.dsc.distress.first_seen_us;
    REQUIRE(model.notifications.dsc.distress.active == true);
    REQUIRE(model.notifications.dsc.distress.acknowledged == false);
    REQUIRE(model.notifications.dsc.distress.resolved == false);
    REQUIRE(model.notifications.dsc.distress.duplicate == false);
    REQUIRE(model.notifications.dsc.distress.repeat_count.value == 1);
    REQUIRE(model.comm.dsc.distress_count.value == 1);

    feed(app, call, now_us);
    REQUIRE(model.notifications.dsc.distress.active == true);
    REQUIRE(model.notifications.dsc.distress.acknowledged == false);
    REQUIRE(model.notifications.dsc.distress.resolved == false);
    REQUIRE(model.notifications.dsc.distress.duplicate == true);
    REQUIRE(model.notifications.dsc.distress.repeat_count.value == 2);
    REQUIRE(model.notifications.dsc.distress.first_seen_us == first_seen_us);
    REQUIRE(model.comm.dsc.distress_count.value == 1);
    REQUIRE(model.comm.dsc.duplicate_count.value == 1);

    feed(app, ack, now_us);
    REQUIRE(model.notifications.dsc.distress.active == true);
    REQUIRE(model.notifications.dsc.distress.acknowledged == true);
    REQUIRE(model.notifications.dsc.distress.resolved == false);
    REQUIRE(model.notifications.dsc.distress.duplicate == false);
    REQUIRE(model.notifications.dsc.distress.repeat_count.value == 3);
    REQUIRE(model.notifications.dsc.distress.first_seen_us == first_seen_us);
    REQUIRE(model.comm.dsc.distress_count.value == 1);
    return 0;
}
