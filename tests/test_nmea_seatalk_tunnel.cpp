#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <signalk_mini.hpp>
#include <nmea0183_connector.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static std::string make_sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body, nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

static void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = make_sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 7, now_us));
}

int main() {
    signalk_mini::SignalKMiniApp<float> app;
    uint64_t now_us = 0;

    feed(app, "ST" "ALK,00,02,00,D2,00", now_us);
    REQUIRE(app.store().model().sea.depth_m.valid);
    NEAR(app.store().model().sea.depth_m.value, 6.4008f, 0.0005f);

    feed(app, "P" "ST" "ALK,100100B4", now_us);
    REQUIRE(app.store().model().wind.apparent.direction_deg.valid);
    NEAR(app.store().model().wind.apparent.direction_deg.value, 90.0f, 0.001f);

    feed(app, "P" "ST" "ALK,2,1,A,1101", now_us);
    REQUIRE(!app.store().model().wind.apparent.speed_kn.valid);
    feed(app, "P" "ST" "ALK,2,2,A,0C03", now_us);
    REQUIRE(app.store().model().wind.apparent.speed_kn.valid);
    NEAR(app.store().model().wind.apparent.speed_kn.value, 12.3f, 0.001f);

    signalk_mini::ModelStore<float> store;
    signalk_mini::SeaTalkInput<float> input(store);
    const uint8_t sog[] = {0x52, 0x01, 0x61, 0x00};
    REQUIRE(input.feed_datagram(sog, sizeof(sog), 8, now_us + 1000));
    REQUIRE(store.model().gnss.fix.speed_kn.valid);
    NEAR(store.model().gnss.fix.speed_kn.value, 9.7f, 0.001f);

    return 0;
}
