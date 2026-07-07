#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d\n", __FILE__, __LINE__); std::exit(2); } } while (0)

static void require_bytes(const seatalk::SeaTalkFrame& frame, const uint8_t* expected, uint8_t length) {
    REQUIRE(frame.length == length);
    REQUIRE(std::memcmp(frame.bytes, expected, length) == 0);
}

int main() {
    seatalk::SeaTalkFrame frame;
    seatalk::SeaTalkDecoded<float> decoded;

    REQUIRE(seatalk::make_depth_m(frame, 6.4008f));
    const uint8_t depth[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    require_bytes(frame, depth, sizeof(depth));
    REQUIRE(seatalk::decode_seatalk_frame(frame, decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::depth);
    NEAR(decoded.value, 6.4008f, 0.0005f);

    REQUIRE(seatalk::make_apparent_wind_angle_deg(frame, 90.0f));
    const uint8_t wind_angle[] = {0x10, 0x01, 0x00, 0xb4};
    require_bytes(frame, wind_angle, sizeof(wind_angle));

    REQUIRE(seatalk::make_apparent_wind_speed_kn(frame, 12.3f));
    const uint8_t wind_speed[] = {0x11, 0x01, 0x0c, 0x03};
    require_bytes(frame, wind_speed, sizeof(wind_speed));

    REQUIRE(seatalk::make_speed_through_water_kn(frame, 6.5f));
    const uint8_t water_speed[] = {0x20, 0x01, 0x41, 0x00};
    require_bytes(frame, water_speed, sizeof(water_speed));

    REQUIRE(seatalk::make_pilot_key(frame, seatalk::SeaTalkPilotKey::plus_1));
    const uint8_t plus_1[] = {0x86, 0x01, 0x07, 0xf8};
    require_bytes(frame, plus_1, sizeof(plus_1));
    REQUIRE(seatalk::decode_seatalk_frame(frame, decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::autopilot_key);
    REQUIRE(decoded.code == 0x07);

    char sentence[96];
    REQUIRE(nmea0183_connector::make_seatalk_depth_m(sentence, sizeof(sentence), 6.4008f) > 0);
    REQUIRE(std::strncmp(sentence, "$STALK,000200D200*", 18) == 0);
    REQUIRE(nmea0183_connector::verify_nmea_checksum(sentence));

    signalk_mini::SignalKMiniApp<float> app;
    REQUIRE(app.nmea0183().feed_line(sentence, 1, 1000));
    REQUIRE(app.store().model().sea.depth_m.valid);
    NEAR(app.store().model().sea.depth_m.value, 6.4008f, 0.0005f);

    REQUIRE(nmea0183_connector::make_seatalk_pilot_key(sentence, sizeof(sentence), seatalk::SeaTalkPilotKey::standby) > 0);
    REQUIRE(nmea0183_connector::verify_nmea_checksum(sentence));

    return 0;
}
