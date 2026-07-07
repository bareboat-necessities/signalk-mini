#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <seatalk.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static seatalk::SeaTalkFrame frame(const uint8_t* bytes, size_t len) {
    seatalk::SeaTalkFrame out;
    REQUIRE(seatalk::seatalk_frame_from_bytes(bytes, len, out));
    return out;
}

static void require_frame_parser() {
    const uint8_t depth[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    seatalk::SeaTalkRxState state;
    seatalk::SeaTalkFrame out;
    REQUIRE(!state.feed_byte(depth[0], out));
    REQUIRE(!state.feed_byte(depth[1], out));
    REQUIRE(!state.feed_byte(depth[2], out));
    REQUIRE(!state.feed_byte(depth[3], out));
    REQUIRE(state.feed_byte(depth[4], out));
    REQUIRE(out.length == 5);
    REQUIRE(out.command() == 0x00);
}

static void require_basic_decodes() {
    seatalk::SeaTalkDecoded<float> decoded;

    const uint8_t depth[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(depth, sizeof(depth)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::depth);
    NEAR(decoded.value, 6.4008f, 0.0005f);

    const uint8_t wind_angle[] = {0x10, 0x01, 0x00, 0xb4};
    REQUIRE(seatalk::decode_seatalk_frame(frame(wind_angle, sizeof(wind_angle)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::apparent_wind_angle);
    NEAR(decoded.value, 90.0f, 0.001f);

    const uint8_t wind_speed[] = {0x11, 0x01, 0x0c, 0x03};
    REQUIRE(seatalk::decode_seatalk_frame(frame(wind_speed, sizeof(wind_speed)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::apparent_wind_speed);
    NEAR(decoded.value, 12.3f, 0.001f);
    NEAR(decoded.secondary_value, 6.32766f, 0.001f);

    const uint8_t water_speed[] = {0x20, 0x01, 0x41, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(water_speed, sizeof(water_speed)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::speed_through_water);
    NEAR(decoded.value, 6.5f, 0.001f);

    const uint8_t log_speed[] = {0x26, 0x04, 0xd2, 0x04, 0x00, 0x00, 0x10};
    REQUIRE(seatalk::decode_seatalk_frame(frame(log_speed, sizeof(log_speed)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::speed_through_water);
    NEAR(decoded.value, 12.34f, 0.001f);

    const uint8_t temp_direct[] = {0x23, 0x01, 0x16, 0x48};
    REQUIRE(seatalk::decode_seatalk_frame(frame(temp_direct, sizeof(temp_direct)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::water_temperature);
    NEAR(decoded.value, 22.0f, 0.001f);

    const uint8_t temp_precise[] = {0x27, 0x01, 0x37, 0x01};
    REQUIRE(seatalk::decode_seatalk_frame(frame(temp_precise, sizeof(temp_precise)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::water_temperature);
    NEAR(decoded.value, 21.1f, 0.001f);

    const uint8_t heading[] = {0x89, 0x01, 0x2d, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(heading, sizeof(heading)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::heading_magnetic);
    NEAR(decoded.value, 90.0f, 0.001f);

    const uint8_t rudder[] = {0x9c, 0x01, 0x2d, 0xfe};
    REQUIRE(seatalk::decode_seatalk_frame(frame(rudder, sizeof(rudder)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::rudder_angle);
    NEAR(decoded.value, -2.0f, 0.001f);
    NEAR(decoded.secondary_value, 90.0f, 0.001f);
}

int main() {
    require_frame_parser();
    require_basic_decodes();
    return 0;
}
