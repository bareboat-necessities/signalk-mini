#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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

    const uint8_t trip_total[] = {0x25, 0x04, 0x39, 0x30, 0x85, 0x1a, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(trip_total, sizeof(trip_total)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::trip_total);
    NEAR(decoded.value, 1234.5f, 0.001f);
    NEAR(decoded.secondary_value, 67.89f, 0.001f);

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

static void require_navigation_decodes() {
    seatalk::SeaTalkDecoded<float> decoded;

    const uint8_t lat[] = {0x50, 0x02, 0x39, 0x28, 0x0f};
    REQUIRE(seatalk::decode_seatalk_frame(frame(lat, sizeof(lat)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::position_latitude);
    NEAR(decoded.value, 57.646666f, 0.0005f);

    const uint8_t lon[] = {0x51, 0x02, 0x0b, 0xa4, 0x90};
    REQUIRE(seatalk::decode_seatalk_frame(frame(lon, sizeof(lon)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::position_longitude);
    NEAR(decoded.value, 11.71f, 0.0005f);

    const uint8_t sog[] = {0x52, 0x01, 0x61, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(sog, sizeof(sog)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::speed_over_ground);
    NEAR(decoded.value, 9.7f, 0.001f);

    const uint8_t cog[] = {0x53, 0x00, 0x2d};
    REQUIRE(seatalk::decode_seatalk_frame(frame(cog, sizeof(cog)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::course_over_ground);
    NEAR(decoded.value, 90.0f, 0.001f);

    const uint8_t time[] = {0x54, 0x01, 0x27, 0x11};
    REQUIRE(seatalk::decode_seatalk_frame(frame(time, sizeof(time)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::time_utc);
    NEAR(decoded.value, 61788.0f, 0.001f);
    REQUIRE(decoded.int_value == 17);
    REQUIRE(decoded.int_secondary_value == 9);
    REQUIRE(decoded.int_third_value == 48);

    const uint8_t date[] = {0x56, 0x81, 0x12, 0x16};
    REQUIRE(seatalk::decode_seatalk_frame(frame(date, sizeof(date)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::date_utc);
    REQUIRE(decoded.int_value == 18);
    REQUIRE(decoded.int_secondary_value == 8);
    REQUIRE(decoded.int_third_value == 2022);

    const uint8_t sats[] = {0x57, 0x81, 0x05, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(sats, sizeof(sats)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::satellite_info);
    REQUIRE(decoded.int_value == 8);
    NEAR(decoded.secondary_value, 5.0f, 0.001f);

    const uint8_t latlon[] = {0x58, 0x25, 0x39, 0x97, 0x71, 0x0b, 0xa6, 0x4c};
    REQUIRE(seatalk::decode_seatalk_frame(frame(latlon, sizeof(latlon)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::position_lat_lon);
    NEAR(decoded.value, 57.64615f, 0.0005f);
    NEAR(decoded.secondary_value, 11.709533f, 0.0005f);

    const uint8_t waypoint_id[] = {0x82, 0x04, 0x81, 0x00, 0x30, 0x00, 0x10};
    REQUIRE(seatalk::decode_seatalk_frame(frame(waypoint_id, sizeof(waypoint_id)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::waypoint_id);
    REQUIRE(std::strcmp(decoded.label, "1234") == 0);

    const uint8_t waypoint_name[] = {0xa1, 0x0d, 0, 0, 0, 0, 0, 0, 'H', 'A', 'R', 'B', 'O', 'R', '1', 0};
    REQUIRE(seatalk::decode_seatalk_frame(frame(waypoint_name, sizeof(waypoint_name)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::waypoint_name);
    REQUIRE(std::strcmp(decoded.label, "HARBOR1") == 0);

    const uint8_t arrival[] = {0xa2, 0x64, 0x00, 'W', 'P', '0', '1'};
    REQUIRE(seatalk::decode_seatalk_frame(frame(arrival, sizeof(arrival)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::arrival_info);
    REQUIRE(decoded.secondary_valid);
    REQUIRE(decoded.third_valid);
    REQUIRE(std::strcmp(decoded.label, "WP01") == 0);
}

static void require_autopilot_decodes() {
    seatalk::SeaTalkDecoded<float> decoded;

    const uint8_t autopilot[] = {0x84, 0x16, 0x40, 0x00, 0x42, 0x00, 0xfe, 0x01, 0x08};
    REQUIRE(seatalk::decode_seatalk_frame(frame(autopilot, sizeof(autopilot)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::autopilot_state);
    REQUIRE(decoded.autopilot_mode == seatalk::SeaTalkAutopilotMode::auto_heading);
    REQUIRE(!decoded.alarm);
    NEAR(decoded.value, 90.0f, 0.001f);
    NEAR(decoded.secondary_value, 90.0f, 0.001f);
    NEAR(decoded.third_value, -2.0f, 0.001f);
    REQUIRE(std::strcmp(decoded.label, "auto") == 0);

    const uint8_t nav_to_wp[] = {0x85, 0xb5, 0x07, 0x01, 0x00, 0x05, 0x07, 0x00};
    REQUIRE(seatalk::decode_seatalk_frame(frame(nav_to_wp, sizeof(nav_to_wp)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::navigation_to_waypoint);
    NEAR(decoded.value, 1.23f, 0.001f);
    NEAR(decoded.secondary_value, 90.0f, 0.001f);
    NEAR(decoded.third_value, 8.0f, 0.001f);

    const uint8_t key[] = {0x86, 0x01, 0x41, 0xbe};
    REQUIRE(seatalk::decode_seatalk_frame(frame(key, sizeof(key)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::autopilot_key);
    REQUIRE(decoded.code == 0x01);
    REQUIRE(decoded.long_press);
    REQUIRE(std::strcmp(decoded.label, "auto") == 0);

    const uint8_t variation[] = {0x99, 0x00, 0xfe};
    REQUIRE(seatalk::decode_seatalk_frame(frame(variation, sizeof(variation)), decoded));
    REQUIRE(decoded.kind == seatalk::SeaTalkDecodedKind::compass_variation);
    NEAR(decoded.value, -2.0f, 0.001f);
}

int main() {
    require_frame_parser();
    require_basic_decodes();
    require_navigation_decodes();
    require_autopilot_decodes();
    return 0;
}
