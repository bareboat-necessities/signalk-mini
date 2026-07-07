#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <seatalk.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

static void accept(seatalk::SeaTalkReceiver<float>& receiver,
                   ship_data_model::DataModel<float>& model,
                   const uint8_t* bytes,
                   size_t len,
                   uint64_t& now_us) {
    now_us += 1000;
    REQUIRE(receiver.accept_datagram(bytes, len, model, now_us, ship_data_model::SensorSource::serial));
}

int main() {
    seatalk::SeaTalkReceiver<float> receiver;
    ship_data_model::DataModel<float> model;
    uint64_t now_us = 0;

    const uint8_t depth[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    accept(receiver, model, depth, sizeof(depth), now_us);
    REQUIRE(model.sea.depth_m.valid);
    REQUIRE(model.sea.depth_below_surface_m.valid);
    NEAR(model.sea.depth_m.value, 6.4008f, 0.0005f);

    const uint8_t engine[] = {0x05, 0x03, 0x00, 0x09, 0xc4, 0x0a};
    accept(receiver, model, engine, sizeof(engine), now_us);
    REQUIRE(model.propulsion.revolutions.speed_rpm.valid);
    REQUIRE(model.propulsion.revolutions.propeller_pitch_percent.valid);
    NEAR(model.propulsion.revolutions.speed_rpm.value, 2500.0f, 0.001f);
    NEAR(model.propulsion.revolutions.propeller_pitch_percent.value, 10.0f, 0.001f);

    const uint8_t wind_angle[] = {0x10, 0x01, 0x00, 0xb4};
    accept(receiver, model, wind_angle, sizeof(wind_angle), now_us);
    REQUIRE(model.wind.apparent.direction_deg.valid);
    NEAR(model.wind.apparent.direction_deg.value, 90.0f, 0.001f);

    const uint8_t wind_speed[] = {0x11, 0x01, 0x0c, 0x03};
    accept(receiver, model, wind_speed, sizeof(wind_speed), now_us);
    REQUIRE(model.wind.apparent.speed_kn.valid);
    REQUIRE(model.wind.apparent.speed_m_s.valid);
    NEAR(model.wind.apparent.speed_kn.value, 12.3f, 0.001f);

    const uint8_t water_speed[] = {0x20, 0x01, 0x41, 0x00};
    accept(receiver, model, water_speed, sizeof(water_speed), now_us);
    REQUIRE(model.sea.speed_kn.valid);
    REQUIRE(model.sea.longitudinal_water_speed_kn.valid);
    REQUIRE(model.sea.water_speed_status == 'A');
    NEAR(model.sea.speed_kn.value, 6.5f, 0.001f);

    const uint8_t trip_total[] = {0x25, 0x04, 0x39, 0x30, 0x85, 0x1a, 0x00};
    accept(receiver, model, trip_total, sizeof(trip_total), now_us);
    REQUIRE(model.sea.total_distance_nmi.valid);
    REQUIRE(model.sea.trip_distance_nmi.valid);
    NEAR(model.sea.total_distance_nmi.value, 1234.5f, 0.001f);
    NEAR(model.sea.trip_distance_nmi.value, 67.89f, 0.001f);

    const uint8_t temperature[] = {0x27, 0x01, 0x37, 0x01};
    accept(receiver, model, temperature, sizeof(temperature), now_us);
    REQUIRE(model.sea.temperature_c.valid);
    NEAR(model.sea.temperature_c.value, 21.1f, 0.001f);

    const uint8_t heading[] = {0x89, 0x01, 0x2d, 0x00};
    accept(receiver, model, heading, sizeof(heading), now_us);
    REQUIRE(model.ins.imu.heading_magnetic_deg.valid);
    NEAR(model.ins.imu.heading_magnetic_deg.value, 90.0f, 0.001f);

    const uint8_t lat[] = {0x50, 0x02, 0x39, 0x28, 0x0f};
    accept(receiver, model, lat, sizeof(lat), now_us);
    REQUIRE(model.gnss.fix.fix_lat_deg.valid);
    NEAR(model.gnss.fix.fix_lat_deg.value, 57.646666f, 0.0005f);

    const uint8_t lon[] = {0x51, 0x02, 0x0b, 0xa4, 0x90};
    accept(receiver, model, lon, sizeof(lon), now_us);
    REQUIRE(model.gnss.fix.fix_lon_deg.valid);
    NEAR(model.gnss.fix.fix_lon_deg.value, 11.71f, 0.0005f);

    const uint8_t sog[] = {0x52, 0x01, 0x61, 0x00};
    accept(receiver, model, sog, sizeof(sog), now_us);
    REQUIRE(model.gnss.fix.speed_kn.valid);
    REQUIRE(model.sea.longitudinal_ground_speed_kn.valid);
    REQUIRE(model.sea.ground_speed_status == 'A');
    NEAR(model.gnss.fix.speed_kn.value, 9.7f, 0.001f);

    const uint8_t cog[] = {0x53, 0x00, 0x2d};
    accept(receiver, model, cog, sizeof(cog), now_us);
    REQUIRE(model.gnss.fix.track_deg.valid);
    NEAR(model.gnss.fix.track_deg.value, 90.0f, 0.001f);

    const uint8_t time[] = {0x54, 0x01, 0x27, 0x11};
    accept(receiver, model, time, sizeof(time), now_us);
    REQUIRE(model.gnss.fix.timestamp_s.valid);
    NEAR(model.gnss.fix.timestamp_s.value, 61788.0f, 0.001f);

    const uint8_t date[] = {0x56, 0x81, 0x12, 0x16};
    accept(receiver, model, date, sizeof(date), now_us);
    REQUIRE(model.gnss.fix.date_day.valid);
    REQUIRE(model.gnss.fix.date_month.valid);
    REQUIRE(model.gnss.fix.date_year.valid);
    REQUIRE(model.gnss.fix.date_day.value == 18);
    REQUIRE(model.gnss.fix.date_month.value == 8);
    REQUIRE(model.gnss.fix.date_year.value == 2022);

    const uint8_t sats[] = {0x57, 0x81, 0x05, 0x00};
    accept(receiver, model, sats, sizeof(sats), now_us);
    REQUIRE(model.gnss.fix.satellites_used.valid);
    REQUIRE(model.gnss.satellites_in_view.satellites_in_view.valid);
    REQUIRE(model.gnss.fix.satellites_used.value == 8);

    const uint8_t latlon[] = {0x58, 0x25, 0x39, 0x97, 0x71, 0x0b, 0xa6, 0x4c};
    accept(receiver, model, latlon, sizeof(latlon), now_us);
    NEAR(model.gnss.fix.fix_lat_deg.value, 57.64615f, 0.0005f);
    NEAR(model.gnss.fix.fix_lon_deg.value, 11.709533f, 0.0005f);

    const uint8_t observed59[] = {0x59, 0x11, 0xce, 0xff};
    accept(receiver, model, observed59, sizeof(observed59), now_us);
    const uint8_t e80_sig[] = {0x61, 0x03, 0x03, 0x00, 0x00, 0x00};
    accept(receiver, model, e80_sig, sizeof(e80_sig), now_us);

    const uint8_t rudder[] = {0x9c, 0x01, 0x2d, 0xfe};
    accept(receiver, model, rudder, sizeof(rudder), now_us);
    REQUIRE(model.steering.rudder.angle_deg.valid);
    NEAR(model.steering.rudder.angle_deg.value, -2.0f, 0.001f);

    const uint8_t waypoint_id[] = {0x82, 0x04, 0x81, 0x00, 0x30, 0x00, 0x10};
    accept(receiver, model, waypoint_id, sizeof(waypoint_id), now_us);
    REQUIRE(std::strcmp(model.route.waypoint.to_waypoint_id, "1234") == 0);
    REQUIRE(std::strcmp(model.route.apb.destination_id, "1234") == 0);

    const uint8_t waypoint_name[] = {0xa1, 0x0d, 0, 0, 0, 0, 0, 0, 'H', 'A', 'R', 'B', 'O', 'R', '1', 0};
    accept(receiver, model, waypoint_name, sizeof(waypoint_name), now_us);
    REQUIRE(std::strcmp(model.route.waypoint.to_waypoint_id, "HARBOR1") == 0);

    const uint8_t arrival[] = {0xa2, 0x64, 0x00, 'W', 'P', '0', '1'};
    accept(receiver, model, arrival, sizeof(arrival), now_us);
    REQUIRE(model.route.waypoint_arrival.perpendicular_passed.value);
    REQUIRE(model.route.waypoint_arrival.arrival_circle_entered.value);
    REQUIRE(model.route.rmb.arrived.value);
    REQUIRE(std::strcmp(model.route.waypoint_arrival.waypoint_id, "WP01") == 0);

    const uint8_t autopilot[] = {0x84, 0x16, 0x40, 0x00, 0x42, 0x00, 0xfe, 0x01, 0x08};
    accept(receiver, model, autopilot, sizeof(autopilot), now_us);
    REQUIRE(model.autopilot.controller.enabled.value);
    REQUIRE(model.autopilot.controller.mode.value == ship_data_model::AutopilotMode::compass);
    REQUIRE(model.autopilot.controller.heading_deg.valid);
    REQUIRE(model.autopilot.controller.heading_command_deg.valid);
    NEAR(model.autopilot.controller.heading_deg.value, 90.0f, 0.001f);
    NEAR(model.autopilot.controller.heading_command_deg.value, 90.0f, 0.001f);
    NEAR(model.steering.rudder.angle_deg.value, -2.0f, 0.001f);

    const uint8_t nav_to_wp[] = {0x85, 0xb5, 0x07, 0x01, 0x00, 0x05, 0x07, 0x00};
    accept(receiver, model, nav_to_wp, sizeof(nav_to_wp), now_us);
    REQUIRE(model.route.apb.xte_nmi.valid);
    REQUIRE(model.route.apb.heading_to_steer_deg.valid);
    REQUIRE(model.route.waypoint.distance_nmi.valid);
    REQUIRE(model.route.apb.mode_hint.value == ship_data_model::AutopilotMode::nav);
    NEAR(model.route.apb.xte_nmi.value, 1.23f, 0.001f);
    NEAR(model.route.apb.heading_to_steer_deg.value, 90.0f, 0.001f);
    NEAR(model.route.waypoint.distance_nmi.value, 8.0f, 0.001f);

    const uint8_t variation[] = {0x99, 0x00, 0xfe};
    accept(receiver, model, variation, sizeof(variation), now_us);
    REQUIRE(model.ins.imu.magnetic_variation_deg.valid);
    NEAR(model.ins.imu.magnetic_variation_deg.value, -2.0f, 0.001f);

    const uint8_t waypoint_def[] = {0x9e, 0x0c, 0x00, 'W', 'P', 'D', 'E', 'F', 0, 0, 0, 0, 0, 0, 0};
    accept(receiver, model, waypoint_def, sizeof(waypoint_def), now_us);

    const uint8_t sat_fix[] = {0xa5, 0x57, 0x22, 0x84, 0x00, 0x12, 0x03, 0x00, 0x10, 0x2a};
    accept(receiver, model, sat_fix, sizeof(sat_fix), now_us);
    REQUIRE(model.gnss.fix.fix_quality.valid);
    REQUIRE(model.gnss.fix.fix_quality.value == 2);

    const uint8_t sat_detail[] = {0xa5, 0x0d, 0x2c, 0x43, 0x77, 0x52, 0x2b, 0x5d, 0x7d, 0x11, 0x6d, 0x4e, 0x0f, 0x27, 0x56, 0x02};
    accept(receiver, model, sat_detail, sizeof(sat_detail), now_us);
    REQUIRE(model.gnss.satellites_in_view.satellite_prn[0].valid);

    const uint8_t sat_used[] = {0xa5, 0x74, 0x96, 0x00, 0x00, 0x00, 0x80};
    accept(receiver, model, sat_used, sizeof(sat_used), now_us);
    const uint8_t sat_done[] = {0xa5, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    accept(receiver, model, sat_done, sizeof(sat_done), now_us);
    const uint8_t wgs84[] = {0xa5, 0xb5, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00};
    accept(receiver, model, wgs84, sizeof(wgs84), now_us);
    const uint8_t diff[] = {0xa7, 0x09, 0x85, 0x82, 0x47, 0x42, 0x8b, 0x00, 0x00, 0x00, 0x00, 0x76};
    accept(receiver, model, diff, sizeof(diff), now_us);
    const uint8_t observed_ad[] = {0xad, 0x00, 0x00};
    accept(receiver, model, observed_ad, sizeof(observed_ad), now_us);

    const uint8_t key[] = {0x86, 0x01, 0x41, 0xbe};
    accept(receiver, model, key, sizeof(key), now_us);
    REQUIRE(std::strcmp(model.notifications.messages.event.event_id, "seatalk_ap_key") == 0);
    REQUIRE(model.notifications.messages.event.event_state == 'L');
    REQUIRE(std::strcmp(model.notifications.messages.event.event_text, "auto") == 0);

    REQUIRE(receiver.decoded_count() == 34);
    REQUIRE(receiver.unsupported_count() == 0);
    return 0;
}
