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

    const uint8_t temperature[] = {0x27, 0x01, 0x37, 0x01};
    accept(receiver, model, temperature, sizeof(temperature), now_us);
    REQUIRE(model.sea.temperature_c.valid);
    NEAR(model.sea.temperature_c.value, 21.1f, 0.001f);

    const uint8_t heading[] = {0x89, 0x01, 0x2d, 0x00};
    accept(receiver, model, heading, sizeof(heading), now_us);
    REQUIRE(model.ins.imu.heading_magnetic_deg.valid);
    NEAR(model.ins.imu.heading_magnetic_deg.value, 90.0f, 0.001f);

    const uint8_t rudder[] = {0x9c, 0x01, 0x2d, 0xfe};
    accept(receiver, model, rudder, sizeof(rudder), now_us);
    REQUIRE(model.steering.rudder.angle_deg.valid);
    NEAR(model.steering.rudder.angle_deg.value, -2.0f, 0.001f);

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

    const uint8_t key[] = {0x86, 0x01, 0x41, 0xbe};
    accept(receiver, model, key, sizeof(key), now_us);
    REQUIRE(std::strcmp(model.notifications.messages.event.event_id, "seatalk_ap_key") == 0);
    REQUIRE(model.notifications.messages.event.event_state == 'L');
    REQUIRE(std::strcmp(model.notifications.messages.event.event_text, "auto") == 0);

    const uint8_t variation[] = {0x99, 0x00, 0xfe};
    accept(receiver, model, variation, sizeof(variation), now_us);
    REQUIRE(model.ins.imu.magnetic_variation_deg.valid);
    NEAR(model.ins.imu.magnetic_variation_deg.value, -2.0f, 0.001f);

    REQUIRE(receiver.decoded_count() == 11);
    REQUIRE(receiver.unsupported_count() == 0);
    return 0;
}
