#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d\n", __FILE__, __LINE__); std::exit(2); } } while (0)

static bool pop_path(signalk_mini::ModelStore<float>& store, const char* path, signalk_mini::SignalKMappedValue<float>& found) {
    signalk_mini::SignalKMapper<float> mapper;
    signalk_mini::ModelChange change;
    bool ok = false;
    while (store.changes().pop(change)) {
        signalk_mini::SignalKMappedValue<float> mapped;
        if (!mapper.map_change(store.model(), change, mapped) || !mapped.path) continue;
        if (std::strcmp(mapped.path, path) == 0) {
            found = mapped;
            ok = true;
        }
    }
    return ok;
}

int main() {
    signalk_mini::ModelStore<float> store;
    signalk_mini::SeaTalkInput<float> input(store);
    signalk_mini::SignalKMappedValue<float> mapped;
    uint64_t now_us = 1000;

    const uint8_t engine[] = {0x05, 0x03, 0x00, 0x09, 0xc4, 0x0a};
    REQUIRE(input.feed_datagram(engine, sizeof(engine), 1, now_us));
    REQUIRE(pop_path(store, "propulsion.main.revolutions", mapped));
    NEAR(mapped.number, 2500.0f / 60.0f, 0.001f);

    const uint8_t trip_total[] = {0x25, 0x04, 0x39, 0x30, 0x85, 0x1a, 0x00};
    now_us += 1000;
    REQUIRE(input.feed_datagram(trip_total, sizeof(trip_total), 1, now_us));
    REQUIRE(pop_path(store, "navigation.log", mapped));

    const uint8_t st_state[] = {0x84, 0x16, 0x40, 0x00, 0x42, 0x00, 0xfe, 0x01, 0x08};
    now_us += 1000;
    REQUIRE(input.feed_datagram(st_state, sizeof(st_state), 1, now_us));
    REQUIRE(pop_path(store, "steering.autopilot.state", mapped));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Text);
    REQUIRE(std::strcmp(mapped.text, "auto") == 0);

    const uint8_t nav_to_wp[] = {0x85, 0xb5, 0x07, 0x01, 0x00, 0x05, 0x07, 0x00};
    now_us += 1000;
    REQUIRE(input.feed_datagram(nav_to_wp, sizeof(nav_to_wp), 1, now_us));
    REQUIRE(pop_path(store, "navigation.courseGreatCircle.crossTrackError", mapped));

    const uint8_t arrival[] = {0xa2, 0x64, 0x00, 'W', 'P', '0', '1'};
    now_us += 1000;
    REQUIRE(input.feed_datagram(arrival, sizeof(arrival), 1, now_us));
    REQUIRE(pop_path(store, "navigation.courseGreatCircle.arrivalCircleEntered", mapped));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Bool);

    const uint8_t display_units[] = {0x24, 0x02, 0x00, 0x00, 0x86};
    now_us += 1000;
    REQUIRE(input.feed_datagram(display_units, sizeof(display_units), 1, now_us));
    REQUIRE(pop_path(store, "notifications.message.text", mapped));
    REQUIRE(mapped.kind == signalk_mini::SignalKMappedValueKind::Text);

    return 0;
}
