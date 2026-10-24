#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include <ArduinoJson.h>
#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

namespace {

const char* const Feed[] = {
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
    "$IIMWV,045.0,R,10.5,N,A*1F",
    "$IIDBT,10.0,f,3.0,M,1.6,F*2B",
    "$IIVHW,,,090.0,M,5.0,N,9.3,K*55",
};

struct CollectingSink {
    std::vector<std::string> deltas;
    size_t accept_limit = SIZE_MAX;

    bool write_signal_k_delta(const char* json, size_t len) {
        if (deltas.size() >= accept_limit) return false;
        deltas.emplace_back(json, len);
        return true;
    }
};

void feed(signalk_mini::ModelStore<float>& store) {
    signalk_mini::Nmea0183Input<float> input(store);
    for (const char* line : Feed) REQUIRE(input.feed_line(line, 7, 1000000ULL, false));
}

// Collects "context|path" for every value, checking each delta is valid JSON and fits.
std::set<std::string> paths_of(const std::vector<std::string>& deltas, size_t max_len) {
    std::set<std::string> paths;
    for (const std::string& delta : deltas) {
        REQUIRE(delta.size() < max_len);
        REQUIRE(delta.size() >= 2 && delta.compare(delta.size() - 2, 2, "\r\n") == 0);
        JsonDocument doc;
        REQUIRE(!deserializeJson(doc, delta));
        const char* context = doc["context"] | "";
        const JsonArrayConst values = doc["updates"][0]["values"];
        for (JsonVariantConst value : values) {
            REQUIRE(!value["value"].isNull());
            paths.insert(std::string(context) + "|" + (value["path"] | ""));
        }
    }
    return paths;
}

signalk_mini::SignalKMiniConfig config_with_buffer(size_t size) {
    signalk_mini::SignalKMiniConfig config;
    config.publisher.json_buffer_size = static_cast<uint16_t>(size);
    config.publisher.max_changes_per_tick = 256;
    return config;
}

void test_small_buffer_splits_without_dropping() {
    const auto large_config = config_with_buffer(4096);
    signalk_mini::ModelStore<float> large_store;
    feed(large_store);
    signalk_mini::SignalKPublisher<float, 4096, 32> large(large_store, large_config);
    CollectingSink large_sink;
    large.publish_pending(large_sink);
    REQUIRE(large.dropped_publish_count() == 0);
    const auto expected = paths_of(large_sink.deltas, 4096);
    REQUIRE(expected.size() >= 8);

    // The embedded profile buffer: before splitting, this feed lost whole batches.
    const auto small_config = config_with_buffer(384);
    signalk_mini::ModelStore<float> small_store;
    feed(small_store);
    signalk_mini::SignalKPublisher<float, 384, 32> small(small_store, small_config);
    CollectingSink small_sink;
    small.publish_pending(small_sink);
    REQUIRE(small.dropped_publish_count() == 0);
    REQUIRE(small_sink.deltas.size() > large_sink.deltas.size());
    REQUIRE(paths_of(small_sink.deltas, 384) == expected);
    REQUIRE(small.published_value_count() == large.published_value_count());
}

void test_snapshot_resumes_from_cursor() {
    const auto config = config_with_buffer(384);
    signalk_mini::ModelStore<float> store;
    feed(store);
    signalk_mini::SignalKPublisher<float, 384, 32> publisher(store, config);

    CollectingSink full;
    REQUIRE(publisher.publish_current(full, 2000000ULL, true));
    const auto expected = paths_of(full.deltas, 384);
    REQUIRE(full.deltas.size() >= 3);

    // A sink that accepts one delta per call must receive the same values, in order, across calls.
    std::vector<std::string> resumed;
    signalk_mini::SignalKSnapshotCursor cursor;
    size_t calls = 0;
    for (;;) {
        CollectingSink one;
        one.accept_limit = 1;
        const bool done = publisher.publish_current(one, 2000000ULL, true, &cursor);
        resumed.insert(resumed.end(), one.deltas.begin(), one.deltas.end());
        ++calls;
        REQUIRE(calls < 64);
        if (done) break;
        REQUIRE(cursor.started());
    }
    REQUIRE(!cursor.started());
    REQUIRE(calls >= full.deltas.size());
    REQUIRE(paths_of(resumed, 384) == expected);
}

template<size_t N>
void fill_text(char (&text)[N], char c) {
    for (size_t i = 0; i + 1 < N; ++i) text[i] = c;
    text[N - 1] = '\0';
}

// A composite object split into several deltas must resume inside the object:
// a client that takes one delta per pass still gets every member.
void test_snapshot_resumes_inside_split_object() {
    const auto config = config_with_buffer(384);
    signalk_mini::ModelStore<float> store;
    auto& alerts = store.model().notifications.alerts;
    const uint64_t now_us = 1000000ULL;
    fill_text(alerts.alert_report.alert_text, 'r');
    alerts.alert_report.last_update_us = now_us;
    fill_text(alerts.condition.description, 'c');
    alerts.condition.last_update_us = now_us;
    fill_text(alerts.command_refused.reason_text, 'x');
    alerts.command_refused.last_update_us = now_us;
    fill_text(alerts.alarm_state.description, 'a');
    alerts.alarm_state.last_update_us = now_us;
    store.mark_changed(signalk_mini::ModelField::AlertStructuredNotification, 7, now_us);
    signalk_mini::SignalKPublisher<float, 384, 32> publisher(store, config);

    CollectingSink full;
    REQUIRE(publisher.publish_current(full, 2000000ULL, true));
    REQUIRE(full.deltas.size() >= 3);
    const auto expected = paths_of(full.deltas, 384);

    std::vector<std::string> resumed;
    signalk_mini::SignalKSnapshotCursor cursor;
    size_t calls = 0;
    for (;;) {
        CollectingSink one;
        one.accept_limit = 1;
        const bool done = publisher.publish_current(one, 2000000ULL, true, &cursor);
        resumed.insert(resumed.end(), one.deltas.begin(), one.deltas.end());
        ++calls;
        REQUIRE(calls < 64);
        if (done) break;
    }
    REQUIRE(paths_of(resumed, 384) == expected);
    REQUIRE(resumed.size() == full.deltas.size());
}

void test_header_that_cannot_fit_is_counted() {
    // 128 bytes cannot hold the delta header, so nothing can be sent.
    const auto config = config_with_buffer(128);
    signalk_mini::ModelStore<float> store;
    store.model().wind.apparent.speed_kn.set(12.0f, 1000000ULL);
    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 7, 1000000ULL);
    signalk_mini::SignalKPublisher<float, 128, 32> publisher(store, config);
    CollectingSink sink;
    publisher.publish_pending(sink);
    REQUIRE(sink.deltas.empty());
    REQUIRE(publisher.dropped_publish_count() == 1);
}

} // namespace

int main() {
    test_small_buffer_splits_without_dropping();
    test_snapshot_resumes_from_cursor();
    test_snapshot_resumes_inside_split_object();
    test_header_that_cannot_fit_is_counted();
    std::puts("test_publisher_split passed");
    return 0;
}
