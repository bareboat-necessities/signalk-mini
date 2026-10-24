#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <ArduinoJson.h>
#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

// Every delta the server sends must fit the embedded (ESP32) JSON buffer, so
// that each one also fits a client's transmit buffer as a whole frame.
static constexpr size_t EmbeddedBuffer = 384;
using EmbeddedPublisher = signalk_mini::SignalKPublisher<float, EmbeddedBuffer, 32>;

namespace {

struct CollectingSink {
    std::vector<std::string> deltas;
    void write_signal_k_delta(const char* json, size_t len) { deltas.emplace_back(json, len); }
};

std::set<std::string> check_deltas(const std::vector<std::string>& deltas) {
    std::set<std::string> paths;
    for (const std::string& delta : deltas) {
        REQUIRE(delta.size() < EmbeddedBuffer);
        JsonDocument doc;
        REQUIRE(!deserializeJson(doc, delta));
        for (JsonVariantConst value : doc["updates"][0]["values"].as<JsonArrayConst>()) {
            REQUIRE(!value["value"].isNull());
            paths.insert(value["path"] | "");
        }
    }
    return paths;
}

signalk_mini::SignalKMiniConfig embedded_config() {
    signalk_mini::SignalKMiniConfig config;
    config.publisher.json_buffer_size = EmbeddedBuffer;
    config.publisher.max_changes_per_tick = 1024;
    return config;
}

std::filesystem::path fixture_dir() {
    return std::filesystem::path(__FILE__).parent_path() / "fixtures" / "nmea";
}

void test_real_fixtures_fit_embedded_buffer() {
    std::vector<std::filesystem::path> files;
    for (const auto& entry : std::filesystem::directory_iterator(fixture_dir())) {
        if (entry.is_regular_file() && entry.path().extension() == ".nmea") files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    REQUIRE(!files.empty());

    for (const auto& file : files) {
        const auto config = embedded_config();
        signalk_mini::ModelStore<float> store;
        signalk_mini::Nmea0183Input<float> input(store);
        EmbeddedPublisher publisher(store, config);
        CollectingSink sink;

        std::ifstream in(file);
        std::string line;
        uint64_t now_us = 1000000ULL;
        while (std::getline(in, line)) {
            const auto start = line.find_first_of("$!");
            if (start == std::string::npos) continue;
            line.erase(0, start);
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
            now_us += 1000;
            input.feed_line(line.c_str(), 7, now_us, false);
            publisher.publish_pending(sink);
        }
        CollectingSink snapshot;
        REQUIRE(publisher.publish_current(snapshot, now_us, true));

        check_deltas(sink.deltas);
        check_deltas(snapshot.deltas);
        if (publisher.dropped_publish_count() != 0) {
            std::fprintf(stderr, "%s dropped %llu values\n", file.filename().string().c_str(),
                         static_cast<unsigned long long>(publisher.dropped_publish_count()));
        }
        REQUIRE(publisher.dropped_publish_count() == 0);
    }
}

template<size_t N>
void fill_escaped(char (&text)[N]) {
    // '"' and '\\' each serialize as two bytes: the worst printable expansion.
    for (size_t i = 0; i + 1 < N; ++i) text[i] = (i % 2) ? '"' : '\\';
    text[N - 1] = '\0';
}

void test_long_text_objects_are_split_not_dropped() {
    const auto config = embedded_config();
    signalk_mini::ModelStore<float> store;
    auto& model = store.model();
    const uint64_t now_us = 2000000ULL;

    auto& inmarsat = model.notifications.inmarsat.safetynet.latest_message;
    fill_escaped(inmarsat.message_id);
    fill_escaped(inmarsat.message_text);
    inmarsat.last_update_us = now_us;
    store.mark_changed(signalk_mini::ModelField::InmarsatSafetyNetStructuredNotification, 7, now_us);

    auto& navtex = model.notifications.navtex.received;
    fill_escaped(navtex.body_text);
    fill_escaped(navtex.subject_label);
    navtex.first_seen_us = now_us;
    navtex.last_update_us = now_us;
    store.mark_changed(signalk_mini::ModelField::NavtexStructuredNotification, 7, now_us);

    auto& alerts = model.notifications.alerts;
    fill_escaped(alerts.alert_report.alert_text);
    alerts.alert_report.last_update_us = now_us;
    fill_escaped(alerts.condition.description);
    alerts.condition.last_update_us = now_us;
    fill_escaped(alerts.command_refused.reason_text);
    alerts.command_refused.last_update_us = now_us;
    store.mark_changed(signalk_mini::ModelField::AlertStructuredNotification, 7, now_us);

    EmbeddedPublisher publisher(store, config);
    CollectingSink sink;
    publisher.publish_pending(sink);

    const auto paths = check_deltas(sink.deltas);
    REQUIRE(publisher.dropped_publish_count() == 0);
    REQUIRE(publisher.truncated_value_count() > 0);
    // Composite objects that cannot fit whole go out as members under sub-paths.
    REQUIRE(paths.count("notifications.alerts.report.text") == 1 || paths.count("notifications.alerts.report") == 1);
    REQUIRE(paths.count("notifications.alerts.condition.description") == 1 || paths.count("notifications.alerts.condition") == 1);
    bool inmarsat_text = false;
    for (const auto& path : paths) {
        if (path.rfind("notifications.inmarsatSafetyNet", 0) == 0 || path.rfind(signalk_mini::signalk_path::NotificationsInmarsatSafetyNet, 0) == 0) {
            inmarsat_text = true;
        }
    }
    REQUIRE(inmarsat_text);
}

void test_text_value_is_truncated_on_utf8_boundary() {
    signalk_mini::SignalKMiniConfig config = embedded_config();
    config.publisher.json_buffer_size = 280;
    signalk_mini::ModelStore<float> store;
    auto& event = store.model().notifications.messages.event;
    // Two-byte UTF-8 characters: a cut must never split one.
    for (size_t i = 0; i + 2 < sizeof(event.event_text); i += 2) {
        event.event_text[i] = static_cast<char>(0xC3);
        event.event_text[i + 1] = static_cast<char>(0xA9);
    }
    event.last_update_us = 1000000ULL;
    store.mark_changed(signalk_mini::ModelField::NotificationEvent, 7, 1000000ULL);

    signalk_mini::SignalKPublisher<float, 280, 32> publisher(store, config);
    CollectingSink sink;
    publisher.publish_pending(sink);
    REQUIRE(publisher.dropped_publish_count() == 0);
    REQUIRE(publisher.truncated_value_count() == 1);
    REQUIRE(sink.deltas.size() == 1);
    JsonDocument doc;
    REQUIRE(!deserializeJson(doc, sink.deltas[0]));
    const char* text = doc["updates"][0]["values"][0]["value"] | "";
    const size_t len = std::strlen(text);
    REQUIRE(len > 0 && len < std::strlen(event.event_text));
    REQUIRE(len % 2 == 0);
}

} // namespace

int main() {
    test_real_fixtures_fit_embedded_buffer();
    test_long_text_objects_are_split_not_dropped();
    test_text_value_is_truncated_on_utf8_boundary();
    std::puts("test_publisher_worst_case passed");
    return 0;
}
