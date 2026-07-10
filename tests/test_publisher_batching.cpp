#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <ArduinoJson.h>
#include <signalk_mini.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

class CaptureConnection final : public async_event_loop::ITcpConnection {
public:
    int read(uint8_t*, size_t) override { return 0; }

    int write(const uint8_t* src, size_t len) override {
        REQUIRE(src != nullptr);
        REQUIRE(size_ + len < sizeof(buffer_));
        std::memcpy(buffer_ + size_, src, len);
        size_ += len;
        buffer_[size_] = '\0';
        ++write_count_;
        return static_cast<int>(len);
    }

    bool valid() const override { return valid_; }
    void close() override { valid_ = false; }
    const async_event_loop::TcpPeerInfo& peer() const override { return peer_; }
    size_t input_size() const override { return 0; }
    size_t output_size() const override { return 0; }
    bool peek(uint8_t*, size_t) override { return false; }
    bool read_exact(uint8_t*, size_t) override { return false; }
    bool read_line(char*, size_t, bool = true) override { return false; }

    const char* text() const { return buffer_; }
    size_t write_count() const { return write_count_; }

private:
    async_event_loop::TcpPeerInfo peer_{};
    char buffer_[1024]{};
    size_t size_ = 0;
    size_t write_count_ = 0;
    bool valid_ = true;
};

struct SingleConnectionRegistry {
    explicit SingleConnectionRegistry(CaptureConnection& connection_ref) : connection(connection_ref) {}

    template<typename Fn>
    void for_each_tx(Fn fn) { fn(connection); }

    void write_signal_k_delta(const char* json, size_t len) {
        connection.write(reinterpret_cast<const uint8_t*>(json), len);
    }

    CaptureConnection& connection;
};

static size_t count_substring(const char* haystack, const char* needle) {
    size_t count = 0;
    const char* p = haystack;
    while (p && *p) {
        p = std::strstr(p, needle);
        if (!p) break;
        ++count;
        p += std::strlen(needle);
    }
    return count;
}

int main() {
    signalk_mini::ConnectorConfig connector;
    connector.label = "nmea-source";

    signalk_mini::SignalKMiniConfig config;
    config.identity.self = "vessels.urn:mrn:imo:mmsi:123456789";
    config.publisher.max_changes_per_tick = 8;
    config.publisher.json_buffer_size = 512;
    config.connectors = &connector;
    config.connector_count = 1;

    signalk_mini::ModelStore<float> store;
    store.model().wind.apparent.direction_deg.set(45.0f, 1000);
    store.model().wind.apparent.speed_kn.set(12.0f, 1000);
    store.mark_changed(signalk_mini::ModelField::WindApparentDirectionDeg, 10, 1000);
    store.mark_changed(signalk_mini::ModelField::WindApparentSpeedKn, 10, 1000);

    signalk_mini::SignalKPublisher<float, 512, 4> publisher(store, config);
    CaptureConnection connection;
    SingleConnectionRegistry registry(connection);

    publisher.publish_pending(registry);

    REQUIRE(connection.write_count() == 1);
    REQUIRE(publisher.published_delta_count() == 1);
    REQUIRE(publisher.published_value_count() == 2);
    REQUIRE(publisher.dropped_publish_count() == 0);
    REQUIRE(count_substring(connection.text(), "\"path\"") == 2);
    REQUIRE(std::strstr(connection.text(), "\"source\":{\"label\":\"nmea-source\"}") != nullptr);
    REQUIRE(std::strstr(connection.text(), "environment.wind.angleApparent") != nullptr);
    REQUIRE(std::strstr(connection.text(), "environment.wind.speedApparent") != nullptr);

    JsonDocument doc;
    REQUIRE(!deserializeJson(doc, connection.text()));
    REQUIRE(std::strcmp(doc["context"] | "", "vessels.urn:mrn:imo:mmsi:123456789") == 0);
    const char* timestamp = doc["updates"][0]["timestamp"].as<const char*>();
    REQUIRE(timestamp != nullptr);
    REQUIRE(std::strlen(timestamp) == 24);
    REQUIRE(timestamp[4] == '-' && timestamp[7] == '-' && timestamp[10] == 'T');
    REQUIRE(timestamp[23] == 'Z');

    return 0;
}
