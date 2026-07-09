#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ArduinoJson.h>
#include <nmea0183_connector.hpp>
#include <signalk_mini.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d: got %.9f expected %.9f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

namespace {

class Fd {
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    ~Fd() { reset(); }
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    Fd& operator=(Fd&& other) noexcept {
        if (this != &other) {
            reset();
            fd_ = other.fd_;
            other.fd_ = -1;
        }
        return *this;
    }
    int get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }
    void reset(int fd = -1) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = fd;
    }
private:
    int fd_ = -1;
};

class TcpListener {
public:
    bool start() {
        Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
        if (!fd.valid()) return false;
        int one = 1;
        (void)::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        if (::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return false;
        if (::listen(fd.get(), 4) != 0) return false;

        socklen_t len = sizeof(addr);
        if (::getsockname(fd.get(), reinterpret_cast<sockaddr*>(&addr), &len) != 0) return false;
        port_ = ntohs(addr.sin_port);
        fd_ = std::move(fd);
        return true;
    }

    uint16_t port() const { return port_; }

    Fd accept_with_timeout(int timeout_ms) const {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_.get(), &rfds);
        timeval tv{};
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        const int rc = ::select(fd_.get() + 1, &rfds, nullptr, nullptr, &tv);
        if (rc <= 0) return Fd{};
        return Fd(::accept(fd_.get(), nullptr, nullptr));
    }

private:
    Fd fd_;
    uint16_t port_ = 0;
};

uint16_t reserve_loopback_port() {
    TcpListener listener;
    REQUIRE(listener.start());
    return listener.port();
}

Fd connect_loopback(uint16_t port, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
        if (!fd.valid()) return Fd{};
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        if (::connect(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) return fd;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return Fd{};
}

bool send_all(int fd, const std::string& data) {
    const char* ptr = data.data();
    size_t remaining = data.size();
    while (remaining > 0) {
        const ssize_t n = ::send(fd, ptr, remaining, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        ptr += n;
        remaining -= static_cast<size_t>(n);
    }
    return true;
}

std::string nmea_sentence(const char* body) {
    char out[192];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c\r\n", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f),
                  nmea0183_connector::to_hex(cs & 0x0f));
    return std::string(out);
}

bool line_has_valid_hello(const std::string& line) {
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, line);
    REQUIRE(!err);

    REQUIRE(std::strcmp(doc["name"] | "", "integration-signalk-server") == 0);
    REQUIRE(std::strcmp(doc["version"] | "", "0.1-integration") == 0);
    REQUIRE(std::strcmp(doc["self"] | "", "vessels.self") == 0);

    JsonArray roles = doc["roles"].as<JsonArray>();
    REQUIRE(!roles.isNull());
    bool has_master = false;
    bool has_main = false;
    for (JsonVariant role : roles) {
        const char* text = role.as<const char*>();
        if (text && std::strcmp(text, "master") == 0) has_master = true;
        if (text && std::strcmp(text, "main") == 0) has_main = true;
    }
    REQUIRE(has_master);
    REQUIRE(has_main);
    return true;
}

bool line_has_valid_clock_delta(const std::string& line) {
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, line);
    REQUIRE(!err);

    JsonArray updates = doc["updates"].as<JsonArray>();
    if (updates.isNull()) return false;
    for (JsonObject update : updates) {
        JsonArray values = update["values"].as<JsonArray>();
        if (values.isNull()) continue;
        for (JsonObject value : values) {
            const char* path = value["path"] | "";
            if (std::strcmp(path, "communication.server.clock") == 0) {
                REQUIRE(value["value"].is<double>() || value["value"].is<float>() || value["value"].is<int>());
                const double clock_s = value["value"] | 0.0;
                REQUIRE(clock_s > 0.0);
                return true;
            }
        }
    }
    return false;
}

bool line_has_valid_wind_speed_delta(const std::string& line) {
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, line);
    REQUIRE(!err);

    JsonArray updates = doc["updates"].as<JsonArray>();
    if (updates.isNull()) return false;

    for (JsonObject update : updates) {
        JsonObject source = update["source"].as<JsonObject>();
        JsonArray values = update["values"].as<JsonArray>();
        if (values.isNull()) continue;
        for (JsonObject value : values) {
            const char* path = value["path"] | "";
            if (std::strcmp(path, "environment.wind.speedApparent") != 0) continue;
            REQUIRE(!source.isNull());
            const char* label = source["label"] | "";
            REQUIRE(std::strcmp(label, "integration-nmea0183-tcp-client") == 0);
            REQUIRE(value["value"].is<double>() || value["value"].is<float>() || value["value"].is<int>());
            const double wind_mps = value["value"] | -1.0;
            NEAR(wind_mps, 6.5848889, 0.001);
            return true;
        }
    }
    return false;
}

template<typename Predicate>
bool wait_for_signal_k_line(int fd, int timeout_ms, Predicate predicate) {
    std::string buffer;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        timeval tv{};
        tv.tv_sec = 0;
        tv.tv_usec = 50000;
        const int rc = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (rc == 0) continue;
        char chunk[512];
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0) return false;
        buffer.append(chunk, static_cast<size_t>(n));
        while (true) {
            const size_t newline = buffer.find('\n');
            if (newline == std::string::npos) break;
            std::string line = buffer.substr(0, newline);
            buffer.erase(0, newline + 1);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            if (predicate(line)) return true;
        }
    }
    return false;
}

bool wait_for_signal_k_hello(int fd, int timeout_ms) {
    return wait_for_signal_k_line(fd, timeout_ms, line_has_valid_hello);
}

bool wait_for_signal_k_clock_delta(int fd, int timeout_ms) {
    return wait_for_signal_k_line(fd, timeout_ms, line_has_valid_clock_delta);
}

bool wait_for_signal_k_wind_delta(int fd, int timeout_ms) {
    return wait_for_signal_k_line(fd, timeout_ms, line_has_valid_wind_speed_delta);
}

} // namespace

int main() {
    TcpListener nmea_source;
    REQUIRE(nmea_source.start());
    const uint16_t signalk_port = reserve_loopback_port();

    signalk_mini::ConnectorConfig nmea_connector;
    nmea_connector.enabled = true;
    nmea_connector.label = "integration-nmea0183-tcp-client";
    nmea_connector.access.allow_rx = true;
    nmea_connector.access.allow_tx = false;
    nmea_connector.protocol.kind = signalk_mini::ConnectorProtocol::Nmea0183;
    nmea_connector.protocol.nmea0183.validate_checksum = false;
    nmea_connector.protocol.nmea0183.validate_checksum_configured = true;
    nmea_connector.transport.kind = signalk_mini::ConnectorTransport::TcpClient;
    nmea_connector.transport.tcp_client.host = "127.0.0.1";
    nmea_connector.transport.tcp_client.port = nmea_source.port();

    signalk_mini::SignalKMiniConfig config;
    config.identity.server_name = "integration-signalk-server";
    config.identity.server_version = "0.1-integration";
    config.identity.self = "vessels.self";
    config.signalk.host = "127.0.0.1";
    config.signalk.port = signalk_port;
    config.signalk.allow_rx = true;
    config.signalk.allow_tx = true;
    config.signalk.websocket.enabled = false;
    config.publisher.interval_us = 1000;
    config.publisher.source_label = "integration-test";
    config.connectors = &nmea_connector;
    config.connector_count = 1;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());

    std::atomic<bool> running{true};
    std::thread loop_thread([&]() {
        while (running.load()) {
            app.tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    Fd nmea_connection = nmea_source.accept_with_timeout(3000);
    REQUIRE(nmea_connection.valid());

    Fd signalk_client = connect_loopback(signalk_port, 3000);
    REQUIRE(signalk_client.valid());
    REQUIRE(wait_for_signal_k_hello(signalk_client.get(), 3000));
    REQUIRE(send_all(signalk_client.get(), "{\"subscribe\":\"all\"}\r\n"));
    REQUIRE(wait_for_signal_k_clock_delta(signalk_client.get(), 3000));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const std::string mwv = nmea_sentence("IIMWV,045.0,R,12.8,N,A");
    REQUIRE(send_all(nmea_connection.get(), mwv));
    REQUIRE(wait_for_signal_k_wind_delta(signalk_client.get(), 3000));

    running = false;
    loop_thread.join();
    return 0;
}
