#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ArduinoJson.h>
#include <signalk_mini.hpp>
#include <signalk_mini/websocket.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

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

bool send_all(int fd, const uint8_t* data, size_t size) {
    const uint8_t* ptr = data;
    size_t remaining = size;
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

bool send_all(int fd, const std::string& data) {
    return send_all(fd, reinterpret_cast<const uint8_t*>(data.data()), data.size());
}

std::vector<uint8_t> masked_client_text_frame(const char* text) {
    const size_t len = std::strlen(text);
    REQUIRE(len < 126);
    const uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
    std::vector<uint8_t> frame(6 + len);
    frame[0] = 0x81;
    frame[1] = 0x80 | static_cast<uint8_t>(len);
    std::memcpy(frame.data() + 2, mask, sizeof(mask));
    for (size_t i = 0; i < len; ++i) frame[6 + i] = static_cast<uint8_t>(text[i]) ^ mask[i & 3u];
    return frame;
}

bool wait_for_http_upgrade(int fd, std::vector<uint8_t>& leftover, int timeout_ms) {
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
        const size_t header_end = buffer.find("\r\n\r\n");
        if (header_end == std::string::npos) continue;
        const std::string header = buffer.substr(0, header_end + 4);
        REQUIRE(header.find("HTTP/1.1 101 Switching Protocols\r\n") == 0);
        REQUIRE(header.find("Upgrade: websocket\r\n") != std::string::npos);
        REQUIRE(header.find("Connection: Upgrade\r\n") != std::string::npos);
        REQUIRE(header.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n") != std::string::npos);
        const size_t rest = header_end + 4;
        leftover.assign(buffer.begin() + static_cast<std::string::difference_type>(rest), buffer.end());
        return true;
    }
    return false;
}

template<typename Predicate>
bool wait_for_ws_text(int fd, std::vector<uint8_t>& buffer, int timeout_ms, Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        while (!buffer.empty()) {
            uint8_t payload[signalk_mini::websocket::DefaultMaxPayloadSize + 1];
            signalk_mini::websocket::FrameView frame;
            size_t consumed = 0;
            const auto decoded = signalk_mini::websocket::decode_frame(buffer.data(), buffer.size(), payload, signalk_mini::websocket::DefaultMaxPayloadSize, frame, consumed, false);
            if (decoded == signalk_mini::websocket::FrameDecodeResult::NeedMore) break;
            REQUIRE(decoded == signalk_mini::websocket::FrameDecodeResult::Ok);
            REQUIRE(consumed > 0);
            buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::vector<uint8_t>::difference_type>(consumed));
            if (frame.opcode != signalk_mini::websocket::Opcode::Text) continue;
            REQUIRE(frame.payload_len < sizeof(payload));
            payload[frame.payload_len] = 0;
            if (predicate(reinterpret_cast<const char*>(payload))) return true;
        }

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
        uint8_t chunk[512];
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n <= 0) return false;
        buffer.insert(buffer.end(), chunk, chunk + n);
    }
    return false;
}

bool has_valid_hello(const char* text) {
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, text);
    REQUIRE(!err);
    REQUIRE(std::strcmp(doc["name"] | "", "integration-signalk-server") == 0);
    REQUIRE(std::strcmp(doc["version"] | "", "0.1-integration") == 0);
    REQUIRE(std::strcmp(doc["self"] | "", "vessels.self") == 0);
    JsonArray roles = doc["roles"].as<JsonArray>();
    REQUIRE(!roles.isNull());
    bool has_master = false;
    bool has_main = false;
    for (JsonVariant role : roles) {
        const char* value = role.as<const char*>();
        if (value && std::strcmp(value, "master") == 0) has_master = true;
        if (value && std::strcmp(value, "main") == 0) has_main = true;
    }
    REQUIRE(has_master);
    REQUIRE(has_main);
    return true;
}

bool has_clock_delta(const char* text) {
    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, text);
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
                return true;
            }
        }
    }
    return false;
}

} // namespace

int main() {
    const uint16_t raw_port = reserve_loopback_port();
    const uint16_t ws_port = reserve_loopback_port();

    signalk_mini::SignalKMiniConfig config;
    config.identity.server_name = "integration-signalk-server";
    config.identity.server_version = "0.1-integration";
    config.identity.self = "vessels.self";
    config.signalk.host = "127.0.0.1";
    config.signalk.port = raw_port;
    config.signalk.max_connections = 2;
    config.signalk.allow_rx = true;
    config.signalk.allow_tx = true;
    config.signalk.websocket.enabled = true;
    config.signalk.websocket.host = "127.0.0.1";
    config.signalk.websocket.port = ws_port;
    config.signalk.websocket.max_connections = 2;
    config.signalk.websocket.allow_rx = true;
    config.signalk.websocket.allow_tx = true;
    config.publisher.interval_us = 1000;
    config.publisher.source_label = "integration-test";
    config.connectors = nullptr;
    config.connector_count = 0;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());

    std::atomic<bool> running{true};
    std::thread loop_thread([&]() {
        while (running.load()) {
            app.tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    Fd client = connect_loopback(ws_port, 3000);
    REQUIRE(client.valid());

    const std::string request =
        "GET /signalk/v1/stream?subscribe=none HTTP/1.1\r\n"
        "Host: 127.0.0.1:3000\r\n"
        "Upgrade: websocket\r\n"
        "Connection: keep-alive, Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n";
    REQUIRE(send_all(client.get(), request));

    std::vector<uint8_t> ws_buffer;
    REQUIRE(wait_for_http_upgrade(client.get(), ws_buffer, 3000));
    REQUIRE(wait_for_ws_text(client.get(), ws_buffer, 3000, has_valid_hello));

    const std::vector<uint8_t> subscribe = masked_client_text_frame("{\"subscribe\":\"all\"}");
    REQUIRE(send_all(client.get(), subscribe.data(), subscribe.size()));
    REQUIRE(wait_for_ws_text(client.get(), ws_buffer, 3000, has_clock_delta));

    running = false;
    loop_thread.join();
    return 0;
}
