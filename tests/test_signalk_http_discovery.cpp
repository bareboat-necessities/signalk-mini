#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ArduinoJson.h>
#include <signalk_mini.hpp>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

namespace {

class Fd {
public:
    Fd() = default;
    explicit Fd(int fd) : fd_(fd) {}
    ~Fd() { if (fd_ >= 0) ::close(fd_); }
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    int get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }
private:
    int fd_ = -1;
};

uint16_t reserve_loopback_port() {
    Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
    REQUIRE(fd.valid());
    int one = 1;
    REQUIRE(::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    REQUIRE(::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    socklen_t len = sizeof(addr);
    REQUIRE(::getsockname(fd.get(), reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    return ntohs(addr.sin_port);
}

Fd connect_loopback(uint16_t port, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
        if (!fd.valid()) return {};
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);
        if (::connect(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) return fd;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return {};
}

bool send_all(int fd, const std::string& data) {
    size_t offset = 0;
    while (offset < data.size()) {
        const ssize_t n = ::send(fd, data.data() + offset, data.size() - offset, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        offset += static_cast<size_t>(n);
    }
    return true;
}

bool read_http_response(int fd, std::string& response, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        timeval tv{};
        tv.tv_usec = 50000;
        const int rc = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (rc == 0) continue;

        char chunk[512];
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return false;
        response.append(chunk, static_cast<size_t>(n));

        const size_t header_end = response.find("\r\n\r\n");
        if (header_end == std::string::npos) continue;
        const size_t length_pos = response.find("Content-Length:");
        REQUIRE(length_pos != std::string::npos && length_pos < header_end);
        size_t value = length_pos + std::strlen("Content-Length:");
        while (value < header_end && (response[value] == ' ' || response[value] == '\t')) ++value;
        size_t body_length = 0;
        while (value < header_end && response[value] >= '0' && response[value] <= '9') {
            body_length = body_length * 10u + static_cast<size_t>(response[value] - '0');
            ++value;
        }
        if (response.size() >= header_end + 4u + body_length) return true;
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
    config.identity.signalk_version = "1.8.2";
    config.identity.self = "vessels.urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555";
    config.signalk.host = "127.0.0.1";
    config.signalk.port = raw_port;
    config.signalk.websocket.enabled = true;
    config.signalk.websocket.host = "127.0.0.1";
    config.signalk.websocket.port = ws_port;

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
    char request[256];
    std::snprintf(request, sizeof(request),
                  "GET /signalk HTTP/1.1\r\nHost: 127.0.0.1:%u\r\nAccept: application/json\r\nConnection: close\r\n\r\n",
                  static_cast<unsigned>(ws_port));
    REQUIRE(send_all(client.get(), request));

    std::string response;
    REQUIRE(read_http_response(client.get(), response, 3000));
    REQUIRE(response.find("HTTP/1.1 200 OK\r\n") == 0);
    REQUIRE(response.find("Content-Type: application/json\r\n") != std::string::npos);

    const size_t body_pos = response.find("\r\n\r\n");
    REQUIRE(body_pos != std::string::npos);
    JsonDocument doc;
    REQUIRE(!deserializeJson(doc, response.substr(body_pos + 4)));

    char expected_ws[128];
    std::snprintf(expected_ws, sizeof(expected_ws),
                  "ws://127.0.0.1:%u/signalk/v1/stream",
                  static_cast<unsigned>(ws_port));
    REQUIRE(std::strcmp(doc["endpoints"]["v1"]["signalk-ws"] | "", expected_ws) == 0);
    REQUIRE(std::strcmp(doc["endpoints"]["v1"]["version"] | "", "1.8.2") == 0);
    REQUIRE(doc["endpoints"]["v1"]["signalk-http"].isNull());
    REQUIRE(std::strcmp(doc["server"]["id"] | "", "integration-signalk-server") == 0);
    REQUIRE(std::strcmp(doc["server"]["version"] | "", "0.1-integration") == 0);

    Fd root_client = connect_loopback(ws_port, 3000);
    REQUIRE(root_client.valid());
    char root_request[192];
    std::snprintf(root_request,
                  sizeof(root_request),
                  "GET / HTTP/1.1\r\nHost: 127.0.0.1:%u\r\nConnection: close\r\n\r\n",
                  static_cast<unsigned>(ws_port));
    REQUIRE(send_all(root_client.get(), root_request));

    std::string root_response;
    REQUIRE(read_http_response(root_client.get(), root_response, 3000));
    REQUIRE(root_response.find("HTTP/1.1 200 OK\r\n") == 0);
    REQUIRE(root_response.find("Content-Type: text/html; charset=utf-8\r\n") != std::string::npos);
    REQUIRE(root_response.find("<h1>SignalK Mini</h1>") != std::string::npos);
    REQUIRE(root_response.find("href=\"/signalk\"") != std::string::npos);

    running = false;
    loop_thread.join();
    return 0;
}
