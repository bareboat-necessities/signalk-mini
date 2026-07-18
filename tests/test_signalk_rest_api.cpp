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
    size_t expected_size = 0;
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

        char chunk[257];
        const ssize_t n = ::recv(fd, chunk, sizeof(chunk), 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        if (n == 0) return expected_size != 0 && response.size() >= expected_size;
        response.append(chunk, static_cast<size_t>(n));

        const size_t header_end = response.find("\r\n\r\n");
        if (header_end == std::string::npos) continue;
        if (expected_size == 0) {
            const size_t length_pos = response.find("Content-Length:");
            REQUIRE(length_pos != std::string::npos && length_pos < header_end);
            size_t value = length_pos + std::strlen("Content-Length:");
            while (value < header_end && (response[value] == ' ' || response[value] == '\t')) ++value;
            size_t body_length = 0;
            while (value < header_end && response[value] >= '0' && response[value] <= '9') {
                body_length = body_length * 10u + static_cast<size_t>(response[value] - '0');
                ++value;
            }
            expected_size = header_end + 4u + body_length;
        }
        if (response.size() >= expected_size) return true;
    }
    return false;
}

struct HttpResponse {
    std::string raw;
    std::string headers;
    std::string body;
};

HttpResponse request(uint16_t port, const char* method, const char* path) {
    Fd client = connect_loopback(port, 3000);
    REQUIRE(client.valid());
    char request_text[768];
    const int request_len = std::snprintf(
        request_text,
        sizeof(request_text),
        "%s %s HTTP/1.1\r\nHost: 127.0.0.1:%u\r\nAccept: application/json\r\nConnection: close\r\n\r\n",
        method,
        path,
        static_cast<unsigned>(port));
    REQUIRE(request_len > 0 && static_cast<size_t>(request_len) < sizeof(request_text));
    REQUIRE(send_all(client.get(), std::string(request_text, static_cast<size_t>(request_len))));

    HttpResponse response;
    REQUIRE(read_http_response(client.get(), response.raw, 5000));
    const size_t split = response.raw.find("\r\n\r\n");
    REQUIRE(split != std::string::npos);
    response.headers = response.raw.substr(0, split + 2);
    response.body = response.raw.substr(split + 4);
    return response;
}

std::string sentence(const char* body) {
    char out[256];
    const uint8_t checksum = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((checksum >> 4) & 0x0f),
                  nmea0183_connector::to_hex(checksum & 0x0f));
    return std::string(out);
}

void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    now_us += 1000;
    const std::string line = sentence(body);
    REQUIRE(app.nmea0183().feed_line(line.c_str(), 0, now_us));
}

void seed_model(signalk_mini::SignalKMiniApp<float>& app) {
    uint64_t now_us = 1000000;
    feed(app, "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    feed(app, "GPRMC,123520,A,4807.038,N,01131.000,E,5.5,84.4,230394,3.1,W,A", now_us);
    feed(app, "GPHDT,123.4,T", now_us);
    feed(app, "GPHDM,120.0,M", now_us);
    feed(app, "GPHDG,121.0,1.0,E,3.1,W", now_us);
    feed(app, "IIMWV,45.0,R,12.3,N,A", now_us);
    feed(app, "IIMWV,225.0,T,10.2,N,A", now_us);
    feed(app, "IIVHW,123.4,T,120.0,M,6.2,N,11.5,K", now_us);
    feed(app, "SDDBT,32.8,f,10.0,M,5.5,F", now_us);
    feed(app, "SDDPT,10.0,0.5,20.0", now_us);
    feed(app, "YXMTW,18.7,C", now_us);
    feed(app, "IIVLW,123.4,N,4.5,N", now_us);
    feed(app, "TIROT,2.5,A", now_us);
    feed(app, "IIRSA,5.0,A,-4.0,A", now_us);
    feed(app, "GPRMB,A,0.10,R,FROM,TO,4807.000,N,01131.500,E,1.2,84.0,5.0,A,A", now_us);
    feed(app, "GPAPB,A,A,0.02,L,N,A,A,100.0,T,WP003,110.0,T,120.0,T", now_us);
}

} // namespace

int main() {
    const uint16_t raw_port = reserve_loopback_port();
    const uint16_t http_port = reserve_loopback_port();

    signalk_mini::SignalKMiniConfig config;
    config.identity.server_name = "rest-integration-server";
    config.identity.server_version = "0.1-rest";
    config.identity.signalk_version = "1.8.2";
    config.identity.self = "vessels.urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555";
    config.signalk.host = "127.0.0.1";
    config.signalk.port = raw_port;
    config.signalk.websocket.enabled = true;
    config.signalk.websocket.host = "127.0.0.1";
    config.signalk.websocket.port = http_port;
    config.connectors = nullptr;
    config.connector_count = 0;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    seed_model(app);

    std::atomic<bool> running{true};
    std::thread loop_thread([&]() {
        while (running.load()) {
            app.tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    const HttpResponse discovery = request(http_port, "GET", "/signalk");
    REQUIRE(discovery.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    JsonDocument discovery_doc;
    REQUIRE(!deserializeJson(discovery_doc, discovery.body));
    char expected_http[128];
    std::snprintf(expected_http, sizeof(expected_http),
                  "http://127.0.0.1:%u/signalk/v1/api/", static_cast<unsigned>(http_port));
    REQUIRE(std::strcmp(discovery_doc["endpoints"]["v1"]["signalk-http"] | "", expected_http) == 0);

    const HttpResponse redirect = request(http_port, "GET", "/signalk/v1/api");
    REQUIRE(redirect.raw.find("HTTP/1.1 308 Permanent Redirect\r\n") == 0);
    REQUIRE(redirect.headers.find("Location: /signalk/v1/api/\r\n") != std::string::npos);

    const HttpResponse root = request(http_port, "GET", "/signalk/v1/api/");
    REQUIRE(root.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    REQUIRE(root.headers.find("Content-Type: application/json\r\n") != std::string::npos);
    REQUIRE(root.body.size() > 1024);
    JsonDocument root_doc;
    REQUIRE(!deserializeJson(root_doc, root.body));
    REQUIRE(std::strcmp(root_doc["version"] | "", "1.8.2") == 0);
    REQUIRE(std::strcmp(root_doc["self"] | "", "urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555") == 0);
    REQUIRE(!root_doc["vessels"]["urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555"]["navigation"]["position"]["value"]["latitude"].isNull());

    const HttpResponse version = request(http_port, "GET", "/signalk/v1/api/version");
    REQUIRE(version.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    REQUIRE(version.body == "\"1.8.2\"");

    const HttpResponse self = request(http_port, "GET", "/signalk/v1/api/self");
    REQUIRE(self.body == "\"urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555\"");

    const HttpResponse position = request(http_port, "GET", "/signalk/v1/api/vessels/self/navigation/position");
    REQUIRE(position.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    JsonDocument position_doc;
    REQUIRE(!deserializeJson(position_doc, position.body));
    REQUIRE(!position_doc["value"]["latitude"].isNull());
    REQUIRE(!position_doc["value"]["longitude"].isNull());
    REQUIRE(std::strcmp(position_doc["$source"] | "", "signalk-mini") == 0);
    REQUIRE(position_doc["timestamp"].is<const char*>());

    const HttpResponse actual_position = request(
        http_port,
        "GET",
        "/signalk/v1/api/vessels/urn:mrn:signalk:uuid:11111111-2222-4333-8444-555555555555/navigation/position");
    REQUIRE(actual_position.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    JsonDocument actual_position_doc;
    REQUIRE(!deserializeJson(actual_position_doc, actual_position.body));
    REQUIRE(actual_position_doc["value"]["latitude"].as<double>() == position_doc["value"]["latitude"].as<double>());
    REQUIRE(actual_position_doc["value"]["longitude"].as<double>() == position_doc["value"]["longitude"].as<double>());
    REQUIRE(std::strcmp(actual_position_doc["$source"] | "", position_doc["$source"] | "") == 0);

    const HttpResponse navigation = request(http_port, "GET", "/signalk/v1/api/vessels/self/navigation");
    REQUIRE(navigation.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    JsonDocument navigation_doc;
    REQUIRE(!deserializeJson(navigation_doc, navigation.body));
    REQUIRE(!navigation_doc["position"].isNull());
    REQUIRE(!navigation_doc["speedOverGround"].isNull());

    const HttpResponse sources = request(http_port, "GET", "/signalk/v1/api/sources/signalk-mini");
    REQUIRE(sources.raw.find("HTTP/1.1 200 OK\r\n") == 0);
    JsonDocument sources_doc;
    REQUIRE(!deserializeJson(sources_doc, sources.body));
    REQUIRE(std::strcmp(sources_doc["type"] | "", "SignalK") == 0);

    const HttpResponse queried_position = request(http_port, "GET", "/signalk/v1/api/vessels/self/navigation/position?unused=true");
    REQUIRE(queried_position.raw.find("HTTP/1.1 200 OK\r\n") == 0);

    const HttpResponse missing = request(http_port, "GET", "/signalk/v1/api/vessels/self/navigation/notAPath");
    REQUIRE(missing.raw.find("HTTP/1.1 404 Not Found\r\n") == 0);

    const HttpResponse write_attempt = request(http_port, "POST", "/signalk/v1/api/vessels/self/navigation/position");
    REQUIRE(write_attempt.raw.find("HTTP/1.1 405 Method Not Allowed\r\n") == 0);
    REQUIRE(write_attempt.headers.find("Allow: GET\r\n") != std::string::npos);

    const HttpResponse root_page = request(http_port, "GET", "/");
    REQUIRE(root_page.body.find("href=\"/signalk/v1/api/\"") != std::string::npos);

    running = false;
    loop_thread.join();
    return 0;
}
