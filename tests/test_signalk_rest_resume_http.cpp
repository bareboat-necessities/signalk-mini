#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ArduinoJson.h>
#include <nmea0183_connector.hpp>
#include <signalk_mini.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

// A REST response larger than the per-client queue limit is resumed from the
// typed model while the client reads slowly. Single-threaded: the test drives
// app.tick() between socket reads.
namespace {

using Clock = std::chrono::steady_clock;

uint16_t free_port() {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd >= 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    REQUIRE(::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    socklen_t len = sizeof(addr);
    REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    ::close(fd);
    return ntohs(addr.sin_port);
}

int connect_client(uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd >= 0);
    int small = 1024;
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    REQUIRE(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    REQUIRE(::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK) == 0);
    return fd;
}

void feed(signalk_mini::SignalKMiniApp<float>& app, const char* body, uint64_t& now_us) {
    char line[160];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(line, sizeof(line), "$%s*%c%c", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    REQUIRE(app.nmea0183().feed_line(line, 0, now_us += 1000));
}

void seed(signalk_mini::SignalKMiniApp<float>& app) {
    uint64_t now_us = 1000000;
    feed(app, "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,10.0,M,46.9,M,,", now_us);
    feed(app, "GPRMC,123520,A,4807.038,N,01131.000,E,5.5,84.4,230394,3.1,W,A", now_us);
    feed(app, "GPHDT,123.4,T", now_us);
    feed(app, "IIMWV,45.0,R,12.3,N,A", now_us);
    feed(app, "IIMWV,225.0,T,10.2,N,A", now_us);
    feed(app, "IIVHW,123.4,T,120.0,M,6.2,N,11.5,K", now_us);
    feed(app, "SDDBT,32.8,f,10.0,M,5.5,F", now_us);
    feed(app, "YXMTW,18.7,C", now_us);
    feed(app, "GPRMB,A,0.10,R,FROM,TO,4807.000,N,01131.500,E,1.2,84.0,5.0,A,A", now_us);
}

// Reads at most `max_bytes` per call; returns false on EOF.
bool read_some(int fd, std::string& out, size_t max_bytes) {
    char buf[512];
    while (max_bytes > 0) {
        const ssize_t n = ::recv(fd, buf, max_bytes < sizeof(buf) ? max_bytes : sizeof(buf), 0);
        if (n > 0) {
            out.append(buf, static_cast<size_t>(n));
            max_bytes -= static_cast<size_t>(n);
            continue;
        }
        if (n == 0) return false;
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }
    return true;
}

std::string get(signalk_mini::SignalKMiniApp<float>& app, uint16_t port, const char* path, size_t bytes_per_tick) {
    const int client = connect_client(port);
    char request[256];
    const int len = std::snprintf(request, sizeof(request), "GET %s HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n", path);
    REQUIRE(::send(client, request, static_cast<size_t>(len), MSG_NOSIGNAL) == len);
    std::string response;
    bool open = true;
    const auto deadline = Clock::now() + std::chrono::seconds(10);
    while (open && Clock::now() < deadline) {
        app.tick();
        open = read_some(client, response, bytes_per_tick);
    }
    ::close(client);
    REQUIRE(!open);
    return response;
}

void collect_keys(JsonVariantConst value, const std::string& prefix, std::set<std::string>& keys) {
    if (!value.is<JsonObjectConst>()) return;
    for (JsonPairConst member : value.as<JsonObjectConst>()) {
        const std::string key = prefix + "/" + member.key().c_str();
        keys.insert(key);
        collect_keys(member.value(), key, keys);
    }
}

} // namespace

int main() {
    signalk_mini::SignalKMiniConfig config;
    config.signalk.host = "127.0.0.1";
    config.signalk.port = free_port();
    config.signalk.websocket.host = "127.0.0.1";
    config.signalk.websocket.port = free_port();
    config.signalk.max_client_queue_bytes = 1100;
    config.publisher.interval_us = 1000;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    seed(app);

    std::vector<char> reference(64 * 1024);
    const int reference_len = app.write_full_model(reference.data(), reference.size());
    REQUIRE(reference_len > 2000);

    // Small responses still get a Content-Length.
    const std::string version = get(app, config.signalk.websocket.port, "/signalk/v1/api/version", 4096);
    REQUIRE(version.find("Content-Length:") != std::string::npos);

    // The full model exceeds the queue limit: it is resumed, without Content-Length.
    const std::string full = get(app, config.signalk.websocket.port, "/signalk/v1/api/", 200);
    const size_t split = full.find("\r\n\r\n");
    REQUIRE(split != std::string::npos);
    const std::string headers = full.substr(0, split);
    REQUIRE(headers.find("HTTP/1.1 200 OK") == 0);
    REQUIRE(headers.find("Content-Length:") == std::string::npos);
    JsonDocument body;
    REQUIRE(!deserializeJson(body, full.substr(split + 4)));
    JsonDocument expected;
    REQUIRE(!deserializeJson(expected, std::string(reference.data(), static_cast<size_t>(reference_len))));
    std::set<std::string> body_keys;
    std::set<std::string> expected_keys;
    collect_keys(body.as<JsonVariantConst>(), "", body_keys);
    collect_keys(expected.as<JsonVariantConst>(), "", expected_keys);
    REQUIRE(body_keys == expected_keys);
    REQUIRE(app.resumable_rest_response_count() == 0);

    std::puts("test_signalk_rest_resume_http passed");
    return 0;
}
