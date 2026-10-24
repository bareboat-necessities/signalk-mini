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
#include <string>
#include <vector>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

// Single-threaded: the test drives app.tick() itself between socket operations,
// so there are no races between the event loop and the test.
namespace {

using Clock = std::chrono::steady_clock;

int make_listener(uint16_t& port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd >= 0);
    int one = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    REQUIRE(::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    REQUIRE(::listen(fd, 4) == 0);
    socklen_t len = sizeof(addr);
    REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    port = ntohs(addr.sin_port);
    REQUIRE(::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK) == 0);
    return fd;
}

uint16_t free_port() {
    uint16_t port = 0;
    const int fd = make_listener(port);
    ::close(fd);
    return port;
}

int connect_client(uint16_t port, int receive_buffer) {
    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    REQUIRE(fd >= 0);
    // A tiny receive window makes the server's output back up quickly.
    if (receive_buffer > 0) (void)::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    REQUIRE(::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    REQUIRE(::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL, 0) | O_NONBLOCK) == 0);
    return fd;
}

// Returns false on EOF.
bool read_available(int fd, std::string& out) {
    char buf[4096];
    for (;;) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n > 0) {
            out.append(buf, static_cast<size_t>(n));
            continue;
        }
        if (n == 0) return false;
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }
}

std::string sentence(const char* body) {
    char out[128];
    const uint8_t cs = nmea0183_connector::nmea_checksum_body(body);
    std::snprintf(out, sizeof(out), "$%s*%c%c\r\n", body,
                  nmea0183_connector::to_hex((cs >> 4) & 0x0f), nmea0183_connector::to_hex(cs & 0x0f));
    return out;
}

struct Rig {
    uint16_t nmea_port = 0;
    int nmea_listener = -1;
    int nmea = -1;
    signalk_mini::ConnectorConfig connector;
    signalk_mini::SignalKMiniConfig config;

    explicit Rig(uint32_t stalled_timeout_ms) {
        nmea_listener = make_listener(nmea_port);
        connector.enabled = true;
        connector.label = "slow-client-nmea";
        connector.access.allow_rx = true;
        connector.protocol.kind = signalk_mini::ConnectorProtocol::Nmea0183;
        connector.protocol.nmea0183.validate_checksum = false;
        connector.protocol.nmea0183.validate_checksum_configured = true;
        connector.transport.kind = signalk_mini::ConnectorTransport::TcpClient;
        connector.transport.tcp_client.host = "127.0.0.1";
        connector.transport.tcp_client.port = nmea_port;
        config.signalk.host = "127.0.0.1";
        config.signalk.port = free_port();
        config.signalk.websocket.enabled = false;
        config.publisher.interval_us = 1000;
        config.publisher.max_changes_per_tick = 256;
        config.publisher.stalled_client_timeout_ms = stalled_timeout_ms;
        config.connectors = &connector;
        config.connector_count = 1;
    }

    ~Rig() {
        if (nmea >= 0) ::close(nmea);
        if (nmea_listener >= 0) ::close(nmea_listener);
    }

    void accept_nmea(signalk_mini::SignalKMiniApp<float>& app) {
        const auto deadline = Clock::now() + std::chrono::seconds(3);
        while (nmea < 0 && Clock::now() < deadline) {
            app.tick();
            nmea = ::accept(nmea_listener, nullptr, nullptr);
        }
        REQUIRE(nmea >= 0);
        REQUIRE(::fcntl(nmea, F_SETFL, ::fcntl(nmea, F_GETFL, 0) | O_NONBLOCK) == 0);
    }
};

void wait_for(signalk_mini::SignalKMiniApp<float>& app, int client, std::string& received, const char* needle) {
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (received.find(needle) == std::string::npos && Clock::now() < deadline) {
        app.tick();
        REQUIRE(read_available(client, received));
    }
    REQUIRE(received.find(needle) != std::string::npos);
}

// Feeds wind sentences until the server has had to put a client into catch-up.
int flood_until_catch_up(signalk_mini::SignalKMiniApp<float>& app, Rig& rig) {
    std::string pending;
    int angle = 0;
    const auto deadline = Clock::now() + std::chrono::seconds(20);
    while (app.stream_catch_up_count() == 0 && Clock::now() < deadline) {
        while (pending.size() < 64 * 1024) {
            char body[64];
            std::snprintf(body, sizeof(body), "IIMWV,%03d.0,R,%04.1f,N,A", angle % 360, 5.0 + (angle % 100) / 10.0);
            pending += sentence(body);
            ++angle;
        }
        const ssize_t n = ::send(rig.nmea, pending.data(), pending.size(), MSG_NOSIGNAL);
        if (n > 0) pending.erase(0, static_cast<size_t>(n));
        app.tick();
    }
    REQUIRE(app.stream_catch_up_count() > 0);
    return angle;
}

void test_slow_client_gets_whole_deltas_then_catches_up() {
    Rig rig(0);
    signalk_mini::SignalKMiniApp<float> app(rig.config);
    REQUIRE(app.begin());
    rig.accept_nmea(app);

    const int client = connect_client(rig.config.signalk.port, 2048);
    std::string received;
    wait_for(app, client, received, "\"roles\"");
    REQUIRE(::send(client, "subscribe all\r\n", 15, MSG_NOSIGNAL) == 15);
    wait_for(app, client, received, "communication.server.clock");

    // Stop reading: the server must skip the client rather than queue or cut deltas.
    flood_until_catch_up(app, rig);

    // A final, distinct value that the client must end up with after catching up.
    const std::string last = sentence("IIMWV,123.0,R,42.0,N,A");
    REQUIRE(::send(rig.nmea, last.data(), last.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(last.size()));

    const auto deadline = Clock::now() + std::chrono::seconds(20);
    auto quiet_since = Clock::now();
    while (Clock::now() < deadline) {
        app.tick();
        const size_t before = received.size();
        REQUIRE(read_available(client, received));
        if (received.size() != before) quiet_since = Clock::now();
        if (app.catching_up_client_count() == 0 && Clock::now() - quiet_since > std::chrono::milliseconds(300)) break;
    }
    REQUIRE(app.catching_up_client_count() == 0);
    ::close(client);

    // Every line is one complete delta: nothing was cut mid-frame.
    double last_speed = -1.0;
    size_t lines = 0;
    size_t start = 0;
    while (start < received.size()) {
        const size_t end = received.find("\r\n", start);
        REQUIRE(end != std::string::npos);
        const std::string line = received.substr(start, end - start);
        start = end + 2;
        JsonDocument doc;
        REQUIRE(!deserializeJson(doc, line));
        ++lines;
        for (JsonVariantConst value : doc["updates"][0]["values"].as<JsonArrayConst>()) {
            if (std::strcmp(value["path"] | "", "environment.wind.speedApparent") == 0) last_speed = value["value"].as<double>();
        }
    }
    REQUIRE(lines > 10);
    // 42 kn in m/s: the latest value arrived after catch-up.
    REQUIRE(last_speed > 21.5 && last_speed < 21.7);
}

void test_stalled_client_is_closed() {
    Rig rig(400);
    signalk_mini::SignalKMiniApp<float> app(rig.config);
    REQUIRE(app.begin());
    rig.accept_nmea(app);

    const int client = connect_client(rig.config.signalk.port, 2048);
    std::string received;
    wait_for(app, client, received, "\"roles\"");
    REQUIRE(::send(client, "subscribe all\r\n", 15, MSG_NOSIGNAL) == 15);
    wait_for(app, client, received, "communication.server.clock");

    flood_until_catch_up(app, rig);
    // Keep the feed live, as on a boat, so the client's queue stays full.
    const std::string wind = sentence("IIMWV,090.0,R,10.0,N,A");
    const auto deadline = Clock::now() + std::chrono::seconds(10);
    while (app.stalled_client_close_count() == 0 && Clock::now() < deadline) {
        (void)::send(rig.nmea, wind.data(), wind.size(), MSG_NOSIGNAL);
        app.tick();
    }
    REQUIRE(app.stalled_client_close_count() == 1);

    // Draining now reaches EOF: the server closed the connection.
    bool eof = false;
    const auto eof_deadline = Clock::now() + std::chrono::seconds(5);
    while (!eof && Clock::now() < eof_deadline) {
        app.tick();
        eof = !read_available(client, received);
    }
    REQUIRE(eof);
    ::close(client);
}

} // namespace

int main() {
    test_slow_client_gets_whole_deltas_then_catches_up();
    test_stalled_client_is_closed();
    std::puts("test_signalk_slow_client passed");
    return 0;
}
