#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <signalk_mini.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

private:
    Fd fd_;
    uint16_t port_ = 0;
};

uint16_t reserve_tcp_loopback_port() {
    TcpListener listener;
    REQUIRE(listener.start());
    return listener.port();
}

uint16_t reserve_udp_loopback_port() {
    Fd fd(::socket(AF_INET, SOCK_DGRAM, 0));
    REQUIRE(fd.valid());

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

bool send_all(int fd, const uint8_t* data, size_t length) {
    const uint8_t* ptr = data;
    size_t remaining = length;
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

bool send_udp_datagram(uint16_t port, const uint8_t* data, size_t length) {
    Fd fd(::socket(AF_INET, SOCK_DGRAM, 0));
    if (!fd.valid()) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    const ssize_t n = ::sendto(fd.get(), data, length, 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    return n == static_cast<ssize_t>(length);
}

template<typename Predicate>
bool wait_until(signalk_mini::SignalKMiniApp<float>& app, Predicate predicate, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        app.tick();
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return predicate();
}

void test_input_adapter() {
    signalk_mini::ModelStore<float> store;
    signalk_mini::SeaTalkInput<float> input(store);
    uint64_t now_us = 1000;

    const uint8_t bad_depth[] = {0x00, 0x02, 0x00};
    REQUIRE(!input.feed_datagram(bad_depth, sizeof(bad_depth), 1, now_us));
    REQUIRE(input.receiver().bad_frame_count() > 0);

    const uint8_t depth[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    now_us += 1000;
    REQUIRE(input.feed_datagram(depth, sizeof(depth), 1, now_us));
    REQUIRE(store.model().sea.depth_m.valid);
    NEAR(store.model().sea.depth_m.value, 6.4008f, 0.0005f);

    const uint8_t wind_head[] = {0x11, 0x01};
    const uint8_t wind_tail[] = {0x0c, 0x03};
    now_us += 1000;
    REQUIRE(!input.feed_octets(wind_head, sizeof(wind_head), 1, now_us));
    now_us += 1000;
    REQUIRE(input.feed_octets(wind_tail, sizeof(wind_tail), 1, now_us));
    REQUIRE(store.model().wind.apparent.speed_kn.valid);
    NEAR(store.model().wind.apparent.speed_kn.value, 12.3f, 0.001f);

    const uint8_t stale_head[] = {0x10, 0x01};
    now_us += 1000;
    REQUIRE(!input.feed_octets(stale_head, sizeof(stale_head), 1, now_us));
    now_us += 600000;
    REQUIRE(input.feed_octets(depth, sizeof(depth), 1, now_us));
    NEAR(store.model().sea.depth_m.value, 6.4008f, 0.0005f);

    const uint8_t two_frames[] = {0x10, 0x01, 0x00, 0xb4, 0x52, 0x01, 0x61, 0x00};
    now_us += 1000;
    REQUIRE(input.feed_octets(two_frames, sizeof(two_frames), 1, now_us));
    REQUIRE(store.model().wind.apparent.direction_deg.valid);
    REQUIRE(store.model().gnss.fix.speed_kn.valid);
    NEAR(store.model().wind.apparent.direction_deg.value, 90.0f, 0.001f);
    NEAR(store.model().gnss.fix.speed_kn.value, 9.7f, 0.001f);
}

void test_udp_connector() {
    const uint16_t signalk_port = reserve_tcp_loopback_port();
    const uint16_t udp_port = reserve_udp_loopback_port();

    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "seatalk-udp-test";
    connector.access.allow_rx = true;
    connector.protocol.kind = signalk_mini::ConnectorProtocol::SeaTalk1;
    connector.transport.kind = signalk_mini::ConnectorTransport::Udp;
    connector.transport.udp.listen_host = "127.0.0.1";
    connector.transport.udp.listen_port = udp_port;

    signalk_mini::SignalKMiniConfig config;
    config.signalk.host = "127.0.0.1";
    config.signalk.port = signalk_port;
    config.connectors = &connector;
    config.connector_count = 1;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());

    const uint8_t depth[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    REQUIRE(send_udp_datagram(udp_port, depth, sizeof(depth)));
    REQUIRE(wait_until(app, [&]() { return app.store().model().sea.depth_m.valid; }, 1000));
    NEAR(app.store().model().sea.depth_m.value, 6.4008f, 0.0005f);
}

void test_tcp_server_connector() {
    const uint16_t signalk_port = reserve_tcp_loopback_port();
    const uint16_t seatalk_port = reserve_tcp_loopback_port();

    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "seatalk-tcp-server-test";
    connector.access.allow_rx = true;
    connector.protocol.kind = signalk_mini::ConnectorProtocol::SeaTalk1;
    connector.transport.kind = signalk_mini::ConnectorTransport::TcpServer;
    connector.transport.tcp_server.host = "127.0.0.1";
    connector.transport.tcp_server.port = seatalk_port;

    signalk_mini::SignalKMiniConfig config;
    config.signalk.host = "127.0.0.1";
    config.signalk.port = signalk_port;
    config.connectors = &connector;
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

    Fd client = connect_loopback(seatalk_port, 3000);
    REQUIRE(client.valid());

    const uint8_t wind_head[] = {0x11, 0x01};
    const uint8_t wind_tail_and_sog[] = {0x0c, 0x03, 0x52, 0x01, 0x61, 0x00};
    REQUIRE(send_all(client.get(), wind_head, sizeof(wind_head)));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    REQUIRE(send_all(client.get(), wind_tail_and_sog, sizeof(wind_tail_and_sog)));

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline) {
        if (app.store().model().wind.apparent.speed_kn.valid && app.store().model().gnss.fix.speed_kn.valid) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    running = false;
    loop_thread.join();

    REQUIRE(app.store().model().wind.apparent.speed_kn.valid);
    REQUIRE(app.store().model().gnss.fix.speed_kn.valid);
    NEAR(app.store().model().wind.apparent.speed_kn.value, 12.3f, 0.001f);
    NEAR(app.store().model().gnss.fix.speed_kn.value, 9.7f, 0.001f);
}

} // namespace

int main() {
    test_input_adapter();
    test_udp_connector();
    test_tcp_server_connector();
    return 0;
}
