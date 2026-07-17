#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <signalk_mini.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

#include "ubx_test_helpers.hpp"

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a) - (b))) > (e)) { std::fprintf(stderr, "NEAR failed %s:%d got=%f expected=%f\n", __FILE__, __LINE__, static_cast<double>(a), static_cast<double>(b)); std::exit(2); } } while (0)

namespace {
class Fd {
public:
    explicit Fd(int fd = -1) : fd_(fd) {}
    ~Fd() { if (fd_ >= 0) ::close(fd_); }
    Fd(const Fd&) = delete;
    Fd& operator=(const Fd&) = delete;
    Fd(Fd&& other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
    Fd& operator=(Fd&& other) noexcept { if (this != &other) { if (fd_ >= 0) ::close(fd_); fd_ = other.fd_; other.fd_ = -1; } return *this; }
    int get() const { return fd_; }
    bool valid() const { return fd_ >= 0; }
private:
    int fd_;
};

uint16_t reserve_port(int type) {
    Fd fd(::socket(AF_INET, type, 0));
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

Fd start_listener(uint16_t port) {
    Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
    if (!fd.valid()) return Fd{};
    int one = 1;
    (void)::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return Fd{};
    if (::listen(fd.get(), 2) != 0) return Fd{};
    const int flags = ::fcntl(fd.get(), F_GETFL, 0);
    (void)::fcntl(fd.get(), F_SETFL, flags | O_NONBLOCK);
    return fd;
}

Fd connect_loopback(uint16_t port) {
    Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
    if (!fd.valid()) return Fd{};
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    if (::connect(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) return Fd{};
    return fd;
}

bool send_all(int fd, const uint8_t* bytes, size_t length) {
    size_t offset = 0;
    while (offset < length) {
        const ssize_t n = ::write(fd, bytes + offset, length - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        offset += static_cast<size_t>(n);
    }
    return true;
}

bool send_udp(uint16_t port, const std::vector<uint8_t>& bytes) {
    Fd fd(::socket(AF_INET, SOCK_DGRAM, 0));
    if (!fd.valid()) return false;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    return ::sendto(fd.get(), bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == static_cast<ssize_t>(bytes.size());
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

signalk_mini::SignalKMiniConfig base_config(uint16_t signalk_port, signalk_mini::ConnectorConfig& connector) {
    signalk_mini::SignalKMiniConfig config;
    config.signalk.host = "127.0.0.1";
    config.signalk.port = signalk_port;
    config.signalk.websocket.enabled = false;
    config.connectors = &connector;
    config.connector_count = 1;
    return config;
}


void test_startup_validation() {
    const uint16_t signal_port = reserve_port(SOCK_STREAM);
    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.transport.kind = signalk_mini::ConnectorTransport::TcpClient;
    connector.transport.tcp_client.host = "127.0.0.1";
    connector.transport.tcp_client.port = 2947;
    auto config = base_config(signal_port, connector);

    connector.protocol.kind = signalk_mini::ConnectorProtocol::Gpsd;
    signalk_mini::SignalKMiniApp<float> gpsd_app(config);
    REQUIRE(!gpsd_app.begin());
    REQUIRE(gpsd_app.last_startup_error() == signalk_mini::MiniSignalKServer<float>::StartupError::UnsupportedConnectorProtocol);

    connector.protocol.kind = signalk_mini::ConnectorProtocol::Ubx;
    connector.protocol.ubx.configure_receiver = true;
    signalk_mini::SignalKMiniApp<float> configured_app(config);
    REQUIRE(!configured_app.begin());
    REQUIRE(configured_app.last_startup_error() == signalk_mini::MiniSignalKServer<float>::StartupError::InvalidConnectorProtocolConfig);

    connector.protocol.ubx.configure_receiver = false;
    connector.access.allow_tx = true;
    signalk_mini::SignalKMiniApp<float> transmit_app(config);
    REQUIRE(!transmit_app.begin());
    REQUIRE(transmit_app.last_startup_error() == signalk_mini::MiniSignalKServer<float>::StartupError::InvalidConnectorProtocolConfig);

    connector.access.allow_tx = false;
    connector.access.allow_rx = false;
    signalk_mini::SignalKMiniApp<float> receive_disabled_app(config);
    REQUIRE(!receive_disabled_app.begin());
    REQUIRE(receive_disabled_app.last_startup_error() == signalk_mini::MiniSignalKServer<float>::StartupError::InvalidConnectorProtocolConfig);

    connector.access.allow_rx = true;
    connector.reconnect.initial_delay_ms = 0;
    signalk_mini::SignalKMiniApp<float> reconnect_app(config);
    REQUIRE(!reconnect_app.begin());
    REQUIRE(reconnect_app.last_startup_error() == signalk_mini::MiniSignalKServer<float>::StartupError::InvalidConnectorReconnectConfig);
}

void test_udp() {
    const uint16_t signal_port = reserve_port(SOCK_STREAM);
    const uint16_t ubx_port = reserve_port(SOCK_DGRAM);
    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "ubx-udp";
    connector.protocol.kind = signalk_mini::ConnectorProtocol::Ubx;
    connector.transport.kind = signalk_mini::ConnectorTransport::Udp;
    connector.transport.udp.listen_host = "127.0.0.1";
    connector.transport.udp.listen_port = ubx_port;
    auto config = base_config(signal_port, connector);
    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    const auto frame = ubx_test::nav_pvt();
    REQUIRE(send_udp(ubx_port, frame));
    REQUIRE(wait_until(app, [&] { return app.store().model().gnss.fix.fix_lat_deg.valid; }, 1000));
    NEAR(app.store().model().gnss.fix.fix_lat_deg.value, 40.7654321f, 0.00001f);

    const auto large_sky_frame = ubx_test::nav_sat(42);
    REQUIRE(large_sky_frame.size() > 512);
    REQUIRE(send_udp(ubx_port, large_sky_frame));
    REQUIRE(wait_until(app, [&] { return app.store().model().gnss.sky_view.observation_count == 42; }, 1000));
    REQUIRE(app.store().model().gnss.sky_view.complete);
    REQUIRE(app.ubx_frame_count() == 2);
}

void test_tcp_server() {
    const uint16_t signal_port = reserve_port(SOCK_STREAM);
    const uint16_t ubx_port = reserve_port(SOCK_STREAM);
    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "ubx-tcp-server";
    connector.protocol.kind = signalk_mini::ConnectorProtocol::Ubx;
    connector.transport.kind = signalk_mini::ConnectorTransport::TcpServer;
    connector.transport.tcp_server.host = "127.0.0.1";
    connector.transport.tcp_server.port = ubx_port;
    auto config = base_config(signal_port, connector);
    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    Fd client;
    REQUIRE(wait_until(app, [&] { client = connect_loopback(ubx_port); return client.valid(); }, 1000));
    const auto frame = ubx_test::nav_pvt();
    REQUIRE(send_all(client.get(), frame.data(), 11));
    for (int i = 0; i < 10; ++i) app.tick();
    REQUIRE(send_all(client.get(), frame.data() + 11, frame.size() - 11));
    REQUIRE(wait_until(app, [&] { return app.store().model().gnss.fix.fix_lon_deg.valid; }, 1000));
    NEAR(app.store().model().gnss.fix.fix_lon_deg.value, -74.1234567f, 0.00001f);
}

void test_serial() {
    const int master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    REQUIRE(master >= 0);
    Fd master_fd(master);
    REQUIRE(::grantpt(master) == 0);
    REQUIRE(::unlockpt(master) == 0);
    char* slave_name = ::ptsname(master);
    REQUIRE(slave_name != nullptr);

    const uint16_t signal_port = reserve_port(SOCK_STREAM);
    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "ubx-serial";
    connector.protocol.kind = signalk_mini::ConnectorProtocol::Ubx;
    connector.transport.kind = signalk_mini::ConnectorTransport::Serial;
    connector.transport.serial.device = slave_name;
    connector.transport.serial.baud = 115200;
    auto config = base_config(signal_port, connector);
    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    const auto frame = ubx_test::nav_pvt();
    REQUIRE(send_all(master_fd.get(), frame.data(), frame.size()));
    REQUIRE(wait_until(app, [&] { return app.store().model().gnss.fix.fix_alt_hae_m.valid; }, 1000));
    NEAR(app.store().model().gnss.fix.fix_alt_hae_m.value, 15.34f, 0.001f);
}

void test_tcp_client_reconnect() {
    const uint16_t signal_port = reserve_port(SOCK_STREAM);
    const uint16_t ubx_port = reserve_port(SOCK_STREAM);
    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "ubx-tcp-client";
    connector.protocol.kind = signalk_mini::ConnectorProtocol::Ubx;
    connector.transport.kind = signalk_mini::ConnectorTransport::TcpClient;
    connector.transport.tcp_client.host = "127.0.0.1";
    connector.transport.tcp_client.port = ubx_port;
    connector.reconnect.enabled = true;
    connector.reconnect.initial_delay_ms = 10;
    connector.reconnect.maximum_delay_ms = 40;
    auto config = base_config(signal_port, connector);
    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    for (int i = 0; i < 20; ++i) { app.tick(); std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
    Fd listener = start_listener(ubx_port);
    REQUIRE(listener.valid());
    Fd accepted;
    REQUIRE(wait_until(app, [&] {
        const int fd = ::accept(listener.get(), nullptr, nullptr);
        if (fd >= 0) accepted = Fd(fd);
        return accepted.valid();
    }, 1500));
    const auto frame = ubx_test::nav_pvt();
    REQUIRE(send_all(accepted.get(), frame.data(), frame.size()));
    REQUIRE(wait_until(app, [&] { return app.store().model().gnss.fix.fix_valid.valid; }, 1000));
    REQUIRE(app.connector_reconnect_count() > 0);
    REQUIRE(app.store().model().gnss.fix.source.value == ship_data_model::SensorSource::ubx);
}
}

int main() {
    test_startup_validation();
    test_udp();
    test_tcp_server();
    test_serial();
    test_tcp_client_reconnect();
    return 0;
}
