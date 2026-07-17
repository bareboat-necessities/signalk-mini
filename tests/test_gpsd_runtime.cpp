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
#include <cstring>
#include <string>
#include <thread>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)
#define NEAR(a,b,e) do { if (std::fabs(static_cast<double>((a)-(b))) > (e)) { std::fprintf(stderr, "NEAR FAILED %s:%d\n", __FILE__, __LINE__); std::exit(2); } } while (0)

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
    void reset(int fd = -1) { if (fd_ >= 0) ::close(fd_); fd_ = fd; }
private:
    int fd_;
};

uint16_t reserve_port() {
    Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
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
    REQUIRE(fd.valid());
    int one = 1;
    REQUIRE(::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) == 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port);
    REQUIRE(::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    REQUIRE(::listen(fd.get(), 2) == 0);
    const int flags = ::fcntl(fd.get(), F_GETFL, 0);
    REQUIRE(::fcntl(fd.get(), F_SETFL, flags | O_NONBLOCK) == 0);
    return fd;
}

bool send_all(int fd, const char* text) {
    size_t offset = 0;
    const size_t length = std::strlen(text);
    while (offset < length) {
        const ssize_t n = ::write(fd, text + offset, length - offset);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return false;
        offset += static_cast<size_t>(n);
    }
    return true;
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
}

int main() {
    const uint16_t gpsd_port = reserve_port();
    const uint16_t signalk_port = reserve_port();
    Fd listener = start_listener(gpsd_port);

    signalk_mini::ConnectorConfig connector;
    connector.enabled = true;
    connector.label = "gpsd-main";
    connector.protocol.kind = signalk_mini::ConnectorProtocol::Gpsd;
    connector.protocol.gpsd.device = "/dev/ttyACM0";
    connector.protocol.gpsd.include_sky = true;
    connector.protocol.gpsd.include_gst = true;
    connector.transport.kind = signalk_mini::ConnectorTransport::TcpClient;
    connector.transport.tcp_client.host = "127.0.0.1";
    connector.transport.tcp_client.port = gpsd_port;
    connector.access.allow_rx = true;
    connector.access.allow_tx = false;
    connector.reconnect.enabled = true;
    connector.reconnect.initial_delay_ms = 10;
    connector.reconnect.maximum_delay_ms = 40;

    signalk_mini::SignalKMiniConfig config;
    config.signalk.host = "127.0.0.1";
    config.signalk.port = signalk_port;
    config.signalk.websocket.enabled = false;
    config.connectors = &connector;
    config.connector_count = 1;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());

    Fd accepted;
    REQUIRE(wait_until(app, [&] {
        const int fd = ::accept(listener.get(), nullptr, nullptr);
        if (fd >= 0) accepted.reset(fd);
        return accepted.valid();
    }, 1000));

    char watch[512]{};
    REQUIRE(wait_until(app, [&] {
        const ssize_t n = ::recv(accepted.get(), watch, sizeof(watch) - 1, MSG_DONTWAIT);
        if (n > 0) watch[n] = '\0';
        return n > 0;
    }, 1000));
    REQUIRE(std::strstr(watch, "?WATCH=") != nullptr);
    REQUIRE(std::strstr(watch, "/dev/ttyACM0") != nullptr);

    REQUIRE(send_all(accepted.get(), "{\"class\":\"VERSION\",\"release\":\"3.25\",\"proto_major\":3,\"proto_minor\":14}\n"));
    const char* tpv = "{\"class\":\"TPV\",\"device\":\"/dev/ttyACM0\",\"mode\":3,\"time\":\"2026-07-17T05:06:07Z\",\"lat\":40.7654321,\"lon\":-74.1234567,\"altMSL\":12.3,\"speed\":2.0,\"track\":81.0}\n";
    REQUIRE(::write(accepted.get(), tpv, 31) == 31);
    for (int i = 0; i < 5; ++i) app.tick();
    REQUIRE(send_all(accepted.get(), tpv + 31));
    REQUIRE(send_all(accepted.get(), "{\"class\":\"SKY\",\"device\":\"/dev/ttyACM0\",\"pdop\":1.2,\"hdop\":0.8,\"vdop\":1.0,\"satellites\":[{\"PRN\":3,\"el\":40,\"az\":120,\"ss\":38,\"used\":true}]}\n"));

    REQUIRE(wait_until(app, [&] { return app.store().model().gnss.fix.fix_lat_deg.valid && app.store().model().gnss.sky_view.observation_count == 1; }, 1200));
    NEAR(app.store().model().gnss.fix.fix_lat_deg.value, 40.7654321f, 0.00001);
    REQUIRE(app.store().model().gnss.fix.source.value == ship_data_model::SensorSource::gpsd);
    REQUIRE(app.store().model().gnss.fix.satellites_used.value == 1);
    NEAR(app.store().model().gnss.fix.hdop.value, 0.8f, 0.001);
    REQUIRE(app.gpsd_record_count() >= 3);

    accepted.reset();
    Fd accepted_again;
    REQUIRE(wait_until(app, [&] {
        const int fd = ::accept(listener.get(), nullptr, nullptr);
        if (fd >= 0) accepted_again.reset(fd);
        return accepted_again.valid();
    }, 1500));
    REQUIRE(app.connector_reconnect_count() > 0);
    char reconnect_watch[512]{};
    REQUIRE(wait_until(app, [&] {
        const ssize_t n = ::recv(accepted_again.get(), reconnect_watch, sizeof(reconnect_watch) - 1, MSG_DONTWAIT);
        if (n > 0) reconnect_watch[n] = '\0';
        return n > 0;
    }, 1000));
    REQUIRE(std::strstr(reconnect_watch, "?WATCH=") != nullptr);
    REQUIRE(std::strstr(reconnect_watch, "/dev/ttyACM0") != nullptr);

    signalk_mini::ConnectorConfig invalid_connector = connector;
    invalid_connector.protocol.gpsd.device = "/dev/bad\"device";
    signalk_mini::SignalKMiniConfig invalid_config = config;
    invalid_config.connectors = &invalid_connector;
    signalk_mini::SignalKMiniApp<float> invalid_app(invalid_config);
    REQUIRE(!invalid_app.begin());
    REQUIRE(invalid_app.last_startup_error() == signalk_mini::MiniSignalKServer<float>::StartupError::InvalidConnectorProtocolConfig);
    return 0;
}
