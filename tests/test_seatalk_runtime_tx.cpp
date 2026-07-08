#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <signalk_mini.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>

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

uint16_t reserve_tcp_loopback_port() {
    Fd fd(::socket(AF_INET, SOCK_STREAM, 0));
    REQUIRE(fd.valid());
    int one = 1;
    (void)::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    REQUIRE(::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);
    REQUIRE(::listen(fd.get(), 1) == 0);

    socklen_t len = sizeof(addr);
    REQUIRE(::getsockname(fd.get(), reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    return ntohs(addr.sin_port);
}

Fd bind_udp_loopback(uint16_t& port_out) {
    Fd fd(::socket(AF_INET, SOCK_DGRAM, 0));
    REQUIRE(fd.valid());

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    REQUIRE(::bind(fd.get(), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0);

    socklen_t len = sizeof(addr);
    REQUIRE(::getsockname(fd.get(), reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    port_out = ntohs(addr.sin_port);
    return fd;
}

int recv_with_timeout(int fd, uint8_t* dst, size_t size, int timeout_ms) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    const int ready = ::select(fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ready <= 0) return ready;
    const ssize_t n = ::recv(fd, dst, size, 0);
    return n < 0 ? -1 : static_cast<int>(n);
}

void test_udp_binary_and_nmea_tx() {
    const uint16_t signalk_port = reserve_tcp_loopback_port();
    uint16_t binary_port = 0;
    uint16_t nmea_port = 0;
    Fd binary_rx = bind_udp_loopback(binary_port);
    Fd nmea_rx = bind_udp_loopback(nmea_port);

    signalk_mini::ConnectorConfig connectors[2];
    connectors[0].enabled = true;
    connectors[0].label = "seatalk-binary-udp-tx";
    connectors[0].access.allow_rx = false;
    connectors[0].access.allow_tx = true;
    connectors[0].protocol.kind = signalk_mini::ConnectorProtocol::SeaTalk1;
    connectors[0].transport.kind = signalk_mini::ConnectorTransport::Udp;
    connectors[0].transport.udp.remote_host = "127.0.0.1";
    connectors[0].transport.udp.remote_port = binary_port;

    connectors[1].enabled = true;
    connectors[1].label = "seatalk-nmea-udp-tx";
    connectors[1].access.allow_rx = false;
    connectors[1].access.allow_tx = true;
    connectors[1].protocol.kind = signalk_mini::ConnectorProtocol::Nmea0183;
    connectors[1].transport.kind = signalk_mini::ConnectorTransport::Udp;
    connectors[1].transport.udp.remote_host = "127.0.0.1";
    connectors[1].transport.udp.remote_port = nmea_port;

    signalk_mini::SignalKMiniConfig config;
    config.signalk.host = "127.0.0.1";
    config.signalk.port = signalk_port;
    config.connectors = connectors;
    config.connector_count = 2;

    signalk_mini::SignalKMiniApp<float> app(config);
    REQUIRE(app.begin());
    REQUIRE(app.transmit_seatalk_depth_m(6.4008f));

    uint8_t buf[128];
    int n = recv_with_timeout(binary_rx.get(), buf, sizeof(buf), 1000);
    const uint8_t expected_binary[] = {0x00, 0x02, 0x00, 0xd2, 0x00};
    REQUIRE(n == static_cast<int>(sizeof(expected_binary)));
    REQUIRE(std::memcmp(buf, expected_binary, sizeof(expected_binary)) == 0);

    n = recv_with_timeout(nmea_rx.get(), buf, sizeof(buf) - 1, 1000);
    REQUIRE(n > 0);
    buf[n] = 0;
    REQUIRE(std::strcmp(reinterpret_cast<const char*>(buf), "$STALK,000200D200*19\r\n") == 0);
}

} // namespace

int main() {
    test_udp_binary_and_nmea_tx();
    return 0;
}
