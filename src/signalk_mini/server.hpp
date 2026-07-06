#pragma once

#include <memory>
#include <stddef.h>
#include <stdint.h>
#include <vector>

#if defined(ARDUINO)
#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <async_event_loop.hpp>
#include "config.hpp"
#include "connection_registry.hpp"
#include "nmea0183_input.hpp"
#include "publisher.hpp"

namespace signalk_mini {

class UdpListenerStream final : public async_event_loop::IByteStream {
public:
    UdpListenerStream() = default;
    ~UdpListenerStream() override { close(); }

    UdpListenerStream(const UdpListenerStream&) = delete;
    UdpListenerStream& operator=(const UdpListenerStream&) = delete;

    bool open(const UdpTransportConfig& config) {
        close();
#if defined(ARDUINO)
        (void)config;
        return false;
#else
        if (config.listen_port == 0) return false;
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return false;

        int one = 1;
        (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (config.allow_broadcast) {
            (void)::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
        }

        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            ::close(fd);
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(config.listen_port);
        if (!config.listen_host || strcmp(config.listen_host, "0.0.0.0") == 0) {
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
        } else if (::inet_pton(AF_INET, config.listen_host, &addr.sin_addr) != 1) {
            ::close(fd);
            return false;
        }

        if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(fd);
            return false;
        }

        fd_ = fd;
        return true;
#endif
    }

    void close() {
#if !defined(ARDUINO)
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
#endif
    }

    int read(uint8_t* dst, size_t max_len) override {
#if defined(ARDUINO)
        (void)dst;
        (void)max_len;
        return 0;
#else
        if (fd_ < 0 || !dst || max_len == 0) return 0;
        const ssize_t n = ::recv(fd_, dst, max_len, 0);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
            return -1;
        }
        return static_cast<int>(n);
#endif
    }

    int write(const uint8_t* src, size_t len) override {
        (void)src;
        (void)len;
        return 0;
    }

    bool readable() const override {
#if defined(ARDUINO)
        return false;
#else
        if (fd_ < 0) return false;
        int bytes = 0;
        return ::ioctl(fd_, FIONREAD, &bytes) == 0 && bytes > 0;
#endif
    }

    bool writable() const override { return false; }

    bool valid() const override {
#if defined(ARDUINO)
        return false;
#else
        return fd_ >= 0;
#endif
    }

    bool is_datagram() const override { return true; }

    int native_fd() const override {
#if defined(ARDUINO)
        return -1;
#else
        return fd_;
#endif
    }

private:
#if !defined(ARDUINO)
    int fd_ = -1;
#endif
};

template<typename Real, size_t MaxSignalKConnections = 8, size_t MaxConnectionsPerConnector = 4>
class MiniSignalKServer {
public:
    explicit MiniSignalKServer(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : config_(config),
          signalk_tcp_server_(loop_.scheduler()),
          publisher_(store_, config_),
          signalk_connections_(*this),
          signalk_line_handler_(signalk_connections_, signalk_options()) {}

    bool begin() {
        if (!loop_.valid()) return false;
        if (!listen(signalk_tcp_server_, signalk_line_handler_, config_.signalk.host, config_.signalk.port)) return false;
        start_configured_connectors();
        timer_ = loop_.on_repeat_us(config_.publisher.interval_us, [this]() { publish(); });
        return static_cast<bool>(timer_);
    }

    void tick() { loop_.tick(); }
    void run_forever() { loop_.run_forever(); }
    ModelStore<Real>& store() { return store_; }
    const ModelStore<Real>& store() const { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea_; }

private:
    static async_event_loop::TcpLineServerOptions signalk_options() {
        async_event_loop::TcpLineServerOptions options;
#if defined(ARDUINO)
        options.backpressure = async_event_loop::TcpBackpressureOptions::embedded_default();
#else
        options.backpressure = async_event_loop::TcpBackpressureOptions::server_default();
#endif
        return options;
    }

    bool listen(async_event_loop::NativeTcpServer& server, async_event_loop::ITcpServerHandler& handler, const char* host, uint16_t port) {
        async_event_loop::TcpListenOptions options;
        options.host = host;
        options.port = port;
        options.reuse_address = true;
        return server.listen(options, handler);
    }

    struct SignalKConnectionHandler : async_event_loop::ITcpLineServerHandler {
        explicit SignalKConnectionHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxSignalKConnections> connections;
        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            ConnectionFlags flags{owner.config_.signalk.allow_rx, owner.config_.signalk.allow_tx};
            if (!connections.add(connection, flags)) connection.close();
        }
        void on_line(async_event_loop::ITcpConnection& connection, async_event_loop::LineView) override { if (!connections.allow_rx(connection)) return; }
        void on_backpressure(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpBackpressureInfo&) override { connections.remove(connection); connection.close(); }
        void on_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { connections.remove(connection); }
        void on_too_many_connections(async_event_loop::ITcpConnection& connection) override { connection.close(); }
    };

    struct ConnectorTcpServerHandler : async_event_loop::ITcpLineServerHandler {
        ConnectorTcpServerHandler(MiniSignalKServer& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        MiniSignalKServer& owner;
        size_t index;
        ConnectionRegistry<MaxConnectionsPerConnector> connections;
        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            if (!connections.add(connection, owner.connector_connection_flags(index))) connection.close();
        }
        void on_line(async_event_loop::ITcpConnection& connection, async_event_loop::LineView line) override {
            if (!connections.allow_rx(connection)) return;
            char text[160];
            async_event_loop::line_to_cstr(line, text);
            owner.handle_connector_nmea_line(index, text);
        }
        void on_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { connections.remove(connection); }
        void on_too_many_connections(async_event_loop::ITcpConnection& connection) override { connection.close(); }
    };
