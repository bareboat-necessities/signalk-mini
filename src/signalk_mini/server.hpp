#pragma once

#include <memory>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <vector>

#if defined(ARDUINO)
#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <async_event_loop.hpp>
#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include "config.hpp"
#include "connection_registry.hpp"
#include "nmea0183_input.hpp"
#include "publisher.hpp"
#include "seatalk_input.hpp"
#include "signalk_hello_writer.hpp"
#include "websocket.hpp"

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
        if (config.listen_port == 0 && (!config.remote_host || config.remote_port == 0)) return false;
        const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd < 0) return false;

        int one = 1;
        (void)::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        if (config.allow_broadcast) (void)::setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));

        int flags = ::fcntl(fd, F_GETFL, 0);
        if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
            ::close(fd);
            return false;
        }

        if (config.listen_port != 0) {
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
        }

        if (config.remote_host && config.remote_port != 0) {
            remote_addr_ = sockaddr_in{};
            remote_addr_.sin_family = AF_INET;
            remote_addr_.sin_port = htons(config.remote_port);
            if (strcmp(config.remote_host, "255.255.255.255") == 0) {
                remote_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);
            } else if (::inet_pton(AF_INET, config.remote_host, &remote_addr_.sin_addr) != 1) {
                ::close(fd);
                return false;
            }
            remote_addr_ready_ = true;
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
        remote_addr_ = sockaddr_in{};
        remote_addr_ready_ = false;
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
#if defined(ARDUINO)
        (void)src;
        (void)len;
        return 0;
#else
        if (fd_ < 0 || !remote_addr_ready_ || !src || len == 0) return 0;
        const ssize_t n = ::sendto(fd_, src, len, 0, reinterpret_cast<const sockaddr*>(&remote_addr_), sizeof(remote_addr_));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
            return -1;
        }
        return static_cast<int>(n);
#endif
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

    bool writable() const override {
#if defined(ARDUINO)
        return false;
#else
        return fd_ >= 0 && remote_addr_ready_;
#endif
    }

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
    sockaddr_in remote_addr_{};
    bool remote_addr_ready_ = false;
#endif
};

template<typename Real, size_t MaxSignalKConnections = 8, size_t MaxConnectionsPerConnector = 4>
class MiniSignalKServer {
public:
    enum class StartupError : uint8_t {
        None = 0,
        InvalidSignalKPort,
        InvalidSignalKMaxConnections,
        InvalidPublisherInterval,
        InvalidConnectorList,
        UnsupportedConnectorProtocol,
        UnsupportedConnectorTransport,
        InvalidConnectorTransportConfig,
        ConnectorStartFailed,
    };

    explicit MiniSignalKServer(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : config_(config),
          signalk_tcp_server_(loop_.scheduler()),
          signalk_websocket_server_(loop_.scheduler()),
          publisher_(store_, config_),
          signalk_connections_(*this),
          signalk_websocket_connections_(*this),
          signalk_line_handler_(signalk_connections_, signalk_options()) {}

    bool begin() {
        reset_startup_diagnostics();
        if (!validate_config()) return false;
        if (!loop_.valid()) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        if (!listen(signalk_tcp_server_, signalk_line_handler_, config_.signalk.host, config_.signalk.port)) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        if (config_.signalk.websocket.enabled) {
            if (!listen(signalk_websocket_server_, signalk_websocket_connections_, config_.signalk.websocket.host, config_.signalk.websocket.port)) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        }
        if (!start_configured_connectors()) return false;
        timer_ = loop_.on_repeat_us(config_.publisher.interval_us, [this]() { publish(); });
        if (!timer_) return set_startup_error(StartupError::InvalidPublisherInterval, config_.connector_count);
        clock_timer_ = loop_.on_repeat_us(1000000ULL, [this]() { publish_server_clock(); });
        if (!clock_timer_) return set_startup_error(StartupError::InvalidPublisherInterval, config_.connector_count);
        return true;
    }

    void tick() { loop_.tick(); }
    void run_forever() { loop_.run_forever(); }
    ModelStore<Real>& store() { return store_; }
    const ModelStore<Real>& store() const { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea_; }
    SeaTalkInput<Real>& seatalk() { return seatalk_; }

    uint64_t dropped_change_count() const { return store_.dropped_change_count(); }
    uint64_t dropped_publish_count() const { return publisher_.dropped_publish_count(); }
    uint64_t published_delta_count() const { return publisher_.published_delta_count(); }
    StartupError last_startup_error() const { return last_startup_error_; }
    size_t last_failed_connector_index() const { return last_failed_connector_index_; }
    uint32_t connector_start_failure_count() const { return connector_start_failure_count_; }

    bool transmit_seatalk_frame(const seatalk::SeaTalkFrame& frame) {
        if (frame.length == 0) return false;
        bool any = false;
        for (auto& ptr : connector_slots_) {
            if (!ptr) continue;
            ConnectorRuntimeSlot& slot = *ptr;
            if (!slot.started || !slot.connection_flags.allow_tx) continue;
            if (slot.config.protocol.kind == ConnectorProtocol::SeaTalk1) {
                if (transmit_seatalk_binary(slot, frame)) any = true;
            } else if (slot.config.protocol.kind == ConnectorProtocol::Nmea0183) {
                if (transmit_seatalk_over_nmea(slot, frame)) any = true;
            }
        }
        return any;
    }

    bool transmit_seatalk_pilot_key(seatalk::SeaTalkPilotKey key, bool long_press = false) {
        seatalk::SeaTalkFrame frame;
        return seatalk::make_pilot_key(frame, key, long_press) && transmit_seatalk_frame(frame);
    }

    bool transmit_seatalk_depth_m(Real depth_m, bool alarm = false) {
        seatalk::SeaTalkFrame frame;
        return seatalk::make_depth_m(frame, depth_m, alarm) && transmit_seatalk_frame(frame);
    }

    bool transmit_seatalk_apparent_wind_angle_deg(Real angle_deg) {
        seatalk::SeaTalkFrame frame;
        return seatalk::make_apparent_wind_angle_deg(frame, angle_deg) && transmit_seatalk_frame(frame);
    }

    bool transmit_seatalk_apparent_wind_speed_kn(Real speed_kn) {
        seatalk::SeaTalkFrame frame;
        return seatalk::make_apparent_wind_speed_kn(frame, speed_kn) && transmit_seatalk_frame(frame);
    }

    bool transmit_seatalk_speed_through_water_kn(Real speed_kn) {
        seatalk::SeaTalkFrame frame;
        return seatalk::make_speed_through_water_kn(frame, speed_kn) && transmit_seatalk_frame(frame);
    }

private:
    static async_event_loop::TcpBackpressureOptions signalk_backpressure_options() {
#if defined(ARDUINO)
        return async_event_loop::TcpBackpressureOptions::embedded_default();
#else
        return async_event_loop::TcpBackpressureOptions::server_default();
#endif
    }

    static async_event_loop::TcpLineServerOptions signalk_options() {
        async_event_loop::TcpLineServerOptions options;
        options.backpressure = signalk_backpressure_options();
        return options;
    }

    static bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

    static bool line_starts_with(const char* text, const char* prefix) {
        return text && prefix && strncmp(text, prefix, strlen(prefix)) == 0;
    }

    static bool is_subscribe_all_text(const char* source) {
        char text[256];
        if (!source) return false;
        size_t n = strlen(source);
        if (n >= sizeof(text)) n = sizeof(text) - 1;
        memcpy(text, source, n);
        text[n] = '\0';
        char* begin = text;
        while (*begin && is_space(*begin)) ++begin;
        char* end = begin + strlen(begin);
        while (end > begin && is_space(end[-1])) --end;
        *end = '\0';

        if (strcmp(begin, "*") == 0 || strcmp(begin, "subscribe") == 0 || strcmp(begin, "subscribe all") == 0) return true;
        if (line_starts_with(begin, "SUBSCRIBE") || line_starts_with(begin, "Subscribe")) return true;
        if (begin[0] == '{' && strstr(begin, "subscribe") && (strstr(begin, "all") || strstr(begin, "*") || strstr(begin, "path"))) return true;
        return false;
    }

    static bool is_subscribe_all_line(async_event_loop::LineView line) {
        char text[256];
        async_event_loop::line_to_cstr(line, text);
        return is_subscribe_all_text(text);
    }

    static ship_data_model::SensorSource sensor_source_for_transport(ConnectorTransport transport) {
        switch (transport) {
        case ConnectorTransport::Serial: return ship_data_model::SensorSource::serial;
        case ConnectorTransport::TcpClient:
        case ConnectorTransport::TcpServer:
        case ConnectorTransport::Udp: return ship_data_model::SensorSource::tcp;
        default: return ship_data_model::SensorSource::none;
        }
    }

    static bool supported_protocol(ConnectorProtocol protocol) {
        return protocol == ConnectorProtocol::Nmea0183 || protocol == ConnectorProtocol::SeaTalk1;
    }

    static bool supported_transport(ConnectorTransport transport) {
        return transport == ConnectorTransport::Serial ||
               transport == ConnectorTransport::TcpClient ||
               transport == ConnectorTransport::TcpServer ||
               transport == ConnectorTransport::Udp;
    }

    bool set_startup_error(StartupError error, size_t connector_index) {
        if (last_startup_error_ == StartupError::None) {
            last_startup_error_ = error;
            last_failed_connector_index_ = connector_index;
        }
        return false;
    }

    void reset_startup_diagnostics() {
        last_startup_error_ = StartupError::None;
        last_failed_connector_index_ = config_.connector_count;
        connector_start_failure_count_ = 0;
    }

    bool validate_transport_config(const ConnectorConfig& connector) {
        switch (connector.transport.kind) {
        case ConnectorTransport::TcpClient:
            return connector.transport.tcp_client.host && connector.transport.tcp_client.host[0] && connector.transport.tcp_client.port != 0;
        case ConnectorTransport::TcpServer:
            return connector.transport.tcp_server.host && connector.transport.tcp_server.host[0] && connector.transport.tcp_server.port != 0;
        case ConnectorTransport::Serial:
            if (connector.transport.serial.baud == 0) return false;
#if defined(ARDUINO)
            return true;
#else
            return connector.transport.serial.device && connector.transport.serial.device[0];
#endif
        case ConnectorTransport::Udp:
            if (connector.access.allow_rx && connector.transport.udp.listen_port == 0) return false;
            if (connector.access.allow_tx && (!connector.transport.udp.remote_host || !connector.transport.udp.remote_host[0] || connector.transport.udp.remote_port == 0)) return false;
            return connector.transport.udp.listen_port != 0 || (connector.transport.udp.remote_host && connector.transport.udp.remote_host[0] && connector.transport.udp.remote_port != 0);
        default:
            return false;
        }
    }

    bool validate_config() {
        if (config_.signalk.port == 0) return set_startup_error(StartupError::InvalidSignalKPort, config_.connector_count);
        if (config_.signalk.max_connections == 0 || config_.signalk.max_connections > MaxSignalKConnections) return set_startup_error(StartupError::InvalidSignalKMaxConnections, config_.connector_count);
        if (config_.signalk.websocket.enabled) {
            if (config_.signalk.websocket.port == 0) return set_startup_error(StartupError::InvalidSignalKPort, config_.connector_count);
            if (config_.signalk.websocket.max_connections == 0 || config_.signalk.websocket.max_connections > MaxSignalKConnections) return set_startup_error(StartupError::InvalidSignalKMaxConnections, config_.connector_count);
        }
        if (config_.publisher.interval_us == 0) return set_startup_error(StartupError::InvalidPublisherInterval, config_.connector_count);
        if (!config_.connectors && config_.connector_count != 0) return set_startup_error(StartupError::InvalidConnectorList, config_.connector_count);

        for (size_t i = 0; i < config_.connector_count; ++i) {
            const ConnectorConfig& connector = config_.connectors[i];
            if (!connector.enabled) continue;
            if (!supported_protocol(connector.protocol.kind)) return set_startup_error(StartupError::UnsupportedConnectorProtocol, i);
            if (!supported_transport(connector.transport.kind)) return set_startup_error(StartupError::UnsupportedConnectorTransport, i);
            if (!validate_transport_config(connector)) return set_startup_error(StartupError::InvalidConnectorTransportConfig, i);
        }
        return true;
    }

    bool listen(async_event_loop::NativeTcpServer& server,
                async_event_loop::ITcpServerHandler& handler,
                const char* host,
                uint16_t port) {
        async_event_loop::TcpListenOptions options;
        options.host = host;
        options.port = port;
        options.reuse_address = true;
        return server.listen(options, handler);
    }

    int write_signal_k_hello(char* dst, size_t dst_size) const {
        SignalKHelloWriter writer;
        return writer.write(dst,
                            dst_size,
                            config_.identity.server_name,
                            config_.identity.server_version,
                            config_.identity.self);
    }

    bool send_signal_k_hello(async_event_loop::ITcpConnection& connection) {
        char hello[256];
        const int len = write_signal_k_hello(hello, sizeof(hello));
        if (len <= 0) return false;
        return write_all(connection, reinterpret_cast<const uint8_t*>(hello), static_cast<size_t>(len));
    }

    bool send_signal_k_hello_websocket(async_event_loop::ITcpConnection& connection) {
        char hello[256];
        int len = write_signal_k_hello(hello, sizeof(hello));
        if (len <= 0) return false;
        while (len > 0 && (hello[len - 1] == '\r' || hello[len - 1] == '\n' || hello[len - 1] == '\0')) --len;
        return write_websocket_text_frame(connection, hello, static_cast<size_t>(len));
    }

    bool write_websocket_text_frame(async_event_loop::ITcpConnection& connection, const char* text, size_t text_len) {
        uint8_t frame[websocket::DefaultMaxPayloadSize + 16];
        size_t frame_len = 0;
        if (!websocket::encode_text_frame(text, text_len, frame, sizeof(frame), frame_len)) return false;
        return write_all(connection, frame, frame_len);
    }

    bool write_websocket_control_frame(async_event_loop::ITcpConnection& connection, websocket::Opcode opcode, const uint8_t* payload, size_t payload_len) {
        uint8_t frame[128];
        size_t frame_len = 0;
        if (!websocket::encode_control_frame(opcode, payload, payload_len, frame, sizeof(frame), frame_len)) return false;
        return write_all(connection, frame, frame_len);
    }

    struct SignalKConnectionHandler : async_event_loop::ITcpLineServerHandler {
        explicit SignalKConnectionHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxSignalKConnections> connections;

        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            if (connections.size() >= owner.config_.signalk.max_connections) {
                connection.close();
                return;
            }
            ConnectionFlags flags{owner.config_.signalk.allow_rx, false};
            if (!connections.add(connection, flags)) {
                connection.close();
                return;
            }
            if (!owner.send_signal_k_hello(connection)) {
                connections.remove(connection);
                connection.close();
            }
        }
        void on_line(async_event_loop::ITcpConnection& connection, async_event_loop::LineView line) override {
            if (!connections.allow_rx(connection)) return;
            if (owner.is_subscribe_all_line(line)) {
                connections.set_allow_tx(connection, owner.config_.signalk.allow_tx);
                owner.publish_server_clock();
            }
        }
        void on_backpressure(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpBackpressureInfo&) override {
            connections.remove(connection);
            connection.close();
        }
        void on_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { connections.remove(connection); }
        void on_too_many_connections(async_event_loop::ITcpConnection& connection) override { connection.close(); }
    };

    struct WebSocketSignalKConnectionHandler : async_event_loop::ITcpServerHandler {
        explicit WebSocketSignalKConnectionHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {
            for (size_t i = 0; i < MaxSignalKConnections; ++i) slots[i].backpressure.configure(owner.signalk_backpressure_options());
        }

        enum class State : uint8_t { Empty, Handshaking, Open };

        struct Slot {
            async_event_loop::ITcpConnection* connection = nullptr;
            ConnectionFlags flags{};
            State state = State::Empty;
            size_t input_size = 0;
            uint8_t input[websocket::DefaultHandshakeBufferSize + 256]{};
            uint8_t payload[websocket::DefaultMaxPayloadSize + 1]{};
            async_event_loop::TcpBackpressureMonitor backpressure;
        };

        MiniSignalKServer& owner;
        Slot slots[MaxSignalKConnections];

        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            if (active_count() >= owner.config_.signalk.websocket.max_connections) {
                connection.close();
                return;
            }
            Slot* slot = acquire(connection);
            if (!slot) {
                connection.close();
                return;
            }
            slot->flags = ConnectionFlags{owner.config_.signalk.websocket.allow_rx, false};
            slot->state = State::Handshaking;
            slot->input_size = 0;
            slot->backpressure.reset();
        }

        void on_data(async_event_loop::ITcpConnection& connection) override {
            Slot* slot = find(connection);
            if (!slot || !slot->flags.allow_rx) return;
            while (connection.valid() && connection.readable()) {
                if (slot->input_size >= sizeof(slot->input)) {
                    close_slot(*slot);
                    return;
                }
                const int n = connection.read(slot->input + slot->input_size, sizeof(slot->input) - slot->input_size);
                if (n < 0) {
                    close_slot(*slot);
                    return;
                }
                if (n == 0) break;
                slot->input_size += static_cast<size_t>(n);
                if (!process_slot(*slot)) return;
            }
            check_backpressure(*slot);
        }

        void on_write_ready(async_event_loop::ITcpConnection& connection) override {
            Slot* slot = find(connection);
            if (slot) check_backpressure(*slot);
        }

        void on_close(async_event_loop::ITcpConnection& connection) override { release(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { release(connection); }

        void write_text_to_tx(const char* text, size_t text_len) {
            if (!text || text_len == 0) return;
            for (size_t i = 0; i < MaxSignalKConnections; ++i) {
                Slot& slot = slots[i];
                if (slot.connection && slot.connection->valid() && slot.state == State::Open && slot.flags.allow_tx) {
                    owner.write_websocket_text_frame(*slot.connection, text, text_len);
                    check_backpressure(slot);
                }
            }
        }

        size_t active_count() const {
            size_t count = 0;
            for (size_t i = 0; i < MaxSignalKConnections; ++i) if (slots[i].connection) ++count;
            return count;
        }

        Slot* find(async_event_loop::ITcpConnection& connection) {
            for (size_t i = 0; i < MaxSignalKConnections; ++i) if (slots[i].connection == &connection) return &slots[i];
            return nullptr;
        }

        Slot* acquire(async_event_loop::ITcpConnection& connection) {
            if (Slot* existing = find(connection)) return existing;
            for (size_t i = 0; i < MaxSignalKConnections; ++i) {
                if (!slots[i].connection) {
                    slots[i].connection = &connection;
                    slots[i].state = State::Handshaking;
                    slots[i].input_size = 0;
                    return &slots[i];
                }
            }
            return nullptr;
        }

        void release(async_event_loop::ITcpConnection& connection) {
            Slot* slot = find(connection);
            if (!slot) return;
            slot->connection = nullptr;
            slot->flags = ConnectionFlags{};
            slot->state = State::Empty;
            slot->input_size = 0;
            slot->backpressure.reset();
        }

        void close_slot(Slot& slot) {
            async_event_loop::ITcpConnection* connection = slot.connection;
            if (!connection) return;
            release(*connection);
            connection->close();
        }

        bool process_slot(Slot& slot) {
            if (slot.state == State::Handshaking) {
                const size_t header_len = websocket_header_length(slot.input, slot.input_size);
                if (header_len == 0) return true;
                websocket::HandshakeRequest request;
                const websocket::HandshakeResult parsed = websocket::parse_client_handshake(reinterpret_cast<const char*>(slot.input), header_len, request);
                if (parsed != websocket::HandshakeResult::Ok) {
                    close_slot(slot);
                    return false;
                }
                char response[192];
                const websocket::HandshakeResult written = websocket::write_server_handshake_response(request.key, response, sizeof(response));
                if (written != websocket::HandshakeResult::Ok || !owner.write_all(*slot.connection, reinterpret_cast<const uint8_t*>(response), strlen(response))) {
                    close_slot(slot);
                    return false;
                }
                if (!owner.send_signal_k_hello_websocket(*slot.connection)) {
                    close_slot(slot);
                    return false;
                }
                consume(slot, header_len);
                slot.state = State::Open;
            }

            while (slot.state == State::Open && slot.input_size > 0) {
                websocket::FrameView frame;
                size_t consumed = 0;
                const websocket::FrameDecodeResult decoded = websocket::decode_frame(slot.input, slot.input_size, slot.payload, websocket::DefaultMaxPayloadSize, frame, consumed, true);
                if (decoded == websocket::FrameDecodeResult::NeedMore) return true;
                if (decoded != websocket::FrameDecodeResult::Ok || consumed == 0) {
                    close_slot(slot);
                    return false;
                }
                if (!handle_frame(slot, frame)) return false;
                consume(slot, consumed);
            }
            return true;
        }

        bool handle_frame(Slot& slot, const websocket::FrameView& frame) {
            switch (frame.opcode) {
            case websocket::Opcode::Text:
                if (frame.payload_len > websocket::DefaultMaxPayloadSize) {
                    close_slot(slot);
                    return false;
                }
                slot.payload[frame.payload_len] = '\0';
                if (owner.is_subscribe_all_text(reinterpret_cast<const char*>(slot.payload))) {
                    slot.flags.allow_tx = owner.config_.signalk.websocket.allow_tx;
                    owner.publish_server_clock();
                }
                return true;
            case websocket::Opcode::Ping:
                if (!owner.write_websocket_control_frame(*slot.connection, websocket::Opcode::Pong, frame.payload, frame.payload_len)) {
                    close_slot(slot);
                    return false;
                }
                return true;
            case websocket::Opcode::Close:
                owner.write_websocket_control_frame(*slot.connection, websocket::Opcode::Close, frame.payload, frame.payload_len);
                close_slot(slot);
                return false;
            case websocket::Opcode::Pong:
                return true;
            default:
                close_slot(slot);
                return false;
            }
        }

        static size_t websocket_header_length(const uint8_t* data, size_t size) {
            if (!data || size < 4) return 0;
            for (size_t i = 3; i < size; ++i) {
                if (data[i - 3] == '\r' && data[i - 2] == '\n' && data[i - 1] == '\r' && data[i] == '\n') return i + 1;
            }
            return 0;
        }

        static void consume(Slot& slot, size_t count) {
            if (count >= slot.input_size) {
                slot.input_size = 0;
                return;
            }
            memmove(slot.input, slot.input + count, slot.input_size - count);
            slot.input_size -= count;
        }

        void check_backpressure(Slot& slot) {
            if (!slot.connection || !slot.connection->valid()) return;
            async_event_loop::TcpBackpressureInfo info;
            if (slot.backpressure.check(*slot.connection, &info) && info.close_on_limit) close_slot(slot);
        }
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

    struct ConnectorTcpRawServerHandler : async_event_loop::ITcpServerHandler {
        ConnectorTcpRawServerHandler(MiniSignalKServer& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        MiniSignalKServer& owner;
        size_t index;
        ConnectionRegistry<MaxConnectionsPerConnector> connections;

        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            if (!connections.add(connection, owner.connector_connection_flags(index))) connection.close();
        }
        void on_data(async_event_loop::ITcpConnection& connection) override {
            if (!connections.allow_rx(connection)) return;
            owner.handle_connector_stream_bytes(index, connection);
        }
        void on_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { connections.remove(connection); }
    };

    struct ConnectorTcpClientHandler : async_event_loop::ITcpClientHandler {
        ConnectorTcpClientHandler(MiniSignalKServer& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        MiniSignalKServer& owner;
        size_t index;

        void on_data(async_event_loop::ITcpConnection& connection) override {
            if (!owner.connector_allow_rx(index)) return;
            if (owner.connector_protocol(index) == ConnectorProtocol::SeaTalk1) {
                owner.handle_connector_stream_bytes(index, connection);
                return;
            }
            char text[160];
            while (connection.read_line(text, sizeof(text))) owner.handle_connector_nmea_line(index, text);
        }
        void on_close(async_event_loop::ITcpConnection&) override {}
        void on_error(int) override {}
    };

    struct ConnectorRuntimeSlot {
        ConnectorRuntimeSlot(MiniSignalKServer& owner_ref, size_t connector_index)
            : owner(owner_ref),
              index(connector_index),
              tcp_server(owner_ref.loop_.scheduler()),
              tcp_client(owner_ref.loop_.scheduler()),
#if defined(ARDUINO)
              serial_stream(Serial),
#else
              serial_stream(),
#endif
              serial_reader(serial_stream, async_event_loop::LineProtocolOptions{}, [this](async_event_loop::LineView line) {
                  owner.handle_connector_serial_line(index, line);
              }),
              seatalk_input(owner_ref.store_),
              tcp_server_handler(owner_ref, connector_index),
              tcp_raw_server_handler(owner_ref, connector_index),
              tcp_client_handler(owner_ref, connector_index),
              tcp_line_handler(tcp_server_handler) {}

        MiniSignalKServer& owner;
        size_t index = 0;
        ConnectorConfig config;
        ConnectionFlags connection_flags{true, false};
        bool nmea0183_validate_checksum = false;
        bool started = false;
        async_event_loop::NativeTcpServer tcp_server;
        async_event_loop::NativeTcpClient tcp_client;
        async_event_loop::NativeSerialStream serial_stream;
        async_event_loop::LineProtocolReader<192> serial_reader;
        UdpListenerStream udp_listener;
        SeaTalkInput<Real> seatalk_input;
        ConnectorTcpServerHandler tcp_server_handler;
        ConnectorTcpRawServerHandler tcp_raw_server_handler;
        ConnectorTcpClientHandler tcp_client_handler;
        async_event_loop::TcpLineServerHandler<192, MaxConnectionsPerConnector> tcp_line_handler;
        async_event_loop::EventHandle serial_event{};
        async_event_loop::EventHandle udp_event{};
    };

    ConnectorRuntimeSlot& connector_slot(size_t index) { return *connector_slots_[index]; }
    const ConnectorRuntimeSlot& connector_slot(size_t index) const { return *connector_slots_[index]; }
    const ConnectionFlags& connector_connection_flags(size_t index) const { return connector_slot(index).connection_flags; }
    bool connector_allow_rx(size_t index) const { return connector_slot(index).connection_flags.allow_rx; }
    ConnectorProtocol connector_protocol(size_t index) const { return connector_slot(index).config.protocol.kind; }
    ship_data_model::SensorSource connector_sensor_source(size_t index) const { return sensor_source_for_transport(connector_slot(index).config.transport.kind); }

    bool start_configured_connectors() {
        connector_slots_.clear();
        if (!config_.connectors || config_.connector_count == 0) return true;
        connector_slots_.reserve(config_.connector_count);
        for (size_t i = 0; i < config_.connector_count; ++i) {
            const ConnectorConfig& connector = config_.connectors[i];
            std::unique_ptr<ConnectorRuntimeSlot> slot(new ConnectorRuntimeSlot(*this, i));
            slot->config = connector;
            slot->connection_flags = ConnectionFlags{connector.access.allow_rx, connector.access.allow_tx};
            slot->nmea0183_validate_checksum = connector.protocol.kind == ConnectorProtocol::Nmea0183
                ? effective_nmea0183_validate_checksum(connector.protocol.nmea0183, connector.transport.kind)
                : false;
            connector_slots_.push_back(std::move(slot));
            ConnectorRuntimeSlot& runtime = *connector_slots_.back();
            if (!connector.enabled) continue;
            if (connector.protocol.kind == ConnectorProtocol::Nmea0183) runtime.started = start_nmea0183_connector(runtime);
            else if (connector.protocol.kind == ConnectorProtocol::SeaTalk1) runtime.started = start_seatalk_connector(runtime);
            if (!runtime.started) {
                ++connector_start_failure_count_;
                return set_startup_error(StartupError::ConnectorStartFailed, i);
            }
        }
        return true;
    }

    bool start_nmea0183_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: {
            async_event_loop::TcpConnectOptions options;
            options.host = slot.config.transport.tcp_client.host;
            options.port = slot.config.transport.tcp_client.port;
            return slot.tcp_client.connect(options, slot.tcp_client_handler);
        }
        case ConnectorTransport::TcpServer: return listen(slot.tcp_server, slot.tcp_line_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_nmea0183_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }

    bool start_seatalk_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: {
            async_event_loop::TcpConnectOptions options;
            options.host = slot.config.transport.tcp_client.host;
            options.port = slot.config.transport.tcp_client.port;
            return slot.tcp_client.connect(options, slot.tcp_client_handler);
        }
        case ConnectorTransport::TcpServer: return listen(slot.tcp_server, slot.tcp_raw_server_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_seatalk_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }

    bool start_nmea0183_serial_connector(ConnectorRuntimeSlot& slot) {
#if defined(ARDUINO)
        Serial.begin(slot.config.transport.serial.baud);
#else
        if (!slot.serial_stream.open(slot.config.transport.serial.device, slot.config.transport.serial.baud)) return false;
#endif
        if (!slot.connection_flags.allow_rx) return true;
        const size_t connector_index = slot.index;
        slot.serial_event = loop_.on_bytes_ready(slot.serial_stream, [this, connector_index]() {
            connector_slot(connector_index).serial_reader.poll(loop_.clock().micros());
        });
        return static_cast<bool>(slot.serial_event);
    }

    bool start_seatalk_serial_connector(ConnectorRuntimeSlot& slot) {
#if defined(ARDUINO)
        Serial.begin(slot.config.transport.serial.baud);
#else
        if (!slot.serial_stream.open(slot.config.transport.serial.device, slot.config.transport.serial.baud)) return false;
#endif
        if (!slot.connection_flags.allow_rx) return true;
        const size_t connector_index = slot.index;
        slot.serial_event = loop_.on_bytes_ready(slot.serial_stream, [this, connector_index]() {
            handle_connector_serial_bytes(connector_index);
        });
        return static_cast<bool>(slot.serial_event);
    }

    bool start_udp_listener(ConnectorRuntimeSlot& slot) {
        if (!slot.udp_listener.open(slot.config.transport.udp)) return false;
        if (!slot.connection_flags.allow_rx) return true;
        const size_t connector_index = slot.index;
        slot.udp_event = loop_.on_bytes_ready(slot.udp_listener, [this, connector_index]() {
            handle_connector_udp_datagrams(connector_index);
        });
        return static_cast<bool>(slot.udp_event);
    }

    void handle_connector_serial_line(size_t index, async_event_loop::LineView line) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (!slot.connection_flags.allow_rx) return;
        char text[160];
        async_event_loop::line_to_cstr(line, text);
        handle_connector_nmea_line(index, text);
    }

    void handle_connector_serial_bytes(size_t index) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (!slot.connection_flags.allow_rx) return;
        handle_connector_stream_bytes(index, slot.serial_stream);
    }

    void handle_connector_stream_bytes(size_t index, async_event_loop::IByteStream& stream) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        const SourceId source_id = static_cast<SourceId>(10 + index);
        const ship_data_model::SensorSource source = connector_sensor_source(index);
        uint8_t buf[128];
        bool changed = false;
        while (stream.readable()) {
            const int n = stream.read(buf, sizeof(buf));
            if (n <= 0) break;
            if (slot.seatalk_input.feed_octets(buf, static_cast<size_t>(n), source_id, loop_.clock().micros(), source)) changed = true;
        }
        if (changed) publish();
    }

    void handle_connector_udp_datagrams(size_t index) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (!slot.connection_flags.allow_rx) return;
        const ship_data_model::SensorSource source = connector_sensor_source(index);
        uint8_t buf[512];
        while (slot.udp_listener.readable()) {
            const int n = slot.udp_listener.read(buf, sizeof(buf) - 1);
            if (n <= 0) break;
            if (slot.config.protocol.kind == ConnectorProtocol::SeaTalk1) {
                if (slot.seatalk_input.feed_datagram(buf, static_cast<size_t>(n), static_cast<SourceId>(10 + index), loop_.clock().micros(), source)) publish();
                continue;
            }
            buf[n] = 0;
            char* begin = reinterpret_cast<char*>(buf);
            char* line = begin;
            for (int i = 0; i <= n; ++i) {
                if (begin[i] == '\r' || begin[i] == '\n' || begin[i] == '\0') {
                    begin[i] = '\0';
                    if (*line) handle_connector_nmea_line(index, line);
                    line = begin + i + 1;
                }
            }
        }
    }

    void handle_connector_nmea_line(size_t index, const char* text) {
        const ConnectorRuntimeSlot& slot = connector_slot(index);
        const SourceId source_id = static_cast<SourceId>(10 + index);
        if (nmea_.feed_line(text, source_id, loop_.clock().micros(), slot.nmea0183_validate_checksum, connector_sensor_source(index))) publish();
    }

    uint64_t server_clock_s() const {
#if !defined(ARDUINO)
        const time_t now = ::time(nullptr);
        if (now > 0) return static_cast<uint64_t>(now);
#endif
        return loop_.clock().micros() / 1000000ULL;
    }

    const char* server_source_label() const {
        if (config_.publisher.source_label && config_.publisher.source_label[0]) return config_.publisher.source_label;
        return config_.identity.server_name ? config_.identity.server_name : "signalk-mini";
    }

    void update_server_clock_model(uint64_t now_us) {
        auto& server = store_.model().comm.server;
        server.source.value = ship_data_model::SensorSource::signalk;
        server.clock_s.set(server_clock_s(), now_us);
        server.last_update_us = now_us;
    }

    void publish_server_clock() {
        const uint64_t now_us = loop_.clock().micros();
        update_server_clock_model(now_us);
        char json[192];
        const unsigned long clock_s = static_cast<unsigned long>(store_.model().comm.server.clock_s.value);
        const int len = snprintf(json,
                                 sizeof(json),
                                 "{\"updates\":[{\"source\":{\"label\":\"%s\"},\"values\":[{\"path\":\"communication.server.clock\",\"value\":%lu}]}]}\r\n",
                                 server_source_label(),
                                 clock_s);
        if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) return;
        signalk_connections_.connections.write_signal_k_delta(json, static_cast<size_t>(len));
        int ws_len = len;
        while (ws_len > 0 && (json[ws_len - 1] == '\r' || json[ws_len - 1] == '\n' || json[ws_len - 1] == '\0')) --ws_len;
        signalk_websocket_connections_.write_text_to_tx(json, static_cast<size_t>(ws_len));
    }

    bool write_all(async_event_loop::IByteStream& stream, const uint8_t* data, size_t length) {
        if (!data || length == 0 || !stream.valid() || !stream.writable()) return false;
        const int n = stream.write(data, length);
        return n == static_cast<int>(length);
    }

    bool transmit_to_tcp_server_connections(ConnectionRegistry<MaxConnectionsPerConnector>& connections, const uint8_t* data, size_t length) {
        bool any = false;
        connections.for_each_tx([&](async_event_loop::ITcpConnection& connection) {
            if (write_all(connection, data, length)) any = true;
        });
        return any;
    }

    bool transmit_bytes_to_slot(ConnectorRuntimeSlot& slot, const uint8_t* data, size_t length) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::Serial: return write_all(slot.serial_stream, data, length);
        case ConnectorTransport::TcpClient: return slot.tcp_client.connected() && write_all(slot.tcp_client.connection(), data, length);
        case ConnectorTransport::TcpServer:
            if (slot.config.protocol.kind == ConnectorProtocol::SeaTalk1) return transmit_to_tcp_server_connections(slot.tcp_raw_server_handler.connections, data, length);
            return transmit_to_tcp_server_connections(slot.tcp_server_handler.connections, data, length);
        case ConnectorTransport::Udp: return write_all(slot.udp_listener, data, length);
        default: return false;
        }
    }

    bool transmit_seatalk_binary(ConnectorRuntimeSlot& slot, const seatalk::SeaTalkFrame& frame) { return transmit_bytes_to_slot(slot, frame.bytes, frame.length); }

    bool transmit_seatalk_over_nmea(ConnectorRuntimeSlot& slot, const seatalk::SeaTalkFrame& frame) {
        char sentence[96];
        const size_t n = nmea0183_connector::make_seatalk_frame(sentence, sizeof(sentence), frame);
        if (n == 0) return false;
        return transmit_bytes_to_slot(slot, reinterpret_cast<const uint8_t*>(sentence), n);
    }

    void publish() {
        struct DeltaFanout {
            ConnectionRegistry<MaxSignalKConnections>& tcp;
            WebSocketSignalKConnectionHandler& websocket;

            void write_signal_k_delta(const char* json, size_t len) {
                tcp.write_signal_k_delta(json, len);
                if (!json || len == 0) return;
                size_t text_len = len;
                while (text_len > 0 && (json[text_len - 1] == '\r' || json[text_len - 1] == '\n' || json[text_len - 1] == '\0')) --text_len;
                websocket.write_text_to_tx(json, text_len);
            }
        } fanout{signalk_connections_.connections, signalk_websocket_connections_};
        publisher_.publish_pending(fanout);
    }

    SignalKMiniConfig config_;
    async_event_loop::EventLoop<> loop_;
    async_event_loop::NativeTcpServer signalk_tcp_server_;
    async_event_loop::NativeTcpServer signalk_websocket_server_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_{store_};
    SeaTalkInput<Real> seatalk_{store_};
    SignalKPublisher<Real> publisher_;
    SignalKConnectionHandler signalk_connections_;
    WebSocketSignalKConnectionHandler signalk_websocket_connections_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKConnections> signalk_line_handler_;
    std::vector<std::unique_ptr<ConnectorRuntimeSlot>> connector_slots_;
    async_event_loop::EventHandle timer_{};
    async_event_loop::EventHandle clock_timer_{};
    StartupError last_startup_error_ = StartupError::None;
    size_t last_failed_connector_index_ = 0;
    uint32_t connector_start_failure_count_ = 0;
};

} // namespace signalk_mini
