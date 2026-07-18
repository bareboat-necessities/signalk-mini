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
#include <ubx.hpp>
#if !defined(ARDUINO)
#include <gpsd.hpp>
#endif
#include "config.hpp"
#include "connection_registry.hpp"
#include "nmea0183_input.hpp"
#include "publisher.hpp"
#include "signalk_full_model_writer.hpp"
#include "signalk_identity.hpp"
#include "seatalk_input.hpp"
#include "ubx_input.hpp"
#if !defined(ARDUINO)
#include "gpsd_input.hpp"
#endif
#include "signalk_hello_writer.hpp"

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
        InvalidSignalKIdentity,
        InvalidPublisherInterval,
        InvalidConnectorList,
        UnsupportedConnectorProtocol,
        UnsupportedConnectorTransport,
        InvalidConnectorTransportConfig,
        InvalidConnectorProtocolConfig,
        InvalidConnectorReconnectConfig,
        ConnectorStartFailed,
    };

    explicit MiniSignalKServer(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : config_(config),
          signalk_tcp_server_(loop_.scheduler()),
          signalk_websocket_server_(loop_.scheduler()),
          publisher_(store_, config_),
          signalk_connections_(*this),
          signalk_websocket_handler_(*this),
          signalk_websocket_protocol_(signalk_websocket_handler_, signalk_backpressure_options()),
          signalk_line_handler_(signalk_connections_, signalk_options()) {}

    bool begin() {
        reset_startup_diagnostics();
        if (!validate_config()) return false;
        if (!loop_.valid()) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        if (!listen(signalk_tcp_server_, signalk_line_handler_, config_.signalk.host, config_.signalk.port)) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        if (config_.signalk.websocket.enabled) {
            if (!listen(signalk_websocket_server_, signalk_websocket_protocol_, config_.signalk.websocket.host, config_.signalk.websocket.port)) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
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
    UbxInput<Real>& ubx() { return ubx_; }

    uint64_t dropped_change_count() const { return store_.dropped_change_count(); }
    uint64_t dropped_publish_count() const { return publisher_.dropped_publish_count(); }
    uint64_t published_delta_count() const { return publisher_.published_delta_count(); }
    uint64_t published_snapshot_count() const { return publisher_.published_snapshot_count(); }
    int write_full_model(char* dst, size_t dst_size) const {
        SignalKFullModelWriter<Real> writer;
        return writer.write(dst, dst_size, store_, config_, loop_.clock().micros());
    }
    StartupError last_startup_error() const { return last_startup_error_; }
    size_t last_failed_connector_index() const { return last_failed_connector_index_; }
    uint32_t connector_start_failure_count() const { return connector_start_failure_count_; }
    uint32_t connector_reconnect_count() const {
        uint32_t total = 0;
        for (const auto& slot : connector_slots_) if (slot) total += slot->reconnect_count;
        return total;
    }
    uint32_t ubx_frame_count() const {
        uint32_t total = ubx_.receiver().diagnostics().frame_count;
        for (const auto& slot : connector_slots_) if (slot && slot->ubx_input) total += slot->ubx_input->receiver().diagnostics().frame_count;
        return total;
    }
    uint32_t ubx_checksum_error_count() const {
        uint32_t total = ubx_.receiver().diagnostics().checksum_error_count;
        for (const auto& slot : connector_slots_) if (slot && slot->ubx_input) total += slot->ubx_input->receiver().diagnostics().checksum_error_count;
        return total;
    }
    uint32_t ubx_oversized_frame_count() const {
        uint32_t total = ubx_.receiver().diagnostics().oversized_frame_count;
        for (const auto& slot : connector_slots_) if (slot && slot->ubx_input) total += slot->ubx_input->receiver().diagnostics().oversized_frame_count;
        return total;
    }
    uint32_t ubx_unsupported_frame_count() const {
        uint32_t total = ubx_.receiver().diagnostics().unsupported_frame_count;
        for (const auto& slot : connector_slots_) if (slot && slot->ubx_input) total += slot->ubx_input->receiver().diagnostics().unsupported_frame_count;
        return total;
    }
#if !defined(ARDUINO)
    uint32_t gpsd_record_count() const {
        uint32_t total = 0;
        for (const auto& slot : connector_slots_) if (slot && slot->gpsd_input) total += slot->gpsd_input->client().diagnostics().record_count;
        return total;
    }
    uint32_t gpsd_malformed_record_count() const {
        uint32_t total = 0;
        for (const auto& slot : connector_slots_) if (slot && slot->gpsd_input) total += slot->gpsd_input->client().diagnostics().malformed_record_count;
        return total;
    }
    uint32_t gpsd_oversized_record_count() const {
        uint32_t total = 0;
        for (const auto& slot : connector_slots_) if (slot && slot->gpsd_input) total += slot->gpsd_input->client().diagnostics().oversized_record_count;
        return total;
    }
    uint32_t gpsd_device_mismatch_count() const {
        uint32_t total = 0;
        for (const auto& slot : connector_slots_) if (slot && slot->gpsd_input) total += slot->gpsd_input->client().diagnostics().device_mismatch_count;
        return total;
    }
#endif

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

    static size_t signalk_text_len(const char* json, size_t len) {
        if (!json) return 0;
        while (len > 0 && (json[len - 1] == '\r' || json[len - 1] == '\n' || json[len - 1] == '\0')) --len;
        return len;
    }

    static ship_data_model::SensorSource sensor_source_for_connector(const ConnectorConfig& connector) {
        if (connector.protocol.kind == ConnectorProtocol::Ubx) return ship_data_model::SensorSource::ubx;
        if (connector.protocol.kind == ConnectorProtocol::Gpsd) return ship_data_model::SensorSource::gpsd;
        switch (connector.transport.kind) {
        case ConnectorTransport::Serial: return ship_data_model::SensorSource::serial;
        case ConnectorTransport::TcpClient:
        case ConnectorTransport::TcpServer:
        case ConnectorTransport::Udp: return ship_data_model::SensorSource::tcp;
        default: return ship_data_model::SensorSource::none;
        }
    }

    static bool supported_protocol(ConnectorProtocol protocol) {
        if (protocol == ConnectorProtocol::Nmea0183 || protocol == ConnectorProtocol::SeaTalk1 || protocol == ConnectorProtocol::Ubx) return true;
#if !defined(ARDUINO)
        if (protocol == ConnectorProtocol::Gpsd) return true;
#endif
        return false;
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
        if (!config_.identity.signalk_version || !config_.identity.signalk_version[0] ||
            !signalk_valid_self_context(config_.identity.self)) {
            return set_startup_error(StartupError::InvalidSignalKIdentity, config_.connector_count);
        }
        if (config_.signalk.max_connections == 0 || config_.signalk.max_connections > MaxSignalKConnections) return set_startup_error(StartupError::InvalidSignalKMaxConnections, config_.connector_count);
        if (config_.signalk.websocket.enabled) {
            if (config_.signalk.websocket.port == 0) return set_startup_error(StartupError::InvalidSignalKPort, config_.connector_count);
            if (config_.signalk.websocket.max_connections == 0 || config_.signalk.websocket.max_connections > MaxSignalKConnections) return set_startup_error(StartupError::InvalidSignalKMaxConnections, config_.connector_count);
        }
        if (config_.publisher.interval_us == 0) return set_startup_error(StartupError::InvalidPublisherInterval, config_.connector_count);
        if (!config_.connectors && config_.connector_count != 0) return set_startup_error(StartupError::InvalidConnectorList, config_.connector_count);
        if (config_.connector_count > MaxConnectorSourceCount) return set_startup_error(StartupError::InvalidConnectorList, config_.connector_count);

        for (size_t i = 0; i < config_.connector_count; ++i) {
            const ConnectorConfig& connector = config_.connectors[i];
            if (!connector.enabled) continue;
            if (!supported_protocol(connector.protocol.kind)) return set_startup_error(StartupError::UnsupportedConnectorProtocol, i);
            if (!supported_transport(connector.transport.kind)) return set_startup_error(StartupError::UnsupportedConnectorTransport, i);
            if (!validate_transport_config(connector)) return set_startup_error(StartupError::InvalidConnectorTransportConfig, i);
            if (connector.protocol.kind == ConnectorProtocol::Gpsd &&
                (connector.transport.kind != ConnectorTransport::TcpClient || !connector.access.allow_rx || connector.access.allow_tx)) {
                return set_startup_error(StartupError::InvalidConnectorProtocolConfig, i);
            }
            if (connector.protocol.kind == ConnectorProtocol::Ubx && connector.protocol.ubx.configure_receiver) return set_startup_error(StartupError::InvalidConnectorProtocolConfig, i);
            if (connector.transport.kind == ConnectorTransport::TcpClient && connector.reconnect.enabled &&
                (connector.reconnect.initial_delay_ms == 0 || connector.reconnect.maximum_delay_ms < connector.reconnect.initial_delay_ms)) {
                return set_startup_error(StartupError::InvalidConnectorReconnectConfig, i);
            }
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
        char timestamp[32];
        const char* timestamp_ptr = format_signalk_timestamp_utc(timestamp, sizeof(timestamp)) ? timestamp : nullptr;
        return writer.write(dst,
                            dst_size,
                            config_.identity.server_name,
                            config_.identity.signalk_version,
                            config_.identity.self,
                            timestamp_ptr);
    }

    bool send_signal_k_hello(async_event_loop::ITcpConnection& connection) {
        char hello[256];
        const int len = write_signal_k_hello(hello, sizeof(hello));
        if (len <= 0) return false;
        return write_all(connection, reinterpret_cast<const uint8_t*>(hello), static_cast<size_t>(len));
    }

    bool send_signal_k_hello_websocket(async_event_loop::ITcpConnection& connection) {
        char hello[256];
        const int len = write_signal_k_hello(hello, sizeof(hello));
        if (len <= 0) return false;
        const size_t text_len = signalk_text_len(hello, static_cast<size_t>(len));
        return signalk_websocket_protocol_.write_text(connection, hello, text_len);
    }

    static bool http_header_name_matches(const char* begin, size_t len, const char* expected) {
        if (!begin || !expected || len != strlen(expected)) return false;
        for (size_t i = 0; i < len; ++i) {
            char a = begin[i];
            char b = expected[i];
            if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
            if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
            if (a != b) return false;
        }
        return true;
    }

    static bool copy_http_host(const char* header, size_t header_len, char* dst, size_t dst_size) {
        if (!header || !dst || dst_size == 0) return false;
        const char* cursor = header;
        const char* end = header + header_len;
        while (cursor < end) {
            const char* line_end = cursor;
            while (line_end + 1 < end && !(line_end[0] == '\r' && line_end[1] == '\n')) ++line_end;
            const char* colon = cursor;
            while (colon < line_end && *colon != ':') ++colon;
            if (colon < line_end && http_header_name_matches(cursor, static_cast<size_t>(colon - cursor), "Host")) {
                const char* value = colon + 1;
                while (value < line_end && (*value == ' ' || *value == '\t')) ++value;
                const size_t len = static_cast<size_t>(line_end - value);
                if (len == 0 || len >= dst_size) return false;
                memcpy(dst, value, len);
                dst[len] = '\0';
                return true;
            }
            if (line_end + 1 >= end) break;
            cursor = line_end + 2;
        }
        return false;
    }

    bool handle_signalk_http_request(async_event_loop::ITcpConnection& connection,
                                     const char* header,
                                     size_t header_len) {
        if (!header || header_len < 16 || strncmp(header, "GET ", 4) != 0) return false;
        const char* target = header + 4;
        const char* end = header + header_len;
        const char* target_end = target;
        while (target_end < end && *target_end != ' ') ++target_end;
        const size_t target_len = static_cast<size_t>(target_end - target);

        if (target_len == 1 && target[0] == '/') {
            static constexpr char Body[] =
                "<!doctype html><html><head><meta charset=\"utf-8\"><title>SignalK Mini</title></head>"
                "<body><h1>SignalK Mini</h1><p>Lightweight Signal K server is running.</p>"
                "<p><a href=\"/signalk\">Signal K discovery</a></p></body></html>";
            char response[384];
            const int response_len = snprintf(
                response,
                sizeof(response),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Content-Length: %u\r\n"
                "Connection: close\r\n"
                "\r\n%s",
                static_cast<unsigned>(sizeof(Body) - 1),
                Body);
            if (response_len <= 0 || static_cast<size_t>(response_len) >= sizeof(response)) return false;
            return write_all(connection,
                             reinterpret_cast<const uint8_t*>(response),
                             static_cast<size_t>(response_len));
        }

        const bool discovery =
            (target_len == 8 && strncmp(target, "/signalk", 8) == 0) ||
            (target_len == 9 && strncmp(target, "/signalk/", 9) == 0);
        if (!discovery) return false;

        char host[128];
        if (!copy_http_host(header, header_len, host, sizeof(host))) {
            snprintf(host, sizeof(host), "127.0.0.1:%u", static_cast<unsigned>(config_.signalk.websocket.port));
        }

        char body[512];
        const int body_len = snprintf(
            body,
            sizeof(body),
            "{\"endpoints\":{\"v1\":{\"version\":\"%s\",\"signalk-ws\":\"ws://%s/signalk/v1/stream\"}},\"server\":{\"id\":\"%s\",\"version\":\"%s\"}}",
            config_.identity.signalk_version ? config_.identity.signalk_version : "1.8.2",
            host,
            config_.identity.server_name ? config_.identity.server_name : "signalk-mini",
            config_.identity.server_version ? config_.identity.server_version : "0.1.0");
        if (body_len <= 0 || static_cast<size_t>(body_len) >= sizeof(body)) return false;

        char response[768];
        const int response_len = snprintf(
            response,
            sizeof(response),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %u\r\nConnection: close\r\nAccess-Control-Allow-Origin: *\r\n\r\n%s",
            static_cast<unsigned>(body_len),
            body);
        if (response_len <= 0 || static_cast<size_t>(response_len) >= sizeof(response)) return false;
        return write_all(connection,
                         reinterpret_cast<const uint8_t*>(response),
                         static_cast<size_t>(response_len));
    }

    static bool is_signalk_stream_resource(const char* resource) {
        if (!resource) return false;
        static constexpr char StreamPath[] = "/signalk/v1/stream";
        const size_t path_len = sizeof(StreamPath) - 1;
        return strncmp(resource, StreamPath, path_len) == 0 &&
               (resource[path_len] == '\0' || resource[path_len] == '?');
    }

    enum class InitialStreamScope : uint8_t { None, Self, All };

    static bool query_parameter_equals(const char* resource, const char* name, const char* expected) {
        if (!resource || !name || !expected) return false;
        const char* query = strchr(resource, '?');
        if (!query) return false;
        ++query;
        const size_t name_len = strlen(name);
        const size_t expected_len = strlen(expected);
        while (*query) {
            const char* end = strchr(query, '&');
            if (!end) end = query + strlen(query);
            const char* equals = static_cast<const char*>(memchr(query, '=', static_cast<size_t>(end - query)));
            if (equals && static_cast<size_t>(equals - query) == name_len && strncmp(query, name, name_len) == 0) {
                const char* value = equals + 1;
                return static_cast<size_t>(end - value) == expected_len && strncmp(value, expected, expected_len) == 0;
            }
            query = *end ? end + 1 : end;
        }
        return false;
    }

    static InitialStreamScope websocket_initial_scope(const char* resource) {
        if (query_parameter_equals(resource, "subscribe", "none")) return InitialStreamScope::None;
        if (query_parameter_equals(resource, "subscribe", "all")) return InitialStreamScope::All;
        return InitialStreamScope::Self;
    }

    static bool websocket_should_send_current_values(const char* resource) {
        return !query_parameter_equals(resource, "sendCachedValues", "false");
    }

    struct TcpConnectionDeltaSink {
        MiniSignalKServer& owner;
        async_event_loop::ITcpConnection& connection;
        void write_signal_k_delta(const char* json, size_t len) {
            owner.write_all(connection, reinterpret_cast<const uint8_t*>(json), len);
        }
    };

    struct WebSocketConnectionDeltaSink {
        MiniSignalKServer& owner;
        async_event_loop::ITcpConnection& connection;
        void write_signal_k_delta(const char* json, size_t len) {
            const size_t text_len = owner.signalk_text_len(json, len);
            if (text_len) owner.signalk_websocket_protocol_.write_text(connection, json, text_len);
        }
    };

    void send_current_values_tcp(async_event_loop::ITcpConnection& connection) {
        update_server_clock_model(loop_.clock().micros());
        TcpConnectionDeltaSink sink{*this, connection};
        publisher_.publish_current(sink, loop_.clock().micros(), true);
    }

    void send_current_values_websocket(async_event_loop::ITcpConnection& connection, bool include_other_contexts) {
        update_server_clock_model(loop_.clock().micros());
        WebSocketConnectionDeltaSink sink{*this, connection};
        publisher_.publish_current(sink, loop_.clock().micros(), include_other_contexts);
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
                if (owner.config_.signalk.allow_tx) owner.send_current_values_tcp(connection);
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

    struct SignalKWebSocketHandler : async_event_loop::IWebSocketServerHandler {
        explicit SignalKWebSocketHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxSignalKConnections> connections;

        bool on_http_request(async_event_loop::ITcpConnection& connection,
                             const async_event_loop::TcpPeerInfo&,
                             const char* header,
                             size_t header_len) override {
            return owner.handle_signalk_http_request(connection, header, header_len);
        }

        bool accept_websocket_request(const async_event_loop::websocket::HandshakeRequest& request) override {
            return owner.is_signalk_stream_resource(request.resource);
        }

        void on_websocket_open(async_event_loop::ITcpConnection& connection,
                               const async_event_loop::TcpPeerInfo&,
                               const async_event_loop::websocket::HandshakeRequest& request) override {
            if (connections.size() >= owner.config_.signalk.websocket.max_connections) {
                connection.close();
                return;
            }
            const InitialStreamScope scope = owner.websocket_initial_scope(request.resource);
            const bool active_subscription = scope != InitialStreamScope::None;
            const bool send_current = active_subscription && owner.config_.publisher.send_current_values_on_connect &&
                                      owner.websocket_should_send_current_values(request.resource);
            ConnectionFlags flags{owner.config_.signalk.websocket.allow_rx,
                                  owner.config_.signalk.websocket.allow_tx && active_subscription};
            if (!connections.add(connection, flags)) {
                connection.close();
                return;
            }
            if (!owner.send_signal_k_hello_websocket(connection)) {
                connections.remove(connection);
                connection.close();
                return;
            }
            if (send_current) owner.send_current_values_websocket(connection, scope == InitialStreamScope::All);
        }

        void on_websocket_text(async_event_loop::ITcpConnection& connection, const char* text, size_t len) override {
            if (!connections.allow_rx(connection)) return;
            char copy[256];
            const size_t copy_len = len < sizeof(copy) ? len : sizeof(copy) - 1;
            if (text && copy_len > 0) memcpy(copy, text, copy_len);
            copy[copy_len] = '\0';
            if (owner.is_subscribe_all_text(copy)) {
                connections.set_allow_tx(connection, owner.config_.signalk.websocket.allow_tx);
                if (owner.config_.signalk.websocket.allow_tx) owner.send_current_values_websocket(connection, true);
            }
        }

        void on_websocket_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_websocket_error(async_event_loop::ITcpConnection& connection, async_event_loop::WebSocketError) override { connections.remove(connection); }

        void write_signal_k_delta(const char* json, size_t len) {
            const size_t text_len = owner.signalk_text_len(json, len);
            if (!json || text_len == 0) return;
            connections.for_each_tx([&](async_event_loop::ITcpConnection& connection) {
                owner.signalk_websocket_protocol_.write_text(connection, json, text_len);
            });
        }
    };

    struct SignalKDeltaFanout {
        ConnectionRegistry<MaxSignalKConnections>& tcp;
        SignalKWebSocketHandler& websocket;

        void write_signal_k_delta(const char* json, size_t len) {
            tcp.write_signal_k_delta(json, len);
            websocket.write_signal_k_delta(json, len);
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
            if ((owner.connector_protocol(index) == ConnectorProtocol::Ubx && connections.size() != 0) ||
                !connections.add(connection, owner.connector_connection_flags(index))) {
                connection.close();
                return;
            }
            owner.on_connector_raw_connected(index);
        }
        void on_data(async_event_loop::ITcpConnection& connection) override {
            if (!connections.allow_rx(connection)) return;
            owner.handle_connector_stream_bytes(index, connection);
        }
        void on_close(async_event_loop::ITcpConnection& connection) override {
            connections.remove(connection);
            owner.on_connector_raw_disconnected(index);
        }
        void on_error(async_event_loop::ITcpConnection& connection, int) override {
            connections.remove(connection);
            owner.on_connector_raw_disconnected(index);
        }
    };

    struct ConnectorTcpClientHandler : async_event_loop::ITcpClientHandler {
        ConnectorTcpClientHandler(MiniSignalKServer& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        MiniSignalKServer& owner;
        size_t index;

        void on_connect(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            owner.on_connector_tcp_connected(index, connection);
        }
        void on_data(async_event_loop::ITcpConnection& connection) override {
            if (!owner.connector_allow_rx(index)) return;
            const ConnectorProtocol protocol = owner.connector_protocol(index);
            if (protocol == ConnectorProtocol::SeaTalk1 || protocol == ConnectorProtocol::Ubx || protocol == ConnectorProtocol::Gpsd) {
                owner.handle_connector_stream_bytes(index, connection);
                return;
            }
            char text[160];
            while (connection.read_line(text, sizeof(text))) owner.handle_connector_nmea_line(index, text);
        }
        void on_close(async_event_loop::ITcpConnection&) override {
            owner.on_connector_tcp_disconnected(index);
            owner.schedule_connector_reconnect(index);
        }
        void on_error(int) override {
            owner.on_connector_tcp_disconnected(index);
            owner.schedule_connector_reconnect(index);
        }
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
        uint32_t reconnect_delay_ms = 0;
        uint32_t reconnect_count = 0;
        async_event_loop::NativeTcpServer tcp_server;
        async_event_loop::NativeTcpClient tcp_client;
        async_event_loop::NativeSerialStream serial_stream;
        async_event_loop::LineProtocolReader<192> serial_reader;
        UdpListenerStream udp_listener;
        SeaTalkInput<Real> seatalk_input;
        std::unique_ptr<UbxInput<Real>> ubx_input;
#if !defined(ARDUINO)
        std::unique_ptr<GpsdInput<Real>> gpsd_input;
#endif
        ConnectorTcpServerHandler tcp_server_handler;
        ConnectorTcpRawServerHandler tcp_raw_server_handler;
        ConnectorTcpClientHandler tcp_client_handler;
        async_event_loop::TcpLineServerHandler<192, MaxConnectionsPerConnector> tcp_line_handler;
        async_event_loop::EventHandle serial_event{};
        async_event_loop::EventHandle udp_event{};
        async_event_loop::EventHandle reconnect_event{};
    };

    ConnectorRuntimeSlot& connector_slot(size_t index) { return *connector_slots_[index]; }
    const ConnectorRuntimeSlot& connector_slot(size_t index) const { return *connector_slots_[index]; }
    const ConnectionFlags& connector_connection_flags(size_t index) const { return connector_slot(index).connection_flags; }
    bool connector_allow_rx(size_t index) const { return connector_slot(index).connection_flags.allow_rx; }
    ConnectorProtocol connector_protocol(size_t index) const { return connector_slot(index).config.protocol.kind; }
    ship_data_model::SensorSource connector_sensor_source(size_t index) const { return sensor_source_for_connector(connector_slot(index).config); }

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
            slot->reconnect_delay_ms = connector.reconnect.initial_delay_ms;
            if (connector.protocol.kind == ConnectorProtocol::Ubx) slot->ubx_input.reset(new UbxInput<Real>(store_));
#if !defined(ARDUINO)
            if (connector.protocol.kind == ConnectorProtocol::Gpsd) {
                slot->gpsd_input.reset(new GpsdInput<Real>(store_, connector.protocol.gpsd.device,
                                                          connector.protocol.gpsd.include_sky,
                                                          connector.protocol.gpsd.include_gst));
            }
#endif
            connector_slots_.push_back(std::move(slot));
            ConnectorRuntimeSlot& runtime = *connector_slots_.back();
            if (!connector.enabled) continue;
            if (connector.protocol.kind == ConnectorProtocol::Nmea0183) runtime.started = start_nmea0183_connector(runtime);
            else if (connector.protocol.kind == ConnectorProtocol::SeaTalk1) runtime.started = start_seatalk_connector(runtime);
            else if (connector.protocol.kind == ConnectorProtocol::Ubx) runtime.started = start_ubx_connector(runtime);
#if !defined(ARDUINO)
            else if (connector.protocol.kind == ConnectorProtocol::Gpsd) runtime.started = start_gpsd_connector(runtime);
#endif
            if (!runtime.started) {
                ++connector_start_failure_count_;
                return set_startup_error(StartupError::ConnectorStartFailed, i);
            }
        }
        return true;
    }

    bool start_nmea0183_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: return connect_tcp_client(slot);
        case ConnectorTransport::TcpServer: return listen(slot.tcp_server, slot.tcp_line_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_nmea0183_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }

    bool start_seatalk_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: return connect_tcp_client(slot);
        case ConnectorTransport::TcpServer: return listen(slot.tcp_server, slot.tcp_raw_server_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_raw_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }

    bool start_ubx_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: return connect_tcp_client(slot);
        case ConnectorTransport::TcpServer: return listen(slot.tcp_server, slot.tcp_raw_server_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_raw_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }
#if !defined(ARDUINO)
    bool start_gpsd_connector(ConnectorRuntimeSlot& slot) {
        return slot.config.transport.kind == ConnectorTransport::TcpClient && connect_tcp_client(slot);
    }
#endif

    bool connect_tcp_client(ConnectorRuntimeSlot& slot) {
        async_event_loop::TcpConnectOptions options;
        options.host = slot.config.transport.tcp_client.host;
        options.port = slot.config.transport.tcp_client.port;
        const bool accepted = slot.tcp_client.connect(options, slot.tcp_client_handler);
        if (accepted) return true;
        return schedule_connector_reconnect(slot.index);
    }

    void reset_connector_binary_stream(size_t index) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (slot.ubx_input) slot.ubx_input->reset_stream();
#if !defined(ARDUINO)
        if (slot.gpsd_input) slot.gpsd_input->on_disconnected();
#endif
        slot.seatalk_input.reset_stream();
    }

    void on_connector_tcp_connected(size_t index, async_event_loop::ITcpConnection& connection) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        slot.reconnect_delay_ms = slot.config.reconnect.initial_delay_ms;
        reset_connector_binary_stream(index);
#if !defined(ARDUINO)
        if (slot.gpsd_input) {
            slot.gpsd_input->on_connected();
            char watch[384];
            const bool encoded = gpsd::make_watch_command(watch, sizeof(watch), slot.config.protocol.gpsd.device);
            const size_t length = encoded ? strlen(watch) : 0;
            if (!encoded || connection.write(reinterpret_cast<const uint8_t*>(watch), length) != static_cast<int>(length)) {
                connection.close();
                on_connector_tcp_disconnected(index);
                schedule_connector_reconnect(index);
            }
        }
#endif
    }

    void on_connector_tcp_disconnected(size_t index) {
#if !defined(ARDUINO)
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (slot.gpsd_input) slot.gpsd_input->on_disconnected();
#else
        (void)index;
#endif
    }

    void on_connector_raw_connected(size_t index) { reset_connector_binary_stream(index); }
    void on_connector_raw_disconnected(size_t index) { reset_connector_binary_stream(index); }

    bool schedule_connector_reconnect(size_t index) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (slot.config.transport.kind != ConnectorTransport::TcpClient || !slot.config.reconnect.enabled) return false;
        if (loop_.valid(slot.reconnect_event)) return true;
        const uint32_t delay_ms = slot.reconnect_delay_ms == 0 ? slot.config.reconnect.initial_delay_ms : slot.reconnect_delay_ms;
        slot.reconnect_event = loop_.on_delay(delay_ms, [this, index]() {
            ConnectorRuntimeSlot& reconnecting = connector_slot(index);
            reconnecting.reconnect_event = async_event_loop::EventHandle{};
            if (reconnecting.reconnect_count < UINT32_MAX) ++reconnecting.reconnect_count;
            connect_tcp_client(reconnecting);
        });
        if (!slot.reconnect_event) return false;
        const uint64_t doubled = static_cast<uint64_t>(delay_ms) * 2ULL;
        slot.reconnect_delay_ms = static_cast<uint32_t>(doubled > slot.config.reconnect.maximum_delay_ms ? slot.config.reconnect.maximum_delay_ms : doubled);
        return true;
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

    bool start_raw_serial_connector(ConnectorRuntimeSlot& slot) {
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
        const SourceId source_id = static_cast<SourceId>(FirstConnectorSourceId + index);
        const ship_data_model::SensorSource source = connector_sensor_source(index);
        uint8_t buf[128];
        bool changed = false;
        while (stream.readable()) {
            const int n = stream.read(buf, sizeof(buf));
            if (n <= 0) break;
            if (slot.config.protocol.kind == ConnectorProtocol::SeaTalk1) {
                if (slot.seatalk_input.feed_octets(buf, static_cast<size_t>(n), source_id, loop_.clock().micros(), source)) changed = true;
            } else if (slot.config.protocol.kind == ConnectorProtocol::Ubx) {
                if (slot.ubx_input && slot.ubx_input->feed_octets(buf, static_cast<size_t>(n), source_id, loop_.clock().micros(), source)) changed = true;
#if !defined(ARDUINO)
            } else if (slot.config.protocol.kind == ConnectorProtocol::Gpsd) {
                if (slot.gpsd_input && slot.gpsd_input->feed_octets(buf, static_cast<size_t>(n), source_id, loop_.clock().micros(), source)) changed = true;
#endif
            }
        }
        if (changed) publish();
    }

    void handle_connector_udp_datagrams(size_t index) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (!slot.connection_flags.allow_rx) return;
        const ship_data_model::SensorSource source = connector_sensor_source(index);
        uint8_t buf[ubx::DefaultMaxPayload + 9];
        while (slot.udp_listener.readable()) {
            const int n = slot.udp_listener.read(buf, sizeof(buf) - 1);
            if (n <= 0) break;
            if (slot.config.protocol.kind == ConnectorProtocol::SeaTalk1) {
                if (slot.seatalk_input.feed_datagram(buf, static_cast<size_t>(n), static_cast<SourceId>(FirstConnectorSourceId + index), loop_.clock().micros(), source)) publish();
                continue;
            }
            if (slot.config.protocol.kind == ConnectorProtocol::Ubx) {
                if (slot.ubx_input && slot.ubx_input->feed_datagram(buf, static_cast<size_t>(n), static_cast<SourceId>(FirstConnectorSourceId + index), loop_.clock().micros(), source)) publish();
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
        const SourceId source_id = static_cast<SourceId>(FirstConnectorSourceId + index);
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
        store_.record_current(ModelField::CommServerClockS, 0);
    }

    void publish_server_clock() {
        const uint64_t now_us = loop_.clock().micros();
        update_server_clock_model(now_us);

        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return;

        char json[384];
        const double clock_s = static_cast<double>(store_.model().comm.server.clock_s.value);
        const int len = signalk_write_scalar_delta(json,
                                                   sizeof(json),
                                                   config_.identity.self ? config_.identity.self : "vessels.urn:mrn:signalk:uuid:00000000-0000-4000-8000-000000000001",
                                                   timestamp,
                                                   server_source_label(),
                                                   "communication.server.clock",
                                                   SignalKMappedValueKind::Number,
                                                   clock_s,
                                                   false,
                                                   nullptr);
        if (len <= 0 || static_cast<size_t>(len) >= sizeof(json)) return;
        SignalKDeltaFanout fanout{signalk_connections_.connections, signalk_websocket_handler_};
        fanout.write_signal_k_delta(json, static_cast<size_t>(len));
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
        SignalKDeltaFanout fanout{signalk_connections_.connections, signalk_websocket_handler_};
        publisher_.publish_pending(fanout);
    }

    SignalKMiniConfig config_;
    async_event_loop::EventLoop<> loop_;
    async_event_loop::NativeTcpServer signalk_tcp_server_;
    async_event_loop::NativeTcpServer signalk_websocket_server_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_{store_};
    SeaTalkInput<Real> seatalk_{store_};
    UbxInput<Real> ubx_{store_};
    SignalKPublisher<Real> publisher_;
    SignalKConnectionHandler signalk_connections_;
    SignalKWebSocketHandler signalk_websocket_handler_;
    async_event_loop::WebSocketServerHandler<MaxSignalKConnections, async_event_loop::websocket::DefaultHandshakeBufferSize, async_event_loop::websocket::DefaultMaxPayloadSize> signalk_websocket_protocol_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKConnections> signalk_line_handler_;
    std::vector<std::unique_ptr<ConnectorRuntimeSlot>> connector_slots_;
    async_event_loop::EventHandle timer_{};
    async_event_loop::EventHandle clock_timer_{};
    StartupError last_startup_error_ = StartupError::None;
    size_t last_failed_connector_index_ = 0;
    uint32_t connector_start_failure_count_ = 0;
};

} // namespace signalk_mini
