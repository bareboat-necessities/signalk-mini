#pragma once

#include <stddef.h>

#if defined(ARDUINO)
#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#endif

#include <async_event_loop.hpp>
#include "config.hpp"
#include "connection_registry.hpp"
#include "nmea0183_input.hpp"
#include "publisher.hpp"

namespace signalk_mini {

template<typename Real, size_t MaxSignalKConnections = 8, size_t MaxNmeaSourceConnections = 4>
class MiniSignalKServer {
public:
    explicit MiniSignalKServer(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : config_(config), signalk_tcp_server_(loop_.scheduler()), nmea_tcp_server_(loop_.scheduler()),
          nmea_tcp_client_(loop_.scheduler()),
#if defined(ARDUINO)
          nmea_serial_stream_(Serial),
#else
          nmea_serial_stream_(),
#endif
          nmea_serial_reader_(nmea_serial_stream_, async_event_loop::LineProtocolOptions{}, [this](async_event_loop::LineView line) { handle_nmea_serial_line(line); }),
          publisher_(store_, config_), signalk_connections_(*this),
          nmea_server_connections_(*this), nmea_client_connection_(*this),
          signalk_line_handler_(signalk_connections_, signalk_options()), nmea_line_handler_(nmea_server_connections_) {}

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

    void start_configured_connectors() {
        const size_t count = config_.connector_count <= max_connector_configs ? config_.connector_count : max_connector_configs;
        for (size_t i = 0; i < count; ++i) {
            const ConnectorConfig& connector = config_.connectors[i];
            if (!connector.enabled) continue;
            if (connector.protocol == ConnectorProtocol::Nmea0183 && connector.transport == ConnectorTransport::TcpClient && !nmea_tcp_client_connector_started_) {
                nmea_tcp_client_connection_flags_ = ConnectionFlags{connector.allow_rx, connector.allow_tx};
                async_event_loop::TcpConnectOptions options;
                options.host = connector.host;
                options.port = connector.port;
                nmea_tcp_client_.connect(options, nmea_client_connection_);
                nmea_tcp_client_connector_started_ = true;
            } else if (connector.protocol == ConnectorProtocol::Nmea0183 && connector.transport == ConnectorTransport::TcpServer && !nmea_tcp_server_connector_started_) {
                nmea_tcp_server_connection_flags_ = ConnectionFlags{connector.allow_rx, connector.allow_tx};
                nmea_tcp_server_connector_started_ = listen(nmea_tcp_server_, nmea_line_handler_, connector.host, connector.port);
            } else if (connector.protocol == ConnectorProtocol::Nmea0183 && connector.transport == ConnectorTransport::Serial && !nmea_serial_connector_started_) {
                nmea_serial_connector_started_ = start_nmea_serial_connector(connector);
            }
        }
    }

    bool start_nmea_serial_connector(const ConnectorConfig& connector) {
        nmea_serial_connection_flags_ = ConnectionFlags{connector.allow_rx, connector.allow_tx};
#if defined(ARDUINO)
        Serial.begin(connector.baud);
#else
        if (!nmea_serial_stream_.open(connector.device, connector.baud)) {
            return false;
        }
#endif
        if (!connector.allow_rx) {
            return true;
        }
        nmea_serial_event_ = loop_.on_bytes_ready(nmea_serial_stream_, [this]() {
            nmea_serial_reader_.poll(loop_.clock().micros());
        });
        return static_cast<bool>(nmea_serial_event_);
    }

    void handle_nmea_line(const char* text, SourceId source_id) {
        if (nmea_.feed_line(text, source_id, loop_.clock().micros())) publish();
    }

    void handle_nmea_serial_line(async_event_loop::LineView line) {
        if (!nmea_serial_connection_flags_.allow_rx) return;
        char text[160];
        async_event_loop::line_to_cstr(line, text);
        handle_nmea_line(text, SourceId(3));
    }

    void publish() { publisher_.publish_pending(signalk_connections_.connections); }

    struct SignalKConnectionHandler : async_event_loop::ITcpLineServerHandler {
        explicit SignalKConnectionHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxSignalKConnections> connections;
        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            ConnectionFlags flags{owner.config_.signalk.allow_rx, owner.config_.signalk.allow_tx};
            if (!connections.add(connection, flags)) connection.close();
        }
        void on_line(async_event_loop::ITcpConnection& connection, async_event_loop::LineView) override {
            if (!connections.allow_rx(connection)) return;
        }
        void on_backpressure(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpBackpressureInfo&) override {
            connections.remove(connection);
            connection.close();
        }
        void on_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { connections.remove(connection); }
        void on_too_many_connections(async_event_loop::ITcpConnection& connection) override { connection.close(); }
    };

    struct NmeaTcpServerConnectionHandler : async_event_loop::ITcpLineServerHandler {
        explicit NmeaTcpServerConnectionHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxNmeaSourceConnections> connections;
        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            if (!connections.add(connection, owner.nmea_tcp_server_connection_flags_)) connection.close();
        }
        void on_line(async_event_loop::ITcpConnection& connection, async_event_loop::LineView line) override {
            if (!connections.allow_rx(connection)) return;
            char text[160];
            async_event_loop::line_to_cstr(line, text);
            owner.handle_nmea_line(text, SourceId(1));
        }
        void on_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_error(async_event_loop::ITcpConnection& connection, int) override { connections.remove(connection); }
        void on_too_many_connections(async_event_loop::ITcpConnection& connection) override { connection.close(); }
    };

    struct NmeaTcpClientConnectionHandler : async_event_loop::ITcpClientHandler {
        explicit NmeaTcpClientConnectionHandler(MiniSignalKServer& owner_ref) : owner(owner_ref) {}
        MiniSignalKServer& owner;
        void on_data(async_event_loop::ITcpConnection& connection) override {
            if (!owner.nmea_tcp_client_connection_flags_.allow_rx) return;
            char text[160];
            while (connection.read_line(text, sizeof(text))) owner.handle_nmea_line(text, SourceId(2));
        }
        void on_close(async_event_loop::ITcpConnection&) override {}
        void on_error(int) override {}
    };

    SignalKMiniConfig config_;
    async_event_loop::EventLoop<> loop_;
    async_event_loop::NativeTcpServer signalk_tcp_server_;
    async_event_loop::NativeTcpServer nmea_tcp_server_;
    async_event_loop::NativeTcpClient nmea_tcp_client_;
    async_event_loop::NativeSerialStream nmea_serial_stream_;
    async_event_loop::LineProtocolReader<192> nmea_serial_reader_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_{store_};
    SignalKPublisher<Real> publisher_;
    SignalKConnectionHandler signalk_connections_;
    NmeaTcpServerConnectionHandler nmea_server_connections_;
    NmeaTcpClientConnectionHandler nmea_client_connection_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKConnections> signalk_line_handler_;
    async_event_loop::TcpLineServerHandler<192, MaxNmeaSourceConnections> nmea_line_handler_;
    ConnectionFlags nmea_tcp_server_connection_flags_{true, false};
    ConnectionFlags nmea_tcp_client_connection_flags_{true, false};
    ConnectionFlags nmea_serial_connection_flags_{true, false};
    bool nmea_tcp_server_connector_started_ = false;
    bool nmea_tcp_client_connector_started_ = false;
    bool nmea_serial_connector_started_ = false;
    async_event_loop::EventHandle nmea_serial_event_{};
    async_event_loop::EventHandle timer_{};
};

} // namespace signalk_mini
