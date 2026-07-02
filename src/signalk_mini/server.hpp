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

template<typename Real, size_t MaxSignalKConnections = 8, size_t MaxConnectionsPerConnector = 4>
class MiniSignalKServer {
public:
    explicit MiniSignalKServer(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : config_(config),
          signalk_tcp_server_(loop_.scheduler()),
          publisher_(store_, config_),
          signalk_connections_(*this),
          signalk_line_handler_(signalk_connections_, signalk_options()),
          connector0_(*this, 0),
          connector1_(*this, 1),
          connector2_(*this, 2),
          connector3_(*this, 3) {}

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

    struct ConnectorTcpServerHandler : async_event_loop::ITcpLineServerHandler {
        ConnectorTcpServerHandler(MiniSignalKServer& owner_ref, size_t connector_index)
            : owner(owner_ref), index(connector_index) {}

        MiniSignalKServer& owner;
        size_t index;
        ConnectionRegistry<MaxConnectionsPerConnector> connections;

        void on_accept(async_event_loop::ITcpConnection& connection, const async_event_loop::TcpPeerInfo&) override {
            if (!connections.add(connection, owner.connector_slot(index).connection_flags)) connection.close();
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

    struct ConnectorTcpClientHandler : async_event_loop::ITcpClientHandler {
        ConnectorTcpClientHandler(MiniSignalKServer& owner_ref, size_t connector_index)
            : owner(owner_ref), index(connector_index) {}

        MiniSignalKServer& owner;
        size_t index;

        void on_data(async_event_loop::ITcpConnection& connection) override {
            const ConnectorRuntimeSlot& slot = owner.connector_slot(index);
            if (!slot.connection_flags.allow_rx) return;
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
              serial_reader(serial_stream, async_event_loop::LineProtocolOptions{}, [this](async_event_loop::LineView line) { owner.handle_connector_serial_line(index, line); }),
              tcp_server_handler(owner_ref, connector_index),
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
        ConnectorTcpServerHandler tcp_server_handler;
        ConnectorTcpClientHandler tcp_client_handler;
        async_event_loop::TcpLineServerHandler<192, MaxConnectionsPerConnector> tcp_line_handler;
        async_event_loop::EventHandle serial_event{};
    };

    ConnectorRuntimeSlot& connector_slot(size_t index) {
        switch (index) {
        case 0: return connector0_;
        case 1: return connector1_;
        case 2: return connector2_;
        default: return connector3_;
        }
    }

    const ConnectorRuntimeSlot& connector_slot(size_t index) const {
        switch (index) {
        case 0: return connector0_;
        case 1: return connector1_;
        case 2: return connector2_;
        default: return connector3_;
        }
    }

    void start_configured_connectors() {
        const size_t count = config_.connector_count <= max_connector_configs ? config_.connector_count : max_connector_configs;
        for (size_t i = 0; i < count; ++i) {
            const ConnectorConfig& connector = config_.connectors[i];
            ConnectorRuntimeSlot& slot = connector_slot(i);
            slot.config = connector;
            slot.connection_flags = ConnectionFlags{connector.allow_rx, connector.allow_tx};
            slot.nmea0183_validate_checksum = connector.protocol == ConnectorProtocol::Nmea0183
                ? effective_nmea0183_validate_checksum(connector.nmea0183, connector.transport)
                : false;

            if (!connector.enabled) continue;
            if (connector.protocol == ConnectorProtocol::Nmea0183) {
                slot.started = start_nmea0183_connector(slot);
            }
        }
    }

    bool start_nmea0183_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport) {
        case ConnectorTransport::TcpClient: {
            async_event_loop::TcpConnectOptions options;
            options.host = slot.config.host;
            options.port = slot.config.port;
            return slot.tcp_client.connect(options, slot.tcp_client_handler);
        }
        case ConnectorTransport::TcpServer:
            return listen(slot.tcp_server, slot.tcp_line_handler, slot.config.host, slot.config.port);
        case ConnectorTransport::Serial:
            return start_nmea0183_serial_connector(slot);
        default:
            return false;
        }
    }

    bool start_nmea0183_serial_connector(ConnectorRuntimeSlot& slot) {
#if defined(ARDUINO)
        Serial.begin(slot.config.baud);
#else
        if (!slot.serial_stream.open(slot.config.device, slot.config.baud)) return false;
#endif
        if (!slot.connection_flags.allow_rx) return true;
        slot.serial_event = loop_.on_bytes_ready(slot.serial_stream, [&slot]() {
            slot.serial_reader.poll(0);
        });
        return static_cast<bool>(slot.serial_event);
    }

    void handle_connector_serial_line(size_t index, async_event_loop::LineView line) {
        ConnectorRuntimeSlot& slot = connector_slot(index);
        if (!slot.connection_flags.allow_rx) return;
        char text[160];
        async_event_loop::line_to_cstr(line, text);
        handle_connector_nmea_line(index, text);
    }

    void handle_connector_nmea_line(size_t index, const char* text) {
        const ConnectorRuntimeSlot& slot = connector_slot(index);
        const SourceId source_id = static_cast<SourceId>(10 + index);
        if (nmea_.feed_line(text, source_id, loop_.clock().micros(), slot.nmea0183_validate_checksum)) publish();
    }

    void publish() { publisher_.publish_pending(signalk_connections_.connections); }

    SignalKMiniConfig config_;
    async_event_loop::EventLoop<> loop_;
    async_event_loop::NativeTcpServer signalk_tcp_server_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_{store_};
    SignalKPublisher<Real> publisher_;
    SignalKConnectionHandler signalk_connections_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKConnections> signalk_line_handler_;
    ConnectorRuntimeSlot connector0_;
    ConnectorRuntimeSlot connector1_;
    ConnectorRuntimeSlot connector2_;
    ConnectorRuntimeSlot connector3_;
    async_event_loop::EventHandle timer_{};
};

} // namespace signalk_mini
