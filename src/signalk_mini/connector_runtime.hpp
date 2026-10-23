#pragma once

#include <memory>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "stream_io.hpp"
#include <async_event_loop.hpp>
#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <ubx.hpp>
#if !defined(ARDUINO)
#include <gpsd.hpp>
#endif
#include "config.hpp"
#include "connection_registry.hpp"
#include "model_store.hpp"
#include "nmea0183_input.hpp"
#include "seatalk_input.hpp"
#include "types.hpp"
#include "ubx_input.hpp"
#include "udp_listener_stream.hpp"
#if !defined(ARDUINO)
#include "gpsd_input.hpp"
#endif

namespace signalk_mini {

struct IConnectorModelListener {
    virtual ~IConnectorModelListener() = default;
    // Called after connector input changed the typed model.
    virtual void on_connector_model_changed() = 0;
};

// Runtime state for configured input/output connectors: transports (serial,
// TCP client/server, UDP), reconnect backoff, protocol decoding into the typed
// model, and SeaTalk transmit fan-out.
template<typename Real, size_t MaxConnectionsPerConnector>
class ConnectorRuntime {
public:
    ConnectorRuntime(const SignalKMiniConfig& config,
                     async_event_loop::EventLoop<>& loop,
                     ModelStore<Real>& store,
                     Nmea0183Input<Real>& nmea,
                     IConnectorModelListener& listener)
        : config_(config), loop_(loop), store_(store), nmea_(nmea), listener_(listener) {}

    ConnectorRuntime(const ConnectorRuntime&) = delete;
    ConnectorRuntime& operator=(const ConnectorRuntime&) = delete;

    // Creates a runtime slot per configured connector and starts the enabled ones.
    // On failure, failed_index names the connector that could not start.
    bool start(size_t& failed_index) {
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
                failed_index = i;
                return false;
            }
        }
        return true;
    }

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

    uint32_t reconnect_count() const {
        uint32_t total = 0;
        for (const auto& slot : connector_slots_) if (slot) total += slot->reconnect_count;
        return total;
    }

    template<typename Fn>
    void for_each_ubx_input(Fn&& fn) const {
        for (const auto& slot : connector_slots_) if (slot && slot->ubx_input) fn(*slot->ubx_input);
    }

#if !defined(ARDUINO)
    template<typename Fn>
    void for_each_gpsd_input(Fn&& fn) const {
        for (const auto& slot : connector_slots_) if (slot && slot->gpsd_input) fn(*slot->gpsd_input);
    }
#endif

private:
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

    struct ConnectorTcpServerHandler : async_event_loop::ITcpLineServerHandler {
        ConnectorTcpServerHandler(ConnectorRuntime& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        ConnectorRuntime& owner;
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
        ConnectorTcpRawServerHandler(ConnectorRuntime& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        ConnectorRuntime& owner;
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
        ConnectorTcpClientHandler(ConnectorRuntime& owner_ref, size_t connector_index) : owner(owner_ref), index(connector_index) {}
        ConnectorRuntime& owner;
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
        ConnectorRuntimeSlot(ConnectorRuntime& owner_ref, size_t connector_index)
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

        ConnectorRuntime& owner;
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


    bool start_nmea0183_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: return connect_tcp_client(slot);
        case ConnectorTransport::TcpServer: return listen_tcp(slot.tcp_server, slot.tcp_line_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_nmea0183_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }

    bool start_seatalk_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: return connect_tcp_client(slot);
        case ConnectorTransport::TcpServer: return listen_tcp(slot.tcp_server, slot.tcp_raw_server_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
        case ConnectorTransport::Serial: return start_raw_serial_connector(slot);
        case ConnectorTransport::Udp: return start_udp_listener(slot);
        default: return false;
        }
    }

    bool start_ubx_connector(ConnectorRuntimeSlot& slot) {
        switch (slot.config.transport.kind) {
        case ConnectorTransport::TcpClient: return connect_tcp_client(slot);
        case ConnectorTransport::TcpServer: return listen_tcp(slot.tcp_server, slot.tcp_raw_server_handler, slot.config.transport.tcp_server.host, slot.config.transport.tcp_server.port);
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
        if (changed) listener_.on_connector_model_changed();
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
                if (slot.seatalk_input.feed_datagram(buf, static_cast<size_t>(n), static_cast<SourceId>(FirstConnectorSourceId + index), loop_.clock().micros(), source)) listener_.on_connector_model_changed();
                continue;
            }
            if (slot.config.protocol.kind == ConnectorProtocol::Ubx) {
                if (slot.ubx_input && slot.ubx_input->feed_datagram(buf, static_cast<size_t>(n), static_cast<SourceId>(FirstConnectorSourceId + index), loop_.clock().micros(), source)) listener_.on_connector_model_changed();
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
        if (nmea_.feed_line(text, source_id, loop_.clock().micros(), slot.nmea0183_validate_checksum, connector_sensor_source(index))) listener_.on_connector_model_changed();
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

    const SignalKMiniConfig& config_;
    async_event_loop::EventLoop<>& loop_;
    ModelStore<Real>& store_;
    Nmea0183Input<Real>& nmea_;
    IConnectorModelListener& listener_;
    std::vector<std::unique_ptr<ConnectorRuntimeSlot>> connector_slots_;
};

} // namespace signalk_mini
