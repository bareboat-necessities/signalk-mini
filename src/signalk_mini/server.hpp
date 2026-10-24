#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(ARDUINO) && !defined(ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP)
#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#endif

#include <async_event_loop.hpp>
#include <nmea0183_connector.hpp>
#include <seatalk.hpp>
#include <ubx.hpp>
#if !defined(ARDUINO)
#include <gpsd.hpp>
#endif
#include "config.hpp"
#include "connector_runtime.hpp"
#include "model_store.hpp"
#include "nmea0183_input.hpp"
#include "publisher.hpp"
#include "seatalk_input.hpp"
#include "signalk_full_model_writer.hpp"
#include "signalk_identity.hpp"
#include "signalk_stream_server.hpp"
#include "ubx_input.hpp"

namespace signalk_mini {

// Signal K mini server: one event loop, the typed model store, the client-facing
// stream/HTTP side (SignalKStreamServer) and configured connectors (ConnectorRuntime).
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
          publisher_(store_, config_),
          stream_(config_, loop_, store_, publisher_),
          publish_on_change_(stream_),
          connectors_(config_, loop_, store_, nmea_, publish_on_change_) {}

    bool begin() {
        reset_startup_diagnostics();
        if (!validate_config()) return false;
        if (!loop_.valid()) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        if (!stream_.listen()) return set_startup_error(StartupError::ConnectorStartFailed, config_.connector_count);
        size_t failed_connector = config_.connector_count;
        if (!connectors_.start(failed_connector)) {
            ++connector_start_failure_count_;
            return set_startup_error(StartupError::ConnectorStartFailed, failed_connector);
        }
        timer_ = loop_.on_repeat_us(config_.publisher.interval_us, [this]() { stream_.publish(); });
        if (!timer_) return set_startup_error(StartupError::InvalidPublisherInterval, config_.connector_count);
        clock_timer_ = loop_.on_repeat_us(1000000ULL, [this]() { stream_.publish_server_clock(); });
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
    uint64_t truncated_value_count() const { return publisher_.truncated_value_count(); }
    uint32_t stream_catch_up_count() const { return stream_.catch_up_count(); }
    uint32_t stalled_client_close_count() const { return stream_.stalled_client_close_count(); }
    size_t catching_up_client_count() { return stream_.catching_up_client_count(); }
    int write_full_model(char* dst, size_t dst_size) const {
        SignalKFullModelWriter<Real> writer;
        return writer.write(dst, dst_size, store_, config_, loop_.clock().micros());
    }
    StartupError last_startup_error() const { return last_startup_error_; }
    size_t last_failed_connector_index() const { return last_failed_connector_index_; }
    uint32_t connector_start_failure_count() const { return connector_start_failure_count_; }
    uint32_t connector_reconnect_count() const { return connectors_.reconnect_count(); }
    uint32_t ubx_frame_count() const {
        return ubx_total([](const ubx::ReceiverDiagnostics& d) { return d.frame_count; });
    }
    uint32_t ubx_checksum_error_count() const {
        return ubx_total([](const ubx::ReceiverDiagnostics& d) { return d.checksum_error_count; });
    }
    uint32_t ubx_oversized_frame_count() const {
        return ubx_total([](const ubx::ReceiverDiagnostics& d) { return d.oversized_frame_count; });
    }
    uint32_t ubx_unsupported_frame_count() const {
        return ubx_total([](const ubx::ReceiverDiagnostics& d) { return d.unsupported_frame_count; });
    }
#if !defined(ARDUINO)
    uint32_t gpsd_record_count() const {
        return gpsd_total([](const gpsd::Diagnostics& d) { return d.record_count; });
    }
    uint32_t gpsd_malformed_record_count() const {
        return gpsd_total([](const gpsd::Diagnostics& d) { return d.malformed_record_count; });
    }
    uint32_t gpsd_oversized_record_count() const {
        return gpsd_total([](const gpsd::Diagnostics& d) { return d.oversized_record_count; });
    }
    uint32_t gpsd_device_mismatch_count() const {
        return gpsd_total([](const gpsd::Diagnostics& d) { return d.device_mismatch_count; });
    }
#endif

    bool transmit_seatalk_frame(const seatalk::SeaTalkFrame& frame) { return connectors_.transmit_seatalk_frame(frame); }

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
    struct PublishOnModelChange final : IConnectorModelListener {
        explicit PublishOnModelChange(SignalKStreamServer<Real, MaxSignalKConnections>& stream_ref) : stream(stream_ref) {}
        void on_connector_model_changed() override { stream.publish(); }
        SignalKStreamServer<Real, MaxSignalKConnections>& stream;
    };

    template<typename Field>
    uint32_t ubx_total(Field field) const {
        uint32_t total = field(ubx_.receiver().diagnostics());
        connectors_.for_each_ubx_input([&](const UbxInput<Real>& input) { total += field(input.receiver().diagnostics()); });
        return total;
    }

#if !defined(ARDUINO)
    template<typename Field>
    uint32_t gpsd_total(Field field) const {
        uint32_t total = 0;
        connectors_.for_each_gpsd_input([&](const GpsdInput<Real>& input) { total += field(input.client().diagnostics()); });
        return total;
    }
#endif

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

    SignalKMiniConfig config_;
    async_event_loop::EventLoop<> loop_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_{store_};
    SeaTalkInput<Real> seatalk_{store_};
    UbxInput<Real> ubx_{store_};
    SignalKPublisher<Real> publisher_;
    SignalKStreamServer<Real, MaxSignalKConnections> stream_;
    PublishOnModelChange publish_on_change_;
    ConnectorRuntime<Real, MaxConnectionsPerConnector> connectors_;
    async_event_loop::EventHandle timer_{};
    async_event_loop::EventHandle clock_timer_{};
    StartupError last_startup_error_ = StartupError::None;
    size_t last_failed_connector_index_ = 0;
    uint32_t connector_start_failure_count_ = 0;
};

} // namespace signalk_mini
