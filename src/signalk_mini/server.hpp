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

template<typename Real, size_t MaxSignalKClients = 8, size_t MaxNmeaSources = 4>
class MiniSignalKServer {
public:
    explicit MiniSignalKServer(const SignalKMiniConfig& cfg = SignalKMiniConfig{})
        : cfg_(cfg), sk_server_(loop_.scheduler()), nmea_server_(loop_.scheduler()),
          nmea_client_(loop_.scheduler()), publisher_(store_, cfg_), sk_(*this),
          nmea_server_cb_(*this), nmea_client_cb_(*this),
          sk_lines_(sk_, sk_options()), nmea_lines_(nmea_server_cb_) {}

    bool begin() {
        if (!loop_.valid()) return false;
        if (!listen(sk_server_, sk_lines_, cfg_.signalk.host, cfg_.signalk.port)) return false;
        start_configured_connectors();
        timer_ = loop_.on_repeat_us(cfg_.publisher.interval_us, [this]() { publish(); });
        return static_cast<bool>(timer_);
    }

    void tick() { loop_.tick(); }
    void run_forever() { loop_.run_forever(); }
    ModelStore<Real>& store() { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea_; }

private:
    static async_event_loop::TcpLineServerOptions sk_options() {
        async_event_loop::TcpLineServerOptions o;
#if defined(ARDUINO)
        o.backpressure = async_event_loop::TcpBackpressureOptions::embedded_default();
#else
        o.backpressure = async_event_loop::TcpBackpressureOptions::server_default();
#endif
        return o;
    }

    bool listen(async_event_loop::NativeTcpServer& s, async_event_loop::ITcpServerHandler& h, const char* host, uint16_t port) {
        async_event_loop::TcpListenOptions o;
        o.host = host;
        o.port = port;
        o.reuse_address = true;
        return s.listen(o, h);
    }

    void start_configured_connectors() {
        const size_t count = cfg_.connector_count <= max_connector_configs ? cfg_.connector_count : max_connector_configs;
        for (size_t i = 0; i < count; ++i) {
            const ConnectorConfig& c = cfg_.connectors[i];
            if (!c.enabled) continue;
            if (c.protocol == ConnectorProtocol::Nmea0183 && c.transport == ConnectorTransport::TcpClient && !nmea_client_started_) {
                nmea_client_flags_ = ConnectionFlags{c.allow_rx, c.allow_tx};
                async_event_loop::TcpConnectOptions o;
                o.host = c.host;
                o.port = c.port;
                nmea_client_.connect(o, nmea_client_cb_);
                nmea_client_started_ = true;
            } else if (c.protocol == ConnectorProtocol::Nmea0183 && c.transport == ConnectorTransport::TcpServer && !nmea_server_started_) {
                nmea_server_flags_ = ConnectionFlags{c.allow_rx, c.allow_tx};
                nmea_server_started_ = listen(nmea_server_, nmea_lines_, c.host, c.port);
            }
        }
    }

    void handle_nmea_line(const char* text, SourceId source_id) {
        if (nmea_.feed_line(text, source_id, loop_.clock().micros())) publish();
    }

    void publish() { publisher_.publish_pending(sk_.clients); }

    struct SkHandler : async_event_loop::ITcpLineServerHandler {
        explicit SkHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxSignalKClients> clients;
        void on_accept(async_event_loop::ITcpConnection& c, const async_event_loop::TcpPeerInfo&) override {
            ConnectionFlags flags{owner.cfg_.signalk.allow_rx, owner.cfg_.signalk.allow_tx};
            if (!clients.add(c, flags)) c.close();
        }
        void on_line(async_event_loop::ITcpConnection& c, async_event_loop::LineView) override {
            if (!clients.allow_rx(c)) return;
        }
        void on_backpressure(async_event_loop::ITcpConnection& c, const async_event_loop::TcpBackpressureInfo&) override { clients.remove(c); c.close(); }
        void on_close(async_event_loop::ITcpConnection& c) override { clients.remove(c); }
        void on_error(async_event_loop::ITcpConnection& c, int) override { clients.remove(c); }
        void on_too_many_connections(async_event_loop::ITcpConnection& c) override { c.close(); }
    };

    struct NmeaServerHandler : async_event_loop::ITcpLineServerHandler {
        explicit NmeaServerHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        ConnectionRegistry<MaxNmeaSources> sources;
        void on_accept(async_event_loop::ITcpConnection& c, const async_event_loop::TcpPeerInfo&) override {
            if (!sources.add(c, owner.nmea_server_flags_)) c.close();
        }
        void on_line(async_event_loop::ITcpConnection& c, async_event_loop::LineView line) override {
            if (!sources.allow_rx(c)) return;
            char text[160];
            async_event_loop::line_to_cstr(line, text);
            owner.handle_nmea_line(text, SourceId(1));
        }
        void on_close(async_event_loop::ITcpConnection& c) override { sources.remove(c); }
        void on_error(async_event_loop::ITcpConnection& c, int) override { sources.remove(c); }
        void on_too_many_connections(async_event_loop::ITcpConnection& c) override { c.close(); }
    };

    struct NmeaClientHandler : async_event_loop::ITcpClientHandler {
        explicit NmeaClientHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        void on_data(async_event_loop::ITcpConnection& c) override {
            if (!owner.nmea_client_flags_.allow_rx) return;
            char text[160];
            while (c.read_line(text, sizeof(text))) owner.handle_nmea_line(text, SourceId(2));
        }
        void on_close(async_event_loop::ITcpConnection&) override {}
        void on_error(int) override {}
    };

    SignalKMiniConfig cfg_;
    async_event_loop::EventLoop<> loop_;
    async_event_loop::NativeTcpServer sk_server_;
    async_event_loop::NativeTcpServer nmea_server_;
    async_event_loop::NativeTcpClient nmea_client_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_{store_};
    SignalKPublisher<Real> publisher_;
    SkHandler sk_;
    NmeaServerHandler nmea_server_cb_;
    NmeaClientHandler nmea_client_cb_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKClients> sk_lines_;
    async_event_loop::TcpLineServerHandler<192, MaxNmeaSources> nmea_lines_;
    ConnectionFlags nmea_server_flags_{true, false};
    ConnectionFlags nmea_client_flags_{true, false};
    bool nmea_server_started_ = false;
    bool nmea_client_started_ = false;
    async_event_loop::EventHandle timer_{};
};

} // namespace signalk_mini
