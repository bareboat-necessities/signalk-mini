#pragma once

#if defined(ARDUINO)
#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#endif

#include <async_event_loop.hpp>
#include "config.hpp"
#include "nmea0183_input.hpp"
#include "publisher.hpp"

namespace signalk_mini {

template<typename Real, size_t MaxSignalKClients = 8, size_t MaxNmeaSources = 4>
class MiniSignalKServer {
public:
    explicit MiniSignalKServer(const SignalKMiniConfig& config = SignalKMiniConfig{})
        : config_(config), signalk_server_(loop_.scheduler()), nmea_server_(loop_.scheduler()),
          publisher_(store_, config_), signalk_(*this), nmea_(*this),
          signalk_handler_(signalk_, signalk_options()), nmea_handler_(nmea_) {}

    bool begin() {
        if (!loop_.valid()) return false;
        if (!listen(signalk_server_, signalk_handler_, config_.signalk_port)) return false;
        if (!listen(nmea_server_, nmea_handler_, config_.nmea0183_tcp_port)) { signalk_server_.close(); return false; }
        timer_ = loop_.on_repeat_us(10000ULL, [this]() { publish(); });
        return static_cast<bool>(timer_);
    }

    void tick() { loop_.tick(); }
    void run_forever() { loop_.run_forever(); }
    ModelStore<Real>& store() { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea_input_; }

private:
    static async_event_loop::TcpLineServerOptions signalk_options() {
        async_event_loop::TcpLineServerOptions o;
#if defined(ARDUINO)
        o.backpressure = async_event_loop::TcpBackpressureOptions::embedded_default();
#else
        o.backpressure = async_event_loop::TcpBackpressureOptions::server_default();
#endif
        return o;
    }

    bool listen(async_event_loop::NativeTcpServer& server, async_event_loop::ITcpServerHandler& handler, uint16_t port) {
        async_event_loop::TcpListenOptions o;
        o.host = config_.host;
        o.port = port;
        o.reuse_address = true;
        return server.listen(o, handler);
    }

    void publish() { publisher_.publish_pending(signalk_.clients); }

    struct SignalKHandler : async_event_loop::ITcpLineServerHandler {
        explicit SignalKHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        async_event_loop::TcpConnectionRegistry<MaxSignalKClients> clients;
        void on_accept(async_event_loop::ITcpConnection& c, const async_event_loop::TcpPeerInfo&) override { if (!clients.add(c)) c.close(); }
        void on_line(async_event_loop::ITcpConnection&, async_event_loop::LineView) override {}
        void on_backpressure(async_event_loop::ITcpConnection& c, const async_event_loop::TcpBackpressureInfo&) override { clients.remove(c); }
        void on_close(async_event_loop::ITcpConnection& c) override { clients.remove(c); }
        void on_error(async_event_loop::ITcpConnection& c, int) override { clients.remove(c); }
        void on_too_many_connections(async_event_loop::ITcpConnection& c) override { c.close(); }
    };

    struct NmeaHandler : async_event_loop::ITcpLineServerHandler {
        explicit NmeaHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        async_event_loop::TcpConnectionRegistry<MaxNmeaSources> sources;
        void on_accept(async_event_loop::ITcpConnection& c, const async_event_loop::TcpPeerInfo&) override { if (!sources.add(c)) c.close(); }
        void on_line(async_event_loop::ITcpConnection&, async_event_loop::LineView line) override {
            char text[160];
            async_event_loop::line_to_cstr(line, text);
            if (owner.nmea_input_.feed_line(text, SourceId(1), owner.loop_.clock().micros())) owner.publish();
        }
        void on_close(async_event_loop::ITcpConnection& c) override { sources.remove(c); }
        void on_error(async_event_loop::ITcpConnection& c, int) override { sources.remove(c); }
        void on_too_many_connections(async_event_loop::ITcpConnection& c) override { c.close(); }
    };

    SignalKMiniConfig config_;
    async_event_loop::EventLoop<> loop_;
    async_event_loop::NativeTcpServer signalk_server_;
    async_event_loop::NativeTcpServer nmea_server_;
    ModelStore<Real> store_{};
    Nmea0183Input<Real> nmea_input_{store_};
    SignalKPublisher<Real> publisher_;
    SignalKHandler signalk_;
    NmeaHandler nmea_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKClients> signalk_handler_;
    async_event_loop::TcpLineServerHandler<192, MaxNmeaSources> nmea_handler_;
    async_event_loop::EventHandle timer_{};
};

} // namespace signalk_mini
