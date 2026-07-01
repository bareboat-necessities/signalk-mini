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
    explicit MiniSignalKServer(const SignalKMiniConfig& cfg = SignalKMiniConfig{})
        : cfg_(cfg), sk_server_(loop_.scheduler()), nmea_server_(loop_.scheduler()),
          nmea_client_(loop_.scheduler()), publisher_(store_, cfg_), sk_(*this),
          nmea_server_cb_(*this), nmea_client_cb_(*this),
          sk_lines_(sk_, sk_options()), nmea_lines_(nmea_server_cb_) {}

    bool begin() {
        if (!loop_.valid()) return false;
        if (cfg_.enable_signalk_tcp && !listen(sk_server_, sk_lines_, cfg_.host, cfg_.signalk_port)) return false;
        if (cfg_.enable_nmea0183_tcp_server && !listen(nmea_server_, nmea_lines_, cfg_.nmea0183_tcp_server_host, cfg_.nmea0183_tcp_server_port)) return false;
        if (cfg_.enable_nmea0183_tcp_client) {
            async_event_loop::TcpConnectOptions o;
            o.host = cfg_.nmea0183_tcp_client_host;
            o.port = cfg_.nmea0183_tcp_client_port;
            nmea_client_.connect(o, nmea_client_cb_);
        }
        timer_ = loop_.on_repeat_us(cfg_.publish_interval_us, [this]() { publish(); });
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

    void handle_nmea_line(const char* text, SourceId source_id) {
        if (nmea_.feed_line(text, source_id, loop_.clock().micros())) publish();
    }
    void publish() { publisher_.publish_pending(sk_.clients); }

    struct SkHandler : async_event_loop::ITcpLineServerHandler {
        explicit SkHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        async_event_loop::TcpConnectionRegistry<MaxSignalKClients> clients;
        void on_accept(async_event_loop::ITcpConnection& c, const async_event_loop::TcpPeerInfo&) override { if (!clients.add(c)) c.close(); }
        void on_line(async_event_loop::ITcpConnection&, async_event_loop::LineView) override {}
        void on_backpressure(async_event_loop::ITcpConnection& c, const async_event_loop::TcpBackpressureInfo&) override { clients.remove(c); c.close(); }
        void on_close(async_event_loop::ITcpConnection& c) override { clients.remove(c); }
        void on_error(async_event_loop::ITcpConnection& c, int) override { clients.remove(c); }
        void on_too_many_connections(async_event_loop::ITcpConnection& c) override { c.close(); }
    };

    struct NmeaServerHandler : async_event_loop::ITcpLineServerHandler {
        explicit NmeaServerHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        async_event_loop::TcpConnectionRegistry<MaxNmeaSources> sources;
        void on_accept(async_event_loop::ITcpConnection& c, const async_event_loop::TcpPeerInfo&) override { if (!sources.add(c)) c.close(); }
        void on_line(async_event_loop::ITcpConnection&, async_event_loop::LineView line) override { char text[160]; async_event_loop::line_to_cstr(line, text); owner.handle_nmea_line(text, SourceId(1)); }
        void on_close(async_event_loop::ITcpConnection& c) override { sources.remove(c); }
        void on_error(async_event_loop::ITcpConnection& c, int) override { sources.remove(c); }
        void on_too_many_connections(async_event_loop::ITcpConnection& c) override { c.close(); }
    };

    struct NmeaClientHandler : async_event_loop::ITcpClientHandler {
        explicit NmeaClientHandler(MiniSignalKServer& o) : owner(o) {}
        MiniSignalKServer& owner;
        void on_data(async_event_loop::ITcpConnection& c) override { char text[160]; while (c.read_line(text, sizeof(text))) owner.handle_nmea_line(text, SourceId(2)); }
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
    async_event_loop::EventHandle timer_{};
};

} // namespace signalk_mini
