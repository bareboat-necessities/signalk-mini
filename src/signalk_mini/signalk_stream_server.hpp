#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "stream_io.hpp"
#include <async_event_loop.hpp>
#include "config.hpp"
#include "connection_registry.hpp"
#include "model_store.hpp"
#include "publisher.hpp"
#include "signalk_hello_writer.hpp"
#include "signalk_http.hpp"
#include "signalk_json_stream_writer.hpp"

namespace signalk_mini {

// The Signal K client-facing side: the main TCP stream listener, the optional
// HTTP/WebSocket listener (which also serves SignalKHttpApi), hello messages,
// subscribe handling, current-value snapshots and delta fan-out.
template<typename Real, size_t MaxSignalKConnections>
class SignalKStreamServer {
public:
    SignalKStreamServer(const SignalKMiniConfig& config,
                        async_event_loop::EventLoop<>& loop,
                        ModelStore<Real>& store,
                        SignalKPublisher<Real>& publisher)
        : config_(config),
          loop_(loop),
          store_(store),
          publisher_(publisher),
          http_(store, config),
          signalk_tcp_server_(loop.scheduler()),
          signalk_websocket_server_(loop.scheduler()),
          signalk_connections_(*this),
          signalk_websocket_handler_(*this),
          signalk_websocket_protocol_(signalk_websocket_handler_, signalk_backpressure_options()),
          signalk_line_handler_(signalk_connections_, signalk_options()) {}

    bool listen() {
        if (!listen_tcp(signalk_tcp_server_, signalk_line_handler_, config_.signalk.host, config_.signalk.port)) return false;
        if (config_.signalk.websocket.enabled &&
            !listen_tcp(signalk_websocket_server_, signalk_websocket_protocol_, config_.signalk.websocket.host, config_.signalk.websocket.port)) {
            return false;
        }
        return true;
    }

    void publish() {
        SignalKDeltaFanout fanout{signalk_connections_.connections, signalk_websocket_handler_};
        publisher_.publish_pending(fanout);
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

    static bool is_signalk_stream_resource(const char* resource) {
        if (!resource) return false;
        static constexpr char StreamPath[] = "/signalk/v1/stream";
        const size_t path_len = sizeof(StreamPath) - 1;
        return strncmp(resource, StreamPath, path_len) == 0 &&
               (resource[path_len] == '\0' || resource[path_len] == '?');
    }

    enum class InitialStreamScope : uint8_t { None, Self, All };

    static InitialStreamScope websocket_initial_scope(const char* resource) {
        if (http::query_parameter_equals(resource, "subscribe", "none")) return InitialStreamScope::None;
        if (http::query_parameter_equals(resource, "subscribe", "all")) return InitialStreamScope::All;
        return InitialStreamScope::Self;
    }

    static bool websocket_should_send_current_values(const char* resource) {
        return !http::query_parameter_equals(resource, "sendCachedValues", "false");
    }

    struct TcpConnectionDeltaSink {
        SignalKStreamServer& owner;
        async_event_loop::ITcpConnection& connection;
        void write_signal_k_delta(const char* json, size_t len) {
            write_all(connection, reinterpret_cast<const uint8_t*>(json), len);
        }
    };

    struct WebSocketConnectionDeltaSink {
        SignalKStreamServer& owner;
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
        explicit SignalKConnectionHandler(SignalKStreamServer& owner_ref) : owner(owner_ref) {}
        SignalKStreamServer& owner;
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
        explicit SignalKWebSocketHandler(SignalKStreamServer& owner_ref) : owner(owner_ref) {}
        SignalKStreamServer& owner;
        ConnectionRegistry<MaxSignalKConnections> connections;

        bool on_http_request(async_event_loop::ITcpConnection& connection,
                             const async_event_loop::TcpPeerInfo&,
                             const char* header,
                             size_t header_len) override {
            return owner.http_.handle_request(connection, header, header_len, owner.loop_.clock().micros());
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
            const bool tx_enabled = owner.config_.signalk.websocket.allow_tx && active_subscription;
            const bool send_current = tx_enabled && owner.config_.publisher.send_current_values_on_connect &&
                                      owner.websocket_should_send_current_values(request.resource);
            ConnectionFlags flags{owner.config_.signalk.websocket.allow_rx, tx_enabled};
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


    const SignalKMiniConfig& config_;
    async_event_loop::EventLoop<>& loop_;
    ModelStore<Real>& store_;
    SignalKPublisher<Real>& publisher_;
    SignalKHttpApi<Real> http_;
    async_event_loop::NativeTcpServer signalk_tcp_server_;
    async_event_loop::NativeTcpServer signalk_websocket_server_;
    SignalKConnectionHandler signalk_connections_;
    SignalKWebSocketHandler signalk_websocket_handler_;
    async_event_loop::WebSocketServerHandler<MaxSignalKConnections, async_event_loop::websocket::DefaultHandshakeBufferSize, async_event_loop::websocket::DefaultMaxPayloadSize> signalk_websocket_protocol_;
    async_event_loop::TcpLineServerHandler<256, MaxSignalKConnections> signalk_line_handler_;
};

} // namespace signalk_mini
