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
#include "signalk_stream_command.hpp"

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
          signalk_line_handler_(signalk_connections_, signalk_options()),
          http_control_(*this) {
        http_.set_response_control(&http_control_);
    }

    SignalKStreamServer(const SignalKStreamServer&) = delete;
    SignalKStreamServer& operator=(const SignalKStreamServer&) = delete;

    bool listen() {
        if (!listen_tcp(signalk_tcp_server_, signalk_line_handler_, config_.signalk.host, config_.signalk.port)) return false;
        if (config_.signalk.websocket.enabled &&
            !listen_tcp(signalk_websocket_server_, signalk_websocket_protocol_, config_.signalk.websocket.host, config_.signalk.websocket.port)) {
            return false;
        }
        return true;
    }

    // Sends pending deltas, then continues catch-up snapshots for clients that
    // fell behind, then disconnects clients whose output stopped draining.
    void publish() {
        SignalKDeltaFanout fanout{*this};
        publisher_.publish_pending(fanout);
        pump_resync();
        close_stalled_clients();
        http_.pump_all();
    }

    size_t resumable_rest_response_count() const { return http_.resumable_response_count(); }

    // Clients that could not take a whole delta and were switched to catch-up.
    uint32_t catch_up_count() const { return catch_up_count_; }
    uint32_t stalled_client_close_count() const { return stalled_client_close_count_; }
    size_t catching_up_client_count() {
        size_t count = 0;
        auto add = [&](async_event_loop::ITcpConnection&, StreamDeliveryState& state) { if (state.resync) ++count; };
        signalk_connections_.connections.for_each_tx_state(add);
        signalk_websocket_handler_.connections.for_each_tx_state(add);
        return count;
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
        SignalKDeltaFanout fanout{*this};
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
        return write_tcp_frame(connection, hello, static_cast<size_t>(len));
    }

    bool send_signal_k_hello_websocket(async_event_loop::ITcpConnection& connection) {
        char hello[256];
        const int len = write_signal_k_hello(hello, sizeof(hello));
        if (len <= 0) return false;
        return write_websocket_frame(connection, hello, static_cast<size_t>(len));
    }

    // Free space for whole frames: the transport's fixed buffer on embedded
    // targets, or the backpressure high-water mark where buffering is unbounded.
    size_t frame_room(async_event_loop::ITcpConnection& connection) const {
        size_t limit = connection.output_capacity();
        // Never below one whole delta (plus a WebSocket header), or nothing could be sent.
        const size_t min_queue = static_cast<size_t>(config_.publisher.json_buffer_size) + 16u;
        size_t configured = config_.signalk.max_client_queue_bytes;
        if (configured != 0 && configured < min_queue) configured = min_queue;
        // The configured limit can only lower the high-water mark: above it the
        // WebSocket protocol handler would close the client on backpressure.
        const size_t high_water = signalk_backpressure_options().high_water_bytes;
        size_t queue_limit = configured != 0 ? configured : high_water;
        if (high_water != 0 && high_water < queue_limit) queue_limit = high_water;
        if (queue_limit != 0 && queue_limit < limit) limit = queue_limit;
        const size_t used = connection.output_size();
        return used >= limit ? 0 : limit - used;
    }

    // Writes a whole frame or nothing, so a client never receives a partial delta.
    bool write_tcp_frame(async_event_loop::ITcpConnection& connection, const char* data, size_t len) const {
        if (!data || len == 0 || !connection.valid() || frame_room(connection) < len) return false;
        return connection.write(reinterpret_cast<const uint8_t*>(data), len) == static_cast<int>(len);
    }

    bool write_websocket_frame(async_event_loop::ITcpConnection& connection, const char* json, size_t len) {
        const size_t text_len = signalk_text_len(json, len);
        if (!json || text_len == 0) return false;
        const size_t header = text_len < 126 ? 2 : (text_len <= 0xFFFFu ? 4 : 10);
        if (!connection.valid() || frame_room(connection) < text_len + header) return false;
        return signalk_websocket_protocol_.write_text(connection, json, text_len);
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

    static size_t websocket_frame_size(const char* json, size_t len) {
        const size_t text_len = signalk_text_len(json, len);
        return text_len + (text_len < 126 ? 2 : (text_len <= 0xFFFFu ? 4 : 10));
    }

    // Stall detection compares the queue with what was there plus what was added.
    static void note_queued(StreamDeliveryState& state, size_t bytes) {
        const uint64_t total = static_cast<uint64_t>(state.queued_since_sample) + bytes;
        state.queued_since_sample = total > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(total);
    }

    bool deliver_tcp(async_event_loop::ITcpConnection& connection, StreamDeliveryState& state, const char* json, size_t len) {
        if (!write_tcp_frame(connection, json, len)) return false;
        note_queued(state, len);
        return true;
    }

    bool deliver_websocket(async_event_loop::ITcpConnection& connection, StreamDeliveryState& state, const char* json, size_t len) {
        if (!write_websocket_frame(connection, json, len)) return false;
        note_queued(state, websocket_frame_size(json, len));
        return true;
    }

    // Catch-up sinks: a refused frame stops the resumable snapshot at its cursor.
    struct TcpConnectionDeltaSink {
        SignalKStreamServer& owner;
        async_event_loop::ITcpConnection& connection;
        StreamDeliveryState& state;
        bool write_signal_k_delta(const char* json, size_t len) { return owner.deliver_tcp(connection, state, json, len); }
    };

    struct WebSocketConnectionDeltaSink {
        SignalKStreamServer& owner;
        async_event_loop::ITcpConnection& connection;
        StreamDeliveryState& state;
        bool write_signal_k_delta(const char* json, size_t len) { return owner.deliver_websocket(connection, state, json, len); }
    };

    void request_catch_up(StreamDeliveryState& state) {
        if (!state.resync && catch_up_count_ < UINT32_MAX) ++catch_up_count_;
        request_resync(state);
    }

    static void request_resync(StreamDeliveryState& state) {
        if (state.resync) {
            state.resync_again = true;
            return;
        }
        state.resync = true;
        state.resync_again = false;
        state.resync_position = 0;
        state.resync_value = 0;
    }

    // Starts the stream for a client with a full current-value snapshot, which
    // uses the same resumable path as catch-up.
    template<typename Registry>
    void start_stream(Registry& registry, async_event_loop::ITcpConnection& connection, bool include_other_contexts) {
        StreamDeliveryState* state = registry.stream_state(connection);
        if (!state) return;
        *state = StreamDeliveryState{};
        state->include_other_contexts = include_other_contexts;
        state->last_drain_ms = now_ms();
        update_server_clock_model(loop_.clock().micros());
        request_resync(*state);
        pump_one(connection, *state, &registry == &signalk_websocket_handler_.connections);
    }

    void stop_stream(StreamDeliveryState* state) {
        if (state) *state = StreamDeliveryState{};
    }

    void pump_one(async_event_loop::ITcpConnection& connection, StreamDeliveryState& state, bool websocket) {
        if (!state.resync) return;
        const uint64_t now_us = loop_.clock().micros();
        bool complete = false;
        SignalKSnapshotCursor cursor{state.resync_position, state.resync_value};
        if (websocket) {
            WebSocketConnectionDeltaSink sink{*this, connection, state};
            complete = publisher_.publish_current(sink, now_us, state.include_other_contexts, &cursor);
        } else {
            TcpConnectionDeltaSink sink{*this, connection, state};
            complete = publisher_.publish_current(sink, now_us, state.include_other_contexts, &cursor);
        }
        state.resync_position = cursor.position;
        state.resync_value = cursor.value;
        if (!complete) return;
        if (state.resync_again) {
            state.resync_again = false;
            state.resync_position = 0;
            state.resync_value = 0;
        } else {
            state.resync = false;
        }
    }

    void pump_resync() {
        signalk_connections_.connections.for_each_tx_state([&](async_event_loop::ITcpConnection& connection, StreamDeliveryState& state) {
            pump_one(connection, state, false);
        });
        signalk_websocket_handler_.connections.for_each_tx_state([&](async_event_loop::ITcpConnection& connection, StreamDeliveryState& state) {
            pump_one(connection, state, true);
        });
    }

    uint32_t now_ms() const { return static_cast<uint32_t>(loop_.clock().micros() / 1000ULL); }

    // Progress means some queued output was sent since the last check: less is
    // queued now than was queued then plus what was added in between.
    bool stalled(async_event_loop::ITcpConnection& connection, StreamDeliveryState& state, uint32_t now) const {
        const size_t queued = connection.output_size();
        const uint32_t queued32 = queued > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(queued);
        if (stream_output_progressed(queued32, state.last_output_bytes, state.queued_since_sample) || state.last_drain_ms == 0) {
            state.last_drain_ms = now;
        }
        state.last_output_bytes = queued32;
        state.queued_since_sample = 0;
        const uint32_t timeout = config_.publisher.stalled_client_timeout_ms;
        return timeout != 0 && queued32 != 0 && now - state.last_drain_ms > timeout;
    }

    void close_stalled_clients() {
        if (config_.publisher.stalled_client_timeout_ms == 0) return;
        const uint32_t now = now_ms();
        async_event_loop::ITcpConnection* stalled_tcp[MaxSignalKConnections]{};
        async_event_loop::ITcpConnection* stalled_ws[MaxSignalKConnections]{};
        size_t tcp_count = 0;
        size_t ws_count = 0;
        signalk_connections_.connections.for_each_state([&](async_event_loop::ITcpConnection& connection, StreamDeliveryState& state) {
            if (stalled(connection, state, now) && tcp_count < MaxSignalKConnections) stalled_tcp[tcp_count++] = &connection;
        });
        signalk_websocket_handler_.connections.for_each_state([&](async_event_loop::ITcpConnection& connection, StreamDeliveryState& state) {
            if (stalled(connection, state, now) && ws_count < MaxSignalKConnections) stalled_ws[ws_count++] = &connection;
        });
        if (stalled_client_close_count_ < UINT32_MAX - (tcp_count + ws_count)) {
            stalled_client_close_count_ += static_cast<uint32_t>(tcp_count + ws_count);
        }
        for (size_t i = 0; i < tcp_count; ++i) {
            signalk_connections_.connections.remove(*stalled_tcp[i]);
            stalled_tcp[i]->close();
        }
        for (size_t i = 0; i < ws_count; ++i) signalk_websocket_protocol_.close_connection(*stalled_ws[i]);
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
            const SignalKStreamCommand command = signalk_stream_command(line.data, line.size);
            if (command == SignalKStreamCommand::Unsubscribe) {
                connections.set_allow_tx(connection, false);
                owner.stop_stream(connections.stream_state(connection));
            } else if (command == SignalKStreamCommand::SubscribeAll) {
                connections.set_allow_tx(connection, owner.config_.signalk.allow_tx);
                if (owner.config_.signalk.allow_tx) owner.start_stream(connections, connection, true);
            }
        }
        // A client above the high-water mark only stops getting deltas until it
        // drains (then it is caught up); it is closed only if it stalls.
        void on_backpressure(async_event_loop::ITcpConnection&, const async_event_loop::TcpBackpressureInfo&) override {}
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

        void on_http_write_ready(async_event_loop::ITcpConnection& connection) override { owner.http_.pump(connection); }
        void on_http_closed(async_event_loop::ITcpConnection& connection) override { owner.http_.release(connection); }

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
            if (send_current) owner.start_stream(connections, connection, scope == InitialStreamScope::All);
        }

        void on_websocket_text(async_event_loop::ITcpConnection& connection, const char* text, size_t len) override {
            if (!connections.allow_rx(connection)) return;
            const SignalKStreamCommand command = signalk_stream_command(text, len);
            if (command == SignalKStreamCommand::Unsubscribe) {
                connections.set_allow_tx(connection, false);
                owner.stop_stream(connections.stream_state(connection));
            } else if (command == SignalKStreamCommand::SubscribeAll) {
                connections.set_allow_tx(connection, owner.config_.signalk.websocket.allow_tx);
                if (owner.config_.signalk.websocket.allow_tx) owner.start_stream(connections, connection, true);
            }
        }

        void on_websocket_close(async_event_loop::ITcpConnection& connection) override { connections.remove(connection); }
        void on_websocket_error(async_event_loop::ITcpConnection& connection, async_event_loop::WebSocketError) override { connections.remove(connection); }

    };

    // Live deltas go to every streaming client that can take the whole frame
    // now. A client that cannot is skipped and marked for catch-up, which later
    // sends it the current model; while it catches up it still gets deltas
    // that fit.
    struct SignalKDeltaFanout {
        SignalKStreamServer& owner;

        void write_signal_k_delta(const char* json, size_t len) {
            if (!json || len == 0) return;
            owner.signalk_connections_.connections.for_each_tx_state([&](async_event_loop::ITcpConnection& connection, StreamDeliveryState& state) {
                if (!owner.deliver_tcp(connection, state, json, len)) owner.request_catch_up(state);
            });
            owner.signalk_websocket_handler_.connections.for_each_tx_state([&](async_event_loop::ITcpConnection& connection, StreamDeliveryState& state) {
                if (!owner.deliver_websocket(connection, state, json, len)) owner.request_catch_up(state);
            });
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
    struct HttpControl final : IHttpResponseControl {
        explicit HttpControl(SignalKStreamServer& owner_ref) : owner(owner_ref) {}
        bool hold(async_event_loop::ITcpConnection& connection) override {
            return owner.signalk_websocket_protocol_.hold_http_response(connection);
        }
        void finish(async_event_loop::ITcpConnection& connection) override {
            owner.signalk_websocket_protocol_.finish_http_response(connection);
        }
        void abort(async_event_loop::ITcpConnection& connection) override {
            owner.signalk_websocket_protocol_.abort_http_response(connection);
        }
        size_t room(async_event_loop::ITcpConnection& connection) override { return owner.frame_room(connection); }
        uint32_t now_ms() override { return owner.now_ms(); }
        SignalKStreamServer& owner;
    };

    HttpControl http_control_;
    uint32_t catch_up_count_ = 0;
    uint32_t stalled_client_close_count_ = 0;
};

} // namespace signalk_mini
