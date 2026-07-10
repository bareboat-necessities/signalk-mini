#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tcp.hpp"
#include "websocket.hpp"

namespace async_event_loop {

enum class WebSocketError : uint8_t {
    BadHandshake,
    UnsupportedVersion,
    PayloadTooLarge,
    ProtocolError,
    Backpressure,
    NoSlot,
    WriteFailed,
};

class IWebSocketServerHandler {
public:
    virtual ~IWebSocketServerHandler() = default;

    virtual void on_websocket_open(ITcpConnection& connection,
                                   const TcpPeerInfo& peer,
                                   const websocket::HandshakeRequest& request) {
        (void)connection;
        (void)peer;
        (void)request;
    }

    virtual void on_websocket_text(ITcpConnection& connection,
                                   const char* text,
                                   size_t len) {
        (void)connection;
        (void)text;
        (void)len;
    }

    virtual void on_websocket_text_begin(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_websocket_text_chunk(ITcpConnection& connection,
                                         const char* text,
                                         size_t len) {
        on_websocket_text(connection, text, len);
    }

    virtual void on_websocket_text_end(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_websocket_binary(ITcpConnection& connection,
                                     const uint8_t* data,
                                     size_t len) {
        (void)connection;
        (void)data;
        (void)len;
    }

    virtual void on_websocket_binary_begin(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_websocket_binary_chunk(ITcpConnection& connection,
                                           const uint8_t* data,
                                           size_t len) {
        on_websocket_binary(connection, data, len);
    }

    virtual void on_websocket_binary_end(ITcpConnection& connection) {
        (void)connection;
    }

    virtual bool on_http_request(ITcpConnection& connection,
                                 const TcpPeerInfo& peer,
                                 const char* header,
                                 size_t header_len) {
        (void)connection;
        (void)peer;
        (void)header;
        (void)header_len;
        return false;
    }

    virtual bool accept_websocket_request(const websocket::HandshakeRequest& request) {
        (void)request;
        return true;
    }

    virtual void on_websocket_close(ITcpConnection& connection) {
        (void)connection;
    }

    virtual void on_websocket_error(ITcpConnection& connection, WebSocketError error) {
        (void)connection;
        (void)error;
    }
};

template<size_t MaxConnections,
         size_t HandshakeBufferSize = websocket::DefaultHandshakeBufferSize,
         size_t ChunkSize = websocket::DefaultMaxPayloadSize,
         size_t MaxMessageSize = 65535>
class WebSocketServerHandler final : public ITcpServerHandler {
public:
    explicit WebSocketServerHandler(
        IWebSocketServerHandler& handler,
        TcpBackpressureOptions backpressure = TcpBackpressureOptions::server_default())
        : handler_(handler), backpressure_(backpressure) {
        static_assert(ChunkSize > 0, "ChunkSize must be greater than zero");
        static_assert(MaxMessageSize >= ChunkSize, "MaxMessageSize must be at least ChunkSize");
        for (size_t i = 0; i < MaxConnections; ++i) slots_[i].backpressure.configure(backpressure_);
    }

    void on_accept(ITcpConnection& connection, const TcpPeerInfo& peer) override {
        if (active_count() >= MaxConnections) {
            handler_.on_websocket_error(connection, WebSocketError::NoSlot);
            connection.close();
            return;
        }
        Slot* slot = acquire(connection);
        if (!slot) {
            handler_.on_websocket_error(connection, WebSocketError::NoSlot);
            connection.close();
            return;
        }
        slot->peer = peer;
        slot->state = State::Handshaking;
        reset_stream_state(*slot);
        slot->backpressure.configure(backpressure_);
    }

    void on_data(ITcpConnection& connection) override {
        Slot* slot = find(connection);
        if (!slot) return;
        if (slot->state == State::HttpClosing) {
            drain_input(connection);
            try_close_drained_http(*slot);
            return;
        }

        while (connection.valid()) {
            if (slot->input_size < sizeof(slot->input) && connection.readable()) {
                const int n = connection.read(slot->input + slot->input_size,
                                              sizeof(slot->input) - slot->input_size);
                if (n < 0) {
                    fail_and_close(*slot, WebSocketError::ProtocolError);
                    return;
                }
                if (n > 0) slot->input_size += static_cast<size_t>(n);
            }

            const size_t before = slot->input_size;
            if (!process_slot(*slot)) return;
            if (!slot->connection) return;
            if (slot->input_size < before) continue;
            if (!connection.readable()) break;
            if (slot->input_size >= sizeof(slot->input)) {
                fail_and_close(*slot, WebSocketError::ProtocolError);
                return;
            }
        }
        if (slot->connection) check_backpressure(*slot);
    }

    void on_write_ready(ITcpConnection& connection) override {
        Slot* slot = find(connection);
        if (!slot) return;
        if (slot->state == State::HttpClosing) {
            try_close_drained_http(*slot);
            return;
        }
        check_backpressure(*slot);
    }

    void on_close(ITcpConnection& connection) override {
        Slot* slot = find(connection);
        if (!slot) return;
        const bool was_open = slot->state == State::Open;
        release(connection);
        if (was_open) handler_.on_websocket_close(connection);
    }

    void on_error(ITcpConnection& connection, int error_code) override {
        (void)error_code;
        Slot* slot = find(connection);
        if (!slot) return;
        if (slot->state != State::HttpClosing) handler_.on_websocket_error(connection, WebSocketError::ProtocolError);
        release(connection);
    }

    bool write_text(ITcpConnection& connection, const char* text, size_t text_len) {
        if (!is_open(connection) || !text || text_len > ChunkSize) return false;
        uint8_t frame[ChunkSize + 16];
        size_t frame_len = 0;
        if (!websocket::encode_text_frame(text, text_len, frame, sizeof(frame), frame_len)) return false;
        return write_all(connection, frame, frame_len);
    }

    bool write_close(ITcpConnection& connection) {
        return write_control(connection, websocket::Opcode::Close, nullptr, 0);
    }

    bool write_pong(ITcpConnection& connection, const uint8_t* payload, size_t payload_len) {
        return write_control(connection, websocket::Opcode::Pong, payload, payload_len);
    }

    size_t active_count() const {
        size_t count = 0;
        for (size_t i = 0; i < MaxConnections; ++i) if (slots_[i].connection) ++count;
        return count;
    }

    bool is_open(ITcpConnection& connection) const {
        const Slot* slot = find_const(connection);
        return slot && slot->state == State::Open && connection.valid();
    }

private:
    enum class State : uint8_t { Empty, Handshaking, Open, HttpClosing };

    struct Slot {
        ITcpConnection* connection = nullptr;
        TcpPeerInfo peer{};
        State state = State::Empty;
        size_t input_size = 0;
        uint8_t input[HandshakeBufferSize + ChunkSize + 16]{};
        uint8_t chunk[ChunkSize + 1]{};
        size_t chunk_size = 0;
        uint8_t control[125]{};
        size_t control_size = 0;
        bool frame_active = false;
        bool frame_fin = false;
        bool frame_control = false;
        websocket::Opcode frame_opcode = websocket::Opcode::Continuation;
        uint64_t frame_remaining = 0;
        uint64_t frame_offset = 0;
        uint8_t frame_mask[4]{};
        bool message_active = false;
        websocket::Opcode message_opcode = websocket::Opcode::Continuation;
        size_t message_size = 0;
        TcpBackpressureMonitor backpressure;
    };

    Slot* find(ITcpConnection& connection) {
        for (size_t i = 0; i < MaxConnections; ++i) if (slots_[i].connection == &connection) return &slots_[i];
        return nullptr;
    }

    const Slot* find_const(ITcpConnection& connection) const {
        for (size_t i = 0; i < MaxConnections; ++i) if (slots_[i].connection == &connection) return &slots_[i];
        return nullptr;
    }

    Slot* acquire(ITcpConnection& connection) {
        if (Slot* existing = find(connection)) return existing;
        for (size_t i = 0; i < MaxConnections; ++i) {
            if (!slots_[i].connection) {
                slots_[i].connection = &connection;
                slots_[i].state = State::Handshaking;
                reset_stream_state(slots_[i]);
                return &slots_[i];
            }
        }
        return nullptr;
    }

    void release(ITcpConnection& connection) {
        Slot* slot = find(connection);
        if (!slot) return;
        slot->connection = nullptr;
        slot->peer = TcpPeerInfo{};
        slot->state = State::Empty;
        reset_stream_state(*slot);
        slot->backpressure.reset();
    }

    static void reset_stream_state(Slot& slot) {
        slot.input_size = 0;
        slot.chunk_size = 0;
        slot.control_size = 0;
        slot.frame_active = false;
        slot.frame_fin = false;
        slot.frame_control = false;
        slot.frame_opcode = websocket::Opcode::Continuation;
        slot.frame_remaining = 0;
        slot.frame_offset = 0;
        memset(slot.frame_mask, 0, sizeof(slot.frame_mask));
        slot.message_active = false;
        slot.message_opcode = websocket::Opcode::Continuation;
        slot.message_size = 0;
    }

    bool process_slot(Slot& slot) {
        if (!slot.connection) return false;
        if (slot.state == State::Handshaking && !process_handshake(slot)) return false;
        if (!slot.connection || slot.state != State::Open) return slot.connection != nullptr;

        while (slot.connection && slot.input_size > 0) {
            if (!slot.frame_active) {
                const HeaderResult result = begin_frame(slot);
                if (result == HeaderResult::NeedMore) return true;
                if (result == HeaderResult::PayloadTooLarge) {
                    fail_and_close(slot, WebSocketError::PayloadTooLarge);
                    return false;
                }
                if (result != HeaderResult::Ok) {
                    fail_and_close(slot, WebSocketError::ProtocolError);
                    return false;
                }
            }
            if (!consume_frame_payload(slot)) return false;
            if (slot.frame_active) return true;
        }
        return slot.connection != nullptr;
    }

    bool process_handshake(Slot& slot) {
        const size_t header_len = websocket_header_length(slot.input, slot.input_size);
        if (header_len == 0) {
            if (slot.input_size >= HandshakeBufferSize) fail_and_close(slot, WebSocketError::BadHandshake);
            return slot.connection != nullptr;
        }

        websocket::HandshakeRequest request;
        const websocket::HandshakeResult parsed = websocket::parse_client_handshake(
            reinterpret_cast<const char*>(slot.input), header_len, request);
        if (parsed != websocket::HandshakeResult::Ok) {
            if (handler_.on_http_request(*slot.connection, slot.peer,
                                         reinterpret_cast<const char*>(slot.input), header_len)) {
                consume(slot, header_len);
                begin_http_close(slot);
                return slot.connection != nullptr;
            }
            fail_and_close(slot, parsed == websocket::HandshakeResult::UnsupportedVersion
                                    ? WebSocketError::UnsupportedVersion
                                    : WebSocketError::BadHandshake);
            return false;
        }

        if (!handler_.accept_websocket_request(request)) {
            static constexpr char NotFound[] =
                "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
            if (!write_all(*slot.connection, reinterpret_cast<const uint8_t*>(NotFound), sizeof(NotFound) - 1)) {
                fail_and_close(slot, WebSocketError::WriteFailed);
                return false;
            }
            consume(slot, header_len);
            begin_http_close(slot);
            return slot.connection != nullptr;
        }

        char response[192];
        const websocket::HandshakeResult written = websocket::write_server_handshake_response(
            request.key, response, sizeof(response));
        if (written != websocket::HandshakeResult::Ok ||
            !write_all(*slot.connection, reinterpret_cast<const uint8_t*>(response), strlen(response))) {
            fail_and_close(slot, WebSocketError::WriteFailed);
            return false;
        }

        slot.state = State::Open;
        handler_.on_websocket_open(*slot.connection, slot.peer, request);
        consume(slot, header_len);
        return true;
    }

    enum class HeaderResult : uint8_t { Ok, NeedMore, ProtocolError, PayloadTooLarge };

    HeaderResult begin_frame(Slot& slot) {
        if (slot.input_size < 2) return HeaderResult::NeedMore;
        const uint8_t b0 = slot.input[0];
        const uint8_t b1 = slot.input[1];
        const bool fin = (b0 & 0x80u) != 0;
        if ((b0 & 0x70u) != 0 || (b1 & 0x80u) == 0) return HeaderResult::ProtocolError;

        const websocket::Opcode opcode = static_cast<websocket::Opcode>(b0 & 0x0Fu);
        uint64_t payload_len = b1 & 0x7Fu;
        size_t offset = 2;
        if (payload_len == 126) {
            if (slot.input_size < 4) return HeaderResult::NeedMore;
            payload_len = (static_cast<uint16_t>(slot.input[2]) << 8) | slot.input[3];
            offset = 4;
        } else if (payload_len == 127) {
            if (slot.input_size < 10) return HeaderResult::NeedMore;
            payload_len = 0;
            for (size_t i = 0; i < 8; ++i) payload_len = (payload_len << 8) | slot.input[2 + i];
            offset = 10;
        }
        if (slot.input_size < offset + 4) return HeaderResult::NeedMore;

        const bool control = static_cast<uint8_t>(opcode) >= 0x8u;
        if (control) {
            if (!fin || payload_len > 125 ||
                (opcode != websocket::Opcode::Close && opcode != websocket::Opcode::Ping && opcode != websocket::Opcode::Pong)) {
                return HeaderResult::ProtocolError;
            }
        } else if (opcode == websocket::Opcode::Text || opcode == websocket::Opcode::Binary) {
            if (slot.message_active) return HeaderResult::ProtocolError;
            slot.message_active = true;
            slot.message_opcode = opcode;
            slot.message_size = 0;
            slot.chunk_size = 0;
            if (opcode == websocket::Opcode::Text) handler_.on_websocket_text_begin(*slot.connection);
            else handler_.on_websocket_binary_begin(*slot.connection);
        } else if (opcode == websocket::Opcode::Continuation) {
            if (!slot.message_active) return HeaderResult::ProtocolError;
        } else {
            return HeaderResult::ProtocolError;
        }

        if (!control && payload_len > MaxMessageSize - slot.message_size) return HeaderResult::PayloadTooLarge;
        memcpy(slot.frame_mask, slot.input + offset, 4);
        consume(slot, offset + 4);
        slot.frame_active = true;
        slot.frame_fin = fin;
        slot.frame_control = control;
        slot.frame_opcode = opcode;
        slot.frame_remaining = payload_len;
        slot.frame_offset = 0;
        slot.control_size = 0;

        if (payload_len == 0) return finish_frame(slot) ? HeaderResult::Ok : HeaderResult::ProtocolError;
        return HeaderResult::Ok;
    }

    bool consume_frame_payload(Slot& slot) {
        while (slot.frame_active && slot.input_size > 0) {
            const size_t available = slot.input_size;
            const size_t remaining = static_cast<size_t>(slot.frame_remaining);
            const size_t take = available < remaining ? available : remaining;
            if (slot.frame_control) {
                if (slot.control_size + take > sizeof(slot.control)) return protocol_failure(slot);
                for (size_t i = 0; i < take; ++i) {
                    slot.control[slot.control_size + i] =
                        slot.input[i] ^ slot.frame_mask[(slot.frame_offset + i) & 3u];
                }
                slot.control_size += take;
            } else {
                size_t consumed_input = 0;
                while (consumed_input < take) {
                    const size_t room = ChunkSize - slot.chunk_size;
                    const size_t part = (take - consumed_input) < room ? (take - consumed_input) : room;
                    for (size_t i = 0; i < part; ++i) {
                        slot.chunk[slot.chunk_size + i] =
                            slot.input[consumed_input + i] ^
                            slot.frame_mask[(slot.frame_offset + consumed_input + i) & 3u];
                    }
                    slot.chunk_size += part;
                    slot.message_size += part;
                    consumed_input += part;
                    if (slot.message_size > MaxMessageSize) {
                        fail_and_close(slot, WebSocketError::PayloadTooLarge);
                        return false;
                    }
                    if (slot.chunk_size == ChunkSize && !flush_chunk(slot)) return false;
                }
            }
            consume(slot, take);
            slot.frame_offset += take;
            slot.frame_remaining -= take;
            if (slot.frame_remaining == 0) return finish_frame(slot);
        }
        return true;
    }

    bool finish_frame(Slot& slot) {
        const bool fin = slot.frame_fin;
        const bool control = slot.frame_control;
        const websocket::Opcode opcode = slot.frame_opcode;
        slot.frame_active = false;

        if (control) {
            if (opcode == websocket::Opcode::Ping) {
                if (!write_pong(*slot.connection, slot.control, slot.control_size)) {
                    fail_and_close(slot, WebSocketError::WriteFailed);
                    return false;
                }
                return true;
            }
            if (opcode == websocket::Opcode::Pong) return true;
            if (opcode == websocket::Opcode::Close) {
                ITcpConnection& connection = *slot.connection;
                (void)write_control(connection, websocket::Opcode::Close, slot.control, slot.control_size);
                close_slot(slot, true);
                return false;
            }
            return protocol_failure(slot);
        }

        if (fin) {
            if (!flush_chunk(slot)) return false;
            if (slot.message_opcode == websocket::Opcode::Text) handler_.on_websocket_text_end(*slot.connection);
            else handler_.on_websocket_binary_end(*slot.connection);
            slot.message_active = false;
            slot.message_opcode = websocket::Opcode::Continuation;
            slot.message_size = 0;
        }
        return true;
    }

    bool flush_chunk(Slot& slot) {
        if (slot.chunk_size == 0) return true;
        if (slot.message_opcode == websocket::Opcode::Text) {
            slot.chunk[slot.chunk_size] = '\0';
            handler_.on_websocket_text_chunk(*slot.connection,
                                             reinterpret_cast<const char*>(slot.chunk),
                                             slot.chunk_size);
        } else {
            handler_.on_websocket_binary_chunk(*slot.connection, slot.chunk, slot.chunk_size);
        }
        slot.chunk_size = 0;
        return slot.connection != nullptr && slot.connection->valid();
    }

    bool protocol_failure(Slot& slot) {
        fail_and_close(slot, WebSocketError::ProtocolError);
        return false;
    }

    bool write_control(ITcpConnection& connection,
                       websocket::Opcode opcode,
                       const uint8_t* payload,
                       size_t payload_len) {
        uint8_t frame[128];
        size_t frame_len = 0;
        if (!websocket::encode_control_frame(opcode, payload, payload_len,
                                             frame, sizeof(frame), frame_len)) return false;
        return write_all(connection, frame, frame_len);
    }

    bool write_all(ITcpConnection& connection, const uint8_t* data, size_t length) {
        if (!data || length == 0 || !connection.valid() || !connection.writable()) return false;
        const int n = connection.write(data, length);
        return n == static_cast<int>(length);
    }

    void begin_http_close(Slot& slot) {
        if (!slot.connection) return;
        slot.state = State::HttpClosing;
        try_close_drained_http(slot);
    }

    void try_close_drained_http(Slot& slot) {
        if (!slot.connection || slot.state != State::HttpClosing || slot.connection->output_size() != 0) return;
        ITcpConnection* connection = slot.connection;
        release(*connection);
        connection->close();
    }

    static void drain_input(ITcpConnection& connection) {
        uint8_t discard[64];
        while (connection.valid() && connection.readable()) {
            const int n = connection.read(discard, sizeof(discard));
            if (n <= 0) break;
        }
    }

    void fail_and_close(Slot& slot, WebSocketError error) {
        if (!slot.connection) return;
        handler_.on_websocket_error(*slot.connection, error);
        close_slot(slot, false);
    }

    void close_slot(Slot& slot, bool notify_close) {
        ITcpConnection* connection = slot.connection;
        if (!connection) return;
        const bool was_open = slot.state == State::Open;
        release(*connection);
        if (notify_close && was_open) handler_.on_websocket_close(*connection);
        connection->close();
    }

    void check_backpressure(Slot& slot) {
        if (!slot.connection || !slot.connection->valid()) return;
        TcpBackpressureInfo info;
        if (slot.backpressure.check(*slot.connection, &info) && info.close_on_limit) {
            fail_and_close(slot, WebSocketError::Backpressure);
        }
    }

    static size_t websocket_header_length(const uint8_t* data, size_t size) {
        if (!data || size < 4) return 0;
        for (size_t i = 3; i < size; ++i) {
            if (data[i - 3] == '\r' && data[i - 2] == '\n' &&
                data[i - 1] == '\r' && data[i] == '\n') return i + 1;
        }
        return 0;
    }

    static void consume(Slot& slot, size_t count) {
        if (count >= slot.input_size) {
            slot.input_size = 0;
            return;
        }
        memmove(slot.input, slot.input + count, slot.input_size - count);
        slot.input_size -= count;
    }

    IWebSocketServerHandler& handler_;
    TcpBackpressureOptions backpressure_;
    Slot slots_[MaxConnections]{};
};

} // namespace async_event_loop
