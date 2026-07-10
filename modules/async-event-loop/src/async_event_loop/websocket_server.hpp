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

    virtual void on_websocket_open(ITcpConnection& connection, const TcpPeerInfo& peer, const websocket::HandshakeRequest& request) {
        (void)connection;
        (void)peer;
        (void)request;
    }

    virtual void on_websocket_text(ITcpConnection& connection, const char* text, size_t len) {
        (void)connection;
        (void)text;
        (void)len;
    }

    virtual void on_websocket_binary(ITcpConnection& connection, const uint8_t* data, size_t len) {
        (void)connection;
        (void)data;
        (void)len;
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
         size_t MaxPayloadSize = websocket::DefaultMaxPayloadSize>
class WebSocketServerHandler final : public ITcpServerHandler {
public:
    explicit WebSocketServerHandler(IWebSocketServerHandler& handler,
                                    TcpBackpressureOptions backpressure = TcpBackpressureOptions::server_default())
        : handler_(handler), backpressure_(backpressure) {
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
        slot->input_size = 0;
        slot->backpressure.configure(backpressure_);
    }

    void on_data(ITcpConnection& connection) override {
        Slot* slot = find(connection);
        if (!slot) return;
        while (connection.valid() && connection.readable()) {
            if (slot->input_size >= sizeof(slot->input)) {
                fail_and_close(*slot, WebSocketError::ProtocolError);
                return;
            }
            const int n = connection.read(slot->input + slot->input_size, sizeof(slot->input) - slot->input_size);
            if (n < 0) {
                fail_and_close(*slot, WebSocketError::ProtocolError);
                return;
            }
            if (n == 0) break;
            slot->input_size += static_cast<size_t>(n);
            if (!process_slot(*slot)) return;
        }
        check_backpressure(*slot);
    }

    void on_write_ready(ITcpConnection& connection) override {
        Slot* slot = find(connection);
        if (slot) check_backpressure(*slot);
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
        handler_.on_websocket_error(connection, WebSocketError::ProtocolError);
        release(connection);
    }

    bool write_text(ITcpConnection& connection, const char* text, size_t text_len) {
        if (!is_open(connection) || !text) return false;
        uint8_t frame[MaxPayloadSize + 16];
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
    enum class State : uint8_t { Empty, Handshaking, Open };

    struct Slot {
        ITcpConnection* connection = nullptr;
        TcpPeerInfo peer{};
        State state = State::Empty;
        size_t input_size = 0;
        uint8_t input[HandshakeBufferSize + MaxPayloadSize + 16]{};
        uint8_t payload[MaxPayloadSize + 1]{};
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
                slots_[i].input_size = 0;
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
        slot->input_size = 0;
        slot->backpressure.reset();
    }

    bool process_slot(Slot& slot) {
        if (!slot.connection) return false;
        if (slot.state == State::Handshaking) {
            const size_t header_len = websocket_header_length(slot.input, slot.input_size);
            if (header_len == 0) {
                if (slot.input_size >= HandshakeBufferSize) fail_and_close(slot, WebSocketError::BadHandshake);
                return slot.connection != nullptr;
            }

            websocket::HandshakeRequest request;
            const websocket::HandshakeResult parsed = websocket::parse_client_handshake(reinterpret_cast<const char*>(slot.input), header_len, request);
            if (parsed != websocket::HandshakeResult::Ok) {
                if (handler_.on_http_request(*slot.connection,
                                             slot.peer,
                                             reinterpret_cast<const char*>(slot.input),
                                             header_len)) {
                    ITcpConnection* connection = slot.connection;
                    release(*connection);
                    connection->close();
                    return false;
                }
                fail_and_close(slot, parsed == websocket::HandshakeResult::UnsupportedVersion ? WebSocketError::UnsupportedVersion : WebSocketError::BadHandshake);
                return false;
            }

            if (!handler_.accept_websocket_request(request)) {
                static constexpr char NotFound[] =
                    "HTTP/1.1 404 Not Found
Content-Length: 0
Connection: close

";
                (void)write_all(*slot.connection,
                                reinterpret_cast<const uint8_t*>(NotFound),
                                sizeof(NotFound) - 1);
                ITcpConnection* connection = slot.connection;
                release(*connection);
                connection->close();
                return false;
            }

            char response[192];
            const websocket::HandshakeResult written = websocket::write_server_handshake_response(request.key, response, sizeof(response));
            if (written != websocket::HandshakeResult::Ok || !write_all(*slot.connection, reinterpret_cast<const uint8_t*>(response), strlen(response))) {
                fail_and_close(slot, WebSocketError::WriteFailed);
                return false;
            }

            slot.state = State::Open;
            handler_.on_websocket_open(*slot.connection, slot.peer, request);
            consume(slot, header_len);
        }

        while (slot.connection && slot.state == State::Open && slot.input_size > 0) {
            websocket::FrameView frame;
            size_t consumed = 0;
            const websocket::FrameDecodeResult decoded = websocket::decode_frame(slot.input, slot.input_size, slot.payload, MaxPayloadSize, frame, consumed, true);
            if (decoded == websocket::FrameDecodeResult::NeedMore) return true;
            if (decoded == websocket::FrameDecodeResult::PayloadTooLarge) {
                fail_and_close(slot, WebSocketError::PayloadTooLarge);
                return false;
            }
            if (decoded != websocket::FrameDecodeResult::Ok || consumed == 0) {
                fail_and_close(slot, WebSocketError::ProtocolError);
                return false;
            }
            if (!handle_frame(slot, frame)) return false;
            consume(slot, consumed);
        }
        return slot.connection != nullptr;
    }

    bool handle_frame(Slot& slot, const websocket::FrameView& frame) {
        if (!slot.connection) return false;
        switch (frame.opcode) {
        case websocket::Opcode::Text:
            if (!frame.fin) {
                fail_and_close(slot, WebSocketError::ProtocolError);
                return false;
            }
            slot.payload[frame.payload_len] = '\0';
            handler_.on_websocket_text(*slot.connection, reinterpret_cast<const char*>(slot.payload), frame.payload_len);
            return true;
        case websocket::Opcode::Binary:
            if (!frame.fin) {
                fail_and_close(slot, WebSocketError::ProtocolError);
                return false;
            }
            handler_.on_websocket_binary(*slot.connection, frame.payload, frame.payload_len);
            return true;
        case websocket::Opcode::Ping:
            if (!write_pong(*slot.connection, frame.payload, frame.payload_len)) {
                fail_and_close(slot, WebSocketError::WriteFailed);
                return false;
            }
            return true;
        case websocket::Opcode::Pong:
            return true;
        case websocket::Opcode::Close: {
            ITcpConnection& connection = *slot.connection;
            write_control(connection, websocket::Opcode::Close, frame.payload, frame.payload_len);
            close_slot(slot, true);
            return false;
        }
        default:
            fail_and_close(slot, WebSocketError::ProtocolError);
            return false;
        }
    }

    bool write_control(ITcpConnection& connection, websocket::Opcode opcode, const uint8_t* payload, size_t payload_len) {
        uint8_t frame[128];
        size_t frame_len = 0;
        if (!websocket::encode_control_frame(opcode, payload, payload_len, frame, sizeof(frame), frame_len)) return false;
        return write_all(connection, frame, frame_len);
    }

    bool write_all(ITcpConnection& connection, const uint8_t* data, size_t length) {
        if (!data || length == 0 || !connection.valid() || !connection.writable()) return false;
        const int n = connection.write(data, length);
        return n == static_cast<int>(length);
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
            if (data[i - 3] == '\r' && data[i - 2] == '\n' && data[i - 1] == '\r' && data[i] == '\n') return i + 1;
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
