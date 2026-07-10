#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <async_event_loop/websocket_server.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

class FakeConnection final : public async_event_loop::ITcpConnection {
public:
    int read(uint8_t* dst, size_t max_len) override {
        if (!dst || max_len == 0 || input_.empty()) return 0;
        const size_t n = std::min(max_len, std::min(input_.size(), read_limit_));
        std::memcpy(dst, input_.data(), n);
        input_.erase(input_.begin(), input_.begin() + static_cast<std::vector<uint8_t>::difference_type>(n));
        return static_cast<int>(n);
    }

    int write(const uint8_t* src, size_t len) override {
        if (!src || len == 0) return 0;
        output_.insert(output_.end(), src, src + len);
        return static_cast<int>(len);
    }

    bool valid() const override { return valid_; }
    void close() override { valid_ = false; }
    const async_event_loop::TcpPeerInfo& peer() const override { return peer_; }
    size_t input_size() const override { return input_.size(); }
    size_t output_size() const override { return 0; }
    bool peek(uint8_t* dst, size_t len) override {
        if (!dst || len > input_.size()) return false;
        std::memcpy(dst, input_.data(), len);
        return true;
    }
    bool read_exact(uint8_t* dst, size_t len) override { return read(dst, len) == static_cast<int>(len); }
    bool read_line(char*, size_t, bool = true) override { return false; }

    void push_input(const uint8_t* data, size_t len) { input_.insert(input_.end(), data, data + len); }
    void push_input(const std::string& text) { push_input(reinterpret_cast<const uint8_t*>(text.data()), text.size()); }
    void clear_output() { output_.clear(); }
    const std::vector<uint8_t>& output() const { return output_; }
    void set_read_limit(size_t value) { read_limit_ = value ? value : 1; }

private:
    async_event_loop::TcpPeerInfo peer_{};
    std::vector<uint8_t> input_;
    std::vector<uint8_t> output_;
    size_t read_limit_ = static_cast<size_t>(-1);
    bool valid_ = true;
};

class CaptureWebSocketHandler final : public async_event_loop::IWebSocketServerHandler {
public:
    void on_websocket_open(async_event_loop::ITcpConnection&, const async_event_loop::TcpPeerInfo&, const async_event_loop::websocket::HandshakeRequest& request) override {
        ++open_count;
        resource = request.resource;
    }

    void on_websocket_text_begin(async_event_loop::ITcpConnection&) override {
        ++text_begin_count;
        current_text.clear();
    }

    void on_websocket_text(async_event_loop::ITcpConnection&, const char* text, size_t len) override {
        ++text_count;
        last_text.assign(text, len);
        current_text.append(text, len);
    }

    void on_websocket_text_end(async_event_loop::ITcpConnection&) override {
        ++text_end_count;
        completed_text = current_text;
    }

    void on_websocket_close(async_event_loop::ITcpConnection&) override { ++close_count; }

    void on_websocket_error(async_event_loop::ITcpConnection&, async_event_loop::WebSocketError error) override {
        ++error_count;
        last_error = error;
    }

    int open_count = 0;
    int text_count = 0;
    int text_begin_count = 0;
    int text_end_count = 0;
    int close_count = 0;
    int error_count = 0;
    async_event_loop::WebSocketError last_error = async_event_loop::WebSocketError::ProtocolError;
    std::string resource;
    std::string last_text;
    std::string current_text;
    std::string completed_text;
};

static std::vector<uint8_t> masked_frame(async_event_loop::websocket::Opcode opcode,
                                         const std::string& text,
                                         bool fin = true) {
    REQUIRE(text.size() <= 65535u);
    const uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
    const size_t header = text.size() < 126 ? 6 : 8;
    std::vector<uint8_t> frame(header + text.size());
    frame[0] = (fin ? 0x80u : 0u) | static_cast<uint8_t>(opcode);
    if (text.size() < 126) {
        frame[1] = 0x80u | static_cast<uint8_t>(text.size());
        std::memcpy(frame.data() + 2, mask, sizeof(mask));
    } else {
        frame[1] = 0x80u | 126u;
        frame[2] = static_cast<uint8_t>((text.size() >> 8) & 0xffu);
        frame[3] = static_cast<uint8_t>(text.size() & 0xffu);
        std::memcpy(frame.data() + 4, mask, sizeof(mask));
    }
    for (size_t i = 0; i < text.size(); ++i) frame[header + i] = static_cast<uint8_t>(text[i]) ^ mask[i & 3u];
    return frame;
}

static std::string valid_handshake() {
    return
        "GET /signalk/v1/stream?subscribe=none HTTP/1.1\r\n"
        "Host: example.local:3000\r\n"
        "Upgrade: websocket\r\n"
        "Connection: keep-alive, Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n";
}

static void open_connection(async_event_loop::WebSocketServerHandler<2, 512, 128>& server,
                            FakeConnection& connection,
                            CaptureWebSocketHandler& capture) {
    async_event_loop::TcpPeerInfo peer{};
    server.on_accept(connection, peer);
    connection.push_input(valid_handshake());
    server.on_data(connection);
    REQUIRE(connection.valid());
    REQUIRE(server.is_open(connection));
    REQUIRE(capture.open_count == 1);
}

static void test_open_text_ping_and_close() {
    CaptureWebSocketHandler capture;
    async_event_loop::WebSocketServerHandler<2, 512, 128> server(capture, async_event_loop::TcpBackpressureOptions::disabled());
    FakeConnection connection;
    open_connection(server, connection, capture);
    REQUIRE(capture.resource == "/signalk/v1/stream?subscribe=none");
    const std::string handshake_output(reinterpret_cast<const char*>(connection.output().data()), connection.output().size());
    REQUIRE(handshake_output.find("HTTP/1.1 101 Switching Protocols\r\n") == 0);

    const std::vector<uint8_t> text = masked_frame(async_event_loop::websocket::Opcode::Text, "subscribe all");
    connection.push_input(text.data(), text.size());
    server.on_data(connection);
    REQUIRE(capture.text_begin_count == 1);
    REQUIRE(capture.text_count == 1);
    REQUIRE(capture.text_end_count == 1);
    REQUIRE(capture.completed_text == "subscribe all");

    connection.clear_output();
    const std::vector<uint8_t> ping = masked_frame(async_event_loop::websocket::Opcode::Ping, "hi");
    connection.push_input(ping.data(), ping.size());
    server.on_data(connection);
    REQUIRE(!connection.output().empty());
    uint8_t decoded[8];
    async_event_loop::websocket::FrameView view;
    size_t consumed = 0;
    REQUIRE(async_event_loop::websocket::decode_frame(connection.output().data(), connection.output().size(), decoded, sizeof(decoded), view, consumed, false) == async_event_loop::websocket::FrameDecodeResult::Ok);
    REQUIRE(view.opcode == async_event_loop::websocket::Opcode::Pong);
    REQUIRE(view.payload_len == 2);
    REQUIRE(std::memcmp(view.payload, "hi", 2) == 0);

    const std::vector<uint8_t> close = masked_frame(async_event_loop::websocket::Opcode::Close, "");
    connection.push_input(close.data(), close.size());
    server.on_data(connection);
    REQUIRE(!connection.valid());
    REQUIRE(capture.close_count == 1);
    REQUIRE(server.active_count() == 0);
}

static std::string large_subscription() {
    std::string json = "{\"context\":\"vessels.self\",\"subscribe\":[";
    for (int i = 0; i < 80; ++i) {
        if (i) json += ',';
        json += "{\"path\":\"environment.test.path" + std::to_string(i) + "\",\"policy\":\"instant\",\"minPeriod\":100}";
    }
    json += "]}";
    REQUIRE(json.size() > 4096);
    return json;
}

static void test_large_frame_with_interrupted_tcp_reads() {
    CaptureWebSocketHandler capture;
    async_event_loop::WebSocketServerHandler<2, 512, 128, 16384> server(capture, async_event_loop::TcpBackpressureOptions::disabled());
    FakeConnection connection;
    open_connection(server, connection, capture);
    connection.set_read_limit(17);

    const std::string json = large_subscription();
    const std::vector<uint8_t> frame = masked_frame(async_event_loop::websocket::Opcode::Text, json);
    connection.push_input(frame.data(), frame.size());
    server.on_data(connection);

    REQUIRE(connection.valid());
    REQUIRE(server.is_open(connection));
    REQUIRE(capture.error_count == 0);
    REQUIRE(capture.text_begin_count == 1);
    REQUIRE(capture.text_end_count == 1);
    REQUIRE(capture.text_count > 1);
    REQUIRE(capture.completed_text == json);
}

static void test_fragmented_message_with_interleaved_ping() {
    CaptureWebSocketHandler capture;
    async_event_loop::WebSocketServerHandler<2, 512, 128, 16384> server(capture, async_event_loop::TcpBackpressureOptions::disabled());
    FakeConnection connection;
    open_connection(server, connection, capture);
    connection.set_read_limit(23);

    const std::string json = large_subscription();
    const size_t split = json.size() / 2;
    const std::vector<uint8_t> first = masked_frame(async_event_loop::websocket::Opcode::Text, json.substr(0, split), false);
    const std::vector<uint8_t> ping = masked_frame(async_event_loop::websocket::Opcode::Ping, "ok");
    const std::vector<uint8_t> second = masked_frame(async_event_loop::websocket::Opcode::Continuation, json.substr(split), true);
    connection.push_input(first.data(), first.size());
    connection.push_input(ping.data(), ping.size());
    connection.push_input(second.data(), second.size());
    server.on_data(connection);

    REQUIRE(connection.valid());
    REQUIRE(server.is_open(connection));
    REQUIRE(capture.error_count == 0);
    REQUIRE(capture.text_begin_count == 1);
    REQUIRE(capture.text_end_count == 1);
    REQUIRE(capture.completed_text == json);
    REQUIRE(!connection.output().empty());
}

static void test_bad_handshake_closes() {
    CaptureWebSocketHandler capture;
    async_event_loop::WebSocketServerHandler<1, 128, 64> server(capture, async_event_loop::TcpBackpressureOptions::disabled());
    FakeConnection connection;
    async_event_loop::TcpPeerInfo peer{};
    server.on_accept(connection, peer);
    connection.push_input("GET / HTTP/1.1\r\nHost: example\r\n\r\n");
    server.on_data(connection);
    REQUIRE(!connection.valid());
    REQUIRE(capture.open_count == 0);
    REQUIRE(capture.error_count == 1);
    REQUIRE(capture.last_error == async_event_loop::WebSocketError::BadHandshake);
    REQUIRE(server.active_count() == 0);
}

int main() {
    test_open_text_ping_and_close();
    test_large_frame_with_interrupted_tcp_reads();
    test_fragmented_message_with_interleaved_ping();
    test_bad_handshake_closes();
    return 0;
}
