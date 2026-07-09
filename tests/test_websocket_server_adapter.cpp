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
        const size_t n = std::min(max_len, input_.size());
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

private:
    async_event_loop::TcpPeerInfo peer_{};
    std::vector<uint8_t> input_;
    std::vector<uint8_t> output_;
    bool valid_ = true;
};

class CaptureWebSocketHandler final : public async_event_loop::IWebSocketServerHandler {
public:
    void on_websocket_open(async_event_loop::ITcpConnection&, const async_event_loop::TcpPeerInfo&, const async_event_loop::websocket::HandshakeRequest& request) override {
        ++open_count;
        resource = request.resource;
    }

    void on_websocket_text(async_event_loop::ITcpConnection&, const char* text, size_t len) override {
        ++text_count;
        last_text.assign(text, len);
    }

    void on_websocket_close(async_event_loop::ITcpConnection&) override { ++close_count; }

    void on_websocket_error(async_event_loop::ITcpConnection&, async_event_loop::WebSocketError error) override {
        ++error_count;
        last_error = error;
    }

    int open_count = 0;
    int text_count = 0;
    int close_count = 0;
    int error_count = 0;
    async_event_loop::WebSocketError last_error = async_event_loop::WebSocketError::ProtocolError;
    std::string resource;
    std::string last_text;
};

static std::vector<uint8_t> masked_frame(async_event_loop::websocket::Opcode opcode, const char* text) {
    const size_t len = text ? std::strlen(text) : 0;
    REQUIRE(len < 126);
    const uint8_t mask[4] = {0x12, 0x34, 0x56, 0x78};
    std::vector<uint8_t> frame(6 + len);
    frame[0] = 0x80 | static_cast<uint8_t>(opcode);
    frame[1] = 0x80 | static_cast<uint8_t>(len);
    std::memcpy(frame.data() + 2, mask, sizeof(mask));
    for (size_t i = 0; i < len; ++i) frame[6 + i] = static_cast<uint8_t>(text[i]) ^ mask[i & 3u];
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

static void test_open_text_ping_and_close() {
    CaptureWebSocketHandler capture;
    async_event_loop::WebSocketServerHandler<2, 512, 128> server(capture, async_event_loop::TcpBackpressureOptions::disabled());
    FakeConnection connection;
    async_event_loop::TcpPeerInfo peer{};

    server.on_accept(connection, peer);
    REQUIRE(server.active_count() == 1);
    connection.push_input(valid_handshake());
    server.on_data(connection);
    REQUIRE(connection.valid());
    REQUIRE(server.is_open(connection));
    REQUIRE(capture.open_count == 1);
    REQUIRE(capture.resource == "/signalk/v1/stream?subscribe=none");
    const std::string handshake_output(reinterpret_cast<const char*>(connection.output().data()), connection.output().size());
    REQUIRE(handshake_output.find("HTTP/1.1 101 Switching Protocols\r\n") == 0);
    REQUIRE(handshake_output.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n") != std::string::npos);

    const std::vector<uint8_t> text = masked_frame(async_event_loop::websocket::Opcode::Text, "subscribe all");
    connection.push_input(text.data(), text.size());
    server.on_data(connection);
    REQUIRE(capture.text_count == 1);
    REQUIRE(capture.last_text == "subscribe all");

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
    test_bad_handshake_closes();
    return 0;
}
