#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <async_event_loop/websocket.hpp>

#define REQUIRE(x) do { if (!(x)) { std::fprintf(stderr, "FAILED %s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } } while (0)

static void test_accept_key_vector() {
    char accept[32];
    const auto result = async_event_loop::websocket::make_accept_key("dGhlIHNhbXBsZSBub25jZQ==", accept, sizeof(accept));
    REQUIRE(result == async_event_loop::websocket::HandshakeResult::Ok);
    REQUIRE(std::strcmp(accept, "s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") == 0);
}

static void test_parse_upgrade_request() {
    const char request[] =
        "GET /signalk/v1/stream?subscribe=none HTTP/1.1\r\n"
        "Host: example.local:3000\r\n"
        "Upgrade: websocket\r\n"
        "Connection: keep-alive, Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n";

    async_event_loop::websocket::HandshakeRequest parsed;
    const auto result = async_event_loop::websocket::parse_client_handshake(request, std::strlen(request), parsed);
    REQUIRE(result == async_event_loop::websocket::HandshakeResult::Ok);
    REQUIRE(std::strcmp(parsed.resource, "/signalk/v1/stream?subscribe=none") == 0);
    REQUIRE(std::strcmp(parsed.key, "dGhlIHNhbXBsZSBub25jZQ==") == 0);
    REQUIRE(parsed.upgrade_websocket);
    REQUIRE(parsed.connection_upgrade);
    REQUIRE(parsed.version_13);
}

static void test_bad_upgrade_requests() {
    const char missing_key[] =
        "GET /signalk/v1/stream HTTP/1.1\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n";
    async_event_loop::websocket::HandshakeRequest parsed;
    REQUIRE(async_event_loop::websocket::parse_client_handshake(missing_key, std::strlen(missing_key), parsed) == async_event_loop::websocket::HandshakeResult::MissingKey);

    const char wrong_version[] =
        "GET /signalk/v1/stream HTTP/1.1\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: abc\r\n"
        "Sec-WebSocket-Version: 12\r\n"
        "\r\n";
    REQUIRE(async_event_loop::websocket::parse_client_handshake(wrong_version, std::strlen(wrong_version), parsed) == async_event_loop::websocket::HandshakeResult::UnsupportedVersion);
}

static void test_write_handshake_response() {
    char response[192];
    const auto result = async_event_loop::websocket::write_server_handshake_response("dGhlIHNhbXBsZSBub25jZQ==", response, sizeof(response));
    REQUIRE(result == async_event_loop::websocket::HandshakeResult::Ok);
    REQUIRE(std::strstr(response, "HTTP/1.1 101 Switching Protocols\r\n") == response);
    REQUIRE(std::strstr(response, "Upgrade: websocket\r\n") != nullptr);
    REQUIRE(std::strstr(response, "Connection: Upgrade\r\n") != nullptr);
    REQUIRE(std::strstr(response, "Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=\r\n") != nullptr);
    REQUIRE(std::strstr(response, "\r\n\r\n") != nullptr);
}

static void test_decode_masked_client_text() {
    const char payload[] = "subscribe all";
    const uint8_t mask[4] = {0x37, 0xfa, 0x21, 0x3d};
    uint8_t frame[64];
    frame[0] = 0x81;
    frame[1] = 0x80 | static_cast<uint8_t>(std::strlen(payload));
    std::memcpy(frame + 2, mask, sizeof(mask));
    for (size_t i = 0; i < std::strlen(payload); ++i) frame[6 + i] = static_cast<uint8_t>(payload[i]) ^ mask[i & 3u];

    uint8_t decoded[32];
    async_event_loop::websocket::FrameView view;
    size_t consumed = 0;
    const auto result = async_event_loop::websocket::decode_frame(frame, 6 + std::strlen(payload), decoded, sizeof(decoded), view, consumed, true);
    REQUIRE(result == async_event_loop::websocket::FrameDecodeResult::Ok);
    REQUIRE(consumed == 6 + std::strlen(payload));
    REQUIRE(view.fin);
    REQUIRE(view.opcode == async_event_loop::websocket::Opcode::Text);
    REQUIRE(view.payload_len == std::strlen(payload));
    REQUIRE(std::memcmp(view.payload, payload, std::strlen(payload)) == 0);
}

static void test_encode_server_text() {
    const char text[] = "{\"updates\":[]}";
    uint8_t frame[64];
    size_t len = 0;
    REQUIRE(async_event_loop::websocket::encode_text_frame(text, std::strlen(text), frame, sizeof(frame), len));
    REQUIRE(len == 2 + std::strlen(text));
    REQUIRE(frame[0] == 0x81);
    REQUIRE(frame[1] == std::strlen(text));
    REQUIRE(std::memcmp(frame + 2, text, std::strlen(text)) == 0);
}

static void test_decode_ping_and_close() {
    const uint8_t ping_mask[4] = {1, 2, 3, 4};
    const char ping_payload[] = "hi";
    uint8_t ping[8] = {0x89, static_cast<uint8_t>(0x80 | 2), ping_mask[0], ping_mask[1], ping_mask[2], ping_mask[3], 0, 0};
    ping[6] = static_cast<uint8_t>(ping_payload[0]) ^ ping_mask[0];
    ping[7] = static_cast<uint8_t>(ping_payload[1]) ^ ping_mask[1];

    uint8_t decoded[8];
    async_event_loop::websocket::FrameView view;
    size_t consumed = 0;
    REQUIRE(async_event_loop::websocket::decode_frame(ping, sizeof(ping), decoded, sizeof(decoded), view, consumed, true) == async_event_loop::websocket::FrameDecodeResult::Ok);
    REQUIRE(view.opcode == async_event_loop::websocket::Opcode::Ping);
    REQUIRE(view.payload_len == 2);
    REQUIRE(std::memcmp(view.payload, ping_payload, 2) == 0);

    const uint8_t fragmented_close[] = {0x08, 0x80, 0, 0, 0, 0};
    REQUIRE(async_event_loop::websocket::decode_frame(fragmented_close, sizeof(fragmented_close), decoded, sizeof(decoded), view, consumed, true) == async_event_loop::websocket::FrameDecodeResult::ProtocolError);
}

static void test_reject_unmasked_client_frame_and_oversized_payload() {
    const uint8_t unmasked[] = {0x81, 0x02, 'o', 'k'};
    uint8_t decoded[4];
    async_event_loop::websocket::FrameView view;
    size_t consumed = 0;
    REQUIRE(async_event_loop::websocket::decode_frame(unmasked, sizeof(unmasked), decoded, sizeof(decoded), view, consumed, true) == async_event_loop::websocket::FrameDecodeResult::ProtocolError);

    const uint8_t too_large[] = {0x81, 0x80 | 126, 0x00, 0x08, 1, 2, 3, 4, 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'};
    REQUIRE(async_event_loop::websocket::decode_frame(too_large, sizeof(too_large), decoded, sizeof(decoded), view, consumed, true) == async_event_loop::websocket::FrameDecodeResult::PayloadTooLarge);
}

int main() {
    test_accept_key_vector();
    test_parse_upgrade_request();
    test_bad_upgrade_requests();
    test_write_handshake_response();
    test_decode_masked_client_text();
    test_encode_server_text();
    test_decode_ping_and_close();
    test_reject_unmasked_client_frame_and_oversized_payload();
    return 0;
}
