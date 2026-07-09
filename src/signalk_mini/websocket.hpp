#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace signalk_mini::websocket {

static constexpr size_t DefaultHandshakeBufferSize = 512;
static constexpr size_t DefaultMaxPayloadSize = 512;
static constexpr char WebSocketGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

enum class HandshakeResult : uint8_t {
    Ok,
    BadRequest,
    MissingKey,
    UnsupportedVersion,
    OutputTooSmall,
};

struct HandshakeRequest {
    char resource[96]{};
    char key[64]{};
    bool upgrade_websocket = false;
    bool connection_upgrade = false;
    bool version_13 = false;
};

enum class Opcode : uint8_t {
    Continuation = 0x0,
    Text = 0x1,
    Binary = 0x2,
    Close = 0x8,
    Ping = 0x9,
    Pong = 0xA,
};

enum class FrameDecodeResult : uint8_t {
    Ok,
    NeedMore,
    ProtocolError,
    PayloadTooLarge,
};

struct FrameView {
    bool fin = false;
    Opcode opcode = Opcode::Continuation;
    const uint8_t* payload = nullptr;
    size_t payload_len = 0;
};

inline bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }
inline char lower_ascii(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c + ('a' - 'A')) : c; }

inline bool ascii_iequals(const char* a, size_t a_len, const char* b) {
    if (!a || !b) return false;
    const size_t b_len = strlen(b);
    if (a_len != b_len) return false;
    for (size_t i = 0; i < a_len; ++i) {
        if (lower_ascii(a[i]) != lower_ascii(b[i])) return false;
    }
    return true;
}

inline bool ascii_icontains_token(const char* value, size_t len, const char* token) {
    if (!value || !token || !token[0]) return false;
    const size_t token_len = strlen(token);
    size_t i = 0;
    while (i < len) {
        while (i < len && (value[i] == ',' || is_space(value[i]))) ++i;
        const size_t begin = i;
        while (i < len && value[i] != ',') ++i;
        size_t end = i;
        while (end > begin && is_space(value[end - 1])) --end;
        size_t token_begin = begin;
        while (token_begin < end && is_space(value[token_begin])) ++token_begin;
        if (end >= token_begin && end - token_begin == token_len && ascii_iequals(value + token_begin, token_len, token)) return true;
        if (i < len && value[i] == ',') ++i;
    }
    return false;
}

inline void copy_trimmed(char* dst, size_t dst_size, const char* src, size_t len) {
    if (!dst || dst_size == 0) return;
    dst[0] = '\0';
    if (!src) return;
    while (len > 0 && is_space(*src)) { ++src; --len; }
    while (len > 0 && is_space(src[len - 1])) --len;
    const size_t copy_len = len < dst_size ? len : dst_size - 1;
    memcpy(dst, src, copy_len);
    dst[copy_len] = '\0';
}

inline bool parse_request_line(const char* line, size_t len, HandshakeRequest& out) {
    const char* end = line + len;
    const char* p = line;
    const char* method = p;
    while (p < end && *p != ' ') ++p;
    if (!ascii_iequals(method, static_cast<size_t>(p - method), "GET")) return false;
    while (p < end && *p == ' ') ++p;
    const char* resource = p;
    while (p < end && *p != ' ') ++p;
    if (resource == p) return false;
    copy_trimmed(out.resource, sizeof(out.resource), resource, static_cast<size_t>(p - resource));
    while (p < end && *p == ' ') ++p;
    return static_cast<size_t>(end - p) >= 8 && memcmp(p, "HTTP/1.", 7) == 0;
}

inline HandshakeResult parse_client_handshake(const char* request, size_t len, HandshakeRequest& out) {
    out = HandshakeRequest{};
    if (!request || len == 0) return HandshakeResult::BadRequest;

    const char* begin = request;
    const char* end = request + len;
    const char* line_end = static_cast<const char*>(memchr(begin, '\n', static_cast<size_t>(end - begin)));
    if (!line_end) return HandshakeResult::BadRequest;
    size_t first_len = static_cast<size_t>(line_end - begin);
    if (first_len > 0 && begin[first_len - 1] == '\r') --first_len;
    if (!parse_request_line(begin, first_len, out)) return HandshakeResult::BadRequest;
    begin = line_end + 1;

    while (begin < end) {
        line_end = static_cast<const char*>(memchr(begin, '\n', static_cast<size_t>(end - begin)));
        if (!line_end) line_end = end;
        size_t line_len = static_cast<size_t>(line_end - begin);
        if (line_len > 0 && begin[line_len - 1] == '\r') --line_len;
        if (line_len == 0) break;

        const char* colon = static_cast<const char*>(memchr(begin, ':', line_len));
        if (colon) {
            const char* name = begin;
            size_t name_len = static_cast<size_t>(colon - begin);
            while (name_len > 0 && is_space(name[name_len - 1])) --name_len;
            const char* value = colon + 1;
            size_t value_len = static_cast<size_t>(begin + line_len - value);
            while (value_len > 0 && is_space(*value)) { ++value; --value_len; }
            while (value_len > 0 && is_space(value[value_len - 1])) --value_len;

            if (ascii_iequals(name, name_len, "Upgrade")) {
                out.upgrade_websocket = ascii_iequals(value, value_len, "websocket");
            } else if (ascii_iequals(name, name_len, "Connection")) {
                out.connection_upgrade = ascii_icontains_token(value, value_len, "Upgrade");
            } else if (ascii_iequals(name, name_len, "Sec-WebSocket-Key")) {
                copy_trimmed(out.key, sizeof(out.key), value, value_len);
            } else if (ascii_iequals(name, name_len, "Sec-WebSocket-Version")) {
                out.version_13 = ascii_iequals(value, value_len, "13");
            }
        }
        begin = line_end < end ? line_end + 1 : end;
    }

    if (!out.upgrade_websocket || !out.connection_upgrade) return HandshakeResult::BadRequest;
    if (!out.key[0]) return HandshakeResult::MissingKey;
    if (!out.version_13) return HandshakeResult::UnsupportedVersion;
    return HandshakeResult::Ok;
}

inline uint32_t sha1_rotl(uint32_t v, uint8_t bits) { return (v << bits) | (v >> (32u - bits)); }

inline void sha1_block(const uint8_t block[64], uint32_t h[5]) {
    uint32_t w[80];
    for (uint8_t i = 0; i < 16; ++i) {
        w[i] = (static_cast<uint32_t>(block[i * 4]) << 24) |
               (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
               (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
               static_cast<uint32_t>(block[i * 4 + 3]);
    }
    for (uint8_t i = 16; i < 80; ++i) w[i] = sha1_rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (uint8_t i = 0; i < 80; ++i) {
        uint32_t f = 0;
        uint32_t k = 0;
        if (i < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999u; }
        else if (i < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1u; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDCu; }
        else { f = b ^ c ^ d; k = 0xCA62C1D6u; }
        const uint32_t temp = sha1_rotl(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = sha1_rotl(b, 30);
        b = a;
        a = temp;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
}

inline void sha1(const uint8_t* data, size_t len, uint8_t digest[20]) {
    uint32_t h[5] = {0x67452301u, 0xEFCDAB89u, 0x98BADCFEu, 0x10325476u, 0xC3D2E1F0u};
    uint8_t block[64]{};
    const uint64_t bit_len = static_cast<uint64_t>(len) * 8u;

    while (len >= 64) {
        sha1_block(data, h);
        data += 64;
        len -= 64;
    }

    memset(block, 0, sizeof(block));
    if (len > 0) memcpy(block, data, len);
    block[len] = 0x80u;
    if (len >= 56) {
        sha1_block(block, h);
        memset(block, 0, sizeof(block));
    }
    for (uint8_t i = 0; i < 8; ++i) block[56 + i] = static_cast<uint8_t>((bit_len >> (56u - 8u * i)) & 0xFFu);
    sha1_block(block, h);

    for (uint8_t i = 0; i < 5; ++i) {
        digest[i * 4] = static_cast<uint8_t>((h[i] >> 24) & 0xFFu);
        digest[i * 4 + 1] = static_cast<uint8_t>((h[i] >> 16) & 0xFFu);
        digest[i * 4 + 2] = static_cast<uint8_t>((h[i] >> 8) & 0xFFu);
        digest[i * 4 + 3] = static_cast<uint8_t>(h[i] & 0xFFu);
    }
}

inline size_t base64_encoded_size(size_t len) { return ((len + 2u) / 3u) * 4u; }

inline bool base64_encode(const uint8_t* data, size_t len, char* out, size_t out_size) {
    static constexpr char table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    if (!out || out_size == 0) return false;
    const size_t encoded = base64_encoded_size(len);
    if (encoded + 1 > out_size) return false;
    size_t pos = 0;
    for (size_t i = 0; i < len; i += 3) {
        const uint32_t b0 = data[i];
        const uint32_t b1 = i + 1 < len ? data[i + 1] : 0;
        const uint32_t b2 = i + 2 < len ? data[i + 2] : 0;
        const uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
        out[pos++] = table[(triple >> 18) & 0x3Fu];
        out[pos++] = table[(triple >> 12) & 0x3Fu];
        out[pos++] = i + 1 < len ? table[(triple >> 6) & 0x3Fu] : '=';
        out[pos++] = i + 2 < len ? table[triple & 0x3Fu] : '=';
    }
    out[pos] = '\0';
    return true;
}

inline HandshakeResult make_accept_key(const char* client_key, char* out, size_t out_size) {
    if (!client_key || !client_key[0]) return HandshakeResult::MissingKey;
    const size_t key_len = strlen(client_key);
    const size_t guid_len = strlen(WebSocketGuid);
    char combined[128];
    if (key_len + guid_len >= sizeof(combined)) return HandshakeResult::BadRequest;
    memcpy(combined, client_key, key_len);
    memcpy(combined + key_len, WebSocketGuid, guid_len);
    const size_t combined_len = key_len + guid_len;

    uint8_t digest[20];
    sha1(reinterpret_cast<const uint8_t*>(combined), combined_len, digest);
    return base64_encode(digest, sizeof(digest), out, out_size) ? HandshakeResult::Ok : HandshakeResult::OutputTooSmall;
}

inline HandshakeResult write_server_handshake_response(const char* client_key, char* out, size_t out_size) {
    if (!out || out_size == 0) return HandshakeResult::OutputTooSmall;
    char accept[32];
    const HandshakeResult key_result = make_accept_key(client_key, accept, sizeof(accept));
    if (key_result != HandshakeResult::Ok) return key_result;
    static constexpr char prefix[] = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ";
    static constexpr char suffix[] = "\r\n\r\n";
    const size_t need = strlen(prefix) + strlen(accept) + strlen(suffix) + 1;
    if (need > out_size) return HandshakeResult::OutputTooSmall;
    memcpy(out, prefix, strlen(prefix));
    memcpy(out + strlen(prefix), accept, strlen(accept));
    memcpy(out + strlen(prefix) + strlen(accept), suffix, strlen(suffix) + 1);
    return HandshakeResult::Ok;
}

inline bool valid_control_payload(Opcode opcode, size_t payload_len) {
    return (opcode == Opcode::Close || opcode == Opcode::Ping || opcode == Opcode::Pong) && payload_len <= 125;
}

inline FrameDecodeResult decode_frame(const uint8_t* data,
                                      size_t len,
                                      uint8_t* payload_dst,
                                      size_t payload_capacity,
                                      FrameView& out,
                                      size_t& consumed,
                                      bool require_mask = true) {
    out = FrameView{};
    consumed = 0;
    if (!data || len < 2) return FrameDecodeResult::NeedMore;

    const uint8_t b0 = data[0];
    const uint8_t b1 = data[1];
    const bool fin = (b0 & 0x80u) != 0;
    const bool rsv = (b0 & 0x70u) != 0;
    const Opcode opcode = static_cast<Opcode>(b0 & 0x0Fu);
    const bool masked = (b1 & 0x80u) != 0;
    uint64_t payload_len = b1 & 0x7Fu;
    size_t offset = 2;

    if (rsv) return FrameDecodeResult::ProtocolError;
    if (require_mask && !masked) return FrameDecodeResult::ProtocolError;
    if (!require_mask && masked) return FrameDecodeResult::ProtocolError;

    if (payload_len == 126) {
        if (len < offset + 2) return FrameDecodeResult::NeedMore;
        payload_len = (static_cast<uint16_t>(data[offset]) << 8) | data[offset + 1];
        offset += 2;
    } else if (payload_len == 127) {
        return FrameDecodeResult::PayloadTooLarge;
    }

    if (static_cast<uint8_t>(opcode) >= 0x8u) {
        if (!fin || !valid_control_payload(opcode, static_cast<size_t>(payload_len))) return FrameDecodeResult::ProtocolError;
    } else if (opcode != Opcode::Text && opcode != Opcode::Binary && opcode != Opcode::Continuation) {
        return FrameDecodeResult::ProtocolError;
    }

    uint8_t mask[4]{};
    if (masked) {
        if (len < offset + 4) return FrameDecodeResult::NeedMore;
        memcpy(mask, data + offset, sizeof(mask));
        offset += 4;
    }

    if (payload_len > payload_capacity) return FrameDecodeResult::PayloadTooLarge;
    if (len < offset + static_cast<size_t>(payload_len)) return FrameDecodeResult::NeedMore;
    if (payload_len > 0 && !payload_dst) return FrameDecodeResult::ProtocolError;

    for (size_t i = 0; i < static_cast<size_t>(payload_len); ++i) {
        const uint8_t byte = data[offset + i];
        payload_dst[i] = masked ? static_cast<uint8_t>(byte ^ mask[i & 3u]) : byte;
    }

    out.fin = fin;
    out.opcode = opcode;
    out.payload = payload_dst;
    out.payload_len = static_cast<size_t>(payload_len);
    consumed = offset + static_cast<size_t>(payload_len);
    return FrameDecodeResult::Ok;
}

inline bool encode_text_frame(const char* text, size_t text_len, uint8_t* dst, size_t dst_capacity, size_t& out_len) {
    out_len = 0;
    if (!dst) return false;
    size_t header_len = 0;
    if (text_len < 126) header_len = 2;
    else if (text_len <= 65535u) header_len = 4;
    else return false;
    if (dst_capacity < header_len + text_len) return false;

    dst[0] = 0x80u | static_cast<uint8_t>(Opcode::Text);
    if (text_len < 126) {
        dst[1] = static_cast<uint8_t>(text_len);
        out_len = 2;
    } else {
        dst[1] = 126;
        dst[2] = static_cast<uint8_t>((text_len >> 8) & 0xFFu);
        dst[3] = static_cast<uint8_t>(text_len & 0xFFu);
        out_len = 4;
    }
    if (text_len > 0) {
        if (!text) return false;
        memcpy(dst + out_len, text, text_len);
    }
    out_len += text_len;
    return true;
}

} // namespace signalk_mini::websocket
