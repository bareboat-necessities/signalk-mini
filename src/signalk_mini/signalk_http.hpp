#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stream_io.hpp"
#include <async_event_loop.hpp>
#include "config.hpp"
#include "model_store.hpp"
#include "signalk_full_model_writer.hpp"
#include "signalk_json_stream_writer.hpp"

namespace signalk_mini {
namespace http {

inline bool http_header_name_matches(const char* begin, size_t len, const char* expected) {
    if (!begin || !expected || len != strlen(expected)) return false;
    for (size_t i = 0; i < len; ++i) {
        char a = begin[i];
        char b = expected[i];
        if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
        if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
        if (a != b) return false;
    }
    return true;
}

inline bool copy_http_host(const char* header, size_t header_len, char* dst, size_t dst_size) {
    if (!header || !dst || dst_size == 0) return false;
    const char* cursor = header;
    const char* end = header + header_len;
    while (cursor < end) {
        const char* line_end = cursor;
        while (line_end + 1 < end && !(line_end[0] == '\r' && line_end[1] == '\n')) ++line_end;
        const char* colon = cursor;
        while (colon < line_end && *colon != ':') ++colon;
        if (colon < line_end && http_header_name_matches(cursor, static_cast<size_t>(colon - cursor), "Host")) {
            const char* value = colon + 1;
            while (value < line_end && (*value == ' ' || *value == '\t')) ++value;
            const size_t len = static_cast<size_t>(line_end - value);
            if (len == 0 || len >= dst_size) return false;
            memcpy(dst, value, len);
            dst[len] = '\0';
            return true;
        }
        if (line_end + 1 >= end) break;
        cursor = line_end + 2;
    }
    return false;
}

inline bool is_decimal_digit(char c) {
    return c >= '0' && c <= '9';
}

inline bool is_hex_digit(char c) {
    return is_decimal_digit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

inline bool valid_http_port(const char* begin, const char* end) {
    if (!begin || !end || begin >= end) return false;
    uint32_t port = 0;
    for (const char* p = begin; p < end; ++p) {
        if (!is_decimal_digit(*p)) return false;
        port = port * 10u + static_cast<uint32_t>(*p - '0');
        if (port > 65535u) return false;
    }
    return port != 0;
}

inline bool valid_http_hostname(const char* begin, const char* end) {
    if (!begin || !end || begin >= end || static_cast<size_t>(end - begin) > 253) return false;
    size_t label_len = 0;
    bool label_ends_with_hyphen = false;
    for (const char* p = begin; p < end; ++p) {
        const char c = *p;
        if (c == '.') {
            if (label_len == 0 || label_ends_with_hyphen) return false;
            label_len = 0;
            label_ends_with_hyphen = false;
            continue;
        }
        const bool alnum = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || is_decimal_digit(c);
        if (!alnum && c != '-') return false;
        if (label_len == 0 && c == '-') return false;
        if (++label_len > 63) return false;
        label_ends_with_hyphen = c == '-';
    }
    return label_len != 0 && !label_ends_with_hyphen;
}

inline bool valid_http_ipv6_literal(const char* begin, const char* end) {
    if (!begin || !end || begin >= end) return false;
    const char* p = begin;
    size_t groups = 0;
    bool compressed = false;
    if (*p == ':') {
        if (p + 1 >= end || p[1] != ':') return false;
        compressed = true;
        p += 2;
        if (p == end) return true;
    }
    while (p < end) {
        size_t digits = 0;
        while (p < end && is_hex_digit(*p) && digits < 4) {
            ++p;
            ++digits;
        }
        if (digits == 0 || (p < end && is_hex_digit(*p))) return false;
        if (++groups > 8) return false;
        if (p == end) break;
        if (*p != ':') return false;
        ++p;
        if (p == end) return false;
        if (*p == ':') {
            if (compressed) return false;
            compressed = true;
            ++p;
            if (p == end) break;
        }
    }
    return compressed ? groups < 8 : groups == 8;
}

inline bool valid_http_host(const char* host) {
    if (!host || !host[0]) return false;
    const char* end = host + strlen(host);
    if (host[0] == '[') {
        const char* close = strchr(host + 1, ']');
        if (!close || !valid_http_ipv6_literal(host + 1, close)) return false;
        if (close + 1 == end) return true;
        return close[1] == ':' && valid_http_port(close + 2, end);
    }
    if (strchr(host, '[') || strchr(host, ']')) return false;
    const char* colon = strchr(host, ':');
    if (!colon) return valid_http_hostname(host, end);
    if (strchr(colon + 1, ':')) return false;
    return valid_http_hostname(host, colon) && valid_http_port(colon + 1, end);
}

inline bool parse_http_request_line(const char* header,
                                    size_t header_len,
                                    const char*& method,
                                    size_t& method_len,
                                    const char*& target,
                                    size_t& target_len) {
    if (!header || header_len < 14) return false;
    const char* end = header + header_len;
    const char* first_space = header;
    while (first_space < end && *first_space != ' ') ++first_space;
    if (first_space == header || first_space >= end) return false;
    method = header;
    method_len = static_cast<size_t>(first_space - header);
    target = first_space + 1;
    const char* second_space = target;
    while (second_space < end && *second_space != ' ' && *second_space != '\r' && *second_space != '\n') ++second_space;
    if (second_space == target || second_space >= end || *second_space != ' ') return false;
    target_len = static_cast<size_t>(second_space - target);

    const char* version = second_space + 1;
    const char* line_end = version;
    while (line_end + 1 < end && !(line_end[0] == '\r' && line_end[1] == '\n')) ++line_end;
    if (line_end + 1 >= end) return false;
    const size_t version_len = static_cast<size_t>(line_end - version);
    return version_len == 8 &&
           (strncmp(version, "HTTP/1.1", 8) == 0 || strncmp(version, "HTTP/1.0", 8) == 0);
}

inline size_t http_path_length(const char* target, size_t target_len) {
    if (!target) return 0;
    const char* query = static_cast<const char*>(memchr(target, '?', target_len));
    return query ? static_cast<size_t>(query - target) : target_len;
}

inline bool query_parameter_equals(const char* resource, const char* name, const char* expected) {
    if (!resource || !name || !expected) return false;
    const char* query = strchr(resource, '?');
    if (!query) return false;
    ++query;
    const size_t name_len = strlen(name);
    const size_t expected_len = strlen(expected);
    while (*query) {
        const char* end = strchr(query, '&');
        if (!end) end = query + strlen(query);
        const char* equals = static_cast<const char*>(memchr(query, '=', static_cast<size_t>(end - query)));
        if (equals && static_cast<size_t>(equals - query) == name_len && strncmp(query, name, name_len) == 0) {
            const char* value = equals + 1;
            return static_cast<size_t>(end - value) == expected_len && strncmp(value, expected, expected_len) == 0;
        }
        query = *end ? end + 1 : end;
    }
    return false;
}

} // namespace http

// Read-only HTTP endpoints served on the WebSocket listener: the root page,
// Signal K discovery (/signalk) and the REST API (/signalk/v1/api/...).
template<typename Real>
class SignalKHttpApi {
public:
    SignalKHttpApi(const ModelStore<Real>& store, const SignalKMiniConfig& config)
        : store_(store), config_(config) {}

    bool handle_request(async_event_loop::ITcpConnection& connection,
                        const char* header,
                        size_t header_len,
                        uint64_t now_us) {
        const char* method = nullptr;
        const char* target = nullptr;
        size_t method_len = 0;
        size_t target_len = 0;
        if (!http::parse_http_request_line(header, header_len, method, method_len, target, target_len)) {
            return send_http_response(connection, "400 Bad Request", "application/json", "{\"message\":\"Malformed HTTP request\"}");
        }
        if (method_len != 3 || strncmp(method, "GET", 3) != 0) {
            return send_http_response(connection, "405 Method Not Allowed", "application/json",
                                      "{\"message\":\"Read-only API; GET required\"}", "Allow: GET\r\n");
        }

        const size_t path_len = http::http_path_length(target, target_len);
        if (path_len == 0 || path_len >= 512) {
            return send_http_response(connection, "414 URI Too Long", "application/json", "{\"message\":\"URI too long\"}");
        }

        if (path_len == 1 && target[0] == '/') {
            static constexpr char Body[] =
                "<!doctype html><html><head><meta charset=\"utf-8\"><title>SignalK Mini</title></head>"
                "<body><h1>SignalK Mini</h1><p>Lightweight Signal K server is running.</p>"
                "<p><a href=\"/signalk\">Signal K discovery</a></p>"
                "<p><a href=\"/signalk/v1/api/\">Signal K REST API</a></p></body></html>";
            return send_http_response(connection, "200 OK", "text/html; charset=utf-8", Body);
        }

        const bool discovery =
            (path_len == 8 && strncmp(target, "/signalk", 8) == 0) ||
            (path_len == 9 && strncmp(target, "/signalk/", 9) == 0);
        if (discovery) {
            char host[128];
            if (!http::copy_http_host(header, header_len, host, sizeof(host)) || !http::valid_http_host(host)) {
                snprintf(host, sizeof(host), "127.0.0.1:%u", static_cast<unsigned>(config_.signalk.websocket.port));
            }

            char http_url[192];
            char websocket_url[192];
            const int http_url_len = snprintf(http_url, sizeof(http_url),
                                              "http://%s/signalk/v1/api/", host);
            const int websocket_url_len = snprintf(websocket_url, sizeof(websocket_url),
                                                   "ws://%s/signalk/v1/stream", host);
            if (http_url_len <= 0 || static_cast<size_t>(http_url_len) >= sizeof(http_url) ||
                websocket_url_len <= 0 || static_cast<size_t>(websocket_url_len) >= sizeof(websocket_url)) {
                return send_http_response(connection, "500 Internal Server Error", "application/json", "{\"message\":\"Discovery serialization failed\"}");
            }

            char body[640];
            SignalKJsonStreamWriter discovery(body, sizeof(body));
            const bool body_ok = discovery.append_raw("{\"endpoints\":{\"v1\":{\"version\":") &&
                discovery.append_quoted(config_.identity.signalk_version ? config_.identity.signalk_version : "1.8.2") &&
                discovery.append_raw(",\"signalk-http\":") && discovery.append_quoted(http_url) &&
                discovery.append_raw(",\"signalk-ws\":") && discovery.append_quoted(websocket_url) &&
                discovery.append_raw("}},\"server\":{\"id\":") &&
                discovery.append_quoted(config_.identity.server_name ? config_.identity.server_name : "signalk-mini") &&
                discovery.append_raw(",\"version\":") &&
                discovery.append_quoted(config_.identity.server_version ? config_.identity.server_version : "0.1.0") &&
                discovery.append_raw("}}");
            if (!body_ok || !discovery.ok()) {
                return send_http_response(connection, "500 Internal Server Error", "application/json", "{\"message\":\"Discovery serialization failed\"}");
            }
            return send_http_response(connection, "200 OK", "application/json", body);
        }

        static constexpr char ApiBase[] = "/signalk/v1/api";
        static constexpr size_t ApiBaseLen = sizeof(ApiBase) - 1;
        if (path_len == ApiBaseLen && strncmp(target, ApiBase, ApiBaseLen) == 0) {
            return send_http_response(connection, "308 Permanent Redirect", "application/json",
                                      "{\"message\":\"Use /signalk/v1/api/\"}",
                                      "Location: /signalk/v1/api/\r\n");
        }
        if (path_len > ApiBaseLen && strncmp(target, ApiBase, ApiBaseLen) == 0 && target[ApiBaseLen] == '/') {
            const char* relative = target + ApiBaseLen + 1;
            size_t relative_len = path_len - ApiBaseLen - 1;
            while (relative_len > 0 && relative[relative_len - 1] == '/') --relative_len;
            char relative_path[384];
            if (relative_len >= sizeof(relative_path)) {
                return send_http_response(connection, "414 URI Too Long", "application/json", "{\"message\":\"Signal K path too long\"}");
            }
            memcpy(relative_path, relative, relative_len);
            relative_path[relative_len] = '\0';
            return send_signalk_rest_response(connection, relative_path, now_us);
        }

        return send_http_response(connection, "404 Not Found", "application/json", "{\"message\":\"Not Found\"}");
    }

private:
    bool send_http_response(async_event_loop::ITcpConnection& connection,
                            const char* status,
                            const char* content_type,
                            const char* body,
                            const char* extra_headers = nullptr) {
        if (!status || !content_type || !body) return false;
        const size_t body_len = strlen(body);
        char header[384];
        const int header_len = snprintf(
            header,
            sizeof(header),
            "HTTP/1.1 %s\r\n"
            "Content-Type: %s\r\n"
            "Content-Length: %u\r\n"
            "Connection: close\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "%s"
            "\r\n",
            status,
            content_type,
            static_cast<unsigned>(body_len),
            extra_headers ? extra_headers : "");
        if (header_len <= 0 || static_cast<size_t>(header_len) >= sizeof(header)) return false;
        return write_all(connection, reinterpret_cast<const uint8_t*>(header), static_cast<size_t>(header_len)) &&
               (body_len == 0 || write_all(connection, reinterpret_cast<const uint8_t*>(body), body_len));
    }

    struct SignalKJsonCountSink {
        size_t size = 0;
        static bool write(void* context, const char*, size_t len) {
            auto* self = static_cast<SignalKJsonCountSink*>(context);
            if (!self || len > SIZE_MAX - self->size) return false;
            self->size += len;
            return true;
        }
    };

    struct SignalKHttpJsonSink {
        explicit SignalKHttpJsonSink(async_event_loop::ITcpConnection& connection_ref)
            : connection(connection_ref) {}

        static bool write(void* context, const char* data, size_t len) {
            auto* self = static_cast<SignalKHttpJsonSink*>(context);
            return self && self->append(data, len);
        }

        bool append(const char* data, size_t len) {
            if (!data) return false;
            while (len > 0) {
                const size_t room = sizeof(buffer) - used;
                const size_t take = len < room ? len : room;
                memcpy(buffer + used, data, take);
                used += take;
                data += take;
                len -= take;
                if (used == sizeof(buffer) && !flush()) return false;
            }
            return true;
        }

        bool flush() {
            if (used == 0) return true;
            if (!write_all(connection, reinterpret_cast<const uint8_t*>(buffer), used)) return false;
            used = 0;
            return true;
        }

        async_event_loop::ITcpConnection& connection;
        char buffer[256];
        size_t used = 0;
    };

    bool send_signalk_rest_response(async_event_loop::ITcpConnection& connection,
                                    const char* relative_path,
                                    uint64_t now_us) {
        SignalKFullModelWriter<Real> model_writer;
        SignalKJsonCountSink count_sink;
        SignalKJsonStreamWriter count_out(&SignalKJsonCountSink::write, &count_sink);
        const SignalKModelWriteResult counted = model_writer.write_api_path(
            count_out, relative_path, store_, config_, now_us);
        if (counted == SignalKModelWriteResult::NotFound) {
            return send_http_response(connection, "404 Not Found", "application/json", "{\"message\":\"Not Found\"}");
        }
        if (counted == SignalKModelWriteResult::InvalidPath) {
            return send_http_response(connection, "400 Bad Request", "application/json", "{\"message\":\"Invalid Signal K path\"}");
        }
        if (counted != SignalKModelWriteResult::Written || !count_out.ok()) {
            return send_http_response(connection, "500 Internal Server Error", "application/json", "{\"message\":\"Serialization failed\"}");
        }

        char response_header[256];
        const int response_header_len = snprintf(
            response_header,
            sizeof(response_header),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %u\r\n"
            "Connection: close\r\n"
            "Access-Control-Allow-Origin: *\r\n"
            "\r\n",
            static_cast<unsigned>(count_sink.size));
        if (response_header_len <= 0 || static_cast<size_t>(response_header_len) >= sizeof(response_header) ||
            !write_all(connection, reinterpret_cast<const uint8_t*>(response_header), static_cast<size_t>(response_header_len))) {
            return false;
        }

        SignalKHttpJsonSink body_sink(connection);
        SignalKJsonStreamWriter body_out(&SignalKHttpJsonSink::write, &body_sink);
        const SignalKModelWriteResult written = model_writer.write_api_path(
            body_out, relative_path, store_, config_, now_us);
        return written == SignalKModelWriteResult::Written && body_out.ok() &&
               body_out.size() == count_sink.size && body_sink.flush();
    }

    const ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
};

} // namespace signalk_mini
