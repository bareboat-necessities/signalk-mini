from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise SystemExit(f'missing marker: {label}')
    return text.replace(old, new, 1)

server = Path('src/signalk_mini/server.hpp')
s = server.read_text()
old = '''        const size_t target_len = static_cast<size_t>(target_end - target);
        const bool discovery =
            (target_len == 8 && strncmp(target, "/signalk", 8) == 0) ||
            (target_len == 9 && strncmp(target, "/signalk/", 9) == 0);
        if (!discovery) return false;
'''
new = '''        const size_t target_len = static_cast<size_t>(target_end - target);

        if (target_len == 1 && target[0] == '/') {
            static constexpr char Body[] =
                "<!doctype html><html><head><meta charset=\"utf-8\"><title>SignalK Mini</title></head>"
                "<body><h1>SignalK Mini</h1><p>Lightweight Signal K server is running.</p>"
                "<p><a href=\"/signalk\">Signal K discovery</a></p></body></html>";
            char response[384];
            const int response_len = snprintf(
                response,
                sizeof(response),
                "HTTP/1.1 200 OK\\r\\n"
                "Content-Type: text/html; charset=utf-8\\r\\n"
                "Content-Length: %u\\r\\n"
                "Connection: close\\r\\n"
                "\\r\\n%s",
                static_cast<unsigned>(sizeof(Body) - 1),
                Body);
            if (response_len <= 0 || static_cast<size_t>(response_len) >= sizeof(response)) return false;
            return write_all(connection,
                             reinterpret_cast<const uint8_t*>(response),
                             static_cast<size_t>(response_len));
        }

        const bool discovery =
            (target_len == 8 && strncmp(target, "/signalk", 8) == 0) ||
            (target_len == 9 && strncmp(target, "/signalk/", 9) == 0);
        if (!discovery) return false;
'''
s = replace_once(s, old, new, 'root route')
server.write_text(s)

test = Path('tests/test_signalk_http_discovery.cpp')
s = test.read_text()
old = '''    REQUIRE(std::strcmp(doc["server"]["id"] | "", "integration-signalk-server") == 0);

    running = false;
'''
new = '''    REQUIRE(std::strcmp(doc["server"]["id"] | "", "integration-signalk-server") == 0);

    Fd root_client = connect_loopback(ws_port, 3000);
    REQUIRE(root_client.valid());
    char root_request[192];
    std::snprintf(root_request,
                  sizeof(root_request),
                  "GET / HTTP/1.1\\r\\nHost: 127.0.0.1:%u\\r\\nConnection: close\\r\\n\\r\\n",
                  static_cast<unsigned>(ws_port));
    REQUIRE(send_all(root_client.get(), root_request));

    std::string root_response;
    REQUIRE(read_http_response(root_client.get(), root_response, 3000));
    REQUIRE(root_response.find("HTTP/1.1 200 OK\\r\\n") == 0);
    REQUIRE(root_response.find("Content-Type: text/html; charset=utf-8\\r\\n") != std::string::npos);
    REQUIRE(root_response.find("<h1>SignalK Mini</h1>") != std::string::npos);
    REQUIRE(root_response.find("href=\\\"/signalk\\\"") != std::string::npos);

    running = false;
'''
s = replace_once(s, old, new, 'root route test')
test.write_text(s)
