#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(ARDUINO) && !defined(ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP)
#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#endif

#include <async_event_loop.hpp>

namespace signalk_mini {

inline bool write_all(async_event_loop::IByteStream& stream, const uint8_t* data, size_t length) {
    if (!data || length == 0 || !stream.valid() || !stream.writable()) return false;
    size_t offset = 0;
    while (offset < length) {
        const int n = stream.write(data + offset, length - offset);
        if (n <= 0 || static_cast<size_t>(n) > length - offset) return false;
        offset += static_cast<size_t>(n);
    }
    return true;
}

inline bool write_all(async_event_loop::IByteStream& stream, const char* text, size_t length) {
    return write_all(stream, reinterpret_cast<const uint8_t*>(text), length);
}

inline bool listen_tcp(async_event_loop::NativeTcpServer& server,
                       async_event_loop::ITcpServerHandler& handler,
                       const char* host,
                       uint16_t port) {
    async_event_loop::TcpListenOptions options;
    options.host = host;
    options.port = port;
    options.reuse_address = true;
    return server.listen(options, handler);
}

} // namespace signalk_mini
