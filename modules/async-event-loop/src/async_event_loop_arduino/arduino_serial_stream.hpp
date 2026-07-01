#pragma once

#include <Arduino.h>
#include "async_event_loop/event_loop.hpp"

namespace async_event_loop {

class ArduinoSerialStream final : public IByteStream {
public:
    explicit ArduinoSerialStream(Stream& stream) : stream_(stream) {}

    int read(uint8_t* dst, size_t max_len) override {
        if (!dst || max_len == 0) {
            return 0;
        }
        size_t n = 0;
        while (n < max_len && stream_.available() > 0) {
            const int c = stream_.read();
            if (c < 0) {
                break;
            }
            dst[n++] = static_cast<uint8_t>(c);
        }
        return static_cast<int>(n);
    }

    int write(const uint8_t* src, size_t len) override {
        if (!src || len == 0) {
            return 0;
        }
        return static_cast<int>(stream_.write(src, len));
    }

    bool readable() const override { return stream_.available() > 0; }
    bool writable() const override { return true; }
    bool valid() const override { return true; }

private:
    Stream& stream_;
};

} // namespace async_event_loop
