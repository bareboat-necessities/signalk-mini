#pragma once

#include <stddef.h>
#include <stdint.h>

namespace seatalk {

static const size_t SEATALK_FRAME_MAX_BYTES = 20;

struct SeaTalkFrame {
    uint8_t bytes[SEATALK_FRAME_MAX_BYTES] = {0};
    uint8_t length = 0;

    uint8_t command() const { return length > 0 ? bytes[0] : 0; }
    uint8_t attribute() const { return length > 1 ? bytes[1] : 0; }
    uint8_t data_length() const { return length >= 3 ? static_cast<uint8_t>(length - 2) : 0; }
};

inline uint8_t expected_seatalk_frame_length(uint8_t attribute) {
    // SeaTalk1 datagram byte 1 low nibble is N, followed by N+1 data bytes.
    // Total byte count is command + attribute + N+1 = N+3.
    return static_cast<uint8_t>((attribute & 0x0f) + 3);
}

inline bool seatalk_frame_from_bytes(const uint8_t* bytes, size_t length, SeaTalkFrame& out) {
    if (!bytes || length < 3 || length > SEATALK_FRAME_MAX_BYTES) return false;
    const uint8_t expected = expected_seatalk_frame_length(bytes[1]);
    if (length != expected) return false;
    out = SeaTalkFrame{};
    for (size_t i = 0; i < length; ++i) out.bytes[i] = bytes[i];
    out.length = static_cast<uint8_t>(length);
    return true;
}

struct SeaTalkRxState {
    uint8_t buffer[SEATALK_FRAME_MAX_BYTES] = {0};
    uint8_t length = 0;
    uint8_t expected_length = 0;
    uint32_t bad_frame_count = 0;

    void reset() {
        length = 0;
        expected_length = 0;
    }

    bool feed_byte(uint8_t byte, SeaTalkFrame& frame_out) {
        if (length >= SEATALK_FRAME_MAX_BYTES) {
            ++bad_frame_count;
            reset();
        }

        buffer[length++] = byte;
        if (length == 2) {
            expected_length = expected_seatalk_frame_length(buffer[1]);
            if (expected_length < 3 || expected_length > SEATALK_FRAME_MAX_BYTES) {
                ++bad_frame_count;
                reset();
                return false;
            }
        }

        if (expected_length != 0 && length == expected_length) {
            const bool ok = seatalk_frame_from_bytes(buffer, length, frame_out);
            if (!ok) ++bad_frame_count;
            reset();
            return ok;
        }

        return false;
    }
};

} // namespace seatalk
