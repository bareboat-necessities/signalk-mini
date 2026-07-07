#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "seatalk_frame.hpp"

namespace seatalk {

inline float seatalk_tx_clamp_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

inline float seatalk_tx_wrap_360(float degrees) {
    while (degrees < 0.0f) degrees += 360.0f;
    while (degrees >= 360.0f) degrees -= 360.0f;
    return degrees;
}

inline uint16_t seatalk_tx_round_u16(float value) {
    if (value <= 0.0f) return 0;
    if (value >= 65535.0f) return 65535u;
    return static_cast<uint16_t>(lroundf(value));
}

inline bool seatalk_make_frame(const uint8_t* bytes, size_t length, SeaTalkFrame& out) {
    return seatalk_frame_from_bytes(bytes, length, out);
}

inline bool seatalk_write_frame_bytes(const SeaTalkFrame& frame, uint8_t* dst, size_t dst_size, size_t& written) {
    written = 0;
    if (!dst || frame.length == 0 || frame.length > dst_size) return false;
    for (uint8_t i = 0; i < frame.length; ++i) dst[i] = frame.bytes[i];
    written = frame.length;
    return true;
}

inline bool encode_depth_m(float depth_m, bool shallow_alarm, SeaTalkFrame& out) {
    const float feet_x10 = seatalk_tx_clamp_float(depth_m, 0.0f, 19999.9f) / 0.3048f * 10.0f;
    const uint16_t raw = seatalk_tx_round_u16(feet_x10);
    const uint8_t bytes[] = {
        0x00,
        0x02,
        static_cast<uint8_t>(shallow_alarm ? 0x80 : 0x00),
        static_cast<uint8_t>(raw & 0xff),
        static_cast<uint8_t>((raw >> 8) & 0xff),
    };
    return seatalk_make_frame(bytes, sizeof(bytes), out);
}

inline bool encode_apparent_wind_angle_deg(float angle_deg, SeaTalkFrame& out) {
    const uint16_t raw = seatalk_tx_round_u16(seatalk_tx_wrap_360(angle_deg) * 2.0f);
    const uint8_t bytes[] = {
        0x10,
        0x01,
        static_cast<uint8_t>((raw >> 8) & 0xff),
        static_cast<uint8_t>(raw & 0xff),
    };
    return seatalk_make_frame(bytes, sizeof(bytes), out);
}

inline bool encode_apparent_wind_speed_kn(float speed_kn, SeaTalkFrame& out) {
    const float clamped = seatalk_tx_clamp_float(speed_kn, 0.0f, 127.9f);
    uint8_t whole = static_cast<uint8_t>(floorf(clamped));
    uint8_t tenths = static_cast<uint8_t>(lroundf((clamped - static_cast<float>(whole)) * 10.0f));
    if (tenths > 9) {
        tenths = 0;
        if (whole < 127) ++whole;
    }
    const uint8_t bytes[] = {0x11, 0x01, whole, tenths};
    return seatalk_make_frame(bytes, sizeof(bytes), out);
}

inline bool encode_speed_through_water_kn(float speed_kn, SeaTalkFrame& out) {
    const uint16_t raw = seatalk_tx_round_u16(seatalk_tx_clamp_float(speed_kn, 0.0f, 6553.5f) * 10.0f);
    const uint8_t bytes[] = {0x20, 0x01, static_cast<uint8_t>(raw & 0xff), static_cast<uint8_t>((raw >> 8) & 0xff)};
    return seatalk_make_frame(bytes, sizeof(bytes), out);
}

inline bool encode_speed_over_ground_kn(float speed_kn, SeaTalkFrame& out) {
    const uint16_t raw = seatalk_tx_round_u16(seatalk_tx_clamp_float(speed_kn, 0.0f, 6553.5f) * 10.0f);
    const uint8_t bytes[] = {0x52, 0x01, static_cast<uint8_t>(raw & 0xff), static_cast<uint8_t>((raw >> 8) & 0xff)};
    return seatalk_make_frame(bytes, sizeof(bytes), out);
}

inline bool encode_control_key(uint8_t key_code, bool long_press, SeaTalkFrame& out) {
    const uint8_t key = static_cast<uint8_t>((key_code & 0x3f) | (long_press ? 0x40 : 0x00));
    const uint8_t bytes[] = {0x86, 0x01, key, static_cast<uint8_t>(0xffu - key)};
    return seatalk_make_frame(bytes, sizeof(bytes), out);
}

inline bool encode_control_standby(SeaTalkFrame& out) { return encode_control_key(0x02, false, out); }
inline bool encode_control_auto(SeaTalkFrame& out) { return encode_control_key(0x01, false, out); }
inline bool encode_control_track(SeaTalkFrame& out) { return encode_control_key(0x03, false, out); }

} // namespace seatalk
