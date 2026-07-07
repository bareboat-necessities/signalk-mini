#pragma once

#include <stddef.h>
#include <stdint.h>

#include "seatalk/seatalk_frame.hpp"

namespace seatalk {

enum class SeaTalkPilotKey : uint8_t {
    auto_mode = 0x01,
    standby = 0x02,
    track = 0x03,
    display = 0x04,
    minus_1 = 0x05,
    minus_10 = 0x06,
    plus_1 = 0x07,
    plus_10 = 0x08,
    minus_response = 0x09,
    plus_response = 0x0a,
    minus_1_plus_1 = 0x20,
    minus_1_plus_10 = 0x21,
    plus_1_plus_10 = 0x22,
    standby_auto = 0x23,
    display_track = 0x24,
    standby_minus_10 = 0x25,
    minus_10_plus_10 = 0x28,
    minus_response_plus_response = 0x2e,
    standby_minus_1 = 0x30,
};

inline float seatalk_tx_abs(float value) {
    return value < 0.0f ? -value : value;
}

inline float seatalk_tx_wrap_360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

inline uint16_t seatalk_tx_round_u16(float value) {
    if (value <= 0.0f) return 0;
    if (value >= 65535.0f) return 65535u;
    return static_cast<uint16_t>(value + 0.5f);
}

inline uint8_t seatalk_tx_round_u8(float value) {
    if (value <= 0.0f) return 0;
    if (value >= 255.0f) return 255u;
    return static_cast<uint8_t>(value + 0.5f);
}

inline bool seatalk_tx_set(SeaTalkFrame& frame, const uint8_t* bytes, uint8_t length) {
    if (!bytes || length < 3 || length > SEATALK_FRAME_MAX_BYTES) return false;
    if (expected_seatalk_frame_length(bytes[1]) != length) return false;
    frame = SeaTalkFrame{};
    for (uint8_t i = 0; i < length; ++i) frame.bytes[i] = bytes[i];
    frame.length = length;
    return true;
}

inline bool make_depth_m(SeaTalkFrame& frame, float depth_m, bool alarm = false) {
    const uint16_t raw_ft_x10 = seatalk_tx_round_u16(depth_m / 0.3048f * 10.0f);
    const uint8_t bytes[] = {
        0x00,
        0x02,
        static_cast<uint8_t>(alarm ? 0x80 : 0x00),
        static_cast<uint8_t>(raw_ft_x10 & 0xff),
        static_cast<uint8_t>((raw_ft_x10 >> 8) & 0xff),
    };
    return seatalk_tx_set(frame, bytes, sizeof(bytes));
}

inline bool make_apparent_wind_angle_deg(SeaTalkFrame& frame, float angle_deg) {
    const uint16_t raw_half_deg = seatalk_tx_round_u16(seatalk_tx_wrap_360(angle_deg) * 2.0f);
    const uint8_t bytes[] = {
        0x10,
        0x01,
        static_cast<uint8_t>((raw_half_deg >> 8) & 0xff),
        static_cast<uint8_t>(raw_half_deg & 0xff),
    };
    return seatalk_tx_set(frame, bytes, sizeof(bytes));
}

inline bool make_apparent_wind_speed_kn(SeaTalkFrame& frame, float speed_kn) {
    if (speed_kn < 0.0f) speed_kn = 0.0f;
    uint8_t whole = static_cast<uint8_t>(speed_kn);
    uint8_t tenth = seatalk_tx_round_u8((speed_kn - static_cast<float>(whole)) * 10.0f);
    if (tenth >= 10u) {
        ++whole;
        tenth = 0;
    }
    if (whole > 127u) whole = 127u;
    const uint8_t bytes[] = {0x11, 0x01, whole, tenth};
    return seatalk_tx_set(frame, bytes, sizeof(bytes));
}

inline bool make_speed_through_water_kn(SeaTalkFrame& frame, float speed_kn) {
    const uint16_t raw_kn_x10 = seatalk_tx_round_u16(speed_kn * 10.0f);
    const uint8_t bytes[] = {
        0x20,
        0x01,
        static_cast<uint8_t>(raw_kn_x10 & 0xff),
        static_cast<uint8_t>((raw_kn_x10 >> 8) & 0xff),
    };
    return seatalk_tx_set(frame, bytes, sizeof(bytes));
}

inline bool make_pilot_key(SeaTalkFrame& frame, SeaTalkPilotKey key, bool long_press = false) {
    uint8_t code = static_cast<uint8_t>(key) & 0x3fu;
    if (long_press) code |= 0x40u;
    const uint8_t bytes[] = {0x86, 0x01, code, static_cast<uint8_t>(0xffu - code)};
    return seatalk_tx_set(frame, bytes, sizeof(bytes));
}

inline bool make_pilot_auto(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::auto_mode); }
inline bool make_pilot_standby(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::standby); }
inline bool make_pilot_track(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::track); }
inline bool make_pilot_plus_1(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::plus_1); }
inline bool make_pilot_minus_1(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::minus_1); }
inline bool make_pilot_plus_10(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::plus_10); }
inline bool make_pilot_minus_10(SeaTalkFrame& frame) { return make_pilot_key(frame, SeaTalkPilotKey::minus_10); }

} // namespace seatalk
