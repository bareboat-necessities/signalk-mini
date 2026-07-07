#pragma once

#include <stdint.h>

#include "seatalk_frame.hpp"

namespace seatalk {

enum class SeaTalkDecodedKind : uint8_t {
    none,
    depth,
    apparent_wind_angle,
    apparent_wind_speed,
    speed_through_water,
    water_temperature,
    heading_magnetic,
    rudder_angle,
    trip_distance,
    total_distance
};

template<typename Real = float>
struct SeaTalkDecoded {
    SeaTalkDecodedKind kind = SeaTalkDecodedKind::none;
    Real value = Real{};
    Real secondary_value = Real{};
    bool secondary_valid = false;
    bool alarm = false;
    uint8_t command = 0;
};

inline float seatalk_kn_to_ms(float kn) {
    return kn * 0.514444f;
}

inline float seatalk_wrap_360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

template<typename Real = float>
bool decode_seatalk_frame(const SeaTalkFrame& frame, SeaTalkDecoded<Real>& out) {
    out = SeaTalkDecoded<Real>{};
    out.command = frame.command();
    const uint8_t* dg = frame.bytes;

    switch (frame.command()) {
    case 0x00: { // Depth: value is feet * 10 in bytes 3..4, little endian by byte order used in references.
        if (frame.length < 5) return false;
        const uint16_t raw_feet_x10 = static_cast<uint16_t>(dg[4]) << 8 | dg[3];
        const float depth_m = (static_cast<float>(raw_feet_x10) / 10.0f) * 0.3048f;
        out.kind = SeaTalkDecodedKind::depth;
        out.value = static_cast<Real>(depth_m);
        out.alarm = (dg[2] & 0x80) != 0;
        return true;
    }
    case 0x10: { // Apparent wind angle: half degrees.
        if (frame.length < 4) return false;
        const uint16_t raw_half_deg = static_cast<uint16_t>(dg[2]) << 8 | dg[3];
        out.kind = SeaTalkDecodedKind::apparent_wind_angle;
        out.value = static_cast<Real>(seatalk_wrap_360(static_cast<float>(raw_half_deg) / 2.0f));
        return true;
    }
    case 0x11: { // Apparent wind speed in knots.
        if (frame.length < 4) return false;
        const float speed_kn = static_cast<float>(dg[2] & 0x7f) + static_cast<float>(dg[3]) / 10.0f;
        out.kind = SeaTalkDecodedKind::apparent_wind_speed;
        out.value = static_cast<Real>(speed_kn);
        out.secondary_value = static_cast<Real>(seatalk_kn_to_ms(speed_kn));
        out.secondary_valid = true;
        return true;
    }
    case 0x20: { // Speed through water, knots * 10.
        if (frame.length < 4) return false;
        const uint16_t raw_kn_x10 = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        const float speed_kn = static_cast<float>(raw_kn_x10) / 10.0f;
        out.kind = SeaTalkDecodedKind::speed_through_water;
        out.value = static_cast<Real>(speed_kn);
        out.secondary_value = static_cast<Real>(seatalk_kn_to_ms(speed_kn));
        out.secondary_valid = true;
        return true;
    }
    case 0x23: { // Water temperature, direct Celsius in byte 2 in common hardware observations.
        if (frame.length < 4) return false;
        out.kind = SeaTalkDecodedKind::water_temperature;
        out.value = static_cast<Real>(static_cast<int8_t>(dg[2]));
        out.alarm = (dg[1] & 0x04) != 0;
        return true;
    }
    case 0x26: { // Log speed: knots * 100, plus optional average/second speed.
        if (frame.length < 7) return false;
        const uint16_t raw_kn_x100 = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        const uint16_t raw_avg_x100 = static_cast<uint16_t>(dg[5]) << 8 | dg[4];
        const float speed_kn = static_cast<float>(raw_kn_x100) / 100.0f;
        out.kind = SeaTalkDecodedKind::speed_through_water;
        out.value = static_cast<Real>(speed_kn);
        out.secondary_value = static_cast<Real>(static_cast<float>(raw_avg_x100) / 100.0f);
        out.secondary_valid = true;
        return true;
    }
    case 0x27: { // Water temperature, (C + 100) * 10 little-endian.
        if (frame.length < 4) return false;
        const uint16_t raw_c = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        out.kind = SeaTalkDecodedKind::water_temperature;
        out.value = static_cast<Real>((static_cast<float>(raw_c) - 100.0f) / 10.0f);
        return true;
    }
    case 0x89: { // Compass heading magnetic. Attribute high nibble carries U.
        if (frame.length < 3) return false;
        const uint8_t u = dg[1] >> 4;
        const uint8_t vw = dg[2];
        float heading = static_cast<float>(u & 0x03) * 90.0f;
        heading += static_cast<float>(vw & 0x3f) * 2.0f;
        heading += static_cast<float>((u >> 2) & 0x03) * 0.5f;
        out.kind = SeaTalkDecodedKind::heading_magnetic;
        out.value = static_cast<Real>(seatalk_wrap_360(heading));
        return true;
    }
    case 0x9c: { // Rudder datagram: heading plus signed rudder angle.
        if (frame.length < 4) return false;
        const uint8_t u = dg[1] >> 4;
        const uint8_t vw = dg[2];
        const int8_t rudder = static_cast<int8_t>(dg[3]);
        float heading = static_cast<float>(u & 0x03) * 90.0f;
        heading += static_cast<float>(vw & 0x3f) * 2.0f;
        heading += static_cast<float>((u >> 2) & 0x03) * 0.5f;
        out.kind = SeaTalkDecodedKind::rudder_angle;
        out.value = static_cast<Real>(rudder);
        out.secondary_value = static_cast<Real>(seatalk_wrap_360(heading));
        out.secondary_valid = true;
        return true;
    }
    case 0x21: { // Trip distance, nautical miles * 100.
        if (frame.length < 5) return false;
        uint32_t raw = static_cast<uint32_t>(dg[4] & 0x0f) << 16;
        raw |= static_cast<uint32_t>(dg[3]) << 8;
        raw |= dg[2];
        out.kind = SeaTalkDecodedKind::trip_distance;
        out.value = static_cast<Real>(static_cast<float>(raw) / 100.0f);
        return true;
    }
    case 0x22: { // Total log, nautical miles * 10.
        if (frame.length < 5) return false;
        uint32_t raw = static_cast<uint32_t>(dg[4]) << 16;
        raw |= static_cast<uint32_t>(dg[3]) << 8;
        raw |= dg[2];
        out.kind = SeaTalkDecodedKind::total_distance;
        out.value = static_cast<Real>(static_cast<float>(raw) / 10.0f);
        return true;
    }
    default:
        return false;
    }
}

} // namespace seatalk
