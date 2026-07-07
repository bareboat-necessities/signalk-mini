#pragma once

#include <stdint.h>
#include <string.h>

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
    total_distance,
    autopilot_state,
    autopilot_key,
    navigation_to_waypoint,
    compass_variation
};

enum class SeaTalkAutopilotMode : uint8_t {
    standby,
    auto_heading,
    vane,
    track
};

template<typename Real = float>
struct SeaTalkDecoded {
    SeaTalkDecodedKind kind = SeaTalkDecodedKind::none;
    Real value = Real{};
    Real secondary_value = Real{};
    Real third_value = Real{};
    bool secondary_valid = false;
    bool third_valid = false;
    bool alarm = false;
    uint8_t command = 0;
    uint8_t flags = 0;
    uint8_t code = 0;
    bool long_press = false;
    SeaTalkAutopilotMode autopilot_mode = SeaTalkAutopilotMode::standby;
    char label[32] = {0};
};

inline float seatalk_kn_to_ms(float kn) {
    return kn * 0.514444f;
}

inline float seatalk_wrap_360(float deg) {
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

inline void seatalk_copy_label(char* dst, size_t dst_len, const char* text) {
    if (!dst || dst_len == 0) return;
    dst[0] = '\0';
    if (!text) return;
    strncpy(dst, text, dst_len - 1);
    dst[dst_len - 1] = '\0';
}

inline const char* seatalk_autopilot_mode_label(SeaTalkAutopilotMode mode) {
    switch (mode) {
    case SeaTalkAutopilotMode::standby: return "standby";
    case SeaTalkAutopilotMode::auto_heading: return "auto";
    case SeaTalkAutopilotMode::vane: return "vane";
    case SeaTalkAutopilotMode::track: return "track";
    }
    return "standby";
}

inline const char* seatalk_autopilot_key_label(uint8_t code, bool long_press) {
    switch (code) {
    case 0x01: return "auto";
    case 0x02: return "standby";
    case 0x03: return "track";
    case 0x04: return "display";
    case 0x05: return "minus_1";
    case 0x06: return "minus_10";
    case 0x07: return "plus_1";
    case 0x08: return "plus_10";
    case 0x09: return "minus_response";
    case 0x0a: return "plus_response";
    case 0x20: return "minus_1_plus_1";
    case 0x21: return "minus_1_plus_10";
    case 0x22: return "plus_1_plus_10";
    case 0x23: return "standby_auto";
    case 0x24: return long_press ? "minus_10_plus_10" : "display_track";
    case 0x25: return "standby_minus_10";
    case 0x28: return long_press ? "display_track" : "minus_10_plus_10";
    case 0x2e: return "minus_response_plus_response";
    case 0x30: return "standby_minus_1";
    default: return "unknown";
    }
}

inline float seatalk_heading_from_u_vw(uint8_t u, uint8_t vw) {
    float heading = static_cast<float>(u & 0x03) * 90.0f;
    heading += static_cast<float>(vw & 0x3f) * 2.0f;
    heading += static_cast<float>((u >> 2) & 0x03) * 0.5f;
    return seatalk_wrap_360(heading);
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
    case 0x84: { // Autopilot CPU state: heading, commanded course, mode, alarm, rudder.
        if (frame.length < 9) return false;
        const uint8_t u = dg[1] >> 4;
        const uint8_t vw = dg[2];
        const uint8_t xy = dg[3];
        const uint8_t z = dg[4];
        const uint8_t alarm_flags = dg[5];
        const int8_t rudder = static_cast<int8_t>(dg[6]);
        const uint8_t status_flags = dg[7];
        const uint8_t v = vw >> 4;
        const float heading = seatalk_heading_from_u_vw(u, vw);
        const float course = static_cast<float>(v >> 2) * 90.0f + static_cast<float>(xy) / 2.0f;
        out.kind = SeaTalkDecodedKind::autopilot_state;
        out.value = static_cast<Real>(heading);
        out.secondary_value = static_cast<Real>(seatalk_wrap_360(course));
        out.secondary_valid = true;
        out.third_value = static_cast<Real>(rudder);
        out.third_valid = true;
        out.flags = status_flags;
        out.code = alarm_flags;
        out.alarm = alarm_flags != 0;
        if (z & 0x08) out.autopilot_mode = SeaTalkAutopilotMode::track;
        else if (z & 0x04) out.autopilot_mode = SeaTalkAutopilotMode::vane;
        else if (z & 0x02) out.autopilot_mode = SeaTalkAutopilotMode::auto_heading;
        else out.autopilot_mode = SeaTalkAutopilotMode::standby;
        seatalk_copy_label(out.label, sizeof(out.label), seatalk_autopilot_mode_label(out.autopilot_mode));
        return true;
    }
    case 0x85: { // Navigation to waypoint: XTE, bearing, range.
        if (frame.length < 8) return false;
        const uint16_t xte_raw = (static_cast<uint16_t>(dg[2]) << 4) | (dg[1] >> 4);
        const uint8_t vu = dg[3];
        const uint8_t zw = dg[4];
        const uint8_t zz = dg[5];
        const uint8_t yf = dg[6];
        const uint8_t wv = static_cast<uint8_t>((zw << 4) | (vu >> 4));
        const float bearing = seatalk_wrap_360(static_cast<float>(vu & 0x03) * 90.0f + static_cast<float>(wv) / 2.0f);
        const uint8_t y = yf >> 4;
        const uint16_t range_raw = (static_cast<uint16_t>(zz) << 4) | (zw >> 4);
        const float range_nmi = static_cast<float>(range_raw) / ((y & 0x01) ? 100.0f : 10.0f);
        out.kind = SeaTalkDecodedKind::navigation_to_waypoint;
        out.value = static_cast<Real>(static_cast<float>(xte_raw) / 100.0f);
        out.secondary_value = static_cast<Real>(bearing);
        out.secondary_valid = true;
        out.third_value = static_cast<Real>(range_nmi);
        out.third_valid = true;
        out.flags = yf;
        return true;
    }
    case 0x86: { // Autopilot keystroke.
        if (frame.length < 4) return false;
        const uint8_t key = dg[2];
        const uint8_t expected = static_cast<uint8_t>(0xffu - key);
        out.kind = SeaTalkDecodedKind::autopilot_key;
        out.code = key & 0x3f;
        out.long_press = (key & 0x40) != 0;
        out.alarm = dg[3] != expected;
        out.value = static_cast<Real>(out.code);
        out.flags = key;
        seatalk_copy_label(out.label, sizeof(out.label), seatalk_autopilot_key_label(out.code, out.long_press));
        return true;
    }
    case 0x89: { // Compass heading magnetic. Attribute high nibble carries U.
        if (frame.length < 3) return false;
        const uint8_t u = dg[1] >> 4;
        const uint8_t vw = dg[2];
        out.kind = SeaTalkDecodedKind::heading_magnetic;
        out.value = static_cast<Real>(seatalk_heading_from_u_vw(u, vw));
        return true;
    }
    case 0x99: { // Magnetic variation, signed degrees. Negative values indicate easterly in observed references.
        if (frame.length < 3) return false;
        out.kind = SeaTalkDecodedKind::compass_variation;
        out.value = static_cast<Real>(static_cast<int8_t>(dg[2]));
        return true;
    }
    case 0x9c: { // Rudder datagram: heading plus signed rudder angle.
        if (frame.length < 4) return false;
        const uint8_t u = dg[1] >> 4;
        const uint8_t vw = dg[2];
        const int8_t rudder = static_cast<int8_t>(dg[3]);
        out.kind = SeaTalkDecodedKind::rudder_angle;
        out.value = static_cast<Real>(rudder);
        out.secondary_value = static_cast<Real>(seatalk_heading_from_u_vw(u, vw));
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
