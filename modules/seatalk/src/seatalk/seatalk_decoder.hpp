#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "seatalk_frame.hpp"

namespace seatalk {

enum class SeaTalkDecodedKind : uint8_t {
    none,
    depth,
    engine_rpm_pitch,
    apparent_wind_angle,
    apparent_wind_speed,
    speed_through_water,
    water_temperature,
    heading_magnetic,
    rudder_angle,
    trip_distance,
    total_distance,
    trip_total,
    display_units,
    lamp_intensity,
    position_latitude,
    position_longitude,
    position_lat_lon,
    speed_over_ground,
    course_over_ground,
    time_utc,
    date_utc,
    satellite_info,
    satellite_detail,
    differential_detail,
    autopilot_state,
    autopilot_key,
    navigation_to_waypoint,
    compass_variation,
    waypoint_id,
    waypoint_name,
    waypoint_definition,
    arrival_info,
    device_status,
    observed_unknown
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
    uint8_t int_value = 0;
    uint8_t int_secondary_value = 0;
    uint16_t int_third_value = 0;
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

inline void seatalk_copy_ascii_bytes(char* dst, size_t dst_len, const uint8_t* src, size_t src_len) {
    if (!dst || dst_len == 0) return;
    dst[0] = '\0';
    if (!src) return;
    size_t out = 0;
    for (size_t i = 0; i < src_len && out + 1 < dst_len; ++i) {
        if (src[i] == 0) break;
        dst[out++] = static_cast<char>(src[i]);
    }
    dst[out] = '\0';
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
    case 0x00: {
        if (frame.length < 5) return false;
        const uint16_t raw_feet_x10 = static_cast<uint16_t>(dg[4]) << 8 | dg[3];
        out.kind = SeaTalkDecodedKind::depth;
        out.value = static_cast<Real>((static_cast<float>(raw_feet_x10) / 10.0f) * 0.3048f);
        out.alarm = (dg[2] & 0x80) != 0;
        return true;
    }
    case 0x05: {
        if (frame.length < 6) return false;
        const int16_t rpm = static_cast<int16_t>((static_cast<uint16_t>(dg[3]) << 8) | dg[4]);
        const int8_t pitch = static_cast<int8_t>(dg[5]);
        out.kind = SeaTalkDecodedKind::engine_rpm_pitch;
        out.value = static_cast<Real>(rpm);
        out.secondary_value = static_cast<Real>(pitch);
        out.secondary_valid = true;
        out.code = dg[2];
        if (dg[2] == 1) seatalk_copy_label(out.label, sizeof(out.label), "starboard");
        else if (dg[2] == 2) seatalk_copy_label(out.label, sizeof(out.label), "port");
        else seatalk_copy_label(out.label, sizeof(out.label), "single");
        return true;
    }
    case 0x10: {
        if (frame.length < 4) return false;
        const uint16_t raw_half_deg = static_cast<uint16_t>(dg[2]) << 8 | dg[3];
        out.kind = SeaTalkDecodedKind::apparent_wind_angle;
        out.value = static_cast<Real>(seatalk_wrap_360(static_cast<float>(raw_half_deg) / 2.0f));
        return true;
    }
    case 0x11: {
        if (frame.length < 4) return false;
        const float speed_kn = static_cast<float>(dg[2] & 0x7f) + static_cast<float>(dg[3]) / 10.0f;
        out.kind = SeaTalkDecodedKind::apparent_wind_speed;
        out.value = static_cast<Real>(speed_kn);
        out.secondary_value = static_cast<Real>(seatalk_kn_to_ms(speed_kn));
        out.secondary_valid = true;
        return true;
    }
    case 0x20: {
        if (frame.length < 4) return false;
        const uint16_t raw_kn_x10 = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        const float speed_kn = static_cast<float>(raw_kn_x10) / 10.0f;
        out.kind = SeaTalkDecodedKind::speed_through_water;
        out.value = static_cast<Real>(speed_kn);
        out.secondary_value = static_cast<Real>(seatalk_kn_to_ms(speed_kn));
        out.secondary_valid = true;
        return true;
    }
    case 0x21: {
        if (frame.length < 5) return false;
        uint32_t raw = static_cast<uint32_t>(dg[4] & 0x0f) << 16;
        raw |= static_cast<uint32_t>(dg[3]) << 8;
        raw |= dg[2];
        out.kind = SeaTalkDecodedKind::trip_distance;
        out.value = static_cast<Real>(static_cast<float>(raw) / 100.0f);
        return true;
    }
    case 0x22: {
        if (frame.length < 5) return false;
        uint32_t raw = static_cast<uint32_t>(dg[4]) << 16;
        raw |= static_cast<uint32_t>(dg[3]) << 8;
        raw |= dg[2];
        out.kind = SeaTalkDecodedKind::total_distance;
        out.value = static_cast<Real>(static_cast<float>(raw) / 10.0f);
        return true;
    }
    case 0x23: {
        if (frame.length < 4) return false;
        out.kind = SeaTalkDecodedKind::water_temperature;
        out.value = static_cast<Real>(static_cast<int8_t>(dg[2]));
        out.alarm = (dg[1] & 0x04) != 0;
        return true;
    }
    case 0x24: {
        if (frame.length < 5) return false;
        out.kind = SeaTalkDecodedKind::display_units;
        out.code = dg[4];
        seatalk_copy_label(out.label, sizeof(out.label), dg[4] == 0x86 ? "km_kmph" : (dg[4] == 0x06 ? "statute_miles_mph" : "nautical_miles_knots"));
        return true;
    }
    case 0x25: {
        if (frame.length < 7) return false;
        uint32_t total = dg[1] & 0xf0;
        total <<= 4;
        total |= dg[3];
        total <<= 8;
        total |= dg[2];
        uint32_t trip = dg[6] & 0x0f;
        trip <<= 8;
        trip |= dg[5];
        trip <<= 8;
        trip |= dg[4];
        out.kind = SeaTalkDecodedKind::trip_total;
        out.value = static_cast<Real>(static_cast<float>(total) / 10.0f);
        out.secondary_value = static_cast<Real>(static_cast<float>(trip) / 100.0f);
        out.secondary_valid = true;
        return true;
    }
    case 0x26: {
        if (frame.length < 7) return false;
        const uint16_t raw_kn_x100 = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        const uint16_t raw_avg_x100 = static_cast<uint16_t>(dg[5]) << 8 | dg[4];
        out.kind = SeaTalkDecodedKind::speed_through_water;
        out.value = static_cast<Real>(static_cast<float>(raw_kn_x100) / 100.0f);
        out.secondary_value = static_cast<Real>(static_cast<float>(raw_avg_x100) / 100.0f);
        out.secondary_valid = true;
        return true;
    }
    case 0x27: {
        if (frame.length < 4) return false;
        const uint16_t raw_c = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        out.kind = SeaTalkDecodedKind::water_temperature;
        out.value = static_cast<Real>((static_cast<float>(raw_c) - 100.0f) / 10.0f);
        return true;
    }
    case 0x30: {
        if (frame.length < 3) return false;
        out.kind = SeaTalkDecodedKind::lamp_intensity;
        out.code = dg[2];
        seatalk_copy_label(out.label, sizeof(out.label), dg[2] == 0x0c ? "high" : (dg[2] == 0x08 ? "medium" : (dg[2] == 0x04 ? "low" : "off")));
        return true;
    }
    case 0x50: {
        if (frame.length < 5) return false;
        uint16_t raw_minutes = static_cast<uint16_t>(dg[4]) << 8 | dg[3];
        const bool south = (raw_minutes & 0x8000u) != 0;
        raw_minutes &= 0x7fffu;
        float lat = static_cast<float>(dg[2]) + (static_cast<float>(raw_minutes) / 100.0f) / 60.0f;
        if (south) lat = -lat;
        out.kind = SeaTalkDecodedKind::position_latitude;
        out.value = static_cast<Real>(lat);
        return true;
    }
    case 0x51: {
        if (frame.length < 5) return false;
        uint16_t raw_minutes = static_cast<uint16_t>(dg[4]) << 8 | dg[3];
        const bool east = (raw_minutes & 0x8000u) != 0;
        raw_minutes &= 0x7fffu;
        float lon = static_cast<float>(dg[2]) + (static_cast<float>(raw_minutes) / 100.0f) / 60.0f;
        if (!east) lon = -lon;
        out.kind = SeaTalkDecodedKind::position_longitude;
        out.value = static_cast<Real>(lon);
        return true;
    }
    case 0x52: {
        if (frame.length < 4) return false;
        const uint16_t raw_kn_x10 = static_cast<uint16_t>(dg[3]) << 8 | dg[2];
        out.kind = SeaTalkDecodedKind::speed_over_ground;
        out.value = static_cast<Real>(static_cast<float>(raw_kn_x10) / 10.0f);
        return true;
    }
    case 0x53: {
        if (frame.length < 3) return false;
        out.kind = SeaTalkDecodedKind::course_over_ground;
        out.value = static_cast<Real>(seatalk_heading_from_u_vw(dg[1] >> 4, dg[2]));
        return true;
    }
    case 0x54: {
        if (frame.length < 4) return false;
        const uint8_t t = dg[1] >> 4;
        const uint8_t rs = dg[2];
        const uint8_t st = static_cast<uint8_t>((rs << 4) | t);
        const uint8_t hour = dg[3];
        const uint8_t minutes = (rs & 0xfc) / 4;
        const uint8_t seconds = st & 0x3f;
        out.kind = SeaTalkDecodedKind::time_utc;
        out.value = static_cast<Real>(static_cast<float>(hour) * 3600.0f + static_cast<float>(minutes) * 60.0f + static_cast<float>(seconds));
        out.int_value = hour;
        out.int_secondary_value = minutes;
        out.int_third_value = seconds;
        return true;
    }
    case 0x56: {
        if (frame.length < 4) return false;
        out.kind = SeaTalkDecodedKind::date_utc;
        out.int_value = dg[2];
        out.int_secondary_value = dg[1] >> 4;
        out.int_third_value = static_cast<uint16_t>(2000u + dg[3]);
        return true;
    }
    case 0x57: {
        if (frame.length < 3) return false;
        out.kind = SeaTalkDecodedKind::satellite_info;
        out.int_value = dg[1] >> 4;
        if (out.int_value > 1) {
            out.secondary_value = static_cast<Real>(dg[2]);
            out.secondary_valid = true;
        }
        return true;
    }
    case 0x58: {
        if (frame.length < 8) return false;
        const uint16_t lat_minutes = static_cast<uint16_t>(dg[3]) << 8 | dg[4];
        const uint16_t lon_minutes = static_cast<uint16_t>(dg[6]) << 8 | dg[7];
        float lat = static_cast<float>(dg[2]) + (static_cast<float>(lat_minutes) / 1000.0f) / 60.0f;
        float lon = static_cast<float>(dg[5]) + (static_cast<float>(lon_minutes) / 1000.0f) / 60.0f;
        if (dg[1] & 0x10) lat = -lat;
        if (!(dg[1] & 0x20)) lon = -lon;
        out.kind = SeaTalkDecodedKind::position_lat_lon;
        out.value = static_cast<Real>(lat);
        out.secondary_value = static_cast<Real>(lon);
        out.secondary_valid = true;
        return true;
    }
    case 0x59: {
        out.kind = SeaTalkDecodedKind::observed_unknown;
        out.code = 0x59;
        seatalk_copy_label(out.label, sizeof(out.label), "observed_59");
        return true;
    }
    case 0x61: {
        out.kind = SeaTalkDecodedKind::observed_unknown;
        out.code = 0x61;
        seatalk_copy_label(out.label, sizeof(out.label), "e80_signature");
        return true;
    }
    case 0x82: {
        if (frame.length < 7) return false;
        const uint8_t xx = dg[2];
        const uint8_t yy = dg[4];
        const uint8_t zz = dg[6];
        out.label[0] = static_cast<char>((xx & 0x3f) + 0x30);
        out.label[1] = static_cast<char>(((yy & 0x0f) * 4 + (xx >> 6)) + 0x30);
        out.label[2] = static_cast<char>(((zz & 0x03) * 16 + (yy >> 4)) + 0x30);
        out.label[3] = static_cast<char>(((zz & 0xfc) / 4) + 0x30);
        out.label[4] = '\0';
        out.kind = SeaTalkDecodedKind::waypoint_id;
        return true;
    }
    case 0x84: {
        if (frame.length < 9) return false;
        const uint8_t u = dg[1] >> 4;
        const uint8_t vw = dg[2];
        const uint8_t xy = dg[3];
        const uint8_t z = dg[4];
        const uint8_t alarm_flags = dg[5];
        const int8_t rudder = static_cast<int8_t>(dg[6]);
        const uint8_t status_flags = dg[7];
        const uint8_t v = vw >> 4;
        out.kind = SeaTalkDecodedKind::autopilot_state;
        out.value = static_cast<Real>(seatalk_heading_from_u_vw(u, vw));
        out.secondary_value = static_cast<Real>(seatalk_wrap_360(static_cast<float>(v >> 2) * 90.0f + static_cast<float>(xy) / 2.0f));
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
    case 0x85: {
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
        out.kind = SeaTalkDecodedKind::navigation_to_waypoint;
        out.value = static_cast<Real>(static_cast<float>(xte_raw) / 100.0f);
        out.secondary_value = static_cast<Real>(bearing);
        out.secondary_valid = true;
        out.third_value = static_cast<Real>(static_cast<float>(range_raw) / ((y & 0x01) ? 100.0f : 10.0f));
        out.third_valid = true;
        out.flags = yf;
        return true;
    }
    case 0x86: {
        if (frame.length < 4) return false;
        const uint8_t key = dg[2];
        out.kind = SeaTalkDecodedKind::autopilot_key;
        out.code = key & 0x3f;
        out.long_press = (key & 0x40) != 0;
        out.alarm = dg[3] != static_cast<uint8_t>(0xffu - key);
        out.value = static_cast<Real>(out.code);
        out.flags = key;
        seatalk_copy_label(out.label, sizeof(out.label), seatalk_autopilot_key_label(out.code, out.long_press));
        return true;
    }
    case 0x89: {
        if (frame.length < 3) return false;
        out.kind = SeaTalkDecodedKind::heading_magnetic;
        out.value = static_cast<Real>(seatalk_heading_from_u_vw(dg[1] >> 4, dg[2]));
        return true;
    }
    case 0x97: {
        out.kind = SeaTalkDecodedKind::device_status;
        out.code = 0x97;
        seatalk_copy_label(out.label, sizeof(out.label), "st7000_probe");
        return true;
    }
    case 0x98: {
        out.kind = SeaTalkDecodedKind::device_status;
        out.code = 0x98;
        seatalk_copy_label(out.label, sizeof(out.label), "autopilot_cpu_response");
        return true;
    }
    case 0x99: {
        if (frame.length < 3) return false;
        out.kind = SeaTalkDecodedKind::compass_variation;
        out.value = static_cast<Real>(static_cast<int8_t>(dg[2]));
        return true;
    }
    case 0x9c: {
        if (frame.length < 4) return false;
        out.kind = SeaTalkDecodedKind::rudder_angle;
        out.value = static_cast<Real>(static_cast<int8_t>(dg[3]));
        out.secondary_value = static_cast<Real>(seatalk_heading_from_u_vw(dg[1] >> 4, dg[2]));
        out.secondary_valid = true;
        return true;
    }
    case 0x9e: {
        out.kind = SeaTalkDecodedKind::waypoint_definition;
        seatalk_copy_label(out.label, sizeof(out.label), "waypoint_definition");
        return true;
    }
    case 0xa1: {
        if (frame.length < 9) return false;
        out.kind = SeaTalkDecodedKind::waypoint_name;
        out.code = dg[1] >> 4;
        seatalk_copy_ascii_bytes(out.label, sizeof(out.label), dg + 8, frame.length - 8);
        return true;
    }
    case 0xa2: {
        if (frame.length < 7) return false;
        out.kind = SeaTalkDecodedKind::arrival_info;
        out.flags = dg[1];
        out.secondary_valid = (dg[1] & 0x20) != 0;
        out.third_valid = (dg[1] & 0x40) != 0;
        seatalk_copy_ascii_bytes(out.label, sizeof(out.label), dg + 3, 4);
        return true;
    }
    case 0xa4: {
        out.kind = SeaTalkDecodedKind::device_status;
        out.code = 0xa4;
        seatalk_copy_label(out.label, sizeof(out.label), dg[1] == 0x12 ? "device_identity" : "device_query");
        return true;
    }
    case 0xa5: {
        if (frame.length < 4) return false;
        out.kind = SeaTalkDecodedKind::satellite_detail;
        out.code = dg[1];
        if (dg[1] == 0x57 && frame.length >= 10) {
            out.int_value = dg[2] & 0x0f;
            out.int_secondary_value = static_cast<uint8_t>(((dg[2] & 0xe0) >> 4) + (dg[3] & 0x01));
            out.value = static_cast<Real>((dg[3] & 0x7c) >> 2);
            out.secondary_valid = true;
            out.secondary_value = static_cast<Real>(static_cast<int16_t>(static_cast<uint16_t>(dg[6]) << 4));
            out.int_third_value = static_cast<uint16_t>(((dg[8] & 0xc0) << 2) + dg[9]);
            seatalk_copy_label(out.label, sizeof(out.label), "gps_fix_detail");
        } else if (dg[1] == 0x74) {
            out.int_value = dg[2] & 0x7f;
            out.flags = dg[2] & 0x80;
            seatalk_copy_label(out.label, sizeof(out.label), "satellites_used");
        } else if (dg[1] == 0x98) {
            seatalk_copy_label(out.label, sizeof(out.label), "satellites_done");
        } else if (dg[1] == 0xb5) {
            seatalk_copy_label(out.label, sizeof(out.label), "wgs84_datum");
        } else if (frame.length >= 16) {
            out.int_secondary_value = dg[1];
            out.int_value = static_cast<uint8_t>((dg[2] & 0xfe) / 2);
            out.value = static_cast<Real>(static_cast<uint16_t>(dg[3]) * 2u + (dg[4] & 0x01));
            out.secondary_value = static_cast<Real>((dg[4] & 0xfe) / 2);
            out.secondary_valid = true;
            out.third_value = static_cast<Real>((dg[5] & 0xfe) / 2);
            out.third_valid = true;
            seatalk_copy_label(out.label, sizeof(out.label), "satellite_detail");
        } else {
            seatalk_copy_label(out.label, sizeof(out.label), "satellite_detail");
        }
        return true;
    }
    case 0xa7: {
        if (frame.length < 12) return false;
        out.kind = SeaTalkDecodedKind::differential_detail;
        out.code = dg[1];
        out.int_value = dg[2];
        out.value = static_cast<Real>(static_cast<uint16_t>(dg[3]) * 2u + (dg[4] & 0x01));
        out.secondary_value = static_cast<Real>((dg[4] & 0xfe) / 2);
        out.secondary_valid = true;
        out.third_value = static_cast<Real>((dg[5] & 0xfe) / 2);
        out.third_valid = true;
        seatalk_copy_label(out.label, sizeof(out.label), "differential_detail");
        return true;
    }
    case 0xad: {
        out.kind = SeaTalkDecodedKind::observed_unknown;
        out.code = 0xad;
        seatalk_copy_label(out.label, sizeof(out.label), "observed_ad");
        return true;
    }
    default:
        return false;
    }
}

} // namespace seatalk
