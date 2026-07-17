#pragma once

#include <stdint.h>
#include <string.h>
#include <vector>

namespace ubx_test {

inline void put_u16(std::vector<uint8_t>& data, size_t offset, uint16_t value) {
    data[offset] = static_cast<uint8_t>(value & 0xffu);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
}

inline void put_i16(std::vector<uint8_t>& data, size_t offset, int16_t value) {
    put_u16(data, offset, static_cast<uint16_t>(value));
}

inline void put_u32(std::vector<uint8_t>& data, size_t offset, uint32_t value) {
    data[offset] = static_cast<uint8_t>(value & 0xffu);
    data[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xffu);
    data[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xffu);
    data[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xffu);
}

inline void put_i32(std::vector<uint8_t>& data, size_t offset, int32_t value) {
    put_u32(data, offset, static_cast<uint32_t>(value));
}

inline std::vector<uint8_t> frame(uint8_t message_class, uint8_t message_id, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> out;
    out.reserve(payload.size() + 8);
    out.push_back(0xB5);
    out.push_back(0x62);
    out.push_back(message_class);
    out.push_back(message_id);
    out.push_back(static_cast<uint8_t>(payload.size() & 0xffu));
    out.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xffu));
    out.insert(out.end(), payload.begin(), payload.end());
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < out.size(); ++i) {
        ck_a = static_cast<uint8_t>(ck_a + out[i]);
        ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }
    out.push_back(ck_a);
    out.push_back(ck_b);
    return out;
}

inline std::vector<uint8_t> nav_pvt(bool fix_ok = true) {
    std::vector<uint8_t> payload(92, 0);
    put_u16(payload, 4, 2026);
    payload[6] = 7;
    payload[7] = 17;
    payload[8] = 12;
    payload[9] = 34;
    payload[10] = 56;
    payload[11] = 0x0b;
    put_u32(payload, 12, 25000000u);
    put_i32(payload, 16, 125000000);
    payload[20] = fix_ok ? 3 : 1;
    payload[21] = fix_ok ? 0x01 : 0x00;
    payload[23] = 12;
    put_i32(payload, 24, -741234567);
    put_i32(payload, 28, 407654321);
    put_i32(payload, 32, 15340);
    put_i32(payload, 36, -18200);
    put_u32(payload, 40, 800);
    put_u32(payload, 44, 1200);
    put_i32(payload, 48, 1000);
    put_i32(payload, 52, 2000);
    put_i32(payload, 56, -300);
    put_i32(payload, 60, 2236);
    put_i32(payload, 64, 1234567);
    put_u32(payload, 68, 150);
    put_u32(payload, 72, 25000);
    put_u16(payload, 76, 145);
    put_i32(payload, 84, 1230000);
    put_i16(payload, 88, -1350);
    put_u16(payload, 90, 25);
    return frame(0x01, 0x07, payload);
}

inline std::vector<uint8_t> nav_dop() {
    std::vector<uint8_t> payload(18, 0);
    put_u16(payload, 4, 250);
    put_u16(payload, 6, 145);
    put_u16(payload, 8, 120);
    put_u16(payload, 10, 180);
    put_u16(payload, 12, 95);
    put_u16(payload, 14, 70);
    put_u16(payload, 16, 65);
    return frame(0x01, 0x04, payload);
}

inline std::vector<uint8_t> nav_sat(size_t satellite_count = 3) {
    if (satellite_count > 80) satellite_count = 80;
    std::vector<uint8_t> payload(8 + satellite_count * 12, 0);
    payload[4] = 1;
    payload[5] = static_cast<uint8_t>(satellite_count);
    for (size_t i = 0; i < satellite_count; ++i) {
        const size_t offset = 8 + i * 12;
        payload[offset] = static_cast<uint8_t>(i == 2 ? 2 : 0);
        payload[offset + 1] = static_cast<uint8_t>(10 + i);
        payload[offset + 2] = static_cast<uint8_t>(40 + i);
        payload[offset + 3] = static_cast<uint8_t>(20 + i);
        put_i16(payload, offset + 4, static_cast<int16_t>(100 + i * 20));
        uint32_t flags = (1u << 4);
        if (i != 1) flags |= (1u << 3);
        if (i == 0) flags |= (1u << 6);
        put_u32(payload, offset + 8, flags);
    }
    return frame(0x01, 0x35, payload);
}

inline std::vector<uint8_t> mon_ver() {
    std::vector<uint8_t> payload(70, 0);
    const char* sw = "EXT CORE 1.00";
    const char* hw = "00190000";
    const char* ext = "PROTVER=27.31";
    memcpy(payload.data(), sw, strlen(sw));
    memcpy(payload.data() + 30, hw, strlen(hw));
    memcpy(payload.data() + 40, ext, strlen(ext));
    return frame(0x0A, 0x04, payload);
}

} // namespace ubx_test
