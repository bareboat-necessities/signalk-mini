#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace signalk_mini {


inline bool signalk_make_uuid_context(char* dst, size_t dst_size, uint64_t high, uint64_t low) {
    if (!dst || dst_size < 66) return false;
    unsigned char bytes[16];
    for (size_t i = 0; i < 8; ++i) bytes[i] = static_cast<unsigned char>((high >> ((7 - i) * 8)) & 0xffU);
    for (size_t i = 0; i < 8; ++i) bytes[8 + i] = static_cast<unsigned char>((low >> ((7 - i) * 8)) & 0xffU);
    bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0fU) | 0x40U);
    bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3fU) | 0x80U);
    const int len = snprintf(dst, dst_size,
        "vessels.urn:mrn:signalk:uuid:%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
        bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
        bytes[8], bytes[9], bytes[10], bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);
    return len > 0 && static_cast<size_t>(len) < dst_size;
}

enum class SignalKVesselIdentityKind : unsigned char {
    Invalid,
    Uuid,
    Mmsi,
    Url,
};

inline const char* signalk_vessel_key(const char* context) {
    if (!context) return nullptr;
    static constexpr char Prefix[] = "vessels.";
    return strncmp(context, Prefix, sizeof(Prefix) - 1) == 0 ? context + sizeof(Prefix) - 1 : nullptr;
}

inline bool signalk_is_hex(char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

inline bool signalk_valid_uuid_key(const char* key) {
    static constexpr char Prefix[] = "urn:mrn:signalk:uuid:";
    if (!key || strncmp(key, Prefix, sizeof(Prefix) - 1) != 0) return false;
    const char* uuid = key + sizeof(Prefix) - 1;
    if (strlen(uuid) != 36 || uuid[8] != '-' || uuid[13] != '-' || uuid[18] != '-' || uuid[23] != '-') return false;
    for (size_t i = 0; i < 36; ++i) {
        if (i == 8 || i == 13 || i == 18 || i == 23) continue;
        if (!signalk_is_hex(uuid[i])) return false;
    }
    return true;
}

inline bool signalk_valid_mmsi_text(const char* mmsi) {
    if (!mmsi || strlen(mmsi) != 9 || mmsi[0] < '2' || mmsi[0] > '7') return false;
    for (size_t i = 1; i < 9; ++i) if (mmsi[i] < '0' || mmsi[i] > '9') return false;
    return true;
}

inline bool signalk_valid_mmsi_key(const char* key) {
    static constexpr char Prefix[] = "urn:mrn:imo:mmsi:";
    return key && strncmp(key, Prefix, sizeof(Prefix) - 1) == 0 &&
           signalk_valid_mmsi_text(key + sizeof(Prefix) - 1);
}

inline bool signalk_valid_url_key(const char* key) {
    if (!key) return false;
    if (strncmp(key, "http://", 7) == 0) return key[7] != '\0';
    if (strncmp(key, "https://", 8) == 0) return key[8] != '\0';
    if (strncmp(key, "mailto:", 7) == 0) return key[7] != '\0';
    if (strncmp(key, "tel:", 4) == 0) return key[4] != '\0';
    return false;
}

inline SignalKVesselIdentityKind signalk_vessel_identity_kind(const char* key) {
    if (signalk_valid_uuid_key(key)) return SignalKVesselIdentityKind::Uuid;
    if (signalk_valid_mmsi_key(key)) return SignalKVesselIdentityKind::Mmsi;
    if (signalk_valid_url_key(key)) return SignalKVesselIdentityKind::Url;
    return SignalKVesselIdentityKind::Invalid;
}

inline bool signalk_valid_self_context(const char* context) {
    const char* key = signalk_vessel_key(context);
    return key && signalk_vessel_identity_kind(key) != SignalKVesselIdentityKind::Invalid;
}

} // namespace signalk_mini
