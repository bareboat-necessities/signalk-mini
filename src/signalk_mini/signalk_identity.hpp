#pragma once

#include <stddef.h>
#include <string.h>

namespace signalk_mini {

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
    if (uuid[14] != '4') return false;
    const char variant = uuid[19];
    if (!(variant == '8' || variant == '9' || variant == 'a' || variant == 'A' ||
          variant == 'b' || variant == 'B')) return false;
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
    return key && (strncmp(key, "http://", 7) == 0 || strncmp(key, "https://", 8) == 0 ||
                   strncmp(key, "mailto:", 7) == 0 || strncmp(key, "tel:", 4) == 0);
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
