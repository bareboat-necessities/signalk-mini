#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>

namespace ubx {

inline constexpr uint8_t Sync1 = 0xB5;
inline constexpr uint8_t Sync2 = 0x62;

inline constexpr uint8_t ClassNav = 0x01;
inline constexpr uint8_t ClassAck = 0x05;
inline constexpr uint8_t ClassMon = 0x0A;

inline constexpr uint8_t NavDop = 0x04;
inline constexpr uint8_t NavPvt = 0x07;
inline constexpr uint8_t NavSat = 0x35;
inline constexpr uint8_t AckNak = 0x00;
inline constexpr uint8_t AckAck = 0x01;
inline constexpr uint8_t MonVer = 0x04;

inline constexpr size_t DefaultMaxPayload = 1024;

enum class UpdateKind : uint8_t {
    None = 0,
    Fix = 1u << 0,
    Dop = 1u << 1,
    Sky = 1u << 2,
    Version = 1u << 3,
    Ack = 1u << 4,
};

inline UpdateKind operator|(UpdateKind left, UpdateKind right) {
    return static_cast<UpdateKind>(static_cast<uint8_t>(left) | static_cast<uint8_t>(right));
}

inline UpdateKind& operator|=(UpdateKind& left, UpdateKind right) {
    left = left | right;
    return left;
}

inline bool has_update(UpdateKind value, UpdateKind bit) {
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(bit)) != 0;
}

struct ReceiverDiagnostics {
    uint32_t frame_count = 0;
    uint32_t checksum_error_count = 0;
    uint32_t oversized_frame_count = 0;
    uint32_t unsupported_frame_count = 0;
    uint32_t discarded_byte_count = 0;
};

struct ReceiverInfo {
    char software_version[31]{};
    char hardware_version[11]{};
    char protocol_version[16]{};
    uint8_t last_ack_class = 0;
    uint8_t last_ack_id = 0;
    bool last_ack_accepted = false;
    bool ack_valid = false;
};

inline uint16_t read_u16_le(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

inline int16_t read_i16_le(const uint8_t* data) {
    return static_cast<int16_t>(read_u16_le(data));
}

inline uint32_t read_u32_le(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

inline int32_t read_i32_le(const uint8_t* data) {
    return static_cast<int32_t>(read_u32_le(data));
}

template<typename Real>
inline Real utc_seconds_of_day(unsigned hour, unsigned minute, unsigned second, int32_t nano) {
    return static_cast<Real>(hour * 3600u + minute * 60u + second) +
           static_cast<Real>(nano) / static_cast<Real>(1000000000);
}

inline bool is_leap_year(int year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

inline bool valid_calendar_date(int year, unsigned month, unsigned day) {
    static constexpr uint8_t DaysPerMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (year < 1970 || month < 1 || month > 12 || day < 1) return false;
    unsigned maximum = DaysPerMonth[month - 1];
    if (month == 2 && is_leap_year(year)) ++maximum;
    return day <= maximum;
}

template<typename Real>
inline Real normalize_degrees(Real value) {
    while (value < static_cast<Real>(0)) value += static_cast<Real>(360);
    while (value >= static_cast<Real>(360)) value -= static_cast<Real>(360);
    return value;
}

template<typename Real, size_t MaxPayload = DefaultMaxPayload>
class Receiver {
public:
    Receiver() = default;

    UpdateKind accept_octets(const uint8_t* bytes,
                             size_t length,
                             ship_data_model::DataModel<Real>& model,
                             uint64_t now_us,
                             ship_data_model::SensorSource source = ship_data_model::SensorSource::ubx) {
        UpdateKind updates = UpdateKind::None;
        if (!bytes) return updates;
        for (size_t i = 0; i < length; ++i) updates |= accept_byte(bytes[i], model, now_us, source);
        return updates;
    }

    UpdateKind accept_datagram(const uint8_t* bytes,
                               size_t length,
                               ship_data_model::DataModel<Real>& model,
                               uint64_t now_us,
                               ship_data_model::SensorSource source = ship_data_model::SensorSource::ubx) {
        return accept_octets(bytes, length, model, now_us, source);
    }

    void reset_stream() {
        state_ = State::Sync1;
        payload_length_ = 0;
        payload_index_ = 0;
        ck_a_ = 0;
        ck_b_ = 0;
    }

    const ReceiverDiagnostics& diagnostics() const { return diagnostics_; }
    const ReceiverInfo& info() const { return info_; }

private:
    enum class State : uint8_t {
        Sync1,
        Sync2,
        Class,
        Id,
        Length1,
        Length2,
        Payload,
        ChecksumA,
        ChecksumB,
    };

    void checksum(uint8_t value) {
        ck_a_ = static_cast<uint8_t>(ck_a_ + value);
        ck_b_ = static_cast<uint8_t>(ck_b_ + ck_a_);
    }

    void start_frame() {
        state_ = State::Class;
        payload_length_ = 0;
        payload_index_ = 0;
        ck_a_ = 0;
        ck_b_ = 0;
    }

    UpdateKind accept_byte(uint8_t value,
                           ship_data_model::DataModel<Real>& model,
                           uint64_t now_us,
                           ship_data_model::SensorSource source) {
        switch (state_) {
        case State::Sync1:
            if (value == Sync1) state_ = State::Sync2;
            else ++diagnostics_.discarded_byte_count;
            break;
        case State::Sync2:
            if (value == Sync2) start_frame();
            else if (value != Sync1) {
                state_ = State::Sync1;
                ++diagnostics_.discarded_byte_count;
            }
            break;
        case State::Class:
            message_class_ = value;
            checksum(value);
            state_ = State::Id;
            break;
        case State::Id:
            message_id_ = value;
            checksum(value);
            state_ = State::Length1;
            break;
        case State::Length1:
            payload_length_ = value;
            checksum(value);
            state_ = State::Length2;
            break;
        case State::Length2:
            payload_length_ |= static_cast<uint16_t>(static_cast<uint16_t>(value) << 8);
            checksum(value);
            payload_index_ = 0;
            if (payload_length_ > MaxPayload) {
                ++diagnostics_.oversized_frame_count;
                state_ = State::Sync1;
            } else {
                state_ = payload_length_ == 0 ? State::ChecksumA : State::Payload;
            }
            break;
        case State::Payload:
            payload_[payload_index_++] = value;
            checksum(value);
            if (payload_index_ == payload_length_) state_ = State::ChecksumA;
            break;
        case State::ChecksumA:
            received_ck_a_ = value;
            state_ = State::ChecksumB;
            break;
        case State::ChecksumB: {
            const bool checksum_ok = received_ck_a_ == ck_a_ && value == ck_b_;
            state_ = State::Sync1;
            if (!checksum_ok) {
                ++diagnostics_.checksum_error_count;
                return UpdateKind::None;
            }
            ++diagnostics_.frame_count;
            return dispatch(model, now_us, source);
        }
        }
        return UpdateKind::None;
    }

    UpdateKind dispatch(ship_data_model::DataModel<Real>& model,
                        uint64_t now_us,
                        ship_data_model::SensorSource source) {
        if (message_class_ == ClassNav && message_id_ == NavPvt && payload_length_ >= 92) return apply_nav_pvt(model, now_us, source);
        if (message_class_ == ClassNav && message_id_ == NavDop && payload_length_ >= 18) return apply_nav_dop(model, now_us, source);
        if (message_class_ == ClassNav && message_id_ == NavSat && payload_length_ >= 8) return apply_nav_sat(model, now_us, source);
        if (message_class_ == ClassMon && message_id_ == MonVer && payload_length_ >= 40) return apply_mon_ver();
        if (message_class_ == ClassAck && (message_id_ == AckAck || message_id_ == AckNak) && payload_length_ >= 2) return apply_ack();
        ++diagnostics_.unsupported_frame_count;
        return UpdateKind::None;
    }

    UpdateKind apply_nav_pvt(ship_data_model::DataModel<Real>& model,
                             uint64_t now_us,
                             ship_data_model::SensorSource source) {
        auto& fix = model.gnss.fix;
        auto& accuracy = model.gnss.fix_accuracy;
        auto& dop = model.gnss.dop_active_satellites;

        const int year = static_cast<int>(read_u16_le(payload_ + 4));
        const unsigned month = payload_[6];
        const unsigned day = payload_[7];
        const unsigned hour = payload_[8];
        const unsigned minute = payload_[9];
        const unsigned second = payload_[10];
        const uint8_t valid = payload_[11];
        const uint8_t fix_type = payload_[20];
        const uint8_t flags = payload_[21];
        const bool fix_ok = (flags & 0x01u) != 0 && fix_type >= 1 && fix_type <= 4;
        int32_t fix_quality = 0;
        if (fix_ok) {
            const uint8_t carrier_solution = static_cast<uint8_t>((flags >> 6) & 0x03u);
            if (carrier_solution == 2) fix_quality = 4;
            else if (carrier_solution == 1) fix_quality = 5;
            else if ((flags & 0x02u) != 0) fix_quality = 2;
            else fix_quality = 1;
        }

        fix.fix_type.set(static_cast<int32_t>(fix_type), now_us);
        fix.fix_quality.set(fix_quality, now_us);
        fix.fix_valid.set(fix_ok, now_us);
        fix.satellites_used.set(static_cast<int32_t>(payload_[23]), now_us);
        dop.fix_mode.set(fix_ok ? (fix_type == 2 ? 2 : 3) : 1, now_us);

        if ((valid & 0x03u) == 0x03u && valid_calendar_date(year, month, day) && hour <= 23 && minute <= 59 && second <= 60) {
            const int32_t nano = read_i32_le(payload_ + 16);
            fix.timestamp_s.set(utc_seconds_of_day<Real>(hour, minute, second, nano), now_us);
            fix.date_year.set(year, now_us);
            fix.date_month.set(static_cast<int32_t>(month), now_us);
            fix.date_day.set(static_cast<int32_t>(day), now_us);
        }

        accuracy.time_accuracy_s.set(static_cast<Real>(read_u32_le(payload_ + 12)) / static_cast<Real>(1000000000), now_us);
        accuracy.horizontal_accuracy_m.set(static_cast<Real>(read_u32_le(payload_ + 40)) / static_cast<Real>(1000), now_us);
        accuracy.vertical_accuracy_m.set(static_cast<Real>(read_u32_le(payload_ + 44)) / static_cast<Real>(1000), now_us);
        accuracy.speed_accuracy_m_s.set(static_cast<Real>(read_u32_le(payload_ + 68)) / static_cast<Real>(1000), now_us);
        accuracy.track_accuracy_deg.set(static_cast<Real>(read_u32_le(payload_ + 72)) / static_cast<Real>(100000), now_us);
        accuracy.pdop.set(static_cast<Real>(read_u16_le(payload_ + 76)) / static_cast<Real>(100), now_us);
        dop.pdop.set(accuracy.pdop.value, now_us);

        if (fix_ok) {
            fix.fix_lon_deg.set(static_cast<Real>(read_i32_le(payload_ + 24)) / static_cast<Real>(10000000), now_us);
            fix.fix_lat_deg.set(static_cast<Real>(read_i32_le(payload_ + 28)) / static_cast<Real>(10000000), now_us);
            fix.fix_alt_hae_m.set(static_cast<Real>(read_i32_le(payload_ + 32)) / static_cast<Real>(1000), now_us);
            fix.fix_alt_msl_m.set(static_cast<Real>(read_i32_le(payload_ + 36)) / static_cast<Real>(1000), now_us);
            fix.geoidal_separation_m.set(fix.fix_alt_hae_m.value - fix.fix_alt_msl_m.value, now_us);
            const Real speed_m_s = static_cast<Real>(read_i32_le(payload_ + 60)) / static_cast<Real>(1000);
            fix.speed_kn.set(speed_m_s / static_cast<Real>(0.5144444444444444), now_us);
            fix.vertical_speed_m_s.set(-static_cast<Real>(read_i32_le(payload_ + 56)) / static_cast<Real>(1000), now_us);
            fix.track_deg.set(normalize_degrees(static_cast<Real>(read_i32_le(payload_ + 64)) / static_cast<Real>(100000)), now_us);
            if (payload_length_ >= 92 && (valid & 0x08u) != 0) {
                fix.declination_deg.set(static_cast<Real>(read_i16_le(payload_ + 88)) / static_cast<Real>(100), now_us);
            }
        }

        fix.source.value = source;
        accuracy.source.value = source;
        dop.source.value = source;
        fix.last_update_us = now_us;
        accuracy.last_update_us = now_us;
        dop.last_update_us = now_us;
        return UpdateKind::Fix | UpdateKind::Dop;
    }

    UpdateKind apply_nav_dop(ship_data_model::DataModel<Real>& model,
                             uint64_t now_us,
                             ship_data_model::SensorSource source) {
        auto& dop = model.gnss.dop_active_satellites;
        auto& fix = model.gnss.fix;
        auto& accuracy = model.gnss.fix_accuracy;
        dop.gdop.set(static_cast<Real>(read_u16_le(payload_ + 4)) / static_cast<Real>(100), now_us);
        dop.pdop.set(static_cast<Real>(read_u16_le(payload_ + 6)) / static_cast<Real>(100), now_us);
        dop.tdop.set(static_cast<Real>(read_u16_le(payload_ + 8)) / static_cast<Real>(100), now_us);
        dop.vdop.set(static_cast<Real>(read_u16_le(payload_ + 10)) / static_cast<Real>(100), now_us);
        dop.hdop.set(static_cast<Real>(read_u16_le(payload_ + 12)) / static_cast<Real>(100), now_us);
        dop.north_dop.set(static_cast<Real>(read_u16_le(payload_ + 14)) / static_cast<Real>(100), now_us);
        dop.east_dop.set(static_cast<Real>(read_u16_le(payload_ + 16)) / static_cast<Real>(100), now_us);
        fix.hdop.set(dop.hdop.value, now_us);
        accuracy.gdop.set(dop.gdop.value, now_us);
        accuracy.pdop.set(dop.pdop.value, now_us);
        accuracy.tdop.set(dop.tdop.value, now_us);
        accuracy.hdop.set(dop.hdop.value, now_us);
        accuracy.vdop.set(dop.vdop.value, now_us);
        accuracy.north_dop.set(dop.north_dop.value, now_us);
        accuracy.east_dop.set(dop.east_dop.value, now_us);
        dop.source.value = source;
        fix.source.value = source;
        accuracy.source.value = source;
        dop.last_update_us = now_us;
        fix.last_update_us = now_us;
        accuracy.last_update_us = now_us;
        return UpdateKind::Dop;
    }

    UpdateKind apply_nav_sat(ship_data_model::DataModel<Real>& model,
                             uint64_t now_us,
                             ship_data_model::SensorSource source) {
        auto& sky = model.gnss.sky_view;
        const uint8_t declared_count = payload_[5];
        const size_t available_count = (payload_length_ - 8u) / 12u;
        const size_t count = declared_count < available_count ? declared_count : available_count;
        sky.clear_observations();
        sky.talker_id[0] = 'U';
        sky.talker_id[1] = 'B';
        sky.talker_id[2] = '\0';
        int32_t used_count = 0;
        const size_t stored_count = count < sky.capacity() ? count : sky.capacity();
        for (size_t i = 0; i < count; ++i) {
            const uint8_t* item = payload_ + 8u + i * 12u;
            const uint32_t flags = read_u32_le(item + 8);
            const bool used = (flags & (1u << 3)) != 0;
            if (used) ++used_count;
            if (i >= stored_count) continue;
            auto& observation = sky.observations[i];
            observation.system_id = static_cast<int16_t>(item[0]);
            observation.satellite_id = static_cast<int16_t>(item[1]);
            observation.satellite_id_valid = true;
            observation.cn0_db_hz = static_cast<Real>(item[2]);
            observation.cn0_valid = true;
            observation.elevation_deg = static_cast<Real>(static_cast<int8_t>(item[3]));
            observation.elevation_valid = true;
            observation.azimuth_true_deg = static_cast<Real>(read_i16_le(item + 4));
            observation.azimuth_valid = true;
            observation.used = used;
            observation.healthy = ((flags >> 4) & 0x03u) == 1u;
            observation.differential_corrections = (flags & (1u << 6)) != 0;
        }
        sky.observation_count = static_cast<uint16_t>(stored_count);
        sky.satellites_in_view.set(static_cast<int32_t>(count), now_us);
        sky.satellites_used.set(used_count, now_us);
        sky.complete = true;
        ++sky.sequence;
        sky.source.value = source;
        sky.last_update_us = now_us;
        model.gnss.fix.satellites_used.set(used_count, now_us);
        model.gnss.fix.source.value = source;
        model.gnss.fix.last_update_us = now_us;
        return UpdateKind::Sky;
    }

    UpdateKind apply_mon_ver() {
        copy_fixed_text(info_.software_version, sizeof(info_.software_version), payload_, 30);
        copy_fixed_text(info_.hardware_version, sizeof(info_.hardware_version), payload_ + 30, 10);
        for (size_t offset = 40; offset + 30 <= payload_length_; offset += 30) {
            const char* extension = reinterpret_cast<const char*>(payload_ + offset);
            static constexpr char Prefix[] = "PROTVER=";
            if (memcmp(extension, Prefix, sizeof(Prefix) - 1) == 0) {
                copy_fixed_text(info_.protocol_version, sizeof(info_.protocol_version), payload_ + offset + sizeof(Prefix) - 1, 30 - (sizeof(Prefix) - 1));
            }
        }
        return UpdateKind::Version;
    }

    UpdateKind apply_ack() {
        info_.last_ack_class = payload_[0];
        info_.last_ack_id = payload_[1];
        info_.last_ack_accepted = message_id_ == AckAck;
        info_.ack_valid = true;
        return UpdateKind::Ack;
    }

    static void copy_fixed_text(char* destination, size_t destination_size, const uint8_t* source, size_t source_size) {
        if (!destination || destination_size == 0) return;
        size_t length = 0;
        while (length < source_size && source[length] != 0 && length + 1 < destination_size) ++length;
        if (length > 0) memcpy(destination, source, length);
        destination[length] = '\0';
        while (length > 0 && destination[length - 1] == ' ') destination[--length] = '\0';
    }

    State state_ = State::Sync1;
    uint8_t message_class_ = 0;
    uint8_t message_id_ = 0;
    uint16_t payload_length_ = 0;
    uint16_t payload_index_ = 0;
    uint8_t payload_[MaxPayload]{};
    uint8_t ck_a_ = 0;
    uint8_t ck_b_ = 0;
    uint8_t received_ck_a_ = 0;
    ReceiverDiagnostics diagnostics_{};
    ReceiverInfo info_{};
};

} // namespace ubx
