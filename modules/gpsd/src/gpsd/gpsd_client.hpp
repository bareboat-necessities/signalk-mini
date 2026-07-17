#pragma once

#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <ship_data_model.hpp>

namespace gpsd {

inline constexpr size_t DefaultMaxRecordLength = 10240;
inline constexpr double MetersPerSecondToKnots = 1.9438444924406;

enum class UpdateKind : uint8_t {
    None = 0,
    Fix = 1u << 0,
    Sky = 1u << 1,
    Accuracy = 1u << 2,
    Version = 1u << 3,
    Session = 1u << 4,
};

inline UpdateKind operator|(UpdateKind a, UpdateKind b) {
    return static_cast<UpdateKind>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
inline UpdateKind& operator|=(UpdateKind& a, UpdateKind b) { a = a | b; return a; }
inline bool has_update(UpdateKind value, UpdateKind bit) {
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(bit)) != 0;
}

enum class SessionState : uint8_t {
    Disconnected,
    WaitingForVersion,
    Watching,
};

struct Diagnostics {
    uint32_t record_count = 0;
    uint32_t malformed_record_count = 0;
    uint32_t oversized_record_count = 0;
    uint32_t ignored_record_count = 0;
    uint32_t device_mismatch_count = 0;
};

struct ServerInfo {
    char release[32]{};
    char revision[32]{};
    int32_t protocol_major = 0;
    int32_t protocol_minor = 0;
    bool valid = false;
};

inline bool nonempty(const char* value) { return value && value[0] != '\0'; }

inline bool make_watch_command(char* output, size_t capacity, const char* device) {
    if (!output || capacity == 0) return false;
    int n = 0;
    if (nonempty(device)) {
        for (const char* p = device; *p; ++p) {
            if (*p == '"' || *p == '\\' || static_cast<unsigned char>(*p) < 0x20u) return false;
        }
        n = snprintf(output, capacity,
                     "?WATCH={\"enable\":true,\"json\":true,\"device\":\"%s\"};\n",
                     device);
    } else {
        n = snprintf(output, capacity, "?WATCH={\"enable\":true,\"json\":true};\n");
    }
    return n > 0 && static_cast<size_t>(n) < capacity;
}

inline bool parse_iso8601_utc(const char* text,
                              int32_t& year,
                              int32_t& month,
                              int32_t& day,
                              double& seconds_of_day) {
    if (!text) return false;
    int y = 0, mo = 0, d = 0, h = 0, mi = 0;
    double sec = 0.0;
    if (sscanf(text, "%d-%d-%dT%d:%d:%lf", &y, &mo, &d, &h, &mi, &sec) != 6) return false;
    if (y < 1970 || mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23 || mi < 0 || mi > 59 || sec < 0.0 || sec >= 61.0) return false;
    year = y;
    month = mo;
    day = d;
    seconds_of_day = static_cast<double>(h * 3600 + mi * 60) + sec;
    return true;
}

template<typename T>
inline bool read_number(JsonObjectConst object, const char* key, T& out) {
    JsonVariantConst value = object[key];
    if (value.isNull() || !(value.is<double>() || value.is<long>() || value.is<unsigned long>())) return false;
    out = static_cast<T>(value.as<double>());
    return true;
}

template<typename Real, size_t MaxRecordLength = DefaultMaxRecordLength>
class Client {
public:
    explicit Client(const char* device = nullptr, bool include_sky = true, bool include_gst = true)
        : device_(device), include_sky_(include_sky), include_gst_(include_gst) {}

    void configure(const char* device, bool include_sky, bool include_gst) {
        device_ = device;
        include_sky_ = include_sky;
        include_gst_ = include_gst;
    }

    void on_connected() {
        reset_record();
        state_ = SessionState::WaitingForVersion;
    }

    void on_disconnected() {
        reset_record();
        state_ = SessionState::Disconnected;
    }

    UpdateKind accept_octets(const uint8_t* bytes,
                             size_t length,
                             ship_data_model::DataModel<Real>& model,
                             uint64_t now_us,
                             ship_data_model::SensorSource source = ship_data_model::SensorSource::gpsd) {
        UpdateKind updates = UpdateKind::None;
        if (!bytes) return updates;
        for (size_t i = 0; i < length; ++i) {
            const uint8_t byte = bytes[i];
            if (discarding_) {
                if (byte == '\n') reset_record();
                continue;
            }
            if (byte == '\n') {
                if (record_length_ > 0) {
                    while (record_length_ > 0 && record_[record_length_ - 1] == '\r') --record_length_;
                    record_[record_length_] = '\0';
                    updates |= accept_record(record_, model, now_us, source);
                }
                reset_record();
                continue;
            }
            if (record_length_ >= MaxRecordLength) {
                ++diagnostics_.oversized_record_count;
                discarding_ = true;
                continue;
            }
            record_[record_length_++] = static_cast<char>(byte);
        }
        return updates;
    }

    UpdateKind accept_record(const char* record,
                             ship_data_model::DataModel<Real>& model,
                             uint64_t now_us,
                             ship_data_model::SensorSource source = ship_data_model::SensorSource::gpsd) {
        if (!record || !record[0]) return UpdateKind::None;
        JsonDocument document;
        const DeserializationError error = deserializeJson(document, record);
        if (error) {
            ++diagnostics_.malformed_record_count;
            return UpdateKind::None;
        }
        JsonObjectConst object = document.as<JsonObjectConst>();
        const char* message_class = object["class"] | "";
        if (!message_class[0]) {
            ++diagnostics_.ignored_record_count;
            return UpdateKind::None;
        }
        ++diagnostics_.record_count;

        if (strcmp(message_class, "VERSION") == 0) return apply_version(object);
        if (strcmp(message_class, "WATCH") == 0) {
            if (object["enable"] | false) state_ = SessionState::Watching;
            return UpdateKind::Session;
        }
        if (strcmp(message_class, "DEVICE") == 0 || strcmp(message_class, "DEVICES") == 0) return UpdateKind::Session;
        if (!device_matches(object)) return UpdateKind::None;
        if (strcmp(message_class, "TPV") == 0) return apply_tpv(object, model, now_us, source);
        if (strcmp(message_class, "SKY") == 0 && include_sky_) return apply_sky(object, model, now_us, source);
        if (strcmp(message_class, "GST") == 0 && include_gst_) return apply_gst(object, model, now_us, source);
        ++diagnostics_.ignored_record_count;
        return UpdateKind::None;
    }

    const Diagnostics& diagnostics() const { return diagnostics_; }
    const ServerInfo& server_info() const { return server_info_; }
    SessionState state() const { return state_; }

private:
    void reset_record() {
        record_length_ = 0;
        discarding_ = false;
        record_[0] = '\0';
    }

    bool device_matches(JsonObjectConst object) {
        if (!nonempty(device_)) return true;
        const char* report_device = object["device"] | "";
        if (report_device[0] && strcmp(report_device, device_) == 0) return true;
        ++diagnostics_.device_mismatch_count;
        return false;
    }

    UpdateKind apply_version(JsonObjectConst object) {
        const char* release = object["release"] | "";
        const char* revision = object["rev"] | "";
        snprintf(server_info_.release, sizeof(server_info_.release), "%s", release);
        snprintf(server_info_.revision, sizeof(server_info_.revision), "%s", revision);
        server_info_.protocol_major = object["proto_major"] | 0;
        server_info_.protocol_minor = object["proto_minor"] | 0;
        server_info_.valid = true;
        return UpdateKind::Version;
    }

    void apply_time(JsonObjectConst object,
                    ship_data_model::GnssData<Real>& fix,
                    ship_data_model::GnssFixAccuracyData<Real>* accuracy,
                    uint64_t now_us) {
        const char* time = object["time"].as<const char*>();
        int32_t year = 0, month = 0, day = 0;
        double seconds = 0.0;
        if (parse_iso8601_utc(time, year, month, day, seconds)) {
            fix.date_year.set(year, now_us);
            fix.date_month.set(month, now_us);
            fix.date_day.set(day, now_us);
            fix.timestamp_s.set(static_cast<Real>(seconds), now_us);
            if (accuracy) accuracy->utc_time_s.set(static_cast<Real>(seconds), now_us);
        }
    }

    UpdateKind apply_tpv(JsonObjectConst object,
                         ship_data_model::DataModel<Real>& model,
                         uint64_t now_us,
                         ship_data_model::SensorSource source) {
        auto& fix = model.gnss.fix;
        auto& accuracy = model.gnss.fix_accuracy;
        const int32_t mode = object["mode"] | 0;
        const bool valid_fix = mode >= 2;
        fix.source.value = source;
        accuracy.source.value = source;
        fix.fix_type.set(mode, now_us);
        fix.fix_valid.set(valid_fix, now_us);
        fix.fix_quality.set(valid_fix ? 1 : 0, now_us);
        apply_time(object, fix, &accuracy, now_us);

        double value = 0.0;
        if (valid_fix) {
            if (read_number(object, "lat", value)) fix.fix_lat_deg.set(static_cast<Real>(value), now_us);
            if (read_number(object, "lon", value)) fix.fix_lon_deg.set(static_cast<Real>(value), now_us);
            if (read_number(object, "altMSL", value)) fix.fix_alt_msl_m.set(static_cast<Real>(value), now_us);
            if (read_number(object, "altHAE", value)) fix.fix_alt_hae_m.set(static_cast<Real>(value), now_us);
            if (fix.fix_alt_msl_m.valid && fix.fix_alt_hae_m.valid &&
                (fix.fix_alt_msl_m.last_update_us == now_us || fix.fix_alt_hae_m.last_update_us == now_us)) {
                fix.geoidal_separation_m.set(fix.fix_alt_hae_m.value - fix.fix_alt_msl_m.value, now_us);
            }
            if (read_number(object, "speed", value)) fix.speed_kn.set(static_cast<Real>(value * MetersPerSecondToKnots), now_us);
            if (read_number(object, "track", value)) fix.track_deg.set(static_cast<Real>(value), now_us);
            if (read_number(object, "climb", value)) fix.vertical_speed_m_s.set(static_cast<Real>(value), now_us);
        }

        double epx = 0.0, epy = 0.0;
        const bool has_epx = read_number(object, "epx", epx);
        const bool has_epy = read_number(object, "epy", epy);
        if (has_epx || has_epy) accuracy.horizontal_accuracy_m.set(static_cast<Real>(has_epx && has_epy ? (epx > epy ? epx : epy) : (has_epx ? epx : epy)), now_us);
        if (read_number(object, "epv", value)) accuracy.vertical_accuracy_m.set(static_cast<Real>(value), now_us);
        if (read_number(object, "eps", value)) accuracy.speed_accuracy_m_s.set(static_cast<Real>(value), now_us);
        if (read_number(object, "epd", value)) accuracy.track_accuracy_deg.set(static_cast<Real>(value), now_us);
        if (read_number(object, "ept", value)) accuracy.time_accuracy_s.set(static_cast<Real>(value), now_us);
        fix.last_update_us = now_us;
        accuracy.last_update_us = now_us;
        return UpdateKind::Fix | UpdateKind::Accuracy;
    }

    UpdateKind apply_sky(JsonObjectConst object,
                         ship_data_model::DataModel<Real>& model,
                         uint64_t now_us,
                         ship_data_model::SensorSource source) {
        auto& fix = model.gnss.fix;
        auto& sky = model.gnss.sky_view;
        auto& dop = model.gnss.dop_active_satellites;
        auto& accuracy = model.gnss.fix_accuracy;
        fix.source.value = source;
        sky.source.value = source;
        dop.source.value = source;
        accuracy.source.value = source;
        sky.clear_observations();

        JsonArrayConst satellites = object["satellites"].as<JsonArrayConst>();
        int32_t used_count = 0;
        size_t total = 0;
        for (JsonObjectConst satellite : satellites) {
            if (total < sky.capacity()) {
                auto& observation = sky.observations[total];
                int32_t integer_value = 0;
                double number = 0.0;
                if (read_number(satellite, "PRN", integer_value) || read_number(satellite, "svid", integer_value)) {
                    observation.satellite_id = static_cast<int16_t>(integer_value);
                    observation.satellite_id_valid = true;
                }
                if (read_number(satellite, "gnssid", integer_value)) observation.system_id = static_cast<int16_t>(integer_value);
                if (read_number(satellite, "sigid", integer_value)) observation.signal_id = static_cast<int16_t>(integer_value);
                if (read_number(satellite, "el", number)) { observation.elevation_deg = static_cast<Real>(number); observation.elevation_valid = true; }
                if (read_number(satellite, "az", number)) { observation.azimuth_true_deg = static_cast<Real>(number); observation.azimuth_valid = true; }
                if (read_number(satellite, "ss", number)) { observation.cn0_db_hz = static_cast<Real>(number); observation.cn0_valid = true; }
                observation.used = satellite["used"] | false;
                observation.healthy = (satellite["health"] | 0) == 1;
                if (observation.used) ++used_count;
                ++total;
            } else if (satellite["used"] | false) {
                ++used_count;
            }
        }
        sky.observation_count = static_cast<uint16_t>(total);
        sky.satellites_in_view.set(static_cast<int32_t>(satellites.size()), now_us);
        sky.satellites_used.set(used_count, now_us);
        fix.satellites_used.set(used_count, now_us);
        sky.complete = true;
        ++sky.sequence;
        sky.last_update_us = now_us;

        double value = 0.0;
#define GPSD_SET_DOP(json_name, field) if (read_number(object, json_name, value)) { dop.field.set(static_cast<Real>(value), now_us); accuracy.field.set(static_cast<Real>(value), now_us); }
        GPSD_SET_DOP("gdop", gdop)
        GPSD_SET_DOP("pdop", pdop)
        GPSD_SET_DOP("tdop", tdop)
        if (read_number(object, "hdop", value)) {
            const Real hdop = static_cast<Real>(value);
            dop.hdop.set(hdop, now_us);
            accuracy.hdop.set(hdop, now_us);
            fix.hdop.set(hdop, now_us);
        }
        GPSD_SET_DOP("vdop", vdop)
        if (read_number(object, "xdop", value)) { dop.east_dop.set(static_cast<Real>(value), now_us); accuracy.east_dop.set(static_cast<Real>(value), now_us); }
        if (read_number(object, "ydop", value)) { dop.north_dop.set(static_cast<Real>(value), now_us); accuracy.north_dop.set(static_cast<Real>(value), now_us); }
#undef GPSD_SET_DOP
        dop.last_update_us = now_us;
        accuracy.last_update_us = now_us;
        return UpdateKind::Sky | UpdateKind::Accuracy;
    }

    UpdateKind apply_gst(JsonObjectConst object,
                         ship_data_model::DataModel<Real>& model,
                         uint64_t now_us,
                         ship_data_model::SensorSource source) {
        auto& accuracy = model.gnss.fix_accuracy;
        accuracy.source.value = source;
        double value = 0.0;
        if (read_number(object, "major", value)) accuracy.semi_major_accuracy_m.set(static_cast<Real>(value), now_us);
        if (read_number(object, "minor", value)) accuracy.semi_minor_accuracy_m.set(static_cast<Real>(value), now_us);
        if (read_number(object, "orient", value)) accuracy.semi_major_orientation_deg.set(static_cast<Real>(value), now_us);
        double lat = 0.0, lon = 0.0;
        const bool has_lat = read_number(object, "lat", lat);
        const bool has_lon = read_number(object, "lon", lon);
        if (has_lat || has_lon) accuracy.horizontal_accuracy_m.set(static_cast<Real>(has_lat && has_lon ? (lat > lon ? lat : lon) : (has_lat ? lat : lon)), now_us);
        if (read_number(object, "alt", value)) accuracy.vertical_accuracy_m.set(static_cast<Real>(value), now_us);
        const char* time = object["time"].as<const char*>();
        int32_t year = 0, month = 0, day = 0;
        double seconds = 0.0;
        if (parse_iso8601_utc(time, year, month, day, seconds)) accuracy.utc_time_s.set(static_cast<Real>(seconds), now_us);
        accuracy.last_update_us = now_us;
        return UpdateKind::Accuracy;
    }

    const char* device_ = nullptr;
    bool include_sky_ = true;
    bool include_gst_ = true;
    char record_[MaxRecordLength + 1]{};
    size_t record_length_ = 0;
    bool discarding_ = false;
    SessionState state_ = SessionState::Disconnected;
    Diagnostics diagnostics_{};
    ServerInfo server_info_{};
};

} // namespace gpsd
