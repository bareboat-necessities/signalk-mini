#pragma once

#include <stdint.h>
#include <stddef.h>

namespace ship_data_model {

enum class SensorSource : uint8_t {
    gpsd,
    servo,
    serial,
    tcp,
    signalk,
    water_wind,
    gps_wind,
    none
};

inline int source_priority(SensorSource source) {
    switch (source) {
    case SensorSource::gpsd:       return 1;
    case SensorSource::servo:      return 1;
    case SensorSource::serial:     return 2;
    case SensorSource::tcp:        return 3;
    case SensorSource::signalk:    return 4;
    case SensorSource::water_wind: return 5;
    case SensorSource::gps_wind:   return 6;
    case SensorSource::none:       return 7;
    }
    return 7;
}

template<typename T>
struct Stamped {
    T value{};
    bool valid = false;
    uint64_t last_update_us = 0;

    void set(const T& v, uint64_t now_us) {
        value = v;
        valid = true;
        last_update_us = now_us;
    }

    bool stale(uint64_t now_us, uint64_t timeout_us) const {
        return !valid || now_us - last_update_us > timeout_us;
    }
};

template<typename T>
struct Setting {
    T value{};
};

template<typename Real = float>
struct NmeaTextRecordData {
    Setting<SensorSource> source;
    char id[24] = {0};
    char code[16] = {0};
    char value[16] = {0};
    char text[72] = {0};
    Stamped<int32_t> field_count;
    uint64_t last_update_us = 0;
};

template<typename T>
struct RangeSetting {
    T value{};
    T min{};
    T max{};
};

template<typename T>
struct TimedCommand {
    T value{};
    bool valid = false;
    uint64_t last_update_us = 0;
    uint64_t last_external_set_us = 0;
    bool use_period = true;

    void set_external(const T& v, uint64_t now_us) {
        value = v;
        valid = true;
        last_update_us = now_us;
        last_external_set_us = now_us;
        use_period = false;
    }

    void set_internal_command(const T& v, uint64_t now_us) {
        value = v;
        valid = true;
        last_update_us = now_us;
        use_period = true;
    }
};

template<typename Real = float>
struct ValuePublicationState {
    Stamped<uint32_t> published_value_count;
    Stamped<uint32_t> published_byte_count;
};

} // namespace ship_data_model
