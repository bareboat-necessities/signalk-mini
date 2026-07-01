#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <ship_data_model.hpp>
#include <nmea0183_connector.hpp>

namespace signalk_mini {

enum class ModelFieldId : uint16_t {
    unknown,
    navigation_fix_lat_deg,
    navigation_fix_lon_deg,
    navigation_gps_speed_kn,
    navigation_gps_track_deg,
    navigation_heading_deg,
    wind_apparent_direction_deg,
    wind_apparent_speed_kn,
    wind_true_direction_deg,
    wind_true_speed_kn,
    water_depth_m,
    water_depth_offset_m
};

struct ModelChange {
    ModelFieldId field = ModelFieldId::unknown;
    uint16_t source_id = 0;
    uint64_t timestamp_us = 0;
    uint64_t sequence = 0;
};

template<size_t N>
class ModelChangeQueue {
public:
    bool push(const ModelChange& change) {
        if (count_ == N) {
            tail_ = (tail_ + 1u) % N;
            --count_;
        }
        queue_[head_] = change;
        head_ = (head_ + 1u) % N;
        ++count_;
        return true;
    }

    bool pop(ModelChange& out) {
        if (count_ == 0) return false;
        out = queue_[tail_];
        tail_ = (tail_ + 1u) % N;
        --count_;
        return true;
    }

    size_t size() const { return count_; }

private:
    ModelChange queue_[N]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
};

template<typename Real, size_t ChangeCapacity = 128>
class ModelStore {
public:
    ship_data_model::DataModel<Real>& model() { return model_; }
    const ship_data_model::DataModel<Real>& model() const { return model_; }

    ModelChangeQueue<ChangeCapacity>& changes() { return changes_; }
    uint64_t sequence() const { return sequence_; }

    void mark_changed(ModelFieldId field, uint16_t source_id, uint64_t now_us) {
        changes_.push(ModelChange{field, source_id, now_us, ++sequence_});
    }

private:
    ship_data_model::DataModel<Real> model_{};
    ModelChangeQueue<ChangeCapacity> changes_{};
    uint64_t sequence_ = 0;
};

template<typename Real>
constexpr Real pi_v() {
    return static_cast<Real>(3.1415926535897932384626433832795L);
}

template<typename Real>
constexpr Real deg_to_rad(Real deg) {
    return deg * pi_v<Real>() / static_cast<Real>(180);
}

template<typename Real>
constexpr Real knots_to_mps(Real knots) {
    return knots * static_cast<Real>(0.51444444444444444444L);
}

template<typename Real>
struct SignalKMappedValue {
    const char* path = nullptr;
    Real number{};
};

template<typename Real>
class SignalKMapper {
public:
    bool map_change(
        const ship_data_model::DataModel<Real>& model,
        const ModelChange& change,
        SignalKMappedValue<Real>& out
    ) const {
        switch (change.field) {
        case ModelFieldId::navigation_gps_speed_kn:
            if (!model.navigation.gps.speed_kn.valid) return false;
            out.path = "navigation.speedOverGround";
            out.number = knots_to_mps<Real>(model.navigation.gps.speed_kn.value);
            return true;
        case ModelFieldId::navigation_gps_track_deg:
            if (!model.navigation.gps.track_deg.valid) return false;
            out.path = "navigation.courseOverGroundTrue";
            out.number = deg_to_rad<Real>(model.navigation.gps.track_deg.value);
            return true;
        case ModelFieldId::navigation_heading_deg:
            if (!model.imu.heading_deg.valid) return false;
            out.path = "navigation.headingTrue";
            out.number = deg_to_rad<Real>(model.imu.heading_deg.value);
            return true;
        case ModelFieldId::wind_apparent_direction_deg:
            if (!model.wind.apparent.direction_deg.valid) return false;
            out.path = "environment.wind.angleApparent";
            out.number = deg_to_rad<Real>(model.wind.apparent.direction_deg.value);
            return true;
        case ModelFieldId::wind_apparent_speed_kn:
            if (!model.wind.apparent.speed_kn.valid) return false;
            out.path = "environment.wind.speedApparent";
            out.number = knots_to_mps<Real>(model.wind.apparent.speed_kn.value);
            return true;
        case ModelFieldId::wind_true_direction_deg:
            if (!model.wind.truewind.direction_deg.valid) return false;
            out.path = "environment.wind.angleTrueWater";
            out.number = deg_to_rad<Real>(model.wind.truewind.direction_deg.value);
            return true;
        case ModelFieldId::wind_true_speed_kn:
            if (!model.wind.truewind.speed_kn.valid) return false;
            out.path = "environment.wind.speedTrue";
            out.number = knots_to_mps<Real>(model.wind.truewind.speed_kn.value);
            return true;
        case ModelFieldId::water_depth_m:
            if (!model.water.depth_m.valid) return false;
            out.path = "environment.depth.belowTransducer";
            out.number = model.water.depth_m.value;
            return true;
        default:
            return false;
        }
    }
};

template<typename Real>
class Nmea0183Input {
public:
    explicit Nmea0183Input(ModelStore<Real>& store) : store_(store) {}

    bool feed_line(const char* line, uint16_t source_id, uint64_t now_us) {
        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(line, sentence)) return false;
        Snapshot before(store_.model());
        bool ok = rx_.apply_sentence(sentence, store_.model(), now_us, ship_data_model::SensorSource::serial);
        if (ok) record_changes(before, source_id, now_us);
        return ok;
    }

    bool feed_byte(char c, uint16_t source_id, uint64_t now_us) {
        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.push(c, sentence)) return false;
        Snapshot before(store_.model());
        bool ok = rx_.apply_sentence(sentence, store_.model(), now_us, ship_data_model::SensorSource::serial);
        if (ok) record_changes(before, source_id, now_us);
        return ok;
    }

    const char* last_error() const {
        const char* rx_error = rx_.last_error();
        if (rx_error && rx_error[0]) return rx_error;
        return parser_.last_error();
    }

private:
    using Model = ship_data_model::DataModel<Real>;

    struct StampState {
        StampState() = default;
        StampState(bool v, uint64_t t) : valid(v), last_update_us(t) {}
        bool valid = false;
        uint64_t last_update_us = 0;
    };

    struct Snapshot {
        explicit Snapshot(const Model& m)
            : lat(m.navigation.gps.fix_lat_deg.valid, m.navigation.gps.fix_lat_deg.last_update_us),
              lon(m.navigation.gps.fix_lon_deg.valid, m.navigation.gps.fix_lon_deg.last_update_us),
              speed(m.navigation.gps.speed_kn.valid, m.navigation.gps.speed_kn.last_update_us),
              track(m.navigation.gps.track_deg.valid, m.navigation.gps.track_deg.last_update_us),
              heading(m.imu.heading_deg.valid, m.imu.heading_deg.last_update_us),
              wind_dir(m.wind.apparent.direction_deg.valid, m.wind.apparent.direction_deg.last_update_us),
              wind_speed(m.wind.apparent.speed_kn.valid, m.wind.apparent.speed_kn.last_update_us),
              true_wind_dir(m.wind.truewind.direction_deg.valid, m.wind.truewind.direction_deg.last_update_us),
              true_wind_speed(m.wind.truewind.speed_kn.valid, m.wind.truewind.speed_kn.last_update_us),
              depth(m.water.depth_m.valid, m.water.depth_m.last_update_us),
              depth_offset(m.water.depth_offset_m.valid, m.water.depth_offset_m.last_update_us) {}

        StampState lat;
        StampState lon;
        StampState speed;
        StampState track;
        StampState heading;
        StampState wind_dir;
        StampState wind_speed;
        StampState true_wind_dir;
        StampState true_wind_speed;
        StampState depth;
        StampState depth_offset;
    };

    static bool changed(const StampState& before, bool valid, uint64_t stamp) {
        return valid && (!before.valid || before.last_update_us != stamp);
    }

    void record_changes(const Snapshot& before, uint16_t source_id, uint64_t now_us) {
        const Model& m = store_.model();
        if (changed(before.lat, m.navigation.gps.fix_lat_deg.valid, m.navigation.gps.fix_lat_deg.last_update_us)) store_.mark_changed(ModelFieldId::navigation_fix_lat_deg, source_id, now_us);
        if (changed(before.lon, m.navigation.gps.fix_lon_deg.valid, m.navigation.gps.fix_lon_deg.last_update_us)) store_.mark_changed(ModelFieldId::navigation_fix_lon_deg, source_id, now_us);
        if (changed(before.speed, m.navigation.gps.speed_kn.valid, m.navigation.gps.speed_kn.last_update_us)) store_.mark_changed(ModelFieldId::navigation_gps_speed_kn, source_id, now_us);
        if (changed(before.track, m.navigation.gps.track_deg.valid, m.navigation.gps.track_deg.last_update_us)) store_.mark_changed(ModelFieldId::navigation_gps_track_deg, source_id, now_us);
        if (changed(before.heading, m.imu.heading_deg.valid, m.imu.heading_deg.last_update_us)) store_.mark_changed(ModelFieldId::navigation_heading_deg, source_id, now_us);
        if (changed(before.wind_dir, m.wind.apparent.direction_deg.valid, m.wind.apparent.direction_deg.last_update_us)) store_.mark_changed(ModelFieldId::wind_apparent_direction_deg, source_id, now_us);
        if (changed(before.wind_speed, m.wind.apparent.speed_kn.valid, m.wind.apparent.speed_kn.last_update_us)) store_.mark_changed(ModelFieldId::wind_apparent_speed_kn, source_id, now_us);
        if (changed(before.true_wind_dir, m.wind.truewind.direction_deg.valid, m.wind.truewind.direction_deg.last_update_us)) store_.mark_changed(ModelFieldId::wind_true_direction_deg, source_id, now_us);
        if (changed(before.true_wind_speed, m.wind.truewind.speed_kn.valid, m.wind.truewind.speed_kn.last_update_us)) store_.mark_changed(ModelFieldId::wind_true_speed_kn, source_id, now_us);
        if (changed(before.depth, m.water.depth_m.valid, m.water.depth_m.last_update_us)) store_.mark_changed(ModelFieldId::water_depth_m, source_id, now_us);
        if (changed(before.depth_offset, m.water.depth_offset_m.valid, m.water.depth_offset_m.last_update_us)) store_.mark_changed(ModelFieldId::water_depth_offset_m, source_id, now_us);
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
};

} // namespace signalk_mini
