#pragma once

#include <stdint.h>
#include <stddef.h>

#include <ship_data_model.hpp>
#include <nmea0183_connector.hpp>

namespace signalk_mini {

using SourceId = uint16_t;

enum class ModelField : uint16_t {
    None = 0,
    NavigationGpsFixLatDeg,
    NavigationGpsFixLonDeg,
    NavigationGpsSpeedKn,
    NavigationGpsTrackDeg,
    ImuHeadingDeg,
    WindApparentDirectionDeg,
    WindApparentSpeedKn,
    WaterDepthM,
};

struct ModelChange {
    ModelField field = ModelField::None;
    SourceId source_id = 0;
    uint64_t timestamp_us = 0;
    uint64_t sequence = 0;
};

template<size_t Capacity>
class ModelChangeQueue {
public:
    bool push(const ModelChange& change) {
        if (count_ == Capacity) {
            tail_ = (tail_ + 1) % Capacity;
            --count_;
        }
        items_[head_] = change;
        head_ = (head_ + 1) % Capacity;
        ++count_;
        return true;
    }

    bool pop(ModelChange& change) {
        if (count_ == 0) return false;
        change = items_[tail_];
        tail_ = (tail_ + 1) % Capacity;
        --count_;
        return true;
    }

    size_t size() const { return count_; }

private:
    ModelChange items_[Capacity]{};
    size_t head_ = 0;
    size_t tail_ = 0;
    size_t count_ = 0;
};

template<typename Real, size_t QueueCapacity = 128>
class ModelStore {
public:
    using Model = ship_data_model::DataModel<Real>;

    Model& model() { return model_; }
    const Model& model() const { return model_; }

    ModelChangeQueue<QueueCapacity>& changes() { return changes_; }
    uint64_t sequence() const { return sequence_; }

    void mark_changed(ModelField field, SourceId source_id, uint64_t now_us) {
        ModelChange change;
        change.field = field;
        change.source_id = source_id;
        change.timestamp_us = now_us;
        change.sequence = ++sequence_;
        changes_.push(change);
    }

private:
    Model model_{};
    ModelChangeQueue<QueueCapacity> changes_{};
    uint64_t sequence_ = 0;
};

template<typename Real>
class Nmea0183Input {
public:
    explicit Nmea0183Input(ModelStore<Real>& store) : store_(store) {}

    bool feed_line(const char* line, SourceId source_id, uint64_t now_us) {
        nmea0183_connector::NmeaSentence sentence;
        if (!parser_.parse_line(line, sentence)) return false;

        const bool applied = rx_.apply_sentence(
            sentence,
            store_.model(),
            now_us,
            ship_data_model::SensorSource::serial
        );
        if (!applied) return false;

        mark_changed_from_sentence(sentence, source_id, now_us);
        return true;
    }

    const char* last_error() const { return rx_.last_error(); }

private:
    void mark_changed_from_sentence(const nmea0183_connector::NmeaSentence& sentence, SourceId source_id, uint64_t now_us) {
        if (nmea0183_connector::sentence_is(sentence, "RMC") ||
            nmea0183_connector::sentence_is(sentence, "GGA") ||
            nmea0183_connector::sentence_is(sentence, "GLL")) {
            store_.mark_changed(ModelField::NavigationGpsFixLatDeg, source_id, now_us);
            store_.mark_changed(ModelField::NavigationGpsFixLonDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "RMC") ||
            nmea0183_connector::sentence_is(sentence, "VTG")) {
            store_.mark_changed(ModelField::NavigationGpsSpeedKn, source_id, now_us);
            store_.mark_changed(ModelField::NavigationGpsTrackDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "HDT") ||
            nmea0183_connector::sentence_is(sentence, "HDM") ||
            nmea0183_connector::sentence_is(sentence, "HDG")) {
            store_.mark_changed(ModelField::ImuHeadingDeg, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "MWV") ||
            nmea0183_connector::sentence_is(sentence, "MWD") ||
            nmea0183_connector::sentence_is(sentence, "VWR")) {
            store_.mark_changed(ModelField::WindApparentDirectionDeg, source_id, now_us);
            store_.mark_changed(ModelField::WindApparentSpeedKn, source_id, now_us);
        }
        if (nmea0183_connector::sentence_is(sentence, "DBT") ||
            nmea0183_connector::sentence_is(sentence, "DPT")) {
            store_.mark_changed(ModelField::WaterDepthM, source_id, now_us);
        }
    }

    ModelStore<Real>& store_;
    nmea0183_connector::Nmea0183StreamParser parser_;
    nmea0183_connector::Nmea0183RxConnector<Real> rx_;
};

template<typename Real>
constexpr Real pi_v() { return static_cast<Real>(3.1415926535897932384626433832795L); }

template<typename Real>
constexpr Real deg_to_rad(Real deg) { return deg * pi_v<Real>() / static_cast<Real>(180); }

template<typename Real>
constexpr Real knots_to_mps(Real knots) { return knots * static_cast<Real>(0.51444444444444444444L); }

template<typename Real>
struct SignalKMappedValue {
    const char* path = nullptr;
    Real number{};
};

template<typename Real>
class SignalKMapper {
public:
    bool map_change(const ship_data_model::DataModel<Real>& model, const ModelChange& change, SignalKMappedValue<Real>& out) const {
        switch (change.field) {
        case ModelField::NavigationGpsFixLatDeg:
            out.path = "navigation.position.value.latitude";
            out.number = model.navigation.gps.fix_lat_deg.value;
            return model.navigation.gps.fix_lat_deg.valid;
        case ModelField::NavigationGpsFixLonDeg:
            out.path = "navigation.position.value.longitude";
            out.number = model.navigation.gps.fix_lon_deg.value;
            return model.navigation.gps.fix_lon_deg.valid;
        case ModelField::NavigationGpsSpeedKn:
            out.path = "navigation.speedOverGround";
            out.number = knots_to_mps<Real>(model.navigation.gps.speed_kn.value);
            return model.navigation.gps.speed_kn.valid;
        case ModelField::NavigationGpsTrackDeg:
            out.path = "navigation.courseOverGroundTrue";
            out.number = deg_to_rad<Real>(model.navigation.gps.track_deg.value);
            return model.navigation.gps.track_deg.valid;
        case ModelField::ImuHeadingDeg:
            out.path = "navigation.headingTrue";
            out.number = deg_to_rad<Real>(model.imu.heading_deg.value);
            return model.imu.heading_deg.valid;
        case ModelField::WindApparentDirectionDeg:
            out.path = "environment.wind.angleApparent";
            out.number = deg_to_rad<Real>(model.wind.apparent.direction_deg.value);
            return model.wind.apparent.direction_deg.valid;
        case ModelField::WindApparentSpeedKn:
            out.path = "environment.wind.speedApparent";
            out.number = knots_to_mps<Real>(model.wind.apparent.speed_kn.value);
            return model.wind.apparent.speed_kn.valid;
        case ModelField::WaterDepthM:
            out.path = "environment.depth.belowTransducer";
            out.number = model.water.depth_m.value;
            return model.water.depth_m.valid;
        default:
            return false;
        }
    }
};

template<typename Real = float>
class SignalKMiniApp {
public:
    using Store = ModelStore<Real>;
    Store& store() { return store_; }
    Nmea0183Input<Real>& nmea0183() { return nmea0183_; }
private:
    Store store_{};
    Nmea0183Input<Real> nmea0183_{store_};
};

} // namespace signalk_mini
