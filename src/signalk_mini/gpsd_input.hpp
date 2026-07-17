#pragma once

#include <stddef.h>
#include <stdint.h>

#include <gpsd.hpp>

#include "memory_profile.hpp"
#include "model_store.hpp"
#include "types.hpp"

namespace signalk_mini {

template<typename Real, size_t QueueCapacity = DefaultModelChangeQueueCapacity>
class GpsdInput {
public:
    explicit GpsdInput(ModelStore<Real, QueueCapacity>& store,
                       const char* device = nullptr,
                       bool include_sky = true,
                       bool include_gst = true)
        : store_(store), client_(device, include_sky, include_gst) {}

    bool feed_octets(const uint8_t* bytes,
                     size_t length,
                     SourceId source_id,
                     uint64_t now_us,
                     ship_data_model::SensorSource source = ship_data_model::SensorSource::gpsd) {
        const gpsd::UpdateKind updates = client_.accept_octets(bytes, length, store_.model(), now_us, source);
        mark_changes(updates, source_id, now_us);
        return updates != gpsd::UpdateKind::None;
    }

    void on_connected() { client_.on_connected(); }
    void on_disconnected() { client_.on_disconnected(); }
    gpsd::Client<Real>& client() { return client_; }
    const gpsd::Client<Real>& client() const { return client_; }

private:
    void mark(ModelField field, SourceId source_id, uint64_t now_us) {
        store_.mark_changed(field, source_id, now_us);
    }

    template<typename T>
    void mark_if_updated(const ship_data_model::Stamped<T>& value,
                         ModelField field,
                         SourceId source_id,
                         uint64_t now_us) {
        if (value.valid && value.last_update_us == now_us) mark(field, source_id, now_us);
    }

    void mark_fix(SourceId source_id, uint64_t now_us) {
        const auto& fix = store_.model().gnss.fix;
        const auto& accuracy = store_.model().gnss.fix_accuracy;
        mark_if_updated(fix.fix_lat_deg, ModelField::GnssFixLatDeg, source_id, now_us);
        mark_if_updated(fix.fix_lon_deg, ModelField::GnssFixLonDeg, source_id, now_us);
        mark_if_updated(fix.speed_kn, ModelField::GnssSpeedKn, source_id, now_us);
        mark_if_updated(fix.track_deg, ModelField::GnssTrackDeg, source_id, now_us);
        mark_if_updated(fix.vertical_speed_m_s, ModelField::GnssVerticalSpeedMs, source_id, now_us);
        mark_if_updated(fix.timestamp_s, ModelField::GnssTimestampS, source_id, now_us);
        mark_if_updated(fix.date_day, ModelField::GnssDateDay, source_id, now_us);
        mark_if_updated(fix.date_month, ModelField::GnssDateMonth, source_id, now_us);
        mark_if_updated(fix.date_year, ModelField::GnssDateYear, source_id, now_us);
        mark_if_updated(fix.fix_quality, ModelField::GnssFixQuality, source_id, now_us);
        mark_if_updated(fix.fix_type, ModelField::GnssFixType, source_id, now_us);
        mark_if_updated(fix.fix_valid, ModelField::GnssFixValid, source_id, now_us);
        mark_if_updated(fix.fix_alt_msl_m, ModelField::GnssFixAltitudeMslM, source_id, now_us);
        mark_if_updated(fix.fix_alt_hae_m, ModelField::GnssFixAltitudeHaeM, source_id, now_us);
        mark_if_updated(fix.geoidal_separation_m, ModelField::GnssGeoidalSeparationM, source_id, now_us);
        mark_if_updated(accuracy.horizontal_accuracy_m, ModelField::GnssFixAccuracyHorizontalM, source_id, now_us);
        mark_if_updated(accuracy.vertical_accuracy_m, ModelField::GnssFixAccuracyVerticalM, source_id, now_us);
        mark_if_updated(accuracy.speed_accuracy_m_s, ModelField::GnssFixAccuracySpeedMs, source_id, now_us);
        mark_if_updated(accuracy.track_accuracy_deg, ModelField::GnssFixAccuracyTrackDeg, source_id, now_us);
        mark_if_updated(accuracy.time_accuracy_s, ModelField::GnssFixAccuracyTimeS, source_id, now_us);
    }

    void mark_accuracy(SourceId source_id, uint64_t now_us) {
        const auto& accuracy = store_.model().gnss.fix_accuracy;
        const auto& dop = store_.model().gnss.dop_active_satellites;
        mark_if_updated(accuracy.horizontal_accuracy_m, ModelField::GnssFixAccuracyHorizontalM, source_id, now_us);
        mark_if_updated(accuracy.vertical_accuracy_m, ModelField::GnssFixAccuracyVerticalM, source_id, now_us);
        mark_if_updated(accuracy.speed_accuracy_m_s, ModelField::GnssFixAccuracySpeedMs, source_id, now_us);
        mark_if_updated(accuracy.track_accuracy_deg, ModelField::GnssFixAccuracyTrackDeg, source_id, now_us);
        mark_if_updated(accuracy.time_accuracy_s, ModelField::GnssFixAccuracyTimeS, source_id, now_us);
        mark_if_updated(accuracy.pdop, ModelField::GnssFixAccuracyPdop, source_id, now_us);
        mark_if_updated(accuracy.vdop, ModelField::GnssFixAccuracyVdop, source_id, now_us);
        mark_if_updated(dop.pdop, ModelField::GnssDopActivePdop, source_id, now_us);
        mark_if_updated(dop.vdop, ModelField::GnssDopActiveVdop, source_id, now_us);
    }

    void mark_sky(SourceId source_id, uint64_t now_us) {
        const auto& sky = store_.model().gnss.sky_view;
        mark_if_updated(sky.satellites_in_view, ModelField::GnssSatellitesInView, source_id, now_us);
        if (sky.last_update_us == now_us && sky.observation_count > 0) {
            mark(ModelField::GnssSatellitePrn0, source_id, now_us);
            mark(ModelField::GnssSatelliteElevationDeg0, source_id, now_us);
            mark(ModelField::GnssSatelliteAzimuthDeg0, source_id, now_us);
            mark(ModelField::GnssSatelliteSnrDb0, source_id, now_us);
        }
    }

    void mark_changes(gpsd::UpdateKind updates, SourceId source_id, uint64_t now_us) {
        if (gpsd::has_update(updates, gpsd::UpdateKind::Fix)) mark_fix(source_id, now_us);
        if (gpsd::has_update(updates, gpsd::UpdateKind::Accuracy)) mark_accuracy(source_id, now_us);
        if (gpsd::has_update(updates, gpsd::UpdateKind::Sky)) mark_sky(source_id, now_us);
    }

    ModelStore<Real, QueueCapacity>& store_;
    gpsd::Client<Real> client_;
};

} // namespace signalk_mini
