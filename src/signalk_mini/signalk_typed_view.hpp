#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.hpp"
#include "model_store.hpp"
#include "signalk_identity.hpp"
#include "signalk_mapper.hpp"

namespace signalk_mini {

template<typename Real>
class SignalKTypedModelView {
public:
    SignalKTypedModelView(const ModelStore<Real>& store,
                          const SignalKMiniConfig& config,
                          uint64_t now_us = 0)
        : store_(store), config_(config), now_us_(now_us) {}

    const ship_data_model::DataModel<Real>& model() const { return store_.model(); }

    bool current_value(ModelField field, SignalKMappedValue<Real>& out) const {
        if (!mapper_.map_field(store_.model(), field, out) || !out.path) return false;
        if (out.last_update_us == 0 && strcmp(out.path, signalk_path::SteeringAutopilotState) == 0) {
            out.last_update_us = store_.autopilot_state_last_update_us();
        }
        if (out.last_update_us == 0) out.last_update_us = aggregate_last_update_us(field);
        if (out.last_update_us == 0 && !store_.has_value(field)) return false;
        if (!is_current(out)) return false;
        out.source_id = source_for(field, out);
        return true;
    }

    template<typename Callback>
    void for_each_current_value(Callback callback) const {
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            const ModelField field = static_cast<ModelField>(i);
            SignalKMappedValue<Real> mapped;
            if (!current_value(field, mapped)) continue;
            if (!is_selected_path_candidate(i, mapped)) continue;
            callback(field, mapped);
        }
    }

    const char* context() const {
        return config_.identity.self && config_.identity.self[0]
            ? config_.identity.self
            : "vessels.urn:mrn:signalk:uuid:00000000-0000-4000-8000-000000000001";
    }

    const char* vessel_key() const {
        const char* key = signalk_vessel_key(context());
        return key ? key : context();
    }

    SourceId current_value_source(ModelField field) const {
        SignalKMappedValue<Real> mapped;
        return current_value(field, mapped) ? mapped.source_id : store_.source_id_for(field);
    }

    bool field_is_current(ModelField field, uint64_t last_update_us) const {
        return store_.has_value(field) && timestamp_is_current(last_update_us);
    }

    template<typename Target>
    bool ais_target_is_current(const Target& target) const {
        uint64_t last_update_us = target.last_seen_us;
        if (target.last_position_update_us > last_update_us) last_update_us = target.last_position_update_us;
        if (target.last_static_update_us > last_update_us) last_update_us = target.last_static_update_us;
        return timestamp_is_current(last_update_us);
    }

    const char* source_label(SourceId source_id) const {
        if (source_id >= FirstConnectorSourceId && config_.connectors) {
            const size_t index = static_cast<size_t>(source_id - FirstConnectorSourceId);
            if (index < config_.connector_count) {
                const char* label = config_.connectors[index].label;
                if (label && label[0]) return label;
            }
        }
        if (config_.publisher.source_label && config_.publisher.source_label[0]) return config_.publisher.source_label;
        return config_.identity.server_name && config_.identity.server_name[0]
            ? config_.identity.server_name
            : "signalk-mini";
    }

    bool source_key(SourceId source_id, char* dst, size_t dst_size) const {
        if (!dst || dst_size == 0) return false;
        int len = 0;
        if (source_id >= FirstConnectorSourceId) {
            len = snprintf(dst, dst_size, "connector%u",
                           static_cast<unsigned>(source_id - FirstConnectorSourceId));
        } else {
            len = snprintf(dst, dst_size, "signalk-mini");
        }
        return len > 0 && static_cast<size_t>(len) < dst_size;
    }

private:
    static uint64_t latest_timestamp(uint64_t a, uint64_t b) { return a > b ? a : b; }

    bool timestamp_is_current(uint64_t last_update_us) const {
        const uint32_t timeout_ms = config_.publisher.current_value_timeout_ms;
        if (timeout_ms == 0 || now_us_ == 0 || last_update_us == 0 || now_us_ <= last_update_us) return true;
        const uint64_t age_us = now_us_ - last_update_us;
        return age_us <= static_cast<uint64_t>(timeout_ms) * 1000ULL;
    }

    bool is_current(const SignalKMappedValue<Real>& mapped) const {
        return timestamp_is_current(mapped.last_update_us);
    }

    uint64_t aggregate_last_update_us(ModelField field) const {
        const auto& model = store_.model();
        switch (field) {
        case ModelField::GnssSatellitePrn0:
        case ModelField::GnssSatelliteElevationDeg0:
        case ModelField::GnssSatelliteAzimuthDeg0:
        case ModelField::GnssSatelliteSnrDb0:
            return latest_timestamp(model.gnss.sky_view.last_update_us,
                                    model.gnss.satellites_in_view.last_update_us);
        case ModelField::RouteApbArrivalCircleEntered:
        case ModelField::RouteApbPerpendicularPassed:
        case ModelField::RouteApbDestinationId:
            return model.route.apb.last_update_us;
        case ModelField::RouteRmbArrived:
        case ModelField::RouteRmbDestinationId:
            return model.route.rmb.last_update_us;
        case ModelField::RouteWaypointToId:
        case ModelField::RouteWaypointFromId:
            return model.route.waypoint.last_update_us;
        case ModelField::RouteWaypointArrivalCircleEntered:
        case ModelField::RouteWaypointPerpendicularPassed:
        case ModelField::RouteWaypointArrivalId:
            return model.route.waypoint_arrival.last_update_us;
        case ModelField::AisSafetyObject:
            return latest_timestamp(model.ais.safety_text.last_update_us,
                                    model.ais.addressed_safety.last_update_us);
        case ModelField::AisDataLinkStatusObject:
            return model.ais.data_link_status.last_update_us;
        case ModelField::DscStructuredNotification: {
            uint64_t timestamp = model.comm.dsc.latest_call.last_update_us;
            timestamp = latest_timestamp(timestamp, model.notifications.dsc.distress.last_update_us);
            timestamp = latest_timestamp(timestamp, model.notifications.dsc.urgency.last_update_us);
            return latest_timestamp(timestamp, model.notifications.dsc.safety.last_update_us);
        }
        case ModelField::InmarsatSafetyNetStructuredNotification:
            return latest_timestamp(model.notifications.inmarsat.safetynet.latest_message.last_update_us,
                                    model.notifications.inmarsat.safetynet.message_count.last_update_us);
        case ModelField::NavtexStructuredNotification: {
            uint64_t timestamp = model.notifications.navtex.received.last_update_us;
            timestamp = latest_timestamp(timestamp, model.notifications.navtex.receiver_mask.last_update_us);
            return latest_timestamp(timestamp, model.notifications.navtex.history.count.last_update_us);
        }
        case ModelField::AlertStructuredNotification: {
            const auto& alerts = model.notifications.alerts;
            uint64_t timestamp = alerts.acknowledgement.last_update_us;
            timestamp = latest_timestamp(timestamp, alerts.acknowledgement_detail.last_update_us);
            timestamp = latest_timestamp(timestamp, alerts.condition.last_update_us);
            timestamp = latest_timestamp(timestamp, alerts.cyclic_list.last_update_us);
            timestamp = latest_timestamp(timestamp, alerts.alert_report.last_update_us);
            timestamp = latest_timestamp(timestamp, alerts.alarm_state.last_update_us);
            timestamp = latest_timestamp(timestamp, alerts.command_refused.last_update_us);
            timestamp = latest_timestamp(timestamp, alerts.heartbeat.last_update_us);
            return latest_timestamp(timestamp, alerts.fire.last_update_us);
        }
        case ModelField::MobStructuredNotification:
            return model.notifications.special.smv.last_update_us;
        case ModelField::LegacyCommObject: {
            const auto& comm = model.comm;
            uint64_t timestamp = comm.radio_frequency_set.last_update_us;
            timestamp = latest_timestamp(timestamp, comm.beacon_control.last_update_us);
            timestamp = latest_timestamp(timestamp, comm.beacon_status.last_update_us);
            timestamp = latest_timestamp(timestamp, comm.return_link_message.last_update_us);
            timestamp = latest_timestamp(timestamp, comm.equipment.control_command.last_update_us);
            timestamp = latest_timestamp(timestamp, comm.equipment.control_operation.last_update_us);
            timestamp = latest_timestamp(timestamp, comm.equipment.control_response.last_update_us);
            timestamp = latest_timestamp(timestamp, comm.equipment.display_control.last_update_us);
            return latest_timestamp(timestamp, comm.equipment.door_status.last_update_us);
        }
        case ModelField::NotificationText:
            return model.notifications.messages.text.last_update_us;
        case ModelField::NotificationEvent:
            return model.notifications.messages.event.last_update_us;
        case ModelField::NotificationEventLog:
            return model.notifications.messages.event_log.last_update_us;
        default:
            return 0;
        }
    }

    bool is_selected_path_candidate(size_t current_index, const SignalKMappedValue<Real>& current) const {
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            if (i == current_index) continue;
            SignalKMappedValue<Real> candidate;
            if (!current_value(static_cast<ModelField>(i), candidate)) continue;
            if (candidate.path != current.path && strcmp(candidate.path, current.path) != 0) continue;
            if (candidate.last_update_us > current.last_update_us) return false;
            if (candidate.last_update_us == current.last_update_us && i < current_index) return false;
        }
        return true;
    }

    SourceId newest_source(ModelField a, uint64_t ta,
                           ModelField b, uint64_t tb,
                           ModelField c = ModelField::None, uint64_t tc = 0,
                           ModelField d = ModelField::None, uint64_t td = 0) const {
        ModelField selected = a;
        uint64_t timestamp = ta;
        if (tb > timestamp) { selected = b; timestamp = tb; }
        if (tc > timestamp) { selected = c; timestamp = tc; }
        if (td > timestamp) { selected = d; }
        return store_.source_id_for(selected);
    }

    SourceId source_for(ModelField field, const SignalKMappedValue<Real>& mapped) const {
        const auto& model = store_.model();
        switch (mapped.object_kind) {
        case SignalKObjectKind::Position:
            return newest_source(ModelField::GnssFixLatDeg, model.gnss.fix.fix_lat_deg.last_update_us,
                                 ModelField::GnssFixLonDeg, model.gnss.fix.fix_lon_deg.last_update_us,
                                 ModelField::GnssFixAltitudeHaeM, model.gnss.fix.fix_alt_hae_m.last_update_us);
        case SignalKObjectKind::Attitude:
            return newest_source(ModelField::ImuPitchDeg, model.ins.imu.pitch_deg.last_update_us,
                                 ModelField::ImuRollDeg, model.ins.imu.roll_deg.last_update_us);
        case SignalKObjectKind::DateTime:
            return newest_source(ModelField::GnssTimestampS, model.gnss.fix.timestamp_s.last_update_us,
                                 ModelField::GnssDateDay, model.gnss.fix.date_day.last_update_us,
                                 ModelField::GnssDateMonth, model.gnss.fix.date_month.last_update_us,
                                 ModelField::GnssDateYear, model.gnss.fix.date_year.last_update_us);
        case SignalKObjectKind::Current:
            return newest_source(ModelField::SeaCurrentSpeedKn, model.sea.current_speed_kn.last_update_us,
                                 ModelField::SeaCurrentDirectionDeg, model.sea.current_direction_deg.last_update_us,
                                 ModelField::SeaCurrentDirectionMagneticDeg, model.sea.current_direction_magnetic_deg.last_update_us);
        default:
            if (mapped.path && strcmp(mapped.path, signalk_path::SteeringAutopilotState) == 0) {
                return store_.autopilot_state_source_id();
            }
            return store_.source_id_for(field);
        }
    }

    const ModelStore<Real>& store_;
    const SignalKMiniConfig& config_;
    uint64_t now_us_ = 0;
    SignalKMapper<Real> mapper_;
};

} // namespace signalk_mini
