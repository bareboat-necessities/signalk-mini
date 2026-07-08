#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <ArduinoJson.h>
#include <ship_data_model.hpp>
#include "signalk_mapper.hpp"
#include "units.hpp"

namespace signalk_mini {

template<typename Real>
class SignalKDeltaWriter {
public:
    int write_number(char* dst, size_t dst_size, const char* source_label, const char* path, Real value) const {
        if (!dst || dst_size == 0 || !path) return 0;

        JsonDocument doc;
        JsonArray updates = doc["updates"].to<JsonArray>();
        JsonObject update = updates.add<JsonObject>();
        JsonObject source = update["source"].to<JsonObject>();
        source["label"] = source_label ? source_label : "signalk-mini";

        JsonArray values = update["values"].to<JsonArray>();
        JsonObject item = values.add<JsonObject>();
        item["path"] = path;
        item["value"] = value;

        return serialize_delta(dst, dst_size, doc);
    }

    int write_mapped(char* dst, size_t dst_size, const char* source_label, const SignalKMappedValue<Real>& value) const {
        if (value.kind == SignalKMappedValueKind::Object) return 0;
        return write_scalar_mapped(dst, dst_size, source_label, value);
    }

    int write_mapped(char* dst,
                     size_t dst_size,
                     const char* source_label,
                     const ship_data_model::DataModel<Real>& model,
                     const SignalKMappedValue<Real>& value) const {
        if (value.kind != SignalKMappedValueKind::Object) return write_scalar_mapped(dst, dst_size, source_label, value);
        if (!dst || dst_size == 0 || !value.path) return 0;

        JsonDocument doc;
        JsonArray updates = doc["updates"].to<JsonArray>();
        JsonObject update = updates.add<JsonObject>();
        JsonObject source = update["source"].to<JsonObject>();
        source["label"] = source_label ? source_label : "signalk-mini";

        JsonArray values = update["values"].to<JsonArray>();
        JsonObject item = values.add<JsonObject>();
        item["path"] = value.path;
        JsonObject object = item["value"].to<JsonObject>();
        if (!write_object_value(model, value.object_kind, object)) return 0;

        return serialize_delta(dst, dst_size, doc);
    }

private:
    int write_scalar_mapped(char* dst, size_t dst_size, const char* source_label, const SignalKMappedValue<Real>& value) const {
        if (!dst || dst_size == 0 || !value.path) return 0;

        JsonDocument doc;
        JsonArray updates = doc["updates"].to<JsonArray>();
        JsonObject update = updates.add<JsonObject>();
        JsonObject source = update["source"].to<JsonObject>();
        source["label"] = source_label ? source_label : "signalk-mini";

        JsonArray values = update["values"].to<JsonArray>();
        JsonObject item = values.add<JsonObject>();
        item["path"] = value.path;
        switch (value.kind) {
        case SignalKMappedValueKind::Number:
            item["value"] = value.number;
            break;
        case SignalKMappedValueKind::Bool:
            item["value"] = value.boolean;
            break;
        case SignalKMappedValueKind::Text:
            item["value"] = value.text ? value.text : "";
            break;
        case SignalKMappedValueKind::Object:
            return 0;
        }

        return serialize_delta(dst, dst_size, doc);
    }

    int serialize_delta(char* dst, size_t dst_size, JsonDocument& doc) const {
        const size_t len = serializeJson(doc, dst, dst_size);
        if (len + 2 >= dst_size) return 0;
        dst[len] = '\r';
        dst[len + 1] = '\n';
        dst[len + 2] = '\0';
        return static_cast<int>(len + 2);
    }

    template<typename StampedValue>
    void set_if_valid(JsonObject object, const char* key, const StampedValue& stamped) const {
        if (stamped.valid) object[key] = stamped.value;
    }

    template<typename StampedValue>
    void set_if_valid_rad(JsonObject object, const char* key, const StampedValue& stamped) const {
        if (stamped.valid) object[key] = deg_to_rad<Real>(static_cast<Real>(stamped.value));
    }

    template<typename StampedValue>
    void set_if_valid_mps(JsonObject object, const char* key, const StampedValue& stamped) const {
        if (stamped.valid) object[key] = knots_to_mps<Real>(static_cast<Real>(stamped.value));
    }

    template<typename StampedValue>
    void set_if_valid_m(JsonObject object, const char* key, const StampedValue& stamped) const {
        if (stamped.valid) object[key] = nmi_to_m<Real>(static_cast<Real>(stamped.value));
    }

    void set_text_if_present(JsonObject object, const char* key, const char* text) const {
        if (text && text[0] != '\0') object[key] = text;
    }

    template<typename Target>
    void write_ais_target(JsonObject object, const Target& target) const {
        set_if_valid(object, "mmsi", target.mmsi);
        set_if_valid(object, "lastMessageType", target.last_message_type);
        set_if_valid(object, "repeatIndicator", target.repeat_indicator);
        set_if_valid(object, "navigationStatus", target.navigation_status);
        set_if_valid(object, "shipType", target.ship_type);
        set_if_valid(object, "imoNumber", target.imo_number);
        set_if_valid(object, "aidType", target.aid_type);
        set_text_if_present(object, "name", target.vessel_name);
        set_text_if_present(object, "callSign", target.call_sign);
        set_text_if_present(object, "destination", target.destination);
        if (target.latitude_deg.valid || target.longitude_deg.valid) {
            JsonObject pos = object["position"].to<JsonObject>();
            set_if_valid(pos, "latitude", target.latitude_deg);
            set_if_valid(pos, "longitude", target.longitude_deg);
        }
        set_if_valid_mps(object, "speedOverGround", target.speed_over_ground_kn);
        set_if_valid_rad(object, "courseOverGroundTrue", target.course_over_ground_deg);
        set_if_valid_rad(object, "headingTrue", target.true_heading_deg);
        set_if_valid(object, "timestamp", target.timestamp_s);
        object["positionAccuracy"] = target.position_accuracy;
        object["raim"] = target.raim;
        object["classB"] = target.class_b;
        object["baseStation"] = target.base_station;
        object["aidToNavigation"] = target.aid_to_navigation;
        object["sarAircraft"] = target.sar_aircraft;
        object["stale"] = target.stale;
    }

    bool write_ais_targets(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        bool any = false;
        for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
            const auto& target = model.ais.targets.targets[i];
            if (!target.occupied || !target.mmsi.valid) continue;
            char key[16];
            snprintf(key, sizeof(key), "%ld", static_cast<long>(target.mmsi.value));
            JsonObject entry = object[key].to<JsonObject>();
            write_ais_target(entry, target);
            any = true;
        }
        set_if_valid(object, "count", model.ais.targets.target_count);
        set_if_valid(object, "replacementCount", model.ais.targets.replacement_count);
        set_if_valid(object, "overflowCount", model.ais.targets.overflow_count);
        return any || model.ais.targets.target_count.valid;
    }

    bool write_ais_own_vessel(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        if (!model.ais.own_vessel.valid) return false;
        write_ais_target(object, model.ais.own_vessel);
        return true;
    }

    bool write_ais_safety(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& text = model.ais.safety_text;
        const auto& addressed = model.ais.addressed_safety;
        bool any = false;
        if (text.last_update_us != 0) {
            JsonObject item = object["broadcast"].to<JsonObject>();
            set_if_valid(item, "sourceMmsi", text.mmsi);
            set_if_valid(item, "messageType", text.message_type);
            set_if_valid(item, "destinationMmsi", text.destination_mmsi);
            set_if_valid(item, "sequenceNumber", text.sequence_number);
            item["retransmit"] = text.retransmit;
            set_text_if_present(item, "text", text.text);
            any = true;
        }
        if (addressed.last_update_us != 0) {
            JsonObject item = object["addressed"].to<JsonObject>();
            set_text_if_present(item, "destinationMmsi", addressed.destination_mmsi);
            set_text_if_present(item, "messageId", addressed.sequential_message_id);
            item["retransmit"] = addressed.retransmit_flag;
            set_text_if_present(item, "text", addressed.safety_text);
            set_if_valid(item, "fieldCount", addressed.field_count);
            any = true;
        }
        return any;
    }

    bool write_ais_data_link_status(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& status = model.ais.data_link_status;
        if (status.last_update_us == 0) return false;
        set_text_if_present(object, "stationId", status.station_id);
        set_text_if_present(object, "slotStatus", status.slot_status);
        set_text_if_present(object, "status", status.status_text);
        set_if_valid(object, "fieldCount", status.field_count);
        return true;
    }

    template<typename DscAlert>
    void write_dsc_alert(JsonObject object, const char* name, const DscAlert& alert) const {
        if (alert.last_update_us == 0) return;
        JsonObject item = object[name].to<JsonObject>();
        set_text_if_present(item, "senderMmsi", alert.sender_mmsi);
        set_text_if_present(item, "addressOrDistressMmsi", alert.address_or_distress_mmsi);
        set_if_valid(item, "category", alert.category);
        set_if_valid(item, "natureOrTelecommand", alert.nature_or_first_telecommand);
        if (alert.latitude_deg.valid || alert.longitude_deg.valid) {
            JsonObject pos = item["position"].to<JsonObject>();
            set_if_valid(pos, "latitude", alert.latitude_deg);
            set_if_valid(pos, "longitude", alert.longitude_deg);
        }
        set_if_valid(item, "timeOfDay", alert.utc_time_s);
        set_text_if_present(item, "text", alert.alert_text);
        item["active"] = alert.active;
        item["acknowledged"] = alert.acknowledged;
        item["resolved"] = alert.resolved;
        item["duplicate"] = alert.duplicate;
        set_if_valid(item, "repeatCount", alert.repeat_count);
    }

    bool write_dsc(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& latest = model.comm.dsc.latest_call;
        if (latest.last_update_us != 0) {
            JsonObject call = object["latestCall"].to<JsonObject>();
            set_if_valid(call, "formatSpecifier", latest.format_specifier);
            set_text_if_present(call, "senderMmsi", latest.sender_mmsi);
            set_if_valid(call, "category", latest.category);
            set_if_valid(call, "natureOrTelecommand", latest.nature_or_first_telecommand);
            set_if_valid(call, "communicationOrTelecommand", latest.communication_or_second_telecommand);
            set_text_if_present(call, "addressOrDistressMmsi", latest.address_or_distress_mmsi);
            if (latest.latitude_deg.valid || latest.longitude_deg.valid) {
                JsonObject pos = call["position"].to<JsonObject>();
                set_if_valid(pos, "latitude", latest.latitude_deg);
                set_if_valid(pos, "longitude", latest.longitude_deg);
            }
            set_if_valid(call, "timeOfDay", latest.utc_time_s);
            set_text_if_present(call, "dseText", latest.dse_decoded_text);
            set_if_valid(call, "fieldCount", latest.field_count);
        }
        write_dsc_alert(object, "distress", model.notifications.dsc.distress);
        write_dsc_alert(object, "urgency", model.notifications.dsc.urgency);
        write_dsc_alert(object, "safety", model.notifications.dsc.safety);
        set_if_valid(object, "callCount", model.comm.dsc.call_count);
        set_if_valid(object, "distressCount", model.comm.dsc.distress_count);
        set_if_valid(object, "urgencyCount", model.comm.dsc.urgency_count);
        set_if_valid(object, "safetyCount", model.comm.dsc.safety_count);
        return latest.last_update_us != 0 || model.notifications.dsc.distress.last_update_us != 0 ||
               model.notifications.dsc.urgency.last_update_us != 0 || model.notifications.dsc.safety.last_update_us != 0;
    }

    bool write_inmarsat(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& data = model.notifications.inmarsat.safetynet;
        const auto& msg = data.latest_message;
        if (msg.last_update_us == 0 && !data.message_count.valid) return false;
        set_if_valid(object, "messageCount", data.message_count);
        set_if_valid(object, "duplicateCount", data.duplicate_count);
        set_if_valid(object, "unsupportedCount", data.unsupported_count);
        JsonObject latest = object["latest"].to<JsonObject>();
        set_text_if_present(latest, "messageId", msg.message_id);
        set_text_if_present(latest, "terminalId", msg.terminal_id);
        latest["msiStatus"] = msg.msi_status;
        set_text_if_present(latest, "lesSequenceNumber", msg.les_sequence_number);
        set_text_if_present(latest, "lesId", msg.les_id);
        latest["oceanRegion"] = msg.ocean_region_code;
        set_text_if_present(latest, "oceanRegionLabel", msg.ocean_region_label);
        latest["priority"] = msg.priority_code;
        set_text_if_present(latest, "priorityLabel", msg.priority_label);
        set_text_if_present(latest, "serviceCode", msg.service_code);
        set_text_if_present(latest, "service", msg.service_label);
        set_text_if_present(latest, "addressKind", msg.address_kind);
        set_if_valid(latest, "navareaMetarea", msg.navarea_metarea_code);
        set_if_valid(latest, "circularRadiusNmi", msg.circular_radius_nmi);
        set_if_valid(latest, "rectangleExtentLatDeg", msg.rectangle_extent_lat_deg);
        set_if_valid(latest, "rectangleExtentLonDeg", msg.rectangle_extent_lon_deg);
        if (msg.circular_center_lat_deg.valid || msg.circular_center_lon_deg.valid) {
            JsonObject center = latest["circularCenter"].to<JsonObject>();
            set_if_valid(center, "latitude", msg.circular_center_lat_deg);
            set_if_valid(center, "longitude", msg.circular_center_lon_deg);
        }
        if (msg.rectangle_sw_lat_deg.valid || msg.rectangle_sw_lon_deg.valid) {
            JsonObject sw = latest["rectangleSouthwest"].to<JsonObject>();
            set_if_valid(sw, "latitude", msg.rectangle_sw_lat_deg);
            set_if_valid(sw, "longitude", msg.rectangle_sw_lon_deg);
        }
        set_text_if_present(latest, "text", msg.message_text);
        latest["complete"] = msg.complete;
        latest["duplicate"] = msg.duplicate;
        latest["acknowledged"] = msg.acknowledged;
        latest["requiresAlarm"] = msg.requires_alarm;
        latest["mandatoryReception"] = msg.mandatory_reception;
        return true;
    }

    bool write_navtex(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& navtex = model.notifications.navtex;
        bool any = false;
        if (navtex.received.first_seen_us != 0) {
            JsonObject latest = object["latest"].to<JsonObject>();
            set_text_if_present(latest, "messageId", navtex.received.navtex_message_id);
            latest["transmitter"] = navtex.received.transmitter_id;
            latest["subject"] = navtex.received.subject_indicator;
            set_text_if_present(latest, "subjectLabel", navtex.received.subject_label);
            set_if_valid(latest, "serialNumber", navtex.received.serial_number);
            set_text_if_present(latest, "text", navtex.received.body_text[0] ? navtex.received.body_text : navtex.received.message_text);
            set_if_valid(latest, "textLength", navtex.received.text_length);
            latest["complete"] = navtex.received.complete;
            latest["overflow"] = navtex.received.overflow;
            latest["duplicate"] = navtex.received.duplicate;
            latest["acknowledged"] = navtex.received.acknowledged;
            any = true;
        }
        if (navtex.receiver_mask.last_update_us != 0) {
            JsonObject mask = object["receiverMask"].to<JsonObject>();
            set_text_if_present(mask, "receiverId", navtex.receiver_mask.receiver_id);
            set_text_if_present(mask, "stationMask", navtex.receiver_mask.station_mask);
            set_text_if_present(mask, "subjectMask", navtex.receiver_mask.subject_mask);
            set_if_valid(mask, "enabledStationCount", navtex.receiver_mask.enabled_station_count);
            set_if_valid(mask, "enabledSubjectCount", navtex.receiver_mask.enabled_subject_count);
            any = true;
        }
        set_if_valid(object, "historyCount", navtex.history.count);
        set_if_valid(object, "duplicateCount", navtex.history.duplicate_count);
        return any || navtex.history.count.valid;
    }

    bool write_alerts(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& alerts = model.notifications.alerts;
        bool any = false;
        if (alerts.alert_report.last_update_us != 0) {
            JsonObject report = object["report"].to<JsonObject>();
            set_text_if_present(report, "id", alerts.alert_report.alert_identifier);
            set_if_valid(report, "instance", alerts.alert_report.alert_instance);
            report["category"] = alerts.alert_report.category;
            report["priority"] = alerts.alert_report.priority;
            report["state"] = alerts.alert_report.alert_state;
            set_text_if_present(report, "text", alerts.alert_report.alert_text);
            any = true;
        }
        if (alerts.alarm_state.last_update_us != 0) {
            JsonObject alarm = object["alarmState"].to<JsonObject>();
            set_text_if_present(alarm, "id", alerts.alarm_state.alarm_identifier);
            set_if_valid(alarm, "localAlarmNumber", alerts.alarm_state.local_alarm_number);
            alarm["conditionState"] = alerts.alarm_state.condition_state;
            alarm["acknowledgementState"] = alerts.alarm_state.acknowledgement_state;
            set_text_if_present(alarm, "description", alerts.alarm_state.description);
            any = true;
        }
        if (alerts.heartbeat.last_update_us != 0) {
            JsonObject heartbeat = object["heartbeat"].to<JsonObject>();
            heartbeat["status"] = alerts.heartbeat.status;
            set_if_valid(heartbeat, "interval", alerts.heartbeat.interval_s);
            set_text_if_present(heartbeat, "sequence", alerts.heartbeat.sequential_message_id);
            any = true;
        }
        if (alerts.fire.last_update_us != 0) {
            JsonObject fire = object["fire"].to<JsonObject>();
            set_text_if_present(fire, "id", alerts.fire.id);
            set_text_if_present(fire, "text", alerts.fire.text);
            set_if_valid(fire, "fieldCount", alerts.fire.field_count);
            any = true;
        }
        return any;
    }

    bool write_mob(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& smv = model.notifications.special.smv;
        if (smv.last_update_us == 0) return false;
        set_text_if_present(object, "id", smv.message_id);
        set_if_valid(object, "mmsi", smv.mmsi);
        set_text_if_present(object, "mmsiText", smv.mmsi_text);
        set_if_valid(object, "timeOfDay", smv.utc_time_s);
        if (smv.latitude_deg.valid || smv.longitude_deg.valid) {
            JsonObject pos = object["position"].to<JsonObject>();
            set_if_valid(pos, "latitude", smv.latitude_deg);
            set_if_valid(pos, "longitude", smv.longitude_deg);
        }
        set_text_if_present(object, "eventType", smv.event_type);
        set_text_if_present(object, "sarCapability", smv.sar_capability);
        set_text_if_present(object, "routeId", smv.route_id);
        object["status"] = smv.status;
        set_text_if_present(object, "description", smv.description);
        return true;
    }

    bool write_legacy_comm(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        bool any = false;
        const auto& rf = model.comm.radio_frequency_set;
        if (rf.last_update_us != 0) {
            JsonObject item = object["radioFrequencySet"].to<JsonObject>();
            set_if_valid(item, "transmittingFrequencyHz", rf.transmitting_frequency_hz);
            set_if_valid(item, "receivingFrequencyHz", rf.receiving_frequency_hz);
            item["communicationMode"] = rf.communication_mode;
            set_if_valid(item, "powerLevel", rf.power_level);
            any = true;
        }
        const auto& rlm = model.comm.return_link_message;
        if (rlm.last_update_us != 0) {
            JsonObject item = object["returnLinkMessage"].to<JsonObject>();
            set_text_if_present(item, "beaconId", rlm.beacon_id);
            set_if_valid(item, "receptionTime", rlm.reception_time_s);
            item["messageCode"] = rlm.message_code;
            set_text_if_present(item, "messageBody", rlm.message_body);
            any = true;
        }
        return any;
    }

    bool write_object_value(const ship_data_model::DataModel<Real>& model, SignalKObjectKind kind, JsonObject object) const {
        switch (kind) {
        case SignalKObjectKind::AisTargets: return write_ais_targets(model, object);
        case SignalKObjectKind::AisOwnVessel: return write_ais_own_vessel(model, object);
        case SignalKObjectKind::AisSafety: return write_ais_safety(model, object);
        case SignalKObjectKind::AisDataLinkStatus: return write_ais_data_link_status(model, object);
        case SignalKObjectKind::Dsc: return write_dsc(model, object);
        case SignalKObjectKind::InmarsatSafetyNet: return write_inmarsat(model, object);
        case SignalKObjectKind::Navtex: return write_navtex(model, object);
        case SignalKObjectKind::Alerts: return write_alerts(model, object);
        case SignalKObjectKind::Mob: return write_mob(model, object);
        case SignalKObjectKind::LegacyComm: return write_legacy_comm(model, object);
        case SignalKObjectKind::None:
        default: return false;
        }
    }
};

} // namespace signalk_mini
