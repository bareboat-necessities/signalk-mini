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
        JsonObject item = start_item(doc, source_label, path);
        item["value"] = value;
        return serialize_delta(dst, dst_size, doc);
    }

    int write_mapped(char* dst, size_t dst_size, const char* source_label, const SignalKMappedValue<Real>& value) const {
        if (value.kind == SignalKMappedValueKind::Object) return 0;
        return write_scalar(dst, dst_size, source_label, value);
    }

    int write_mapped(char* dst, size_t dst_size, const char* source_label, const ship_data_model::DataModel<Real>& model, const SignalKMappedValue<Real>& value) const {
        if (value.kind != SignalKMappedValueKind::Object) return write_scalar(dst, dst_size, source_label, value);
        if (!dst || dst_size == 0 || !value.path) return 0;
        JsonDocument doc;
        JsonObject item = start_item(doc, source_label, value.path);
        JsonObject object = item["value"].to<JsonObject>();
        if (!write_object(model, value.object_kind, object)) return 0;
        return serialize_delta(dst, dst_size, doc);
    }

private:
    JsonObject start_item(JsonDocument& doc, const char* source_label, const char* path) const {
        JsonArray updates = doc["updates"].to<JsonArray>();
        JsonObject update = updates.add<JsonObject>();
        JsonObject source = update["source"].to<JsonObject>();
        source["label"] = source_label ? source_label : "signalk-mini";
        JsonArray values = update["values"].to<JsonArray>();
        JsonObject item = values.add<JsonObject>();
        item["path"] = path;
        return item;
    }

    int write_scalar(char* dst, size_t dst_size, const char* source_label, const SignalKMappedValue<Real>& value) const {
        if (!dst || dst_size == 0 || !value.path) return 0;
        JsonDocument doc;
        JsonObject item = start_item(doc, source_label, value.path);
        if (value.kind == SignalKMappedValueKind::Number) item["value"] = value.number;
        else if (value.kind == SignalKMappedValueKind::Bool) item["value"] = value.boolean;
        else if (value.kind == SignalKMappedValueKind::Text) item["value"] = value.text ? value.text : "";
        else return 0;
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
    void set(JsonObject object, const char* key, const StampedValue& stamped) const { if (stamped.valid) object[key] = stamped.value; }
    void set(JsonObject object, const char* key, char value) const { if (value) object[key] = static_cast<int>(value); }
    template<typename StampedValue>
    void set_rad(JsonObject object, const char* key, const StampedValue& stamped) const { if (stamped.valid) object[key] = deg_to_rad<Real>(static_cast<Real>(stamped.value)); }
    template<typename StampedValue>
    void set_mps(JsonObject object, const char* key, const StampedValue& stamped) const { if (stamped.valid) object[key] = knots_to_mps<Real>(static_cast<Real>(stamped.value)); }
    void set_text(JsonObject object, const char* key, const char* text) const { if (text && text[0]) object[key] = text; }

    template<typename LatStamped, typename LonStamped>
    void set_position(JsonObject object, const LatStamped& lat, const LonStamped& lon) const {
        if (!lat.valid && !lon.valid) return;
        JsonObject pos = object["position"].to<JsonObject>();
        set(pos, "latitude", lat);
        set(pos, "longitude", lon);
    }

    bool write_ais_targets(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        bool any = false;
        for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
            const auto& t = model.ais.targets.targets[i];
            if (!t.occupied || !t.mmsi.valid) continue;
            char key[16];
            snprintf(key, sizeof(key), "%ld", static_cast<long>(t.mmsi.value));
            JsonObject o = object[key].to<JsonObject>();
            set(o, "mmsi", t.mmsi);
            set(o, "lastMessageType", t.last_message_type);
            set(o, "navigationStatus", t.navigation_status);
            set_position(o, t.latitude_deg, t.longitude_deg);
            set_mps(o, "speedOverGround", t.speed_over_ground_kn);
            set_rad(o, "courseOverGroundTrue", t.course_over_ground_deg);
            set_rad(o, "headingTrue", t.true_heading_deg);
            set(o, "shipType", t.ship_type);
            set(o, "imoNumber", t.imo_number);
            set_text(o, "name", t.vessel_name);
            set_text(o, "callSign", t.call_sign);
            set_text(o, "destination", t.destination);
            o["classB"] = t.class_b;
            o["aidToNavigation"] = t.aid_to_navigation;
            o["stale"] = t.stale;
            any = true;
        }
        set(object, "count", model.ais.targets.target_count);
        set(object, "replacementCount", model.ais.targets.replacement_count);
        set(object, "overflowCount", model.ais.targets.overflow_count);
        return any || model.ais.targets.target_count.valid;
    }

    bool write_ais_own(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& t = model.ais.own_vessel;
        if (!t.valid) return false;
        set(object, "messageType", t.message_type);
        set(object, "mmsi", t.mmsi);
        set(object, "navigationStatus", t.navigation_status);
        set_position(object, t.latitude_deg, t.longitude_deg);
        set_mps(object, "speedOverGround", t.speed_over_ground_kn);
        set_rad(object, "courseOverGroundTrue", t.course_over_ground_deg);
        set_rad(object, "headingTrue", t.true_heading_deg);
        set(object, "shipType", t.ship_type);
        set(object, "imoNumber", t.imo_number);
        set_text(object, "name", t.vessel_name);
        set_text(object, "callSign", t.call_sign);
        set_text(object, "destination", t.destination);
        object["classB"] = t.class_b;
        object["aidToNavigation"] = t.aid_to_navigation;
        return true;
    }

    bool write_ais_safety(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        bool any = false;
        const auto& b = model.ais.safety_text;
        if (b.last_update_us) {
            JsonObject o = object["broadcast"].to<JsonObject>();
            set(o, "sourceMmsi", b.mmsi);
            set(o, "destinationMmsi", b.destination_mmsi);
            set(o, "sequenceNumber", b.sequence_number);
            o["retransmit"] = b.retransmit;
            set_text(o, "text", b.text);
            any = true;
        }
        const auto& a = model.ais.addressed_safety;
        if (a.last_update_us) {
            JsonObject o = object["addressed"].to<JsonObject>();
            set_text(o, "destinationMmsi", a.destination_mmsi);
            set_text(o, "messageId", a.sequential_message_id);
            set(o, "retransmit", a.retransmit_flag);
            set_text(o, "text", a.safety_text);
            any = true;
        }
        return any;
    }

    bool write_ais_status(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& s = model.ais.data_link_status;
        if (!s.last_update_us) return false;
        set_text(object, "stationId", s.station_id);
        set_text(object, "slotStatus", s.slot_status);
        set_text(object, "status", s.status_text);
        set(object, "fieldCount", s.field_count);
        return true;
    }

    template<typename Alert>
    void set_dsc_alert(JsonObject object, const char* key, const Alert& a) const {
        if (!a.last_update_us) return;
        JsonObject o = object[key].to<JsonObject>();
        set_text(o, "senderMmsi", a.sender_mmsi);
        set_text(o, "addressOrDistressMmsi", a.address_or_distress_mmsi);
        set(o, "category", a.category);
        set(o, "natureOrTelecommand", a.nature_or_first_telecommand);
        set_position(o, a.latitude_deg, a.longitude_deg);
        set(o, "timeOfDay", a.utc_time_s);
        set_text(o, "text", a.alert_text);
        o["active"] = a.active;
        o["acknowledged"] = a.acknowledged;
        o["resolved"] = a.resolved;
        o["duplicate"] = a.duplicate;
    }

    bool write_dsc(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& c = model.comm.dsc.latest_call;
        if (c.last_update_us) {
            JsonObject o = object["latestCall"].to<JsonObject>();
            set(o, "formatSpecifier", c.format_specifier);
            set_text(o, "senderMmsi", c.sender_mmsi);
            set(o, "category", c.category);
            set(o, "natureOrTelecommand", c.nature_or_first_telecommand);
            set_text(o, "addressOrDistressMmsi", c.address_or_distress_mmsi);
            set_position(o, c.latitude_deg, c.longitude_deg);
            set(o, "timeOfDay", c.utc_time_s);
            set_text(o, "dseText", c.dse_decoded_text);
        }
        set_dsc_alert(object, "distress", model.notifications.dsc.distress);
        set_dsc_alert(object, "urgency", model.notifications.dsc.urgency);
        set_dsc_alert(object, "safety", model.notifications.dsc.safety);
        set(object, "callCount", model.comm.dsc.call_count);
        return c.last_update_us || model.notifications.dsc.distress.last_update_us || model.notifications.dsc.urgency.last_update_us || model.notifications.dsc.safety.last_update_us;
    }

    bool write_inmarsat(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& d = model.notifications.inmarsat.safetynet;
        const auto& m = d.latest_message;
        if (!m.last_update_us && !d.message_count.valid) return false;
        set(object, "messageCount", d.message_count);
        set(object, "duplicateCount", d.duplicate_count);
        JsonObject o = object["latest"].to<JsonObject>();
        set_text(o, "messageId", m.message_id);
        set_text(o, "terminalId", m.terminal_id);
        set(o, "priority", m.priority_code);
        set_text(o, "priorityLabel", m.priority_label);
        set_text(o, "serviceCode", m.service_code);
        set_text(o, "service", m.service_label);
        set_text(o, "addressKind", m.address_kind);
        set_text(o, "text", m.message_text);
        o["complete"] = m.complete;
        o["duplicate"] = m.duplicate;
        o["acknowledged"] = m.acknowledged;
        o["requiresAlarm"] = m.requires_alarm;
        o["mandatoryReception"] = m.mandatory_reception;
        return true;
    }

    bool write_navtex(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& n = model.notifications.navtex;
        bool any = false;
        if (n.received.first_seen_us) {
            JsonObject o = object["latest"].to<JsonObject>();
            set_text(o, "messageId", n.received.navtex_message_id);
            set(o, "transmitter", n.received.transmitter_id);
            set(o, "subject", n.received.subject_indicator);
            set_text(o, "subjectLabel", n.received.subject_label);
            set(o, "serialNumber", n.received.serial_number);
            set_text(o, "text", n.received.body_text[0] ? n.received.body_text : n.received.message_text);
            o["complete"] = n.received.complete;
            o["overflow"] = n.received.overflow;
            o["duplicate"] = n.received.duplicate;
            any = true;
        }
        if (n.receiver_mask.last_update_us) {
            JsonObject o = object["receiverMask"].to<JsonObject>();
            set_text(o, "receiverId", n.receiver_mask.receiver_id);
            set_text(o, "stationMask", n.receiver_mask.station_mask);
            set_text(o, "subjectMask", n.receiver_mask.subject_mask);
            any = true;
        }
        set(object, "historyCount", n.history.count);
        return any || n.history.count.valid;
    }

    bool write_alerts(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& a = model.notifications.alerts;
        bool any = false;
        if (a.alert_report.last_update_us) {
            JsonObject o = object["report"].to<JsonObject>();
            set_text(o, "id", a.alert_report.alert_identifier);
            set(o, "instance", a.alert_report.alert_instance);
            set(o, "category", a.alert_report.category);
            set(o, "priority", a.alert_report.priority);
            set(o, "state", a.alert_report.alert_state);
            set_text(o, "text", a.alert_report.alert_text);
            any = true;
        }
        if (a.alarm_state.last_update_us) {
            JsonObject o = object["alarmState"].to<JsonObject>();
            set_text(o, "id", a.alarm_state.alarm_identifier);
            set(o, "conditionState", a.alarm_state.condition_state);
            set(o, "acknowledgementState", a.alarm_state.acknowledgement_state);
            set_text(o, "description", a.alarm_state.description);
            any = true;
        }
        return any;
    }

    bool write_mob(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& m = model.notifications.special.smv;
        if (!m.last_update_us) return false;
        set_text(object, "id", m.message_id);
        set(object, "mmsi", m.mmsi);
        set(object, "timeOfDay", m.utc_time_s);
        set_position(object, m.latitude_deg, m.longitude_deg);
        set_text(object, "eventType", m.event_type);
        set_text(object, "description", m.description);
        set(object, "status", m.status);
        return true;
    }

    bool write_legacy_comm(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        bool any = false;
        const auto& rf = model.comm.radio_frequency_set;
        if (rf.last_update_us) {
            JsonObject o = object["radioFrequencySet"].to<JsonObject>();
            set(o, "transmittingFrequencyHz", rf.transmitting_frequency_hz);
            set(o, "receivingFrequencyHz", rf.receiving_frequency_hz);
            set(o, "communicationMode", rf.communication_mode);
            any = true;
        }
        const auto& rlm = model.comm.return_link_message;
        if (rlm.last_update_us) {
            JsonObject o = object["returnLinkMessage"].to<JsonObject>();
            set_text(o, "beaconId", rlm.beacon_id);
            set(o, "receptionTime", rlm.reception_time_s);
            set(o, "messageCode", rlm.message_code);
            set_text(o, "messageBody", rlm.message_body);
            any = true;
        }
        return any;
    }

    bool write_object(const ship_data_model::DataModel<Real>& model, SignalKObjectKind kind, JsonObject object) const {
        switch (kind) {
        case SignalKObjectKind::AisTargets: return write_ais_targets(model, object);
        case SignalKObjectKind::AisOwnVessel: return write_ais_own(model, object);
        case SignalKObjectKind::AisSafety: return write_ais_safety(model, object);
        case SignalKObjectKind::AisDataLinkStatus: return write_ais_status(model, object);
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
