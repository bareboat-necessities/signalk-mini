#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <ArduinoJson.h>
#include <ship_data_model.hpp>
#include "units.hpp"

namespace signalk_mini {

template<typename Real>
class SignalKAisDeltaWriter {
public:
    bool write_targets(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
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

    bool write_own_vessel(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
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

    bool write_safety(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
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

    bool write_data_link_status(const ship_data_model::DataModel<Real>& model, JsonObject object) const {
        const auto& s = model.ais.data_link_status;
        if (!s.last_update_us) return false;
        set_text(object, "stationId", s.station_id);
        set_text(object, "slotStatus", s.slot_status);
        set_text(object, "status", s.status_text);
        set(object, "fieldCount", s.field_count);
        return true;
    }

private:
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
};

} // namespace signalk_mini
