#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "signalk_delta_writer.hpp"
#include "signalk_identity.hpp"
#include "signalk_json_stream_writer.hpp"
#include "signalk_typed_view.hpp"
#include "units.hpp"

namespace signalk_mini {

template<typename Real>
class SignalKFullModelWriter {
public:
    int write(char* dst,
              size_t dst_size,
              const ModelStore<Real>& store,
              const SignalKMiniConfig& config,
              uint64_t now_us = 0) const {
        if (!dst || dst_size == 0) return 0;
        SignalKTypedModelView<Real> view(store, config, now_us);
        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return 0;

        SignalKJsonStreamWriter out(dst, dst_size);
        if (!out.append_raw("{\"version\":")) return 0;
        if (!out.append_quoted(config.identity.signalk_version && config.identity.signalk_version[0]
                                   ? config.identity.signalk_version : "1.8.2")) return 0;
        if (!out.append_raw(",\"self\":")) return 0;
        if (!out.append_quoted(view.vessel_key())) return 0;
        if (!out.append_raw(",\"vessels\":{")) return 0;
        if (!out.append_quoted(view.vessel_key()) || !out.append_char(':') || !out.append_char('{')) return 0;
        if (!write_vessel_identity(out, view.vessel_key())) return 0;
        if (has_self_projection_values(view)) {
            if (!out.append_char(',') || !write_branch(out, view, "", timestamp)) return 0;
        }
        if (!out.append_char('}')) return 0;
        if (!write_ais_vessels(out, view, timestamp)) return 0;
        if (!out.append_raw("},\"sources\":{")) return 0;
        if (!write_sources(out, view, config)) return 0;
        if (!out.append_raw("}}")) return 0;
        return out.ok() ? static_cast<int>(out.size()) : 0;
    }

private:
    static bool is_vessel_mmsi(int32_t mmsi) {
        return mmsi >= 200000000 && mmsi <= 799999999;
    }

    static bool format_mmsi(char* dst, size_t dst_size, int32_t mmsi) {
        if (!dst || dst_size < 10 || !is_vessel_mmsi(mmsi)) return false;
        const int len = snprintf(dst, dst_size, "%09ld", static_cast<long>(mmsi));
        return len == 9;
    }

    bool write_vessel_identity(SignalKJsonStreamWriter& out, const char* vessel_key) const {
        const SignalKVesselIdentityKind kind = signalk_vessel_identity_kind(vessel_key);
        if (kind == SignalKVesselIdentityKind::Uuid) {
            return out.append_raw("\"uuid\":") && out.append_quoted(vessel_key);
        }
        if (kind == SignalKVesselIdentityKind::Mmsi) {
            static constexpr char Prefix[] = "urn:mrn:imo:mmsi:";
            return out.append_raw("\"mmsi\":") && out.append_quoted(vessel_key + sizeof(Prefix) - 1);
        }
        if (kind == SignalKVesselIdentityKind::Url) {
            return out.append_raw("\"url\":") && out.append_quoted(vessel_key);
        }
        return false;
    }

    static bool path_under_prefix(const char* path, const char* prefix, const char*& remainder) {
        if (!path || !prefix) return false;
        const size_t prefix_len = strlen(prefix);
        if (prefix_len == 0) { remainder = path; return path[0] != '\0'; }
        if (strncmp(path, prefix, prefix_len) != 0 || path[prefix_len] != '.') return false;
        remainder = path + prefix_len + 1;
        return remainder[0] != '\0';
    }

    static size_t next_segment_length(const char* remainder) {
        const char* dot = strchr(remainder, '.');
        return dot ? static_cast<size_t>(dot - remainder) : strlen(remainder);
    }

    bool skip_from_self_projection(ModelField field, const SignalKMappedValue<Real>& mapped) const {
        return field == ModelField::AisTargetsObject || mapped.object_kind == SignalKObjectKind::AisTargets;
    }

    bool has_self_projection_values(const SignalKTypedModelView<Real>& view) const {
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            const ModelField field = static_cast<ModelField>(i);
            SignalKMappedValue<Real> mapped;
            if (view.current_value(field, mapped) && !skip_from_self_projection(field, mapped)) return true;
        }
        return false;
    }

    bool is_selected_segment_candidate(const SignalKTypedModelView<Real>& view,
                                       size_t current_index,
                                       const SignalKMappedValue<Real>& current,
                                       const char* prefix,
                                       const char* segment,
                                       size_t segment_len) const {
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            if (i == current_index) continue;
            SignalKMappedValue<Real> candidate;
            const ModelField field = static_cast<ModelField>(i);
            if (!view.current_value(field, candidate) || skip_from_self_projection(field, candidate)) continue;
            const char* remainder = nullptr;
            if (!path_under_prefix(candidate.path, prefix, remainder)) continue;
            const size_t candidate_len = next_segment_length(remainder);
            if (candidate_len != segment_len || strncmp(remainder, segment, segment_len) != 0) continue;
            if (candidate.last_update_us > current.last_update_us) return false;
            if (candidate.last_update_us == current.last_update_us && i < current_index) return false;
        }
        return true;
    }

    bool write_branch(SignalKJsonStreamWriter& out,
                      const SignalKTypedModelView<Real>& view,
                      const char* prefix,
                      const char* timestamp) const {
        bool first = true;
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            const ModelField field = static_cast<ModelField>(i);
            SignalKMappedValue<Real> mapped;
            if (!view.current_value(field, mapped) || skip_from_self_projection(field, mapped)) continue;
            const char* remainder = nullptr;
            if (!path_under_prefix(mapped.path, prefix, remainder)) continue;
            const size_t segment_len = next_segment_length(remainder);
            if (segment_len == 0 || !is_selected_segment_candidate(view, i, mapped, prefix, remainder, segment_len)) continue;

            if (!first && !out.append_char(',')) return false;
            first = false;
            char segment[96];
            if (segment_len >= sizeof(segment)) return false;
            memcpy(segment, remainder, segment_len);
            segment[segment_len] = '\0';
            if (!out.append_quoted(segment) || !out.append_char(':')) return false;

            const bool leaf = remainder[segment_len] == '\0';
            if (leaf) {
                if (!write_leaf(out, view, mapped, timestamp)) return false;
                continue;
            }

            char child_prefix[192];
            const int len = prefix[0]
                ? snprintf(child_prefix, sizeof(child_prefix), "%s.%s", prefix, segment)
                : snprintf(child_prefix, sizeof(child_prefix), "%s", segment);
            if (len <= 0 || static_cast<size_t>(len) >= sizeof(child_prefix)) return false;
            if (!out.append_char('{') || !write_branch(out, view, child_prefix, timestamp) || !out.append_char('}')) return false;
        }
        return true;
    }

    bool write_leaf(SignalKJsonStreamWriter& out,
                    const SignalKTypedModelView<Real>& view,
                    const SignalKMappedValue<Real>& mapped,
                    const char* timestamp) const {
        char source_key[24];
        if (!view.source_key(mapped.source_id, source_key, sizeof(source_key))) return false;
        if (!out.append_raw("{\"value\":")) return false;
        if (!delta_writer_.write_value(out, view.model(), mapped)) return false;
        if (!out.append_raw(",\"$source\":")) return false;
        if (!out.append_quoted(source_key)) return false;
        if (!out.append_raw(",\"timestamp\":")) return false;
        if (!out.append_quoted(timestamp)) return false;
        return out.append_char('}');
    }

    static const char* connector_type(ConnectorProtocol protocol) {
        switch (protocol) {
        case ConnectorProtocol::Nmea0183: return "NMEA0183";
        case ConnectorProtocol::SeaTalk1: return "SeaTalk1";
        case ConnectorProtocol::Ubx: return "UBX";
        case ConnectorProtocol::Gpsd: return "GPSD";
        case ConnectorProtocol::Nmea2000: return "NMEA2000";
        case ConnectorProtocol::SignalK: return "SignalK";
        case ConnectorProtocol::GenericSensor: return "GenericSensor";
        default: return "unknown";
        }
    }

    bool write_sources(SignalKJsonStreamWriter& out,
                       const SignalKTypedModelView<Real>& view,
                       const SignalKMiniConfig& config) const {
        bool first = true;
        auto write_source = [&](SourceId source_id, const char* label, const char* type) -> bool {
            char key[24];
            if (!view.source_key(source_id, key, sizeof(key))) return false;
            if (!first && !out.append_char(',')) return false;
            first = false;
            return out.append_quoted(key) && out.append_raw(":{\"label\":") &&
                   out.append_quoted(label ? label : "signalk-mini") &&
                   out.append_raw(",\"type\":") && out.append_quoted(type) && out.append_char('}');
        };

        if (!write_source(0, view.source_label(0), "SignalK")) return false;
        for (size_t i = 0; i < config.connector_count; ++i) {
            const ConnectorConfig& connector = config.connectors[i];
            if (!connector.enabled) continue;
            const SourceId source_id = static_cast<SourceId>(FirstConnectorSourceId + i);
            if (!write_source(source_id, view.source_label(source_id), connector_type(connector.protocol.kind))) return false;
        }
        return true;
    }

    bool write_ais_vessels(SignalKJsonStreamWriter& out,
                           const SignalKTypedModelView<Real>& view,
                           const char* timestamp) const {
        const auto& table = view.model().ais.targets;
        if (!table.target_count.valid ||
            !view.field_is_current(ModelField::AisTargetsObject, table.target_count.last_update_us)) return true;
        const SourceId source_id = view.current_value_source(ModelField::AisTargetsObject);
        char source_key[24];
        if (!view.source_key(source_id, source_key, sizeof(source_key))) return false;
        for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
            const auto& target = table.targets[i];
            if (!target.occupied || !view.ais_target_is_current(target) ||
                !target.mmsi.valid || !is_vessel_mmsi(target.mmsi.value)) continue;
            char mmsi[10];
            if (!format_mmsi(mmsi, sizeof(mmsi), target.mmsi.value)) continue;
            char vessel_key[48];
            const int key_len = snprintf(vessel_key, sizeof(vessel_key), "urn:mrn:imo:mmsi:%s", mmsi);
            if (key_len <= 0 || static_cast<size_t>(key_len) >= sizeof(vessel_key)) return false;
            if (strcmp(vessel_key, view.vessel_key()) == 0) continue;
            if (!out.append_char(',') || !out.append_quoted(vessel_key) || !out.append_raw(":{")) return false;
            bool first = true;
            if (target.vessel_name[0]) {
                if (!out.append_raw("\"name\":" ) || !out.append_quoted(target.vessel_name)) return false;
                first = false;
            }
            if (target.mmsi.valid) {
                if (!first && !out.append_char(',')) return false;
                first = false;
                if (!out.append_raw("\"mmsi\":" ) || !out.append_quoted(mmsi)) return false;
            }
            const bool has_position = target.latitude_deg.valid && target.longitude_deg.valid;
            const bool has_navigation = has_position || target.speed_over_ground_kn.valid ||
                                        target.course_over_ground_deg.valid || target.true_heading_deg.valid;
            if (has_navigation) {
                if (!first && !out.append_char(',')) return false;
                if (!out.append_raw("\"navigation\":{")) return false;
                bool nav_first = true;
                if (has_position) {
                    if (!out.append_raw("\"position\":{\"value\":{")) return false;
                    bool pos_first = true;
                    if (target.latitude_deg.valid) {
                        if (!out.append_raw("\"latitude\":" ) || !out.append_real(target.latitude_deg.value)) return false;
                        pos_first = false;
                    }
                    if (target.longitude_deg.valid) {
                        if (!pos_first && !out.append_char(',')) return false;
                        if (!out.append_raw("\"longitude\":" ) || !out.append_real(target.longitude_deg.value)) return false;
                    }
                    if (!out.append_raw("},\"$source\":" ) || !out.append_quoted(source_key) ||
                        !out.append_raw(",\"timestamp\":" ) || !out.append_quoted(timestamp) || !out.append_char('}')) return false;
                    nav_first = false;
                }
                if (target.speed_over_ground_kn.valid && !write_ais_number_leaf(out, nav_first, "speedOverGround",
                        knots_to_mps<Real>(target.speed_over_ground_kn.value), source_key, timestamp)) return false;
                if (target.speed_over_ground_kn.valid) nav_first = false;
                if (target.course_over_ground_deg.valid && !write_ais_number_leaf(out, nav_first, "courseOverGroundTrue",
                        deg_to_rad<Real>(target.course_over_ground_deg.value), source_key, timestamp)) return false;
                if (target.course_over_ground_deg.valid) nav_first = false;
                if (target.true_heading_deg.valid && !write_ais_number_leaf(out, nav_first, "headingTrue",
                        deg_to_rad<Real>(target.true_heading_deg.value), source_key, timestamp)) return false;
                if (!out.append_char('}')) return false;
            }
            if (!out.append_char('}')) return false;
        }
        return true;
    }

    bool write_ais_number_leaf(SignalKJsonStreamWriter& out,
                               bool first,
                               const char* path,
                               Real value,
                               const char* source_key,
                               const char* timestamp) const {
        if (!first && !out.append_char(',')) return false;
        return out.append_quoted(path) && out.append_raw(":{\"value\":") && out.append_real(value) &&
               out.append_raw(",\"$source\":") && out.append_quoted(source_key) &&
               out.append_raw(",\"timestamp\":") && out.append_quoted(timestamp) && out.append_char('}');
    }

    SignalKDeltaWriter<Real> delta_writer_;
};

} // namespace signalk_mini
