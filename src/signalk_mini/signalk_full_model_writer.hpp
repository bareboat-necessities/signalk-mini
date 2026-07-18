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

enum class SignalKModelWriteResult : uint8_t {
    Written,
    NotFound,
    InvalidPath,
    WriteFailed,
};

template<typename Real>
class SignalKFullModelWriter {
public:
    int write(char* dst,
              size_t dst_size,
              const ModelStore<Real>& store,
              const SignalKMiniConfig& config,
              uint64_t now_us = 0) const {
        if (!dst || dst_size == 0) return 0;
        SignalKJsonStreamWriter out(dst, dst_size);
        if (!write_document(out, store, config, now_us)) return 0;
        return out.ok() ? static_cast<int>(out.size()) : 0;
    }

    bool write_document(SignalKJsonStreamWriter& out,
                        const ModelStore<Real>& store,
                        const SignalKMiniConfig& config,
                        uint64_t now_us = 0) const {
        SignalKTypedModelView<Real> view(store, config, now_us);
        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return false;

        if (!out.append_raw("{\"version\":")) return false;
        if (!out.append_quoted(signalk_version(config))) return false;
        if (!out.append_raw(",\"self\":")) return false;
        if (!out.append_quoted(view.vessel_key())) return false;
        if (!out.append_raw(",\"vessels\":")) return false;
        if (!write_vessels_object(out, view, timestamp)) return false;
        if (!out.append_raw(",\"sources\":")) return false;
        if (!write_sources_object(out, view, config)) return false;
        return out.append_char('}');
    }

    SignalKModelWriteResult write_api_path(SignalKJsonStreamWriter& out,
                                           const char* relative_path,
                                           const ModelStore<Real>& store,
                                           const SignalKMiniConfig& config,
                                           uint64_t now_us = 0) const {
        if (!relative_path) return SignalKModelWriteResult::InvalidPath;
        if (!valid_relative_path(relative_path)) return SignalKModelWriteResult::InvalidPath;
        if (relative_path[0] == '\0') {
            return write_document(out, store, config, now_us)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }

        SignalKTypedModelView<Real> view(store, config, now_us);
        char timestamp[32];
        if (!format_signalk_timestamp_utc(timestamp, sizeof(timestamp))) return SignalKModelWriteResult::WriteFailed;

        if (strcmp(relative_path, "version") == 0) {
            return out.append_quoted(signalk_version(config))
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(relative_path, "self") == 0) {
            return out.append_quoted(view.vessel_key())
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(relative_path, "sources") == 0) {
            return write_sources_object(out, view, config)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strncmp(relative_path, "sources/", 8) == 0) {
            return write_source_path(out, relative_path + 8, view, config);
        }
        if (strcmp(relative_path, "vessels") == 0) {
            return write_vessels_object(out, view, timestamp)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strncmp(relative_path, "vessels/", 8) == 0) {
            return write_vessel_path(out, relative_path + 8, view, config, timestamp);
        }
        return SignalKModelWriteResult::NotFound;
    }

private:
    static const char* signalk_version(const SignalKMiniConfig& config) {
        return config.identity.signalk_version && config.identity.signalk_version[0]
            ? config.identity.signalk_version : "1.8.2";
    }

    static bool valid_relative_path(const char* path) {
        if (!path || path[0] == '/') return false;
        if (strchr(path, '?') || strchr(path, '#') || strchr(path, '%') || strchr(path, '\\')) return false;
        if (path[0] == '\0') return true;
        const size_t len = strlen(path);
        if (len >= 384 || path[len - 1] == '/') return false;
        const char* segment = path;
        while (*segment) {
            const char* slash = strchr(segment, '/');
            const size_t segment_len = slash ? static_cast<size_t>(slash - segment) : strlen(segment);
            if (segment_len == 0 || segment_len >= 192) return false;
            if ((segment_len == 1 && segment[0] == '.') ||
                (segment_len == 2 && segment[0] == '.' && segment[1] == '.')) return false;
            if (!slash) break;
            segment = slash + 1;
        }
        return true;
    }

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

    SignalKModelWriteResult write_identity_path(SignalKJsonStreamWriter& out,
                                                const char* path,
                                                const char* vessel_key) const {
        const SignalKVesselIdentityKind kind = signalk_vessel_identity_kind(vessel_key);
        if (strcmp(path, "uuid") == 0 && kind == SignalKVesselIdentityKind::Uuid) {
            return out.append_quoted(vessel_key) ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(path, "mmsi") == 0 && kind == SignalKVesselIdentityKind::Mmsi) {
            static constexpr char Prefix[] = "urn:mrn:imo:mmsi:";
            return out.append_quoted(vessel_key + sizeof(Prefix) - 1)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(path, "url") == 0 && kind == SignalKVesselIdentityKind::Url) {
            return out.append_quoted(vessel_key) ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        return SignalKModelWriteResult::NotFound;
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

    bool has_branch_values(const SignalKTypedModelView<Real>& view, const char* prefix) const {
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            const ModelField field = static_cast<ModelField>(i);
            SignalKMappedValue<Real> mapped;
            if (!view.current_value(field, mapped) || skip_from_self_projection(field, mapped)) continue;
            const char* remainder = nullptr;
            if (path_under_prefix(mapped.path, prefix, remainder)) return true;
        }
        return false;
    }

    bool find_exact_value(const SignalKTypedModelView<Real>& view,
                          const char* path,
                          SignalKMappedValue<Real>& selected) const {
        bool found = false;
        size_t selected_index = 0;
        const size_t count = static_cast<size_t>(ModelField::Count);
        for (size_t i = 1; i < count; ++i) {
            const ModelField field = static_cast<ModelField>(i);
            SignalKMappedValue<Real> candidate;
            if (!view.current_value(field, candidate) || skip_from_self_projection(field, candidate) ||
                strcmp(candidate.path, path) != 0) continue;
            if (!found || candidate.last_update_us > selected.last_update_us ||
                (candidate.last_update_us == selected.last_update_us && i < selected_index)) {
                selected = candidate;
                selected_index = i;
                found = true;
            }
        }
        return found;
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

    bool write_self_vessel(SignalKJsonStreamWriter& out,
                           const SignalKTypedModelView<Real>& view,
                           const char* timestamp) const {
        if (!out.append_char('{') || !write_vessel_identity(out, view.vessel_key())) return false;
        if (has_self_projection_values(view)) {
            if (!out.append_char(',') || !write_branch(out, view, "", timestamp)) return false;
        }
        return out.append_char('}');
    }

    SignalKModelWriteResult write_self_path(SignalKJsonStreamWriter& out,
                                            const char* path,
                                            const SignalKTypedModelView<Real>& view,
                                            const char* timestamp) const {
        if (!path || path[0] == '\0') {
            return write_self_vessel(out, view, timestamp)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        const SignalKModelWriteResult identity = write_identity_path(out, path, view.vessel_key());
        if (identity != SignalKModelWriteResult::NotFound) return identity;

        char dotted[256];
        const size_t len = strlen(path);
        if (len == 0 || len >= sizeof(dotted)) return SignalKModelWriteResult::InvalidPath;
        for (size_t i = 0; i <= len; ++i) dotted[i] = path[i] == '/' ? '.' : path[i];

        SignalKMappedValue<Real> mapped;
        if (find_exact_value(view, dotted, mapped)) {
            return write_leaf(out, view, mapped, timestamp)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (!has_branch_values(view, dotted)) return SignalKModelWriteResult::NotFound;
        if (!out.append_char('{') || !write_branch(out, view, dotted, timestamp) || !out.append_char('}')) {
            return SignalKModelWriteResult::WriteFailed;
        }
        return SignalKModelWriteResult::Written;
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

    bool write_source_value(SignalKJsonStreamWriter& out, const char* label, const char* type) const {
        return out.append_raw("{\"label\":") && out.append_quoted(label ? label : "signalk-mini") &&
               out.append_raw(",\"type\":") && out.append_quoted(type) && out.append_char('}');
    }

    bool write_sources_object(SignalKJsonStreamWriter& out,
                              const SignalKTypedModelView<Real>& view,
                              const SignalKMiniConfig& config) const {
        if (!out.append_char('{')) return false;
        bool first = true;
        auto write_source = [&](SourceId source_id, const char* label, const char* type) -> bool {
            char key[24];
            if (!view.source_key(source_id, key, sizeof(key))) return false;
            if (!first && !out.append_char(',')) return false;
            first = false;
            return out.append_quoted(key) && out.append_char(':') && write_source_value(out, label, type);
        };

        if (!write_source(0, view.source_label(0), "SignalK")) return false;
        for (size_t i = 0; i < config.connector_count; ++i) {
            const ConnectorConfig& connector = config.connectors[i];
            if (!connector.enabled) continue;
            const SourceId source_id = static_cast<SourceId>(FirstConnectorSourceId + i);
            if (!write_source(source_id, view.source_label(source_id), connector_type(connector.protocol.kind))) return false;
        }
        return out.append_char('}');
    }

    SignalKModelWriteResult write_source_path(SignalKJsonStreamWriter& out,
                                              const char* path,
                                              const SignalKTypedModelView<Real>& view,
                                              const SignalKMiniConfig& config) const {
        if (!path || !path[0] || strchr(path, '/')) return SignalKModelWriteResult::NotFound;
        char key[24];
        if (view.source_key(0, key, sizeof(key)) && strcmp(path, key) == 0) {
            return write_source_value(out, view.source_label(0), "SignalK")
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        for (size_t i = 0; i < config.connector_count; ++i) {
            const ConnectorConfig& connector = config.connectors[i];
            if (!connector.enabled) continue;
            const SourceId source_id = static_cast<SourceId>(FirstConnectorSourceId + i);
            if (!view.source_key(source_id, key, sizeof(key)) || strcmp(path, key) != 0) continue;
            return write_source_value(out, view.source_label(source_id), connector_type(connector.protocol.kind))
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        return SignalKModelWriteResult::NotFound;
    }

    bool write_vessels_object(SignalKJsonStreamWriter& out,
                              const SignalKTypedModelView<Real>& view,
                              const char* timestamp) const {
        if (!out.append_char('{') || !out.append_quoted(view.vessel_key()) || !out.append_char(':') ||
            !write_self_vessel(out, view, timestamp)) return false;
        if (!write_ais_vessels(out, view, timestamp)) return false;
        return out.append_char('}');
    }

    SignalKModelWriteResult write_vessel_path(SignalKJsonStreamWriter& out,
                                              const char* path,
                                              const SignalKTypedModelView<Real>& view,
                                              const SignalKMiniConfig& config,
                                              const char* timestamp) const {
        (void)config;
        if (!path || !path[0]) return SignalKModelWriteResult::NotFound;
        const char* slash = strchr(path, '/');
        const size_t key_len = slash ? static_cast<size_t>(slash - path) : strlen(path);
        const char* remainder = slash ? slash + 1 : "";

        if ((key_len == 4 && strncmp(path, "self", 4) == 0) ||
            (strlen(view.vessel_key()) == key_len && strncmp(path, view.vessel_key(), key_len) == 0)) {
            return write_self_path(out, remainder, view, timestamp);
        }

        const auto* target = find_ais_target(view, path, key_len);
        if (!target) return SignalKModelWriteResult::NotFound;
        return write_ais_path(out, *target, remainder, view, timestamp);
    }

    const ship_data_model::AisTargetData<Real>* find_ais_target(const SignalKTypedModelView<Real>& view,
                                                       const char* key,
                                                       size_t key_len) const {
        const auto& table = view.model().ais.targets;
        if (!table.target_count.valid ||
            !view.field_is_current(ModelField::AisTargetsObject, table.target_count.last_update_us)) return nullptr;
        for (uint8_t i = 0; i < ship_data_model::AIS_TARGET_TABLE_CAPACITY; ++i) {
            const auto& target = table.targets[i];
            if (!target.occupied || !view.ais_target_is_current(target) ||
                !target.mmsi.valid || !is_vessel_mmsi(target.mmsi.value)) continue;
            char mmsi[10];
            if (!format_mmsi(mmsi, sizeof(mmsi), target.mmsi.value)) continue;
            char vessel_key[48];
            const int len = snprintf(vessel_key, sizeof(vessel_key), "urn:mrn:imo:mmsi:%s", mmsi);
            if (len <= 0 || static_cast<size_t>(len) >= sizeof(vessel_key)) continue;
            if (static_cast<size_t>(len) == key_len && strncmp(key, vessel_key, key_len) == 0) return &target;
        }
        return nullptr;
    }

    bool write_ais_vessel(SignalKJsonStreamWriter& out,
                          const ship_data_model::AisTargetData<Real>& target,
                          const char* source_key,
                          const char* timestamp) const {
        if (!out.append_char('{')) return false;
        bool first = true;
        char mmsi[10];
        if (!format_mmsi(mmsi, sizeof(mmsi), target.mmsi.value)) return false;
        if (target.vessel_name[0]) {
            if (!out.append_raw("\"name\":") || !out.append_quoted(target.vessel_name)) return false;
            first = false;
        }
        if (target.mmsi.valid) {
            if (!first && !out.append_char(',')) return false;
            first = false;
            if (!out.append_raw("\"mmsi\":") || !out.append_quoted(mmsi)) return false;
        }
        const bool has_position = target.latitude_deg.valid && target.longitude_deg.valid;
        const bool has_navigation = has_position || target.speed_over_ground_kn.valid ||
                                    target.course_over_ground_deg.valid || target.true_heading_deg.valid;
        if (has_navigation) {
            if (!first && !out.append_char(',')) return false;
            if (!out.append_raw("\"navigation\":")) return false;
            if (!write_ais_navigation(out, target, source_key, timestamp)) return false;
        }
        return out.append_char('}');
    }

    bool write_ais_navigation(SignalKJsonStreamWriter& out,
                              const ship_data_model::AisTargetData<Real>& target,
                              const char* source_key,
                              const char* timestamp) const {
        if (!out.append_char('{')) return false;
        bool first = true;
        const bool has_position = target.latitude_deg.valid && target.longitude_deg.valid;
        if (has_position) {
            if (!out.append_raw("\"position\":{\"value\":{\"latitude\":")) return false;
            if (!out.append_real(target.latitude_deg.value) || !out.append_raw(",\"longitude\":")) return false;
            if (!out.append_real(target.longitude_deg.value) || !out.append_raw("},\"$source\":")) return false;
            if (!out.append_quoted(source_key) || !out.append_raw(",\"timestamp\":")) return false;
            if (!out.append_quoted(timestamp) || !out.append_char('}')) return false;
            first = false;
        }
        if (target.speed_over_ground_kn.valid && !write_ais_number_leaf(out, first, "speedOverGround",
                knots_to_mps<Real>(target.speed_over_ground_kn.value), source_key, timestamp)) return false;
        if (target.speed_over_ground_kn.valid) first = false;
        if (target.course_over_ground_deg.valid && !write_ais_number_leaf(out, first, "courseOverGroundTrue",
                deg_to_rad<Real>(target.course_over_ground_deg.value), source_key, timestamp)) return false;
        if (target.course_over_ground_deg.valid) first = false;
        if (target.true_heading_deg.valid && !write_ais_number_leaf(out, first, "headingTrue",
                deg_to_rad<Real>(target.true_heading_deg.value), source_key, timestamp)) return false;
        return out.append_char('}');
    }

    SignalKModelWriteResult write_ais_path(SignalKJsonStreamWriter& out,
                                           const ship_data_model::AisTargetData<Real>& target,
                                           const char* path,
                                           const SignalKTypedModelView<Real>& view,
                                           const char* timestamp) const {
        const SourceId source_id = view.current_value_source(ModelField::AisTargetsObject);
        char source_key[24];
        if (!view.source_key(source_id, source_key, sizeof(source_key))) return SignalKModelWriteResult::WriteFailed;
        if (!path || path[0] == '\0') {
            return write_ais_vessel(out, target, source_key, timestamp)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(path, "name") == 0 && target.vessel_name[0]) {
            return out.append_quoted(target.vessel_name)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(path, "mmsi") == 0 && target.mmsi.valid) {
            char mmsi[10];
            if (!format_mmsi(mmsi, sizeof(mmsi), target.mmsi.value)) return SignalKModelWriteResult::WriteFailed;
            return out.append_quoted(mmsi) ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(path, "navigation") == 0) {
            const bool any = (target.latitude_deg.valid && target.longitude_deg.valid) || target.speed_over_ground_kn.valid ||
                             target.course_over_ground_deg.valid || target.true_heading_deg.valid;
            if (!any) return SignalKModelWriteResult::NotFound;
            return write_ais_navigation(out, target, source_key, timestamp)
                ? SignalKModelWriteResult::Written : SignalKModelWriteResult::WriteFailed;
        }
        if (strcmp(path, "navigation/position") == 0 && target.latitude_deg.valid && target.longitude_deg.valid) {
            if (!out.append_raw("{\"value\":{\"latitude\":")) return SignalKModelWriteResult::WriteFailed;
            if (!out.append_real(target.latitude_deg.value) || !out.append_raw(",\"longitude\":")) return SignalKModelWriteResult::WriteFailed;
            if (!out.append_real(target.longitude_deg.value) || !out.append_raw("},\"$source\":")) return SignalKModelWriteResult::WriteFailed;
            if (!out.append_quoted(source_key) || !out.append_raw(",\"timestamp\":")) return SignalKModelWriteResult::WriteFailed;
            if (!out.append_quoted(timestamp) || !out.append_char('}')) return SignalKModelWriteResult::WriteFailed;
            return SignalKModelWriteResult::Written;
        }
        Real value{};
        if (strcmp(path, "navigation/speedOverGround") == 0 && target.speed_over_ground_kn.valid) {
            value = knots_to_mps<Real>(target.speed_over_ground_kn.value);
        } else if (strcmp(path, "navigation/courseOverGroundTrue") == 0 && target.course_over_ground_deg.valid) {
            value = deg_to_rad<Real>(target.course_over_ground_deg.value);
        } else if (strcmp(path, "navigation/headingTrue") == 0 && target.true_heading_deg.valid) {
            value = deg_to_rad<Real>(target.true_heading_deg.value);
        } else {
            return SignalKModelWriteResult::NotFound;
        }
        if (!out.append_raw("{\"value\":") || !out.append_real(value) || !out.append_raw(",\"$source\":")) {
            return SignalKModelWriteResult::WriteFailed;
        }
        if (!out.append_quoted(source_key) || !out.append_raw(",\"timestamp\":")) return SignalKModelWriteResult::WriteFailed;
        if (!out.append_quoted(timestamp) || !out.append_char('}')) return SignalKModelWriteResult::WriteFailed;
        return SignalKModelWriteResult::Written;
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
            if (!out.append_char(',') || !out.append_quoted(vessel_key) || !out.append_char(':')) return false;
            if (!write_ais_vessel(out, target, source_key, timestamp)) return false;
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
