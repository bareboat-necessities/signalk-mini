#pragma once

#include <string.h>
#include "core_data_types.hpp"
#include "field_def.hpp"

namespace ship_data_model {

template<typename Real> struct DataModel;

inline void copy_data_text(char* out, size_t out_size, const char* text) {
    if (!out || out_size == 0) return;
    size_t i = 0;
    if (text) {
        for (; i + 1 < out_size && text[i]; ++i) out[i] = text[i];
    }
    out[i] = '\0';
}

inline bool data_model_name_starts_with(const char* text, const char* prefix) {
    if (!text || !prefix) return false;
    return strncmp(text, prefix, strlen(prefix)) == 0;
}

inline bool looks_like_pypilot_runtime_name(const char* name) {
    return data_model_name_starts_with(name, "ap.") ||
           data_model_name_starts_with(name, "imu.") ||
           data_model_name_starts_with(name, "gps.") ||
           data_model_name_starts_with(name, "wind.") ||
           data_model_name_starts_with(name, "truewind.") ||
           data_model_name_starts_with(name, "apb.") ||
           data_model_name_starts_with(name, "water.") ||
           data_model_name_starts_with(name, "rudder.") ||
           data_model_name_starts_with(name, "servo.");
}

inline const FieldMeta* field_meta_from_name(const char* name) {
    if (!name) return nullptr;
    for (size_t i = 0; i < field_definition_count; ++i) {
        if (strcmp(field_definitions[i].pypilot_name, name) == 0) return &field_definitions[i];
    }
    return nullptr;
}

inline FieldId field_id_from_name(const char* name) {
    const FieldMeta* meta = field_meta_from_name(name);
    if (meta) return meta->id;
    return looks_like_pypilot_runtime_name(name) ? FieldId::compat_value : FieldId::unknown;
}

inline const FieldMeta* field_meta(FieldId id) {
    for (size_t i = 0; i < field_definition_count; ++i) if (field_definitions[i].id == id) return &field_definitions[i];
    return nullptr;
}

inline const char* pypilot_name(FieldId id) {
    const FieldMeta* meta = field_meta(id);
    return meta ? meta->pypilot_name : "";
}

template<typename Real>
inline bool apply_number(DataModel<Real>& model, FieldId id, Real value, uint64_t now_us) {
    switch (id) {
    case FieldId::server_uptime_s: model.server.uptime_s.set(value, now_us); return true;
    case FieldId::ap_heading_deg: model.ap.heading_deg.set(value, now_us); return true;
    case FieldId::ap_tack_progress_0_1: model.tack.progress_0_1.set(value, now_us); return true;
    case FieldId::wind_filtered_direction_deg: model.wind.apparent.filtered_direction_deg.set(value, now_us); return true;
    case FieldId::servo_current_a: model.servo.current_a.set(value, now_us); return true;
    case FieldId::servo_telemetry_current_a: model.servo_telemetry.current_a.set(value, now_us); return true;
    case FieldId::pilot_basic_P: model.pilots.basic.P.gain.value = value; return true;
    case FieldId::pilot_basic_Pgain: model.pilots.basic.P.contribution.set(value, now_us); return true;
    case FieldId::imu_calibration_heading_offset_deg: model.imu.heading_offset_deg.value = value; return true;
    default: return false;
    }
}

template<typename Real>
inline bool read_number(const DataModel<Real>& model, FieldId id, Real& out) {
    switch (id) {
    case FieldId::server_uptime_s: out = model.server.uptime_s.value; return model.server.uptime_s.valid;
    case FieldId::ap_heading_deg: out = model.ap.heading_deg.value; return model.ap.heading_deg.valid;
    case FieldId::servo_telemetry_current_a: out = model.servo_telemetry.current_a.value; return model.servo_telemetry.current_a.valid;
    case FieldId::pilot_basic_P: out = model.pilots.basic.P.gain.value; return true;
    case FieldId::imu_calibration_heading_offset_deg: out = model.imu.heading_offset_deg.value; return true;
    default: return false;
    }
}

template<typename Real>
inline bool apply_bool(DataModel<Real>& model, FieldId id, bool value, uint64_t) {
    if (id == FieldId::ap_enabled) { model.ap.enabled.value = value; return true; }
    return false;
}

template<typename Real>
inline bool read_bool(const DataModel<Real>& model, FieldId id, bool& out) {
    if (id == FieldId::ap_enabled) { out = model.ap.enabled.value; return true; }
    return false;
}

template<typename Real>
inline bool apply_mode(DataModel<Real>& model, FieldId id, AutopilotMode value, uint64_t) {
    if (id == FieldId::ap_mode) { model.ap.mode.value = value; return true; }
    return false;
}

template<typename Real>
inline bool apply_source(DataModel<Real>& model, FieldId id, SensorSource value, uint64_t) {
    if (id == FieldId::wind_source) { model.wind.apparent.source.value = value; return true; }
    return false;
}

template<typename Real>
inline bool apply_pilot(DataModel<Real>& model, FieldId id, PilotName value, uint64_t) {
    if (id == FieldId::ap_pilot) { model.ap.pilot.value = value; return true; }
    return false;
}

template<typename Real>
inline bool apply_rudder_calibration_state(DataModel<Real>& model, FieldId id, RudderCalibrationState value, uint64_t) {
    if (id == FieldId::rudder_calibration_state) { model.rudder.calibration_state.value = value; return true; }
    return false;
}

template<typename Real>
inline bool apply_string(DataModel<Real>& model, FieldId id, const char* value, uint64_t) {
    if (id == FieldId::server_version) { copy_data_text(model.server.version, sizeof(model.server.version), value); return true; }
    if (id == FieldId::status_last_error) { copy_data_text(model.status.last_error, sizeof(model.status.last_error), value); return true; }
    return false;
}

template<typename Real>
inline bool read_string(const DataModel<Real>& model, FieldId id, const char*& out) {
    if (id == FieldId::server_version) { out = model.server.version; return true; }
    if (id == FieldId::status_last_error) { out = model.status.last_error; return true; }
    return false;
}

} // namespace ship_data_model
