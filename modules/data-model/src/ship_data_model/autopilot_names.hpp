#pragma once

#include <string.h>
#include "core_data_types.hpp"

namespace ship_data_model {

inline bool sensor_source_from_name(const char* name, SensorSource& out) {
    if (!name) return false;
    if (strcmp(name, "gpsd") == 0) out = SensorSource::gpsd;
    else if (strcmp(name, "servo") == 0) out = SensorSource::servo;
    else if (strcmp(name, "serial") == 0) out = SensorSource::serial;
    else if (strcmp(name, "tcp") == 0) out = SensorSource::tcp;
    else if (strcmp(name, "signalk") == 0) out = SensorSource::signalk;
    else if (strcmp(name, "water_wind") == 0) out = SensorSource::water_wind;
    else if (strcmp(name, "gps_wind") == 0) out = SensorSource::gps_wind;
    else if (strcmp(name, "none") == 0) out = SensorSource::none;
    else return false;
    return true;
}

inline bool pilot_from_name(const char* name, PilotName& out) {
    if (!name) return false;
    if (strcmp(name, "basic") == 0) out = PilotName::basic;
    else if (strcmp(name, "absolute") == 0) out = PilotName::absolute;
    else if (strcmp(name, "wind") == 0) out = PilotName::wind;
    else if (strcmp(name, "gps") == 0) out = PilotName::gps;
    else if (strcmp(name, "rate") == 0) out = PilotName::rate;
    else if (strcmp(name, "simple") == 0) out = PilotName::simple;
    else if (strcmp(name, "vmg") == 0) out = PilotName::vmg;
    else if (strcmp(name, "deadzone") == 0) out = PilotName::deadzone;
    else if (strcmp(name, "fuzzy") == 0) out = PilotName::fuzzy;
    else if (strcmp(name, "learning") == 0) out = PilotName::learning;
    else return false;
    return true;
}

} // namespace ship_data_model
