#pragma once

#include <stddef.h>
#include <ArduinoJson.h>
#include "signalk_mapper.hpp"

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

        const size_t len = serializeJson(doc, dst, dst_size);
        if (len + 2 >= dst_size) return 0;
        dst[len] = '\r';
        dst[len + 1] = '\n';
        dst[len + 2] = '\0';
        return static_cast<int>(len + 2);
    }

    int write_mapped(char* dst, size_t dst_size, const char* source_label, const SignalKMappedValue<Real>& value) const {
        return write_number(dst, dst_size, source_label, value.path, value.number);
    }
};

} // namespace signalk_mini
