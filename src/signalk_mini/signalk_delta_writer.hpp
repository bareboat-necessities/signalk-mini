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

        return serialize_delta(dst, dst_size, doc);
    }

    int write_mapped(char* dst, size_t dst_size, const char* source_label, const SignalKMappedValue<Real>& value) const {
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
        }

        return serialize_delta(dst, dst_size, doc);
    }

private:
    int serialize_delta(char* dst, size_t dst_size, JsonDocument& doc) const {
        const size_t len = serializeJson(doc, dst, dst_size);
        if (len + 2 >= dst_size) return 0;
        dst[len] = '\r';
        dst[len + 1] = '\n';
        dst[len + 2] = '\0';
        return static_cast<int>(len + 2);
    }
};

} // namespace signalk_mini
