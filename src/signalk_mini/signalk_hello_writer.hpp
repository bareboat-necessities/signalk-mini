#pragma once

#include <stddef.h>
#include <ArduinoJson.h>

namespace signalk_mini {

class SignalKHelloWriter {
public:
    int write(char* dst,
              size_t dst_size,
              const char* server_name,
              const char* signalk_version,
              const char* self_path,
              const char* timestamp = nullptr) const {
        if (!dst || dst_size == 0) return 0;

        JsonDocument doc;
        doc["name"] = server_name && server_name[0] ? server_name : "signalk-mini";
        doc["version"] = signalk_version && signalk_version[0] ? signalk_version : "1.8.2";
        if (timestamp && timestamp[0]) doc["timestamp"] = timestamp;
        doc["self"] = self_path && self_path[0]
            ? self_path
            : "vessels.urn:mrn:signalk:uuid:00000000-0000-4000-8000-000000000001";
        JsonArray roles = doc["roles"].to<JsonArray>();
        roles.add("master");
        roles.add("main");

        const size_t json_len = measureJson(doc);
        if (json_len == 0 || json_len + 3 > dst_size) return 0;

        const size_t len = serializeJson(doc, dst, dst_size);
        if (len != json_len) return 0;
        dst[len] = '\r';
        dst[len + 1] = '\n';
        dst[len + 2] = '\0';
        return static_cast<int>(len + 2);
    }
};

} // namespace signalk_mini
