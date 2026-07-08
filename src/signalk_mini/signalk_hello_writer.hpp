#pragma once

#include <stddef.h>
#include <ArduinoJson.h>

namespace signalk_mini {

class SignalKHelloWriter {
public:
    int write(char* dst,
              size_t dst_size,
              const char* server_name,
              const char* server_version,
              const char* self_path) const {
        if (!dst || dst_size == 0) return 0;

        JsonDocument doc;
        doc["name"] = server_name && server_name[0] ? server_name : "signalk-mini";
        doc["version"] = server_version && server_version[0] ? server_version : "0.1.0";
        doc["self"] = self_path && self_path[0] ? self_path : "vessels.self";
        JsonArray roles = doc["roles"].to<JsonArray>();
        roles.add("master");
        roles.add("main");

        const size_t len = serializeJson(doc, dst, dst_size);
        if (len + 2 >= dst_size) return 0;
        dst[len] = '\r';
        dst[len + 1] = '\n';
        dst[len + 2] = '\0';
        return static_cast<int>(len + 2);
    }
};

} // namespace signalk_mini
