#pragma once

#if defined(ARDUINO)

#include <Arduino.h>
#include <WiFi.h>
#include <string.h>

namespace async_event_loop {

struct WiFiConnectOptions {
    const char* ssid = nullptr;
    const char* password = nullptr;
    uint32_t timeout_ms = 15000UL;
    uint32_t poll_interval_ms = 250UL;
    bool station_mode = true;
    bool reject_placeholder_credentials = true;
    const char* placeholder_ssid = "ssid";
};

enum class WiFiConnectResult {
    Connected,
    MissingCredentials,
    Timeout
};

inline bool wifi_credentials_configured(const char* ssid, const char* placeholder_ssid = "ssid") {
    if (!ssid || ssid[0] == '\0') {
        return false;
    }
    if (placeholder_ssid && placeholder_ssid[0] != '\0' && strcmp(ssid, placeholder_ssid) == 0) {
        return false;
    }
    return true;
}

inline const char* wifi_connect_result_text(WiFiConnectResult result) {
    switch (result) {
    case WiFiConnectResult::Connected:
        return "wifi connected";
    case WiFiConnectResult::MissingCredentials:
        return "wifi credentials are placeholders";
    case WiFiConnectResult::Timeout:
        return "wifi connect timeout";
    }
    return "wifi connect failed";
}

inline WiFiConnectResult connect_wifi_result(const WiFiConnectOptions& options) {
    if (options.reject_placeholder_credentials &&
        !wifi_credentials_configured(options.ssid, options.placeholder_ssid)) {
        return WiFiConnectResult::MissingCredentials;
    }

    if (options.station_mode) {
        WiFi.mode(WIFI_STA);
    }
    WiFi.begin(options.ssid ? options.ssid : "", options.password ? options.password : "");

    const unsigned long start_ms = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start_ms >= options.timeout_ms) {
            return WiFiConnectResult::Timeout;
        }
        delay(options.poll_interval_ms);
    }
    return WiFiConnectResult::Connected;
}

inline WiFiConnectResult connect_wifi_result(const char* ssid,
                                             const char* password,
                                             uint32_t timeout_ms = 15000UL,
                                             uint32_t poll_interval_ms = 250UL) {
    WiFiConnectOptions options;
    options.ssid = ssid;
    options.password = password;
    options.timeout_ms = timeout_ms;
    options.poll_interval_ms = poll_interval_ms;
    return connect_wifi_result(options);
}

inline bool connect_wifi(const WiFiConnectOptions& options) {
    return connect_wifi_result(options) == WiFiConnectResult::Connected;
}

inline bool connect_wifi(const char* ssid,
                         const char* password,
                         uint32_t timeout_ms = 15000UL,
                         uint32_t poll_interval_ms = 250UL) {
    return connect_wifi_result(ssid, password, timeout_ms, poll_interval_ms) == WiFiConnectResult::Connected;
}

} // namespace async_event_loop

#endif // defined(ARDUINO)
