#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP

#include <Arduino.h>
#include <WiFi.h>
#include <signalk_mini.hpp>

#if __has_include("signalk_generic_esp32_wifi_secret.h")
#include "signalk_generic_esp32_wifi_secret.h"
#endif

#ifndef SIGNALK_WIFI_SSID
#define SIGNALK_WIFI_SSID ""
#endif

#ifndef SIGNALK_WIFI_PASSWORD
#define SIGNALK_WIFI_PASSWORD ""
#endif

#ifndef SIGNALK_AP_SSID
#define SIGNALK_AP_SSID "signalk-mini-esp32"
#endif

#ifndef SIGNALK_AP_PASSWORD
#define SIGNALK_AP_PASSWORD ""
#endif

#ifndef SIGNALK_SERVER_PORT
#define SIGNALK_SERVER_PORT 20223
#endif

#ifndef SIGNALK_NMEA_BAUD
#define SIGNALK_NMEA_BAUD 4800
#endif

#ifndef SIGNALK_NMEA_RX_PIN
#define SIGNALK_NMEA_RX_PIN -1
#endif

#ifndef SIGNALK_NMEA_TX_PIN
#define SIGNALK_NMEA_TX_PIN -1
#endif

#ifndef SIGNALK_STATUS_INTERVAL_MS
#define SIGNALK_STATUS_INTERVAL_MS 1000
#endif

#ifndef SIGNALK_WIFI_CONNECT_TIMEOUT_MS
#define SIGNALK_WIFI_CONNECT_TIMEOUT_MS 12000
#endif

namespace {

constexpr size_t nmea_buffer_size = 160;
char signalk_self_context[80]{};

signalk_mini::SignalKMiniConfig make_config() {
  const uint64_t chip_id = ESP.getEfuseMac();
  signalk_mini::signalk_make_uuid_context(
    signalk_self_context, sizeof(signalk_self_context),
    0x534b4d494e490000ULL ^ chip_id,
    0x47454e4553503332ULL ^ (chip_id << 1));
  return signalk_mini::make_sketch_owned_io_config(
    "signalk-mini-generic-esp32",
    "generic-esp32",
    SIGNALK_SERVER_PORT,
    10000,
    32,
    512,
    signalk_self_context);
}

signalk_mini::SignalKMiniConfig config = make_config();
signalk_mini::SignalKMiniApp<float> app(config);

bool setup_failed = false;
bool nmea_uart_enabled = false;
bool wifi_station_mode = false;
uint32_t nmea_lines_seen = 0;
uint32_t nmea_lines_accepted = 0;
uint32_t last_status_ms = 0;
char nmea_line[nmea_buffer_size];
size_t nmea_line_len = 0;

bool wifi_configured() {
  return SIGNALK_WIFI_SSID[0] != '\0';
}

bool start_wifi_station() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(SIGNALK_WIFI_SSID, SIGNALK_WIFI_PASSWORD);

  const uint32_t start_ms = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start_ms < SIGNALK_WIFI_CONNECT_TIMEOUT_MS) {
    delay(100);
  }

  wifi_station_mode = WiFi.status() == WL_CONNECTED;
  return wifi_station_mode;
}

bool start_wifi_ap() {
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (SIGNALK_AP_PASSWORD[0] != '\0') {
    return WiFi.softAP(SIGNALK_AP_SSID, SIGNALK_AP_PASSWORD);
  }
  return WiFi.softAP(SIGNALK_AP_SSID);
}

IPAddress active_ip() {
  return wifi_station_mode ? WiFi.localIP() : WiFi.softAPIP();
}

void begin_wifi() {
  Serial.println("WiFi starting");

  if (wifi_configured() && start_wifi_station()) {
    Serial.print("WiFi STA connected: ");
    Serial.println(active_ip());
    return;
  }

  if (!start_wifi_ap()) {
    setup_failed = true;
    Serial.println("WiFi setup failed");
    return;
  }

  Serial.print("WiFi AP started: ");
  Serial.println(SIGNALK_AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(active_ip());
}

void begin_nmea_uart() {
#if SIGNALK_NMEA_RX_PIN >= 0
  Serial1.begin(SIGNALK_NMEA_BAUD, SERIAL_8N1, SIGNALK_NMEA_RX_PIN, SIGNALK_NMEA_TX_PIN);
  nmea_uart_enabled = true;
  Serial.print("NMEA0183 UART enabled: RX=");
  Serial.print(SIGNALK_NMEA_RX_PIN);
  Serial.print(" TX=");
  Serial.print(SIGNALK_NMEA_TX_PIN);
  Serial.print(" baud=");
  Serial.println(SIGNALK_NMEA_BAUD);
#else
  nmea_uart_enabled = false;
  Serial.println("NMEA0183 UART disabled; define SIGNALK_NMEA_RX_PIN to enable Serial1 input");
#endif
}

void publish_nmea_line(const char* line) {
  if (!line || !*line) return;
  ++nmea_lines_seen;
  if (app.nmea0183().feed_line(line, signalk_mini::SourceId(1), static_cast<uint64_t>(micros()), true)) {
    ++nmea_lines_accepted;
  }
}

void poll_nmea_uart() {
  if (!nmea_uart_enabled) return;

  while (Serial1.available() > 0) {
    const int ch = Serial1.read();
    if (ch < 0) break;

    if (ch == '\r') continue;
    if (ch == '\n') {
      nmea_line[nmea_line_len] = '\0';
      publish_nmea_line(nmea_line);
      nmea_line_len = 0;
      continue;
    }

    if (nmea_line_len + 1 < sizeof(nmea_line)) {
      nmea_line[nmea_line_len++] = static_cast<char>(ch);
    } else {
      nmea_line_len = 0;
    }
  }
}

void update_status() {
  const uint32_t now = millis();
  if (now - last_status_ms < SIGNALK_STATUS_INTERVAL_MS) return;
  last_status_ms = now;

  Serial.print(wifi_station_mode ? "STA " : "AP ");
  Serial.print(active_ip());
  Serial.print(" SignalK:");
  Serial.print(SIGNALK_SERVER_PORT);
  Serial.print(" NMEA:");
  Serial.print(nmea_uart_enabled ? "on" : "off");
  Serial.print(" seen:");
  Serial.print(nmea_lines_seen);
  Serial.print(" accepted:");
  Serial.println(nmea_lines_accepted);
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("signalk-mini generic ESP32 boot");

  begin_wifi();
  if (setup_failed) return;

  begin_nmea_uart();

  if (!app.begin()) {
    setup_failed = true;
    Serial.println("signalk-mini app begin failed");
    return;
  }

  Serial.println("Signal K server started");
  update_status();
}

void loop() {
  if (setup_failed) {
    delay(1000);
    return;
  }

  poll_nmea_uart();
  app.tick();
  update_status();
}
