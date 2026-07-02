#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP

#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <signalk_mini.hpp>

#if __has_include("signalk_atoms3r_wifi_secret.h")
#include "signalk_atoms3r_wifi_secret.h"
#endif

#ifndef SIGNALK_WIFI_SSID
#define SIGNALK_WIFI_SSID ""
#endif

#ifndef SIGNALK_WIFI_PASSWORD
#define SIGNALK_WIFI_PASSWORD ""
#endif

#ifndef SIGNALK_AP_SSID
#define SIGNALK_AP_SSID "signalk-mini-atoms3r"
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

signalk_mini::SignalKMiniConfig make_config() {
  signalk_mini::SignalKMiniConfig cfg;
  cfg.identity.server_name = "signalk-mini-atoms3r";
  cfg.identity.server_version = "0.1.0";
  cfg.signalk.host = "0.0.0.0";
  cfg.signalk.port = SIGNALK_SERVER_PORT;
  cfg.signalk.allow_rx = true;
  cfg.signalk.allow_tx = true;
  cfg.publisher.interval_us = 10000;
  cfg.publisher.max_changes_per_tick = 32;
  cfg.publisher.json_buffer_size = 512;
  cfg.publisher.source_label = "atoms3r";

  // On AtomS3R the sketch owns the board UART and feeds NMEA into the typed model
  // directly. The core server still owns the Signal K TCP protocol server.
  cfg.connectors = nullptr;
  cfg.connector_count = 0;
  return cfg;
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

void log_line(const char* text) {
  Serial.println(text);
}

void draw_header(const char* state) {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.println("signalk-mini");
  M5.Display.println("AtomS3R");
  M5.Display.println(state);
}

bool wifi_configured() {
  return SIGNALK_WIFI_SSID[0] != '\0';
}

bool start_wifi_station() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(SIGNALK_WIFI_SSID, SIGNALK_WIFI_PASSWORD);

  const uint32_t start_ms = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start_ms < SIGNALK_WIFI_CONNECT_TIMEOUT_MS) {
    M5.update();
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
  draw_header("WiFi starting");

  if (wifi_configured() && start_wifi_station()) {
    Serial.print("WiFi STA connected: ");
    Serial.println(active_ip());
    return;
  }

  if (!start_wifi_ap()) {
    setup_failed = true;
    log_line("WiFi setup failed");
    draw_header("WiFi failed");
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

void draw_status() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.println("signalk-mini AtomS3R");
  M5.Display.print(wifi_station_mode ? "STA " : "AP  ");
  M5.Display.println(active_ip());
  M5.Display.print("Signal K TCP: ");
  M5.Display.println(SIGNALK_SERVER_PORT);
  M5.Display.print("NMEA UART: ");
  M5.Display.println(nmea_uart_enabled ? "on" : "off");
  M5.Display.print("NMEA seen: ");
  M5.Display.println(nmea_lines_seen);
  M5.Display.print("accepted: ");
  M5.Display.println(nmea_lines_accepted);

  const auto& model = app.store().model();
  if (model.wind.apparent.speed_kn.valid) {
    M5.Display.print("AWS kn: ");
    M5.Display.println(model.wind.apparent.speed_kn.value, 1);
  }
  if (model.wind.apparent.direction_deg.valid) {
    M5.Display.print("AWA deg: ");
    M5.Display.println(model.wind.apparent.direction_deg.value, 1);
  }
}

void update_status() {
  const uint32_t now = millis();
  if (now - last_status_ms < SIGNALK_STATUS_INTERVAL_MS) return;
  last_status_ms = now;
  draw_status();
}

} // namespace

void setup() {
  auto m5_config = M5.config();
  M5.begin(m5_config);

  Serial.begin(115200);
  delay(100);

  draw_header("booting");
  log_line("signalk-mini AtomS3R boot");

  begin_wifi();
  if (setup_failed) return;

  begin_nmea_uart();

  if (!app.begin()) {
    setup_failed = true;
    log_line("signalk-mini app begin failed");
    draw_header("app failed");
    return;
  }

  log_line("Signal K server started");
  draw_status();
}

void loop() {
  M5.update();

  if (setup_failed) {
    delay(1000);
    return;
  }

  poll_nmea_uart();
  app.tick();
  update_status();
}
