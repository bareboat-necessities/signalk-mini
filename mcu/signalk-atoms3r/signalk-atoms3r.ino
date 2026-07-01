#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP
#include <Arduino.h>
#include <signalk_mini/server.hpp>

signalk_mini::SignalKMiniConfig config;
signalk_mini::MiniSignalKServer<float> app(config);
bool setup_failed = false;

void setup() {
  Serial.begin(115200);
  if (!app.begin()) {
    Serial.println("signalk-mini setup failed");
    setup_failed = true;
  }
}

void loop() {
  if (setup_failed) {
    delay(1000);
    return;
  }
  app.tick();
}
