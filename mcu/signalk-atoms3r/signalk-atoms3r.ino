#define ASYNC_EVENT_LOOP_ENABLE_ARDUINO_WIFI_TCP

#include <Arduino.h>
#include <M5Unified.h>
#include <signalk_mini.hpp>

signalk_mini::SignalKMiniConfig config;
signalk_mini::SignalKMiniApp<float> app(config);
bool setup_failed = false;

void setup() {
  auto m5_config = M5.config();
  M5.begin(m5_config);

  Serial.begin(115200);
  delay(100);

  M5.Display.setTextSize(1);
  M5.Display.println("signalk-mini");

  if (!app.begin()) {
    Serial.println("signalk-mini setup failed");
    M5.Display.println("setup failed");
    setup_failed = true;
    return;
  }

  Serial.println("signalk-mini started");
  M5.Display.println("started");
}

void loop() {
  M5.update();

  if (setup_failed) {
    delay(1000);
    return;
  }

  app.tick();
}
