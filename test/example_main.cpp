#include <Arduino.h>
#include "ds24xx_gpio.h"

ds24xx::DS24xxComponent *comp = nullptr;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("DS24xx minimal test starting");

  // OneWire pin 4 is used as an example; change for your hardware.
  comp = new ds24xx::DS24xxComponent(4);
  comp->setup();

  // Try setting channel 0 high and report result
  bool ok = comp->write_channel(0, true);
  Serial.print("write_channel(0, true): ");
  Serial.println(ok ? "OK" : "FAIL");

  // Allow component to poll/update once
  comp->loop();
}

void loop() {
  delay(5000);
  if (!comp) return;
  // Toggle channel 0 low
  bool ok = comp->write_channel(0, false);
  Serial.print("write_channel(0, false): ");
  Serial.println(ok ? "OK" : "FAIL");
  comp->loop();
}
