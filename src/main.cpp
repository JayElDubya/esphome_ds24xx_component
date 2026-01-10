#include "esphome.h"
#include "ds24xx_gpio.h"

ds24xx::DS24xxComponent *ds24xx_global = nullptr;

void setup() {
  delay(10);
  ESP_LOGD("main", "Starting DS24xx ESPHome example");

  // Example: create component on OneWire pin 4 and register with App
  ds24xx_global = new ds24xx::DS24xxComponent(4);
  App.register_component(ds24xx_global);

  // Create an output for channel 0 and register it so ESPHome will expose it
  auto *o0 = new ds24xx::DS24xxOutput(ds24xx_global, 0);
  App.register_component(o0);
}

void loop() {
  App.loop();
}