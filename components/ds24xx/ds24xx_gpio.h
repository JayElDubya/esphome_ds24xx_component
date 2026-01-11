#pragma once

#include "esphome.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"

// Lightweight OneWire stub if real OneWire not available
#if defined(__has_include)
#  if __has_include(<OneWire.h>)
#    include <OneWire.h>
#  else
#    define DS24XX_NO_ONEWIRE
#  endif
#else
#  include <OneWire.h>
#endif

#if defined(DS24XX_NO_ONEWIRE)
class OneWire {
 public:
  explicit OneWire(uint8_t) {}
  void reset_search() {}
  bool search(uint8_t *) { return false; }
  void reset() {}
  void select(const uint8_t *) {}
  void write(uint8_t) {}
  void write(uint8_t, uint8_t) {}
  uint8_t read() { return 0; }
  static uint8_t crc8(const uint8_t *, uint8_t) { return 0; }
};
#endif

#if defined(__has_include)
#  if __has_include("esphome/components/onewire/onewire.h")
#    include "esphome/components/onewire/onewire.h"
#    define DS24XX_HAVE_ESPHOME_ONEWIRE 1
#  endif
#endif

#include <vector>
#include <array>
#include "esphome/core/log.h"

namespace ds24xx {

static const char *TAG = "ds24xx";

constexpr uint8_t FAMILY_DS2413 = 0x3A;
constexpr uint8_t FAMILY_DS2408 = 0x29;

constexpr uint8_t DS2413_ACCESS_READ = 0xF5;
constexpr uint8_t DS2413_ACCESS_WRITE = 0x5A;
constexpr uint8_t DS2413_ACK_SUCCESS = 0xAA;

constexpr uint8_t DS2408_CMD_CHANNEL_ACCESS_READ = 0xF5;
constexpr uint8_t DS2408_CMD_CHANNEL_ACCESS_WRITE = 0x5A;
constexpr uint8_t DS2408_CONFIRM_OK = 0xFF;

class DS24xxComponent;

class DS24xxOutput : public ::esphome::output::BinaryOutput, public ::esphome::Component {
 public:
  DS24xxOutput(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0);
  void write_state(bool state) override;

 private:
  DS24xxComponent *parent_;
  uint8_t channel_;
  uint8_t device_index_;
};

class DS24xxBinarySensor : public ::esphome::binary_sensor::BinarySensor, public ::esphome::Component {
 public:
  DS24xxBinarySensor(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0);

 private:
  DS24xxComponent *parent_;
  uint8_t channel_;
  uint8_t device_index_;
};

class DS24xxComponent : public ::esphome::Component {
 public:
  explicit DS24xxComponent(uint8_t one_wire_pin, bool inverted = false);
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
  explicit DS24xxComponent(onewire::OneWireBus *bus, bool inverted = false);
#else
  explicit DS24xxComponent(void *bus, bool inverted = false);
#endif

  void setup() override;
  void loop() override;

  bool write_channel(uint8_t channel, bool value, uint8_t device_index = 0);
  void attach_binary(DS24xxBinarySensor *bs) { binaries_.push_back(bs); }
  void attach_output(DS24xxOutput *out) { outputs_.push_back(out); }

 private:
  uint8_t one_wire_pin_{};
  bool inverted_{false};
  bool use_onewire_bus_{false};
  void *oneWireBus_{nullptr};
  OneWire *oneWire_{nullptr};
  std::vector<std::array<uint8_t,8>> addresses_;
  std::vector<uint8_t> families_;
  std::vector<uint8_t> states_;
  std::vector<DS24xxBinarySensor *> binaries_;
  std::vector<DS24xxOutput *> outputs_;

  bool read_state_from_device(size_t device_index);
  bool write_state_to_device(size_t device_index);
};

// Minimal inline implementations
inline DS24xxComponent::DS24xxComponent(uint8_t one_wire_pin, bool inverted)
  : one_wire_pin_(one_wire_pin), inverted_(inverted) { oneWire_ = new OneWire(one_wire_pin_); }
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
inline DS24xxComponent::DS24xxComponent(onewire::OneWireBus *bus, bool inverted)
  : one_wire_pin_(0), inverted_(inverted), use_onewire_bus_(true), oneWireBus_(bus) {}
#else
inline DS24xxComponent::DS24xxComponent(void *bus, bool inverted)
  : one_wire_pin_(0), inverted_(inverted), use_onewire_bus_(true), oneWireBus_(bus) {}
#endif

inline void DS24xxComponent::setup() {
  // discovery (pin-based OneWire only)
  addresses_.clear();
  families_.clear();
  states_.clear();
  if (use_onewire_bus_) {
    // discovery via shared OneWire bus
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
    auto *bus = static_cast<onewire::OneWireBus *>(oneWireBus_);
    if (!bus) return;
    uint8_t addr[8];
    bus->reset_search();
    while (bus->search(addr)) {
      if (OneWire::crc8(addr, 7) != addr[7])
        continue;
      std::array<uint8_t, 8> a{};
      for (int i = 0; i < 8; ++i) a[i] = addr[i];
            addresses_.push_back(a);
            families_.push_back(addr[0]);
            states_.push_back(0);
            ESP_LOGI(TAG, "Found device %02X%02X%02X%02X%02X%02X%02X%02X family=0x%02X",
              addr[7], addr[6], addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], addr[0]);
    }
#else
    return;
#endif
  } else if (!oneWire_) {
    return;
  }
  uint8_t addr[8];
  oneWire_->reset_search();
  while (oneWire_->search(addr)) {
    if (OneWire::crc8(addr, 7) != addr[7])
      continue;
    std::array<uint8_t,8> a{};
    for (int i = 0; i < 8; ++i) a[i] = addr[i];
    addresses_.push_back(a);
    families_.push_back(addr[0]);
    states_.push_back(0);
    ESP_LOGI(TAG, "Found device %02X%02X%02X%02X%02X%02X%02X%02X family=0x%02X",
            addr[7], addr[6], addr[5], addr[4], addr[3], addr[2], addr[1], addr[0], addr[0]);
  }
  if (addresses_.empty())
    ESP_LOGW(TAG, "No ds24xx devices found on pin %u", (unsigned)one_wire_pin_);
  else
    ESP_LOGI(TAG, "Discovered %u ds24xx device(s)", (unsigned)addresses_.size());
}

inline void DS24xxComponent::loop() {
  static unsigned long last = 0;
  const unsigned long now = millis();
  // run every 5s
  if (now - last < 5000) return;
  last = now;

  if (addresses_.empty()) {
    ESP_LOGI(TAG, "No devices - retrying discovery");
    setup();
    return;
  }

  for (size_t i = 0; i < addresses_.size(); ++i) {
    uint8_t old = states_[i];
    if (!read_state_from_device(i)) {
      ESP_LOGW(TAG, "Failed to read device %u", (unsigned)i);
      continue;
    }
    uint8_t nw = states_[i];
    if (nw != old) {
      ESP_LOGI(TAG, "Device %u state changed: 0x%02X -> 0x%02X", (unsigned)i, old, nw);
    }
  }
}

inline bool DS24xxComponent::read_state_from_device(size_t device_index) {
  if (device_index >= addresses_.size()) return false;
  const auto &addr = addresses_[device_index];
  uint8_t family = families_[device_index];
  if (use_onewire_bus_) {
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
    auto *bus = static_cast<onewire::OneWireBus *>(oneWireBus_);
    if (!bus) return false;
    bus->reset();
    bus->select(addr.data());
    if (family == FAMILY_DS2413) {
      bus->write(DS2413_ACCESS_READ);
      uint8_t val = bus->read();
      states_[device_index] = val;
      ESP_LOGD(TAG, "Read DS2413[%u]=0x%02X", (unsigned)device_index, val);
      return true;
    } else if (family == FAMILY_DS2408) {
      bus->write(DS2408_CMD_CHANNEL_ACCESS_READ);
      uint8_t val = bus->read();
      states_[device_index] = val;
      ESP_LOGD(TAG, "Read DS2408[%u]=0x%02X", (unsigned)device_index, val);
      return true;
    }
#else
    return false;
#endif
  } else {
    oneWire_->reset();
    oneWire_->select(addr.data());
    if (family == FAMILY_DS2413) {
      oneWire_->write(DS2413_ACCESS_READ);
      uint8_t val = oneWire_->read();
      states_[device_index] = val;
      ESP_LOGD(TAG, "Read DS2413[%u]=0x%02X", (unsigned)device_index, val);
      return true;
    } else if (family == FAMILY_DS2408) {
      oneWire_->write(DS2408_CMD_CHANNEL_ACCESS_READ);
      uint8_t val = oneWire_->read();
      states_[device_index] = val;
      ESP_LOGD(TAG, "Read DS2408[%u]=0x%02X", (unsigned)device_index, val);
      return true;
    }
    return false;
  }
}

inline bool DS24xxComponent::write_state_to_device(size_t device_index) {
  if (device_index >= addresses_.size()) return false;
  const auto &addr = addresses_[device_index];
  uint8_t family = families_[device_index];
  if (use_onewire_bus_) {
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
    auto *bus = static_cast<onewire::OneWireBus *>(oneWireBus_);
    if (!bus) return false;
    bus->reset();
    bus->select(addr.data());
    if (family == FAMILY_DS2413) {
      bus->write(DS2413_ACCESS_WRITE);
      bus->write(states_[device_index]);
      uint8_t ack = bus->read();
      (void)ack;
      ESP_LOGD(TAG, "Wrote DS2413[%u]=0x%02X", (unsigned)device_index, states_[device_index]);
      return true;
    } else if (family == FAMILY_DS2408) {
      bus->write(DS2408_CMD_CHANNEL_ACCESS_WRITE);
      bus->write(states_[device_index]);
      uint8_t conf = bus->read();
      (void)conf;
      ESP_LOGD(TAG, "Wrote DS2408[%u]=0x%02X", (unsigned)device_index, states_[device_index]);
      return true;
    }
#else
    return false;
#endif
  } else {
    oneWire_->reset();
    oneWire_->select(addr.data());
    if (family == FAMILY_DS2413) {
      oneWire_->write(DS2413_ACCESS_WRITE);
      oneWire_->write(states_[device_index]);
      uint8_t ack = oneWire_->read();
      (void)ack;
      ESP_LOGD(TAG, "Wrote DS2413[%u]=0x%02X", (unsigned)device_index, states_[device_index]);
      return true;
    } else if (family == FAMILY_DS2408) {
      oneWire_->write(DS2408_CMD_CHANNEL_ACCESS_WRITE);
      oneWire_->write(states_[device_index]);
      uint8_t conf = oneWire_->read();
      (void)conf;
      ESP_LOGD(TAG, "Wrote DS2408[%u]=0x%02X", (unsigned)device_index, states_[device_index]);
      return true;
    }
    return false;
  }
}

inline bool DS24xxComponent::write_channel(uint8_t channel, bool value, uint8_t device_index) {
  if (device_index >= states_.size()) return false;
  uint8_t family = families_[device_index];
  uint8_t &st = states_[device_index];
  ESP_LOGD(TAG, "write_channel dev=%u ch=%u val=%u", (unsigned)device_index, (unsigned)channel, (unsigned)value);
  if (family == FAMILY_DS2413) {
    // DS2413 has two PIO bits in lowest bits
    uint8_t mask = (1 << channel);
    if (value) st |= mask; else st &= ~mask;
    bool ok = write_state_to_device(device_index);
    ESP_LOGI(TAG, "DS2413 write dev=%u -> %s", (unsigned)device_index, ok ? "OK" : "FAIL");
    return ok;
  } else if (family == FAMILY_DS2408) {
    // DS2408 channels map to bits 0-7
    uint8_t mask = (1 << channel);
    if (value) st |= mask; else st &= ~mask;
    bool ok = write_state_to_device(device_index);
    ESP_LOGI(TAG, "DS2408 write dev=%u -> %s", (unsigned)device_index, ok ? "OK" : "FAIL");
    return ok;
  }
  return false;
}

inline DS24xxBinarySensor::DS24xxBinarySensor(DS24xxComponent *parent, uint8_t channel, uint8_t device_index)
    : parent_(parent), channel_(channel), device_index_(device_index) { parent_->attach_binary(this); }

inline DS24xxOutput::DS24xxOutput(DS24xxComponent *parent, uint8_t channel, uint8_t device_index)
    : parent_(parent), channel_(channel), device_index_(device_index) { parent_->attach_output(this); }

inline void DS24xxOutput::write_state(bool state) { if (parent_) parent_->write_channel(channel_, state, device_index_); }

inline DS24xxComponent *ds24xx_register_component(uint8_t one_wire_pin, bool inverted = false) { auto *c = new DS24xxComponent(one_wire_pin, inverted); ::esphome::App.register_component(c); return c; }
inline DS24xxOutput *ds24xx_register_output(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0) { auto *o = new DS24xxOutput(parent, channel, device_index); ::esphome::App.register_component(o); return o; }

}  // namespace ds24xx
