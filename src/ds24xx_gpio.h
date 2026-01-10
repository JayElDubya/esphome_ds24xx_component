// DS2413 / DS2408 custom component for ESPHome
// - Supports DS2413 (2 channels) with attachable Switch and BinarySensor objects
// - OneWire pin is configurable in YAML via lambda (see example in README or below)
// - Per-channel mode: use Switch (output) or BinarySensor (input)
/*
NEXT STEPS / Suggested work to finish integration and upstream PR
- Implement DS2408 read/write command sequences (see DS2408 datasheet):
  - Read PIO latch/status, perform strobe/write latch as required.
  - Handle strong pull-up timing if powering loads during writes.
- do we need to use outputs instead of switches to enable use for things like lights?
- Add robust device discovery and su13 / DS2408pport for multiple devices on bus.
- Expose YAML schema (C++ registration) so users can configure in YAML
  without lambdas (create a proper PlatformIO integration for esphome).
- Add unit tests / integration tests (mock OneWire) and example YAML configs.
- Add documentation/README and a contribution PR checklist for esphome.

Example YAML (using lambdas as interim):
  custom_component:
    - lambda: |-
        ds24xx_global = new DS24xxComponent(4); // oneWire on GPIO4
        App.register_component(ds24xx_global);
        auto *o0 = new DS24xxOutput(ds24xx_global, 0);
        App.register_component(o0);
        App.register_output(o0);
        return {o0};

If you want to upstream this to esphome:
- Implement YAML schema and registration functions in C++ (no-lambda config).
- Follow esphome contribution guidelines: include examples, tests, and docs.
test*/

#pragma once
#include "esphome.h"
#include <OneWire.h>
#include <vector>
#include <array>

// This is an example-named copy of the DS24xx component to make filenames clearer
// (DS24xx 1-wire GPIO expanders). Use this file as the example reference for
// upstreaming to esphome.

#define FAMILY_DS2413       0x3A
#define FAMILY_DS2408       0x29

#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA

// DS2408 command codes (see datasheets/ds2408.pdf)
// - Channel-Access Read  (samples PIO pins):    0xF5
// - Channel-Access Write (write PIO latch byte): 0x5A
// - Read PIO Registers (endless read with CRC16):0xF0
// Confirmation byte for Channel-Access Write success is 0xFF
#define DS2408_CMD_CHANNEL_ACCESS_READ   0xF5
#define DS2408_CMD_CHANNEL_ACCESS_WRITE  0x5A
#define DS2408_CMD_READ_PIO_REGS         0xF0
#define DS2408_CONFIRM_OK                0xFF

// Note: When performing Channel-Access Write the datasheet describes
// that the inverted data byte may require a strong pull-up during the
// final transition. If the OneWire library supports it, use the write
// overload that accepts a power flag (e.g. write(byte, power)) to
// request a strong pull-up during that byte.

// NOTE: DS24xxSwitch was removed. Use DS24xxOutput + ESPHome `output` switch
// platform when a Switch abstraction is desired.
namespace ds24xx {

class DS24xxComponent;

class DS24xxOutput : public ::esphome::output::BinaryOutput, public ::esphome::Component {
 public:
  DS24xxOutput(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0);
  void write_state(bool state) override;
  void setup() override {}
  void loop() override {}
  uint8_t get_channel() const { return channel_; }
  uint8_t get_device_index() const { return device_index_; }

 private:
  DS24xxComponent *parent_;
  uint8_t channel_;
  uint8_t device_index_;
};

class DS24xxBinarySensor : public ::esphome::binary_sensor::BinarySensor, public ::esphome::Component {
 public:
  DS24xxBinarySensor(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0);
  void setup() override {}
  void loop() override {}
  uint8_t get_channel() const { return channel_; }
  uint8_t get_device_index() const { return device_index_; }

 private:
  DS24xxComponent *parent_;
  uint8_t channel_;
  uint8_t device_index_;
};

class DS24xxComponent : public ::esphome::Component {
 public:
  explicit DS24xxComponent(uint8_t one_wire_pin, bool inverted = false)
      : one_wire_pin_(one_wire_pin), inverted_(inverted) {
    oneWire_ = new OneWire(one_wire_pin_);
  }

  void setup() override {
    ESP_LOGD("ds24xx", "Searching for DS24xx devices on pin %u", this->one_wire_pin_);
    oneWire_->reset_search();
    delay(50);
    uint8_t buf[8];
    while (oneWire_->search(buf)) {
      if (OneWire::crc8(buf, 7) != buf[7]) {
        ESP_LOGW("ds24xx", "Found device with invalid CRC; skipping");
        continue;
      }
      std::array<uint8_t, 8> a{};
      for (int i = 0; i < 8; ++i) a[i] = buf[i];
      uint8_t fam = a[0];
      if (fam != FAMILY_DS2413 && fam != FAMILY_DS2408) {
        ESP_LOGW("ds24xx", "Found unsupported device family 0x%02X; skipping", fam);
        continue;
      }
      addresses_.push_back(a);
      families_.push_back(fam);
      states_.push_back(0);
      ESP_LOGD("ds24xx", "Discovered device index %u family 0x%02X", (uint32_t)addresses_.size()-1, fam);
    }
    if (addresses_.empty()) {
      ESP_LOGW("ds24xx", "No supported devices found on the bus");
      return;
    }
    // initialize state by reading each device
    for (size_t i = 0; i < addresses_.size(); ++i) this->read_state_from_device(i);
  }

  void loop() override {
    // poll each device and update attached sensors for that device
    for (size_t dev = 0; dev < addresses_.size(); ++dev) {
      if (!this->read_state_from_device(dev)) continue;
        // switches are removed; use Outputs and ESPHome `output` switch platform
      for (auto *b : binaries_) {
        if (!b) continue;
        if (b->get_device_index() != dev) continue;
        bool v = (states_[dev] >> b->get_channel()) & 0x01;
        b->publish_state(v);
      }
      for (auto *o : outputs_) {
        if (!o) continue;
        if (o->get_device_index() != dev) continue;
        bool v = ((states_[dev] >> o->get_channel()) & 0x01);
        o->set_state(v);
      }
    }
  }

  // write a single channel and push to device
  bool write_channel(uint8_t channel, bool value, uint8_t device_index = 0) {
    if (device_index >= states_.size()) return false;
    uint8_t fam = families_[device_index];
    if (fam == FAMILY_DS2413) {
      if (channel > 1) return false; // DS2413 only has 2 channels
    } else if (fam == FAMILY_DS2408) {
      if (channel > 7) return false; // DS2408 has 8 channels
    }
    if (value) states_[device_index] |= (1 << channel);
    else states_[device_index] &= ~(1 << channel);
    return write_state_to_device(device_index);
  }

  // attach helpers
  void attach_binary(DS24xxBinarySensor *bs) { binaries_.push_back(bs); }
  void attach_output(DS24xxOutput *out) { outputs_.push_back(out); }

 private:
  OneWire *oneWire_{nullptr};
  uint8_t one_wire_pin_;
  bool inverted_{false};
  std::vector<std::array<uint8_t,8>> addresses_;
  std::vector<uint8_t> families_;
  std::vector<uint8_t> states_;
  // switches removed; use Outputs instead
  std::vector<DS24xxBinarySensor *> binaries_;
  std::vector<DS24xxOutput *> outputs_;

  bool read_state_from_device(size_t device_index) {
    if (device_index >= addresses_.size()) return false;
    uint8_t fam = families_[device_index];
    auto &addr = addresses_[device_index];
    if (fam == FAMILY_DS2413) {
      oneWire_->reset();
      oneWire_->select(addr.data());
      oneWire_->write(DS2413_ACCESS_READ);
      uint8_t results = oneWire_->read();
      bool ok = (((~results) & 0x0F) == (results >> 4));
      uint8_t nibble = results & 0x0F;
      if (!ok) {
        ESP_LOGW("ds24xx", "Read CRC/validation failed for device %u", (uint32_t)device_index);
        return false;
      }
      states_[device_index] = nibble & 0x03; // only two LSBs valid
      if (inverted_) states_[device_index] ^= 0x03;
      return true;
    } else if (fam == FAMILY_DS2408) {
      oneWire_->reset();
      oneWire_->select(addr.data());
      oneWire_->write(DS2408_CMD_CHANNEL_ACCESS_READ); // 0xF5
      uint8_t pio_status = oneWire_->read();
      states_[device_index] = pio_status;
      if (inverted_) states_[device_index] = ~states_[device_index];
      return true;
    }
    return false;
  }

  bool write_state_to_device(size_t device_index) {
    if (device_index >= addresses_.size()) return false;
    uint8_t fam = families_[device_index];
    auto &addr = addresses_[device_index];
    if (fam == FAMILY_DS2413) {
      uint8_t to_write = states_[device_index] & 0x03;
      if (inverted_) to_write ^= 0x03;
      uint8_t payload = to_write | 0xFC;
      oneWire_->reset();
      oneWire_->select(addr.data());
      oneWire_->write(DS2413_ACCESS_WRITE);
      oneWire_->write(payload);
      oneWire_->write(~payload);
      uint8_t ack = oneWire_->read();
      if (ack != DS2413_ACK_SUCCESS) {
        ESP_LOGW("ds24xx", "Write failed for device %u (ack: 0x%02X)", (uint32_t)device_index, ack);
        oneWire_->reset();
        return false;
      }
      oneWire_->read();
      oneWire_->reset();
      return true;
    } else if (fam == FAMILY_DS2408) {
      uint8_t to_write = states_[device_index] & 0xFF;
      if (inverted_) to_write = ~to_write;
      oneWire_->reset();
      oneWire_->select(addr.data());
      oneWire_->write(DS2408_CMD_CHANNEL_ACCESS_WRITE);
      oneWire_->write(to_write);
      oneWire_->write((uint8_t)(~to_write), 1);
      uint8_t confirm = oneWire_->read();
      if (confirm != DS2408_CONFIRM_OK) {
        ESP_LOGW("ds24xx", "DS2408 write failed for device %u (confirm: 0x%02X)", (uint32_t)device_index, confirm);
        oneWire_->reset();
        return false;
      }
      uint8_t status = oneWire_->read();
      states_[device_index] = status;
      if (inverted_) states_[device_index] = ~states_[device_index];
      oneWire_->reset();
      return true;
    }
    return false;
  }
};

// Implementation of helper classes
// DS2413Switch removed; no inline implementations remain.

inline DS24xxBinarySensor::DS24xxBinarySensor(DS24xxComponent *parent, uint8_t channel, uint8_t device_index)
    : parent_(parent), channel_(channel), device_index_(device_index) {
  parent_->attach_binary(this);
}

inline DS24xxOutput::DS24xxOutput(DS24xxComponent *parent, uint8_t channel, uint8_t device_index)
    : parent_(parent), channel_(channel), device_index_(device_index) {
  parent_->attach_output(this);
}

inline void DS24xxOutput::write_state(bool state) {
  if (!parent_) return;
  parent_->write_channel(channel_, state, device_index_);
}

// ------------------------------------------------------------------
// C++ registration helpers for code-generated YAML integration
// These helpers make it easy for a platform-specific codegen (or a
// PlatformIO/ESPhome integration) to instantiate and register the
// component and outputs without requiring lambdas in YAML.
// Example generated usage (C++):
//   auto *ds = ds24xx_register_component(4, false);
//   auto *o0 = ds24xx_register_output(ds, 0, 0, "o0");
//   // generated code may keep pointers to `o0` for later reference

inline DS24xxComponent *ds24xx_register_component(uint8_t one_wire_pin, bool inverted = false) {
  auto *c = new DS24xxComponent(one_wire_pin, inverted);
  ::esphome::App.register_component(c);
  return c;
}

inline DS24xxOutput *ds24xx_register_output(DS24xxComponent *parent, uint8_t channel,
                                            uint8_t device_index = 0) {
  auto *o = new DS24xxOutput(parent, channel, device_index);
  ::esphome::App.register_component(o);
  return o;
}

}  // namespace ds24xx

