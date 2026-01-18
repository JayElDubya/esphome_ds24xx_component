#pragma once
/**
 * @file ds24xx.h
 * @brief ESPHome component for Dallas DS2413 (2-channel) and DS2408 (8-channel) GPIO expanders.
 *
 * This component controls open-drain outputs via ESPHome's shared 1-Wire bus.
 *
 * @see https://www.analog.com/en/products/ds2413.html (DS2413 datasheet)
 * @see https://www.analog.com/en/products/ds2408.html (DS2408 datasheet)
 */

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/output/binary_output.h"

// Binary sensor is optional - only included if available
#if __has_include("esphome/components/binary_sensor/binary_sensor.h")
#include "esphome/components/binary_sensor/binary_sensor.h"
#define DS24XX_HAVE_BINARY_SENSOR 1
#endif

// Detect ESPHome's one_wire component
#if defined(__has_include)
#  if __has_include("esphome/components/one_wire/one_wire_bus.h")
#    include "esphome/components/one_wire/one_wire_bus.h"
#    define DS24XX_HAVE_ESPHOME_ONEWIRE 1
     namespace ds24xx { namespace onewire = ::esphome::one_wire; }
#  elif __has_include("esphome/components/one_wire/one_wire.h")
#    include "esphome/components/one_wire/one_wire.h"
#    define DS24XX_HAVE_ESPHOME_ONEWIRE 1
     namespace ds24xx { namespace onewire = ::esphome::one_wire; }
#  endif
#endif

// Also check for GPIO-backed OneWire implementation
#if defined(__has_include) && !defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
#  if __has_include("esphome/components/gpio/one_wire/gpio_one_wire.h")
#    include "esphome/components/gpio/one_wire/gpio_one_wire.h"
#    define DS24XX_HAVE_ESPHOME_ONEWIRE 1
     namespace ds24xx { namespace onewire = ::esphome::one_wire; }
#  endif
#endif

#if !defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
#error "DS24xx requires ESPHome's one_wire component. Please configure a one_wire bus."
#endif

#include <vector>
#include <array>

namespace ds24xx {

/// DS2413/DS2408 Protocol Constants
static constexpr uint8_t FAMILY_DS2413 = 0x3A;        ///< DS2413 family code (2-channel)
static constexpr uint8_t FAMILY_DS2408 = 0x29;        ///< DS2408 family code (8-channel)
static constexpr uint8_t DS2413_ACCESS_READ = 0xF5;   ///< PIO Access Read command
static constexpr uint8_t DS2413_ACCESS_WRITE = 0x5A;  ///< PIO Access Write command
static constexpr uint8_t DS2413_ACK_SUCCESS = 0xAA;   ///< Write acknowledgment byte
static constexpr uint8_t DS2408_CMD_READ = 0xF5;      ///< Channel Access Read command
static constexpr uint8_t DS2408_CMD_WRITE = 0x5A;     ///< Channel Access Write command

/// Convert 8-byte 1-Wire address to uint64_t (little-endian format)
inline uint64_t addr_to_uint64(const std::array<uint8_t, 8> &addr) {
  uint64_t result = 0;
  for (int i = 7; i >= 0; --i) {
    result = (result << 8) | addr[i];
  }
  return result;
}

/// Format address as hex string for logging (MSB first)
inline void format_address(const std::array<uint8_t, 8> &addr, char *buf, size_t len) {
  int off = 0;
  for (int i = 7; i >= 0 && off < static_cast<int>(len) - 2; --i) {
    off += snprintf(buf + off, len - off, "%02X", addr[i]);
  }
}

// Forward declarations
class DS24xxComponent;

/**
 * @brief Output class for DS2413/DS2408 channels.
 *
 * Each instance controls a single output channel on a DS24xx device.
 * Supports per-output inversion independent of component-level inversion.
 */
class DS24xxOutput : public esphome::output::BinaryOutput, public esphome::Component {
 public:
  DS24xxOutput(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0);

  void write_state(bool state) override;

  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  bool get_inverted() const { return this->inverted_; }

 protected:
  DS24xxComponent *parent_;
  uint8_t channel_;
  uint8_t device_index_;
  bool inverted_{false};
};

#if defined(DS24XX_HAVE_BINARY_SENSOR)
/**
 * @brief Binary sensor class for DS2413/DS2408 input channels.
 *
 * Reads the PIO latch state for monitoring input pins.
 */
class DS24xxBinarySensor : public esphome::binary_sensor::BinarySensor, public esphome::Component {
 public:
  DS24xxBinarySensor(DS24xxComponent *parent, uint8_t channel, uint8_t device_index = 0);

  void set_inverted(bool inverted) { this->inverted_ = inverted; }
  bool get_inverted() const { return this->inverted_; }

 protected:
  DS24xxComponent *parent_;
  uint8_t channel_;
  uint8_t device_index_;
  bool inverted_{false};
};
#endif  // DS24XX_HAVE_BINARY_SENSOR

/**
 * @brief Main component managing DS2413/DS2408 devices on a 1-Wire bus.
 *
 * Features:
 * - Auto-discovery of DS2413 and DS2408 devices
 * - Uses ESPHome's shared one_wire bus
 * - Multiple device support via device_index
 * - Component-level and per-output inversion
 * - Exponential backoff retry on discovery failures
 */
class DS24xxComponent : public esphome::Component {
 public:
  /// Constructor for shared ESPHome 1-Wire bus
  explicit DS24xxComponent(onewire::OneWireBus *bus, bool inverted = false);

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return esphome::setup_priority::DATA; }

  /**
   * @brief Write a value to a specific channel on a device.
   * @param channel Channel number (0-1 for DS2413, 0-7 for DS2408)
   * @param value Desired output state (true=ON, false=OFF)
   * @param device_index Index of device when multiple are present
   * @param output_inverted Per-output inversion flag
   * @return true if write succeeded
   */
  bool write_channel(uint8_t channel, bool value, uint8_t device_index = 0, bool output_inverted = false);

#if defined(DS24XX_HAVE_BINARY_SENSOR)
  void attach_binary(DS24xxBinarySensor *sensor) { this->binaries_.push_back(sensor); }
#endif
  void attach_output(DS24xxOutput *output) { this->outputs_.push_back(output); }

  size_t get_device_count() const { return this->addresses_.size(); }

 protected:
  static const char *const TAG;

  bool inverted_{false};
  onewire::OneWireBus *bus_{nullptr};

  std::vector<std::array<uint8_t, 8>> addresses_;
  std::vector<uint8_t> families_;
  std::vector<uint8_t> states_;
#if defined(DS24XX_HAVE_BINARY_SENSOR)
  std::vector<DS24xxBinarySensor *> binaries_;
#endif
  std::vector<DS24xxOutput *> outputs_;

  uint32_t last_update_ms_{0};
  uint32_t update_interval_ms_{5000};
  static constexpr uint32_t UPDATE_INTERVAL_MAX_MS = 60000;
  int consecutive_failures_{0};
  std::vector<int> device_error_counters_;

  bool read_state_from_device_(size_t device_index);
  bool write_state_to_device_(size_t device_index);
  void discover_devices_();
};

// ============================================================================
// Inline Implementations
// ============================================================================

inline const char *const DS24xxComponent::TAG = "ds24xx";

inline DS24xxComponent::DS24xxComponent(onewire::OneWireBus *bus, bool inverted)
    : inverted_(inverted), bus_(bus) {}

inline void DS24xxComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DS24xx...");
  this->discover_devices_();
}

inline void DS24xxComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DS24xx GPIO Expander:");
  ESP_LOGCONFIG(TAG, "  Global Inversion: %s", YESNO(this->inverted_));
  ESP_LOGCONFIG(TAG, "  Devices Found: %u", this->addresses_.size());

  for (size_t i = 0; i < this->addresses_.size(); i++) {
    char addr_str[20];
    format_address(this->addresses_[i], addr_str, sizeof(addr_str));
    const char *type = (this->families_[i] == FAMILY_DS2413) ? "DS2413" : "DS2408";
    uint8_t channels = (this->families_[i] == FAMILY_DS2413) ? 2 : 8;
    ESP_LOGCONFIG(TAG, "    [%u] %s %s (%u channels)", i, type, addr_str, channels);
  }
}

inline void DS24xxComponent::discover_devices_() {
  this->addresses_.clear();
  this->families_.clear();
  this->states_.clear();

  if (this->bus_ == nullptr) {
    ESP_LOGW(TAG, "1-Wire bus pointer is null");
    return;
  }

  // Trigger search if no devices cached
  const auto &devs = this->bus_->get_devices();
  if (devs.empty()) {
    ESP_LOGD(TAG, "No devices cached, triggering search...");
    this->bus_->search();
  }

  // Filter for DS2413/DS2408 devices
  for (uint64_t dev : this->bus_->get_devices()) {
    uint8_t family = dev & 0xFF;
    if (family == FAMILY_DS2413 || family == FAMILY_DS2408) {
      std::array<uint8_t, 8> addr{};
      uint64_t v = dev;
      for (int i = 0; i < 8; ++i) {
        addr[i] = static_cast<uint8_t>(v & 0xFF);
        v >>= 8;
      }
      this->addresses_.push_back(addr);
      this->families_.push_back(family);
      this->states_.push_back(0xFF);  // All outputs OFF (active-low)

      char addr_str[20];
      format_address(addr, addr_str, sizeof(addr_str));
      ESP_LOGI(TAG, "Discovered %s: %s", (family == FAMILY_DS2413) ? "DS2413" : "DS2408", addr_str);
    }
  }

  if (this->addresses_.empty()) {
    ESP_LOGW(TAG, "No DS2413/DS2408 devices found");
  }

  this->device_error_counters_.assign(this->addresses_.size(), 0);
}

inline void DS24xxComponent::loop() {
  const uint32_t now = millis();

  // Retry discovery with exponential backoff if no devices found
  if (this->addresses_.empty()) {
    if (now - this->last_update_ms_ < this->update_interval_ms_) {
      return;
    }
    this->last_update_ms_ = now;
    this->consecutive_failures_++;

    ESP_LOGD(TAG, "Retrying discovery (interval %ums)", this->update_interval_ms_);
    this->update_interval_ms_ = std::min(this->update_interval_ms_ * 2, UPDATE_INTERVAL_MAX_MS);
    this->discover_devices_();
    return;
  }

  // Periodic state polling for binary sensors
  if (now - this->last_update_ms_ < this->update_interval_ms_) {
    return;
  }
  this->last_update_ms_ = now;

  for (size_t i = 0; i < this->addresses_.size(); i++) {
    uint8_t old_state = this->states_[i];

    if (!this->read_state_from_device_(i)) {
      this->device_error_counters_[i]++;
      // Only log first few errors to avoid spam
      if (this->device_error_counters_[i] <= 3) {
        ESP_LOGW(TAG, "Failed to read device %u (errors: %d)", i, this->device_error_counters_[i]);
      }
      continue;
    }

    this->device_error_counters_[i] = 0;
    this->consecutive_failures_ = 0;
    this->update_interval_ms_ = 5000;

    if (this->states_[i] != old_state) {
      ESP_LOGD(TAG, "Device %u state: 0x%02X -> 0x%02X", i, old_state, this->states_[i]);
    }
  }
}

inline bool DS24xxComponent::read_state_from_device_(size_t device_index) {
  if (device_index >= this->addresses_.size()) {
    return false;
  }

  const auto &addr = this->addresses_[device_index];
  uint8_t family = this->families_[device_index];

  if (this->bus_ == nullptr) {
    return false;
  }

  uint64_t addr64 = addr_to_uint64(addr);
  if (!this->bus_->select(addr64)) {
    return false;
  }

  uint8_t cmd = (family == FAMILY_DS2413) ? DS2413_ACCESS_READ : DS2408_CMD_READ;
  this->bus_->write8(cmd);
  this->states_[device_index] = this->bus_->read8();
  return true;
}

inline bool DS24xxComponent::write_state_to_device_(size_t device_index) {
  if (device_index >= this->addresses_.size()) {
    return false;
  }

  const auto &addr = this->addresses_[device_index];
  uint8_t family = this->families_[device_index];

  if (this->bus_ == nullptr) {
    return false;
  }

  uint64_t addr64 = addr_to_uint64(addr);
  if (!this->bus_->select(addr64)) {
    ESP_LOGW(TAG, "Select failed for device %u (write)", device_index);
    return false;
  }

  if (family == FAMILY_DS2413) {
    // DS2413 write protocol: command + state + ~state + read ack
    uint8_t state_byte = this->states_[device_index] & 0x03;
    uint8_t state_inv = ~state_byte;
    this->bus_->write8(DS2413_ACCESS_WRITE);
    this->bus_->write8(state_byte);
    this->bus_->write8(state_inv);
    uint8_t ack = this->bus_->read8();
    return (ack == DS2413_ACK_SUCCESS);
  } else {
    // DS2408 protocol
    this->bus_->write8(DS2408_CMD_WRITE);
    this->bus_->write8(this->states_[device_index]);
    this->bus_->read8();  // Confirmation byte
    return true;
  }
}

inline bool DS24xxComponent::write_channel(uint8_t channel, bool value, uint8_t device_index, bool output_inverted) {
  if (device_index >= this->states_.size()) {
    ESP_LOGW(TAG, "Invalid device index %u", device_index);
    return false;
  }

  uint8_t family = this->families_[device_index];
  uint8_t max_channel = (family == FAMILY_DS2413) ? 1 : 7;
  if (channel > max_channel) {
    ESP_LOGW(TAG, "Invalid channel %u for device %u (max: %u)", channel, device_index, max_channel);
    return false;
  }

  uint8_t &state = this->states_[device_index];

  // Apply inversions: per-output first, then component-level
  bool actual = output_inverted ? !value : value;
  if (this->inverted_) {
    actual = !actual;
  }

  ESP_LOGD(TAG, "Channel %u.%u: %s -> %s", device_index, channel, value ? "ON" : "OFF", actual ? "ON" : "OFF");

  // DS2413/DS2408 are active-low: 0=conduct(ON), 1=non-conduct(OFF)
  uint8_t mask = (1 << channel);
  if (actual) {
    state &= ~mask;  // Clear bit = ON
  } else {
    state |= mask;   // Set bit = OFF
  }

  return this->write_state_to_device_(device_index);
}

inline DS24xxOutput::DS24xxOutput(DS24xxComponent *parent, uint8_t channel, uint8_t device_index)
    : parent_(parent), channel_(channel), device_index_(device_index) {
  this->parent_->attach_output(this);
}

inline void DS24xxOutput::write_state(bool state) {
  if (this->parent_ != nullptr) {
    this->parent_->write_channel(this->channel_, state, this->device_index_, this->inverted_);
  }
}

#if defined(DS24XX_HAVE_BINARY_SENSOR)
inline DS24xxBinarySensor::DS24xxBinarySensor(DS24xxComponent *parent, uint8_t channel, uint8_t device_index)
    : parent_(parent), channel_(channel), device_index_(device_index) {
  this->parent_->attach_binary(this);
}
#endif  // DS24XX_HAVE_BINARY_SENSOR

}  // namespace ds24xx
