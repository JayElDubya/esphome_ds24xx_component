// Inlined DS24xx/DS2408 component header for external_component build
#pragma once
#include "esphome.h"
#include "esphome/components/output/binary_output.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/component.h"
// Prefer the real OneWire library when available; otherwise provide a
// minimal stub so the component can still be parsed at build-time.
#if defined(__has_include)
#  if __has_include(<OneWire.h>)
#    include <OneWire.h>
#  else
#    define DS24XX_NO_ONEWIRE
#  endif
#else
// conservative fallback: try to include and if not available define stub
#  include <OneWire.h>
#endif

#if defined(DS24XX_NO_ONEWIRE)
class OneWire {
 public:
	explicit OneWire(uint8_t pin) {}
	void reset_search() {}
	bool search(uint8_t *addr) { return false; }
	void reset() {}
	void select(const uint8_t *addr) {}
	void write(uint8_t v) {}
	void write(uint8_t v, uint8_t power) { (void)power; }
	uint8_t read() { return 0; }
	static uint8_t crc8(const uint8_t *addr, uint8_t len) { (void)addr; return 0; }
};
#endif
#include <vector>
#include <array>
// Include esphome one-wire bus when available so users can supply shared bus
#if defined(__has_include)
#  if __has_include("esphome/components/onewire/onewire.h")
#    include "esphome/components/onewire/onewire.h"
#    define DS24XX_HAVE_ESPHOME_ONEWIRE 1
#  endif
#endif

// This is an example-named copy of the DS24xx component to make filenames clearer
// (DS24xx 1-wire GPIO expanders). Use this file as the example reference for
// upstreaming to esphome.

#define FAMILY_DS2413       0x3A
#define FAMILY_DS2408       0x29

#define DS2413_ACCESS_READ  0xF5
#define DS2413_ACCESS_WRITE 0x5A
#define DS2413_ACK_SUCCESS  0xAA

// DS2408 command codes (see datasheets/ds2408.pdf)
// - Channel-Access Read  (samples PIO pins):     0xF5
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

// Ensure the esphome Application type is available so we can call
// ::esphome::App.register_component() from the registration helpers.
#include "esphome/core/application.h"
// Forward-declare App as a fallback (the header defines it when available).
namespace esphome { class Application; extern Application App; }

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
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
		bus_ = nullptr;
#endif
	}

#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
	// Construct from a shared esphome one_wire bus
	explicit DS24xxComponent(::esphome::onewire::OneWireBus *bus, bool inverted = false)
			: one_wire_pin_(0xFF), inverted_(inverted), bus_(bus) {
		if (bus_)
			oneWire_ = bus_->get_onewire();
		else
			oneWire_ = nullptr;
	}
#endif

 	void setup() override {
		ESP_LOGD("ds24xx", "Searching for DS24xx devices on pin %u", this->one_wire_pin_);
		uint8_t buf[8];
		bool any_found = false;
		// Try a few searches with increasing delays — sometimes the bus
		// isn't immediately stable during early setup compared to the
		// gpio.one_wire component. Retry to improve discovery reliability.
		for (int attempt = 0; attempt < 3 && !any_found; ++attempt) {
			ESP_LOGD("ds24xx", "Attempt %d to search 1-wire (pin %u)", attempt+1, this->one_wire_pin_);
			oneWire_->reset_search();
			delay(50 * (attempt + 1));
			while (oneWire_->search(buf)) {
				any_found = true;
			// Debug: print raw address bytes and quick CRC checks
			char raw[3*8 + 1]; raw[0] = '\0';
			for (int i = 0; i < 8; ++i) {
				char t[4]; snprintf(t, sizeof(t), "%02X", buf[i]);
				strcat(raw, t);
				if (i != 7) strcat(raw, " ");
			}
			uint8_t crc_orig = OneWire::crc8(buf, 7);
			uint8_t crc_buf = buf[7];
			uint8_t rev[8]; for (int i = 0; i < 8; ++i) rev[i] = buf[7 - i];
			uint8_t crc_rev = OneWire::crc8(rev, 7);
			ESP_LOGD("ds24xx", "Found raw 1-wire addr: %s crc_orig=0x%02X buf[7]=0x%02X crc_rev=0x%02X (rev order)", raw, crc_orig, crc_buf, crc_rev);
			// Try to locate a family byte and normalize the 8-byte address
			// by rotating (forward or reverse) so the family is at index 0
			// and the CRC check passes for the resulting order.
			std::array<uint8_t, 8> a{};
			bool normalized = false;
			for (int dir = 0; dir < 2 && !normalized; ++dir) {
				// dir==0 -> forward rotation, dir==1 -> reverse rotation
				for (int shift = 0; shift < 8 && !normalized; ++shift) {
					uint8_t candidate = buf[shift];
					if (candidate != FAMILY_DS2413 && candidate != FAMILY_DS2408) continue;
					// build rotated address
					for (int j = 0; j < 8; ++j) {
						int idx = (dir == 0) ? (shift + j) % 8 : (shift - j + 8) % 8;
						a[j] = buf[idx];
					}
					// validate CRC for the normalized order
					if (OneWire::crc8(a.data(), 7) == a[7]) {
						normalized = true;
						ESP_LOGD("ds24xx", "Normalized 1-wire address (dir=%d shift=%d)", dir, shift);
						break;
					}
				}
			}
			if (!normalized) {
				char hexbuf[3*8 + 1];
				hexbuf[0] = '\0';
				for (int i = 0; i < 8; ++i) {
					char tmp[4];
					snprintf(tmp, sizeof(tmp), "%02X", buf[i]);
					strcat(hexbuf, tmp);
					if (i != 7) strcat(hexbuf, " ");
				}
				ESP_LOGW("ds24xx", "Found device with invalid CRC; skipping raw=%s", hexbuf);
				continue;
			}
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
		ESP_LOGD("ds24xx", "Found %u supported device(s)", (uint32_t)addresses_.size());
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
		if (addresses_.empty()) {
			ESP_LOGW("ds24xx", "Write ignored: no devices discovered on the bus");
			return false;
		}
		if (device_index >= states_.size()) {
			ESP_LOGW("ds24xx", "Write ignored: invalid device_index %u (have %u)", (uint32_t)device_index, (uint32_t)states_.size());
			return false;
		}
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
#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
 	::esphome::onewire::OneWireBus *bus_{nullptr};
#endif
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
			ESP_LOGD("ds24xx", "DS2413 write dev=%u to_write=0x%02X payload=0x%02X", (uint32_t)device_index, to_write, payload);
			oneWire_->reset();
			oneWire_->select(addr.data());
			oneWire_->write(DS2413_ACCESS_WRITE);
			oneWire_->write(payload);
			// write inverted payload with strong pull-up (some devices require strong pull-up)
			oneWire_->write((uint8_t)(~payload), 1);
			uint8_t ack = oneWire_->read();
			ESP_LOGD("ds24xx", "DS2413 ack=0x%02X", ack);
			if (ack != DS2413_ACK_SUCCESS) {
				ESP_LOGW("ds24xx", "Write failed for device %u (ack: 0x%02X)", (uint32_t)device_index, ack);
				oneWire_->reset();
				return false;
			}
			// read trailing byte (if any) and reset bus
			oneWire_->read();
			oneWire_->reset();
			ESP_LOGD("ds24xx", "DS2413 write successful dev=%u", (uint32_t)device_index);
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
	ESP_LOGD("ds24xx", "Output write request dev=%u ch=%u state=%u", (uint32_t)device_index_, (uint32_t)channel_, (uint32_t)state);
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

#if defined(DS24XX_HAVE_ESPHOME_ONEWIRE)
inline DS24xxComponent *ds24xx_register_component(::esphome::onewire::OneWireBus *bus, bool inverted = false) {
	auto *c = new DS24xxComponent(bus, inverted);
	::esphome::App.register_component(c);
	return c;
}
#endif

inline DS24xxOutput *ds24xx_register_output(DS24xxComponent *parent, uint8_t channel,
													uint8_t device_index = 0) {
	auto *o = new DS24xxOutput(parent, channel, device_index);
	::esphome::App.register_component(o);
	return o;
}

}  // namespace ds24xx

