````markdown
# ds24xx

ESPHome external component for 1-Wire GPIO expanders (DS2413 and DS2408).

This external component provides a generic `ds24xx` component which discovers DS2413 and DS2408 devices on a 1-Wire bus and exposes per-channel `output` and `binary_sensor` objects.

Quick example (no lambdas):

```yaml
components:
  - lambda: |-
      // Not needed when using external component in components/

# When using this external component (placed in `components/ds24xx`):

ds24xx:
  id: ds1
  one_wire_pin: 13
  inverted: false
  outputs:
    - id: o0
      channel: 0
    - id: o1
      channel: 1

switch:
  - platform: output
    name: "Generic Output 0"
    output: o0

binary_sensor:
  - platform: template
    name: "DS24xx input"
    lambda: 'return id(ds1).states[0] & 0x01;'
```

Checklist for contribution

- [x] Provide `manifest.json` with metadata and `codeowners`.
- [x] Provide `__init__.py` codegen with `CONFIG_SCHEMA` and `to_code()`.
- [x] Provide C++ header implementing `DS24xxComponent`, `DS24xxOutput`, and `DS24xxBinarySensor`.
- [x] Include example YAML in `examples/` showing common usage.
- [ ] Add unit tests or integration tests (recommended: mock `OneWire` behavior).
- [ ] Replace inlined or stubbed OneWire code with the real OneWire dependency in the component (or document the PlatformIO `lib_deps`).

Notes

- The component supports multiple devices on the bus; discovered devices are indexed by `device_index` (default 0).
- DS2413 provides two channels (0/1); DS2408 provides eight channels (0..7).
- DS2408 write operations may require a strong pull-up during the final byte; ensure your hardware supports strong pull-up when switching loads.

Contribution tips

- Run `esphome compile examples/fireplace_under.yaml` to verify the build.
- Add `codeowners` in `manifest.json` with the GitHub handle of the maintainer.
- Add unit tests in `test/` to validate read/write sequences using a mock `OneWire` implementation.

````
