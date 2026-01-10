# DS24xx 1-Wire GPIO Component for ESPHome

Lightweight custom component providing 1-Wire GPIO expanders (DS2413 and DS2408)
for ESPHome. This repository contains a minimal implementation that exposes device
outputs as generic ESPHome `Output` objects so they can be used as switches,
lights, or other output types.

**Files of interest**
- `src/ds24xx_gpio.h` — main component, outputs, and read/write sequences.
 - `examples/fireplace_under.yaml` — ESPhome example showing usage with the
   new external component (no-lambda config).
- `datasheets/ds2408.pdf` — DS2408 datasheet (kept here for reference).

- Quick notes
- Supported devices: DS2413 (2-channel) and basic DS2408 support implemented
  for Channel-Access Read/Write sequences. See `src/ds24xx_gpio.h` for details.
- Outputs are exposed as generic `Output` objects (class `DS24xxOutput`) so
  they can be used as switches or lights within ESPHome.
- DS2408 Channel-Access Write may require a strong pull-up during the inverted
  byte transmission. The code uses the OneWire library's `write(byte, power)`
  overload when available to request a strong pull-up — verify your
  OneWire library supports this feature on your platform.

Usage (develop/test)
1. Copy this folder into your ESPHome configuration directory as
  `components/ds24xx` or include the sources in your build tree.
2. Use the example YAML in `examples/fireplace_under.yaml` as a starting point or
   adapt it to your project. The lambda registers the component and a single
   output on channel 0.

Example YAML snippet (see `examples/fireplace_under.yaml`):

```yaml
custom_component:
  - lambda: |-
      auto *ds = new ds24xx::DS24xxComponent(4);
      App.register_component(ds);
      auto *o0 = new ds24xx::DS24xxOutput(ds, 0);
      App.register_component(o0);
      return {o0};
```

To expose the output as a switch in ESPHome YAML, use the `output` platform:

```yaml
switch:
  - platform: output
    name: "DS24xx Channel 0"
    output: o0

If integrating this into ESPhome code generation (so users can configure
`ds24xx` in YAML without lambdas), the code generator can call the
inline C++ registration helpers included in `src/ds24xx_gpio.h`:

```cpp
// generated C++ from YAML
auto *ds = ds24xx_register_component(4, false);
auto *o0 = ds24xx_register_output(ds, 0, 0);
// register output under generated identifier so YAML can reference it
```

Creating a proper YAML schema and codegen plugin (Python) for ESPHome is
the recommended next step to fully remove lambdas from user-facing configs.
```

Build
- With ESPHome installed, run:

```bash
esphome compile examples/fireplace_under.yaml
```

or place the component under your ESPHome `components/` directory and
reference it from your normal ESPhome YAML.

Contributing
- If you want to upstream this to ESPHome, implement a proper YAML schema
  (no-lambda config) and add unit/integration tests. See `src/ds24xx_gpio.h`
  for the TODO comment block describing next steps.

License
- This repository is provided as-is for experimentation. No license header is
  included — add one if you plan to reuse or distribute this code.
