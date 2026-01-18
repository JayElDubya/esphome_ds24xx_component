# DS24xx ESPHome Component

ESPHome component for Dallas/Maxim DS2413 (2-channel) and DS2408 (8-channel) 1-Wire GPIO expanders.

## Features

- **DS2413 Support**: 2 open-drain output channels
- **DS2408 Support**: 8 open-drain output channels  
- **Shared 1-Wire Bus**: Uses ESPHome's `one_wire` component
- **Per-Output Inversion**: Control inversion at output level
- **Binary Sensors**: Read input states from PIO pins
- **Auto-Discovery**: Automatically finds DS2413/DS2408 devices on the bus

## Installation

Add as an external component in your ESPHome configuration:

```yaml
external_components:
  - source: github://JayElDubya/esphome_ds24xx_component@master
    components: [ds24xx]
```

## Configuration

### Basic Example

```yaml
one_wire:
  - platform: gpio
    id: ow_bus
    pin: GPIO13

ds24xx:
  id: ds_expander
  one_wire: ow_bus
  outputs:
    - id: output_0
      channel: 0
    - id: output_1
      channel: 1
      inverted: true  # Per-output inversion
```

### Multiple Devices

```yaml
ds24xx:
  id: ds_expander
  one_wire: ow_bus
  outputs:
    - id: dev0_ch0
      channel: 0
      device_index: 0  # First DS2413/DS2408 on bus
    - id: dev1_ch0
      channel: 0
      device_index: 1  # Second device on bus
```

### Binary Sensors (Input Mode)

```yaml
ds24xx:
  id: ds_expander
  one_wire: ow_bus
  binary_sensors:
    - id: input_0
      channel: 0
      inverted: true
```

## Configuration Options

### Main Component

| Option | Type | Required | Default | Description |
|--------|------|----------|---------|-------------|
| `id` | ID | Yes | - | Component identifier |
| `one_wire` | ID | **Yes** | - | Reference to shared 1-Wire bus |
| `inverted` | bool | No | `false` | Global output inversion |
| `outputs` | list | No | `[]` | List of output configurations |
| `binary_sensors` | list | No | `[]` | List of binary sensor configurations |

### Output Configuration

| Option | Type | Required | Default | Description |
|--------|------|----------|---------|-------------|
| `id` | ID | Yes | - | Output identifier |
| `channel` | int | Yes | - | Channel number (0-1 for DS2413, 0-7 for DS2408) |
| `device_index` | int | No | `0` | Device index when multiple devices on bus |
| `inverted` | bool | No | `false` | Per-output inversion |

### Binary Sensor Configuration

| Option | Type | Required | Default | Description |
|--------|------|----------|---------|-------------|
| `id` | ID | Yes | - | Sensor identifier |
| `channel` | int | Yes | - | Channel number (0-1 for DS2413, 0-7 for DS2408) |
| `device_index` | int | No | `0` | Device index when multiple devices on bus |
| `inverted` | bool | No | `false` | Invert sensor reading |

## Using Outputs

After configuration, use outputs with switches, lights, or automations:

```yaml
switch:
  - platform: output
    name: "Relay 1"
    output: output_0

  - platform: output
    name: "Relay 2"  
    output: output_1
```

## Hardware Notes

### DS2413
- Family code: `0x3A`
- 2 open-drain PIO channels
- Active-low outputs (0 = conducting/ON)

### DS2408
- Family code: `0x29`
- 8 open-drain PIO channels
- Active-low outputs (0 = conducting/ON)

### Wiring
- **VCC**: 3.3V or 5V
- **GND**: Ground
- **IO**: 1-Wire data (requires 4.7kΩ pull-up resistor)

## Troubleshooting

### No devices found
- Check wiring and pull-up resistor
- Verify correct GPIO pin in `one_wire` configuration
- Enable debug logging: `logger: level: DEBUG`

### Outputs not switching
- Verify device family code in logs (should be `0x3A` for DS2413 or `0x29` for DS2408)
- Check `inverted` settings
- DS24xx outputs are active-low by default

## License

MIT License - See LICENSE file for details.

## References

- [DS2413 Datasheet](https://www.analog.com/en/products/ds2413.html)
- [DS2408 Datasheet](https://www.analog.com/en/products/ds2408.html)
- [ESPHome External Components](https://esphome.io/components/external_components.html)
- [ESPHome 1-Wire Bus](https://esphome.io/components/one_wire.html)
