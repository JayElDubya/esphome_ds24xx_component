"""
DS24xx ESPHome Component

Provides support for Dallas DS2413 (2-channel) and DS2408 (8-channel) 1-Wire GPIO expanders.
Requires ESPHome's shared one_wire bus.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output as output_ns
from esphome.components import one_wire as one_wire_ns
from esphome.const import CONF_ID, CONF_CHANNEL, CONF_INVERTED

# Optional binary_sensor support
try:
    from esphome.components import binary_sensor as binary_sensor_ns
    HAVE_BINARY_SENSOR = True
except ImportError:
    binary_sensor_ns = None
    HAVE_BINARY_SENSOR = False

# Configuration keys
CONF_ONE_WIRE = "one_wire"
CONF_OUTPUTS = "outputs"
CONF_BINARY_SENSORS = "binary_sensors"
CONF_DEVICE_INDEX = "device_index"

# Namespace and class declarations
ds24xx_ns = cg.esphome_ns.namespace("ds24xx")
DS24xxComponent = ds24xx_ns.class_("DS24xxComponent", cg.Component)
DS24xxOutput = ds24xx_ns.class_("DS24xxOutput", output_ns.BinaryOutput, cg.Component)

if HAVE_BINARY_SENSOR:
    DS24xxBinarySensor = ds24xx_ns.class_(
        "DS24xxBinarySensor", binary_sensor_ns.BinarySensor, cg.Component
    )

# Schema for individual outputs
OUTPUT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DS24xxOutput),
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=7),
        cv.Optional(CONF_DEVICE_INDEX, default=0): cv.uint8_t,
        cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    }
)

# Schema for binary sensors (optional)
if HAVE_BINARY_SENSOR:
    BINARY_SENSOR_SCHEMA = cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DS24xxBinarySensor),
            cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=7),
            cv.Optional(CONF_DEVICE_INDEX, default=0): cv.uint8_t,
            cv.Optional(CONF_INVERTED, default=False): cv.boolean,
        }
    )

# Main component schema
_SCHEMA_FIELDS = {
    cv.GenerateID(): cv.declare_id(DS24xxComponent),
    cv.Required(CONF_ONE_WIRE): cv.use_id(one_wire_ns.OneWireBus),
    cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    cv.Optional(CONF_OUTPUTS, default=[]): cv.ensure_list(OUTPUT_SCHEMA),
}

if HAVE_BINARY_SENSOR:
    _SCHEMA_FIELDS[cv.Optional(CONF_BINARY_SENSORS, default=[])] = cv.ensure_list(
        BINARY_SENSOR_SCHEMA
    )

CONFIG_SCHEMA = cv.Schema(_SCHEMA_FIELDS).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    """Generate C++ code for the ds24xx component."""
    # Get the shared 1-Wire bus
    bus = await cg.get_variable(config[CONF_ONE_WIRE])

    # Create main component
    comp = cg.new_Pvariable(config[CONF_ID], bus, config[CONF_INVERTED])
    await cg.register_component(comp, config)

    # Create outputs
    for output_conf in config.get(CONF_OUTPUTS, []):
        out = cg.new_Pvariable(
            output_conf[CONF_ID],
            comp,
            output_conf[CONF_CHANNEL],
            output_conf[CONF_DEVICE_INDEX],
        )
        cg.add(out.set_inverted(output_conf[CONF_INVERTED]))
        await output_ns.register_output(out, output_conf)

    # Create binary sensors (if available)
    if HAVE_BINARY_SENSOR:
        for sensor_conf in config.get(CONF_BINARY_SENSORS, []):
            sensor = cg.new_Pvariable(
                sensor_conf[CONF_ID],
                comp,
                sensor_conf[CONF_CHANNEL],
                sensor_conf[CONF_DEVICE_INDEX],
            )
            cg.add(sensor.set_inverted(sensor_conf[CONF_INVERTED]))
            await binary_sensor_ns.register_binary_sensor(sensor, sensor_conf)
