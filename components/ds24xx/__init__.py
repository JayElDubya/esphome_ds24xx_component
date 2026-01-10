import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output as output_ns
from esphome.components import binary_sensor as binary_sensor_ns
try:
    from esphome.components import onewire as onewire_ns
    HAVE_ONEWIRE = True
except Exception:
    onewire_ns = None
    HAVE_ONEWIRE = False
from esphome.const import CONF_ID

AUTO_LOAD = ["output", "binary_sensor"]

ds24xx_ns = cg.esphome_ns.namespace('ds24xx')
DS24xxComponent = ds24xx_ns.class_('DS24xxComponent', cg.Component)
DS24xxOutput = ds24xx_ns.class_('DS24xxOutput', output_ns.BinaryOutput)
DS24xxBinarySensor = ds24xx_ns.class_('DS24xxBinarySensor', binary_sensor_ns.BinarySensor)

CONF_ONE_WIRE_PIN = 'one_wire_pin'
CONF_ONE_WIRE = 'one_wire'
CONF_INVERTED = 'inverted'
CONF_OUTPUTS = 'outputs'
CONF_BINARY_SENSORS = 'binary_sensors'
CONF_CHANNEL = 'channel'
CONF_DEVICE_INDEX = 'device_index'

OUTPUT_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DS24xxOutput),
    cv.Required(CONF_CHANNEL): cv.int_,
    cv.Optional(CONF_DEVICE_INDEX, default=0): cv.int_,
})

BINARY_SENSOR_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DS24xxBinarySensor),
    cv.Required(CONF_CHANNEL): cv.int_,
    cv.Optional(CONF_DEVICE_INDEX, default=0): cv.int_,
})

# Build schema fields dynamically so `one_wire` key is accepted even when
# the onewire component is not present (we'll raise a clear error in to_code
# if user supplies `one_wire` but the runtime ESPhome environment lacks it).
schema_fields = {
    cv.GenerateID(): cv.declare_id(DS24xxComponent),
    cv.Optional(CONF_ONE_WIRE_PIN): cv.int_,
    cv.Optional(CONF_INVERTED, default=False): cv.boolean,
    cv.Optional(CONF_OUTPUTS, default=[]): cv.ensure_list(OUTPUT_SCHEMA),
    cv.Optional(CONF_BINARY_SENSORS, default=[]): cv.ensure_list(BINARY_SENSOR_SCHEMA),
}

if HAVE_ONEWIRE:
    schema_fields[cv.Optional(CONF_ONE_WIRE)] = cv.use_id(onewire_ns.OneWireBus)
else:
    # accept the key syntactically so YAML validation won't fail; we'll error
    # clearly during code generation if the user actually provided it.
    schema_fields[cv.Optional(CONF_ONE_WIRE)] = cv.Any()

CONFIG_SCHEMA = cv.Schema(schema_fields).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    # create main component, prefer shared one_wire bus when provided
    inverted = config[CONF_INVERTED]
    if HAVE_ONEWIRE and CONF_ONE_WIRE in config:
        ow = await cg.get_variable(config[CONF_ONE_WIRE])
        comp = cg.new_Pvariable(config[CONF_ID], ow, inverted)
    else:
        one_wire_pin = config[CONF_ONE_WIRE_PIN]
        comp = cg.new_Pvariable(config[CONF_ID], one_wire_pin, inverted)
    await cg.register_component(comp, config)

    # create outputs
    for o in config.get(CONF_OUTPUTS, []):
        out = cg.new_Pvariable(o[CONF_ID], comp, o[CONF_CHANNEL], o[CONF_DEVICE_INDEX])
        # register with ESPHome output subsystem
        await output_ns.register_output(out, o)

    # create binary sensors
    for b in config.get(CONF_BINARY_SENSORS, []):
        bs = cg.new_Pvariable(b[CONF_ID], comp, b[CONF_CHANNEL], b[CONF_DEVICE_INDEX])
        await binary_sensor_ns.register_binary_sensor(bs, b)
