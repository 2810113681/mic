"""RE1.0 board ES8388 audio_dac — uses hardware-verified register sequence."""

import esphome.codegen as cg
from esphome.components import i2c
from esphome.components.audio_dac import AudioDac
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@local"]

re1_es8388_ns = cg.esphome_ns.namespace("re1_es8388")
Re1Es8388 = re1_es8388_ns.class_("Re1Es8388", AudioDac, cg.Component, i2c.I2CDevice)

DEPENDENCIES = ["i2c"]

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(Re1Es8388)})
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x10))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
