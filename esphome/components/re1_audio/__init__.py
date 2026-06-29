"""ESPHome external component for the N16R8 RE1.0 / ES8388 audio board.

Wraps the ES8388 codec (initialised over I2C) plus a self-contained I2S RX/TX
driver, exposing two simple actions you can call from automations / lambdas:

    id(audio).record_then_play(uint32_t seconds);
    id(audio).play_last();
    id(audio).log_diag();

This is intentionally NOT using the standard `i2s_audio` / `microphone` /
`speaker` components, because ES8388 is not yet a first-class codec there
and we already have a hardware-verified configuration.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID

CODEOWNERS = ["@local"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = []

re1_audio_ns = cg.esphome_ns.namespace("re1_audio")
Re1Audio = re1_audio_ns.class_("Re1Audio", cg.Component, i2c.I2CDevice)

CONF_MCLK_PIN = "mclk_pin"
CONF_BCLK_PIN = "bclk_pin"
CONF_LRCK_PIN = "lrck_pin"
CONF_DIN_PIN = "din_pin"
CONF_DOUT_PIN = "dout_pin"
CONF_SAMPLE_RATE = "sample_rate"
CONF_RECORD_SECONDS_MAX = "record_seconds_max"
CONF_PLAYBACK_GAIN = "playback_gain"
CONF_RECORD_GAIN = "record_gain"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Re1Audio),
            cv.Required(CONF_MCLK_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_BCLK_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_LRCK_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_DIN_PIN): cv.int_range(min=0, max=48),
            cv.Required(CONF_DOUT_PIN): cv.int_range(min=0, max=48),
            cv.Optional(CONF_SAMPLE_RATE, default=24000): cv.int_range(
                min=8000, max=48000
            ),
            cv.Optional(CONF_RECORD_SECONDS_MAX, default=10): cv.int_range(
                min=1, max=30
            ),
            cv.Optional(CONF_PLAYBACK_GAIN, default=1.0): cv.float_range(
                min=0.25, max=8.0
            ),
            cv.Optional(CONF_RECORD_GAIN, default=1.0): cv.float_range(
                min=0.25, max=8.0
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x10))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_pins(
        config[CONF_MCLK_PIN],
        config[CONF_BCLK_PIN],
        config[CONF_LRCK_PIN],
        config[CONF_DIN_PIN],
        config[CONF_DOUT_PIN],
    ))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_record_seconds_max(config[CONF_RECORD_SECONDS_MAX]))
    cg.add(var.set_playback_gain(config[CONF_PLAYBACK_GAIN]))
    cg.add(var.set_record_gain(config[CONF_RECORD_GAIN]))
