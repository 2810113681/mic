#pragma once

#include "esphome/components/audio_dac/audio_dac.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"

namespace esphome {
namespace re1_es8388 {

/// ES8388 codec init for N16R8 RE1.0 — same register sequence as re1_audio / PlatformIO.
class Re1Es8388 : public audio_dac::AudioDac, public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  // Must run after I2C bus (1000) and BEFORE i2s_audio (600). re1_audio inits codec
  // then I2S in one setup(); voice_assistant uses separate i2s — codec must go first.
  float get_setup_priority() const override { return 990.0f; }

  bool set_mute_off() override;
  bool set_mute_on() override;
  bool set_volume(float volume) override;
  bool is_muted() override { return this->is_muted_; }
  float volume() override { return this->volume_; }

 protected:
  bool codec_write_(uint8_t reg, uint8_t val, const char *note);
  bool codec_read_(uint8_t reg, uint8_t &val);
  bool codec_init_();
  bool codec_set_dac_mute_(bool mute);
  bool codec_set_output_enabled_(bool enabled);

  float volume_{1.0f};
};

}  // namespace re1_es8388
}  // namespace esphome
