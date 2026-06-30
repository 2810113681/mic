#include "re1_es8388.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace re1_es8388 {

static const char *const TAG = "re1_es8388";

bool Re1Es8388::codec_write_(uint8_t reg, uint8_t val, const char *note) {
  if (!this->write_byte(reg, val)) {
    ESP_LOGE(TAG, "I2C write FAIL reg=0x%02X val=0x%02X (%s)", reg, val, note);
    return false;
  }
  ESP_LOGV(TAG, "WR reg=0x%02X val=0x%02X (%s)", reg, val, note);
  return true;
}

bool Re1Es8388::codec_set_dac_mute_(bool mute) {
  return this->codec_write_(0x19, mute ? 0x3C : 0x20, "DAC mute");
}

bool Re1Es8388::codec_set_output_enabled_(bool enabled) {
  return this->codec_write_(0x04, enabled ? 0x3C : 0xC0, "DAC output enable");
}

bool Re1Es8388::codec_read_(uint8_t reg, uint8_t &val) {
  uint8_t v = 0;
  if (!this->read_byte(reg, &v)) {
    return false;
  }
  val = v;
  return true;
}

bool Re1Es8388::codec_init_() {
  ESP_LOGI(TAG, "ES8388 init start  i2c_addr=0x%02X", this->address_);

  uint8_t probe = 0;
  if (!this->codec_read_(0x00, probe)) {
    ESP_LOGE(TAG, "I2C probe FAILED (cannot read reg 0x00) — check SDA=GPIO14 SCL=GPIO47");
    return false;
  }
  ESP_LOGI(TAG, "I2C probe OK  reg0x00=0x%02X", probe);
  if (!this->codec_write_(0x08, 0x00, "MASTERMODE: codec slave")) return false;
  if (!this->codec_write_(0x01, 0x50, "CONTROL2: bias/reference")) return false;
  if (!this->codec_write_(0x02, 0x00, "CHIPPOWER: normal")) return false;
  if (!this->codec_write_(0x00, 0x16, "CONTROL1: play+record + EnRef")) return false;
  if (!this->codec_write_(0x35, 0xA0, "DLL tuning")) return false;
  if (!this->codec_write_(0x37, 0xD0, "LOUT tuning")) return false;
  if (!this->codec_write_(0x39, 0xD0, "ROUT tuning")) return false;

  if (!this->codec_write_(0x04, 0xC0, "DAC output OFF during init")) return false;
  if (!this->codec_write_(0x17, 0x18, "DAC fmt: 16-bit I2S")) return false;
  if (!this->codec_write_(0x18, 0x02, "DAC ratio 256Fs")) return false;
  if (!this->codec_write_(0x1A, 0x00, "DAC L vol 0dB")) return false;
  if (!this->codec_write_(0x1B, 0x00, "DAC R vol 0dB")) return false;
  if (!this->codec_write_(0x26, 0x00, "DAC mixer source")) return false;
  if (!this->codec_write_(0x27, 0x90, "left mixer: DAC only")) return false;
  if (!this->codec_write_(0x2A, 0x90, "right mixer: DAC only")) return false;
  if (!this->codec_write_(0x2B, 0x80, "ADC/DAC same LRCK")) return false;
  if (!this->codec_write_(0x2D, 0x00, "analog out R default")) return false;
  if (!this->codec_write_(0x2E, 0x1E, "LOUT1 vol")) return false;
  if (!this->codec_write_(0x2F, 0x1E, "ROUT1 vol")) return false;
  if (!this->codec_write_(0x30, 0x1E, "LOUT2 vol")) return false;
  if (!this->codec_write_(0x31, 0x1E, "ROUT2 vol")) return false;

  if (!this->codec_write_(0x03, 0xFF, "ADC power down before setup")) return false;
  if (!this->codec_write_(0x09, 0x88, "ADC PGA L=R=24dB")) return false;
  if (!this->codec_write_(0x0A, 0xF0, "ADC route: differential")) return false;
  if (!this->codec_write_(0x0B, 0x02, "ADC control3 (LIN1/RIN1 pair)")) return false;
  if (!this->codec_write_(0x0C, 0x0C, "ADC fmt: 16-bit I2S")) return false;
  if (!this->codec_write_(0x0D, 0x02, "ADC ratio 256Fs")) return false;
  if (!this->codec_write_(0x0F, 0x00, "ADC unmute + HPF off")) return false;
  if (!this->codec_write_(0x10, 0x00, "ADC L vol 0dB")) return false;
  if (!this->codec_write_(0x11, 0x00, "ADC R vol 0dB")) return false;
  if (!this->codec_write_(0x12, 0x00, "ALC off")) return false;
  if (!this->codec_write_(0x13, 0xA0, "ALC target/hold")) return false;
  if (!this->codec_write_(0x14, 0x12, "ALC decay/attack")) return false;
  if (!this->codec_write_(0x15, 0x06, "ALC mode")) return false;
  if (!this->codec_write_(0x16, 0x00, "ALC hold")) return false;
  if (!this->codec_write_(0x03, 0x09, "ADC power on + MICBIAS")) return false;

  if (!this->codec_write_(0x02, 0xF0, "power-up phase 1")) return false;
  delay(1);
  if (!this->codec_write_(0x02, 0x00, "power-up phase 2")) return false;

  this->codec_set_dac_mute_(true);
  this->codec_set_output_enabled_(false);
  ESP_LOGI(TAG, "ES8388 init done (output muted until playback)");
  return true;
}

void Re1Es8388::setup() {
  ESP_LOGI(TAG, "setup() begin (priority AFTER_CONNECTION, same as re1_audio)");
  // Let codec power/I2C bus settle after WiFi stack boot (matches re1_audio timing).
  delay(50);
  if (!this->codec_init_()) {
    ESP_LOGE(TAG, "ES8388 init failed — check I2C wiring SDA=14 SCL=47 addr=0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  this->is_muted_ = true;
  this->volume_ = 1.0f;
  ESP_LOGI(TAG, "setup() done");
}

void Re1Es8388::dump_config() {
  ESP_LOGCONFIG(TAG, "RE1 ES8388 Audio DAC:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Failed to initialize");
  }
}

bool Re1Es8388::set_mute_off() {
  if (!this->codec_set_output_enabled_(true)) return false;
  if (!this->codec_set_dac_mute_(false)) return false;
  this->is_muted_ = false;
  return true;
}

bool Re1Es8388::set_mute_on() {
  if (!this->codec_set_dac_mute_(true)) return false;
  if (!this->codec_set_output_enabled_(false)) return false;
  this->is_muted_ = true;
  return true;
}

bool Re1Es8388::set_volume(float volume) {
  volume = clamp(volume, 0.0f, 1.0f);
  this->volume_ = volume;
  // ES8388 DAC digital volume: 0x00 = 0 dB, higher values = attenuation.
  uint8_t reg_val = static_cast<uint8_t>((1.0f - volume) * 192.0f);
  if (!this->codec_write_(0x1A, reg_val, "DAC L vol")) return false;
  if (!this->codec_write_(0x1B, reg_val, "DAC R vol")) return false;
  return true;
}

}  // namespace re1_es8388
}  // namespace esphome
