#pragma once

#include <Arduino.h>
#include <Wire.h>

enum class AdcRoute : uint8_t {
  SE_LIN1_RIN1 = 0,
  SE_LIN2_RIN2,
  DIFF_LIN1_RIN1,
  DIFF_LIN2_RIN2,
};

struct Es8388DigitalConfig {
  const char *name;          // tag in logs
  uint8_t master_mode_reg;   // reg 0x08
  uint8_t dac_format_reg;    // reg 0x17
  uint8_t adc_format_reg;    // reg 0x0C

  // Default: ESP=master, codec=slave, 16-bit I2S Philips for both DAC and ADC.
  // adc_fmt=0x0C  : reg 0x0C bit[3:2]=11 (16-bit) | bit[1:0]=00 (I2S Philips)
  Es8388DigitalConfig(const char *n = "I2S16_CODEC_SLAVE", uint8_t master = 0x00,
                      uint8_t dac_fmt = 0x18, uint8_t adc_fmt = 0x0C)
      : name(n), master_mode_reg(master), dac_format_reg(dac_fmt), adc_format_reg(adc_fmt) {}
};

class Es8388Codec {
 public:
  explicit Es8388Codec(uint8_t addr) : addr_(addr) {}

  void begin(int sda, int scl, uint32_t freq = 100000);
  bool ping();
  void scan();

  bool init(AdcRoute route);
  bool init(AdcRoute route, const Es8388DigitalConfig &digital_cfg);
  bool set_dac_mute(bool mute);
  bool set_output_enabled(bool enable);
  bool set_analog_bypass(bool enable);
  void dump_registers();
  bool debug_write_register(uint8_t reg, uint8_t val);
  bool debug_read_register(uint8_t reg, uint8_t &val);

 private:
  uint8_t addr_;
  bool write_reg(uint8_t reg, uint8_t val);
  bool read_reg(uint8_t reg, uint8_t &val);
};
