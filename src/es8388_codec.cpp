#include "es8388_codec.h"

#define CLOGI(fmt, ...) \
  Serial.printf("[%10lu][CODEC][INFO ] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#define CLOGW(fmt, ...) \
  Serial.printf("[%10lu][CODEC][WARN ] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#define CLOGE(fmt, ...) \
  Serial.printf("[%10lu][CODEC][ERROR] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)

static const char *route_name(AdcRoute route) {
  switch (route) {
    case AdcRoute::SE_LIN1_RIN1:
      return "SE_LIN1_RIN1";
    case AdcRoute::SE_LIN2_RIN2:
      return "SE_LIN2_RIN2";
    case AdcRoute::DIFF_LIN1_RIN1:
      return "DIFF_LIN1_RIN1";
    case AdcRoute::DIFF_LIN2_RIN2:
      return "DIFF_LIN2_RIN2";
    default:
      return "UNKNOWN";
  }
}

// Manual I2C bus recovery: when a slave got stuck mid-byte (e.g. after an unclean
// MCU reset / esptool stub flasher run) it can hold SDA low forever, blocking all
// subsequent transactions. Toggling SCL up to 9 times while SDA is still low gives
// the slave a chance to finish its in-flight read and release SDA, after which we
// issue a manual STOP and let the regular Wire driver take over.
static void i2c_bus_recovery(int sda, int scl) {
  // First sample the lines without any drive: if the bus has external pull-ups
  // and no slave is holding SDA low, both lines should already read HIGH.
  pinMode(scl, INPUT_PULLUP);
  pinMode(sda, INPUT_PULLUP);
  delayMicroseconds(50);
  int sda_idle = digitalRead(sda);
  int scl_idle = digitalRead(scl);
  Serial.printf("[%10lu][CODEC][INFO ] i2c_pre_recovery_state sda_idle=%d scl_idle=%d\n",
                (unsigned long)millis(), sda_idle, scl_idle);

  pinMode(scl, OUTPUT_OPEN_DRAIN);
  pinMode(sda, INPUT_PULLUP);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);

  int sda_low_cycles = 0;
  for (int i = 0; i < 9; ++i) {
    if (digitalRead(sda) == HIGH) break;
    sda_low_cycles++;
    digitalWrite(scl, LOW);
    delayMicroseconds(10);
    digitalWrite(scl, HIGH);
    delayMicroseconds(10);
  }

  // Generate a manual STOP condition: SDA goes from low to high while SCL is high.
  pinMode(sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda, LOW);
  delayMicroseconds(10);
  digitalWrite(scl, HIGH);
  delayMicroseconds(10);
  digitalWrite(sda, HIGH);
  delayMicroseconds(10);

  // Release pins so Wire.begin can reclaim them.
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  Serial.printf("[%10lu][CODEC][INFO ] i2c_bus_recovery sda=%d scl=%d sda_low_cycles=%d\n",
                (unsigned long)millis(), sda, scl, sda_low_cycles);
}

void Es8388Codec::begin(int sda, int scl, uint32_t freq) {
  i2c_bus_recovery(sda, scl);
  Wire.begin(sda, scl, freq);
  CLOGI("Wire.begin sda=%d scl=%d freq=%lu", sda, scl, (unsigned long)freq);
}

bool Es8388Codec::write_reg(uint8_t reg, uint8_t val) {
  for (int attempt = 1; attempt <= 3; ++attempt) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    int rc = Wire.endTransmission();
    if (rc == 0) {
      return true;
    }
    if (attempt < 3) {
      delay(1);
    } else {
      CLOGE("I2C write failed addr=0x%02X reg=0x%02X val=0x%02X rc=%d", addr_, reg, val, rc);
    }
  }
  return false;
}

bool Es8388Codec::read_reg(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  int rc = Wire.endTransmission(false);
  if (rc != 0) {
    CLOGE("I2C read addr stage failed addr=0x%02X reg=0x%02X rc=%d", addr_, reg, rc);
    return false;
  }
  int got = Wire.requestFrom((int)addr_, 1);
  if (got != 1) {
    CLOGE("I2C read data stage failed addr=0x%02X reg=0x%02X got=%d", addr_, reg, got);
    return false;
  }
  val = Wire.read();
  return true;
}

bool Es8388Codec::ping() {
  Wire.beginTransmission(addr_);
  int rc = Wire.endTransmission();
  bool ok = (rc == 0);
  CLOGI("ping addr=0x%02X result=%s rc=%d", addr_, ok ? "ok" : "fail", rc);
  return ok;
}

void Es8388Codec::scan() {
  CLOGI("I2C scan start");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      CLOGI("I2C device found 0x%02X%s", a, (a == addr_) ? " <-ES8388" : "");
    }
  }
  CLOGI("I2C scan end found=%u", found);
}

bool Es8388Codec::init(AdcRoute route) {
  Es8388DigitalConfig default_cfg;
  return init(route, default_cfg);
}

bool Es8388Codec::init(AdcRoute route, const Es8388DigitalConfig &digital_cfg) {
  CLOGI("init start route=%s addr=0x%02X digital=%s", route_name(route), addr_,
        (digital_cfg.name && digital_cfg.name[0] != '\0') ? digital_cfg.name : "UNNAMED");
  auto wr = [&](uint8_t reg, uint8_t val, const char *note) -> bool {
    bool ok = write_reg(reg, val);
    if (ok) {
      CLOGI("WR reg=0x%02X val=0x%02X note=%s", reg, val, note);
    } else {
      CLOGE("WR FAILED reg=0x%02X val=0x%02X note=%s", reg, val, note);
    }
    return ok;
  };

  // Base config: align with known stable ES8388 startup sequence.
  if (!wr(0x08, digital_cfg.master_mode_reg, "MASTERMODE (I2S master/slave)")) return false;
  if (!wr(0x01, 0x50, "CONTROL2: bias/reference")) return false;
  if (!wr(0x02, 0x00, "CHIPPOWER: normal power")) return false;
  if (!wr(0x00, 0x16, "CONTROL1: play+record mode + EnRef")) return false;

  // Keep these as in many ESP32 ES8388 bring-ups.
  if (!wr(0x35, 0xA0, "internal DLL tuning")) return false;
  if (!wr(0x37, 0xD0, "LOUT gain tuning")) return false;
  if (!wr(0x39, 0xD0, "ROUT gain tuning")) return false;

  // DAC path.
  if (!wr(0x04, 0xC0, "DAC output off during init")) return false;
  if (!wr(0x17, digital_cfg.dac_format_reg, "DAC serial format/word length")) return false;
  if (!wr(0x18, 0x02, "DAC ratio 256Fs")) return false;
  if (!wr(0x1A, 0x00, "DAC L volume 0dB")) return false;
  if (!wr(0x1B, 0x00, "DAC R volume 0dB")) return false;
  if (!wr(0x26, 0x00, "DAC mixer source")) return false;
  if (!wr(0x27, 0x90, "left mixer: DAC only")) return false;
  if (!wr(0x2A, 0x90, "right mixer: DAC only")) return false;
  if (!wr(0x2B, 0x80, "ADC/DAC same LRCK")) return false;
  if (!wr(0x2D, 0x00, "analog output resistance default")) return false;
  if (!wr(0x2E, 0x1E, "LOUT1 volume 0dB")) return false;
  if (!wr(0x2F, 0x1E, "ROUT1 volume 0dB")) return false;
  if (!wr(0x30, 0x1E, "LOUT2 volume 0dB")) return false;
  if (!wr(0x31, 0x1E, "ROUT2 volume 0dB")) return false;

  // ADC path.
  if (!wr(0x03, 0xFF, "ADC power down before ADC setup")) return false;
  // ESP-ADF default: 0x88 -> MicAmpL=MicAmpR=24dB, suitable for typical electret/MEMS mic.
  // 0x00 (0dB) is too weak for mic-level signals and was a contributor to the all-zero capture.
  if (!wr(0x09, 0x88, "ADC PGA gain L=24dB R=24dB")) return false;

  uint8_t reg10 = 0x00;
  // 0x0B decides LIN1/RIN1 vs LIN2/RIN2 path pairing.
  // Use route-dependent value instead of fixed mono-right mode.
  uint8_t reg11 = 0x02;
  switch (route) {
    case AdcRoute::SE_LIN1_RIN1:
      reg10 = 0x00;
      reg11 = 0x02;
      break;
    case AdcRoute::SE_LIN2_RIN2:
      reg10 = 0x50;
      reg11 = 0x82;
      break;
    case AdcRoute::DIFF_LIN1_RIN1:
      reg10 = 0xF0;
      reg11 = 0x02;
      break;
    case AdcRoute::DIFF_LIN2_RIN2:
      // ES8388 difference mode is selected via 0xF0.
      reg10 = 0xF0;
      reg11 = 0x82;
      break;
  }

  CLOGI("ADC route resolved reg0A=0x%02X reg0B=0x%02X", reg10, reg11);
  if (!wr(0x0A, reg10, "ADC input route select")) return false;
  if (!wr(0x0B, reg11, "ADC control3")) return false;
  if (!wr(0x0C, digital_cfg.adc_format_reg, "ADC serial format/word length")) return false;
  if (!wr(0x0D, 0x02, "ADC ratio 256Fs")) return false;
  // ES8388 datasheet: reg 0x0F bit[5]=ADCMUTE (1=mute), bit[6]=ADC_HPF.
  // Old value 0x60 actually muted the ADC digital output (=> ASDOUT stuck at 0),
  // which made i2s_read return all-zero samples even though clocks were running.
  // Use 0x00 to ensure ADCMUTE=0 + HPF off (cleanest baseline for verification).
  if (!wr(0x0F, 0x00, "ADC unmute (bit5=0) + HPF off")) return false;
  if (!wr(0x10, 0x00, "ADC left volume 0dB")) return false;
  if (!wr(0x11, 0x00, "ADC right volume 0dB")) return false;

  // Keep ALC disabled for deterministic debug capture.
  if (!wr(0x12, 0x00, "ALC control off")) return false;
  if (!wr(0x13, 0xA0, "ALC target/hold")) return false;
  if (!wr(0x14, 0x12, "ALC decay/attack")) return false;
  if (!wr(0x15, 0x06, "ALC mode")) return false;
  if (!wr(0x16, 0x00, "ALC hold")) return false;
  // ES8388 datasheet ADCPOWER (reg 0x03):
  //   bit[3] PdnMICB - 1=MICBIAS ENABLED, 0=disabled
  // The previous value 0x00 actually DISABLED MICBIAS, so an electret mic got
  // no bias voltage and produced no signal -> ADC always read zero, even though
  // the analog bypass path could still pick up weak coupling.
  // 0x09 (= 00001001) matches the ESP-ADF reference: MICBIAS on, all ADC paths up.
  if (!wr(0x03, 0x09, "ADC power on + MICBIAS enabled (bit3=1)")) return false;

  // Apply
  if (!wr(0x02, 0xF0, "apply power-up phase 1")) return false;
  delay(1);
  if (!wr(0x02, 0x00, "apply power-up phase 2")) return false;
  // Keep speaker path closed by default to avoid boot noise.
  if (!set_dac_mute(true)) return false;
  if (!set_output_enabled(false)) return false;
  CLOGI("init done: DAC muted + output disabled");
  return true;
}

bool Es8388Codec::set_dac_mute(bool mute) {
  // 0x3C: mute with soft ramp, 0x20: unmute with soft ramp.
  uint8_t val = mute ? 0x3C : 0x20;
  bool ok = write_reg(0x19, val);
  CLOGI("set_dac_mute(%s) reg0x19=0x%02X result=%s", mute ? "true" : "false", val,
        ok ? "ok" : "fail");
  return ok;
}

bool Es8388Codec::set_output_enabled(bool enable) {
  // 0x3C: enable full output path (board uses differential path into external AMP),
  // 0xC0: power down DAC output.
  uint8_t val = enable ? 0x3C : 0xC0;
  bool ok = write_reg(0x04, val);
  CLOGI("set_output_enabled(%s) reg0x04=0x%02X result=%s", enable ? "true" : "false", val,
        ok ? "ok" : "fail");
  return ok;
}

bool Es8388Codec::set_analog_bypass(bool enable) {
  // Route ADC analog input directly to output mixer for hardware path diagnosis.
  // If this has audible voice, MIC analog front-end is alive and issue is digital ADC/I2S path.
  bool ok = true;
  if (enable) {
    ok &= write_reg(0x26, 0x00);  // no DAC mix
    ok &= write_reg(0x27, 0x50);  // LOUT from LIN
    ok &= write_reg(0x2A, 0x50);  // ROUT from RIN
    ok &= write_reg(0x2B, 0x80);  // route control
    ok &= write_reg(0x03, 0x00);  // keep analog path open during bypass test
    ok &= write_reg(0x19, 0x20);  // DAC unmute soft
    ok &= write_reg(0x04, 0x3C);  // output enable (full path)
  } else {
    // Restore normal digital capture + DAC playback route after temporary analog bypass.
    ok &= write_reg(0x26, 0x00);  // DAC mixer source normal
    ok &= write_reg(0x27, 0x90);  // restore DAC mix to LOUT
    ok &= write_reg(0x2A, 0x90);  // restore DAC mix to ROUT
    ok &= write_reg(0x2B, 0x80);  // keep LRCK route
    ok &= write_reg(0x0F, 0x60);  // ADC unmute + fade in
    ok &= write_reg(0x03, 0x00);  // restore ADC power/input path
    ok &= write_reg(0x19, 0x3C);  // DAC mute soft
    ok &= write_reg(0x04, 0xC0);  // output off
  }
  CLOGI("set_analog_bypass(%s) result=%s", enable ? "true" : "false", ok ? "ok" : "fail");
  return ok;
}

bool Es8388Codec::debug_write_register(uint8_t reg, uint8_t val) {
  return write_reg(reg, val);
}

bool Es8388Codec::debug_read_register(uint8_t reg, uint8_t &val) {
  return read_reg(reg, val);
}

void Es8388Codec::dump_registers() {
  CLOGI("register dump start");
  for (uint8_t r = 0; r <= 0x3A; ++r) {
    uint8_t v = 0;
    if (read_reg(r, v)) {
      Serial.printf(" [%02X]=%02X", r, v);
    } else {
      Serial.printf(" [%02X]=??", r);
      CLOGW("register read failed reg=0x%02X", r);
    }
    if ((r & 7) == 7) Serial.println();
  }
  Serial.println();
  CLOGI("register dump end");
}
