#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/i2c/i2c.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace re1_audio {

// Self-contained record-and-playback component for the N16R8 RE1.0 board
// (ES32-S3 + ES8388 codec). The ES8388 register sequence and the L/R diff
// mixing here mirror the hardware-verified PlatformIO version in
// `src/main.cpp` + `src/es8388_codec.cpp`, so behaviour is identical.
class Re1Audio : public Component, public i2c::I2CDevice {
 public:
  void set_pins(int mclk, int bclk, int lrck, int din, int dout) {
    pin_mclk_ = mclk;
    pin_bclk_ = bclk;
    pin_lrck_ = lrck;
    pin_din_ = din;
    pin_dout_ = dout;
  }
  void set_sample_rate(uint32_t sr) { sample_rate_ = sr; }
  void set_record_seconds_max(uint32_t s) { record_seconds_max_ = s; }
  void set_playback_gain(float g) { playback_gain_ = g; }
  void set_record_gain(float g) { record_gain_ = g; }

  // Component lifecycle.
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override {
    // Bring up codec/I2S after I2C bus is ready but before user automations.
    return setup_priority::AFTER_CONNECTION;
  }

  // Public actions, callable from YAML lambdas.
  // record_then_play(5) -> record `seconds` of audio then auto-play it back.
  void record_then_play(uint32_t seconds);
  // play_last() -> just play whatever is currently in the record buffer.
  void play_last();
  // log_diag() -> print one-shot diagnostic line over serial.
  void log_diag();

 protected:
  enum class State : uint8_t { kIdle, kRecording, kPlaying };

  // ===== ES8388 codec config =====
  bool codec_init_();
  bool codec_write_(uint8_t reg, uint8_t val, const char *note);
  bool codec_read_(uint8_t reg, uint8_t &val);
  bool codec_set_dac_mute_(bool mute);
  bool codec_set_output_enabled_(bool enabled);

  // ===== I2S driver =====
  bool i2s_install_();
  void i2s_uninstall_();

  // ===== Recording / Playback step (called from loop) =====
  void step_recording_();
  void step_playing_();
  void start_recording_(uint32_t seconds);
  void stop_recording_(const char *reason);
  void start_playback_();
  void stop_playback_(const char *reason);

  // ===== Pins / config =====
  int pin_mclk_{8};
  int pin_bclk_{3};
  int pin_lrck_{9};
  int pin_din_{10};
  int pin_dout_{46};
  uint32_t sample_rate_{24000};
  uint32_t record_seconds_max_{10};
  float playback_gain_{1.0f};
  float record_gain_{1.0f};

  // ===== Runtime state =====
  State state_{State::kIdle};
  bool codec_ok_{false};
  bool i2s_installed_{false};

  // PSRAM-backed mono int16 buffer.
  int16_t *record_buffer_{nullptr};
  size_t record_capacity_{0};
  size_t record_filled_{0};
  size_t play_pos_{0};
  uint32_t record_target_samples_{0};
  uint32_t record_started_ms_{0};
  uint32_t play_started_ms_{0};
  uint32_t last_log_ms_{0};

  // Stats accumulators.
  int32_t stat_min_l_{0};
  int32_t stat_max_l_{0};
  int32_t stat_min_r_{0};
  int32_t stat_max_r_{0};
  int32_t stat_min_m_{0};
  int32_t stat_max_m_{0};
  uint64_t stat_sum_abs_m_{0};
  uint32_t stat_frames_{0};
};

}  // namespace re1_audio
}  // namespace esphome
