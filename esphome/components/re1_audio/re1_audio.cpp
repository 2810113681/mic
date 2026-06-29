#include "re1_audio.h"

#include "esphome/core/hal.h"  // millis(), delay()

#include <cstring>
#include <esp_heap_caps.h>
#include <driver/i2s.h>

namespace esphome {
namespace re1_audio {

static const char *const TAG = "re1_audio";

// ============================================================================
//  ES8388 codec init - mirrors hardware-verified PlatformIO config
// ============================================================================
bool Re1Audio::codec_write_(uint8_t reg, uint8_t val, const char *note) {
  // i2c::I2CDevice::write_byte returns true on success.
  if (!this->write_byte(reg, val)) {
    ESP_LOGE(TAG, "I2C write FAIL reg=0x%02X val=0x%02X (%s)", reg, val, note);
    return false;
  }
  ESP_LOGD(TAG, "WR reg=0x%02X val=0x%02X (%s)", reg, val, note);
  return true;
}

bool Re1Audio::codec_read_(uint8_t reg, uint8_t &val) {
  uint8_t v = 0;
  if (!this->read_byte(reg, &v)) return false;
  val = v;
  return true;
}

bool Re1Audio::codec_set_dac_mute_(bool mute) {
  return codec_write_(0x19, mute ? 0x3C : 0x20, "DAC mute");
}

bool Re1Audio::codec_set_output_enabled_(bool enabled) {
  return codec_write_(0x04, enabled ? 0x3C : 0xC0, "DAC output enable");
}

bool Re1Audio::codec_init_() {
  ESP_LOGI(TAG, "ES8388 init start  i2c_addr=0x%02X", this->address_);

  // Master/slave + power.
  if (!codec_write_(0x08, 0x00, "MASTERMODE: codec slave")) return false;
  if (!codec_write_(0x01, 0x50, "CONTROL2: bias/reference")) return false;
  if (!codec_write_(0x02, 0x00, "CHIPPOWER: normal")) return false;
  if (!codec_write_(0x00, 0x16, "CONTROL1: play+record + EnRef")) return false;

  // Internal DLL / output driver tuning (matches reference projects).
  if (!codec_write_(0x35, 0xA0, "DLL tuning")) return false;
  if (!codec_write_(0x37, 0xD0, "LOUT tuning")) return false;
  if (!codec_write_(0x39, 0xD0, "ROUT tuning")) return false;

  // ===== DAC path =====
  if (!codec_write_(0x04, 0xC0, "DAC output OFF during init")) return false;
  // reg 0x17 DACCONTROL1: 0x18 = 16-bit + I2S Philips.
  if (!codec_write_(0x17, 0x18, "DAC fmt: 16-bit I2S")) return false;
  if (!codec_write_(0x18, 0x02, "DAC ratio 256Fs")) return false;
  if (!codec_write_(0x1A, 0x00, "DAC L vol 0dB")) return false;
  if (!codec_write_(0x1B, 0x00, "DAC R vol 0dB")) return false;
  if (!codec_write_(0x26, 0x00, "DAC mixer source")) return false;
  if (!codec_write_(0x27, 0x90, "left mixer: DAC only")) return false;
  if (!codec_write_(0x2A, 0x90, "right mixer: DAC only")) return false;
  if (!codec_write_(0x2B, 0x80, "ADC/DAC same LRCK")) return false;
  if (!codec_write_(0x2D, 0x00, "analog out R default")) return false;
  if (!codec_write_(0x2E, 0x1E, "LOUT1 vol")) return false;
  if (!codec_write_(0x2F, 0x1E, "ROUT1 vol")) return false;
  if (!codec_write_(0x30, 0x1E, "LOUT2 vol")) return false;
  if (!codec_write_(0x31, 0x1E, "ROUT2 vol")) return false;

  // ===== ADC path =====
  if (!codec_write_(0x03, 0xFF, "ADC power down before setup")) return false;
  if (!codec_write_(0x09, 0x88, "ADC PGA L=R=24dB")) return false;
  // Differential MIC1/MIC2 routing (LIN1/RIN1 differential).
  if (!codec_write_(0x0A, 0xF0, "ADC route: differential")) return false;
  if (!codec_write_(0x0B, 0x02, "ADC control3 (LIN1/RIN1 pair)")) return false;
  // reg 0x0C ADCCONTROL4: bit[3:2]=11 (16bit), bit[1:0]=00 (I2S Philips).
  // Was 0x0D (Left-Justify) which caused 1-BCLK offset = loud buzz.
  if (!codec_write_(0x0C, 0x0C, "ADC fmt: 16-bit I2S")) return false;
  if (!codec_write_(0x0D, 0x02, "ADC ratio 256Fs")) return false;
  if (!codec_write_(0x0F, 0x00, "ADC unmute + HPF off")) return false;
  if (!codec_write_(0x10, 0x00, "ADC L vol 0dB")) return false;
  if (!codec_write_(0x11, 0x00, "ADC R vol 0dB")) return false;

  // ALC off (deterministic capture).
  if (!codec_write_(0x12, 0x00, "ALC off")) return false;
  if (!codec_write_(0x13, 0xA0, "ALC target/hold")) return false;
  if (!codec_write_(0x14, 0x12, "ALC decay/attack")) return false;
  if (!codec_write_(0x15, 0x06, "ALC mode")) return false;
  if (!codec_write_(0x16, 0x00, "ALC hold")) return false;

  // ADCPOWER bit3 = MICBIAS enable.
  if (!codec_write_(0x03, 0x09, "ADC power on + MICBIAS")) return false;

  // Phase up.
  if (!codec_write_(0x02, 0xF0, "power-up phase 1")) return false;
  delay(1);
  if (!codec_write_(0x02, 0x00, "power-up phase 2")) return false;

  // Start with output muted to avoid boot pop.
  codec_set_dac_mute_(true);
  codec_set_output_enabled_(false);
  ESP_LOGI(TAG, "ES8388 init done (output muted until first PLAY)");
  return true;
}

// ============================================================================
//  I2S driver - ESP master, codec slave, 16-bit Philips I2S, stereo
// ============================================================================
bool Re1Audio::i2s_install_() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate = sample_rate_;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 6;
  cfg.dma_buf_len = 256;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;
  cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = pin_mclk_;
  pins.bck_io_num = pin_bclk_;
  pins.ws_io_num = pin_lrck_;
  pins.data_out_num = pin_dout_;
  pins.data_in_num = pin_din_;

  esp_err_t e = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "i2s_driver_install failed err=0x%x", (int)e);
    return false;
  }
  e = i2s_set_pin(I2S_NUM_0, &pins);
  if (e != ESP_OK) {
    ESP_LOGE(TAG, "i2s_set_pin failed err=0x%x", (int)e);
    i2s_driver_uninstall(I2S_NUM_0);
    return false;
  }
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_installed_ = true;
  ESP_LOGI(TAG,
           "I2S installed sr=%u bits=16 mclk=%d bclk=%d lrck=%d din=%d dout=%d",
           (unsigned)sample_rate_, pin_mclk_, pin_bclk_, pin_lrck_, pin_din_,
           pin_dout_);
  return true;
}

void Re1Audio::i2s_uninstall_() {
  if (i2s_installed_) {
    i2s_driver_uninstall(I2S_NUM_0);
    i2s_installed_ = false;
  }
}

// ============================================================================
//  Component lifecycle
// ============================================================================
void Re1Audio::setup() {
  ESP_LOGI(TAG, "setup() begin");

  // Allocate record buffer in PSRAM (mono, int16, sized for max seconds).
  record_capacity_ = sample_rate_ * record_seconds_max_;
  size_t bytes = record_capacity_ * sizeof(int16_t);
  record_buffer_ = (int16_t *) heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
  if (record_buffer_ == nullptr) {
    ESP_LOGW(TAG, "PSRAM alloc failed (%u bytes), falling back to DRAM",
             (unsigned)bytes);
    record_buffer_ = (int16_t *) heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
  }
  if (record_buffer_ == nullptr) {
    ESP_LOGE(TAG, "FATAL: cannot allocate record buffer (%u bytes)",
             (unsigned)bytes);
    this->mark_failed();
    return;
  }
  std::memset(record_buffer_, 0, bytes);
  ESP_LOGI(TAG, "record buffer allocated %u bytes (%u samples / %us max)",
           (unsigned)bytes, (unsigned)record_capacity_,
           (unsigned)record_seconds_max_);

  if (!codec_init_()) {
    ESP_LOGE(TAG, "ES8388 init FAILED");
    this->mark_failed();
    return;
  }
  codec_ok_ = true;

  if (!i2s_install_()) {
    ESP_LOGE(TAG, "I2S install FAILED");
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG,
           "setup() done. Press BTN1 to record-and-play 5s, BTN2 to replay, "
           "BTN3 to log diag.");
}

void Re1Audio::loop() {
  switch (state_) {
    case State::kIdle:
      break;
    case State::kRecording:
      step_recording_();
      break;
    case State::kPlaying:
      step_playing_();
      break;
  }
}

void Re1Audio::dump_config() {
  ESP_LOGCONFIG(TAG, "Re1Audio:");
  ESP_LOGCONFIG(TAG, "  I2C address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  MCLK pin:    GPIO%d", pin_mclk_);
  ESP_LOGCONFIG(TAG, "  BCLK pin:    GPIO%d", pin_bclk_);
  ESP_LOGCONFIG(TAG, "  LRCK pin:    GPIO%d", pin_lrck_);
  ESP_LOGCONFIG(TAG, "  DIN  pin:    GPIO%d  (codec ASDOUT -> ESP RX)",
                pin_din_);
  ESP_LOGCONFIG(TAG, "  DOUT pin:    GPIO%d  (ESP TX -> codec DSDIN)",
                pin_dout_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %u Hz", (unsigned)sample_rate_);
  ESP_LOGCONFIG(TAG, "  Max record:  %u s", (unsigned)record_seconds_max_);
  ESP_LOGCONFIG(TAG, "  Codec OK:    %s", codec_ok_ ? "yes" : "no");
}

// ============================================================================
//  Public actions
// ============================================================================
void Re1Audio::record_then_play(uint32_t seconds) {
  if (state_ != State::kIdle) {
    ESP_LOGW(TAG, "record_then_play ignored, state=%d", (int)state_);
    return;
  }
  if (seconds == 0) seconds = 5;
  if (seconds > record_seconds_max_) seconds = record_seconds_max_;
  start_recording_(seconds);
}

void Re1Audio::play_last() {
  if (state_ != State::kIdle) {
    ESP_LOGW(TAG, "play_last ignored, state=%d", (int)state_);
    return;
  }
  if (record_filled_ == 0) {
    ESP_LOGW(TAG, "play_last: buffer empty, record something first");
    return;
  }
  start_playback_();
}

void Re1Audio::log_diag() {
  ESP_LOGI(TAG,
           "DIAG state=%d codec_ok=%d i2s=%d sr=%u rec_filled=%u/%u play_pos=%u",
           (int)state_, codec_ok_ ? 1 : 0, i2s_installed_ ? 1 : 0,
           (unsigned)sample_rate_, (unsigned)record_filled_,
           (unsigned)record_capacity_, (unsigned)play_pos_);
  ESP_LOGI(TAG, "DIAG pins mclk=%d bclk=%d lrck=%d din=%d dout=%d", pin_mclk_,
           pin_bclk_, pin_lrck_, pin_din_, pin_dout_);
}

// ============================================================================
//  Recording / Playback engine
// ============================================================================
void Re1Audio::start_recording_(uint32_t seconds) {
  ESP_LOGI(TAG, "REC start seconds=%u", (unsigned)seconds);
  codec_set_dac_mute_(true);
  codec_set_output_enabled_(false);
  record_filled_ = 0;
  record_target_samples_ = sample_rate_ * seconds;
  if (record_target_samples_ > record_capacity_) {
    record_target_samples_ = record_capacity_;
  }
  stat_min_l_ = stat_min_r_ = stat_min_m_ = INT32_MAX;
  stat_max_l_ = stat_max_r_ = stat_max_m_ = INT32_MIN;
  stat_sum_abs_m_ = 0;
  stat_frames_ = 0;
  record_started_ms_ = millis();
  last_log_ms_ = record_started_ms_;
  i2s_zero_dma_buffer(I2S_NUM_0);
  state_ = State::kRecording;
}

void Re1Audio::stop_recording_(const char *reason) {
  uint32_t elapsed = millis() - record_started_ms_;
  uint32_t avg_abs = stat_frames_ > 0
                         ? (uint32_t)(stat_sum_abs_m_ / stat_frames_)
                         : 0;
  ESP_LOGI(TAG,
           "REC stop reason=%s samples=%u ms=%u L[%d,%d] R[%d,%d] M[%d,%d] "
           "avg|M|=%u",
           reason, (unsigned)record_filled_, (unsigned)elapsed,
           (int)stat_min_l_, (int)stat_max_l_, (int)stat_min_r_,
           (int)stat_max_r_, (int)stat_min_m_, (int)stat_max_m_,
           (unsigned)avg_abs);
  state_ = State::kIdle;
}

void Re1Audio::step_recording_() {
  // Pull one chunk per loop iteration (256 stereo frames = 1024 bytes).
  static constexpr size_t kFrames = 256;
  int16_t buf[kFrames * 2];
  size_t got_bytes = 0;
  esp_err_t e = i2s_read(I2S_NUM_0, buf, sizeof(buf), &got_bytes, 0);
  if (e != ESP_OK) {
    if (e != ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "i2s_read err=0x%x", (int)e);
    }
    return;
  }
  size_t frames = got_bytes / 4;  // 2 ch * 2 bytes
  for (size_t i = 0; i < frames; ++i) {
    if (record_filled_ >= record_target_samples_) break;
    int16_t l = buf[i * 2 + 0];
    int16_t r = buf[i * 2 + 1];
    // L and R come from differential MIC1/MIC2 pairs that are physically
    // wired in opposite polarity on this board, so combine via (L-R)/2 to
    // double the signal and reject common-mode noise. This is the same
    // mixing we use in the PlatformIO firmware after the same diagnosis.
    int32_t mono = ((int32_t)l - (int32_t)r) / 2;
    mono = (int32_t) ((float) mono * record_gain_);
    if (mono > 32767) mono = 32767;
    if (mono < -32768) mono = -32768;
    record_buffer_[record_filled_++] = (int16_t)mono;

    if (l < stat_min_l_) stat_min_l_ = l;
    if (l > stat_max_l_) stat_max_l_ = l;
    if (r < stat_min_r_) stat_min_r_ = r;
    if (r > stat_max_r_) stat_max_r_ = r;
    if (mono < stat_min_m_) stat_min_m_ = mono;
    if (mono > stat_max_m_) stat_max_m_ = mono;
    stat_sum_abs_m_ += (uint32_t)(mono < 0 ? -mono : mono);
    stat_frames_++;
  }

  uint32_t now = millis();
  if (now - last_log_ms_ >= 500) {
    last_log_ms_ = now;
    ESP_LOGI(TAG, "REC progress %u/%u (%.1f%%)",
             (unsigned)record_filled_, (unsigned)record_target_samples_,
             100.0f * record_filled_ / (float)record_target_samples_);
  }

  if (record_filled_ >= record_target_samples_) {
    stop_recording_("target_reached");
    // Auto-chain into playback so the user instantly hears their own voice.
    start_playback_();
  }
}

void Re1Audio::start_playback_() {
  if (record_filled_ == 0) {
    ESP_LOGW(TAG, "playback skipped: empty buffer");
    return;
  }
  ESP_LOGI(TAG, "PLAY start samples=%u (%u ms)", (unsigned)record_filled_,
           (unsigned)(record_filled_ * 1000UL / sample_rate_));
  codec_set_output_enabled_(true);
  codec_set_dac_mute_(false);
  play_pos_ = 0;
  play_started_ms_ = millis();
  last_log_ms_ = play_started_ms_;
  i2s_zero_dma_buffer(I2S_NUM_0);
  state_ = State::kPlaying;
}

void Re1Audio::stop_playback_(const char *reason) {
  uint32_t elapsed = millis() - play_started_ms_;
  ESP_LOGI(TAG, "PLAY stop reason=%s pos=%u/%u ms=%u", reason,
           (unsigned)play_pos_, (unsigned)record_filled_, (unsigned)elapsed);
  codec_set_dac_mute_(true);
  codec_set_output_enabled_(false);
  state_ = State::kIdle;
}

void Re1Audio::step_playing_() {
  static constexpr size_t kFrames = 256;
  int16_t buf[kFrames * 2];
  size_t to_emit = kFrames;
  size_t remaining = record_filled_ - play_pos_;
  if (remaining < to_emit) to_emit = remaining;
  if (to_emit == 0) {
    stop_playback_("end_of_buffer");
    return;
  }
  for (size_t i = 0; i < to_emit; ++i) {
    int16_t s = record_buffer_[play_pos_++];
    int32_t y = (int32_t) ((float) s * playback_gain_);
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    int16_t out = (int16_t) y;
    buf[i * 2 + 0] = out;
    buf[i * 2 + 1] = out;
  }
  size_t bytes = to_emit * 4;
  size_t written = 0;
  esp_err_t e = i2s_write(I2S_NUM_0, buf, bytes, &written, portMAX_DELAY);
  if (e != ESP_OK) {
    ESP_LOGW(TAG, "i2s_write err=0x%x", (int)e);
    stop_playback_("i2s_write_error");
    return;
  }

  uint32_t now = millis();
  if (now - last_log_ms_ >= 500) {
    last_log_ms_ = now;
    ESP_LOGI(TAG, "PLAY progress %u/%u (%.1f%%)", (unsigned)play_pos_,
             (unsigned)record_filled_,
             100.0f * play_pos_ / (float)record_filled_);
  }
}

}  // namespace re1_audio
}  // namespace esphome
