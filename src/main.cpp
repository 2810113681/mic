#include <Arduino.h>
#include <ctype.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

extern "C" {
#include "driver/gpio.h"
#include "driver/i2s.h"
}

#include "board_config.h"
#include "es8388_codec.h"

static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
static constexpr uint32_t DEBOUNCE_MS = 40;
static constexpr uint32_t MAX_RECORD_MS = 10000;
static constexpr uint32_t LOG_STATS_INTERVAL_MS = 500;
static constexpr size_t SERIAL_CMD_MAX_LEN = 96;
static constexpr bool USE_FIXED_NETLIST_CONFIG_FIRST = true;
static constexpr bool FORCE_NETLIST_FIXED_ONLY = true;
static constexpr bool RUN_STARTUP_BYPASS_TEST = true;
static constexpr uint32_t STARTUP_BYPASS_PRE_DELAY_MS = 1200;
static constexpr uint32_t STARTUP_BYPASS_MS = 2500;
static constexpr uint32_t CAPTURE_VERIFY_MS = 220;
static constexpr uint32_t CAPTURE_VERIFY_RETRY_MS = 260;
static constexpr bool PROBE_ENABLE_DIN_DOUT_SWAP = false;
static constexpr bool PROBE_ENABLE_FULL_PERMUTATION = false;
static constexpr size_t I2S_FRAME_SAMPLES = 256;  // stereo frames per chunk
static constexpr size_t I2S_MAX_BYTES_PER_SAMPLE = 4;
static constexpr size_t I2S_MAX_CHUNK_BYTES =
    I2S_FRAME_SAMPLES * 2 * I2S_MAX_BYTES_PER_SAMPLE;  // stereo max-chunk raw bytes
static constexpr size_t MAX_MONO_SAMPLES =
    (size_t)AUDIO_SAMPLE_RATE * (MAX_RECORD_MS / 1000);

#define LOGI(fmt, ...) Serial.printf("[%10lu][INFO ] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[%10lu][WARN ] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)
#define LOGE(fmt, ...) Serial.printf("[%10lu][ERROR] " fmt "\n", (unsigned long)millis(), ##__VA_ARGS__)

static inline int32_t abs_s16(int16_t v) {
  return (v == INT16_MIN) ? 32768 : (v < 0 ? -v : v);
}

enum class AudioState : uint8_t {
  kIdle = 0,
  kRecording,
  kPlaying,
};

static const char *state_name(AudioState s) {
  switch (s) {
    case AudioState::kIdle:
      return "IDLE";
    case AudioState::kRecording:
      return "RECORDING";
    case AudioState::kPlaying:
      return "PLAYING";
    default:
      return "UNKNOWN";
  }
}

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

// Start from the most stable tested transport path, then fallback probing will adjust if needed.
// Default to ESP32-as-I2S-master / codec-as-slave. The codec-master profile (4)
// was tested and the ES8388 silently produced no BCLK/LRCK on this hardware
// (every i2s_read timed out), so we keep the slave profile as the working baseline.
static constexpr size_t FIXED_PROFILE_INDEX = 0;  // I2S16_CODEC_SLAVE
// Netlist confirms MIC1/MIC2 are wired DIFFERENTIALLY to LINPUT1+LINPUT2 /
// RINPUT1+RINPUT2 (R20/R21/R22/R23 are 0R, MIC1 -> RIN_P+RIN_N, MIC2 ->
// LIN_P+LIN_N). MICBIAS pin (U1.25) is NC; bias comes from VCC_3V3 via
// 3x1kOhm divider chains. So differential LIN1/RIN1 is the correct ADC route
// for this board, not single-ended.
static constexpr AdcRoute FIXED_ADC_ROUTE = AdcRoute::DIFF_LIN1_RIN1;

struct I2sPinMap {
  const char *name;
  int mclk;
  int bclk;
  int ws;
  int dout;
  int din;

  I2sPinMap()
      : name("UNSET"),
        mclk(I2S_PIN_NO_CHANGE),
        bclk(I2S_PIN_NO_CHANGE),
        ws(I2S_PIN_NO_CHANGE),
        dout(I2S_PIN_NO_CHANGE),
        din(I2S_PIN_NO_CHANGE) {}

  I2sPinMap(const char *n, int m, int b, int w, int o, int i)
      : name(n), mclk(m), bclk(b), ws(w), dout(o), din(i) {}
};

enum class SlotExtractMode : uint8_t {
  kDirect16 = 0,
  kFrom32High16,
  kFrom32Low16,
};

struct DigitalProfile {
  const char *name;
  bool esp_i2s_master;
  i2s_comm_format_t comm_format;
  i2s_bits_per_sample_t i2s_bits;
  SlotExtractMode rx_slot_mode;
  SlotExtractMode tx_slot_mode;
  Es8388DigitalConfig codec;

  DigitalProfile(const char *n = "UNSET", bool esp_master = true,
                 i2s_comm_format_t comm = I2S_COMM_FORMAT_STAND_I2S,
                 i2s_bits_per_sample_t bits = I2S_BITS_PER_SAMPLE_16BIT,
                 SlotExtractMode rx_mode = SlotExtractMode::kDirect16,
                 SlotExtractMode tx_mode = SlotExtractMode::kDirect16,
                 const Es8388DigitalConfig &cfg = Es8388DigitalConfig())
      : name(n),
        esp_i2s_master(esp_master),
        comm_format(comm),
        i2s_bits(bits),
        rx_slot_mode(rx_mode),
        tx_slot_mode(tx_mode),
        codec(cfg) {}
};

static const char *slot_mode_name(SlotExtractMode m) {
  switch (m) {
    case SlotExtractMode::kDirect16:
      return "DIRECT16";
    case SlotExtractMode::kFrom32High16:
      return "FROM32_HIGH16";
    case SlotExtractMode::kFrom32Low16:
      return "FROM32_LOW16";
    default:
      return "UNKNOWN";
  }
}

static uint32_t bits_to_u32(i2s_bits_per_sample_t bits) {
  switch (bits) {
    case I2S_BITS_PER_SAMPLE_8BIT:
      return 8;
    case I2S_BITS_PER_SAMPLE_16BIT:
      return 16;
    case I2S_BITS_PER_SAMPLE_24BIT:
      return 24;
    case I2S_BITS_PER_SAMPLE_32BIT:
      return 32;
    default:
      return 16;
  }
}

static size_t bytes_per_sample(i2s_bits_per_sample_t bits) { return (bits_to_u32(bits) > 16) ? 4 : 2; }

static const DigitalProfile kDigitalProfiles[] = {
    {
        "I2S16_CODEC_SLAVE",
        true,
        I2S_COMM_FORMAT_STAND_I2S,
        I2S_BITS_PER_SAMPLE_16BIT,
        SlotExtractMode::kDirect16,
        SlotExtractMode::kDirect16,
        // ES8388 ADCCONTROL4 (reg 0x0C):
        //   bit[3:2] WL_ADC : 11 = 16-bit
        //   bit[1:0] SFI_ADC: 00 = I2S Philips, 01 = Left-Just, 10 = Right-Just, 11 = DSP
        // ESP I2S uses Philips standard, so we MUST use 00 here. The previous
        // value 0x0D (= 16-bit + Left-Just) caused a 1-BCLK offset between
        // codec and ESP which manifested as loud buzz on playback.
        {"I2S16_CODEC_SLAVE", 0x00, 0x18, 0x0C},
    },
    {
        "I2S32H_CODEC_SLAVE",
        true,
        I2S_COMM_FORMAT_STAND_I2S,
        I2S_BITS_PER_SAMPLE_32BIT,
        SlotExtractMode::kFrom32High16,
        SlotExtractMode::kFrom32High16,
        {"I2S32H_CODEC_SLAVE", 0x00, 0x20, 0x10},
    },
    {
        "I2S32L_CODEC_SLAVE",
        true,
        I2S_COMM_FORMAT_STAND_I2S,
        I2S_BITS_PER_SAMPLE_32BIT,
        SlotExtractMode::kFrom32Low16,
        SlotExtractMode::kFrom32Low16,
        {"I2S32L_CODEC_SLAVE", 0x00, 0x20, 0x10},
    },
    {
        "MSB16_CODEC_SLAVE",
        true,
        I2S_COMM_FORMAT_STAND_MSB,
        I2S_BITS_PER_SAMPLE_16BIT,
        SlotExtractMode::kDirect16,
        SlotExtractMode::kDirect16,
        {"MSB16_CODEC_SLAVE", 0x00, 0x1A, 0x0D},
    },
    {
        "I2S16_CODEC_MASTER",
        false,
        I2S_COMM_FORMAT_STAND_I2S,
        I2S_BITS_PER_SAMPLE_16BIT,
        SlotExtractMode::kDirect16,
        SlotExtractMode::kDirect16,
        {"I2S16_CODEC_MASTER", 0x80, 0x18, 0x0C},
    },
};

static constexpr size_t DIGITAL_PROFILE_COUNT =
    sizeof(kDigitalProfiles) / sizeof(kDigitalProfiles[0]);

struct AudioStats {
  int16_t min_l = INT16_MAX;
  int16_t max_l = INT16_MIN;
  int16_t min_r = INT16_MAX;
  int16_t max_r = INT16_MIN;
  int16_t min_m = INT16_MAX;
  int16_t max_m = INT16_MIN;

  uint64_t sum_abs_m = 0;
  uint64_t sum_sq_m = 0;
  int64_t sum_m = 0;
  uint64_t sum_abs_lr_diff = 0;
  int64_t sum_lr_mul = 0;

  uint32_t frame_count = 0;
  uint32_t clip_count = 0;
  uint32_t near_zero_count = 0;
  uint32_t zero_cross_count = 0;
  uint32_t large_delta_count = 0;
  uint64_t sum_abs_delta_m = 0;
  uint64_t sum_sq_delta_m = 0;
  uint32_t delta_count = 0;
  int16_t prev_m = 0;
  bool has_prev_m = false;

  void reset() {
    min_l = INT16_MAX;
    max_l = INT16_MIN;
    min_r = INT16_MAX;
    max_r = INT16_MIN;
    min_m = INT16_MAX;
    max_m = INT16_MIN;
    sum_abs_m = 0;
    sum_sq_m = 0;
    sum_m = 0;
    sum_abs_lr_diff = 0;
    sum_lr_mul = 0;
    frame_count = 0;
    clip_count = 0;
    near_zero_count = 0;
    zero_cross_count = 0;
    large_delta_count = 0;
    sum_abs_delta_m = 0;
    sum_sq_delta_m = 0;
    delta_count = 0;
    prev_m = 0;
    has_prev_m = false;
  }

  void add_frame(int16_t l, int16_t r, int16_t m) {
    if (l < min_l) min_l = l;
    if (l > max_l) max_l = l;
    if (r < min_r) min_r = r;
    if (r > max_r) max_r = r;
    if (m < min_m) min_m = m;
    if (m > max_m) max_m = m;

    int32_t am = abs_s16(m);
    sum_abs_m += (uint32_t)am;
    sum_sq_m += (uint64_t)((int32_t)m * (int32_t)m);
    sum_m += m;
    sum_abs_lr_diff += (uint32_t)abs((int32_t)l - (int32_t)r);
    sum_lr_mul += (int64_t)l * (int64_t)r;
    frame_count++;

    if (am >= 32760) clip_count++;
    if (am <= 8) near_zero_count++;

    if (has_prev_m) {
      int32_t delta = (int32_t)m - (int32_t)prev_m;
      int32_t abs_delta = (delta < 0) ? -delta : delta;
      sum_abs_delta_m += (uint32_t)abs_delta;
      sum_sq_delta_m += (uint64_t)((int64_t)delta * (int64_t)delta);
      delta_count++;

      bool prev_nonneg = (prev_m >= 0);
      bool now_nonneg = (m >= 0);
      if (prev_nonneg != now_nonneg) {
        zero_cross_count++;
      }
      if (abs_delta >= 6000) {
        large_delta_count++;
      }
    }

    prev_m = m;
    has_prev_m = true;
  }

  int32_t mono_peak() const {
    if (frame_count == 0) return 0;
    int32_t p1 = abs_s16(min_m);
    int32_t p2 = abs_s16(max_m);
    return (p1 > p2) ? p1 : p2;
  }

  float mono_rms() const {
    if (frame_count == 0) return 0.0f;
    return sqrtf((float)sum_sq_m / (float)frame_count);
  }

  float mono_mean() const {
    if (frame_count == 0) return 0.0f;
    return (float)sum_m / (float)frame_count;
  }

  float silence_ratio_percent() const {
    if (frame_count == 0) return 0.0f;
    return 100.0f * (float)near_zero_count / (float)frame_count;
  }

  float clip_ratio_percent() const {
    if (frame_count == 0) return 0.0f;
    return 100.0f * (float)clip_count / (float)frame_count;
  }

  float avg_abs_lr_diff() const {
    if (frame_count == 0) return 0.0f;
    return (float)sum_abs_lr_diff / (float)frame_count;
  }

  float zero_cross_ratio_percent() const {
    if (delta_count == 0) return 0.0f;
    return 100.0f * (float)zero_cross_count / (float)delta_count;
  }

  float delta_rms() const {
    if (delta_count == 0) return 0.0f;
    return sqrtf((float)sum_sq_delta_m / (float)delta_count);
  }

  float delta_avg_abs() const {
    if (delta_count == 0) return 0.0f;
    return (float)sum_abs_delta_m / (float)delta_count;
  }

  float large_delta_ratio_percent() const {
    if (delta_count == 0) return 0.0f;
    return 100.0f * (float)large_delta_count / (float)delta_count;
  }

  float crest_factor() const {
    float rms = mono_rms();
    if (rms <= 0.0f) return 0.0f;
    return (float)mono_peak() / rms;
  }

  float activity_ratio_percent() const {
    float silence = silence_ratio_percent();
    if (silence >= 100.0f) return 0.0f;
    if (silence <= 0.0f) return 100.0f;
    return 100.0f - silence;
  }
};

struct I2sIoStats {
  uint32_t read_calls = 0;
  uint32_t read_ok = 0;
  uint32_t read_timeout = 0;
  uint32_t read_err = 0;
  uint64_t read_bytes = 0;

  uint32_t write_calls = 0;
  uint32_t write_ok = 0;
  uint32_t write_partial = 0;
  uint32_t write_timeout = 0;
  uint32_t write_err = 0;
  uint64_t write_bytes = 0;

  void reset() {
    read_calls = 0;
    read_ok = 0;
    read_timeout = 0;
    read_err = 0;
    read_bytes = 0;
    write_calls = 0;
    write_ok = 0;
    write_partial = 0;
    write_timeout = 0;
    write_err = 0;
    write_bytes = 0;
  }
};

alignas(16) static uint8_t g_i2s_rx_chunk[I2S_MAX_CHUNK_BYTES];
alignas(16) static uint8_t g_i2s_tx_chunk[I2S_MAX_CHUNK_BYTES];

static int16_t *g_record_mono = nullptr;
static size_t g_recorded_samples = 0;
static size_t g_record_capacity_samples = 0;
static uint32_t g_record_start_ms = 0;
static size_t g_play_pos = 0;

static AudioState g_state = AudioState::kIdle;
static Es8388Codec g_codec(ES8388_I2C_ADDR);
static AdcRoute g_adc_route = AdcRoute::DIFF_LIN1_RIN1;
static size_t g_digital_profile_index = 0;
static bool g_i2s_driver_installed = false;
static I2sPinMap g_i2s_pins = {"NETLIST_DEFAULT", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS,
                               PIN_I2S_DOUT, PIN_I2S_DIN};
static bool g_capture_path_ok = false;
static uint32_t g_capture_verify_fail_count = 0;

static AudioStats g_rec_stats_total;
static AudioStats g_rec_stats_window;
static I2sIoStats g_rec_i2s_total;
static I2sIoStats g_rec_i2s_window;

static AudioStats g_play_stats_total;
static AudioStats g_play_stats_window;
static I2sIoStats g_play_i2s_total;
static I2sIoStats g_play_i2s_window;

static uint32_t g_rec_last_log_ms = 0;
static uint32_t g_play_last_log_ms = 0;
static uint32_t g_last_read_error_log_ms = 0;
static uint32_t g_last_write_error_log_ms = 0;

static bool g_rec_first_chunk_dumped = false;
static bool g_play_first_chunk_dumped = false;

static int32_t g_dc_q8 = 0;
static bool g_dc_ready = false;
static constexpr uint8_t DC_BLOCK_ALPHA_SHIFT = 9;  // 1/512 low-pass for DC estimate
static bool g_raw_12bit_like = false;
static constexpr uint8_t RAW_12BIT_GAIN_SHIFT = 3;  // x8 for 0..4095 style capture

static const DigitalProfile &active_profile() {
  if (g_digital_profile_index >= DIGITAL_PROFILE_COUNT) {
    g_digital_profile_index = 0;
  }
  return kDigitalProfiles[g_digital_profile_index];
}

static size_t active_frame_bytes() { return bytes_per_sample(active_profile().i2s_bits) * 2; }

static void decode_stereo_frame(const uint8_t *frame_ptr, const DigitalProfile &profile, int16_t &l,
                                int16_t &r) {
  if (bytes_per_sample(profile.i2s_bits) == 2) {
    memcpy(&l, frame_ptr, sizeof(int16_t));
    memcpy(&r, frame_ptr + sizeof(int16_t), sizeof(int16_t));
    return;
  }

  int32_t l32 = 0;
  int32_t r32 = 0;
  memcpy(&l32, frame_ptr, sizeof(int32_t));
  memcpy(&r32, frame_ptr + sizeof(int32_t), sizeof(int32_t));

  if (profile.rx_slot_mode == SlotExtractMode::kFrom32Low16) {
    l = (int16_t)(l32 & 0xFFFF);
    r = (int16_t)(r32 & 0xFFFF);
  } else {
    l = (int16_t)(l32 >> 16);
    r = (int16_t)(r32 >> 16);
  }
}

static void encode_stereo_frame(uint8_t *frame_ptr, const DigitalProfile &profile, int16_t s) {
  if (bytes_per_sample(profile.i2s_bits) == 2) {
    memcpy(frame_ptr, &s, sizeof(int16_t));
    memcpy(frame_ptr + sizeof(int16_t), &s, sizeof(int16_t));
    return;
  }

  int32_t l32 = 0;
  int32_t r32 = 0;
  if (profile.tx_slot_mode == SlotExtractMode::kFrom32Low16) {
    l32 = (uint16_t)s;
    r32 = (uint16_t)s;
  } else {
    l32 = ((int32_t)s << 16);
    r32 = ((int32_t)s << 16);
  }

  memcpy(frame_ptr, &l32, sizeof(int32_t));
  memcpy(frame_ptr + sizeof(int32_t), &r32, sizeof(int32_t));
}

static int16_t dc_block_process(int32_t raw_sample) {
  const int32_t raw_q8 = raw_sample << 8;
  if (!g_dc_ready) {
    g_dc_q8 = raw_q8;
    g_dc_ready = true;
  } else {
    g_dc_q8 += ((raw_q8 - g_dc_q8) >> DC_BLOCK_ALPHA_SHIFT);
  }

  int32_t centered = (raw_q8 - g_dc_q8) >> 8;
  if (centered > 32767) centered = 32767;
  if (centered < -32768) centered = -32768;
  return (int16_t)centered;
}

static int16_t apply_mono_gain(int16_t s) {
  if (!g_raw_12bit_like) {
    return s;
  }

  int32_t v = ((int32_t)s) << RAW_12BIT_GAIN_SHIFT;
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

struct DebouncedButton {
  int pin = -1;
  bool stable = true;
  bool last_read = true;
  uint32_t last_change_ms = 0;

  void begin(int gpio) {
    pin = gpio;
    pinMode(pin, INPUT);  // external pull-up + key to GND from netlist
    stable = digitalRead(pin);
    last_read = stable;
    last_change_ms = millis();
  }

  bool fell() {
    bool now_read = digitalRead(pin);
    uint32_t now_ms = millis();

    if (now_read != last_read) {
      last_read = now_read;
      last_change_ms = now_ms;
    }

    if ((now_ms - last_change_ms) >= DEBOUNCE_MS && now_read != stable) {
      bool prev = stable;
      stable = now_read;
      return (prev == HIGH && stable == LOW);
    }
    return false;
  }
};

static DebouncedButton g_btn_record;
static DebouncedButton g_btn_play;
static char g_serial_cmd_buf[SERIAL_CMD_MAX_LEN];
static size_t g_serial_cmd_len = 0;
static bool g_serial_cmd_overflow = false;

static bool apply_i2s_pin_map(const I2sPinMap &pins, const char *reason);
static bool verify_digital_capture_once(const char *reason, uint32_t verify_ms,
                                        AudioStats *audio_out, I2sIoStats *io_out);
static bool apply_fixed_netlist_config(const char *reason, bool verify_capture);
static void service_serial_commands();

static void set_state(AudioState next, const char *reason) {
  if (g_state != next) {
    LOGI("[STATE] %s -> %s, reason=%s", state_name(g_state), state_name(next), reason);
    g_state = next;
  }
}

static void codec_set_dac_mute(bool mute, const char *reason) {
  bool ok = g_codec.set_dac_mute(mute);
  LOGI("[CODEC] set_dac_mute(%s) reason=%s result=%s", mute ? "true" : "false", reason,
       ok ? "ok" : "fail");
}

static void codec_set_output(bool enable, const char *reason) {
  bool ok = g_codec.set_output_enabled(enable);
  LOGI("[CODEC] set_output_enabled(%s) reason=%s result=%s", enable ? "true" : "false",
       reason, ok ? "ok" : "fail");
}

static void dump_stereo_frames(const uint8_t *buf, size_t frames, size_t max_frames,
                               const char *tag) {
  const DigitalProfile &profile = active_profile();
  const size_t frame_bytes = bytes_per_sample(profile.i2s_bits) * 2;
  size_t n = (frames < max_frames) ? frames : max_frames;
  Serial.printf("[%10lu][DUMP ] %s frames=%u (L,R):", (unsigned long)millis(), tag,
                (unsigned)n);
  for (size_t i = 0; i < n; ++i) {
    int16_t l = 0;
    int16_t r = 0;
    decode_stereo_frame(buf + i * frame_bytes, profile, l, r);
    Serial.printf(" (%d,%d)", l, r);
  }
  Serial.println();
}

static void dump_mono_samples(const int16_t *buf, size_t samples, size_t max_samples,
                              const char *tag) {
  size_t n = (samples < max_samples) ? samples : max_samples;
  Serial.printf("[%10lu][DUMP ] %s samples=%u:", (unsigned long)millis(), tag, (unsigned)n);
  for (size_t i = 0; i < n; ++i) {
    Serial.printf(" %d", buf[i]);
  }
  Serial.println();
}

static void print_audio_stats(const char *prefix, const AudioStats &s) {
  if (s.frame_count == 0) {
    LOGI("%s frames=0", prefix);
    return;
  }

  LOGI(
      "%s frames=%u L[min=%d max=%d] R[min=%d max=%d] M[min=%d max=%d peak=%ld rms=%.1f "
      "mean=%.1f] silence=%.2f%% activity=%.2f%% clip=%.2f%% lr_diff_avg=%.1f crest=%.2f "
      "zcr=%.2f%% drms=%.1f dabs=%.1f djump=%.2f%%",
      prefix, (unsigned)s.frame_count, s.min_l, s.max_l, s.min_r, s.max_r, s.min_m, s.max_m,
      (long)s.mono_peak(), s.mono_rms(), s.mono_mean(), s.silence_ratio_percent(),
      s.activity_ratio_percent(), s.clip_ratio_percent(), s.avg_abs_lr_diff(), s.crest_factor(),
      s.zero_cross_ratio_percent(), s.delta_rms(), s.delta_avg_abs(), s.large_delta_ratio_percent());
}

static void print_i2s_stats(const char *prefix, const I2sIoStats &io) {
  LOGI(
      "%s read{calls=%u ok=%u timeout=%u err=%u bytes=%llu} write{calls=%u ok=%u "
      "partial=%u timeout=%u err=%u bytes=%llu}",
      prefix, (unsigned)io.read_calls, (unsigned)io.read_ok, (unsigned)io.read_timeout,
      (unsigned)io.read_err, (unsigned long long)io.read_bytes, (unsigned)io.write_calls,
      (unsigned)io.write_ok, (unsigned)io.write_partial, (unsigned)io.write_timeout,
      (unsigned)io.write_err, (unsigned long long)io.write_bytes);
}

static void diagnose_recording_result() {
  if (g_rec_stats_total.frame_count == 0) {
    LOGW("[REC][DIAG] no frames captured");
    return;
  }

  const float rms = g_rec_stats_total.mono_rms();
  const int32_t peak = g_rec_stats_total.mono_peak();
  const float silence = g_rec_stats_total.silence_ratio_percent();
  const float clip = g_rec_stats_total.clip_ratio_percent();
  const float mean = g_rec_stats_total.mono_mean();
  const float activity = g_rec_stats_total.activity_ratio_percent();
  const float crest = g_rec_stats_total.crest_factor();
  const float zcr = g_rec_stats_total.zero_cross_ratio_percent();
  const float drms = g_rec_stats_total.delta_rms();
  const float jump = g_rec_stats_total.large_delta_ratio_percent();
  const int32_t l_span = (int32_t)g_rec_stats_total.max_l - (int32_t)g_rec_stats_total.min_l;
  const int32_t r_span = (int32_t)g_rec_stats_total.max_r - (int32_t)g_rec_stats_total.min_r;

  LOGI("[REC][DIAG] features activity=%.2f%% crest=%.2f zcr=%.2f%% drms=%.1f jump=%.2f%%",
       activity, crest, zcr, drms, jump);

  if (peak < 50 || rms < 10.0f) {
    LOGW("[REC][DIAG] very low signal. likely MIC/input issue (bias/gain/route/wiring)");
  }
  if (silence > 98.0f) {
    LOGW("[REC][DIAG] near-silence dominant. likely MIC path not receiving valid audio");
  }
  if (clip > 2.0f) {
    LOGW("[REC][DIAG] clipping high (%.2f%%). gain may be too high or input saturated", clip);
  }
  if (fabsf(mean) > 1500.0f) {
    LOGW("[REC][DIAG] strong DC offset (mean=%.1f). possible analog bias issue", mean);
  }
  if (l_span <= 4 && r_span > 200) {
    LOGW("[REC][DIAG] left channel is near-constant while right has activity. possible route/slot mismatch");
  } else if (r_span <= 4 && l_span > 200) {
    LOGW("[REC][DIAG] right channel is near-constant while left has activity. possible route/slot mismatch");
  }
  if (silence > 80.0f && crest > 5.0f &&
      (zcr > 40.0f || jump > 5.0f || drms > (rms * 2.2f))) {
    LOGW("[REC][DIAG] waveform looks tone/interference-like (possible clock/EMI coupling)");
  }

  if (peak > 300 && rms > 80.0f && clip < 1.0f && silence < 90.0f) {
    LOGI("[REC][DIAG] captured waveform looks valid. if playback is still noise, focus on output/I2S/DAC path");
  }
}

struct AudioProbeResult {
  bool valid = false;
  I2sPinMap pins = {};
  AdcRoute route = AdcRoute::DIFF_LIN1_RIN1;
  AudioStats audio = {};
  I2sIoStats io = {};
  float score = -1.0f;
  float score_base = 0.0f;
  float score_penalty = 1.0f;
  bool tone_like = false;
};

struct ProbeQuality {
  float score = -1.0f;
  float base_score = 0.0f;
  float penalty = 1.0f;
  bool tone_like = false;
};

static ProbeQuality evaluate_probe_quality(const AudioStats &audio) {
  ProbeQuality q = {};
  if (audio.frame_count == 0) {
    q.base_score = -3.0f;
    q.score = -3.0f;
    return q;
  }
  const float peak = (float)audio.mono_peak();
  const float rms = audio.mono_rms();
  const float silence = audio.silence_ratio_percent();
  const float activity = fmaxf(0.0f, 100.0f - silence) / 100.0f;
  const float crest = audio.crest_factor();
  const float zcr = audio.zero_cross_ratio_percent();
  const float jump = audio.large_delta_ratio_percent();
  const float drms = audio.delta_rms();
  const float lr_diff = audio.avg_abs_lr_diff();
  const int32_t l_span = (int32_t)audio.max_l - (int32_t)audio.min_l;
  const int32_t r_span = (int32_t)audio.max_r - (int32_t)audio.min_r;
  const int32_t m_span = (int32_t)audio.max_m - (int32_t)audio.min_m;

  q.base_score = (rms * activity * activity) + (0.02f * peak * activity);
  if (peak <= 1.0f && rms < 1.0f) {
    q.base_score = -1.0f;
  }
  if (peak <= 1.0f && rms <= 1.2f && l_span <= 1 && r_span <= 1 && m_span <= 1) {
    q.base_score = -2.0f;
  }

  if (silence > 80.0f && crest > 5.0f) {
    q.penalty *= 0.35f;
    q.tone_like = true;
  }
  if (zcr > 45.0f && silence > 70.0f) {
    q.penalty *= 0.50f;
    q.tone_like = true;
  }
  if (jump > 8.0f && silence > 75.0f) {
    q.penalty *= 0.60f;
    q.tone_like = true;
  }
  if (drms > (rms * 2.2f) && silence > 70.0f) {
    q.penalty *= 0.60f;
    q.tone_like = true;
  }
  if (jump > 20.0f && zcr > 10.0f) {
    q.penalty *= 0.25f;
    q.tone_like = true;
  }
  if (lr_diff < 2.0f && rms > 500.0f && jump > 8.0f) {
    q.penalty *= 0.20f;
    q.tone_like = true;
  }
  if (activity < 0.02f) {
    q.penalty *= 0.10f;
  }

  q.score = q.base_score * q.penalty;
  return q;
}

static bool reinstall_i2s_for_active_profile(const char *reason) {
  const DigitalProfile &profile = active_profile();

  if (g_i2s_driver_installed) {
    i2s_stop(I2S_PORT);
    i2s_driver_uninstall(I2S_PORT);
    g_i2s_driver_installed = false;
    delay(2);
  }

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)((profile.esp_i2s_master ? I2S_MODE_MASTER : I2S_MODE_SLAVE) |
                          I2S_MODE_TX | I2S_MODE_RX);
  cfg.sample_rate = AUDIO_SAMPLE_RATE;
  cfg.bits_per_sample = profile.i2s_bits;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = profile.comm_format;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = AUDIO_SAMPLE_RATE * 256;

  LOGI(
      "[I2S] installing driver: reason=%s profile=%s esp_role=%s codec_role=%s rate=%u bits=%u comm=0x%X rx_mode=%s "
      "tx_mode=%s dma_count=%u dma_len=%u mclk=%u",
      reason, profile.name, profile.esp_i2s_master ? "MASTER" : "SLAVE",
      (profile.codec.master_mode_reg & 0x80) ? "MASTER" : "SLAVE", (unsigned)AUDIO_SAMPLE_RATE,
      (unsigned)bits_to_u32(profile.i2s_bits),
      (unsigned)profile.comm_format, slot_mode_name(profile.rx_slot_mode),
      slot_mode_name(profile.tx_slot_mode), (unsigned)cfg.dma_buf_count,
      (unsigned)cfg.dma_buf_len, (unsigned)cfg.fixed_mclk);

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err != ESP_OK) {
    LOGE("[I2S] driver_install failed profile=%s err=%d", profile.name, (int)err);
    return false;
  }
  g_i2s_driver_installed = true;

  if (!apply_i2s_pin_map(g_i2s_pins, "driver_profile_apply_pin_map")) {
    return false;
  }

  if (profile.esp_i2s_master) {
    err = i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, profile.i2s_bits, I2S_CHANNEL_STEREO);
    if (err != ESP_OK) {
      LOGE("[I2S] set_clk failed profile=%s err=%d", profile.name, (int)err);
      return false;
    }
  } else {
    LOGI("[I2S] set_clk skipped: ESP in SLAVE mode, using external BCLK/LRCK from codec");
  }

  i2s_zero_dma_buffer(I2S_PORT);
  err = i2s_start(I2S_PORT);
  if (err != ESP_OK) {
    LOGE("[I2S] start failed profile=%s err=%d", profile.name, (int)err);
    return false;
  }

  LOGI("[I2S] ready profile=%s", profile.name);
  return true;
}

static bool apply_digital_profile(size_t profile_index, const char *reason) {
  if (profile_index >= DIGITAL_PROFILE_COUNT) {
    LOGE("[DPROBE] invalid profile index=%u", (unsigned)profile_index);
    return false;
  }

  g_digital_profile_index = profile_index;
  const DigitalProfile &profile = active_profile();
  LOGI(
      "[DPROBE] apply profile reason=%s idx=%u name=%s esp_role=%s codec_role=%s bits=%u comm=0x%X codec_master_reg=0x%02X "
      "codec_dac_fmt=0x%02X codec_adc_fmt=0x%02X",
      reason, (unsigned)profile_index, profile.name, profile.esp_i2s_master ? "MASTER" : "SLAVE",
      (profile.codec.master_mode_reg & 0x80) ? "MASTER" : "SLAVE",
      (unsigned)bits_to_u32(profile.i2s_bits),
      (unsigned)profile.comm_format, profile.codec.master_mode_reg, profile.codec.dac_format_reg,
      profile.codec.adc_format_reg);
  return reinstall_i2s_for_active_profile(reason);
}

static bool apply_i2s_pin_map(const I2sPinMap &pins, const char *reason) {
  i2s_pin_config_t cfg = {};
  cfg.mck_io_num = pins.mclk;
  cfg.bck_io_num = pins.bclk;
  cfg.ws_io_num = pins.ws;
  cfg.data_out_num = pins.dout;
  cfg.data_in_num = pins.din;

  LOGI("[I2S] apply pin map reason=%s name=%s mclk=%d bclk=%d ws=%d dout=%d din=%d", reason,
       pins.name, pins.mclk, pins.bclk, pins.ws, pins.dout, pins.din);

  esp_err_t err = i2s_set_pin(I2S_PORT, &cfg);
  if (err != ESP_OK) {
    LOGE("[I2S] set_pin failed for map=%s err=%d", pins.name, (int)err);
    return false;
  }

  g_i2s_pins = pins;
  return true;
}

static void capture_probe_window(uint32_t probe_ms, AudioStats &audio, I2sIoStats &io) {
  const DigitalProfile &profile = active_profile();
  const size_t frame_bytes = bytes_per_sample(profile.i2s_bits) * 2;

  audio.reset();
  io.reset();

  const uint32_t start_ms = millis();
  while ((millis() - start_ms) < probe_ms) {
    size_t bytes_read = 0;
    io.read_calls++;

    esp_err_t err =
        i2s_read(I2S_PORT, g_i2s_rx_chunk, sizeof(g_i2s_rx_chunk), &bytes_read, pdMS_TO_TICKS(20));
    if (err != ESP_OK || bytes_read == 0) {
      if (err == ESP_ERR_TIMEOUT || bytes_read == 0) {
        io.read_timeout++;
      } else {
        io.read_err++;
      }
      continue;
    }

    io.read_ok++;
    io.read_bytes += bytes_read;

    const size_t in_frames = bytes_read / frame_bytes;
    for (size_t i = 0; i < in_frames; ++i) {
      int16_t l = 0;
      int16_t r = 0;
      decode_stereo_frame(g_i2s_rx_chunk + i * frame_bytes, profile, l, r);
      int16_t m = (int16_t)(((int32_t)l + (int32_t)r) / 2);
      audio.add_frame(l, r, m);
    }
  }
}

static bool auto_probe_input_path() {
  const DigitalProfile &profile = active_profile();
  const AdcRoute routes[] = {
      AdcRoute::DIFF_LIN1_RIN1,
      AdcRoute::DIFF_LIN2_RIN2,
      AdcRoute::SE_LIN1_RIN1,
      AdcRoute::SE_LIN2_RIN2,
  };
  static constexpr uint32_t PROBE_MS_FIXED = 260;
  static constexpr uint32_t PROBE_MS_FALLBACK = 120;

  AudioProbeResult best = {};
  float best_score = -1e30f;
  float best_rms = -1.0f;
  int32_t best_peak = -1;
  size_t tested = 0;

  auto consider_combo = [&](const I2sPinMap &map, AdcRoute route, uint32_t probe_ms) {
    tested++;
    LOGI("[PROBE] testing profile=%s map=%s route=%s", profile.name, map.name, route_name(route));
    if (!g_codec.init(route, profile.codec)) {
      LOGW("[PROBE] codec init failed profile=%s map=%s route=%s", profile.name, map.name,
           route_name(route));
      return;
    }
    codec_set_dac_mute(true, "probe_combo");
    codec_set_output(false, "probe_combo");

    i2s_zero_dma_buffer(I2S_PORT);
    delay(20);

    AudioStats audio;
    I2sIoStats io;
    capture_probe_window(probe_ms, audio, io);

    const int32_t peak = audio.mono_peak();
    const float rms = audio.mono_rms();
    const float silence = audio.silence_ratio_percent();
    const ProbeQuality quality = evaluate_probe_quality(audio);
    LOGI(
        "[PROBE] result [%u] profile=%s map=%s route=%s peak=%ld rms=%.1f silence=%.2f%% "
        "score=%.2f base=%.2f penalty=%.2f lr_diff=%.1f crest=%.2f zcr=%.2f%% jump=%.2f%% tone_like=%s "
        "frames=%u",
        (unsigned)tested, profile.name, map.name, route_name(route), (long)peak, rms, silence,
        quality.score,
        quality.base_score, quality.penalty, audio.avg_abs_lr_diff(), audio.crest_factor(),
        audio.zero_cross_ratio_percent(), audio.large_delta_ratio_percent(),
        quality.tone_like ? "yes" : "no", (unsigned)audio.frame_count);
    print_i2s_stats("[PROBE][I2S]", io);

    bool better = false;
    if (!best.valid) {
      better = true;
    } else if (quality.score > (best_score + 0.01f)) {
      better = true;
    } else if (fabsf(quality.score - best_score) <= 0.01f && rms > (best_rms + 0.1f)) {
      better = true;
    } else if (fabsf(quality.score - best_score) <= 0.01f && fabsf(rms - best_rms) <= 0.1f &&
               peak > best_peak) {
      better = true;
    }

    if (better) {
      best.valid = true;
      best.pins = map;
      best.route = route;
      best.audio = audio;
      best.io = io;
      best.score = quality.score;
      best.score_base = quality.base_score;
      best.score_penalty = quality.penalty;
      best.tone_like = quality.tone_like;
      best_score = quality.score;
      best_peak = peak;
      best_rms = rms;
    }
  };

  auto probe_map = [&](const I2sPinMap &map, const char *reason, uint32_t probe_ms) {
    if (!apply_i2s_pin_map(map, reason)) {
      return;
    }
    i2s_zero_dma_buffer(I2S_PORT);
    delay(10);

    for (AdcRoute route : routes) {
      consider_combo(map, route, probe_ms);
    }
  };

  const I2sPinMap netlist_map = {"NETLIST_DEFAULT", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS,
                                 PIN_I2S_DOUT, PIN_I2S_DIN};
  LOGI("[PROBE] start ADC route probing on netlist-fixed I2S map profile=%s bits=%u comm=0x%X",
       profile.name, (unsigned)bits_to_u32(profile.i2s_bits), (unsigned)profile.comm_format);
  LOGI(
      "[PROBE] fixed map=%s mclk=%d bclk=%d ws=%d dout=%d din=%d routes=%u each=%u ms "
      "codec_master_reg=0x%02X adc_fmt=0x%02X dac_fmt=0x%02X",
      netlist_map.name, netlist_map.mclk, netlist_map.bclk, netlist_map.ws, netlist_map.dout,
      netlist_map.din, (unsigned)(sizeof(routes) / sizeof(routes[0])), (unsigned)PROBE_MS_FIXED,
      profile.codec.master_mode_reg, profile.codec.adc_format_reg, profile.codec.dac_format_reg);
  LOGI("[PROBE] keep speaking/tapping during probe window");
  probe_map(netlist_map, "probe_fixed_map", PROBE_MS_FIXED);

  if (!best.valid) {
    LOGE("[PROBE] failed to get any valid probe candidate");
    return false;
  }

  if (best.audio.mono_peak() <= 1) {
    LOGW("[PROBE] fixed map weak/all-zero. trying WS<->DOUT swap (DIN unchanged)");
    const I2sPinMap ws_dout_swap_map = {"SWAP_WS_DOUT_KEEP_DIN", PIN_I2S_MCLK, PIN_I2S_BCLK,
                                        PIN_I2S_DOUT, PIN_I2S_WS, PIN_I2S_DIN};
    probe_map(ws_dout_swap_map, "probe_swap_ws_dout", PROBE_MS_FIXED);

    if (PROBE_ENABLE_DIN_DOUT_SWAP) {
      LOGW("[PROBE] fixed map weak/all-zero. trying DIN<->DOUT swap based on netlist only");
      const I2sPinMap swap_map = {"SWAP_DIN_DOUT", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS,
                                  PIN_I2S_DIN, PIN_I2S_DOUT};
      probe_map(swap_map, "probe_swap_din_dout", PROBE_MS_FIXED);
    }

    if (PROBE_ENABLE_FULL_PERMUTATION && best.audio.mono_peak() <= 1) {
      static I2sPinMap pin_maps[24];
      static char pin_map_names[24][48];
      size_t pin_map_count = 0;
      const int gpio_pool[4] = {PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN};

      for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
          if (j == i) continue;
          for (int k = 0; k < 4; ++k) {
            if (k == i || k == j) continue;
            for (int m = 0; m < 4; ++m) {
              if (m == i || m == j || m == k) continue;
              snprintf(pin_map_names[pin_map_count], sizeof(pin_map_names[pin_map_count]),
                       "AUTO_B%d_W%d_O%d_I%d", gpio_pool[i], gpio_pool[j], gpio_pool[k],
                       gpio_pool[m]);
              pin_maps[pin_map_count] = I2sPinMap(pin_map_names[pin_map_count], PIN_I2S_MCLK,
                                                  gpio_pool[i], gpio_pool[j], gpio_pool[k],
                                                  gpio_pool[m]);
              pin_map_count++;
            }
          }
        }
      }

      LOGW("[PROBE] entering full permutation fallback (debug mode)");
      LOGI("[PROBE] fallback pin_maps=%u routes=%u combos=%u each=%u ms", (unsigned)pin_map_count,
           (unsigned)(sizeof(routes) / sizeof(routes[0])),
           (unsigned)(pin_map_count * (sizeof(routes) / sizeof(routes[0]))),
           (unsigned)PROBE_MS_FALLBACK);

      for (size_t map_idx = 0; map_idx < pin_map_count; ++map_idx) {
        probe_map(pin_maps[map_idx], "probe_fallback_map", PROBE_MS_FALLBACK);
      }
    }
  }

  if (!best.valid) {
    LOGE("[PROBE] failed to get any valid probe candidate after fallback");
    return false;
  }

  LOGI(
      "[PROBE] selected profile=%s map=%s route=%s bclk=%d ws=%d dout=%d din=%d peak=%ld rms=%.1f "
      "silence=%.2f%% score=%.2f base=%.2f penalty=%.2f tone_like=%s",
      profile.name, best.pins.name, route_name(best.route), best.pins.bclk, best.pins.ws,
      best.pins.dout, best.pins.din, (long)best.audio.mono_peak(), best.audio.mono_rms(),
      best.audio.silence_ratio_percent(), best.score, best.score_base, best.score_penalty,
      best.tone_like ? "yes" : "no");

  if (!apply_i2s_pin_map(best.pins, "probe_select")) {
    return false;
  }

  g_adc_route = best.route;
  if (!g_codec.init(g_adc_route, profile.codec)) {
    LOGE("[PROBE] final codec init failed profile=%s route=%s", profile.name,
         route_name(g_adc_route));
    return false;
  }
  codec_set_dac_mute(true, "probe_finalize");
  codec_set_output(false, "probe_finalize");
  i2s_zero_dma_buffer(I2S_PORT);

  if (best.audio.mono_peak() == 0) {
    LOGW("[PROBE] all candidates still peak=0. likely hardware input path issue or unsupported codec config");
  }
  if (best.tone_like) {
    LOGW("[PROBE] selected candidate still looks interference-like. keep speaking while probing or check MIC/GND/clock coupling");
  }
  g_capture_path_ok = (best.audio.mono_peak() >= 20 || best.audio.mono_rms() >= 5.0f);
  return true;
}

struct DigitalProbeCandidate {
  bool valid = false;
  size_t profile_index = 0;
  I2sPinMap pins = {};
  AdcRoute route = AdcRoute::DIFF_LIN1_RIN1;
  AudioStats verify_audio = {};
  I2sIoStats verify_io = {};
  float score = -1.0f;
  bool alive = false;
};

static bool auto_probe_digital_chain(const char *reason) {
  LOGI("[DPROBE] start reason=%s profiles=%u", reason, (unsigned)DIGITAL_PROFILE_COUNT);

  DigitalProbeCandidate best = {};

  for (size_t idx = 0; idx < DIGITAL_PROFILE_COUNT; ++idx) {
    if (!apply_digital_profile(idx, "dprobe_profile_iter")) {
      LOGW("[DPROBE] skip profile idx=%u due to apply failure", (unsigned)idx);
      continue;
    }

    const DigitalProfile &profile = active_profile();
    LOGI("[DPROBE] probing profile idx=%u name=%s", (unsigned)idx, profile.name);

    if (!auto_probe_input_path()) {
      LOGW("[DPROBE] profile=%s auto_probe_input_path failed", profile.name);
      continue;
    }

    AudioStats verify_audio;
    I2sIoStats verify_io;
    bool alive =
        verify_digital_capture_once("dprobe_post_route", CAPTURE_VERIFY_RETRY_MS, &verify_audio, &verify_io);
    ProbeQuality q = evaluate_probe_quality(verify_audio);

    float score = q.score;
    if (alive) {
      score += 100000.0f;  // hard prefer alive candidates
    }

    LOGI(
        "[DPROBE] result profile=%s alive=%s score=%.2f base=%.2f penalty=%.2f peak=%ld rms=%.2f "
        "silence=%.2f%%",
        profile.name, alive ? "yes" : "no", score, q.base_score, q.penalty,
        (long)verify_audio.mono_peak(), verify_audio.mono_rms(), verify_audio.silence_ratio_percent());

    if (!best.valid || score > best.score) {
      best.valid = true;
      best.profile_index = idx;
      best.pins = g_i2s_pins;
      best.route = g_adc_route;
      best.verify_audio = verify_audio;
      best.verify_io = verify_io;
      best.score = score;
      best.alive = alive;
    }
  }

  if (!best.valid) {
    LOGE("[DPROBE] no valid digital profile candidate");
    return false;
  }

  if (!apply_digital_profile(best.profile_index, "dprobe_select_profile")) {
    LOGE("[DPROBE] failed to apply selected profile idx=%u", (unsigned)best.profile_index);
    return false;
  }
  if (!apply_i2s_pin_map(best.pins, "dprobe_select_map")) {
    LOGE("[DPROBE] failed to apply selected pin map");
    return false;
  }

  g_adc_route = best.route;
  const DigitalProfile &selected = active_profile();
  if (!g_codec.init(g_adc_route, selected.codec)) {
    LOGE("[DPROBE] final codec init failed profile=%s route=%s", selected.name,
         route_name(g_adc_route));
    return false;
  }
  codec_set_dac_mute(true, "dprobe_finalize");
  codec_set_output(false, "dprobe_finalize");
  i2s_zero_dma_buffer(I2S_PORT);

  LOGI(
      "[DPROBE] selected profile=%s map=%s route=%s alive=%s score=%.2f peak=%ld rms=%.2f "
      "silence=%.2f%%",
      selected.name, best.pins.name, route_name(best.route), best.alive ? "yes" : "no",
      best.score, (long)best.verify_audio.mono_peak(), best.verify_audio.mono_rms(),
      best.verify_audio.silence_ratio_percent());
  g_capture_path_ok =
      best.alive || (best.verify_audio.mono_peak() >= 20 || best.verify_audio.mono_rms() >= 5.0f);

  return true;
}

static bool apply_fixed_netlist_config(const char *reason, bool verify_capture) {
  LOGI("[FIX] apply netlist-first config reason=%s profile_idx=%u route=%s", reason,
       (unsigned)FIXED_PROFILE_INDEX, route_name(FIXED_ADC_ROUTE));

  if (!apply_digital_profile(FIXED_PROFILE_INDEX, "fixed_profile")) {
    LOGE("[FIX] apply_digital_profile failed idx=%u", (unsigned)FIXED_PROFILE_INDEX);
    return false;
  }

  const I2sPinMap netlist_map = {"NETLIST_DEFAULT", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS,
                                 PIN_I2S_DOUT, PIN_I2S_DIN};
  if (!apply_i2s_pin_map(netlist_map, "fixed_netlist_map")) {
    LOGE("[FIX] apply_i2s_pin_map failed");
    return false;
  }

  g_adc_route = FIXED_ADC_ROUTE;
  const DigitalProfile &profile = active_profile();
  if (!g_codec.init(g_adc_route, profile.codec)) {
    LOGE("[FIX] codec init failed profile=%s route=%s", profile.name, route_name(g_adc_route));
    return false;
  }

  codec_set_dac_mute(true, "fixed_config");
  codec_set_output(false, "fixed_config");
  i2s_zero_dma_buffer(I2S_PORT);

  if (!verify_capture) {
    g_capture_path_ok = true;
    return true;
  }

  AudioStats verify_audio;
  I2sIoStats verify_io;
  bool alive =
      verify_digital_capture_once("fixed_config_verify", CAPTURE_VERIFY_RETRY_MS, &verify_audio, &verify_io);
  g_capture_path_ok = alive;
  LOGI("[FIX] verify result alive=%s peak=%ld rms=%.2f silence=%.2f%%",
       alive ? "yes" : "no", (long)verify_audio.mono_peak(), verify_audio.mono_rms(),
       verify_audio.silence_ratio_percent());
  return alive;
}


static bool verify_digital_capture_once(const char *reason, uint32_t verify_ms,
                                        AudioStats *audio_out = nullptr,
                                        I2sIoStats *io_out = nullptr) {
  const DigitalProfile &profile = active_profile();
  i2s_zero_dma_buffer(I2S_PORT);
  delay(20);

  AudioStats audio;
  I2sIoStats io;
  capture_probe_window(verify_ms, audio, io);

  if (audio_out) {
    *audio_out = audio;
  }
  if (io_out) {
    *io_out = io;
  }

  const bool has_frames = (audio.frame_count > 0);
  const int32_t peak = audio.mono_peak();
  const float rms = audio.mono_rms();
  const float silence = audio.silence_ratio_percent();
  const float activity = audio.activity_ratio_percent();
  const int32_t l_span = (int32_t)audio.max_l - (int32_t)audio.min_l;
  const int32_t r_span = (int32_t)audio.max_r - (int32_t)audio.min_r;
  const int32_t m_span = (int32_t)audio.max_m - (int32_t)audio.min_m;
  const bool stuck_pattern =
      has_frames && peak <= 1 && rms <= 1.2f && l_span <= 1 && r_span <= 1 && m_span <= 1;
  const bool has_meaningful_signal =
      has_frames &&
      ((peak >= 20) || (rms >= 5.0f) || (activity >= 1.0f && (l_span > 4 || r_span > 4 || m_span > 4)));
  const bool looks_alive = has_meaningful_signal && !stuck_pattern;

  LOGI(
      "[VERIFY] reason=%s profile=%s frames=%u peak=%ld rms=%.2f silence=%.2f%% activity=%.2f%% "
      "span(L,R,M)=(%ld,%ld,%ld) stuck=%s alive=%s",
      reason, profile.name, (unsigned)audio.frame_count, (long)peak, rms, silence, activity,
      (long)l_span, (long)r_span, (long)m_span, stuck_pattern ? "yes" : "no",
      looks_alive ? "yes" : "no");
  print_i2s_stats("[VERIFY][I2S]", io);
  return looks_alive;
}

static bool ensure_digital_capture_ready(const char *reason) {
  const DigitalProfile &profile = active_profile();
  LOGI("[VERIFY] ensure capture ready reason=%s profile=%s", reason, profile.name);

  if (verify_digital_capture_once(reason, CAPTURE_VERIFY_MS)) {
    g_capture_path_ok = true;
    return true;
  }

  g_capture_path_ok = false;
  g_capture_verify_fail_count++;
  LOGW("[VERIFY] digital capture check failed (count=%u). trying recovery on current map/route",
       (unsigned)g_capture_verify_fail_count);

  if (!apply_i2s_pin_map(g_i2s_pins, "verify_recover_current_map")) {
    LOGE("[VERIFY] recover_current_map failed");
    return false;
  }
  if (!g_codec.init(g_adc_route, profile.codec)) {
    LOGE("[VERIFY] recover_current_codec_init failed profile=%s route=%s", profile.name,
         route_name(g_adc_route));
    return false;
  }
  codec_set_dac_mute(true, "verify_recover_current");
  codec_set_output(false, "verify_recover_current");

  if (verify_digital_capture_once("recover_current", CAPTURE_VERIFY_RETRY_MS)) {
    g_capture_path_ok = true;
    return true;
  }

  if (USE_FIXED_NETLIST_CONFIG_FIRST) {
    LOGW("[VERIFY] recover_current failed. trying fixed netlist config before full probe");
    if (apply_fixed_netlist_config("verify_recover_fixed", false) &&
        verify_digital_capture_once("recover_fixed", CAPTURE_VERIFY_RETRY_MS)) {
      g_capture_path_ok = true;
      return true;
    }
  }

  if (FORCE_NETLIST_FIXED_ONLY) {
    LOGW("[VERIFY] fixed-only mode: skip auto-probe fallback");
    LOGE("[VERIFY] digital capture still invalid under fixed netlist config");
    return false;
  }

  LOGW("[VERIFY] recover_current still failed. running digital+route probe fallback");
  if (!auto_probe_digital_chain("verify_recovery")) {
    LOGE("[VERIFY] auto_probe_digital_chain failed during recovery");
    return false;
  }
  codec_set_dac_mute(true, "verify_after_reprobe");
  codec_set_output(false, "verify_after_reprobe");

  if (verify_digital_capture_once("after_reprobe", CAPTURE_VERIFY_RETRY_MS)) {
    g_capture_path_ok = true;
    return true;
  }

  LOGE("[VERIFY] digital capture still invalid after reprobe");
  return false;
}

static bool init_i2s_duplex() {
  return apply_digital_profile(FIXED_PROFILE_INDEX, "driver_init");
}

static bool ensure_record_buffer() {
  if (g_record_mono && g_record_capacity_samples >= MAX_MONO_SAMPLES) {
    return true;
  }

  if (g_record_mono) {
    free(g_record_mono);
    g_record_mono = nullptr;
    g_record_capacity_samples = 0;
    g_recorded_samples = 0;
  }

  const size_t bytes_need = MAX_MONO_SAMPLES * sizeof(int16_t);
  LOGI("[MEM] allocating record buffer: samples=%u bytes=%u", (unsigned)MAX_MONO_SAMPLES,
       (unsigned)bytes_need);

  g_record_mono = (int16_t *)ps_malloc(bytes_need);
  if (!g_record_mono) {
    LOGW("[MEM] ps_malloc failed, trying internal malloc");
    g_record_mono = (int16_t *)malloc(bytes_need);
  }

  if (!g_record_mono) {
    LOGE("[MEM] allocation failed");
    return false;
  }

  g_record_capacity_samples = MAX_MONO_SAMPLES;
  LOGI("[MEM] buffer ok ptr=%p capacity_samples=%u", g_record_mono,
       (unsigned)g_record_capacity_samples);
  return true;
}

static void stop_playback() {
  if (g_state == AudioState::kPlaying) {
    codec_set_dac_mute(true, "stop_playback");
    delay(5);
    codec_set_output(false, "stop_playback");
    set_state(AudioState::kIdle, "playback_complete");
    digitalWrite(PIN_LED, LOW);

    print_audio_stats("[PLAY][TOTAL]", g_play_stats_total);
    print_i2s_stats("[PLAY][TOTAL][I2S]", g_play_i2s_total);
  }
}

static void start_recording() {
  if (!ensure_record_buffer()) {
    LOGE("[REC] start failed: no buffer");
    return;
  }

  if (g_state == AudioState::kPlaying) {
    stop_playback();
  }

  if (!ensure_digital_capture_ready("start_recording_precheck")) {
    if (FORCE_NETLIST_FIXED_ONLY) {
      LOGW("[REC] precheck failed in fixed-only mode, continue recording for validation");
    } else {
      LOGE("[REC] start blocked: digital capture path invalid after recovery attempts");
      return;
    }
  }

  codec_set_dac_mute(true, "start_recording");
  codec_set_output(false, "start_recording");
  const DigitalProfile &profile = active_profile();

  g_recorded_samples = 0;
  g_record_start_ms = millis();
  g_play_pos = 0;
  g_rec_last_log_ms = g_record_start_ms;
  g_rec_first_chunk_dumped = false;
  g_dc_q8 = 0;
  g_dc_ready = false;
  g_raw_12bit_like = true;

  g_rec_stats_total.reset();
  g_rec_stats_window.reset();
  g_rec_i2s_total.reset();
  g_rec_i2s_window.reset();

  set_state(AudioState::kRecording, "user_button");
  digitalWrite(PIN_LED, HIGH);
  i2s_zero_dma_buffer(I2S_PORT);

  LOGI("[REC] start: profile=%s bits=%u slot_rx=%s cap_samples=%u cap_ms=%u", profile.name,
       (unsigned)bits_to_u32(profile.i2s_bits), slot_mode_name(profile.rx_slot_mode),
       (unsigned)g_record_capacity_samples, (unsigned)MAX_RECORD_MS);
}

static void stop_recording(const char *reason) {
  if (g_state != AudioState::kRecording) {
    return;
  }

  set_state(AudioState::kIdle, reason);
  digitalWrite(PIN_LED, LOW);

  uint32_t elapsed_ms = millis() - g_record_start_ms;
  float recorded_ms = (g_recorded_samples * 1000.0f) / (float)AUDIO_SAMPLE_RATE;
  LOGI("[REC] stop reason=%s samples=%u elapsed_ms=%u stored_audio_ms=%.1f", reason,
       (unsigned)g_recorded_samples, (unsigned)elapsed_ms, recorded_ms);

  print_audio_stats("[REC][TOTAL]", g_rec_stats_total);
  print_i2s_stats("[REC][TOTAL][I2S]", g_rec_i2s_total);
  LOGI("[REC][DC] blocker_ready=%s dc_estimate=%.1f(raw units)", g_dc_ready ? "yes" : "no",
       g_dc_q8 / 256.0f);
  LOGI("[REC][GAIN] raw_12bit_like=%s gain_shift=%u", g_raw_12bit_like ? "yes" : "no",
       g_raw_12bit_like ? (unsigned)RAW_12BIT_GAIN_SHIFT : 0U);
  diagnose_recording_result();

  if (g_rec_stats_total.mono_peak() <= 0) {
    g_capture_path_ok = false;
    LOGW("[REC][DIAG] all-zero capture observed. capture path marked invalid for next start");
  } else {
    g_capture_path_ok = true;
  }

  if (g_recorded_samples > 0) {
    dump_mono_samples(g_record_mono, g_recorded_samples, 16, "[REC][HEAD]");
    size_t tail_n = (g_recorded_samples < 16) ? g_recorded_samples : 16;
    dump_mono_samples(&g_record_mono[g_recorded_samples - tail_n], tail_n, tail_n,
                      "[REC][TAIL]");
  }
}

static void start_playback() {
  if (g_state != AudioState::kIdle) {
    LOGW("[PLAY] ignored start, state=%s", state_name(g_state));
    return;
  }
  const DigitalProfile &profile = active_profile();

  if (!g_record_mono || g_recorded_samples == 0) {
    LOGW("[PLAY] no recording available");
    return;
  }

  g_play_pos = 0;
  g_play_last_log_ms = millis();
  g_play_first_chunk_dumped = false;

  g_play_stats_total.reset();
  g_play_stats_window.reset();
  g_play_i2s_total.reset();
  g_play_i2s_window.reset();

  set_state(AudioState::kPlaying, "user_button");
  digitalWrite(PIN_LED, HIGH);

  codec_set_output(true, "start_playback");
  i2s_zero_dma_buffer(I2S_PORT);
  delay(2);
  codec_set_dac_mute(false, "start_playback");

  float audio_ms = (g_recorded_samples * 1000.0f) / (float)AUDIO_SAMPLE_RATE;
  LOGI("[PLAY] start profile=%s bits=%u slot_tx=%s samples=%u audio_ms=%.1f", profile.name,
       (unsigned)bits_to_u32(profile.i2s_bits), slot_mode_name(profile.tx_slot_mode),
       (unsigned)g_recorded_samples, audio_ms);
  dump_mono_samples(g_record_mono, g_recorded_samples, 16, "[PLAY][SRC_HEAD]");
}

static void log_record_window() {
  uint32_t elapsed_ms = millis() - g_record_start_ms;
  float stored_ms = (g_recorded_samples * 1000.0f) / (float)AUDIO_SAMPLE_RATE;

  LOGI("[REC][WIN] elapsed_ms=%u stored_samples=%u stored_ms=%.1f", (unsigned)elapsed_ms,
       (unsigned)g_recorded_samples, stored_ms);
  print_audio_stats("[REC][WIN][AUDIO]", g_rec_stats_window);
  print_i2s_stats("[REC][WIN][I2S]", g_rec_i2s_window);

  g_rec_stats_window.reset();
  g_rec_i2s_window.reset();
}

static void log_play_window() {
  float percent = (g_recorded_samples == 0)
                      ? 0.0f
                      : (100.0f * (float)g_play_pos / (float)g_recorded_samples);
  LOGI("[PLAY][WIN] progress=%u/%u (%.1f%%)", (unsigned)g_play_pos,
       (unsigned)g_recorded_samples, percent);
  print_audio_stats("[PLAY][WIN][AUDIO]", g_play_stats_window);
  print_i2s_stats("[PLAY][WIN][I2S]", g_play_i2s_window);

  g_play_stats_window.reset();
  g_play_i2s_window.reset();
}

static void service_recording() {
  if (g_state != AudioState::kRecording) {
    return;
  }
  const DigitalProfile &profile = active_profile();
  const size_t frame_bytes = bytes_per_sample(profile.i2s_bits) * 2;

  if ((millis() - g_record_start_ms) >= MAX_RECORD_MS) {
    stop_recording("timeout");
    return;
  }

  if (g_recorded_samples >= g_record_capacity_samples) {
    stop_recording("buffer_full");
    return;
  }

  size_t bytes_read = 0;
  g_rec_i2s_total.read_calls++;
  g_rec_i2s_window.read_calls++;

  esp_err_t err =
      i2s_read(I2S_PORT, g_i2s_rx_chunk, sizeof(g_i2s_rx_chunk), &bytes_read, pdMS_TO_TICKS(20));

  if (err != ESP_OK || bytes_read == 0) {
    if (err == ESP_ERR_TIMEOUT || bytes_read == 0) {
      g_rec_i2s_total.read_timeout++;
      g_rec_i2s_window.read_timeout++;
    } else {
      g_rec_i2s_total.read_err++;
      g_rec_i2s_window.read_err++;
      uint32_t now_ms = millis();
      if (now_ms - g_last_read_error_log_ms > 1000) {
        LOGW("[REC] i2s_read err=%d bytes=%u", (int)err, (unsigned)bytes_read);
        g_last_read_error_log_ms = now_ms;
      }
    }

    if (millis() - g_rec_last_log_ms >= LOG_STATS_INTERVAL_MS) {
      log_record_window();
      g_rec_last_log_ms = millis();
    }
    return;
  }

  g_rec_i2s_total.read_ok++;
  g_rec_i2s_window.read_ok++;
  g_rec_i2s_total.read_bytes += bytes_read;
  g_rec_i2s_window.read_bytes += bytes_read;

  size_t in_frames = bytes_read / frame_bytes;
  const size_t frames_can_store = g_record_capacity_samples - g_recorded_samples;
  if (in_frames > frames_can_store) {
    in_frames = frames_can_store;
  }

  if (!g_rec_first_chunk_dumped && in_frames > 0) {
    dump_stereo_frames(g_i2s_rx_chunk, in_frames, 12, "[REC][RX_FIRST]");
    g_rec_first_chunk_dumped = true;
  }

  for (size_t i = 0; i < in_frames; ++i) {
    int16_t l = 0;
    int16_t r = 0;
    decode_stereo_frame(g_i2s_rx_chunk + i * frame_bytes, profile, l, r);
    if (g_raw_12bit_like) {
      if (abs((int32_t)l) > 5000 || abs((int32_t)r) > 5000) {
        g_raw_12bit_like = false;
      }
    }
    // L and R come from two physically separate differential pairs
    // (LIN1 = MIC2, RIN1 = MIC1) with OPPOSITE polarity on this board.
    // Empirically: speaking gives e.g. L = +6000, R = -6000 simultaneously,
    // so (L+R)/2 cancels out (~0). The correct combination is (L-R)/2,
    // which doubles the signal and rejects common-mode hum/noise.
    int32_t mono_raw = ((int32_t)l - (int32_t)r) / 2;
    int16_t mono = dc_block_process(mono_raw);
    mono = apply_mono_gain(mono);

    g_record_mono[g_recorded_samples++] = mono;

    g_rec_stats_total.add_frame(l, r, mono);
    g_rec_stats_window.add_frame(l, r, mono);
  }

  uint32_t now_ms = millis();
  if (now_ms - g_rec_last_log_ms >= LOG_STATS_INTERVAL_MS) {
    log_record_window();
    g_rec_last_log_ms = now_ms;
  }

  if (g_recorded_samples >= g_record_capacity_samples) {
    stop_recording("buffer_full");
  }
}

static void service_playback() {
  if (g_state != AudioState::kPlaying) {
    return;
  }
  const DigitalProfile &profile = active_profile();
  const size_t frame_bytes = bytes_per_sample(profile.i2s_bits) * 2;

  if (g_play_pos >= g_recorded_samples) {
    stop_playback();
    return;
  }

  size_t frames = I2S_FRAME_SAMPLES;
  if ((g_recorded_samples - g_play_pos) < frames) {
    frames = g_recorded_samples - g_play_pos;
  }

  for (size_t i = 0; i < frames; ++i) {
    int16_t s = g_record_mono[g_play_pos + i];
    encode_stereo_frame(g_i2s_tx_chunk + i * frame_bytes, profile, s);

    g_play_stats_total.add_frame(s, s, s);
    g_play_stats_window.add_frame(s, s, s);
  }

  if (!g_play_first_chunk_dumped && frames > 0) {
    dump_stereo_frames(g_i2s_tx_chunk, frames, 12, "[PLAY][TX_FIRST]");
    g_play_first_chunk_dumped = true;
  }

  size_t bytes_to_write = frames * frame_bytes;
  size_t bytes_written = 0;

  g_play_i2s_total.write_calls++;
  g_play_i2s_window.write_calls++;

  esp_err_t err = i2s_write(I2S_PORT, g_i2s_tx_chunk, bytes_to_write, &bytes_written,
                            pdMS_TO_TICKS(60));

  if (err != ESP_OK) {
    if (err == ESP_ERR_TIMEOUT) {
      g_play_i2s_total.write_timeout++;
      g_play_i2s_window.write_timeout++;
    } else {
      g_play_i2s_total.write_err++;
      g_play_i2s_window.write_err++;
      uint32_t now_ms = millis();
      if (now_ms - g_last_write_error_log_ms > 1000) {
        LOGW("[PLAY] i2s_write err=%d asked=%u wrote=%u", (int)err, (unsigned)bytes_to_write,
             (unsigned)bytes_written);
        g_last_write_error_log_ms = now_ms;
      }
    }
  } else {
    if (bytes_written == bytes_to_write) {
      g_play_i2s_total.write_ok++;
      g_play_i2s_window.write_ok++;
    } else if (bytes_written == 0) {
      g_play_i2s_total.write_timeout++;
      g_play_i2s_window.write_timeout++;
    } else {
      g_play_i2s_total.write_partial++;
      g_play_i2s_window.write_partial++;
    }
    g_play_i2s_total.write_bytes += bytes_written;
    g_play_i2s_window.write_bytes += bytes_written;
  }

  const size_t written_frames = bytes_written / frame_bytes;
  g_play_pos += written_frames;

  uint32_t now_ms = millis();
  if (now_ms - g_play_last_log_ms >= LOG_STATS_INTERVAL_MS) {
    log_play_window();
    g_play_last_log_ms = now_ms;
  }

  if (written_frames == 0) {
    delay(1);
  }
}

static char *trim_spaces_inplace(char *s) {
  if (!s) return s;
  while (*s && isspace((unsigned char)*s)) {
    ++s;
  }
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[--n] = '\0';
  }
  return s;
}

static bool parse_u8_token(const char *token, uint8_t &out) {
  if (!token) return false;
  while (*token && isspace((unsigned char)*token)) {
    ++token;
  }
  if (*token == '\0') return false;

  char *end = nullptr;
  unsigned long v = strtoul(token, &end, 0);
  if (end && *end == '\0' && v <= 0xFFUL) {
    out = (uint8_t)v;
    return true;
  }

  bool has_hex_alpha = false;
  for (const char *p = token; *p; ++p) {
    if ((*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f')) {
      has_hex_alpha = true;
      break;
    }
  }
  if (!has_hex_alpha) return false;

  end = nullptr;
  v = strtoul(token, &end, 16);
  if (end && *end == '\0' && v <= 0xFFUL) {
    out = (uint8_t)v;
    return true;
  }
  return false;
}

static bool serial_require_idle(const char *cmd_name) {
  if (g_state != AudioState::kIdle) {
    LOGW("[SER] %s ignored, state=%s", cmd_name, state_name(g_state));
    return false;
  }
  return true;
}

static void serial_cmd_regr(char *args) {
  if (!serial_require_idle("REGR")) return;
  char *reg_s = trim_spaces_inplace(args);
  if (!reg_s || *reg_s == '\0') {
    LOGW("[SER][REGR] usage: REGR <REG> (e.g. REGR 0X09)");
    return;
  }
  uint8_t reg = 0;
  if (!parse_u8_token(reg_s, reg)) {
    LOGW("[SER][REGR] invalid reg token: %s", reg_s);
    return;
  }
  uint8_t val = 0;
  bool ok = g_codec.debug_read_register(reg, val);
  if (ok) {
    LOGI("[SER][REGR] reg=0x%02X val=0x%02X", reg, val);
  } else {
    LOGE("[SER][REGR] failed reg=0x%02X", reg);
  }
}

static void serial_cmd_regw(char *args) {
  if (!serial_require_idle("REGW")) return;
  char *p = trim_spaces_inplace(args);
  if (!p || *p == '\0') {
    LOGW("[SER][REGW] usage: REGW <REG> <VAL> (e.g. REGW 0X09 0X44)");
    return;
  }

  char *save = nullptr;
  char *reg_s = strtok_r(p, " ,\t", &save);
  char *val_s = strtok_r(nullptr, " ,\t", &save);
  if (!reg_s || !val_s) {
    LOGW("[SER][REGW] usage: REGW <REG> <VAL>");
    return;
  }

  uint8_t reg = 0;
  uint8_t val = 0;
  if (!parse_u8_token(reg_s, reg) || !parse_u8_token(val_s, val)) {
    LOGW("[SER][REGW] invalid tokens reg=%s val=%s", reg_s, val_s);
    return;
  }

  bool ok = g_codec.debug_write_register(reg, val);
  uint8_t rb = 0;
  bool rb_ok = g_codec.debug_read_register(reg, rb);
  LOGI("[SER][REGW] reg=0x%02X val=0x%02X write=%s readback=%s rb=0x%02X", reg, val,
       ok ? "ok" : "fail", rb_ok ? "ok" : "fail", rb);
}

static void serial_cmd_regs(char *args) {
  if (!serial_require_idle("REGS")) return;
  char *p = trim_spaces_inplace(args);
  if (!p || *p == '\0') {
    LOGW("[SER][REGS] usage: REGS <R=V,R=V,...> (e.g. REGS 09=44,0F=60,03=09)");
    return;
  }

  uint32_t ok_count = 0;
  uint32_t fail_count = 0;
  char *save = nullptr;
  for (char *item = strtok_r(p, ",", &save); item != nullptr; item = strtok_r(nullptr, ",", &save)) {
    char *pair = trim_spaces_inplace(item);
    if (!pair || *pair == '\0') continue;

    char *eq = strchr(pair, '=');
    if (!eq) {
      LOGW("[SER][REGS] invalid pair (missing '='): %s", pair);
      fail_count++;
      continue;
    }
    *eq = '\0';
    char *reg_s = trim_spaces_inplace(pair);
    char *val_s = trim_spaces_inplace(eq + 1);

    uint8_t reg = 0;
    uint8_t val = 0;
    if (!parse_u8_token(reg_s, reg) || !parse_u8_token(val_s, val)) {
      LOGW("[SER][REGS] invalid pair: %s=%s", reg_s ? reg_s : "(null)", val_s ? val_s : "(null)");
      fail_count++;
      continue;
    }

    bool w_ok = g_codec.debug_write_register(reg, val);
    uint8_t rb = 0;
    bool r_ok = g_codec.debug_read_register(reg, rb);
    if (w_ok && r_ok) {
      ok_count++;
      LOGI("[SER][REGS] reg=0x%02X val=0x%02X rb=0x%02X", reg, val, rb);
    } else {
      fail_count++;
      LOGW("[SER][REGS] reg=0x%02X val=0x%02X write=%s readback=%s rb=0x%02X", reg, val,
           w_ok ? "ok" : "fail", r_ok ? "ok" : "fail", rb);
    }
  }

  LOGI("[SER][REGS] done ok=%u fail=%u", (unsigned)ok_count, (unsigned)fail_count);
}


static void serial_cmd_profiles() {
  const DigitalProfile &cur = active_profile();
  LOGI("[SER][PROFILES] count=%u active_idx=%u active=%s", (unsigned)DIGITAL_PROFILE_COUNT,
       (unsigned)g_digital_profile_index, cur.name);

  for (size_t i = 0; i < DIGITAL_PROFILE_COUNT; ++i) {
    const DigitalProfile &p = kDigitalProfiles[i];
    LOGI(
        "[SER][PROFILES] idx=%u name=%s esp_role=%s codec_role=%s bits=%u comm=0x%X "
        "rx=%s tx=%s reg08=0x%02X reg17=0x%02X reg0C=0x%02X",
        (unsigned)i, p.name, p.esp_i2s_master ? "MASTER" : "SLAVE",
        (p.codec.master_mode_reg & 0x80) ? "MASTER" : "SLAVE",
        (unsigned)bits_to_u32(p.i2s_bits), (unsigned)p.comm_format, slot_mode_name(p.rx_slot_mode),
        slot_mode_name(p.tx_slot_mode), p.codec.master_mode_reg, p.codec.dac_format_reg,
        p.codec.adc_format_reg);
  }
}

static void serial_cmd_profile(char *args) {
  if (!serial_require_idle("PROFILE")) return;

  char *idx_s = trim_spaces_inplace(args);
  if (!idx_s || *idx_s == '\0') {
    LOGW("[SER][PROFILE] usage: PROFILE <IDX> (use PROFILES to list)");
    return;
  }

  uint8_t idx_u8 = 0;
  if (!parse_u8_token(idx_s, idx_u8)) {
    LOGW("[SER][PROFILE] invalid idx token: %s", idx_s);
    return;
  }

  size_t idx = (size_t)idx_u8;
  if (idx >= DIGITAL_PROFILE_COUNT) {
    LOGW("[SER][PROFILE] idx out of range: %u (count=%u)", (unsigned)idx,
         (unsigned)DIGITAL_PROFILE_COUNT);
    serial_cmd_profiles();
    return;
  }

  if (!apply_digital_profile(idx, "serial_cmd_profile_apply")) {
    LOGE("[SER][PROFILE] apply_digital_profile failed idx=%u", (unsigned)idx);
    return;
  }

  const DigitalProfile &profile = active_profile();
  if (!g_codec.init(g_adc_route, profile.codec)) {
    LOGE("[SER][PROFILE] codec init failed idx=%u name=%s route=%s", (unsigned)idx,
         profile.name, route_name(g_adc_route));
    return;
  }

  codec_set_dac_mute(true, "serial_cmd_profile");
  codec_set_output(false, "serial_cmd_profile");
  i2s_zero_dma_buffer(I2S_PORT);

  AudioStats verify_audio;
  I2sIoStats verify_io;
  bool alive = verify_digital_capture_once("serial_cmd_profile_verify", CAPTURE_VERIFY_RETRY_MS,
                                           &verify_audio, &verify_io);
  g_capture_path_ok = alive;

  LOGI("[SER][PROFILE] applied idx=%u name=%s route=%s alive=%s peak=%ld rms=%.2f silence=%.2f%%",
       (unsigned)idx, profile.name, route_name(g_adc_route), alive ? "yes" : "no",
       (long)verify_audio.mono_peak(), verify_audio.mono_rms(),
       verify_audio.silence_ratio_percent());
  print_i2s_stats("[SER][PROFILE][I2S]", verify_io);
}

static bool serial_parse_route_idx(uint8_t idx, AdcRoute &route) {
  switch (idx) {
    case 0:
      route = AdcRoute::SE_LIN1_RIN1;
      return true;
    case 1:
      route = AdcRoute::SE_LIN2_RIN2;
      return true;
    case 2:
      route = AdcRoute::DIFF_LIN1_RIN1;
      return true;
    case 3:
      route = AdcRoute::DIFF_LIN2_RIN2;
      return true;
    default:
      return false;
  }
}

static uint8_t serial_route_to_idx(AdcRoute route) {
  switch (route) {
    case AdcRoute::SE_LIN1_RIN1:
      return 0;
    case AdcRoute::SE_LIN2_RIN2:
      return 1;
    case AdcRoute::DIFF_LIN1_RIN1:
      return 2;
    case AdcRoute::DIFF_LIN2_RIN2:
      return 3;
    default:
      return 0xFF;
  }
}

static void serial_cmd_routes() {
  uint8_t active_idx = serial_route_to_idx(g_adc_route);
  LOGI("[SER][ROUTES] active_idx=%u active=%s", (unsigned)active_idx, route_name(g_adc_route));
  LOGI("[SER][ROUTES] idx=0 name=%s", route_name(AdcRoute::SE_LIN1_RIN1));
  LOGI("[SER][ROUTES] idx=1 name=%s", route_name(AdcRoute::SE_LIN2_RIN2));
  LOGI("[SER][ROUTES] idx=2 name=%s", route_name(AdcRoute::DIFF_LIN1_RIN1));
  LOGI("[SER][ROUTES] idx=3 name=%s", route_name(AdcRoute::DIFF_LIN2_RIN2));
}

static void serial_cmd_route(char *args) {
  if (!serial_require_idle("ROUTE")) return;

  char *idx_s = trim_spaces_inplace(args);
  if (!idx_s || *idx_s == '\0') {
    LOGW("[SER][ROUTE] usage: ROUTE <IDX> (use ROUTES to list)");
    return;
  }

  uint8_t idx_u8 = 0;
  if (!parse_u8_token(idx_s, idx_u8)) {
    LOGW("[SER][ROUTE] invalid idx token: %s", idx_s);
    return;
  }

  AdcRoute route = AdcRoute::DIFF_LIN1_RIN1;
  if (!serial_parse_route_idx(idx_u8, route)) {
    LOGW("[SER][ROUTE] idx out of range: %u", (unsigned)idx_u8);
    serial_cmd_routes();
    return;
  }

  g_adc_route = route;
  const DigitalProfile &profile = active_profile();
  if (!g_codec.init(g_adc_route, profile.codec)) {
    LOGE("[SER][ROUTE] codec init failed idx=%u route=%s profile=%s", (unsigned)idx_u8,
         route_name(g_adc_route), profile.name);
    return;
  }

  codec_set_dac_mute(true, "serial_cmd_route");
  codec_set_output(false, "serial_cmd_route");
  i2s_zero_dma_buffer(I2S_PORT);

  AudioStats verify_audio;
  I2sIoStats verify_io;
  bool alive = verify_digital_capture_once("serial_cmd_route_verify", CAPTURE_VERIFY_RETRY_MS,
                                           &verify_audio, &verify_io);
  g_capture_path_ok = alive;

  LOGI(
      "[SER][ROUTE] applied idx=%u route=%s profile=%s alive=%s peak=%ld rms=%.2f silence=%.2f%%",
      (unsigned)idx_u8, route_name(g_adc_route), profile.name, alive ? "yes" : "no",
      (long)verify_audio.mono_peak(), verify_audio.mono_rms(), verify_audio.silence_ratio_percent());
  print_i2s_stats("[SER][ROUTE][I2S]", verify_io);
}

static void serial_cmd_selftest() {
  if (!serial_require_idle("SELFTEST")) return;

  if (!ensure_record_buffer()) {
    LOGE("[SER][SELFTEST] no record buffer");
    return;
  }

  const uint32_t tone_ms = 1200;
  const float tone_hz = 1000.0f;
  const float amp = 9000.0f;
  size_t samples = ((size_t)AUDIO_SAMPLE_RATE * tone_ms) / 1000;
  if (samples > g_record_capacity_samples) {
    samples = g_record_capacity_samples;
  }

  const float kTwoPi = 6.28318530718f;
  const float phase_step = kTwoPi * tone_hz / (float)AUDIO_SAMPLE_RATE;
  float phase = 0.0f;

  for (size_t i = 0; i < samples; ++i) {
    int16_t s = (int16_t)(sinf(phase) * amp);
    g_record_mono[i] = s;
    phase += phase_step;
    if (phase >= kTwoPi) {
      phase -= kTwoPi;
    }
  }

  g_recorded_samples = samples;
  g_play_pos = 0;

  LOGI("[SER][SELFTEST] tone generated ms=%u hz=%.1f amp=%.1f samples=%u", (unsigned)tone_ms,
       tone_hz, amp, (unsigned)g_recorded_samples);
  dump_mono_samples(g_record_mono, g_recorded_samples, 16, "[SER][SELFTEST][HEAD]");

  start_playback();
}

// Dump raw bytes coming straight out of i2s_read - bypasses any decode/gain step.
// Used to confirm whether the line carries any non-zero bytes regardless of how
// our software interprets slots/bits.
static void serial_cmd_rxraw() {
  if (!serial_require_idle("RXRAW")) return;

  const size_t kBytes = 128;
  uint8_t buf[kBytes];
  memset(buf, 0xAA, sizeof(buf));  // poison value so we can spot "i2s_read didn't write"
  size_t bytes_read = 0;
  i2s_zero_dma_buffer(I2S_PORT);
  delay(10);
  esp_err_t err = i2s_read(I2S_PORT, buf, kBytes, &bytes_read, pdMS_TO_TICKS(300));
  LOGI("[SER][RXRAW] err=%d bytes_read=%u (poison=0xAA before call)",
       (int)err, (unsigned)bytes_read);

  size_t dump = bytes_read > 0 ? bytes_read : kBytes;
  if (dump > kBytes) dump = kBytes;
  char line[3 * 16 + 16];
  for (size_t off = 0; off < dump; off += 16) {
    size_t end = off + 16;
    if (end > dump) end = dump;
    int pos = 0;
    for (size_t i = off; i < end; ++i) {
      pos += snprintf(line + pos, sizeof(line) - pos, "%02X ", buf[i]);
    }
    LOGI("[SER][RXRAW][HEX %02u] %s", (unsigned)off, line);
  }
}

// Detach GPIO10 (or whatever current DIN pin is) from the I2S peripheral and
// sample it as a plain GPIO input as fast as possible. If the ASDOUT line is
// driven by the ES8388 (BCLK is running, so ASDOUT toggles bit-by-bit), we
// should see a healthy mix of HIGH/LOW samples. If it is electrically dead
// (line stuck at 0V or floating tied to GND) we will see all zeros / all ones.
// This is the most direct hardware-level evidence we can collect from firmware.
static void serial_cmd_pinprobe(char *args) {
  if (!serial_require_idle("PINPROBE")) return;

  // Non-destructive probe: we keep I2S running and only enable/disable the
  // GPIO pad internal pull-up/pull-down. This is critical because IO Matrix
  // routes the pin level to both I2S peripheral AND the GPIO read path, so
  // digitalRead/gpio_get_level continue to work on the pin even while it is
  // wired to I2S RX signal.
  //
  // Pull tests tell us a lot:
  //   - Internal pull-up wins (read=1) -> line is HIGH-Z (no driver), codec
  //     ASDOUT is not driving the line (could be still in tri-state, or
  //     internally stuck floating, or wire is broken).
  //   - Internal pull-down wins (read=0) -> consistent with high-Z too.
  //   - Internal pull-up CAN'T win (read=0 even with PU) -> line is being
  //     ACTIVELY DRIVEN LOW by something stronger (codec push-pull output
  //     stuck at 0, or short to GND).
  //   - With both pulls disabled, watch for transitions in a tight loop ->
  //     if codec is driving real digital traffic we should see them.

  // Optional argument: explicit GPIO number to probe. If omitted, default to
  // the current I2S DIN. This lets us probe MCLK/BCLK/WS/DOUT to confirm
  // ESP-side I2S is alive and to compare ESP-driven vs codec-driven lines.
  int din_pin = g_i2s_pins.din;
  const char *probe_label = "DIN";
  if (args != nullptr) {
    char *trimmed = trim_spaces_inplace(args);
    if (trimmed != nullptr && *trimmed != '\0') {
      char *endp = nullptr;
      long val = strtol(trimmed, &endp, 0);
      if (endp != trimmed && val >= 0 && val < 64) {
        din_pin = (int)val;
        if (val == g_i2s_pins.mclk) probe_label = "MCLK";
        else if (val == g_i2s_pins.bclk) probe_label = "BCLK";
        else if (val == g_i2s_pins.ws) probe_label = "WS";
        else if (val == g_i2s_pins.dout) probe_label = "DOUT";
        else if (val == g_i2s_pins.din) probe_label = "DIN";
        else probe_label = "USER";
      } else {
        LOGW("[SER][PINPROBE] bad gpio arg '%s', falling back to DIN=%d", trimmed, din_pin);
      }
    }
  }
  const gpio_num_t gpio = (gpio_num_t)din_pin;
  LOGI("[SER][PINPROBE] non-destructive probe on gpio=%d label=%s (I2S kept running)", din_pin, probe_label);

  gpio_pullup_dis(gpio);
  gpio_pulldown_dis(gpio);
  delayMicroseconds(200);

  gpio_pullup_en(gpio);
  delayMicroseconds(500);
  int idle_pu = gpio_get_level(gpio);
  gpio_pullup_dis(gpio);

  gpio_pulldown_en(gpio);
  delayMicroseconds(500);
  int idle_pd = gpio_get_level(gpio);
  gpio_pulldown_dis(gpio);
  delayMicroseconds(200);

  const int kSamples = 8192;
  int high = 0;
  int transitions = 0;
  int first = gpio_get_level(gpio);
  int last = first;
  uint32_t t0 = micros();
  for (int i = 0; i < kSamples; ++i) {
    int v = gpio_get_level(gpio);
    if (v) ++high;
    if (v != last) ++transitions;
    last = v;
  }
  uint32_t dt = micros() - t0;

  LOGI("[SER][PINPROBE] pin=%d idle_with_pullup=%d idle_with_pulldown=%d",
       din_pin, idle_pu, idle_pd);
  LOGI("[SER][PINPROBE] pin=%d samples=%d high=%d low=%d transitions=%d duration_us=%lu first=%d last=%d",
       din_pin, kSamples, high, kSamples - high, transitions, (unsigned long)dt, first, last);

  if (transitions > 0) {
    LOGI("[SER][PINPROBE] %s is TOGGLING -> line is electrically alive", probe_label);
  } else if (high == 0 && idle_pu == 1) {
    LOGW("[SER][PINPROBE] %s is FLOATING / HIGH-Z (no driver, internal PU wins)", probe_label);
    LOGW("[SER][PINPROBE]   -> nothing is driving this line right now");
  } else if (high == 0 && idle_pu == 0) {
    LOGW("[SER][PINPROBE] %s is ACTIVELY HELD LOW (PU cannot win)", probe_label);
    LOGW("[SER][PINPROBE]   -> something is sinking line to GND (driver stuck low or short-to-GND)");
  } else if (high == kSamples && idle_pd == 1) {
    LOGW("[SER][PINPROBE] %s is ACTIVELY HELD HIGH (PD cannot win)", probe_label);
  } else {
    LOGW("[SER][PINPROBE] %s inconclusive: high=%d idle_pu=%d idle_pd=%d", probe_label, high, idle_pu, idle_pd);
  }
}

static void serial_cmd_rxsnap() {
  if (!serial_require_idle("RXSNAP")) return;

  const uint32_t snap_ms = 300;
  const DigitalProfile &profile = active_profile();
  const size_t frame_bytes = bytes_per_sample(profile.i2s_bits) * 2;

  codec_set_dac_mute(true, "serial_cmd_rxsnap");
  codec_set_output(false, "serial_cmd_rxsnap");
  i2s_zero_dma_buffer(I2S_PORT);
  delay(10);

  AudioStats audio;
  I2sIoStats io;
  audio.reset();
  io.reset();

  uint32_t lr_both_zero = 0;
  uint32_t lr_any_nonzero = 0;
  uint32_t l_nonzero = 0;
  uint32_t r_nonzero = 0;
  bool dumped = false;

  uint32_t t0 = millis();
  while ((millis() - t0) < snap_ms) {
    size_t bytes_read = 0;
    io.read_calls++;

    esp_err_t err =
        i2s_read(I2S_PORT, g_i2s_rx_chunk, sizeof(g_i2s_rx_chunk), &bytes_read, pdMS_TO_TICKS(20));

    if (err != ESP_OK || bytes_read == 0) {
      if (err == ESP_ERR_TIMEOUT || bytes_read == 0) {
        io.read_timeout++;
      } else {
        io.read_err++;
      }
      continue;
    }

    io.read_ok++;
    io.read_bytes += bytes_read;

    size_t frames = bytes_read / frame_bytes;
    if (!dumped && frames > 0) {
      dump_stereo_frames(g_i2s_rx_chunk, frames, 12, "[SER][RXSNAP][FIRST]");
      dumped = true;
    }

    for (size_t i = 0; i < frames; ++i) {
      int16_t l = 0;
      int16_t r = 0;
      decode_stereo_frame(g_i2s_rx_chunk + i * frame_bytes, profile, l, r);
      int16_t m = (int16_t)(((int32_t)l + (int32_t)r) / 2);
      audio.add_frame(l, r, m);

      if (l == 0 && r == 0) {
        lr_both_zero++;
      } else {
        lr_any_nonzero++;
      }
      if (l != 0) l_nonzero++;
      if (r != 0) r_nonzero++;
    }
  }

  LOGI(
      "[SER][RXSNAP] ms=%u profile=%s route=%s frames=%u lr_nonzero=%u lr_both_zero=%u l_nonzero=%u r_nonzero=%u",
      (unsigned)snap_ms, profile.name, route_name(g_adc_route), (unsigned)audio.frame_count,
      (unsigned)lr_any_nonzero, (unsigned)lr_both_zero, (unsigned)l_nonzero,
      (unsigned)r_nonzero);
  print_audio_stats("[SER][RXSNAP][AUDIO]", audio);
  print_i2s_stats("[SER][RXSNAP][I2S]", io);

  if (audio.frame_count == 0) {
    LOGW("[SER][RXSNAP] no frames captured");
  } else if (lr_any_nonzero == 0) {
    LOGW("[SER][RXSNAP] all captured LR frames are zero");
  }
}

static bool serial_parse_pinmap_idx(uint8_t idx, I2sPinMap &map) {
  switch (idx) {
    case 0:
      map = I2sPinMap("NETLIST_DEFAULT", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT,
                      PIN_I2S_DIN);
      return true;
    case 1:
      map = I2sPinMap("SWAP_DIN_DOUT", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DIN,
                      PIN_I2S_DOUT);
      return true;
    case 2:
      map = I2sPinMap("SWAP_WS_DOUT_KEEP_DIN", PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_DOUT,
                      PIN_I2S_WS, PIN_I2S_DIN);
      return true;
    default:
      return false;
  }
}

static uint8_t serial_pinmap_to_idx(const I2sPinMap &map) {
  if (map.mclk == PIN_I2S_MCLK && map.bclk == PIN_I2S_BCLK && map.ws == PIN_I2S_WS &&
      map.dout == PIN_I2S_DOUT && map.din == PIN_I2S_DIN) {
    return 0;
  }
  if (map.mclk == PIN_I2S_MCLK && map.bclk == PIN_I2S_BCLK && map.ws == PIN_I2S_WS &&
      map.dout == PIN_I2S_DIN && map.din == PIN_I2S_DOUT) {
    return 1;
  }
  if (map.mclk == PIN_I2S_MCLK && map.bclk == PIN_I2S_BCLK && map.ws == PIN_I2S_DOUT &&
      map.dout == PIN_I2S_WS && map.din == PIN_I2S_DIN) {
    return 2;
  }
  return 0xFF;
}

static void serial_cmd_pinmaps() {
  uint8_t active_idx = serial_pinmap_to_idx(g_i2s_pins);
  LOGI("[SER][PINMAPS] active_idx=%u active_name=%s mclk=%d bclk=%d ws=%d dout=%d din=%d",
       (unsigned)active_idx, g_i2s_pins.name, g_i2s_pins.mclk, g_i2s_pins.bclk, g_i2s_pins.ws,
       g_i2s_pins.dout, g_i2s_pins.din);
  LOGI("[SER][PINMAPS] idx=0 name=NETLIST_DEFAULT mclk=%d bclk=%d ws=%d dout=%d din=%d",
       PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN);
  LOGI("[SER][PINMAPS] idx=1 name=SWAP_DIN_DOUT mclk=%d bclk=%d ws=%d dout=%d din=%d",
       PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DIN, PIN_I2S_DOUT);
  LOGI(
      "[SER][PINMAPS] idx=2 name=SWAP_WS_DOUT_KEEP_DIN mclk=%d bclk=%d ws=%d dout=%d din=%d",
      PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_DOUT, PIN_I2S_WS, PIN_I2S_DIN);
}

static void serial_cmd_pinmap(char *args) {
  if (!serial_require_idle("PINMAP")) return;

  char *idx_s = trim_spaces_inplace(args);
  if (!idx_s || *idx_s == '\0') {
    LOGW("[SER][PINMAP] usage: PINMAP <IDX> (use PINMAPS to list)");
    return;
  }

  uint8_t idx_u8 = 0;
  if (!parse_u8_token(idx_s, idx_u8)) {
    LOGW("[SER][PINMAP] invalid idx token: %s", idx_s);
    return;
  }

  I2sPinMap map;
  if (!serial_parse_pinmap_idx(idx_u8, map)) {
    LOGW("[SER][PINMAP] idx out of range: %u", (unsigned)idx_u8);
    serial_cmd_pinmaps();
    return;
  }

  if (!apply_i2s_pin_map(map, "serial_cmd_pinmap")) {
    LOGE("[SER][PINMAP] apply_i2s_pin_map failed idx=%u", (unsigned)idx_u8);
    return;
  }

  i2s_zero_dma_buffer(I2S_PORT);
  delay(10);

  AudioStats verify_audio;
  I2sIoStats verify_io;
  bool alive = verify_digital_capture_once("serial_cmd_pinmap_verify", CAPTURE_VERIFY_RETRY_MS,
                                           &verify_audio, &verify_io);
  g_capture_path_ok = alive;

  LOGI(
      "[SER][PINMAP] applied idx=%u name=%s route=%s profile=%s alive=%s peak=%ld rms=%.2f silence=%.2f%%",
      (unsigned)idx_u8, g_i2s_pins.name, route_name(g_adc_route), active_profile().name,
      alive ? "yes" : "no", (long)verify_audio.mono_peak(), verify_audio.mono_rms(),
      verify_audio.silence_ratio_percent());
  print_i2s_stats("[SER][PINMAP][I2S]", verify_io);
}
static void serial_print_help() {
  LOGI("[SER] commands:");
  LOGI("[SER]   HELP");
  LOGI("[SER]   STATUS");
  LOGI("[SER]   REBOOT");
  LOGI("[SER]   REC_START");
  LOGI("[SER]   REC_STOP");
  LOGI("[SER]   REC_TOGGLE");
  LOGI("[SER]   PLAY");
  LOGI("[SER]   STOP");
  LOGI("[SER]   VERIFY");
  LOGI("[SER]   BYPASS_ON / BYPASS_OFF");
  LOGI("[SER]   PROFILES");
  LOGI("[SER]   PROFILE <IDX>");
  LOGI("[SER]   ROUTES");
  LOGI("[SER]   ROUTE <IDX>");
  LOGI("[SER]   SELFTEST");
  LOGI("[SER]   RXSNAP");
  LOGI("[SER]   RXRAW       (hex dump 128 bytes from i2s_read)");
  LOGI("[SER]   PINPROBE [GPIO]  (sample any GPIO live; default=DIN)");
  LOGI("[SER]   PINMAPS");
  LOGI("[SER]   PINMAP <IDX>");
  LOGI("[SER]   REGR <REG>");
  LOGI("[SER]   REGW <REG> <VAL>");
  LOGI("[SER]   REGS <R=V,R=V,...>");
  LOGI("[SER]   REGDUMP");
}

static void serial_print_status(const char *reason) {
  const DigitalProfile &profile = active_profile();
  LOGI(
      "[SER][STATUS] reason=%s state=%s profile=%s esp_role=%s codec_role=%s route=%s "
      "capture_ok=%s rec_samples=%u rec_capacity=%u play_pos=%u",
      reason, state_name(g_state), profile.name, profile.esp_i2s_master ? "MASTER" : "SLAVE",
      (profile.codec.master_mode_reg & 0x80) ? "MASTER" : "SLAVE", route_name(g_adc_route),
      g_capture_path_ok ? "yes" : "no", (unsigned)g_recorded_samples,
      (unsigned)g_record_capacity_samples, (unsigned)g_play_pos);
  LOGI("[SER][STATUS] pins mclk=%d bclk=%d ws=%d dout=%d din=%d", g_i2s_pins.mclk,
       g_i2s_pins.bclk, g_i2s_pins.ws, g_i2s_pins.dout, g_i2s_pins.din);
}

static void serial_exec_command(char *line) {
  char *cmd = line;
  while (*cmd && isspace((unsigned char)*cmd)) {
    cmd++;
  }
  size_t line_len = strlen(cmd);
  while (line_len > 0 && isspace((unsigned char)cmd[line_len - 1])) {
    cmd[--line_len] = '\0';
  }
  if (line_len == 0) {
    return;
  }

  char *args = cmd;
  while (*args && !isspace((unsigned char)*args)) {
    ++args;
  }
  if (*args) {
    *args = '\0';
    ++args;
    args = trim_spaces_inplace(args);
  } else {
    args = (char *)"";
  }

  for (char *p = cmd; *p; ++p) {
    *p = (char)toupper((unsigned char)*p);
  }

  if (args && *args) {
    LOGI("[SER] cmd=%s args=%s", cmd, args);
  } else {
    LOGI("[SER] cmd=%s", cmd);
  }

  if (strcmp(cmd, "HELP") == 0) {
    serial_print_help();
    return;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    serial_print_status("serial_cmd");
    return;
  }
  if (strcmp(cmd, "PROFILES") == 0) {
    serial_cmd_profiles();
    return;
  }
  if (strcmp(cmd, "PROFILE") == 0) {
    serial_cmd_profile(args);
    return;
  }
  if (strcmp(cmd, "ROUTES") == 0) {
    serial_cmd_routes();
    return;
  }
  if (strcmp(cmd, "ROUTE") == 0) {
    serial_cmd_route(args);
    return;
  }
  if (strcmp(cmd, "SELFTEST") == 0) {
    serial_cmd_selftest();
    return;
  }
  if (strcmp(cmd, "RXSNAP") == 0) {
    serial_cmd_rxsnap();
    return;
  }
  if (strcmp(cmd, "RXRAW") == 0) {
    serial_cmd_rxraw();
    return;
  }
  if (strcmp(cmd, "PINPROBE") == 0) {
    serial_cmd_pinprobe(args);
    return;
  }
  if (strcmp(cmd, "PINMAPS") == 0) {
    serial_cmd_pinmaps();
    return;
  }
  if (strcmp(cmd, "PINMAP") == 0) {
    serial_cmd_pinmap(args);
    return;
  }
  if (strcmp(cmd, "REBOOT") == 0) {
    LOGW("[SER] reboot requested");
    Serial.flush();
    delay(50);
    ESP.restart();
    return;
  }
  if (strcmp(cmd, "REC_START") == 0) {
    if (g_state == AudioState::kIdle) {
      start_recording();
    } else {
      LOGW("[SER] REC_START ignored, state=%s", state_name(g_state));
    }
    return;
  }
  if (strcmp(cmd, "REC_STOP") == 0) {
    if (g_state == AudioState::kRecording) {
      stop_recording("serial_cmd");
    } else {
      LOGW("[SER] REC_STOP ignored, state=%s", state_name(g_state));
    }
    return;
  }
  if (strcmp(cmd, "REC_TOGGLE") == 0) {
    if (g_state == AudioState::kRecording) {
      stop_recording("serial_cmd_toggle");
    } else if (g_state == AudioState::kIdle) {
      start_recording();
    } else {
      LOGW("[SER] REC_TOGGLE ignored, state=%s", state_name(g_state));
    }
    return;
  }
  if (strcmp(cmd, "PLAY") == 0) {
    if (g_state == AudioState::kIdle) {
      start_playback();
    } else {
      LOGW("[SER] PLAY ignored, state=%s", state_name(g_state));
    }
    return;
  }
  if (strcmp(cmd, "STOP") == 0) {
    if (g_state == AudioState::kRecording) {
      stop_recording("serial_cmd_stop");
    } else if (g_state == AudioState::kPlaying) {
      stop_playback();
    } else {
      LOGW("[SER] STOP ignored, state=%s", state_name(g_state));
    }
    return;
  }
  if (strcmp(cmd, "VERIFY") == 0) {
    if (g_state == AudioState::kIdle) {
      bool ok = ensure_digital_capture_ready("serial_cmd_verify");
      LOGI("[SER] VERIFY result=%s", ok ? "ok" : "fail");
    } else {
      LOGW("[SER] VERIFY ignored, state=%s", state_name(g_state));
    }
    return;
  }
  if (strcmp(cmd, "BYPASS_ON") == 0) {
    if (g_state != AudioState::kIdle) {
      LOGW("[SER] BYPASS_ON ignored, state=%s", state_name(g_state));
      return;
    }
    bool ok = g_codec.set_analog_bypass(true);
    LOGI("[SER] BYPASS_ON result=%s", ok ? "ok" : "fail");
    return;
  }
  if (strcmp(cmd, "BYPASS_OFF") == 0) {
    if (g_state != AudioState::kIdle) {
      LOGW("[SER] BYPASS_OFF ignored, state=%s", state_name(g_state));
      return;
    }
    bool ok = g_codec.set_analog_bypass(false);
    LOGI("[SER] BYPASS_OFF result=%s", ok ? "ok" : "fail");
    return;
  }
  if (strcmp(cmd, "REGR") == 0) {
    serial_cmd_regr(args);
    return;
  }
  if (strcmp(cmd, "REGW") == 0) {
    serial_cmd_regw(args);
    return;
  }
  if (strcmp(cmd, "REGS") == 0) {
    serial_cmd_regs(args);
    return;
  }
  if (strcmp(cmd, "REGDUMP") == 0) {
    if (!serial_require_idle("REGDUMP")) return;
    g_codec.dump_registers();
    return;
  }

  LOGW("[SER] unknown command: %s", cmd);
  serial_print_help();
}

static void service_serial_commands() {
  while (Serial.available() > 0) {
    int ch = Serial.read();
    if (ch < 0) {
      break;
    }

    if (ch == '\r' || ch == '\n') {
      if (g_serial_cmd_overflow) {
        LOGW("[SER] command too long (>=%u), dropped", (unsigned)SERIAL_CMD_MAX_LEN);
      } else if (g_serial_cmd_len > 0) {
        g_serial_cmd_buf[g_serial_cmd_len] = '\0';
        serial_exec_command(g_serial_cmd_buf);
      }
      g_serial_cmd_len = 0;
      g_serial_cmd_overflow = false;
      continue;
    }

    if (g_serial_cmd_overflow) {
      continue;
    }
    if (g_serial_cmd_len + 1 < SERIAL_CMD_MAX_LEN) {
      g_serial_cmd_buf[g_serial_cmd_len++] = (char)ch;
    } else {
      g_serial_cmd_overflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  LOGI("======== AUDIO DIAGNOSTIC BUILD ========");
  LOGI("[PIN] I2C SDA=%d SCL=%d", PIN_I2C_SDA, PIN_I2C_SCL);
  LOGI("[PIN] I2S default mclk=%d bclk=%d ws=%d dout=%d din=%d", PIN_I2S_MCLK, PIN_I2S_BCLK,
       PIN_I2S_WS, PIN_I2S_DOUT, PIN_I2S_DIN);
  LOGI("[PIN] BTN REC(GPIO12)=%d BTN PLAY(GPIO13)=%d LED=%d", PIN_BTN1, PIN_BTN3, PIN_LED);
  LOGI("[CFG] sample_rate=%u max_record_ms=%u chunk_frames=%u", (unsigned)AUDIO_SAMPLE_RATE,
       (unsigned)MAX_RECORD_MS, (unsigned)I2S_FRAME_SAMPLES);
  LOGI("[CFG] digital_profiles=%u (format/slot/master fallback enabled)",
       (unsigned)DIGITAL_PROFILE_COUNT);
  LOGI("[CFG] fixed_netlist_first=%s fixed_profile_idx=%u fixed_route=%s",
       USE_FIXED_NETLIST_CONFIG_FIRST ? "true" : "false", (unsigned)FIXED_PROFILE_INDEX,
       route_name(FIXED_ADC_ROUTE));

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  g_btn_record.begin(PIN_BTN1);  // GPIO12
  g_btn_play.begin(PIN_BTN3);    // GPIO13

  g_codec.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!g_codec.ping()) {
    LOGE("[FAIL] ES8388 not found at 0x%02X", ES8388_I2C_ADDR);
    while (true) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(150);
    }
  }
  LOGI("[CODEC] ES8388 detected at 0x%02X", ES8388_I2C_ADDR);

  if (!init_i2s_duplex()) {
    LOGE("[FAIL] I2S init failed");
    while (true) {
      delay(1000);
    }
  }

  if (!ensure_record_buffer()) {
    LOGE("[FAIL] record buffer init failed");
    while (true) {
      delay(1000);
    }
  }

  bool setup_path_ready = false;
  if (USE_FIXED_NETLIST_CONFIG_FIRST) {
    setup_path_ready = apply_fixed_netlist_config("setup_fixed", false);
    LOGI("[SETUP] fixed netlist config result=%s", setup_path_ready ? "ok" : "fail");
  }
  if (!setup_path_ready) {
    LOGW("[SETUP] fixed config unavailable, falling back to digital+route probe");
    if (!auto_probe_digital_chain("setup_fallback_probe")) {
      LOGE("[FAIL] digital+route probe failed");
      while (true) {
        delay(1000);
      }
    }
  }
  const DigitalProfile &selected_profile = active_profile();
  LOGI("[CODEC] final init ok profile=%s route=%s", selected_profile.name, route_name(g_adc_route));
  LOGI("[I2S] final map=%s mclk=%d bclk=%d ws=%d dout=%d din=%d", g_i2s_pins.name, g_i2s_pins.mclk,
       g_i2s_pins.bclk, g_i2s_pins.ws, g_i2s_pins.dout, g_i2s_pins.din);
  g_codec.dump_registers();

  if (RUN_STARTUP_BYPASS_TEST) {
    LOGI("[BYPASS] warmup before startup bypass: %u ms", STARTUP_BYPASS_PRE_DELAY_MS);
    delay(STARTUP_BYPASS_PRE_DELAY_MS);
    LOGI("[BYPASS] startup analog bypass test ON for %u ms. speak near MIC now.", STARTUP_BYPASS_MS);
    bool ok_on = g_codec.set_analog_bypass(true);
    LOGI("[BYPASS] enable result=%s", ok_on ? "ok" : "fail");
    delay(STARTUP_BYPASS_MS);
    bool ok_off = g_codec.set_analog_bypass(false);
    LOGI("[BYPASS] disable result=%s", ok_off ? "ok" : "fail");
    LOGI("[BYPASS] test finished");
  }

  if (!apply_i2s_pin_map(g_i2s_pins, "setup_restore_map")) {
    LOGE("[FAIL] setup restore map failed");
    while (true) {
      delay(1000);
    }
  }
  const DigitalProfile &setup_profile = active_profile();
  if (!g_codec.init(g_adc_route, setup_profile.codec)) {
    LOGE("[FAIL] setup restore codec init failed profile=%s route=%s", setup_profile.name,
         route_name(g_adc_route));
    while (true) {
      delay(1000);
    }
  }

  codec_set_dac_mute(true, "setup");
  codec_set_output(false, "setup");
  i2s_zero_dma_buffer(I2S_PORT);

  g_capture_path_ok = ensure_digital_capture_ready("setup_post_bypass");
  if (!g_capture_path_ok) {
    LOGW("[READY] digital capture is still invalid now; REC button will auto-retry recovery");
  }

  LOGI("[READY] GPIO12: record start/stop, auto-stop at 10s");
  LOGI("[READY] GPIO13: playback (if recording exists)");
  LOGI("[READY] serial control enabled on USB CDC (115200)");
  serial_print_help();
}

void loop() {
  service_serial_commands();

  if (g_btn_record.fell()) {
    LOGI("[BTN] REC pressed, state=%s", state_name(g_state));
    if (g_state == AudioState::kRecording) {
      stop_recording("button");
    } else if (g_state == AudioState::kIdle) {
      start_recording();
    } else {
      LOGW("[BTN] REC ignored in state=%s", state_name(g_state));
    }
  }

  if (g_btn_play.fell()) {
    LOGI("[BTN] PLAY pressed, state=%s", state_name(g_state));
    if (g_state == AudioState::kIdle) {
      start_playback();
    } else {
      LOGW("[BTN] PLAY ignored in state=%s", state_name(g_state));
    }
  }

  service_recording();
  service_playback();
  delay(1);
}







