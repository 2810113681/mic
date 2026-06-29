#pragma once

#include <Arduino.h>

// Netlist-based mapping (N16R8 board, ES8388 QFN-28):
// U1.1  -> GPIO8  (MCLK)
// U1.5  -> GPIO3  (SCLK/BCLK)
// U1.6  -> GPIO46 (DSDIN, ESP->CODEC)
// U1.7  -> GPIO9  (LRCK/WS)
// U1.8  -> GPIO10 (ASDOUT, CODEC->ESP)
// I2C
static constexpr int PIN_I2C_SDA = 14;
static constexpr int PIN_I2C_SCL = 47;
static constexpr uint8_t ES8388_I2C_ADDR = 0x10;

// I2S - matches official ES8388 QFN-28 datasheet pinout (Rev 5.0, Jul 2018).
//
//   Pin 1 (MCLK)   -> GPIO8
//   Pin 5 (SCLK)   -> GPIO3
//   Pin 6 (DSDIN)  -> GPIO46  : DAC data INPUT to codec  (ESP TX)
//   Pin 7 (LRCK)   -> GPIO9
//   Pin 8 (ASDOUT) -> GPIO10  : ADC data OUTPUT from codec (ESP RX)
//
// The earlier "swap test" (DOUT=10, DIN=46) was a misdiagnosis: it routed
// ESP's push-pull TX onto the codec's ASDOUT push-pull OUTPUT pin and likely
// damaged that output driver. Do NOT swap these again.
static constexpr int PIN_I2S_MCLK = 8;
static constexpr int PIN_I2S_BCLK = 3;
static constexpr int PIN_I2S_WS = 9;
static constexpr int PIN_I2S_DOUT = 46; // ESP TX -> ES8388 Pin 6 (DSDIN)
static constexpr int PIN_I2S_DIN = 10;  // ES8388 Pin 8 (ASDOUT) -> ESP RX

// UI
static constexpr int PIN_BTN1 = 12;
static constexpr int PIN_BTN2 = 11;
static constexpr int PIN_BTN3 = 13;
static constexpr int PIN_LED = 21;

// Align with reference project default.
static constexpr uint32_t AUDIO_SAMPLE_RATE = 24000;
