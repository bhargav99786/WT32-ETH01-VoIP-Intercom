/*
 * I2S Hardware Test Sketch for INMP441 (Mic) & MAX98357A (Speaker)
 * 
 * Features:
 * 1. Startup Test Tone (440 Hz Beep) - verifies Speaker amplifier output.
 * 2. Live Serial Volume Meter - verifies Microphone audio input with ASCII bar graph.
 * 3. Real-time Mic-to-Speaker Audio Loopback - speak into mic to hear on speaker.
 * 4. Serial Commands:
 *    - Send 't' in Serial Monitor -> Plays 1-second Test Tone.
 *    - Send 'l' in Serial Monitor -> Toggles Live Audio Loopback (ON/OFF).
 * 
 * Pin Mapping:
 *   ESP32-C3 (ETH01 EVO):
 *     - Mic:     BCLK = 18, WS = 19, SD = 1
 *     - Speaker: BCLK = 5,  WS = 2,  DIN = 4
 *   Standard ESP32 (WT32-ETH01):
 *     - Mic:     BCLK = 32, WS = 33, SD = 36
 *     - Speaker: BCLK = 14, WS = 15, DIN = 4
 */

#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>

#ifndef I2S_NUM_1
  #include <esp_rom_gpio.h>
  #include <soc/gpio_sig_map.h>
  #include <soc/usb_serial_jtag_reg.h>
  #include <soc/gpio_periph.h>
  #include <driver/gpio.h>
#endif

// --- Pin Definitions ---
#ifdef I2S_NUM_1
  // Standard ESP32 (WT32-ETH01)
  #define I2S_PORT_RX I2S_NUM_0
  #define I2S_PORT_TX I2S_NUM_1

  #define I2S_RX_BCLK_PIN  32
  #define I2S_RX_LRC_PIN   33
  #define I2S_RX_DOUT_PIN  36

  #define I2S_TX_BCLK_PIN  14
  #define I2S_TX_LRC_PIN   15
  #define I2S_TX_DIN_PIN   4
#else
  // ESP32-C3 (ETH01 EVO)
  #define I2S_PORT_RX I2S_NUM_0
  #define I2S_PORT_TX I2S_NUM_0

  #define I2S_RX_BCLK_PIN  18
  #define I2S_RX_LRC_PIN   19
  #define I2S_RX_DOUT_PIN  1

  #define I2S_TX_BCLK_PIN  5
  #define I2S_TX_LRC_PIN   2
  #define I2S_TX_DIN_PIN   4
#endif

#define SAMPLE_RATE 16000 // 16 kHz sample rate for test
bool loopback_enabled = true;

void setup_i2s() {
#ifdef I2S_NUM_1
  // Dual I2S Port Setup for ESP32
  i2s_config_t i2s_config_rx = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_rx = {
      .bck_io_num = I2S_RX_BCLK_PIN,
      .ws_io_num = I2S_RX_LRC_PIN,
      .data_out_num = I2S_PIN_NO_CHANGE,
      .data_in_num = I2S_RX_DOUT_PIN
  };

  i2s_driver_install(I2S_PORT_RX, &i2s_config_rx, 0, NULL);
  i2s_set_pin(I2S_PORT_RX, &pin_config_rx);
  i2s_start(I2S_PORT_RX);

  i2s_config_t i2s_config_tx = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_tx = {
      .bck_io_num = I2S_TX_BCLK_PIN,
      .ws_io_num = I2S_TX_LRC_PIN,
      .data_out_num = I2S_TX_DIN_PIN,
      .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_PORT_TX, &i2s_config_tx, 0, NULL);
  i2s_set_pin(I2S_PORT_TX, &pin_config_tx);
  i2s_start(I2S_PORT_TX);
#else
  // Single I2S Port Setup with Independent Pins for ESP32-C3
  // Disconnect USB PHY from GPIO 18 and GPIO 19 so they function as standard GPIOs
  CLEAR_PERI_REG_MASK(USB_SERIAL_JTAG_CONF0_REG, USB_SERIAL_JTAG_USB_PAD_ENABLE);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_RX_BCLK_PIN], PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_RX_LRC_PIN],  PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_RX_DOUT_PIN], PIN_FUNC_GPIO);

  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_TX_BCLK_PIN], PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_TX_LRC_PIN],  PIN_FUNC_GPIO);
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[I2S_TX_DIN_PIN],  PIN_FUNC_GPIO);

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 256,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_TX_BCLK_PIN,
      .ws_io_num = I2S_TX_LRC_PIN,
      .data_out_num = I2S_TX_DIN_PIN,
      .data_in_num = I2S_RX_DOUT_PIN
  };

  i2s_driver_install(I2S_PORT_RX, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT_RX, &pin_config);

  // Route active master clock lines to Mic BCLK & LRC pins via ESP32-C3 GPIO Matrix
  gpio_set_direction((gpio_num_t)I2S_RX_BCLK_PIN, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_direction((gpio_num_t)I2S_RX_LRC_PIN, GPIO_MODE_INPUT_OUTPUT);
  esp_rom_gpio_connect_out_signal(I2S_RX_BCLK_PIN, I2SO_BCK_OUT_IDX, false, false);
  esp_rom_gpio_connect_out_signal(I2S_RX_LRC_PIN, I2SO_WS_OUT_IDX, false, false);

  // Connect Mic BCLK & WS pins back to the I2S RX receiver clock inputs
  esp_rom_gpio_connect_in_signal(I2S_RX_BCLK_PIN, I2SI_BCK_IN_IDX, false);
  esp_rom_gpio_connect_in_signal(I2S_RX_LRC_PIN, I2SI_WS_IN_IDX, false);

  // Route Mic Data In pin explicitly to I2S RX Data Input signal
  gpio_set_direction((gpio_num_t)I2S_RX_DOUT_PIN, GPIO_MODE_INPUT);
  esp_rom_gpio_connect_in_signal(I2S_RX_DOUT_PIN, I2SI_SD_IN_IDX, false);

  i2s_start(I2S_PORT_RX);
#endif
}

void play_test_tone(float freq_hz, int duration_ms) {
  Serial.printf("[TEST TONE] Playing %.0f Hz beep for %d ms...\n", freq_hz, duration_ms);
  int total_samples = (SAMPLE_RATE * duration_ms) / 1000;
  float phase_inc = 2.0 * M_PI * freq_hz / SAMPLE_RATE;
  float phase = 0.0;

  int chunk_size = 128;
  for (int i = 0; i < total_samples; i += chunk_size) {
    int samples_to_generate = (total_samples - i > chunk_size) ? chunk_size : (total_samples - i);
    
#ifdef I2S_NUM_1
    int16_t tone_buf[chunk_size];
    for (int s = 0; s < samples_to_generate; s++) {
      tone_buf[s] = (int16_t)(sinf(phase) * 8000.0f);
      phase += phase_inc;
      if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
    size_t bytes_written = 0;
    i2s_write(I2S_PORT_TX, tone_buf, samples_to_generate * sizeof(int16_t), &bytes_written, portMAX_DELAY);
#else
    int32_t tone_buf_32[chunk_size * 2]; // Stereo 32-bit
    for (int s = 0; s < samples_to_generate; s++) {
      int16_t val16 = (int16_t)(sinf(phase) * 8000.0f);
      int32_t val32 = ((int32_t)val16) << 16;
      tone_buf_32[s * 2]     = val32; // Left
      tone_buf_32[s * 2 + 1] = val32; // Right
      phase += phase_inc;
      if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
    }
    size_t bytes_written = 0;
    i2s_write(I2S_PORT_TX, tone_buf_32, samples_to_generate * 2 * sizeof(int32_t), &bytes_written, portMAX_DELAY);
#endif
  }
  Serial.println("[TEST TONE] Tone completed.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==============================================");
  Serial.println("  I2S Hardware Test: INMP441 Mic & MAX98357A Speaker");
  Serial.println("==============================================");
  
#ifdef I2S_NUM_1
  Serial.println("[TARGET] Standard ESP32 Target (Dual I2S Ports)");
  Serial.printf("  Mic Pins    -> BCLK: %d, WS: %d, SD: %d\n", I2S_RX_BCLK_PIN, I2S_RX_LRC_PIN, I2S_RX_DOUT_PIN);
  Serial.printf("  Speaker Pins-> BCLK: %d, WS: %d, DIN: %d\n", I2S_TX_BCLK_PIN, I2S_TX_LRC_PIN, I2S_TX_DIN_PIN);
#else
  Serial.println("[TARGET] ESP32-C3 Target (ETH01 EVO / Single I2S Port)");
  Serial.printf("  Mic Pins    -> BCLK: %d, WS: %d, SD: %d\n", I2S_RX_BCLK_PIN, I2S_RX_LRC_PIN, I2S_RX_DOUT_PIN);
  Serial.printf("  Speaker Pins-> BCLK: %d, WS: %d, DIN: %d\n", I2S_TX_BCLK_PIN, I2S_TX_LRC_PIN, I2S_TX_DIN_PIN);
#endif

  Serial.println("\nCommands in Serial Monitor:");
  Serial.println("  't' -> Play 1-second 440 Hz test beep tone");
  Serial.println("  'l' -> Toggle live mic-to-speaker loopback ON/OFF");
  Serial.println("==============================================\n");

  setup_i2s();
  
  // Play startup beep tone to confirm speaker is working immediately!
  play_test_tone(440.0, 1000);
}

unsigned long last_meter_time = 0;

void loop() {
  // Check Serial Commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 't' || cmd == 'T') {
      play_test_tone(440.0, 1000);
    } else if (cmd == 'l' || cmd == 'L') {
      loopback_enabled = !loopback_enabled;
      Serial.printf("[LOOPBACK] Live Passthrough is now %s\n", loopback_enabled ? "ENABLED" : "DISABLED");
    }
  }

  // Read Audio from INMP441 Microphone
  int32_t raw_samples[256]; // 128 Left + 128 Right
  size_t bytes_read = 0;
  i2s_read(I2S_PORT_RX, raw_samples, sizeof(raw_samples), &bytes_read, portMAX_DELAY);

  if (bytes_read > 0) {
    int num_stereo_samples = bytes_read / sizeof(int32_t);
    int num_mono_samples = num_stereo_samples / 2;

    int16_t pcm_buf[128];
    int32_t pcm_max = 0;

    for (int i = 0; i < num_mono_samples && i < 128; i++) {
      // INMP441 outputs 24-bit data MSB-aligned in a 32-bit slot
      int32_t left_sample  = raw_samples[i * 2] >> 16;
      int32_t right_sample = raw_samples[i * 2 + 1] >> 16;
      
      // Select active channel sample (whether L/R pin is wired to GND or VCC)
      int32_t sample32 = (abs(left_sample) > abs(right_sample)) ? left_sample : right_sample;
      
      sample32 = sample32 * 8; // 8x digital gain
      if (sample32 > 32767) sample32 = 32767;
      if (sample32 < -32768) sample32 = -32768;

      pcm_buf[i] = (int16_t)sample32;
      int32_t sample_mag = abs((int)pcm_buf[i]);
      if (sample_mag > pcm_max) {
        pcm_max = sample_mag;
      }
    }

    // Print live Volume Bar Graph every 100ms
    if (millis() - last_meter_time > 100) {
      last_meter_time = millis();
      int bars = (pcm_max * 30) / 32768;
      String meter = "[";
      for (int b = 0; b < 30; b++) {
        meter += (b < bars) ? "#" : " ";
      }
      meter += "]";
      uint32_t raw_debug = (uint32_t)raw_samples[0];
      Serial.printf("[MIC] %s Peak: %5d  (Raw0: 0x%08X, Loopback: %s)\n", 
                    meter.c_str(), pcm_max, raw_debug, loopback_enabled ? "ON" : "OFF");
    }

    // Live Audio Passthrough to Speaker
    if (loopback_enabled) {
      size_t bytes_written = 0;
#ifdef I2S_NUM_1
      i2s_write(I2S_PORT_TX, pcm_buf, num_mono_samples * sizeof(int16_t), &bytes_written, 0);
#else
      int32_t out_32[128 * 2];
      for (int i = 0; i < num_mono_samples && i < 128; i++) {
        int32_t v = ((int32_t)pcm_buf[i]) << 16;
        out_32[i * 2]     = v;
        out_32[i * 2 + 1] = v;
      }
      i2s_write(I2S_PORT_TX, out_32, num_mono_samples * 2 * sizeof(int32_t), &bytes_written, 0);
#endif
    }
  }
}
