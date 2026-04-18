/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the codec interface for the ESP_AVB component.
 */

#include "avb.h"
#include "es8311_codec.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

/* Default settings */
#define AVB_RECV_BUF_SIZE (2400)
#define AVB_SAMPLE_RATE (48000)
#define AVB_BITS_PER_SAMPLE (24)
#define AVB_MCLK_MULTIPLE                                                      \
  (384) // If not using 24-bit data width, 256 should be enough

/* I2C port and GPIOs */
#define I2C_NUM (0)
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 ||                    \
    CONFIG_IDF_TARGET_ESP32S3
#define I2C_SCL_IO (GPIO_NUM_14)
#define I2C_SDA_IO (GPIO_NUM_15)
#define GPIO_OUTPUT_PA (GPIO_NUM_46)
#elif CONFIG_IDF_TARGET_ESP32H2
#define I2C_SCL_IO (GPIO_NUM_8)
#define I2C_SDA_IO (GPIO_NUM_9)
#elif CONFIG_IDF_TARGET_ESP32P4
#define I2C_SCL_IO (GPIO_NUM_8)
#define I2C_SDA_IO (GPIO_NUM_7)
#define GPIO_OUTPUT_PA (GPIO_NUM_53)
#else
#define I2C_SCL_IO (GPIO_NUM_6)
#define I2C_SDA_IO (GPIO_NUM_7)
#endif
#define I2C_FREQ_HZ (100000)
#define I2C_CODEC_ADDR (0x18u)

/* I2S port and GPIOs */
#define I2S_NUM (0)
#if CONFIG_IDF_TARGET_ESP32P4
#define I2S_MCK_IO (GPIO_NUM_13)
#define I2S_BCK_IO (GPIO_NUM_12)
#define I2S_WS_IO (GPIO_NUM_10)
#define I2S_DO_IO (GPIO_NUM_9)
#define I2S_DI_IO (GPIO_NUM_11)
#else
#define I2S_MCK_IO (GPIO_NUM_16)
#define I2S_BCK_IO (GPIO_NUM_9)
#define I2S_WS_IO (GPIO_NUM_45)
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 ||                    \
    CONFIG_IDF_TARGET_ESP32S3
#define I2S_DO_IO (GPIO_NUM_8)
#define I2S_DI_IO (GPIO_NUM_10)
#else
#define I2S_DO_IO (GPIO_NUM_2)
#define I2S_DI_IO (GPIO_NUM_3)
#endif
#endif

/* I2C address for ES8311 codec (8-bit format: esp_codec_dev right-shifts
 * internally) */
#define ES8311_CODEC_ADDR ES8311_CODEC_DEFAULT_ADDR

#define TAG "AVB-CODEC"

/* Configure the I2S driver
 * Typically the I2S driver must be reconfigured when the stream params change
 *
 * @param state: AVB state
 */
esp_err_t avb_config_i2s(avb_state_s *state) {

  // Create an I2S channel and set the handles in the state
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(state->config.i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  // Align DMA frames with AVTP Class A packet size (6 samples @ 48kHz = 125μs).
  // This ensures i2s_channel_read returns data aligned with the talker's
  // 125μs busy-wait loop, and the listener's drain timer works with small
  // chunks.
  chan_cfg.dma_frame_num = 6; // 6 samples = 1 AVTP packet worth
  chan_cfg.dma_desc_num = 16; // 16 descriptors = 2ms buffering
  ESP_ERROR_CHECK(
      i2s_new_channel(&chan_cfg, &state->i2s_tx_handle, &state->i2s_rx_handle));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(state->config.default_sample_rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          state->config.default_bits_per_sample, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = I2S_MCK_IO,
              .bclk = I2S_BCK_IO,
              .ws = I2S_WS_IO,
              .dout = I2S_DO_IO,
              .din = I2S_DI_IO,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  std_cfg.clk_cfg.mclk_multiple = AVB_MCLK_MULTIPLE;
  /* Use APLL as the clock source so the Milan media-clock PLL
   * (avb_mclk / avb_mclk_apll) can retune MCLK with sub-ppm precision
   * without having to disable/reconfigure the I2S channel. */
  std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;

  // Initialize and enable the I2S TX and RX channels
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_rx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_rx_handle));

  /* Initialise the media-clock PLL now that I2S (and hence APLL) is up */
  uint32_t nominal_mclk =
      state->config.default_sample_rate * AVB_MCLK_MULTIPLE;
  if (avb_pll_init(nominal_mclk) != 0) {
    avbwarn("PLL init failed (sample clock will free-run)");
  }

  avbinfo("I2S channels initialized");
  return ESP_OK;
}

/* Configure ES8311 codec using esp_codec_dev component
 *
 * Uses the new I2C master API and es8311_codec_new() interface
 * compatible with ESP-IDF 6.x.
 */
static esp_err_t avb_config_codec_es8311(avb_state_s *state) {

  // Check for valid number of channels
  if (state->config.num_channels_input != 1 ||
      state->config.num_channels_output != 1) {
    ESP_LOGE("ES8311", "Unsupported number of channels: %d in, %d out",
             state->config.num_channels_input,
             state->config.num_channels_output);
    return ESP_FAIL;
  }

  // Check for valid bits per sample
  if (state->config.default_bits_per_sample != 16 &&
      state->config.default_bits_per_sample != 24) {
    ESP_LOGE("ES8311", "Unsupported bits per sample: %d",
             state->config.default_bits_per_sample);
    return ESP_FAIL;
  }

  // Check for valid sample rate
  if (state->config.default_sample_rate != 44100 &&
      state->config.default_sample_rate != 48000 &&
      state->config.default_sample_rate != 96000) {
    ESP_LOGE("ES8311", "Unsupported sample rate: %lu",
             state->config.default_sample_rate);
    return ESP_FAIL;
  }

  // Setup the I2C master bus
  i2c_master_bus_handle_t i2c_bus_handle;
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM,
      .sda_io_num = I2C_SDA_IO,
      .scl_io_num = I2C_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus_handle), TAG,
                      "create I2C master bus failed");

  // Setup the control interface for the codec
  audio_codec_i2c_cfg_t i2c_cfg = {
      .addr = ES8311_CODEC_ADDR,
      .bus_handle = i2c_bus_handle,
  };
  const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl_if) {
    ESP_LOGE("ES8311", "Failed to create codec control interface");
    return ESP_FAIL;
  }

  // Setup the GPIO interface for the codec (handles PA pin)
  const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

  // Setup the ES8311 codec interface
  es8311_codec_cfg_t es8311_cfg = {
      .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
      .ctrl_if = ctrl_if,
      .gpio_if = gpio_if,
      .pa_pin = state->config.output_pa_pin,
      .use_mclk = true,
      .mclk_div = AVB_MCLK_MULTIPLE,
  };
  const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
  if (!codec_if) {
    ESP_LOGE("ES8311", "Failed to create ES8311 codec interface");
    return ESP_FAIL;
  }

  /* Configure and start the codec directly — bypass esp_codec_dev_open()
   * which reconfigures I2S (disable/re-enable cycle) and can misalign the
   * BCLK phase, causing only the top 8 bits of 24-bit audio to be captured.
   *
   * Since both talker and listener use i2s_channel_read/write directly
   * (not esp_codec_dev_read/write), no I2S data interface is needed. */
  esp_codec_dev_sample_info_t fs = {
      .bits_per_sample = state->config.default_bits_per_sample,
      .channel = 2,
      .sample_rate = state->config.default_sample_rate,
      .mclk_multiple = AVB_MCLK_MULTIPLE,
  };
  if (codec_if->set_fs && codec_if->set_fs(codec_if, &fs) != 0) {
    ESP_LOGE("ES8311", "Failed to set codec sample format");
    return ESP_FAIL;
  }
  if (codec_if->enable && codec_if->enable(codec_if, true) != 0) {
    ESP_LOGE("ES8311", "Failed to enable codec");
    return ESP_FAIL;
  }
  state->codec_enabled = true;
  state->codec_if = codec_if;

  /* Initialize AECP control values and ranges from codec-specific defaults */
  state->codec_ranges = (codec_control_range_s)ES8311_CONTROL_RANGES;
  state->ctrl_speaker_vol = state->codec_ranges.vol_default_tenth_db / 10.0f;
  state->ctrl_mic_gain = state->codec_ranges.gain_default_tenth_db / 10.0f;

  /* Apply initial volume and gain from control defaults */
  if (codec_if->set_vol) {
    codec_if->set_vol(codec_if, state->ctrl_speaker_vol);
  }
  if (codec_if->set_mic_gain) {
    codec_if->set_mic_gain(codec_if, state->ctrl_mic_gain);
  }

  ESP_LOGI("ES8311", "Codec configured and opened (ADC+DAC active)");
  return ESP_OK;
}

/* Configure the CODEC
 * (currently only ES8311 codec is supported)
 *
 * @param state: AVB state
 */
esp_err_t avb_config_codec(avb_state_s *state) {
  switch (state->config.codec_type) {
  case avb_codec_type_es8311:
    return avb_config_codec_es8311(state);
  default:
    ESP_LOGE("AVB", "Unsupported codec type: %d", state->config.codec_type);
    return ESP_FAIL;
  }
  ESP_LOGI("AVB", "Codec configured");
}

/* Set speaker volume via codec interface */
void avb_codec_set_vol(avb_state_s *state, float db) {
  const audio_codec_if_t *codec = (const audio_codec_if_t *)state->codec_if;
  if (codec && codec->set_vol) {
    codec->set_vol(codec, db);
  }
}

/* Set mic gain via codec interface */
void avb_codec_set_mic_gain(avb_state_s *state, float db) {
  const audio_codec_if_t *codec = (const audio_codec_if_t *)state->codec_if;
  if (codec && codec->set_mic_gain) {
    codec->set_mic_gain(codec, db);
  }
}
