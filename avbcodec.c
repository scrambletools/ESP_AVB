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
#include "es8388_codec.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

#define I2C_NUM (0)
#define AVB_MCLK_MULTIPLE                                                      \
  (384) // If not using 24-bit data width, 256 should be enough

#define TAG "AVB-CODEC"

static const avb_codec_caps_s s_es8311_caps = {
    .sample_rates = {.sample_rates = {48000, 96000}, .num_rates = 2},
    .bit_rates = {.bit_rates = {24}, .num_rates = 1},
    .max_input_channels = 1,
    .max_output_channels = 1,
    .control_ranges = {.vol_min_tenth_db = -955,
                       .vol_max_tenth_db = 320,
                       .vol_step_tenth_db = 5,
                       .vol_default_tenth_db = 100,
                       .gain_min_tenth_db = 0,
                       .gain_max_tenth_db = 420,
                       .gain_step_tenth_db = 60,
                       .gain_default_tenth_db = 60},
};

static const avb_codec_caps_s s_es8388_caps = {
    .sample_rates = {.sample_rates = {48000, 96000, 192000}, .num_rates = 3},
    .bit_rates = {.bit_rates = {24}, .num_rates = 1},
    .max_input_channels = 2,
    .max_output_channels = 2,
    .control_ranges = {.vol_min_tenth_db = -960,
                       .vol_max_tenth_db = 0,
                       .vol_step_tenth_db = 5,
                       .vol_default_tenth_db = -100,
                       .gain_min_tenth_db = 0,
                       .gain_max_tenth_db = 240,
                       .gain_step_tenth_db = 30,
                       .gain_default_tenth_db = 90},
};

int16_t avb_codec_quantize_tenth_db(const codec_control_range_s *ranges,
                                     bool gain, int16_t value_tenth_db) {
  int16_t min = gain ? ranges->gain_min_tenth_db : ranges->vol_min_tenth_db;
  int16_t max = gain ? ranges->gain_max_tenth_db : ranges->vol_max_tenth_db;
  int16_t step = gain ? ranges->gain_step_tenth_db : ranges->vol_step_tenth_db;

  if (value_tenth_db <= min) {
    return min;
  }
  if (value_tenth_db >= max) {
    return max;
  }
  if (step <= 0) {
    return value_tenth_db;
  }

  int32_t offset = value_tenth_db - min;
  int32_t steps = (offset + (step / 2)) / step;
  int32_t quantized = min + (steps * step);
  if (quantized < min) {
    quantized = min;
  } else if (quantized > max) {
    quantized = max;
  }
  return (int16_t)quantized;
}

const avb_codec_caps_s *avb_codec_get_caps(avb_codec_type_t codec_type) {
  switch (codec_type) {
  case avb_codec_type_es8311:
    return &s_es8311_caps;
  case avb_codec_type_es8388:
    return &s_es8388_caps;
  default:
    return NULL;
  }
}

static bool codec_caps_support_sample_rate(const avb_codec_caps_s *caps,
                                           uint32_t sample_rate) {
  for (uint8_t i = 0; i < caps->sample_rates.num_rates; i++) {
    if (caps->sample_rates.sample_rates[i] == sample_rate)
      return true;
  }
  return false;
}

/* Configure the I2S driver
 * Typically the I2S driver must be reconfigured when the stream params change
 *
 * @param state: AVB state
 */
esp_err_t avb_config_i2s(avb_state_s *state) {

  // Create an I2S channel and set the handles in the state
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(state->config.i2s_port, I2S_ROLE_MASTER);
  /* Auto-clear stale DMA content so the DAC emits silence on underrun
   * rather than a loop of old samples. */
  chan_cfg.auto_clear = true;
  /* Small DMA descriptors (6 frames = 1 AAF Class A packet worth,
   * 125 µs at 48 kHz). 16 of them = 2 ms TX ring — Milan Class A's
   * max_transit_time window. The listener drain in avtp.c runs as an
   * esp_timer 1 ms task that calls i2s_channel_write, so the I2S
   * driver paces playout at the DAC rate without needing on_sent
   * callback bookkeeping. */
  chan_cfg.dma_frame_num = 6;
  chan_cfg.dma_desc_num = 16;
  ESP_ERROR_CHECK(
      i2s_new_channel(&chan_cfg, &state->i2s_tx_handle, &state->i2s_rx_handle));
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(state->config.default_sample_rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
          state->config.default_bits_per_sample, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = state->config.codec_pins.mclk,
              .bclk = state->config.codec_pins.bclk,
              .ws = state->config.codec_pins.ws,
              .dout = state->config.codec_pins.dout,
              .din = state->config.codec_pins.din,
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
  /* Big-endian in-memory sample layout: byte[0]=MSB, byte[2]=LSB. This
   * matches AVTP wire order so the stream-in handler can memcpy AAF
   * payloads straight to the jitter ring with no per-sample shuffle. */
  std_cfg.slot_cfg.big_endian = true;

  // Initialize the I2S TX and RX channels
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_rx_handle, &std_cfg));

  // Enable the I2S TX and RX channels
  ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_rx_handle));

  /* Publish the effective listener-side rates so the PLL and the
   * stream-input drain no longer hardcode 48 kHz / 288000 B/s. The
   * drain always outputs stereo 24-bit (2 ch × 3 B/frame); only the
   * sample rate changes with config. */
  state->media_clock.listener_sample_rate = state->config.default_sample_rate;
  state->media_clock.listener_byterate =
      state->config.default_sample_rate * 2u * 3u;

  /* Initialise the media-clock PLL now that I2S (and hence APLL) is up */
  uint32_t nominal_mclk =
      state->config.default_sample_rate * AVB_MCLK_MULTIPLE;
  if (avb_pll_init(nominal_mclk) != 0) {
    avbwarn("PLL init failed (sample clock will free-run)");
  }

  avbinfo("I2S channels initialized");
  return ESP_OK;
}

/* Per-codec factory result: the chip-specific create step yields these. */
typedef struct {
  const audio_codec_if_t *codec_if;
} codec_factory_result_s;

static esp_err_t codec_factory_es8311(avb_state_s *state,
                                      i2c_master_bus_handle_t bus,
                                      const audio_codec_gpio_if_t *gpio_if,
                                      codec_factory_result_s *out) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(avb_codec_type_es8311);
  if (!codec_caps_support_sample_rate(caps, state->config.default_sample_rate)) {
    ESP_LOGE(TAG, "ES8311: unsupported sample rate %lu",
             state->config.default_sample_rate);
    return ESP_FAIL;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8311_CODEC_DEFAULT_ADDR,
                                   .bus_handle = bus};
  const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl) {
    ESP_LOGE(TAG, "ES8311: failed to create I2C control interface");
    return ESP_FAIL;
  }
  es8311_codec_cfg_t cfg = {
      .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
      .ctrl_if = ctrl,
      .gpio_if = gpio_if,
      .pa_pin = state->config.codec_pins.pa,
      .use_mclk = true,
      .mclk_div = AVB_MCLK_MULTIPLE,
  };
  out->codec_if = es8311_codec_new(&cfg);
  if (!out->codec_if) {
    ESP_LOGE(TAG, "ES8311: failed to create codec interface");
    return ESP_FAIL;
  }
  return ESP_OK;
}

static esp_err_t codec_factory_es8388(avb_state_s *state,
                                      i2c_master_bus_handle_t bus,
                                      const audio_codec_gpio_if_t *gpio_if,
                                      codec_factory_result_s *out) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(avb_codec_type_es8388);
  if (!codec_caps_support_sample_rate(caps, state->config.default_sample_rate)) {
    ESP_LOGE(TAG, "ES8388: unsupported sample rate %lu",
             state->config.default_sample_rate);
    return ESP_FAIL;
  }
  audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8388_CODEC_DEFAULT_ADDR,
                                   .bus_handle = bus};
  const audio_codec_ctrl_if_t *ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);
  if (!ctrl) {
    ESP_LOGE(TAG, "ES8388: failed to create I2C control interface");
    return ESP_FAIL;
  }
  es8388_codec_cfg_t cfg = {
      .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
      .ctrl_if = ctrl,
      .gpio_if = gpio_if,
      .pa_pin = state->config.codec_pins.pa,
      .master_mode = false,
  };
  out->codec_if = es8388_codec_new(&cfg);
  if (!out->codec_if) {
    ESP_LOGE(TAG, "ES8388: failed to create codec interface");
    return ESP_FAIL;
  }
  return ESP_OK;
}

/* Configure the codec selected by state->config.codec_type.
 *
 * Generic shell handles I2C bus + GPIO + sample-format/enable + AECP control
 * range loading. Per-codec factory builds the chip-specific cfg struct and
 * calls the matching *_codec_new(). Both codecs are then driven through the
 * codec-agnostic audio_codec_if_t vtable.
 *
 * Bypasses esp_codec_dev_open() because that reconfigures I2S (disable/
 * re-enable) and can misalign BCLK phase — corrupting the lower bits of
 * 24-bit captures. Talker/listener use i2s_channel_read/write directly,
 * so no audio_codec_data_if is needed here.
 */
esp_err_t avb_config_codec(avb_state_s *state) {
  const avb_codec_caps_s *caps = avb_codec_get_caps(state->config.codec_type);
  if (!caps) {
    ESP_LOGE(TAG, "Unsupported codec type: %d", state->config.codec_type);
    return ESP_FAIL;
  }
  if (state->config.input_channels_usable > caps->max_input_channels ||
      state->config.output_channels_usable > caps->max_output_channels) {
    ESP_LOGE(TAG, "Unsupported channel count: %d in, %d out (caps %u/%u)",
             state->config.input_channels_usable,
             state->config.output_channels_usable, caps->max_input_channels,
             caps->max_output_channels);
    return ESP_FAIL;
  }
  if (state->supported_bits_per_sample.num_rates == 0) {
    ESP_LOGE(TAG, "No effective codec bit-depth capability");
    return ESP_FAIL;
  }

  i2c_master_bus_handle_t bus;
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_NUM,
      .sda_io_num = state->config.codec_pins.i2c_sda,
      .scl_io_num = state->config.codec_pins.i2c_scl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus), TAG,
                      "create I2C master bus failed");

  const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

  codec_factory_result_s result = {0};
  esp_err_t err;
  switch (state->config.codec_type) {
  case avb_codec_type_es8311:
    err = codec_factory_es8311(state, bus, gpio_if, &result);
    break;
  case avb_codec_type_es8388:
    err = codec_factory_es8388(state, bus, gpio_if, &result);
    break;
  default:
    ESP_LOGE(TAG, "Unsupported codec type: %d", state->config.codec_type);
    return ESP_FAIL;
  }
  if (err != ESP_OK) {
    return err;
  }

  esp_codec_dev_sample_info_t fs = {
      .bits_per_sample = state->config.default_bits_per_sample,
      .channel = 2,
      .sample_rate = state->config.default_sample_rate,
      .mclk_multiple = AVB_MCLK_MULTIPLE,
  };
  if (result.codec_if->set_fs &&
      result.codec_if->set_fs(result.codec_if, &fs) != 0) {
    ESP_LOGE(TAG, "Failed to set codec sample format");
    return ESP_FAIL;
  }
  if (result.codec_if->enable &&
      result.codec_if->enable(result.codec_if, true) != 0) {
    ESP_LOGE(TAG, "Failed to enable codec");
    return ESP_FAIL;
  }
  state->codec_enabled = true;
  state->codec_if = result.codec_if;

  state->codec_ranges = caps->control_ranges;
  state->codec_ranges.vol_default_tenth_db = avb_codec_quantize_tenth_db(
      &state->codec_ranges, false, state->config.default_speaker_vol_tenth_db);
  state->codec_ranges.gain_default_tenth_db = avb_codec_quantize_tenth_db(
      &state->codec_ranges, true, state->config.default_mic_gain_tenth_db);
  state->ctrl_speaker_vol = state->codec_ranges.vol_default_tenth_db / 10.0f;
  state->ctrl_mic_gain = state->codec_ranges.gain_default_tenth_db / 10.0f;

  if (result.codec_if->set_vol) {
    result.codec_if->set_vol(result.codec_if, state->ctrl_speaker_vol);
  }
  if (result.codec_if->set_mic_gain) {
    result.codec_if->set_mic_gain(result.codec_if, state->ctrl_mic_gain);
  }

  ESP_LOGI(TAG, "Codec configured and enabled (ADC+DAC active)");
  return ESP_OK;
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
