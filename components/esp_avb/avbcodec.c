/*
 * Copyright 2025 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 * 
 * This file provides the codec interface for the ESP_AVB component.
 */

#include "avb.h"

/* Default settings */
#define AVB_RECV_BUF_SIZE   (2400)
#define AVB_SAMPLE_RATE     (48000)
#define AVB_BITS_PER_SAMPLE (24)
#define AVB_MCLK_MULTIPLE   (384) // If not using 24-bit data width, 256 should be enough

/* I2C port and GPIOs */
#define I2C_NUM         (0)
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define I2C_SCL_IO      (GPIO_NUM_14)
#define I2C_SDA_IO      (GPIO_NUM_15)
#define GPIO_OUTPUT_PA  (GPIO_NUM_46)
#elif CONFIG_IDF_TARGET_ESP32H2
#define I2C_SCL_IO      (GPIO_NUM_8)
#define I2C_SDA_IO      (GPIO_NUM_9)
#elif CONFIG_IDF_TARGET_ESP32P4
#define I2C_SCL_IO      (GPIO_NUM_8)
#define I2C_SDA_IO      (GPIO_NUM_7)
#define GPIO_OUTPUT_PA  (GPIO_NUM_53)
#else
#define I2C_SCL_IO      (GPIO_NUM_6)
#define I2C_SDA_IO      (GPIO_NUM_7)
#endif
#define I2C_FREQ_HZ     (100000)
#define I2C_CODEC_ADDR  (0x18u)

/* I2S port and GPIOs */
#define I2S_NUM         (0)
#if CONFIG_IDF_TARGET_ESP32P4
#define I2S_MCK_IO      (GPIO_NUM_13)
#define I2S_BCK_IO      (GPIO_NUM_12)
#define I2S_WS_IO       (GPIO_NUM_10)
#define I2S_DO_IO       (GPIO_NUM_9)
#define I2S_DI_IO       (GPIO_NUM_11)
#else
#define I2S_MCK_IO      (GPIO_NUM_16)
#define I2S_BCK_IO      (GPIO_NUM_9)
#define I2S_WS_IO       (GPIO_NUM_45)
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define I2S_DO_IO       (GPIO_NUM_8)
#define I2S_DI_IO       (GPIO_NUM_10)
#else
#define I2S_DO_IO       (GPIO_NUM_2)
#define I2S_DI_IO       (GPIO_NUM_3)
#endif
#endif

/* I2C address for ES8311 codec */
#define ES8311_CODEC_ADDR (0x18u)

#define TAG "AVB-CODEC"

/* Configure the I2S driver 
* Typically the I2S driver must be reconfigured when the stream params change
* 
* @param state: AVB state
*/
esp_err_t avb_config_i2s(avb_state_s *state) {

    // Create an I2S channel and set the handles in the state
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(
        state->config.i2s_port, 
        I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &state->i2s_tx_handle, &state->i2s_rx_handle));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(state->config.default_sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(state->config.default_bits_per_sample, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = AVB_MCLK_MULTIPLE;

    // Initialize and enable the I2S TX and RX channels
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(state->i2s_rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(state->i2s_rx_handle));

    avbinfo("I2S channels initialized");
    return ESP_OK;
}

/* Configure ES8311 codec */
static esp_err_t avb_config_codec_es8311(avb_state_s *state) {

    // Check for valid number of channels
    if (state->config.num_channels_input != 1 || state->config.num_channels_output != 1) {
        ESP_LOGE("ES8311", "Unsupported number of channels: %d in, %d out", state->config.num_channels_input, state->config.num_channels_output);
        return ESP_FAIL;
    }

    // Check for valid bits per sample
    if (state->config.default_bits_per_sample != 16 && state->config.default_bits_per_sample != 24) {
        ESP_LOGE("ES8311", "Unsupported bits per sample: %d", state->config.default_bits_per_sample);
        return ESP_FAIL;
    }

    // Check for valid sample rate
    if (state->config.default_sample_rate != 44100 && state->config.default_sample_rate != 48000 && state->config.default_sample_rate != 96000) {
        ESP_LOGE("ES8311", "Unsupported sample rate: %lu", state->config.default_sample_rate);
        return ESP_FAIL;
    }


    // Setup the I2C bus handle using older i2c API which works with es8311 component
    const i2c_config_t es_i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_NUM, &es_i2c_cfg), TAG, "config i2c failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0), TAG, "install i2c driver failed");

    /* Initialize es8311 codec */
    es8311_handle_t es_handle = es8311_create(I2C_NUM, ES8311_ADDRRES_0);
    ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");

    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = state->config.default_sample_rate * AVB_MCLK_MULTIPLE,
        .sample_frequency = state->config.default_sample_rate
    };

    ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, state->config.default_bits_per_sample, state->config.default_bits_per_sample));
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, state->config.default_sample_rate * AVB_MCLK_MULTIPLE, state->config.default_sample_rate), TAG, "set es8311 sample frequency failed");

    // Setup the GPIO for the PA and set it to high
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PA),
        .mode = GPIO_MODE_OUTPUT,      
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE  
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_OUTPUT_PA, 1);

    return ESP_OK;
}

/* Configure ES8311 codec using ESP_CODEC_DEV 
 * but it's currently failing on open with I2C errors
 */
static esp_err_t avb_config_codec_es8311_ng(avb_state_s *state) {

    // Check for valid number of channels
    if (state->config.num_channels_input != 1 || state->config.num_channels_output != 1) {
        ESP_LOGE("ES8311", "Unsupported number of channels: %d in, %d out", state->config.num_channels_input, state->config.num_channels_output);
        return ESP_FAIL;
    }

    // Check for valid bits per sample
    if (state->config.default_bits_per_sample != 16 && state->config.default_bits_per_sample != 24) {
        ESP_LOGE("ES8311", "Unsupported bits per sample: %d", state->config.default_bits_per_sample);
        return ESP_FAIL;
    }

    // Check for valid sample rate
    if (state->config.default_sample_rate != 44100 && state->config.default_sample_rate != 48000 && state->config.default_sample_rate != 96000) {
        ESP_LOGE("ES8311", "Unsupported sample rate: %lu", state->config.default_sample_rate);
        return ESP_FAIL;
    }

    // Setup the data interface for the codec
    audio_codec_i2s_cfg_t i2s_cfg = {
        .rx_handle = state->i2s_rx_handle,
        .tx_handle = state->i2s_tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!data_if) {
        ESP_LOGE("ES8311", "Failed to create codec data interface");
        return ESP_FAIL;
    }

    // Setup the I2C bus handle
    i2c_master_bus_handle_t i2c_bus_handle;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = 0,
        .sda_io_num = GPIO_NUM_7,
        .scl_io_num = GPIO_NUM_8,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    // Setup the control interface for the codec
    audio_codec_i2c_cfg_t i2c_cfg = {
        .addr = ES8311_CODEC_ADDR,
        .bus_handle = i2c_bus_handle
    };
    const audio_codec_ctrl_if_t *out_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!out_ctrl_if) {
        ESP_LOGE("ES8311", "Failed to create codec control interface");
        return ESP_FAIL;
    }
    ESP_LOGI("ES8311", "Codec control interface created");

    // Setup the GPIO interface for the codec
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    // Setup the ES8311 codec interface
    es8311_codec_cfg_t es8311_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .ctrl_if = out_ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = state->config.output_pa_pin,
        .use_mclk = true,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);

    ESP_LOGI("ES8311", "Codec interface created");

    // Create the codec device handle and store it in the state
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = codec_if,                  // codec interface from es8311_codec_new
        .data_if = data_if,                    // data interface from audio_codec_new_i2s_data
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT, // codec support both playback and record
    };
    esp_codec_dev_handle_t codec_dev = esp_codec_dev_new(&dev_cfg);
    state->config.codec_handle = codec_dev;

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
