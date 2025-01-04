/* 
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * AVB Example Application
 *
 * This application demonstrates the use of the ESP_AVB component 
 * to create an AVB talker and/or listener.
 */

#include <string.h>
#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_eth.h>
#include <esp_netif.h>
#include <esp_check.h>
#include <ethernet_init.h>
#include <esp_vfs_l2tap.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <ptpd.h>
#include <esp_eth_time.h>
#include "esp_avb.h"
#include "es8311.h" // audio codec used for this example

/* Example configurations */
#define EXAMPLE_RECV_BUF_SIZE   (2400)
#define EXAMPLE_SAMPLE_RATE     (48000)
#define EXAMPLE_MCLK_MULTIPLE   (384) // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_FREQ_HZ    (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)
#if CONFIG_EXAMPLE_AVB_LISTENER
#define EXAMPLE_SPEAKER_VOLUME  CONFIG_EXAMPLE_AVB_SPEAKER_VOLUME
#endif
#if CONFIG_EXAMPLE_AVB_TALKER
#define EXAMPLE_MIC_GAIN        CONFIG_EXAMPLE_AVB_MIC_GAIN
#endif

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

static const char *TAG = "avb_example";
static struct timespec s_next_time;
static bool s_gpio_level;

static const char err_reason[][30] = {"input param is invalid",
                                      "operation timeout"};
i2s_chan_handle_t tx_handle = NULL;
i2s_chan_handle_t rx_handle = NULL;

typedef struct {
    char name[16];
    TaskHandle_t handle;
} task_info_t;

/* Import music file as buffer */
#if CONFIG_EXAMPLE_AVB_TALKER
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");
#endif

static void gpio_init(void)
{
    /* Initialize the output pin for PA */
    gpio_config_t gpio_pa_output_cfg = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PA), // Select GPIO48
        .mode = GPIO_MODE_OUTPUT,                 // Set as output mode
        .pull_up_en = GPIO_PULLUP_DISABLE,        // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE            // Disable interrupt
    };
    gpio_config(&gpio_pa_output_cfg);
    gpio_set_level(GPIO_OUTPUT_PA, 1); // set high

    /* Initialize the output pin for time pulse indicator LED */
    gpio_config_t gpio_pulse_led_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_EXAMPLE_AVB_PULSE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_pulse_led_cfg);
    gpio_set_level(CONFIG_EXAMPLE_AVB_PULSE_GPIO, 0); // set low
}

static esp_err_t es8311_codec_init(void)
{
    /* Initialize I2C peripheral */
    const i2c_config_t es_i2c_cfg = {
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
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
        .mclk_frequency = EXAMPLE_MCLK_FREQ_HZ,
        .sample_frequency = EXAMPLE_SAMPLE_RATE
    };

    ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE, EXAMPLE_SAMPLE_RATE), TAG, "set es8311 sample frequency failed");
#if CONFIG_EXAMPLE_AVB_LISTENER
    ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, (uint8_t)EXAMPLE_SPEAKER_VOLUME, NULL), TAG, "set es8311 volume failed");
#endif
#if CONFIG_EXAMPLE_AVB_TALKER
    ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");
    ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG, "set es8311 microphone gain failed");
#endif
    return ESP_OK;
}

static esp_err_t i2s_driver_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
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
    std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    return ESP_OK;
}

static void i2s_to_codec(void *args)
{
    int *stream_in_data = malloc(EXAMPLE_RECV_BUF_SIZE);
    if (!stream_in_data) {
        ESP_LOGE(TAG, "[i2s] No memory for read data buffer");
        abort();
    }
    esp_err_t ret = ESP_OK;
    size_t bytes_read = 0;
    size_t bytes_write = 0;
    ESP_LOGI(TAG, "[i2s] Echo start");

    while (1) {
        memset(stream_in_data, 0, EXAMPLE_RECV_BUF_SIZE);
        /* Read sample data from stream in */
        // TBD

        /* Write sample data to earphone */
        ret = i2s_channel_write(tx_handle, stream_in_data, EXAMPLE_RECV_BUF_SIZE, &bytes_write, 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[i2s] i2s write failed, %s", err_reason[ret == ESP_ERR_TIMEOUT]);
            abort();
        }
        if (bytes_read != bytes_write) {
            ESP_LOGW(TAG, "[i2s] %d bytes read but only %d bytes are written", bytes_read, bytes_write);
        }
    }
    vTaskDelete(NULL);
}

void init_ethernet_and_netif(void)
{
    uint8_t eth_port_cnt;
    esp_eth_handle_t *eth_handles;

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_eth_init(&eth_handles, &eth_port_cnt));
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_vfs_l2tap_intf_register(NULL));

    esp_netif_inherent_config_t esp_netif_base_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
    esp_netif_config_t esp_netif_config = {
        .base = &esp_netif_base_config,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH
    };
    char if_key_str[10];
    char if_desc_str[10];
    char num_str[3];
    for (int i = 0; i < eth_port_cnt; i++) {
        itoa(i, num_str, 10);
        strcat(strcpy(if_key_str, "ETH_"), num_str);
        strcat(strcpy(if_desc_str, "eth"), num_str);
        esp_netif_base_config.if_key = if_key_str;
        esp_netif_base_config.if_desc = if_desc_str;
        esp_netif_base_config.route_prio -= i*5;
        esp_netif_t *eth_netif = esp_netif_new(&esp_netif_config);

        // attach Ethernet driver to TCP/IP stack
        ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handles[i])));
    }
    // Start Ethernet
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
    }
}

/* Callback function to toggle the output pin for time pulse indicator LED */
IRAM_ATTR bool ts_callback(esp_eth_mediator_t *eth, void *user_args)
{
    gpio_set_level(CONFIG_EXAMPLE_AVB_PULSE_GPIO, s_gpio_level ^= 1);

    // Set the next target time
    struct timespec interval = {
        .tv_sec = 0,
        .tv_nsec = CONFIG_EXAMPLE_AVB_PULSE_WIDTH_NS
    };
    timespecadd(&s_next_time, &interval, &s_next_time);

    struct timespec curr_time;
    esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &curr_time);

    // check that the next time is in the future
    if (timespeccmp(&s_next_time, &curr_time, >)) {
        esp_eth_clock_set_target_time(CLOCK_PTP_SYSTEM, &s_next_time);
    }
    return false;
}

void app_main(void)
{
    struct timespec cur_time;

    /* Initialize GPIOs */
    gpio_init();

    /* Initialize i2s interface to codec */
    if (i2s_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2S: driver init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "I2S: driver init success");
    }

    /* Initialize i2c peripheral and config es8311 codec by i2c */
    if (es8311_codec_init() != ESP_OK) {
        ESP_LOGE(TAG, "es8311 codec init failed");
        abort();
    } else {
        ESP_LOGI(TAG, "es8311 codec init success");
    }

    /* Play a piece of music if configured as talker */
#if CONFIG_EXAMPLE_AVB_TALKER
    xTaskCreate(i2s_to_codec, "i2s_to_codec", 4096, NULL, 5, NULL);
#endif

    /* Startup Ethernet */
    init_ethernet_and_netif();

    /* Start the PTP daemon task */
    int pid = ptpd_start("ETH_0");

    /* Wait for the clock to be available */
    while (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time) == -1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Set up AVB configuration */
    struct avb_config_s config = {
        .interface = "ETH_0",
#if CONFIG_EXAMPLE_AVB_TALKER
        .talker = true,
#else
        .talker = false,
#endif
#if CONFIG_EXAMPLE_AVB_LISTENER
        .listener = true,
#else
        .listener = false,
#endif
        .controller = false // net yet supported
    };

    /* Start the AVB task */
    avb_start(&config);

    /* Register callback function which will toggle output pin */
    esp_eth_clock_register_target_cb(CLOCK_PTP_SYSTEM, ts_callback);

    bool first_pass = true;
    bool clock_source_valid = false;
    bool clock_source_valid_last = false;
    int32_t clock_source_valid_cnt = 0;

    /* Task handles for memory consumption monitoring */
    static const uint period = 1000; // wait between checks in ms
    static const uint threshold = 1000; // size of high watermark
    char t0_name[] = "main_task";  // usually under 16 chars
    char t1_name[] = "AVB";
    char t2_name[] = "PTPD";
    TaskHandle_t t0 = xTaskGetHandle( t0_name );
    TaskHandle_t t1 = xTaskGetHandle( t1_name );
    TaskHandle_t t2 = xTaskGetHandle( t2_name );

    /* Main loop */
    while (1) {
        struct ptpd_status_s ptp_status;
        // if valid PTP status then increment the valid counter
        if (ptpd_status(pid, &ptp_status) == 0) {
            if (ptp_status.clock_source_valid) {
                clock_source_valid_cnt++;
            } else {
                clock_source_valid_cnt = 0;
            }
        // if PTP status is not valid then decrement the valid counter
        } else {
            if (clock_source_valid_cnt > 0) {
                clock_source_valid_cnt--;
            }
        }
        // consider the clock source valid only after n consequent intervals to be sure clock was synced
        if (clock_source_valid_cnt > 2) {
            clock_source_valid = true;
        } else {
            clock_source_valid = false;
        }
        // source validity changed => resync the pulse for ptp slave OR when the first pass to PTP master
        // starts generating its pulses
        if ((clock_source_valid == true && clock_source_valid_last == false) || first_pass) {
            first_pass = false;

            // get the current time (now synced)
            esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time);

            // compute the next pulse time
            s_next_time.tv_sec = 1;
            timespecadd(&s_next_time, &cur_time, &s_next_time);
            s_next_time.tv_nsec = CONFIG_EXAMPLE_AVB_PULSE_WIDTH_NS;

            // set the output pin (functionality disabled for now)
            // ESP_LOGI(TAG, "Starting Pulse train");
            // ESP_LOGI(TAG, "curr time: %llu.%09lu", cur_time.tv_sec, cur_time.tv_nsec);
            // ESP_LOGI(TAG, "next time: %llu.%09lu", s_next_time.tv_sec, s_next_time.tv_nsec);
            // s_gpio_level = 0;
            // gpio_set_level(CONFIG_EXAMPLE_AVB_PULSE_GPIO, s_gpio_level);

            // set the next pulse time
            esp_eth_clock_set_target_time(CLOCK_PTP_SYSTEM, &s_next_time);
        }
        else {
            ESP_LOGW(TAG, "PTP clock not synced (count %ld)", clock_source_valid_cnt);
        }   
        clock_source_valid_last = clock_source_valid;

        // Check memory consumption of tasks periodically, report any tasks with high watermark under threshold
        if (uxTaskGetStackHighWaterMark(t0) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t0_name, uxTaskGetStackHighWaterMark(t0));
        if (uxTaskGetStackHighWaterMark(t1) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t1_name, uxTaskGetStackHighWaterMark(t1));
        if (uxTaskGetStackHighWaterMark(t1) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t2_name, uxTaskGetStackHighWaterMark(t2));
    }
}
