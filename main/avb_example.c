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
#include <esp_intr_alloc.h>
#include <ethernet_init.h>
#include <esp_eth_time.h>
#include <esp_vfs_l2tap.h>
#include <driver/gpio.h>
#include <ptpd.h>
#include "esp_avb.h"

/* Example configurations */
#define AVB_SPEAKER_VOLUME  CONFIG_EXAMPLE_AVB_SPEAKER_VOLUME
#define AVB_MIC_GAIN        CONFIG_EXAMPLE_AVB_MIC_GAIN

typedef struct {
    char name[16];
    TaskHandle_t handle;
} task_info_t;

static const char *TAG = "avb_example";
static struct timespec s_next_time;
static bool s_gpio_level;
esp_eth_handle_t *eth_handles;
char avb_interface[10];

/* Import music file as buffer */
#if CONFIG_EXAMPLE_AVB_TALKER
extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");
#endif

/* Initialize GPIOs for application */
static esp_err_t gpio_init(void) {
    /* Initialize the output pin for time pulse indicator LED */
    gpio_config_t gpio_pulse_led_cfg = {
        .pin_bit_mask = (1ULL << CONFIG_EXAMPLE_AVB_PULSE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&gpio_pulse_led_cfg), TAG, "GPIO config failed");
    ESP_RETURN_ON_ERROR(gpio_set_level(CONFIG_EXAMPLE_AVB_PULSE_GPIO, 0), TAG, "GPIO set level failed"); // set low

    return ESP_OK;
}

/* Initialize Ethernet and netif on all available ports */
void init_ethernet_and_netif(void) {
    uint8_t eth_port_cnt;

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

        // Set the first interface as the AVB interface
        if (i == 0) {
            memcpy(avb_interface, if_key_str, sizeof(if_key_str));
            ESP_LOGI(TAG, "AVB interface: %s", avb_interface);
        }
    }
    // Start Ethernet
    for (int i = 0; i < eth_port_cnt; i++) {
        ESP_ERROR_CHECK(esp_eth_start(eth_handles[i]));
    }
}

/* Callback function to toggle the output pin for time pulse indicator LED */
IRAM_ATTR bool ts_callback(esp_eth_mediator_t *eth, void *user_args) {
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

void app_main(void) {
    struct timespec cur_time;

    /* Initialize GPIOs if needed */
    if (gpio_init() != ESP_OK) {
        ESP_LOGE(TAG, "GPIO init failed");
        abort();
    }

    /* Start Ethernet */
    init_ethernet_and_netif();

    /* Start PTP */
    int pid = ptpd_start(avb_interface);

    /* Wait for the clock to be available */
    while (esp_eth_clock_gettime(CLOCK_PTP_SYSTEM, &cur_time) == -1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* Set up the AVB configuration */
    avb_config_s avb_config = AVB_DEFAULT_CONFIG();
#if CONFIG_EXAMPLE_AVB_TALKER
    avb_config.talker = true;
#endif
#if CONFIG_EXAMPLE_AVB_LISTENER
    avb_config.listener = true;
#endif

    /* Set the Ethernet handle in the AVB config */
    avb_config.eth_handle = eth_handles[0];

    /* Set the Ethernet interface in the AVB config */
    avb_config.eth_interface = "ETH_0";

    /* Start AVB*/
    avb_start(&avb_config);

    /* After AVB is started, you can set additional codec options */
    // MUST USE EVENT TO MAKE SURE CODEC IS INITIALIZED, THEN USE STATUS TO GET CODEC HANDLE
    // void *codec_handle = avb_get_codec_handle();
    // if (codec_handle) {
    //     if (avb_config.talker) {
    //         esp_codec_dev_set_in_gain(codec_handle, 30.0);
    //     }
    //     if (avb_config.listener) {
    //         esp_codec_dev_set_out_vol(codec_handle, 60.0);
    //     }
    // }

    /* Register callback function for time pulse indicator LED */
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
            //ESP_LOGW(TAG, "PTP clock not synced (count %ld)", clock_source_valid_cnt);
        }   
        clock_source_valid_last = clock_source_valid;

        // Check memory consumption of tasks periodically, report any tasks with high watermark under threshold
        if (uxTaskGetStackHighWaterMark(t0) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t0_name, uxTaskGetStackHighWaterMark(t0));
        if (uxTaskGetStackHighWaterMark(t1) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t1_name, uxTaskGetStackHighWaterMark(t1));
        if (uxTaskGetStackHighWaterMark(t2) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t2_name, uxTaskGetStackHighWaterMark(t2));
    }
}
