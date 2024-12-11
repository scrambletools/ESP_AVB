#include <stdio.h>
#include <inttypes.h>
#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include <driver/timer.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_eth.h>
#include <esp_netif.h>
#include <lwip/ip_addr.h>
#include "talker.h"

// static IP fallback values
#define FALLBACK_IP_ADDR            "169.254.18.100"
#define FALLBACK_NETMASK_ADDR       "255.255.0.0"
#define FALLBACK_GW_ADDR            "169.254.18.100"
#define FALLBACK_MAIN_DNS_SERVER    "169.254.18.100"
#define FALLBACK_BACKUP_DNS_SERVER  "169.254.18.1"
#define DHCP_TIMEOUT 10000000
// timer settings
#define TIMER_DIVIDER   (16)  //  Hardware timer clock divider
#define TIMER_SCALE     (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

// Ethernet driver handle
static esp_eth_handle_t eth_handle = NULL;

// Define logging tag
static const char *TAG = "MAIN";

// Global flag for IP status
static bool ip_obtained = false;  

// Setup DNS config
static esp_err_t set_dns_config(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

// Setup static IP address (if needed)
static void set_static_ip(esp_netif_t *netif)
{
    esp_err_t result = esp_netif_dhcpc_stop(netif);
    if ((result != ESP_OK) 
        &&  (result != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)) {
        ESP_LOGE(TAG, "Failed to stop dhcp client: %d", result);
        return;
    }
    // Manually set ip
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(FALLBACK_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(FALLBACK_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(FALLBACK_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    // Set the IP obtained flag to true
    ip_obtained = true;
    ESP_LOGI(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", FALLBACK_IP_ADDR, FALLBACK_NETMASK_ADDR, FALLBACK_GW_ADDR);
    // Set dns server
    ESP_ERROR_CHECK(set_dns_config(netif, ipaddr_addr(FALLBACK_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(set_dns_config(netif, ipaddr_addr(FALLBACK_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

// Fallback to static IP when DHCP fails
static void fallback_to_static_ip(esp_netif_t *netif, bool wait) {
    // Select and initialize basic parameters of the timer
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .divider = TIMER_DIVIDER,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);
    uint64_t task_counter_value;
    
    while (!ip_obtained) {
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &task_counter_value);
        if (!wait || (task_counter_value > DHCP_TIMEOUT)) {
            ESP_LOGI(TAG, "Setting static IP as fallback.");
            set_static_ip(netif);
        }
        else {
            ESP_LOGI(TAG, "(%llu) Waiting for IP...", task_counter_value);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Handle Ethernet events
static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == ETHERNET_EVENT_CONNECTED) {

        ESP_LOGI(TAG, "Ethernet Link Up");
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay to ensure link is stable

        // Grab the network interface from event arg
        esp_netif_t *eth_netif = (esp_netif_t *)arg;

        // Start AVB talker (doesn't use IP)
        start_talker(eth_handle);

        // Start DHCP
        esp_netif_ip_info_t ip_info;
        static bool waitForDhcp = true; // wait for dhcp to timeout or not
        if (esp_netif_get_ip_info(eth_netif, &ip_info) == ESP_OK) {
            if (ip_info.ip.addr != 0) {
                ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
            } else {
                ESP_LOGI(TAG, "IP Address not assigned yet.");
                fallback_to_static_ip(eth_netif, waitForDhcp);
            }
        } else {
            waitForDhcp = false; // no need to wait for dhcp to time out
            ESP_LOGE(TAG, "Failed to get IP address, assigning manual address.");
            fallback_to_static_ip(eth_netif, waitForDhcp);
        }
    } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
        ESP_LOGI(TAG, "Ethernet Link Down");
        // Stop AVB talker
        stop_talker();
    } else if (event_id == ETHERNET_EVENT_START) {
        ESP_LOGI(TAG, "Ethernet Started");
    } else if (event_id == ETHERNET_EVENT_STOP) {
        ESP_LOGI(TAG, "Ethernet Stopped");
    }
}

// Handle IP events
static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    
    // Set the IP obtained flag to true
    ip_obtained = true;
}

// Initialize Ethernet for ESP32 with LAN8720 PHY
static esp_netif_t* init_ethernet()
{
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop if not already created
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Configure GPIO16 to enable the oscillator
    gpio_reset_pin(GPIO_NUM_16);
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_16, 1); // Set GPIO16 high to enable the oscillator

    // Configure default Ethernet MAC and PHY settings
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    // Configure the SMI (MDC and MDIO) GPIO pins
    esp32_emac_config.smi_gpio.mdc_num = GPIO_NUM_23;  // MDC pin
    esp32_emac_config.smi_gpio.mdio_num = GPIO_NUM_18; // MDIO pin

    // Create MAC instance using the internal ESP32 EMAC
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    // Configure PHY settings
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;  // Use the default PHY address
    phy_config.reset_gpio_num = GPIO_NUM_5;
    
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
	gpio_set_level(GPIO_NUM_5, 0); // Set low to reset
	vTaskDelay(pdMS_TO_TICKS(100)); // Delay for PHY reset
	gpio_set_level(GPIO_NUM_5, 1); // Set high to take PHY out of reset

    // Create PHY instance for LAN8720
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    // Ethernet configuration linking MAC and PHY
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

    // Install Ethernet driver
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // Create netif for Ethernet
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_config);

    // Attach Ethernet driver to TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register the event handler for Ethernet events, passing the netif as argument
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, eth_netif));

    // Register Got IP event handler
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, eth_netif));

    // Start the Ethernet driver
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    // Initialize L2 TAP VFS interface
    ESP_ERROR_CHECK(esp_vfs_l2tap_intf_register(NULL));

    return eth_netif;
}

void app_main()
{
    // Initialize NVS (non-volatile storage)
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Initialize Ethernet and establish a connection
    init_ethernet();

    // Check memory consumption of tasks periodically, report any tasks with high watermark under threshold
    static const uint period = 10000; // wait between checks in ms
    static const uint threshold = 1000; // size of high watermark
    char t0_name[] = "main_task";  // usually under 16 chars
    char t1_name[] = "watch_gptp"; // usually under 16 chars
    char t2_name[] = "watch_avtp"; // usually under 16 chars
    char t3_name[] = "watch_msrp"; // usually under 16 chars
    char t4_name[] = "watch_mvrp"; // usually under 16 chars
    TaskHandle_t t0 = xTaskGetHandle( t0_name );
    TaskHandle_t t1 = xTaskGetHandle( t1_name );
    TaskHandle_t t2 = xTaskGetHandle( t2_name );
    TaskHandle_t t3 = xTaskGetHandle( t3_name );
    TaskHandle_t t4 = xTaskGetHandle( t4_name );
    while(1) {
        if (uxTaskGetStackHighWaterMark(t0) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t0_name, uxTaskGetStackHighWaterMark(t0));
        if (uxTaskGetStackHighWaterMark(t1) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t1_name, uxTaskGetStackHighWaterMark(t1));
        if (uxTaskGetStackHighWaterMark(t2) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t2_name, uxTaskGetStackHighWaterMark(t2));
        if (uxTaskGetStackHighWaterMark(t3) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t3_name, uxTaskGetStackHighWaterMark(t3));
        if (uxTaskGetStackHighWaterMark(t4) < threshold)
            ESP_LOGI(TAG, "TASK %s high water mark = %d", t4_name, uxTaskGetStackHighWaterMark(t4));
        vTaskDelay(pdMS_TO_TICKS(period));
    }
}
