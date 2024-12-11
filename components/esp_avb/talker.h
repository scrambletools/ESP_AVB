#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_TALKER_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_TALKER_H_

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_eth.h>
#include <esp_mac.h>
#include <esp_netif.h>
#include <esp_vfs_l2tap.h>
#include <lwip/prot/ethernet.h> // Ethernet header
#include <lwip/ip_addr.h>
#include <arpa/inet.h> // ntohs
#include <freertos/queue.h>
#include "network.h"

// Functions
static void gptp_task(void *pvParameters);
static void check_local_gm(void* arg);
static void send_gptp_pdelay_request(void* arg);
static void pdelay_response_task(void *pvParameters);
static void send_gptp_announce(void* arg);
static void send_gptp_sync(void* arg);
static void avtp_task(void *pvParameters);
static void msrp_task(void *pvParameters);
static void mvrp_task(void *pvParameters);
void start_talker(esp_netif_iodriver_handle handle);
void stop_talker();

#endif /* ESP_AVB_SIMPLE_TALKER_INCLUDE_TALKER_H_ */
