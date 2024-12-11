#ifndef ESP_AVB_SIMPLE_TALKER_INCLUDE_NETWORK_H_
#define ESP_AVB_SIMPLE_TALKER_INCLUDE_NETWORK_H_

#include <esp_netif.h>
#include <esp_vfs_l2tap.h>
#include <lwip/prot/ethernet.h> // Ethernet header
#include "atdecc.h"
#include "gptp.h"
#include "avtp.h"

// for l2tap
#define INVALID_FD     -1
#define ETH_INTERFACE "ETH_DEF"
#define PRINT_FRAME_FORMAT 0 // 0=short, 1=long

// Functions
void set_eth_handle(esp_netif_iodriver_handle handle);
esp_err_t get_frame(ethertype_t ethertype, eth_frame_t *frame);
esp_err_t send_frame(eth_frame_t *frame);
void set_frame_header(uint8_t *dest_addr, ethertype_t ethertype, eth_frame_t *frame);
void print_frame(eth_frame_t *frame, int format);
void print_frame_of_type(avb_frame_type_t type, eth_frame_t *frame, int format);
int init_l2tap_fd(int flags, ethertype_t ethertype);
esp_err_t init_all_l2tap_fds();
void close_all_l2tap_fds();

#endif // ESP_AVB_SIMPLE_TALKER_INCLUDE_NETWORK_H_
