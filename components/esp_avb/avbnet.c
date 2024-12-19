/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 */

#include "avb.h"

// Define logging tag
static const char *TAG = "NET";

// Managed mac addresses
static uint8_t local_eth_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t local_wifi_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t controller_mac_addr[ETH_ADDR_LEN] = { 0 };
static uint8_t listener_mac_addr[ETH_ADDR_LEN] = { 0 }; // remote device
static uint8_t talker_mac_addr[ETH_ADDR_LEN] = { 0 }; // remote device

/* Initialize the network interface */
int avb_net_init(struct avb_state_s *state, const char *interface) {
  /* Create a handle to the Ethernet driver */
  esp_eth_handle_t eth_handle;

  // Create 3 L2TAP interfaces (FDs) for AVTP, MSRP, and MVRP
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {

    int fd = open("/dev/net/tap", 0);
    avbinfo("fd: %d", fd);
    state->l2if[i] = fd;
    if (state->l2if[i] < 0) {
      avberr("Failed to create tx l2if: %d\n", errno);
      return ERROR;
    }
    // Set Ethernet interface on which to get raw frames
    if (ioctl(state->l2if[i], L2TAP_S_INTF_DEVICE, interface) < 0) {
      avberr("failed to set network interface at fd %d: errno %d\n", fd, errno);
      return ERROR;
    }

    // Set the ethertype based on the protocol index
    uint16_t ethertype;
    switch(i) {
      case AVTP:
        ethertype = ethertype_avtp;
        break;
      case MSRP:
        ethertype = ethertype_msrp;
        break;
      case MVRP:
        ethertype = ethertype_mvrp;
        break;
      default:
        avberr("Invalid protocol index\n");
        return ERROR;
    }
    // Set the Ethertype filter (frames with this type will be available through the state->tx_l2if)
    if (ioctl(state->l2if[i], L2TAP_S_RCV_FILTER, &ethertype) < 0) {
      avberr("failed to set Ethertype filter for fd %d: errno %d\n", fd, errno);
      return ERROR;
    }
    // Enable time stamping in driver
    if (ioctl(state->l2if[i], L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0) {
      avberr("failed to get l2if eth_handle for fd %d: errno %d\n", fd, errno);
      return ERROR;
    }

    // Enable time stamping in L2TAP (clock already initialized by PTPd)
    if(ioctl(state->l2if[i], L2TAP_S_TIMESTAMP_EN) < 0) {
      avberr("failed to enable time stamping in L2TAP fd %d: errno %d\n", fd, errno);
      return ERROR;
    }
    avbinfo("Successfully initialized L2TAP fd %d for ethertype %x", fd, ethertype);
  }
  // get HW address
  esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, &state->internal_mac_addr);
  return OK;
}

/* Create an Ethernet frame */
void avb_create_eth_frame(uint8_t *eth_frame, 
                          eth_addr_t *dest_addr, 
                          struct avb_state_s *state, 
                          ethertype_t ethertype, 
                          void *msg, 
                          uint16_t msg_len) {
  struct eth_hdr eth_hdr = {
    .type = htons(ethertype)
  };
  memcpy(&eth_hdr.dest.addr, dest_addr, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->internal_mac_addr, ETH_ADDR_LEN);
  memcpy(eth_frame, &eth_hdr, sizeof(eth_hdr));
  memcpy(eth_frame + sizeof(eth_hdr), msg, msg_len);
}

/* Send an Ethernet frame */
int avb_net_send(struct avb_state_s *state, 
                 ethertype_t ethertype, 
                 void *msg, 
                 uint16_t msg_len, 
                 struct timespec *ts) {
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];
  // Set destination address based on ethertype
  eth_addr_t dest_addr;
  switch(ethertype) {
    case ethertype_avtp:
      memcpy(&dest_addr, &BCAST_MAC_ADDR, ETH_ADDR_LEN);
      break;
    case ethertype_msrp:
      memcpy(&dest_addr, &LLDP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
      break;
    case ethertype_mvrp:
      memcpy(&dest_addr, &SPANTREE_MAC_ADDR, ETH_ADDR_LEN);
      break;
    default:
      avberr("Invalid ethertype: %d", ethertype);
      return ERROR;
  }
  int l2if;
  avb_create_eth_frame(eth_frame, &dest_addr, state, ethertype, msg, msg_len);

  // wrap "Info Records Buffer" into union to ensure proper alignment of data (this is typically needed when
  // accessing double word variables or structs containing double word variables)
  union {
      uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
      l2tap_irec_hdr_t align;
  } u;

  l2tap_extended_buff_t msg_ext_buff;

  msg_ext_buff.info_recs_len = sizeof(u.info_recs_buff);
  msg_ext_buff.info_recs_buff = u.info_recs_buff;
  msg_ext_buff.buff = eth_frame;
  msg_ext_buff.buff_len = sizeof(eth_frame);

  l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&msg_ext_buff);
  ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
  ts_info->type = L2TAP_IREC_TIME_STAMP;

  // Get the L2IF for the given ethertype
  switch(ethertype) {
    case ethertype_avtp:
      l2if = state->l2if[AVTP];
      break;
    case ethertype_msrp:
      l2if = state->l2if[MSRP];
      break;
    case ethertype_mvrp:
      l2if = state->l2if[MVRP];
      break;
    default:
      avberr("Invalid ethertype: %d", ethertype);
      return ERROR;
  }

  int ret = write(l2if, &msg_ext_buff, 0);

  // check if write was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP)
    {
      *ts = *(struct timespec *)ts_info->data;
      avbinfo( "avb_net_send: ts is %lld.%09ld", (long long)ts->tv_sec, ts->tv_nsec);
    }
  return ret;
}

/* Receive an Ethernet frame */
int avb_net_recv(struct avb_state_s *state, 
                 int l2if, 
                 void *msg, 
                 uint16_t msg_len, 
                 struct timespec *ts) {
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];

  // wrap "Info Records Buffer" into union to ensure proper alignment of data (this is typically needed when
  // accessing double word variables or structs containing double word variables)
  union {
      uint8_t info_recs_buff[L2TAP_IREC_SPACE(sizeof(struct timespec))];
      l2tap_irec_hdr_t align;
  } u;
  l2tap_extended_buff_t msg_ext_buff;

  msg_ext_buff.info_recs_len = sizeof(u.info_recs_buff);
  msg_ext_buff.info_recs_buff = u.info_recs_buff;
  msg_ext_buff.buff = eth_frame;
  msg_ext_buff.buff_len = sizeof(eth_frame);

  l2tap_irec_hdr_t *ts_info = L2TAP_IREC_FIRST(&msg_ext_buff);
  ts_info->len = L2TAP_IREC_LEN(sizeof(struct timespec));
  ts_info->type = L2TAP_IREC_TIME_STAMP;

  int ret = read(l2if, &msg_ext_buff, 0);

  // check if read was successful, ts exists and ts_info is valid
  if (ret > 0 && ts && ts_info->type == L2TAP_IREC_TIME_STAMP) {
    *ts = *(struct timespec *)ts_info->data;
    avbinfo("avb_net_recv: ts is %lld.%09ld", (long long)ts->tv_sec, ts->tv_nsec);
  }
  memcpy(msg, &eth_frame[ETH_HEADER_LEN], ret);
  return ret;
}

