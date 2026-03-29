/*
 * Copyright 2024 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the network interface for the ESP_AVB component.
 */

#include "avb.h"

#define DMA_BUFFER_SIZE 1024
#define I2S_SAMPLE_RATE 48000

#define TAG "AVB-NET"

// Shared DMA buffer
uint8_t *shared_dma_buffer;

// Semaphore to synchronize Ethernet RX and I2S TX
SemaphoreHandle_t eth_rx_ready;

/* Initialize the network interface */
int avb_net_init(avb_state_s *state) {

  // Create 3 L2TAP interfaces (FDs) for AVTP, MSRP, and MVRP
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {

    int fd = open("/dev/net/tap", 0);
    state->l2if[i] = fd;
    if (state->l2if[i] < 0) {
      avberr("Failed to create tx l2if: %d", errno);
      return ERROR;
    }

    // Set Ethernet interface on which to get raw frames
    if (ioctl(state->l2if[i], L2TAP_S_INTF_DEVICE,
              state->config.eth_interface) < 0) {
      avberr(
          "failed to set network interface at fd %d on interface %s: errno %d",
          fd, state->config.eth_interface, errno);
      return ERROR;
    }

    // Set the ethertype based on the protocol index
    uint16_t ethertype;
    switch (i) {
    case AVTP:
      ethertype = ethertype_avtp;
      break;
    case MSRP:
      ethertype = ethertype_msrp;
      break;
    case MVRP:
      ethertype = ethertype_mvrp;
      break;
    case VLAN:
      ethertype = ethertype_vlan;
      break;
    default:
      avberr("Invalid protocol index\n");
      return ERROR;
    }
    // Set the Ethertype filter (frames with this type will be available through
    // the state->tx_l2if)
    if (ioctl(state->l2if[i], L2TAP_S_RCV_FILTER, &ethertype) < 0) {
      avberr("Failed to set Ethertype filter for fd %d: errno %d", fd, errno);
      return ERROR;
    }

    // TX timestamps intentionally NOT enabled on AVB sockets. The timestamped
    // transmit path (esp_eth_transmit_ctrl_vargs) busy-waits for DMA completion
    // while holding the EMAC TX mutex, blocking the real-time AVB-OUT task on
    // core 1. Only the PTP daemon (which has its own socket) needs TX timestamps.
    avbinfo("Initialized L2TAP fd %d for ethertype %x", fd, ethertype);
  }

  /* Create a handle to the Ethernet driver */
  esp_eth_handle_t eth_handle;
  if (ioctl(state->l2if[0], L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) < 0) {
    avberr("Failed to get eth_handle for fd %d: errno %d", state->l2if[0],
           errno);
    return ERROR;
  }

  // Get MAC address and store in state
  esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, &state->internal_mac_addr);
  return OK;
}

/* Create an Ethernet frame */
void avb_create_eth_frame(uint8_t *eth_frame, eth_addr_t *dest_addr,
                          avb_state_s *state, ethertype_t ethertype, void *msg,
                          uint16_t msg_len, uint8_t *vlan_id) {
  struct eth_hdr eth_hdr = {.type = htons(ethertype)};
  uint16_t vid = vlan_id ? octets_to_uint(vlan_id, 2) : 0;
  uint16_t prio = state->msrp_mappings[0].priority;
  struct eth_vlan_hdr eth_vlan_hdr = {
      .prio_vid = htons((prio << 13) | (vid & 0x0FFF)),
      .tpid = htons(ethertype_avtp)};
  memcpy(&eth_hdr.dest.addr, dest_addr, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->internal_mac_addr, ETH_ADDR_LEN);
  memcpy(eth_frame, &eth_hdr, sizeof(eth_hdr));
  if (ethertype == ethertype_vlan) {
    memcpy(eth_frame + sizeof(eth_hdr), &eth_vlan_hdr, sizeof(eth_vlan_hdr));
    memcpy(eth_frame + sizeof(eth_hdr) + sizeof(eth_vlan_hdr), msg, msg_len);
  } else {
    memcpy(eth_frame + sizeof(eth_hdr), msg, msg_len);
  }
}

/* Send an Ethernet frame */
int avb_net_send_to(avb_state_s *state, ethertype_t ethertype, void *msg,
                    uint16_t msg_len, struct timespec *ts,
                    eth_addr_t *dest_addr) {
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];
  int l2if;

  // Create the Ethernet frame
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len,
                       NULL);

  // Get the L2IF for the given ethertype
  switch (ethertype) {
  case ethertype_avtp:
    l2if = state->l2if[AVTP];
    break;
  case ethertype_msrp:
    l2if = state->l2if[MSRP];
    break;
  case ethertype_mvrp:
    l2if = state->l2if[MVRP];
    break;
  case ethertype_vlan:
    l2if = state->l2if[VLAN];
    break;
  default:
    avberr("Invalid ethertype: %d", ethertype);
    return ERROR;
  }

  int ret = write(l2if, eth_frame, sizeof(eth_frame));
  return ret;
}

/* Send an Ethernet frame with VLAN ID */
int avb_net_send_to_vlan(avb_state_s *state, ethertype_t ethertype, void *msg,
                         uint16_t msg_len, struct timespec *ts,
                         eth_addr_t *dest_addr, uint8_t *vlan_id) {
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN + sizeof(struct eth_vlan_hdr)];
  int l2if;

  // Create the Ethernet frame
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len,
                       vlan_id);

  switch (ethertype) {
  case ethertype_avtp:
    l2if = state->l2if[AVTP];
    break;
  case ethertype_msrp:
    l2if = state->l2if[MSRP];
    break;
  case ethertype_mvrp:
    l2if = state->l2if[MVRP];
    break;
  case ethertype_vlan:
    l2if = state->l2if[VLAN];
    break;
  default:
    avberr("Invalid ethertype: %d", ethertype);
    return ERROR;
  }

  int ret = write(l2if, eth_frame, sizeof(eth_frame));
  return ret;
}

int avb_net_send(avb_state_s *state, ethertype_t ethertype, void *msg,
                 uint16_t msg_len, struct timespec *ts) {
  eth_addr_t dest_addr;

  // Set destination address based on ethertype
  switch (ethertype) {
  case ethertype_avtp:
    uint8_t subtype;
    memcpy(&subtype, msg, 1);
    if (subtype == avtp_subtype_maap) {
      memcpy(&dest_addr, &MAAP_MCAST_MAC_ADDR, ETH_ADDR_LEN);
    } else {
      memcpy(&dest_addr, &BCAST_MAC_ADDR, ETH_ADDR_LEN);
    }
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
  return avb_net_send_to(state, ethertype, msg, msg_len, ts, &dest_addr);
}

/* Receive an Ethernet frame */
int avb_net_recv(int l2if, void *msg, uint16_t msg_len, struct timespec *ts,
                 eth_addr_t *src_addr) {
  uint8_t eth_frame[msg_len + ETH_HEADER_LEN];

  int ret = read(l2if, eth_frame, sizeof(eth_frame));
  if (ret <= 0) {
    return ret;
  }

  // copy source address from the frame
  memcpy(src_addr, &eth_frame[ETH_ADDR_LEN], ETH_ADDR_LEN);

  // copy the message from the frame (subtract Ethernet header)
  int payload_len = ret - ETH_HEADER_LEN;
  if (payload_len > 0 && payload_len <= msg_len) {
    memcpy(msg, &eth_frame[ETH_HEADER_LEN], payload_len);
  }
  return payload_len;
}

// Ethernet RX callback
static esp_err_t eth_on_data(esp_eth_handle_t eth_handle, uint8_t *buf,
                             uint32_t len) {
  if (len > DMA_BUFFER_SIZE) {
    ESP_LOGE(TAG, "Ethernet frame too large for DMA buffer");
    return ESP_OK;
  }

  // wire to I2S

  return ESP_OK;
}
