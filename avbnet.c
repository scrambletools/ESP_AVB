/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component
 *
 * This component provides an implementation of an AVB talker and listener.
 *
 * This file provides the network interface for the ESP_AVB component.
 *
 * RX Architecture: A single EMAC RX callback (avb_unified_rx_cb) handles ALL
 * incoming Ethernet frames. VLAN-tagged AVTP stream data is dispatched to a
 * registered handler callback (runs inline in EMAC task for lowest latency).
 * Control frames (AVTP, MSRP, MVRP) are copied to a queue and consumed by the
 * AVB main loop via avb_net_recv_ctrl(). All other frames (PTP, ARP, IP) pass
 * through to the IP stack via esp_netif_receive().
 *
 * TX Architecture: Unchanged — uses L2TAP write() on per-ethertype fds.
 */

#include "avb.h"
#include "esp_timer.h"
#include <esp_netif.h>
#include <esp_vfs_l2tap.h>

#define TAG "AVB-NET"

/* Control frame queue depth — ample for MSRP/MVRP/ATDECC rates */
#define CTRL_RX_QUEUE_DEPTH 16

/* File-static state for unified EMAC RX dispatcher */
static QueueHandle_t s_ctrl_rx_queue = NULL;
static avb_stream_rx_handler_t s_stream_handler = NULL;
static void *s_stream_ctx = NULL;
static esp_netif_t *s_eth_netif = NULL;

/* Drop counter no longer used (inline handler has no queue to overflow),
 * but keep the function to avoid breaking callers that read it. */
static volatile uint32_t s_stream_rx_drops = 0;

/* Diagnostic: count PTP frames (0x88f7) that reach avb_unified_rx_cb so we
 * can compare against ptpd's own rx_sync counter. */
static volatile uint32_t s_ptp_rx_seen = 0;
uint32_t avb_net_ptp_rx_seen(void) { return s_ptp_rx_seen; }

/* Unified EMAC RX callback — dispatches ALL incoming Ethernet frames.
 * Runs in the EMAC RX FreeRTOS task context (not ISR).
 *
 * Routing:
 *   0x8100 (VLAN) → stream handler callback (if registered)
 *   0x22f0 (AVTP) → ctrl_rx_queue (protocol_idx = AVTP)
 *   0x22ea (MSRP) → ctrl_rx_queue (protocol_idx = MSRP)
 *   0x88f5 (MVRP) → ctrl_rx_queue (protocol_idx = MVRP)
 *   default        → esp_netif_receive (IP stack for PTP, ARP, etc.)
 */
static esp_err_t avb_unified_rx_cb(esp_eth_handle_t eth_handle, uint8_t *buf,
                                   uint32_t len, void *priv, void *info) {
  if (len < ETH_HEADER_LEN) {
    free(buf);
    return ESP_OK;
  }

  /* Read ethertype at offset 12-13 (big-endian) */
  uint16_t ethertype = (buf[12] << 8) | buf[13];

  /* Count ALL PTP frames (0x88f7) at entry — before any switch branch
   * and before any early-return. Compared against ptpd's own rx counters
   * to identify where in the path PTP frames are being lost. */
  if (ethertype == 0x88f7) {
    s_ptp_rx_seen++;
  }

  switch (ethertype) {
  case 0x8100: { /* VLAN — stream data */
    /* Inline handler call — matches the original stable architecture.
     * The queue-based split we tried (emac_rx → queue → AVB-IN task)
     * added ~10 µs per frame of task-wake + context-switch overhead
     * that at 8000 pps cost ~80 ms/sec of extra CPU and drove the NIC
     * DMA ring into overflow. Keeping the handler here means emac_rx
     * does one self-contained pass per frame: alloc → memcpy →
     * handler → free. */
    if (s_stream_handler && len > 18) {
      /* Strip ETH header (14) + VLAN tag (4) = 18 bytes → raw AVTP */
      s_stream_handler(buf + 18, len - 18, s_stream_ctx);
    }
    free(buf);
    return ESP_OK;
  }
  case 0x22f0: /* AVTP (control: ADP, AECP, ACMP, MAAP) */
  case 0x22ea: /* MSRP */
  case 0x88f5: /* MVRP */
  {
    if (s_ctrl_rx_queue) {
      ctrl_rx_pkt_t pkt;
      /* Map ethertype to protocol index */
      switch (ethertype) {
      case 0x22f0:
        pkt.protocol_idx = AVTP;
        break;
      case 0x22ea:
        pkt.protocol_idx = MSRP;
        break;
      case 0x88f5:
        pkt.protocol_idx = MVRP;
        break;
      default:
        pkt.protocol_idx = AVTP;
        break;
      }
      /* Copy source MAC from offset 6 */
      memcpy(pkt.src_addr, buf + ETH_ADDR_LEN, ETH_ADDR_LEN);
      /* Copy payload (strip ETH header) */
      uint32_t payload_len = len - ETH_HEADER_LEN;
      if (payload_len > AVB_MAX_MSG_LEN)
        payload_len = AVB_MAX_MSG_LEN;
      pkt.length = payload_len;
      memcpy(pkt.data, buf + ETH_HEADER_LEN, payload_len);
      /* Non-blocking send — drop if queue full rather than stalling EMAC */
      xQueueSend(s_ctrl_rx_queue, &pkt, 0);
    }
    /* Control frames are fully handled via ctrl_rx_queue — skip L2TAP
     * filter (nobody reads from L2TAP fds, main loop uses the queue).
     * Must free buf since we're not passing it to esp_netif_receive. */
    free(buf);
    return ESP_OK;
  }
  default: {
    /* PTP, ARP, IP, etc. — pass through L2TAP filter then to IP stack.
     * esp_vfs_l2tap_eth_filter_frame frees buf when it matches a filter
     * (eb_handle=NULL path calls free(buf) internally), so we must NOT
     * free buf ourselves when frame_len==0. */
    size_t frame_len = len;
    esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, info);
    if (frame_len > 0) {
      return esp_netif_receive(s_eth_netif, buf, frame_len, NULL);
    }
    /* L2TAP consumed and freed buf — nothing more to do */
    return ESP_OK;
  }
  }
}

/* Initialize the network interface.
 * Opens L2TAP fds for TX, creates ctrl_rx_queue, registers EMAC callback.
 */
int avb_net_init(avb_state_s *state) {

  /* Open L2TAP fds for TX (and to register ethertype filters so L2TAP
   * write() works). RX is handled by the unified EMAC callback above. */
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
    // Set the Ethertype filter — required for L2TAP write() to work,
    // and also registers this ethertype with the L2TAP filter so the
    // EMAC callback can pass unmatched frames through.
    if (ioctl(state->l2if[i], L2TAP_S_RCV_FILTER, &ethertype) < 0) {
      avberr("Failed to set Ethertype filter for fd %d: errno %d", fd, errno);
      return ERROR;
    }

    // TX timestamps intentionally NOT enabled on AVB sockets. The timestamped
    // transmit path (esp_eth_transmit_ctrl_vargs) busy-waits for DMA completion
    // while holding the EMAC TX mutex, blocking the real-time AVB-OUT task on
    // core 1. Only the PTP daemon (which has its own socket) needs TX
    // timestamps.
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

  /* Create the control frame RX queue */
  s_ctrl_rx_queue = xQueueCreate(CTRL_RX_QUEUE_DEPTH, sizeof(ctrl_rx_pkt_t));
  if (!s_ctrl_rx_queue) {
    avberr("Failed to create ctrl_rx_queue");
    return ERROR;
  }
  state->ctrl_rx_queue = s_ctrl_rx_queue;

  /* Register unified EMAC RX callback — intercepts ALL frames before
   * they reach L2TAP or the IP stack. */
  s_eth_netif = esp_netif_get_handle_from_ifkey(state->config.eth_interface);
  esp_eth_update_input_path_info(state->config.eth_handle, avb_unified_rx_cb,
                                 s_eth_netif);
  avbinfo("Unified EMAC RX dispatcher registered");

  return OK;
}

/* Create an Ethernet frame */
void avb_create_eth_frame(uint8_t *eth_frame, eth_addr_t *dest_addr,
                          avb_state_s *state, ethertype_t ethertype, void *msg,
                          uint16_t msg_len, uint8_t *vlan_id) {
  struct eth_hdr eth_hdr = {.type = htons(ethertype)};
  uint16_t vid = vlan_id ? octets_to_uint(vlan_id, 2) : 0;
  uint16_t prio = state->msrp_mappings[0].priority;
  struct eth_vlan_hdr eth_vlan_hdr = {.prio_vid =
                                          htons((prio << 13) | (vid & 0x0FFF)),
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

/* Receive next control frame from the unified EMAC RX dispatcher.
 * Blocks up to timeout_ms. Returns payload length, or 0 on timeout.
 * protocol_idx is set to AVTP/MSRP/MVRP. */
int avb_net_recv_ctrl(avb_state_s *state, int *protocol_idx, void *msg,
                      uint16_t msg_len, eth_addr_t *src_addr, int timeout_ms) {
  ctrl_rx_pkt_t pkt;
  if (xQueueReceive(s_ctrl_rx_queue, &pkt, pdMS_TO_TICKS(timeout_ms)) !=
      pdTRUE) {
    return 0; /* timeout — no frame available */
  }
  *protocol_idx = pkt.protocol_idx;
  memcpy(src_addr, pkt.src_addr, ETH_ADDR_LEN);
  uint16_t copy_len = pkt.length < msg_len ? pkt.length : msg_len;
  memcpy(msg, pkt.data, copy_len);
  return copy_len;
}

/* Register stream RX handler. Invoked inline from the EMAC RX callback
 * context (avb_unified_rx_cb VLAN branch), so the handler must be fast
 * (~25 µs at 8000 pps). Pass NULL to unregister. */
void avb_net_set_stream_rx_handler(avb_stream_rx_handler_t handler, void *ctx) {
  s_stream_ctx = ctx;
  /* Write handler last with memory barrier semantics —
   * the callback checks s_stream_handler != NULL as gate */
  s_stream_handler = handler;
}

uint32_t avb_net_stream_rx_drops(void) { return s_stream_rx_drops; }
