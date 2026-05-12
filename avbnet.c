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
#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
#include "avbbridge.h"
#endif

#define TAG "AVB-NET"

/* Control frame queue depth — ample for MSRP/MVRP/ATDECC rates */
#define CTRL_RX_QUEUE_DEPTH 16

/* File-static state for unified EMAC RX dispatcher */
static QueueHandle_t s_ctrl_rx_queue = NULL;
static avb_stream_rx_handler_t s_stream_handler = NULL;
static void *s_stream_ctx = NULL;
static esp_netif_t *s_eth_netif = NULL;
#if CONFIG_ESP_AVB_NUM_PORTS > 1
static esp_netif_t *s_wifi_netif = NULL;
#endif

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
/* esp_wifi_internal_tx forward decl. Same rationale as the equivalent
 * decl above avb_net_transmit_raw further down — declared locally so
 * esp_avb doesn't gain a hard esp_wifi PRIV_REQUIRES; the symbol is
 * provided by esp_wifi (native) or esp_wifi_remote (P4 host with
 * coprocessor). The bridge needs this for the Wi-Fi-egress branch in
 * avb_bridge_forward. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* avb_state_s pointer for the bridge RX path. Set during avb_net_init,
 * read by the EMAC + Wi-Fi RX callbacks to dispatch egress sends. */
static avb_state_s *s_bridge_state = NULL;

/* Map an EtherType to the L2 protocol index used to select a per-port
 * L2TAP fd. Returns -1 for unknown EtherTypes. */
static int avb_bridge_protocol_idx(uint16_t ethertype) {
  switch (ethertype) {
  case 0x22f0:
    return AVTP;
  case 0x22ea:
    return MSRP;
  case 0x88f5:
    return MVRP;
  case 0x8100:
    return VLAN;
  default:
    return -1;
  }
}

/* Forward a raw Ethernet frame to the egress port. Dispatches by the
 * egress port's medium:
 *   - ethernet: esp_eth_transmit on the bridge's EMAC handle. The
 *     frame buffer is borrowed (synchronous DMA enqueue copies into
 *     the descriptor ring, so the caller may free immediately).
 *   - wifi:     esp_wifi_internal_tx on WIFI_IF_AP. We malloc-copy
 *     because the wifi driver may reference the buffer until tx_done;
 *     freeing here is safe because IDF's wifi TX path internally
 *     copies the frame onto its own descriptor (see the matching
 *     comment in avb_net_transmit_raw).
 * Other media drop silently. The (void)ethertype param is kept for
 * future per-ethertype shaping/policing decisions. */
static void avb_bridge_forward(int egress_port, uint16_t ethertype,
                               const uint8_t *frame, uint32_t len) {
  (void)ethertype;
  if (egress_port < 0 || egress_port >= CONFIG_ESP_AVB_NUM_PORTS) {
    return;
  }
  avb_port_s *p = &s_bridge_state->port[egress_port];

  if (p->medium == avb_port_medium_ethernet) {
    if (s_bridge_state->config.eth_handle) {
      (void)esp_eth_transmit(s_bridge_state->config.eth_handle, (void *)frame,
                             len);
    }
    return;
  }

  if (p->medium == avb_port_medium_wifi) {
    void *buf = malloc(len);
    if (!buf) {
      return;
    }
    memcpy(buf, frame, len);
    /* WIFI_IF_AP=1 — the bridge's SoftAP. */
    (void)esp_wifi_internal_tx(1 /* WIFI_IF_AP */, buf, len);
    free(buf);
    return;
  }
}
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

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

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  /* Bridge dispatch — runs on every ingress (EMAC for port 0, Wi-Fi
   * AP for port 1). The ingress port is derived from the eth_handle:
   * non-NULL means EMAC ingress (port 0), NULL means the Wi-Fi RX
   * shim (port 1). This holds because the v1 bridge topology is
   * fixed at port[0]=ethernet, port[1]=wifi. The classifier picks the
   * other port as egress. */
  if (s_bridge_state != NULL) {
    uint8_t pcp = 0;
    if (ethertype == 0x8100 && len >= 16) {
      pcp = (buf[14] >> 5) & 0x07; /* TPID at 12-13, TCI at 14-15; PCP = TCI[15:13] */
    }
    int ingress_port = (eth_handle == NULL) ? 1 : 0;
    avb_bridge_disposition_t d =
        avb_bridge_classify(ingress_port, ethertype, pcp);
    switch (d.verdict) {
    case AVB_BRIDGE_DROP:
      free(buf);
      return ESP_OK;
    case AVB_BRIDGE_BRIDGE:
      avb_bridge_forward(d.egress_port, ethertype, buf, len);
      free(buf);
      return ESP_OK;
    case AVB_BRIDGE_TERMINATE:
    default:
      break; /* fall through to existing per-protocol dispatch */
    }
  }
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

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
     * free buf ourselves when frame_len==0.
     *
     * L2TAP only applies on Ethernet ingress — wifi-port-0 RX (Phase
     * 6b.2) passes eth_handle=NULL and we skip straight to
     * esp_netif_receive. */
    size_t frame_len = len;
    if (eth_handle != NULL) {
      esp_vfs_l2tap_eth_filter_frame(eth_handle, buf, &frame_len, info);
    }
    if (frame_len > 0) {
      return esp_netif_receive(s_eth_netif, buf, frame_len, NULL);
    }
    /* L2TAP consumed and freed buf — nothing more to do */
    return ESP_OK;
  }
  }
}

/* Wi-Fi RX path (Phase 6b.2 step 3). esp_wifi delivers each L2 frame
 * via this callback in the Wi-Fi driver's task context. The eb
 * descriptor owns the buffer, so we copy out, free eb, and feed the
 * fresh heap buffer into avb_unified_rx_cb (which assumes ownership
 * and free()s on its way out). One memcpy per frame; acceptable for
 * control-plane rates. The talker fast-path TX bypasses this entirely
 * (it sends via avb_net_transmit_raw → esp_wifi_internal_tx). */
extern esp_err_t esp_wifi_internal_free_rx_buffer(void *eb);
extern esp_err_t esp_wifi_internal_reg_rxcb(
    int wifi_if, esp_err_t (*fn)(void *buffer, uint16_t len, void *eb));

static esp_err_t avb_wifi_rx_cb(void *buffer, uint16_t len, void *eb) {
  uint8_t *buf = malloc(len);
  if (!buf) {
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_FAIL;
  }
  memcpy(buf, buffer, len);
  esp_wifi_internal_free_rx_buffer(eb);
  return avb_unified_rx_cb(NULL /* no eth_handle */, buf, (uint32_t)len,
                           NULL /* priv */, NULL /* info */);
}

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
/* Override esp_wifi_remote's WEAK channel_rx symbol. esp_wifi_remote_net2.c
 * (the variant the P4 host links because CONFIG_ESP_WIFI_ENABLED=y) routes
 * frames coming up from the C6 coprocessor over SDIO into a private static
 * s_rx_fn[] array that's set by its own wifi_start handler — and crucially,
 * esp_wifi_internal_reg_rxcb resolves to the real esp_wifi's variant which
 * writes to a *different* (unused) array. So a normal _reg_rxcb call is a
 * silent no-op on this host.
 *
 * We provide a strong override of esp_wifi_remote_channel_rx that snoops
 * the frame into avb_wifi_rx_cb (which malloc-copies, frees the source
 * buffer, and re-enters avb_unified_rx_cb with eth_handle=NULL — the
 * bridge classifier interprets that as ingress_port=1 → Eth egress).
 *
 * Bridge topology has no STA, so every frame here is from WIFI_IF_AP. */
extern esp_err_t esp_wifi_remote_channel_rx(void *h, void *buffer,
                                            void *buff_to_free, size_t len);
esp_err_t esp_wifi_remote_channel_rx(void *h, void *buffer,
                                     void *buff_to_free, size_t len) {
  (void)h;
  return avb_wifi_rx_cb(buffer, (uint16_t)len, buff_to_free);
}
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

/* Open the four per-protocol L2TAP fds for one port and bind them to
 * the port's interface key. Works for any netif (Eth or Wi-Fi); the
 * caller does the medium-specific MAC / promiscuous setup separately. */
static int avb_net_init_port_l2tap(avb_state_s *state, int port_index) {
  const char *iface = state->port[port_index].eth_interface;
  if (!iface || iface[0] == '\0') {
    avberr("port[%d]: empty interface name", port_index);
    return ERROR;
  }
  for (int i = 0; i < AVB_NUM_PROTOCOLS; i++) {
    int fd = open("/dev/net/tap", 0);
    state->port[port_index].l2if[i] = fd;
    if (fd < 0) {
      avberr("port[%d]: failed to create l2if[%d]: %d", port_index, i, errno);
      return ERROR;
    }
    if (ioctl(fd, L2TAP_S_INTF_DEVICE, iface) < 0) {
      avberr("port[%d]: bind fd %d to %s failed: %d", port_index, fd, iface,
             errno);
      return ERROR;
    }
    uint16_t ethertype;
    switch (i) {
    case AVTP: ethertype = ethertype_avtp; break;
    case MSRP: ethertype = ethertype_msrp; break;
    case MVRP: ethertype = ethertype_mvrp; break;
    case VLAN: ethertype = ethertype_vlan; break;
    default: avberr("Invalid protocol index"); return ERROR;
    }
    if (ioctl(fd, L2TAP_S_RCV_FILTER, &ethertype) < 0) {
      avberr("port[%d]: set ethertype filter on fd %d: %d", port_index, fd,
             errno);
      return ERROR;
    }
    avbinfo("port[%d]: L2TAP fd %d for ethertype %x", port_index, fd,
            ethertype);
  }
  return OK;
}

/* Initialize the network interfaces for all configured ports.
 *
 * Port-0 medium can be ethernet (wired endpoint or bridge) or wifi
 * (c6 endpoint, Phase 6b). On the wifi path we skip L2TAP fds
 * (L2TAP is Ethernet-only) and pull the MAC via esp_netif — the
 * frame TX/RX path uses esp_wifi_internal_* APIs instead, wired
 * later in this function.
 *
 * When NUM_PORTS > 1, port 1 is the second medium (typically wifi
 * for the bridge's SoftAP).
 */
int avb_net_init(avb_state_s *state) {
  /* ---------- Port 0 ---------- */
  if (state->port[0].medium == avb_port_medium_ethernet) {
    /* L2TAP fds are kept even on the bridge build: the per-protocol
     * fds are used by the TX path (avb_net_send_to writes the frame
     * via write(l2if[ethertype], ...) which routes to esp_eth_transmit).
     * The L2TAP RX filter is benign here because the netif glue's
     * eth_input_to_netif handler — the only caller of
     * esp_vfs_l2tap_eth_filter_frame — is replaced below by
     * avb_unified_rx_cb via esp_eth_update_input_path_info, so the
     * RX filter never fires. */
    if (avb_net_init_port_l2tap(state, 0) != OK) {
      return ERROR;
    }
    esp_eth_handle_t eth_handle;
    if (ioctl(state->port[0].l2if[0], L2TAP_G_DEVICE_DRV_HNDL, &eth_handle) <
        0) {
      avberr("Failed to get eth_handle: %d", errno);
      return ERROR;
    }
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR,
                  &state->port[0].internal_mac_addr);
  } else if (state->port[0].medium == avb_port_medium_wifi) {
    /* Wi-Fi endpoint or AP. No L2TAP — esp_vfs_l2tap is Ethernet-only
     * in IDF. MAC + netif handle come from esp_netif (medium-agnostic);
     * data-plane RX/TX is wired further down via esp_wifi_internal_*. */
    s_eth_netif =
        esp_netif_get_handle_from_ifkey(state->port[0].eth_interface);
    if (!s_eth_netif) {
      avbwarn("port[0]: no netif for if_key '%s'",
              state->port[0].eth_interface);
    }
    if (s_eth_netif && esp_netif_get_mac(s_eth_netif,
                                         state->port[0].internal_mac_addr) !=
                           ESP_OK) {
      avbwarn("port[0]: esp_netif_get_mac on '%s' failed",
              state->port[0].eth_interface);
    }
    avbinfo("port[0]: WIFI MAC %02x:%02x:%02x:%02x:%02x:%02x",
            state->port[0].internal_mac_addr[0],
            state->port[0].internal_mac_addr[1],
            state->port[0].internal_mac_addr[2],
            state->port[0].internal_mac_addr[3],
            state->port[0].internal_mac_addr[4],
            state->port[0].internal_mac_addr[5]);
  }

  /* ---------- Port 1+ : Wi-Fi (or other media) ---------- */
#if CONFIG_ESP_AVB_NUM_PORTS > 1
  if (state->port[1].medium == avb_port_medium_wifi) {
    /* Wi-Fi: skip L2TAP (esp_vfs_l2tap is Ethernet-only in IDF) and
     * mirror the port[0]-wifi path — resolve the netif by if_key, pull
     * the MAC via esp_netif. RX/TX wiring happens further down via
     * esp_wifi_internal_reg_rxcb / esp_wifi_internal_tx. */
    s_wifi_netif = esp_netif_get_handle_from_ifkey(
        state->port[1].eth_interface);
    if (!s_wifi_netif) {
      avbwarn("port[1]: no netif for if_key '%s'",
              state->port[1].eth_interface);
    }
    if (s_wifi_netif &&
        esp_netif_get_mac(s_wifi_netif,
                          state->port[1].internal_mac_addr) != ESP_OK) {
      avbwarn("port[1]: esp_netif_get_mac on '%s' failed",
              state->port[1].eth_interface);
    }
    avbinfo("port[1]: WIFI_AP MAC %02x:%02x:%02x:%02x:%02x:%02x",
            state->port[1].internal_mac_addr[0],
            state->port[1].internal_mac_addr[1],
            state->port[1].internal_mac_addr[2],
            state->port[1].internal_mac_addr[3],
            state->port[1].internal_mac_addr[4],
            state->port[1].internal_mac_addr[5]);
  } else if (state->port[1].medium == avb_port_medium_ethernet) {
    /* Second Ethernet — same setup pattern as port 0 (not exercised
     * in current bridge designs). */
    if (avb_net_init_port_l2tap(state, 1) != OK) {
      return ERROR;
    }
  }
#endif

  /* Create the control frame RX queue (shared across ports). */
  s_ctrl_rx_queue = xQueueCreate(CTRL_RX_QUEUE_DEPTH, sizeof(ctrl_rx_pkt_t));
  if (!s_ctrl_rx_queue) {
    avberr("Failed to create ctrl_rx_queue");
    return ERROR;
  }
  state->ctrl_rx_queue = s_ctrl_rx_queue;

  /* Register the unified RX callback for port 0. On Ethernet this is
   * the EMAC's input-path hook; on Wi-Fi (c6 endpoint) it's the
   * esp_wifi_internal_reg_rxcb shim that adapts the Wi-Fi RX buffer
   * ownership model and re-enters avb_unified_rx_cb. */
  if (state->port[0].medium == avb_port_medium_ethernet) {
    s_eth_netif = esp_netif_get_handle_from_ifkey(state->config.eth_interface);
    esp_eth_update_input_path_info(state->config.eth_handle, avb_unified_rx_cb,
                                   s_eth_netif);
    avbinfo("Unified EMAC RX dispatcher registered");
  } else {
    esp_err_t r = esp_wifi_internal_reg_rxcb(0 /* WIFI_IF_STA */,
                                             avb_wifi_rx_cb);
    if (r != ESP_OK) {
      avberr("esp_wifi_internal_reg_rxcb failed: %s", esp_err_to_name(r));
      return ERROR;
    }
    avbinfo("Unified Wi-Fi RX dispatcher registered on WIFI_IF_STA");
  }

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE
  s_bridge_state = state;

  /* Bridge port[1] = Wi-Fi AP. Wi-Fi-side ingress is wired up at link
   * time via the WEAK esp_wifi_remote_channel_rx override below
   * (esp_wifi_internal_reg_rxcb doesn't work here — esp_wifi_remote
   * uses esp_wifi_remote_net2.c which has its own private s_rx_fn[]
   * that internal_reg_rxcb doesn't touch, so the registration would
   * be a no-op). All frames coming up from the C6 coprocessor arrive
   * via esp_wifi_remote_channel_rx → avb_wifi_rx_cb → avb_unified_rx_cb
   * with eth_handle=NULL (= ingress_port 1). */
#if CONFIG_ESP_AVB_NUM_PORTS > 1
  if (state->port[1].medium == avb_port_medium_wifi) {
    avbinfo("Bridge L2 forwarder armed: Eth(port0) <-> Wi-Fi-AP(port1) "
            "(via esp_wifi_remote_channel_rx override)");
  } else {
    avbwarn("Bridge build but port[1] medium is not wifi; "
            "Wi-Fi-side ingress will not bridge");
  }
#endif
#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

  return OK;
}

/* Create an Ethernet frame */
void avb_create_eth_frame(uint8_t *eth_frame, eth_addr_t *dest_addr,
                          avb_state_s *state, ethertype_t ethertype, void *msg,
                          uint16_t msg_len, uint8_t *vlan_id) {
  struct eth_hdr eth_hdr = {.type = htons(ethertype)};
  uint16_t vid = vlan_id ? octets_to_uint(vlan_id, 2) : 0;
  uint16_t prio = state->msrp_mappings[0].priority;
  if (vlan_id) {
    for (uint16_t i = 0; i < state->msrp_mappings_count; i++) {
      if (memcmp(vlan_id, state->msrp_mappings[i].vlan_id, 2) == 0) {
        prio = state->msrp_mappings[i].priority;
        break;
      }
    }
  }
  struct eth_vlan_hdr eth_vlan_hdr = {.prio_vid =
                                          htons((prio << 13) | (vid & 0x0FFF)),
                                      .tpid = htons(ethertype_avtp)};
  memcpy(&eth_hdr.dest.addr, dest_addr, ETH_ADDR_LEN);
  memcpy(&eth_hdr.src.addr, state->port[0].internal_mac_addr, ETH_ADDR_LEN);
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

  // Create the Ethernet frame
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len,
                       NULL);

  /* Wi-Fi port: L2TAP is Ethernet-only, so we bypass the per-ethertype
   * fds and push the frame straight at the Wi-Fi driver. */
  if (state->port[0].medium == avb_port_medium_wifi) {
    return (avb_net_transmit_raw(NULL, eth_frame, sizeof(eth_frame)) == ESP_OK)
               ? (int)sizeof(eth_frame)
               : ERROR;
  }

  // Get the L2IF for the given ethertype
  int l2if;
  switch (ethertype) {
  case ethertype_avtp:
    l2if = state->port[0].l2if[AVTP];
    break;
  case ethertype_msrp:
    l2if = state->port[0].l2if[MSRP];
    break;
  case ethertype_mvrp:
    l2if = state->port[0].l2if[MVRP];
    break;
  case ethertype_vlan:
    l2if = state->port[0].l2if[VLAN];
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

  // Create the Ethernet frame
  avb_create_eth_frame(eth_frame, dest_addr, state, ethertype, msg, msg_len,
                       vlan_id);

  /* Wi-Fi port: L2TAP is Ethernet-only — go straight to wifi raw TX. */
  if (state->port[0].medium == avb_port_medium_wifi) {
    return (avb_net_transmit_raw(NULL, eth_frame, sizeof(eth_frame)) == ESP_OK)
               ? (int)sizeof(eth_frame)
               : ERROR;
  }

  int l2if;
  switch (ethertype) {
  case ethertype_avtp:
    l2if = state->port[0].l2if[AVTP];
    break;
  case ethertype_msrp:
    l2if = state->port[0].l2if[MSRP];
    break;
  case ethertype_mvrp:
    l2if = state->port[0].l2if[MVRP];
    break;
  case ethertype_vlan:
    l2if = state->port[0].l2if[VLAN];
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

/* esp_wifi_internal_tx — raw L2 frame send on a Wi-Fi interface.
 * Declared in esp_private/wifi.h but we forward-declare here so
 * esp_avb doesn't gain a hard esp_wifi PRIV_REQUIRES. The link
 * resolves on Wi-Fi-capable targets (esp_wifi is in the build via
 * the application's own deps); on Ethernet-only targets like
 * esp32p4 the symbol is provided by esp_wifi_remote's stub layer
 * and would error at runtime if called — but we only call it from
 * the wifi branch below, which an ethernet build never enters. */
extern esp_err_t esp_wifi_internal_tx(int wifi_if, void *buffer, size_t len);

/* Fast-path raw frame TX. See avb.h docstring for rationale.
 *
 * Branch on the caller's eth_handle: a non-NULL handle means the
 * port has an EMAC-backed transport (esp_eth — wired endpoint or
 * bridge), and we go through esp_eth_transmit for direct DMA. A NULL
 * handle means the wifi data-plane (c6 wireless endpoint, Phase
 * 6b.2), and we shoot the frame at the Wi-Fi STA interface via
 * esp_wifi_internal_tx. */
esp_err_t avb_net_transmit_raw(esp_eth_handle_t eth_handle,
                               const void *frame, size_t frame_len) {
  if (eth_handle != NULL) {
    return esp_eth_transmit(eth_handle, (void *)frame, frame_len);
  }
  /* Wi-Fi path: esp_wifi_internal_tx synchronously enqueues the buffer
   * to the Wi-Fi MAC's TX descriptor ring. Some IDF builds reference
   * the caller's buffer until tx_done — passing a stack buffer (as
   * avb_net_send_to does for control frames) is a use-after-stack-pop
   * footgun. Copy into the heap so the buffer outlives the caller's
   * stack frame; the wifi driver will free or we let it leak per its
   * ownership model. To stay safe, malloc + esp_wifi_internal_tx +
   * free on error path; if tx succeeds, the wifi driver currently
   * copies internally so freeing here is OK. */
  /* Wi-Fi path: esp_wifi_internal_tx synchronously enqueues to the
   * wifi MAC's TX descriptors. Some IDF builds reference the caller's
   * buffer until tx_done — copy into heap so the buffer outlives the
   * caller's stack frame. */
  void *buf = malloc(frame_len);
  if (!buf) {
    return ESP_ERR_NO_MEM;
  }
  memcpy(buf, frame, frame_len);
  esp_err_t r = esp_wifi_internal_tx(0 /* WIFI_IF_STA */, buf, frame_len);
  free(buf);
  return r;
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
