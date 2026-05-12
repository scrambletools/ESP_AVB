/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Bridge-role L2 forwarding task.
 *
 * Phase 5a skeleton — defines the public init/stop surface and the
 * forwarding task entry point. The actual EtherType dispatch
 * (PTP terminated per port; MSRP/MVRP terminated and re-declared via
 * the MRP MAP machinery; AVTP frames classified by VLAN PCP and
 * routed through avbfqtss; everything else best-effort; Class A from
 * Ethernet dropped on Wi-Fi egress per the Class-B-only Wi-Fi policy)
 * is filled in incrementally in Phase 5b.
 *
 * esp_avb terminology: "port" indexes into avb_state_s.port[]. With
 * CONFIG_ESP_AVB_NUM_PORTS=2 (bridge build) port[0] is Ethernet and
 * port[1] is Wi-Fi.
 */

#include "avbbridge.h"

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "avb_bridge";

static TaskHandle_t s_bridge_task = NULL;
static volatile bool s_bridge_stop = false;

static void bridge_task(void *arg) {
  avb_state_s *state = (avb_state_s *)arg;
  ESP_LOGI(TAG, "bridge L2 forwarding task started; ports=%d",
           CONFIG_ESP_AVB_NUM_PORTS);

  while (!s_bridge_stop) {
    /* Phase 5b: poll/select across both ports' L2TAP fds, dispatch
     * by EtherType, route through FQTSS for AVTP frames. Skeleton
     * just sleeps for now. */
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ESP_LOGI(TAG, "bridge L2 forwarding task exiting");
  s_bridge_task = NULL;
  vTaskDelete(NULL);
}

int avb_bridge_init(avb_state_s *state) {
  if (s_bridge_task != NULL) {
    ESP_LOGW(TAG, "bridge already running");
    return 0;
  }
  s_bridge_stop = false;
  /* Pin to core 1 alongside AVB-OUT so the EMAC RX cores stay free.
   * Phase 5b may rebalance once shaper load is measured. */
  BaseType_t r = xTaskCreatePinnedToCore(bridge_task, "AVB-BR", 8192,
                                         state, 22, &s_bridge_task, 1);
  return (r == pdPASS) ? 0 : -1;
}

void avb_bridge_stop(avb_state_s *state) {
  (void)state;
  s_bridge_stop = true;
}

/* EtherType / VLAN-PCP based forwarding decision tree. Pure function;
 * pure-software CBS shaping decisions live in avbfqtss.c, MSRP/MVRP
 * attribute propagation lives in msrp.c — this just decides which
 * subsystem owns the frame.
 *
 * Per IEEE 802.1Q-2022 / 802.1AS-2020:
 *   PTP (0x88f7)    — link-local; each port runs its own time-aware
 *                      bridge logic; the bridge does NOT forward.
 *   MSRP (0x22ea),
 *   MVRP (0x88f5)   — MRP attribute frames; the bridge MAP terminates
 *                      and re-declares per port, never forwards raw.
 *   AVTP control
 *   (0x22f0)        — ADP / AECP / ACMP / MAAP. We forward to the
 *                      other port best-effort (no FQTSS shaping —
 *                      these are low-rate and untagged).
 *   AVTP stream
 *   (0x8100 VLAN)   — Class A (PCP 3) and Class B (PCP 2) audio /
 *                      video stream data. Routed through FQTSS.
 *                      Class A from Ethernet to Wi-Fi is dropped per
 *                      the v1 plan (Wi-Fi admits Class B only).
 *   Other           — IP, ARP, etc. — forwarded best-effort. */

#define ETHERTYPE_PTP  0x88f7
#define ETHERTYPE_AVTP 0x22f0
#define ETHERTYPE_MSRP 0x22ea
#define ETHERTYPE_MVRP 0x88f5
#define ETHERTYPE_VLAN 0x8100

avb_bridge_disposition_t avb_bridge_classify(int ingress_port,
                                             uint16_t ethertype,
                                             uint8_t vlan_pcp) {
  avb_bridge_disposition_t d = {0};
  /* Egress is always the OTHER port. Asserts NUM_PORTS == 2 in the
   * bridge build; with NUM_PORTS=1 the bridge can't actually forward
   * anything (Phase 5 carries NUM_PORTS=1 until the AP project
   * bumps it). */
  d.egress_port = (uint8_t)((ingress_port == 0) ? 1 : 0);

  switch (ethertype) {
  case ETHERTYPE_PTP:
  case ETHERTYPE_MSRP:
  case ETHERTYPE_MVRP:
    /* Terminated locally by ptpd / msrp.c MRP MAP. The existing
     * per-protocol handlers in avb_process_rx_message do the work; the
     * bridge does NOT forward raw frames. */
    d.verdict = AVB_BRIDGE_TERMINATE;
    return d;

  case ETHERTYPE_AVTP:
    /* AVTP control (untagged) — ADP / AECP / ACMP / MAAP. Forward
     * best-effort to the other port (no shaping — low-rate). The
     * bridge endpoint also lets these reach its own protocol handler
     * via the existing dispatch in avb_process_rx_message; the
     * forwarder copies the frame onto the egress wire in parallel. */
    d.verdict = AVB_BRIDGE_BRIDGE;
    d.shaped = false;
    return d;

  case ETHERTYPE_VLAN:
    /* VLAN-tagged AVTP stream data. PCP encodes the SR class:
     *   PCP 3 → Class A, PCP 2 → Class B (matches our Kconfig
     *   defaults ESP_AVB_VLAN_PRIO_CLASS_A / _CLASS_B). */
    if (vlan_pcp == CONFIG_ESP_AVB_VLAN_PRIO_CLASS_A) {
      d.sr_class = AVB_SR_CLASS_A;
    } else if (vlan_pcp == CONFIG_ESP_AVB_VLAN_PRIO_CLASS_B) {
      d.sr_class = AVB_SR_CLASS_B;
    } else {
      /* Unrecognized PCP on a VLAN frame — best-effort forward. */
      d.verdict = AVB_BRIDGE_BRIDGE;
      d.shaped = false;
      return d;
    }
    /* Class A → Wi-Fi: dropped per v1 plan (Wi-Fi admits Class B
     * only). egress_port == 1 means "Wi-Fi" here because the bridge
     * is wired as port[0] = Ethernet, port[1] = Wi-Fi. */
    if (d.sr_class == AVB_SR_CLASS_A && d.egress_port == 1) {
      d.verdict = AVB_BRIDGE_DROP;
      return d;
    }
    d.verdict = AVB_BRIDGE_BRIDGE;
    d.shaped = true;
    return d;

  default:
    /* Untagged non-AVB traffic (ARP, IP, etc.) — best-effort. */
    d.verdict = AVB_BRIDGE_BRIDGE;
    d.shaped = false;
    return d;
  }
}

/* ----------------------------------------------------------------------
 * Bridge-role stubs for endpoint-only ATDECC functions.
 *
 * The bridge has no ATDECC entity (per the plan: transparent L2 bridge
 * per Milan/802.1Q semantics). atdecc.c is excluded from the bridge
 * build, but avb.c contains unconditional calls into ATDECC dispatch
 * paths from the periodic-send and RX-message routines. Rather than
 * thread #ifdefs through avb.c, satisfy the linker with no-op stubs
 * here. Each function's purpose is annotated for future readers; if
 * the bridge ever grows a Controller-class entity (Phase 8 v2), these
 * stubs become the entry points to a slim ATDECC subset.
 * ---------------------------------------------------------------------- */

int avb_send_adp_entity_available(avb_state_s *state) {
  (void)state;
  return 0;
}

int avb_send_aecp_unsol_get_counters(avb_state_s *state,
                                     aem_desc_type_t descriptor_type,
                                     uint16_t index) {
  (void)state;
  (void)descriptor_type;
  (void)index;
  return 0;
}

void avb_update_avb_interface_from_ptp(avb_state_s *state) { (void)state; }

int avb_process_adp(avb_state_s *state, adp_message_s *msg,
                    eth_addr_t *src_addr) {
  (void)state;
  (void)msg;
  (void)src_addr;
  return 0;
}

int avb_process_aecp(avb_state_s *state, aecp_message_u *msg,
                     eth_addr_t *src_addr) {
  (void)state;
  (void)msg;
  (void)src_addr;
  return 0;
}

int avb_process_acmp(avb_state_s *state, acmp_message_s *msg) {
  (void)state;
  (void)msg;
  return 0;
}

void avb_periodic_fast_connect(avb_state_s *state) { (void)state; }

void avb_process_inflight_timeouts(avb_state_s *state) { (void)state; }

int avb_find_entity_by_addr(avb_state_s *state, eth_addr_t *entity_addr,
                            avb_entity_type_t entity_type) {
  (void)state;
  (void)entity_addr;
  (void)entity_type;
  return -1;
}

int avb_send_cvu_srp_attr(avb_state_s *state, void *attr, int attr_list_len,
                          const char *label) {
  (void)state;
  (void)attr;
  (void)attr_list_len;
  (void)label;
  return 0;
}

/* avbcodec.c stubs — bridge has no codec / I2S. */
esp_err_t avb_config_i2s(avb_state_s *state) {
  (void)state;
  return ESP_OK;
}

esp_err_t avb_config_codec(avb_state_s *state) {
  (void)state;
  return ESP_OK;
}

int16_t avb_codec_quantize_tenth_db(const codec_control_range_s *ranges,
                                    bool gain, int16_t value_tenth_db) {
  (void)ranges;
  (void)gain;
  return value_tenth_db;
}

void avb_codec_set_vol(avb_state_s *state, float db) {
  (void)state;
  (void)db;
}

void avb_codec_set_mic_gain(avb_state_s *state, float db) {
  (void)state;
  (void)db;
}

/* avbpll.c stubs — bridge has no media-clock PLL. */
void avb_pll_tick(avb_state_s *state) { (void)state; }
void avb_pll_print_stats(avb_state_s *state) { (void)state; }

#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */
