/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * Bridge-role internal API. Compiled in only when
 * CONFIG_ESP_AVB_ROLE_BRIDGE is selected. Defines the surface that
 * avb.c uses to start / stop the bridge subsystems and that
 * avbbridge.c / avbfqtss.c / msrp.c share.
 */

#ifndef ESP_AVB_BRIDGE_H
#define ESP_AVB_BRIDGE_H

#include "avb.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE

/* SR class identifiers, matching IEEE 802.1Q-2018 Table 35-7. Class A
 * is index 0, Class B is index 1; bridge tracks running bandwidth
 * separately for each. Defined here at the top because both the
 * forwarder (verdict struct) and the admission tables reference it. */
typedef enum {
  AVB_SR_CLASS_A = 0,
  AVB_SR_CLASS_B = 1,
  AVB_SR_CLASS_COUNT
} avb_sr_class_e;

/* ---- avbbridge.c: L2 forwarding ---- */

/* Initialize the bridge's L2 forwarding task. Spawns the worker task
 * pinned to a dedicated core and registers RX hooks on each port's
 * L2TAP fds for the EtherTypes we forward (AVTP, MSRP, MVRP, plus
 * untagged best-effort). Returns 0 on success, negative errno on
 * failure. */
int avb_bridge_init(avb_state_s *state);

/* Stop the bridge forwarding task and release resources. Safe to
 * call from any task context. */
void avb_bridge_stop(avb_state_s *state);

/* Per-frame disposition decision computed in avb_bridge_classify().
 * The forwarding integration in avbnet.c (called from the EMAC RX
 * trampoline when CONFIG_ESP_AVB_ROLE_BRIDGE is selected) acts on
 * this verdict: BRIDGE → enqueue on egress port via FQTSS,
 * TERMINATE → let the existing per-protocol handler run,
 * DROP → free buffer and ignore. */
typedef enum {
  AVB_BRIDGE_DROP = 0,
  AVB_BRIDGE_TERMINATE = 1,
  AVB_BRIDGE_BRIDGE = 2,
} avb_bridge_verdict_e;

typedef struct {
  avb_bridge_verdict_e verdict;
  uint8_t egress_port;     /* meaningful when verdict == BRIDGE */
  avb_sr_class_e sr_class; /* meaningful when verdict == BRIDGE
                              and the frame is a shaped AVTP stream */
  bool shaped;             /* if true, route via FQTSS instead of raw TX */
} avb_bridge_disposition_t;

/* Classify a frame by its (ingress_port, ethertype, vlan_pcp). Pure
 * function, no side effects — feeds the forwarder. The vlan_pcp
 * argument is used only when ethertype == 0x8100 (VLAN-tagged AVTP);
 * for untagged frames pass 0. */
avb_bridge_disposition_t avb_bridge_classify(int ingress_port,
                                             uint16_t ethertype,
                                             uint8_t vlan_pcp);

/* ---- avbfqtss.c: 802.1Qav credit-based shaper ---- */

/* Initialize per-port-per-class CBS queues and start the shaper
 * worker. Reads idle_slope / send_slope / link_rate from
 * avb_srp_admission_s for each (port, class) pair and recomputes when
 * admission state changes. */
int avb_fqtss_init(avb_state_s *state);

void avb_fqtss_stop(avb_state_s *state);

/* Enqueue a frame on the FQTSS shaper. The shaper owns the bytes
 * after a successful enqueue (it copies the payload). Returns:
 *   0   accepted
 *  -1   bad arg / out-of-memory
 *  -2   class has no admitted bandwidth (caller should route to
 *       best-effort instead)
 *  -3   per-class queue full */
int avb_fqtss_enqueue(int port_index, int sr_class,
                      const void *frame, size_t length);

/* Set the idle_slope (bytes-per-second-equivalent in bits-per-second
 * units) for a (port, class) pair. Called from the MSRP admission
 * path when the admitted bandwidth changes. send_slope is derived
 * internally as idle_slope - link_rate. */
int avb_fqtss_set_idle_slope(int port_index, int sr_class,
                             int64_t idle_slope_bps);

/* Update the effective link rate for a port. Used when the Wi-Fi
 * port's MCS rate changes (the AP-side ESP-Hosted re-negotiation),
 * or when an Ethernet port renegotiates. The shaper recomputes
 * send_slope from this. */
int avb_fqtss_set_link_rate(int port_index, int64_t link_rate_bps);

/* ---- msrp.c: MSRP bandwidth admission with 75% cap ---- */

/* Per-port-per-class admission state. Lives inside the bridge
 * subsystem; avb.c does not touch directly. */
typedef struct {
  uint32_t link_rate_bps;     /* effective link rate, Ethernet=1 Gbps,
                                  Wi-Fi=admitted PHY rate (Class B only). */
  uint32_t admitted_bps;      /* sum of admitted talker advertisements. */
  uint32_t cap_bps;           /* 0.75 * link_rate / class_share. */
} avb_admission_class_s;

int avb_srp_admission_init(avb_state_s *state);
void avb_srp_admission_stop(avb_state_s *state);

/* Try to admit a talker advertisement. Returns 0 on success (and
 * updates admitted_bps), -ENOSPC when the request would exceed the
 * 75 % cap (caller should emit MSRP TALKER_FAILED with
 * insufficient_bandwidth_for_traffic_class). */
int avb_srp_admission_try_admit(int port_index, avb_sr_class_e cls,
                                uint32_t request_bps);

/* Release a previously admitted reservation. Safe if not currently
 * admitted (no-ops). */
void avb_srp_admission_release(int port_index, avb_sr_class_e cls,
                               uint32_t request_bps);

#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */

#ifdef __cplusplus
}
#endif

#endif /* ESP_AVB_BRIDGE_H */
