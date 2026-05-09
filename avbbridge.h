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

/* ---- avbfqtss.c: 802.1Qav credit-based shaper ---- */

/* Initialize per-port-per-class CBS queues and start the shaper
 * worker. Reads idle_slope / send_slope / link_rate from
 * avb_srp_admission_s for each (port, class) pair and recomputes when
 * admission state changes. */
int avb_fqtss_init(avb_state_s *state);

void avb_fqtss_stop(avb_state_s *state);

/* Enqueue a frame on the FQTSS shaper. Class A / Class B drive
 * credit-based shaping; everything else is best-effort. Frame
 * ownership transfers on success; on -EAGAIN the caller retains and
 * may retry. */
int avb_fqtss_enqueue(int port_index, int sr_class,
                      const void *frame, size_t length);

/* ---- msrp.c: MSRP bandwidth admission with 75% cap ---- */

/* SR class identifiers, matching IEEE 802.1Q-2018 Table 35-7. Class A
 * is index 0, Class B is index 1; bridge tracks running bandwidth
 * separately for each. */
typedef enum {
  AVB_SR_CLASS_A = 0,
  AVB_SR_CLASS_B = 1,
  AVB_SR_CLASS_COUNT
} avb_sr_class_e;

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
