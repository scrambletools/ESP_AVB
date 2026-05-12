/*
 * Copyright 2026 Scramble Tools
 * License: MIT
 *
 * 802.1Qav credit-based shaper for the bridge role.
 *
 * Per IEEE 802.1Q-2018 Annex L, each SR-class queue maintains a
 * "credit" value with these dynamics:
 *
 *   idle_slope     bandwidth allocated to the class, bytes/s
 *                  (Phase 5d will set this from MSRP admission)
 *   port_tx_rate   link rate, bytes/s
 *   send_slope     idle_slope - port_tx_rate    (negative)
 *   creditMax      worst-case interference burst, bytes
 *   creditMin      worst-case credit deficit, bytes
 *
 *   while queue non-empty AND credit ≥ 0  →  transmit a frame:
 *      during the transmit interval, credit decreases at send_slope
 *   while queue empty OR transmission blocked (higher class on wire):
 *      credit increases at idle_slope, clamped to creditMax
 *   credit clamps to creditMin on the floor
 *
 * Since the bridge has only software-driven egress (no hardware
 * Qav assist on either ESP32-P4 EMAC or ESP-Hosted-Wi-Fi), we
 * approximate the continuous slope dynamics with a periodic
 * (esp_timer-driven) tick that integrates Δcredit = idle_slope · Δt
 * between ticks. Frame transmission is treated as instantaneous —
 * we deduct frame_size_bytes from credit at dequeue time. This
 * captures the long-run bandwidth bound; per-frame burst latency is
 * bounded by creditMax which we set to one MTU.
 *
 * Per-port-per-class state and queues live in s_class[]; one worker
 * task drains all of them. The worker is pinned to core 0 at high
 * priority so it interleaves with EMAC RX rather than competing with
 * AVB-OUT on core 1. Tick interval is 100 µs — fine enough that a
 * 1518-byte burst (12 144 bits) is granular at our slopes (idle_slope
 * up to 0.75 × 1 Gbps = 750 Mbps → ~94 KB/s = 9.4 bytes/100µs).
 */

#include "avbbridge.h"

#ifdef CONFIG_ESP_AVB_ROLE_BRIDGE

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "avb_fqtss";

#define FQTSS_TICK_US     100
#define FQTSS_QUEUE_DEPTH 64
#define FQTSS_MTU_BYTES   1518

/* Frame envelope passed through the per-class queues. The bridge
 * forwarder (Phase 5b) will call avb_fqtss_enqueue with a frame copy;
 * the worker eventually frees it after egress. */
typedef struct {
  uint8_t port_index;
  uint8_t sr_class;
  uint16_t length;
  uint8_t *payload; /* heap-allocated copy, freed after transmit */
} fqtss_frame_t;

typedef struct {
  /* Configuration — touched by avb_fqtss_set_idle_slope. */
  int64_t idle_slope_bps;
  int64_t link_rate_bps;
  int64_t credit_max_x256_bytes;
  int64_t credit_min_x256_bytes;
  /* Runtime state — touched only by the worker task. */
  int64_t credit_x256_bytes; /* fixed-point Q8 byte units */
  int64_t last_tick_us;
  QueueHandle_t queue;
} fqtss_class_state_t;

static fqtss_class_state_t
    s_class[CONFIG_ESP_AVB_NUM_PORTS][AVB_SR_CLASS_COUNT];
static TaskHandle_t s_worker = NULL;
static volatile bool s_stop = false;

/* Advance credit by dt_us according to idle_slope. Reads idle_slope
 * non-atomically; a torn read off a slope-set write is harmless (the
 * next tick reconverges within one FQTSS_TICK_US window). */
static inline void fqtss_credit_advance(fqtss_class_state_t *c, int64_t dt_us) {
  /* delta_x256_bytes = idle_slope_bps · dt_us · 256 / (8 · 1e6)
   *                  = idle_slope_bps · dt_us / 31250
   * Folding constants keeps the int64 product safely bounded:
   *   0.75 Gbps × 100 µs ≈ 75e6 · 100 ≈ 7.5e9 → fits int64 trivially. */
  int64_t delta = c->idle_slope_bps * dt_us / 31250LL;
  c->credit_x256_bytes += delta;
  if (c->credit_x256_bytes > c->credit_max_x256_bytes) {
    c->credit_x256_bytes = c->credit_max_x256_bytes;
  }
}

/* If queue has a frame and credit allows, dequeue + transmit one.
 * Phase 5b owns the actual egress path; this skeleton frees the
 * buffer to keep the queue draining during local testing.
 * Returns 1 if a frame was sent, 0 otherwise. */
static int fqtss_try_send_one(int port_index, int cls) {
  fqtss_class_state_t *c = &s_class[port_index][cls];
  fqtss_frame_t frame;
  if (xQueuePeek(c->queue, &frame, 0) != pdTRUE) {
    return 0;
  }
  /* Credit threshold: must be ≥ 0 in real bytes. credit_x256_bytes is
   * fixed-point ×256, so the threshold is 0. */
  if (c->credit_x256_bytes < 0) {
    return 0;
  }
  if (xQueueReceive(c->queue, &frame, 0) != pdTRUE) {
    return 0;
  }
  /* Phase 5b will hand the payload to the egress port's socket here.
   * Skeleton: drop on floor. */
  free(frame.payload);
  /* Deduct frame bytes from credit; clamp to creditMin. */
  c->credit_x256_bytes -= (int64_t)frame.length * 256;
  if (c->credit_x256_bytes < c->credit_min_x256_bytes) {
    c->credit_x256_bytes = c->credit_min_x256_bytes;
  }
  return 1;
}

static void fqtss_worker(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "FQTSS worker started; tick=%d µs", FQTSS_TICK_US);
  int64_t last_us = esp_timer_get_time();
  while (!s_stop) {
    int64_t now = esp_timer_get_time();
    int64_t dt = now - last_us;
    last_us = now;
    for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
      for (int cl = 0; cl < AVB_SR_CLASS_COUNT; cl++) {
        fqtss_credit_advance(&s_class[p][cl], dt);
        /* Drain everything the credit window allows this tick. */
        while (fqtss_try_send_one(p, cl)) {
        }
      }
    }
    /* vTaskDelay rounds to FreeRTOS tick granularity. With the
     * default 100 Hz tick (10 ms) this dramatically over-relaxes the
     * intended 100 µs cadence. AVB workloads typically run with
     * tick=1000 Hz (1 ms) — confirmed in ESP-AVB-Endpoint's
     * sdkconfig.defaults via CONFIG_FREERTOS_HZ. The skeleton
     * accepts ms-level granularity; Phase 5d may switch to an
     * esp_timer-driven semaphore for sub-ms ticks if jitter
     * measurements warrant it. */
    vTaskDelay(1);
  }
  ESP_LOGI(TAG, "FQTSS worker exiting");
  s_worker = NULL;
  vTaskDelete(NULL);
}

int avb_fqtss_init(avb_state_s *state) {
  for (int p = 0; p < CONFIG_ESP_AVB_NUM_PORTS; p++) {
    /* Link rate. Ethernet ports default to 1 Gbps; Wi-Fi ports start
     * at 0 (unknown) and Phase 5b will call set_idle_slope once the
     * AP has measured the negotiated MCS rate. */
    int64_t link_bps =
        (state->port[p].medium == avb_port_medium_ethernet) ? 1000000000LL : 0LL;
    for (int cl = 0; cl < AVB_SR_CLASS_COUNT; cl++) {
      fqtss_class_state_t *cs = &s_class[p][cl];
      cs->link_rate_bps = link_bps;
      cs->idle_slope_bps = 0; /* admission updates fill this in later */
      cs->credit_max_x256_bytes = (int64_t)FQTSS_MTU_BYTES * 256;
      cs->credit_min_x256_bytes = -(int64_t)FQTSS_MTU_BYTES * 256;
      cs->credit_x256_bytes = 0;
      cs->last_tick_us = esp_timer_get_time();
      cs->queue = xQueueCreate(FQTSS_QUEUE_DEPTH, sizeof(fqtss_frame_t));
      if (!cs->queue) {
        ESP_LOGE(TAG, "queue alloc failed (port %d class %d)", p, cl);
        return -1;
      }
    }
  }
  s_stop = false;
  /* Pinned to core 0 alongside emac_rx; priority above the AVB main
   * task (21) but below the EMAC RX task (22). Intent: shaper drains
   * promptly without preempting RX. */
  BaseType_t r = xTaskCreatePinnedToCore(fqtss_worker, "AVB-FQ", 4096, NULL, 23,
                                         &s_worker, 0);
  if (r != pdPASS) {
    ESP_LOGE(TAG, "worker task create failed");
    return -1;
  }
  ESP_LOGI(TAG, "FQTSS shaper init: %d port(s) x %d class(es)",
           CONFIG_ESP_AVB_NUM_PORTS, AVB_SR_CLASS_COUNT);
  return 0;
}

void avb_fqtss_stop(avb_state_s *state) {
  (void)state;
  s_stop = true;
  /* Worker self-deletes. Queue handles leak across stop in skeleton —
   * Phase 5b adds proper teardown when the bridge actually shuts down. */
}

int avb_fqtss_enqueue(int port_index, int sr_class, const void *frame,
                      size_t length) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) {
    return -1;
  }
  if (sr_class < 0 || sr_class >= AVB_SR_CLASS_COUNT) {
    return -1;
  }
  if (length == 0 || length > FQTSS_MTU_BYTES) {
    return -1;
  }
  fqtss_class_state_t *cs = &s_class[port_index][sr_class];
  if (cs->idle_slope_bps == 0) {
    /* Class has no admitted bandwidth; reject so caller can route to
     * best-effort instead. */
    return -2;
  }
  fqtss_frame_t f;
  f.port_index = (uint8_t)port_index;
  f.sr_class = (uint8_t)sr_class;
  f.length = (uint16_t)length;
  f.payload = malloc(length);
  if (!f.payload) {
    return -1;
  }
  memcpy(f.payload, frame, length);
  if (xQueueSend(cs->queue, &f, 0) != pdTRUE) {
    free(f.payload);
    return -3; /* queue full */
  }
  return 0;
}

int avb_fqtss_set_idle_slope(int port_index, int sr_class,
                             int64_t idle_slope_bps) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) {
    return -1;
  }
  if (sr_class < 0 || sr_class >= AVB_SR_CLASS_COUNT) {
    return -1;
  }
  fqtss_class_state_t *cs = &s_class[port_index][sr_class];
  if (cs->link_rate_bps > 0 && idle_slope_bps > cs->link_rate_bps) {
    return -1;
  }
  /* Two non-atomic 64-bit writes; reader (the worker) tolerates a
   * brief mid-write read by reconverging within one tick. */
  cs->idle_slope_bps = idle_slope_bps;
  return 0;
}

int avb_fqtss_set_link_rate(int port_index, int64_t link_rate_bps) {
  if (port_index < 0 || port_index >= CONFIG_ESP_AVB_NUM_PORTS) {
    return -1;
  }
  if (link_rate_bps < 0) {
    return -1;
  }
  for (int cl = 0; cl < AVB_SR_CLASS_COUNT; cl++) {
    s_class[port_index][cl].link_rate_bps = link_rate_bps;
  }
  return 0;
}

#endif /* CONFIG_ESP_AVB_ROLE_BRIDGE */
