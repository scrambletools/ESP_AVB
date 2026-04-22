/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * ESP_AVB Component — CPU/task diagnostic
 *
 * Temporary investigative tool. Prints per-task %CPU, per-core
 * utilization, and stack high-water marks every AVB_STATS_WINDOW_MS.
 * Computes deltas between two snapshots of uxTaskGetSystemState so
 * each window is independent.
 *
 * Enable by calling avb_cpu_stats_start() during init. Remove or
 * #ifdef-gate once the performance investigation is complete.
 */

#include "avb.h"
#include "esp_timer.h"
#include "soc/emac_dma_struct.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define AVB_STATS_MAX_TASKS 32
#define AVB_STATS_WINDOW_MS 10000

typedef struct {
  UBaseType_t tn; /* xTaskNumber — stable per task, used to match snapshots */
  uint32_t rt;    /* ulRunTimeCounter at snapshot time */
} snap_entry_t;

static snap_entry_t s_prev[AVB_STATS_MAX_TASKS];
static UBaseType_t s_prev_n = 0;
static int64_t s_prev_at_us = 0;

/* Previous cumulative productive-work microseconds for AVB-OUT. Delta
 * against the window gives the "real" CPU AVB-OUT needs (excludes
 * busy-wait). */
static uint64_t s_prev_avbout_work_us = 0;

/* Previous cumulative PTP frame count seen in avb_unified_rx_cb. Delta is
 * the count of PTP frames that emac_rx delivered to our callback this
 * window — compare against ptpd's rx_sync to see if drops are in our
 * callback or in the L2TAP filter/queue path below it. */
static uint32_t s_prev_ptp_rx_seen = 0;

/* Starts with "IDLE" — we match by prefix because ESP-IDF's idle task
 * name is "IDLE0" / "IDLE1" on dual-core builds. */
static bool is_idle_task(const char *name) {
  return name && name[0] == 'I' && name[1] == 'D' && name[2] == 'L' &&
         name[3] == 'E';
}

static void avb_cpu_stats_tick(void) {
  TaskStatus_t cur[AVB_STATS_MAX_TASKS];
  uint32_t cur_total;
  UBaseType_t cur_n =
      uxTaskGetSystemState(cur, AVB_STATS_MAX_TASKS, &cur_total);
  int64_t now = esp_timer_get_time();
  int64_t window_us = now - s_prev_at_us;

  /* On the first call we just save the snapshot and wait for the next
   * window — there's no prior baseline to diff against. */
  if (s_prev_at_us == 0 || window_us < 1000) {
    goto save_baseline;
  }

  uint32_t idle0_delta = 0, idle1_delta = 0;
  avbinfo("CPU-STATS: window=%lldms (%u tasks)", window_us / 1000,
          (unsigned)cur_n);
  avbinfo("  %-16s core prio  %%cpu  stackHW  state", "task");

  for (UBaseType_t i = 0; i < cur_n; i++) {
    /* Find matching prior entry by stable xTaskNumber. */
    uint32_t prev_rt = 0;
    bool found = false;
    for (UBaseType_t j = 0; j < s_prev_n; j++) {
      if (s_prev[j].tn == cur[i].xTaskNumber) {
        prev_rt = s_prev[j].rt;
        found = true;
        break;
      }
    }
    uint32_t drt = found ? (cur[i].ulRunTimeCounter - prev_rt) : 0;

    /* Runtime counter is sourced from esp_timer (µs) in ESP-IDF by
     * default, matching window_us units — so percentage is direct. */
    uint32_t pct = (uint32_t)((uint64_t)drt * 100u / (uint64_t)window_us);

    const char *core_s;
    switch (cur[i].xCoreID) {
    case 0:
      core_s = "0";
      break;
    case 1:
      core_s = "1";
      break;
    default:
      core_s = "*";
      break; /* tskNO_AFFINITY / unpinned */
    }

    const char *state_s;
    switch (cur[i].eCurrentState) {
    case eRunning:
      state_s = "run";
      break;
    case eReady:
      state_s = "rdy";
      break;
    case eBlocked:
      state_s = "blk";
      break;
    case eSuspended:
      state_s = "sus";
      break;
    case eDeleted:
      state_s = "del";
      break;
    default:
      state_s = "?";
      break;
    }

    avbinfo("  %-16s %4s %4u  %3u%%  %7u  %s", cur[i].pcTaskName, core_s,
            (unsigned)cur[i].uxCurrentPriority, pct,
            (unsigned)cur[i].usStackHighWaterMark, state_s);

    if (is_idle_task(cur[i].pcTaskName)) {
      if (cur[i].xCoreID == 0)
        idle0_delta = drt;
      else if (cur[i].xCoreID == 1)
        idle1_delta = drt;
    }
  }

  /* Per-core busy % derived directly from IDLE runtime on that core.
   * Each core had `window_us` of wall time available; IDLE time is
   * time not spent running anything else. */
  uint32_t cpu0_busy =
      100 - (uint32_t)((uint64_t)idle0_delta * 100u / (uint64_t)window_us);
  uint32_t cpu1_busy =
      100 - (uint32_t)((uint64_t)idle1_delta * 100u / (uint64_t)window_us);
  avbinfo("  ====> CPU0 busy=%u%% idle=%u%%  |  CPU1 busy=%u%% idle=%u%%",
          cpu0_busy, 100 - cpu0_busy, cpu1_busy, 100 - cpu1_busy);

  /* Break AVB-OUT's CPU time down into "real work" vs. busy-wait spin.
   * work_pct = productive µs accumulated this window / window µs × 100.
   * The busy-wait portion = (AVB-OUT total %CPU) − work_pct. */
  uint64_t cur_work = avb_stream_out_work_us_total();
  if (cur_work >= s_prev_avbout_work_us) {
    uint64_t dwork = cur_work - s_prev_avbout_work_us;
    uint32_t work_pct =
        (uint32_t)((dwork * 100ull) / (uint64_t)window_us);
    avbinfo("  ====> AVB-OUT real work=%u%% (rest of its slot is busy-wait)",
            work_pct);
  }
  s_prev_avbout_work_us = cur_work;

  /* PTP frames seen in our RX callback this window. ptpd's rx_sync from
   * the ptpd-late line should be close to this for syncs; a gap here
   * means L2TAP tail-dropped on full queue. */
  uint32_t cur_ptp = avb_net_ptp_rx_seen();
  uint32_t dptp = cur_ptp - s_prev_ptp_rx_seen;
  avbinfo("  ====> PTP frames seen in emac_rx_cb this window: %u", dptp);
  s_prev_ptp_rx_seen = cur_ptp;

  /* EMAC DMA hardware missed-frame counter. missed_fc counts frames the
   * DMA discarded because the Host Receive Buffer was unavailable (the
   * RX descriptor ring was full). overflow_fc counts frames lost inside
   * the MTL FIFO (before DMA). Both fields clear on read. If missed_fc
   * is non-zero we're losing packets at the NIC because emac_rx isn't
   * draining fast enough — the most likely explanation for PTP frames
   * visible on the wire but never reaching avb_unified_rx_cb. */
  uint32_t dma_miss = EMAC_DMA.dmamissedfr.val;
  avbinfo("  ====> EMAC DMA missed_fc=%u overflow_fc=%u bmfc_ovf=%u bfoc_ovf=%u",
          (unsigned)(dma_miss & 0xFFFF),
          (unsigned)((dma_miss >> 17) & 0x7FF),
          (unsigned)((dma_miss >> 16) & 1),
          (unsigned)((dma_miss >> 28) & 1));

save_baseline:
  s_prev_n = cur_n < AVB_STATS_MAX_TASKS ? cur_n : AVB_STATS_MAX_TASKS;
  for (UBaseType_t i = 0; i < s_prev_n; i++) {
    s_prev[i].tn = cur[i].xTaskNumber;
    s_prev[i].rt = cur[i].ulRunTimeCounter;
  }
  s_prev_at_us = now;
}

static void avb_cpu_stats_task(void *arg) {
  (void)arg;
  /* Initial settle before the first snapshot. */
  vTaskDelay(pdMS_TO_TICKS(5000));
  avb_cpu_stats_tick(); /* seed baseline */
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(AVB_STATS_WINDOW_MS));
    avb_cpu_stats_tick();
  }
}

void avb_cpu_stats_start(void) {
  /* Pin to core 0. Unpinned placed us on core 1 behind AVB-OUT's prio-24
   * busy-wait, which meant we only woke up every ~22 s instead of the
   * requested 10 s — making the "window" math in the per-task %CPU
   * numbers misleading. Core 0 has >20 % idle headroom even under
   * streaming, so a low-priority sampler there is reliable. Priority 5
   * sits above AVB-NVS (3) and PTPD (2) so neither can delay our wake,
   * and well below real-time work so we never disturb it. */
  xTaskCreatePinnedToCore(avb_cpu_stats_task, "AVB-STATS", 4096, NULL, 5,
                          NULL, 0);
}
