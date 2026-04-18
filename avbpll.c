/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * Milan media-clock PLL.
 *
 * Architecture
 * ============
 * The PLL closes a loop between our local I2S MCLK and the AVB network's
 * gPTP-synchronised media clock. Three pieces live here:
 *
 *   1. Measurement
 *      The stream-in drain callback (avtp.c) feeds
 *      state->media_clock.i2s_bytes_written — the cumulative count of
 *      bytes the I2S DMA actually consumed. Compared against gPTP time
 *      this directly reveals our MCLK rate:
 *          expected_bytes = 288000 * elapsed_gPTP_seconds
 *          ppm_error      = (actual_bytes - expected) * 1e6 / expected
 *      Two windows are tracked: the last 5 s (instant, noisy, shows
 *      transient behaviour) and since-baseline (cumulative, averages out
 *      the ~1 ms drain-cycle aliasing that otherwise injects ±200 ppm
 *      of measurement noise).
 *
 *   2. Control
 *      The cumulative ppm error is exactly the opposite of the
 *      correction we need to apply, so a simple proportional loop
 *      (gain = 1.0) suffices — no integrator required because the input
 *      is already integrated. After applying a correction we reset the
 *      baseline so the next window measures error relative to the new
 *      hardware state instead of blending corrected and uncorrected
 *      periods.
 *
 *   3. Hardware tuning
 *      The bottom of this file is a thin abstraction over the ESP32-P4
 *      APLL — conceptually the same role played by a Cirrus Logic
 *      CS2000 in a discrete AVB design: a fractional-N PLL whose output
 *      ratio can be shifted via a software-writable coefficient, with
 *      an internal sigma-delta modulator absorbing the coefficient
 *      steps without audible glitches. Porting to a different SoC or
 *      to an external clock chip only touches the `mclk_hw_*`
 *      functions; the measurement and control layers above stay
 *      SoC-independent.
 */
#include "avb.h"
#include "esp_timer.h"
#include "hal/clk_tree_ll.h" /* CLK_LL_APLL_MIN_HZ */
#include "soc/rtc.h" /* rtc_clk_apll_coeff_calc / _set */
#include <inttypes.h>
#include <stdatomic.h>

static const char *TAG = "avb_pll";

/* ---------------------------------------------------------------------------
 * Hardware backend: ESP32-P4 APLL
 *
 * We bypass ESP-IDF's periph_rtc_apll_freq_set() and write the APLL
 * sigma-delta coefficients directly. Reason: the public helper has a
 * refcount gate that silently drops the retune when more than one
 * peripheral owns APLL — which is exactly our situation, since both
 * the I2S TX and RX channels acquire APLL at init (ref_cnt = 2). Using
 * rtc_clk_apll_coeff_calc + rtc_clk_apll_coeff_set skips that gate and
 * reprograms the SDM directly; the modulator absorbs the coefficient
 * step without audio glitches within the audio band.
 *
 * The CS2000 analogy: everything in this section is what the chip would
 * do for us internally, driven by I2C ratio writes. Replace this section
 * with an I2C driver and the rest of avbpll.c compiles unchanged.
 * ------------------------------------------------------------------------- */

static struct {
  bool initialised;
  uint32_t nominal_mclk_hz;
  uint32_t mclk_div;         /* I2S driver's internal MCLK divider */
  uint32_t nominal_apll_hz;  /* = nominal_mclk_hz * mclk_div */
  uint32_t actual_apll_hz;   /* last frequency programmed */
} s_hw;

static int mclk_hw_init(uint32_t nominal_mclk_hz) {
  if (nominal_mclk_hz == 0)
    return -1;
  /* Match the ESP-IDF I2S driver's divider selection so our retune
   * targets the same APLL frequency the driver computed. */
  int mclk_div = (int)((CLK_LL_APLL_MIN_HZ / nominal_mclk_hz) + 1);
  if (mclk_div < 2)
    mclk_div = 2;
  s_hw.nominal_mclk_hz = nominal_mclk_hz;
  s_hw.mclk_div = (uint32_t)mclk_div;
  s_hw.nominal_apll_hz = nominal_mclk_hz * (uint32_t)mclk_div;
  s_hw.actual_apll_hz = s_hw.nominal_apll_hz;
  s_hw.initialised = true;
  ESP_LOGI(TAG, "apll backend ready: mclk=%" PRIu32
               " Hz div=%" PRIu32 " apll_target=%" PRIu32 " Hz",
           nominal_mclk_hz, s_hw.mclk_div, s_hw.nominal_apll_hz);
  return 0;
}

/* Apply an absolute ppm offset (Q16) to the nominal MCLK.
 *   actual_mclk = nominal_mclk * (1 + ppm/1e6)
 * Positive → faster, negative → slower. Repeated calls replace the prior
 * tuning; they're not incremental. */
static int mclk_hw_tune_ppm_q16(int32_t ppm_q16) {
  if (!s_hw.initialised)
    return -1;
  int64_t delta_hz =
      ((int64_t)s_hw.nominal_apll_hz * (int64_t)ppm_q16) /
      (1000000LL * 65536LL);
  int64_t target_signed = (int64_t)s_hw.nominal_apll_hz + delta_hz;
  if (target_signed <= 0)
    return -1;
  uint32_t target_hz = (uint32_t)target_signed;

  uint32_t o_div = 0, sdm0 = 0, sdm1 = 0, sdm2 = 0;
  uint32_t real_hz =
      rtc_clk_apll_coeff_calc(target_hz, &o_div, &sdm0, &sdm1, &sdm2);
  if (real_hz == 0) {
    ESP_LOGW(TAG, "apll coeff calc rejected: target=%" PRIu32 " out of range",
             target_hz);
    return -1;
  }
  rtc_clk_apll_coeff_set(o_div, sdm0, sdm1, sdm2);
  s_hw.actual_apll_hz = real_hz;
  return 0;
}

static void mclk_hw_deinit(void) {
  /* APLL coefficient state stays in hardware; nothing to free here. */
  s_hw.initialised = false;
}

/* ---------------------------------------------------------------------------
 * Measurement + control loop — SoC-independent.
 * ------------------------------------------------------------------------- */

/* Baseline snapshot used to compute cumulative ppm from the start of a
 * measurement window (or from the last applied correction). */
static struct {
  bool valid;
  uint32_t startup_skipped_ticks;
  uint64_t base_i2s_bytes;
  uint64_t base_gptp_ns;
  uint64_t prev_i2s_bytes;
  uint64_t prev_gptp_ns;
  int64_t next_print_us;
  int64_t next_correction_us;
} s_pll;

/* Nominal byte-rate of the stream we're measuring: 48000 frames × 2 ch ×
 * 3 bytes/sample after listener downmix. TODO: make this a runtime value
 * when we support rates other than 48 kHz. */
#define AVB_PLL_NOMINAL_BYTERATE 288000

/* Minimum absolute correction to apply — avoid wearing on the APLL
 * coefficient for sub-ppm noise. */
#define AVB_PLL_CORRECTION_DEADBAND_Q16 (1 * 65536 / 2) /* 0.5 ppm */

/* How long to let the cumulative window build up before applying a
 * correction. The measurement noise is dominated by ~1 ms drain-cycle
 * phase aliasing on the 5 s log boundary; that aliasing averages down
 * as 1/sqrt(N) with the number of 5 s windows covered, so a 60 s
 * correction window cuts the per-correction noise roughly in half
 * compared to 30 s. 60 s + gain 0.5 gives an effective PLL time
 * constant of ~120 s — slow enough to ignore noise, still fast enough
 * to track a real crystal drift over a few minutes. */
#define AVB_PLL_CORRECTION_INTERVAL_US (60 * 1000 * 1000)

/* Proportional gain, Q16. At 1.0 (65536) a single correction would
 * theoretically zero out the observed error — but that also pumps the
 * full measurement noise into the applied value, causing a slow random
 * walk. Gain < 1.0 damps that: the loop still converges (exponentially
 * with time constant ≈ interval / gain) and each correction contributes
 * only a fraction of the measured ppm, so measurement noise averages
 * out across many corrections. */
#define AVB_PLL_GAIN_Q16 (65536 / 2) /* 0.5 */

/* Safety clamp on the total applied correction. A healthy crystal is
 * ≤±100 ppm; anything larger is almost certainly a measurement error
 * (e.g. drain underruns at stream startup) and we'd rather leave MCLK
 * near nominal than drag it far off on bad data. */
#define AVB_PLL_MAX_APPLIED_PPM_Q16 ((int32_t)(100 * 65536))

/* Safety clamp on a single correction step. Prevents a single noisy
 * cumulative sample from yanking MCLK; the loop can still converge
 * quickly for real offsets by accumulating small steps. */
#define AVB_PLL_MAX_STEP_PPM_Q16 ((int32_t)(10 * 65536))

/* Reject clearly-anomalous cumulative measurements — e.g. the cumul
 * briefly explodes to tens of thousands of ppm when gPTP resyncs after
 * a system event. Reset the baseline instead of applying. */
#define AVB_PLL_CUMUL_SANITY_LIMIT_PPM_Q16 ((int32_t)(500 * 65536))

/* Skip the first couple of tick windows after a stream connects. The
 * very first seconds include the pre-fill burst and any drain-underrun
 * recovery, which would produce a large spurious "cumulative" error. */
#define AVB_PLL_STARTUP_SKIP_TICKS 3

/* Read i2s_bytes_written + gPTP together; returns false when either is
 * unavailable (stream not running, gPTP not synced). */
static bool read_sample(avb_state_s *state, uint64_t *bytes_out,
                        uint64_t *gptp_ns_out) {
  struct timespec gptp;
  if (clock_gettime(CLOCK_PTP_SYSTEM, &gptp) != 0)
    return false;
  uint64_t bytes = atomic_load_explicit(&state->media_clock.i2s_bytes_written,
                                        memory_order_relaxed);
  *bytes_out = bytes;
  *gptp_ns_out =
      (uint64_t)gptp.tv_sec * 1000000000ULL + (uint64_t)gptp.tv_nsec;
  return true;
}

static int32_t compute_ppm_q16(uint64_t bytes_delta, uint64_t elapsed_ns) {
  if (elapsed_ns == 0 || bytes_delta == 0)
    return 0;
  int64_t expected =
      (int64_t)AVB_PLL_NOMINAL_BYTERATE * (int64_t)elapsed_ns / 1000000000LL;
  int64_t byte_error = (int64_t)bytes_delta - expected;
  return (int32_t)((byte_error * 1000000LL * (1LL << 16)) / expected);
}

static void print_stats(avb_state_s *state, int32_t inst_ppm_q16,
                        int32_t cumul_ppm_q16) {
  uint32_t crf_n = state->media_clock.crf_samples;
  uint32_t aaf_n = state->media_clock.aaf_samples;
  int32_t crf_mean =
      crf_n > 0 ? (int32_t)(state->media_clock.crf_drift_sum_ns / crf_n) : 0;
  int32_t aaf_mean =
      aaf_n > 0 ? (int32_t)(state->media_clock.aaf_drift_sum_ns / aaf_n) : 0;
  int32_t inst_centippm = (int32_t)(((int64_t)inst_ppm_q16 * 100) >> 16);
  int32_t cumul_centippm = (int32_t)(((int64_t)cumul_ppm_q16 * 100) >> 16);
  int32_t applied_centippm =
      (int32_t)(((int64_t)state->media_clock.pll_applied_ppm_q16 * 100) >> 16);
  /* Report actual HW frequency achieved vs nominal, so we can confirm the
   * APLL retune is really taking effect (derived from the last value the
   * coefficient calc returned). */
  int32_t hw_delta_hz = (int32_t)s_hw.actual_apll_hz - (int32_t)s_hw.nominal_apll_hz;
  int32_t hw_applied_ppm = s_hw.nominal_apll_hz > 0
                               ? (int32_t)((int64_t)hw_delta_hz * 1000000LL /
                                           s_hw.nominal_apll_hz)
                               : 0;
  avbinfo("MCLK: crf n=%lu drift=%lldns mean=%ldns min=%ldns max=%ldns | "
          "aaf n=%lu drift=%ldns mean=%ldns min=%ldns max=%ldns | "
          "pll inst=%ld.%02ld cumul=%ld.%02ld applied=%ld.%02ld hw=%ld ppm",
          crf_n, state->media_clock.crf_last_drift_ns, crf_mean,
          state->media_clock.crf_drift_min_ns,
          state->media_clock.crf_drift_max_ns, aaf_n,
          state->media_clock.aaf_last_drift_ns, aaf_mean,
          state->media_clock.aaf_drift_min_ns,
          state->media_clock.aaf_drift_max_ns, inst_centippm / 100,
          (inst_centippm < 0 ? -inst_centippm : inst_centippm) % 100,
          cumul_centippm / 100,
          (cumul_centippm < 0 ? -cumul_centippm : cumul_centippm) % 100,
          applied_centippm / 100,
          (applied_centippm < 0 ? -applied_centippm : applied_centippm) % 100,
          hw_applied_ppm);

  /* Drift-stat windows reset per log; last_drift preserved as latest */
  state->media_clock.crf_drift_sum_ns = 0;
  state->media_clock.crf_drift_min_ns = 0;
  state->media_clock.crf_drift_max_ns = 0;
  state->media_clock.crf_samples = 0;
  state->media_clock.aaf_drift_sum_ns = 0;
  state->media_clock.aaf_drift_min_ns = 0;
  state->media_clock.aaf_drift_max_ns = 0;
  state->media_clock.aaf_samples = 0;
}

int avb_pll_init(uint32_t nominal_mclk_hz) {
  s_pll.valid = false;
  s_pll.next_print_us = 0;
  s_pll.next_correction_us = 0;
  return mclk_hw_init(nominal_mclk_hz);
}

void avb_pll_deinit(void) {
  mclk_hw_deinit();
  s_pll.valid = false;
}

void avb_pll_tick(avb_state_s *state) {
  if (!state)
    return;
  int64_t now_us = esp_timer_get_time();
  if (now_us < s_pll.next_print_us)
    return;
  s_pll.next_print_us = now_us + 5 * 1000 * 1000; /* 5 s */

  uint64_t bytes_now, gptp_now_ns;
  if (!read_sample(state, &bytes_now, &gptp_now_ns)) {
    s_pll.valid = false;
    return;
  }

  /* First valid sample — seed the baselines, nothing to compare yet */
  if (!s_pll.valid) {
    s_pll.base_i2s_bytes = bytes_now;
    s_pll.base_gptp_ns = gptp_now_ns;
    s_pll.prev_i2s_bytes = bytes_now;
    s_pll.prev_gptp_ns = gptp_now_ns;
    s_pll.valid = true;
    s_pll.startup_skipped_ticks = 0;
    s_pll.next_correction_us = now_us + AVB_PLL_CORRECTION_INTERVAL_US;
    return;
  }

  /* gPTP discontinuity guard. If more than 2× the tick interval elapsed
   * since the last sample, the gPTP clock likely jumped (resync after a
   * stream interruption, ptpd adjustment, etc.). Re-seed the baseline
   * so the next window measures against the new clock state rather than
   * producing garbage ±thousands-of-ppm cumulative numbers. */
  uint64_t elapsed_since_prev_ns = gptp_now_ns - s_pll.prev_gptp_ns;
  if (elapsed_since_prev_ns > 20ULL * 1000000000ULL) {
    ESP_LOGW(TAG, "gPTP discontinuity (%llu ns gap) — resetting baseline",
             elapsed_since_prev_ns);
    s_pll.base_i2s_bytes = bytes_now;
    s_pll.base_gptp_ns = gptp_now_ns;
    s_pll.prev_i2s_bytes = bytes_now;
    s_pll.prev_gptp_ns = gptp_now_ns;
    s_pll.next_correction_us = now_us + AVB_PLL_CORRECTION_INTERVAL_US;
    return;
  }

  /* Skip the first few tick windows — the pre-fill burst and any drain
   * underrun recovery produce huge spurious "cumulative" ppm errors
   * that would drive a garbage correction. Re-seed the baseline after
   * each skipped tick so the first real measurement starts clean. */
  if (s_pll.startup_skipped_ticks < AVB_PLL_STARTUP_SKIP_TICKS) {
    s_pll.startup_skipped_ticks++;
    s_pll.base_i2s_bytes = bytes_now;
    s_pll.base_gptp_ns = gptp_now_ns;
    s_pll.prev_i2s_bytes = bytes_now;
    s_pll.prev_gptp_ns = gptp_now_ns;
    s_pll.next_correction_us = now_us + AVB_PLL_CORRECTION_INTERVAL_US;
    return;
  }

  /* Instant (5 s window) and cumulative ppm */
  int32_t inst_ppm_q16 =
      compute_ppm_q16(bytes_now - s_pll.prev_i2s_bytes,
                      gptp_now_ns - s_pll.prev_gptp_ns);
  int32_t cumul_ppm_q16 =
      compute_ppm_q16(bytes_now - s_pll.base_i2s_bytes,
                      gptp_now_ns - s_pll.base_gptp_ns);
  state->media_clock.pll_last_ppm_error_q16 = inst_ppm_q16;
  state->media_clock.pll_cumulative_ppm_error_q16 = cumul_ppm_q16;

  /* Sliding window — next 'instant' is vs now */
  s_pll.prev_i2s_bytes = bytes_now;
  s_pll.prev_gptp_ns = gptp_now_ns;

  /* Periodically fold the measured cumulative error into the applied
   * correction. cumul_ppm is already integrated so a P-controller with
   * gain 1.0 converges in one step if the plant is linear:
   *   new_applied = old_applied + (-cumul_ppm_observed)
   * After applying, reset the baseline so future measurements don't
   * blend corrected and uncorrected time. */
  /* Sanity: if cumul is wildly out of range (common after a gPTP resync
   * that fell just below the discontinuity threshold, or after an EMAC
   * watchdog event), the measurement is noise — reset the baseline and
   * don't apply anything this cycle. */
  if (cumul_ppm_q16 > AVB_PLL_CUMUL_SANITY_LIMIT_PPM_Q16 ||
      cumul_ppm_q16 < -AVB_PLL_CUMUL_SANITY_LIMIT_PPM_Q16) {
    ESP_LOGW(TAG,
             "cumul %ld ppm out of sanity range — rejecting, resetting baseline",
             (long)(cumul_ppm_q16 / 65536));
    s_pll.base_i2s_bytes = bytes_now;
    s_pll.base_gptp_ns = gptp_now_ns;
    s_pll.next_correction_us = now_us + AVB_PLL_CORRECTION_INTERVAL_US;
    return;
  }

  if (now_us >= s_pll.next_correction_us &&
      (cumul_ppm_q16 > AVB_PLL_CORRECTION_DEADBAND_Q16 ||
       cumul_ppm_q16 < -AVB_PLL_CORRECTION_DEADBAND_Q16)) {
    /* Proportional controller with gain < 1 to damp measurement noise */
    int32_t step_q16 =
        (int32_t)(((int64_t)cumul_ppm_q16 * AVB_PLL_GAIN_Q16) >> 16);
    /* Clamp per-step — single-window noise can still be big. */
    if (step_q16 > AVB_PLL_MAX_STEP_PPM_Q16)
      step_q16 = AVB_PLL_MAX_STEP_PPM_Q16;
    if (step_q16 < -AVB_PLL_MAX_STEP_PPM_Q16)
      step_q16 = -AVB_PLL_MAX_STEP_PPM_Q16;
    int32_t new_applied = state->media_clock.pll_applied_ppm_q16 - step_q16;
    /* Clamp total applied — anything beyond ±100 ppm is a bad measurement,
     * not a real crystal offset for any commercially sane oscillator. */
    if (new_applied > AVB_PLL_MAX_APPLIED_PPM_Q16)
      new_applied = AVB_PLL_MAX_APPLIED_PPM_Q16;
    if (new_applied < -AVB_PLL_MAX_APPLIED_PPM_Q16)
      new_applied = -AVB_PLL_MAX_APPLIED_PPM_Q16;
    if (mclk_hw_tune_ppm_q16(new_applied) == 0) {
      state->media_clock.pll_applied_ppm_q16 = new_applied;
      /* Reset baseline so the next cumulative measurement is
       * relative to the new hardware state. */
      s_pll.base_i2s_bytes = bytes_now;
      s_pll.base_gptp_ns = gptp_now_ns;
    }
    s_pll.next_correction_us = now_us + AVB_PLL_CORRECTION_INTERVAL_US;
  }

  /* Only print if something was actually measured */
  if (state->media_clock.crf_samples > 0 ||
      state->media_clock.aaf_samples > 0) {
    print_stats(state, inst_ppm_q16, cumul_ppm_q16);
  }
}
