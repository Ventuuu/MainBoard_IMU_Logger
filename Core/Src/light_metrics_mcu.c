/*
 * light_metrics_mcu.c
 *
 * On-MCU light exposure metrics derived from AS7341 full-spectrum data.
 *
 * Acquisition model
 * -----------------
 * Raw AS7341 channel counts are accumulated over LIGHT_METRICS_WINDOW
 * consecutive 10 Hz readings (= 1 second).  At the end of each window the
 * accumulated sums are divided by the window size to produce per-channel
 * averages, and all metrics are computed once from those averages.  The
 * accumulator is then cleared for the next window.
 *
 * This approach:
 *   - Reduces sample-to-sample noise without an FIR/IIR filter.
 *   - Keeps the ISR cost low: only additions at 10 Hz, one division burst
 *     (9 uint32 divides) at 1 Hz.
 *   - Preserves the existing BLE send rate: the ISR sends the stale
 *     instantaneous metrics at 100 Hz using the last closed window values.
 *
 * Metric definitions (all based on averaged channels, no CLEAR)
 * -------------------------------------------------------------
 *   BlueIndex              = avg_F3  (saturated to uint16)
 *   BlueFrac    (Q15)      = avg_F3 / sum_avg(F1..F8)
 *   SunLikeIndex (Q15)     = (avg_F7 + avg_F8) / sum_avg(F1..F8)
 *   uvRisk                 = (avg_F1+avg_F2+avg_F3)^2 / sum_avg(F1..F8)
 *   blueWeightedIll        = avg_F3^2 / sum_avg(F1..F8)
 *
 * Q15 overflow protection
 * -----------------------
 * The intermediate product  numerator * 32767  can exceed 32 bits when
 * channel counts are large (AS7341 max count ~65535 per channel, sum up
 * to ~524280 for 8 channels).  All Q15 numerator multiplications are
 * performed in uint64_t before the final uint32_t division, ensuring no
 * overflow for any physically reachable input.
 */

#include "light_metrics_mcu.h"

/* -------------------------------------------------------------------------
 * Window accumulator
 * ---------------------------------------------------------------------- */

typedef struct {
    uint32_t F1, F2, F3, F4, F5, F6, F7, F8;
    uint8_t  count;   /* samples collected so far [0 .. LIGHT_METRICS_WINDOW-1] */
} SpectrumAccum;

static SpectrumAccum s_accum = {0};

/* -------------------------------------------------------------------------
 * Instantaneous metric state (last closed window)
 * ---------------------------------------------------------------------- */

static uint16_t s_blueIndex      = 0U;
static uint16_t s_blueFrac_q15   = 0U;
static uint16_t s_sunLikeIdx_q15 = 0U;
static uint32_t s_uvRisk         = 0U;
static uint32_t s_blueWeightedIll= 0U;

/* -------------------------------------------------------------------------
 * Cumulative dose accumulators (rectangle rule, 1 Hz update)
 * ---------------------------------------------------------------------- */

static uint64_t s_uvDoseAccum                 = 0ULL;
static uint64_t s_blueExposureAccum           = 0ULL;
static uint64_t s_circadianDoseAccum          = 0ULL;
static uint64_t s_blueExposureArtificialAccum = 0ULL;
static uint64_t s_blueExposureNaturalAccum    = 0ULL;

#define LM_CIRCADIAN_START_H  (20U)
#define LM_CIRCADIAN_END_H    (24U)

/* -------------------------------------------------------------------------
 * Internal: compute metrics from one averaged spectrum
 * ---------------------------------------------------------------------- */

static void compute_metrics(uint32_t F1, uint32_t F2, uint32_t F3,
                             uint32_t F4, uint32_t F5, uint32_t F6,
                             uint32_t F7, uint32_t F8,
                             const Time_Struct *timestamp,
                             uint16_t mains_hz)
{
    uint32_t sum_all = F1 + F2 + F3 + F4 + F5 + F6 + F7 + F8;
    if (sum_all == 0U) {
        s_blueIndex       = 0U;
        s_blueFrac_q15    = 0U;
        s_sunLikeIdx_q15  = 0U;
        s_uvRisk          = 0U;
        s_blueWeightedIll = 0U;
        return;
    }

    /* BlueIndex = F3 only (480 nm), saturated to uint16 */
    s_blueIndex = (F3 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)F3;

    /*
     * Q15 ratios — cast numerator operand to uint64_t BEFORE multiplying
     * by 32767 to prevent overflow.  The maximum unreduced numerator is
     * 2 * 65535 * 32767 ≈ 4.29e9, which overflows uint32_t (max ~4.29e9).
     * Using uint64_t gives a comfortable 64-bit ceiling.
     */

    /* BlueFrac Q15 = F3 / sum(F1..F8) */
    s_blueFrac_q15 = (uint16_t)(((uint64_t)F3 * 32767ULL) / (uint64_t)sum_all);

    /* SunLikeIndex Q15 = (F7+F8) / sum(F1..F8) */
    uint32_t redSum = F7 + F8;
    s_sunLikeIdx_q15 = (uint16_t)(((uint64_t)redSum * 32767ULL) / (uint64_t)sum_all);

    /*
     * uvRisk = (F1+F2+F3)^2 / sum(F1..F8)
     *
     * The squared numerator can reach (3 * 65535)^2 ≈ 3.8e10, which needs
     * uint64_t for the intermediate product before dividing back to uint32_t.
     */
    uint32_t uvSum = F1 + F2 + F3;
    s_uvRisk = (uint32_t)(((uint64_t)uvSum * (uint64_t)uvSum) / (uint64_t)sum_all);

    /*
     * blueWeightedIll = F3^2 / sum(F1..F8)
     * F3 max = 65535 → F3^2 = 4.29e9, fits in uint64_t safely.
     */
    s_blueWeightedIll = (uint32_t)(((uint64_t)F3 * (uint64_t)F3) / (uint64_t)sum_all);

    /* --- Cumulative accumulators (1 Hz, rectangle rule) --- */
    s_uvDoseAccum       += (uint64_t)s_uvRisk;
    s_blueExposureAccum += (uint64_t)s_blueWeightedIll;

    uint8_t is_artificial = (mains_hz == 50U) || (mains_hz == 60U);
    if (is_artificial) {
        s_blueExposureArtificialAccum += (uint64_t)s_blueWeightedIll;
    } else {
        s_blueExposureNaturalAccum += (uint64_t)s_blueWeightedIll;
    }

    /* Circadian gate: artificial light between 20:00 and 24:00 */
    uint32_t seconds_of_day = ((uint32_t)timestamp->hh * 3600U)
                            + ((uint32_t)timestamp->mm * 60U)
                            + (uint32_t)timestamp->ss;
    if (is_artificial
            && (seconds_of_day >= (LM_CIRCADIAN_START_H * 3600U))
            && (seconds_of_day <  (LM_CIRCADIAN_END_H   * 3600U))) {
        s_circadianDoseAccum += (uint64_t)s_blueWeightedIll;
    }
}

/* -------------------------------------------------------------------------
 * Public: reset
 * ---------------------------------------------------------------------- */

void LightMetrics_Reset(void)
{
    s_accum = (SpectrumAccum){0};

    s_blueIndex       = 0U;
    s_blueFrac_q15    = 0U;
    s_sunLikeIdx_q15  = 0U;
    s_uvRisk          = 0U;
    s_blueWeightedIll = 0U;

    s_uvDoseAccum                 = 0ULL;
    s_blueExposureAccum           = 0ULL;
    s_circadianDoseAccum          = 0ULL;
    s_blueExposureArtificialAccum = 0ULL;
    s_blueExposureNaturalAccum    = 0ULL;
}

/* -------------------------------------------------------------------------
 * Public: update (call at 10 Hz)
 * ---------------------------------------------------------------------- */

uint8_t LightMetrics_Update(const AS7341_Spectrum *spectrum,
                             const Time_Struct     *timestamp,
                             uint16_t               mains_hz)
{
    if (spectrum == NULL || timestamp == NULL) {
        return 0U;
    }

    /*
     * Channel mapping in AS7341_Spectrum:
     *   ch[0]  → F1  (415 nm, violet)
     *   ch[1]  → F2  (445 nm, deep blue)
     *   ch[2]  → F3  (480 nm, blue)       ← BlueIndex source
     *   ch[3]  → F4  (515 nm, cyan)
     *   ch[4]  → Clear (pass 1)           — not used in metrics
     *   ch[5]  → NIR   (pass 1)           — not used in metrics
     *   ch[6]  → F5  (555 nm, green)
     *   ch[7]  → F6  (590 nm, yellow)
     *   ch[8]  → F7  (630 nm, orange-red)
     *   ch[9]  → F8  (680 nm, red)
     *   ch[10] → Clear (pass 2)           — not used in metrics
     *   ch[11] → NIR   (pass 2)           — not used in metrics
     */
    s_accum.F1 += spectrum->ch[0];
    s_accum.F2 += spectrum->ch[1];
    s_accum.F3 += spectrum->ch[2];
    s_accum.F4 += spectrum->ch[3];
    s_accum.F5 += spectrum->ch[6];
    s_accum.F6 += spectrum->ch[7];
    s_accum.F7 += spectrum->ch[8];
    s_accum.F8 += spectrum->ch[9];
    s_accum.count++;

    if (s_accum.count < LIGHT_METRICS_WINDOW) {
        return 0U;  /* window not yet complete */
    }

    /* Window complete: compute per-channel averages */
    uint32_t avg_F1 = s_accum.F1 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F2 = s_accum.F2 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F3 = s_accum.F3 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F4 = s_accum.F4 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F5 = s_accum.F5 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F6 = s_accum.F6 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F7 = s_accum.F7 / LIGHT_METRICS_WINDOW;
    uint32_t avg_F8 = s_accum.F8 / LIGHT_METRICS_WINDOW;

    compute_metrics(avg_F1, avg_F2, avg_F3, avg_F4,
                    avg_F5, avg_F6, avg_F7, avg_F8,
                    timestamp, mains_hz);

    /* Reset accumulator for next window */
    s_accum = (SpectrumAccum){0};

    return 1U;  /* metrics refreshed */
}

/* -------------------------------------------------------------------------
 * Instantaneous getters
 * ---------------------------------------------------------------------- */

uint16_t LightMetrics_GetBlueIndex(void)               { return s_blueIndex;        }
uint16_t LightMetrics_GetBlueFracQ15(void)             { return s_blueFrac_q15;     }
uint16_t LightMetrics_GetSunLikeIndexQ15(void)         { return s_sunLikeIdx_q15;   }
uint32_t LightMetrics_GetUvRisk(void)                  { return s_uvRisk;           }
uint32_t LightMetrics_GetBlueWeightedIlluminance(void) { return s_blueWeightedIll;  }

/* -------------------------------------------------------------------------
 * Cumulative getters
 * ---------------------------------------------------------------------- */

uint64_t LightMetrics_GetUvDoseAccum(void)                 { return s_uvDoseAccum;                 }
uint64_t LightMetrics_GetBlueExposureAccum(void)           { return s_blueExposureAccum;           }
uint64_t LightMetrics_GetBlueExposureArtificialAccum(void) { return s_blueExposureArtificialAccum; }
uint64_t LightMetrics_GetBlueExposureNaturalAccum(void)    { return s_blueExposureNaturalAccum;    }
uint64_t LightMetrics_GetCircadianDoseAccum(void)          { return s_circadianDoseAccum;          }
