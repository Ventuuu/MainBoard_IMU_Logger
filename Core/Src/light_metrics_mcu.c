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
 * Metric definitions (all Q15 fractions of sum(F1..F8))
 * ------------------------------------------------------
 *   BlueIndex              = avg_F3  (saturated to uint16, raw count)
 *   BlueFrac    (Q15)      = avg_F3             / sum_avg
 *   SunLikeIndex (Q15)     = (avg_F7 + avg_F8)  / sum_avg
 *   uvRisk      (Q15)      = (avg_F1+F2+F3)     / sum_avg
 *   blueWeightedIll (Q15)  = (avg_F3 + avg_F4)  / sum_avg
 *
 * Q15 overflow protection
 * -----------------------
 * Numerator * 32767 can reach ~524280 * 32767 ~ 1.72e10, exceeding uint32_t.
 * All Q15 multiplications are performed in uint64_t before dividing.
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
static uint16_t s_uvRisk_q15     = 0U;
static uint16_t s_blueCyanFrac_q15 = 0U;

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
        s_blueIndex        = 0U;
        s_blueFrac_q15     = 0U;
        s_sunLikeIdx_q15   = 0U;
        s_uvRisk_q15       = 0U;
        s_blueCyanFrac_q15 = 0U;
        return;
    }

    /* BlueIndex = avg_F3 (480 nm), saturated to uint16 */
    s_blueIndex = (F3 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)F3;

    /*
     * All four Q15 ratios share the same pattern:
     *   result = (uint16_t)(((uint64_t)numerator * 32767ULL) / (uint64_t)sum_all)
     *
     * Casting to uint64_t before multiplying by 32767 prevents overflow:
     *   max numerator ~ 3 * 65535 = 196605
     *   196605 * 32767 ~ 6.44e9  > UINT32_MAX (4.29e9)  -- would overflow uint32_t
     *   fits comfortably in uint64_t (max ~1.84e19)
     */

    /* BlueFrac Q15: F3 / sum */
    s_blueFrac_q15 = (uint16_t)(((uint64_t)F3 * 32767ULL) / (uint64_t)sum_all);

    /* SunLikeIndex Q15: (F7+F8) / sum */
    s_sunLikeIdx_q15 = (uint16_t)(((uint64_t)(F7 + F8) * 32767ULL) / (uint64_t)sum_all);

    /* uvRisk Q15: (F1+F2+F3) / sum */
    s_uvRisk_q15 = (uint16_t)(((uint64_t)(F1 + F2 + F3) * 32767ULL) / (uint64_t)sum_all);

    /* blueCyanFrac Q15: (F3+F4) / sum */
    s_blueCyanFrac_q15 = (uint16_t)(((uint64_t)(F3 + F4) * 32767ULL) / (uint64_t)sum_all);

    /* --- Cumulative accumulators (1 Hz, rectangle rule) --- */
    s_uvDoseAccum       += (uint64_t)s_uvRisk_q15;
    s_blueExposureAccum += (uint64_t)s_blueCyanFrac_q15;

    uint8_t is_artificial = (mains_hz == 50U) || (mains_hz == 60U);
    if (is_artificial) {
        s_blueExposureArtificialAccum += (uint64_t)s_blueCyanFrac_q15;
    } else {
        s_blueExposureNaturalAccum += (uint64_t)s_blueCyanFrac_q15;
    }

    /* Circadian gate: artificial light between 20:00 and 24:00 */
    uint32_t seconds_of_day = ((uint32_t)timestamp->hh * 3600U)
                            + ((uint32_t)timestamp->mm * 60U)
                            + (uint32_t)timestamp->ss;
    if (is_artificial
            && (seconds_of_day >= (LM_CIRCADIAN_START_H * 3600U))
            && (seconds_of_day <  (LM_CIRCADIAN_END_H   * 3600U))) {
        s_circadianDoseAccum += (uint64_t)s_blueCyanFrac_q15;
    }
}

/* -------------------------------------------------------------------------
 * Public: reset
 * ---------------------------------------------------------------------- */

void LightMetrics_Reset(void)
{
    s_accum = (SpectrumAccum){0};

    s_blueIndex        = 0U;
    s_blueFrac_q15     = 0U;
    s_sunLikeIdx_q15   = 0U;
    s_uvRisk_q15       = 0U;
    s_blueCyanFrac_q15 = 0U;

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
     *   ch[0]  -> F1  (415 nm, violet)
     *   ch[1]  -> F2  (445 nm, deep blue)
     *   ch[2]  -> F3  (480 nm, blue)       <- BlueIndex source
     *   ch[3]  -> F4  (515 nm, cyan)
     *   ch[4]  -> Clear (pass 1)           -- not used in metrics
     *   ch[5]  -> NIR   (pass 1)           -- not used in metrics
     *   ch[6]  -> F5  (555 nm, green)
     *   ch[7]  -> F6  (590 nm, yellow)
     *   ch[8]  -> F7  (630 nm, orange-red)
     *   ch[9]  -> F8  (680 nm, red)
     *   ch[10] -> Clear (pass 2)           -- not used in metrics
     *   ch[11] -> NIR   (pass 2)           -- not used in metrics
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
        return 0U;
    }

    /* Window complete: compute per-channel averages then metrics */
    compute_metrics(s_accum.F1 / LIGHT_METRICS_WINDOW,
                    s_accum.F2 / LIGHT_METRICS_WINDOW,
                    s_accum.F3 / LIGHT_METRICS_WINDOW,
                    s_accum.F4 / LIGHT_METRICS_WINDOW,
                    s_accum.F5 / LIGHT_METRICS_WINDOW,
                    s_accum.F6 / LIGHT_METRICS_WINDOW,
                    s_accum.F7 / LIGHT_METRICS_WINDOW,
                    s_accum.F8 / LIGHT_METRICS_WINDOW,
                    timestamp, mains_hz);

    /* Reset accumulator for next window */
    s_accum = (SpectrumAccum){0};

    return 1U;
}

/* -------------------------------------------------------------------------
 * Instantaneous getters
 * ---------------------------------------------------------------------- */

uint16_t LightMetrics_GetBlueIndex(void)               { return s_blueIndex;         }
uint16_t LightMetrics_GetBlueFracQ15(void)             { return s_blueFrac_q15;      }
uint16_t LightMetrics_GetSunLikeIndexQ15(void)         { return s_sunLikeIdx_q15;    }
uint32_t LightMetrics_GetUvRisk(void)                  { return s_uvRisk_q15;        }
uint32_t LightMetrics_GetBlueWeightedIlluminance(void) { return s_blueCyanFrac_q15;  }

/* -------------------------------------------------------------------------
 * Cumulative getters
 * ---------------------------------------------------------------------- */

uint64_t LightMetrics_GetUvDoseAccum(void)                 { return s_uvDoseAccum;                 }
uint64_t LightMetrics_GetBlueExposureAccum(void)           { return s_blueExposureAccum;           }
uint64_t LightMetrics_GetBlueExposureArtificialAccum(void) { return s_blueExposureArtificialAccum; }
uint64_t LightMetrics_GetBlueExposureNaturalAccum(void)    { return s_blueExposureNaturalAccum;    }
uint64_t LightMetrics_GetCircadianDoseAccum(void)          { return s_circadianDoseAccum;          }
