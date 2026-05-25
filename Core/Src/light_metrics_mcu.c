/*
 * light_metrics_mcu.c
 *
 * On-MCU implementation of simple light exposure metrics derived from
 * AS7341 spectral data.
 *
 * See light_metrics_mcu.h for high-level documentation.
 */

#include "light_metrics_mcu.h"

/* --- Internal storage for latest values and accumulators ------------------ */

static uint16_t s_blueIndex = 0U;          /* F3 + F4 (saturated to 16 bits) */
static uint16_t s_blueFrac_q15 = 0U;       /* Q15 representation 0..1 → 0..32767 */
static uint32_t s_uvRisk = 0U;             /* Proxy, arbitrary units */
static uint32_t s_blueWeightedIll = 0U;    /* Proxy, arbitrary units */

static uint64_t s_uvDoseAccum = 0ULL;
static uint64_t s_blueExposureAccum = 0ULL;
static uint64_t s_circadianDoseAccum = 0ULL;

/* New: split blue exposure by light type inferred from flicker classification */
static uint64_t s_blueExposureArtificialAccum = 0ULL;
static uint64_t s_blueExposureNaturalAccum    = 0ULL;

/* Default circadian-sensitive window: 20:00–24:00 (local logical time). */
#define LM_CIRCADIAN_START_H  (20U)
#define LM_CIRCADIAN_END_H    (24U)

void LightMetrics_Reset(void)
{
    s_blueIndex = 0U;
    s_blueFrac_q15 = 0U;
    s_uvRisk = 0U;
    s_blueWeightedIll = 0U;
    s_uvDoseAccum = 0ULL;
    s_blueExposureAccum = 0ULL;
    s_circadianDoseAccum = 0ULL;
    s_blueExposureArtificialAccum = 0ULL;
    s_blueExposureNaturalAccum = 0ULL;
}

void LightMetrics_Update(const AS7341_Spectrum *spectrum,
                         const Time_Struct *timestamp,
                         uint16_t mains_hz)
{
    if (spectrum == NULL || timestamp == NULL) {
        return;
    }

    /*
     * Channel mapping assumption (matching as7341_driver.c):
     *   ch[0..7]  → F1..F8
     *   ch[8]     → Clear
     *   ch[9]     → NIR
     */
    uint32_t F1    = spectrum->ch[0];
    uint32_t F2    = spectrum->ch[1];
    uint32_t F3    = spectrum->ch[2];
    uint32_t F4    = spectrum->ch[3];
    uint32_t F5    = spectrum->ch[4];
    uint32_t F6    = spectrum->ch[5];
    uint32_t F7    = spectrum->ch[6];
    uint32_t F8    = spectrum->ch[7];
    uint32_t CLEAR = spectrum->ch[8];

    uint32_t sum_all = F1 + F2 + F3 + F4 + F5 + F6 + F7 + F8;
    if (sum_all == 0U) {
        /* Dark / no light: keep latest accumulators, zero instantaneous values. */
        s_blueIndex = 0U;
        s_blueFrac_q15 = 0U;
        s_uvRisk = 0U;
        s_blueWeightedIll = 0U;
        return;
    }

    /* --- BlueIndex = F3 + F4 --------------------------------------------- */
    uint32_t blueIndex32 = F3 + F4;
    if (blueIndex32 > 0xFFFFUL) {
        s_blueIndex = 0xFFFFU;
    } else {
        s_blueIndex = (uint16_t)blueIndex32;
    }

    /* --- BlueFrac (Q15) = (F3+F4) / sum(F1..F8) --------------------------- */
    s_blueFrac_q15 = (uint16_t)((blueIndex32 * 32767UL) / sum_all);

    /* --- Blue-weighted illuminance = (F3+F4) * CLEAR ---------------------- */
    s_blueWeightedIll = blueIndex32 * CLEAR;  /* 32-bit, application responsible for scaling */

    /* --- UV_risk ≈ (F1+F2+F3)/sum(F1..F8) * CLEAR ------------------------- */
    uint32_t shortSum = F1 + F2 + F3;
    s_uvRisk = (shortSum * CLEAR) / sum_all;

    /* --- Accumulate doses (rectangle rule, 1 unit per light sample @ 10 Hz) */
    s_uvDoseAccum       += (uint64_t)s_uvRisk;
    s_blueExposureAccum += (uint64_t)s_blueWeightedIll;

    /* --- Classify light type from mains flicker --------------------------- */
    uint8_t is_artificial = (mains_hz == 50U) || (mains_hz == 60U);

    if (is_artificial) {
        s_blueExposureArtificialAccum += (uint64_t)s_blueWeightedIll;
    } else {
        /* Treat non-flickering samples as likely natural/daylight or DC LED. */
        s_blueExposureNaturalAccum += (uint64_t)s_blueWeightedIll;
    }

    /* --- Circadian dose: only inside [20:00, 24:00) ----------------------- */
    uint32_t seconds_of_day = ((uint32_t)timestamp->hh * 3600U)
                            + ((uint32_t)timestamp->mm * 60U)
                            + (uint32_t)timestamp->ss;
    uint32_t start_s = LM_CIRCADIAN_START_H * 3600U;
    uint32_t end_s   = LM_CIRCADIAN_END_H * 3600U;

    if (seconds_of_day >= start_s && seconds_of_day < end_s) {
        /* For circadian disruption we care most about artificial sources. */
        if (is_artificial) {
            s_circadianDoseAccum += (uint64_t)s_blueWeightedIll;
        }
    }
}

/* --- Getters ------------------------------------------------------------- */

uint16_t LightMetrics_GetBlueIndex(void)
{
    return s_blueIndex;
}

uint16_t LightMetrics_GetBlueFracQ15(void)
{
    return s_blueFrac_q15;
}

uint32_t LightMetrics_GetUvRisk(void)
{
    return s_uvRisk;
}

uint32_t LightMetrics_GetBlueWeightedIlluminance(void)
{
    return s_blueWeightedIll;
}

uint64_t LightMetrics_GetUvDoseAccum(void)
{
    return s_uvDoseAccum;
}

uint64_t LightMetrics_GetBlueExposureAccum(void)
{
    return s_blueExposureAccum;
}

uint64_t LightMetrics_GetBlueExposureArtificialAccum(void)
{
    return s_blueExposureArtificialAccum;
}

uint64_t LightMetrics_GetBlueExposureNaturalAccum(void)
{
    return s_blueExposureNaturalAccum;
}

uint64_t LightMetrics_GetCircadianDoseAccum(void)
{
    return s_circadianDoseAccum;
}
