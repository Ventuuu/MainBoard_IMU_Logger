/*
 * light_metrics_mcu.c
 *
 * On-MCU light exposure metrics derived from AS7341 full-spectrum data.
 * See light_metrics_mcu.h for documentation.
 */

#include "light_metrics_mcu.h"

/* --- Static state -------------------------------------------------------- */

static uint16_t s_blueIndex      = 0U;  /* F3 + F4 (saturated to 16 bits) */
static uint16_t s_blueFrac_q15   = 0U;  /* (F3+F4)/sum Q15 */
static uint16_t s_sunLikeIdx_q15 = 0U;  /* (F7+F8)/sum Q15 — outdoor discriminator */
static uint32_t s_uvRisk         = 0U;  /* UV proxy */
static uint32_t s_blueWeightedIll= 0U;  /* Blue-weighted illuminance */

static uint64_t s_uvDoseAccum                  = 0ULL;
static uint64_t s_blueExposureAccum            = 0ULL;
static uint64_t s_circadianDoseAccum           = 0ULL;
static uint64_t s_blueExposureArtificialAccum  = 0ULL;
static uint64_t s_blueExposureNaturalAccum     = 0ULL;

#define LM_CIRCADIAN_START_H  (20U)
#define LM_CIRCADIAN_END_H    (24U)

/* --- Public: reset ------------------------------------------------------- */

void LightMetrics_Reset(void)
{
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

/* --- Public: update ------------------------------------------------------ */

void LightMetrics_Update(const AS7341_Spectrum *spectrum,
                         const Time_Struct *timestamp,
                         uint16_t mains_hz)
{
    if (spectrum == NULL || timestamp == NULL) {
        return;
    }

    /*
     * Canonical channel mapping in AS7341_Spectrum:
     *   ch[0]  → F1  (415 nm, violet)
     *   ch[1]  → F2  (445 nm, deep blue)
     *   ch[2]  → F3  (480 nm, blue)
     *   ch[3]  → F4  (515 nm, cyan)
     *   ch[4]  → Clear (pass 1)
     *   ch[5]  → NIR   (pass 1)
     *   ch[6]  → F5  (555 nm, green)
     *   ch[7]  → F6  (590 nm, yellow)
     *   ch[8]  → F7  (630 nm, orange-red)
     *   ch[9]  → F8  (680 nm, red)
     *   ch[10] → Clear (pass 2)
     *   ch[11] → NIR   (pass 2)
     */
    uint32_t F1    = spectrum->ch[0];
    uint32_t F2    = spectrum->ch[1];
    uint32_t F3    = spectrum->ch[2];
    uint32_t F4    = spectrum->ch[3];
    uint32_t F5    = spectrum->ch[6];
    uint32_t F6    = spectrum->ch[7];
    uint32_t F7    = spectrum->ch[8];
    uint32_t F8    = spectrum->ch[9];
    uint32_t CLEAR = spectrum->ch[10];  /* Pass-2 Clear for alignment with logging */

    uint32_t sum_all = F1 + F2 + F3 + F4 + F5 + F6 + F7 + F8;
    if (sum_all == 0U) {
        s_blueIndex       = 0U;
        s_blueFrac_q15    = 0U;
        s_sunLikeIdx_q15  = 0U;
        s_uvRisk          = 0U;
        s_blueWeightedIll = 0U;
        return;
    }

    /* BlueIndex = F3 + F4 (saturate to uint16) */
    uint32_t blueIndex32 = F3 + F4;
    s_blueIndex = (blueIndex32 > 0xFFFFUL) ? 0xFFFFU : (uint16_t)blueIndex32;

    /* BlueFrac Q15 = (F3+F4) / sum(F1..F8) */
    s_blueFrac_q15 = (uint16_t)((blueIndex32 * 32767UL) / sum_all);

    /*
     * SunLikeIndex Q15 = (F7+F8) / sum(F1..F8)
     *
     * F7 (630 nm) and F8 (680 nm) are the red channels.
     * Sunlight is spectrally broad with substantial red content.
     * Cool-white phosphor LEDs have very little energy at 630-680 nm.
     * This ratio is the primary outdoor-vs-artificial discriminator.
     */
    uint32_t redIndex32 = F7 + F8;
    s_sunLikeIdx_q15 = (uint16_t)((redIndex32 * 32767UL) / sum_all);

    /* Blue-weighted illuminance = (F3+F4) * CLEAR */
    s_blueWeightedIll = blueIndex32 * CLEAR;

    /* UV_risk ≈ (F1+F2+F3)/sum * CLEAR */
    uint32_t shortSum = F1 + F2 + F3;
    s_uvRisk = (shortSum * CLEAR) / sum_all;

    /* Accumulate doses (rectangle rule at 10 Hz) */
    s_uvDoseAccum       += (uint64_t)s_uvRisk;
    s_blueExposureAccum += (uint64_t)s_blueWeightedIll;

    /* Split by artificial vs natural based on mains flicker */
    uint8_t is_artificial = (mains_hz == 50U) || (mains_hz == 60U);
    if (is_artificial) {
        s_blueExposureArtificialAccum += (uint64_t)s_blueWeightedIll;
    } else {
        s_blueExposureNaturalAccum += (uint64_t)s_blueWeightedIll;
    }

    /* Circadian dose: only inside [20:00, 24:00) AND artificial */
    uint32_t seconds_of_day = ((uint32_t)timestamp->hh * 3600U)
                            + ((uint32_t)timestamp->mm * 60U)
                            + (uint32_t)timestamp->ss;
    uint32_t start_s = LM_CIRCADIAN_START_H * 3600U;
    uint32_t end_s   = LM_CIRCADIAN_END_H   * 3600U;
    if ((seconds_of_day >= start_s) && (seconds_of_day < end_s) && is_artificial) {
        s_circadianDoseAccum += (uint64_t)s_blueWeightedIll;
    }
}

/* --- Instantaneous getters ---------------------------------------------- */

uint16_t LightMetrics_GetBlueIndex(void)              { return s_blueIndex;       }
uint16_t LightMetrics_GetBlueFracQ15(void)            { return s_blueFrac_q15;    }
uint16_t LightMetrics_GetSunLikeIndexQ15(void)        { return s_sunLikeIdx_q15;  }
uint32_t LightMetrics_GetUvRisk(void)                 { return s_uvRisk;          }
uint32_t LightMetrics_GetBlueWeightedIlluminance(void){ return s_blueWeightedIll; }

/* --- Cumulative getters -------------------------------------------------- */

uint64_t LightMetrics_GetUvDoseAccum(void)                 { return s_uvDoseAccum;                 }
uint64_t LightMetrics_GetBlueExposureAccum(void)           { return s_blueExposureAccum;           }
uint64_t LightMetrics_GetBlueExposureArtificialAccum(void) { return s_blueExposureArtificialAccum; }
uint64_t LightMetrics_GetBlueExposureNaturalAccum(void)    { return s_blueExposureNaturalAccum;    }
uint64_t LightMetrics_GetCircadianDoseAccum(void)          { return s_circadianDoseAccum;          }
