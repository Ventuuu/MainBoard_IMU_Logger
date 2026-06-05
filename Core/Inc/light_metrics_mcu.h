/**
 * @file light_metrics_mcu.h
 * @brief On-MCU light exposure metrics computed from AS7341 spectral data.
 *
 * Metrics exposed (per sample / cumulative):
 *   - BlueIndex                = F3 + F4
 *   - BlueFrac (Q15)           = (F3 + F4) / sum(F1..F8)
 *   - SunLikeIndex (Q15)       = (F7 + F8) / sum(F1..F8)
 *                                High → red-rich sun-like; Low → artificial blue
 *   - UV_risk                  ≈ (F1+F2+F3)/sum(F1..F8) * Clear
 *   - BlueWeightedIlluminance  = (F3 + F4) * Clear
 *   - UV_dose_accum
 *   - BlueExposure_accum
 *   - BlueExposureArtificial
 *   - BlueExposureNatural
 *   - CircadianDose_accum
 */

#ifndef INC_LIGHT_METRICS_MCU_H_
#define INC_LIGHT_METRICS_MCU_H_

#include <stdint.h>
#include "as7341_driver.h"
#include "Memory_operations.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Update instantaneous metrics and cumulative doses for one light sample.
 * Call at 10 Hz after AS7341_ReadFullSpectrum().
 *
 * @param spectrum   Latest full spectral frame (F1..F8, Clear, NIR).
 * @param timestamp  Current logical time (hh:mm:ss) for circadian gating.
 * @param mains_hz   Flicker classification: 0, 50, or 60.
 */
void LightMetrics_Update(const AS7341_Spectrum *spectrum,
                         const Time_Struct *timestamp,
                         uint16_t mains_hz);

/* --- Instantaneous getters (latest sample, use for BLE packets) ---------- */

/** BlueIndex = F3 + F4 (saturated to 16 bits). */
uint16_t LightMetrics_GetBlueIndex(void);

/** BlueFrac Q15: (F3+F4)/sum(F1..F8) mapped to 0..32767. */
uint16_t LightMetrics_GetBlueFracQ15(void);

/**
 * SunLikeIndex Q15: (F7+F8)/sum(F1..F8) mapped to 0..32767.
 *
 * This is the red-channel fraction of the spectrum.
 * Sunlight has significant red content → high SunLikeIndex.
 * Cool-white LEDs have much less red → low SunLikeIndex.
 * Use in combination with CLEAR > threshold to decide "outdoors in sun".
 *
 * Typical observed ranges (to refine with real data):
 *   Outdoor sunlight   : ~6000–12000 (Q15)
 *   Indoor cool-white  : ~800–2500   (Q15)
 *   Indoor warm-white  : ~2500–5000  (Q15)
 */
uint16_t LightMetrics_GetSunLikeIndexQ15(void);

/** UV_risk proxy: (F1+F2+F3)/sum * CLEAR (32-bit, arbitrary units). */
uint32_t LightMetrics_GetUvRisk(void);

/** Blue-weighted illuminance: (F3+F4) * CLEAR (32-bit, arbitrary units). */
uint32_t LightMetrics_GetBlueWeightedIlluminance(void);

/* --- Cumulative dose getters --------------------------------------------- */

uint64_t LightMetrics_GetUvDoseAccum(void);
uint64_t LightMetrics_GetBlueExposureAccum(void);
uint64_t LightMetrics_GetBlueExposureArtificialAccum(void);
uint64_t LightMetrics_GetBlueExposureNaturalAccum(void);
uint64_t LightMetrics_GetCircadianDoseAccum(void);

/** Reset all instantaneous values and accumulators (call at session start). */
void LightMetrics_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LIGHT_METRICS_MCU_H_ */
