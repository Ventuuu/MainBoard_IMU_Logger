/**
 * @file light_metrics_mcu.h
 * @brief On-MCU light exposure metrics computed from AS7341 spectral data.
 *
 * This module provides a lightweight, integer-only implementation of a few
 * key light exposure metrics so they can be streamed over BLE without
 * sending the full spectral frame at high rate.
 *
 * Metrics exposed (per sample / cumulative):
 *   - BlueIndex                = F3 + F4
 *   - BlueFrac (Q15)           = (F3 + F4) / sum(F1..F8)
 *   - UV_risk                  ≈ (F1+F2+F3)/sum(F1..F8) * Clear
 *   - BlueWeightedIlluminance  = (F3 + F4) * Clear
 *   - UV_dose_accum            = sum(UV_risk)
 *   - BlueExposure_accum       = sum(BlueWeightedIlluminance)
 *   - CircadianDose_accum      = sum(BlueWeightedIlluminance) only when
 *                                timestamp is within an evening window
 *                                (default 20:00–24:00).
 *
 * All doses are expressed in arbitrary units per light sample (10 Hz).
 * The application or mobile app is expected to rescale them by the
 * effective sampling interval if absolute units are required.
 */

#ifndef INC_LIGHT_METRICS_MCU_H_
#define INC_LIGHT_METRICS_MCU_H_

#include <stdint.h>

#include "as7341_driver.h"  /* AS7341_Spectrum */
#include "Memory_operations.h"  /* Time_Struct */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Update instantaneous metrics and cumulative doses for one light sample.
 *
 * This should be called at the light sensor rate (10 Hz), after a successful
 * AS7341_ReadFullSpectrum() call and after the timestamp has been updated.
 *
 * @param[in] spectrum   Latest full spectral frame (F1..F8, Clear, NIR).
 * @param[in] timestamp  Current logical time (hh:mm:ss) used for circadian
 *                       window gating; not necessarily wall-clock time.
 */
void LightMetrics_Update(const AS7341_Spectrum *spectrum,
                         const Time_Struct *timestamp,
                        uint16_t mains_hz = 50U);

/* --- Getters for latest instantaneous values (for BLE or debugging) ------ */

/** @return Latest BlueIndex = F3 + F4 (saturated to 16 bits). */
uint16_t LightMetrics_GetBlueIndex(void);

/**
 * @return Latest BlueFrac in Q15 format (0.0–1.0 mapped to 0–32767).
 *         0 if sum(F1..F8) == 0.
 */
uint16_t LightMetrics_GetBlueFracQ15(void);

/** @return Latest UV_risk proxy (32-bit, arbitrary units). */
uint32_t LightMetrics_GetUvRisk(void);

/** @return Latest blue-weighted illuminance proxy (32-bit, arbitrary units). */
uint32_t LightMetrics_GetBlueWeightedIlluminance(void);

/* --- Getters for cumulative doses (monotonically non-decreasing) --------- */

/** @return Cumulative UV risk dose (sum of UV_risk over samples). */
uint64_t LightMetrics_GetUvDoseAccum(void);

/** @return Cumulative blue-weighted exposure over all samples. */
uint64_t LightMetrics_GetBlueExposureAccum(void);

/**
 * @return Cumulative circadian-relevant dose (only samples whose timestamp
 *         is within the configured window contribute).
 */
uint64_t LightMetrics_GetCircadianDoseAccum(void);

/**
 * @brief Reset all instantaneous values and accumulators.
 */
void LightMetrics_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LIGHT_METRICS_MCU_H_ */
