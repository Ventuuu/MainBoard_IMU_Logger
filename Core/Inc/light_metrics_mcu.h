/**
 * @file light_metrics_mcu.h
 * @brief On-MCU light exposure metrics computed from AS7341 spectral data.
 *
 * Acquisition model
 * -----------------
 * The AS7341 is read at 10 Hz from the TIM2 ISR.  Raw channel counts are
 * accumulated over 10 consecutive readings (one second) and the metrics are
 * computed once from the averaged spectrum at the end of each window.  This
 * suppresses sample-to-sample noise while keeping the MCU computation budget
 * low (one division burst per second instead of per sample).
 *
 * Metrics (instantaneous = last completed 1-second window)
 * ---------------------------------------------------------
 *   BlueIndex          = F3  (480 nm only, saturated to 16 bits)
 *   BlueFrac  (Q15)    = F3 / sum(F1..F8)              [0..32767]
 *   SunLikeIndex (Q15) = (F7+F8) / sum(F1..F8)         [0..32767]
 *   UV_risk  (Q15)     = (F1+F2+F3) / sum(F1..F8)      [0..32767]
 *   BlueWeightedIll (Q15) = (F3+F4) / sum(F1..F8)      [0..32767]
 *
 * All Q15 values: 32767 = 100 %, 0 = 0 %.
 * Intermediate multiplications use uint64_t to prevent overflow.
 *
 * Cumulative accumulators (rectangle rule, 1 Hz update)
 * ------------------------------------------------------
 *   UV_dose_accum
 *   BlueExposure_accum
 *   BlueExposureArtificial_accum
 *   BlueExposureNatural_accum
 *   CircadianDose_accum          (artificial light, 20:00-24:00 only)
 */

#ifndef INC_LIGHT_METRICS_MCU_H_
#define INC_LIGHT_METRICS_MCU_H_

#include <stdint.h>
#include "as7341_driver.h"
#include "Memory_operations.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Number of 10 Hz light samples that form one averaging window (= 1 second). */
#define LIGHT_METRICS_WINDOW  10U

/**
 * @brief Accumulate one raw spectrum sample into the internal window buffer.
 *
 * Call this at 10 Hz immediately after every successful AS7341_ReadFullSpectrum().
 * When LIGHT_METRICS_WINDOW samples have been collected the function computes
 * all metrics from the averaged spectrum, resets the accumulator, and returns
 * 1 (metrics refreshed).  Returns 0 on all other calls.
 *
 * @param spectrum   Latest full spectral frame (F1..F8, Clear, NIR).
 * @param timestamp  Current logical time (hh:mm:ss) for circadian gating.
 *                   Sampled at the moment the window closes.
 * @param mains_hz   Flicker classification: 0, 50, or 60.
 * @return           1 if metrics were recomputed this call, 0 otherwise.
 */
uint8_t LightMetrics_Update(const AS7341_Spectrum *spectrum,
                            const Time_Struct     *timestamp,
                            uint16_t               mains_hz);

/* --- Instantaneous getters (last completed window, safe for BLE) --------- */

/** BlueIndex = F3 (480 nm), averaged over window, saturated to 16 bits. */
uint16_t LightMetrics_GetBlueIndex(void);

/** BlueFrac Q15: F3 / sum(F1..F8). */
uint16_t LightMetrics_GetBlueFracQ15(void);

/**
 * SunLikeIndex Q15: (F7+F8) / sum(F1..F8).
 *
 * High value -> red-rich, sun-like spectrum.
 * Low value  -> artificial (cool-white LED/fluorescent).
 *
 * Indicative ranges (calibrate with field data):
 *   Outdoor sunlight  : ~6000-12000
 *   Indoor cool-white : ~800-2500
 *   Indoor warm-white : ~2500-5000
 */
uint16_t LightMetrics_GetSunLikeIndexQ15(void);

/**
 * UV_risk Q15: (F1+F2+F3) / sum(F1..F8).
 * Fraction of total spectral power in the violet/blue-UV bands.
 * 32767 = all energy in F1-F3; 0 = none.
 */
uint32_t LightMetrics_GetUvRisk(void);

/**
 * BlueWeightedIlluminance Q15: (F3+F4) / sum(F1..F8).
 * Fraction of total spectral power in the blue+cyan bands (480+515 nm).
 * Complements BlueFrac (F3 only) by including the cyan channel.
 * 32767 = all energy in F3+F4; 0 = none.
 */
uint32_t LightMetrics_GetBlueWeightedIlluminance(void);

/* --- Cumulative dose getters (rectangle rule, 1 Hz) ---------------------- */

uint64_t LightMetrics_GetUvDoseAccum(void);
uint64_t LightMetrics_GetBlueExposureAccum(void);
uint64_t LightMetrics_GetBlueExposureArtificialAccum(void);
uint64_t LightMetrics_GetBlueExposureNaturalAccum(void);
uint64_t LightMetrics_GetCircadianDoseAccum(void);

/** Reset all state - call once at session start. */
void LightMetrics_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_LIGHT_METRICS_MCU_H_ */
