/**
 * @file imu_metrics.h
 * @brief Pedestrian motion metrics derived from the LSM6DSO16IS at 100 Hz.
 *
 * Three metrics are computed from the IMU data stream. The IMU driver
 * (imu_driver.c) already converts raw register values to physical units
 * before populating IMU_Data, so this module receives:
 *   acc  fields in g    (gravitational units)
 *   gyro fields in dps  (degrees per second)
 *
 *  1. Step Count
 *     Vector magnitude M(t) = sqrt(ax^2 + ay^2 + az^2) - 1.0  [g, DC-removed]
 *     Band-pass filtered at [1.5-2.5 Hz] with a 2nd-order Butterworth IIR
 *     biquad.  A dynamic threshold (rolling mean of local maxima over 2 s)
 *     plus a 300 ms debounce detects each heel-strike.
 *
 *  2. Cadence  (steps per minute)
 *     Timestamps of the last IMU_METRICS_CADENCE_BUF steps are stored in a
 *     circular buffer.  Cadence = (N / (T_newest - T_oldest)) * 60.
 *     Returns 0 until at least 2 steps are recorded.
 *
 *  3. Activity State  (IDLE / WALKING / RUNNING)
 *     The variance sigma^2 of M(t) over a 1-second (100-sample) window is
 *     computed with the Welford online algorithm.
 *     Thresholds (in g^2):
 *       IDLE    : sigma^2 < 0.05
 *       WALKING : 0.05 <= sigma^2 < 0.40
 *       RUNNING : sigma^2 >= 0.40
 */

#ifndef INC_IMU_METRICS_H_
#define INC_IMU_METRICS_H_

#include <stdint.h>
#include "imu_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Configuration constants -------------------------------------------- */

/** IMU sample rate (Hz). Must match TIM2 configuration in main.c. */
#define IMU_METRICS_SAMPLE_RATE_HZ    100U

/** Debounce lockout after a step detection (samples at 100 Hz = 300 ms). */
#define IMU_METRICS_DEBOUNCE_SAMPLES   30U

/** Dynamic threshold window: local-maxima rolling average over 2 seconds. */
#define IMU_METRICS_PEAK_WINDOW       200U

/** Cadence circular buffer depth (last N step timestamps). */
#define IMU_METRICS_CADENCE_BUF         8U

/** Variance window for activity state (1 second at 100 Hz). */
#define IMU_METRICS_VAR_WINDOW        100U

/* --- Activity state enum ------------------------------------------------- */

typedef enum {
    IMU_ACTIVITY_IDLE    = 0,
    IMU_ACTIVITY_WALKING = 1,
    IMU_ACTIVITY_RUNNING = 2,
} ImuActivityState;

/* --- Public API ---------------------------------------------------------- */

/**
 * @brief Process one IMU sample (call from TIM2 ISR at 100 Hz).
 *
 * @param acc   Pointer to accelerometer data in g  (already converted by driver).
 * @param gyro  Pointer to gyroscope data in dps    (already converted by driver).
 */
void ImuMetrics_Update(const IMU_Data *acc, const IMU_Data *gyro);

/** @return Cumulative step count since last reset. */
uint32_t ImuMetrics_GetStepCount(void);

/**
 * @return Cadence in steps per minute (0 if fewer than 2 steps recorded).
 *         Updated on every new step detection.
 */
uint16_t ImuMetrics_GetCadence(void);

/** @return Latest activity state derived from the last 1-second window. */
ImuActivityState ImuMetrics_GetActivityState(void);

/** Reset all state (call at session start or on demand). */
void ImuMetrics_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_IMU_METRICS_H_ */
