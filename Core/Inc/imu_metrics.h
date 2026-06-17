/**
 * @file imu_metrics.h
 * @brief Pedestrian motion metrics derived from the LSM6DSO16IS at 100 Hz.
 *
 * Three metrics are computed from the raw accelerometer (and, for activity
 * state, gyroscope) data stream:
 *
 *  1. Step Count
 *     Raw LSB values are converted to g, the vector magnitude is computed
 *     (gravity bias removed), band-pass filtered at [1.5-2.5 Hz] with a
 *     2nd-order Butterworth IIR biquad, then a dynamic threshold + 300 ms
 *     debounce detects each heel-strike.
 *
 *  2. Cadence  (steps per minute)
 *     Timestamps of the last IMU_METRICS_CADENCE_BUF steps are stored in a
 *     circular buffer.  Cadence = (N / (T_newest - T_oldest)) * 60.
 *     Returns 0 until at least 2 steps are recorded.
 *
 *  3. Activity State  (IDLE / WALKING / RUNNING)
 *     The variance sigma^2 of the acceleration magnitude M(t) over a 1-second
 *     (100-sample) window is computed with the Welford online algorithm.
 *     Thresholds (in g^2):
 *       IDLE    : sigma^2 < 0.05
 *       WALKING : 0.05 <= sigma^2 < 0.40
 *       RUNNING : sigma^2 >= 0.40
 *
 * Sensitivity constants (must match IMU configuration in main.c):
 *   kAccelSens = 2.0f / 32767.0f   (g per LSB, FS = +/-2 g)
 *   kGyroSens  = 1.0f / 175.0f     (dps per LSB, FS = +/-250 dps)
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
#define IMU_METRICS_SAMPLE_RATE_HZ   100U

/** Debounce lockout after a step detection (samples at 100 Hz = 300 ms). */
#define IMU_METRICS_DEBOUNCE_SAMPLES  30U

/** Dynamic threshold window: local-maxima rolling average over 2 seconds. */
#define IMU_METRICS_PEAK_WINDOW       200U

/** Cadence circular buffer depth (last N step timestamps). */
#define IMU_METRICS_CADENCE_BUF       8U

/** Variance window for activity state (1 second at 100 Hz). */
#define IMU_METRICS_VAR_WINDOW        100U

/** Accel sensitivity: g per LSB (FS = +/-2 g, 16-bit signed). */
#define IMU_METRICS_ACCEL_SENS        (2.0f / 32767.0f)

/** Gyro sensitivity: dps per LSB (FS = +/-250 dps). */
#define IMU_METRICS_GYRO_SENS         (1.0f / 175.0f)

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
 * Internally advances the band-pass filter, dynamic threshold, debounce
 * timer, cadence buffer, and Welford variance accumulator.
 *
 * @param acc   Pointer to latest accelerometer data (raw LSB floats).
 * @param gyro  Pointer to latest gyroscope data (raw LSB floats).
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
