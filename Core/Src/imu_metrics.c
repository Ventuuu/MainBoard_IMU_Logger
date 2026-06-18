/**
 * @file imu_metrics.c
 * @brief Pedestrian motion metrics: step count, cadence, activity state.
 *
 * Inputs: IMU_Data structs already converted to physical units by imu_driver.c.
 *   acc  -> g    (gravitational units, FS +/-2 g by default)
 *   gyro -> dps  (degrees per second, FS +/-250 dps by default)
 * No further sensitivity scaling is applied here.
 *
 * -----------------------------------------------------------------------
 * Band-pass IIR filter
 * -----------------------------------------------------------------------
 * A single 2nd-order Butterworth biquad band-pass [1.5-2.5 Hz / 100 Hz]
 * implemented in Direct Form II Transposed. Coefficients computed with
 * scipy.signal.butter(1, [1.5, 2.5], btype='bandpass', fs=100):
 *
 *   b = [ 0.03046875,  0.0, -0.03046875 ]
 *   a = [ 1.0,        -1.92472215,  0.93906251 ]
 *
 * Frequency response:
 *   1.5 Hz : -3.01 dB  (lower -3 dB cutoff)
 *   2.0 Hz : -0.07 dB  (near-flat passband centre)
 *   2.5 Hz : -3.01 dB  (upper -3 dB cutoff)
 *
 * -----------------------------------------------------------------------
 * Dynamic threshold
 * -----------------------------------------------------------------------
 * Local maxima of the filtered signal are detected (sample n is a maximum
 * when filtered[n-1] < filtered[n] > filtered[n+1]). A rolling buffer of
 * the last IMU_METRICS_PEAK_WINDOW (200) samples' peak values feeds a
 * running mean that forms the adaptive threshold.
 *
 * -----------------------------------------------------------------------
 * Welford online variance
 * -----------------------------------------------------------------------
 * The variance of M(t) over a 100-sample window is computed with Welford's
 * algorithm (numerically stable, single-pass). The window resets every
 * IMU_METRICS_VAR_WINDOW samples for a clean 1 Hz activity state update.
 */

#include "imu_metrics.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Band-pass biquad coefficients  [1.5-2.5 Hz, fs=100 Hz]
 * Direct Form II Transposed: a0 normalised to 1.0
 * ---------------------------------------------------------------------- */

#define BPF_B0   0.0304687471f
#define BPF_B1   0.0000000000f
#define BPF_B2  (-0.0304687471f)
#define BPF_A1  (-1.9247221530f)
#define BPF_A2   0.9390625058f

/* -------------------------------------------------------------------------
 * Activity state variance thresholds (g^2)
 * ---------------------------------------------------------------------- */

#define THRESH_IDLE_WALKING  0.05f
#define THRESH_WALKING_RUN   0.40f

/* -------------------------------------------------------------------------
 * Internal state
 * ---------------------------------------------------------------------- */

/* Band-pass filter delay line (Direct Form II Transposed) */
static float s_bpf_w1 = 0.0f;
static float s_bpf_w2 = 0.0f;

/* Previous two filtered samples for local-maximum detection */
static float s_prev_filt  = 0.0f;
static float s_prev2_filt = 0.0f;

/* Dynamic threshold: rolling buffer of local peak values */
static float    s_peak_buf[IMU_METRICS_PEAK_WINDOW];
static uint16_t s_peak_head  = 0U;
static float    s_peak_sum   = 0.0f;
static uint16_t s_peak_count = 0U;

/* Debounce lockout counter (counts down from DEBOUNCE_SAMPLES to 0) */
static uint16_t s_debounce_ctr = 0U;

/* Step counter */
static uint32_t s_step_count = 0U;

/* Cadence circular buffer of step timestamps (100 Hz tick counter) */
static uint32_t s_step_ts[IMU_METRICS_CADENCE_BUF];
static uint8_t  s_ts_head  = 0U;
static uint8_t  s_ts_count = 0U;
static uint16_t s_cadence  = 0U;

/* Global tick counter (100 Hz) */
static uint32_t s_tick = 0U;

/* Welford online variance for activity state */
static uint32_t s_var_n    = 0U;
static float    s_var_mean = 0.0f;
static float    s_var_M2   = 0.0f;
static ImuActivityState s_activity = IMU_ACTIVITY_IDLE;

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

/**
 * Apply one sample through the biquad band-pass filter.
 * Direct Form II Transposed:
 *   y[n] = b0*x[n] + w1[n-1]
 *   w1   = b1*x[n] - a1*y[n] + w2[n-1]
 *   w2   = b2*x[n] - a2*y[n]
 */
static float bpf_step(float x)
{
    float y  = BPF_B0 * x + s_bpf_w1;
    s_bpf_w1 = BPF_B1 * x - BPF_A1 * y + s_bpf_w2;
    s_bpf_w2 = BPF_B2 * x - BPF_A2 * y;
    return y;
}

/**
 * Update cadence estimate using the circular timestamp buffer.
 * Called immediately after s_step_count is incremented.
 */
static void update_cadence(void)
{
    s_step_ts[s_ts_head] = s_tick;
    s_ts_head = (uint8_t)((s_ts_head + 1U) % IMU_METRICS_CADENCE_BUF);
    if (s_ts_count < IMU_METRICS_CADENCE_BUF) {
        s_ts_count++;
    }

    if (s_ts_count < 2U) {
        s_cadence = 0U;
        return;
    }

    uint8_t  oldest_idx = s_ts_head % IMU_METRICS_CADENCE_BUF;
    uint32_t t_old      = s_step_ts[oldest_idx];
    uint32_t dt_ticks   = s_tick - t_old;

    if (dt_ticks == 0U) {
        return;
    }

    /*
     * cadence (spm) = (N_intervals / dt_seconds) * 60
     *              = (N_intervals * 60 * SAMPLE_RATE) / dt_ticks
     * factor = 60 * 100 = 6000
     */
    uint32_t n_intervals = (uint32_t)(s_ts_count - 1U);
    s_cadence = (uint16_t)((n_intervals * 6000UL) / dt_ticks);
}

/* -------------------------------------------------------------------------
 * Public: reset
 * ---------------------------------------------------------------------- */

void ImuMetrics_Reset(void)
{
    s_bpf_w1      = 0.0f;
    s_bpf_w2      = 0.0f;
    s_prev_filt   = 0.0f;
    s_prev2_filt  = 0.0f;

    memset(s_peak_buf, 0, sizeof(s_peak_buf));
    s_peak_head  = 0U;
    s_peak_sum   = 0.0f;
    s_peak_count = 0U;

    s_debounce_ctr = 0U;
    s_step_count   = 0U;

    memset(s_step_ts, 0, sizeof(s_step_ts));
    s_ts_head  = 0U;
    s_ts_count = 0U;
    s_cadence  = 0U;

    s_tick = 0U;

    s_var_n    = 0U;
    s_var_mean = 0.0f;
    s_var_M2   = 0.0f;
    s_activity = IMU_ACTIVITY_IDLE;
}

/* -------------------------------------------------------------------------
 * Public: update (called from main loop drain path)
 * ---------------------------------------------------------------------- */

void ImuMetrics_Update(const IMU_Data *acc, const IMU_Data *gyro)
{
    if (acc == NULL || gyro == NULL) {
        return;
    }

    s_tick++;

    /* ------------------------------------------------------------------
     * 1. Vector magnitude (gravity removed)
     *    Inputs are already in g (converted by imu_driver.c).
     *    M(t) = sqrt(ax^2 + ay^2 + az^2) - 1.0  [g]
     * ------------------------------------------------------------------ */
    float mag_acc = sqrtf(acc->x * acc->x +
                          acc->y * acc->y +
                          acc->z * acc->z) - 1.0f;

    /* Gyroscope magnitude [dps] — reserved for future gyro-variance feature */
    (void)(gyro->x);
    (void)(gyro->y);
    (void)(gyro->z);

    /* ------------------------------------------------------------------
     * 2. Welford online variance of mag_acc over IMU_METRICS_VAR_WINDOW
     * ------------------------------------------------------------------ */
    s_var_n++;
    float delta  = mag_acc - s_var_mean;
    s_var_mean  += delta / (float)s_var_n;
    float delta2 = mag_acc - s_var_mean;
    s_var_M2    += delta * delta2;

    if (s_var_n >= IMU_METRICS_VAR_WINDOW) {
        float variance = s_var_M2 / (float)(s_var_n - 1U);

        if (variance < THRESH_IDLE_WALKING) {
            s_activity = IMU_ACTIVITY_IDLE;
        } else if (variance < THRESH_WALKING_RUN) {
            s_activity = IMU_ACTIVITY_WALKING;
        } else {
            s_activity = IMU_ACTIVITY_RUNNING;
        }

        s_var_n    = 0U;
        s_var_mean = 0.0f;
        s_var_M2   = 0.0f;
    }

    /* ------------------------------------------------------------------
     * 3. Band-pass filter [1.5-2.5 Hz]
     * ------------------------------------------------------------------ */
    float filt = bpf_step(mag_acc);

    /* ------------------------------------------------------------------
     * 4. Dynamic threshold: rolling mean of local peak values
     * ------------------------------------------------------------------ */
    float threshold = (s_peak_count > 0U)
                      ? (s_peak_sum / (float)s_peak_count)
                      : 0.0f;

    if ((s_prev_filt > s_prev2_filt) &&
        (s_prev_filt > filt) &&
        (s_prev_filt > 0.0f))
    {
        s_peak_sum -= s_peak_buf[s_peak_head];
        s_peak_buf[s_peak_head] = s_prev_filt;
        s_peak_sum += s_prev_filt;
        s_peak_head = (uint16_t)((s_peak_head + 1U) % IMU_METRICS_PEAK_WINDOW);
        if (s_peak_count < IMU_METRICS_PEAK_WINDOW) {
            s_peak_count++;
        }
    }

    /* ------------------------------------------------------------------
     * 5. Step detection: upward threshold crossing + debounce
     * ------------------------------------------------------------------ */
    if (s_debounce_ctr > 0U) {
        s_debounce_ctr--;
    }

    if ((s_debounce_ctr == 0U) &&
        (s_prev_filt <= threshold) &&
        (filt > threshold) &&
        (threshold > 0.0f))
    {
        s_step_count++;
        s_debounce_ctr = IMU_METRICS_DEBOUNCE_SAMPLES;
        update_cadence();
    }

    s_prev2_filt = s_prev_filt;
    s_prev_filt  = filt;

    /* ------------------------------------------------------------------
     * 6. Cadence Timeout (Zeroing)
     * ------------------------------------------------------------------ */
    if (s_ts_count > 0U) {
        // Find the index of the most recently recorded step
        uint8_t newest_idx = (s_ts_head + IMU_METRICS_CADENCE_BUF - 1U) % IMU_METRICS_CADENCE_BUF;
        uint32_t ticks_since_last_step = s_tick - s_step_ts[newest_idx];
        
        // If 200 ticks (2.0 seconds at 100 Hz) have passed without a step, user has stopped.
        if (ticks_since_last_step > 200U) { 
            s_cadence = 0U;
        }
    }
}

/* -------------------------------------------------------------------------
 * Public: getters
 * ---------------------------------------------------------------------- */

uint32_t         ImuMetrics_GetStepCount(void)    { return s_step_count; }
uint16_t         ImuMetrics_GetCadence(void)       { return s_cadence;    }
ImuActivityState ImuMetrics_GetActivityState(void) { return s_activity;   }
