/**
 * @file imu_metrics.c
 * @brief Pedestrian motion metrics: step count, cadence, activity state.
 *
 * Inputs: IMU_Data structs already converted to physical units.
 * acc  -> g    (gravitational units)
 * gyro -> dps  (degrees per second)
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

/* -------------------------------------------------------------------------
 * Zero-Crossing Activity State Tracking
 * ---------------------------------------------------------------------- */
#define IMU_METRICS_ZC_WINDOW 100U // 1 second window at 100Hz
static uint16_t s_zc_window_ctr = 0U;
static uint16_t s_zc_count = 0U;
static ImuActivityState s_activity = IMU_ACTIVITY_IDLE;


/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

static float bpf_step(float x)
{
    float y  = BPF_B0 * x + s_bpf_w1;
    s_bpf_w1 = BPF_B1 * x - BPF_A1 * y + s_bpf_w2;
    s_bpf_w2 = BPF_B2 * x - BPF_A2 * y;
    return y;
}

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

    s_zc_window_ctr = 0U;
    s_zc_count      = 0U;
    s_activity      = IMU_ACTIVITY_IDLE;
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
     * ------------------------------------------------------------------ */
    float mag_acc = sqrtf(acc->x * acc->x +
                          acc->y * acc->y +
                          acc->z * acc->z) - 1.0f;

    /* Gyroscope magnitude [dps] — reserved for future use */
    (void)(gyro->x);
    (void)(gyro->y);
    (void)(gyro->z);

    /* ------------------------------------------------------------------
     * 2. Band-pass filter [1.5-2.5 Hz]
     * ------------------------------------------------------------------ */
    float filt = bpf_step(mag_acc);

    /* ------------------------------------------------------------------
     * 3. Dynamic threshold: rolling mean of local peak values
     * ------------------------------------------------------------------ */
    float threshold = (s_peak_count > 0U)
                      ? (s_peak_sum / (float)s_peak_count)
                      : 0.0f;

    // Hard Noise Floor: Prevents counting desk vibrations
    if (threshold < 0.02f) {
        threshold = 0.02f;
    }

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
     * 4. Rhythmic Activity State (Zero-Crossing Analysis)
     * ------------------------------------------------------------------ */
    s_zc_window_ctr++;
    
    // Count threshold crossings (Rhythmic Energy)
    if ((s_prev_filt <= threshold) && (filt > threshold) && (threshold > 0.0f)) {
        s_zc_count++;
    }

    // Evaluate every 1 second
    if (s_zc_window_ctr >= IMU_METRICS_ZC_WINDOW) {
        
        if (s_zc_count == 0U) {
            s_activity = IMU_ACTIVITY_IDLE;
        } else if (s_zc_count < 3U) { 
            // 1-2 crossings = walking
            s_activity = IMU_ACTIVITY_WALKING;
        } else {
            // 3+ crossings = running
            s_activity = IMU_ACTIVITY_RUNNING;
        }

        s_zc_window_ctr = 0U;
        s_zc_count = 0U;
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

    // Advance filter history
    s_prev2_filt = s_prev_filt;
    s_prev_filt  = filt;

    /* ------------------------------------------------------------------
     * 6. Cadence Timeout (Zeroing)
     * ------------------------------------------------------------------ */
    if (s_ts_count > 0U) {
        uint8_t newest_idx = (s_ts_head + IMU_METRICS_CADENCE_BUF - 1U) % IMU_METRICS_CADENCE_BUF;
        uint32_t ticks_since_last_step = s_tick - s_step_ts[newest_idx];
        
        if (ticks_since_last_step > 200U) { 
            s_cadence = 0U;
        }
    }
}

/* -------------------------------------------------------------------------
 * Public: getters
 * ---------------------------------------------------------------------- */

uint32_t         ImuMetrics_GetStepCount(void)    { return s_step_count; }
uint16_t         ImuMetrics_GetCadence(void)      { return s_cadence;    }
ImuActivityState ImuMetrics_GetActivityState(void) { return s_activity;   }