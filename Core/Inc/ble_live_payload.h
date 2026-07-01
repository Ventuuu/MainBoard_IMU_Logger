#ifndef INC_BLE_LIVE_PAYLOAD_H_
#define INC_BLE_LIVE_PAYLOAD_H_

/**
 * @file  ble_live_payload.h
 * @brief BLE live-streaming payload definitions for the A9 firmware.
 *
 * Two message types are defined:
 *
 *  BLE_MSG_LIVE_METRICS (0x50)
 *  ----------------------------
 *  Compact per-epoch derived-metric packet.  Sent once per 7-second
 *  sensor epoch.  Every byte carries meaningful data; no zero-padding.
 *
 *  Wire layout (11 bytes total):
 *  ┌──────────┬────────────────────────────────────────────────────┐
 *  │ Byte(s)  │ Field                                              │
 *  ├──────────┼────────────────────────────────────────────────────┤
 *  │ 0..3     │ epoch_ms        — epoch start tick (uint32 LE)     │
 *  │ 4..5     │ step_count      — cumulative steps (uint16 LE)     │
 *  │ 6        │ activity_state  — ImuActivityState enum (uint8)    │
 *  │ 7        │ exposure_class  — LightExposureClass enum (uint8)  │
 *  │ 8        │ light_color_intensity — reserved, 0xFF until impl  │
 *  │ 9        │ environment_class — AudioEnvironmentClass (uint8)  │
 *  │ 10..11   │ laeq_centi_dba  — LAeq × 100 (int16 LE)           │
 *  │ 12       │ payload_flags   — BLE_LIVE_FLAG_* bitmask (uint8)  │
 *  └──────────┴────────────────────────────────────────────────────┘
 *  Total: 13 bytes
 *
 *  BLE_MSG_RAW_DATA (0x51)
 *  -------------------------
 *  One raw sensor frame.  The sensor_id byte selects which sensor
 *  block is populated; unused blocks are omitted entirely — the
 *  payload length declared in the BLE frame header tells the receiver
 *  exactly how many bytes to expect.
 *
 *  Wire layout when sensor_id == BLE_RAW_SENSOR_IMU (32 bytes):
 *  ┌──────────┬────────────────────────────────────────────────────┐
 *  │ Byte(s)  │ Field                                              │
 *  ├──────────┼────────────────────────────────────────────────────┤
 *  │ 0..3     │ sample_ms   — sample timestamp ms (uint32 LE)      │
 *  │ 4        │ sensor_id   — BLE_RAW_SENSOR_IMU (0x01)           │
 *  │ 5..10    │ accel_raw   — XYZ 3×int16 LE (±2 g, 0.061 mg/LSB) │
 *  │ 11..16   │ gyro_raw    — XYZ 3×int16 LE (±250 dps,8.75mdps)  │
 *  └──────────┴────────────────────────────────────────────────────┘
 *  Total: 17 bytes
 *
 *  Wire layout when sensor_id == BLE_RAW_SENSOR_LIGHT (25 bytes):
 *  ┌──────────┬────────────────────────────────────────────────────┐
 *  │ Byte(s)  │ Field                                              │
 *  ├──────────┼────────────────────────────────────────────────────┤
 *  │ 0..3     │ sample_ms   — sample timestamp ms (uint32 LE)      │
 *  │ 4        │ sensor_id   — BLE_RAW_SENSOR_LIGHT (0x02)         │
 *  │ 5..6     │ f1_counts   — 415 nm (uint16 LE)                   │
 *  │ 7..8     │ f2_counts   — 445 nm (uint16 LE)                   │
 *  │ 9..10    │ f3_counts   — 480 nm (uint16 LE)                   │
 *  │ 11..12   │ f4_counts   — 515 nm (uint16 LE)                   │
 *  │ 13..14   │ f5_counts   — 555 nm (uint16 LE)                   │
 *  │ 15..16   │ f6_counts   — 590 nm (uint16 LE)                   │
 *  │ 17..18   │ f7_counts   — 630 nm (uint16 LE)                   │
 *  │ 19..20   │ f8_counts   — 680 nm (uint16 LE)                   │
 *  │ 21..22   │ clear_counts— broadband (uint16 LE)                │
 *  │ 23..24   │ nir_counts  — near-infrared (uint16 LE)            │
 *  └──────────┴────────────────────────────────────────────────────┘
 *  Total: 25 bytes
 *
 *  NOTE: Mic raw PCM is intentionally excluded from BLE_MSG_RAW_DATA.
 *        Only the derived LAeq metric (in BLE_MSG_LIVE_METRICS) is
 *        streamed for the microphone channel.
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Message type codes (extend ble_sync_protocol.h range 0x50..0x5F)   */
/* ------------------------------------------------------------------ */

/** Compact per-epoch derived metrics from all three sensors. */
#define BLE_MSG_LIVE_METRICS    0x50U

/** Raw sensor frame; which sensor is indicated by BleLiveRawPayload.sensor_id. */
#define BLE_MSG_RAW_DATA        0x51U

/* ------------------------------------------------------------------ */
/* Sensor-ID values used in BleLiveRawPayload.sensor_id               */
/* ------------------------------------------------------------------ */

#define BLE_RAW_SENSOR_IMU      0x01U   /**< Raw accel + gyro frame.         */
#define BLE_RAW_SENSOR_LIGHT    0x02U   /**< Raw AS7341 spectral frame.       */
/* 0x03 (mic) intentionally omitted — only LAeq is streamed for audio. */

/* ------------------------------------------------------------------ */
/* payload_flags bitmask for BleLiveMetricsPayload                    */
/* ------------------------------------------------------------------ */

/** IMU metrics in this packet are valid. */
#define BLE_LIVE_FLAG_IMU_VALID         (1U << 0)
/** Light metrics in this packet are valid. */
#define BLE_LIVE_FLAG_LIGHT_VALID       (1U << 1)
/** Mic metrics in this packet are valid. */
#define BLE_LIVE_FLAG_MIC_VALID         (1U << 2)
/** Light sensor saturated during this epoch. */
#define BLE_LIVE_FLAG_LIGHT_SATURATED   (1U << 3)
/** Mic clipped during this epoch. */
#define BLE_LIVE_FLAG_MIC_CLIPPED       (1U << 4)

/* ------------------------------------------------------------------ */
/* IMU activity-state enumeration                                      */
/* ------------------------------------------------------------------ */

typedef enum
{
    IMU_ACTIVITY_IDLE    = 0U,  /**< Device is stationary.               */
    IMU_ACTIVITY_WALKING = 1U,  /**< Walking detected.                    */
    IMU_ACTIVITY_RUNNING = 2U,  /**< Running detected.                    */
    IMU_ACTIVITY_UNKNOWN = 0xFFU /**< Classification not yet available.   */
} ImuActivityState;

/* ------------------------------------------------------------------ */
/* BLE_MSG_LIVE_METRICS payload  (13 bytes, no padding)               */
/* ------------------------------------------------------------------ */

typedef struct __attribute__((packed))
{
    uint32_t epoch_ms;              /**< Epoch start timestamp (ms since boot). */
    uint16_t step_count;            /**< Cumulative step count (wraps at 65535). */
    uint8_t  activity_state;        /**< ImuActivityState; 0xFF = unknown.       */
    uint8_t  exposure_class;        /**< LightExposureClass (0=Dark … 4=VeryHigh).*/
    uint8_t  light_color_intensity; /**< Reserved — set to 0xFF until implemented.*/
    uint8_t  environment_class;     /**< AudioEnvironmentClass (0=VeryQuiet…6=High).*/
    int16_t  laeq_centi_dba;        /**< LAeq × 100 in dBA. INT16_MIN = invalid.  */
    uint8_t  payload_flags;         /**< BLE_LIVE_FLAG_* validity / status bits.  */
} BleLiveMetricsPayload;

_Static_assert(sizeof(BleLiveMetricsPayload) == 13U,
               "BleLiveMetricsPayload must be exactly 13 bytes");
_Static_assert(offsetof(BleLiveMetricsPayload, epoch_ms)              == 0U,
               "BleLiveMetricsPayload.epoch_ms offset must be 0");
_Static_assert(offsetof(BleLiveMetricsPayload, step_count)            == 4U,
               "BleLiveMetricsPayload.step_count offset must be 4");
_Static_assert(offsetof(BleLiveMetricsPayload, activity_state)        == 6U,
               "BleLiveMetricsPayload.activity_state offset must be 6");
_Static_assert(offsetof(BleLiveMetricsPayload, exposure_class)        == 7U,
               "BleLiveMetricsPayload.exposure_class offset must be 7");
_Static_assert(offsetof(BleLiveMetricsPayload, light_color_intensity) == 8U,
               "BleLiveMetricsPayload.light_color_intensity offset must be 8");
_Static_assert(offsetof(BleLiveMetricsPayload, environment_class)     == 9U,
               "BleLiveMetricsPayload.environment_class offset must be 9");
_Static_assert(offsetof(BleLiveMetricsPayload, laeq_centi_dba)        == 10U,
               "BleLiveMetricsPayload.laeq_centi_dba offset must be 10");
_Static_assert(offsetof(BleLiveMetricsPayload, payload_flags)         == 12U,
               "BleLiveMetricsPayload.payload_flags offset must be 12");

/* ------------------------------------------------------------------ */
/* BLE_MSG_RAW_DATA payload variants                                   */
/* ------------------------------------------------------------------ */

/**
 * Raw IMU frame — sensor_id == BLE_RAW_SENSOR_IMU.
 * 17 bytes, no padding.
 */
typedef struct __attribute__((packed))
{
    uint32_t sample_ms;     /**< Sample timestamp in ms since boot.     */
    uint8_t  sensor_id;     /**< Must be BLE_RAW_SENSOR_IMU (0x01).    */
    int16_t  accel_x;       /**< Accelerometer X, 0.061 mg/LSB @ ±2 g. */
    int16_t  accel_y;       /**< Accelerometer Y.                        */
    int16_t  accel_z;       /**< Accelerometer Z.                        */
    int16_t  gyro_x;        /**< Gyroscope X, 8.75 mdps/LSB @ ±250 dps.*/
    int16_t  gyro_y;        /**< Gyroscope Y.                            */
    int16_t  gyro_z;        /**< Gyroscope Z.                            */
} BleLiveRawImuPayload;

_Static_assert(sizeof(BleLiveRawImuPayload) == 17U,
               "BleLiveRawImuPayload must be exactly 17 bytes");
_Static_assert(offsetof(BleLiveRawImuPayload, sample_ms) == 0U,
               "BleLiveRawImuPayload.sample_ms offset must be 0");
_Static_assert(offsetof(BleLiveRawImuPayload, sensor_id) == 4U,
               "BleLiveRawImuPayload.sensor_id offset must be 4");
_Static_assert(offsetof(BleLiveRawImuPayload, accel_x)   == 5U,
               "BleLiveRawImuPayload.accel_x offset must be 5");
_Static_assert(offsetof(BleLiveRawImuPayload, gyro_x)    == 11U,
               "BleLiveRawImuPayload.gyro_x offset must be 11");

/**
 * Raw AS7341 spectral frame — sensor_id == BLE_RAW_SENSOR_LIGHT.
 * 25 bytes, no padding.
 */
typedef struct __attribute__((packed))
{
    uint32_t sample_ms;     /**< Sample timestamp in ms since boot.  */
    uint8_t  sensor_id;     /**< Must be BLE_RAW_SENSOR_LIGHT (0x02).*/
    uint16_t f1_counts;     /**< 415 nm — violet channel.            */
    uint16_t f2_counts;     /**< 445 nm — indigo channel.            */
    uint16_t f3_counts;     /**< 480 nm — blue channel.              */
    uint16_t f4_counts;     /**< 515 nm — cyan channel.              */
    uint16_t f5_counts;     /**< 555 nm — green channel.             */
    uint16_t f6_counts;     /**< 590 nm — yellow channel.            */
    uint16_t f7_counts;     /**< 630 nm — orange channel.            */
    uint16_t f8_counts;     /**< 680 nm — red channel.               */
    uint16_t clear_counts;  /**< Broadband clear channel.            */
    uint16_t nir_counts;    /**< Near-infrared channel.              */
} BleLiveRawLightPayload;

_Static_assert(sizeof(BleLiveRawLightPayload) == 25U,
               "BleLiveRawLightPayload must be exactly 25 bytes");
_Static_assert(offsetof(BleLiveRawLightPayload, sample_ms)    == 0U,
               "BleLiveRawLightPayload.sample_ms offset must be 0");
_Static_assert(offsetof(BleLiveRawLightPayload, sensor_id)    == 4U,
               "BleLiveRawLightPayload.sensor_id offset must be 4");
_Static_assert(offsetof(BleLiveRawLightPayload, f1_counts)    == 5U,
               "BleLiveRawLightPayload.f1_counts offset must be 5");
_Static_assert(offsetof(BleLiveRawLightPayload, clear_counts) == 21U,
               "BleLiveRawLightPayload.clear_counts offset must be 21");
_Static_assert(offsetof(BleLiveRawLightPayload, nir_counts)   == 23U,
               "BleLiveRawLightPayload.nir_counts offset must be 23");

/* ------------------------------------------------------------------ */
/* Builder helpers — inline, zero-dependency                           */
/* ------------------------------------------------------------------ */

/**
 * Populate a BleLiveMetricsPayload from the three sensor result structs.
 *
 * @param out           Destination payload struct.
 * @param epoch_ms      Current epoch start tick in milliseconds.
 * @param step_count    Cumulative step count from IMU processing.
 * @param activity      ImuActivityState classification result.
 * @param exposure      LightExposureClass from AS7341 pipeline.
 * @param laeq_cdba     Estimated LAeq × 100 dBA (INT16_MIN if invalid).
 * @param env_class     AudioEnvironmentClass from mic pipeline.
 * @param flags         BLE_LIVE_FLAG_* bitmask; caller sets validity bits.
 */
static inline void BleLiveMetrics_Build(
        BleLiveMetricsPayload *out,
        uint32_t epoch_ms,
        uint16_t step_count,
        uint8_t  activity,
        uint8_t  exposure,
        int16_t  laeq_cdba,
        uint8_t  env_class,
        uint8_t  flags)
{
    out->epoch_ms              = epoch_ms;
    out->step_count            = step_count;
    out->activity_state        = activity;
    out->exposure_class        = exposure;
    out->light_color_intensity = 0xFFU; /* reserved — not yet implemented */
    out->environment_class     = env_class;
    out->laeq_centi_dba        = laeq_cdba;
    out->payload_flags         = flags;
}

/**
 * Populate a BleLiveRawImuPayload from a six-byte raw accelerometer
 * buffer and a six-byte raw gyroscope buffer (LSB-first layout, same
 * as the on-NAND SensorRecord format).
 *
 * @param out        Destination payload struct.
 * @param sample_ms  Timestamp of this raw sample.
 * @param accel_raw  Pointer to 6 bytes: [xl xh yl yh zl zh] accel.
 * @param gyro_raw   Pointer to 6 bytes: [xl xh yl yh zl zh] gyro.
 */
static inline void BleLiveRawImu_Build(
        BleLiveRawImuPayload *out,
        uint32_t sample_ms,
        const uint8_t *accel_raw,
        const uint8_t *gyro_raw)
{
    out->sample_ms = sample_ms;
    out->sensor_id = BLE_RAW_SENSOR_IMU;
    out->accel_x = (int16_t)((uint16_t)accel_raw[0] | ((uint16_t)accel_raw[1] << 8U));
    out->accel_y = (int16_t)((uint16_t)accel_raw[2] | ((uint16_t)accel_raw[3] << 8U));
    out->accel_z = (int16_t)((uint16_t)accel_raw[4] | ((uint16_t)accel_raw[5] << 8U));
    out->gyro_x  = (int16_t)((uint16_t)gyro_raw[0]  | ((uint16_t)gyro_raw[1]  << 8U));
    out->gyro_y  = (int16_t)((uint16_t)gyro_raw[2]  | ((uint16_t)gyro_raw[3]  << 8U));
    out->gyro_z  = (int16_t)((uint16_t)gyro_raw[4]  | ((uint16_t)gyro_raw[5]  << 8U));
}

/**
 * Populate a BleLiveRawLightPayload from an AS7341_Spectrum struct.
 *
 * @param out        Destination payload struct.
 * @param sample_ms  Timestamp of this measurement.
 * @param s          Pointer to the AS7341_Spectrum holding raw counts.
 */
static inline void BleLiveRawLight_Build(
        BleLiveRawLightPayload *out,
        uint32_t sample_ms,
        uint16_t f1, uint16_t f2, uint16_t f3, uint16_t f4,
        uint16_t f5, uint16_t f6, uint16_t f7, uint16_t f8,
        uint16_t clear, uint16_t nir)
{
    out->sample_ms    = sample_ms;
    out->sensor_id    = BLE_RAW_SENSOR_LIGHT;
    out->f1_counts    = f1;
    out->f2_counts    = f2;
    out->f3_counts    = f3;
    out->f4_counts    = f4;
    out->f5_counts    = f5;
    out->f6_counts    = f6;
    out->f7_counts    = f7;
    out->f8_counts    = f8;
    out->clear_counts = clear;
    out->nir_counts   = nir;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_BLE_LIVE_PAYLOAD_H_ */
