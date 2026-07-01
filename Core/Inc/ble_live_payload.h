/**
 * @file  ble_live_payload.h
 * @brief BLE live metrics packet definitions.
 *
 * Three independent fixed-size packets are used:
 *   - IMU metrics  : sent every BLE_LIVE_IMU_INTERVAL_MS  (1 s)
 *   - Light metrics: sent every BLE_LIVE_ENV_INTERVAL_MS  (3 s), only on value change
 *   - Mic metrics  : sent every BLE_LIVE_ENV_INTERVAL_MS  (3 s), only on value change
 *
 * The RN4871 is kept in low-power deep sleep between transmissions.
 * Wake it with BLE_WAKE() before any UART write, then assert BLE_SLEEP() afterwards.
 */

#ifndef BLE_LIVE_PAYLOAD_H
#define BLE_LIVE_PAYLOAD_H

#include <stdint.h>
#include "stm32u5xx_hal.h"

/* -----------------------------------------------------------------------
 * Transmission interval constants
 * ----------------------------------------------------------------------- */

/** IMU metrics transmission interval [ms] — tied to step-count reporting. */
#define BLE_LIVE_IMU_INTERVAL_MS     1000U

/** Light / Mic metrics transmission interval [ms] — epoch-driven, change-only. */
#define BLE_LIVE_ENV_INTERVAL_MS     3000U

/* -----------------------------------------------------------------------
 * Message type identifiers  (1-byte header field)
 * ----------------------------------------------------------------------- */

#define BLE_MSG_IMU_METRICS          0x50U   /**< IMU metrics packet    */
#define BLE_MSG_LIGHT_METRICS        0x51U   /**< Light metrics packet  */
#define BLE_MSG_MIC_METRICS          0x52U   /**< Mic metrics packet    */

/* -----------------------------------------------------------------------
 * Packet structs  (packed, no padding)
 * ----------------------------------------------------------------------- */

/**
 * @brief IMU metrics packet — 7 bytes total.
 *
 * Sent every BLE_LIVE_IMU_INTERVAL_MS regardless of change.
 *
 * | Offset | Size | Field          | Description                           |
 * |--------|------|----------------|---------------------------------------|
 * |   0    |  1   | msg_type       | Always BLE_MSG_IMU_METRICS (0x50)     |
 * |   1    |  4   | step_count     | Cumulative step count (little-endian) |
 * |   5    |  1   | activity_state | See BleLiveActivityState enum         |
 * |   6    |  1   | reserved       | Set to 0x00                           |
 */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;          /**< BLE_MSG_IMU_METRICS */
    uint32_t step_count;        /**< Cumulative steps since boot */
    uint8_t  activity_state;    /**< BleLiveActivityState */
    uint8_t  reserved;
} BleLiveImuPayload;

/**
 * @brief Light metrics packet — 3 bytes total.
 *
 * Sent every BLE_LIVE_ENV_INTERVAL_MS, only when value has changed.
 *
 * | Offset | Size | Field                  | Description                         |
 * |--------|------|------------------------|-------------------------------------|
 * |   0    |  1   | msg_type               | Always BLE_MSG_LIGHT_METRICS (0x51) |
 * |   1    |  1   | exposure_class         | See BleLiveLightExposureClass enum  |
 * |   2    |  1   | light_color_intensity  | 0-255 normalized intensity          |
 */
typedef struct __attribute__((packed)) {
    uint8_t msg_type;               /**< BLE_MSG_LIGHT_METRICS */
    uint8_t exposure_class;         /**< BleLiveLightExposureClass */
    uint8_t light_color_intensity;  /**< Normalized 0-255 */
} BleLiveLightPayload;

/**
 * @brief Mic / audio environment metrics packet — 4 bytes total.
 *
 * Sent every BLE_LIVE_ENV_INTERVAL_MS, only when value has changed.
 *
 * | Offset | Size | Field             | Description                                    |
 * |--------|------|-------------------|------------------------------------------------|
 * |   0    |  1   | msg_type          | Always BLE_MSG_MIC_METRICS (0x52)              |
 * |   1    |  1   | environment_class | See BleLiveEnvClass enum                       |
 * |   2    |  2   | laeq_x10          | LAeq x10, little-endian (e.g. 653 = 65.3 dB)  |
 */
typedef struct __attribute__((packed)) {
    uint8_t  msg_type;            /**< BLE_MSG_MIC_METRICS */
    uint8_t  environment_class;   /**< BleLiveEnvClass */
    uint16_t laeq_x10;            /**< LAeq x10 in dB, little-endian */
} BleLiveMicPayload;

/* -----------------------------------------------------------------------
 * Enum definitions
 * ----------------------------------------------------------------------- */

typedef enum {
    BLE_ACTIVITY_UNKNOWN    = 0x00,
    BLE_ACTIVITY_STATIONARY = 0x01,
    BLE_ACTIVITY_WALKING    = 0x02,
    BLE_ACTIVITY_RUNNING    = 0x03,
} BleLiveActivityState;

typedef enum {
    BLE_LIGHT_EXPOSURE_DARK    = 0x00,
    BLE_LIGHT_EXPOSURE_DIM     = 0x01,
    BLE_LIGHT_EXPOSURE_INDOOR  = 0x02,
    BLE_LIGHT_EXPOSURE_BRIGHT  = 0x03,
    BLE_LIGHT_EXPOSURE_OUTDOOR = 0x04,
} BleLiveLightExposureClass;

typedef enum {
    BLE_ENV_CLASS_QUIET     = 0x00,
    BLE_ENV_CLASS_MODERATE  = 0x01,
    BLE_ENV_CLASS_LOUD      = 0x02,
    BLE_ENV_CLASS_VERY_LOUD = 0x03,
} BleLiveEnvClass;

/* -----------------------------------------------------------------------
 * RN4871 low-power control helpers
 *
 * Usage pattern:
 *   BLE_WAKE();
 *   BLE_UART_Transmit(&imu_pkt, sizeof(imu_pkt));
 *   BLE_SLEEP();
 *
 * GPIO pin definitions below must match your CubeMX board configuration.
 * ----------------------------------------------------------------------- */

/** GPIO port/pin for the RN4871 SW_BTN / wake line — adjust to match main.h */
#define BLE_WAKE_GPIO_PORT   GPIOB
#define BLE_WAKE_GPIO_PIN    GPIO_PIN_5

/**
 * @brief Wake the RN4871 from deep sleep.
 *
 * Pulls SW_BTN low for >2 ms, then releases it and waits for the
 * module to become ready before returning.
 */
#define BLE_WAKE()   do { \
    HAL_GPIO_WritePin(BLE_WAKE_GPIO_PORT, BLE_WAKE_GPIO_PIN, GPIO_PIN_RESET); \
    HAL_Delay(3U); \
    HAL_GPIO_WritePin(BLE_WAKE_GPIO_PORT, BLE_WAKE_GPIO_PIN, GPIO_PIN_SET); \
    HAL_Delay(5U); /* Allow module to fully wake and stabilise */ \
} while(0)

/**
 * @brief Put the RN4871 into deep sleep.
 *
 * Call immediately after UART transmission is complete.
 * Asserts SW_BTN low to trigger the deep-sleep entry sequence.
 */
#define BLE_SLEEP()  do { \
    HAL_Delay(2U); /* Ensure UART TX FIFO is flushed */ \
    HAL_GPIO_WritePin(BLE_WAKE_GPIO_PORT, BLE_WAKE_GPIO_PIN, GPIO_PIN_RESET); \
} while(0)

#endif /* BLE_LIVE_PAYLOAD_H */
