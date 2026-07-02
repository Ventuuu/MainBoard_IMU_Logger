/**
 * @file  ble_live_payload.h
 * @brief BLE live-streaming packet definitions and workflow interface.
 *
 * ============================================================
 * DUAL WORKFLOW OVERVIEW
 * ============================================================
 *
 * The firmware supports two mutually exclusive operating modes:
 *
 *  WORKFLOW A — BLE Live Mode (mobile app connected)
 *  ---------------------------------------------------
 *  Entry:  RN4871 signals "CONNECT" on transparent UART.
 *  Exit:   RN4871 signals "DISCONNECT", or connection-lost watchdog fires.
 *
 *  While active:
 *   - IMU metrics packet (BleLiveImuPayload)  sent every BLE_LIVE_IMU_INTERVAL_MS.
 *   - Light packet       (BleLiveLightPayload) sent every BLE_LIVE_ENV_INTERVAL_MS
 *                         only when exposure_class or blue_clear_ratio has changed.
 *   - Mic packet         (BleLiveMicPayload)   sent every BLE_LIVE_ENV_INTERVAL_MS
 *                         only when environment_class or laeq_x10 has changed.
 *   - NAND flash is NOT written.
 *   - USER BUTTON has NO effect.
 *   - The legacy BLE sync (bulk download) is blocked.
 *   - The RN4871 is woken with BLE_WakeUp() before each UART write and
 *     put back to low-power with BLE_EnterLowPowerMode() immediately after.
 *
 *  WORKFLOW B — Legacy Button-Driven Mode (no app connection)
 *  -----------------------------------------------------------
 *  All existing behaviour is preserved unchanged:
 *   - Short press → start acquisition (sensors + NAND logging).
 *   - Short press during acquisition → trigger BLE sync (bulk download).
 *   - USB connect → download mode.
 *   - 5-second hold → factory erase.
 *
 * ============================================================
 * PACKET TIMING
 * ============================================================
 *
 *  BLE_LIVE_IMU_INTERVAL_MS  =  1000 ms  (every second, unconditional)
 *  BLE_LIVE_ENV_INTERVAL_MS  =  7000 ms  (aligned to SENSOR_EPOCH_MS,
 *                                          change-gated for light and mic)
 *
 * ============================================================
 * LOW-POWER USAGE PATTERN
 * ============================================================
 *
 *  BLE_WakeUp();                          // from bluetooth.h
 *  BLE_TransmitTransparent(buf, len, t);  // from bluetooth.h
 *  BLE_EnterLowPowerMode();               // from bluetooth.h
 *
 * ============================================================
 */

#ifndef BLE_LIVE_PAYLOAD_H
#define BLE_LIVE_PAYLOAD_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Transmission interval constants
 *
 * BLE_LIVE_ENV_INTERVAL_MS is intentionally aligned to SENSOR_EPOCH_MS
 * (7000 ms) defined in main.c so that light and mic packets are always
 * dispatched at the natural end of a completed sensor epoch.
 * ----------------------------------------------------------------------- */

/** IMU metrics TX interval [ms] — unconditional, every second. */
#define BLE_LIVE_IMU_INTERVAL_MS    1000U

/**
 * Light / Mic TX interval [ms] — change-gated, epoch-aligned.
 * Must match SENSOR_EPOCH_MS in main.c (7000 ms).
 */
#define BLE_LIVE_ENV_INTERVAL_MS    7000U

/* -----------------------------------------------------------------------
 * Connection-lost watchdog
 *
 * If no UART activity is received from the RN4871 within this window
 * while in BLE Live mode, the firmware treats the link as lost and
 * returns to STATE_IDLE.
 * ----------------------------------------------------------------------- */
#define BLE_LIVE_WATCHDOG_MS        15000U

/* -----------------------------------------------------------------------
 * Message type identifiers  (first byte of every packet)
 * ----------------------------------------------------------------------- */

#define BLE_MSG_IMU_METRICS         0x50U  /**< IMU metrics packet              */
#define BLE_MSG_LIGHT_METRICS       0x51U  /**< Light/spectral metrics packet   */
#define BLE_MSG_MIC_METRICS         0x52U  /**< Microphone / audio env packet   */
#define BLE_MSG_CONNECTION_EVENT    0x53U  /**< Connect / disconnect notification*/

/* -----------------------------------------------------------------------
 * Packet structs  (packed — no internal padding, wire-safe)
 * ----------------------------------------------------------------------- */

/**
 * @brief IMU metrics packet — 7 bytes.
 *
 * Sent every BLE_LIVE_IMU_INTERVAL_MS regardless of value change.
 *
 * | Off | Sz | Field          | Notes                                    |
 * |-----|----|----------------|------------------------------------------|
 * |  0  |  1 | msg_type       | BLE_MSG_IMU_METRICS (0x50)               |
 * |  1  |  4 | step_count     | Cumulative steps since live-mode entry   |
 * |  5  |  1 | activity_state | BleLiveActivityState                     |
 * |  6  |  1 | reserved       | Always 0x00; reserved for future flags   |
 */
typedef struct __attribute__((packed))
{
    uint8_t  msg_type;        /**< BLE_MSG_IMU_METRICS                  */
    uint32_t step_count;      /**< Little-endian cumulative step count  */
    uint8_t  activity_state;  /**< BleLiveActivityState                 */
    uint8_t  reserved;        /**< Must be 0x00                         */
} BleLiveImuPayload;

/**
 * @brief Light / spectral metrics packet - 4 bytes.
 *
 * Sent every BLE_LIVE_ENV_INTERVAL_MS, **only** when exposure_class or
 * blue_clear_ratio has changed since the last transmission.
 *
 * | Off | Sz | Field            | Notes                               |
 * |-----|----|------------------|-------------------------------------|
 * |  0  |  1 | msg_type         | BLE_MSG_LIGHT_METRICS (0x51)        |
 * |  1  |  1 | exposure_class   | BleLiveLightExposureClass           |
 * |  2  |  2 | blue_clear_ratio | Blue / Clear ratio, scaled by 10000 |
 */
typedef struct __attribute__((packed))
{
    uint8_t  msg_type;          /**< BLE_MSG_LIGHT_METRICS              */
    uint8_t  exposure_class;    /**< BleLiveLightExposureClass          */
    uint16_t blue_clear_ratio;  /**< Blue / Clear ratio, scaled by 10000 */
} BleLiveLightPayload;

_Static_assert(sizeof(BleLiveLightPayload) == 4U,
               "Unexpected BleLiveLightPayload size");

/**
 * @brief Microphone / audio environment metrics packet — 4 bytes.
 *
 * Sent every BLE_LIVE_ENV_INTERVAL_MS, **only** when environment_class or
 * laeq_x10 has changed since the last transmission.
 *
 * | Off | Sz | Field             | Notes                                 |
 * |-----|----|-------------------|---------------------------------------|
 * |  0  |  1 | msg_type          | BLE_MSG_MIC_METRICS (0x52)            |
 * |  1  |  1 | environment_class | BleLiveEnvClass                       |
 * |  2  |  2 | laeq_x10          | LAeq × 10, little-endian (653 = 65.3) |
 */
typedef struct __attribute__((packed))
{
    uint8_t  msg_type;           /**< BLE_MSG_MIC_METRICS               */
    uint8_t  environment_class;  /**< BleLiveEnvClass                   */
    uint16_t laeq_x10;           /**< LAeq × 10 [dB], little-endian     */
} BleLiveMicPayload;

/**
 * @brief Connection event notification packet — 2 bytes.
 *
 * Sent by the MCU to the app immediately on entering or leaving BLE Live
 * mode, so the app can display accurate connection state.
 *
 * | Off | Sz | Field      | Notes                                        |
 * |-----|----|------------|----------------------------------------------|
 * |  0  |  1 | msg_type   | BLE_MSG_CONNECTION_EVENT (0x53)              |
 * |  1  |  1 | event      | BleLiveConnectionEvent                       |
 */
typedef struct __attribute__((packed))
{
    uint8_t msg_type;  /**< BLE_MSG_CONNECTION_EVENT                      */
    uint8_t event;     /**< BleLiveConnectionEvent                        */
} BleLiveConnectionPayload;

/* -----------------------------------------------------------------------
 * Enumeration types
 * ----------------------------------------------------------------------- */

/** Activity classification derived from IMU step-count delta and variance. */
typedef enum
{
    BLE_ACTIVITY_UNKNOWN    = 0x00,  /**< Insufficient data to classify    */
    BLE_ACTIVITY_STATIONARY = 0x01,  /**< No significant movement          */
    BLE_ACTIVITY_WALKING    = 0x02,  /**< Moderate cadence                 */
    BLE_ACTIVITY_RUNNING    = 0x03,  /**< High cadence / step rate         */
} BleLiveActivityState;

/** Light exposure classification (mirrors LightExposureClass in main.c). */
typedef enum
{
    BLE_LIGHT_EXPOSURE_DARK    = 0x00,  /**< < 3 clear counts              */
    BLE_LIGHT_EXPOSURE_DIM     = 0x01,  /**< 3 – 49 clear counts           */
    BLE_LIGHT_EXPOSURE_INDOOR  = 0x02,  /**< 50 – 6499 clear counts        */
    BLE_LIGHT_EXPOSURE_BRIGHT  = 0x03,  /**< 6500 – 9799 clear counts      */
    BLE_LIGHT_EXPOSURE_OUTDOOR = 0x04,  /**< ≥ 9800 clear counts           */
} BleLiveLightExposureClass;

/** Audio environment classification (subset of AudioEnvironmentClass). */
typedef enum
{
    BLE_ENV_CLASS_VERY_QUIET    = 0x00,  /**< LAeq < 35 dB                 */
    BLE_ENV_CLASS_QUIET         = 0x01,  /**< 35 – 44 dB                   */
    BLE_ENV_CLASS_MODERATE      = 0x02,  /**< 45 – 54 dB                   */
    BLE_ENV_CLASS_LIVELY        = 0x03,  /**< 55 – 64 dB                   */
    BLE_ENV_CLASS_NOISY         = 0x04,  /**< 65 – 74 dB                   */
    BLE_ENV_CLASS_VERY_NOISY    = 0x05,  /**< 75 – 84 dB                   */
    BLE_ENV_CLASS_HIGH_EXPOSURE = 0x06,  /**< ≥ 85 dB                      */
    BLE_ENV_CLASS_UNAVAILABLE   = 0xFF,  /**< Sensor not ready / invalid   */
} BleLiveEnvClass;

/** Connection event codes carried in BleLiveConnectionPayload. */
typedef enum
{
    BLE_CONN_EVENT_LIVE_START = 0x01,  /**< Live mode entered, sensors on  */
    BLE_CONN_EVENT_LIVE_STOP  = 0x02,  /**< Live mode exited, sensors off  */
} BleLiveConnectionEvent;

/* -----------------------------------------------------------------------
 * Workflow state tracking
 *
 * BleWorkflow is stored in the single global  g_ble_workflow  and is the
 * authoritative discriminator between the two operating modes:
 *
 *   BLE_WORKFLOW_LEGACY  — no app connected; user button drives everything.
 *   BLE_WORKFLOW_LIVE    — app connected; continuous sensors + live packets.
 *
 * Transitions (executed only from the main-loop workflow arbiter):
 *
 *   LEGACY → LIVE  : ble_connected rises AND current_state == STATE_IDLE
 *   LIVE   → LEGACY: ble_connected falls  OR  watchdog expires
 *                    (transition deferred if a factory-erase is in progress)
 * ----------------------------------------------------------------------- */

typedef enum
{
    BLE_WORKFLOW_LEGACY = 0,  /**< Default: button-driven, NAND logging   */
    BLE_WORKFLOW_LIVE   = 1,  /**< App-connected: live streaming, no NAND */
} BleWorkflow;

/* -----------------------------------------------------------------------
 * Live-mode interface
 *
 * These functions are implemented in ble_live.c (to be created) and called
 * from the main application loop.  All functions are non-blocking.
 * ----------------------------------------------------------------------- */

/**
 * @brief  Initialise the live-mode subsystem.
 *
 * Call once during board initialisation, after BLE_Initialize() returns.
 * Resets all internal timestamps and cached payload values.
 */
void BLE_Live_Init(void);

/**
 * @brief  Main-loop processing tick for the live-mode subsystem.
 *
 * Call from the main loop whenever current_state == STATE_BLE_LIVE.
 * Internally checks elapsed time against BLE_LIVE_IMU_INTERVAL_MS and
 * BLE_LIVE_ENV_INTERVAL_MS, wakes the RN4871, transmits due packets, and
 * returns the module to low-power mode.
 *
 * Also runs the connection-lost watchdog: if no UART RX activity is seen
 * within BLE_LIVE_WATCHDOG_MS, sets ble_connected = 0 so the workflow
 * arbiter can tear down the live session.
 *
 * @param  now_ms   Current HAL_GetTick() value.
 */
void BLE_Live_Process(uint32_t now_ms);

/**
 * @brief  Notify the live-mode subsystem that a BLE connection was established.
 *
 * Called by the workflow arbiter when ble_connected rises in STATE_IDLE.
 * Sends BLE_CONN_EVENT_LIVE_START to the app and initialises per-session
 * step-count and cached-value state.
 *
 * @param  now_ms   Current HAL_GetTick() value.
 */
void BLE_Live_OnConnected(uint32_t now_ms);

/**
 * @brief  Notify the live-mode subsystem that the BLE connection was lost.
 *
 * Called by the workflow arbiter when ble_connected falls or the watchdog
 * fires.  Sends BLE_CONN_EVENT_LIVE_STOP, stops sensors, and clears
 * per-session state.
 *
 * @param  now_ms   Current HAL_GetTick() value.
 */
void BLE_Live_OnDisconnected(uint32_t now_ms);

/**
 * @brief  Attempt to transmit an IMU metrics packet if the interval has elapsed.
 *
 * Called from BLE_Live_Process(); may also be called directly after a
 * ProcessSensorTick() that completes a 1-second IMU epoch.
 *
 * @param  now_ms        Current HAL_GetTick() value.
 * @param  step_count    Latest cumulative step count.
 * @param  activity      Latest activity classification.
 */
void BLE_Live_TryNotifyImu(uint32_t now_ms,
                           uint32_t step_count,
                           BleLiveActivityState activity);

/**
 * @brief  Attempt to transmit light and mic packets if the epoch has elapsed
 *         and values have changed.
 *
 * Called from BLE_Live_Process() after a sensor epoch completes.
 *
 * @param  now_ms        Current HAL_GetTick() value.
 * @param  exp_class     Latest light exposure class.
 * @param  blue_clear_ratio Latest Blue / Clear ratio, scaled by 10000.
 * @param  env_class     Latest audio environment class.
 * @param  laeq_x10      Latest LAeq × 10 value.
 */
void BLE_Live_TryNotifyEnv(uint32_t             now_ms,
                           BleLiveLightExposureClass exp_class,
                           uint16_t             blue_clear_ratio,
                           BleLiveEnvClass      env_class,
                           uint16_t             laeq_x10);

/* -----------------------------------------------------------------------
 * Connection detection helper
 *
 * The RN4871 sends ASCII status strings on its transparent UART when a
 * central connects or disconnects:
 *
 *   Connect:     "%CONNECT,1,<MAC>%\r\n"
 *   Disconnect:  "%DISCONNECT%\r\n"
 *
 * BleConnection_Process() should be called from the main loop at all
 * times (both workflows).  It parses incoming single-byte UART RX
 * interrupt data accumulated in the BLE RX ring buffer and sets or clears
 * the  ble_connected  flag accordingly.
 *
 * The workflow arbiter in main.c then acts on flag changes:
 *
 *   ble_connected rises  AND  current_state == STATE_IDLE
 *       → enter STATE_BLE_LIVE, call BLE_Live_OnConnected()
 *
 *   ble_connected falls  (any state)
 *       → if STATE_BLE_LIVE: call BLE_Live_OnDisconnected(), return to STATE_IDLE
 *       → if legacy state : ignore (connection attempt during acquisition;
 *                            will be handled when state returns to IDLE)
 * ----------------------------------------------------------------------- */

/**
 * @brief  Poll BLE UART RX buffer for connect/disconnect status strings.
 *
 * Call unconditionally from the main loop.  Updates ble_connected.
 *
 * @param  now_ms   Current HAL_GetTick() value.
 */
void BleConnection_Process(uint32_t now_ms);

#endif /* BLE_LIVE_PAYLOAD_H */
