/**
 * @file  ble_live.c
 * @brief BLE Live Mode — continuous sensor packet dispatch with RN4871
 *        low-power control.
 *
 * Implements the public API declared in ble_live_payload.h:
 *
 *   BLE_Live_Init()
 *   BLE_Live_OnConnected()
 *   BLE_Live_OnDisconnected()
 *   BLE_Live_Process()
 *   BLE_Live_TryNotifyImu()
 *   BLE_Live_TryNotifyEnv()
 *   BleConnection_Process()
 *
 * Low-power contract
 * ------------------
 * The RN4871 is kept in low-power sleep between transmissions.
 * Every outgoing UART write is bracketed:
 *
 *   BLE_WakeUp();
 *   BLE_TransmitTransparent(buf, len, UART_TIMEOUT);
 *   BLE_EnterLowPowerMode();
 *
 * Workflow arbitration
 * --------------------
 * This file owns g_ble_workflow and ble_connected.  main.c must call
 * BleConnection_Process() unconditionally from the main loop, and act on
 * g_ble_workflow transitions to enter / exit STATE_BLE_LIVE.
 *
 * Interaction with the legacy workflow
 * -------------------------------------
 * If BLE connects while an acquisition or BLE-sync session is already in
 * progress (STATE_ACQUISITION or STATE_BLE_SYNC), the connection flag is
 * latched but the transition to BLE_WORKFLOW_LIVE is deferred until the
 * device returns to STATE_IDLE.  The user button is completely ignored
 * while g_ble_workflow == BLE_WORKFLOW_LIVE.
 */

/* ---- Includes ---------------------------------------------------------- */
#include "ble_live_payload.h"
#include "bluetooth.h"
#include "main.h"        /* AppState, current_state, HAL_GetTick            */
#include <string.h>
#include <stdint.h>

/* ---- External symbols from main.c -------------------------------------- */
extern volatile AppState current_state;
extern volatile uint8_t  ble_sync_active;
extern volatile uint8_t  factory_erase_in_progress;

/* ---- Module-internal state --------------------------------------------- */

/** Authoritative workflow selector — read by main.c loop. */
BleWorkflow g_ble_workflow = BLE_WORKFLOW_LEGACY;

/**
 * Set to 1 by BleConnection_Process() when "%CONNECT" is parsed,
 * cleared when "%DISCONNECT" is parsed or the watchdog fires.
 * main.c reads this to drive STATE_BLE_LIVE entry/exit.
 */
volatile uint8_t ble_connected = 0U;

/* RX parser state for connect/disconnect detection */
#define RX_BUF_SIZE 64U
static uint8_t  s_rx_byte;                  /* single-byte IT target        */
static char     s_rx_buf[RX_BUF_SIZE];      /* rolling line accumulator     */
static uint8_t  s_rx_idx = 0U;

/* Per-session timestamps */
static uint32_t s_imu_last_tx_ms  = 0U;
static uint32_t s_env_last_tx_ms  = 0U;
static uint32_t s_watchdog_last_activity_ms = 0U;

/* Cached last-sent values for change-gating (env packets) */
static uint8_t  s_last_exposure_class   = 0xFFU;  /* invalid sentinel       */
static uint16_t s_last_blue_clear_ratio = 0xFFFFU;
static uint8_t  s_last_env_class        = 0xFFU;
static uint16_t s_last_laeq_x10        = 0xFFFFU;

/* Session step-count base (reset on each LIVE connection) */
static uint32_t s_step_count_base = 0U;

/* ---- Forward declarations ---------------------------------------------- */
static void Live_SendPacket(const uint8_t *buf, uint16_t len);
static void Live_SendConnectionEvent(BleLiveConnectionEvent evt);
static void Live_ResetSessionState(void);
static void RxParser_Feed(char c, uint32_t now_ms);

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Initialise the live-mode subsystem.
 *        Call once after BLE_Initialize() during board init.
 */
void BLE_Live_Init(void)
{
    g_ble_workflow  = BLE_WORKFLOW_LEGACY;
    ble_connected   = 0U;
    s_rx_idx        = 0U;
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    Live_ResetSessionState();

    /* Arm the first single-byte UART RX interrupt */
    BLE_StartReceiveByteIT((uint8_t *)&s_rx_byte);
}

/**
 * @brief Called by the workflow arbiter when ble_connected rises in
 *        STATE_IDLE.  Sends BLE_CONN_EVENT_LIVE_START to the app and
 *        initialises per-session state.
 */
void BLE_Live_OnConnected(uint32_t now_ms)
{
    Live_ResetSessionState();
    s_imu_last_tx_ms              = now_ms;
    s_env_last_tx_ms              = now_ms;
    s_watchdog_last_activity_ms   = now_ms;

    g_ble_workflow = BLE_WORKFLOW_LIVE;

    Live_SendConnectionEvent(BLE_CONN_EVENT_LIVE_START);
}

/**
 * @brief Called by the workflow arbiter when ble_connected falls or the
 *        watchdog fires.  Sends BLE_CONN_EVENT_LIVE_STOP and clears state.
 */
void BLE_Live_OnDisconnected(uint32_t now_ms)
{
    (void)now_ms;

    Live_SendConnectionEvent(BLE_CONN_EVENT_LIVE_STOP);
    Live_ResetSessionState();
    ble_connected  = 0U;
    g_ble_workflow = BLE_WORKFLOW_LEGACY;
}

/**
 * @brief Main-loop tick for the live-mode subsystem.
 *        Call only while current_state == STATE_BLE_LIVE.
 *
 * Checks elapsed intervals, transmits due packets, and runs the
 * connection-lost watchdog.
 */
void BLE_Live_Process(uint32_t now_ms)
{
    /* Connection-lost watchdog */
    if ((now_ms - s_watchdog_last_activity_ms) >= BLE_LIVE_WATCHDOG_MS)
    {
        ble_connected = 0U;
        BLE_Live_OnDisconnected(now_ms);
        return;
    }

    /* IMU packet is dispatched from BLE_Live_TryNotifyImu() which is
     * called by ProcessSensorTick() every second — nothing else needed here
     * for IMU.  ENV packets are dispatched from BLE_Live_TryNotifyEnv()
     * after each sensor epoch.  Both are driven externally; BLE_Live_Process
     * serves as the watchdog runner and a safety fallback. */
}

/**
 * @brief Transmit an IMU metrics packet if BLE_LIVE_IMU_INTERVAL_MS has
 *        elapsed since the last transmission.
 */
void BLE_Live_TryNotifyImu(uint32_t         now_ms,
                           uint16_t        step_count)
{
    BleLiveImuPayload pkt;

    if (g_ble_workflow != BLE_WORKFLOW_LIVE)
    {
        return;
    }

    if ((now_ms - s_imu_last_tx_ms) < BLE_LIVE_IMU_INTERVAL_MS)
    {
        return;
    }

    pkt.msg_type       = BLE_MSG_IMU_METRICS;
    pkt.step_count     = (step_count >= s_step_count_base)
                         ? (step_count - s_step_count_base)
                         : step_count;
    pkt.reserved       = 0x00U;

    Live_SendPacket((const uint8_t *)&pkt, (uint16_t)sizeof(pkt));
    s_imu_last_tx_ms = now_ms;
}

/**
 * @brief Transmit light and mic packets if BLE_LIVE_ENV_INTERVAL_MS has
 *        elapsed and respective values have changed since last TX.
 */
void BLE_Live_TryNotifyEnv(uint32_t              now_ms,
                           BleLiveLightExposureClass exp_class,
                           uint16_t              blue_clear_ratio,
                           BleLiveEnvClass        env_class,
                           uint16_t              laeq_x10)
{
    if (g_ble_workflow != BLE_WORKFLOW_LIVE)
    {
        return;
    }

    if ((now_ms - s_env_last_tx_ms) < BLE_LIVE_ENV_INTERVAL_MS)
    {
        return;
    }

    /* Light packet — change-gated */
    if (((uint8_t)exp_class != s_last_exposure_class) ||
        (blue_clear_ratio    != s_last_blue_clear_ratio))
    {
        BleLiveLightPayload lpkt;
        lpkt.msg_type         = BLE_MSG_LIGHT_METRICS;
        lpkt.exposure_class   = (uint8_t)exp_class;
        lpkt.blue_clear_ratio = blue_clear_ratio;

        Live_SendPacket((const uint8_t *)&lpkt, (uint16_t)sizeof(lpkt));

        s_last_exposure_class   = (uint8_t)exp_class;
        s_last_blue_clear_ratio = blue_clear_ratio;
    }

    /* Mic packet — change-gated */
    if (((uint8_t)env_class != s_last_env_class) ||
        (laeq_x10            != s_last_laeq_x10))
    {
        BleLiveMicPayload mpkt;
        mpkt.msg_type           = BLE_MSG_MIC_METRICS;
        mpkt.environment_class  = (uint8_t)env_class;
        mpkt.laeq_x10           = laeq_x10;

        Live_SendPacket((const uint8_t *)&mpkt, (uint16_t)sizeof(mpkt));

        s_last_env_class = (uint8_t)env_class;
        s_last_laeq_x10  = laeq_x10;
    }

    s_env_last_tx_ms = now_ms;
}

/**
 * @brief Poll BLE UART RX buffer for connect/disconnect status strings.
 *        Call unconditionally from the main loop (both workflows).
 */
void BleConnection_Process(uint32_t now_ms)
{
    /* Byte is accumulated one at a time via IT callback.
     * BLE_StartReceiveByteIT() re-arms itself in the HAL callback; we
     * consume the accumulated line here. */
    (void)now_ms;   /* now_ms passed to RxParser_Feed via the IT path below */
}

/* =========================================================================
 * HAL UART RX complete callback hook
 *
 * Called from stm32u5xx_it.c (or HAL_UART_RxCpltCallback override) every
 * time a single byte is received on the BLE UART.
 * ========================================================================= */

/**
 * @brief  Weak-overridable hook — called by the UART RX complete ISR.
 *         Feeds the received byte into the line parser and re-arms the IT.
 *
 * @note   This function is called from interrupt context; keep it short.
 */
void BLE_Live_RxByteCallback(void)
{
    uint32_t now_ms = HAL_GetTick();

    /* Update watchdog on any RX activity while in LIVE mode */
    if (g_ble_workflow == BLE_WORKFLOW_LIVE)
    {
        s_watchdog_last_activity_ms = now_ms;
    }

    RxParser_Feed((char)s_rx_byte, now_ms);

    /* Re-arm for the next byte */
    BLE_StartReceiveByteIT((uint8_t *)&s_rx_byte);
}

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/**
 * @brief Wake RN4871, transmit buf, return to low-power mode.
 */
static void Live_SendPacket(const uint8_t *buf, uint16_t len)
{
    BLE_WakeUp();
    BLE_TransmitTransparent(buf, len, UART_TIMEOUT);
    BLE_EnterLowPowerMode();
}

/**
 * @brief Send a BleLiveConnectionPayload event packet.
 */
static void Live_SendConnectionEvent(BleLiveConnectionEvent evt)
{
    BleLiveConnectionPayload pkt;
    pkt.msg_type = BLE_MSG_CONNECTION_EVENT;
    pkt.event    = (uint8_t)evt;
    Live_SendPacket((const uint8_t *)&pkt, (uint16_t)sizeof(pkt));
}

/**
 * @brief Reset all per-session cached state (called on connect and disconnect).
 */
static void Live_ResetSessionState(void)
{
    s_imu_last_tx_ms              = 0U;
    s_env_last_tx_ms              = 0U;
    s_watchdog_last_activity_ms   = 0U;
    s_step_count_base             = 0U;
    s_last_exposure_class         = 0xFFU;
    s_last_blue_clear_ratio       = 0xFFFFU;
    s_last_env_class              = 0xFFU;
    s_last_laeq_x10               = 0xFFFFU;
}

/**
 * @brief Accumulate received bytes into a line buffer and detect
 *        RN4871 status strings.
 *
 * RN4871 transparent UART status strings:
 *   Connect:    "%CONNECT,1,<MAC>%\r\n"
 *   Disconnect: "%DISCONNECT%\r\n"
 */
static void RxParser_Feed(char c, uint32_t now_ms)
{
    (void)now_ms;

    /* Discard non-printable except CR/LF */
    if ((c == '\r') || (c == '\n'))
    {
        /* Null-terminate and parse */
        s_rx_buf[s_rx_idx] = '\0';

        if (strstr(s_rx_buf, "%CONNECT") != NULL)
        {
            if (ble_connected == 0U)
            {
                ble_connected = 1U;
                /* Transition to LIVE deferred to main-loop arbiter so it
                 * can check current_state == STATE_IDLE safely. */
            }
        }
        else if (strstr(s_rx_buf, "%DISCONNECT") != NULL)
        {
            ble_connected = 0U;
            /* If we were in LIVE mode the main-loop arbiter calls
             * BLE_Live_OnDisconnected() on the next iteration. */
        }

        /* Reset accumulator */
        s_rx_idx = 0U;
        memset(s_rx_buf, 0, sizeof(s_rx_buf));
        return;
    }

    if (s_rx_idx < (RX_BUF_SIZE - 1U))
    {
        s_rx_buf[s_rx_idx++] = c;
    }
    else
    {
        /* Overflow — discard and restart */
        s_rx_idx = 0U;
        memset(s_rx_buf, 0, sizeof(s_rx_buf));
    }
}
