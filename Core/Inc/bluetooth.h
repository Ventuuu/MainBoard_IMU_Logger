#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

/**
 * \file bluetooth.h
 * \brief Library for interacting with the RN4871 Bluetooth Low Energy module.
 *
 * This header file defines the functions and data types used to control the
 * RN4871 BLE module.
 */

#include "stdio.h"
#include "stdint.h"
#include <stdbool.h>

// --- Constants Definitions ---
#define PACKET_LENGTH  20    // Total BLE packet length in bytes
#define TEXT_LENGTH    500   // Maximum length for text strings
#define UART_TIMEOUT   1000  // Timeout in milliseconds for UART operations

// Unified packet byte markers
#define BLE_PKT_START       0x7Bu  // ASCII '{'
#define BLE_PKT_END         0x7Du  // ASCII '}'
#define BLE_MSGTYPE_UNIFIED 0x55u  // Unified sensor frame identifier

// --- Enumerations ---

/**
 * @brief Legacy per-sensor data type identifiers (kept for backward compat).
 */
typedef enum {
    DATA_TYPE_IMU_ACCELERATION, ///< Acceleration data from IMU
    DATA_TYPE_IMU_GYROSCOPE     ///< Gyroscope data from IMU
} BLE_DataType;

/**
 * @brief Activity state classification reported inside the unified packet.
 *
 * Byte [5] of the 20-byte frame.
 */
typedef enum {
    ACTIVITY_IDLE    = 0x00u, ///< No significant movement detected
    ACTIVITY_WALKING = 0x01u, ///< Walking gait detected
    ACTIVITY_RUNNING = 0x02u  ///< Running gait detected
} BLE_ActivityState;

/**
 * @brief Payload fields for the unified 20-byte BLE sensor packet.
 *
 * Populate this struct and pass it to BLE_SendUnifiedPacket().  All
 * multi-byte integers are stored in host byte-order here; the function
 * serialises them as little-endian on the wire.
 *
 * Frame layout (20 bytes):
 *   [0]      Start sentinel  : 0x7B
 *   [1]      MsgType         : 0x55
 *   [2..3]   stepCount       : UInt16-LE
 *   [4]      cadence         : UInt8
 *   [5]      activityState   : UInt8  (BLE_ActivityState)
 *   [6..7]   uvRisk          : UInt16-LE
 *   [8..9]   blueLightIntensity : UInt16-LE
 *   [10..11] blueLightRatio  : UInt16-LE  (Q15 fixed-point)
 *   [12..13] sunLikeIndex    : UInt16-LE
 *   [14..15] metric1_clear   : UInt16-LE
 *   [16..18] reserved        : 0x00 0x00 0x00
 *   [19]     End sentinel    : 0x7D
 */
typedef struct {
    uint16_t          stepCount;           ///< Cumulative steps since power-on
    uint8_t           cadence;             ///< Steps per minute
    BLE_ActivityState activityState;       ///< Current activity classification
    uint16_t          uvRisk;             ///< UV exposure proxy (abs. or frac.)
    uint16_t          blueLightIntensity; ///< Integrated AS7341 F3+F4 power
    uint16_t          blueLightRatio;     ///< Blue/total ratio, Q15 fixed-point
    uint16_t          sunLikeIndex;       ///< Outdoor/indoor discriminator
    uint16_t          metric1_clear;      ///< AS7341 clear channel raw count
} BLE_UnifiedPayload;

// --- Function Prototypes ---
void BLE_HardReset(void);
void BLE_Initialize(void);
void BLE_EnterDormantMode(void);
void BLE_WakeUp(void);
void BLE_EnterLowPowerMode(void);
void BLE_ExitLowPowerMode(void);
void BLE_SetSlowAdvertisements(void);
void BLE_SendData(uint8_t *data, uint8_t data_length);
void BLE_ReceiveData(uint8_t *data, uint8_t data_length);
void BLE_StartRXInterrupt(void);
bool BLE_IsRawModeActive(void);
void BLE_SendBatteryPacket(uint8_t battery_percent);
// Legacy packet sender (backward compatible)
void BLE_SendPacket(BLE_DataType ble_data_type, uint8_t *data_buffer);

void BLE_SendRawLightPacket(uint8_t *raw_light_array);
/**
 * @brief Serialise and transmit a unified 20-byte sensor packet.
 *
 * Builds the framed packet from @p payload and sends it over the
 * Transparent UART service.  All multi-byte fields are written
 * little-endian (LSB first).  The three reserved bytes [16..18] are
 * always zeroed.
 *
 * @param payload  Pointer to a populated BLE_UnifiedPayload struct.
 */
void BLE_SendUnifiedPacket(const BLE_UnifiedPayload *payload);

#endif /* INC_BLUETOOTH_H_ */
