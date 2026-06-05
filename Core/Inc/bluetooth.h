/**
 * @file bluetooth.h
 * @brief Library for interacting with the RN4871 Bluetooth Low Energy module.
 *
 * Packet protocol (20 bytes, framed):
 *
 *   Byte  0    : 0x7B  '{'
 *   Byte  1    : MsgType byte  (see BLE_DataType below)
 *   Bytes 2-18 : payload, sensor-specific, all values little-endian
 *   Byte  19   : 0x7D  '}'
 *
 * MsgType bytes match the Flutter app's MsgType enum exactly:
 *   0x41 ('A') imuAccel  — ax, ay, az (Int16-LE) + stepCount (UInt16-LE)
 *   0x47 ('G') imuGyro   — gx, gy, gz (Int16-LE)
 *   0x4C ('L') light     — uvRisk, blueLightIntensity, blueLightRatio,
 *                           sunLikeIndex, metric1 (all UInt16-LE)
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include <stdio.h>
#include <stdint.h>

/* --- Constants ----------------------------------------------------------- */
#define PACKET_LENGTH   20      /* Fixed BLE packet length (bytes) */
#define TEXT_LENGTH     500     /* Maximum length for text strings */
#define UART_TIMEOUT    1000    /* UART operation timeout (ms) */

/* --- Data type identifiers ----------------------------------------------- */
/**
 * BLE_DataType maps to the Flutter app's MsgType enum byte values.
 * Do NOT change the numeric values — they are part of the wire protocol.
 */
typedef enum {
    DATA_TYPE_IMU_ACCELERATION = 0x41,  /* 'A' — imuAccel  */
    DATA_TYPE_IMU_GYROSCOPE    = 0x47,  /* 'G' — imuGyro   */
    DATA_TYPE_LIGHT            = 0x4C,  /* 'L' — light     */
} BLE_DataType;

/* --- Structured payload types ------------------------------------------- */

/**
 * Payload for DATA_TYPE_IMU_ACCELERATION.
 * ax/ay/az are raw Int16 LSB values from the IMU (app applies kAccelSens).
 * step_count is the cumulative step counter from the firmware pedometer.
 */
typedef struct {
    int16_t  ax;
    int16_t  ay;
    int16_t  az;
    uint16_t step_count;
} BLE_ImuAccelPayload;

/**
 * Payload for DATA_TYPE_IMU_GYROSCOPE.
 * gx/gy/gz are raw Int16 LSB values (app applies kGyroSens).
 */
typedef struct {
    int16_t gx;
    int16_t gy;
    int16_t gz;
} BLE_ImuGyroPayload;

/**
 * Payload for DATA_TYPE_LIGHT.
 * All fields are unsigned 16-bit; the app stores them as REAL in the
 * sensor_snapshots table columns f1..f5:
 *
 *   f1 = uv_risk              — UV proxy (F1+F2+F3)/sum * CLEAR, 16-bit
 *   f2 = blue_light_intensity — BlueIndex = F3+F4 (16-bit)
 *   f3 = blue_light_ratio     — BlueFrac Q15 (0..32767 maps to 0.0..1.0)
 *   f4 = sun_like_index       — RedFrac Q15  (0..32767 maps to 0.0..1.0)
 *                               High value → sun-like spectrum (outdoor)
 *                               Low value  → blue-dominant artificial light
 *   f5 = metric1              — CLEAR channel raw count (brightness proxy)
 */
typedef struct {
    uint16_t uv_risk;
    uint16_t blue_light_intensity;
    uint16_t blue_light_ratio;
    uint16_t sun_like_index;
    uint16_t metric1_clear;
} BLE_LightPayload;

/* --- Function Prototypes ------------------------------------------------- */
void BLE_HardReset(void);
void BLE_Initialize(void);
void BLE_EnterDormantMode(void);
void BLE_WakeUp(void);
void BLE_EnterLowPowerMode(void);
void BLE_ExitLowPowerMode(void);
void BLE_SetSlowAdvertisements(void);
void BLE_SendData(uint8_t *data, uint8_t data_length);
void BLE_ReceiveData(uint8_t *data, uint8_t data_length);

/**
 * @brief Send a structured IMU accelerometer packet (MsgType 0x41 'A').
 * @param payload  Pointer to ax/ay/az + step_count values.
 */
void BLE_SendImuAccelPacket(const BLE_ImuAccelPayload *payload);

/**
 * @brief Send a structured IMU gyroscope packet (MsgType 0x47 'G').
 * @param payload  Pointer to gx/gy/gz values.
 */
void BLE_SendImuGyroPacket(const BLE_ImuGyroPayload *payload);

/**
 * @brief Send a structured light sensor packet (MsgType 0x4C 'L').
 * @param payload  Pointer to all five light metric fields.
 */
void BLE_SendLightPacket(const BLE_LightPayload *payload);

/**
 * @brief Legacy generic packet sender (kept for backward compatibility).
 * @deprecated Use BLE_SendImuAccelPacket / BLE_SendImuGyroPacket instead.
 */
void BLE_SendPacket(BLE_DataType ble_data_type, uint8_t *data_buffer);

#endif /* INC_BLUETOOTH_H_ */
