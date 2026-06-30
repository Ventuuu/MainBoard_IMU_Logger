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

// --- Constants Definitions ---
// These are standard values used for the BLE communication packets and timeouts.
#define TEXT_LENGTH 500     // Maximum length for text strings
#define UART_TIMEOUT 1000   // Timeout in milliseconds for UART operations

// --- Function Prototypes ---
// These are the public functions available for use.
void BLE_HardReset(void);
void BLE_Initialize(void);
void BLE_EnterDormantMode(void);
void BLE_WakeUp(void);
void BLE_EnterLowPowerMode(void);
void BLE_ExitLowPowerMode(void);
void BLE_SetSlowAdvertisements(void);
int BLE_ConfigureLowPower(void);
void BLE_WakeUart(void);
void BLE_ReleaseUartForLowPower(void);
void BLE_SendData(uint8_t* data, uint8_t data_length);
void BLE_ReceiveData(uint8_t* data, uint8_t data_length);
int BLE_TransmitTransparent(const uint8_t *data,
                            uint16_t data_length,
                            uint32_t timeout_ms);
int BLE_StartReceiveByteIT(uint8_t *byte);
void BLE_FlushTransparentReceive(void);

extern volatile uint32_t ble_lp_wake_count;
extern volatile uint32_t ble_lp_release_count;
extern volatile uint32_t ble_lp_config_success_count;
extern volatile uint32_t ble_lp_config_error_count;
extern volatile uint32_t ble_lp_last_wake_ms;
extern volatile uint32_t ble_lp_last_release_ms;
extern volatile uint8_t ble_uart_awake;
extern volatile uint8_t ble_lp_configured;

#endif /* INC_BLUETOOTH_H_ */
