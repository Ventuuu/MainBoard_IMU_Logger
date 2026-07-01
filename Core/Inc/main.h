/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../USB_Device/App/usb_device.h"
#include <bluetooth.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
void App_UpdateDownloadLed(void);
void App_UpdateFactoryEraseLed(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IMU_IS_INT1_Pin GPIO_PIN_13
#define IMU_IS_INT1_GPIO_Port GPIOC
#define IMU_IS_INT1_EXTI_IRQn EXTI13_IRQn
#define IMU_IS_INT2_Pin GPIO_PIN_0
#define IMU_IS_INT2_GPIO_Port GPIOA
#define IMU_IS_INT2_EXTI_IRQn EXTI0_IRQn
#define SPI3_CS_NAND_Pin GPIO_PIN_4
#define SPI3_CS_NAND_GPIO_Port GPIOA
#define USER_BUTTON_Pin GPIO_PIN_10
#define USER_BUTTON_GPIO_Port GPIOB
#define USER_BUTTON_EXTI_IRQn EXTI10_IRQn
#define BLE_P0_0_Pin GPIO_PIN_6
#define BLE_P0_0_GPIO_Port GPIOC
#define BLE_P3_6_Pin GPIO_PIN_7
#define BLE_P3_6_GPIO_Port GPIOC
#define BLE_UART_RX_IND_Pin GPIO_PIN_8
#define BLE_UART_RX_IND_GPIO_Port GPIOC
#define BLE_RESET_Pin GPIO_PIN_9
#define BLE_RESET_GPIO_Port GPIOC
#define BLE_CONFIG_Pin GPIO_PIN_15
#define BLE_CONFIG_GPIO_Port GPIOA
#define MCU_I_O_2_Pin GPIO_PIN_4
#define MCU_I_O_2_GPIO_Port GPIOB
#define MCU_I_O_2_EXTI_IRQn EXTI4_IRQn
#define MCU_I_O_1_Pin GPIO_PIN_5
#define MCU_I_O_1_GPIO_Port GPIOB
#define MCU_I_O_1_EXTI_IRQn EXTI5_IRQn
#define MCU_GREEN_LED_Pin GPIO_PIN_6
#define MCU_GREEN_LED_GPIO_Port GPIOB
#define MCU_RED_LED_Pin GPIO_PIN_7
#define MCU_RED_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define I2C_TIMEOUT 100

/**
 * @brief Main application state machine.
 *
 * Two mutually exclusive top-level workflows share these states:
 *
 * WORKFLOW A — BLE Live (mobile app connected)
 * ------------------------------------------------
 *   STATE_IDLE  ──(BLE connect detected)──►  STATE_BLE_LIVE
 *   STATE_BLE_LIVE  ──(BLE disconnect)──►    STATE_IDLE
 *
 *   In STATE_BLE_LIVE:
 *     - Sensors run continuously (IMU @ 100 Hz, light+mic @ 7 s epoch).
 *     - Computed metrics are transmitted as live BLE packets on their
 *       respective intervals (see ble_live_payload.h).
 *     - NAND flash is NOT written.
 *     - USER BUTTON is ignored.
 *     - The legacy BLE sync (bulk download) is blocked.
 *
 * WORKFLOW B — Legacy Button-Driven (no app connection)
 * -------------------------------------------------------
 *   STATE_IDLE  ──(short press)──►  STATE_ACQUISITION
 *                                       │
 *                         (short press) ▼
 *                               STATE_BLE_SYNC  (bulk NAND download)
 *   STATE_IDLE  ──(USB connect)──►  STATE_USB_CONNECTED
 *                                       │
 *                         (short press) ▼
 *                               STATE_DOWNLOAD  (VCP data export)
 *   STATE_IDLE  ──(5 s hold)───►  STATE_FACTORY_ERASE
 *
 * A BLE connection event while in any legacy state is queued and acted
 * upon only after that state returns to STATE_IDLE.
 */
typedef enum {
    STATE_IDLE,           /**< Waiting; BLE advertising in background.          */
    STATE_ACQUISITION,    /**< Legacy: actively logging sensor data to NAND.    */
    STATE_USB_CONNECTED,  /**< Legacy: USB VCP connected.                       */
    STATE_DOWNLOAD,       /**< Legacy: streaming NAND pages over USB VCP.       */
    STATE_BLE_SYNC,       /**< Legacy: bulk NAND page download over BLE UART.   */
    STATE_FACTORY_ERASE,  /**< Legacy: erasing all NAND data + BLE sync metadata.*/
    STATE_BLE_LIVE        /**< BLE Live: continuous sensor read + live packets. */
} AppState;

/**
 * @brief  Set by the BLE transparent-UART receive callback when the RN4871
 *         signals a connection (status string "CONNECT" received on UART).
 *         Cleared when "DISCONNECT" is received or when the connection-lost
 *         watchdog fires.
 *
 * Written only from: BLE UART RX interrupt / BleConnection_Process().
 * Read from:         main-loop workflow arbiter.
 *
 * @note   Declared volatile because it is shared between ISR and main-loop.
 */
extern volatile uint8_t ble_connected;

/**
 * @brief  Set to 1 while STATE_BLE_LIVE is the active workflow.
 *         Guards sensor start/stop and NAND write inhibit logic.
 *
 * Written only from: main-loop workflow arbiter.
 * Read from:         ProcessSensorTick, AudioScheduler, NAND append paths.
 */
extern volatile uint8_t ble_live_mode_active;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
