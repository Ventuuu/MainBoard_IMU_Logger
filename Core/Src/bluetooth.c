/**
 * \file bluetooth.c
 * \brief Implementation file for RN4871 Bluetooth Low Energy functions.
 *
 * This file contains the implementation of functions for configuring and
 * communicating with the Microchip RN4871 Bluetooth® Low Energy Module.
 *
 * The RN4871 module is a fully certified Bluetooth Smart module that is
 * controlled primarily through ASCII commands sent from a host MCU to its UART.
 * The module operates in two main modes:
 * - Data mode: Acts as a data pipe, transparently transferring serial data.
 * - Command mode: Interprets UART data as ASCII commands for configuration.
 *
 * Default UART Settings:
 * Baud Rate: 115200
 * Data Bits: 8
 * Parity: None
 * Stop Bits: 1
 * Flow Control: Disabled
 */

#include <bluetooth.h>
#include "main.h"

extern UART_HandleTypeDef huart3;

// --- Helper Functions (Internal to this file) ---
static void enter_command_mode(void);
static void exit_command_mode(void);

// --- Public Function Implementations ---

/**
 * @brief Performs a hard reset of the BLE module via the MCU's reset pin.
 */
void BLE_HardReset(void) {
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, 0);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, 1);
}

/**
 * @brief Configures the RN4871 BLE Module at startup.
 */
void BLE_Initialize(void) {
    uint8_t reboot_response[9]       = {0};
    uint8_t command_ok_response[100] = {0};

    BLE_HardReset();
    HAL_UART_Receive(&huart3, reboot_response, sizeof(reboot_response), UART_TIMEOUT);

    enter_command_mode();

    uint8_t device_name[] = "SN,A9 - Big Bad Board\r";
    BLE_SendData(device_name, sizeof(device_name) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);

    uint8_t enable_transparent_uart[] = "SS,C0\r";
    BLE_SendData(enable_transparent_uart, sizeof(enable_transparent_uart) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);

    uint8_t enable_uart_rx_ind[] = "SW,0C,04\r";
    BLE_SendData(enable_uart_rx_ind, sizeof(enable_uart_rx_ind) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);

    uint8_t reboot_command[] = "R,1\r";
    BLE_SendData(reboot_command, sizeof(reboot_command) - 1);
    HAL_Delay(100);

    exit_command_mode();
}

/**
 * @brief Configures the RN4871 BLE Module to enter Dormant (Deep Sleep) mode.
 */
void BLE_EnterDormantMode(void) {
    uint8_t reboot_response[9] = {0};

    BLE_HardReset();
    HAL_UART_Receive(&huart3, reboot_response, sizeof(reboot_response), UART_TIMEOUT);

    HAL_GPIO_WritePin(BLE_UART_RX_IND_GPIO_Port, BLE_UART_RX_IND_Pin, 1);
    HAL_Delay(10);

    enter_command_mode();

    uint8_t dormant_mode_command[] = "O,0\r";
    BLE_SendData(dormant_mode_command, sizeof(dormant_mode_command) - 1);
    HAL_Delay(100);
}

/**
 * @brief Wakes the RN4871 BLE Module from Dormant/Sleep mode.
 */
void BLE_WakeUp(void) {
    HAL_GPIO_WritePin(BLE_UART_RX_IND_GPIO_Port, BLE_UART_RX_IND_Pin, 0);
    HAL_Delay(5);
    BLE_Initialize();
}

/**
 * @brief Configures the RN4871 BLE Module to enter Low-Power mode.
 */
void BLE_EnterLowPowerMode(void) {
    uint8_t command_ok_response[100] = {0};
    enter_command_mode();
    uint8_t low_power_command[] = "SO,1\r";
    BLE_SendData(low_power_command, sizeof(low_power_command) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);
    exit_command_mode();
}

/**
 * @brief Exits the RN4871 BLE Module from Low-Power mode.
 */
void BLE_ExitLowPowerMode(void) {
    uint8_t command_ok_response[100] = {0};
    enter_command_mode();
    uint8_t exit_low_power_command[] = "SO,0\r";
    BLE_SendData(exit_low_power_command, sizeof(exit_low_power_command) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);
    exit_command_mode();
}

/**
 * @brief Configures the RN4871 BLE Module for slow advertisements (1000 ms interval).
 */
void BLE_SetSlowAdvertisements(void) {
    uint8_t command_ok_response[100] = {0};
    enter_command_mode();
    uint8_t slow_ads_command[] = "A,03E8,002F\r";
    BLE_SendData(slow_ads_command, sizeof(slow_ads_command) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);
    exit_command_mode();
}

/**
 * @brief Sends raw bytes to the connected BLE device via Transparent UART.
 */
void BLE_SendData(uint8_t *data, uint8_t data_length) {
    HAL_UART_Transmit(&huart3, data, data_length, UART_TIMEOUT);
}

/**
 * @brief Receives raw bytes from the connected BLE device.
 */
void BLE_ReceiveData(uint8_t *data, uint8_t data_length) {
    HAL_UART_Receive(&huart3, data, data_length, UART_TIMEOUT);
}

/**
 * @brief (Legacy) Sends a structured data packet for IMU acceleration or gyroscope.
 *
 * Packet layout (PACKET_LENGTH = 20 bytes):
 *   [0]    '{'
 *   [1]    'A' | 'G' | 'U'  (data type byte)
 *   [2..7] 6 axis bytes (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
 *   [8..18] 0x00 padding
 *   [19]   '}'
 *
 * @param ble_data_type  DATA_TYPE_IMU_ACCELERATION or DATA_TYPE_IMU_GYROSCOPE.
 * @param data_buffer    Pointer to 6 bytes: [X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB].
 */
void BLE_SendPacket(BLE_DataType ble_data_type, uint8_t *data_buffer) {
    uint8_t ble_packet[PACKET_LENGTH];

    ble_packet[0]                 = '{';
    ble_packet[PACKET_LENGTH - 1] = '}';
    for (uint8_t i = 1; i < PACKET_LENGTH - 1; i++) {
        ble_packet[i] = 0;
    }

    switch (ble_data_type) {
        case DATA_TYPE_IMU_ACCELERATION: ble_packet[1] = 'A'; break;
        case DATA_TYPE_IMU_GYROSCOPE:    ble_packet[1] = 'G'; break;
        default:                         ble_packet[1] = 'U'; break;
    }

    ble_packet[2] = data_buffer[0]; // X LSB
    ble_packet[3] = data_buffer[1]; // X MSB
    ble_packet[4] = data_buffer[2]; // Y LSB
    ble_packet[5] = data_buffer[3]; // Y MSB
    ble_packet[6] = data_buffer[4]; // Z LSB
    ble_packet[7] = data_buffer[5]; // Z MSB

    BLE_SendData(ble_packet, sizeof(ble_packet));
}

/**
 * @brief Serialise and transmit a unified 20-byte sensor packet.
 *
 * Frame layout:
 *   [0]      0x7B  - start sentinel
 *   [1]      0x55  - MsgType: unified sensor frame
 *   [2..3]   stepCount        (UInt16-LE)
 *   [4]      cadence          (UInt8)
 *   [5]      activityState    (UInt8: 0=Idle, 1=Walking, 2=Running)
 *   [6..7]   uvRisk           (UInt16-LE)
 *   [8..9]   blueLightIntensity (UInt16-LE)
 *   [10..11] blueLightRatio   (UInt16-LE, Q15 fixed-point)
 *   [12..13] sunLikeIndex     (UInt16-LE)
 *   [14..15] metric1_clear    (UInt16-LE)
 *   [16..18] reserved         (0x00 x3)
 *   [19]     0x7D  - end sentinel
 *
 * @param payload  Pointer to a fully populated BLE_UnifiedPayload struct.
 */
void BLE_SendUnifiedPacket(const BLE_UnifiedPayload *payload) {
    uint8_t frame[PACKET_LENGTH];

    frame[0]  = BLE_PKT_START;
    frame[1]  = BLE_MSGTYPE_UNIFIED;

    frame[2]  = (uint8_t)(payload->stepCount & 0xFFu);
    frame[3]  = (uint8_t)(payload->stepCount >> 8u);
    frame[4]  = payload->cadence;
    frame[5]  = (uint8_t)payload->activityState;

    frame[6]  = (uint8_t)(payload->uvRisk & 0xFFu);
    frame[7]  = (uint8_t)(payload->uvRisk >> 8u);
    frame[8]  = (uint8_t)(payload->blueLightIntensity & 0xFFu);
    frame[9]  = (uint8_t)(payload->blueLightIntensity >> 8u);
    frame[10] = (uint8_t)(payload->blueLightRatio & 0xFFu);
    frame[11] = (uint8_t)(payload->blueLightRatio >> 8u);
    frame[12] = (uint8_t)(payload->sunLikeIndex & 0xFFu);
    frame[13] = (uint8_t)(payload->sunLikeIndex >> 8u);
    frame[14] = (uint8_t)(payload->metric1_clear & 0xFFu);
    frame[15] = (uint8_t)(payload->metric1_clear >> 8u);

    frame[16] = 0x00u;
    frame[17] = 0x00u;
    frame[18] = 0x00u;

    frame[19] = BLE_PKT_END;

    BLE_SendData(frame, PACKET_LENGTH);
}

// --- Helper Function Implementations ---

static void enter_command_mode(void) {
    uint8_t command_mode_sequence[]    = "$$$";
    uint8_t command_prompt_response[5] = {0};
    BLE_SendData(command_mode_sequence, sizeof(command_mode_sequence) - 1);
    HAL_UART_Receive(&huart3, command_prompt_response, sizeof(command_prompt_response), UART_TIMEOUT);
    HAL_Delay(100);
}

static void exit_command_mode(void) {
    uint8_t data_mode_command[] = "---\r";
    BLE_SendData(data_mode_command, sizeof(data_mode_command) - 1);
    HAL_Delay(100);
}
