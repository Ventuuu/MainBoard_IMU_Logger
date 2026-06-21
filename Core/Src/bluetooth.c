/**
 * \file bluetooth.c
 * \brief Implementation file for RN4871 Bluetooth Low Energy functions.
 */

#include "bluetooth.h"
#include "main.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart3;

// --- Global RX Variables for Developer Mode ---
volatile bool is_raw_mode = false;   // Read by main.c to determine what to send
uint8_t ble_rx_byte;                 // 1-byte buffer for the UART interrupt
static uint8_t rx_cmdbuf[4];         // Holding tank for the 4-byte command
static uint8_t rx_cmd_idx = 0;

// --- Helper Functions (Internal to this file) ---
static void enter_command_mode(void);
static void exit_command_mode(void);

// --- Public Function Implementations ---

void BLE_HardReset(void) {
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
}

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

    //exit_command_mode();
}

/**
 * @brief Starts the asynchronous UART interrupt to listen for Flutter commands.
 * MUST be called once in main.c after BLE_Initialize().
 */
void BLE_StartRXInterrupt(void) {
    // Prime the UART to interrupt when 1 byte is received
    HAL_UART_Receive_IT(&huart3, &ble_rx_byte, 1);
}

/**
 * @brief Checks if the Flutter app has requested Developer Mode.
 */
bool BLE_IsRawModeActive(void) {
    return is_raw_mode;
}

void BLE_EnterDormantMode(void) { /* ... Kept identical ... */ }
void BLE_WakeUp(void) { /* ... Kept identical ... */ }
void BLE_EnterLowPowerMode(void) { /* ... Kept identical ... */ }
void BLE_ExitLowPowerMode(void) { /* ... Kept identical ... */ }
void BLE_SetSlowAdvertisements(void) { /* ... Kept identical ... */ }

void BLE_SendData(uint8_t *data, uint8_t data_length) {
    HAL_UART_Transmit(&huart3, data, data_length, UART_TIMEOUT);
}

void BLE_ReceiveData(uint8_t *data, uint8_t data_length) {
    HAL_UART_Receive(&huart3, data, data_length, UART_TIMEOUT);
}

/**
 * @brief Sends a structured data packet for 100Hz Raw Data (Developer Mode).
 */
void BLE_SendPacket(BLE_DataType ble_data_type, uint8_t *data_buffer) {
    uint8_t ble_packet[PACKET_LENGTH] = {0};

    ble_packet[0] = 0x7B; // Start sentinel '{'
    ble_packet[PACKET_LENGTH - 1] = 0x7D; // End sentinel '}'

    // Updated to match Flutter MsgType hex codes
    switch (ble_data_type) {
        case DATA_TYPE_IMU_ACCELERATION: ble_packet[1] = 0x01; break;
        case DATA_TYPE_IMU_GYROSCOPE:    ble_packet[1] = 0x02; break;
        default:                         ble_packet[1] = 0x00; break;
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
 * @brief Sends a structured data packet for the Live Battery %.
 */
void BLE_SendBatteryPacket(uint8_t battery_percent) {
    uint8_t frame[PACKET_LENGTH] = {0};
    
    frame[0] = 0x7B;       // Start sentinel
    frame[1] = 0xB0;       // MsgType.battery from Flutter
    frame[2] = battery_percent;
    frame[19] = 0x7D;      // End sentinel

    BLE_SendData(frame, PACKET_LENGTH);
}

/**
 * @brief Serialise and transmit a unified 20-byte sensor packet (Metrics Mode).
 */
void BLE_SendUnifiedPacket(const BLE_UnifiedPayload *payload) {
    uint8_t frame[PACKET_LENGTH] = {0};

    frame[0]  = 0x7B; // BLE_PKT_START
    frame[1]  = 0x55; // BLE_MSGTYPE_UNIFIED

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

    frame[19] = 0x7D; // BLE_PKT_END

    BLE_SendData(frame, PACKET_LENGTH);
}

/**
 * @brief Transmits the 8 visible spectral bands (F1-F8) in a single Dev Mode packet.
 */
void BLE_SendRawLightPacket(uint8_t *raw_light_array) {
    uint8_t frame[PACKET_LENGTH] = {0};

    frame[0] = 0x7B; // Start
    frame[1] = 0x03; // MsgType 0x03 = Raw Visible Spectrum
    
    // Copy only F1 through F8 (16 bytes)
    for(int i = 0; i < 16; i++) {
        frame[2 + i] = raw_light_array[i];
    }
    
    frame[18] = 0x00; // 1 byte of padding leftover
    frame[19] = 0x7D; // End
    
    BLE_SendData(frame, PACKET_LENGTH);
}
// /**
//  * @brief Splits the 22-byte raw light array into two 20-byte BLE packets for Developer Mode.
//  */
// void BLE_SendRawLightPackets(uint8_t *raw_light_array) {
//     uint8_t pkt1[PACKET_LENGTH] = {0};
//     uint8_t pkt2[PACKET_LENGTH] = {0};

//     // --- Packet 1: F1 through F8 (16 bytes of payload) ---
//     pkt1[0] = 0x7B;
//     pkt1[1] = 0x03; // MsgType 0x03 = Raw Visible Spectrum
//     for(int i = 0; i < 16; i++) {
//         pkt1[2 + i] = raw_light_array[i];
//     }
//     pkt1[19] = 0x7D;
//     BLE_SendData(pkt1, PACKET_LENGTH);

//     // --- Packet 2: Clear, NIR, and Flicker (6 bytes of payload) ---
//     pkt2[0] = 0x7B;
//     pkt2[1] = 0x04; // MsgType 0x04 = Raw Invisible Spectrum
//     for(int i = 0; i < 6; i++) {
//         pkt2[2 + i] = raw_light_array[16 + i];
//     }
//     pkt2[19] = 0x7D;
//     BLE_SendData(pkt2, PACKET_LENGTH);
// }

// --- Background UART RX Interrupt Handler ---

/**
 * @brief Catches incoming bytes from the RN4871 transparent UART in the background.
 * Looks for [0x7B, 0xC0, payload, 0x7D].
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        
        rx_cmdbuf[rx_cmd_idx++] = ble_rx_byte;

        // Check 1: Is the first byte our Start Marker?
        if (rx_cmd_idx == 1 && rx_cmdbuf[0] != 0x7B) {
            rx_cmd_idx = 0; // Garbage byte, throw it out
        } 
        // Check 2: Do we have a full 4-byte command?
        else if (rx_cmd_idx >= 4) {
            if (rx_cmdbuf[1] == 0xC0 && rx_cmdbuf[3] == 0x7D) {
                // Command is valid! Flip the mode.
                // 0x01 = Raw Mode, 0x00 = Metrics Mode
                is_raw_mode = (rx_cmdbuf[2] == 0x01);
            }
            rx_cmd_idx = 0; // Reset for the next command
        }

        // Re-arm the interrupt to catch the next byte
        HAL_UART_Receive_IT(&huart3, &ble_rx_byte, 1);
    }
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