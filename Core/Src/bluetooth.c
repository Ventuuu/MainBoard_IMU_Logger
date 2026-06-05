/**
 * @file bluetooth.c
 * @brief RN4871 BLE module driver — packet builders aligned with Flutter app.
 *
 * Packet format (20 bytes):
 *   [0]     0x7B '{'
 *   [1]     MsgType byte
 *   [2..18] payload (little-endian)
 *   [19]    0x7D '}'
 *
 * This matches the Flutter app's MainShell._onPacket() decoder exactly.
 * See bluetooth.h for field-by-field documentation.
 */

#include "bluetooth.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

/* --- Internal helpers ---------------------------------------------------- */

static void enter_command_mode(void);
static void exit_command_mode(void);

/** Write a UInt16 in little-endian order into dst[0] and dst[1]. */
static inline void put_u16le(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFU);
    dst[1] = (uint8_t)((v >> 8U) & 0xFFU);
}

/** Write an Int16 in little-endian order into dst[0] and dst[1]. */
static inline void put_i16le(uint8_t *dst, int16_t v)
{
    put_u16le(dst, (uint16_t)v);
}

/** Initialise a zeroed, framed 20-byte packet and set the MsgType byte. */
static inline void init_packet(uint8_t *pkt, uint8_t msg_type)
{
    memset(pkt, 0, PACKET_LENGTH);
    pkt[0]                 = 0x7BU;  /* '{' */
    pkt[1]                 = msg_type;
    pkt[PACKET_LENGTH - 1] = 0x7DU;  /* '}' */
}

/* --- Public: module lifecycle -------------------------------------------- */

void BLE_HardReset(void)
{
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);
}

void BLE_Initialize(void)
{
    uint8_t reboot_response[9]       = {0};
    uint8_t command_ok_response[100] = {0};

    BLE_HardReset();
    HAL_UART_Receive(&huart3, reboot_response, sizeof(reboot_response), UART_TIMEOUT);

    enter_command_mode();

    /* Device name — the app filters by the prefix "BLE_SW" */
    uint8_t device_name[] = "SN,BLE_SW\r";
    BLE_SendData(device_name, sizeof(device_name) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);

    /* Enable Transparent UART service (UUID 49535343-FE7D-...) */
    uint8_t enable_transparent_uart[] = "SS,C0\r";
    BLE_SendData(enable_transparent_uart, sizeof(enable_transparent_uart) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);

    /* UART RX indication pin */
    uint8_t enable_uart_rx_ind[] = "SW,0C,04\r";
    BLE_SendData(enable_uart_rx_ind, sizeof(enable_uart_rx_ind) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);

    uint8_t reboot_command[] = "R,1\r";
    BLE_SendData(reboot_command, sizeof(reboot_command) - 1);
    HAL_Delay(100);

    exit_command_mode();
}

void BLE_EnterDormantMode(void)
{
    uint8_t reboot_response[9] = {0};
    BLE_HardReset();
    HAL_UART_Receive(&huart3, reboot_response, sizeof(reboot_response), UART_TIMEOUT);
    HAL_GPIO_WritePin(BLE_UART_RX_IND_GPIO_Port, BLE_UART_RX_IND_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    enter_command_mode();
    uint8_t dormant_mode_command[] = "O,0\r";
    BLE_SendData(dormant_mode_command, sizeof(dormant_mode_command) - 1);
    HAL_Delay(100);
}

void BLE_WakeUp(void)
{
    HAL_GPIO_WritePin(BLE_UART_RX_IND_GPIO_Port, BLE_UART_RX_IND_Pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    BLE_Initialize();
}

void BLE_EnterLowPowerMode(void)
{
    uint8_t command_ok_response[100] = {0};
    enter_command_mode();
    uint8_t low_power_command[] = "SO,1\r";
    BLE_SendData(low_power_command, sizeof(low_power_command) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);
    exit_command_mode();
}

void BLE_ExitLowPowerMode(void)
{
    uint8_t command_ok_response[100] = {0};
    enter_command_mode();
    uint8_t exit_low_power_command[] = "SO,0\r";
    BLE_SendData(exit_low_power_command, sizeof(exit_low_power_command) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);
    exit_command_mode();
}

void BLE_SetSlowAdvertisements(void)
{
    uint8_t command_ok_response[100] = {0};
    enter_command_mode();
    uint8_t slow_ads_command[] = "A,03E8,002F\r";
    BLE_SendData(slow_ads_command, sizeof(slow_ads_command) - 1);
    HAL_UART_Receive(&huart3, command_ok_response, sizeof(command_ok_response), UART_TIMEOUT);
    exit_command_mode();
}

void BLE_SendData(uint8_t *data, uint8_t data_length)
{
    HAL_UART_Transmit(&huart3, data, data_length, UART_TIMEOUT);
}

void BLE_ReceiveData(uint8_t *data, uint8_t data_length)
{
    HAL_UART_Receive(&huart3, data, data_length, UART_TIMEOUT);
}

/* --- Public: typed packet senders ---------------------------------------- */

void BLE_SendImuAccelPacket(const BLE_ImuAccelPayload *payload)
{
    /*
     * IMU Accelerometer packet — MsgType 0x41 ('A')
     *
     *   [0]    0x7B
     *   [1]    0x41
     *   [2-3]  ax  (Int16-LE)
     *   [4-5]  ay  (Int16-LE)
     *   [6-7]  az  (Int16-LE)
     *   [8-9]  stepCount (UInt16-LE)
     *   [10-18] 0x00
     *   [19]   0x7D
     */
    uint8_t pkt[PACKET_LENGTH];
    init_packet(pkt, (uint8_t)DATA_TYPE_IMU_ACCELERATION);

    put_i16le(&pkt[2], payload->ax);
    put_i16le(&pkt[4], payload->ay);
    put_i16le(&pkt[6], payload->az);
    put_u16le(&pkt[8], payload->step_count);

    BLE_SendData(pkt, PACKET_LENGTH);
}

void BLE_SendImuGyroPacket(const BLE_ImuGyroPayload *payload)
{
    /*
     * IMU Gyroscope packet — MsgType 0x47 ('G')
     *
     *   [0]    0x7B
     *   [1]    0x47
     *   [2-3]  gx  (Int16-LE)
     *   [4-5]  gy  (Int16-LE)
     *   [6-7]  gz  (Int16-LE)
     *   [8-18] 0x00
     *   [19]   0x7D
     */
    uint8_t pkt[PACKET_LENGTH];
    init_packet(pkt, (uint8_t)DATA_TYPE_IMU_GYROSCOPE);

    put_i16le(&pkt[2], payload->gx);
    put_i16le(&pkt[4], payload->gy);
    put_i16le(&pkt[6], payload->gz);

    BLE_SendData(pkt, PACKET_LENGTH);
}

void BLE_SendLightPacket(const BLE_LightPayload *payload)
{
    /*
     * Light sensor packet — MsgType 0x4C ('L')
     *
     *   [0]    0x7B
     *   [1]    0x4C
     *   [2-3]  uv_risk              → app f1 (uvRisk)
     *   [4-5]  blue_light_intensity → app f2 (blueLightIntensity)
     *   [6-7]  blue_light_ratio     → app f3 (blueLightRatio)
     *   [8-9]  sun_like_index       → app f4 (sunLikeIndex)
     *   [10-11] metric1_clear       → app f5 (metric1)
     *   [12-18] 0x00
     *   [19]   0x7D
     */
    uint8_t pkt[PACKET_LENGTH];
    init_packet(pkt, (uint8_t)DATA_TYPE_LIGHT);

    put_u16le(&pkt[2],  payload->uv_risk);
    put_u16le(&pkt[4],  payload->blue_light_intensity);
    put_u16le(&pkt[6],  payload->blue_light_ratio);
    put_u16le(&pkt[8],  payload->sun_like_index);
    put_u16le(&pkt[10], payload->metric1_clear);

    BLE_SendData(pkt, PACKET_LENGTH);
}

/* Legacy generic packet sender (kept for backward compatibility). */
void BLE_SendPacket(BLE_DataType ble_data_type, uint8_t *data_buffer)
{
    uint8_t pkt[PACKET_LENGTH];
    init_packet(pkt, (uint8_t)ble_data_type);

    /* Preserve old behaviour: copy 6 axis bytes at [2..7]. */
    pkt[2] = data_buffer[0];
    pkt[3] = data_buffer[1];
    pkt[4] = data_buffer[2];
    pkt[5] = data_buffer[3];
    pkt[6] = data_buffer[4];
    pkt[7] = data_buffer[5];

    BLE_SendData(pkt, PACKET_LENGTH);
}

/* --- Internal helpers ---------------------------------------------------- */

static void enter_command_mode(void)
{
    uint8_t command_mode_sequence[] = "$$$";
    uint8_t command_prompt_response[5] = {0};
    BLE_SendData(command_mode_sequence, sizeof(command_mode_sequence) - 1);
    HAL_UART_Receive(&huart3, command_prompt_response,
                     sizeof(command_prompt_response), UART_TIMEOUT);
    HAL_Delay(100);
}

static void exit_command_mode(void)
{
    uint8_t data_mode_command[] = "---\r";
    BLE_SendData(data_mode_command, sizeof(data_mode_command) - 1);
    HAL_Delay(100);
}
