/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application file for MainBoard_IMU_Logger project.
  ******************************************************************************
  * @functionality  : This firmware implements a complete Data Logger for the on board IMU
  *                   and the AS7341 spectral light sensor.
  * @details        : The application operates using a State Machine triggered by a
  * single USER BUTTON. It performs three primary tasks:
  * 1. Real-time Acquisition: Reads Accelerometer/Gyroscope data from the LSM6DSO16IS
  *    via I2C at 100 Hz (TIM2), and Clear/NIR channels plus selected spectral
  *    filters and flicker status from the AS7341 at ~10 Hz (every 10th timer tick)
  *    on the same I2C bus (hi2c3).
  * 2. Wireless Transmission: Sends data packets via Bluetooth Low Energy (BLE)
  *    using the UART interface.
  * 3. Data Logging: Saves acquired data to NAND Flash memory.
  *
  * Saved data can be downloaded via a USB Virtual COM Port (VCP)
  * connection, also initiated by the USER BUTTON.
  *
  * @intended_use   : Starting template for Smart Wearables Course
  * exploring IMU/light sensor interfacing, BLE communication, and memory management.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "../../USB_Device/App/usb_device.h"
#include "SPI.h"
#include "SPI_NAND.h"
#include "Memory_operations.h"
#include "led_driver.h"
#include "imu_driver.h"
#include "bluetooth.h"
#include "as7341_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Light sensor is sampled every LIGHT_SUBSAMPLE IMU ticks (100 Hz / 10 = 10 Hz) */
#define LIGHT_SUBSAMPLE  10U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c3;

MDF_HandleTypeDef MdfHandle0;
MDF_FilterConfigTypeDef MdfFilterConfig0;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

// --- State Machine ---
static AppState current_state = STATE_IDLE;

// --- Global Flags ---
uint8_t usb_flag = 0;

// --- IMU data ---
static IMU_Data accelerometer_data;
static IMU_Data gyroscope_data;

uint8_t raw_accelerometer[6] = {0};
uint8_t raw_gyroscope[6]     = {0};

// --- Light sensor data ---
static AS7341_Data light_data;
static AS7341_Spectrum spectrum;        /* full spectral frame */

/*
 * raw_light layout (14 bytes):
 *   [0..7]   4 selected filter channels (F1..F4 or similar, low/high SMUX
 *            groups – see Python decoder for exact mapping)
 *   [8..9]   Clear        (uint16, little-endian)
 *   [10..11] NIR          (uint16, little-endian)
 *   [12..13] Flicker freq (uint16, little-endian: 0, 1, 100 or 120)
 */
uint8_t raw_light[14] = {0};
static uint8_t light_tick = 0; /* subsample counter */

/// ----- NAND FLASH variables ----- ///

uint8_t NAND_packet[4096] = {0};
uint16_t sample = 0;
uint16_t blocco_scritto = 0;
uint8_t pagina_scritta=0;
uint16_t b = 0;

read_address_t blocco;
column_address_t colonna = 0;

uint16_t bad_blocks[2048]={-1};
uint8_t bad_blocks2[2048]={0};

uint8_t data_letto[4096] = {0};
int exit_flag = 0;

// Timestamp variables //
Time_Struct timestamp;
uint16_t tim = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_MDF1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_MDF1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  LED_On(LED_RED);

  BLE_Initialize();
  MX_USB_Device_Init();
  HAL_Delay(1000);

  spi_nand_init();
  find_bad_blocks(bad_blocks);

  if(IMU_Init() == 1) {
    IMU_ConfigAccelerometer(ACC_ODR_52HZ, ACC_FS_2G, 1);
    IMU_ConfigGyroscope(GYR_ODR_52HZ, GYR_FS_250DPS, 1);
  } else {
    LED_Toggle(LED_RED); HAL_Delay(500);
    LED_Toggle(LED_RED); HAL_Delay(500);
    LED_Toggle(LED_RED); HAL_Delay(500);
    LED_Toggle(LED_RED); HAL_Delay(500);
    LED_Toggle(LED_RED); HAL_Delay(500);
    LED_Toggle(LED_RED); HAL_Delay(500);
  }

  /* Initialize the AS7341 light sensor on the same I2C bus (hi2c3). */
  if (AS7341_Init() != 1) {
    /* Light sensor not found or failed: blink RED 3x quickly to warn,
     * but continue running (IMU logging still works). */
    for (uint8_t i = 0; i < 3; i++) {
      LED_Toggle(LED_RED); HAL_Delay(150);
      LED_Toggle(LED_RED); HAL_Delay(150);
    }
  }

  LED_Off(LED_RED);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(current_state)
	  {
	  	  case STATE_IDLE:
	  		if(!usb_flag)
		    {
	  			//MX_USB_Device_Init();
		    }
	  		else
	  		{
			   current_state = STATE_USB_CONNECTED;
			   LED_On(LED_GREEN);
		    }
	  		break;

	  	  case STATE_ACQUISITION:
	  		   /* All acquisition handled in TIM2 callback */
			break;

	  	  case STATE_USB_CONNECTED:
	  		break;

	  	  case STATE_DOWNLOAD:
	  		  read_memory_and_transmit();
			 current_state = STATE_USB_CONNECTED;
	  		 break;
	  }

  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  TIM2 period elapsed callback — 100 Hz IMU + 10 Hz light sensor.
  *
  * The IMU is read on every tick (100 Hz).
  * The AS7341 is read every LIGHT_SUBSAMPLE ticks (10 Hz) because its
  * integration time (~18 ms) is longer than one IMU tick (10 ms).
  * Between light reads, the previous raw_light[] value is reused in the
  * NAND packet so every record is the same fixed size (BYTES_PER_SAMPLE).
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){

        /* --- Read IMU (always) --- */
        IMU_ReadAccelerometerData(&accelerometer_data, raw_accelerometer);
        IMU_ReadGyroscopeData(&gyroscope_data, raw_gyroscope);

        /* --- Read light sensor (every LIGHT_SUBSAMPLE ticks = 10 Hz) --- */
        light_tick++;
        if (light_tick >= LIGHT_SUBSAMPLE) {
            light_tick = 0;

            /* Full spectrum: 12 channels (F1–F8, Clear, NIR) */
            if (AS7341_ReadFullSpectrum(&spectrum)) {
                /* Example mapping: take 4 filter channels plus Clear/NIR.
                 * Here we pick spectrum.ch[0..3] as 4 filters; adjust in
                 * Python script to assign exact wavelengths. */
                for (uint8_t i = 0; i < 4; i++) {
                    uint16_t v = spectrum.ch[i];
                    raw_light[2U * i]     = (uint8_t)(v & 0xFFU);
                    raw_light[2U * i + 1] = (uint8_t)(v >> 8);
                }

                /* Clear and NIR: use two of the remaining channels.
                 * For now, reuse previous Clear/NIR behaviour by mapping
                 * to channels 4 and 5; adjust if you change SMUX mapping. */
                uint16_t clear = spectrum.ch[4];
                uint16_t nir   = spectrum.ch[5];
                raw_light[8]  = (uint8_t)(clear & 0xFFU);
                raw_light[9]  = (uint8_t)(clear >> 8);
                raw_light[10] = (uint8_t)(nir & 0xFFU);
                raw_light[11] = (uint8_t)(nir >> 8);
            }

            /* Flicker: use Adafruit-style status codes (0, 1, 100, 120).
             * For now, call detectFlickerHz-equivalent via STATUS/FD_STATUS
             * helper when you add it to as7341_driver.c. Here we zero it
             * to keep the packet layout stable. */
            uint16_t flicker_hz = 0U;
            raw_light[12] = (uint8_t)(flicker_hz & 0xFFU);
            raw_light[13] = (uint8_t)(flicker_hz >> 8);
        }

        /* --- BLE transmission (IMU only, unchanged) --- */
        BLE_SendPacket(DATA_TYPE_IMU_ACCELERATION, raw_accelerometer);
        BLE_SendPacket(DATA_TYPE_IMU_GYROSCOPE, raw_gyroscope);

        /* --- Timestamp @ 100 Hz --- */
        timestamp.sss = tim * 10;
		if(timestamp.sss == 1000) {
			timestamp.ss++;
			timestamp.sss = 0;
			tim = 0;
			if (timestamp.ss == 60){
				timestamp.mm++;
				timestamp.ss = 0;
				if (timestamp.mm == 60){
					timestamp.hh++;
					timestamp.mm = 0;
				}
			}
		}
		tim++;

		/* --- Pack and save to NAND (IMU + light) --- */
		write_packet(sample, timestamp, raw_accelerometer, raw_gyroscope, raw_light, NAND_packet);
		sample++;
        write_memory();
	}
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == USER_BUTTON_Pin)
	{
		switch(current_state) {
			case STATE_IDLE:
				erase_memory();
				current_state = STATE_ACQUISITION;
				HAL_TIM_Base_Start_IT(&htim2);
				LED_On(LED_GREEN);
			break;
			case STATE_ACQUISITION:
				current_state = STATE_IDLE;
				HAL_TIM_Base_Stop_IT(&htim2);
				LED_Off(LED_GREEN);
				break;
			case STATE_USB_CONNECTED:
				exit_flag = 0;
				current_state = STATE_DOWNLOAD;
				break;
			default:
				break;
		}
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == USER_BUTTON_Pin)
	{
	}
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
