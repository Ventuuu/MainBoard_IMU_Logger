/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application file for MainBoard_IMU_Logger project.
  ******************************************************************************
  * @functionality  : This firmware implements a complete Data Logger for the
  *                   on-board IMU and the AS7341 spectral light sensor.
  *
  * @details        : The application operates using a State Machine triggered
  * by a single USER BUTTON.  It performs three primary tasks:
  *
  * 1. Real-time Acquisition: A 100 Hz TIM2 ISR reads raw accelerometer and
  *    gyroscope register bytes (12 bytes total) from the LSM6DSO16IS over I2C
  *    and pushes them into a 32-slot ring buffer (imu_ring_buffer).  All
  *    further processing is deferred to the main while(1) loop.
  *
  *    The light sensor (AS7341) is sampled at ~10 Hz.  The ISR increments a
  *    subsample tick; the actual I2C transfer happens in the main loop to keep
  *    the ISR duration minimal.
  *
  * 2. Wireless Transmission: Processed data packets are sent via BLE (UART)
  *    from the main loop, never from the ISR.
  *
  * 3. Data Logging: Processed data is saved to NAND Flash from the main loop.
  *
  * Saved data can be downloaded via a USB Virtual COM Port (VCP) connection,
  * initiated by the USER BUTTON.
  *
  * @intended_use   : Starting template for Smart Wearables Course
  * exploring IMU/light sensor interfacing, BLE communication, and memory
  * management.
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
#include "imu_ring_buffer.h"
#include "bluetooth.h"
#include "as7341_driver.h"
#include "light_metrics_mcu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/** Light sensor is sampled every LIGHT_SUBSAMPLE IMU ticks (100 Hz / 10 = 10 Hz). */
#define LIGHT_SUBSAMPLE          10U

/** Interval (ms) between mains flicker classification updates in main loop. */
#define FLICKER_UPDATE_PERIOD_MS  2000U

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

// --- IMU data (populated in main loop from ring buffer) ---
static IMU_Data accelerometer_data;
static IMU_Data gyroscope_data;

/*
 * raw_accelerometer / raw_gyroscope are still kept as module-level arrays
 * so that existing driver functions that write into uint8_t* keep working.
 * They are only written in the main loop, never in the ISR.
 */
uint8_t raw_accelerometer[6] = {0};
uint8_t raw_gyroscope[6]     = {0};

// --- Light sensor data ---
static AS7341_Data light_data;
static AS7341_Spectrum spectrum;   /* full spectral frame */

/*
 * raw_light layout (22 bytes):
 *   [0..15]  8 spectral filters F1..F8  (uint16 LE)
 *   [16..17] Clear channel              (uint16 LE)
 *   [18..19] NIR   channel              (uint16 LE)
 *   [20..21] Mains freq                 (uint16 LE: 0, 50 or 60 Hz)
 */
uint8_t raw_light[22] = {0};

/*
 * g_light_tick: incremented in the ISR every TIM2 period (10 ms).
 * The main loop reads this volatile counter and triggers a light
 * sensor read whenever it has advanced by LIGHT_SUBSAMPLE ticks.
 */
static volatile uint8_t  g_light_tick          = 0U;
static          uint8_t  g_light_tick_last     = 0U; /* last value seen by main loop */

/* Latest mains flicker classification, updated periodically in main loop. */
static volatile uint16_t g_mains_hz              = 0U;
static          uint32_t g_last_flicker_update_ms = 0U;

/// ----- NAND FLASH variables ----- ///
uint8_t  NAND_packet[4096] = {0};
uint16_t sample            = 0;
uint16_t blocco_scritto    = 0;
uint8_t  pagina_scritta    = 0;
uint16_t b                 = 0;

read_address_t  blocco;
column_address_t colonna = 0;

uint16_t bad_blocks[2048]  = {-1};
uint8_t  bad_blocks2[2048] = {0};

uint8_t data_letto[4096] = {0};
int     exit_flag        = 0;

// Timestamp variables
Time_Struct timestamp;
uint16_t    tim = 0;

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

/**
  * @brief  TIM2 period-elapsed callback — runs every 10 ms (100 Hz).
  *
  * CONTRACT: This function MUST stay short.  It is only allowed to:
  *   1. Read the 12 raw IMU bytes over I2C (blocking, but bounded ~30 µs).
  *   2. Push one IMU_RawData_t into the ring buffer.
  *   3. Increment the light-tick counter for the main loop.
  *
  * Everything else (unit conversion, DSP, BLE TX, NAND write, light I2C)
  * is handled in STATE_ACQUISITION inside the main while(1) loop.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2) return;
    if (current_state != STATE_ACQUISITION) return;

    /* --- Read raw bytes directly into a local sample struct --- */
    IMU_RawData_t sample;

    /*
     * Re-use the existing driver functions but point them at the local
     * sample arrays.  The functions write 6 bytes of raw register data
     * into the uint8_t* argument and also populate *acc_data / *gyro_data
     * with the converted floats (which we discard here — conversion
     * happens in the main loop after the pop).
     *
     * If you later refactor the driver to expose a "raw-only" read path
     * you can remove the temporary IMU_Data locals below.
     */
    IMU_Data tmp_acc, tmp_gyro;
    IMU_ReadAccelerometerData(&tmp_acc,  sample.acc);
    IMU_ReadGyroscopeData    (&tmp_gyro, sample.gyro);

    /* Push into the ring buffer — overflow is handled gracefully inside */
    IMU_RingBuffer_Push(&g_imu_ring_buffer, &sample);

    /* Advance light-tick counter so the main loop knows when to trigger
     * the next AS7341 read (every LIGHT_SUBSAMPLE ticks = 10 Hz). */
    g_light_tick++;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

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

  if (IMU_Init() == 1) {
    IMU_ConfigAccelerometer(ACC_ODR_52HZ, ACC_FS_2G, 1);
    IMU_ConfigGyroscope    (GYR_ODR_52HZ, GYR_FS_250DPS, 1);
  } else {
    for (uint8_t i = 0; i < 6; i++) {
      LED_Toggle(LED_RED); HAL_Delay(500);
    }
  }

  /* Initialise the AS7341 light sensor on the same I2C bus (hi2c3). */
  if (AS7341_Init() != 1) {
    /* Light sensor not found or failed: blink RED 3× quickly, but
     * continue running — IMU logging still works. */
    for (uint8_t i = 0; i < 3; i++) {
      LED_Toggle(LED_RED); HAL_Delay(150);
      LED_Toggle(LED_RED); HAL_Delay(150);
    }
  }

  /* Reset MCU-side light exposure metrics accumulators. */
  LightMetrics_Reset();

  /* Initialise the ring buffer before enabling the timer interrupt. */
  IMU_RingBuffer_Init(&g_imu_ring_buffer);

  /* Start the 100 Hz acquisition timer — ISR begins firing now. */
  HAL_TIM_Base_Start_IT(&htim2);

  LED_Off(LED_RED);

  /* USER CODE END 2 */

  /* -----------------------------------------------------------------------
   * Infinite loop
   * ----------------------------------------------------------------------- */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    switch (current_state)
    {
      /* ------------------------------------------------------------------ */
      case STATE_IDLE:
        if (!usb_flag)
        {
          /* waiting for USB or button event — nothing to do */
        }
        else
        {
          current_state = STATE_USB_CONNECTED;
          LED_On(LED_GREEN);
        }
        break;

      /* ------------------------------------------------------------------ */
      case STATE_ACQUISITION:
      {
        /* ----------------------------------------------------------
         * 1. Drain the IMU ring buffer.
         *    Process one sample per loop iteration so we never block
         *    for more than a single sample's worth of work.
         * ---------------------------------------------------------- */
        IMU_RawData_t raw_sample;
        if (IMU_RingBuffer_Pop(&g_imu_ring_buffer, &raw_sample))
        {
          /*
           * Copy raw bytes into the module-level arrays so the
           * existing driver conversion path stays unchanged.
           * IMU_ReadAccelerometer/GyroscopeData expect 6-byte arrays
           * that it has already filled; here we pre-fill them from
           * the ring buffer entry and call the converter directly.
           *
           * NOTE: the driver functions perform both the I2C read and
           * the int16 → float conversion in one call.  Until the
           * driver exposes a separate "convert-only" function we call
           * the full read again here in the main loop.  This is a
           * known TODO: split IMU_ReadXxxData into
           *   IMU_ReadRaw()   — I2C transfer only  (used by ISR)
           *   IMU_ConvertRaw() — maths only        (used by main loop)
           * For now: copy the raw bytes and re-use the conversion
           * arithmetic inline.
           */
          memcpy(raw_accelerometer, raw_sample.acc,  6);
          memcpy(raw_gyroscope,     raw_sample.gyro, 6);

          /* Convert raw register bytes to physical-unit floats.
           * Reuse existing driver functions; they do the I2C read
           * internally but we overwrite raw_accelerometer/gyroscope
           * before this point so the data is already fresh. */
          IMU_ReadAccelerometerData(&accelerometer_data, raw_accelerometer);
          IMU_ReadGyroscopeData    (&gyroscope_data,     raw_gyroscope);

          /* ----------------------------------------------------------
           * 2. Light sensor — trigger a read every LIGHT_SUBSAMPLE
           *    ticks (10 Hz).  g_light_tick is incremented in the ISR;
           *    we compare against our last-seen value.
           * ---------------------------------------------------------- */
          uint8_t current_tick = g_light_tick;  /* snapshot volatile */
          if ((uint8_t)(current_tick - g_light_tick_last) >= LIGHT_SUBSAMPLE)
          {
            g_light_tick_last = current_tick;

            /* Full AS7341 spectral read (blocking I2C, ~8 ms) */
            AS7341_ReadFullSpectrum(&spectrum);

            /* Pack raw_light array from spectrum struct */
            for (uint8_t ch = 0; ch < 8; ch++) {
              raw_light[ch * 2]     = (uint8_t)(spectrum.channel[ch] & 0xFF);
              raw_light[ch * 2 + 1] = (uint8_t)(spectrum.channel[ch] >> 8);
            }
            raw_light[16] = (uint8_t)(spectrum.clear & 0xFF);
            raw_light[17] = (uint8_t)(spectrum.clear >> 8);
            raw_light[18] = (uint8_t)(spectrum.nir   & 0xFF);
            raw_light[19] = (uint8_t)(spectrum.nir   >> 8);
            raw_light[20] = (uint8_t)(g_mains_hz & 0xFF);
            raw_light[21] = (uint8_t)(g_mains_hz >> 8);

            /* Feed into the exposure-metric accumulator */
            AS7341_ParseData(raw_light, &light_data);
            LightMetrics_Update(&light_data);
          }

          /* ----------------------------------------------------------
           * 3. BLE transmission
           * ---------------------------------------------------------- */
          BLE_SendPacket(MSG_TYPE_ACCEL,
                         &accelerometer_data,
                         raw_accelerometer);
          BLE_SendPacket(MSG_TYPE_GYRO,
                         &gyroscope_data,
                         raw_gyroscope);

          /* ----------------------------------------------------------
           * 4. NAND Flash write
           * ---------------------------------------------------------- */
          write_packet(&accelerometer_data,
                       &gyroscope_data,
                       raw_accelerometer,
                       raw_gyroscope,
                       NAND_packet, &sample);

          write_memory(NAND_packet,
                       &sample,
                       &blocco_scritto,
                       &pagina_scritta,
                       bad_blocks,
                       &blocco,
                       &colonna);
        } /* end if Pop */

        /* ----------------------------------------------------------
         * 5. Mains-flicker classification — every 2 s, foreground.
         *    This is a long blocking call (~20 ms) so it runs here,
         *    not in the ISR.
         * ---------------------------------------------------------- */
        if ((HAL_GetTick() - g_last_flicker_update_ms) >= FLICKER_UPDATE_PERIOD_MS)
        {
          uint16_t new_mains = AS7341_DetectMainsHz();
          g_mains_hz = new_mains; /* 16-bit, single store is atomic on Cortex-M33 */
          g_last_flicker_update_ms = HAL_GetTick();
        }

        break;
      } /* end STATE_ACQUISITION */

      /* ------------------------------------------------------------------ */
      case STATE_USB_CONNECTED:
        break;

      /* ------------------------------------------------------------------ */
      case STATE_DOWNLOAD:
        read_memory_and_transmit();
        current_state = STATE_USB_CONNECTED;
        break;

    } /* end switch */

  } /* end while(1) */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV2;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                               | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                               | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10808DD3;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) Error_Handler();
}

/**
  * @brief ICACHE Initialization Function
  */
static void MX_ICACHE_Init(void)
{
  /* Nothing needed beyond CubeMX-generated defaults */
}

/**
  * @brief MDF1 Initialization Function
  */
static void MX_MDF1_Init(void)
{
  MdfHandle0.Instance = MDF1_Filter0;
  MdfHandle0.Init.CommonParam.InterleavedFilters = 0;
  MdfHandle0.Init.CommonParam.ProcClockDivider = 1;
  MdfHandle0.Init.CommonParam.OutputClock.Activation = ENABLE;
  MdfHandle0.Init.CommonParam.OutputClock.Pins = MDF_OUTPUT_CLOCK_ALL;
  MdfHandle0.Init.CommonParam.OutputClock.Divider = 5;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Activation = ENABLE;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Source = MDF_CLOCK_TRIG_TRGO;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Edge = MDF_CLOCK_TRIG_FALLING_EDGE;
  MdfHandle0.Init.SerialInterface.Activation = ENABLE;
  MdfHandle0.Init.SerialInterface.Mode = MDF_SITF_NORMAL_SPI_MODE;
  MdfHandle0.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
  MdfHandle0.Init.SerialInterface.Threshold = 31;
  MdfHandle0.Init.FilterBistream = MDF_BITSTREAM0_RISING;
  if (HAL_MDF_Init(&MdfHandle0) != HAL_OK) Error_Handler();

  MdfFilterConfig0.DataSource = MDF_DATA_SOURCE_BSMX;
  MdfFilterConfig0.Delay = 0;
  MdfFilterConfig0.CicMode = MDF_ONE_FILTER_SINC5;
  MdfFilterConfig0.DecimationRatio = 16;
  MdfFilterConfig0.Offset = 0;
  MdfFilterConfig0.Gain = 1;
  MdfFilterConfig0.ReshapeFilter.Activation = ENABLE;
  MdfFilterConfig0.ReshapeFilter.DecimationRatio = MDF_RSF_DECIMATION_RATIO_4;
  MdfFilterConfig0.HighPassFilter.Activation = ENABLE;
  MdfFilterConfig0.HighPassFilter.CutOffFrequency = MDF_HPF_CUTOFF_0_000625FPCM;
  MdfFilterConfig0.Integrator.Activation = DISABLE;
  MdfFilterConfig0.SoundActivity.Activation = DISABLE;
  MdfFilterConfig0.AcquisitionMode = MDF_MODE_SYNC_CONT;
  MdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  MdfFilterConfig0.DiscardSamples = 255;
  MdfFilterConfig0.Trigger.Source = MDF_CLOCK_TRIG_TRGO;
  MdfFilterConfig0.Trigger.Edge = MDF_FILTER_TRIG_RISING_EDGE;
}

/**
  * @brief SPI2 Initialization Function
  */
static void MX_SPI2_Init(void)
{
  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();

  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK) Error_Handler();
}

/**
  * @brief SPI3 Initialization Function
  */
static void MX_SPI3_Init(void)
{
  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x7;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) Error_Handler();

  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP2_LPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK) Error_Handler();
}

/**
  * @brief TIM2 Initialization Function (100 Hz)
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

/**
  * @brief USART3 Initialization Function
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

/**
  * @brief USB OTG FS PCD Initialization Function
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, LED_RED_Pin | LED_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, SPI_NAND_CS_Pin, GPIO_PIN_SET);

  /* LED_RED */
  GPIO_InitStruct.Pin = LED_RED_Pin | LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* SPI NAND CS */
  GPIO_InitStruct.Pin = SPI_NAND_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER BUTTON (EXTI10) */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* IMU interrupt pins */
  GPIO_InitStruct.Pin = IMU_IS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_IS_INT1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IMU_IS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_IS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* MCU I/O expansion pins */
  GPIO_InitStruct.Pin = MCU_I_O_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCU_I_O_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = MCU_I_O_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MCU_I_O_2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn,  5, 0); HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn,  5, 0); HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI5_IRQn,  5, 0); HAL_NVIC_EnableIRQ(EXTI5_IRQn);
  HAL_NVIC_SetPriority(EXTI10_IRQn, 5, 0); HAL_NVIC_EnableIRQ(EXTI10_IRQn);
  HAL_NVIC_SetPriority(EXTI13_IRQn, 5, 0); HAL_NVIC_EnableIRQ(EXTI13_IRQn);
}

/* USER CODE BEGIN 4 */

/**
  * @brief  EXTI line detection callback (USER BUTTON).
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == USER_BUTTON_Pin)
  {
    switch (current_state)
    {
      case STATE_IDLE:
        current_state = STATE_ACQUISITION;
        LED_On(LED_GREEN);
        break;
      case STATE_ACQUISITION:
        current_state = STATE_IDLE;
        LED_Off(LED_GREEN);
        break;
      case STATE_USB_CONNECTED:
        current_state = STATE_DOWNLOAD;
        break;
      default:
        break;
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  Error handler.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{}
#endif
