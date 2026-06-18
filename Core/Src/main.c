/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application — Flag-and-Fetch IMU architecture.
  ******************************************************************************
  * @details
  *
  * Interrupt / main-loop split
  * ---------------------------------------------------------------------------
  *
  *  TIM2 ISR  (100 Hz, NVIC priority 6)            duration < 1 µs
  *  -----------------------------------------------------------------------
  *  • Sets  g_imu_fetch_flag = 1
  *  • Increments g_light_tick
  *  • Returns immediately — NO I2C, NO math, NO memory writes.
  *
  *  STATE_ACQUISITION in while(1)                  thread context
  *  -----------------------------------------------------------------------
  *  Fetch path  (triggered by g_imu_fetch_flag)
  *    1. Clear flag inside __disable_irq critical section.
  *    2. HAL_I2C_Master_Transmit / Receive for accelerometer  ← blocking,
  *    3. HAL_I2C_Master_Transmit / Receive for gyroscope        but now the
  *       NVIC can preempt these at any time — BLE UART (prio 5)
  *       and USB (prio 5) will seamlessly interrupt, service the
  *       radio / host, and return here to finish the I2C wait.
  *    4. Pack bytes → IMU_RawData_t → IMU_RingBuffer_Push.
  *
  *  Drain path  (runs every iteration, independent of fetch)
  *    5. IMU_RingBuffer_Pop → convert raw bytes → IMU_Data floats.
  *    6. Light sensor read every LIGHT_SUBSAMPLE ticks (10 Hz).
  *    7. BLE_SendUnifiedPacket (sent once per 1-second light window).
  *    8. write_packet + write_memory  (NAND Flash).
  *    9. Mains flicker update every 2 s.
  *
  * NVIC priority table
  * ---------------------------------------------------------------------------
  *  Priority 5  USART3_IRQn   — RN4871 BLE UART
  *  Priority 5  OTG_FS_IRQn   — USB VCP
  *  Priority 6  TIM2_IRQn     — 100 Hz tripwire (flag only)
  *  Priority 6  EXTI*_IRQn    — USER_BUTTON, IMU data-ready
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
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
/** Light sensor sampled every LIGHT_SUBSAMPLE IMU ticks (100 Hz / 10 = 10 Hz) */
#define LIGHT_SUBSAMPLE           10U
/** Interval (ms) between mains flicker classification updates */
#define FLICKER_UPDATE_PERIOD_MS  2000U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef   hi2c3;
MDF_HandleTypeDef   MdfHandle0;
MDF_FilterConfigTypeDef MdfFilterConfig0;
SPI_HandleTypeDef   hspi2;
SPI_HandleTypeDef   hspi3;
TIM_HandleTypeDef   htim2;
UART_HandleTypeDef  huart3;
PCD_HandleTypeDef   hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* --- State machine -------------------------------------------------------- */
static AppState current_state = STATE_IDLE;

/* --- Global flags --------------------------------------------------------- */
uint8_t usb_flag = 0;

/* --- IMU working buffers (written only in main loop) --------------------- */
static IMU_Data accelerometer_data;
static IMU_Data gyroscope_data;
uint8_t raw_accelerometer[6] = {0};
uint8_t raw_gyroscope[6]     = {0};

/* --- Light sensor --------------------------------------------------------- */
static AS7341_Spectrum spectrum;
uint8_t raw_light[22] = {0};

/*
 * g_light_tick      — incremented in the ISR (volatile, 8-bit wraps freely).
 * g_light_tick_last — last value consumed by main loop (non-volatile copy).
 */
static volatile uint8_t g_light_tick      = 0U;
static          uint8_t g_light_tick_last = 0U;

/* Latest mains flicker classification (updated every 2 s in main loop) */
static volatile uint16_t g_mains_hz              = 0U;
static          uint32_t g_last_flicker_update_ms = 0U;

/* --- NAND Flash ----------------------------------------------------------- */
uint8_t  NAND_packet[4096] = {0};
uint16_t sample            = 0;
uint16_t blocco_scritto    = 0;
uint8_t  pagina_scritta    = 0;
uint16_t b                 = 0;
read_address_t   blocco;
column_address_t colonna   = 0;
uint16_t bad_blocks[2048]  = {-1};
uint8_t  bad_blocks2[2048] = {0};
uint8_t  data_letto[4096]  = {0};
int      exit_flag         = 0;

/* --- Timestamp ------------------------------------------------------------ */
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

/* ============================================================
 * USER CODE BEGIN 0
 *
 * TIM2 period-elapsed callback — the tripwire.
 *
 * CONTRACT (must never be violated):
 *   ✓  May set volatile flags
 *   ✓  May increment counters
 *   ✗  Must NOT call any HAL peripheral function
 *   ✗  Must NOT perform floating-point arithmetic
 *   ✗  Must NOT write to NAND / BLE / USB
 *
 * Duration target: < 1 µs  (verified: 2 instructions + return)
 * ============================================================ */
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2) return;
    if (current_state != STATE_ACQUISITION) return;

    g_imu_fetch_flag = 1U;
    g_light_tick++;
}
/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

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

  if (AS7341_Init() != 1) {
    for (uint8_t i = 0; i < 3; i++) {
      LED_Toggle(LED_RED); HAL_Delay(150);
      LED_Toggle(LED_RED); HAL_Delay(150);
    }
  }

  LightMetrics_Reset();
  IMU_RingBuffer_Init(&g_imu_ring_buffer);

  /*
   * Set NVIC priorities BEFORE starting the timer.
   * BLE (USART3) and USB (OTG_FS) at priority 5 so they preempt TIM2
   * and the main loop's blocking I2C calls (thread priority ~15).
   */
  HAL_NVIC_SetPriority(USART3_IRQn,  5, 0);
  HAL_NVIC_SetPriority(OTG_FS_IRQn,  5, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn,    6, 0);
  HAL_NVIC_SetPriority(EXTI0_IRQn,   6, 0);
  HAL_NVIC_SetPriority(EXTI4_IRQn,   6, 0);
  HAL_NVIC_SetPriority(EXTI5_IRQn,   6, 0);
  HAL_NVIC_SetPriority(EXTI10_IRQn,  6, 0);
  HAL_NVIC_SetPriority(EXTI13_IRQn,  6, 0);

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
        if (usb_flag) {
          current_state = STATE_USB_CONNECTED;
          LED_On(LED_GREEN);
        }
        break;

      /* ------------------------------------------------------------------ */
      case STATE_ACQUISITION:
      {
        /* ==============================================================
         * FETCH PATH — read IMU in thread context (NVIC-preemptible)
         * ============================================================== */
        uint8_t do_fetch;

        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        do_fetch         = g_imu_fetch_flag;
        g_imu_fetch_flag = 0U;
        if (!primask) __enable_irq();

        if (do_fetch)
        {
            IMU_ReadAccelerometerData(&accelerometer_data, raw_accelerometer);
            IMU_ReadGyroscopeData    (&gyroscope_data,     raw_gyroscope);

            IMU_RawData_t raw_sample;
            memcpy(raw_sample.acc,  raw_accelerometer, 6);
            memcpy(raw_sample.gyro, raw_gyroscope,     6);
            IMU_RingBuffer_Push(&g_imu_ring_buffer, &raw_sample);
        }

        /* ==============================================================
         * DRAIN PATH — one buffered sample per loop iteration
         * ============================================================== */
        IMU_RawData_t popped;
        if (IMU_RingBuffer_Pop(&g_imu_ring_buffer, &popped))
        {
            memcpy(raw_accelerometer, popped.acc,  6);
            memcpy(raw_gyroscope,     popped.gyro, 6);
            IMU_ReadAccelerometerData(&accelerometer_data, raw_accelerometer);
            IMU_ReadGyroscopeData    (&gyroscope_data,     raw_gyroscope);

            /* ----------------------------------------------------------
             * Light sensor — triggered every LIGHT_SUBSAMPLE ticks (10 Hz)
             * ---------------------------------------------------------- */
            uint8_t cur_tick = g_light_tick;
            if ((uint8_t)(cur_tick - g_light_tick_last) >= LIGHT_SUBSAMPLE)
            {
                g_light_tick_last = cur_tick;

                AS7341_ReadFullSpectrum(&spectrum);

                /*
                 * Pack spectral channels into raw_light[22]:
                 *   [0..15]  F1..F8  (ch[0]..ch[7], 8x2 bytes LE)
                 *   [16..17] Clear   (ch[10], 2nd SMUX pass, LE)
                 *   [18..19] NIR     (ch[11], 2nd SMUX pass, LE)
                 *   [20..21] mains_hz (uint16 LE)
                 *
                 * AS7341_Spectrum.ch[] layout (as7341_driver.h):
                 *   low  SMUX: ch[0]=F1, [1]=F2, [2]=F3, [3]=F4,
                 *              ch[4]=Clear, [5]=NIR
                 *   high SMUX: ch[6]=F5, [7]=F6, [8]=F7, [9]=F8,
                 *              ch[10]=Clear, [11]=NIR
                 */
                for (uint8_t i = 0; i < 8; i++) {
                    raw_light[i * 2]     = (uint8_t)(spectrum.ch[i] & 0xFF);
                    raw_light[i * 2 + 1] = (uint8_t)(spectrum.ch[i] >> 8);
                }
                raw_light[16] = (uint8_t)(spectrum.ch[10] & 0xFF);  /* Clear */
                raw_light[17] = (uint8_t)(spectrum.ch[10] >> 8);
                raw_light[18] = (uint8_t)(spectrum.ch[11] & 0xFF);  /* NIR   */
                raw_light[19] = (uint8_t)(spectrum.ch[11] >> 8);
                raw_light[20] = (uint8_t)(g_mains_hz & 0xFF);
                raw_light[21] = (uint8_t)(g_mains_hz >> 8);

                /*
                 * LightMetrics_Update returns 1 once per 1-second window
                 * (every LIGHT_METRICS_WINDOW = 10 calls at 10 Hz).
                 * Only send the unified BLE packet when metrics are fresh.
                 *
                 * BLE_UnifiedPayload fields:
                 *   stepCount          — stubbed 0 (no step driver yet)
                 *   cadence            — stubbed 0
                 *   activityState      — stubbed ACTIVITY_IDLE
                 *   uvRisk             — LightMetrics_GetUvRisk()          (Q15)
                 *   blueLightIntensity — LightMetrics_GetBlueIndex()
                 *   blueLightRatio     — LightMetrics_GetBlueFracQ15()     (Q15)
                 *   sunLikeIndex       — LightMetrics_GetSunLikeIndexQ15() (Q15)
                 *   metric1_clear      — spectrum.ch[10] (Clear, 2nd SMUX)
                 */
                if (LightMetrics_Update(&spectrum, &timestamp, g_mains_hz))
                {
                    BLE_UnifiedPayload ble_payload;
                    ble_payload.stepCount           = 0U;
                    ble_payload.cadence             = 0U;
                    ble_payload.activityState       = ACTIVITY_IDLE;
                    ble_payload.uvRisk              = (uint16_t)LightMetrics_GetUvRisk();
                    ble_payload.blueLightIntensity  = LightMetrics_GetBlueIndex();
                    ble_payload.blueLightRatio      = LightMetrics_GetBlueFracQ15();
                    ble_payload.sunLikeIndex        = LightMetrics_GetSunLikeIndexQ15();
                    ble_payload.metric1_clear       = spectrum.ch[10];
                    BLE_SendUnifiedPacket(&ble_payload);
                }
            }

            /* ----------------------------------------------------------
             * NAND Flash write
             * write_packet(uint16_t sample, Time_Struct ts,
             *              uint8_t *accel, uint8_t *gyro,
             *              uint8_t *light_raw, uint8_t *NAND_packet)
             * write_memory(void)
             * ---------------------------------------------------------- */
            write_packet(sample, timestamp,
                         raw_accelerometer,
                         raw_gyroscope,
                         raw_light,
                         NAND_packet);
            write_memory();

            if (IMU_RingBuffer_OverflowCount(&g_imu_ring_buffer) > 0) {
                LED_On(LED_RED);
            }
        }

        /* ==============================================================
         * Mains flicker classification — every 2 s
         * ============================================================== */
        if ((HAL_GetTick() - g_last_flicker_update_ms) >= FLICKER_UPDATE_PERIOD_MS)
        {
            g_mains_hz = AS7341_DetectMainsHz();
            g_last_flicker_update_ms = HAL_GetTick();
        }

        break;
      }

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

/* ============================================================
 * Peripheral initialisation — unchanged from CubeMX output
 * ============================================================ */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK) Error_Handler();

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST      = RCC_PLLMBOOST_DIV2;
  RCC_OscInitStruct.PLL.PLLM            = 2;
  RCC_OscInitStruct.PLL.PLLN            = 12;
  RCC_OscInitStruct.PLL.PLLR            = 2;
  RCC_OscInitStruct.PLL.PLLP            = 2;
  RCC_OscInitStruct.PLL.PLLQ            = 3;
  RCC_OscInitStruct.PLL.PLLRGE         = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN       = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                   | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C3_Init(void)
{
  hi2c3.Instance                    = I2C3;
  hi2c3.Init.Timing                 = 0x10808DD3;
  hi2c3.Init.OwnAddress1            = 0;
  hi2c3.Init.AddressingMode         = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode        = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2            = 0;
  hi2c3.Init.OwnAddress2Masks       = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode        = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode          = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) Error_Handler();
}

static void MX_ICACHE_Init(void) { /* CubeMX defaults */ }

static void MX_MDF1_Init(void)
{
  MdfHandle0.Instance                                         = MDF1_Filter0;
  MdfHandle0.Init.CommonParam.InterleavedFilters              = 0;
  MdfHandle0.Init.CommonParam.ProcClockDivider                = 1;
  MdfHandle0.Init.CommonParam.OutputClock.Activation          = ENABLE;
  MdfHandle0.Init.CommonParam.OutputClock.Pins                = MDF_OUTPUT_CLOCK_ALL;
  MdfHandle0.Init.CommonParam.OutputClock.Divider             = 5;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Activation  = ENABLE;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Source      = MDF_CLOCK_TRIG_TRGO;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Edge        = MDF_CLOCK_TRIG_FALLING_EDGE;
  MdfHandle0.Init.SerialInterface.Activation                  = ENABLE;
  MdfHandle0.Init.SerialInterface.Mode                        = MDF_SITF_NORMAL_SPI_MODE;
  MdfHandle0.Init.SerialInterface.ClockSource                 = MDF_SITF_CCK0_SOURCE;
  MdfHandle0.Init.SerialInterface.Threshold                   = 31;
  MdfHandle0.Init.FilterBistream                              = MDF_BITSTREAM0_RISING;
  if (HAL_MDF_Init(&MdfHandle0) != HAL_OK) Error_Handler();

  MdfFilterConfig0.DataSource            = MDF_DATA_SOURCE_BSMX;
  MdfFilterConfig0.Delay                 = 0;
  MdfFilterConfig0.CicMode               = MDF_ONE_FILTER_SINC5;
  MdfFilterConfig0.DecimationRatio       = 16;
  MdfFilterConfig0.Offset                = 0;
  MdfFilterConfig0.Gain                  = 1;
  MdfFilterConfig0.ReshapeFilter.Activation     = ENABLE;
  MdfFilterConfig0.ReshapeFilter.DecimationRatio = MDF_RSF_DECIMATION_RATIO_4;
  MdfFilterConfig0.HighPassFilter.Activation    = ENABLE;
  MdfFilterConfig0.HighPassFilter.CutOffFrequency = MDF_HPF_CUTOFF_0_000625FPCM;
  MdfFilterConfig0.Integrator.Activation         = DISABLE;
  MdfFilterConfig0.SoundActivity.Activation      = DISABLE;
  MdfFilterConfig0.AcquisitionMode               = MDF_MODE_SYNC_CONT;
  MdfFilterConfig0.FifoThreshold                 = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  MdfFilterConfig0.DiscardSamples                = 255;
  MdfFilterConfig0.Trigger.Source                = MDF_CLOCK_TRIG_TRGO;
  MdfFilterConfig0.Trigger.Edge                  = MDF_FILTER_TRIG_RISING_EDGE;
}

static void MX_SPI2_Init(void)
{
  SPI_AutonomousModeConfTypeDef cfg = {0};
  hspi2.Instance               = SPI2;
  hspi2.Init.Mode              = SPI_MODE_MASTER;
  hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi2.Init.NSS               = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial     = 0x7;
  hspi2.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness  = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap                  = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement   = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity           = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK) Error_Handler();
  cfg.TriggerState     = SPI_AUTO_MODE_DISABLE;
  cfg.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  cfg.TriggerPolarity  = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &cfg) != HAL_OK) Error_Handler();
}

static void MX_SPI3_Init(void)
{
  SPI_AutonomousModeConfTypeDef cfg = {0};
  hspi3.Instance               = SPI3;
  hspi3.Init.Mode              = SPI_MODE_MASTER;
  hspi3.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi3.Init.NSS               = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial     = 0x7;
  hspi3.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.MasterSSIdleness  = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap                  = SPI_IO_SWAP_DISABLE;
  hspi3.Init.ReadyMasterManagement   = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi3.Init.ReadyPolarity           = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi3) != HAL_OK) Error_Handler();
  cfg.TriggerState     = SPI_AUTO_MODE_DISABLE;
  cfg.TriggerSelection = SPI_GRP2_LPDMA_CH0_TCF_TRG;
  cfg.TriggerPolarity  = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3, &cfg) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig      = {0};
  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 7200 - 1;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 99;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance            = USART3;
  huart3.Init.BaudRate       = 115200;
  huart3.Init.WordLength     = UART_WORDLENGTH_8B;
  huart3.Init.StopBits       = UART_STOPBITS_1;
  huart3.Init.Parity         = UART_PARITY_NONE;
  huart3.Init.Mode           = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling   = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
}

static void MX_USB_OTG_FS_PCD_Init(void)
{
  hpcd_USB_OTG_FS.Instance                = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints      = 6;
  hpcd_USB_OTG_FS.Init.speed              = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface         = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable         = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable   = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable         = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1  = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* --- Output pins: LEDs and NAND CS ------------------------------------ */
  HAL_GPIO_WritePin(MCU_GREEN_LED_GPIO_Port, MCU_GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MCU_RED_LED_GPIO_Port,   MCU_RED_LED_Pin,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SPI3_CS_NAND_GPIO_Port,  SPI3_CS_NAND_Pin,  GPIO_PIN_SET);

  /* Green LED */
  GPIO_InitStruct.Pin   = MCU_GREEN_LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_GREEN_LED_GPIO_Port, &GPIO_InitStruct);

  /* Red LED */
  GPIO_InitStruct.Pin   = MCU_RED_LED_Pin;
  HAL_GPIO_Init(MCU_RED_LED_GPIO_Port, &GPIO_InitStruct);

  /* NAND CS */
  GPIO_InitStruct.Pin   = SPI3_CS_NAND_Pin;
  HAL_GPIO_Init(SPI3_CS_NAND_GPIO_Port, &GPIO_InitStruct);

  /* User button */
  GPIO_InitStruct.Pin  = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* IMU interrupt pins */
  GPIO_InitStruct.Pin  = IMU_IS_INT1_Pin;
  HAL_GPIO_Init(IMU_IS_INT1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = IMU_IS_INT2_Pin;
  HAL_GPIO_Init(IMU_IS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* Auxiliary I/O pins */
  GPIO_InitStruct.Pin  = MCU_I_O_1_Pin;
  HAL_GPIO_Init(MCU_I_O_1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = MCU_I_O_2_Pin;
  HAL_GPIO_Init(MCU_I_O_2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
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

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
