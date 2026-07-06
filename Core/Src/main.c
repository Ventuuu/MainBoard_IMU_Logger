/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main application — Alternating Phased Acquisition Architecture
  ******************************************************************************
  * @details
  *
  * This firmware implements a strict 2.5-second alternating duty cycle 
  * designed for ultra-low power consumption and accurate telemetry gathering.
  *
  * State Machine (`STATE_ACQUISITION`)
  * ---------------------------------------------------------------------------
  * The main loop toggles between two primary hardware states:
  *
  * PHASE 1: Environment Phase (0ms - 500ms)
  *   • IMU is powered down (ODR = 0).
  *   • Light Sensor (AS7341) is driven asynchronously and continuously 
  *     accumulates readings.
  *   • Microphone (MDF) continuously loops 64ms DMA transfers to compute LAeq.
  *   • At exactly 500ms: Environment metrics are finalized (averaged),
  *     and sensors are sent back to sleep.
  *
  * PHASE 2: IMU Phase (500ms - 2500ms)
  *   • Light & Mic DMA pipelines are halted entirely.
  *   • IMU wakes up (ODR = 52Hz) and accumulates raw steps natively via 
  *     the hardware EXTI line.
  *   • The step-counter processes all buffered data concurrently.
  *   • At 2500ms: The cycle resets back to PHASE 1.
  *
  * BLE Packet Dispatch
  * ---------------------------------------------------------------------------
  * A 20-byte unified BLE payload is dispatched exactly every 500ms:
  *   - 1x at the end of PHASE 1.
  *   - 4x distributed perfectly across PHASE 2.
  * The payload safely stitches together the freshly advancing Step Count 
  * and the statically stashed Environment readings until the next Env phase.
  *
  * Recent Modifications
  * ---------------------------------------------------------------------------
  * - IMU Step Counter inputs updated: Port 1 (gravity) receives 'g', 
  *   Port 2 (dynamic force) receives 'm/s^2' (scaled by 9.81f).
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "main.h"
#include "audio_ring_buffer.h"

/* Private includes ----------------------------------------------------------*/
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
#include "StepCounter.h"
#include "bluetooth.h"
#include "as7341_driver.h"
#include "as7341_processing_config.h"
#include "light_metrics_mcu.h"
#include "mic_metrics.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/** Light sensor sampled every LIGHT_SUBSAMPLE IMU ticks (100 Hz / 10 = 10 Hz) */
#define LIGHT_SUBSAMPLE           10U

#define AUDIO_HARDWARE_WARMUP_SAMPLES 255U
#define AUDIO_SOFTWARE_WARMUP_SAMPLES \
    (AUDIO_WARMUP_SAMPLES - AUDIO_HARDWARE_WARMUP_SAMPLES)
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

/* --- State machine -------------------------------------------------------- */
AppState current_state = STATE_ACQUISITION;

/* --- Global flags --------------------------------------------------------- */
uint8_t usb_flag = 0;

/* --- IMU working buffers (written only in main loop) --------------------- */
static IMU_Data accelerometer_data;
static IMU_Data gyroscope_data;
uint8_t raw_accelerometer[6] = {0};
uint8_t raw_gyroscope[6]     = {0};

/*
 * g_light_tick      — incremented in the ISR (volatile, 8-bit wraps freely).
 * g_light_tick_last — last value consumed by main loop (non-volatile copy).
 */
static volatile uint8_t g_light_tick      = 0U;

// --- Microphone Variables ---
#define AUDIO_DMA_BUFFER_SAMPLES (2 * AUDIO_CHUNK_SAMPLES)
int16_t audio_dma_buffer[AUDIO_DMA_BUFFER_SAMPLES];
AudioRingBuffer g_audio_ring_buffer;
uint8_t microphone_active = 0;
MDF_DmaConfigTypeDef mic_dma_config;

volatile uint16_t g_last_laeq_x10 = 0;
volatile uint8_t  g_last_audio_env = 0;

static uint8_t    g_last_light_class = 0;
static uint16_t   g_last_blue_clear_ratio = 0;
static uint16_t   g_last_color_temp = 0;

typedef enum {
    PHASE_ENV = 0,
    PHASE_IMU = 1
} AcqPhase;

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //-> no longer needed since we're using the timer purely as a tripwire to set a flag, and not for any timing-sensitive operations. The main loop can check the flag and perform the necessary I2C reads and processing without being constrained by the ISR execution time.
{
    if (htim != &htim2) return;
    if (current_state != STATE_ACQUISITION) return;

    g_imu_fetch_flag = 1U;
    g_light_tick++;
}
static void MicrophoneClock_Enable(void) {
    MDF1->CKGCR |= MDF_CKGCR_CKDEN;
    MdfHandle0.Instance->SITFCR |= MDF_SITFCR_SITFEN;
}

static void MicrophoneClock_Disable(void) {
    MDF1->CKGCR &= ~MDF_CKGCR_CKDEN;
}

void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *hmdf) {
    if (hmdf->Instance == MDF1_Filter0) {
        AudioRing_EnqueueFromIsr(&g_audio_ring_buffer, &audio_dma_buffer[0], AUDIO_CHUNK_SAMPLES);
    }
}

void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf) {
    if (hmdf->Instance == MDF1_Filter0) {
        AudioRing_EnqueueFromIsr(&g_audio_ring_buffer, &audio_dma_buffer[AUDIO_CHUNK_SAMPLES], AUDIO_CHUNK_SAMPLES);
    }
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
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

  BLE_Initialize();
  MX_USB_Device_Init();
  HAL_Delay(1000);

  spi_nand_init();
  find_bad_blocks(bad_blocks);

  if (IMU_Init() == 1) {
    IMU_ConfigAccelerometer(ACC_ODR_52HZ, ACC_FS_2G, 1);
    IMU_ConfigGyroscope    (GYR_ODR_52HZ, GYR_FS_250DPS, 1);}
   else {
    for (uint8_t i = 0; i < 6; i++) {
      LED_Toggle(LED_RED); HAL_Delay(500);
    }
  }

  if (AS7341_Init() != 1) {
    for (uint8_t i = 0; i < 3; i++) {
      LED_Toggle(LED_RED); HAL_Delay(150);
      }
  }

  LightMetrics_Init();
  LightMetrics_StartSession(HAL_GetTick());
  StepCounter_initialize();
  IMU_RingBuffer_Init(&g_imu_ring_buffer);

  /*
   * Set NVIC priorities BEFORE starting the timer.
   * BLE (USART3) and USB (OTG_FS) at priority 5 so they preempt TIM2
   * and the main loop's blocking I2C calls (thread priority ~15).
   */
  HAL_NVIC_SetPriority(USART3_IRQn,  5, 0);
  HAL_NVIC_SetPriority(OTG_FS_IRQn,  5, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn,    6, 0);

  HAL_TIM_Base_Start_IT (&htim2);

  LED_Off(LED_RED);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t last_blink = 0;
    if (HAL_GetTick() - last_blink > 500) {
        last_blink = HAL_GetTick();
    }
    
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
        uint32_t current_tick = HAL_GetTick();
        
        static AcqPhase current_phase = PHASE_ENV;
        static uint32_t phase_start_ms = 0;
        static uint8_t  phase_just_entered = 1;
        static uint8_t  light_was_pending = 0;

        if (phase_start_ms == 0) {
             phase_start_ms = current_tick;
        }

        if (current_phase == PHASE_ENV) 
        {
            static uint32_t laeq_sum = 0;
            static uint16_t laeq_count = 0;

            if (phase_just_entered) 
            {
                phase_just_entered = 0;
                laeq_sum = 0;
                laeq_count = 0;
                
                // 1. Turn IMU OFF
                IMU_ConfigAccelerometer(ACC_ODR_POWER_DOWN, ACC_FS_2G, 1);
                IMU_ConfigGyroscope(GYR_ODR_POWER_DOWN, GYR_FS_250DPS, 1);
                
                // 2. Start Light Sensor (Async)
                LightMetrics_StartSession(current_tick);
                LightMetrics_RequestSample();
                light_was_pending = 1;
                
                // 3. Start Mic DMA with Ring Buffer
                AudioRing_Reset(&g_audio_ring_buffer);
                MicrophoneClock_Enable();
                
                mic_dma_config.Address    = (uint32_t)audio_dma_buffer;
                mic_dma_config.DataLength = AUDIO_DMA_BUFFER_SAMPLES * sizeof(int16_t);
                mic_dma_config.MsbOnly    = ENABLE;
                
                HAL_MDF_AcqStart_DMA(&MdfHandle0, &MdfFilterConfig0, &mic_dma_config);
                microphone_active = 1;
            }
            
            // --- Continuously Drive Light Sensor ---
            LightMetrics_ProcessPendingSample();
            if (light_was_pending && LightMetrics_HasPendingSample() == 0) 
            {
                // Request the next sample immediately to keep accumulating
                LightMetrics_RequestSample();
                light_was_pending = 1;
            }
            
            // --- Drain Audio Ring Buffer ---
            while (AudioRing_Count(&g_audio_ring_buffer) >= AUDIO_CHUNK_SAMPLES)
            {
                static int16_t chunk[AUDIO_CHUNK_SAMPLES];
                uint32_t popped = AudioRing_PopChunk(&g_audio_ring_buffer, chunk, AUDIO_CHUNK_SAMPLES);
                
                if (popped > 0) {
                    uint16_t calc_laeq;
                    uint8_t calc_env;
                    MicMetrics_ProcessFrame(chunk, popped, &calc_laeq, &calc_env);
                    
                    laeq_sum += calc_laeq;
                    laeq_count++;
                    g_last_audio_env = calc_env;
                }
            }
            
            // --- Check for Phase Transition (500ms) ---
            if (current_tick - phase_start_ms >= 500) 
            {
                LightSensorResultRecord light_res;
                LightMetrics_FinalizeSession(&light_res);
                
                g_last_blue_clear_ratio = light_res.blue_clear_ratio;
                g_last_color_temp       = light_res.color_temp;
                g_last_light_class      = light_res.light_level_class;
                
                // Stop Mic Recording safely
                if (microphone_active) {
                    HAL_MDF_AcqStop_DMA(&MdfHandle0);
                    MicrophoneClock_Disable();
                    microphone_active = 0;
                }
                
                // Compute Audio Average
                if (laeq_count > 0) {
                    g_last_laeq_x10 = (uint16_t)(laeq_sum / laeq_count);
                }
                
                // --- Dispatch 500ms BLE Packet ---
                BLE_UnifiedPayload ble_payload;
                ble_payload.stepCount          = StepCounter_Y.stepnumber;
                ble_payload.light_level_class  = g_last_light_class;
                ble_payload.blue_clear_ratio   = g_last_blue_clear_ratio;
                ble_payload.color_temp         = g_last_color_temp;
                ble_payload.laeq_x10           = g_last_laeq_x10;
                ble_payload.audio_env_class    = g_last_audio_env;
                
                BLE_SendUnifiedPacket(&ble_payload);
                
                current_phase = PHASE_IMU;
                phase_start_ms = current_tick;
                phase_just_entered = 1;
            }
        }
        else if (current_phase == PHASE_IMU)
        {
            static uint32_t imu_ble_timer = 0;
            if (phase_just_entered) 
            {
                phase_just_entered = 0;
                imu_ble_timer = phase_start_ms;
                
                // Turn IMU ON (104 Hz matches Simulink's 0.01s / 100Hz expectation)
                IMU_ConfigAccelerometer(ACC_ODR_104HZ, ACC_FS_2G, 1);
                IMU_ConfigGyroscope(GYR_ODR_104HZ, GYR_FS_250DPS, 1);
            }
            
            // --- Process IMU Data (Interrupts will fire and push to Ring Buffer) ---
            uint8_t do_fetch = g_imu_fetch_flag;
            g_imu_fetch_flag = 0U;

            if (do_fetch)
            {
                IMU_ReadAccelerometerRaw(raw_accelerometer);
                IMU_ReadGyroscopeRaw(raw_gyroscope);

                IMU_RawData_t raw_sample;
                memcpy(raw_sample.acc,  raw_accelerometer, 6);
                memcpy(raw_sample.gyro, raw_gyroscope,     6);
                IMU_RingBuffer_Push(&g_imu_ring_buffer, &raw_sample);
            }

            IMU_RawData_t popped;
            while (IMU_RingBuffer_Pop(&g_imu_ring_buffer, &popped))
            {
                IMU_ConvertAccelRawToFloat(&accelerometer_data, popped.acc);
                IMU_ConvertGyroRawToFloat(&gyroscope_data,     popped.gyro);
                
                StepCounter_U.In1[0] = gyroscope_data.x;
                StepCounter_U.In1[1] = gyroscope_data.y;
                StepCounter_U.In1[2] = gyroscope_data.z;
                StepCounter_U.In2[0] = accelerometer_data.x;
                StepCounter_U.In2[1] = accelerometer_data.y;
                StepCounter_U.In2[2] = accelerometer_data.z;
                StepCounter_step();
            }
            
            // --- 500ms BLE Dispatch during IMU Phase ---
            if (current_tick - imu_ble_timer >= 500)
            {
                imu_ble_timer = current_tick;
                
                BLE_UnifiedPayload ble_payload;
                ble_payload.stepCount          = StepCounter_Y.stepnumber;
                ble_payload.light_level_class  = g_last_light_class;
                ble_payload.blue_clear_ratio   = g_last_blue_clear_ratio;
                ble_payload.color_temp         = g_last_color_temp;
                ble_payload.laeq_x10           = g_last_laeq_x10;
                ble_payload.audio_env_class    = g_last_audio_env;
                
                BLE_SendUnifiedPacket(&ble_payload);
            }
            
            // --- Check for Phase Transition (2000ms) ---
            if (current_tick - phase_start_ms >= 2000) 
            {
                current_phase = PHASE_ENV;
                phase_start_ms = current_tick;
                phase_just_entered = 1;
            }
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

    /* Enter SLEEP mode to save power. Wake up on next interrupt (e.g. SysTick, DMA, I2C) */
    __WFI();

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
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
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10808DD3;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief MDF1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_MDF1_Init(void)
{

  /* USER CODE BEGIN MDF1_Init 0 */

  /* USER CODE END MDF1_Init 0 */

  /* USER CODE BEGIN MDF1_Init 1 */

  /* USER CODE END MDF1_Init 1 */

  /**
    MdfHandle0 structure initialization and HAL_MDF_Init function call
  */
  MdfHandle0.Instance = MDF1_Filter0;
  MdfHandle0.Init.CommonParam.InterleavedFilters = 0;
  MdfHandle0.Init.CommonParam.ProcClockDivider = 1;
  MdfHandle0.Init.CommonParam.OutputClock.Activation = ENABLE;
  MdfHandle0.Init.CommonParam.OutputClock.Pins = MDF_OUTPUT_CLOCK_ALL;
  MdfHandle0.Init.CommonParam.OutputClock.Divider = 5;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Activation = DISABLE;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Source = MDF_CLOCK_TRIG_TRGO;
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Edge = MDF_CLOCK_TRIG_FALLING_EDGE;
  MdfHandle0.Init.SerialInterface.Activation = ENABLE;
  MdfHandle0.Init.SerialInterface.Mode = MDF_SITF_NORMAL_SPI_MODE;
  MdfHandle0.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
  MdfHandle0.Init.SerialInterface.Threshold = 31;
  MdfHandle0.Init.FilterBistream = MDF_BITSTREAM0_RISING;
  if (HAL_MDF_Init(&MdfHandle0) != HAL_OK)
  {
    Error_Handler();
  }

  /**
    MdfFilterConfig0, MdfOldConfig0 and/or MdfScdConfig0 structures initialization

    WARNING : only structures are filled, no specific init function call for filter
  */
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
  MdfFilterConfig0.AcquisitionMode = MDF_MODE_ASYNC_CONT;
  MdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  MdfFilterConfig0.DiscardSamples = AUDIO_HARDWARE_WARMUP_SAMPLES;
  MdfFilterConfig0.Trigger.Source = MDF_CLOCK_TRIG_TRGO;
  MdfFilterConfig0.Trigger.Edge = MDF_FILTER_TRIG_RISING_EDGE;
  /* USER CODE BEGIN MDF1_Init 2 */

  /* USER CODE END MDF1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
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
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi2, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  SPI_AutonomousModeConfTypeDef HAL_SPI_AutonomousMode_Cfg_Struct = {0};

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
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
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerState = SPI_AUTO_MODE_DISABLE;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerSelection = SPI_GRP2_LPDMA_CH0_TCF_TRG;
  HAL_SPI_AutonomousMode_Cfg_Struct.TriggerPolarity = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&hspi3, &HAL_SPI_AutonomousMode_Cfg_Struct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|BLE_P0_0_Pin|BLE_P3_6_Pin|BLE_UART_RX_IND_Pin
                          |BLE_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|SPI3_CS_NAND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CONFIG_GPIO_Port, BLE_CONFIG_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCU_GREEN_LED_Pin|MCU_RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IMU_IS_INT1_Pin */
  GPIO_InitStruct.Pin = IMU_IS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_IS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 BLE_P0_0_Pin BLE_P3_6_Pin BLE_UART_RX_IND_Pin
                           BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|BLE_P0_0_Pin|BLE_P3_6_Pin|BLE_UART_RX_IND_Pin
                          |BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_IS_INT2_Pin */
  GPIO_InitStruct.Pin = IMU_IS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_IS_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 SPI3_CS_NAND_Pin BLE_CONFIG_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|SPI3_CS_NAND_Pin|BLE_CONFIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_I_O_2_Pin MCU_I_O_1_Pin */
  GPIO_InitStruct.Pin = MCU_I_O_2_Pin|MCU_I_O_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MCU_GREEN_LED_Pin MCU_RED_LED_Pin */
  GPIO_InitStruct.Pin = MCU_GREEN_LED_Pin|MCU_RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI5_IRQn);

  HAL_NVIC_SetPriority(EXTI10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI10_IRQn);

  HAL_NVIC_SetPriority(EXTI13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
