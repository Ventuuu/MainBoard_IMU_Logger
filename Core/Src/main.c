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
 *    via I2C at 100 Hz (TIM2), and computes a final AS7341 session-level
 *    light result outside interrupt context.
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
#include "light_metrics_mcu.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

//--- Microphone acquisition variables ---
#define AUDIO_BUFFER_SIZE 1024U

int16_t audio_buffer[AUDIO_BUFFER_SIZE];
MDF_DmaConfigTypeDef mic_dma_config;

static volatile uint8_t microphone_active = 0U;
static volatile uint8_t audio_buffer_ready = 0U;

// --- State Machine ---
static volatile AppState current_state = STATE_IDLE;
static uint32_t state_led_last_toggle_ms = 0U;
static AppState previous_state_led = STATE_IDLE;
static uint8_t state_led_initialized = 0U;

// --- Global Flags ---
volatile uint8_t usb_flag = 0U;
static volatile uint8_t start_acquisition_requested = 0U;
static volatile uint8_t stop_acquisition_requested = 0U;
static volatile uint32_t sensor_tick_pending = 0U;
static volatile uint8_t light_finalize_pending = 0U;

// --- IMU data ---
static IMU_Data accelerometer_data;
static IMU_Data gyroscope_data;

uint8_t raw_accelerometer[6] = {0};
uint8_t raw_gyroscope[6]     = {0};

/*
 * raw_light layout (22 bytes):
 *   Legacy AS7341 area in the 40-byte IMU record. The new light pipeline keeps
 *   this area zero-filled and writes one final LOG_MAGIC_LIGHT page instead.
 */
uint8_t raw_light[22] = {0};
static LightSensorResultRecord light_result;

/// ----- NAND FLASH variables ----- ///

static NandLogger nand_logger;
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
static void UpdateStateLed(AppState state);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t Time_ToMilliseconds(Time_Struct t)
{
    return ((uint32_t)t.hh * 3600000UL) +
           ((uint32_t)t.mm * 60000UL) +
           ((uint32_t)t.ss * 1000UL) +
           ((uint32_t)t.sss);
}

static void UpdateStateLed(AppState state)
{
    uint32_t now = HAL_GetTick();
    uint32_t blink_interval_ms = 0U;

    if ((state_led_initialized == 0U) || (state != previous_state_led))
    {
        state_led_initialized = 1U;
        previous_state_led = state;
        state_led_last_toggle_ms = now;

        switch (state)
        {
            case STATE_IDLE:
                LED_Off(LED_GREEN);
                return;

            case STATE_ACQUISITION:
                LED_On(LED_GREEN);
                return;

            case STATE_USB_CONNECTED:
            case STATE_DOWNLOAD:
                LED_On(LED_GREEN);
                return;

            default:
                LED_Off(LED_GREEN);
                return;
        }
    }

    switch (state)
    {
        case STATE_IDLE:
            LED_Off(LED_GREEN);
            break;

        case STATE_ACQUISITION:
            LED_On(LED_GREEN);
            break;

        case STATE_USB_CONNECTED:
            blink_interval_ms = 500U;
            break;

        case STATE_DOWNLOAD:
            blink_interval_ms = 125U;
            break;

        default:
            LED_Off(LED_GREEN);
            break;
    }

    if ((blink_interval_ms != 0U) &&
        ((now - state_led_last_toggle_ms) >= blink_interval_ms))
    {
        state_led_last_toggle_ms = now;
        LED_Toggle(LED_GREEN);
    }
}

void App_UpdateDownloadLed(void)
{
    UpdateStateLed(STATE_DOWNLOAD);
}

//--michrophone acquisition complete callback: set flag and stop acquisition to prevent overwriting buffer before processing ----//
void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf)
{
    if (hmdf != &MdfHandle0)
    {
        return;
    }

    microphone_active = 0U;

    if (current_state == STATE_ACQUISITION)
    {
        audio_buffer_ready = 1U;
    }
    else
    {
        audio_buffer_ready = 0U;
    }
}
/* USER CODE END 0 */
static void StopAcquisition(void)
{
    uint32_t stop_ms = HAL_GetTick();

    HAL_TIM_Base_Stop_IT(&htim2);

    if (microphone_active)
    {
        HAL_MDF_AcqStop_DMA(&MdfHandle0);
        microphone_active = 0U;
    }

    audio_buffer_ready = 0U;
    sensor_tick_pending = 0U;
    stop_acquisition_requested = 0U;
    LightMetrics_StopSession(stop_ms);
    light_finalize_pending = 1U;

    current_state = STATE_IDLE;
    UpdateStateLed(current_state);
}

static void ProcessSensorTick(void)
{
    /* --- Read IMU --- */
    IMU_ReadAccelerometerData(&accelerometer_data, raw_accelerometer);
    IMU_ReadGyroscopeData(&gyroscope_data, raw_gyroscope);

    LightMetrics_RequestSample();

    /* --- BLE transmission --- */
    BLE_SendPacket(DATA_TYPE_IMU_ACCELERATION, raw_accelerometer);
    BLE_SendPacket(DATA_TYPE_IMU_GYROSCOPE, raw_gyroscope);

    /* --- Timestamp @ 100 Hz --- */
    timestamp.sss = tim * 10U;

    if (timestamp.sss == 1000U)
    {
        timestamp.ss++;
        timestamp.sss = 0U;
        tim = 0U;

        if (timestamp.ss == 60U)
        {
            timestamp.mm++;
            timestamp.ss = 0U;

            if (timestamp.mm == 60U)
            {
                timestamp.hh++;
                timestamp.mm = 0U;
            }
        }
    }

    tim++;

    /* --- NAND sensor logging --- */
    if (NANDLogger_AppendSensorRecord(&nand_logger,
                                      timestamp,
                                      raw_accelerometer,
                                      raw_gyroscope,
                                      raw_light) != LOG_OK)
    {
        StopAcquisition();
        LED_On(LED_RED);
    }
}



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

LED_On(LED_RED);
HAL_Delay(3000);

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
  if (NANDLogger_Init(&nand_logger) != LOG_OK) {
    Error_Handler();
}


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

  /* Configure AS7341 timing/gain and reset session accumulators. */
  LightMetrics_Init();

  LED_Off(LED_RED);

  /* USER CODE END 2 */
  mic_dma_config.Address    = (uint32_t)audio_buffer;
  mic_dma_config.DataLength = AUDIO_BUFFER_SIZE * sizeof(int16_t);
  mic_dma_config.MsbOnly    = ENABLE;
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    UpdateStateLed(current_state);

	  switch(current_state)
	  {
      case STATE_IDLE:

        if (light_finalize_pending)
        {
          light_finalize_pending = 0U;
          LightMetrics_FinalizeSession(&light_result);

          if (NANDLogger_AppendLightResult(&nand_logger,
                                           &light_result,
                                           HAL_GetTick()) != LOG_OK)
          {
            LED_On(LED_RED);
          }

          LightMetrics_ResetSession();
        }

        if (start_acquisition_requested)
        {
          start_acquisition_requested = 0U;

          if (NANDLogger_EraseAllGoodBlocks(&nand_logger) != LOG_OK)
          {
            Error_Handler();
          }
          timestamp.hh = 0U;
          timestamp.mm = 0U;
          timestamp.ss = 0U;
          timestamp.sss = 0U;

          tim = 0U;
          sensor_tick_pending = 0U;
          memset(raw_light, 0, sizeof(raw_light));
          LightMetrics_StartSession(HAL_GetTick());

          audio_buffer_ready = 0U;
          microphone_active = 0U;
          stop_acquisition_requested = 0U;
          current_state = STATE_ACQUISITION;
          UpdateStateLed(current_state);
          HAL_TIM_Base_Start_IT(&htim2);

          break;
        }

        HAL_TIM_Base_Stop_IT(&htim2);

        if (microphone_active)
        {
        HAL_MDF_AcqStop_DMA(&MdfHandle0);
        microphone_active = 0U;
        }

        audio_buffer_ready = 0U;

        if (usb_flag)
        {
          current_state = STATE_USB_CONNECTED;
        }

        break;

        case STATE_ACQUISITION:

          if (stop_acquisition_requested)
          {
              stop_acquisition_requested = 0U;
              StopAcquisition();
              break;
          }
          
          while ((sensor_tick_pending > 0U) &&
                  (current_state == STATE_ACQUISITION) &&
                  (stop_acquisition_requested == 0U))
          {
            sensor_tick_pending--;
             ProcessSensorTick();
          }

          if ((current_state == STATE_ACQUISITION) &&
              (stop_acquisition_requested == 0U))
          {
            LightMetrics_ProcessPendingSample();
          }

          if (stop_acquisition_requested)
          {
            stop_acquisition_requested = 0U;
            StopAcquisition();
            break;
          }  
        
          if (audio_buffer_ready && current_state == STATE_ACQUISITION)
          {
            audio_buffer_ready = 0U;

            if (NANDLogger_AppendAudioBuffer(&nand_logger,
                                 audio_buffer,
                                 AUDIO_BUFFER_SIZE,
                                 Time_ToMilliseconds(timestamp)) != LOG_OK)
            {
              StopAcquisition();
              LED_On(LED_RED);
            }
          }
          else if (!microphone_active && current_state == STATE_ACQUISITION)
          {
            if (HAL_MDF_AcqStart_DMA(&MdfHandle0,
                                 &MdfFilterConfig0,
                                 &mic_dma_config) != HAL_OK)
            {
              Error_Handler();
            }

            microphone_active = 1U;
          }

          break;

	  	  case STATE_USB_CONNECTED:
	  		 break;

	  	  case STATE_DOWNLOAD:
	  		  if (NANDLogger_DownloadAll(&nand_logger) != LOG_OK) 
          { 
          Error_Handler();
          }
			 current_state = STATE_USB_CONNECTED;
	  		 break;
    }

  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  TIM2 period elapsed callback — 100 Hz sensor scheduler.
  *
  * Interrupt context only queues work. IMU reads, AS7341 I2C accesses,
  * processing, NAND writes and USB transfers are handled in the main loop.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != &htim2)
    {
        return;
    }

    if (current_state != STATE_ACQUISITION)
    {
        return;
    }

    sensor_tick_pending++;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == USER_BUTTON_Pin)
  {
		switch(current_state) 
    {
			case STATE_IDLE:
        start_acquisition_requested = 1U;
        break;
			case STATE_ACQUISITION:
        stop_acquisition_requested = 1U;
        break;
			case STATE_USB_CONNECTED:
				exit_flag = 0;
				current_state = STATE_DOWNLOAD;
			  break;
			default:
        current_state = STATE_IDLE;
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
  LED_Off(LED_GREEN);

  while (1)
  {
    LED_Toggle(LED_RED);
    HAL_Delay(200);
  }
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

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
  MdfHandle0.Init.CommonParam.OutputClock.Trigger.Activation = ENABLE;
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
  MdfFilterConfig0.AcquisitionMode = MDF_MODE_SYNC_CONT;
  MdfFilterConfig0.FifoThreshold = MDF_FIFO_THRESHOLD_NOT_EMPTY;
  MdfFilterConfig0.DiscardSamples = 255;
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
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


