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
 *    via I2C at 100 Hz (TIM2), and stores raw AS7341 samples as LRAW pages
 *    outside interrupt context.
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
#include <math.h>
#include <stdint.h>
#include "../../USB_Device/App/usb_device.h"
#include "SPI.h"
#include "SPI_NAND.h"
#include "Memory_operations.h"
#include "led_driver.h"
#include "imu_driver.h"
#include "bluetooth.h"
#include "ble_sync.h"
#include "as7341_driver.h"
#include "as7341_processing_config.h"

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

volatile uint32_t mcu_reset_csr_at_boot = 0U;

//--- Microphone acquisition variables ---
#define AUDIO_CHUNK_SAMPLES 1024U
#define AUDIO_DMA_BUFFER_SAMPLES (2U * AUDIO_CHUNK_SAMPLES)
#define AUDIO_RING_SLOT_COUNT 32U
#define AUDIO_WINDOW_TARGET_SAMPLES 24000U
#define AUDIO_WINDOW_REQUIRED_CHUNKS \
    ((AUDIO_WINDOW_TARGET_SAMPLES + AUDIO_CHUNK_SAMPLES - 1U) / AUDIO_CHUNK_SAMPLES)
#define AUDIO_WINDOW_PERIOD_MS 10000U
#define AUDIO_BASIC_FEATURE_HISTORY_CAPACITY 16U
#define AUDIO_SAMPLE_RATE_HZ 48000U
/* Approximate 1 kHz host calibration offset; this is not a certified SPL meter. */
#define AUDIO_SPL_CALIBRATION_OFFSET_DB 122.40
#ifndef AUDIO_STORE_RAW_PCM
#define AUDIO_STORE_RAW_PCM 0U
#endif
#ifndef AUDIO_STORE_FEATURE_RECORD
#define AUDIO_STORE_FEATURE_RECORD 1U
#endif
#define AUDIO_DB_CENTI_INVALID INT16_MIN

#define AUDIO_FLAG_COMPLETE (1U << 0)
#define AUDIO_FLAG_ACQUISITION_VALID (1U << 1)
#define AUDIO_FLAG_A_WEIGHTED_FEATURE_VALID (1U << 2)
#define AUDIO_FLAG_CLIPPED (1U << 3)
#define AUDIO_FLAG_HIGH_LEVEL (1U << 4)
#define AUDIO_FLAG_SILENT_OR_UNAVAILABLE (1U << 5)
#define AUDIO_FLAG_IMPULSIVE_EVENT (1U << 6)
#ifndef LIGHT_STORE_RAW_LRAW
#define LIGHT_STORE_RAW_LRAW 0U
#endif
#ifndef LIGHT_STORE_FEATURE_RECORD
#define LIGHT_STORE_FEATURE_RECORD 1U
#endif
#define LIGHT_EXPOSURE_UNAVAILABLE 255U
#define LIGHT_THRESHOLD_DARK_TO_LOW_COUNTS 3U
#define LIGHT_THRESHOLD_LOW_TO_MODERATE_COUNTS 50U
#define LIGHT_THRESHOLD_MODERATE_TO_HIGH_COUNTS 6500U
#define LIGHT_THRESHOLD_HIGH_TO_VERY_HIGH_COUNTS 9800U
#define LIGHT_SATURATION_CLEAR_COUNTS 10000U
#define LIGHT_FLAG_COMPLETE (1U << 0)
#define LIGHT_FLAG_ACQUISITION_VALID (1U << 1)
#define LIGHT_FLAG_CLASSIFICATION_VALID (1U << 2)
#define LIGHT_FLAG_SATURATED (1U << 3)
#define LIGHT_FLAG_I2C_ERROR (1U << 4)
#define LIGHT_FLAG_SMUX_ERROR (1U << 5)
#define USER_BUTTON_DEBOUNCE_MS 250U
#define USER_BUTTON_LONG_PRESS_MS 5000U
#define NAND_STARTUP_SELF_TEST_ENABLE 0U
#ifndef NAND_FORCE_ERASE_ON_BOOT
#define NAND_FORCE_ERASE_ON_BOOT 0U
#endif
#ifndef NAND_FLUSH_COMPACT_RECORDS_EVERY_CYCLE
#define NAND_FLUSH_COMPACT_RECORDS_EVERY_CYCLE 1U
#endif

typedef struct
{
    int16_t samples[AUDIO_CHUNK_SAMPLES];
} AudioRingSlot;

typedef enum
{
    FACTORY_ERASE_ERROR_NONE = 0,
    FACTORY_ERASE_ERROR_NOT_IDLE,
    FACTORY_ERASE_ERROR_MDF_STOP,
    FACTORY_ERASE_ERROR_BUFFER_DISCARD,
    FACTORY_ERASE_ERROR_DATA_ERASE,
    FACTORY_ERASE_ERROR_DATA_RECOVERY,
    FACTORY_ERASE_ERROR_BLE_METADATA,
    FACTORY_ERASE_ERROR_BLE_UART
} FactoryEraseError;

typedef enum
{
    AUDIO_ENV_VERY_QUIET = 0,
    AUDIO_ENV_QUIET = 1,
    AUDIO_ENV_MODERATE = 2,
    AUDIO_ENV_LIVELY = 3,
    AUDIO_ENV_NOISY = 4,
    AUDIO_ENV_VERY_NOISY = 5,
    AUDIO_ENV_HIGH_EXPOSURE = 6,
    AUDIO_ENV_UNAVAILABLE = 255
} AudioEnvironmentClass;

typedef enum
{
    LIGHT_DARK = 0,
    LIGHT_LOW_EXPOSURE = 1,
    LIGHT_MODERATE_EXPOSURE = 2,
    LIGHT_HIGH_EXPOSURE = 3,
    LIGHT_VERY_HIGH_EXPOSURE = 4
} LightExposureClass;

typedef struct
{
    uint32_t window_sequence;
    uint32_t sample_timestamp_ms;
    uint16_t f1;
    uint16_t f2;
    uint16_t f3;
    uint16_t f4;
    uint16_t f5;
    uint16_t f6;
    uint16_t f7;
    uint16_t f8;
    uint16_t clear;
    uint16_t nir;
    uint8_t previous_exposure_class;
    uint8_t exposure_class;
    uint8_t flags;
    uint8_t complete;
    uint8_t acquisition_valid;
    uint8_t classification_valid;
    uint8_t saturated;
} LightFeatureDiagnostics;

typedef struct
{
    double b0;
    double b1;
    double b2;
    double a1;
    double a2;
} AudioBiquadCoefficients;

typedef struct
{
    double s1;
    double s2;
} AudioBiquadState;

typedef struct
{
    uint32_t window_index;
    uint32_t window_start_ms;
    uint32_t sample_count;

    double mean_counts;
    double rms_zero_mean_counts;
    double rms_zero_mean_dbfs;

    uint32_t absolute_peak_counts;
    double peak_dbfs;

    uint32_t clipped_sample_count;
    double clipped_sample_percentage;

    double a_weighted_rms_counts;
    double a_weighted_rms_dbfs;
    double estimated_laeq_dba;

    uint8_t environment_class;
    uint8_t audio_flags;
    uint8_t acquisition_valid;
    uint8_t a_weighting_valid;
    uint8_t record_valid;

    uint8_t complete;
    uint8_t valid;
} AudioBasicFeatureDebug;

/*
 * Denominator convention: 1 + a1*z^-1 + a2*z^-2. The DF-II transposed
 * state updates therefore subtract a1*y and a2*y.
 * Jens Hee, "A-weighting filter for 44.1 and 48 kHz sampling", 2019.
 */
static const AudioBiquadCoefficients audio_a_weighting_biquads[3] =
{
    {0.96525096525, -1.34730163086, 0.38205066561,
     -1.34730722798, 0.34905752979},
    {0.94696969696, -1.89393939393, 0.94696969696,
     -1.89387049481, 0.89515976917},
    {0.64666542810, -0.38362237137, -0.26304305672,
     -1.34730722798, 0.34905752979}
};

static int16_t audio_dma_buffer[AUDIO_DMA_BUFFER_SAMPLES];
static AudioRingSlot audio_ring[AUDIO_RING_SLOT_COUNT];
static int16_t audio_window_pcm[AUDIO_WINDOW_TARGET_SAMPLES];

volatile uint32_t audio_window_pcm_samples = 0U;
AudioBasicFeatureDebug audio_basic_feature_latest;
AudioBasicFeatureDebug audio_basic_feature_history[AUDIO_BASIC_FEATURE_HISTORY_CAPACITY];
volatile uint32_t audio_basic_features_computed = 0U;
volatile uint32_t audio_basic_features_invalid = 0U;
volatile uint32_t audio_basic_feature_history_write_index = 0U;
volatile uint32_t audio_basic_feature_history_count = 0U;
volatile uint32_t audio_basic_feature_processing_last_ms = 0U;
volatile uint32_t audio_basic_feature_processing_max_ms = 0U;
volatile uint32_t audio_a_weighting_computed = 0U;
volatile uint32_t audio_a_weighting_invalid = 0U;
volatile uint32_t audio_a_weighting_processing_last_ms = 0U;
volatile uint32_t audio_a_weighting_processing_max_ms = 0U;
volatile uint32_t audio_total_feature_processing_last_ms = 0U;
volatile uint32_t audio_total_feature_processing_max_ms = 0U;

MDF_DmaConfigTypeDef mic_dma_config;

static volatile uint8_t microphone_active = 0U;
static volatile uint8_t audio_accept_chunks = 0U;
volatile uint32_t audio_ring_head = 0U;
volatile uint32_t audio_ring_tail = 0U;

typedef struct
{
    uint32_t session_start_tick_ms;
    uint32_t session_stop_tick_ms;

    uint32_t dma_start_attempt_count;
    uint32_t dma_start_ok_count;
    uint32_t dma_start_error_count;

    uint32_t dma_half_complete_count;
    uint32_t dma_full_complete_count;
    uint32_t dma_complete_count;
    uint32_t dma_error_callback_count;

    uint32_t buffer_ready_count;
    uint32_t buffer_drop_or_overwrite_count;

    uint32_t audio_chunks_enqueued;
    uint32_t audio_chunks_dequeued;
    uint32_t audio_ring_overflow_count;
    uint32_t audio_ring_high_watermark;
    uint32_t audio_ring_count_at_stop;

    uint32_t audio_window_target_samples;
    uint32_t audio_dma_samples_produced;
    uint32_t audio_window_chunks_published;
    uint32_t audio_window_samples_published;
    uint32_t audio_window_samples_accepted;
    uint32_t audio_samples_discarded_beyond_window;
    uint32_t audio_samples_discarded_during_stop;
    uint32_t audio_ring_overflow_samples;

    uint32_t audio_window_stop_request_count;
    uint32_t audio_window_target_stop_request_count;
    uint32_t audio_windows_completed;
    uint32_t audio_windows_incomplete;
    uint32_t audio_window_target_reached;
    uint32_t audio_window_incomplete;

    uint32_t dma_session_start_ok_count;
    uint32_t dma_session_start_error_count;
    uint32_t dma_session_stop_ok_count;
    uint32_t dma_session_stop_error_count;

    uint32_t final_partial_chunk_discarded_count;

    uint32_t mdf_stop_attempt_count;
    uint32_t mdf_stop_ok_count;
    uint32_t mdf_stop_error_count;

    uint32_t nand_append_attempt_count;
    uint32_t nand_append_ok_count;
    uint32_t nand_append_error_count;

    uint32_t audio_pages_confirmed_written;

    uint32_t page_sequence_before_last_append;
    uint32_t page_sequence_after_last_append;
    uint32_t last_page_sequence_delta;

    uint32_t last_dma_start_tick_ms;
    uint32_t last_dma_complete_tick_ms;
    uint32_t last_nand_append_tick_ms;

    uint32_t last_mdf_error_code;
    uint32_t last_dma_error_code;

    int32_t last_dma_start_status;
    int32_t last_mdf_stop_status;
    int32_t last_nand_append_status;
} MicDiagnostics;

volatile MicDiagnostics mic_diag;

volatile uint32_t audio_windows_requested = 0U;
volatile uint32_t audio_windows_started = 0U;
volatile uint32_t audio_windows_completed = 0U;
volatile uint32_t audio_windows_incomplete = 0U;
volatile uint32_t audio_windows_missed = 0U;
volatile uint32_t audio_scheduler_first_deadline_ms = 0U;
volatile uint32_t audio_scheduler_next_deadline_ms = 0U;
volatile uint32_t audio_window_last_start_ms = 0U;
volatile uint32_t audio_window_previous_start_ms = 0U;
volatile uint32_t audio_window_start_interval_ms = 0U;
static uint32_t current_window_sequence = 0U;
static uint32_t next_window_sequence = 1U;

static uint8_t audio_scheduler_enabled = 0U;
static uint8_t audio_scheduler_last_busy_interval_valid = 0U;
static uint32_t audio_scheduler_last_busy_start_ms = 0U;
static uint32_t audio_scheduler_last_busy_end_ms = 0U;
volatile uint8_t acquisition_paused_for_ble_sync = 0U;
volatile uint32_t acquisition_cycles_skipped_for_ble_sync = 0U;

// --- State Machine ---
static volatile AppState current_state = STATE_IDLE;
static uint32_t state_led_last_toggle_ms = 0U;
static AppState previous_state_led = STATE_IDLE;
static uint8_t state_led_initialized = 0U;
static uint32_t ble_sync_abort_led_until_ms = 0U;
static uint32_t ble_sync_abort_led_last_toggle_ms = 0U;
static uint32_t ble_sync_observed_aborted_sessions = 0U;

// --- Global Flags ---
volatile uint8_t usb_flag = 0U;
static volatile uint8_t start_acquisition_requested = 0U;
static volatile uint8_t stop_acquisition_requested = 0U;
static volatile uint8_t download_requested = 0U;
static volatile uint32_t sensor_tick_pending = 0U;
static volatile uint32_t user_button_last_event_ms = 0U;
static volatile uint8_t user_button_pressed = 0U;
static volatile uint8_t user_button_release_pending = 0U;
static volatile uint8_t user_button_long_press_triggered = 0U;
static volatile uint32_t user_button_press_start_ms = 0U;
static volatile AppState user_button_press_state = STATE_IDLE;
static volatile uint8_t factory_erase_requested = 0U;
static volatile uint8_t factory_erase_in_progress = 0U;

volatile uint32_t button_rising_count = 0U;
volatile uint32_t button_short_press_count = 0U;
volatile uint32_t button_long_press_count = 0U;
volatile uint32_t start_request_count = 0U;
volatile uint32_t stop_request_count = 0U;
volatile uint32_t download_request_count = 0U;
volatile AppState debug_state_at_button = STATE_IDLE;
volatile uint32_t factory_erase_request_count = 0U;
volatile uint32_t factory_erase_attempt_count = 0U;
volatile uint32_t factory_erase_completed_count = 0U;
volatile uint32_t factory_erase_failure_count = 0U;
volatile uint32_t factory_erase_rejected_count = 0U;
volatile uint32_t factory_erase_duration_ms = 0U;
volatile int32_t factory_erase_last_error = FACTORY_ERASE_ERROR_NONE;

// --- IMU data ---
static IMU_Data accelerometer_data;
static IMU_Data gyroscope_data;

uint8_t raw_accelerometer[6] = {0};
uint8_t raw_gyroscope[6]     = {0};

/*
 * raw_light layout (22 bytes):
 *   Legacy AS7341 area in the 40-byte IMU record. The raw-count pipeline keeps
 *   this area zero-filled and writes dedicated LOG_MAGIC_LIGHT_RAW pages.
 */
uint8_t raw_light[22] = {0};

static AS7341_Spectrum light_spectrum;
static uint8_t as7341_available = 0U;
static uint8_t light_measurement_pending = 0U;
static uint8_t light_exposure_state_valid = 0U;
static LightExposureClass light_exposure_state = LIGHT_DARK;
static uint32_t light_session_start_ms = 0U;
#if (LIGHT_STORE_RAW_LRAW != 0U)
static uint32_t light_sample_index = 0U;
#endif

volatile uint32_t light_samples_requested = 0U;
volatile uint32_t light_samples_acquired = 0U;
volatile uint32_t light_samples_saved = 0U;
volatile uint32_t light_samples_discarded = 0U;
volatile uint32_t light_measurements_requested = 0U;
volatile uint32_t light_measurements_started = 0U;
volatile uint32_t light_measurements_completed = 0U;
volatile uint32_t light_measurements_failed = 0U;
volatile uint32_t light_measurement_processing_last_ms = 0U;
volatile uint32_t light_measurement_processing_max_ms = 0U;
volatile LightFeatureDiagnostics light_feature_latest;

/// ----- NAND FLASH variables ----- ///

static NandLogger nand_logger;
int exit_flag = 0;

volatile int32_t nand_startup_self_test_result = -1;
volatile uint32_t nand_startup_self_test_stage = 0U;
volatile uint16_t nand_startup_self_test_block = UINT16_MAX;
volatile uint8_t nand_startup_self_test_completed = 0U;

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
static LightExposureClass Light_ClassifyInitial(uint16_t clear_raw);
static LightExposureClass Light_ClassifyWithHysteresis(uint16_t clear_raw);
static LogStatus AcquireAndStoreLightMeasurement(void);
static void AudioRing_Reset(void);
static uint32_t AudioRing_Count(void);
static void AudioRing_EnqueueFromIsr(const int16_t *samples);
static LogStatus Audio_AppendNextQueuedChunk(uint32_t timestamp_ms);
static LogStatus Audio_DrainQueuedChunks(uint32_t timestamp_ms);
static AudioBasicFeatureDebug Audio_ComputeBasicFeatures(
        const int16_t *samples,
        uint32_t sample_count);
static uint8_t Audio_ComputeAWeightedFeatures(
        const int16_t *samples,
        uint32_t sample_count,
        uint32_t sample_rate_hz,
        double mean_counts,
        double *rms_counts,
        double *rms_dbfs);
static AudioEnvironmentClass Audio_ClassifyEnvironment(double estimated_laeq_dba);
#if (AUDIO_STORE_FEATURE_RECORD != 0U)
static AudioFeatureRecordV1 Audio_BuildFeatureRecord(
        const AudioBasicFeatureDebug *features);
#endif
static void Audio_PublishBasicFeatures(uint8_t window_complete,
                                       LogStatus drain_status);
static void AudioScheduler_Init(void);
static void AudioScheduler_Process(uint32_t now_ms);
static void AudioScheduler_RecordWindowStart(uint32_t start_ms);
static void AudioScheduler_RecordWindowEnd(uint32_t end_ms);
static void AudioScheduler_PauseForBleSync(void);
static void AudioScheduler_ResumeAfterBleSync(uint32_t now_ms);
static void ProcessBleSync(uint32_t now_ms);
static void UserButton_HandleShortPress(AppState pressed_state);
static void UserButton_Process(uint32_t now_ms);
static int SmartWearable_FactoryEraseNand(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static Time_Struct Time_FromElapsedMilliseconds(uint32_t elapsed_ms)
{
    Time_Struct t;

    t.hh = (uint8_t)(elapsed_ms / 3600000U);
    t.mm = (uint8_t)((elapsed_ms / 60000U) % 60U);
    t.ss = (uint8_t)((elapsed_ms / 1000U) % 60U);
    t.sss = (uint16_t)(elapsed_ms % 1000U);

    return t;
}

static void UserButton_HandleShortPress(AppState pressed_state)
{
    button_short_press_count++;

    switch (pressed_state)
    {
        case STATE_IDLE:
        case STATE_ACQUISITION:
            ble_sync_requested = 1U;
            break;

        case STATE_BLE_SYNC:
            if (ble_sync_active != 0U)
            {
                ble_sync_abort_requested = 1U;
            }
            break;

        case STATE_USB_CONNECTED:
            exit_flag = 0;
            download_requested = 1U;
            download_request_count++;
            break;

        default:
            break;
    }
}

static void UserButton_Process(uint32_t now_ms)
{
    GPIO_PinState pin_state;

    if ((factory_erase_in_progress != 0U) ||
        (user_button_pressed == 0U))
    {
        return;
    }

    if (user_button_press_state == STATE_IDLE)
    {
        start_acquisition_requested = 0U;
        ble_sync_requested = 0U;
    }

    pin_state = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);
    if ((pin_state == GPIO_PIN_RESET) &&
        ((now_ms - user_button_last_event_ms) >= USER_BUTTON_DEBOUNCE_MS))
    {
        if ((user_button_long_press_triggered == 0U) &&
            ((now_ms - user_button_press_start_ms) >= USER_BUTTON_LONG_PRESS_MS) &&
            (user_button_press_state == STATE_IDLE) &&
            (current_state == STATE_IDLE) &&
            (usb_flag == 0U) &&
            (ble_sync_active == 0U) &&
            (microphone_active == 0U))
        {
            user_button_long_press_triggered = 1U;
            factory_erase_requested = 1U;
            button_long_press_count++;
            factory_erase_request_count++;
        }

        AppState pressed_state = user_button_press_state;
        uint8_t long_press_triggered = user_button_long_press_triggered;

        user_button_pressed = 0U;
        user_button_release_pending = 0U;
        user_button_long_press_triggered = 0U;
        user_button_press_start_ms = 0U;
        user_button_press_state = STATE_IDLE;
        user_button_last_event_ms = now_ms;

        if (long_press_triggered == 0U)
        {
            UserButton_HandleShortPress(pressed_state);
        }
        return;
    }

    if ((pin_state == GPIO_PIN_SET) &&
        (user_button_release_pending != 0U))
    {
        user_button_release_pending = 0U;
    }

    if ((user_button_long_press_triggered == 0U) &&
        ((now_ms - user_button_press_start_ms) >= USER_BUTTON_LONG_PRESS_MS) &&
        (user_button_press_state == STATE_IDLE) &&
        (current_state == STATE_IDLE) &&
        (usb_flag == 0U) &&
        (ble_sync_active == 0U) &&
        (microphone_active == 0U))
    {
        user_button_long_press_triggered = 1U;
        factory_erase_requested = 1U;
        button_long_press_count++;
        factory_erase_request_count++;
    }
}

static int SmartWearable_FactoryEraseNand(void)
{
    uint32_t erase_start_ms = HAL_GetTick();
    LogStatus logger_status;

    if ((current_state != STATE_IDLE) || (usb_flag != 0U) ||
        (ble_sync_active != 0U))
    {
        factory_erase_requested = 0U;
        factory_erase_rejected_count++;
        factory_erase_last_error = FACTORY_ERASE_ERROR_NOT_IDLE;
        return 1;
    }

    factory_erase_requested = 0U;
    factory_erase_in_progress = 1U;
    factory_erase_attempt_count++;
    factory_erase_last_error = FACTORY_ERASE_ERROR_NONE;
    factory_erase_duration_ms = 0U;

    start_acquisition_requested = 0U;
    stop_acquisition_requested = 0U;
    download_requested = 0U;
    ble_sync_requested = 0U;
    ble_sync_abort_requested = 0U;
    sensor_tick_pending = 0U;
    light_measurement_pending = 0U;
    acquisition_paused_for_ble_sync = 1U;
    audio_accept_chunks = 0U;

    current_state = STATE_FACTORY_ERASE;
    state_led_initialized = 0U;
    UpdateStateLed(current_state);
    HAL_TIM_Base_Stop_IT(&htim2);

    if (microphone_active != 0U)
    {
        if (HAL_MDF_AcqStop_DMA(&MdfHandle0) != HAL_OK)
        {
            factory_erase_failure_count++;
            factory_erase_last_error = FACTORY_ERASE_ERROR_MDF_STOP;
            return -1;
        }
        microphone_active = 0U;
    }

    AudioRing_Reset();
    audio_window_pcm_samples = 0U;

    logger_status = NANDLogger_DiscardPendingBuffers(&nand_logger);
    if (logger_status != LOG_OK)
    {
        factory_erase_failure_count++;
        factory_erase_last_error = FACTORY_ERASE_ERROR_BUFFER_DISCARD;
        return -1;
    }

    logger_status = NANDLogger_EraseAllGoodBlocks(&nand_logger);
    if (logger_status != LOG_OK)
    {
        factory_erase_failure_count++;
        factory_erase_last_error = FACTORY_ERASE_ERROR_DATA_ERASE;
        return -1;
    }

    logger_status = NANDLogger_Recover(&nand_logger);
    if (logger_status != LOG_OK)
    {
        factory_erase_failure_count++;
        factory_erase_last_error = FACTORY_ERASE_ERROR_DATA_RECOVERY;
        return -1;
    }

    if (BleSync_FactoryReset(&nand_logger) != 0)
    {
        factory_erase_failure_count++;
        factory_erase_last_error =
                (ble_sync_last_error == BLE_SYNC_ERROR_UART) ?
                FACTORY_ERASE_ERROR_BLE_UART :
                FACTORY_ERASE_ERROR_BLE_METADATA;
        return -1;
    }

    current_window_sequence = 0U;
    next_window_sequence = nand_recovered_next_window_sequence;
    audio_scheduler_first_deadline_ms = HAL_GetTick() + AUDIO_WINDOW_PERIOD_MS;
    audio_scheduler_next_deadline_ms = audio_scheduler_first_deadline_ms;
    audio_scheduler_last_busy_interval_valid = 0U;
    acquisition_paused_for_ble_sync = 0U;

    factory_erase_duration_ms = HAL_GetTick() - erase_start_ms;
    factory_erase_completed_count++;
    factory_erase_in_progress = 0U;
    current_state = STATE_IDLE;
    state_led_initialized = 0U;
    UpdateStateLed(current_state);

    return 0;
}

static uint8_t AudioScheduler_DeadlineWasBusy(uint32_t deadline_ms)
{
    uint32_t busy_duration_ms;
    uint32_t deadline_offset_ms;

    if (audio_scheduler_last_busy_interval_valid == 0U)
    {
        return 0U;
    }

    busy_duration_ms = audio_scheduler_last_busy_end_ms -
                       audio_scheduler_last_busy_start_ms;
    deadline_offset_ms = deadline_ms - audio_scheduler_last_busy_start_ms;

    return (deadline_offset_ms <= busy_duration_ms) ? 1U : 0U;
}

static void AudioScheduler_Init(void)
{
    uint32_t now_ms = HAL_GetTick();

    audio_windows_requested = 0U;
    audio_windows_started = 0U;
    audio_windows_completed = 0U;
    audio_windows_incomplete = 0U;
    audio_windows_missed = 0U;
    audio_window_last_start_ms = 0U;
    audio_window_previous_start_ms = 0U;
    audio_window_start_interval_ms = 0U;

    audio_scheduler_first_deadline_ms = now_ms;
    audio_scheduler_next_deadline_ms = now_ms;
    audio_scheduler_last_busy_start_ms = 0U;
    audio_scheduler_last_busy_end_ms = 0U;
    audio_scheduler_last_busy_interval_valid = 0U;
    audio_scheduler_enabled = 1U;

    light_measurement_pending = 0U;
    light_exposure_state_valid = 0U;
    light_exposure_state = LIGHT_DARK;
    light_measurements_requested = 0U;
    light_measurements_started = 0U;
    light_measurements_completed = 0U;
    light_measurements_failed = 0U;
    light_measurement_processing_last_ms = 0U;
    light_measurement_processing_max_ms = 0U;
    memset((void *)&light_feature_latest, 0, sizeof(light_feature_latest));
    light_feature_latest.previous_exposure_class = LIGHT_EXPOSURE_UNAVAILABLE;
    light_feature_latest.exposure_class = LIGHT_EXPOSURE_UNAVAILABLE;
}

static void AudioScheduler_Process(uint32_t now_ms)
{
    uint32_t due_deadlines;
    uint32_t elapsed_ms;
    uint8_t can_start;
    uint8_t deadline_was_busy;

    if (next_window_sequence == UINT32_MAX)
    {
        nand_storage_full = 1U;
        storage_full_latched = 1U;
    }

    if ((audio_scheduler_enabled == 0U) ||
        (acquisition_paused_for_ble_sync != 0U) ||
        (user_button_pressed != 0U) ||
        (factory_erase_requested != 0U) ||
        (factory_erase_in_progress != 0U) ||
        (ble_sync_requested != 0U) ||
        ((int32_t)(now_ms - audio_scheduler_next_deadline_ms) < 0))
    {
        return;
    }

    elapsed_ms = now_ms - audio_scheduler_next_deadline_ms;
    due_deadlines = (elapsed_ms / AUDIO_WINDOW_PERIOD_MS) + 1U;
    audio_windows_requested += due_deadlines;

    can_start = ((current_state == STATE_IDLE) &&
                 (usb_flag == 0U) &&
                 (start_acquisition_requested == 0U) &&
                 (stop_acquisition_requested == 0U) &&
                 (microphone_active == 0U) &&
                 (storage_full_latched == 0U)) ? 1U : 0U;

    deadline_was_busy = AudioScheduler_DeadlineWasBusy(
            audio_scheduler_next_deadline_ms);

    if ((due_deadlines == 1U) &&
        (can_start != 0U) &&
        (deadline_was_busy == 0U))
    {
        start_acquisition_requested = 1U;
        start_request_count++;
    }
    else
    {
        audio_windows_missed += due_deadlines;
    }

    audio_scheduler_next_deadline_ms += due_deadlines * AUDIO_WINDOW_PERIOD_MS;
}

static void AudioScheduler_RecordWindowStart(uint32_t start_ms)
{
    if (audio_windows_started != 0U)
    {
        audio_window_previous_start_ms = audio_window_last_start_ms;
        audio_window_start_interval_ms = start_ms - audio_window_last_start_ms;
    }

    audio_window_last_start_ms = start_ms;
    audio_windows_started++;
    audio_scheduler_last_busy_start_ms = start_ms;
    audio_scheduler_last_busy_interval_valid = 0U;
}

static void AudioScheduler_RecordWindowEnd(uint32_t end_ms)
{
    audio_scheduler_last_busy_end_ms = end_ms;
    audio_scheduler_last_busy_interval_valid = 1U;
}

static void AudioScheduler_PauseForBleSync(void)
{
    acquisition_paused_for_ble_sync = 1U;
    start_acquisition_requested = 0U;
}

static void AudioScheduler_ResumeAfterBleSync(uint32_t now_ms)
{
    if ((audio_scheduler_enabled != 0U) &&
        ((int32_t)(now_ms - audio_scheduler_next_deadline_ms) >= 0))
    {
        uint32_t elapsed_ms = now_ms - audio_scheduler_next_deadline_ms;
        acquisition_cycles_skipped_for_ble_sync +=
                (elapsed_ms / AUDIO_WINDOW_PERIOD_MS) + 1U;
    }

    audio_scheduler_next_deadline_ms = now_ms + AUDIO_WINDOW_PERIOD_MS;
    audio_scheduler_last_busy_interval_valid = 0U;
    acquisition_paused_for_ble_sync = 0U;
    start_acquisition_requested = 0U;
}

static uint32_t AudioRing_CountFrom(uint32_t head, uint32_t tail)
{
    if (head >= tail)
    {
        return head - tail;
    }

    return (AUDIO_RING_SLOT_COUNT - tail) + head;
}

static uint32_t AudioRing_Count(void)
{
    uint32_t head = audio_ring_head;
    uint32_t tail = audio_ring_tail;

    return AudioRing_CountFrom(head, tail);
}

static void AudioRing_Reset(void)
{
    audio_accept_chunks = 0U;
    audio_ring_head = 0U;
    audio_ring_tail = 0U;
    __DMB();
}

static void AudioRing_UpdateHighWatermark(uint32_t count)
{
    if (count > mic_diag.audio_ring_high_watermark)
    {
        mic_diag.audio_ring_high_watermark = count;
    }
}

static void AudioRing_EnqueueFromIsr(const int16_t *samples)
{
    uint32_t head;
    uint32_t tail;
    uint32_t next_head;

    if (current_state != STATE_ACQUISITION)
    {
        return;
    }

    if (audio_accept_chunks == 0U)
    {
        if (mic_diag.audio_window_target_reached != 0U)
        {
            mic_diag.audio_samples_discarded_beyond_window += AUDIO_CHUNK_SAMPLES;
        }
        else
        {
            mic_diag.audio_samples_discarded_during_stop += AUDIO_CHUNK_SAMPLES;
        }
        return;
    }

    head = audio_ring_head;
    tail = audio_ring_tail;
    next_head = head + 1U;
    if (next_head >= AUDIO_RING_SLOT_COUNT)
    {
        next_head = 0U;
    }

    if (next_head == tail)
    {
        mic_diag.audio_ring_overflow_count++;
        mic_diag.audio_ring_overflow_samples += AUDIO_CHUNK_SAMPLES;
        mic_diag.buffer_drop_or_overwrite_count++;
        return;
    }

    memcpy(audio_ring[head].samples, samples, AUDIO_CHUNK_SAMPLES * sizeof(int16_t));
    __DMB();

    audio_ring_head = next_head;
    mic_diag.audio_chunks_enqueued++;
    mic_diag.audio_window_chunks_published++;
    mic_diag.audio_window_samples_published += AUDIO_CHUNK_SAMPLES;
    mic_diag.buffer_ready_count++;
    AudioRing_UpdateHighWatermark(AudioRing_CountFrom(next_head, tail));

    if ((mic_diag.audio_window_chunks_published >= AUDIO_WINDOW_REQUIRED_CHUNKS) &&
        (mic_diag.audio_window_target_reached == 0U))
    {
        mic_diag.audio_window_target_reached = 1U;
        audio_accept_chunks = 0U;
        __DMB();

        if (stop_acquisition_requested == 0U)
        {
            stop_acquisition_requested = 1U;
            mic_diag.audio_window_stop_request_count++;
            mic_diag.audio_window_target_stop_request_count++;
        }
    }
}

static AudioBasicFeatureDebug Audio_ComputeBasicFeatures(
        const int16_t *samples,
        uint32_t sample_count)
{
    AudioBasicFeatureDebug features = {0};
    int64_t sample_sum = 0;
    double centered_energy = 0.0;
    uint32_t absolute_peak = 0U;
    uint32_t clipped_count = 0U;

    features.sample_count = sample_count;
    features.rms_zero_mean_dbfs = -INFINITY;
    features.peak_dbfs = -INFINITY;
    features.a_weighted_rms_dbfs = -INFINITY;
    features.estimated_laeq_dba = -INFINITY;
    features.environment_class = (uint8_t)AUDIO_ENV_UNAVAILABLE;

    if ((samples == NULL) || (sample_count == 0U))
    {
        return features;
    }

    for (uint32_t i = 0U; i < sample_count; i++)
    {
        sample_sum += samples[i];
    }

    features.mean_counts = (double)sample_sum / (double)sample_count;

    for (uint32_t i = 0U; i < sample_count; i++)
    {
        int32_t sample = samples[i];
        uint32_t magnitude = (sample < 0) ? (uint32_t)(-sample) : (uint32_t)sample;
        double centered = (double)sample - features.mean_counts;

        centered_energy += centered * centered;

        if (magnitude > absolute_peak)
        {
            absolute_peak = magnitude;
        }

        if ((sample == INT16_MIN) || (sample == INT16_MAX))
        {
            clipped_count++;
        }
    }

    features.rms_zero_mean_counts = sqrt(centered_energy / (double)sample_count);
    features.absolute_peak_counts = absolute_peak;
    features.clipped_sample_count = clipped_count;
    features.clipped_sample_percentage =
            (100.0 * (double)clipped_count) / (double)sample_count;

    if (features.rms_zero_mean_counts > 0.0)
    {
        features.rms_zero_mean_dbfs =
                20.0 * log10(features.rms_zero_mean_counts / 32768.0);
    }

    if (absolute_peak > 0U)
    {
        features.peak_dbfs = 20.0 * log10((double)absolute_peak / 32768.0);
    }

    return features;
}

static uint8_t Audio_ComputeAWeightedFeatures(
        const int16_t *samples,
        uint32_t sample_count,
        uint32_t sample_rate_hz,
        double mean_counts,
        double *rms_counts,
        double *rms_dbfs)
{
    AudioBiquadState states[3] = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
    double weighted_energy = 0.0;
    double mean_square;

    if ((rms_counts == NULL) || (rms_dbfs == NULL))
    {
        return 0U;
    }

    *rms_counts = 0.0;
    *rms_dbfs = -INFINITY;

    if ((samples == NULL) || (sample_count == 0U) ||
        (sample_rate_hz != AUDIO_SAMPLE_RATE_HZ) ||
        !isfinite(mean_counts))
    {
        return 0U;
    }

    /* Each window is isolated by about 9.5 s, so all biquad states start at zero. */
    for (uint32_t i = 0U; i < sample_count; i++)
    {
        double section_input = (double)samples[i] - mean_counts;

        for (uint32_t section = 0U; section < 3U; section++)
        {
            const AudioBiquadCoefficients *coefficients =
                    &audio_a_weighting_biquads[section];
            double output = coefficients->b0 * section_input + states[section].s1;
            double next_s1 = coefficients->b1 * section_input -
                             coefficients->a1 * output + states[section].s2;
            double next_s2 = coefficients->b2 * section_input -
                             coefficients->a2 * output;

            if (!isfinite(output) || !isfinite(next_s1) || !isfinite(next_s2))
            {
                return 0U;
            }

            states[section].s1 = next_s1;
            states[section].s2 = next_s2;
            section_input = output;
        }

        weighted_energy += section_input * section_input;
        if (!isfinite(weighted_energy) || (weighted_energy < 0.0))
        {
            return 0U;
        }
    }

    mean_square = weighted_energy / (double)sample_count;
    if (!isfinite(mean_square) || (mean_square < 0.0))
    {
        return 0U;
    }

    *rms_counts = sqrt(mean_square);
    if (!isfinite(*rms_counts) || (*rms_counts <= 0.0))
    {
        *rms_counts = 0.0;
        return 0U;
    }

    *rms_dbfs = 20.0 * log10(*rms_counts / 32768.0);
    if (!isfinite(*rms_dbfs))
    {
        *rms_dbfs = -INFINITY;
        return 0U;
    }

    return 1U;
}

static AudioEnvironmentClass Audio_ClassifyEnvironment(double estimated_laeq_dba)
{
    if (!isfinite(estimated_laeq_dba))
    {
        return AUDIO_ENV_UNAVAILABLE;
    }

    if (estimated_laeq_dba < 35.0) return AUDIO_ENV_VERY_QUIET;
    if (estimated_laeq_dba < 45.0) return AUDIO_ENV_QUIET;
    if (estimated_laeq_dba < 55.0) return AUDIO_ENV_MODERATE;
    if (estimated_laeq_dba < 65.0) return AUDIO_ENV_LIVELY;
    if (estimated_laeq_dba < 75.0) return AUDIO_ENV_NOISY;
    if (estimated_laeq_dba < 85.0) return AUDIO_ENV_VERY_NOISY;

    return AUDIO_ENV_HIGH_EXPOSURE;
}

#if (AUDIO_STORE_FEATURE_RECORD != 0U)
static int16_t Audio_RoundSaturateInt16(double value)
{
    double rounded;

    if (!isfinite(value))
    {
        return 0;
    }

    rounded = round(value);
    if (rounded <= (double)INT16_MIN) return INT16_MIN;
    if (rounded >= (double)INT16_MAX) return INT16_MAX;

    return (int16_t)rounded;
}

static int16_t Audio_DbToCenti(double value_db)
{
    double scaled;
    double rounded;

    if (!isfinite(value_db))
    {
        return AUDIO_DB_CENTI_INVALID;
    }

    scaled = value_db * 100.0;
    if (!isfinite(scaled))
    {
        return AUDIO_DB_CENTI_INVALID;
    }

    rounded = round(scaled);
    if (rounded <= (double)(INT16_MIN + 1)) return (int16_t)(INT16_MIN + 1);
    if (rounded >= (double)INT16_MAX) return INT16_MAX;

    return (int16_t)rounded;
}

static AudioFeatureRecordV1 Audio_BuildFeatureRecord(
        const AudioBasicFeatureDebug *features)
{
    AudioFeatureRecordV1 record = {0};

    if (features == NULL)
    {
        record.rms_z_centi_dbfs = AUDIO_DB_CENTI_INVALID;
        record.rms_a_centi_dbfs = AUDIO_DB_CENTI_INVALID;
        record.estimated_laeq_centi_dba = AUDIO_DB_CENTI_INVALID;
        record.peak_centi_dbfs = AUDIO_DB_CENTI_INVALID;
        record.environment_class = (uint8_t)AUDIO_ENV_UNAVAILABLE;
        return record;
    }

    record.window_sequence = features->window_index;
    record.window_start_ms = features->window_start_ms;
    record.sample_count = (features->sample_count <= UINT16_MAX) ?
                          (uint16_t)features->sample_count : UINT16_MAX;
    record.mean_counts_rounded = Audio_RoundSaturateInt16(features->mean_counts);
    record.rms_z_centi_dbfs = Audio_DbToCenti(features->rms_zero_mean_dbfs);
    record.rms_a_centi_dbfs = Audio_DbToCenti(features->a_weighted_rms_dbfs);
    record.estimated_laeq_centi_dba = Audio_DbToCenti(features->estimated_laeq_dba);
    record.peak_centi_dbfs = Audio_DbToCenti(features->peak_dbfs);
    record.clipped_sample_count =
            (features->clipped_sample_count <= UINT16_MAX) ?
            (uint16_t)features->clipped_sample_count : UINT16_MAX;
    record.environment_class = features->environment_class;
    record.flags = features->audio_flags;

    return record;
}
#endif

static void Audio_PublishBasicFeatures(uint8_t window_complete,
                                       LogStatus drain_status)
{
    AudioBasicFeatureDebug features;
#if (AUDIO_STORE_FEATURE_RECORD != 0U)
    AudioFeatureRecordV1 feature_record;
#endif
    uint32_t total_processing_start_ms;
    uint32_t basic_processing_start_ms;
    uint32_t a_weighting_start_ms;
    uint32_t processing_elapsed_ms;
    uint32_t history_index;
    uint8_t z_values_are_finite;

    total_processing_start_ms = HAL_GetTick();
    basic_processing_start_ms = HAL_GetTick();
    features = Audio_ComputeBasicFeatures(
            audio_window_pcm,
            audio_window_pcm_samples);
    processing_elapsed_ms = HAL_GetTick() - basic_processing_start_ms;

    audio_basic_feature_processing_last_ms = processing_elapsed_ms;
    if (processing_elapsed_ms > audio_basic_feature_processing_max_ms)
    {
        audio_basic_feature_processing_max_ms = processing_elapsed_ms;
    }

    features.window_index = current_window_sequence;
    features.window_start_ms = audio_window_last_start_ms;
    features.complete =
            ((window_complete != 0U) &&
             (features.sample_count == AUDIO_WINDOW_TARGET_SAMPLES)) ? 1U : 0U;

    features.acquisition_valid =
            ((features.sample_count == mic_diag.audio_window_samples_accepted) &&
             (drain_status == LOG_OK) &&
             (mic_diag.audio_ring_overflow_count == 0U) &&
             (mic_diag.last_dma_start_status == (int32_t)HAL_OK) &&
             (mic_diag.last_mdf_stop_status == (int32_t)HAL_OK) &&
             (mic_diag.last_mdf_error_code == 0U) &&
             (mic_diag.last_dma_error_code == 0U)) ? 1U : 0U;

    z_values_are_finite =
            (isfinite(features.mean_counts) &&
             isfinite(features.rms_zero_mean_counts) &&
             isfinite(features.rms_zero_mean_dbfs) &&
             isfinite(features.peak_dbfs) &&
             isfinite(features.clipped_sample_percentage)) ? 1U : 0U;

    features.valid =
            ((features.sample_count == AUDIO_WINDOW_TARGET_SAMPLES) &&
             (features.complete != 0U) &&
             (features.acquisition_valid != 0U) &&
             (z_values_are_finite != 0U)) ? 1U : 0U;

    a_weighting_start_ms = HAL_GetTick();
    features.a_weighting_valid = Audio_ComputeAWeightedFeatures(
            audio_window_pcm,
            audio_window_pcm_samples,
            AUDIO_SAMPLE_RATE_HZ,
            features.mean_counts,
            &features.a_weighted_rms_counts,
            &features.a_weighted_rms_dbfs);
    processing_elapsed_ms = HAL_GetTick() - a_weighting_start_ms;

    audio_a_weighting_processing_last_ms = processing_elapsed_ms;
    if (processing_elapsed_ms > audio_a_weighting_processing_max_ms)
    {
        audio_a_weighting_processing_max_ms = processing_elapsed_ms;
    }

    audio_a_weighting_computed++;
    if (features.a_weighting_valid != 0U)
    {
        features.estimated_laeq_dba =
                features.a_weighted_rms_dbfs +
                AUDIO_SPL_CALIBRATION_OFFSET_DB;
        if (!isfinite(features.estimated_laeq_dba))
        {
            features.estimated_laeq_dba = -INFINITY;
            features.a_weighting_valid = 0U;
        }
    }

    if (features.a_weighting_valid == 0U)
    {
        audio_a_weighting_invalid++;
    }

    features.environment_class = (uint8_t)Audio_ClassifyEnvironment(
            features.estimated_laeq_dba);
    features.audio_flags = 0U;

    if (features.complete != 0U)
        features.audio_flags |= AUDIO_FLAG_COMPLETE;
    if (features.acquisition_valid != 0U)
        features.audio_flags |= AUDIO_FLAG_ACQUISITION_VALID;
    if (features.a_weighting_valid != 0U)
        features.audio_flags |= AUDIO_FLAG_A_WEIGHTED_FEATURE_VALID;
    if (features.clipped_sample_count > 0U)
        features.audio_flags |= AUDIO_FLAG_CLIPPED;
    if (isfinite(features.estimated_laeq_dba) &&
        (features.estimated_laeq_dba >= 85.0))
        features.audio_flags |= AUDIO_FLAG_HIGH_LEVEL;
    if (features.a_weighting_valid == 0U)
        features.audio_flags |= AUDIO_FLAG_SILENT_OR_UNAVAILABLE;

    features.record_valid =
            ((features.complete != 0U) &&
             (features.acquisition_valid != 0U) &&
             (features.valid != 0U) &&
             (features.a_weighting_valid != 0U)) ? 1U : 0U;

    history_index = audio_basic_feature_history_write_index;
    audio_basic_feature_history[history_index] = features;
    audio_basic_feature_latest = features;

    history_index++;
    if (history_index >= AUDIO_BASIC_FEATURE_HISTORY_CAPACITY)
    {
        history_index = 0U;
    }
    audio_basic_feature_history_write_index = history_index;

    if (audio_basic_feature_history_count < AUDIO_BASIC_FEATURE_HISTORY_CAPACITY)
    {
        audio_basic_feature_history_count++;
    }

    audio_basic_features_computed++;
    if (features.valid == 0U)
    {
        audio_basic_features_invalid++;
    }

#if (AUDIO_STORE_FEATURE_RECORD != 0U)
    feature_record = Audio_BuildFeatureRecord(&features);
    audio_feature_records_generated++;
#endif

    processing_elapsed_ms = HAL_GetTick() - total_processing_start_ms;
    audio_total_feature_processing_last_ms = processing_elapsed_ms;
    if (processing_elapsed_ms > audio_total_feature_processing_max_ms)
    {
        audio_total_feature_processing_max_ms = processing_elapsed_ms;
    }

#if (AUDIO_STORE_FEATURE_RECORD != 0U)
    if (NANDLogger_AppendAudioFeatureRecord(&nand_logger,
                                            &feature_record) != LOG_OK)
    {
        LED_On(LED_RED);
    }
#endif
}

static LogStatus Audio_AppendNextQueuedChunk(uint32_t timestamp_ms)
{
    LogStatus append_status = LOG_OK;
    uint32_t tail;
    uint32_t next_tail;
#if (AUDIO_STORE_RAW_PCM != 0U)
    uint32_t page_delta = 0U;
#endif
    uint32_t remaining_samples;
    uint32_t accepted_samples;
    uint32_t discarded_samples;
    uint32_t window_offset;

#if (AUDIO_STORE_RAW_PCM == 0U)
    (void)timestamp_ms;
#endif

    if (audio_ring_tail == audio_ring_head)
    {
        return LOG_OK;
    }

    __DMB();
    tail = audio_ring_tail;
    next_tail = tail + 1U;
    if (next_tail >= AUDIO_RING_SLOT_COUNT)
    {
        next_tail = 0U;
    }

    if (mic_diag.audio_window_samples_accepted < mic_diag.audio_window_target_samples)
    {
        remaining_samples = mic_diag.audio_window_target_samples -
                            mic_diag.audio_window_samples_accepted;
        accepted_samples = (remaining_samples < AUDIO_CHUNK_SAMPLES) ?
                           remaining_samples : AUDIO_CHUNK_SAMPLES;
    }
    else
    {
        accepted_samples = 0U;
    }

    discarded_samples = AUDIO_CHUNK_SAMPLES - accepted_samples;

    if (accepted_samples > 0U)
    {
        window_offset = audio_window_pcm_samples;
        if ((window_offset > AUDIO_WINDOW_TARGET_SAMPLES) ||
            (accepted_samples > (AUDIO_WINDOW_TARGET_SAMPLES - window_offset)))
        {
            return LOG_ERR_BAD_ARGUMENT;
        }

        memcpy(&audio_window_pcm[window_offset],
               audio_ring[tail].samples,
               accepted_samples * sizeof(int16_t));

        mic_diag.audio_window_samples_accepted += accepted_samples;
        audio_window_pcm_samples = window_offset + accepted_samples;

#if (AUDIO_STORE_RAW_PCM != 0U)
        mic_diag.nand_append_attempt_count++;
        mic_diag.page_sequence_before_last_append = nand_logger.page_sequence;

        append_status = NANDLogger_AppendAudioBuffer(
                &nand_logger,
                audio_ring[tail].samples,
                accepted_samples,
                timestamp_ms);

        mic_diag.last_nand_append_status = (int32_t)append_status;
        mic_diag.page_sequence_after_last_append = nand_logger.page_sequence;

        if (mic_diag.page_sequence_after_last_append >= mic_diag.page_sequence_before_last_append)
        {
            page_delta = mic_diag.page_sequence_after_last_append -
                         mic_diag.page_sequence_before_last_append;
        }

        mic_diag.last_page_sequence_delta = page_delta;
        mic_diag.last_nand_append_tick_ms = HAL_GetTick();

        if (append_status != LOG_OK)
        {
            mic_diag.nand_append_error_count++;
        }
        else
        {
            mic_diag.nand_append_ok_count++;

            if (page_delta == 1U)
            {
                mic_diag.audio_pages_confirmed_written++;
            }
        }
#endif
    }

    mic_diag.audio_samples_discarded_beyond_window += discarded_samples;
    audio_ring_tail = next_tail;
    mic_diag.audio_chunks_dequeued++;

    return append_status;
}

static LogStatus Audio_DrainQueuedChunks(uint32_t timestamp_ms)
{
    LogStatus status = LOG_OK;
    LogStatus first_error = LOG_OK;

    while (audio_ring_tail != audio_ring_head)
    {
        status = Audio_AppendNextQueuedChunk(timestamp_ms);
        if ((status != LOG_OK) && (first_error == LOG_OK))
        {
            first_error = status;
        }
    }

    return first_error;
}

static void UpdateStateLed(AppState state)
{
    uint32_t now = HAL_GetTick();
    uint32_t blink_interval_ms = 0U;

    if ((storage_full_latched != 0U) && (state != STATE_FACTORY_ERASE))
    {
        LED_On(LED_GREEN);
        LED_On(LED_RED);
        return;
    }

    if ((ble_sync_abort_led_until_ms != 0U) &&
        ((int32_t)(now - ble_sync_abort_led_until_ms) < 0))
    {
        LED_Off(LED_GREEN);
        if ((now - ble_sync_abort_led_last_toggle_ms) >= 150U)
        {
            ble_sync_abort_led_last_toggle_ms = now;
            LED_Toggle(LED_RED);
        }
        return;
    }
    else if (ble_sync_abort_led_until_ms != 0U)
    {
        ble_sync_abort_led_until_ms = 0U;
        LED_Off(LED_RED);
        state_led_initialized = 0U;
    }

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

            case STATE_BLE_SYNC:
                LED_Off(LED_RED);
                LED_On(LED_GREEN);
                return;

            case STATE_FACTORY_ERASE:
                LED_Off(LED_RED);
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

        case STATE_BLE_SYNC:
            blink_interval_ms = 250U;
            break;

        case STATE_FACTORY_ERASE:
            blink_interval_ms = 250U;
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

void App_UpdateFactoryEraseLed(void)
{
    if (factory_erase_in_progress != 0U)
    {
        UpdateStateLed(STATE_FACTORY_ERASE);
    }
}

static void MicDiagnostics_UpdateErrorCodes(void)
{
    mic_diag.last_mdf_error_code = MdfHandle0.ErrorCode;

    if (MdfHandle0.hdma != NULL)
    {
        mic_diag.last_dma_error_code = MdfHandle0.hdma->ErrorCode;
    }
    else
    {
        mic_diag.last_dma_error_code = 0U;
    }
}

void HAL_MDF_AcqHalfCpltCallback(MDF_HandleTypeDef *hmdf)
{
    if (hmdf != &MdfHandle0)
    {
        return;
    }

    mic_diag.dma_half_complete_count++;
    mic_diag.dma_complete_count++;
    mic_diag.audio_dma_samples_produced += AUDIO_CHUNK_SAMPLES;
    mic_diag.last_dma_complete_tick_ms = HAL_GetTick();

    AudioRing_EnqueueFromIsr(&audio_dma_buffer[0]);
}

void HAL_MDF_AcqCpltCallback(MDF_HandleTypeDef *hmdf)
{
    if (hmdf != &MdfHandle0)
    {
        return;
    }

    mic_diag.dma_full_complete_count++;
    mic_diag.dma_complete_count++;
    mic_diag.audio_dma_samples_produced += AUDIO_CHUNK_SAMPLES;
    mic_diag.last_dma_complete_tick_ms = HAL_GetTick();

    AudioRing_EnqueueFromIsr(&audio_dma_buffer[AUDIO_CHUNK_SAMPLES]);
}

void HAL_MDF_ErrorCallback(MDF_HandleTypeDef *hmdf)
{
    if (hmdf != &MdfHandle0)
    {
        return;
    }

    mic_diag.dma_error_callback_count++;
    MicDiagnostics_UpdateErrorCodes();
}
/* USER CODE END 0 */
static void StopAcquisition(void)
{
    uint32_t stop_ms = HAL_GetTick();
    LogStatus flush_status;
    LogStatus drain_status;
    HAL_StatusTypeDef stop_status;
    uint8_t window_complete;

    HAL_TIM_Base_Stop_IT(&htim2);

    mic_diag.session_stop_tick_ms = stop_ms;
    audio_accept_chunks = 0U;

    if (microphone_active)
    {
        mic_diag.mdf_stop_attempt_count++;
        stop_status = HAL_MDF_AcqStop_DMA(&MdfHandle0);
        mic_diag.last_mdf_stop_status = (int32_t)stop_status;
        MicDiagnostics_UpdateErrorCodes();

        if (stop_status == HAL_OK)
        {
            mic_diag.mdf_stop_ok_count++;
            mic_diag.dma_session_stop_ok_count++;
            mic_diag.final_partial_chunk_discarded_count++;
        }
        else
        {
            mic_diag.mdf_stop_error_count++;
            mic_diag.dma_session_stop_error_count++;
        }

        microphone_active = 0U;
    }

    mic_diag.audio_ring_count_at_stop = AudioRing_Count();
    drain_status = Audio_DrainQueuedChunks(stop_ms);
    if (drain_status != LOG_OK)
    {
        LED_On(LED_RED);
    }

    if (mic_diag.audio_window_samples_accepted == mic_diag.audio_window_target_samples)
    {
        mic_diag.audio_windows_completed++;
        mic_diag.audio_window_incomplete = 0U;
        audio_windows_completed++;
        window_complete = 1U;
    }
    else
    {
        mic_diag.audio_windows_incomplete++;
        mic_diag.audio_window_incomplete = 1U;
        audio_windows_incomplete++;
        window_complete = 0U;
    }

    if (light_measurement_pending != 0U)
    {
        light_measurement_pending = 0U;
        if ((storage_full_latched == 0U) &&
            (AcquireAndStoreLightMeasurement() != LOG_OK))
        {
            LED_On(LED_RED);
        }
    }

    Audio_PublishBasicFeatures(window_complete, drain_status);

    sensor_tick_pending = 0U;
    stop_acquisition_requested = 0U;

    current_state = STATE_IDLE;

#if (NAND_FLUSH_COMPACT_RECORDS_EVERY_CYCLE != 0U)
    flush_status = NANDLogger_FlushAll(&nand_logger, stop_ms);
#else
    flush_status = NANDLogger_FlushWindowData(&nand_logger, stop_ms);
#endif
    if (flush_status != LOG_OK)
    {
        LED_On(LED_RED);
    }

    UpdateStateLed(current_state);

    AudioScheduler_RecordWindowEnd(HAL_GetTick());
}

static LightExposureClass Light_ClassifyInitial(uint16_t clear_raw)
{
    if (clear_raw < LIGHT_THRESHOLD_DARK_TO_LOW_COUNTS)
        return LIGHT_DARK;
    if (clear_raw < LIGHT_THRESHOLD_LOW_TO_MODERATE_COUNTS)
        return LIGHT_LOW_EXPOSURE;
    if (clear_raw < LIGHT_THRESHOLD_MODERATE_TO_HIGH_COUNTS)
        return LIGHT_MODERATE_EXPOSURE;
    if (clear_raw < LIGHT_THRESHOLD_HIGH_TO_VERY_HIGH_COUNTS)
        return LIGHT_HIGH_EXPOSURE;

    return LIGHT_VERY_HIGH_EXPOSURE;
}

static uint16_t Light_GetBoundaryThreshold(LightExposureClass lower_class)
{
    switch (lower_class)
    {
        case LIGHT_DARK:
            return LIGHT_THRESHOLD_DARK_TO_LOW_COUNTS;
        case LIGHT_LOW_EXPOSURE:
            return LIGHT_THRESHOLD_LOW_TO_MODERATE_COUNTS;
        case LIGHT_MODERATE_EXPOSURE:
            return LIGHT_THRESHOLD_MODERATE_TO_HIGH_COUNTS;
        case LIGHT_HIGH_EXPOSURE:
        default:
            return LIGHT_THRESHOLD_HIGH_TO_VERY_HIGH_COUNTS;
    }
}

static uint16_t Light_GetReturnThreshold(uint16_t threshold)
{
    uint16_t margin = (uint16_t)(((uint32_t)threshold * 5U + 99U) / 100U);

    if (margin < 1U)
    {
        margin = 1U;
    }

    return (uint16_t)(threshold - margin);
}

static LightExposureClass Light_ClassifyWithHysteresis(uint16_t clear_raw)
{
    LightExposureClass classification = light_exposure_state;

    while (classification < LIGHT_VERY_HIGH_EXPOSURE)
    {
        uint16_t threshold = Light_GetBoundaryThreshold(classification);

        if (clear_raw < threshold)
        {
            break;
        }

        classification = (LightExposureClass)((uint32_t)classification + 1U);
    }

    while (classification > LIGHT_DARK)
    {
        LightExposureClass lower_class =
                (LightExposureClass)((uint32_t)classification - 1U);
        uint16_t threshold = Light_GetBoundaryThreshold(lower_class);

        if (clear_raw >= Light_GetReturnThreshold(threshold))
        {
            break;
        }

        classification = lower_class;
    }

    return classification;
}

static LogStatus AcquireAndStoreLightMeasurement(void)
{
    LightFeatureRecordV1 feature_record = {0};
    LightFeatureDiagnostics latest = {0};
    LogStatus storage_status = LOG_OK;
    uint32_t processing_start_ms = HAL_GetTick();
    uint8_t measurement_valid = 0U;
    uint8_t previous_exposure_class = LIGHT_EXPOSURE_UNAVAILABLE;
#if (AS7341_ENABLE_SMUX_DIAGNOSTICS != 0U)
    uint32_t i2c_errors_before = as7341_diag_i2c_error_count;
    uint32_t smux_errors_before = as7341_diag_smux_timeout_count;
#endif

    light_measurements_started++;
    light_samples_requested++;
    feature_record.window_sequence = current_window_sequence;
    feature_record.exposure_class = LIGHT_EXPOSURE_UNAVAILABLE;

    if (light_exposure_state_valid != 0U)
    {
        previous_exposure_class = (uint8_t)light_exposure_state;
    }

    memset(&light_spectrum, 0, sizeof(light_spectrum));
    if ((as7341_available != 0U) &&
        (AS7341_ReadFullSpectrum(&light_spectrum) == 1U))
    {
        LightExposureClass classification;

        feature_record.f1 = light_spectrum.ch[0];
        feature_record.f2 = light_spectrum.ch[1];
        feature_record.f3 = light_spectrum.ch[2];
        feature_record.f4 = light_spectrum.ch[3];
        feature_record.f5 = light_spectrum.ch[4];
        feature_record.f6 = light_spectrum.ch[5];
        feature_record.f7 = light_spectrum.ch[6];
        feature_record.f8 = light_spectrum.ch[7];
        feature_record.clear = light_spectrum.ch[8];
        feature_record.nir = light_spectrum.ch[9];
        feature_record.sample_timestamp_ms = HAL_GetTick();

        if (light_exposure_state_valid == 0U)
        {
            classification = Light_ClassifyInitial(feature_record.clear);
        }
        else
        {
            classification =
                    Light_ClassifyWithHysteresis(feature_record.clear);
        }

        if (feature_record.clear >= LIGHT_SATURATION_CLEAR_COUNTS)
        {
            classification = LIGHT_VERY_HIGH_EXPOSURE;
            feature_record.flags |= LIGHT_FLAG_SATURATED;
        }

        light_exposure_state = classification;
        light_exposure_state_valid = 1U;
        feature_record.exposure_class = (uint8_t)classification;
        feature_record.flags |= LIGHT_FLAG_COMPLETE |
                                LIGHT_FLAG_ACQUISITION_VALID |
                                LIGHT_FLAG_CLASSIFICATION_VALID;
        measurement_valid = 1U;
        light_measurements_completed++;
        light_samples_acquired++;

#if (LIGHT_STORE_RAW_LRAW != 0U)
        {
            LightRawSampleRecord raw_record;
            LogStatus raw_status;

            raw_record.sample_elapsed_ms =
                    feature_record.sample_timestamp_ms - light_session_start_ms;
            raw_record.sample_index = light_sample_index;
            raw_record.f1_counts = feature_record.f1;
            raw_record.f2_counts = feature_record.f2;
            raw_record.f3_counts = feature_record.f3;
            raw_record.f4_counts = feature_record.f4;
            raw_record.f5_counts = feature_record.f5;
            raw_record.f6_counts = feature_record.f6;
            raw_record.f7_counts = feature_record.f7;
            raw_record.f8_counts = feature_record.f8;
            raw_record.clear_counts = feature_record.clear;
            raw_record.nir_counts = feature_record.nir;

            raw_status = NANDLogger_AppendLightRawRecord(
                    &nand_logger,
                    &raw_record,
                    feature_record.sample_timestamp_ms);
            if (raw_status == LOG_OK)
            {
                light_sample_index++;
                light_samples_saved++;
            }
            else
            {
                light_samples_discarded++;
                storage_status = raw_status;
            }
        }
#endif
    }
    else
    {
        feature_record.sample_timestamp_ms = HAL_GetTick();
        light_measurements_failed++;
        light_samples_discarded++;
        LED_On(LED_RED);

#if (AS7341_ENABLE_SMUX_DIAGNOSTICS != 0U)
        if (as7341_diag_i2c_error_count != i2c_errors_before)
        {
            feature_record.flags |= LIGHT_FLAG_I2C_ERROR;
        }
        if (as7341_diag_smux_timeout_count != smux_errors_before)
        {
            feature_record.flags |= LIGHT_FLAG_SMUX_ERROR;
        }
#endif
    }

#if (LIGHT_STORE_FEATURE_RECORD != 0U)
    light_feature_records_generated++;
    {
        LogStatus feature_status = NANDLogger_AppendLightFeatureRecord(
                &nand_logger,
                &feature_record);

        if (feature_status != LOG_OK)
        {
            storage_status = feature_status;
        }
    }
#endif

    latest.window_sequence = feature_record.window_sequence;
    latest.sample_timestamp_ms = feature_record.sample_timestamp_ms;
    latest.f1 = feature_record.f1;
    latest.f2 = feature_record.f2;
    latest.f3 = feature_record.f3;
    latest.f4 = feature_record.f4;
    latest.f5 = feature_record.f5;
    latest.f6 = feature_record.f6;
    latest.f7 = feature_record.f7;
    latest.f8 = feature_record.f8;
    latest.clear = feature_record.clear;
    latest.nir = feature_record.nir;
    latest.previous_exposure_class = previous_exposure_class;
    latest.exposure_class = feature_record.exposure_class;
    latest.flags = feature_record.flags;
    latest.complete = ((feature_record.flags & LIGHT_FLAG_COMPLETE) != 0U) ? 1U : 0U;
    latest.acquisition_valid = measurement_valid;
    latest.classification_valid =
            ((feature_record.flags & LIGHT_FLAG_CLASSIFICATION_VALID) != 0U) ?
            1U : 0U;
    latest.saturated =
            ((feature_record.flags & LIGHT_FLAG_SATURATED) != 0U) ? 1U : 0U;
    light_feature_latest = latest;

    light_measurement_processing_last_ms = HAL_GetTick() - processing_start_ms;
    if (light_measurement_processing_last_ms >
        light_measurement_processing_max_ms)
    {
        light_measurement_processing_max_ms =
                light_measurement_processing_last_ms;
    }

    return storage_status;
}

static void ProcessSensorTick(void)
{
    uint32_t elapsed_ms;

    /* --- Read IMU --- */
    IMU_ReadAccelerometerData(&accelerometer_data, raw_accelerometer);
    IMU_ReadGyroscopeData(&gyroscope_data, raw_gyroscope);

    /* --- Timestamp from real elapsed session time, shared with light samples --- */
    elapsed_ms = HAL_GetTick() - light_session_start_ms;
    timestamp = Time_FromElapsedMilliseconds(elapsed_ms);
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

static void ProcessBleSync(uint32_t now_ms)
{
    uint8_t was_active = ble_sync_active;

    if ((ble_sync_active == 0U) && (ble_sync_requested != 0U))
    {
        if (usb_flag != 0U)
        {
            ble_sync_requested = 0U;
        }
        else if (current_state == STATE_IDLE)
        {
            AudioScheduler_PauseForBleSync();
            if (BleSync_StartSession(&nand_logger, now_ms) == 0)
            {
                current_state = STATE_BLE_SYNC;
                state_led_initialized = 0U;
            }
            else
            {
                ble_sync_requested = 0U;
                AudioScheduler_ResumeAfterBleSync(now_ms);
                ble_sync_abort_led_last_toggle_ms = now_ms;
                ble_sync_abort_led_until_ms = now_ms + 2000U;
            }
        }
    }

    if ((ble_sync_active != 0U) && (usb_flag != 0U))
    {
        BleSync_RequestUsbPreemption();
    }

    BleSync_Process(&nand_logger, now_ms, usb_flag);

    if ((was_active != 0U) && (ble_sync_active == 0U))
    {
        AudioScheduler_ResumeAfterBleSync(now_ms);
        current_state = (usb_flag != 0U) ?
                        STATE_USB_CONNECTED : STATE_IDLE;
        state_led_initialized = 0U;

        if (ble_sync_sessions_aborted !=
            ble_sync_observed_aborted_sessions)
        {
            ble_sync_observed_aborted_sessions = ble_sync_sessions_aborted;
            ble_sync_abort_led_last_toggle_ms = now_ms;
            ble_sync_abort_led_until_ms = now_ms + 2000U;
            LED_On(LED_RED);
        }
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

  mcu_reset_csr_at_boot = RCC->CSR;

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

  int nand_init_result = spi_nand_init();

#if (NAND_STARTUP_SELF_TEST_ENABLE != 0U)
  if (nand_init_result != SPI_NAND_RET_OK)
  {
    nand_startup_self_test_result = nand_init_result;
    nand_startup_self_test_stage = 6U;
    nand_startup_self_test_completed = 1U;

    while (1)
    {
      __NOP();
    }
  }
#else
  if (nand_init_result != SPI_NAND_RET_OK)
  {
    nand_recovery_started = 1U;
    nand_recovery_failed = 1U;
    Error_Handler();
  }
#endif

  LogStatus nand_logger_init_status = NANDLogger_Init(&nand_logger);

#if (NAND_STARTUP_SELF_TEST_ENABLE != 0U)
  if (nand_logger_init_status != LOG_OK)
  {
    nand_startup_self_test_result = (int32_t)nand_logger_init_status;
    nand_startup_self_test_stage = 6U;
    nand_startup_self_test_completed = 1U;

    while (1)
    {
      __NOP();
    }
  }

  if (nand_logger.good_block_count == 0U)
  {
    nand_startup_self_test_result = (int32_t)LOG_ERR_NO_GOOD_BLOCKS;
    nand_startup_self_test_stage = 6U;
    nand_startup_self_test_completed = 1U;

    while (1)
    {
      __NOP();
    }
  }

  uint16_t test_good_block_index = nand_logger.good_block_count - 1U;
  uint16_t test_block = nand_logger.good_blocks[test_good_block_index];
  read_address_t nand_test_row = {0};
  int self_test_ret;

  nand_logger.good_block_count--;

  nand_test_row.block = test_block;
  nand_test_row.page = 0U;
  nand_test_row.dummy = 0U;

  nand_startup_self_test_block = test_block;
  nand_startup_self_test_stage = 1U;

  nand_startup_self_test_stage = 2U;
  self_test_ret = spi_nand_block_erase(nand_test_row);

  if (self_test_ret != SPI_NAND_RET_OK)
  {
    nand_startup_self_test_result = self_test_ret;
    nand_startup_self_test_stage = 6U;
    nand_startup_self_test_completed = 1U;

    while (1)
    {
      __NOP();
    }
  }

  nand_startup_self_test_stage = 3U;

  nand_startup_self_test_stage = 4U;
  self_test_ret = spi_nand_page_program_self_test(nand_test_row);
  nand_startup_self_test_result = self_test_ret;
  nand_startup_self_test_completed = 1U;
  nand_startup_self_test_stage = (self_test_ret == SPI_NAND_RET_OK) ? 5U : 6U;

  while (1)
  {
    __NOP();
  }
#else
  if (nand_logger_init_status != LOG_OK) {
    Error_Handler();
  }
#endif

  int ble_sync_init_status = BleSync_Init(&nand_logger);
  if (ble_sync_init_status != 0)
  {
    /* Logging remains available; BLE sync stays disabled until next reset. */
    LED_On(LED_RED);
  }

#if (NAND_FORCE_ERASE_ON_BOOT != 0U)
  if (NANDLogger_EraseAllGoodBlocks(&nand_logger) != LOG_OK)
  {
    Error_Handler();
  }
  if ((ble_sync_init_status == 0) &&
      (BleSync_StartNewLogGeneration() != 0))
  {
    LED_On(LED_RED);
  }
#endif

  if (NANDLogger_Recover(&nand_logger) != LOG_OK)
  {
    Error_Handler();
  }

  next_window_sequence = nand_recovered_next_window_sequence;


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
    as7341_available = 0U;
    for (uint8_t i = 0; i < 3; i++) {
      LED_Toggle(LED_RED); HAL_Delay(150);
      LED_Toggle(LED_RED); HAL_Delay(150);
    }
  } else {
    as7341_available = 1U;
    AS7341_ConfigTimingAndGain(AS7341_PROCESSING_ATIME,
                               AS7341_PROCESSING_ASTEP,
                               AS7341_PROCESSING_GAIN);
  }

  if ((storage_full_latched == 0U) && (ble_sync_init_status == 0))
  {
    LED_Off(LED_RED);
  }
  AudioScheduler_Init();

  /* USER CODE END 2 */
  mic_dma_config.Address    = (uint32_t)audio_dma_buffer;
  mic_dma_config.DataLength = AUDIO_DMA_BUFFER_SAMPLES * sizeof(int16_t);
  mic_dma_config.MsbOnly    = ENABLE;
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t main_loop_now_ms = HAL_GetTick();
    UserButton_Process(main_loop_now_ms);

    if (factory_erase_requested != 0U)
    {
      int factory_erase_result = SmartWearable_FactoryEraseNand();
      if (factory_erase_result < 0)
      {
        Error_Handler();
      }
      main_loop_now_ms = HAL_GetTick();
    }

    ProcessBleSync(main_loop_now_ms);
    AudioScheduler_Process(main_loop_now_ms);
    UpdateStateLed(current_state);

	  switch(current_state)
	  {
      case STATE_IDLE:

        if (start_acquisition_requested)
        {
          HAL_StatusTypeDef start_status;

          start_acquisition_requested = 0U;

          if ((storage_full_latched != 0U) ||
              (next_window_sequence == UINT32_MAX))
          {
            nand_storage_full = 1U;
            storage_full_latched = 1U;
            UpdateStateLed(current_state);
            break;
          }

          memset((void *)&mic_diag, 0, sizeof(mic_diag));
          mic_diag.audio_window_target_samples = AUDIO_WINDOW_TARGET_SAMPLES;
          mic_diag.session_start_tick_ms = HAL_GetTick();

          timestamp.hh = 0U;
          timestamp.mm = 0U;
          timestamp.ss = 0U;
          timestamp.sss = 0U;

          tim = 0U;
          sensor_tick_pending = 0U;
          memset(raw_light, 0, sizeof(raw_light));
          light_measurement_pending = 0U;
          light_session_start_ms = HAL_GetTick();
#if (LIGHT_STORE_RAW_LRAW != 0U)
          light_sample_index = 0U;
#endif
          light_samples_requested = 0U;
          light_samples_acquired = 0U;
          light_samples_saved = 0U;
          light_samples_discarded = 0U;

          audio_window_pcm_samples = 0U;
          AudioRing_Reset();
          microphone_active = 0U;
          stop_acquisition_requested = 0U;
          current_state = STATE_ACQUISITION;
          UpdateStateLed(current_state);

          mic_dma_config.Address    = (uint32_t)audio_dma_buffer;
          mic_dma_config.DataLength = AUDIO_DMA_BUFFER_SAMPLES * sizeof(int16_t);
          mic_dma_config.MsbOnly    = ENABLE;

          audio_accept_chunks = 1U;
          mic_diag.dma_start_attempt_count++;
          mic_diag.last_dma_start_tick_ms = HAL_GetTick();

          start_status = HAL_MDF_AcqStart_DMA(&MdfHandle0,
                                              &MdfFilterConfig0,
                                              &mic_dma_config);

          mic_diag.last_dma_start_status = (int32_t)start_status;
          MicDiagnostics_UpdateErrorCodes();

          if (start_status == HAL_OK)
          {
            mic_diag.dma_start_ok_count++;
            mic_diag.dma_session_start_ok_count++;
            microphone_active = 1U;
            current_window_sequence = next_window_sequence;
            next_window_sequence++;
            AudioScheduler_RecordWindowStart(mic_diag.last_dma_start_tick_ms);
            light_measurement_pending = 1U;
            light_measurements_requested++;
            HAL_TIM_Base_Start_IT(&htim2);
          }
          else
          {
            audio_accept_chunks = 0U;
            mic_diag.dma_start_error_count++;
            mic_diag.dma_session_start_error_count++;
            Error_Handler();
          }

          break;
        }

        HAL_TIM_Base_Stop_IT(&htim2);

        if (microphone_active)
        {
          HAL_StatusTypeDef stop_status;

          mic_diag.mdf_stop_attempt_count++;
          stop_status = HAL_MDF_AcqStop_DMA(&MdfHandle0);
          mic_diag.last_mdf_stop_status = (int32_t)stop_status;
          MicDiagnostics_UpdateErrorCodes();

          if (stop_status == HAL_OK)
          {
            mic_diag.mdf_stop_ok_count++;
          }
          else
          {
            mic_diag.mdf_stop_error_count++;
          }

          microphone_active = 0U;
        }

        audio_accept_chunks = 0U;

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

          if ((sensor_tick_pending > 0U) &&
                  (current_state == STATE_ACQUISITION) &&
                  (stop_acquisition_requested == 0U))
          {
            sensor_tick_pending--;
             ProcessSensorTick();
          }

          if (stop_acquisition_requested)
          {
            stop_acquisition_requested = 0U;
              StopAcquisition();
              break;
          }  

          break;

          case STATE_USB_CONNECTED:
          if (download_requested)
          {
            download_requested = 0U;
            current_state = STATE_DOWNLOAD;
          }
            break;

          case STATE_DOWNLOAD:
            if (NANDLogger_DownloadAll(&nand_logger) != LOG_OK)
          { 
            Error_Handler();
          }
            current_state = STATE_USB_CONNECTED;
            break;

          case STATE_BLE_SYNC:
            /* BleSync_Process() advances the transfer outside interrupt context. */
            break;

          case STATE_FACTORY_ERASE:
            /* SmartWearable_FactoryEraseNand() owns this synchronous state. */
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
    uint32_t now;

	if(GPIO_Pin != USER_BUTTON_Pin)
    {
        return;
    }

    if ((factory_erase_in_progress != 0U) ||
        (user_button_pressed != 0U) ||
        (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin) != GPIO_PIN_SET))
    {
        return;
    }

    now = HAL_GetTick();

    if ((now - user_button_last_event_ms) < USER_BUTTON_DEBOUNCE_MS)
    {
        return;
    }

    user_button_last_event_ms = now;
    user_button_pressed = 1U;
    user_button_release_pending = 0U;
    user_button_long_press_triggered = 0U;
    user_button_press_start_ms = now;
    user_button_press_state = current_state;
    button_rising_count++;
    debug_state_at_button = current_state;

    if (current_state == STATE_IDLE)
    {
        start_acquisition_requested = 0U;
        ble_sync_requested = 0U;
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
	if ((GPIO_Pin == USER_BUTTON_Pin) &&
        (factory_erase_in_progress == 0U) &&
        (user_button_pressed != 0U))
	{
        user_button_release_pending = 1U;
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
    HAL_Delay(700);
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
