#include "app_sensor_workflow.h"
#include "main.h"

static void LiveMode_Start(uint32_t now_ms)
{
    live_mode_requested = 0U;
    live_mode_stop_requested = 0U;
    live_mode_active = 1U;
    SensorSuperframe_Init(now_ms);
}

static void LiveMode_Stop(void)
{
    live_mode_requested = 0U;
    live_mode_stop_requested = 0U;
    live_mode_active = 0U;
    SensorPhase_StopImuRun();
    SensorPhase_StopMicWindow();
    light_measurement_pending = 0U;
    combined_metrics_pending = 0U;
    env_start_done = 0U;
    env_end_done = 0U;
}

static void SensorSuperframe_Init(uint32_t now_ms)
{
    acquisition_timebase_ms = now_ms; //SensorPhase_EnterEnvStart(now_ms);
    imu_next_sample_tick_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
    sensor_tick_pending = 0U;
    SensorPhase_EnterEnvStart(now_ms); 
}

static void SensorSuperframe_Process(uint32_t now_ms)
{
    uint32_t cycle_elapsed_ms = now_ms - sensor_cycle_start_ms;

    switch (sensor_phase)
    {
    case SENSOR_PHASE_ENV_START:
    if ((env_start_done == 0U) &&
    ((now_ms - sensor_phase_start_ms) >= SENSOR_ENV_START_MS))
    {
        SensorPhase_EnterImuRun(now_ms);
    }
    break;

    case SENSOR_PHASE_IMU_RUN:
    if (imu_window_active != 0U)
    {
        SensorPhase_TrySendStepBle(now_ms);
    }
    if (cycle_elapsed_ms >= SENSOR_IMU_END_MS)
    {
        SensorPhase_EnterEnvEnd(now_ms);
    }
    break;

    case SENSOR_PHASE_ENV_END:
    if ((combined_metrics_pending != 0U) &&
    (SensorPhase_IsEnvMetricsReady() != 0U))
    {
        combined_metrics_pending = 0U;
        env_end_done = 1U;
        /* TODO: replace with real BLE live combined notification call. */
    }
    if (cycle_elapsed_ms >= SENSOR_EPOCH_MS)
    {
        SensorPhase_EnterEnvStart(now_ms);
    }
    break;

    default:
    SensorPhase_EnterEnvStart(now_ms);
    break;
    }
}

static void SensorPhase_EnterEnvStart(uint32_t now_ms)
{
    sensor_phase = SENSOR_PHASE_ENV_START;
    sensor_phase_start_ms = now_ms;
    sensor_cycle_start_ms = now_ms;
    env_start_done = 0U;
    env_end_done = 0U;
    combined_metrics_pending = 0U;
    imu_window_active = 0U;
    SensorPhase_StopImuRun();
    SensorPhase_StartMicWindow();
    SensorPhase_RequestLightMeasurement();
}

static void SensorPhase_EnterImuRun(uint32_t now_ms)
{
    sensor_phase = SENSOR_PHASE_IMU_RUN;
    sensor_phase_start_ms = now_ms;
    last_step_ble_tx_ms = now_ms;
    env_start_done = 1U;
    SensorPhase_StopMicWindow(); //    light_measurement_pending = 0U;
    imu_window_active = 1U;
    imu_next_sample_tick_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
}

static void SensorPhase_EnterEnvEnd(uint32_t now_ms)
{
    sensor_phase = SENSOR_PHASE_ENV_END;
    sensor_phase_start_ms = now_ms;
    env_end_done = 0U;
    combined_metrics_pending = 1U;
    SensorPhase_StopImuRun();
    SensorPhase_StartMicWindow();
    SensorPhase_RequestLightMeasurement();
}

static void SensorPhase_StopImuRun(void)
{
    imu_window_active = 0U;
    sensor_tick_pending = 0U;
}

static void SensorPhase_StartMicWindow(void)
{
HAL_StatusTypeDef start_status;

if (microphone_active != 0U)
{
    return;
}

memset((void *)&mic_diag, 0, sizeof(mic_diag));
mic_diag.audio_window_target_samples = AUDIO_WINDOW_TARGET_SAMPLES;
mic_diag.session_start_tick_ms = HAL_GetTick();
audio_software_warmup_remaining = AUDIO_SOFTWARE_WARMUP_SAMPLES;
audio_diag_pcm_sample_rate_hz = AUDIO_SAMPLE_RATE_HZ;
audio_diag_target_valid_samples = AUDIO_WINDOW_TARGET_SAMPLES;
audio_diag_captured_samples = AUDIO_HARDWARE_WARMUP_SAMPLES;
audio_diag_valid_samples = 0U;
audio_diag_warmup_target_samples = AUDIO_WARMUP_SAMPLES;
audio_diag_warmup_discarded_samples = AUDIO_HARDWARE_WARMUP_SAMPLES;
audio_diag_dma_half_callbacks = 0U;
audio_diag_dma_full_callbacks = 0U;
audio_diag_stale_callbacks = 0U;
audio_diag_ring_overflows = 0U;
audio_diag_ring_max_occupancy = 0U;
audio_diag_first_valid_sample = 0;
audio_diag_last_valid_sample = 0;
audio_diag_pcm_crc32 = 0U;
audio_diag_rms_z_dbfs = 0.0;
audio_diag_rms_a_dbfs = 0.0;
audio_diag_peak_dbfs = 0.0;
audio_diag_laeq_dba = 0.0;
audio_diag_environment_class = AUDIO_ENV_UNAVAILABLE;
audio_diag_mdf_start_count = 0U;
audio_diag_mdf_stop_count = 0U;
audio_diag_start_failures = 0U;
audio_diag_stop_failures = 0U;
audio_window_pcm_samples = 0U;
AudioRing_Reset();
mic_capture_samples = AUDIO_HARDWARE_WARMUP_SAMPLES;
mic_valid_samples = 0U;
mic_warmup_samples_discarded = AUDIO_HARDWARE_WARMUP_SAMPLES;

mic_dma_config.Address = (uint32_t)audio_dma_buffer;
mic_dma_config.DataLength = AUDIO_DMA_BUFFER_SAMPLES * sizeof(int16_t);
mic_dma_config.MsbOnly = ENABLE;

MicrophoneClock_Enable();
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
    audio_diag_mdf_start_count++;
    microphone_active = 1U;
    mic_active = 1U;
    mic_window_count++;
    }
else
    {
    audio_accept_chunks = 0U;
    mic_diag.dma_start_error_count++;
    mic_diag.dma_session_start_error_count++;
    audio_diag_start_failures++;
    LED_On(LED_RED);
    }
}

static void SensorPhase_StopMicWindow(void)
{
    HAL_StatusTypeDef stop_status;

    audio_accept_chunks = 0U;

    if (microphone_active == 0U)
    {
        MicrophoneClock_Disable();
        return;
    }

    mic_diag.mdf_stop_attempt_count++;
    stop_status = HAL_MDF_AcqStop_DMA(&MdfHandle0);
    mic_diag.last_mdf_stop_status = (int32_t)stop_status;
    MicDiagnostics_UpdateErrorCodes();

    if (stop_status == HAL_OK)
    {
        mic_diag.mdf_stop_ok_count++;
        mic_diag.dma_session_stop_ok_count++;
        audio_diag_mdf_stop_count++;
    }
    else
    {
        mic_diag.mdf_stop_error_count++;
        mic_diag.dma_session_stop_error_count++;
        audio_diag_stop_failures++;
    }

    microphone_active = 0U;
    mic_active = 0U;
    MicrophoneClock_Disable();

    mic_diag.audio_ring_count_at_stop = AudioRing_Count();
    if (Audio_DrainQueuedChunks(HAL_GetTick()) != LOG_OK)
    {
        nand_write_error_count++;
        LED_On(LED_RED);
    }

    mic_valid_samples = mic_diag.audio_window_samples_accepted;
    mic_last_window_duration_ms = HAL_GetTick() - mic_diag.last_dma_start_tick_ms;
    Audio_PublishBasicFeatures(0U, LOG_OK);
}

static void SensorPhase_RequestLightMeasurement(void)
{
    if ((light_measurement_pending == 0U) && (light_active == 0U))
    {
        light_measurement_pending = 1U;
        light_measurements_requested++;
    }
}

static uint8_t SensorPhase_IsEnvMetricsReady(void)
{
const uint8_t light_ready =
    ((light_active == 0U) && (light_measurement_pending == 0U) &&
    (light_measurement_count > 0U)) ? 1U : 0U;
const uint8_t mic_ready =
    ((microphone_active == 0U) &&
    (audio_basic_features_computed > 0U)) ? 1U : 0U;

return (uint8_t)((light_ready != 0U) && (mic_ready != 0U));
}

static void SensorPhase_TrySendStepBle(uint32_t now_ms)
{
    if ((now_ms - last_step_ble_tx_ms) < SENSOR_STEP_BLE_PERIOD_MS)
    {
        return;
    }

    last_step_ble_tx_ms = now_ms;
    // TODO: replace with real BLE live step notification call.
}

