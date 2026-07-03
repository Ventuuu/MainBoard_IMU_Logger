#include "app_system_ops.h"
#include "main.h"

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
        mic_active = 0U;
    }
    MicrophoneClock_Disable();

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
    acquisition_timebase_ms = HAL_GetTick();
    imu_next_sample_tick_ms = acquisition_timebase_ms + IMU_SAMPLE_PERIOD_MS;
    sensor_tick_pending = 0U;
    (void)HAL_TIM_Base_Start_IT(&htim2);

    factory_erase_duration_ms = HAL_GetTick() - erase_start_ms;
    factory_erase_completed_count++;
    factory_erase_in_progress = 0U;
    current_state = STATE_IDLE;
    state_led_initialized = 0U;
    UpdateStateLed(current_state);

    return 0;
}

