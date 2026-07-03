#include "app_audio.h"
#include "main.h"

static void AudioRing_Reset(void)
{
    audio_accept_chunks = 0U;
    audio_ring_head = 0U;
    audio_ring_tail = 0U;
    __DMB();
}

static void MicrophoneClock_Enable(void)
{
    MDF1->CKGCR |= MDF_CKGCR_CKDEN;
    MdfHandle0.Instance->SITFCR |= MDF_SITFCR_SITFEN;
}

static void MicrophoneClock_Disable(void)
{
    MdfHandle0.Instance->SITFCR &= ~MDF_SITFCR_SITFEN;
    MDF1->CKGCR &= ~MDF_CKGCR_CKDEN;
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

static uint32_t AudioRing_Count(void)
{
    uint32_t head = audio_ring_head;
    uint32_t tail = audio_ring_tail;

    return AudioRing_CountFrom(head, tail);
}

static uint32_t AudioRing_CountFrom(uint32_t head, uint32_t tail)
{
    if (head >= tail)
    {
        return head - tail;
    }

    return (AUDIO_RING_SLOT_COUNT - tail) + head;
}

static void AudioRing_UpdateHighWatermark(uint32_t count)
{
    if (count > mic_diag.audio_ring_high_watermark)
    {
        mic_diag.audio_ring_high_watermark = count;
    }
    if (count > audio_diag_ring_max_occupancy)
    {
        audio_diag_ring_max_occupancy = count;
    }
}

static void AudioRing_EnqueueFromIsr(const int16_t *samples)
{
    uint32_t head;
    uint32_t tail;
    uint32_t next_head;

    if (current_state != STATE_ACQUISITION)
    {
        audio_diag_stale_callbacks++;
        return;
    }

    if (audio_accept_chunks == 0U)
    {
        audio_diag_stale_callbacks++;
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
        mic_overrun_count++;
        audio_diag_ring_overflows++;
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
    uint32_t skipped_samples;
    uint32_t available_samples;
    uint32_t window_offset;
    const int16_t *valid_samples;

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

    skipped_samples = (audio_software_warmup_remaining < AUDIO_CHUNK_SAMPLES) ?
                      audio_software_warmup_remaining : AUDIO_CHUNK_SAMPLES;
    audio_software_warmup_remaining -= skipped_samples;
    mic_warmup_samples_discarded += skipped_samples;
    audio_diag_warmup_discarded_samples += skipped_samples;

    available_samples = AUDIO_CHUNK_SAMPLES - skipped_samples;
    valid_samples = &audio_ring[tail].samples[skipped_samples];

    if (mic_diag.audio_window_samples_accepted < mic_diag.audio_window_target_samples)
    {
        remaining_samples = mic_diag.audio_window_target_samples -
                            mic_diag.audio_window_samples_accepted;
        accepted_samples = (remaining_samples < available_samples) ?
                           remaining_samples : available_samples;
    }
    else
    {
        accepted_samples = 0U;
    }

    discarded_samples = available_samples - accepted_samples;

    if (accepted_samples > 0U)
    {
        window_offset = audio_window_pcm_samples;
        if ((window_offset > AUDIO_WINDOW_TARGET_SAMPLES) ||
            (accepted_samples > (AUDIO_WINDOW_TARGET_SAMPLES - window_offset)))
        {
            return LOG_ERR_BAD_ARGUMENT;
        }

        memcpy(&audio_window_pcm[window_offset],
               valid_samples,
               accepted_samples * sizeof(int16_t));

        if (window_offset == 0U)
        {
            audio_diag_first_valid_sample = valid_samples[0];
        }
        audio_diag_last_valid_sample = valid_samples[accepted_samples - 1U];

        mic_diag.audio_window_samples_accepted += accepted_samples;
        audio_window_pcm_samples = window_offset + accepted_samples;
        mic_valid_samples = audio_window_pcm_samples;
        audio_diag_valid_samples = audio_window_pcm_samples;

#if (AUDIO_STORE_RAW_PCM != 0U)
        mic_diag.nand_append_attempt_count++;
        mic_diag.page_sequence_before_last_append = nand_logger.page_sequence;

        append_status = NANDLogger_AppendAudioBuffer(
                &nand_logger,
                valid_samples,
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

static uint32_t Audio_Crc32(const int16_t *samples, uint32_t sample_count)
{
#if (AUDIO_ENABLE_WINDOW_DIAGNOSTICS != 0U)
    const uint8_t *bytes = (const uint8_t *)samples;
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t byte_count = sample_count * sizeof(int16_t);

    for (uint32_t i = 0U; i < byte_count; i++)
    {
        if ((i & 0x7FU) == 0U)
        {
            App_ServiceTimeCriticalTasks();
        }
        crc ^= bytes[i];
        for (uint32_t bit = 0U; bit < 8U; bit++)
        {
            uint32_t mask = 0U - (crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
#else
    (void)samples;
    (void)sample_count;
    return 0U;
#endif
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
        if ((i & 0x7FU) == 0U)
        {
            App_ServiceTimeCriticalTasks();
        }
        sample_sum += samples[i];
    }

    features.mean_counts = (double)sample_sum / (double)sample_count;

    for (uint32_t i = 0U; i < sample_count; i++)
    {
        int32_t sample = samples[i];
        uint32_t magnitude = (sample < 0) ? (uint32_t)(-sample) : (uint32_t)sample;
        double centered = (double)sample - features.mean_counts;

        if ((i & 0x7FU) == 0U)
        {
            App_ServiceTimeCriticalTasks();
        }

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

    /* Each one-second window is independent, so all biquad states start at zero. */
    for (uint32_t i = 0U; i < sample_count; i++)
    {
        double section_input = (double)samples[i] - mean_counts;

        if ((i & 0x7FU) == 0U)
        {
            App_ServiceTimeCriticalTasks();
        }

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

    if (estimated_laeq_dba < 40.0) return AUDIO_ENV_VERY_QUIET;
    if (estimated_laeq_dba < 45.0) return AUDIO_ENV_QUIET;
    if (estimated_laeq_dba < 55.0) return AUDIO_ENV_MODERATE;
    if (estimated_laeq_dba < 65.0) return AUDIO_ENV_LIVELY;
    if (estimated_laeq_dba < 70.0) return AUDIO_ENV_NOISY;
    if (estimated_laeq_dba < 85.0) return AUDIO_ENV_VERY_NOISY;

    return AUDIO_ENV_HIGH_EXPOSURE;
}

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

    audio_diag_pcm_crc32 =
            (features.sample_count == AUDIO_WINDOW_TARGET_SAMPLES) ?
            Audio_Crc32(audio_window_pcm, features.sample_count) : 0U;
    audio_diag_rms_z_dbfs = features.rms_zero_mean_dbfs;
    audio_diag_rms_a_dbfs = features.a_weighted_rms_dbfs;
    audio_diag_peak_dbfs = features.peak_dbfs;
    audio_diag_laeq_dba = features.estimated_laeq_dba;
    audio_diag_environment_class = features.environment_class;

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
        nand_write_error_count++;
        LED_On(LED_RED);
    }
    else
    {
        afea_records_written++;
    }
#endif
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

    acquisition_epoch_count = 0U;
    acquisition_epoch_start_tick = now_ms;
    acquisition_deadline_miss_count = 0U;
    acquisition_timebase_ms = now_ms;
    imu_next_sample_tick_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
    imu_samples_current_epoch = 0U;
    imu_samples_total = 0U;
    imu_missed_samples = 0U;
    imu_buffer_overrun_count = 0U;

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
        acquisition_epoch_start_tick = audio_scheduler_next_deadline_ms;
        acquisition_epoch_count++;
        imu_samples_current_epoch = 0U;
        start_acquisition_requested = 1U;
        start_request_count++;
    }
    else
    {
        audio_windows_missed += due_deadlines;
        acquisition_deadline_miss_count += due_deadlines;
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
    sensor_tick_pending = 0U;
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
    sensor_tick_pending = 0U;
    imu_next_sample_tick_ms = now_ms + IMU_SAMPLE_PERIOD_MS;
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

