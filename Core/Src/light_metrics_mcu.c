/*
 * light_metrics_mcu.c
 *
 * Session-level AS7341 processing:
 * - acquire complete F1..F8, NIR and Clear samples;
 * - apply configurable dark-count correction;
 * - accumulate valid samples only;
 * - compute normalized multispectral signature and Clear-based index.
 */

#include "light_metrics_mcu.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

#include "light_temperature.h"
#include "as7341_processing_config.h"
#include "main.h"

#define LIGHT_CHANNEL_COUNT 10U
#define LIGHT_SIGNATURE_CHANNEL_COUNT 9U
#define LIGHT_BLUE_CHANNEL_INDEX LIGHT_CH_F3

_Static_assert(sizeof(LightSensorResultRecord) == AS7341_LIGHT_RESULT_RECORD_BYTES,
               "Unexpected LightSensorResultRecord size");
_Static_assert(offsetof(LightSensorResultRecord, blue_clear_ratio) == 36U,
               "Unexpected blue_clear_ratio offset");
_Static_assert(offsetof(LightSensorResultRecord, light_level_class) == 38U,
               "Unexpected light_level_class offset");

typedef enum
{
    LIGHT_CH_F1 = 0,
    LIGHT_CH_F2,
    LIGHT_CH_F3,
    LIGHT_CH_F4,
    LIGHT_CH_F5,
    LIGHT_CH_F6,
    LIGHT_CH_F7,
    LIGHT_CH_F8,
    LIGHT_CH_NIR,
    LIGHT_CH_CLEAR
} LightChannelIndex;

static const uint16_t s_dark_counts[LIGHT_CHANNEL_COUNT] =
{
    AS7341_DARK_F1_COUNTS,
    AS7341_DARK_F2_COUNTS,
    AS7341_DARK_F3_COUNTS,
    AS7341_DARK_F4_COUNTS,
    AS7341_DARK_F5_COUNTS,
    AS7341_DARK_F6_COUNTS,
    AS7341_DARK_F7_COUNTS,
    AS7341_DARK_F8_COUNTS,
    AS7341_DARK_NIR_COUNTS,
    AS7341_DARK_CLEAR_COUNTS
};

static uint64_t s_sum[LIGHT_CHANNEL_COUNT];
static uint32_t s_sample_count;
static uint32_t s_session_start_ms;
static uint32_t s_session_stop_ms;
static uint32_t s_last_sample_ms;
static uint32_t s_error_count;
static uint32_t s_overflow_drop_count;
static uint8_t s_session_active;
static uint8_t s_pending_samples;

static uint8_t s_async_active = 0U;
static AS7341_Spectrum s_async_spectrum;

static int s_light_temps[SAMPLE_SIZE];
static uint16_t s_light_temp_count = 0;

static uint16_t light_subtract_dark(uint16_t raw, uint16_t dark)
{
    return (raw > dark) ? (uint16_t)(raw - dark) : 0U;
}

static uint8_t light_accumulate_sample(const uint16_t corrected[LIGHT_CHANNEL_COUNT])
{
    for (uint8_t i = 0U; i < LIGHT_CHANNEL_COUNT; i++)
    {
        if ((UINT64_MAX - s_sum[i]) < corrected[i])
        {
            s_overflow_drop_count++;
            return 0U;
        }
    }

    for (uint8_t i = 0U; i < LIGHT_CHANNEL_COUNT; i++)
    {
        s_sum[i] += corrected[i];
    }

    s_sample_count++;
    return 1U;
}

static void light_compute_means(uint32_t mean[LIGHT_CHANNEL_COUNT])
{
    for (uint8_t i = 0U; i < LIGHT_CHANNEL_COUNT; i++)
    {
        mean[i] = 0U;
    }

    if (s_sample_count == 0U)
    {
        return;
    }

    for (uint8_t i = 0U; i < LIGHT_CHANNEL_COUNT; i++)
    {
        mean[i] = (uint32_t)(s_sum[i] / s_sample_count);
    }
}

void LightMetrics_Init(void)
{
    AS7341_ConfigTimingAndGain(AS7341_PROCESSING_ATIME,
                               AS7341_PROCESSING_ASTEP,
                               AS7341_PROCESSING_GAIN);
    LightMetrics_ResetSession();
}

void LightMetrics_StartSession(uint32_t session_start_ms)
{
    LightMetrics_ResetSession();
    s_session_active = 1U;
    s_session_start_ms = session_start_ms;
    s_session_stop_ms = session_start_ms;
    s_last_sample_ms = session_start_ms - AS7341_PROCESSING_MIN_PERIOD_MS;
}

void LightMetrics_RequestSample(void)
{
    if (s_session_active != 0U)
    {
        s_pending_samples = 1U;
    }
}

void LightMetrics_ProcessPendingSample(void)
{
    AS7341_Spectrum spectrum;
    uint16_t corrected[LIGHT_CHANNEL_COUNT];
    uint32_t now_ms;

    if ((s_session_active == 0U) || (s_pending_samples == 0U))
    {
        return;
    }

    now_ms = HAL_GetTick();
    if (s_async_active == 0U) 
    {
        if ((uint32_t)(now_ms - s_last_sample_ms) < AS7341_PROCESSING_MIN_PERIOD_MS)
        {
            return;
        }

        if (AS7341_StartFullSpectrumAsync(&s_async_spectrum) == 0U)
        {
            s_error_count++;
            s_pending_samples = 0U;
            return;
        }
        s_async_active = 1U;
    }

    AS7341_AsyncResult res = AS7341_ProcessFullSpectrumAsync(now_ms);
    if (res == AS7341_ASYNC_BUSY) 
    {
        return;
    } 
    else if (res == AS7341_ASYNC_ERROR) 
    {
        s_error_count++;
        s_async_active = 0U;
        s_pending_samples = 0U;
        return;
    }

    s_async_active = 0U;
    s_pending_samples = 0U;
    spectrum = s_async_spectrum;

    /*
     * Driver mapping:
     *   SMUX F1F4_Clear_NIR: CH0..CH5 -> F1,F2,F3,F4,Clear,NIR
     *                        stored in spectrum.ch[0..5]
     *   SMUX F5F8_Clear_NIR: CH0..CH5 -> F5,F6,F7,F8,Clear,NIR
     *                        stored in spectrum.ch[6..11]
     */
    corrected[LIGHT_CH_F1] = light_subtract_dark(spectrum.ch[0], s_dark_counts[LIGHT_CH_F1]);
    corrected[LIGHT_CH_F2] = light_subtract_dark(spectrum.ch[1], s_dark_counts[LIGHT_CH_F2]);
    corrected[LIGHT_CH_F3] = light_subtract_dark(spectrum.ch[2], s_dark_counts[LIGHT_CH_F3]);
    corrected[LIGHT_CH_F4] = light_subtract_dark(spectrum.ch[3], s_dark_counts[LIGHT_CH_F4]);
    corrected[LIGHT_CH_F5] = light_subtract_dark(spectrum.ch[4], s_dark_counts[LIGHT_CH_F5]);
    corrected[LIGHT_CH_F6] = light_subtract_dark(spectrum.ch[5], s_dark_counts[LIGHT_CH_F6]);
    corrected[LIGHT_CH_F7] = light_subtract_dark(spectrum.ch[6], s_dark_counts[LIGHT_CH_F7]);
    corrected[LIGHT_CH_F8] = light_subtract_dark(spectrum.ch[7], s_dark_counts[LIGHT_CH_F8]);
    
    corrected[LIGHT_CH_CLEAR] = light_subtract_dark(spectrum.ch[8], s_dark_counts[LIGHT_CH_CLEAR]);
    corrected[LIGHT_CH_NIR]   = light_subtract_dark(spectrum.ch[9], s_dark_counts[LIGHT_CH_NIR]);

    if (light_accumulate_sample(corrected) != 0U)
    {
        s_last_sample_ms = now_ms;
        if (s_light_temp_count < SAMPLE_SIZE) {
            s_light_temps[s_light_temp_count++] = get_light_temperature((uint16_t*)corrected);
        }
    }
}

void LightMetrics_StopSession(uint32_t stop_ms)
{
    s_session_active = 0U;
    s_pending_samples = 0U;
    s_async_active = 0U;
    s_session_stop_ms = stop_ms;
}

void LightMetrics_FinalizeSession(LightSensorResultRecord *result)
{
    uint32_t mean[LIGHT_CHANNEL_COUNT];
    uint32_t max_mean = 0U;

    if (result == NULL)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    result->format_version = AS7341_LIGHT_RECORD_FORMAT_VERSION;
    result->sample_count = s_sample_count;
    result->session_start_ms = s_session_start_ms;
    result->acquisition_duration_ms = (uint32_t)(s_session_stop_ms - s_session_start_ms);

    light_compute_means(mean);
    result->clear_mean_counts = mean[LIGHT_CH_CLEAR];
    result->light_level_class = (uint8_t)AS7341_ClassifyAmbientLight(mean[LIGHT_CH_CLEAR]);

    if (mean[LIGHT_CH_CLEAR] != 0U)
    {
        result->blue_clear_ratio =
            (uint16_t)((((uint64_t)mean[LIGHT_BLUE_CHANNEL_INDEX] * AS7341_NORMALIZATION_SCALE) +
                        (mean[LIGHT_CH_CLEAR] / 2U)) /
                       mean[LIGHT_CH_CLEAR]);
    }

    if (s_light_temp_count > 0) {
        result->color_temp = (uint16_t)get_avg_light_temperature(s_light_temps);
    } else {
        result->color_temp = 0;
    }
    s_light_temp_count = 0;

    for (uint8_t i = 0U; i < LIGHT_SIGNATURE_CHANNEL_COUNT; i++)
    {
        if (mean[i] > max_mean)
        {
            max_mean = mean[i];
        }
    }

    if (max_mean == 0U)
    {
        return;
    }

    for (uint8_t i = 0U; i < LIGHT_SIGNATURE_CHANNEL_COUNT; i++)
    {
        result->normalized[i] =
            (uint16_t)((((uint64_t)mean[i] * AS7341_NORMALIZATION_SCALE) +
                        (max_mean / 2U)) /
                       max_mean);
    }
}

void LightMetrics_ResetSession(void)
{
    memset(s_sum, 0, sizeof(s_sum));
    s_sample_count = 0U;
    s_session_start_ms = 0U;
    s_session_stop_ms = 0U;
    s_last_sample_ms = 0U;
    s_error_count = 0U;
    s_overflow_drop_count = 0U;
    s_session_active = 0U;
    s_pending_samples = 0U;
}

uint8_t LightMetrics_HasPendingSample(void)
{
    return s_pending_samples;
}

uint8_t LightMetrics_IsSessionActive(void)
{
    return s_session_active;
}

uint32_t LightMetrics_GetLastErrorCount(void)
{
    return s_error_count;
}

uint32_t LightMetrics_GetOverflowDropCount(void)
{
    return s_overflow_drop_count;
}

static uint32_t Light_GetReturnThreshold(uint32_t threshold) {
    return threshold - (threshold / 20); // 5% margin
}

BleLiveLightExposureClass AS7341_ClassifyAmbientLight(uint32_t clear_mean_counts)
{
    static BleLiveLightExposureClass s_current_class = LIGHT_DARK;
    
    BleLiveLightExposureClass raw_up = LIGHT_DARK;
    if (clear_mean_counts >= 1500) raw_up = LIGHT_VERY_HIGH_EXPOSURE;
    else if (clear_mean_counts >= 500) raw_up = LIGHT_HIGH_EXPOSURE;
    else if (clear_mean_counts >= 80) raw_up = LIGHT_MODERATE_EXPOSURE;
    else if (clear_mean_counts >= 10) raw_up = LIGHT_LOW_EXPOSURE;
    
    BleLiveLightExposureClass raw_down = LIGHT_VERY_HIGH_EXPOSURE;
    if (clear_mean_counts < Light_GetReturnThreshold(10)) raw_down = LIGHT_DARK;
    else if (clear_mean_counts < Light_GetReturnThreshold(80)) raw_down = LIGHT_LOW_EXPOSURE;
    else if (clear_mean_counts < Light_GetReturnThreshold(500)) raw_down = LIGHT_MODERATE_EXPOSURE;
    else if (clear_mean_counts < Light_GetReturnThreshold(1500)) raw_down = LIGHT_HIGH_EXPOSURE;
    
    if (raw_up > s_current_class) {
        s_current_class = raw_up;
    } else if (raw_down < s_current_class) {
        s_current_class = raw_down;
    }
    
    return s_current_class;
}
