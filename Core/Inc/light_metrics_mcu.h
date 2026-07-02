/**
 * @file light_metrics_mcu.h
 * @brief Session-level AS7341 processing for normalized spectral results.
 */

#ifndef INC_LIGHT_METRICS_MCU_H_
#define INC_LIGHT_METRICS_MCU_H_

#include <stdint.h>

#include "as7341_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LIGHT_LEVEL_DARK = 0,
    LIGHT_LEVEL_LOW,
    LIGHT_LEVEL_NORMAL_INDOOR,
    LIGHT_LEVEL_BRIGHT,
    LIGHT_LEVEL_OUTDOOR,
    LIGHT_LEVEL_DIRECT_SUN
} LightLevelClass;

typedef struct
{
    uint16_t format_version;
    uint16_t normalized[9];
    uint32_t clear_mean_counts;
    uint32_t sample_count;
    uint32_t acquisition_duration_ms;
    uint32_t session_start_ms;
    uint16_t blue_clear_ratio;
    uint8_t light_level_class;
} LightSensorResultRecord;

void LightMetrics_Init(void);
void LightMetrics_StartSession(uint32_t session_start_ms);
void LightMetrics_RequestSample(void);
void LightMetrics_ProcessPendingSample(void);
void LightMetrics_StopSession(uint32_t stop_ms);
void LightMetrics_FinalizeSession(LightSensorResultRecord *result);
void LightMetrics_ResetSession(void);

uint8_t LightMetrics_HasPendingSample(void);
uint8_t LightMetrics_IsSessionActive(void);
uint32_t LightMetrics_GetLastErrorCount(void);
uint32_t LightMetrics_GetOverflowDropCount(void);

LightLevelClass AS7341_ClassifyAmbientLight(uint32_t clear_mean_counts);

#ifdef __cplusplus
}
#endif

#endif /* INC_LIGHT_METRICS_MCU_H_ */
