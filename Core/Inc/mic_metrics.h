#ifndef __MIC_METRICS_H
#define __MIC_METRICS_H

#include <stdint.h>

/**
 * @brief Calculates the raw Decibel Full Scale (dBFS) value.
 * Returns negative values (e.g., -40.0 dBFS). 
 * 0 dBFS is the absolute maximum clipping point of the hardware.
 */
float MicMetrics_CalculateNoise_dBFS(int16_t* pcm_data, uint16_t length);

/**
 * @brief Calculates the calibrated Sound Pressure Level (dBSPL).
 * Returns positive values (e.g., 65.0 dBSPL for normal conversation).
 * Calibrated specifically for the MP34DT06J MEMS microphone.
 */
float MicMetrics_CalculateNoise_dBSPL(int16_t* pcm_data, uint16_t length);

#endif /* __MIC_METRICS_H */