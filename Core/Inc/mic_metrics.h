#ifndef __MIC_METRICS_H
#define __MIC_METRICS_H

#include <stdint.h>

/**
 * @brief Calculates both the raw Decibel Full Scale (dBFS) and calibrated Sound Pressure Level (dBSPL) values.
 * Returns negative values for dBFS (e.g., -40.0 dBFS) and positive values for dBSPL (e.g., 65.0 dBSPL).
 * 0 dBFS is the absolute maximum clipping point of the hardware.
 */
void MicMetrics_ProcessFrame(int32_t* pcm_data, uint16_t length, float* out_dbfs, float* out_dbspl);
#endif /* __MIC_METRICS_H */