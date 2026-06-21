#include "mic_metrics.h"
#include <math.h>

/*
 * MP34DT06J Datasheet Calibration Constants:
 * Sensitivity = -26 dBFS at 94 dBSPL (1 kHz)
 * Therefore, the offset to map the digital dBFS to physical dBSPL is:
 * 94 dBSPL - (-26 dBFS) = +120.0f
 */
#define MIC_OFFSET_TO_SPL  120.0f

float MicMetrics_CalculateNoise_dBFS(int16_t* pcm_data, uint16_t length) {
    float sum_of_squares = 0.0f;
    
    // 1. Square every sample and sum them up
    for (uint16_t i = 0; i < length; i++) {
        float sample = (float)pcm_data[i]; 
        sum_of_squares += (sample * sample);
    }
    
    // 2. Calculate the Mean (average)
    float mean_square = sum_of_squares / (float)length;
    
    // 3. Calculate the Root
    float rms = sqrtf(mean_square);
    
    // 4. Convert to Logarithmic Decibel scale
    float dbfs_level = -100.0f; // Default absolute silence floor
    if (rms > 0) {
        // 32768 is the maximum absolute value of a signed 16-bit integer
        dbfs_level = 20.0f * log10f(rms / 32768.0f); 
    }
    
    return dbfs_level;
}

float MicMetrics_CalculateNoise_dBSPL(int16_t* pcm_data, uint16_t length) {
    // 1. Get the raw digital dBFS value
    float dbfs = MicMetrics_CalculateNoise_dBFS(pcm_data, length);
    
    // 2. Apply the hardware-specific calibration offset
    float dbspl = dbfs + MIC_OFFSET_TO_SPL;
    
    // 3. Clamp the lower bound (e.g., a completely silent anechoic chamber is ~0 dBSPL)
    if (dbspl < 0.0f) {
        dbspl = 0.0f;
    }
    
    return dbspl;
}