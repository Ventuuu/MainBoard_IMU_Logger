#include "mic_metrics.h"
#include <math.h>

/*
 * MP34DT06J Datasheet Calibration Constants:
 * AOP (Acoustic Overload Point) = 122.5 dBSPL 
 * Therefore, 0 dBFS digital maximum maps to 122.5 dBSPL physically.
 */
#define MIC_OFFSET_TO_SPL  122.5f

// Calculates both metrics in a single pass
void MicMetrics_ProcessFrame(int16_t* pcm_data, uint16_t length, float* out_dbfs, float* out_dbspl) {
    if (length == 0) return;

    // 1. Calculate the DC Offset (The resting "silence" bias)
    float sum = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        sum += (float)pcm_data[i];
    }
    float dc_offset = sum / (float)length;
    
    // 2. Subtract the DC bias from every sample, square it, and sum it
    float sum_of_squares = 0.0f;
    for (uint16_t i = 0; i < length; i++) {
        // Isolate the actual acoustic AC wave!
        float ac_sample = (float)pcm_data[i] - dc_offset; 
        sum_of_squares += (ac_sample * ac_sample);
    }
    
    // 3. Calculate the Mean (average) of the pure acoustic energy
    float mean_square = sum_of_squares / (float)length;
    
    // Prevent log10(0) crash in absolute silence
    if (mean_square < 1.0f) {
        mean_square = 1.0f; 
    }

    // 4. Calculate the Root
    float rms = sqrtf(mean_square);
    
    // 5. Convert to Logarithmic Decibel scale (dBFS)
    // 32768 is the maximum absolute value of a signed 16-bit integer
    *out_dbfs = 20.0f * log10f(rms / 32768.0f); 
    
    // 6. Apply the hardware-specific calibration offset for SPL
    *out_dbspl = *out_dbfs + MIC_OFFSET_TO_SPL;
    
    // 7. Clamp values to prevent UI layout breaks or integer overflow
    if (*out_dbfs < -100.0f) *out_dbfs = -100.0f;
    if (*out_dbspl < 0.0f) *out_dbspl = 0.0f;
    if (*out_dbspl > 122.5f) *out_dbspl = 122.5f; // Clamped to physical limit
}