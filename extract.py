import sys

with open('d:/MainBoard_Logger/Core/Src/main_new.c', 'r') as f:
    lines = f.readlines()

start_idx_biquads = 0
for i, l in enumerate(lines):
    if l.startswith("typedef struct") and "double b0;" in lines[i+2]:
        start_idx_biquads = i
        break
end_idx_biquads = 0
for i in range(start_idx_biquads, len(lines)):
    if "static int16_t audio_dma_buffer" in lines[i]:
        end_idx_biquads = i
        break

start_idx_func = 1480  # static AudioBasicFeatureDebug Audio_ComputeBasicFeatures
end_idx_func = 1680
for i in range(1645, len(lines)):
    if "#if (AUDIO_STORE_FEATURE_RECORD != 0U)" in lines[i] or "static AudioFeatureRecordV1" in lines[i]:
        end_idx_func = i
        break

with open('d:/MainBoard_Logger/Core/Src/mic_metrics.c', 'w') as out:
    out.write('#include "mic_metrics.h"\n')
    out.write('#include <math.h>\n')
    out.write('#include "main.h"\n\n')
    out.write("".join(lines[start_idx_biquads:end_idx_biquads]))
    out.write('\n')
    out.write("".join(lines[start_idx_func:end_idx_func]))
    out.write('\n')
    out.write('''
void MicMetrics_ProcessFrame(const int32_t* pcm, uint32_t len, float* out_dbfs, float* out_spl) {
    int16_t pcm16[256];
    uint32_t count = len > 256 ? 256 : len;
    for(uint32_t i=0; i<count; i++) {
        pcm16[i] = (int16_t)pcm[i]; 
    }
    
    AudioBasicFeatureDebug features = Audio_ComputeBasicFeatures(pcm16, count);
    Audio_ComputeAWeightedFeatures(pcm16, count, 48000, features.mean_counts, &features.a_weighted_rms_counts, &features.a_weighted_rms_dbfs);
    
    *out_dbfs = features.rms_zero_mean_dbfs;
    *out_spl = features.a_weighted_rms_dbfs + 122.40; // AUDIO_SPL_CALIBRATION_OFFSET_DB
}
''')

