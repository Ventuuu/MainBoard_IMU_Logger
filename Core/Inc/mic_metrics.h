#ifndef MIC_METRICS_H
#define MIC_METRICS_H
#include <stdint.h>
void MicMetrics_ProcessFrame(const int16_t* pcm, uint32_t len, uint16_t* out_laeq_x10, uint8_t* out_env_class);
#endif
