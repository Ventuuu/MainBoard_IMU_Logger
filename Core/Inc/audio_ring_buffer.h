#ifndef AUDIO_RING_BUFFER_H
#define AUDIO_RING_BUFFER_H

#include <stdint.h>

#define AUDIO_RING_SLOT_COUNT 8192U
#define AUDIO_CHUNK_SAMPLES   1024U

typedef struct {
    int16_t buffer[AUDIO_RING_SLOT_COUNT];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
    volatile uint32_t overflow_count;
} AudioRingBuffer;

void AudioRing_Init(AudioRingBuffer* rb);
void AudioRing_Reset(AudioRingBuffer* rb);
void AudioRing_EnqueueFromIsr(AudioRingBuffer* rb, const int16_t* data, uint32_t length);
uint32_t AudioRing_PopChunk(AudioRingBuffer* rb, int16_t* out_data, uint32_t max_length);
uint32_t AudioRing_Count(AudioRingBuffer* rb);

#endif // AUDIO_RING_BUFFER_H
