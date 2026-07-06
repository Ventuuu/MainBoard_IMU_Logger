#include "audio_ring_buffer.h"
#include "main.h"

void AudioRing_Init(AudioRingBuffer* rb) {
    AudioRing_Reset(rb);
}

void AudioRing_Reset(AudioRingBuffer* rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->overflow_count = 0;
}

void AudioRing_EnqueueFromIsr(AudioRingBuffer* rb, const int16_t* data, uint32_t length) {
    uint32_t h = rb->head;
    uint32_t t = rb->tail;
    
    for (uint32_t i = 0; i < length; i++) {
        uint32_t next_head = (h + 1) % AUDIO_RING_SLOT_COUNT;
        // CORRECT APPROACH: If buffer is full, drop the NEW data and do NOT touch the tail!
        if (next_head == t) {
            rb->overflow_count++;
            break; // Stop adding data to prevent overwriting unread samples
        }
        
        rb->buffer[h] = data[i];
        h = next_head;
    }
    
    // Ensure all memory writes to the buffer are completed before updating head
    __DMB();
    
    // Update head exactly once at the end so it's atomic from the reader's perspective
    rb->head = h;
}

uint32_t AudioRing_PopChunk(AudioRingBuffer* rb, int16_t* out_data, uint32_t max_length) {
    uint32_t popped = 0;
    uint32_t h = rb->head;
    uint32_t t = rb->tail;
    
    while (popped < max_length && h != t) {
        out_data[popped++] = rb->buffer[t];
        t = (t + 1) % AUDIO_RING_SLOT_COUNT;
    }
    
    // Ensure all reads from the buffer are completed before updating tail
    __DMB();
    
    // Update tail exactly once at the end
    rb->tail = t;
    return popped;
}

uint32_t AudioRing_Count(AudioRingBuffer* rb) {
    uint32_t h = rb->head;
    uint32_t t = rb->tail;
    if (h >= t) {
        return h - t;
    } else {
        return AUDIO_RING_SLOT_COUNT - t + h;
    }
}
