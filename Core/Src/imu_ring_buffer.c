/**
 * @file  imu_ring_buffer.c
 * @brief Flag-and-Fetch ring buffer — implementation.
 *
 * The ISR sets g_imu_fetch_flag and returns immediately.
 * The main loop reads the flag, clears it, performs the blocking I2C
 * read in thread context (so the NVIC can preempt freely), then calls
 * IMU_RingBuffer_Push to queue the sample for downstream processing.
 *
 * See imu_ring_buffer.h for the full architecture contract.
 */

#include "imu_ring_buffer.h"
#include <string.h>
#include "cmsis_gcc.h"   /* __disable_irq / __enable_irq / __DMB */

/* --------------------------------------------------------------------------
 * Global singleton — zero-initialised in BSS at startup.
 * -------------------------------------------------------------------------- */
IMU_RingBuffer_t g_imu_ring_buffer;

/* --------------------------------------------------------------------------
 * Fetch flag — set by TIM2 ISR, cleared by main loop.
 * Volatile: compiler must re-read from RAM on every access.
 * -------------------------------------------------------------------------- */
volatile uint8_t g_imu_fetch_flag = 0U;

/* --------------------------------------------------------------------------
 * IMU_RingBuffer_Init
 * -------------------------------------------------------------------------- */
void IMU_RingBuffer_Init(IMU_RingBuffer_t *rb)
{
    memset(rb, 0, sizeof(IMU_RingBuffer_t));
}

/* --------------------------------------------------------------------------
 * IMU_RingBuffer_Push
 *
 * Called from the main-loop fetch path (thread context), NOT from the ISR.
 * Overflow policy: overwrite oldest slot so the buffer always holds
 * the most recent data.
 * -------------------------------------------------------------------------- */
void IMU_RingBuffer_Push(IMU_RingBuffer_t *rb, const IMU_RawData_t *sample)
{
    uint32_t next_head = (rb->head + 1U) & IMU_RB_MASK;

    if (next_head == rb->tail) {
        /* Buffer full — advance tail to drop the oldest entry */
        rb->tail = (rb->tail + 1U) & IMU_RB_MASK;
        rb->overflow_cnt++;
    }

    memcpy(&rb->buf[rb->head], sample, sizeof(IMU_RawData_t));

    /* Data-memory barrier: ensure payload is written before index update
     * is visible to the drain path running on the same core. */
    __DMB();

    rb->head = next_head;
}

/* --------------------------------------------------------------------------
 * IMU_RingBuffer_Pop
 *
 * Called from the main-loop drain path (thread context).
 * A minimal critical section guards the tail index update.
 * -------------------------------------------------------------------------- */
uint8_t IMU_RingBuffer_Pop(IMU_RingBuffer_t *rb, IMU_RawData_t *out)
{
    if (rb->head == rb->tail) {
        return 0U; /* empty */
    }

    /* Read data BEFORE advancing tail. DMB ensures we see the payload
     * written by Push before we consume the slot. */
    __DMB();
    memcpy(out, &rb->buf[rb->tail], sizeof(IMU_RawData_t));

    /* Advance tail atomically — tiny critical section (~2 cycles masked) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rb->tail = (rb->tail + 1U) & IMU_RB_MASK;
    if (!primask) __enable_irq();

    return 1U;
}
