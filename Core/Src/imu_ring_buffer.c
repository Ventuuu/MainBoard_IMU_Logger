/**
 * @file  imu_ring_buffer.c
 * @brief Lock-free ring buffer for raw IMU samples — implementation.
 *
 * See imu_ring_buffer.h for the full design contract.
 */

#include "imu_ring_buffer.h"
#include <string.h>   /* memcpy */
#include "cmsis_gcc.h" /* __disable_irq / __enable_irq */

/* --------------------------------------------------------------------------
 * Global singleton instance — zero-initialised in BSS at startup.
 * -------------------------------------------------------------------------- */
IMU_RingBuffer_t g_imu_ring_buffer;

/* --------------------------------------------------------------------------
 * IMU_RingBuffer_Init
 * -------------------------------------------------------------------------- */
void IMU_RingBuffer_Init(IMU_RingBuffer_t *rb)
{
    memset(rb, 0, sizeof(IMU_RingBuffer_t));
}

/* --------------------------------------------------------------------------
 * IMU_RingBuffer_Push  (called from ISR)
 *
 * Overflow policy: overwrite oldest slot, advance tail so the consumer
 * always sees the most recent data rather than stale data.
 * -------------------------------------------------------------------------- */
void IMU_RingBuffer_Push(IMU_RingBuffer_t *rb, const IMU_RawData_t *sample)
{
    uint32_t next_head = (rb->head + 1U) & IMU_RB_MASK;

    if (next_head == rb->tail) {
        /* Buffer full — drop oldest sample, advance consumer index */
        rb->tail = (rb->tail + 1U) & IMU_RB_MASK;
        rb->overflow_cnt++;
    }

    memcpy(&rb->buf[rb->head], sample, sizeof(IMU_RawData_t));

    /* Barrier: ensure the data write is visible before the index update */
    __DMB();

    rb->head = next_head;
}

/* --------------------------------------------------------------------------
 * IMU_RingBuffer_Pop  (called from main loop)
 *
 * A brief critical section guards the tail index read-modify-write so
 * the compiler cannot split it across two instructions.
 * -------------------------------------------------------------------------- */
uint8_t IMU_RingBuffer_Pop(IMU_RingBuffer_t *rb, IMU_RawData_t *out)
{
    if (rb->head == rb->tail) {
        return 0U; /* empty */
    }

    /* Copy data BEFORE advancing tail so the ISR cannot overwrite the
     * slot we are reading.  The __DMB ensures we see the data written
     * by the ISR before we consumed the index. */
    __DMB();
    memcpy(out, &rb->buf[rb->tail], sizeof(IMU_RawData_t));

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    rb->tail = (rb->tail + 1U) & IMU_RB_MASK;
    if (!primask) __enable_irq();

    return 1U;
}
