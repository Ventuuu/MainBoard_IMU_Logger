#ifndef IMU_RING_BUFFER_H
#define IMU_RING_BUFFER_H

/**
 * @file  imu_ring_buffer.h
 * @brief Lock-free power-of-2 ring buffer for raw IMU samples.
 *
 * Design contract
 * ---------------
 * The ISR (HAL_TIM_PeriodElapsedCallback) is the ONLY producer.
 * The main while(1) loop is the ONLY consumer.
 * With a single producer and single consumer on a Cortex-M33 this
 * would be naturally lock-free, BUT the HAL I2C read inside the ISR
 * already disables interrupts internally, and the Push/Pop pair use
 * a minimal __disable_irq / __enable_irq critical section to guarantee
 * atomic index updates on compilers that may split 32-bit stores.
 *
 * Memory cost
 * -----------
 * IMU_RawData_t  = 12 bytes (6 acc + 6 gyro)
 * Buffer depth   = IMU_RB_SIZE = 32  (must be a power of 2)
 * Total SRAM     = 32 × 12 = 384 bytes + 8 bytes overhead = 392 bytes
 *
 * Overflow policy
 * ---------------
 * If the main loop is stalled and the buffer fills up, Push silently
 * drops the oldest sample (oldest entry is overwritten) and increments
 * the overflow counter.  The counter is readable via
 * IMU_RingBuffer_OverflowCount() for diagnostics / LED alerting.
 */

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Buffer depth — MUST be a power of 2 so the index wrap is a cheap AND mask.
 * 32 slots × 12 bytes = 384 bytes.  At 100 Hz the buffer holds 320 ms of
 * headroom before an overflow.  Increase to 64 (768 B) if needed.
 * -------------------------------------------------------------------------- */
#define IMU_RB_SIZE  32U
#define IMU_RB_MASK  (IMU_RB_SIZE - 1U)

/* --------------------------------------------------------------------------
 * Raw sample: exactly the 6-byte register dumps the ISR reads over I2C.
 * Conversion to physical units (m/s², °/s) is deferred to the main loop.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t acc[6];   /**< OUTZ_H_A ... OUTX_L_A  (little-endian twos-complement) */
    uint8_t gyro[6];  /**< OUTZ_H_G ... OUTX_L_G  (little-endian twos-complement) */
} IMU_RawData_t;

/* --------------------------------------------------------------------------
 * Ring buffer control structure.
 * head : written exclusively by the ISR (producer)
 * tail : written exclusively by the main loop (consumer)
 * -------------------------------------------------------------------------- */
typedef struct {
    IMU_RawData_t buf[IMU_RB_SIZE]; /**< Static storage — no heap used        */
    volatile uint32_t head;          /**< Next write index (producer)           */
    volatile uint32_t tail;          /**< Next read  index (consumer)           */
    volatile uint32_t overflow_cnt;  /**< Incremented each time a slot is lost  */
} IMU_RingBuffer_t;

/* Global singleton — defined in imu_ring_buffer.c */
extern IMU_RingBuffer_t g_imu_ring_buffer;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief Initialise (zero) the ring buffer.  Call once before starting TIM2.
 */
void IMU_RingBuffer_Init(IMU_RingBuffer_t *rb);

/**
 * @brief Push one raw sample into the ring buffer.
 *
 * Safe to call from ISR context.  If the buffer is full the oldest
 * entry is silently dropped and overflow_cnt is incremented.
 *
 * @param rb   Pointer to the ring buffer instance.
 * @param sample  Pointer to the IMU_RawData_t to copy in.
 */
void IMU_RingBuffer_Push(IMU_RingBuffer_t *rb, const IMU_RawData_t *sample);

/**
 * @brief Pop one raw sample from the ring buffer.
 *
 * Safe to call from main-loop context.
 *
 * @param rb   Pointer to the ring buffer instance.
 * @param out  Destination struct.  Written only if a sample is available.
 * @return 1 if a sample was copied into *out, 0 if the buffer was empty.
 */
uint8_t IMU_RingBuffer_Pop(IMU_RingBuffer_t *rb, IMU_RawData_t *out);

/**
 * @brief Returns 1 if the ring buffer contains no pending samples.
 */
static inline uint8_t IMU_RingBuffer_IsEmpty(const IMU_RingBuffer_t *rb)
{
    return (rb->head == rb->tail);
}

/**
 * @brief Returns the number of samples currently waiting in the buffer.
 *
 * This value may change asynchronously while the ISR is running; treat
 * it as an approximate snapshot.
 */
static inline uint32_t IMU_RingBuffer_Count(const IMU_RingBuffer_t *rb)
{
    return (rb->head - rb->tail) & IMU_RB_MASK;
}

/**
 * @brief Returns the total number of samples dropped due to overflow.
 */
static inline uint32_t IMU_RingBuffer_OverflowCount(const IMU_RingBuffer_t *rb)
{
    return rb->overflow_cnt;
}

#endif /* IMU_RING_BUFFER_H */
