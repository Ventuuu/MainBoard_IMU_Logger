#ifndef IMU_RING_BUFFER_H
#define IMU_RING_BUFFER_H

/**
 * @file  imu_ring_buffer.h
 * @brief Lock-free power-of-2 ring buffer for raw IMU samples,
 *        paired with the Flag-and-Fetch ISR architecture.
 *
 * ============================================================
 * Architecture overview
 * ============================================================
 *
 *  EXTI ISR (IMU_IS_INT1_Pin, priority 6)
 *  ─────────────────────────────────────────────────────────
 *  1. Set  g_imu_fetch_flag = 1          (< 1 µs)
 *  2. Increment g_light_tick             (< 1 µs)
 *  3. Return — ISR is done.
 *
 *  while(1) — STATE_ACQUISITION  (thread / unprivileged context)
 *  ─────────────────────────────────────────────────────────
 *  1. Check g_imu_fetch_flag.
 *  2. Clear flag under __disable_irq critical section.
 *  3. Call IMU_ReadAccelerometerData()  — blocking I2C, but now in
 *     thread context so the NVIC can freely preempt with any IRQ
 *     of higher priority (USART3/BLE at prio 5, OTG_FS/USB, TIM2).
 *  4. Call IMU_ReadGyroscopeData()      — same.
 *  5. Pack raw bytes → IMU_RawData_t → Push into ring buffer.
 *  6. Drain ring buffer → convert → BLE TX → NAND write.
 *
 * ============================================================
 * NVIC priority table (lower number = higher priority)
 * ============================================================
 *
 *  Priority 5  USART3_IRQn   — BLE radio link (RN4871 UART)
 *  Priority 5  OTG_FS_IRQn   — USB Virtual COM Port
 *  Priority 6  EXTI_IRQn     — IMU_IS_INT1_Pin (100 Hz tripwire, flag only)
 *  Priority 6  EXTI*_IRQn    — Button + IMU data-ready pins
 *
 *  With this table:
 *  - The main loop I2C blocking call can be preempted by ANY IRQ
 *    because it runs at thread priority (effectively priority 15+).
 *  - The 100 ms I2C timeout in the worst case only stalls the
 *    main loop, never the interrupt vector table.
 *
 * ============================================================
 * Ring buffer: memory cost
 * ============================================================
 *
 *  IMU_RawData_t  = 12 bytes (6 acc + 6 gyro)
 *  Buffer depth   = IMU_RB_SIZE = 32  (must stay a power of 2)
 *  Total SRAM     = 32 × 12 + 8 overhead = 392 bytes
 *
 * ============================================================
 * Overflow policy
 * ============================================================
 *
 *  If the main loop is delayed and the buffer fills, Push silently
 *  overwrites the oldest sample and increments overflow_cnt.
 *  Read with IMU_RingBuffer_OverflowCount() for diagnostics.
 */

#include <stdint.h>

/* --------------------------------------------------------------------------
 * Buffer depth — MUST be a power of 2.
 * 32 slots × 12 bytes = 384 bytes.  At 100 Hz → 320 ms of headroom.
 * -------------------------------------------------------------------------- */
#define IMU_RB_SIZE  32U
#define IMU_RB_MASK  (IMU_RB_SIZE - 1U)

/* --------------------------------------------------------------------------
 * Fetch flag — set by IMU EXTI ISR, cleared by main loop.
 *
 * Declared volatile so the compiler never caches it in a register.
 * The main loop must clear it inside a __disable_irq / __enable_irq
 * critical section to avoid a race where the ISR fires between the
 * read and the clear.
 * -------------------------------------------------------------------------- */
extern volatile uint8_t g_imu_fetch_flag;

/* --------------------------------------------------------------------------
 * Raw sample struct.
 * Stores exactly the 6-byte register dumps read over I2C.
 * Conversion to physical units is deferred to the main loop.
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t acc[6];   /**< OUTX_L_A … OUTZ_H_A (little-endian two's-complement) */
    uint8_t gyro[6];  /**< OUTX_L_G … OUTZ_H_G (little-endian two's-complement) */
} IMU_RawData_t;

/* --------------------------------------------------------------------------
 * Ring buffer control structure.
 * head : written exclusively by the fetch path (main loop, after ISR flag)
 * tail : written exclusively by the drain path (main loop)
 * -------------------------------------------------------------------------- */
typedef struct {
    IMU_RawData_t    buf[IMU_RB_SIZE]; /**< Static storage — no heap          */
    volatile uint32_t head;             /**< Next write index (producer)        */
    volatile uint32_t tail;             /**< Next read  index (consumer)        */
    volatile uint32_t overflow_cnt;     /**< Samples dropped due to overflow    */
} IMU_RingBuffer_t;

/* Global singleton — defined in imu_ring_buffer.c */
extern IMU_RingBuffer_t g_imu_ring_buffer;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/** @brief Zero the ring buffer.  Call once before starting TIM2. */
void IMU_RingBuffer_Init(IMU_RingBuffer_t *rb);

/**
 * @brief Push one raw sample (called from the main-loop fetch path).
 *
 * NOT called from the ISR in this architecture — the ISR only sets
 * g_imu_fetch_flag.  Push is invoked from thread context after the
 * blocking I2C read completes.
 *
 * Overflow policy: oldest entry overwritten, overflow_cnt incremented.
 */
void IMU_RingBuffer_Push(IMU_RingBuffer_t *rb, const IMU_RawData_t *sample);

/**
 * @brief Pop one raw sample (called from the main-loop drain path).
 * @return 1 if a sample was copied into *out, 0 if the buffer was empty.
 */
uint8_t IMU_RingBuffer_Pop(IMU_RingBuffer_t *rb, IMU_RawData_t *out);

/** @brief Returns 1 if the ring buffer contains no pending samples. */
static inline uint8_t IMU_RingBuffer_IsEmpty(const IMU_RingBuffer_t *rb)
{
    return (rb->head == rb->tail);
}

/** @brief Approximate number of samples currently in the buffer. */
static inline uint32_t IMU_RingBuffer_Count(const IMU_RingBuffer_t *rb)
{
    return (rb->head - rb->tail) & IMU_RB_MASK;
}

/** @brief Total samples dropped due to overflow since last Init. */
static inline uint32_t IMU_RingBuffer_OverflowCount(const IMU_RingBuffer_t *rb)
{
    return rb->overflow_cnt;
}

#endif /* IMU_RING_BUFFER_H */
