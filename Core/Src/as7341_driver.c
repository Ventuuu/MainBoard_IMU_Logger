/**
 * \file as7341_driver.c
 * \brief Implementation of the AS7341 spectral light sensor driver.
 *
 * Uses hi2c3 (shared I2C bus with the IMU). Mirrors the coding style of
 * imu_driver.c for consistency within the project.
 *
 * Acquisition strategy:
 *   - ATIME = 29  -> 30 integration steps
 *   - ASTEP = 599 -> 600 us per step => total = 18 ms per measurement
 *   - AGAIN = 3   -> x8 gain (adjustable depending on light conditions)
 *   We poll AVALID before reading; if the bit is not set within the timeout,
 *   the previous sample is returned unchanged.
 */

#include "as7341_driver.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;

/* ---- Private helper prototypes ----------------------------------------- */
static uint8_t as7341_read_register(uint8_t reg_addr, uint8_t *data, uint16_t len);
static uint8_t as7341_write_register(uint8_t reg_addr, uint8_t value);
static uint8_t as7341_wait_avalid(uint32_t timeout_ms);

/* ---- Public Function Implementations ------------------------------------ */

/**
 * @brief Initializes the AS7341.
 *
 * Steps:
 *  1. Power on (PON).
 *  2. Set integration time: ATIME=29, ASTEP=599 -> ~18 ms per cycle.
 *  3. Set gain to x8 (AGAIN=3).
 *  4. Enable spectral measurement (SP_EN).
 *
 * @return 1 on success, 0 if device not responding.
 */
uint8_t AS7341_Init(void) {
    uint8_t id = 0;

    /* The AS7341 does not have a WHO_AM_I register in the traditional sense,
     * but register 0x92 (ID register) should return 0x09 for AS7341. */
    if (!as7341_read_register(0x92U, &id, 1) || (id & 0xFCU) != 0x24U) {
        /* 0x92 returns 0x24 on AS7341-DLGT; mask lower 2 bits for revision */
        return 0;
    }

    /* Step 1: Power on */
    if (!as7341_write_register(AS7341_REG_ENABLE, AS7341_PON)) return 0;
    HAL_Delay(5); /* tPON stabilization */

    /* Step 2: Integration time ATIME = 29 */
    if (!as7341_write_register(AS7341_REG_ATIME, 29U)) return 0;

    /* Step 3: ASTEP = 599 (LSB=0x57, MSB=0x02) */
    if (!as7341_write_register(AS7341_REG_ASTEP_L, 0x57U)) return 0;
    if (!as7341_write_register(AS7341_REG_ASTEP_H, 0x02U)) return 0;

    /* Step 4: AGAIN = 3 (x8 gain) in CFG1 register */
    if (!as7341_write_register(AS7341_REG_AGAIN, 0x03U)) return 0;

    /* Step 5: Enable spectral measurements */
    if (!as7341_write_register(AS7341_REG_ENABLE, AS7341_PON | AS7341_SP_EN)) return 0;

    return 1;
}

/**
 * @brief Reads Clear and NIR channels from the AS7341.
 *
 * The AS7341 default SMUX configuration after power-on maps the high channels
 * (F7, F8, Clear, NIR) to output registers CH0-CH3 starting at 0x95.
 * We read CH2 (Clear = 0x99) and CH3 (NIR = 0x9B) specifically.
 *
 * raw_data layout (4 bytes):
 *   [0] Clear LSB, [1] Clear MSB, [2] NIR LSB, [3] NIR MSB
 *
 * @param light_data  Destination struct for converted uint16 values.
 * @param raw_data    4-byte flat buffer for NAND storage.
 */
void AS7341_ReadChannels(AS7341_Data *light_data, uint8_t *raw_data) {
    uint8_t buf[4] = {0};

    /* Wait for a valid conversion (up to 50 ms) */
    if (!as7341_wait_avalid(50U)) {
        /* Timeout: leave raw_data and light_data unchanged */
        return;
    }

    /* Read Clear channel (CH2) = 2 bytes at 0x99 */
    if (!as7341_read_register(AS7341_REG_CH2_L, &buf[0], 2U)) return;

    /* Read NIR channel (CH3) = 2 bytes at 0x9B */
    if (!as7341_read_register(AS7341_REG_CH3_L, &buf[2], 2U)) return;

    /* Copy to raw output for NAND packet */
    raw_data[0] = buf[0];  /* Clear LSB */
    raw_data[1] = buf[1];  /* Clear MSB */
    raw_data[2] = buf[2];  /* NIR LSB   */
    raw_data[3] = buf[3];  /* NIR MSB   */

    /* Convert to uint16 for the struct */
    light_data->clear = (uint16_t)((buf[1] << 8) | buf[0]);
    light_data->nir   = (uint16_t)((buf[3] << 8) | buf[2]);
}

/* ---- Private Helper Implementations ------------------------------------ */

/**
 * @brief Reads one or more bytes from the AS7341 via I2C.
 */
static uint8_t as7341_read_register(uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Master_Transmit(&hi2c3, AS7341_I2C_ADDRESS << 1, &reg_addr, 1, AS7341_I2C_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(&hi2c3, AS7341_I2C_ADDRESS << 1, data, len, AS7341_I2C_TIMEOUT) != HAL_OK)
        return 0;
    return 1;
}

/**
 * @brief Writes a single byte to an AS7341 register via I2C.
 */
static uint8_t as7341_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx[2] = {reg_addr, value};
    if (HAL_I2C_Master_Transmit(&hi2c3, AS7341_I2C_ADDRESS << 1, tx, 2, AS7341_I2C_TIMEOUT) != HAL_OK)
        return 0;
    return 1;
}

/**
 * @brief Polls STATUS2.AVALID until set or timeout.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return 1 if AVALID was set before timeout, 0 if timed out.
 */
static uint8_t as7341_wait_avalid(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    uint8_t status = 0;
    while ((HAL_GetTick() - start) < timeout_ms) {
        as7341_read_register(AS7341_REG_STATUS2, &status, 1);
        if (status & AS7341_AVALID) return 1;
        HAL_Delay(2);
    }
    return 0;
}
