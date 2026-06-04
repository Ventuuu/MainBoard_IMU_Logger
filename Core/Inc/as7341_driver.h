/**
 * \file as7341_driver.h
 * \brief Driver header for the ams AS7341 11-channel spectral light sensor.
 *
 * Expanded to support full spectral readout (F1–F8, Clear, NIR) using
 * SMUX configurations inspired by the Adafruit AS7341 library, while
 * preserving the existing Clear+NIR-only API for backward compatibility.
 */

#ifndef INC_AS7341_DRIVER_H_
#define INC_AS7341_DRIVER_H_

#include <stdint.h>

/* ---- I2C Address -------------------------------------------------------- */
#define AS7341_I2C_ADDRESS      0x39U
#define AS7341_I2C_TIMEOUT      100U

/* ---- Register Map (subset) ---------------------------------------------- */
#define AS7341_REG_ENABLE       0x80U
#define AS7341_REG_ATIME        0x81U
#define AS7341_REG_WTIME        0x83U
#define AS7341_REG_ASTEP_L      0xCAU
#define AS7341_REG_ASTEP_H      0xCBU
#define AS7341_REG_AGAIN        0xAAU
#define AS7341_REG_STATUS       0x93U
#define AS7341_REG_STATUS2      0xA3U
#define AS7341_REG_CFG0         0xA9U
#define AS7341_REG_CFG1         0xAAU
#define AS7341_REG_CFG6         0xAFU

/* Channel output registers (16-bit, LSB first) */
#define AS7341_REG_CH0_L        0x95U
#define AS7341_REG_CH0_H        0x96U
#define AS7341_REG_CH1_L        0x97U
#define AS7341_REG_CH1_H        0x98U
#define AS7341_REG_CH2_L        0x99U
#define AS7341_REG_CH2_H        0x9AU
#define AS7341_REG_CH3_L        0x9BU
#define AS7341_REG_CH3_H        0x9CU
#define AS7341_REG_CH4_L        0x9DU
#define AS7341_REG_CH4_H        0x9EU
#define AS7341_REG_CH5_L        0x9FU
#define AS7341_REG_CH5_H        0xA0U

/* SMUX RAM base address (bank 1, accessed via CFG0.REGBANK) */
#define AS7341_SMUX_RAM_BASE    0x00U

/* Flicker detection registers */
#define AS7341_REG_FD_TIME1     0xD8U
#define AS7341_REG_FD_TIME2     0xDAU
#define AS7341_REG_FD_STATUS    0xDBU

/* ENABLE register bits */
#define AS7341_PON              0x01U
#define AS7341_SP_EN            0x02U
#define AS7341_SMUXEN           0x10U
#define AS7341_FDEN             0x40U

/* STATUS2 AVALID bit */
#define AS7341_AVALID           0x40U

/* CFG0: set bit4=1 to access register bank 0x60–0x74 (SMUX RAM etc.) */
#define AS7341_REG_BANK_ACCESS  0x10U

/* CFG6 SMUX command bits (lower 2 bits) */
typedef enum {
    AS7341_SMUX_CMD_ROM_RESET = 0,
    AS7341_SMUX_CMD_READ      = 1,
    AS7341_SMUX_CMD_WRITE     = 2,
} AS7341_SmuxCmd;

/* Gain settings (CFG1.AGAIN) */
typedef enum {
    AS7341_GAIN_0_5X  = 0,
    AS7341_GAIN_1X    = 1,
    AS7341_GAIN_2X    = 2,
    AS7341_GAIN_4X    = 3,
    AS7341_GAIN_8X    = 4,
    AS7341_GAIN_16X   = 5,
    AS7341_GAIN_32X   = 6,
    AS7341_GAIN_64X   = 7,
    AS7341_GAIN_128X  = 8,
    AS7341_GAIN_256X  = 9,
    AS7341_GAIN_512X  = 10,
} AS7341_Gain;

/* ---- Data Structures ---------------------------------------------------- */

/**
 * @brief Container for the two light sensor channels logged by legacy code.
 */
typedef struct {
    uint16_t clear;  /**< Clear (broadband) channel count */
    uint16_t nir;    /**< Near-infrared channel count     */
} AS7341_Data;

/**
 * @brief Container for a full spectral frame (F1–F8, Clear, NIR).
 *
 * Ordering matches Adafruit convention when using two SMUX configurations:
 *   low:  F1,F2,F3,F4,Clear,NIR  (ch[0..5])
 *   high: F5,F6,F7,F8,Clear,NIR  (ch[6..11])
 */
typedef struct {
    uint16_t ch[12];
} AS7341_Spectrum;

/* ---- Public Function Prototypes ----------------------------------------- */

uint8_t AS7341_Init(void);

/**
 * @brief Legacy helper: reads Clear and NIR channels only.
 *        Kept for backward compatibility with BYTES_PER_SAMPLE = 21.
 */
void AS7341_ReadChannels(AS7341_Data *light_data, uint8_t *raw_data);

/**
 * @brief Configure integration time and gain in a higher-level way.
 *        Thin wrapper around ATIME/ASTEP/AGAIN registers.
 */
void AS7341_ConfigTimingAndGain(uint8_t atime, uint16_t astep, AS7341_Gain gain);

/**
 * @brief Blocking helper that reads all 6 CHx registers into an array.
 *        Used internally by the full-spectrum API.
 */
uint8_t AS7341_ReadSixChannels(uint16_t *dst6);

/**
 * @brief Reads a full spectral frame (12 channels: F1–F8, Clear, NIR) using
 *        two SMUX configurations (low and high) inspired by Adafruit.
 *
 * This call is blocking and may take roughly 2× the integration time.
 *
 * @param[out] spectrum  Pointer to AS7341_Spectrum struct to fill.
 * @return 1 on success, 0 on error/timeout.
 */
uint8_t AS7341_ReadFullSpectrum(AS7341_Spectrum *spectrum);

/**
 * @brief Runs the on-chip flicker engine and returns an equivalent mains
 *        frequency classification.
 *
 * Return value:
 *   0  -> no mains flicker detected / unknown
 *   50 -> 100 Hz flicker (50 Hz mains)
 *   60 -> 120 Hz flicker (60 Hz mains)
 */
uint16_t AS7341_DetectMainsHz(void);

#endif /* INC_AS7341_DRIVER_H_ */
