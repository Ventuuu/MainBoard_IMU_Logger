/**
 * \file as7341_driver.h
 * \brief Driver header for the ams AS7341 11-channel spectral light sensor.
 *
 * Communicates over I2C (shared hi2c3 bus with the IMU).
 * This driver logs two channels: Clear (channel 8) and NIR (channel 9),
 * each as a 16-bit unsigned value, giving 4 bytes per light sample.
 */

#ifndef INC_AS7341_DRIVER_H_
#define INC_AS7341_DRIVER_H_

#include <stdint.h>

/* ---- I2C Address -------------------------------------------------------- */
/* ATTN: AS7341 default 7-bit I2C address (pin ADDR low = 0x39) */
#define AS7341_I2C_ADDRESS      0x39U
#define AS7341_I2C_TIMEOUT      100U

/* ---- Register Map ------------------------------------------------------- */
#define AS7341_REG_ENABLE       0x80U  /* Power-on, SMUX, spectral enable    */
#define AS7341_REG_ATIME        0x81U  /* Integration time step count        */
#define AS7341_REG_ASTEP_L      0xCAU  /* Integration step size LSB          */
#define AS7341_REG_ASTEP_H      0xCBU  /* Integration step size MSB          */
#define AS7341_REG_AGAIN        0xAAU  /* Spectral gain control (CFG1)       */
#define AS7341_REG_STATUS2      0xA3U  /* AVALID bit lives here              */
#define AS7341_REG_STATUS       0x93U  /* STATUS register                    */
#define AS7341_REG_CFG0         0xA9U  /* Low power / register bank select   */

/* Channel output registers (16-bit, LSB first) for SMUX config F1-F8+Clear+NIR */
/* When low-channel SMUX is active (default after power-on):                    */
/*   CH0=F1, CH1=F2, CH2=F3, CH3=F4, CH4=F5, CH5=F6                            */
/* When high-channel SMUX is active:                                            */
/*   CH0=F7, CH1=F8, CH2=Clear, CH3=NIR                                        */
/* We use the high-channel map to capture Clear and NIR.                        */
#define AS7341_REG_CH0_L        0x95U  /* CH0 data LSB (F7 or Clear)         */
#define AS7341_REG_CH2_L        0x99U  /* CH2 data LSB -> Clear channel      */
#define AS7341_REG_CH3_L        0x9BU  /* CH3 data LSB -> NIR channel        */

/* ENABLE register bits */
#define AS7341_PON              0x01U  /* Power ON                           */
#define AS7341_SP_EN            0x02U  /* Spectral measurement enable        */
#define AS7341_SMUXEN           0x10U  /* SMUX enable (for channel remapping)*/

/* STATUS2 AVALID bit: set when a complete spectral cycle is ready */
#define AS7341_AVALID           0x40U

/* CFG0: set bit5=1 to access register bank > 0x80 */
#define AS7341_REG_BANK_ACCESS  0x10U

/* ---- Data Structures ---------------------------------------------------- */

/**
 * @brief Container for the two light sensor channels logged by this driver.
 *
 * clear: integration of all visible wavelengths (broadband white).
 * nir  : near-infrared channel, useful for ambient light correction.
 */
typedef struct {
    uint16_t clear;  /**< Clear (broadband) channel count */
    uint16_t nir;    /**< Near-infrared channel count     */
} AS7341_Data;

/* ---- Public Function Prototypes ----------------------------------------- */

/**
 * @brief Initializes the AS7341 and verifies its presence via WHO_AM_I.
 * @return 1 on success, 0 on failure.
 */
uint8_t AS7341_Init(void);

/**
 * @brief Reads the Clear and NIR channels from the AS7341 into a flat byte buffer.
 *
 * The buffer must be at least 4 bytes long:
 *   raw[0..1] = Clear (LSB first)
 *   raw[2..3] = NIR   (LSB first)
 *
 * @param light_data  Pointer to AS7341_Data struct for converted channel values.
 * @param raw_data    Pointer to 4-byte buffer for raw bytes (used by write_packet).
 */
void AS7341_ReadChannels(AS7341_Data *light_data, uint8_t *raw_data);

#endif /* INC_AS7341_DRIVER_H_ */
