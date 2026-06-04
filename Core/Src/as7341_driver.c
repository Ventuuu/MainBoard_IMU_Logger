/**
 * \file as7341_driver.c
 * \brief Implementation of the AS7341 spectral light sensor driver.
 *
 * Now supports two layers of API:
 *   1) Legacy Clear+NIR-only readout (AS7341_ReadChannels) used by existing
 *      logging code (4 bytes per sample for light).
 *   2) Full-spectrum readout (AS7341_ReadFullSpectrum) using two SMUX
 *      configurations inspired by the Adafruit AS7341 library to acquire
 *      F1–F8 + Clear + NIR (12 channels).
 */

#include "as7341_driver.h"
#include "main.h"

#include "stm32u5xx_hal.h"


extern I2C_HandleTypeDef hi2c3;

/* ---- Private helper prototypes ----------------------------------------- */
static uint8_t as7341_read_register(uint8_t reg_addr, uint8_t *data, uint16_t len);
static uint8_t as7341_write_register(uint8_t reg_addr, uint8_t value);
static uint8_t as7341_wait_avalid(uint32_t timeout_ms);
static void    as7341_select_regbank(uint8_t enable_bank1);
static void    as7341_smux_apply(AS7341_SmuxCmd cmd);
static void    as7341_smux_setup_F1F4_Clear_NIR(void);
static void    as7341_smux_setup_F5F8_Clear_NIR(void);
static void    as7341_smux_setup_FlickerPD(void);
static uint16_t as7341_decode_flicker_mains(uint8_t fd_status);

/* ---- Public Function Implementations ------------------------------------ */

uint8_t AS7341_Init(void) {
    uint8_t id = 0;

    /* WHOAMI/ID check: register 0x92 should return 0x09 (Adafruit) or 0x24 w/ rev bits. */
    if (!as7341_read_register(0x92U, &id, 1)) {
        return 0;
    }

    /* Simple sanity check on ID; accept both common encodings. */
    if (!((id == 0x09U) || ((id & 0xFCU) == 0x24U))) {
        return 0;
    }

    /* Power on */
    if (!as7341_write_register(AS7341_REG_ENABLE, AS7341_PON)) return 0;
    HAL_Delay(5);

    /* Default timing/gain similar to previous implementation: ATIME=29, ASTEP=599, GAIN=x8 */
    AS7341_ConfigTimingAndGain(29U, 599U, AS7341_GAIN_8X);

    /* Enable spectral measurements */
    if (!as7341_write_register(AS7341_REG_ENABLE, AS7341_PON | AS7341_SP_EN)) return 0;

    return 1;
}

void AS7341_ConfigTimingAndGain(uint8_t atime, uint16_t astep, AS7341_Gain gain) {
    uint8_t step_l = (uint8_t)(astep & 0xFFU);
    uint8_t step_h = (uint8_t)(astep >> 8);

    as7341_write_register(AS7341_REG_ATIME, atime);
    as7341_write_register(AS7341_REG_ASTEP_L, step_l);
    as7341_write_register(AS7341_REG_ASTEP_H, step_h);
    as7341_write_register(AS7341_REG_AGAIN, (uint8_t)gain);
}

void AS7341_ReadChannels(AS7341_Data *light_data, uint8_t *raw_data) {
    uint8_t buf[4] = {0};

    if (!as7341_wait_avalid(50U)) {
        return; /* timeout: leave previous values */
    }

    /* Default high-channel map: CH2=Clear, CH3=NIR */
    if (!as7341_read_register(AS7341_REG_CH2_L, &buf[0], 2U)) return;
    if (!as7341_read_register(AS7341_REG_CH3_L, &buf[2], 2U)) return;

    raw_data[0] = buf[0];
    raw_data[1] = buf[1];
    raw_data[2] = buf[2];
    raw_data[3] = buf[3];

    light_data->clear = (uint16_t)((buf[1] << 8) | buf[0]);
    light_data->nir   = (uint16_t)((buf[3] << 8) | buf[2]);
}

uint8_t AS7341_ReadSixChannels(uint16_t *dst6) {
    uint8_t buf[12];
    if (!as7341_wait_avalid(50U)) {
        return 0;
    }
    
    if (!as7341_read_register(AS7341_REG_CH0_L, buf, sizeof(buf))) {
        return 0;
    }
    
    for (uint8_t i = 0; i < 6; i++) {
        uint8_t l = buf[2U * i];
        uint8_t h = buf[2U * i + 1U];
        dst6[i] = (uint16_t)((h << 8) | l);
    }

    return 1;
}

uint8_t AS7341_ReadFullSpectrum(AS7341_Spectrum *spectrum) {
    uint16_t tmp[6];
    
    if (spectrum == NULL) return 0;

    /* --- Low channels: F1–F4 + Clear + NIR --- */
    as7341_select_regbank(1U);
    as7341_smux_setup_F1F4_Clear_NIR();
    as7341_smux_apply(AS7341_SMUX_CMD_WRITE);
    as7341_select_regbank(0U);

    if (!AS7341_ReadSixChannels(tmp)) return 0;
    for (uint8_t i = 0; i < 6; i++) {
        spectrum->ch[i] = tmp[i];
    }

    /* --- High channels: F5–F8 + Clear + NIR --- */
    as7341_select_regbank(1U);
    as7341_smux_setup_F5F8_Clear_NIR();
    as7341_smux_apply(AS7341_SMUX_CMD_WRITE);
    as7341_select_regbank(0U);


    if (!AS7341_ReadSixChannels(tmp))return 0;
    for (uint8_t i = 0; i < 6; i++) {
        spectrum->ch[6U + i] = tmp[i]; // PROBLEM! breaking line!
    }

    return 1;
}

uint16_t AS7341_DetectMainsHz(void) {
    uint8_t enable = 0;
    uint8_t status = 0;
    uint32_t start = 0;

    /* 1) Disable spectral & flicker, then enable power only. */
    if (!as7341_read_register(AS7341_REG_ENABLE, &enable, 1U)) {
        return 0U;
    }

    enable &= (uint8_t)~(AS7341_SP_EN | AS7341_FDEN | AS7341_SMUXEN);
    enable |= AS7341_PON;
    if (!as7341_write_register(AS7341_REG_ENABLE, enable)) {
        return 0U;
    }

    /* 2) Configure SMUX chain for flicker PD (Adafruit FDConfig pattern). */
    as7341_select_regbank(1U);
    as7341_smux_setup_FlickerPD();
    as7341_smux_apply(AS7341_SMUX_CMD_WRITE);
    as7341_select_regbank(0U);

    /* 3) Set a modest flicker integration time and gain. These values
     * are chosen to mirror Adafruit defaults sufficiently for mains
     * detection (100/120 Hz). */
    as7341_write_register(AS7341_REG_FD_TIME1, 0xFFU);
    as7341_write_register(AS7341_REG_FD_TIME2, 0x03U);

    /* 4) Enable spectral engine and flicker detection. */
    if (!as7341_read_register(AS7341_REG_ENABLE, &enable, 1U)) {
        return 0U;
    }
    enable |= (AS7341_SP_EN | AS7341_FDEN);
    if (!as7341_write_register(AS7341_REG_ENABLE, enable)) {
        return 0U;
    }

    /* 5) Wait for flicker measurement to complete. Datasheet suggests
     * tens to hundreds of ms; we poll FD_STATUS for a bounded time. */
    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 300U) {
        if (!as7341_read_register(AS7341_REG_FD_STATUS, &status, 1U)) {
            return 0U;
        }
        /* If any flicker classification bits are set, decode and return. */
        if (status != 0U) {
            return as7341_decode_flicker_mains(status);
        }
        //HAL_Delay(10U);
    }

    /* Timeout or no valid flicker detected. */
    return 0U;
}

/* ---- Private Helper Implementations ------------------------------------ */

static uint8_t as7341_read_register(uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Master_Transmit(&hi2c3, AS7341_I2C_ADDRESS << 1, &reg_addr, 1, AS7341_I2C_TIMEOUT) != HAL_OK)
        return 0;
    if (HAL_I2C_Master_Receive(&hi2c3, AS7341_I2C_ADDRESS << 1, data, len, AS7341_I2C_TIMEOUT) != HAL_OK)
        return 0;
    return 1;
}

static uint8_t as7341_write_register(uint8_t reg_addr, uint8_t value) {
    uint8_t tx[2] = {reg_addr, value};
    if (HAL_I2C_Master_Transmit(&hi2c3, AS7341_I2C_ADDRESS << 1, tx, 2, AS7341_I2C_TIMEOUT) != HAL_OK)
        return 0;
    return 1;
}

static uint8_t as7341_wait_avalid(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    uint8_t status = 0;
    while ((HAL_GetTick() - start) < timeout_ms) {
        as7341_read_register(AS7341_REG_STATUS2, &status, 1);
        if (status & AS7341_AVALID) return 1;
        //HAL_Delay(2);
    }
    return 0;
}

static void as7341_select_regbank(uint8_t enable_bank1) {
    uint8_t cfg0 = 0;
    if (!as7341_read_register(AS7341_REG_CFG0, &cfg0, 1)) return;

    if (enable_bank1) {
        cfg0 |= AS7341_REG_BANK_ACCESS;
    } else {
        cfg0 &= (uint8_t)(~AS7341_REG_BANK_ACCESS);
    }

    as7341_write_register(AS7341_REG_CFG0, cfg0);
}

static void as7341_smux_apply(AS7341_SmuxCmd cmd) {
    /* Program CFG6 lower bits with SMUX command, then set SMUXEN in ENABLE. */
    uint8_t cfg6 = 0;
    uint8_t enable = 0;

    as7341_read_register(AS7341_REG_CFG6, &cfg6, 1);
    cfg6 &= (uint8_t)~0x03U;
    cfg6 |= (uint8_t)cmd & 0x03U;
    as7341_write_register(AS7341_REG_CFG6, cfg6);

    as7341_read_register(AS7341_REG_ENABLE, &enable, 1);
    enable |= AS7341_SMUXEN;
    as7341_write_register(AS7341_REG_ENABLE, enable);

    /* SMUXEN self-clears when the SMUX command is finished; no need to poll. */
}

/* The following SMUX configurations are direct translations of the
 * Adafruit_AS7341::setup_F1F4_Clear_NIR and setup_F5F8_Clear_NIR functions,
 * adapted to run on STM32 using our register helpers. */

static void as7341_smux_setup_F1F4_Clear_NIR(void) {
    as7341_write_register(0x00U, 0x30U); /* F3 left -> ADC2 */
    as7341_write_register(0x01U, 0x01U); /* F1 left -> ADC0 */
    as7341_write_register(0x02U, 0x00U);
    as7341_write_register(0x03U, 0x00U); /* F8 left disabled */
    as7341_write_register(0x04U, 0x00U); /* F6 left disabled */
    as7341_write_register(0x05U, 0x42U); /* F4 left -> ADC3, F2 left -> ADC1 */
    as7341_write_register(0x06U, 0x00U); /* F5 left disabled */
    as7341_write_register(0x07U, 0x00U); /* F7 left disabled */
    as7341_write_register(0x08U, 0x50U); /* CLEAR -> ADC4 */
    as7341_write_register(0x09U, 0x00U); /* F5 right disabled */
    as7341_write_register(0x0AU, 0x00U); /* F7 right disabled */
    as7341_write_register(0x0BU, 0x00U);
    as7341_write_register(0x0CU, 0x20U); /* F2 right -> ADC1 */
    as7341_write_register(0x0DU, 0x04U); /* F4 right -> ADC3 */
    as7341_write_register(0x0EU, 0x00U); /* F6/F8 right disabled */
    as7341_write_register(0x0FU, 0x30U); /* F3 right -> ADC2 */
    as7341_write_register(0x10U, 0x01U); /* F1 right -> ADC0 */
    as7341_write_register(0x11U, 0x50U); /* CLEAR right -> ADC4 */
    as7341_write_register(0x12U, 0x00U);
    as7341_write_register(0x13U, 0x06U); /* NIR -> ADC5 */
}

static void as7341_smux_setup_F5F8_Clear_NIR(void) {
    as7341_write_register(0x00U, 0x00U); /* F3 left disable */
    as7341_write_register(0x01U, 0x00U); /* F1 left disable */
    as7341_write_register(0x02U, 0x00U);
    as7341_write_register(0x03U, 0x40U); /* F8 left -> ADC3 */
    as7341_write_register(0x04U, 0x02U); /* F6 left -> ADC1 */
    as7341_write_register(0x05U, 0x00U); /* F4/F2 disabled */
    as7341_write_register(0x06U, 0x10U); /* F5 left -> ADC0 */
    as7341_write_register(0x07U, 0x03U); /* F7 left -> ADC2 */
    as7341_write_register(0x08U, 0x50U); /* CLEAR left -> ADC4 */
    as7341_write_register(0x09U, 0x10U); /* F5 right -> ADC0 */
    as7341_write_register(0x0AU, 0x03U); /* F7 right -> ADC2 */
    as7341_write_register(0x0BU, 0x00U);
    as7341_write_register(0x0CU, 0x00U); /* F2 right disable */
    as7341_write_register(0x0DU, 0x00U); /* F4 right disable */
    as7341_write_register(0x0EU, 0x24U); /* F8 right -> ADC3, F6 right -> ADC1 */
    as7341_write_register(0x0FU, 0x00U); /* F3 right disable */
    as7341_write_register(0x10U, 0x00U); /* F1 right disable */
    as7341_write_register(0x11U, 0x50U); /* CLEAR right -> ADC4 */
    as7341_write_register(0x12U, 0x00U);
    as7341_write_register(0x13U, 0x06U); /* NIR -> ADC5 */
}

/* SMUX configuration for flicker PD, adapted from Adafruit_AS7341::FDConfig. */
static void as7341_smux_setup_FlickerPD(void) {
    as7341_write_register(0x00U, 0x00U);
    as7341_write_register(0x01U, 0x00U);
    as7341_write_register(0x02U, 0x00U);
    as7341_write_register(0x03U, 0x00U);
    as7341_write_register(0x04U, 0x00U);
    as7341_write_register(0x05U, 0x00U);
    as7341_write_register(0x06U, 0x00U);
    as7341_write_register(0x07U, 0x00U);
    as7341_write_register(0x08U, 0x00U);
    as7341_write_register(0x09U, 0x00U);
    as7341_write_register(0x0AU, 0x00U);
    as7341_write_register(0x0BU, 0x00U);
    as7341_write_register(0x0CU, 0x00U);
    as7341_write_register(0x0DU, 0x00U);
    as7341_write_register(0x0EU, 0x00U);
    as7341_write_register(0x0FU, 0x00U);
    as7341_write_register(0x10U, 0x00U);
    as7341_write_register(0x11U, 0x00U);
    as7341_write_register(0x12U, 0x00U);
    as7341_write_register(0x13U, 0x60U); /* Flicker PD -> ADC5, matching Adafruit FDConfig */
}

/* Decode FD_STATUS into equivalent mains frequency classification. The
 * exact bit mapping is device/firmware specific; this helper assumes the
 * common convention where particular FD_STATUS codes indicate 100 Hz,
 * 120 Hz, or unknown flicker types, mirroring Adafruit's
 * Adafruit_AS7341::decodeFlickerDetectStatus behavior. */
static uint16_t as7341_decode_flicker_mains(uint8_t fd_status) {
    switch (fd_status) {
        case 45: /* 100 Hz flicker detected */
            return 50U;  /* 50 Hz mains */
        case 46: /* 120 Hz flicker detected */
            return 60U;  /* 60 Hz mains */
        case 44: /* flicker, unknown frequency */
        default:
            return 0U;   /* treat as natural / non-mains */
    }
}
