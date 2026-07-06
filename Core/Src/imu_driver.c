/**
 * \file imu_driver.c
 * \brief Implementation file for the LSM6DSO16IS IMU driver.
 *
 * This file contains the low-level functions for communicating with the IMU
 * sensor via I2C, including register reads/writes and data processing.
 */

#include "imu_driver.h"
#include "main.h"
#include "string.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c3;

// --- Private Global Variables ---
// Store the configured full-scale settings. These are used to calculate the
// correct sensitivity for data conversion.
static uint8_t accelerometer_full_scale;
static uint8_t gyroscope_full_scale;

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t imu_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len);
static uint8_t imu_write_register(uint8_t reg_addr, uint8_t reg_data);
//static float get_accel_sensitivity(uint8_t accel_fs);
//static float get_gyro_sensitivity(uint8_t gyro_fs);

// --- Public Function Implementations ---

/**
 * @brief Initializes the IMU and verifies its presence via a WHO_AM_I check.
 *
 * @return 1 on success (WHO_AM_I matches), 0 on failure.
 */
uint8_t IMU_Init(void) {
    uint8_t who_am_i_value = 0;
    // Read the WHO_AM_I register and check if the returned value matches the expected one
    if (imu_read_register(IMU_WHO_AM_I_REG, &who_am_i_value, 1) && (who_am_i_value == IMU_WHO_AM_I_VALUE)) {
        return 1; // Success
    }
    return 0; // Failure
}

/**
 * @brief Configures the accelerometer's ODR, Full Scale, and performance mode.
 * @param odr The Output Data Rate macro (e.g., ACC_ODR_104HZ).
 * @param scale The Full Scale range macro (e.g., ACC_FS_4G).
 * @param high_performance_mode 1 for High Performance, 0 for Low Power.
 */
void IMU_ConfigAccelerometer(uint8_t odr, uint8_t scale, uint8_t high_performance_mode) {
    // Combine ODR and FS into a single value for CTRL1_XL register
    uint8_t ctrl1_value = odr | scale;
    imu_write_register(IMU_ACC_CTRL1_REG, ctrl1_value);

    // Store the selected scale for later use in data conversion
    accelerometer_full_scale = scale;

    // Read the current state of the CTRL6_C register to modify only the performance bit
    uint8_t ctrl6_value;
    imu_read_register(IMU_ACC_CTRL6_REG, &ctrl6_value, 1);

    // The HIGH_PERF_DISABLE_BIT (bit 4) in CTRL6_C is set to 0 for High Performance
    // and to 1 for Low Power.
    if (high_performance_mode) {
        // Clear the bit to enable High Performance mode
        ctrl6_value &= ~ACC_HIGH_PERF_DISABLE_BIT;
    } else {
        // Set the bit to enable Low Power mode
        ctrl6_value |= ACC_HIGH_PERF_DISABLE_BIT;
    }

    // Write the new value back to the register
    imu_write_register(IMU_ACC_CTRL6_REG, ctrl6_value);

    imu_write_register(IMU_INT1_CTRL_REG, 0x01);
}

/**
 * @brief Configures the gyroscope's ODR, Full Scale, and performance mode.
 * @param odr The Output Data Rate macro (e.g., GYR_ODR_104HZ).
 * @param scale The Full Scale range macro (e.g., GYR_FS_250DPS).
 * @param high_performance_mode 1 for High Performance, 0 for Low Power.
 */
void IMU_ConfigGyroscope(uint8_t odr, uint8_t scale, uint8_t high_performance_mode) {
    // Combine ODR and FS into a single value for CTRL2_G register
    uint8_t ctrl2_value = odr | scale;
    imu_write_register(IMU_GYR_CTRL2_REG, ctrl2_value);

    // Store the selected scale for later use in data conversion
    gyroscope_full_scale = scale;

    // Read the current state of the CTRL7_G register to modify only the performance bit
    uint8_t ctrl7_value;
    imu_read_register(IMU_GYR_CTRL7_REG, &ctrl7_value, 1);

    // The HIGH_PERF_DISABLE_BIT (bit 7) in CTRL7_G is set to 0 for High Performance
    // and to 1 for Low Power.
    if (high_performance_mode) {
        // Clear the bit to enable High Performance mode
        ctrl7_value &= ~GYR_HIGH_PERF_DISABLE_BIT;
    } else {
        // Set the bit to enable Low Power mode
        ctrl7_value |= GYR_HIGH_PERF_DISABLE_BIT;
    }

    // Write the new value back to the register
    imu_write_register(IMU_GYR_CTRL7_REG, ctrl7_value);
}

/* ====================================================================
 * FETCH PATH: I2C Reads Only (Called immediately after interrupt)
 * ==================================================================== */
void IMU_ReadAccelerometerRaw(uint8_t *raw_data) {
    imu_read_register(IMU_ACC_OUT_X_L_REG, raw_data, 6);
}

void IMU_ReadGyroscopeRaw(uint8_t *raw_data) {
    imu_read_register(IMU_GYR_OUT_X_L_REG, raw_data, 6);
}

/* ====================================================================
 * DRAIN PATH: DSP Math Only (Safe to call from Ring Buffer)
 * ==================================================================== */
void IMU_ConvertAccelRawToFloat(IMU_Data *acc_data, const uint8_t *raw_data) {
    int16_t x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    //float sensitivity = get_accel_sensitivity(accelerometer_full_scale);
    float sensitivity = 2.0 / 32767.0;

    acc_data->x = (float)x_raw * 9.81f * sensitivity;
    acc_data->y = (float)y_raw * 9.81f * sensitivity;
    acc_data->z = (float)z_raw * 9.81f * sensitivity;
}

void IMU_ConvertGyroRawToFloat(IMU_Data *gyro_data, const uint8_t *raw_data) {
    int16_t x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    int16_t y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    int16_t z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    //float sensitivity = get_gyro_sensitivity(gyroscope_full_scale);
    float sensitivity = 1.0 / 175.0;

    gyro_data->x = (float)x_raw * DEG2RAD * sensitivity;
    gyro_data->y = (float)y_raw * DEG2RAD * sensitivity;
    gyro_data->z = (float)z_raw * DEG2RAD * sensitivity;
}

// --- Private Function Implementations (Helper Functions) ---

/**
 * @brief Reads a specific number of bytes from the IMU's register via I2C.
 *
 * This is a low-level function that encapsulates the I2C read operation.
 * @param reg_addr The address of the register to read from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param data_len The number of bytes to read.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t imu_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len) {
    // Send the register address to the IMU to prepare for reading.
    // This is a two-step process in I2C: first a write, then a read.
    if (HAL_I2C_Master_Transmit(&hi2c3, IMU_I2C_ADDRESS << 1, &reg_addr, 1, IMU_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }

    // Read the requested number of bytes from the IMU.
    if (HAL_I2C_Master_Receive(&hi2c3, IMU_I2C_ADDRESS << 1, data, data_len, IMU_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}

/**
 * @brief Writes a single byte of data to a specific register on the IMU via I2C.
 *
 * @param reg_addr The address of the register to write to.
 * @param reg_data The byte of data to write.
 * @return 1 on success, 0 on I2C communication error.
 */
static uint8_t imu_write_register(uint8_t reg_addr, uint8_t reg_data) {
    uint8_t tx_buffer[] = {reg_addr, reg_data};
    if (HAL_I2C_Master_Transmit(&hi2c3, IMU_I2C_ADDRESS << 1, tx_buffer, sizeof(tx_buffer), IMU_I2C_TIMEOUT) != HAL_OK) {
        return 0; // Communication error
    }
    return 1; // Success
}

/**
 * @brief Calculates the correct sensitivity for the accelerometer based on its full scale setting.
 *
 * The sensitivity is a conversion factor that maps the raw ADC value to a real-world
 * physical value (in this case, 'g'). The values are taken from the sensor's datasheet.
 *
 * @param accel_fs The configured full scale macro (e.g., ACC_FS_4G).
 * @return The sensitivity in 'g' per LSB (Least Significant Bit).
 */
// static float get_accel_sensitivity(uint8_t accel_fs) {
//     switch(accel_fs) {
//         case ACC_FS_2G:  return 0.061f / 1000.0f;
//         case ACC_FS_4G:  return 0.122f / 1000.0f;
//         case ACC_FS_8G:  return 0.244f / 1000.0f;
//         case ACC_FS_16G: return 0.488f / 1000.0f;
//         default:         return 0.061f / 1000.0f; // Default to 2g if invalid
//     }
// }

/**
 * @brief Calculates the correct sensitivity for the gyroscope based on its full scale setting.
 *
 * @param gyro_fs The configured full scale macro (e.g., GYR_FS_250DPS).
 * @return The sensitivity in 'dps' (degrees per second) per LSB.
 */
// static float get_gyro_sensitivity(uint8_t gyro_fs) {
//     switch(gyro_fs) {
//         case GYR_FS_250DPS:  return 8.75f / 1000.0f;
//         case GYR_FS_500DPS:  return 17.50f / 1000.0f;
//         case GYR_FS_1000DPS: return 35.0f / 1000.0f;
//         case GYR_FS_2000DPS: return 70.0f / 1000.0f;
//         default:             return 8.75f / 1000.0f; // Default to 250 dps if invalid
//     }
// }
