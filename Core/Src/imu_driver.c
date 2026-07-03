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
#include "StepCounter.h"

extern I2C_HandleTypeDef hi2c3;

// --- Private Global Variables ---
// Store the configured full-scale settings. These are used to calculate the
// correct sensitivity for data conversion.
static uint8_t accelerometer_full_scale;
static uint8_t gyroscope_full_scale;

static inline float imu_acc_sensitivity_fs2g(void) { return 2.0f / 32767.0f; }
static inline float imu_gyro_sensitivity_175lsb_dps(void) { return 1.0f / 175.0f; }

// --- Private Function Prototypes (Helper Functions) ---
// These functions are only used internally by the driver
static uint8_t imu_read_register(uint8_t reg_addr, uint8_t* data, uint16_t data_len);
static uint8_t imu_write_register(uint8_t reg_addr, uint8_t reg_data);
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

uint8_t IMU_EnableCoherentReads(void) {
    uint8_t ctrl3 = 0U;

    if (!imu_read_register(IMU_CTRL3_REG, &ctrl3, 1U)) {
        return 0U;
    }

    ctrl3 |= (IMU_CTRL3_BDU | IMU_CTRL3_IF_INC);
    return imu_write_register(IMU_CTRL3_REG, ctrl3);
}

uint8_t IMU_ReadCombinedData(IMU_Data *acc_data,
                             IMU_Data *gyro_data,
                             uint8_t *raw_accelerometer,
                             uint8_t *raw_gyroscope,
                             uint8_t *step_count) {
    uint8_t raw[12];
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    const float acc_sensitivity = imu_acc_sensitivity_fs2g();
    const float gyro_sensitivity = imu_gyro_sensitivity_175lsb_dps();

    if ((acc_data == NULL) || (gyro_data == NULL) ||
        (raw_accelerometer == NULL) || (raw_gyroscope == NULL)) {
        return 0U;
    }

    /* Gyroscope 0x22..0x27 and accelerometer 0x28..0x2D are contiguous. */
    if (!imu_read_register(IMU_GYR_OUT_X_L_REG, raw, sizeof(raw))) {
        return 0U;
    }

    memcpy(raw_gyroscope, &raw[0], 6U);
    memcpy(raw_accelerometer, &raw[6], 6U);

    gyro_x = (int16_t)(((uint16_t)raw[1] << 8U) | raw[0]);
    gyro_y = (int16_t)(((uint16_t)raw[3] << 8U) | raw[2]);
    gyro_z = (int16_t)(((uint16_t)raw[5] << 8U) | raw[4]);
    acc_x = (int16_t)(((uint16_t)raw[7] << 8U) | raw[6]);
    acc_y = (int16_t)(((uint16_t)raw[9] << 8U) | raw[8]);
    acc_z = (int16_t)(((uint16_t)raw[11] << 8U) | raw[10]);
    
    gyro_data->x = (float)gyro_x * (float)DEG2RAD * gyro_sensitivity;
    gyro_data->y = (float)gyro_y * (float)DEG2RAD * gyro_sensitivity;
    gyro_data->z = (float)gyro_z * (float)DEG2RAD * gyro_sensitivity;
    acc_data->x = (float)acc_x * 9.81f * acc_sensitivity;
    acc_data->y = (float)acc_y * 9.81f * acc_sensitivity;
    acc_data->z = (float)acc_z * 9.81f * acc_sensitivity;

    // Send the accelerometer and gyroscope data via BLE
    StepCounter_U.In1[0] = (real_T)(float)gyro_data->x;
    StepCounter_U.In1[1] = (real_T)(float)gyro_data->y;
    StepCounter_U.In1[2] = (real_T)(float)gyro_data->z;

    StepCounter_U.In2[0] = (real_T)(float)acc_data->x;
    StepCounter_U.In2[1] = (real_T)(float)acc_data->y;
    StepCounter_U.In2[2] = (real_T)(float)acc_data->z;

    /* --- Read steps   ---*/
    StepCounter_step();
    *step_count = (uint8_t)((StepCounter_Y.stepnumber > 0U) ? (StepCounter_Y.stepnumber - 1U) : 0U);

    return 1U;
}

/**
 * @brief Reads the accelerometer data from the sensor and converts it to 'g'.
 * @param acc_data Pointer to an IMU_Data struct to store the converted data.
 */
void IMU_ReadAccelerometerData(IMU_Data *acc_data, uint8_t *raw_data) {
    //uint8_t raw_data[6]; // Buffer to hold 6 bytes for X, Y, Z axes
    int16_t x_raw, y_raw, z_raw;
    float sensitivity = 2.0 / 32767.0;

    // Read 6 bytes of data starting from the first accelerometer register.
    // The sensor auto-increments the register address, so a single read operation
    // fetches all 3 axes (X, Y, Z).
    imu_read_register(IMU_ACC_OUT_X_L_REG, raw_data, 6);

    // Combine LSB and MSB to form a signed 16-bit integer for each axis
    x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Get the correct sensitivity factor based on the configured full scale
    // sensitivity = get_accel_sensitivity(accelerometer_full_scale);

    // Convert raw data to 'g' (gravitational force) and store it in the struct
    acc_data->x = (double)x_raw * 9.81f * sensitivity;
    acc_data->y = (double)y_raw * 9.81f * sensitivity;
    acc_data->z = (double)z_raw * 9.81f * sensitivity;
}

/**
 * @brief Reads the gyroscope data from the sensor and converts it to 'dps'.
 * @param gyro_data Pointer to an IMU_Data struct to store the converted data.
 */
void IMU_ReadGyroscopeData(IMU_Data *gyro_data, uint8_t *raw_data) {
    //uint8_t raw_data[6]; // Buffer to hold 6 bytes for X, Y, Z axes
    int16_t x_raw, y_raw, z_raw;
    float sensitivity = 1.0 / 175.0;

    // Read 6 bytes of data starting from the first gyroscope register
    imu_read_register(IMU_GYR_OUT_X_L_REG, raw_data, 6);

    // Combine LSB and MSB to form a signed 16-bit integer for each axis
    x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
    y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
    z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    // Get the correct sensitivity factor based on the configured full scale
    //sensitivity = get_gyro_sensitivity(gyroscope_full_scale);

    // Convert raw data to 'dps' (degrees per second) and store it in the struct
    gyro_data->x = (double)x_raw * DEG2RAD * sensitivity;
    gyro_data->y = (double)y_raw * DEG2RAD * sensitivity;
    gyro_data->z = (double)z_raw * DEG2RAD * sensitivity;
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