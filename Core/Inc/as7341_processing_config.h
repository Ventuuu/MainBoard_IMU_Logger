/*
 * as7341_processing_config.h
 *
 * Central configuration for the AS7341 session-level processing pipeline.
 */

#ifndef INC_AS7341_PROCESSING_CONFIG_H_
#define INC_AS7341_PROCESSING_CONFIG_H_

#include <stdint.h>

#include "as7341_driver.h"

/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_PROCESSING_GAIN AS7341_GAIN_8X

/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_PROCESSING_ATIME 9U

/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_PROCESSING_ASTEP 999U

/* t_int = (ATIME + 1) * (ASTEP + 1) * 2.78 us = 27800 us. */
#define AS7341_PROCESSING_INTEGRATION_TIME_US 27800UL

/* Two SMUX integrations plus read/configuration margin. */
#define AS7341_PROCESSING_MIN_PERIOD_MS 80UL

#define AS7341_LIGHT_RECORD_FORMAT_VERSION 1U
#define AS7341_NORMALIZATION_SCALE 10000U
#define AS7341_LIGHT_RESULT_RECORD_BYTES 44U

/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F1_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F2_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F3_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F4_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F5_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F6_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F7_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_F8_COUNTS    0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_NIR_COUNTS   0U
/* TODO_CALIBRATION: replace after experimental calibration */
#define AS7341_DARK_CLEAR_COUNTS 0U

/* Provisional thresholds in mean Clear counts at the configured gain/timing. */
/* TODO_CALIBRATION: replace after experimental calibration */
#define LIGHT_THRESHOLD_DARK_TO_LOW        50UL
/* TODO_CALIBRATION: replace after experimental calibration */
#define LIGHT_THRESHOLD_LOW_TO_NORMAL      500UL
/* TODO_CALIBRATION: replace after experimental calibration */
#define LIGHT_THRESHOLD_NORMAL_TO_BRIGHT   5000UL
/* TODO_CALIBRATION: replace after experimental calibration */
#define LIGHT_THRESHOLD_BRIGHT_TO_OUTDOOR  20000UL
/* TODO_CALIBRATION: replace after experimental calibration */
#define LIGHT_THRESHOLD_OUTDOOR_TO_SUN     50000UL

/* UINT64 accumulator overflow policy: discard the entire sample. */
#define AS7341_PROCESSING_DISCARD_ON_OVERFLOW 1U

/* HAL_GetTick() unsigned subtraction is used, so wrap-around is supported. */
#define AS7341_PROCESSING_MAX_DURATION_MS UINT32_MAX

#endif /* INC_AS7341_PROCESSING_CONFIG_H_ */
