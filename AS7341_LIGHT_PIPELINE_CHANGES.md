# AS7341 Light Pipeline Changes

## Files modified

- `Core/Src/main.c`
- `Core/Inc/Memory_operations.h`
- `Core/Src/Memory_operations.c`
- `Core/Inc/light_metrics_mcu.h`
- `Core/Src/light_metrics_mcu.c`

## Files created

- `Core/Inc/as7341_processing_config.h`
- `AS7341_LIGHT_PIPELINE_CHANGES.md`

## Previous architecture

- TIM2 queued 100 Hz sensor ticks.
- `ProcessSensorTick()` read IMU on each tick.
- Every `LIGHT_SUBSAMPLE` ticks it also read AS7341 full spectrum, detected mains flicker with `AS7341_DetectMainsHz()`, updated the old `LightMetrics_Update()` pipeline, and copied 22 raw light bytes into each 40-byte IMU sensor record.
- `NANDLogger_AppendSensorRecord()` wrote combined timestamp + IMU + raw light records into `SENS` pages.
- `NANDLogger_DownloadAll()` sent complete NAND pages over USB without interpreting payloads.

## New architecture

- IMU and audio acquisition behavior is preserved.
- The 40-byte sensor record remains unchanged in size.
- The legacy light area in the sensor record, offsets 17..38, is reserved and zero-filled.
- AS7341 processing is session based in `light_metrics_mcu.c`.
- One final light result is written after acquisition stop as a dedicated `LITE` page.
- `NANDLogger_DownloadAll()` still sends complete NAND pages opaquely, so USB framing is unchanged.
- `AS7341_DetectMainsHz()` remains in the driver but is no longer called by the active light path.

## Workflow: start

When `STATE_IDLE` receives `start_acquisition_requested`:

- erase good NAND blocks as before;
- reset timestamp and sensor tick counters;
- zero `raw_light[22]`;
- call `LightMetrics_StartSession(HAL_GetTick())`;
- start TIM2 and enter `STATE_ACQUISITION`.

## Workflow: acquisition

- TIM2 interrupt only increments `sensor_tick_pending`.
- The main loop consumes pending ticks with `ProcessSensorTick()`.
- `ProcessSensorTick()` reads accelerometer and gyroscope exactly as before, sends BLE IMU packets, updates timestamp, writes the 40-byte sensor record, and calls `LightMetrics_RequestSample()`.
- `LightMetrics_ProcessPendingSample()` runs in the main loop, not in interrupt context.
- The minimum AS7341 complete-sample period is configured by `AS7341_PROCESSING_MIN_PERIOD_MS`.

## Workflow: stop

When the button requests stop:

- `stop_acquisition_requested` is set in the GPIO callback.
- The main loop calls `StopAcquisition()`.
- TIM2 and microphone DMA are stopped as before.
- `LightMetrics_StopSession(HAL_GetTick())` records the stop time.
- `light_finalize_pending` is set.
- `current_state` returns to `STATE_IDLE`.

## Workflow: finalization

At the start of the `STATE_IDLE` case, if `light_finalize_pending` is set:

- `LightMetrics_FinalizeSession()` computes means, normalized signature, Clear index, class, duration, and sample count;
- `NANDLogger_AppendLightResult()` flushes any partial `SENS` page;
- a dedicated `LITE` page is serialized and written;
- the light session state is reset.

## SMUX handling and mapping

The driver uses `AS7341_ReadFullSpectrum()`, which performs two SMUX configurations. A sample is accepted only if the full function succeeds.

SMUX low configuration `as7341_smux_setup_F1F4_Clear_NIR()`:

| ADC output | Physical channel | `AS7341_Spectrum` index |
|---|---:|---:|
| CH0 | F1 | `ch[0]` |
| CH1 | F2 | `ch[1]` |
| CH2 | F3 | `ch[2]` |
| CH3 | F4 | `ch[3]` |
| CH4 | Clear | `ch[4]` |
| CH5 | NIR | `ch[5]` |

SMUX high configuration `as7341_smux_setup_F5F8_Clear_NIR()`:

| ADC output | Physical channel | `AS7341_Spectrum` index |
|---|---:|---:|
| CH0 | F5 | `ch[6]` |
| CH1 | F6 | `ch[7]` |
| CH2 | F7 | `ch[8]` |
| CH3 | F8 | `ch[9]` |
| CH4 | Clear | `ch[10]` |
| CH5 | NIR | `ch[11]` |

Clear and NIR are measured in both SMUX phases. The processing code averages the two corrected Clear readings into one Clear sample and averages the two corrected NIR readings into one NIR sample.

The driver does not expose a saturation flag through the active API. Saturation is therefore not rejected explicitly and is listed as an open calibration/driver limitation.

## Math

Dark-count correction:

```text
C_i,corr(k) = max(C_i(k) - D_i, 0)
```

Temporal mean:

```text
mean_i = sum_i / sample_count
```

If `sample_count == 0`, all means, normalized values and Clear index are zero, and the class is `LIGHT_LEVEL_DARK`.

Normalized signature:

```text
max_mean = max(mean_f1..mean_f8, mean_nir)
normalized_i = round(10000 * mean_i / max_mean)
```

Clear is excluded from normalization and from `max_mean`.

Clear index:

```text
ambient_light_index = clear_mean_counts
```

This is a relative mean-count index, not lux, irradiance, or luminance.

## Classes

- `0`: `LIGHT_LEVEL_DARK`
- `1`: `LIGHT_LEVEL_LOW`
- `2`: `LIGHT_LEVEL_NORMAL_INDOOR`
- `3`: `LIGHT_LEVEL_BRIGHT`
- `4`: `LIGHT_LEVEL_OUTDOOR`
- `5`: `LIGHT_LEVEL_DIRECT_SUN`

Initial thresholds in `Core/Inc/as7341_processing_config.h`:

- dark to low: 50 Clear counts
- low to normal: 500 Clear counts
- normal to bright: 5000 Clear counts
- bright to outdoor: 20000 Clear counts
- outdoor to direct sun: 50000 Clear counts

These thresholds are provisional, invented, not scientifically validated, and dependent on gain and integration time.

## Configuration location

All AS7341 processing parameters are in `Core/Inc/as7341_processing_config.h`.

- Gain: `AS7341_PROCESSING_GAIN`
- ATIME: `AS7341_PROCESSING_ATIME`
- ASTEP: `AS7341_PROCESSING_ASTEP`
- Integration time: `AS7341_PROCESSING_INTEGRATION_TIME_US`
- Minimum complete-sample period: `AS7341_PROCESSING_MIN_PERIOD_MS`
- Dark counts: `AS7341_DARK_*_COUNTS`
- Thresholds: `LIGHT_THRESHOLD_*`
- Normalization scale: `AS7341_NORMALIZATION_SCALE`
- Record version: `AS7341_LIGHT_RECORD_FORMAT_VERSION`
- Overflow policy: `AS7341_PROCESSING_DISCARD_ON_OVERFLOW`

To change gain, ATIME, ASTEP, dark counts, or thresholds, edit only that header.

## TODO_CALIBRATION list

The following macros are marked `TODO_CALIBRATION`:

- `AS7341_PROCESSING_GAIN`
- `AS7341_PROCESSING_ATIME`
- `AS7341_PROCESSING_ASTEP`
- `AS7341_DARK_F1_COUNTS`
- `AS7341_DARK_F2_COUNTS`
- `AS7341_DARK_F3_COUNTS`
- `AS7341_DARK_F4_COUNTS`
- `AS7341_DARK_F5_COUNTS`
- `AS7341_DARK_F6_COUNTS`
- `AS7341_DARK_F7_COUNTS`
- `AS7341_DARK_F8_COUNTS`
- `AS7341_DARK_NIR_COUNTS`
- `AS7341_DARK_CLEAR_COUNTS`
- `LIGHT_THRESHOLD_DARK_TO_LOW`
- `LIGHT_THRESHOLD_LOW_TO_NORMAL`
- `LIGHT_THRESHOLD_NORMAL_TO_BRIGHT`
- `LIGHT_THRESHOLD_BRIGHT_TO_OUTDOOR`
- `LIGHT_THRESHOLD_OUTDOOR_TO_SUN`

## NAND and USB page framing

Existing page header, little-endian STM32 struct layout:

| Offset | Type | Field |
|---:|---|---|
| 0 | `uint32_t` | magic |
| 4 | `uint8_t` | version |
| 5 | `uint8_t` | header_size |
| 6 | `uint16_t` | payload_bytes |
| 8 | `uint32_t` | page_sequence |
| 12 | `uint32_t` | timestamp_ms |

Light page magic:

```text
LOG_MAGIC_LIGHT = 0x4554494C = "LITE"
```

USB layout is unchanged:

- `LOGSTART` marker;
- `uint32_t total_pages` little-endian;
- each complete 4096-byte NAND page;
- `LOGEND!!` marker.

The new light record is available over USB because `NANDLogger_DownloadAll()` sends the `LITE` page exactly like `SENS` and `AUD0` pages.

## Light result payload layout

Payload size: 40 bytes. Endianness: little-endian. Normalization scale: `10000 = 1.0000`.

| Offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 2 | `uint16_t` | `format_version` |
| 2 | 2 | `uint16_t` | `normalized_f1` |
| 4 | 2 | `uint16_t` | `normalized_f2` |
| 6 | 2 | `uint16_t` | `normalized_f3` |
| 8 | 2 | `uint16_t` | `normalized_f4` |
| 10 | 2 | `uint16_t` | `normalized_f5` |
| 12 | 2 | `uint16_t` | `normalized_f6` |
| 14 | 2 | `uint16_t` | `normalized_f7` |
| 16 | 2 | `uint16_t` | `normalized_f8` |
| 18 | 2 | `uint16_t` | `normalized_nir` |
| 20 | 4 | `uint32_t` | `clear_mean_counts` |
| 24 | 4 | `uint32_t` | `sample_count` |
| 28 | 4 | `uint32_t` | `acquisition_duration_ms` |
| 32 | 4 | `uint32_t` | `session_start_ms` |
| 36 | 1 | `uint8_t` | `light_level_class` |
| 37 | 3 | `uint8_t[3]` | reserved, zero |

No raw F1-F8/NIR counts, flicker, lux, irradiance, saturation flag, or source classification are stored.

## Error handling

- I2C or SMUX failure: `AS7341_ReadFullSpectrum()` returns 0; sample is discarded and `sample_count` is unchanged.
- Incomplete sample: discarded because both SMUX phases must complete.
- Overflow: if any accumulator would overflow, the entire sample is discarded and `sample_count` is unchanged.
- Stop before a sample starts: no new sample starts because pending work is skipped when `stop_acquisition_requested` is set.
- Stop during a blocking sample: the sample is accepted only if `AS7341_ReadFullSpectrum()` completes successfully.
- `sample_count == 0`: result fields are zero except `format_version`, timestamps/duration, and class `LIGHT_LEVEL_DARK`.
- NAND write failure of final light page: red LED is turned on; no retry policy was added.
- USB download during finalization: `STATE_IDLE` finalizes before accepting USB-connected/download behavior.
- Double start: start is driven by `start_acquisition_requested`; session state is reset at each start.
- Double finalization: `light_finalize_pending` is cleared before write.

## Parser Python impact

The parser must be updated to recognize `LOG_MAGIC_LIGHT` pages and parse the 40-byte payload above. Existing `SENS` sensor pages still have 40-byte records, but offsets 17..38 no longer contain AS7341 raw samples. They should be treated as reserved zeros.

Because only normalized signature and Clear mean are stored, the parser cannot reconstruct absolute mean counts for F1-F8/NIR after download.

## Build result

Command run:

```powershell
cmake --build build
```

Result: failed.

Observed application-code issue fixed during this work:

- `LED_BLUE` was referenced in `main.c` but the LED driver defines only `LED_GREEN` and `LED_RED`. The undefined calls were removed.

Current blocking build errors are in generated/HAL compilation and assembler target selection, not in the AS7341 pipeline code:

- `selected processor does not support 'dsb 0xF' in ARM mode`
- `selected processor does not support 'isb 0xF' in ARM mode`
- `selected processor does not support 'dmb 0xF' in ARM mode`
- `selected processor does not support 'cpsid i' in ARM mode`
- `selected processor does not support requested special purpose register -- 'mrs r3,primask'`

The compile command shown by CMake lacks an evident Cortex-M CPU/Thumb target flag. Toolchain/build configuration was not changed.

Warning still observed before the assembler failure:

- `Core/Src/SPI_NAND.c`: passing a `const uint8_t *` to `spi_write(uint8_t *)` discards `const`.

## Assumptions and open issues

- The USB protocol is page-opaque; no CDC framing change is required.
- The host parser is expected to use page magic and payload size.
- The AS7341 driver mapping in comments matches the current SMUX setup functions.
- Clear/NIR duplicated across SMUX phases are averaged.
- Saturation is not detectable through the current public AS7341 API.
- Thresholds and dark counts require experimental calibration.
- The final light page is one page even though the payload is only 40 bytes.
- No CRC was found in the active NAND page format.

## Git operations

- No commit was executed.
- No push was executed.
- No branch was created.
- No branch was changed.
- Work remained on branch `Application_light_sensor`.
