# AS7341 Light Sensor Implementation Guide

This guide breaks down how the **ams AS7341 11-Channel Spectral Sensor** is implemented in this project. Because reading all 11 light channels takes significant time (over 100ms), the firmware uses a non-blocking asynchronous state machine over I2C.

Here is how you can replicate this architecture in your own project.

---

## 1. Hardware Initialization (I2C)

The AS7341 communicates via standard I2C (default address `0x39`). Before the sensor can be used, the STM32's hardware I2C peripheral must be initialized via CubeMX (e.g., `MX_I2C1_Init()`), which configures the SDA and SCL pins.

Once the hardware is ready, the sensor is booted and configured:

```c
// 1. Initialize the sensor (checks connection and powers up)
if (AS7341_Init() != 1) {
    // Handle I2C failure
}

// 2. Configure the Integration Time and Sensitivity (Gain)
// The longer the ATIME and ASTEP, the more light it collects (better in dark rooms).
AS7341_ConfigTimingAndGain(
    AS7341_PROCESSING_ATIME,   // E.g., 9
    AS7341_PROCESSING_ASTEP,   // E.g., 999
    AS7341_PROCESSING_GAIN     // E.g., AS7341_GAIN_8X
);
```

---

## 2. The SMUX (Multiplexer) Problem

The AS7341 can measure 11 different types of light (F1 through F8, Clear, Near-Infrared, and Flicker). **However, it only has 6 physical ADCs (Analog-to-Digital Converters).** 

To read all 11 channels, the firmware must perform a two-step process using the internal SMUX (Sensor Multiplexer):
1. **Low Phase**: Map the ADCs to read channels `F1, F2, F3, F4, Clear, NIR`.
2. **High Phase**: Re-map the ADCs to read channels `F5, F6, F7, F8, Clear, NIR`.

---

## 3. The Asynchronous State Machine

Because integrating light takes a long time, the MCU **cannot** just sit and wait. If it did, it would block the IMU and Bluetooth from running! 

Instead, the firmware uses an asynchronous state machine (`AS7341_ProcessFullSpectrumAsync`). 

### Step A: Starting the Measurement
When the `SensorSuperframe` decides it's time to check the light (during the 500ms `PHASE_ENV` window), it kicks off the process:
```c
AS7341_Spectrum s_async_spectrum;

// This function returns instantly! It just sets the state machine to START.
AS7341_StartFullSpectrumAsync(&s_async_spectrum);
```

### Step B: The Process Loop
Inside the main loop, `LightMetrics_ProcessPendingSample(now_ms)` is called continuously. It checks the `as7341_async.state` and advances it when the sensor is ready:

```c
AS7341_AsyncResult result = AS7341_ProcessFullSpectrumAsync(now_ms);

if (result == AS7341_ASYNC_COMPLETE) {
    // We successfully read all 11 channels!
    light_exposure_state_valid = 1;
}
```

### Inside the State Machine (`as7341_driver.c`)
Here is how the state machine advances without blocking the CPU:

1. **`AS7341_ASYNC_SETUP_LOW`**: Sends the I2C commands to configure the SMUX for the "Low Phase" (F1-F4) and sets a deadline timeout.
2. **`AS7341_ASYNC_WAIT_SMUX_LOW`**: Returns instantly if the SMUX isn't ready. Once the AS7341 confirms the SMUX is mapped, advances to start.
3. **`AS7341_ASYNC_START_LOW`**: Sends the `AS7341_SP_EN` I2C command to physically start measuring light. Sets an integration deadline.
4. **`AS7341_ASYNC_WAIT_DATA_LOW`**: Returns instantly until `now_ms` passes the deadline. Once time is up, reads the F1-F4 registers via I2C.
5. **`AS7341_ASYNC_SETUP_HIGH`**: Reconfigures the SMUX for the "High Phase" (F5-F8).
6. **`AS7341_ASYNC_WAIT_SMUX_HIGH`** -> **`AS7341_ASYNC_START_HIGH`** -> **`AS7341_ASYNC_WAIT_DATA_HIGH`**: (Repeats the exact same waiting and reading process for the upper channels).
7. **`AS7341_ASYNC_COMPLETE`**: Both halves are stitched together into the `AS7341_Spectrum` struct.

---

## 4. Processing the Spectrum & Color Temperature

Once the 11-channel spectrum is successfully captured, the data is pushed into the **Light Metrics DSP Pipeline**.

### Hardware Spectral Calibration
The AS7341 is built on silicon, which physically absorbs Near-Infrared/Red light about 4.5 times more efficiently than Violet/Blue light. To calculate accurate color temperatures, the firmware first equalizes this hardware bias by applying an inverse-sensitivity multiplier array to the 8 spectral channels:
- `F1` (Violet) receives a `4.5x` multiplier.
- `F8` (Red) receives a `1.0x` multiplier.

### Color Temperature Calculation
The firmware uses a 3D Euclidean distance algorithm to match the calibrated RGB output to the nearest point on the Planckian Locus (Black Body Radiation curve). The algorithm calculates the distance `sqrt(dr^2 + dg^2 + db^2)` between the measured light and a hardcoded Kelvin lookup table, returning a precise `uint16_t` color temperature (e.g., 6000K).

### DSP Accumulation (500ms Window)
The firmware collects roughly **6 full spectrum samples** during the 500ms `PHASE_ENV` window. 
```c
// Step 1: Accumulate temperature for every valid sample
if (s_light_temp_count < SAMPLE_SIZE) {
    s_light_temps[s_light_temp_count++] = get_light_temperature((uint16_t*)corrected);
}

// Step 2: Average and finalize at the end of the 500ms window
if (s_light_temp_count > 0) {
    result->color_temp = (uint16_t)get_avg_light_temperature(s_light_temps);
}
```
An internal `remove_outsiders` function cleans the array of outliers (noise spikes) before computing the final average.

### Blue Light Ratio & Brightness
The firmware also computes absolute brightness (`light_ambient`) and the **Blue Light Ratio** (`blue_clear_ratio`), which tracks the intensity of the `F3` (Blue) channel relative to the `Clear` channel. *Note: In direct sunlight, sensor saturation can cause this ratio to artificially peg to 100%+ without a physical diffuser.*

These final extracted features (`light_ambient`, `color_temp`, and `blue_clear_ratio`) are packaged into the unified `0x55` BLE packet and broadcasted to the phone 5 times per 2.5-second phase cycle!
