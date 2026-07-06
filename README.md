# MainBoard Edge-DSP Wearable Logger

Firmware for a wearable sensor board based on the **STM32U5A5** microcontroller. The device captures motion (LSM6DSO16IS IMU) and spectral light (AS7341), performs on-device Digital Signal Processing (DSP) to derive physiological metrics, and streams a unified 2 Hz telemetry payload over Bluetooth Low Energy (RN4871).

---

## Hardware Components

| Part | Role |
|---|---|
| STM32U5A5 | Main MCU — sensor orchestration, Q15 DSP fusion, BLE bridge, NAND write |
| LSM6DSO16IS | 6-axis IMU (accelerometer + gyroscope) via I²C |
| AS7341 | 10-channel spectral light sensor via I²C |
| IMP34DT05TR | MEMS PDM microphone (Audio DSP / noise logging) |
| RN4871 | Bluetooth Low Energy module via UART3 |
| MT29F4G01ABAFDWB | 4 Gb SPI NAND Flash (2048 blocks × 64 pages × 4096 B) |

---

## Repository Structure

```text
MainBoard_IMU_Logger/
├── Core/
│   ├── Inc/
│   │   ├── main.h                # AppState enum, peripheral handles, global externs
│   │   ├── imu_driver.h          # LSM6DSO16IS register map & raw read API
│   │   ├── imu_metrics.h         # Step counting, cadence & activity-state API
│   │   ├── imu_ring_buffer.h     # Lock-free power-of-2 ring buffer for IMU samples
│   │   ├── as7341_driver.h       # AS7341 SMUX config, two-pass readout API
│   │   ├── light_metrics_mcu.h   # Photobiology DSP pipeline & Bluetooth packing
│   │   ├── light_temperature.h   # Color temperature algorithm & AS7341 calibration
│   │   ├── bluetooth.h           # RN4871 UART API & 0x7B/0x7D packet framing
│   │   ├── led_driver.h          # Status LED blink patterns
│   │   ├── Memory_operations.h   # High-level NAND bad-block bookkeeping
│   │   ├── SPI_NAND.h            # Low-level NAND page / block / cache ops
│   │   └── SPI.h                 # SPI HAL wrapper typedefs
│   └── Src/
│       ├── main.c                # AppState machine, 1 Hz telemetry fusion loop
│       ├── imu_driver.c          # LSM6DSO16IS init & sample read
│       ├── imu_metrics.c         # Step detector, cadence filter, activity classifier
│       ├── imu_ring_buffer.c     # Ring buffer push / pop implementation
│       ├── as7341_driver.c       # AS7341 init, SMUX reconfiguration, channel read
│       ├── light_metrics_mcu.c   # Photobiology pipeline execution
│       ├── light_temperature.c   # Euclidean distance color temp & equalization
│       ├── bluetooth.c           # RN4871 init, BLE packet build & UART transmit
│       ├── led_driver.c          # LED GPIO helpers
│       ├── Memory_operations.c   # scan_bad_blocks(), erase_bad_blocks()
│       └── SPI_NAND.c            # Page read/program/erase, spi_write/spi_read
├── USB_Device/                   # STM32 USB CDC middleware (auto-generated)
├── Drivers/                      # STM32U5 HAL + CMSIS (auto-generated)
├── CMakeLists.txt
├── MainBoard_IMU_Logger.ioc      # STM32CubeMX project file
└── README.md
```
--- 

## AppState Machine
```text
STATE_IDLE
  │  button press / BLE connect
  ▼
STATE_RECORDING  ──────────────────────────────────────────────────────┐
  │  ISR loop: IMU @ 104 Hz -> ring buffer -> step classifier          │
  │  1 Hz loop: Read AS7341 -> fuse metrics -> TX 0x55 BLE packet      │
  │  NAND write: Triggered when page buffer fills                      │
  ▼                                                                    │
STATE_USB_CONNECTED   (USB cable detected → dump NAND via CDC)         │
  │  read_memory_and_transmit() → 'T' sentinel → back to IDLE          │
  └────────────────────────────────────────────────────────────────────┘
```

## BLE Communication Protocol (v2)

All packets are exactly 20 bytes to fit securely within a single BLE notification (ATT MTU = 23 B). To ensure payload integrity on the receiving client, all packets are wrapped with standard framing bytes (0x7B Start, 0x7D End).

MsgType Reference
Byte 1 Value	Name	Usage
0x55	UNIFIED_STATE	Production: 1 Hz fused kinematic, photobiology, and audio telemetry
0x01	ACCEL	Dev Mode: High-frequency raw X,Y,Z acceleration
0x02	GYRO	Dev Mode: High-frequency raw X,Y,Z angular rate
0x0C	BATTERY	Production: Asynchronous battery level notifications

## UNIFIED_STATE Payload (0x55)

All multi-byte values are transmitted in Little-Endian byte order.
Frame layout (20 bytes):
- `[0]      Start sentinel   : 0x7B`
- `[1]      MsgType          : 0x55`
- `[2..3]   stepCount        : uint16_t (Cumulative robust steps)`
- `[4]      light_level_class: uint8_t (0=DARK, 1=LOW, 2=MOD, 3=HIGH, 4=VERY_HIGH)`
- `[5..6]   blue_clear_ratio : uint16_t (Q15 fixed-point fractional ratio)`
- `[7..8]   color_temp       : uint16_t (Color Temperature in Kelvin)`
- `[9..10]  laeq_x10         : uint16_t (A-weighted decibels * 10)`
- `[11]     audio_env_class  : uint8_t (0=QUIET, 1=MODERATE, 2=LOUD)`
- `[12..18] padding          : 0x00 * 7`
- `[19]     End sentinel     : 0x7D`

## Photobiology Metrics (light_metrics_mcu.c)

The AS7341 SMUX is dynamically reconfigured to capture both visible and NIR bands.
- **Light Level Class**: Derived directly from the AS7341 `Clear` channel counts. The firmware uses an 8x Gain and 27.8ms integration time, with thresholds specifically calibrated to this dynamic range (10, 80, 500, 1500) to distinguish darkness from direct light exposure.
- **Blue Ratio**: `F3 / Clear`. This accurately represents the proportion of blue light in the environment, transmitted as a Q15 fractional multiplier.
- **Color Temperature**: Computed by physically equalizing the 8-channel silicon responsivity bias (e.g., boosting F1 4.5x) and snapping the calibrated color to the nearest Kelvin value on the Planckian Locus via 3D Euclidean distance.

## Audio Metrics (mic_metrics.c)

The IMP34DT05TR PDM microphone captures high-frequency audio which is processed natively on-device.
- **LAeq (dBA)**: Calculates Equivalent Continuous Sound Level using standard A-Weighting DSP filters on 48kHz audio buffers. The baseline offset is calibrated to accurately reflect a ~45dBA room floor.
- **Audio Environment Class**: A hysteresis-filtered classifier dividing LAeq into `QUIET` (< 50 dBA), `MODERATE` (< 70 dBA), and `LOUD` (>= 70 dBA).

## IMU Metrics (imu_metrics.c / StepCounter.c)

Raw sensors are configured to ±4 g (Accel) and ±250 dps (Gyro).
- **Step Count**: Evaluated using a Simulink-generated Step Counter model. 
  - *Note: The model has been manually augmented with a robust rising-edge filter to completely eliminate false-positive step counting when the user is stationary.* 
  - *The inputs were also properly scaled to accept standard 'g' and 'm/s²' physical vectors.*

## Getting Started
Compiling & Flashing (STM32CubeIDE)

    Open MainBoard_IMU_Logger.ioc in STM32CubeIDE 1.15+.

    Build the Release configuration.

    Flash via ST-LINK (Run → Debug) or drag-and-drop the generated .hex file onto the STM32 DFU mass-storage drive.

    On first boot, the firmware calls scan_bad_blocks() (~3 s) to establish the NAND allocation table, then enters STATE_IDLE.

## Headless Build (CMake / VS Code)
cmake --preset Release
cmake --build build/Release

## BLE Interfacing
### Attribute	UUID
Service	6E400001-B5A3-F393-E0A9-E50E24DCCA9E
TX (Notify)	6E400003-B5A3-F393-E0A9-E50E24DCCA9E
RX (Write)	6E400002-B5A3-F393-E0A9-E50E24DCCA9E

## Branch Guide
### Branch	Description
main	Stable v1 baseline — High-frequency IMU logging directly to NAND.
edge-dsp-v2	Active deployment — On-device Q15 data fusion, 1 Hz telemetry framing (0x55), and decoupled lock-free IMU buffering.
