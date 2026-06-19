# MainBoard IMU Logger

Firmware and Flutter companion app for a wearable sensor board based on the **STM32U5A5** microcontroller. The device captures motion (LSM6DSO16IS IMU), spectral light (AS7341), ambient audio (IMP34DT05TR MEMS microphone), and streams data over Bluetooth Low Energy (RN4871) to a paired mobile device.

---

## Hardware Components

| Part | Role |
|---|---|
| STM32U5A5 | Main MCU — sensor orchestration, BLE bridge, NAND write |
| LSM6DSO16IS | 6-axis IMU (accelerometer + gyroscope) via I²C |
| AS7341 | 10-channel spectral light sensor via I²C |
| IMP34DT05TR | MEMS PDM microphone |
| RN4871 | Bluetooth Low Energy module via UART3 |
| MT29F4G01ABAFDWB | 4 Gb SPI NAND Flash (2048 blocks × 64 pages × 4096 B) |

---

## Repository Structure

```
MainBoard_IMU_Logger/
├── Core/
│   ├── Inc/
│   │   ├── main.h                # AppState enum, peripheral handles, global externs
│   │   ├── imu_driver.h          # LSM6DSO16IS register map & raw read API
│   │   ├── imu_metrics.h         # Step counting, cadence & activity-state API
│   │   ├── imu_ring_buffer.h     # Lock-free power-of-2 ring buffer for IMU samples
│   │   ├── as7341_driver.h       # AS7341 SMUX config, two-pass readout API
│   │   ├── light_metrics_mcu.h   # Q15 light metric computation (lux, CCT, PPFD, etc.)
│   │   ├── bluetooth.h           # RN4871 UART command API & BLE packet types
│   │   ├── led_driver.h          # Status LED blink patterns
│   │   ├── Memory_operations.h   # High-level NAND bad-block bookkeeping
│   │   ├── SPI_NAND.h            # Low-level NAND page / block / cache ops
│   │   └── SPI.h                 # SPI HAL wrapper typedefs
│   └── Src/
│       ├── main.c                # AppState machine, main task loop
│       ├── imu_driver.c          # LSM6DSO16IS init & sample read
│       ├── imu_metrics.c         # Step detector, cadence filter, activity classifier
│       ├── imu_ring_buffer.c     # Ring buffer push / pop implementation
│       ├── as7341_driver.c       # AS7341 init, SMUX reconfiguration, channel read
│       ├── light_metrics_mcu.c   # Illuminance, CCT, PPFD, SunLikeIndex, UV Index
│       ├── bluetooth.c           # RN4871 init, BLE packet build & UART transmit
│       ├── led_driver.c          # LED GPIO helpers
│       ├── Memory_operations.c   # scan_bad_blocks(), erase_bad_blocks()
│       └── SPI_NAND.c            # Page read/program/erase, spi_write/spi_read
├── USB_Device/                   # STM32 USB CDC middleware (auto-generated)
├── USB_Middlewares/
├── Drivers/                      # STM32U5 HAL + CMSIS (auto-generated)
├── analysis/                     # Off-device Python scripts for logged data
├── CMakeLists.txt
├── MainBoard_IMU_Logger.ioc      # STM32CubeMX project file
└── README.md
```

---

## Firmware Architecture

### AppState Machine

`main.c` owns a single `AppState current_state` variable. All other modules reference it via `extern`. State transitions:

```
STATE_IDLE
  │  button press / BLE connect
  ▼
STATE_RECORDING  ──────────────────────────────────────────────────────┐
  │  sensor loop: IMU @ 100 Hz, light @ ~1 Hz, BLE stream @ 1 Hz      │
  │  NAND write when sample buffer full                                 │
  ▼                                                                    │
STATE_USB_CONNECTED   (USB cable detected → dump NAND via CDC)        │
  │  read_memory_and_transmit() → 'T' sentinel → back to IDLE         │
  └───────────────────────────────────────────────────────────────────►┘
```

### Task Rates

| Sensor | Sample rate | BLE stream rate |
|---|---|---|
| Accelerometer (LSM6DSO16IS) | 104 Hz | ~1 Hz (bundled with LIGHT packet) |
| Gyroscope (LSM6DSO16IS) | 104 Hz | ~1 Hz |
| AS7341 light | SMUX cycle (~1 s) | 1 Hz |
| IMU metrics (step / cadence) | 100 Hz update | 1 Hz (inside LIGHT packet) |

### Key Design Decisions

- **Q15 fixed-point arithmetic** for all light metrics — avoids soft-FPU overhead on the Cortex-M33.
- **Metric-only BLE streaming** — raw 12-bit AS7341 counts are condensed into 5 derived metrics before transmission, keeping packets within the 20-byte ATT MTU.
- **Little-endian wire format** throughout (matches STM32 native byte order).
- **Bad-block table** (`bad_blocks[2048]`) built at startup by `scan_bad_blocks()` — only validated entries are ever written.
- **Ring buffer (`imu_ring_buffer`)** — power-of-2 capacity, index-masked, separates 104 Hz acquisition from the 1 Hz BLE drain path without dynamic allocation.
- **Single `AppState` owner** — `current_state` is defined once in `main.c`; all other translation units use `extern AppState current_state` to avoid silent state divergence.

---

## BLE Communication Protocol

All packets are exactly **20 bytes** to fit within a single BLE notification (ATT MTU = 23 B, 3 B overhead).

### Packet Header (bytes 0–1)

| Byte | Field | Description |
|---|---|---|
| 0 | `MsgType` | Message type identifier (see table below) |
| 1 | `seq` | Rolling 8-bit sequence number |

### `MsgType` Reference

| Value | Name | Payload |
|---|---|---|
| `0x01` | `ACCEL` | X, Y, Z acceleration (Q15, ±4 g) |
| `0x02` | `GYRO` | X, Y, Z angular rate (Q15, ±250 dps) |
| `0x03` | `LIGHT` | 5 spectral metrics + 3 IMU metrics |
| `0xFF` | `HEARTBEAT` | Uptime ticks (4 B), battery % (1 B), reserved |

### Packet Layouts

#### `ACCEL` (type `0x01`)

```
Byte  0     1     2–3    4–5    6–7    8–19
      0x01  seq   X_q15  Y_q15  Z_q15  reserved (0x00)
```

#### `GYRO` (type `0x02`)

```
Byte  0     1     2–3    4–5    6–7    8–19
      0x02  seq   X_q15  Y_q15  Z_q15  reserved (0x00)
```

Sensitivity:
- Accelerometer: `0.000061 g / LSB` (±4 g FS, Q15)
- Gyroscope: `0.00763 dps / LSB` (±250 dps FS, Q15)

#### `LIGHT` (type `0x03`)

```
Byte  0     1     2–3          4–5    6–7          8–9           10–11
      0x03  seq   Illuminance  CCT_K  PPFD_umol    SunLikeIdx    UV_Index

Byte  12–13       14        15              16–19
      stepCount   cadence   activityState   reserved (0x00)
```

| Field | Type | Units / notes |
|---|---|---|
| `Illuminance` | `uint16_t` | lux |
| `CCT_K` | `uint16_t` | Kelvin |
| `PPFD_umol` | `uint16_t` | µmol/m²/s |
| `SunLikeIdx` | `uint16_t` | scaled ×1000 (0–1000) |
| `UV_Index` | `uint16_t` | scaled ×10 |
| `stepCount` | `uint16_t` | cumulative steps (wraps at 65535) |
| `cadence` | `uint8_t` | steps/min |
| `activityState` | `uint8_t` | `0`=IDLE, `1`=WALKING, `2`=RUNNING |

---

## Light Metrics (AS7341 + `light_metrics_mcu`)

### Channel Map

| Channel | Centre λ | Role |
|---|---|---|
| F1 | 415 nm | Violet |
| F2 | 445 nm | Blue |
| F3 | 480 nm | Cyan |
| F4 | 515 nm | Green |
| F5 | 555 nm | Yellow-green |
| F6 | 590 nm | Amber |
| F7 | 630 nm | Red |
| F8 | 680 nm | Deep red |
| NIR | 855 nm | Near-infrared (SunLikeIndex only) |
| Clear | broadband | Illuminance reference |

### Derived Metrics (`light_metrics_mcu.c`)

| Metric | Definition |
|---|---|
| **Illuminance** (lux) | Weighted sum of F1–F8 using CIE 1931 V(λ) approximation, Q15 |
| **CCT** (K) | Correlated Colour Temperature via McCamy's approximation from chromaticity (x, y) |
| **PPFD** (µmol/m²/s) | Photosynthetically active radiation (400–700 nm), F3–F7 weighted by photon energy |
| **SunLikeIndex** | Ratio of NIR / (F1–F8 visible sum); high outdoors, low under artificial light |
| **UV Index** | Scaled F1 (415 nm) count; rough proxy — not a calibrated radiometric value |

#### SunLikeIndex Discrimination

| SunLikeIndex | Environment |
|---|---|
| > 0.35 | Outdoor / direct sun |
| 0.15 – 0.35 | Mixed / near window |
| < 0.15 | Indoor / artificial light |

---

## IMU Metrics (`imu_metrics.c`)

### Raw Sensor Configuration

- **Accelerometer FS**: ±4 g → sensitivity `0.000061 g/LSB`
- **Gyroscope FS**: ±250 dps → sensitivity `0.00763 dps/LSB`

### Computed Metrics

| Metric | Description |
|---|---|
| **Step count** | Cumulative `uint32_t`; incremented by a threshold-crossing detector on the vertical acceleration axis at 100 Hz. Resets on power cycle. |
| **Cadence** | Steps per minute, computed over a sliding 5-second window. |
| **Activity state** | `IDLE` / `WALKING` / `RUNNING` classified from cadence thresholds and resultant acceleration magnitude. |

### Ring Buffer (`imu_ring_buffer.c`)

A statically allocated, power-of-2 ring buffer decouples the 104 Hz ISR-driven acquisition from the 1 Hz BLE drain. Capacity and element size are configured at compile time in `imu_ring_buffer.h`. Index wrapping uses bitwise AND (no modulo).

---

## Flutter App Architecture

```
┌─────────────────────────────────────┐
│           Presentation Layer         │
│  LiveDashboard  SessionHistory  Map  │
└───────────────┬─────────────────────┘
                │ Riverpod providers
┌───────────────▼─────────────────────┐
│            Domain Layer              │
│  SensorBuffer  SessionStore  Steps   │
└───────────────┬─────────────────────┘
                │
┌───────────────▼─────────────────────┐
│         Data / BLE Layer             │
│  MyStream  BleRepository  SqlStore   │
└─────────────────────────────────────┘
```

### Key Domain Classes

| Class | Responsibility |
|---|---|
| `MyStream` | Wraps `flutter_blue_plus` characteristic notifications; decodes 20-byte frames into typed `SensorEvent` objects |
| `MsgType` | Dart enum mirroring the firmware `MsgType` values (`ACCEL`, `GYRO`, `LIGHT`, `HEARTBEAT`) |
| `SensorBuffer` | Ring buffer (capacity 512) per sensor type; feeds live charts |
| `SessionStore` | Persists raw `sensor_snapshots` rows and writes `session_summary` on session end |

### In-Memory Aggregates (per session)

| Aggregate | Update trigger |
|---|---|
| Step count | Each `LIGHT` packet (`stepCount` field) |
| Cadence | Each `LIGHT` packet (`cadence` field) |
| Activity state | Each `LIGHT` packet (`activityState` field) |
| Mean illuminance | Each `LIGHT` packet |
| Mean CCT | Each `LIGHT` packet |
| Peak UV index | Each `LIGHT` packet |
| Distance (m) | Derived from step count × stride estimate |

---

## Database Schema

```
sessions                   sensor_snapshots
─────────────────          ─────────────────────────────────
id (PK)                    id (PK)
started_at DATETIME        session_id (FK → sessions.id)
ended_at DATETIME          ts DATETIME
step_count INTEGER         msg_type INTEGER
                           ch0 REAL   -- x_accel / illuminance
                           ch1 REAL   -- y_accel / CCT
session_summary            ch2 REAL   -- z_accel / PPFD
─────────────────          ch3 REAL   -- x_gyro  / SunLikeIdx
session_id (FK)            ch4 REAL   -- y_gyro  / UV_Index
mean_lux REAL              ch5 REAL   -- z_gyro  / stepCount
mean_cct REAL              ch6 REAL   --          cadence
peak_uv REAL               ch7 REAL   --          activityState
total_steps INTEGER
distance_m REAL
```

---

## Getting Started

### Firmware (STM32CubeIDE)

1. Open `MainBoard_IMU_Logger.ioc` in **STM32CubeIDE 1.15+**.
2. Build the `Release` configuration (or `Debug` for ST-LINK live watch).
3. Flash via ST-LINK (`Run → Debug`) or drag-and-drop `.hex` onto the DFU drive.
4. On first boot the firmware calls `scan_bad_blocks()` (~3 s) then enters `STATE_IDLE`.

### Firmware (CMake / VS Code)

```bash
cmake --preset Debug   # or Release
cmake --build build/Debug
```

The `CMakePresets.json` at the repo root configures the ARM GCC toolchain automatically.

### Flutter App

```bash
flutter pub get
flutter run
```

Requires Flutter 3.19+ and a device with BLE support. Grant `BLUETOOTH_SCAN` + `BLUETOOTH_CONNECT` permissions on Android 12+.

### BLE Pairing

| Attribute | Value |
|---|---|
| Device name | `IMU_Logger` |
| Service UUID | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` (Nordic UART Service) |
| Notify characteristic | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |
| Write characteristic | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |

---

## Branch Guide

| Branch | Contents |
|---|---|
| `main` | Stable baseline — IMU logging to NAND only |
| `ble` | Adds RN4871 BLE streaming (IMU only) |
| `ble+light+imu` | **Active development** — AS7341 light + IMU metrics streamed over BLE |

### What's New in `ble+light+imu`

- `as7341_driver` — SMUX reconfiguration for two-pass 8-channel readout
- `light_metrics_mcu` — five derived light metrics (lux, CCT, PPFD, SunLikeIndex, UV Index) computed in Q15 fixed-point
- `imu_metrics` — step counter, cadence filter, and activity classifier (IDLE / WALKING / RUNNING) running at 100 Hz
- `imu_ring_buffer` — lock-free power-of-2 ring buffer decoupling acquisition from BLE drain
- `led_driver` — status LED patterns tied to `AppState` transitions
- **`LIGHT` packet extended** to 20 bytes: 5 spectral fields + `stepCount` + `cadence` + `activityState`
- Flutter `LightCard` widget with real-time lux / CCT gauges and `SunLikeIndex` outdoor/indoor badge
- Session summary persisted to SQLite including `mean_lux`, `mean_cct`, `peak_uv`, `total_steps`, `distance_m`
