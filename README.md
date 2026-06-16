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
| MT29F4G01ABAFDWB | 4 Gb SPI NAND Flash (512 blocks × 64 pages × 4096 B) |

---

## Repository Structure

```
MainBoard_IMU_Logger/
├── Core/
│   ├── Inc/
│   │   ├── main.h               # AppState enum, global externs
│   │   ├── imu_driver.h         # LSM6DSO16IS register map & API
│   │   ├── bluetooth.h          # RN4871 UART API
│   │   ├── light_sensor.h       # AS7341 channel map & metrics
│   │   ├── Memory_operations.h  # High-level NAND bookkeeping
│   │   ├── SPI_NAND.h           # Low-level NAND page/block ops
│   │   └── SPI.h                # SPI HAL wrappers
│   └── Src/
│       ├── main.c               # Task loop, AppState machine
│       ├── imu_driver.c
│       ├── bluetooth.c
│       ├── light_sensor.c
│       ├── Memory_operations.c
│       └── SPI_NAND.c
└── README.md
```

---

## Firmware Architecture

### Task Rates

| Sensor | ODR | BLE stream rate |
|---|---|---|
| Accelerometer | 104 Hz | 50 Hz (every 2nd sample) |
| Gyroscope | 104 Hz | 50 Hz |
| AS7341 light | SMUX cycle | ~10 Hz |

### Key Design Decisions

- **Q15 fixed-point arithmetic** for light metrics — avoids soft-FPU overhead on the Cortex-M33.
- **Metric-only BLE streaming** — raw 12-bit AS7341 counts are condensed into 5 derived metrics before transmission, keeping packets within the 20-byte ATT MTU.
- **Little-endian wire format** throughout (matches STM32 native byte order).
- **Bad-block table** (`bad_blocks[2048]`) is built at startup by scanning the first page of every block for the `0xFF` bad-block marker; only entries in this table are ever written.

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
| `0x03` | `LIGHT` | 5 derived spectral metrics |
| `0xFF` | `HEARTBEAT` | Uptime ticks (4 B), battery % (1 B), reserved |

### Packet Layouts

#### `ACCEL` / `GYRO` (type `0x01` / `0x02`)

```
Byte  0     1     2–3    4–5    6–7    8–19
      type  seq   X_q15  Y_q15  Z_q15  reserved (0x00)
```

Sensitivity:
- Accelerometer: `0.000061 g / LSB` (±4 g FS, Q15)
- Gyroscope: `0.00763 dps / LSB` (±250 dps FS, Q15)

#### `LIGHT` (type `0x03`)

```
Byte  0     1     2–3       4–5       6–7         8–9          10–11        12–19
      0x03  seq   Illuminance  CCT_K   PPFD_umol   SunLikeIdx   UV_Index   reserved
```

All light fields are `uint16_t`, little-endian.

---

## Light Metrics (AS7341)

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
| NIR | 855 nm | Near-infrared (excluded from visible metrics) |
| Clear | broadband | Illuminance reference |

### Derived Metrics

| Metric | Definition |
|---|---|
| **Illuminance** (lux) | Weighted sum of F1–F8 using CIE 1931 V(λ) approximation |
| **CCT** (K) | Correlated Colour Temperature via McCamy's approximation from chromaticity (x, y) |
| **PPFD** (µmol/m²/s) | Photosynthetically active radiation (400–700 nm), F3–F7 weighted by photon energy |
| **SunLikeIndex** | Ratio of NIR / (F1–F8 visible sum); high outdoors, low under artificial light |
| **UV Index** | Scaled F1 (415 nm) count; rough proxy only — not a calibrated radiometric value |

#### SunLikeIndex Discrimination

| SunLikeIndex | Environment |
|---|---|
| > 0.35 | Outdoor / direct sun |
| 0.15 – 0.35 | Mixed / near window |
| < 0.15 | Indoor / artificial light |

---

## IMU & Step Counting

- **Accelerometer FS**: ±4 g → sensitivity `0.000061 g/LSB`
- **Gyroscope FS**: ±250 dps → sensitivity `0.00763 dps/LSB`
- **Step count**: cumulative, stored as `uint32_t`; incremented by a threshold-crossing detector on the vertical acceleration axis running at 50 Hz. The counter resets only on explicit BLE command or power cycle.

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
| `MsgType` | Dart enum mirroring the firmware `MsgType` values |
| `SensorBuffer` | Ring buffer (capacity 512) per sensor type; feeds live charts |
| `SessionStore` | Persists raw `sensor_snapshots` rows and writes `session_summary` on session end |

### In-Memory Aggregates (per session)

| Aggregate | Update frequency |
|---|---|
| Step count | Each `ACCEL` packet |
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
                           ch0 REAL   -- x / illuminance / —
                           ch1 REAL   -- y / CCT / —
session_summary            ch2 REAL   -- z / PPFD / —
─────────────────          ch3 REAL   -- — / SunLikeIdx / —
session_id (FK)            ch4 REAL   -- — / UV_Index / —
mean_lux REAL
mean_cct REAL
peak_uv REAL
total_steps INTEGER
distance_m REAL
```

---

## Getting Started

### Firmware

1. Open `MainBoard_IMU_Logger.ioc` in **STM32CubeIDE 1.15+**.
2. Build the `Release` configuration.
3. Flash via ST-LINK (`Run → Debug` or drag-and-drop `.hex` onto the DFU drive).
4. On first boot the firmware runs `scan_bad_blocks()` (~3 s) then enters `STATE_IDLE`.

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
| `ble+light+imu` | **Active development** — adds AS7341 light metrics to BLE stream |

### What's New in `ble+light+imu`

- AS7341 driver with SMUX reconfiguration for two-pass 8-channel readout
- Five derived light metrics streamed as `MsgType 0x03` packets
- Flutter `LightCard` widget with real-time lux / CCT gauges
- `SunLikeIndex` outdoor/indoor badge in the live dashboard
- Session summary persisted to SQLite including `mean_lux`, `mean_cct`, `peak_uv`
