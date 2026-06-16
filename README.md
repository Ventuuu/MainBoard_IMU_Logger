# MainBoard\_IMU\_Logger

Firmware for the **STM32U5A5** wearable board that logs IMU and spectral-light data,
streams it over BLE, and archives it to NAND Flash.  
Active branch: **`ble+light+imu`**

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [Repository Structure](#2-repository-structure)
3. [Firmware Architecture](#3-firmware-architecture)
4. [BLE Communication Protocol](#4-ble-communication-protocol)
5. [Light Metrics](#5-light-metrics)
6. [IMU & Step Counting](#6-imu--step-counting)
7. [NAND Flash Logging](#7-nand-flash-logging)
8. [Getting Started](#8-getting-started)
9. [Branch Guide](#9-branch-guide)

---

## 1. Hardware Overview

| Modality | IC | Interface | Role |
|---|---|---|---|
| Spectral light | AS7341 | I2C3 | 10-channel visible + NIR, mains-flicker detect |
| IMU | LSM6DSO16IS | I2C3 | Accel + Gyro, machine-learning core |
| Microphone | IMP34DT05TR | MDF (PDM) | Digital MEMS microphone (future use) |
| BLE | RN4871 | USART3 @ 115 200 baud | Wireless data streaming |
| Storage | MT29F4G01ABAFDWB | SPI2 | 4 Gb NAND Flash |
| MCU | STM32U5A5 | — | Cortex-M33, 160 MHz |

---

## 2. Repository Structure

```
MainBoard_IMU_Logger/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── imu_driver.h
│   │   ├── as7341_driver.h
│   │   ├── light_metrics_mcu.h
│   │   ├── bluetooth.h
│   │   ├── SPI_NAND.h
│   │   ├── Memory_operations.h
│   │   └── led_driver.h
│   └── Src/
│       ├── main.c                  ← state machine, TIM2 ISR, sensor fusion
│       ├── imu_driver.c            ← LSM6DSO16IS read/config
│       ├── as7341_driver.c         ← full-spectrum read, mains-flicker detect
│       ├── light_metrics_mcu.c     ← exposure / circadian / SunLikeIndex
│       ├── bluetooth.c             ← RN4871 init, typed packet builder
│       ├── SPI_NAND.c              ← low-level NAND Flash driver
│       └── Memory_operations.c     ← write_packet / write_memory / read_memory
├── USB_Device/                     ← ST USB VCP middleware
└── README.md
```

---

## 3. Firmware Architecture

### State Machine

```
STATE_IDLE
  │  USER_BUTTON (rising)  → erase NAND, start TIM2
  ▼
STATE_ACQUISITION          ← TIM2 ISR fires at 100 Hz
  │  USER_BUTTON (rising)  → stop TIM2
  ▼
STATE_IDLE

(USB cable detected at any time)
  ▼
STATE_USB_CONNECTED
  │  USER_BUTTON (rising)  → download via VCP
  ▼
STATE_DOWNLOAD  →  back to STATE_USB_CONNECTED
```

### TIM2 ISR — 100 Hz sampling loop (`main.c`)

```
Every tick  (10 ms):
  1. IMU_ReadAccelerometerData()   → raw_accelerometer[6]
  2. IMU_ReadGyroscopeData()       → raw_gyroscope[6]
  3. BLE_SendPacket(ACCEL, ...)
  4. BLE_SendPacket(GYRO,  ...)
  5. Increment timestamp

Every 10th tick  (100 ms / 10 Hz):
  6. AS7341_ReadFullSpectrum()     → spectrum.ch[0..11]
  7. Pack raw_light[22]  (F1–F8, Clear2, NIR2, mains_hz)
  8. LightMetrics_Update()

Every tick  (always):
  9. write_packet(sample, timestamp, accel, gyro, light, NAND_packet)
 10. write_memory()
```

**Design decisions**

| Decision | Rationale |
|---|---|
| Light subsampled at 10 Hz | AS7341 integration ≈ 18 ms; faster polling wastes power |
| `g_mains_hz` updated every 2 s in foreground | `AS7341_DetectMainsHz()` is blocking; keep ISR short |
| Q15 fixed-point in `light_metrics_mcu.c` | No FPU needed; fits Cortex-M33 without `-mfpu` |
| Metric-only BLE streaming (no raw spectral) | 20-byte MTU constraint; raw 12-ch = 24 bytes |
| Little-endian wire format | Matches Flutter `ByteData.getInt16(offset, Endian.little)` |

---

## 4. BLE Communication Protocol

The RN4871 is configured in **Transparent UART** mode.  
Every packet is exactly **20 bytes** to fit inside one BLE notification MTU.

### Frame Layout

```
Byte  0      : MsgType  (uint8)
Bytes 1–2    : sequence number (uint16 LE)
Bytes 3–18   : payload  (16 bytes)
Byte  19     : checksum = XOR of bytes 0–18
```

### MsgType Reference

| Value | Name | Payload description |
|---|---|---|
| `0x01` | `DATA_TYPE_IMU_ACCELERATION` | 6 raw bytes from LSM6DSO16IS accel registers |
| `0x02` | `DATA_TYPE_IMU_GYROSCOPE` | 6 raw bytes from LSM6DSO16IS gyro registers |
| `0x03` | `DATA_TYPE_LIGHT_METRICS` | 5 × int16 LE metrics (see §5) |
| `0xFF` | `MSG_TYPE_HEARTBEAT` | 1-byte payload = 0xBE; sent every 1 s |

### Packet Detail — Accel / Gyro (type `0x01` / `0x02`)

```
[0]     MsgType
[1–2]   sequence (uint16 LE)
[3]     OUT_X_L
[4]     OUT_X_H
[5]     OUT_Y_L
[6]     OUT_Y_H
[7]     OUT_Z_L
[8]     OUT_Z_H
[9–18]  padding 0x00
[19]    XOR checksum
```

To convert raw bytes to physical units (accelerometer, FS = ±2 g):

```
int16_t raw_x = (int16_t)((buf[4] << 8) | buf[3]);
float   ax    = raw_x * 0.061f / 1000.0f;   // g
```

Gyroscope (FS = ±250 dps):

```
int16_t raw_x = (int16_t)((buf[4] << 8) | buf[3]);
float   gx    = raw_x * 8.75f / 1000.0f;    // dps
```

### Packet Detail — Light Metrics (type `0x03`)

```
[0]     0x03
[1–2]   sequence (uint16 LE)
[3–4]   DaylightScore   (int16 LE, 0–10 000, scaled ×100)
[5–6]   CircadianDose   (int16 LE, melanopic lux ×10)
[7–8]   ExposureIndex   (int16 LE, 0–10 000, scaled ×100)
[9–10]  SunLikeIndex    (int16 LE, 0 = artificial, 1 = natural)
[11–12] ColorTemp       (int16 LE, Kelvin)
[13–18] padding 0x00
[19]    XOR checksum
```

---

## 5. Light Metrics

The AS7341 reads 8 narrowband filters (F1 – F8) plus Clear and NIR channels.  
`LightMetrics_Update()` (called at 10 Hz) computes five running metrics.

### AS7341 Channel Map

| Index | Channel | Peak λ (nm) |
|---|---|---|
| 0 | F1 | 415 |
| 1 | F2 | 445 |
| 2 | F3 | 480 |
| 3 | F4 | 515 |
| 4 | Clear (pass 1) | broadband |
| 5 | NIR (pass 1) | ~910 |
| 6 | F5 | 555 |
| 7 | F6 | 590 |
| 8 | F7 | 630 |
| 9 | F8 | 680 |
| 10 | Clear (pass 2) | broadband |
| 11 | NIR (pass 2) | ~910 |

> Clear and NIR from the **second** SMUX pass are used for logging  
> (indices 10 and 11) as they are captured after the F5–F8 group.

### Metric Definitions

| Metric | Definition |
|---|---|
| **DaylightScore** | Weighted ratio of short-λ (F1–F3) to total visible; 0–100 |
| **CircadianDose** | Running integral of melanopic-weighted irradiance (Q15, µW·s/cm²) |
| **ExposureIndex** | Broadband intensity normalised to maximum expected outdoor level |
| **SunLikeIndex** | Outdoor vs. artificial discriminator (see table below) |
| **ColorTemp** | Correlated colour temperature derived from F2/F6 ratio |

### SunLikeIndex Discrimination Table

| NIR / Clear ratio | Classification | SunLikeIndex |
|---|---|---|
| ≥ 0.30 | Natural / sunlight | 1 |
| < 0.30 | Artificial lighting | 0 |

> Sunlight has a strong NIR component; LED and fluorescent sources do not.

---

## 6. IMU & Step Counting

### Sensitivity Constants

| Sensor | Full Scale | LSB sensitivity |
|---|---|---|
| Accelerometer | ±2 g | 0.061 mg/LSB |
| Gyroscope | ±250 dps | 8.75 mdps/LSB |

### Cumulative Step Count

Steps are tracked via a simple zero-crossing counter on the vertical-axis
accelerometer magnitude, low-pass filtered at 5 Hz (implemented in
`imu_driver.c`).  The count is exposed as a `uint32_t` through
`IMU_GetStepCount()` and reset on each new `STATE_ACQUISITION` entry.

---

## 7. NAND Flash Logging

### Page Layout

```
MT29F4G01 page = 4 096 data bytes + 256 spare bytes
Blocks per device : 2 048
Pages per block   : 64
```

### Record Format (`write_packet`)

Each sample written to NAND is **BYTES\_PER\_SAMPLE** bytes wide:

```
[0–1]   sample index    (uint16 LE)
[2]     hh  (timestamp hours)
[3]     mm  (timestamp minutes)
[4]     ss  (timestamp seconds)
[5–6]   sss (timestamp milliseconds, uint16 LE)
[7–12]  raw_accelerometer[6]
[13–18] raw_gyroscope[6]
[19–40] raw_light[22]   (F1–F8, Clear2, NIR2, mains_hz)
```

Bad blocks are detected at startup (`find_bad_blocks`) and stored in
`bad_blocks[]`; they are skipped transparently during writes and reads.

---

## 8. Getting Started

### Firmware (STM32CubeIDE)

```bash
# Clone
git clone https://github.com/YR-trove/MainBoard_IMU_Logger.git
cd MainBoard_IMU_Logger
git checkout ble+light+imu
```

1. Open **STM32CubeIDE** → *File → Open Projects from File System* → select the repo root.
2. Build with **Release** configuration (or Debug for SWO tracing).
3. Flash via **ST-Link** or **DFU** (USB cable, `ioc` BOOT0 = 1).

### Flutter App

The companion app (separate repository) expects:

- **BLE Service UUID**: `49535343-FE7D-4AE5-8FA9-9FAFD205E455`
- **TX Characteristic**: `49535343-1E4D-4BD9-BA61-23C647249616`
- **RX Characteristic**: `49535343-8841-43F4-A8D4-ECBE34729BB3`

Pair the device named **`BLE_SW`** (set by `SN,BLE_SW\r` in `BLE_Initialize()`).

### First Run

| Step | Action | LED |
|---|---|---|
| Power on | RED on during init | 🔴 |
| Init complete | RED off | ⚫ |
| Press button (short) | Erase + start logging | 🟢 |
| Press button again | Stop logging | ⚫ |
| Plug USB cable | USB connected | 🟢 |
| Press button (USB) | Download via VCP @ 115 200 baud | — |

---

## 9. Branch Guide

| Branch | Contents |
|---|---|
| `main` | Stable baseline (IMU logging only) |
| `ble` | BLE transparent UART streaming added |
| `ble+light` | AS7341 spectral sensor integrated |
| `ble+light+imu` | **Current** — full feature set + light metrics MCU |

### What's New in `ble+light+imu`

- **AS7341 full-spectrum driver** — dual-SMUX pass, 12 channels at 10 Hz
- **`light_metrics_mcu.c`** — Q15 fixed-point DaylightScore, CircadianDose,  
  ExposureIndex, SunLikeIndex, ColorTemp running at 10 Hz in ISR
- **Mains flicker classification** — `AS7341_DetectMainsHz()` called every 2 s  
  in foreground; result tagged into every NAND record
- **`raw_light[22]`** packed into every NAND sample alongside IMU data
- **`DATA_TYPE_LIGHT_METRICS` BLE packet** (type `0x03`) with 5 int16 metrics
- **`SN,BLE_SW\r` device name** set during `BLE_Initialize()`
