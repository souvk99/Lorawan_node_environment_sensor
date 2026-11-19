# WisBlock RAK4631 Environment Monitoring Node (PlatformIO)

This project contains the PlatformIO port of the environment monitoring and LoRaWAN transmission node based on the **v07_working_no_bugs** firmware.

---

## 🛠️ Hardware Overview

- **Core Module:** RAK4631 (Nordic nRF52840 + SX1262 LoRa transceiver)
- **Base Board:** RAK5005-O / RAK19007 WisBlock Base
- **Sensors:**
  - **RAK1901 (SHTC3):** Temperature & Relative Humidity
  - **RAK12037 (SCD30):** $CO_2$ NDIR Sensor
  - **RAK12047 (SGP40):** VOC Gas Index (Sensirion Gas Index Algorithm)
  - **RAK12004 (MQ2):** Flammable gas / smoke via $I^2C$ ADC

---

## 📁 Project Structure

```text
platformio_code/
├── platformio.ini         # PlatformIO project configuration & library dependencies
├── rakwireless/           # Official RAKwireless WisBlock board & variant definitions
├── src/
│   └── main.cpp           # Main application code (derived from v07_working_no_bugs)
├── include/               # Header files
└── lib/                   # Project-specific private libraries
```

---

## 🚀 Quick Start (VS Code / PlatformIO)

1. Open **VS Code** with the **PlatformIO IDE** extension installed.
2. Select **File > Open Folder...** and choose `platformio_code`.
3. PlatformIO will automatically read `platformio.ini` and download required libraries:
   - `SX126x-Arduino` (by beegee-tokyo)
   - `SparkFun SHTC3 Humidity and Temperature Sensor Library`
   - `SparkFun SCD30 Arduino Library`
   - `Sensirion I2C SGP40`
   - `Sensirion Gas Index Algorithm`
4. Connect your RAK4631 via USB.
5. Click **Build** (checkmark icon) and **Upload** (right arrow icon) in the bottom PlatformIO toolbar.
6. Open the **Serial Monitor** at `115200` baud.

---

## ⚙️ Configuration

### 1. LoRaWAN Region & Keys
In `src/main.cpp`:
- Set `g_CurrentRegion` (default: `LORAMAC_REGION_IN865`).
- Update your OTAA credentials:
  - `nodeDeviceEUI[8]`
  - `nodeAppEUI[8]`
  - `nodeAppKey[16]`

### 2. Sensor Offsets
- `TEMP_OFFSET_C`: Self-heating offset (default `-1.0f`).
- `HUM_OFFSET_PCT`: Humidity correction (default `+2.0f`).
- `CO2_SCALE`: SCD30 calibration scalar (default `1.00f`).
- `MQ2_RO_CLEAN_AIR`: Clean air resistance (default `9.83f`).

---

## 📦 12-Byte Uplink Payload Format

| Offset | Size (Bytes) | Field | Type | Scale / Encoding |
|:---:|:---:|:---|:---|:---|
| 0 | 2 | `alarm_status` | uint16 | 0 = Normal, 1 = Alarm |
| 2 | 2 | `temp` | int16 | $\times 100$ (signed) |
| 4 | 2 | `hum` | uint16 | $\times 100$ |
| 6 | 2 | `co2` | uint16 | ppm |
| 8 | 2 | `voc` | uint16 | VOC Index $\times 10$ |
| 10 | 2 | `gas` | uint16 | MQ2 ppm $\times 10$ |
