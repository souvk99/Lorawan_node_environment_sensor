# 🌍 WisBlock LoRaWAN Environmental Monitoring Node & Flash-Backed USB Alert Configurator

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Supported-orange.svg)](https://platformio.org/)
[![Hardware](https://img.shields.io/badge/Hardware-RAK4631%20%7C%20nRF52840-blue.svg)](https://store.rakwireless.com/)
[![LoRaWAN](https://img.shields.io/badge/LoRaWAN-1.0.3%20Class%20A-green.svg)](https://www.thethingsnetwork.org/)
[![GUI](https://img.shields.io/badge/GUI-Python%20%7C%20Tkinter-purple.svg)](https://python.org/)
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)](LICENSE)

An industrial-grade, multi-parameter environmental telemetry node built on the **RAKwireless WisBlock Core (RAK4631)**, featuring real-time indoor air quality, climate, and combustible gas monitoring over **LoRaWAN**.

The project incorporates an **abstracted edge configuration system**: an intuitive dark-themed desktop GUI app communicates over USB serial to read, tune, and persist alert thresholds directly into the microcontroller's **internal Flash memory (LittleFS)**—eliminating the need to re-compile or re-flash C++ firmware whenever operating thresholds change.

---

## 📑 Table of Contents
1. [Device Philosophy & Objectives](#-device-philosophy--objectives)
2. [Hardware Architecture](#-hardware-architecture)
3. [System Architecture & Data Flow](#-system-architecture--data-flow)
4. [Desktop Configurator App](#-desktop-configurator-app)
5. [Firmware Features & Sensor Pipeline](#-firmware-features--sensor-pipeline)
6. [LoRaWAN Uplink Payload Specification](#-lorawan-uplink-payload-specification)
7. [Repository Structure](#-repository-structure)
8. [Quick Start & Setup](#-quick-start--setup)
9. [Payload Decoder (TTN / ChirpStack)](#-payload-decoder-ttn--chirpstack)

---

## 🎯 Device Philosophy & Objectives

Modern IoT sensor deployments often fail in the field due to rigid, hardcoded firmware: changing a warning threshold or transmission cadence typically requires an embedded engineer with compiler toolchains, source repositories, and flashing cables. 

This project solves that operational bottleneck through three core design principles:

### 1. Complete Hardware Abstraction
* **Firmware as an Engine:** The firmware acts purely as an acquisition and communication engine. It does not enforce hardcoded, immutable alert logic.
* **Non-Programmer Friendly:** Field operators, technicians, and QA testers can connect the node over USB, open a graphical app, and tune alarm limits in seconds with zero exposure to C++ code.

### 2. Edge-Level Non-Volatile Persistence
* Dynamic settings are stored locally in the Nordic nRF52840 internal Flash file system (**`InternalFS` / LittleFS**).
* Configured thresholds survive battery swaps, power outages, and field resets without requiring cloud downlink synchronization.

### 3. Dual-Cadence Adaptive Alerting
* **Normal Mode (60s):** Periodic telemetry for battery preservation and LoRaWAN regional duty-cycle compliance.
* **Alarm Mode (15s):** If any threshold is breached, the node immediately transmits an uplink packet (bypassing the timer) and accelerates transmission to 15-second intervals until the hazard clears.

---

## 🛠️ Hardware Architecture

The device is built around the modular **RAKwireless WisBlock** ecosystem:

| Module | Component | Description & Interface |
|:---|:---|:---|
| **Core Module** | **RAK4631** | Nordic nRF52840 (ARM Cortex-M4F @ 64MHz) + Semtech SX1262 LoRa transceiver |
| **Baseboard** | **RAK5005-O / RAK19007** | Modular WisBlock base with battery charging, power switching, and $I^2C$/SPI buses |
| **Climate Sensor** | **RAK1901 (SHTC3)** | High-accuracy Ambient Temperature ($-40\text{ to }85^\circ\text{C}$) & Relative Humidity ($0\text{ to }100\%$) |
| **$\text{CO}_2$ Sensor** | **RAK12037 (SCD30)** | True dual-channel NDIR Photoacoustic $\text{CO}_2$ sensor ($400\text{ to }10,000\text{ ppm}$) |
| **VOC Gas Sensor** | **RAK12047 (SGP40)** | Sensirion MOx Multi-pixel Gas Sensor running the **Sensirion Gas Index Algorithm** ($1\text{ to }500\text{ idx}$) |
| **Gas / Smoke** | **RAK12004 (MQ2)** | Flammable gas & smoke sensor paired with an onboard $I^2C$ ADC (MCP3221 / ADC081C027) |

---

## 🔄 System Architecture & Data Flow

```text
       ┌────────────────────────────────────────────────────────┐
       │               WisBlock RAK4631 Node                    │
       │                                                        │
       │  [SHTC3]   [SCD30]   [SGP40 (1Hz)]   [MQ2 (I2C ADC)]   │
       │     │         │            │                │          │
       │     └─────────┴──────┬─────┴────────────────┘          │
       │                      ▼                                 │
       │          Calibrations & Compensation                   │
       │                      ▼                                 │
       │          Check Alarms vs. Active Config                │
       │                      ▼                                 │
       │        [InternalFS Flash: /node_config.bin]            │
       │                      ▲                                 │
       └──────────────────────┼─────────────────┬───────────────┘
                              │ USB Serial      │ LoRaWAN 865-915 MHz
                              │ (115200 baud)   │ (12-Byte Binary Frame)
                              ▼                 ▼
             ┌─────────────────────────┐   ┌─────────────────────────┐
             │  Desktop Configurator   │   │  LoRaWAN Gateway        │
             │   (Python Tkinter GUI)  │   │  (The Things Network /  │
             │                         │   │   ChirpStack v4)        │
             │  • Real-time Validation │   └─────────────────────────┘
             │  • Single-Item Updates  │
             │  • Read/Flash to Memory │
             └─────────────────────────┘
```

---

## 💻 Desktop Configurator App

The project includes a standalone desktop tool ([`node_configurator_gui.py`](file:///c:/Users/souvk/Downloads/Lorawan_node_environment_sensor/node_configurator_gui.py)) with a single-click Windows launcher ([`run_app.bat`](file:///c:/Users/souvk/Downloads/Lorawan_node_environment_sensor/run_app.bat)):

### Key Capabilities
* **Auto-Discovery:** Automatically scans serial COM ports and highlights the connected WisBlock device.
* **Live Hardware Synchronization:** Click **"Read Values from Node"** to fetch and display the exact thresholds currently stored in Flash.
* **Partial / Single-Item Updates:** Only fill in the parameters you wish to alter; all blank inputs are skipped, preserving their existing values on the hardware.
* **Side-by-Side Range Guides:** The physical sensor limits are displayed beside each entry field.
* **Real-time Error Guardrails:** Flags non-numeric input, out-of-bounds parameters, and relational conflicts (e.g. $T_{\min} \ge T_{\max}$) with inline alerts before any write occurs.
* **Instantaneous Flash Write:** Updates the node in **$< 0.5\text{ seconds}$** over USB without requiring firmware recompilation.

---

## 🔬 Firmware Features & Sensor Pipeline

The production firmware ([`platformio_code/src/main.cpp`](file:///c:/Users/souvk/Downloads/Lorawan_node_environment_sensor/platformio_code/src/main.cpp)) incorporates several stability features:

### 1. Mandatory 1 Hz SGP40 VOC Cadence
Sensirion’s VOC Gas Index Algorithm requires sampling at an exact $1\text{ Hz}$ frequency. The firmware decouples VOC processing from the LoRaWAN transmission timer, feeding compensated temperature and relative humidity readings into `VOCGasIndexAlgorithm.process()` every $1000\text{ ms}$.

### 2. Built-in Calibration Constants
* **PCB Self-Heating:** Temperature reading is offset by `-1.00 °C` to neutralize thermal dissipation from the nRF52840 MCU and SX1262 LoRa power amplifier.
* **Humidity Compensation:** Relative humidity is corrected by `+2.00 %`.
* **Clean-Air Baseline:** MQ2 calculation utilizes clean-air baseline resistance $R_0 = 9.83\text{ k}\Omega$.

### 3. Non-Blocking USB Command Protocol
The node listens for serial commands on `Serial` without stalling sensor acquisition:
* `GET_CONFIG`: Responds with active parameters formatted as JSON (`CONFIG_DATA:{...}`).
* `SET_CONFIG:key1=val1;key2=val2`: Updates specified parameters and commits them to Flash (`CONFIG_RESULT:OK_SAVED`).
* `RESET_CONFIG`: Reverts Flash memory to factory defaults (`CONFIG_RESULT:OK_RESET`).

---

## 📦 LoRaWAN Uplink Payload Specification

The uplink frame is strictly packed into **12 bytes** using `#pragma pack(1)`:

| Byte Offset | Size | Field Name | Data Type | Encoding / Scale |
|:---:|:---:|:---|:---|:---|
| `0` | 2 | `alarm_status` | `uint16_t` | `0` = Normal, `1` = Alarm Active |
| `2` | 2 | `temp` | `int16_t` | Signed, $\text{Value} = \text{Temp}(^\circ\text{C}) \times 100$ |
| `4` | 2 | `hum` | `uint16_t` | $\text{Value} = \text{RH}(\%) \times 100$ |
| `6` | 2 | `co2` | `uint16_t` | Direct $\text{CO}_2$ concentration in $\text{ppm}$ |
| `8` | 2 | `voc` | `uint16_t` | $\text{Value} = \text{VOC Index} \times 10$ |
| `10` | 2 | `gas` | `uint16_t` | $\text{Value} = \text{MQ2 Gas}(\text{ppm}) \times 10$ |

---

## 📁 Repository Structure

```text
Lorawan_node_environment_sensor/
│
├── README.md                      # Comprehensive project documentation
├── node_configurator_gui.py       # Desktop GUI Alert Configurator (Python/Tkinter)
├── run_app.bat                    # 1-click Windows launcher for the GUI
├── platformio.ini                 # Root workspace PlatformIO forwarding config
│
├── .vscode/                       # VS Code workspace tasks & serial monitor bindings
│   ├── tasks.json                 # Dedicated COM8 monitor task
│   ├── settings.json              # PlatformIO extension settings
│   └── launch.json                # J-Link / Cortex debug configuration
│
├── platformio_code/               # Production PlatformIO project
│   ├── platformio.ini             # Target board environment & library definitions
│   ├── README.md                  # Firmware-specific build notes
│   ├── include/                   # Header search directory
│   ├── lib/                       # Private project libraries
│   ├── rakwireless/               # Official RAKwireless WisBlock board & variant definitions
│   │   ├── boards/rak4630.json    # Pin & architecture definitions for RAK4630/4631
│   │   └── variants/rak4630/      # Hardware variant mapping (variant.cpp, variant.h)
│   └── src/
│       └── main.cpp               # Production firmware (Flash LittleFS + LoRaWAN)
│
└── Arduino_files/                 # Firmware evolution & heritage sketches
    └── environment_monitoring/
        ├── v01/ to v06_prod/      # Historical development versions
        ├── v07_working/           # Stable base sketch
        ├── v07_working_no_bugs/   # Gold standard reference sketch
        └── main/                  # Independent Arduino IDE distribution
```

---

## 🚀 Quick Start & Setup

### Prerequisites
* [VS Code](https://code.visualstudio.com/) with the **PlatformIO IDE** extension.
* Python 3.10+ (includes `tkinter`).
* Install PySerial:
  ```powershell
  pip install pyserial
  ```

### 1. Build and Flash Firmware
1. Connect the RAK4631 via USB.
2. In VS Code, open the repository root folder.
3. Open the integrated terminal and compile/flash:
   ```powershell
   pio run -t upload
   ```
   *(Or click the checkmark/arrow icons in the bottom PlatformIO toolbar).*

### 2. Configure Node Alerts via Desktop App
1. Double-click **[`run_app.bat`](file:///c:/Users/souvk/Downloads/Lorawan_node_environment_sensor/run_app.bat)** (or run `python node_configurator_gui.py`).
2. Select your device port (e.g. `COM8`).
3. Click **"Read Values from Node"** to view current parameters.
4. Modify any threshold and click **"APPLY & SAVE TO DEVICE"**.

---

## 📡 Payload Decoder (TTN / ChirpStack)

Copy and paste this JavaScript codec into your **The Things Network (TTN v3)** or **ChirpStack v4** application:

```javascript
function decodeUplink(input) {
    var bytes = input.bytes;
    if (bytes.length < 12) {
        return { errors: ["Invalid payload length: expected 12 bytes"] };
    }

    // Decode little-endian fields
    var alarm_raw = bytes[0] | (bytes[1] << 8);
    var temp_raw  = (bytes[2] | (bytes[3] << 8));
    if (temp_raw > 32767) temp_raw -= 65536; // Sign extension for int16

    var hum_raw   = bytes[4] | (bytes[5] << 8);
    var co2_raw   = bytes[6] | (bytes[7] << 8);
    var voc_raw   = bytes[8] | (bytes[9] << 8);
    var gas_raw   = bytes[10] | (bytes[11] << 8);

    return {
        data: {
            alarm_status: alarm_raw === 1 ? "ALARM" : "NORMAL",
            alarm_active: alarm_raw === 1,
            temperature: parseFloat((temp_raw / 100.0).toFixed(2)),
            humidity: parseFloat((hum_raw / 100.0).toFixed(2)),
            co2: co2_raw,
            voc_index: parseFloat((voc_raw / 10.0).toFixed(1)),
            gas_ppm: parseFloat((gas_raw / 10.0).toFixed(1))
        }
    };
}
```

---

## 📜 License
This project is open-source under the [MIT License](LICENSE).
Built with the [RAKwireless WisBlock](https://docs.rakwireless.com/Product-Categories/WisBlock/) modular hardware system.
