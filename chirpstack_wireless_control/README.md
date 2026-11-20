# ChirpStack Wireless Over-The-Air (OTA) Control & Closed-Loop ACK

This directory contains the complete wireless configuration and live monitoring system for the **WisBlock RAK4631 LoRaWAN Environment Node**. It enables operators to remotely tune sensor alarm thresholds, sampling intervals, and alert boundaries over-the-air via **ChirpStack v4**, backed by an automated closed-loop acknowledgment protocol.

---

## 🌟 System Architecture & Philosophy

```
+---------------------------------------------------------------------------------+
|                       DESKTOP CONFIGURATOR GUI                                 |
|  - Real-time parameter inputs & validation                                      |
|  - Live sensor telemetry display (Temp, Hum, CO2, VOC, Gas)                     |
|  - Luminous Closed-Loop ACK Banner: confirms hardware Flash storage live        |
+-------------------+-----------------------------------------^-------------------+
                    |                                         |
         1. Enqueue Downlink (FPort 10)            4. Live ACK Event (FPort 11)
          via REST API / gRPC                       via MQTT application/+/...
                    |                                         |
                    v                                         |
+-------------------------------------------------------------+-------------------+
|                        CHIRPSTACK NETWORK SERVER v4                             |
|  - Host: 172.16.50.30                                                           |
|  - Web UI: http://172.16.50.30:8080                                             |
|  - REST API: http://172.16.50.30:8090                                           |
|  - MQTT Broker: 172.16.50.30:1883                                               |
+-------------------+-----------------------------------------^-------------------+
                    |                                         |
         Downlink during RX1/RX2                   Uplink Transmission
                    |                                         |
                    v                                         |
+-------------------------------------------------------------+-------------------+
|                      LORAWAN GATEWAY (0016c001f17a7331)                         |
|  - IN865 Band, Multi-channel packet forwarder                                   |
+-------------------+-----------------------------------------^-------------------+
                    |                                         |
         RF Downlink (865 MHz)                     RF Uplink (865 MHz)
                    |                                         |
                    v                                         |
+-------------------------------------------------------------+-------------------+
|                  WISBLOCK RAK4631 ENVIRONMENT SENSOR NODE                       |
|  - DevEUI: ac1f09fffe22024e                                                     |
|  - Receives Downlink on FPort 10                                                |
|  - Decodes parameters, updates in-memory thresholds                             |
|  - Commits new settings directly to Internal LittleFS Flash (/node_config.bin)  |
|  - Transmits immediate ACK Uplink on FPort 11 echoing confirmed Flash values    |
+---------------------------------------------------------------------------------+
```

---

## 📡 Wireless Communication Protocol

### 1. Downlink Configuration (FPort 10)
Downlinks are queued in ChirpStack and transmitted to the node during its Class A reception window (RX1 / RX2).

#### Command `0x01`: Partial / Single Parameter Update (`CMD_SET_PARAM`)
Updates one or more specific parameters without modifying untouched parameters.
- **Wire Format**: `[0x01, PARAM_ID_1, VAL_H, VAL_L, PARAM_ID_2, VAL_H, VAL_L, ...]`
- **Parameter Mapping**:
  | ID | Parameter | Type | Scale | Description | Example (Hex) |
  |---|---|---|---|---|---|
  | `0x01` | `temp_min` | `int16` | x100 | Min temperature threshold | `0x01, 0xFC, 0x18` (-10.0°C) |
  | `0x02` | `temp_max` | `int16` | x100 | Max temperature threshold | `0x02, 0x0F, 0x0A` (38.5°C) |
  | `0x03` | `hum_min` | `uint16` | x100 | Min humidity threshold | `0x03, 0x07, 0xD0` (20.0%) |
  | `0x04` | `hum_max` | `uint16` | x100 | Max humidity threshold | `0x04, 0x1F, 0x40` (80.0%) |
  | `0x05` | `gas_warning_ppm` | `uint16` | 1 | Gas warning level (MQ2) | `0x05, 0x01, 0xF4` (500 ppm) |
  | `0x06` | `gas_alert_ppm` | `uint16` | 1 | Gas alert level (MQ2) | `0x06, 0x03, 0xE8` (1000 ppm) |
  | `0x07` | `co2_normal_max` | `uint16` | 1 | CO2 elevated warning (SCD30) | `0x07, 0x01, 0xF4` (500 ppm) |
  | `0x08` | `co2_warning_max` | `uint16` | 1 | CO2 critical alert (SCD30) | `0x08, 0x03, 0xE8` (1000 ppm) |
  | `0x09` | `voc_normal_max` | `uint16` | 1 | VOC warning index (SGP40) | `0x09, 0x00, 0x64` (100 idx) |
  | `0x0A` | `voc_warning_max` | `uint16` | 1 | VOC alert index (SGP40) | `0x0A, 0x00, 0xC8` (200 idx) |
  | `0x0B` | `send_interval_s` | `uint16` | 1 | Normal cadence interval | `0x0B, 0x00, 0x3C` (60 s) |
  | `0x0C` | `alert_interval_s` | `uint16` | 1 | Alarm burst interval | `0x0C, 0x00, 0x0F` (15 s) |

#### Command `0x02`: Set All Parameters (`CMD_SET_ALL`)
Sets all 12 parameters in a single packed 25-byte payload:
`[0x02, temp_min(2B), temp_max(2B), hum_min(2B), hum_max(2B), gas_warn(2B), gas_alert(2B), co2_norm(2B), co2_warn(2B), voc_norm(2B), voc_warn(2B), send_s(2B), alert_s(2B)]`

#### Command `0x03`: Factory Reset (`CMD_RESET`)
Payload: `[0x03]` (1 byte). Restores default thresholds and commits to Flash.

#### Command `0x04`: Query Configuration (`CMD_QUERY`)
Payload: `[0x04]` (1 byte). Triggers the node to immediately transmit an ACK packet with its active Flash parameters.

---

### 2. Uplink Acknowledgment & Verification (FPort 11)
When the node receives a configuration downlink, applies the parameters, and saves them to Flash, it immediately transmits an **ACK Uplink on FPort 11**.

- **Packet Size**: 27 bytes
- **Packet Structure**:
  - `Byte 0`: `Status Code` (`0x00` = OK / Success, `0x01` = Bad Command, `0x02` = Flash Save Failed)
  - `Byte 1`: `Command Reference` (Echoes the received command, e.g. `0x01`, `0x02`, `0x03`, `0x04`)
  - `Byte 2`: `Updated Parameter Count`
  - `Bytes 3..26`: Complete active parameter snapshot read directly from LittleFS Flash:
    - `temp_min` (int16, x100)
    - `temp_max` (int16, x100)
    - `hum_min` (uint16, x100)
    - `hum_max` (uint16, x100)
    - `gas_warning_ppm` (uint16)
    - `gas_alert_ppm` (uint16)
    - `co2_normal_max` (uint16)
    - `co2_warning_max` (uint16)
    - `voc_normal_max` (uint16)
    - `voc_warning_max` (uint16)
    - `send_interval_s` (uint16)
    - `alert_interval_s` (uint16)

---

## 💻 Graphical User Interface (GUI)

The GUI provides an operator interface with full real-time feedback:

1. **Connection Bar**:
   - ChirpStack Network Server URL, Ports, and DevEUI
   - Single-click connection toggle with automatic JWT generation and MQTT subscription
2. **Live Sensor Telemetry**:
   - Shows live incoming measurements from FPort 2:
     - `Temperature (°C)`
     - `Humidity (%)`
     - `CO2 (ppm)`
     - `VOC Index`
     - `Gas (ppm)`
     - `Alarm Status (🟢 NORMAL / ⚠️ ALARM ACTIVE)`
3. **Closed-Loop ACK Confirmation Banner**:
   - Displays real-time state:
     - **Idle / Ready**: Neutral slate.
     - **Downlink Queued**: Pulsing amber banner: `⏳ Downlink queued in ChirpStack. Waiting for node's next transmission window...`
     - **Confirmed**: **Luminous Green Banner**: `✅ CONFIRMED: Node successfully applied configuration & saved to Flash! [RTT: 28.4s | Cmd: 0x01]`
     - **Auto-Sync**: The GUI automatically updates all parameter input fields with the node's confirmed Flash values.
4. **Partial Parameter Editing**:
   - Leave unchanged fields blank. Only filled fields are sent in the downlink.

---

## 🚀 How to Run

### Option 1: Direct Double-Click
Double-click `run_wireless_app.bat` in this folder.

### Option 2: Command Line
```powershell
cd chirpstack_wireless_control
python wireless_configurator_gui.py
```

### Option 3: From USB Configurator
From the main USB Configurator (`node_configurator_gui.py`), click the purple **"🌐 Wireless Mode (ChirpStack)"** button in the header bar.

---

## 🧪 Automated Verification Script

A standalone automated end-to-end verification script is provided in `test_downlink_ack.py`:

```powershell
python test_downlink_ack.py
```

### Verified Test Run:
```
============================================================
STARTING LIVE CHIRPSTACK WIRELESS CONFIGURATION TEST
============================================================
[Step 1] Authenticating with ChirpStack...
[ChirpStackClient] Login successful! JWT acquired (233 chars).
SUCCESS: ChirpStack Authenticated.

[Step 2] Connecting to MQTT broker...
[ChirpStackClient] MQTT Connected! Subscribed to topic: application/+/device/ac1f09fffe22024e/event/up
SUCCESS: MQTT connected and listening.

[Step 3] Flushing existing downlink queue...
[Step 4] Enqueueing Downlink: {'temp_max': 38.5} -> Raw Hex: 01020f0a
[ChirpStackClient] Downlink queued successfully! Item ID: 24aaf748-e016-4b64-99dd-1264d786e3b6 (FPort 10, 4 bytes)
SUCCESS: Downlink queued (Queue ID: 24aaf748-e016-4b64-99dd-1264d786e3b6)

[Step 5] Waiting for Node's uplink transmission...
Class A node will receive the downlink during RX1/RX2 window and reply with ACK on FPort 11.
Waiting up to 75 seconds...
[LIVE TELEMETRY FPort 2] Temp: 33.63C | Hum: 42.28% | CO2: 485 ppm | Alarm: NORMAL
[ChirpStackClient] ACK RECEIVED ON FPORT 11: OK (Cmd 0x01)

============================================================
>>> [SUCCESS] RECEIVED LIVE ACK UPLINK ON FPORT 11! <<<
============================================================
Status Code : 0 (OK)
Command Ref : 0x01
Params Count: 1
Confirmed Flash Configuration from Node:
  temp_min            : -35.0
  temp_max            : 38.5
  hum_min             : 20.0
  hum_max             : 80.0
  gas_warning_ppm     : 500
  gas_alert_ppm       : 1000
  co2_normal_max      : 500
  co2_warning_max     : 1000
  voc_normal_max      : 100
  voc_warning_max     : 200
  send_interval_s     : 60
  alert_interval_s    : 15
============================================================
```
