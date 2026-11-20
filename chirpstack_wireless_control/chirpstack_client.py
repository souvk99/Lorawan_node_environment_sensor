"""
ChirpStack v4 Integration Client for RAK4631 LoRaWAN Environment Node.
Supports:
  - Authentication via gRPC-web / REST API
  - Enqueueing downlink configuration packets (FPort 10)
  - Real-time MQTT monitoring for Telemetry (FPort 2) & Closed-Loop ACK Confirmation (FPort 11)
"""

import base64
import json
import struct
import threading
import time
from typing import Callable, Dict, Optional, Any
import paho.mqtt.client as mqtt
import requests

# Parameter IDs matching firmware
PARAM_MAP = {
    "temp_min":         (0x01, "h", 100.0),   # signed int16, / 100
    "temp_max":         (0x02, "h", 100.0),   # signed int16, / 100
    "hum_min":          (0x03, "H", 100.0),   # uint16, / 100
    "hum_max":          (0x04, "H", 100.0),   # uint16, / 100
    "gas_warning_ppm":  (0x05, "H", 1.0),     # uint16, ppm
    "gas_alert_ppm":    (0x06, "H", 1.0),     # uint16, ppm
    "co2_normal_max":   (0x07, "H", 1.0),     # uint16, ppm
    "co2_warning_max":  (0x08, "H", 1.0),     # uint16, ppm
    "voc_normal_max":   (0x09, "H", 1.0),     # uint16, index
    "voc_warning_max":  (0x0A, "H", 1.0),     # uint16, index
    "send_interval_s":  (0x0B, "H", 1.0),     # uint16, sec
    "alert_interval_s": (0x0C, "H", 1.0),     # uint16, sec
}

PARAM_ORDER = [
    "temp_min", "temp_max", "hum_min", "hum_max",
    "gas_warning_ppm", "gas_alert_ppm",
    "co2_normal_max", "co2_warning_max",
    "voc_normal_max", "voc_warning_max",
    "send_interval_s", "alert_interval_s"
]

CMD_SET_PARAM = 0x01
CMD_SET_ALL   = 0x02
CMD_RESET     = 0x03
CMD_QUERY     = 0x04

ACK_STATUS_OK       = 0x00
ACK_STATUS_ERR_CMD  = 0x01
ACK_STATUS_ERR_SAVE = 0x02

FPORT_DOWNLINK = 10
FPORT_ACK      = 11
FPORT_DATA     = 2


class ChirpStackClient:
    def __init__(self,
                 host: str = "172.16.50.30",
                 web_port: int = 8080,
                 api_port: int = 8090,
                 mqtt_port: int = 1883,
                 username: str = "",
                 password: str = "",
                 dev_eui: str = ""):
        self.host = host
        self.web_port = web_port
        self.api_port = api_port
        self.mqtt_port = mqtt_port
        self.username = username
        self.password = password
        self.dev_eui = dev_eui.lower().replace("-", "").replace(":", "")

        self.jwt_token: Optional[str] = None
        self.mqtt_client: Optional[mqtt.Client] = None
        self.is_connected_mqtt = False

        # Callbacks
        self.on_ack: Optional[Callable[[Dict[str, Any]], None]] = None
        self.on_data: Optional[Callable[[Dict[str, Any]], None]] = None
        self.on_log: Optional[Callable[[str], None]] = None

    def log(self, msg: str):
        if self.on_log:
            self.on_log(msg)
        else:
            print(f"[ChirpStackClient] {msg}")

    # ============================================================
    # AUTHENTICATION
    # ============================================================

    def login(self) -> bool:
        """Authenticate with ChirpStack v4 and obtain JWT token."""
        url = f"http://{self.host}:{self.web_port}/api.InternalService/Login"
        headers = {
            "Content-Type": "application/grpc-web+proto",
            "x-grpc-web": "1"
        }
        # Protobuf encoding: {email: 1, password: 2}
        proto = (b"\x0a" + bytes([len(self.username)]) + self.username.encode("utf-8") +
                 b"\x12" + bytes([len(self.password)]) + self.password.encode("utf-8"))
        frame = b"\x00" + len(proto).to_bytes(4, "big") + proto

        try:
            r = requests.post(url, headers=headers, data=frame, timeout=5)
            if r.status_code == 200:
                # Parse JWT from response
                # Format: 5 byte frame header + 0x0A + length + JWT string
                content = r.content
                # Find start of JWT ("eyJ")
                idx = content.find(b"eyJ")
                if idx != -1:
                    end_idx = content.find(b"\x80", idx)
                    if end_idx == -1:
                        end_idx = len(content)
                    self.jwt_token = content[idx:end_idx].decode("utf-8", errors="ignore").strip()
                    self.log(f"Login successful! JWT acquired ({len(self.jwt_token)} chars).")
                    return True
                else:
                    self.log(f"Login failed: JWT marker not found in response: {content[:100]}")
            else:
                self.log(f"Login HTTP failed: code {r.status_code}")
        except Exception as e:
            self.log(f"Login exception: {e}")
        return False

    def _get_headers(self) -> dict:
        if not self.jwt_token:
            self.login()
        return {
            "Authorization": f"Bearer {self.jwt_token}",
            "Content-Type": "application/json"
        }

    # ============================================================
    # DOWNLINK QUEUE (REST API)
    # ============================================================

    def enqueue_downlink(self, data_bytes: bytes, fport: int = FPORT_DOWNLINK, confirmed: bool = False) -> Optional[str]:
        """Enqueue downlink item via ChirpStack REST API."""
        url = f"http://{self.host}:{self.api_port}/api/devices/{self.dev_eui}/queue"
        b64_data = base64.b64encode(data_bytes).decode("ascii")
        payload = {
            "queueItem": {
                "confirmed": confirmed,
                "data": b64_data,
                "fPort": fport
            }
        }

        try:
            r = requests.post(url, headers=self._get_headers(), json=payload, timeout=5)
            if r.status_code == 401 or r.status_code == 403:
                # Token expired, re-login
                self.login()
                r = requests.post(url, headers=self._get_headers(), json=payload, timeout=5)

            if r.status_code == 200:
                res = r.json()
                queue_id = res.get("id")
                self.log(f"Downlink queued successfully! Item ID: {queue_id} (FPort {fport}, {len(data_bytes)} bytes)")
                return queue_id
            else:
                self.log(f"Failed to enqueue downlink (HTTP {r.status_code}): {r.text}")
        except Exception as e:
            self.log(f"Enqueue exception: {e}")
        return None

    def get_queue(self) -> list:
        """Fetch current queue items."""
        url = f"http://{self.host}:{self.api_port}/api/devices/{self.dev_eui}/queue"
        try:
            r = requests.get(url, headers=self._get_headers(), timeout=5)
            if r.status_code == 200:
                return r.json().get("result", [])
        except Exception as e:
            self.log(f"Get queue exception: {e}")
        return []

    def flush_queue(self) -> bool:
        """Clear downlink queue."""
        url = f"http://{self.host}:{self.api_port}/api/devices/{self.dev_eui}/queue"
        try:
            r = requests.delete(url, headers=self._get_headers(), timeout=5)
            return r.status_code == 200
        except Exception as e:
            self.log(f"Flush queue exception: {e}")
        return False

    # ============================================================
    # PACKET ENCODERS & DECODERS
    # ============================================================

    @staticmethod
    def encode_set_params(params: Dict[str, Any]) -> bytes:
        """
        Encode partial or single parameter updates.
        Packet format: [0x01, param_id, val_h, val_l, ...]
        """
        buf = bytearray([CMD_SET_PARAM])
        for k, v in params.items():
            if k in PARAM_MAP and v is not None and str(v).strip() != "":
                pid, fmt, scale = PARAM_MAP[k]
                raw = int(round(float(v) * scale))
                packed = struct.pack(f">{fmt}", raw)
                buf.append(pid)
                buf.extend(packed)
        return bytes(buf)

    @staticmethod
    def encode_set_all(params: Dict[str, Any]) -> bytes:
        """
        Encode all 12 parameters.
        Packet format: [0x02, 24 bytes of 12 fields]
        """
        buf = bytearray([CMD_SET_ALL])
        for k in PARAM_ORDER:
            pid, fmt, scale = PARAM_MAP[k]
            val = params.get(k, 0)
            raw = int(round(float(val) * scale))
            packed = struct.pack(f">{fmt}", raw)
            buf.extend(packed)
        return bytes(buf)

    @staticmethod
    def encode_reset() -> bytes:
        return bytes([CMD_RESET])

    @staticmethod
    def encode_query() -> bytes:
        return bytes([CMD_QUERY])

    @staticmethod
    def decode_ack(payload: bytes) -> Optional[Dict[str, Any]]:
        """
        Decode 27-byte ACK payload from FPort 11:
          byte 0: status (0=OK, 1=CMD_ERR, 2=SAVE_ERR)
          byte 1: cmd_ref
          byte 2: count
          bytes 3..26: 12 fields (int16/uint16 big-endian)
        """
        if len(payload) < 27:
            return None

        status = payload[0]
        cmd_ref = payload[1]
        count = payload[2]

        t_min, t_max, h_min, h_max, g_warn, g_alert, c_norm, c_warn, v_norm, v_warn, s_int, a_int = \
            struct.unpack(">h h H H H H H H H H H H", payload[3:27])

        status_str = "OK" if status == ACK_STATUS_OK else ("CMD_ERR" if status == ACK_STATUS_ERR_CMD else "SAVE_ERR")

        return {
            "status_code": status,
            "status_str": status_str,
            "cmd_ref": cmd_ref,
            "count": count,
            "config": {
                "temp_min": t_min / 100.0,
                "temp_max": t_max / 100.0,
                "hum_min": h_min / 100.0,
                "hum_max": h_max / 100.0,
                "gas_warning_ppm": g_warn,
                "gas_alert_ppm": g_alert,
                "co2_normal_max": c_norm,
                "co2_warning_max": c_warn,
                "voc_normal_max": v_norm,
                "voc_warning_max": v_warn,
                "send_interval_s": s_int,
                "alert_interval_s": a_int
            }
        }

    @staticmethod
    def decode_telemetry(payload: bytes) -> Optional[Dict[str, Any]]:
        """
        Decode 12-byte telemetry payload from FPort 2:
          uint16 alarm_status (0/1)
          int16  temp (x100)
          uint16 hum (x100)
          uint16 co2
          uint16 voc (x10)
          uint16 gas (x10)
        """
        if len(payload) < 12:
            return None
        alarm, temp_raw, hum_raw, co2, voc_raw, gas_raw = struct.unpack("<H h H H H H", payload[:12])
        return {
            "alarm_status": alarm,
            "alarm_str": "ALARM ACTIVE" if alarm == 1 else "NORMAL",
            "temperature_c": temp_raw / 100.0,
            "humidity_pct": hum_raw / 100.0,
            "co2_ppm": co2,
            "voc_index": voc_raw / 10.0,
            "gas_ppm": gas_raw / 10.0
        }

    # ============================================================
    # MQTT LISTENER
    # ============================================================

    def start_mqtt(self):
        """Connect to ChirpStack MQTT broker and subscribe to device events."""
        if self.mqtt_client is not None:
            return

        def on_connect(client, userdata, flags, rc, properties=None):
            self.is_connected_mqtt = True
            topic = f"application/+/device/{self.dev_eui}/event/up"
            client.subscribe(topic)
            self.log(f"MQTT Connected! Subscribed to topic: {topic}")

        def on_message(client, userdata, msg):
            try:
                data = json.loads(msg.payload.decode("utf-8"))
                fport = data.get("fPort")
                b64_data = data.get("data", "")
                raw_bytes = base64.b64decode(b64_data) if b64_data else b""

                if fport == FPORT_ACK:
                    ack_info = self.decode_ack(raw_bytes)
                    if ack_info:
                        self.log(f"ACK RECEIVED ON FPORT 11: {ack_info['status_str']} (Cmd 0x{ack_info['cmd_ref']:02X})")
                        if self.on_ack:
                            self.on_ack(ack_info)
                elif fport == FPORT_DATA:
                    tel = self.decode_telemetry(raw_bytes)
                    if tel:
                        self.log(f"Telemetry Uplink: Temp={tel['temperature_c']}C, Hum={tel['humidity_pct']}%, CO2={tel['co2_ppm']}, VOC={tel['voc_index']}, Alarm={tel['alarm_str']}")
                        if self.on_data:
                            self.on_data(tel)
            except Exception as e:
                self.log(f"Error handling MQTT message: {e}")

        def on_disconnect(client, userdata, *args, **kwargs):
            self.is_connected_mqtt = False
            self.log("MQTT Disconnected")

        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = on_connect
        self.mqtt_client.on_message = on_message
        self.mqtt_client.on_disconnect = on_disconnect

        try:
            self.mqtt_client.connect(self.host, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.log(f"Failed to start MQTT connection: {e}")

    def stop_mqtt(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None
            self.is_connected_mqtt = False
            self.log("MQTT client stopped.")
