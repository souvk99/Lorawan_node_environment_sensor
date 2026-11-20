#!/usr/bin/env python3
"""
ChirpStack Wireless Configurator for WisBlock RAK4631 LoRaWAN Environment Node.
Provides full closed-loop wireless threshold tuning, live telemetry monitoring,
and real-time ACK confirmation from node Flash storage.
"""

import sys
import time
import base64
import struct
import threading
import tkinter as tk
from tkinter import ttk, messagebox

try:
    from chirpstack_client import (
        ChirpStackClient,
        PARAM_MAP,
        PARAM_ORDER,
        FPORT_DOWNLINK,
        FPORT_ACK,
        FPORT_DATA,
        ACK_STATUS_OK
    )
except ImportError:
    messagebox.showerror("Error", "Could not import chirpstack_client.py")
    sys.exit(1)


# UI Palette (Cyber Dark Theme)
BG_DARK      = "#0d1117"
CARD_BG      = "#161b22"
INPUT_BG     = "#0b0f14"
BORDER_CLR   = "#30363d"
TEXT_MAIN    = "#e6edf3"
TEXT_MUTED   = "#8b949e"
TEXT_HINT    = "#58a6ff"
ACCENT_CYAN  = "#00d2ff"
ACCENT_GREEN = "#2ea043"
ACCENT_WARN  = "#f0883e"
ACCENT_ERR   = "#f85149"
ACCENT_PURPLE= "#8957e5"

FONT_TITLE   = ("Segoe UI", 13, "bold")
FONT_SUB     = ("Segoe UI", 9)
FONT_LABEL   = ("Segoe UI", 9, "bold")
FONT_INPUT   = ("Consolas", 10)
FONT_RANGE   = ("Consolas", 8)
FONT_ERR     = ("Segoe UI", 8)
FONT_LOG     = ("Consolas", 9)
FONT_TELEMETRY = ("Segoe UI", 11, "bold")


PARAMS = [
    {
        "key": "temp_min",
        "label": "Temperature Min Threshold",
        "unit": "°C",
        "min": -40.0,
        "max": 85.0,
        "is_float": True,
        "category": "Climate (SHTC3)",
        "default": 10.0
    },
    {
        "key": "temp_max",
        "label": "Temperature Max Threshold",
        "unit": "°C",
        "min": -40.0,
        "max": 85.0,
        "is_float": True,
        "category": "Climate (SHTC3)",
        "default": 40.0
    },
    {
        "key": "hum_min",
        "label": "Humidity Min Threshold",
        "unit": "%",
        "min": 0.0,
        "max": 100.0,
        "is_float": True,
        "category": "Climate (SHTC3)",
        "default": 20.0
    },
    {
        "key": "hum_max",
        "label": "Humidity Max Threshold",
        "unit": "%",
        "min": 0.0,
        "max": 100.0,
        "is_float": True,
        "category": "Climate (SHTC3)",
        "default": 80.0
    },
    {
        "key": "co2_normal_max",
        "label": "CO2 Warning Level (Elevated)",
        "unit": "ppm",
        "min": 400,
        "max": 5000,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 500
    },
    {
        "key": "co2_warning_max",
        "label": "CO2 Alert Level (Critical)",
        "unit": "ppm",
        "min": 400,
        "max": 10000,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 1000
    },
    {
        "key": "voc_normal_max",
        "label": "VOC Index Warning Level",
        "unit": "idx",
        "min": 1,
        "max": 500,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 100
    },
    {
        "key": "voc_warning_max",
        "label": "VOC Index Alert Level",
        "unit": "idx",
        "min": 1,
        "max": 500,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 200
    },
    {
        "key": "gas_warning_ppm",
        "label": "Gas/Smoke Warning Level",
        "unit": "ppm",
        "min": 10,
        "max": 5000,
        "is_float": False,
        "category": "Flammable Gas & Timers",
        "default": 500
    },
    {
        "key": "gas_alert_ppm",
        "label": "Gas/Smoke Alert Level",
        "unit": "ppm",
        "min": 10,
        "max": 9999,
        "is_float": False,
        "category": "Flammable Gas & Timers",
        "default": 1000
    },
    {
        "key": "send_interval_s",
        "label": "Normal Send Interval",
        "unit": "sec",
        "min": 10,
        "max": 3600,
        "is_float": False,
        "category": "Flammable Gas & Timers",
        "default": 60
    },
    {
        "key": "alert_interval_s",
        "label": "Alarm Send Interval",
        "unit": "sec",
        "min": 5,
        "max": 600,
        "is_float": False,
        "category": "Flammable Gas & Timers",
        "default": 15
    }
]


class WirelessConfiguratorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ChirpStack Wireless Configurator — WisBlock RAK4631")
        self.geometry("1020x860")
        self.minsize(920, 760)
        self.configure(bg=BG_DARK)

        # ChirpStack Client
        self.client: Optional[ChirpStackClient] = None
        self.is_connected = False
        self.waiting_for_ack = False
        self.last_downlink_time = 0

        # UI state variables
        self.server_host_var = tk.StringVar(value="172.16.50.30")
        self.web_port_var = tk.StringVar(value="8080")
        self.api_port_var = tk.StringVar(value="8090")
        self.mqtt_port_var = tk.StringVar(value="1883")
        self.user_var = tk.StringVar(value="admin")
        self.pass_var = tk.StringVar(value="admin")
        self.deveui_var = tk.StringVar(value="ac1f09fffe22024e")

        self.status_var = tk.StringVar(value="Disconnected")
        self.ack_banner_var = tk.StringVar(value="Ready. Connect to ChirpStack to tune node thresholds wirelessly.")

        self.inputs = {}
        self.err_labels = {}
        self.entry_boxes = {}
        self.telemetry_labels = {}

        self._setup_styles()
        self._build_header()
        self._build_connection_bar()
        self._build_telemetry_bar()
        self._build_ack_banner()
        self._build_parameters_grid()
        self._build_action_bar()
        self._build_console()

        # Connect automatically on startup
        self.after(500, self.connect_chirpstack)

    def _setup_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TCombobox",
                        fieldbackground=INPUT_BG,
                        background=CARD_BG,
                        foreground=TEXT_MAIN,
                        darkcolor=BORDER_CLR,
                        lightcolor=BORDER_CLR)

    def _build_header(self):
        header = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        header.pack(fill="x", padx=14, pady=(10, 4))

        top_row = tk.Frame(header, bg=CARD_BG)
        top_row.pack(fill="x", padx=16, pady=8)

        title = tk.Label(top_row, text="🌐 CHIRPSTACK WIRELESS NODE CONFIGURATOR",
                         font=FONT_TITLE, fg=ACCENT_CYAN, bg=CARD_BG)
        title.pack(side="left")

        badge = tk.Label(top_row, text=" LoRaWAN Class A • Closed-Loop ACK ", font=("Consolas", 9, "bold"),
                         fg="#ffffff", bg=ACCENT_PURPLE, padx=8, pady=2)
        badge.pack(side="right")

        sub = tk.Label(header,
                       text="Over-The-Air Downlink Tuning (FPort 10) & Verified Flash Acknowledgment (FPort 11) via ChirpStack v4.",
                       font=FONT_SUB, fg=TEXT_MUTED, bg=CARD_BG)
        sub.pack(anchor="w", padx=16, pady=(0, 8))

    def _build_connection_bar(self):
        bar = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        bar.pack(fill="x", padx=14, pady=4)

        row = tk.Frame(bar, bg=CARD_BG)
        row.pack(fill="x", padx=14, pady=8)

        # Server Host
        tk.Label(row, text="ChirpStack:", font=FONT_LABEL, fg=TEXT_MAIN, bg=CARD_BG).pack(side="left", padx=(0, 4))
        tk.Entry(row, textvariable=self.server_host_var, width=14, bg=INPUT_BG, fg=TEXT_MAIN,
                 insertbackground=TEXT_MAIN, font=FONT_INPUT, bd=1, relief="solid").pack(side="left", padx=(0, 10))

        # DevEUI
        tk.Label(row, text="DevEUI:", font=FONT_LABEL, fg=TEXT_MAIN, bg=CARD_BG).pack(side="left", padx=(0, 4))
        tk.Entry(row, textvariable=self.deveui_var, width=18, bg=INPUT_BG, fg=ACCENT_CYAN,
                 insertbackground=TEXT_MAIN, font=FONT_INPUT, bd=1, relief="solid").pack(side="left", padx=(0, 10))

        # Connect / Disconnect Button
        self.btn_connect = tk.Button(row, text="⚡ Connect ChirpStack", command=self.toggle_connection,
                                     bg="#238636", fg="#ffffff", activebackground="#2ea043", activeforeground="#ffffff",
                                     font=("Segoe UI", 9, "bold"), bd=0, padx=12, pady=3, cursor="hand2")
        self.btn_connect.pack(side="left", padx=4)

        # Query Config Button
        self.btn_query = tk.Button(row, text="📥 Query Flash Config", command=self.query_node_config,
                                   bg="#21262d", fg=TEXT_MAIN, activebackground="#30363d", activeforeground=TEXT_MAIN,
                                   font=("Segoe UI", 9), bd=1, relief="solid", padx=10, pady=3, cursor="hand2")
        self.btn_query.pack(side="left", padx=6)

        # Status label
        self.lbl_status = tk.Label(row, textvariable=self.status_var, font=("Consolas", 9),
                                   fg=ACCENT_WARN, bg=CARD_BG)
        self.lbl_status.pack(side="right", padx=6)

    def _build_telemetry_bar(self):
        tbar = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        tbar.pack(fill="x", padx=14, pady=4)

        inner = tk.Frame(tbar, bg=CARD_BG)
        inner.pack(fill="x", padx=14, pady=6)

        tk.Label(inner, text="📡 LIVE SENSOR TELEMETRY (FPort 2):", font=FONT_LABEL, fg=ACCENT_CYAN, bg=CARD_BG).pack(side="left", padx=(0, 12))

        fields = [
            ("temp", "Temp: --.- °C"),
            ("hum",  "Hum: --.- %"),
            ("co2",  "CO2: ---- ppm"),
            ("voc",  "VOC: ---"),
            ("gas",  "Gas: ---- ppm"),
            ("alarm","Status: WAITING")
        ]

        for k, initial_text in fields:
            lbl = tk.Label(inner, text=initial_text, font=FONT_TELEMETRY, fg=TEXT_MAIN, bg=CARD_BG, padx=8)
            lbl.pack(side="left", padx=4)
            self.telemetry_labels[k] = lbl

    def _build_ack_banner(self):
        self.banner_frame = tk.Frame(self, bg="#21262d", bd=1, relief="solid")
        self.banner_frame.pack(fill="x", padx=14, pady=4)

        self.banner_lbl = tk.Label(self.banner_frame, textvariable=self.ack_banner_var,
                                   font=("Segoe UI", 10, "bold"), fg=TEXT_MUTED, bg="#21262d", pady=8, padx=12)
        self.banner_lbl.pack(fill="x")

    def _build_parameters_grid(self):
        container = tk.Frame(self, bg=BG_DARK)
        container.pack(fill="both", expand=True, padx=14, pady=4)

        categories = {}
        for p in PARAMS:
            cat = p["category"]
            if cat not in categories:
                categories[cat] = []
            categories[cat].append(p)

        cat_names = list(categories.keys())
        col_frames = []

        for i, cat in enumerate(cat_names):
            frame = tk.Frame(container, bg=CARD_BG, bd=1, relief="solid")
            frame.pack(side="left", fill="both", expand=True, padx=4)
            col_frames.append(frame)

            header = tk.Label(frame, text=cat.upper(), font=("Segoe UI", 9, "bold"),
                              fg=ACCENT_CYAN, bg=CARD_BG, pady=8)
            header.pack(fill="x")

            sep = tk.Frame(frame, bg=BORDER_CLR, height=1)
            sep.pack(fill="x")

            for p in categories[cat]:
                self._create_field(frame, p)

    def _create_field(self, parent, p):
        row = tk.Frame(parent, bg=CARD_BG)
        row.pack(fill="x", padx=12, pady=5)

        lbl_text = f"{p['label']} ({p['unit']})"
        lbl = tk.Label(row, text=lbl_text, font=FONT_LABEL, fg=TEXT_MAIN, bg=CARD_BG, anchor="w")
        lbl.pack(fill="x")

        box_row = tk.Frame(row, bg=CARD_BG)
        box_row.pack(fill="x", pady=(2, 0))

        var = tk.StringVar()
        self.inputs[p["key"]] = var

        entry = tk.Entry(box_row, textvariable=var, font=FONT_INPUT, bg=INPUT_BG, fg=TEXT_MAIN,
                         insertbackground=TEXT_MAIN, bd=1, relief="solid")
        entry.pack(side="left", fill="x", expand=True)
        self.entry_boxes[p["key"]] = entry

        # Live range check on keystroke
        var.trace_add("write", lambda *args, key=p["key"]: self._validate_single_field(key))

        unit_lbl = tk.Label(box_row, text=p["unit"], font=FONT_RANGE, fg=TEXT_MUTED, bg=CARD_BG, width=4)
        unit_lbl.pack(side="right", padx=(4, 0))

        help_row = tk.Frame(row, bg=CARD_BG)
        help_row.pack(fill="x", pady=(1, 0))

        rng_text = f"Valid: {p['min']} to {p['max']}"
        rng_lbl = tk.Label(help_row, text=rng_text, font=FONT_RANGE, fg=TEXT_MUTED, bg=CARD_BG, anchor="w")
        rng_lbl.pack(side="left")

        err_lbl = tk.Label(help_row, text="", font=FONT_ERR, fg=ACCENT_ERR, bg=CARD_BG, anchor="e")
        err_lbl.pack(side="right")
        self.err_labels[p["key"]] = err_lbl

    def _validate_single_field(self, key):
        param = next((p for p in PARAMS if p["key"] == key), None)
        if not param:
            return True

        val_str = self.inputs[key].get().strip()
        if not val_str:
            self.err_labels[key].config(text="")
            self.entry_boxes[key].config(highlightbackground=BORDER_CLR, highlightcolor=ACCENT_CYAN)
            return True

        try:
            val = float(val_str) if param["is_float"] else int(val_str)
            if val < param["min"] or val > param["max"]:
                self.err_labels[key].config(text=f"Range {param['min']}-{param['max']}")
                self.entry_boxes[key].config(highlightbackground=ACCENT_ERR, highlightcolor=ACCENT_ERR)
                return False
            else:
                self.err_labels[key].config(text="")
                self.entry_boxes[key].config(highlightbackground=BORDER_CLR, highlightcolor=ACCENT_CYAN)
                return True
        except ValueError:
            self.err_labels[key].config(text="Invalid number")
            self.entry_boxes[key].config(highlightbackground=ACCENT_ERR, highlightcolor=ACCENT_ERR)
            return False

    def _build_action_bar(self):
        bar = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        bar.pack(fill="x", padx=14, pady=4)

        inner = tk.Frame(bar, bg=CARD_BG)
        inner.pack(fill="x", padx=16, pady=8)

        # Transmit via ChirpStack button
        self.btn_send = tk.Button(inner, text="🚀 Transmit Over-The-Air via ChirpStack",
                                  command=self.send_wireless_config,
                                  bg="#1f6feb", fg="#ffffff", activebackground="#388bfd", activeforeground="#ffffff",
                                  font=("Segoe UI", 10, "bold"), bd=0, padx=16, pady=6, cursor="hand2")
        self.btn_send.pack(side="left", padx=(0, 10))

        # Reset button
        btn_reset = tk.Button(inner, text="🔄 Reset Defaults (Wireless)", command=self.reset_wireless_config,
                              bg="#21262d", fg=TEXT_MUTED, activebackground="#da3633", activeforeground="#ffffff",
                              font=("Segoe UI", 9), bd=1, relief="solid", padx=10, pady=5, cursor="hand2")
        btn_reset.pack(side="left", padx=4)

        # Clear Fields
        btn_clear = tk.Button(inner, text="Clear Blanks", command=self.clear_all_fields,
                              bg="#21262d", fg=TEXT_MUTED, activebackground="#30363d", activeforeground=TEXT_MAIN,
                              font=("Segoe UI", 9), bd=1, relief="solid", padx=10, pady=5, cursor="hand2")
        btn_clear.pack(side="left", padx=4)

        # Helper hint
        hint = tk.Label(inner, text="💡 Tip: Leave unmodified fields blank to update only selected items.",
                        font=FONT_SUB, fg=TEXT_MUTED, bg=CARD_BG)
        hint.pack(side="right")

    def _build_console(self):
        cframe = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        cframe.pack(fill="both", expand=True, padx=14, pady=(4, 10))

        header = tk.Frame(cframe, bg=CARD_BG)
        header.pack(fill="x", padx=12, pady=4)

        tk.Label(header, text="WIRELESS ACTIVITY & ACK AUDIT LOG", font=("Segoe UI", 8, "bold"),
                 fg=TEXT_MUTED, bg=CARD_BG).pack(side="left")

        btn_clear_log = tk.Button(header, text="Clear", command=self.clear_console,
                                  font=("Segoe UI", 7), bg="#21262d", fg=TEXT_MUTED, bd=0, padx=6, cursor="hand2")
        btn_clear_log.pack(side="right")

        self.console = tk.Text(cframe, height=5, bg=INPUT_BG, fg=TEXT_MAIN,
                               insertbackground=TEXT_MAIN, font=FONT_LOG,
                               bd=0, padx=8, pady=6)
        self.console.pack(fill="both", expand=True)

    def log(self, text, color=None):
        def _append():
            ts = time.strftime("[%H:%M:%S] ")
            self.console.insert("end", ts + text + "\n")
            if color:
                end_idx = self.console.index("end-1c")
                start_idx = f"{end_idx} linestart + {len(ts)}c"
                self.console.tag_add(color, start_idx, end_idx)
                self.console.tag_config(color, foreground=color)
            self.console.see("end")
        self.after(0, _append)

    def clear_console(self):
        self.console.delete("1.0", "end")

    def clear_all_fields(self):
        for var in self.inputs.values():
            var.set("")

    # ============================================================
    # CHIRPSTACK CONNECTION
    # ============================================================

    def toggle_connection(self):
        if self.is_connected:
            self.disconnect_chirpstack()
        else:
            self.connect_chirpstack()

    def connect_chirpstack(self):
        host = self.server_host_var.get().strip()
        deveui = self.deveui_var.get().strip()

        self.status_var.set("Connecting to ChirpStack...")
        self.lbl_status.config(fg=ACCENT_WARN)

        def _do_connect():
            try:
                self.client = ChirpStackClient(
                    host=host,
                    web_port=int(self.web_port_var.get().strip()),
                    api_port=int(self.api_port_var.get().strip()),
                    mqtt_port=int(self.mqtt_port_var.get().strip()),
                    username=self.user_var.get().strip(),
                    password=self.pass_var.get().strip(),
                    dev_eui=deveui
                )
                self.client.on_log = lambda msg: self.log(msg)
                self.client.on_ack = self.handle_ack_received
                self.client.on_data = self.handle_telemetry_received

                if not self.client.login():
                    self.after(0, lambda: self._on_connect_failed("Could not authenticate with ChirpStack API."))
                    return

                self.client.start_mqtt()
                self.after(0, self._on_connect_success)
            except Exception as e:
                self.after(0, lambda: self._on_connect_failed(str(e)))

        threading.Thread(target=_do_connect, daemon=True).start()

    def _on_connect_success(self):
        self.is_connected = True
        self.status_var.set("🟢 ChirpStack Online")
        self.lbl_status.config(fg=ACCENT_GREEN)
        self.btn_connect.config(text="Disconnect", bg="#da3633", activebackground="#f85149")
        self.log("Connected to ChirpStack API & MQTT Broker.", ACCENT_GREEN)

    def _on_connect_failed(self, err):
        self.is_connected = False
        self.status_var.set("🔴 Disconnected")
        self.lbl_status.config(fg=ACCENT_ERR)
        self.btn_connect.config(text="⚡ Connect ChirpStack", bg="#238636")
        self.log(f"Connection Failed: {err}", ACCENT_ERR)

    def disconnect_chirpstack(self):
        if self.client:
            self.client.stop_mqtt()
            self.client = None
        self.is_connected = False
        self.status_var.set("Disconnected")
        self.lbl_status.config(fg=TEXT_MUTED)
        self.btn_connect.config(text="⚡ Connect ChirpStack", bg="#238636")
        self.log("Disconnected from ChirpStack.")

    # ============================================================
    # TELEMETRY & ACK HANDLERS
    # ============================================================

    def handle_telemetry_received(self, tel):
        def _update():
            self.telemetry_labels["temp"].config(text=f"Temp: {tel['temperature_c']:.1f} °C")
            self.telemetry_labels["hum"].config(text=f"Hum: {tel['humidity_pct']:.1f} %")
            self.telemetry_labels["co2"].config(text=f"CO2: {tel['co2_ppm']} ppm")
            self.telemetry_labels["voc"].config(text=f"VOC: {tel['voc_index']:.0f}")
            self.telemetry_labels["gas"].config(text=f"Gas: {tel['gas_ppm']:.0f} ppm")

            if tel["alarm_status"] == 1:
                self.telemetry_labels["alarm"].config(text="Status: ⚠️ ALARM ACTIVE", fg=ACCENT_ERR)
            else:
                self.telemetry_labels["alarm"].config(text="Status: 🟢 NORMAL", fg=ACCENT_GREEN)
        self.after(0, _update)

    def handle_ack_received(self, ack_info):
        def _update():
            self.waiting_for_ack = False
            rtt = time.time() - self.last_downlink_time if self.last_downlink_time > 0 else 0

            status_code = ack_info["status_code"]
            status_str = ack_info["status_str"]
            cmd_ref = ack_info["cmd_ref"]
            cfg = ack_info["config"]

            if status_code == ACK_STATUS_OK:
                # Update Banner
                self.banner_frame.config(bg="#1f6f39")
                self.banner_lbl.config(
                    bg="#1f6f39", fg="#ffffff",
                    text=f"✅ CONFIRMED: Node successfully applied configuration & saved to Flash! [RTT: {rtt:.1f}s | Cmd: 0x{cmd_ref:02X}]"
                )
                self.log(f"ACK VERIFIED: Flash configuration confirmed over LoRaWAN (RTT: {rtt:.1f}s)!", ACCENT_GREEN)

                # Update entry fields with exact confirmed Flash values
                for k, v in cfg.items():
                    if k in self.inputs:
                        self.inputs[k].set(str(v))
                        self.err_labels[k].config(text="")
            else:
                self.banner_frame.config(bg="#8b1111")
                self.banner_lbl.config(
                    bg="#8b1111", fg="#ffffff",
                    text=f"❌ ACK ERROR: Node reported error {status_str} (Code {status_code})"
                )
                self.log(f"ACK ERROR: Node rejected or failed to save (Status: {status_str})", ACCENT_ERR)

            self.btn_send.config(text="🚀 Transmit Over-The-Air via ChirpStack", state="normal")
            self.btn_query.config(state="normal")
        self.after(0, _update)

    # ============================================================
    # ACTIONS
    # ============================================================

    def send_wireless_config(self):
        if not self.is_connected or not self.client:
            messagebox.showerror("Not Connected", "Please connect to ChirpStack first.")
            return

        # Gather updated parameters (skip blanks)
        updates = {}
        for p in PARAMS:
            k = p["key"]
            val_str = self.inputs[k].get().strip()
            if val_str:
                if not self._validate_single_field(k):
                    messagebox.showerror("Validation Error", f"Invalid value for {p['label']}.")
                    return
                updates[k] = float(val_str) if p["is_float"] else int(val_str)

        if not updates:
            messagebox.showinfo("No Changes", "All fields are blank. Enter at least one threshold to update.")
            return

        payload_bytes = self.client.encode_set_params(updates)
        self.log(f"Queueing Downlink: {len(updates)} parameters ({len(payload_bytes)} bytes) -> Hex: {payload_bytes.hex()}")

        # Update UI to waiting state
        self.waiting_for_ack = True
        self.last_downlink_time = time.time()
        self.banner_frame.config(bg="#593c00")
        self.banner_lbl.config(
            bg="#593c00", fg="#f0883e",
            text="⏳ Downlink queued in ChirpStack. Waiting for node's next transmission window to apply & ACK..."
        )
        self.btn_send.config(text="⏳ Downlink Queued (Awaiting Node ACK...)", state="disabled")
        self.btn_query.config(state="disabled")

        def _do_send():
            queue_id = self.client.enqueue_downlink(payload_bytes, fport=FPORT_DOWNLINK)
            if not queue_id:
                self.after(0, lambda: self._on_send_failed("ChirpStack rejected downlink enqueue."))

        threading.Thread(target=_do_send, daemon=True).start()

    def _on_send_failed(self, err):
        self.waiting_for_ack = False
        self.banner_frame.config(bg="#8b1111")
        self.banner_lbl.config(bg="#8b1111", fg="#ffffff", text=f"❌ Failed to queue downlink: {err}")
        self.btn_send.config(text="🚀 Transmit Over-The-Air via ChirpStack", state="normal")
        self.btn_query.config(state="normal")
        self.log(f"Enqueue Error: {err}", ACCENT_ERR)

    def query_node_config(self):
        if not self.is_connected or not self.client:
            messagebox.showerror("Not Connected", "Please connect to ChirpStack first.")
            return

        payload_bytes = self.client.encode_query()
        self.log("Queueing Query Command (0x04) to fetch active Flash config...")

        self.waiting_for_ack = True
        self.last_downlink_time = time.time()
        self.banner_frame.config(bg="#593c00")
        self.banner_lbl.config(
            bg="#593c00", fg="#f0883e",
            text="⏳ Query queued. Waiting for node transmission window to receive active Flash parameters..."
        )
        self.btn_send.config(state="disabled")
        self.btn_query.config(state="disabled")

        def _do_query():
            self.client.enqueue_downlink(payload_bytes, fport=FPORT_DOWNLINK)

        threading.Thread(target=_do_query, daemon=True).start()

    def reset_wireless_config(self):
        if not self.is_connected or not self.client:
            messagebox.showerror("Not Connected", "Please connect to ChirpStack first.")
            return

        if not messagebox.askyesno("Confirm Wireless Reset",
                                   "Are you sure you want to reset all node thresholds to factory defaults wirelessly?"):
            return

        payload_bytes = self.client.encode_reset()
        self.log("Queueing Reset Command (0x03) to restore factory defaults in Flash...")

        self.waiting_for_ack = True
        self.last_downlink_time = time.time()
        self.banner_frame.config(bg="#593c00")
        self.banner_lbl.config(
            bg="#593c00", fg="#f0883e",
            text="⏳ Reset command queued. Waiting for node transmission window to reset Flash & reply..."
        )
        self.btn_send.config(state="disabled")
        self.btn_query.config(state="disabled")

        def _do_reset():
            self.client.enqueue_downlink(payload_bytes, fport=FPORT_DOWNLINK)

        threading.Thread(target=_do_reset, daemon=True).start()


if __name__ == "__main__":
    app = WirelessConfiguratorApp()
    app.mainloop()
