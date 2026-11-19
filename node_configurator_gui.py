#!/usr/bin/env python3
"""
WisBlock RAK4631 Environment Monitoring Node Configurator
Hardware abstraction and dynamic alert threshold manager.
"""

import sys
import time
import json
import tkinter as tk
from tkinter import ttk, messagebox
import threading

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    messagebox.showerror(
        "Missing Dependency",
        "pyserial is required. Please install it using: pip install pyserial"
    )
    sys.exit(1)


# Color Palette (Techie Dark Cyber / Clean Slate)
BG_DARK      = "#0f141c"
CARD_BG      = "#171e28"
INPUT_BG     = "#0b0f14"
INPUT_BORDER = "#2b3544"
TEXT_MAIN    = "#e6edf3"
TEXT_MUTED   = "#8b949e"
TEXT_HINT    = "#58a6ff"
ACCENT_CYAN  = "#00d2ff"
ACCENT_GREEN = "#2ea043"
ACCENT_WARN  = "#f0883e"
ACCENT_ERR   = "#f85149"
BORDER_CLR   = "#30363d"
FONT_TITLE   = ("Segoe UI", 14, "bold")
FONT_SUB     = ("Segoe UI", 9)
FONT_LABEL   = ("Segoe UI", 9, "bold")
FONT_INPUT   = ("Consolas", 10)
FONT_RANGE   = ("Consolas", 8)
FONT_ERR     = ("Segoe UI", 8)
FONT_LOG     = ("Consolas", 9)


# Parameter Definitions & Physical Constraints
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
        "key": "co2_norm",
        "label": "CO2 Warning Level (Elevated)",
        "unit": "ppm",
        "min": 400,
        "max": 5000,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 500
    },
    {
        "key": "co2_warn",
        "label": "CO2 Alert Level (Critical)",
        "unit": "ppm",
        "min": 400,
        "max": 10000,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 1000
    },
    {
        "key": "voc_norm",
        "label": "VOC Index Warning Level",
        "unit": "idx",
        "min": 1,
        "max": 500,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 100
    },
    {
        "key": "voc_warn",
        "label": "VOC Index Alert Level",
        "unit": "idx",
        "min": 1,
        "max": 500,
        "is_float": False,
        "category": "Air Quality (SCD30 & SGP40)",
        "default": 200
    },
    {
        "key": "gas_warn",
        "label": "Gas/Smoke Warning Level",
        "unit": "ppm",
        "min": 10,
        "max": 5000,
        "is_float": False,
        "category": "Flammable Gas & Timing",
        "default": 500
    },
    {
        "key": "gas_alert",
        "label": "Gas/Smoke Alert Level",
        "unit": "ppm",
        "min": 10,
        "max": 9999,
        "is_float": False,
        "category": "Flammable Gas & Timing",
        "default": 1000
    },
    {
        "key": "send_s",
        "label": "Normal Send Interval",
        "unit": "sec",
        "min": 10,
        "max": 3600,
        "is_float": False,
        "category": "Flammable Gas & Timing",
        "default": 60
    },
    {
        "key": "alert_s",
        "label": "Alarm Send Interval",
        "unit": "sec",
        "min": 5,
        "max": 600,
        "is_float": False,
        "category": "Flammable Gas & Timing",
        "default": 15
    }
]


class NodeConfiguratorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("WisBlock RAK4631 Node Alert Configurator")
        self.geometry("960x780")
        self.minsize(860, 700)
        self.configure(bg=BG_DARK)

        self.ser = None
        self.port_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Status: Ready (Select COM port)")
        
        self.inputs = {}     # key -> tk.StringVar
        self.err_labels = {} # key -> tk.Label
        self.entry_boxes = {}
        self.cached_config = {}

        self._setup_styles()
        self._build_header()
        self._build_com_bar()
        self._build_parameters_grid()
        self._build_action_bar()
        self._build_console()

        self.refresh_ports()

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
        header.pack(fill="x", padx=14, pady=(12, 6))

        top_row = tk.Frame(header, bg=CARD_BG)
        top_row.pack(fill="x", padx=16, pady=10)

        title = tk.Label(top_row, text="⚡ WISBLOCK RAK4631 ALERT TUNER", font=FONT_TITLE, fg=ACCENT_CYAN, bg=CARD_BG)
        title.pack(side="left")

        badge = tk.Label(top_row, text=" v07_working_no_bugs • Dynamic Config ", font=("Consolas", 9, "bold"),
                         fg="#ffffff", bg="#1f6feb", padx=8, pady=2)
        badge.pack(side="right")

        sub = tk.Label(header, text="Abstracted USB threshold controller — tune node alerts and sensor alarms without re-flashing C++ code.",
                       font=FONT_SUB, fg=TEXT_MUTED, bg=CARD_BG)
        sub.pack(anchor="w", padx=16, pady=(0, 10))

    def _build_com_bar(self):
        bar = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        bar.pack(fill="x", padx=14, pady=6)

        inner = tk.Frame(bar, bg=CARD_BG)
        inner.pack(fill="x", padx=16, pady=8)

        lbl = tk.Label(inner, text="Target Device (COM Port):", font=FONT_LABEL, fg=TEXT_MAIN, bg=CARD_BG)
        lbl.pack(side="left", padx=(0, 8))

        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var, width=32, state="readonly")
        self.port_combo.pack(side="left", padx=4)

        btn_refresh = tk.Button(inner, text="🔄 Refresh", command=self.refresh_ports,
                                bg="#21262d", fg=TEXT_MAIN, activebackground="#30363d", activeforeground=TEXT_MAIN,
                                bd=1, relief="solid", padx=8, cursor="hand2")
        btn_refresh.pack(side="left", padx=6)

        btn_read = tk.Button(inner, text="📥 Read Values from Node", command=self.read_from_device,
                             bg="#238636", fg="#ffffff", activebackground="#2ea043", activeforeground="#ffffff",
                             font=("Segoe UI", 9, "bold"), bd=0, padx=12, pady=3, cursor="hand2")
        btn_read.pack(side="right", padx=4)

        self.status_pill = tk.Label(inner, textvariable=self.status_var, font=("Consolas", 9),
                                    fg=TEXT_MUTED, bg=CARD_BG)
        self.status_pill.pack(side="right", padx=14)

    def _build_parameters_grid(self):
        container = tk.Frame(self, bg=BG_DARK)
        container.pack(fill="both", expand=True, padx=14, pady=6)

        # 3 logical category columns
        categories = [
            ("Climate (SHTC3)", "#58a6ff"),
            ("Air Quality (SCD30 & SGP40)", "#7ee787"),
            ("Flammable Gas & Timing", "#d2a8ff")
        ]

        col_frames = {}
        for idx, (cat_name, cat_color) in enumerate(categories):
            frame = tk.Frame(container, bg=CARD_BG, bd=1, relief="solid")
            frame.grid(row=0, column=idx, sticky="nsew", padx=6, pady=2)
            container.grid_columnconfigure(idx, weight=1)

            # Category Header
            cat_header = tk.Frame(frame, bg="#1c2430", height=28)
            cat_header.pack(fill="x")
            cat_lbl = tk.Label(cat_header, text=cat_name, font=("Segoe UI", 10, "bold"),
                               fg=cat_color, bg="#1c2430", padx=10, pady=4)
            cat_lbl.pack(anchor="w")

            col_frames[cat_name] = frame

        # Populate parameter inputs inside each category
        for p in PARAMS:
            parent = col_frames[p["category"]]
            box = tk.Frame(parent, bg=CARD_BG)
            box.pack(fill="x", padx=12, pady=6)

            # Label Row
            top_line = tk.Frame(box, bg=CARD_BG)
            top_line.pack(fill="x")
            plbl = tk.Label(top_line, text=p["label"], font=FONT_LABEL, fg=TEXT_MAIN, bg=CARD_BG)
            plbl.pack(side="left")

            # Entry & Range Row
            entry_row = tk.Frame(box, bg=CARD_BG)
            entry_row.pack(fill="x", pady=(3, 1))

            var = tk.StringVar(value="")  # Intentionally blank initially as requested
            self.inputs[p["key"]] = var

            entry = tk.Entry(entry_row, textvariable=var, font=FONT_INPUT, bg=INPUT_BG, fg="#58a6ff",
                             insertbackground="#58a6ff", bd=1, relief="solid", highlightthickness=1,
                             highlightcolor=ACCENT_CYAN, highlightbackground=INPUT_BORDER, width=10)
            entry.pack(side="left", padx=(0, 6))
            self.entry_boxes[p["key"]] = entry

            # Real-time validation on typing
            entry.bind("<KeyRelease>", lambda e, k=p["key"]: self.validate_field(k))

            range_text = f"Range: {p['min']} to {p['max']} {p['unit']}"
            range_lbl = tk.Label(entry_row, text=range_text, font=FONT_RANGE, fg=TEXT_MUTED, bg=CARD_BG)
            range_lbl.pack(side="left")

            # Inline Error Label
            err_lbl = tk.Label(box, text="", font=FONT_ERR, fg=ACCENT_ERR, bg=CARD_BG, anchor="w")
            err_lbl.pack(fill="x")
            self.err_labels[p["key"]] = err_lbl

    def _build_action_bar(self):
        bar = tk.Frame(self, bg=CARD_BG, bd=1, relief="solid")
        bar.pack(fill="x", padx=14, pady=6)

        inner = tk.Frame(bar, bg=CARD_BG)
        inner.pack(fill="x", padx=16, pady=10)

        btn_save = tk.Button(inner, text="🚀 APPLY & SAVE TO DEVICE (FLASH)", command=self.apply_to_device,
                             bg="#1f6feb", fg="#ffffff", activebackground="#388bfd", activeforeground="#ffffff",
                             font=("Segoe UI", 10, "bold"), bd=0, padx=18, pady=6, cursor="hand2")
        btn_save.pack(side="left", padx=(0, 10))

        btn_default = tk.Button(inner, text="↺ Load Factory Defaults", command=self.load_defaults,
                                bg="#21262d", fg=TEXT_MAIN, activebackground="#30363d", activeforeground=TEXT_MAIN,
                                font=("Segoe UI", 9), bd=1, relief="solid", padx=12, pady=5, cursor="hand2")
        btn_default.pack(side="left", padx=4)

        btn_clear = tk.Button(inner, text="Clear All Fields", command=self.clear_fields,
                              bg="#21262d", fg=TEXT_MUTED, activebackground="#30363d", activeforeground=TEXT_MAIN,
                              font=("Segoe UI", 9), bd=1, relief="solid", padx=10, pady=5, cursor="hand2")
        btn_clear.pack(side="left", padx=4)

        self.global_err = tk.Label(inner, text="", font=("Segoe UI", 9, "bold"), fg=ACCENT_ERR, bg=CARD_BG)
        self.global_err.pack(side="right", padx=10)

    def _build_console(self):
        con_frame = tk.Frame(self, bg=BG_DARK)
        con_frame.pack(fill="x", padx=14, pady=(2, 12))

        con_hdr = tk.Frame(con_frame, bg=BG_DARK)
        con_hdr.pack(fill="x", pady=(0, 2))
        lbl = tk.Label(con_hdr, text="Console Output & Hardware Log:", font=("Segoe UI", 8, "bold"),
                       fg=TEXT_MUTED, bg=BG_DARK)
        lbl.pack(side="left")

        self.log_text = tk.Text(con_frame, height=5, bg="#070a0e", fg="#58a6ff", insertbackground="#ffffff",
                                font=FONT_LOG, bd=1, relief="solid", highlightthickness=0)
        self.log_text.pack(fill="x")
        self.log("Ready. Select the COM port of your WisBlock node to begin.")

    def log(self, message):
        t = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{t}] {message}\n")
        self.log_text.see("end")

    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = []
        preferred_idx = 0
        for idx, p in enumerate(ports):
            desc = f"{p.device} - {p.description}"
            port_list.append(desc)
            if "239A:8029" in (p.hwid or "") or "COM8" in p.device:
                preferred_idx = idx

        self.port_combo["values"] = port_list
        if port_list:
            self.port_combo.current(preferred_idx)
            self.log(f"Detected {len(port_list)} serial port(s). Active selection: {port_list[preferred_idx]}")
        else:
            self.port_var.set("")
            self.log("No serial ports found. Please connect your WisBlock USB cable.")

    def get_selected_com(self):
        val = self.port_var.get()
        if not val:
            return None
        return val.split(" ")[0].strip()

    def validate_field(self, key):
        p = next(item for item in PARAMS if item["key"] == key)
        raw = self.inputs[key].get().strip()

        if not raw:
            self.err_labels[key].config(text="")
            self.entry_boxes[key].config(highlightbackground=INPUT_BORDER)
            return True, None

        try:
            val = float(raw) if p["is_float"] else int(raw)
        except ValueError:
            self.err_labels[key].config(text=f"✖ Error: Must be numeric ({'decimal' if p['is_float'] else 'integer'})")
            self.entry_boxes[key].config(highlightbackground=ACCENT_ERR)
            return False, "Not numeric"

        if val < p["min"] or val > p["max"]:
            self.err_labels[key].config(text=f"✖ Out of range: must be between {p['min']} and {p['max']}")
            self.entry_boxes[key].config(highlightbackground=ACCENT_ERR)
            return False, "Out of range"

        # Valid
        self.err_labels[key].config(text="")
        self.entry_boxes[key].config(highlightbackground=ACCENT_GREEN)
        return True, val

    def validate_all(self):
        has_errors = False
        parsed_vals = {}
        empty_count = 0

        for p in PARAMS:
            k = p["key"]
            raw = self.inputs[k].get().strip()
            if not raw:
                # Blank is allowed! It keeps the device's current value.
                empty_count += 1
                self.err_labels[k].config(text="")
                self.entry_boxes[k].config(highlightbackground=INPUT_BORDER)
                continue

            valid, val = self.validate_field(k)
            if not valid:
                has_errors = True
            else:
                parsed_vals[k] = val

        if has_errors:
            self.global_err.config(text="⚠ Please fix the errors highlighted above.")
            return False, None

        if empty_count == len(PARAMS):
            self.global_err.config(text="⚠ Enter at least one parameter to update (or click Read Values from Node).")
            return False, None

        # Relational validation
        relations = [
            ("temp_min", "temp_max", "Temp Min must be less than Temp Max"),
            ("hum_min", "hum_max", "Humidity Min must be less than Humidity Max"),
            ("co2_norm", "co2_warn", "CO2 Warning must be less than CO2 Alert"),
            ("voc_norm", "voc_warn", "VOC Warning must be less than VOC Alert"),
            ("gas_warn", "gas_alert", "Gas Warning must be less than Gas Alert"),
            ("alert_s", "send_s", "Alert send interval must be <= Normal send interval")
        ]

        for k1, k2, msg in relations:
            val1 = parsed_vals.get(k1, self.cached_config.get(k1))
            val2 = parsed_vals.get(k2, self.cached_config.get(k2))
            if val1 is not None and val2 is not None:
                if val1 >= val2:
                    if k1 in parsed_vals:
                        self.err_labels[k1].config(text=f"✖ {msg}")
                        self.entry_boxes[k1].config(highlightbackground=ACCENT_ERR)
                    if k2 in parsed_vals:
                        self.err_labels[k2].config(text=f"✖ {msg}")
                        self.entry_boxes[k2].config(highlightbackground=ACCENT_ERR)
                    self.global_err.config(text=f"⚠ {msg}")
                    return False, None

        self.global_err.config(text="")
        return True, parsed_vals

    def clear_fields(self):
        for k in self.inputs:
            self.inputs[k].set("")
            self.err_labels[k].config(text="")
            self.entry_boxes[k].config(highlightbackground=INPUT_BORDER)
        self.global_err.config(text="")
        self.log("Cleared all inputs.")

    def load_defaults(self):
        for p in PARAMS:
            self.inputs[p["key"]].set(str(p["default"]))
            self.err_labels[p["key"]].config(text="")
            self.entry_boxes[p["key"]].config(highlightbackground=ACCENT_GREEN)
        self.global_err.config(text="")
        self.log("Loaded factory default values into input fields.")

    def read_from_device(self):
        port = self.get_selected_com()
        if not port:
            messagebox.showwarning("No Port Selected", "Please select a valid COM port first.")
            return

        threading.Thread(target=self._worker_read_config, args=(port,), daemon=True).start()

    def _worker_read_config(self, port):
        self.status_var.set(f"Connecting to {port}...")
        self.log(f"Connecting to {port} @ 115200 baud to query current configuration...")
        try:
            ser = serial.Serial(port, 115200, timeout=2.5)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.4)
            ser.reset_input_buffer()
            ser.write(b"GET_CONFIG\n")

            start = time.time()
            data_json = None
            while time.time() - start < 3.5:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if "CONFIG_DATA:" in line:
                    raw_json = line.split("CONFIG_DATA:")[1].strip()
                    data_json = json.loads(raw_json)
                    break
            ser.close()

            if data_json:
                self.log(f"Successfully received parameters from {port}: {data_json}")
                self.status_var.set(f"● Connected: {port} (Data Synced)")
                # Populate UI on main thread
                self.after(0, lambda: self._populate_received_config(data_json))
            else:
                self.log(f"Node on {port} did not respond with CONFIG_DATA. (Make sure node is powered on).")
                self.status_var.set(f"● {port}: No Response")
                self.after(0, lambda: messagebox.showwarning("No Response", f"Node on {port} did not reply. Check USB connection."))
        except Exception as e:
            self.log(f"Serial Error: {e}")
            self.status_var.set("Error: Connection Failed")
            self.after(0, lambda: messagebox.showerror("Serial Error", str(e)))

    def _populate_received_config(self, data):
        self.cached_config.update(data)
        for k, v in data.items():
            if k in self.inputs:
                self.inputs[k].set(str(v))
                self.err_labels[k].config(text="")
                self.entry_boxes[k].config(highlightbackground=ACCENT_GREEN)
        self.global_err.config(text="")
        self.log("All parameter fields populated from hardware Flash memory!")

    def apply_to_device(self):
        valid, parsed_vals = self.validate_all()
        if not valid:
            return

        port = self.get_selected_com()
        if not port:
            messagebox.showwarning("No Port Selected", "Please select a valid COM port first.")
            return

        threading.Thread(target=self._worker_apply_config, args=(port, parsed_vals), daemon=True).start()

    def _worker_apply_config(self, port, vals):
        num_updated = len(vals)
        keys_summary = ", ".join(vals.keys())
        self.status_var.set(f"Updating {num_updated} item(s) on {port}...")
        self.log(f"Writing {num_updated} parameter(s) to {port}: {keys_summary} (other parameters remain unchanged)")

        # Build command: SET_CONFIG:key1=val1;key2=val2...
        pairs = [f"{k}={v}" for k, v in vals.items()]
        cmd = "SET_CONFIG:" + ";".join(pairs) + "\n"

        try:
            ser = serial.Serial(port, 115200, timeout=2.5)
            ser.dtr = True
            ser.rts = True
            time.sleep(0.4)
            ser.reset_input_buffer()
            ser.write(cmd.encode("utf-8"))

            start = time.time()
            saved = False
            while time.time() - start < 3.5:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if "CONFIG_RESULT:OK_SAVED" in line:
                    saved = True
                    break
            ser.close()

            if saved:
                self.cached_config.update(vals)
                self.log(f"SUCCESS: Node on {port} saved {num_updated} updated parameter(s) to internal Flash memory!")
                self.status_var.set(f"● Saved: {port} ({num_updated} updated)")
                self.after(0, lambda: messagebox.showinfo(
                    "Success",
                    f"Successfully updated {num_updated} parameter(s) in Node Flash!\n\n"
                    f"Updated fields: {keys_summary}\n\n"
                    "All other parameters remain unchanged on the hardware."
                ))
            else:
                self.log(f"Failed to confirm save from {port}.")
                self.status_var.set(f"● Error on {port}")
                self.after(0, lambda: messagebox.showerror("Save Failed", "Node did not confirm configuration save."))
        except Exception as e:
            self.log(f"Serial Error: {e}")
            self.status_var.set("Error: Connection Failed")
            self.after(0, lambda: messagebox.showerror("Serial Error", str(e)))


if __name__ == "__main__":
    app = NodeConfiguratorApp()
    app.mainloop()
