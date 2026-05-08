#!/usr/bin/env python3
"""
PolarAlign Controller – Desktop GUI for the ESP32 Polar Alignment System
Cross-platform (Windows / macOS / Linux) — requires Python 3.8+ and pyserial.

Install:  pip3 install pyserial
Run:      python3 PolarAlignGUI.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import threading
import re
import json
import platform

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial is required.  Install with:  pip3 install pyserial")
    raise SystemExit(1)

IS_MAC = platform.system() == "Darwin"
MONO = "Menlo" if IS_MAC else "Consolas"


# ─────────────────────────────────────────────────────────────
# CLICKABLE LABEL (works everywhere)
# ─────────────────────────────────────────────────────────────
def make_button(parent, text, bg, fg="white", font=None, width=None,
                height=1, padx=8, pady=4, command=None):
    font = font or ("Helvetica", 12, "bold")
    lbl = tk.Label(parent, text=text, bg=bg, fg=fg, font=font,
                   cursor="hand2", relief="raised", bd=2,
                   padx=padx, pady=pady)
    if width:
        lbl.configure(width=width)

    def _enter(_):  lbl.configure(relief="groove")
    def _leave(_):  lbl.configure(relief="raised")
    def _press(_):  lbl.configure(relief="sunken")
    def _release(_):
        lbl.configure(relief="raised")
        if command: command()

    lbl.bind("<Enter>", _enter)
    lbl.bind("<Leave>", _leave)
    lbl.bind("<ButtonPress-1>", _press)
    lbl.bind("<ButtonRelease-1>", _release)
    return lbl


# ─────────────────────────────────────────────────────────────
# SERIAL MANAGER
# ─────────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self, on_line, on_status, on_disconnect):
        self.ser = None
        self.port = None
        self._running = False
        self._thread = None
        self._on_line = on_line
        self._on_status = on_status
        self._on_disconnect = on_disconnect
        self._lock = threading.Lock()
        self._re = re.compile(
            r"<(?P<st>\w+)\|MPos:"
            r"(?P<x>[+-]?\d+\.?\d*),(?P<y>[+-]?\d+\.?\d*),(?P<z>[+-]?\d+\.?\d*)\|")

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baud=115200):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.3)
            self.port = port
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            return True
        except Exception as e:
            self._on_line(f"CONNECTION ERROR: {e}")
            return False

    def disconnect(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except: pass
        self.ser = None
        self.port = None

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def send(self, cmd):
        if not self.connected: return
        with self._lock:
            try: self.ser.write((cmd + "\n").encode("ascii"))
            except Exception as e:
                self._on_line(f"SEND ERROR: {e}")
                self._on_disconnect()

    def poll(self):
        if not self.connected: return
        with self._lock:
            try: self.ser.write(b"?")
            except: pass

    def _loop(self):
        while self._running and self.ser and self.ser.is_open:
            try:
                raw = self.ser.readline()
                if not raw: continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line: continue
                m = self._re.search(line)
                if m:
                    self._on_status(m.group("st"),
                                    float(m.group("x")),
                                    float(m.group("y")))
                else:
                    self._on_line(line)
            except serial.SerialException:
                self._on_line("SERIAL ERROR — disconnected")
                self._on_disconnect()
                break
            except: continue


# ─────────────────────────────────────────────────────────────
# CONFIG DEFINITIONS
# ─────────────────────────────────────────────────────────────
CONFIG_PARAMS = [
    ("MOTOR_FULL_STEPS",   "Motor steps per revolution",          200.0, float, "steps (1.8° = 200)"),
    ("MICROSTEPPING_AZM",  "AZM microstepping",                   16,    int,   "µsteps"),
    ("MICROSTEPPING_ALT",  "ALT microstepping",                   4,     int,   "µsteps"),
    ("GEAR_RATIO_AZM",     "AZM gear ratio (harmonic drive)",     100.0, float, ":1"),
    ("ALT_MOTOR_GEARBOX",  "ALT total gear ratio (UMOT × crank)", 496.0, float, ":1  (30:1→149, 100:1→496)"),
    ("ALT_SCREW_PITCH_MM", "ALT lead screw pitch",                2.0,   float, "mm/rev (T8 = 2)"),
    ("ALT_RADIUS_MM",      "ALT pivot-to-screw distance",         60.0,  float, "mm"),
    ("AXIS_REV_AZM",       "Reverse AZM direction",               True,  bool,  ""),
    ("AXIS_REV_ALT",       "Reverse ALT direction",               True,  bool,  ""),
    ("HOME_SAFETY_MARGIN", "Home pull-off margin",                 0.2,   float, "degrees"),
    ("RMS_CURRENT_AZM",    "AZM motor current",                   600,   int,   "mA"),
    ("RMS_CURRENT_ALT",    "ALT motor current",                   300,   int,   "mA  (≤400 for UMOT)"),
    ("AZM_LIMIT_NEG",      "AZM travel limit (negative)",        -30.0,  float, "degrees"),
    ("AZM_LIMIT_POS",      "AZM travel limit (positive)",         30.0,  float, "degrees"),
    ("ALT_LIMIT_NEG",      "ALT travel limit (negative)",          0.0,  float, "degrees"),
    ("ALT_LIMIT_POS",      "ALT travel limit (positive)",          5.0,  float, "degrees"),
    ("FEEDBACK_MIN_SCALE", "Feedback report minimum scale",        0.50,  float, "(0–1)"),
]


# ─────────────────────────────────────────────────────────────
# APPLICATION
# ─────────────────────────────────────────────────────────────
class App:

    POLL_MS = 500

    def __init__(self, root):
        self.root = root
        self.root.title("PolarAlign Controller")
        self.root.minsize(1150, 700)
        self.root.configure(bg="#f0f0f0")

        self.azm = 0.0
        self.alt = 0.0
        self.state = "—"
        self._polling = False

        self.serial = SerialManager(
            on_line=self._cb_line,
            on_status=self._cb_status,
            on_disconnect=self._cb_disconnect)

        self._build()
        self._refresh_ports()

    # ── BUILD ────────────────────────────────────────────────

    def _build(self):
        # — Top: Connection bar (full width) —
        cf = ttk.LabelFrame(self.root, text="Connection", padding=10)
        cf.pack(fill="x", padx=10, pady=(10, 5))

        ttk.Label(cf, text="Port:", font=("Helvetica", 12)).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(cf, textvariable=self.port_var,
                                     width=20, state="readonly",
                                     font=("Helvetica", 12))
        self.port_cb.pack(side="left", padx=6)
        ttk.Button(cf, text=" ⟳ ", command=self._refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(cf, text="  Connect  ",
                                    command=self._toggle_conn)
        self.conn_btn.pack(side="left", padx=10)
        self.conn_lbl = tk.Label(cf, text=" ● DISCONNECTED ",
                                  fg="gray", font=("Helvetica", 12, "bold"))
        self.conn_lbl.pack(side="left", padx=10)

        # — Top: Status bar (full width, dark) —
        sf = tk.Frame(self.root, bg="#1a1a2e", padx=16, pady=14)
        sf.pack(fill="x", padx=10, pady=5)

        self.st_lbl = tk.Label(sf, text="—", font=("Helvetica", 20, "bold"),
                                fg="#666", bg="#1a1a2e", width=6, anchor="w")
        self.st_lbl.pack(side="left", padx=(0, 20))
        self.azm_lbl = tk.Label(sf, text="AZM    0.000°    (   0.0')",
                                 font=(MONO, 18), fg="#00e5ff", bg="#1a1a2e")
        self.azm_lbl.pack(side="left", padx=(0, 30))
        self.alt_lbl = tk.Label(sf, text="ALT    0.000°    (   0.0')",
                                 font=(MONO, 18), fg="#ffab00", bg="#1a1a2e")
        self.alt_lbl.pack(side="left")

        # — Main area: PanedWindow (left=controls 2/3, right=log 1/3) —
        paned = tk.PanedWindow(self.root, orient="horizontal",
                                sashwidth=6, sashrelief="raised",
                                bg="#cccccc")
        paned.pack(fill="both", expand=True, padx=10, pady=(5, 10))

        # Left panel: tabs
        left = ttk.Frame(paned)
        paned.add(left, stretch="always")

        nb = ttk.Notebook(left)
        nb.pack(fill="both", expand=True)
        self._build_ctrl(nb)
        self._build_config(nb)

        # Right panel: serial log
        right = ttk.Frame(paned)
        paned.add(right, stretch="always")

        log_lf = ttk.LabelFrame(right, text="Serial Log", padding=6)
        log_lf.pack(fill="both", expand=True)

        self.log = scrolledtext.ScrolledText(log_lf, font=(MONO, 10),
                                              state="disabled", wrap="word",
                                              bg="#0d1117", fg="#c9d1d9")
        self.log.pack(fill="both", expand=True)

        log_btns = tk.Frame(log_lf)
        log_btns.pack(fill="x", pady=(4, 0))
        ttk.Button(log_btns, text="Clear", command=self._clear_log).pack(side="right")

        # Send raw command
        send_frame = tk.Frame(log_lf)
        send_frame.pack(fill="x", pady=(4, 0))
        ttk.Label(send_frame, text="Raw:", font=("Helvetica", 10)).pack(side="left")
        self.raw_entry = tk.Entry(send_frame, font=(MONO, 10))
        self.raw_entry.pack(side="left", fill="x", expand=True, padx=(4, 4))
        self.raw_entry.bind("<Return>", lambda _: self._send_raw())
        ttk.Button(send_frame, text="Send", command=self._send_raw).pack(side="right")

        # Set initial sash position (2/3 – 1/3) after window is mapped
        self.root.after(50, lambda: paned.sash_place(0,
            int(self.root.winfo_width() * 0.65), 0))

    def _build_ctrl(self, nb):
        tab = ttk.Frame(nb, padding=14)
        nb.add(tab, text="  ★ Control  ")

        # — AZM —
        af = ttk.LabelFrame(tab, text="  Azimuth (AZM)  ", padding=12)
        af.pack(fill="x", pady=(0, 10))
        row = tk.Frame(af)
        row.pack()
        for d in [-5.0, -1.0, -0.1, 0.1, 1.0, 5.0]:
            s = "+" if d > 0 else ""
            bg = "#c62828" if d < 0 else "#2e7d32"
            make_button(row, f" {s}{d}° ", bg=bg,
                        font=("Helvetica", 14, "bold"), pady=8,
                        command=lambda d=d: self._jog("AZM", d)
                        ).pack(side="left", padx=3)
        gr = tk.Frame(af)
        gr.pack(pady=(10, 0))
        tk.Label(gr, text="Go to (°):", font=("Helvetica", 12)).pack(side="left")
        self.azm_e = tk.Entry(gr, width=10, font=("Helvetica", 13))
        self.azm_e.pack(side="left", padx=6)
        ttk.Button(gr, text="  Go  ",
                   command=lambda: self._goto("AZM", self.azm_e)).pack(side="left")

        # — ALT —
        al = ttk.LabelFrame(tab, text="  Altitude (ALT)  ", padding=12)
        al.pack(fill="x", pady=(0, 10))
        row2 = tk.Frame(al)
        row2.pack()
        for d in [-5.0, -1.0, -0.1, 0.1, 1.0, 5.0]:
            s = "+" if d > 0 else ""
            bg = "#c62828" if d < 0 else "#2e7d32"
            make_button(row2, f" {s}{d}° ", bg=bg,
                        font=("Helvetica", 14, "bold"), pady=8,
                        command=lambda d=d: self._jog("ALT", d)
                        ).pack(side="left", padx=3)
        gr2 = tk.Frame(al)
        gr2.pack(pady=(10, 0))
        tk.Label(gr2, text="Go to (°):", font=("Helvetica", 12)).pack(side="left")
        self.alt_e = tk.Entry(gr2, width=10, font=("Helvetica", 13))
        self.alt_e.pack(side="left", padx=6)
        ttk.Button(gr2, text="  Go  ",
                   command=lambda: self._goto("ALT", self.alt_e)).pack(side="left")

        # — System —
        sl = ttk.LabelFrame(tab, text="  System Commands  ", padding=12)
        sl.pack(fill="x")
        sr = tk.Frame(sl)
        sr.pack()
        for cmd, txt, bg in [
            ("HOME",     " HOME \n Homing + Tare ",     "#1565c0"),
            ("DIAG",     " DIAG \n Full Diagnostic ",    "#e65100"),
            ("RST",      " RST \n Soft Reset ",          "#b71c1c"),
            ("AZM:ZERO", " AZM:ZERO \n Tare Azimuth ",  "#1b5e20"),
        ]:
            make_button(sr, txt, bg=bg, font=("Helvetica", 11, "bold"),
                        padx=14, pady=6,
                        command=lambda c=cmd: self._send(c)
                        ).pack(side="left", padx=5)

    def _build_config(self, nb):
        tab = ttk.Frame(nb, padding=14)
        nb.add(tab, text="  ⚙ Firmware Config  ")

        canvas = tk.Canvas(tab, highlightthickness=0)
        sb = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        sf = ttk.Frame(canvas)
        sf.bind("<Configure>", lambda _: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=sf, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        ttk.Label(sf, text="Edit values, then 'Generate Arduino Code' to copy-paste into the .ino",
                  foreground="gray", font=("Helvetica", 11)
                  ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))

        self.cfg = {}
        for i, (key, label, default, typ, hint) in enumerate(CONFIG_PARAMS, 1):
            ttk.Label(sf, text=label, font=("Helvetica", 11)).grid(
                row=i, column=0, sticky="w", padx=(0, 10), pady=3)
            if typ == bool:
                var = tk.BooleanVar(value=default)
                ttk.Checkbutton(sf, variable=var).grid(row=i, column=1, sticky="w")
                self.cfg[key] = ("bool", var, default)
            else:
                var = tk.StringVar(value=str(default))
                ttk.Entry(sf, textvariable=var, width=12,
                          font=("Helvetica", 11)).grid(row=i, column=1, sticky="w")
                self.cfg[key] = (typ.__name__, var, default)
            ttk.Label(sf, text=hint, foreground="gray",
                      font=("Helvetica", 10)).grid(
                row=i, column=2, sticky="w", padx=(10, 0))

        bf = ttk.Frame(sf)
        bf.grid(row=len(CONFIG_PARAMS)+2, column=0, columnspan=3, pady=16, sticky="w")
        ttk.Button(bf, text="  Generate Arduino Code  ",
                   command=self._gen_code).pack(side="left", padx=(0, 8))
        ttk.Button(bf, text="  Save (.json)  ",
                   command=self._save_cfg).pack(side="left", padx=(0, 8))
        ttk.Button(bf, text="  Load (.json)  ",
                   command=self._load_cfg).pack(side="left")

    # ── CALLBACKS ────────────────────────────────────────────

    def _cb_line(self, line):
        self.root.after(0, self._log, line)

    def _cb_status(self, st, x, y):
        self.root.after(0, self._upd_status, st, x, y)

    def _cb_disconnect(self):
        self.root.after(0, self._disconnect)

    # ── CONNECTION ───────────────────────────────────────────

    def _refresh_ports(self):
        ports = SerialManager.list_ports()
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[-1])

    def _toggle_conn(self):
        if self.serial.connected:
            self._disconnect()
        else:
            p = self.port_var.get()
            if not p:
                messagebox.showwarning("No Port", "Select a port first.")
                return
            if self.serial.connect(p):
                self.conn_btn.configure(text="  Disconnect  ")
                self.conn_lbl.configure(text=f" ● CONNECTED ({p}) ", fg="#2e7d32")
                self._polling = True
                self._poll()

    def _disconnect(self):
        self._polling = False
        self.serial.disconnect()
        self.conn_btn.configure(text="  Connect  ")
        self.conn_lbl.configure(text=" ● DISCONNECTED ", fg="gray")
        self.st_lbl.configure(text="—", fg="#666")

    def _poll(self):
        if self._polling and self.serial.connected:
            self.serial.poll()
            self.root.after(self.POLL_MS, self._poll)

    def _upd_status(self, st, x, y):
        self.azm, self.alt = x, y
        ad, ald = x / 60.0, y / 60.0
        colors = {"Idle": "#4CAF50", "Run": "#FF9800", "Hold": "#f44336"}
        self.st_lbl.configure(text=st, fg=colors.get(st, "#666"))
        self.azm_lbl.configure(text=f"AZM  {ad:+8.3f}°   ({x:+8.1f}')")
        self.alt_lbl.configure(text=f"ALT  {ald:+8.3f}°   ({y:+8.1f}')")

    # ── COMMANDS ─────────────────────────────────────────────

    def _jog(self, axis, delta):
        cur = (self.azm if axis == "AZM" else self.alt) / 60.0
        self._send(f"{axis}:{cur + delta:.4f}")

    def _goto(self, axis, entry):
        try: t = float(entry.get())
        except ValueError:
            messagebox.showwarning("Error", "Enter a valid number.")
            return
        self._send(f"{axis}:{t:.4f}")

    def _send(self, cmd):
        if not self.serial.connected:
            messagebox.showwarning("Not Connected", "Connect first.")
            return
        self._log(f">>> {cmd}")
        self.serial.send(cmd)

    def _send_raw(self):
        cmd = self.raw_entry.get().strip()
        if not cmd: return
        self._send(cmd)
        self.raw_entry.delete(0, "end")

    # ── LOG ──────────────────────────────────────────────────

    def _log(self, txt):
        self.log.configure(state="normal")
        self.log.insert("end", txt + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    # ── CONFIG ───────────────────────────────────────────────

    def _read_cfg(self):
        v = {}
        for key, _, default, _, _ in CONFIG_PARAMS:
            wt, var, dflt = self.cfg[key]
            if wt == "bool": v[key] = var.get()
            elif wt == "int":
                try: v[key] = int(var.get())
                except: v[key] = dflt
            else:
                try: v[key] = float(var.get())
                except: v[key] = dflt
        return v

    def _gen_code(self):
        v = self._read_cfg()
        lines = [
            "/* ───── HARDWARE SETTINGS ───── */",
            f"constexpr float MOTOR_FULL_STEPS = {v['MOTOR_FULL_STEPS']:.1f}f;",
            f"constexpr uint16_t MICROSTEPPING_AZM = {v['MICROSTEPPING_AZM']};",
            f"constexpr uint16_t MICROSTEPPING_ALT = {v['MICROSTEPPING_ALT']};",
            f"constexpr float GEAR_RATIO_AZM = {v['GEAR_RATIO_AZM']:.1f}f;",
            f"constexpr float ALT_MOTOR_GEARBOX = {v['ALT_MOTOR_GEARBOX']:.1f}f;",
            f"constexpr float ALT_SCREW_PITCH_MM = {v['ALT_SCREW_PITCH_MM']:.1f}f;",
            f"constexpr float ALT_RADIUS_MM = {v['ALT_RADIUS_MM']:.1f}f;",
            "",
            f"constexpr bool AXIS_REV_AZM = {'true' if v['AXIS_REV_AZM'] else 'false'};",
            f"constexpr bool AXIS_REV_ALT = {'true' if v['AXIS_REV_ALT'] else 'false'};",
            "",
            f"constexpr float HOME_SAFETY_MARGIN = {v['HOME_SAFETY_MARGIN']:.1f}f;",
            f"constexpr uint16_t RMS_CURRENT_AZM = {v['RMS_CURRENT_AZM']};",
            f"constexpr uint16_t RMS_CURRENT_ALT = {v['RMS_CURRENT_ALT']};",
            "",
            "/* ───── TRAVEL LIMITS (in degrees) ───── */",
            f"constexpr float AZM_LIMIT_NEG = {v['AZM_LIMIT_NEG']:.1f}f;",
            f"constexpr float AZM_LIMIT_POS = {v['AZM_LIMIT_POS']:+.1f}f;",
            f"constexpr float ALT_LIMIT_NEG = {v['ALT_LIMIT_NEG']:+.1f}f;",
            f"constexpr float ALT_LIMIT_POS = {v['ALT_LIMIT_POS']:+.1f}f;",
            "",
            "/* ───── FEEDBACK REPORT SCALING ───── */",
            f"constexpr float FEEDBACK_MIN_SCALE = {v['FEEDBACK_MIN_SCALE']:.2f}f;",
        ]
        code = "\n".join(lines)

        w = tk.Toplevel(self.root)
        w.title("Generated Arduino Code")
        w.geometry("660x500")
        t = scrolledtext.ScrolledText(w, font=(MONO, 11), wrap="none",
                                       bg="#0d1117", fg="#c9d1d9")
        t.pack(fill="both", expand=True, padx=10, pady=10)
        t.insert("1.0", code)
        t.configure(state="disabled")

        def cp():
            self.root.clipboard_clear()
            self.root.clipboard_append(code)
            messagebox.showinfo("Copied", "Copied to clipboard!", parent=w)

        bf = ttk.Frame(w)
        bf.pack(fill="x", padx=10, pady=(0, 10))
        ttk.Button(bf, text="  Copy to Clipboard  ", command=cp).pack(side="left")
        ttk.Button(bf, text="  Close  ", command=w.destroy).pack(side="right")

    def _save_cfg(self):
        v = self._read_cfg()
        p = filedialog.asksaveasfilename(defaultextension=".json",
                                          filetypes=[("JSON", "*.json")],
                                          initialfile="polaralign_config.json")
        if p:
            with open(p, "w") as f: json.dump(v, f, indent=2)
            messagebox.showinfo("Saved", f"Saved to:\n{p}")

    def _load_cfg(self):
        p = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not p: return
        try:
            with open(p) as f: v = json.load(f)
            for key, *_ in CONFIG_PARAMS:
                if key in v:
                    wt, var, _ = self.cfg[key]
                    var.set(bool(v[key]) if wt == "bool" else str(v[key]))
            messagebox.showinfo("Loaded", f"Loaded from:\n{p}")
        except Exception as e:
            messagebox.showerror("Error", f"Load failed:\n{e}")

    def close(self):
        self._polling = False
        self.serial.disconnect()
        self.root.destroy()


# ─────────────────────────────────────────────────────────────
def main():
    root = tk.Tk()
    try:
        if not IS_MAC: ttk.Style().theme_use("clam")
    except: pass
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()

if __name__ == "__main__":
    main()
