#!/usr/bin/env python3
"""
PolarAlign Controller v15.03g-V3 – Desktop GUI for the ESP32 Polar Alignment System
Cross-platform (Windows / macOS / Linux) — requires Python 3.8+ and pyserial.

Profiles: Proto V1 (commercial tilt plate) and V2 CNC (ALT V3 bielle geometry).
Profile is selected at startup and sets all hardware-specific defaults in the
Firmware Config tab. All other functionality is identical between profiles.

Changelog vs v15.03g-V2:
  - NEW: Jog buttons redesigned — 5 increments (0.001°/3.6" to 5°/300')
  - NEW: Arc labels (arcmin/arcsec) shown above each button column
  - NEW: Separate +/− rows for cleaner layout

Changelog vs v15.03g:
  - NEW: Profile selector dialog at startup (Proto V1 / V2 CNC)
  - FIX: ALT_LIMIT_NEG default corrected to -2.0° for V2 profile
  - FIX: AXIS_REV_ALT default corrected to False for V2 profile
  - FIX: ALT_MOTOR_GEARBOX (via TILT_CRANK_RATIO) updated for V2 geometry
  - FIX: ALT_LIMIT_POS default corrected to 5.0° for Proto, 10.0° for V2

Previous changelog (v15.03g):
  - FIX: Serial log no longer collapsed at startup (sash placement loop)
  - FIX: DTR/RTS disabled on connect (prevents ESP32 reboot when GUI connects)
  - FIX: AZM:LRN button removed (command doesn't exist in v15.03g firmware;
          AZM:ZERO already resets learning state internally)
  - NEW: ALT/AZM Learning panel — live display of MPU error and learned ratios,
          parsed from firmware serial output (ML Ratio: / MPU: / AZM ML: lines)

Install:  pip3 install pyserial
Run:      python3 PolarAlignGUI_v15_03g_V2.py
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
# HARDWARE PROFILES
# Keys must match CONFIG_PARAMS key names exactly.
# Only the values that differ between profiles are listed here —
# everything else falls back to the CONFIG_PARAMS default.
# ─────────────────────────────────────────────────────────────
PROFILES = {
    "Proto V1": {
        "TILT_CRANK_RATIO": 4.96,
        "AXIS_REV_ALT":     True,
        "ALT_LIMIT_NEG":    0.0,
        "ALT_LIMIT_POS":    5.0,
    },
    "V2 CNC": {
        "TILT_CRANK_RATIO": 6.94,
        "AXIS_REV_ALT":     False,
        "ALT_LIMIT_NEG":   -2.0,
        "ALT_LIMIT_POS":   10.0,
    },
}


# ─────────────────────────────────────────────────────────────
# PROFILE SELECTOR — shown once at startup, blocks until chosen
# ─────────────────────────────────────────────────────────────
def ask_profile(root):
    """
    Modal dialog that asks the user which hardware profile to load.
    Returns the profile name string (key in PROFILES).
    Destroys the app if the window is closed without choosing.
    """
    chosen = tk.StringVar(value="")

    dlg = tk.Toplevel(root)
    dlg.title("Select Hardware Profile")
    dlg.resizable(False, False)
    dlg.grab_set()          # modal
    dlg.protocol("WM_DELETE_WINDOW", root.destroy)

    tk.Label(dlg, text="Select your hardware profile:",
             font=("Helvetica", 13, "bold"), pady=12).pack(padx=30)

    btn_frame = tk.Frame(dlg)
    btn_frame.pack(padx=30, pady=(0, 20))

    profile_descs = {
        "Proto V1": "Commercial tilt plate\nUMOT 30:1 × 4.96 crank\nALT: 0° to +5°",
        "V2 CNC":   "V3 CNC bielle geometry\nUMOT 30:1 × 6.94 crank\nALT: −2° to +10°",
    }

    for name, desc in profile_descs.items():
        f = tk.Frame(btn_frame, bd=2, relief="ridge", padx=14, pady=10)
        f.pack(side="left", padx=10)
        tk.Label(f, text=name, font=("Helvetica", 13, "bold")).pack()
        tk.Label(f, text=desc, font=("Helvetica", 10), fg="#555",
                 justify="center").pack(pady=(4, 8))
        tk.Button(
            f, text=f"  Use {name}  ",
            font=("Helvetica", 11, "bold"),
            bg="#1565c0", fg="white", relief="raised",
            cursor="hand2",
            command=lambda n=name: (chosen.set(n), dlg.destroy())
        ).pack()

    root.wait_window(dlg)

    if not chosen.get():
        root.destroy()
        raise SystemExit(0)

    return chosen.get()


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

    def _enter(_):   lbl.configure(relief="groove")
    def _leave(_):   lbl.configure(relief="raised")
    def _press(_):   lbl.configure(relief="sunken")
    def _release(_):
        lbl.configure(relief="raised")
        if command: command()

    lbl.bind("<Enter>",          _enter)
    lbl.bind("<Leave>",          _leave)
    lbl.bind("<ButtonPress-1>",  _press)
    lbl.bind("<ButtonRelease-1>", _release)
    return lbl


# ─────────────────────────────────────────────────────────────
# SERIAL MANAGER
# ─────────────────────────────────────────────────────────────
class SerialManager:
    def __init__(self, on_line, on_status, on_mpu, on_disconnect):
        self.ser = None
        self.port = None
        self._running = False
        self._thread = None
        self._on_line = on_line
        self._on_status = on_status
        self._on_mpu = on_mpu
        self._on_disconnect = on_disconnect
        self._lock = threading.Lock()
        self._re = re.compile(
            r"<(?P<st>\w+)\|MPos:"
            r"(?P<x>[+-]?\d+\.?\d*),(?P<y>[+-]?\d+\.?\d*),(?P<z>[+-]?\d+\.?\d*)\|")
        self._re_mpu = re.compile(r"^MPU:([+-]?\d+\.?\d+),([+-]?\d+\.?\d+)$")

    @staticmethod
    def list_ports():
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port, baud=115200):
        self.disconnect()
        try:
            self.ser = serial.Serial(port, baud, timeout=0.3,
                                     dsrdtr=False,   # FIX: prevent ESP32 reboot on connect
                                     rtscts=False)
            self.ser.dtr = False                     # explicit — belt & suspenders
            self.ser.rts = False
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

    def poll_mpu(self):
        if not self.connected: return
        with self._lock:
            try: self.ser.write(b"MPU\n")
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
                    mm = self._re_mpu.match(line)
                    if mm:
                        self._on_mpu(float(mm.group(1)), float(mm.group(2)))
                    else:
                        self._on_line(line)
            except serial.SerialException:
                self._on_line("SERIAL ERROR — disconnected")
                self._on_disconnect()
                break
            except: continue


# ─────────────────────────────────────────────────────────────
# CONFIG DEFINITIONS
# These are the base defaults; the selected profile overrides
# the hardware-specific entries at runtime (see App.__init__).
# ─────────────────────────────────────────────────────────────
CONFIG_PARAMS = [
    ("MOTOR_FULL_STEPS",   "Motor steps per revolution",          200.0, float, "steps (1.8° = 200)"),
    ("MICROSTEPPING_AZM",  "AZM microstepping",                    16,   int,   "µsteps"),
    ("MICROSTEPPING_ALT",  "ALT microstepping",                     4,   int,   "µsteps"),
    ("GEAR_RATIO_AZM",     "AZM gear ratio (harmonic drive)",     100.0, float, ":1"),
    ("UMOT_RATIO",         "ALT motor gearbox (UMOT)",             30.0, float, ":1  (30 or 100)"),
    ("TILT_CRANK_RATIO",   "ALT tilt crank ratio",                 4.96, float, ":1  (Proto=4.96 / V2=6.94)"),
    ("ALT_SCREW_PITCH_MM", "ALT lead screw pitch",                  2.0, float, "mm/rev (T8 = 2)"),
    ("ALT_RADIUS_MM",      "ALT pivot-to-screw distance",          60.0, float, "mm"),
    ("AXIS_REV_AZM",       "Reverse AZM direction",               True,  bool,  ""),
    ("AXIS_REV_ALT",       "Reverse ALT direction",               True,  bool,  "Proto=True / V2=False"),
    ("HOME_SAFETY_MARGIN", "Home pull-off margin",                  0.2, float, "degrees"),
    ("RMS_CURRENT_AZM",    "AZM motor current",                   600,   int,   "mA"),
    ("RMS_CURRENT_ALT",    "ALT motor current",                   300,   int,   "mA  (≤400 for UMOT)"),
    ("AZM_LIMIT_NEG",      "AZM travel limit (negative)",        -30.0,  float, "degrees"),
    ("AZM_LIMIT_POS",      "AZM travel limit (positive)",         30.0,  float, "degrees"),
    ("ALT_LIMIT_NEG",      "ALT travel limit (negative)",          0.0,  float, "degrees  (Proto=0 / V2=−2)"),
    ("ALT_LIMIT_POS",      "ALT travel limit (positive)",         10.0,  float, "degrees  (Proto=5 / V2=10)"),
    ("FEEDBACK_MIN_SCALE", "Feedback report minimum scale",        0.50, float, "(0–1)"),
]

# Regex patterns for parsing firmware learning output
RE_ALT_RATIO = re.compile(r"ML Ratio:\s*([\d.]+)\s*\(was\s*([\d.]+)\)")
RE_ALT_MPU   = re.compile(r"MPU:\s*act=([\d.+-]+)\s+tgt=([\d.+-]+)\s+err=([\d.+-]+)")
RE_AZM_ML    = re.compile(r"AZM ML:\s*([\d.]+)→([\d.]+)\s*\(prev=([\d.]+)' curr=([\d.]+)'")


# ─────────────────────────────────────────────────────────────
# APPLICATION
# ─────────────────────────────────────────────────────────────
class App:

    POLL_MS = 500

    def __init__(self, root, profile_name):
        self.root = root
        self.profile_name = profile_name
        self.profile = PROFILES[profile_name]

        self.root.title(f"PolarAlign Controller v15.03g  —  {profile_name}")
        self.root.minsize(1200, 700)
        self.root.geometry("1400x800")
        self.root.configure(bg="#f0f0f0")

        self.azm = 0.0
        self.alt = 0.0
        self.state = "—"
        self._polling = False

        # Learning state — updated by parsing serial lines
        self._alt_ratio      = None
        self._alt_ratio_prev = None
        self._alt_mpu_act    = None
        self._alt_mpu_tgt    = None
        self._alt_mpu_err    = None
        self._azm_ratio      = None
        self._azm_ratio_prev = None

        self.serial = SerialManager(
            on_line=self._cb_line,
            on_status=self._cb_status,
            on_mpu=self._cb_mpu,
            on_disconnect=self._cb_disconnect)

        self._build()
        self._refresh_ports()

    # ── BUILD ────────────────────────────────────────────────

    def _build(self):
        # — Top: Connection bar —
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

        # Profile badge
        badge_bg = "#1565c0" if self.profile_name == "V2 CNC" else "#4a148c"
        tk.Label(cf, text=f"  {self.profile_name}  ",
                 font=("Helvetica", 11, "bold"),
                 bg=badge_bg, fg="white", relief="flat",
                 padx=8, pady=2).pack(side="right", padx=10)

        # — Status bar (AZM / ALT / MPU) —
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
        self.mpu_lbl = tk.Label(sf, text="MPU  —",
                                 font=(MONO, 14), fg="#aaaaaa", bg="#1a1a2e")
        self.mpu_lbl.pack(side="right", padx=(20, 0))

        # — Main area: PanedWindow (left 65% controls, right 35% log) —
        self._paned = tk.PanedWindow(self.root, orient="horizontal",
                                      sashwidth=6, sashrelief="raised",
                                      bg="#cccccc")
        self._paned.pack(fill="both", expand=True, padx=10, pady=(5, 10))

        left = ttk.Frame(self._paned)
        self._paned.add(left, stretch="always")

        nb = ttk.Notebook(left)
        nb.pack(fill="both", expand=True)
        self._build_ctrl(nb)
        self._build_config(nb)

        right = ttk.Frame(self._paned)
        self._paned.add(right, stretch="always")

        log_lf = ttk.LabelFrame(right, text="Serial Log", padding=6)
        log_lf.pack(fill="both", expand=True)

        self.log = scrolledtext.ScrolledText(log_lf, font=(MONO, 10),
                                              state="disabled", wrap="word",
                                              bg="#0d1117", fg="#c9d1d9")
        self.log.pack(fill="both", expand=True)

        log_btns = tk.Frame(log_lf)
        log_btns.pack(fill="x", pady=(4, 0))
        ttk.Button(log_btns, text="Clear", command=self._clear_log).pack(side="right")

        send_frame = tk.Frame(log_lf)
        send_frame.pack(fill="x", pady=(4, 0))
        ttk.Label(send_frame, text="Raw:", font=("Helvetica", 10)).pack(side="left")
        self.raw_entry = tk.Entry(send_frame, font=(MONO, 10))
        self.raw_entry.pack(side="left", fill="x", expand=True, padx=(4, 4))
        self.raw_entry.bind("<Return>", lambda _: self._send_raw())
        ttk.Button(send_frame, text="Send", command=self._send_raw).pack(side="right")

        # FIX: sash placement — two-pass with update_idletasks
        self.root.after(200, self._place_sash)

    def _place_sash(self):
        self.root.update_idletasks()
        w = self.root.winfo_width()
        if w < 400:
            self.root.after(100, self._place_sash)
            return
        pos = int(w * 0.75)
        self._paned.sash_place(0, pos, 0)
        self.root.after(150, lambda: self._paned.sash_place(0, int(self.root.winfo_width() * 0.75), 0))

    def _build_ctrl(self, nb):
        tab = ttk.Frame(nb, padding=14)
        nb.add(tab, text="  ★ Control  ")

        # — AZM —
        af = ttk.LabelFrame(tab, text="  Azimuth (AZM)  ", padding=10)
        af.pack(fill="x", pady=(0, 6))

        # Jog increments: (degrees, label_deg, label_arc)
        JOG_STEPS = [
            (0.001,  "0.001°",  '3.6"'),
            (0.01,   "0.01°",   '36"'),
            (0.1,    "0.1°",    "6'"),
            (1.0,    "1°",      "60'"),
            (5.0,    "5°",      "300'"),
        ]

        # Header row — arc labels
        hdr = tk.Frame(af)
        hdr.pack()
        tk.Label(hdr, text="", width=6).pack(side="left")  # spacer
        for _, ldeg, larc in JOG_STEPS:
            tk.Label(hdr, text=larc, font=("Helvetica", 9), fg="#aaaaaa",
                     width=7, anchor="center").pack(side="left", padx=2)

        # Negative row
        rn = tk.Frame(af)
        rn.pack(pady=(2, 1))
        tk.Label(rn, text="  −  ", font=("Helvetica", 10, "bold"),
                 fg="#ff5252", width=4).pack(side="left")
        for delta, ldeg, larc in JOG_STEPS:
            make_button(rn, f"−{ldeg}", bg="#c62828",
                        font=("Helvetica", 11, "bold"), pady=6, padx=4,
                        command=lambda d=-delta: self._jog("AZM", d)
                        ).pack(side="left", padx=2)

        # Positive row
        rp = tk.Frame(af)
        rp.pack(pady=(1, 6))
        tk.Label(rp, text="  +  ", font=("Helvetica", 10, "bold"),
                 fg="#69f0ae", width=4).pack(side="left")
        for delta, ldeg, larc in JOG_STEPS:
            make_button(rp, f"+{ldeg}", bg="#2e7d32",
                        font=("Helvetica", 11, "bold"), pady=6, padx=4,
                        command=lambda d=delta: self._jog("AZM", d)
                        ).pack(side="left", padx=2)

        gr = tk.Frame(af)
        gr.pack(pady=(2, 0))
        tk.Label(gr, text="Go to (°):", font=("Helvetica", 12)).pack(side="left")
        self.azm_e = tk.Entry(gr, width=10, font=("Helvetica", 13))
        self.azm_e.pack(side="left", padx=6)
        ttk.Button(gr, text="  Go  ",
                   command=lambda: self._goto("AZM", self.azm_e)).pack(side="left")

        # — ALT —
        alt_limits = (
            self.profile.get("ALT_LIMIT_NEG",
                             next(d for k,_,d,_,_ in CONFIG_PARAMS if k=="ALT_LIMIT_NEG")),
            self.profile.get("ALT_LIMIT_POS",
                             next(d for k,_,d,_,_ in CONFIG_PARAMS if k=="ALT_LIMIT_POS")),
        )
        al = ttk.LabelFrame(tab,
                             text=f"  Altitude (ALT)   [{alt_limits[0]:.0f}° to +{alt_limits[1]:.0f}°]  ",
                             padding=10)
        al.pack(fill="x", pady=(0, 6))

        # Header row — arc labels
        hdr2 = tk.Frame(al)
        hdr2.pack()
        tk.Label(hdr2, text="", width=6).pack(side="left")
        for _, ldeg, larc in JOG_STEPS:
            tk.Label(hdr2, text=larc, font=("Helvetica", 9), fg="#aaaaaa",
                     width=7, anchor="center").pack(side="left", padx=2)

        # Negative row
        rn2 = tk.Frame(al)
        rn2.pack(pady=(2, 1))
        tk.Label(rn2, text="  −  ", font=("Helvetica", 10, "bold"),
                 fg="#ff5252", width=4).pack(side="left")
        for delta, ldeg, larc in JOG_STEPS:
            make_button(rn2, f"−{ldeg}", bg="#c62828",
                        font=("Helvetica", 11, "bold"), pady=6, padx=4,
                        command=lambda d=-delta: self._jog("ALT", d)
                        ).pack(side="left", padx=2)

        # Positive row
        rp2 = tk.Frame(al)
        rp2.pack(pady=(1, 6))
        tk.Label(rp2, text="  +  ", font=("Helvetica", 10, "bold"),
                 fg="#69f0ae", width=4).pack(side="left")
        for delta, ldeg, larc in JOG_STEPS:
            make_button(rp2, f"+{ldeg}", bg="#2e7d32",
                        font=("Helvetica", 11, "bold"), pady=6, padx=4,
                        command=lambda d=delta: self._jog("ALT", d)
                        ).pack(side="left", padx=2)

        gr2 = tk.Frame(al)
        gr2.pack(pady=(2, 0))
        tk.Label(gr2, text="Go to (°):", font=("Helvetica", 12)).pack(side="left")
        self.alt_e = tk.Entry(gr2, width=10, font=("Helvetica", 13))
        self.alt_e.pack(side="left", padx=6)
        ttk.Button(gr2, text="  Go  ",
                   command=lambda: self._goto("ALT", self.alt_e)).pack(side="left")

        # — Learning Monitor —
        lm = ttk.LabelFrame(tab, text="  Learning Monitor  ", padding=10)
        lm.pack(fill="x", pady=(0, 6))

        lm_left  = tk.Frame(lm)
        lm_left.pack(side="left", expand=True, fill="both", padx=(0, 10))
        lm_right = tk.Frame(lm)
        lm_right.pack(side="left", expand=True, fill="both")

        # ALT column
        tk.Label(lm_left, text="ALT (MPU)", font=("Helvetica", 11, "bold"),
                 fg="#ffab00").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        tk.Label(lm_left, text="Learned ratio:", font=(MONO, 10)).grid(
            row=1, column=0, sticky="w")
        self._lbl_alt_ratio = tk.Label(lm_left, text="—", font=(MONO, 10),
                                        fg="#4CAF50", width=14, anchor="w")
        self._lbl_alt_ratio.grid(row=1, column=1, sticky="w", padx=(6, 0))
        tk.Label(lm_left, text="MPU error:", font=(MONO, 10)).grid(
            row=2, column=0, sticky="w")
        self._lbl_alt_err = tk.Label(lm_left, text="—", font=(MONO, 10),
                                      fg="#4CAF50", width=14, anchor="w")
        self._lbl_alt_err.grid(row=2, column=1, sticky="w", padx=(6, 0))
        tk.Label(lm_left, text="act / tgt:", font=(MONO, 10)).grid(
            row=3, column=0, sticky="w")
        self._lbl_alt_acttgt = tk.Label(lm_left, text="—", font=(MONO, 10),
                                         fg="#aaaaaa", width=18, anchor="w")
        self._lbl_alt_acttgt.grid(row=3, column=1, sticky="w", padx=(6, 0))

        # Vertical separator
        ttk.Separator(lm, orient="vertical").pack(side="left", fill="y", padx=8)

        # AZM column
        tk.Label(lm_right, text="AZM (residual)", font=("Helvetica", 11, "bold"),
                 fg="#00e5ff").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        tk.Label(lm_right, text="Learned ratio:", font=(MONO, 10)).grid(
            row=1, column=0, sticky="w")
        self._lbl_azm_ratio = tk.Label(lm_right, text="—", font=(MONO, 10),
                                        fg="#4CAF50", width=14, anchor="w")
        self._lbl_azm_ratio.grid(row=1, column=1, sticky="w", padx=(6, 0))
        tk.Label(lm_right, text="Last update:", font=(MONO, 10)).grid(
            row=2, column=0, sticky="w")
        self._lbl_azm_upd = tk.Label(lm_right, text="—", font=(MONO, 10),
                                      fg="#aaaaaa", width=18, anchor="w")
        self._lbl_azm_upd.grid(row=2, column=1, sticky="w", padx=(6, 0))

        # — System Commands —
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
        sf.bind("<Configure>", lambda _: canvas.configure(
            scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=sf, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        ttk.Label(sf,
            text=f"Profile: {self.profile_name}  —  edit values, then 'Generate Arduino Code'",
            foreground="#1565c0", font=("Helvetica", 11, "bold")
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 12))

        self.cfg = {}
        for i, (key, label, default, typ, hint) in enumerate(CONFIG_PARAMS, 1):
            # Apply profile override if present
            effective_default = self.profile.get(key, default)

            ttk.Label(sf, text=label, font=("Helvetica", 11)).grid(
                row=i, column=0, sticky="w", padx=(0, 10), pady=3)
            if typ == bool:
                var = tk.BooleanVar(value=effective_default)
                ttk.Checkbutton(sf, variable=var).grid(row=i, column=1, sticky="w")
                self.cfg[key] = ("bool", var, effective_default)
            else:
                var = tk.StringVar(value=str(effective_default))
                ttk.Entry(sf, textvariable=var, width=12,
                          font=("Helvetica", 11)).grid(row=i, column=1, sticky="w")
                self.cfg[key] = (typ.__name__, var, effective_default)
            ttk.Label(sf, text=hint, foreground="gray",
                      font=("Helvetica", 10)).grid(
                row=i, column=2, sticky="w", padx=(10, 0))

        bf = ttk.Frame(sf)
        bf.grid(row=len(CONFIG_PARAMS)+2, column=0, columnspan=3,
                pady=16, sticky="w")
        ttk.Button(bf, text="  Generate Arduino Code  ",
                   command=self._gen_code).pack(side="left", padx=(0, 8))
        ttk.Button(bf, text="  Save (.json)  ",
                   command=self._save_cfg).pack(side="left", padx=(0, 8))
        ttk.Button(bf, text="  Load (.json)  ",
                   command=self._load_cfg).pack(side="left")

    # ── CALLBACKS ────────────────────────────────────────────

    def _cb_line(self, line):
        self.root.after(0, self._process_line, line)

    def _cb_status(self, st, x, y):
        self.root.after(0, self._upd_status, st, x, y)

    def _cb_mpu(self, tared, raw):
        self.root.after(0, self._upd_mpu, tared, raw)

    def _cb_disconnect(self):
        self.root.after(0, self._disconnect)

    def _process_line(self, line):
        """Parse learning data from firmware serial output, then log the line."""

        # ALT: "ML Ratio: 62450.23 (was 62329.00)"
        m = RE_ALT_RATIO.search(line)
        if m:
            new_r, old_r = float(m.group(1)), float(m.group(2))
            delta = new_r - old_r
            sign = "+" if delta >= 0 else ""
            self._lbl_alt_ratio.configure(
                text=f"{new_r:.1f}  ({sign}{delta:.1f})",
                fg="#4CAF50" if abs(delta) < 50 else "#FF9800")

        # ALT: "MPU: act=2.341 tgt=2.500 err=0.159 (observe)"
        m = RE_ALT_MPU.search(line)
        if m:
            act, tgt, err = float(m.group(1)), float(m.group(2)), float(m.group(3))
            err_arcmin = err * 60.0
            self._lbl_alt_err.configure(
                text=f"{err_arcmin:+.2f}'",
                fg="#4CAF50" if abs(err_arcmin) < 1.0 else
                   "#FF9800" if abs(err_arcmin) < 3.0 else "#f44336")
            self._lbl_alt_acttgt.configure(
                text=f"{act:.3f}° / {tgt:.3f}°")

        # AZM: "AZM ML: 888.88→891.23 (prev=10.00' curr=2.50'"
        m = RE_AZM_ML.search(line)
        if m:
            old_r, new_r = float(m.group(1)), float(m.group(2))
            prev_arc, curr_arc = float(m.group(3)), float(m.group(4))
            delta = new_r - old_r
            sign = "+" if delta >= 0 else ""
            self._lbl_azm_ratio.configure(
                text=f"{new_r:.2f}  ({sign}{delta:.2f})",
                fg="#4CAF50" if abs(delta) < 5 else "#FF9800")
            self._lbl_azm_upd.configure(
                text=f"{prev_arc:.1f}'→{curr_arc:.1f}'")

        self._log(line)

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
                self.conn_lbl.configure(
                    text=f" ● CONNECTED ({p}) ", fg="#2e7d32")
                self._polling = True
                self._poll()
                self._poll_mpu()

    def _disconnect(self):
        self._polling = False
        self.serial.disconnect()
        self.conn_btn.configure(text="  Connect  ")
        self.conn_lbl.configure(text=" ● DISCONNECTED ", fg="gray")
        self.st_lbl.configure(text="—", fg="#666")
        self.mpu_lbl.configure(text="MPU  —", fg="#aaaaaa")

    def _poll(self):
        if self._polling and self.serial.connected:
            self.serial.poll()
            self.root.after(self.POLL_MS, self._poll)

    def _poll_mpu(self):
        if self._polling and self.serial.connected:
            self.serial.poll_mpu()
            self.root.after(2000, self._poll_mpu)

    def _upd_status(self, st, x, y):
        self.azm, self.alt = x, y
        ad, ald = x / 60.0, y / 60.0
        colors = {"Idle": "#4CAF50", "Run": "#FF9800", "Hold": "#f44336"}
        self.st_lbl.configure(text=st, fg=colors.get(st, "#666"))
        self.azm_lbl.configure(text=f"AZM  {ad:+8.3f}°   ({x:+8.1f}')")
        self.alt_lbl.configure(text=f"ALT  {ald:+8.3f}°   ({y:+8.1f}')")

    def _upd_mpu(self, tared, raw):
        self.mpu_lbl.configure(text=f"MPU  {tared:+.2f}°", fg="#66bb6a")

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
        alt_gearbox = v['UMOT_RATIO'] * v['TILT_CRANK_RATIO']
        lines = [
            f"/* ───── HARDWARE SETTINGS ({self.profile_name}) ───── */",
            f"constexpr float    MOTOR_FULL_STEPS   = {v['MOTOR_FULL_STEPS']:.1f}f;",
            f"constexpr uint16_t MICROSTEPPING_AZM  = {v['MICROSTEPPING_AZM']};",
            f"constexpr uint16_t MICROSTEPPING_ALT  = {v['MICROSTEPPING_ALT']};",
            f"constexpr float    GEAR_RATIO_AZM     = {v['GEAR_RATIO_AZM']:.1f}f;",
            f"constexpr float    ALT_MOTOR_GEARBOX  = {alt_gearbox:.1f}f;"
            f"       // UMOT {v['UMOT_RATIO']:.0f}:1 × {v['TILT_CRANK_RATIO']:.2f} crank",
            f"constexpr float    ALT_SCREW_PITCH_MM = {v['ALT_SCREW_PITCH_MM']:.1f}f;",
            f"constexpr float    ALT_RADIUS_MM      = {v['ALT_RADIUS_MM']:.1f}f;",
            "",
            f"constexpr bool     AXIS_REV_AZM       = {'true' if v['AXIS_REV_AZM'] else 'false'};",
            f"constexpr bool     AXIS_REV_ALT       = {'true' if v['AXIS_REV_ALT'] else 'false'};",
            "",
            f"constexpr float    HOME_SAFETY_MARGIN = {v['HOME_SAFETY_MARGIN']:.1f}f;",
            f"constexpr uint16_t RMS_CURRENT_AZM    = {v['RMS_CURRENT_AZM']};",
            f"constexpr uint16_t RMS_CURRENT_ALT    = {v['RMS_CURRENT_ALT']};",
            "",
            "/* ───── TRAVEL LIMITS (degrees) ───── */",
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
        w.title(f"Generated Arduino Code — {self.profile_name}")
        w.geometry("700x520")
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
        v["_profile"] = self.profile_name   # embed profile name in JSON
        p = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
            initialfile=f"polaralign_{self.profile_name.replace(' ', '_').lower()}.json")
        if p:
            with open(p, "w") as f: json.dump(v, f, indent=2)
            messagebox.showinfo("Saved", f"Saved to:\n{p}")

    def _load_cfg(self):
        p = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not p: return
        try:
            with open(p) as f: v = json.load(f)
            saved_profile = v.get("_profile", "")
            if saved_profile and saved_profile != self.profile_name:
                if not messagebox.askyesno(
                    "Profile mismatch",
                    f"Config was saved for '{saved_profile}' but current profile "
                    f"is '{self.profile_name}'.\nLoad anyway?"):
                    return
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
    root.withdraw()   # hide main window until profile is chosen
    try:
        if not IS_MAC: ttk.Style().theme_use("clam")
    except: pass

    profile_name = ask_profile(root)

    root.deiconify()  # show main window
    app = App(root, profile_name)
    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()


if __name__ == "__main__":
    main()