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
# LIGHT THEME PALETTE  (soft grays, modern)
# ─────────────────────────────────────────────────────────────
BG        = "#f0f2f5"   # root background
BG  = "#ffffff"   # panel/frame background
BORDER    = "#dee2e8"   # subtle border
TXT       = "#1e293b"   # primary text
TXT_DIM   = "#64748b"   # secondary text
CYAN      = "#0284c7"   # AZM accent (sky blue)
AMBER     = "#d97706"   # ALT accent (amber)
GREEN     = "#16a34a"   # positive / connected
RED_DIM   = "#dc2626"   # negative

# Button palette — medium saturation, readable on white bg
BTN_WEST  = "#dc2626"   # AZM West  (red)
BTN_EAST  = "#16a34a"   # AZM East  (green)
BTN_DOWN  = "#ea580c"   # ALT Down  (orange)
BTN_UP    = "#2563eb"   # ALT Up    (blue)

# Arc-unit increments used for jog buttons
# Each tuple: (degrees, primary_label, deg_equiv_label)
# Ordered LARGE → SMALL (left to right within each group)
ARCMIN_STEPS = [
    (30/60,  "30'",  "0.500°"),
    (10/60,  "10'",  "0.167°"),
    (5/60,   "5'",   "0.083°"),
    (1/60,   "1'",   "0.017°"),
]
ARCSEC_STEPS = [
    (30/3600, '30"', "0.0083°"),
    (10/3600, '10"', "0.0028°"),
    (5/3600,  '5"',  "0.0014°"),
    (1/3600,  '1"',  "0.00028°"),
]



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
        self.root.configure(bg=BG)

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
        cf = tk.LabelFrame(self.root, text="Connection", bg=BG, fg=TXT_DIM, font=("Helvetica", 10), padx=10, pady=6)
        cf.pack(fill="x", padx=10, pady=(10, 5))

        tk.Label(cf, text="Port:", font=("Helvetica", 13), bg=BG, fg=TXT).pack(side="left")
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
                                  fg="gray", bg=BG, font=("Helvetica", 12, "bold"))
        self.conn_lbl.pack(side="left", padx=10)

        # Profile badge
        badge_bg = "#1565c0" if self.profile_name == "V2 CNC" else "#4a148c"
        tk.Label(cf, text=f"  {self.profile_name}  ",
                 font=("Helvetica", 11, "bold"),
                 bg=badge_bg, fg="white", relief="flat",
                 padx=8, pady=2).pack(side="right", padx=10)

        # — Status bar (AZM / ALT / MPU) —
        sf = tk.Frame(self.root, bg=BORDER, padx=16, pady=12)
        sf.pack(fill="x", padx=10, pady=5)

        self.st_lbl = tk.Label(sf, text="—", font=("Helvetica", 20, "bold"),
                                fg="#666", bg=BG, width=6, anchor="w")
        self.st_lbl.pack(side="left", padx=(0, 20))
        self.azm_lbl = tk.Label(sf, text="AZM    0.000°    (   0.0')",
                                 font=(MONO, 18), fg="#00e5ff", bg=BG)
        self.azm_lbl.pack(side="left", padx=(0, 30))
        self.alt_lbl = tk.Label(sf, text="ALT    0.000°    (   0.0')",
                                 font=(MONO, 18), fg="#ffab00", bg=BG)
        self.alt_lbl.pack(side="left")
        self.mpu_lbl = tk.Label(sf, text="MPU  —",
                                 font=(MONO, 14), fg="#aaaaaa", bg=BG)
        self.mpu_lbl.pack(side="right", padx=(20, 0))

        # — Main area: PanedWindow (left 65% controls, right 35% log) —
        self._paned = tk.PanedWindow(self.root, orient="horizontal",
                                      sashwidth=6, sashrelief="raised",
                                      bg=BORDER)
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
                                              bg="#1a1a1a", fg="#c9d1d9")
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
        tab = tk.Frame(nb, bg=BG, padx=14, pady=12)
        tab.configure(bg=BG)
        nb.add(tab, text="  ★ Control  ")

        def jog(parent, label, sublabel, color, delta, axis):
            make_button(parent, f"{label}\n{sublabel}", bg=color, fg="white",
                        font=("Helvetica", 12, "bold"), padx=10, pady=7,
                        command=lambda: self._jog(axis, delta)
                        ).pack(side="left", padx=3, pady=4)

        # ── AZM ──────────────────────────────────────────────────────────
        azm_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        azm_outer.pack(fill="x", pady=(0, 8))
        tk.Label(azm_outer, text="  Azimuth (AZM)  —  mouvements RELATIFS",
                 bg=BG, fg=CYAN, font=("Helvetica", 11, "bold"),
                 anchor="w").pack(fill="x", padx=8, pady=(6,2))
        azm_lf = tk.Frame(azm_outer, bg=BG, padx=10, pady=6)
        azm_lf.pack(fill="x")

        dh = tk.Frame(azm_lf, bg=BG)
        dh.pack(fill="x")
        tk.Label(dh, text="← OUEST", bg=BG, fg=BTN_WEST,
                 font=("Helvetica", 12, "bold")).pack(side="left")
        tk.Label(dh, text="EST →",   bg=BG, fg=BTN_EAST,
                 font=("Helvetica", 12, "bold")).pack(side="right")

        azm_row = tk.Frame(azm_lf, bg=BG)
        azm_row.pack()
        # OUEST: −30' −10' −5' −1' | −30" −10" −5" −1"
        for deg, lbl, eq in ARCMIN_STEPS:
            jog(azm_row, f"−{lbl}", eq, BTN_WEST, -deg, "AZM")
        tk.Label(azm_row, text=" | ", bg=BG, fg=BORDER,
                 font=("Helvetica", 14)).pack(side="left")
        for deg, lbl, eq in ARCSEC_STEPS:
            jog(azm_row, f"−{lbl}", eq, BTN_WEST, -deg, "AZM")

        tk.Label(azm_row, text="  ⊕  ", bg=BG, fg=TXT_DIM,
                 font=("Helvetica", 16)).pack(side="left")

        # EST: +1" +5" +10" +30" | +1' +5' +10' +30'
        for deg, lbl, eq in reversed(ARCSEC_STEPS):
            jog(azm_row, f"+{lbl}", eq, BTN_EAST, deg, "AZM")
        tk.Label(azm_row, text=" | ", bg=BG, fg=BORDER,
                 font=("Helvetica", 14)).pack(side="left")
        for deg, lbl, eq in reversed(ARCMIN_STEPS):
            jog(azm_row, f"+{lbl}", eq, BTN_EAST, deg, "AZM")

        ga = tk.Frame(azm_lf, bg=BG)
        ga.pack(pady=(8, 0))
        tk.Label(ga, text="Aller à (°)  [ABSOLU]:", bg=BG,
                 fg=TXT_DIM, font=("Helvetica", 13)).pack(side="left")
        self.azm_e = tk.Entry(ga, width=10, font=("Helvetica", 12),
                               bg=BORDER, relief="flat")
        self.azm_e.pack(side="left", padx=6)
        make_button(ga, " Go ", bg="#556677", fg="white",
                    font=("Helvetica", 11, "bold"), padx=10, pady=4,
                    command=lambda: self._goto("AZM", self.azm_e)
                    ).pack(side="left")

        # ── ALT ──────────────────────────────────────────────────────────
        alt_limits = (
            self.profile.get("ALT_LIMIT_NEG",
                             next(d for k,_,d,_,_ in CONFIG_PARAMS if k=="ALT_LIMIT_NEG")),
            self.profile.get("ALT_LIMIT_POS",
                             next(d for k,_,d,_,_ in CONFIG_PARAMS if k=="ALT_LIMIT_POS")),
        )
        alt_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        alt_outer.pack(fill="x", pady=(0, 8))
        tk.Label(alt_outer,
                 text=f"  Altitude (ALT)  [{alt_limits[0]:.0f}° à +{alt_limits[1]:.0f}°]  —  mouvements RELATIFS",
                 bg=BG, fg=AMBER, font=("Helvetica", 11, "bold"),
                 anchor="w").pack(fill="x", padx=8, pady=(6,2))
        alt_lf = tk.Frame(alt_outer, bg=BG, padx=10, pady=6)
        alt_lf.pack(fill="x")

        # UP row
        tk.Label(alt_lf, text="▲  HAUT", bg=BG, fg=BTN_UP,
                 font=("Helvetica", 13, "bold")).pack(anchor="w")
        up_row = tk.Frame(alt_lf, bg=BG)
        up_row.pack(pady=(2, 4))
        for deg, lbl, eq in ARCMIN_STEPS:
            jog(up_row, f"+{lbl}", eq, BTN_UP, deg, "ALT")
        tk.Label(up_row, text=" | ", bg=BG, fg=BORDER,
                 font=("Helvetica", 14)).pack(side="left")
        for deg, lbl, eq in ARCSEC_STEPS:
            jog(up_row, f"+{lbl}", eq, BTN_UP, deg, "ALT")

        tk.Frame(alt_lf, bg=BORDER, height=1).pack(fill="x", padx=4, pady=2)

        # DOWN row — exact mirror
        dn_row = tk.Frame(alt_lf, bg=BG)
        dn_row.pack(pady=(4, 2))
        for deg, lbl, eq in ARCMIN_STEPS:
            jog(dn_row, f"−{lbl}", eq, BTN_DOWN, -deg, "ALT")
        tk.Label(dn_row, text=" | ", bg=BG, fg=BORDER,
                 font=("Helvetica", 14)).pack(side="left")
        for deg, lbl, eq in ARCSEC_STEPS:
            jog(dn_row, f"−{lbl}", eq, BTN_DOWN, -deg, "ALT")
        tk.Label(alt_lf, text="▼  BAS", bg=BG, fg=BTN_DOWN,
                 font=("Helvetica", 13, "bold")).pack(anchor="w")

        galt = tk.Frame(alt_lf, bg=BG)
        galt.pack(pady=(8, 0))
        tk.Label(galt, text="Aller à (°)  [ABSOLU]:", bg=BG,
                 fg=TXT_DIM, font=("Helvetica", 13)).pack(side="left")
        self.alt_e = tk.Entry(galt, width=10, font=("Helvetica", 12),
                               bg=BORDER, relief="flat")
        self.alt_e.pack(side="left", padx=6)
        make_button(galt, " Go ", bg="#556677", fg="white",
                    font=("Helvetica", 11, "bold"), padx=10, pady=4,
                    command=lambda: self._goto("ALT", self.alt_e)
                    ).pack(side="left")

                # — Learning Monitor —
        lm_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        lm_outer.pack(fill="x", pady=(0, 6))
        tk.Label(lm_outer, text="  Learning Monitor", bg=BG, fg=TXT_DIM,
                 font=("Helvetica", 10), anchor="w").pack(fill="x", padx=8, pady=(4,2))
        lm = tk.Frame(lm_outer, bg=BG, pady=4, padx=8)
        lm.pack(fill="x", pady=(0, 6))

        lm_left  = tk.Frame(lm, bg=BG)
        lm_left.pack(side="left", expand=True, fill="both", padx=(0, 10))
        lm_right = tk.Frame(lm, bg=BG)
        lm_right.pack(side="left", expand=True, fill="both")

        # ALT column
        tk.Label(lm_left, text="ALT (MPU)", bg=BG, font=("Helvetica", 11, "bold"),
                 fg="#ffab00").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        tk.Label(lm_left, text="Learned ratio:", bg=BG, fg=TXT_DIM, font=(MONO, 11)).grid(
            row=1, column=0, sticky="w")
        self._lbl_alt_ratio = tk.Label(lm_left, text="—", font=(MONO, 11),
                                        fg="#4CAF50", width=14, anchor="w")
        self._lbl_alt_ratio.grid(row=1, column=1, sticky="w", padx=(6, 0))
        tk.Label(lm_left, text="MPU error:", bg=BG, fg=TXT_DIM, font=(MONO, 11)).grid(
            row=2, column=0, sticky="w")
        self._lbl_alt_err = tk.Label(lm_left, text="—", font=(MONO, 10),
                                      fg="#4CAF50", width=14, anchor="w")
        self._lbl_alt_err.grid(row=2, column=1, sticky="w", padx=(6, 0))
        tk.Label(lm_left, text="act / tgt:", bg=BG, fg=TXT_DIM, font=(MONO, 11)).grid(
            row=3, column=0, sticky="w")
        self._lbl_alt_acttgt = tk.Label(lm_left, text="—", font=(MONO, 10),
                                         fg="#aaaaaa", width=18, anchor="w")
        self._lbl_alt_acttgt.grid(row=3, column=1, sticky="w", padx=(6, 0))

        # Vertical separator
        ttk.Separator(lm, orient="vertical").pack(side="left", fill="y", padx=8)

        # AZM column
        tk.Label(lm_right, text="AZM (residual)", bg=BG, font=("Helvetica", 11, "bold"),
                 fg="#00e5ff").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 4))
        tk.Label(lm_right, text="Learned ratio:", bg=BG, fg=TXT_DIM, font=(MONO, 11)).grid(
            row=1, column=0, sticky="w")
        self._lbl_azm_ratio = tk.Label(lm_right, text="—", font=(MONO, 11),
                                        fg="#4CAF50", width=14, anchor="w")
        self._lbl_azm_ratio.grid(row=1, column=1, sticky="w", padx=(6, 0))
        tk.Label(lm_right, text="Last update:", bg=BG, fg=TXT_DIM, font=(MONO, 11)).grid(
            row=2, column=0, sticky="w")
        self._lbl_azm_upd = tk.Label(lm_right, text="—", font=(MONO, 10),
                                      fg="#aaaaaa", width=18, anchor="w")
        self._lbl_azm_upd.grid(row=2, column=1, sticky="w", padx=(6, 0))

        # — System Commands —
        sl_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        sl_outer.pack(fill="x")
        tk.Label(sl_outer, text="  System Commands", bg=BG, fg=TXT_DIM,
                 font=("Helvetica", 10), anchor="w").pack(fill="x", padx=8, pady=(4,2))
        sl = tk.Frame(sl_outer, bg=BG, pady=8, padx=10)
        sl.pack(fill="x")
        sr = tk.Frame(sl, bg=BG)
        sr.pack()
        for cmd, txt, bg in [
            ("HOME",     " HOME \n Homing+Tare ",   "#1d4ed8"),
            ("DIAG",     " DIAG \n Diagnostic ",    "#b45309"),
            ("RST",      " RST \n Soft Reset ",     "#b91c1c"),
            ("AZM:ZERO", " AZM:ZERO \n Tare AZM ", "#15803d"),
        ]:
            make_button(sr, txt, bg=bg, font=("Helvetica", 12, "bold"),
                        padx=16, pady=8,
                        command=lambda c=cmd: self._send(c)
                        ).pack(side="left", padx=6)

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
                                       bg="#1a1a1a", fg="#c9d1d9")
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
    root.withdraw()
    try:
        if not IS_MAC: ttk.Style().theme_use("clam")
    except: pass

    profile_name = ask_profile(root)
    root.deiconify()

    try:
        app = App(root, profile_name)
        root.protocol("WM_DELETE_WINDOW", app.close)
        root.mainloop()
    except Exception as _e:
        import traceback
        msg = traceback.format_exc()
        print(msg)
        try:
            messagebox.showerror("Erreur au démarrage", msg)
        except:
            pass


if __name__ == "__main__":
    main()
