#!/usr/bin/env python3
"""
PolarAlign Controller v17 – Desktop GUI for the ESP32 Polar Alignment System
Cross-platform (Windows / macOS / Linux) — requires Python 3.8+ and pyserial.

Profiles: Proto V1 (commercial tilt plate) and V2 CNC (ALT V3 bielle geometry).
Profile is selected at startup and sets all hardware-specific defaults in the
Firmware Config tab. All other functionality is identical between profiles.

Changelog vs v15.03g-V4 (matches firmware v16.00):
  - REMOVED: AZM learning monitor (ratio / last / auto-blc) — firmware v16
             froze the AZM ratio at theoretical and made AZM backlash manual.
             Parsing of "AZM ML:", "AZM BLC ML:", "AZM ratio STABLE" removed.
  - NEW: AZM backlash panel in the status bar — shows the current firmware
         value (parsed from BLC? / BLC:AZM replies), with an arcmin entry and
         a Set button (sends BLC:AZM:<deg>) plus a refresh (sends BLC?).
  - NEW: BLC? sent automatically ~1 s after connect to populate the display.
  - FIX: Proto V1 profile ALT travel was 0..+5° — inherited from a pre-v15
         mechanical assumption. The firmware limits BOTH profiles to +10°
         (ALT_LIMIT_POS = 10.0f), so the GUI now declares 0..+10° for Proto V1
         (PROFILES dict + profile selector dialog).
  - FIX: V2 CNC profile TILT_CRANK_RATIO was 6.94, which made the Firmware
         Config tab generate ALT_MOTOR_GEARBOX = 30 x 6.94 = 208.2 while the
         firmware actually runs cfg_ALT_MOTOR_GEARBOX = 124.0 on that profile.
         Set to 4.13 (124.0 / 30) so the generated reference code matches the
         board. Display-only change: the generator does not drive the ESP32,
         which reads the value from its NVS profile.
  - NOTE: fully compatible with firmware v16.00 (status regex unchanged —
          the new closing '>' is outside the matched pattern; error:20 lines
          simply appear in the serial log).

Previous changelog (v15.03g-V4):
  - NEW: Last successful COM port persisted to ~/.polaralign_gui.json
  - v17: AUTO ALIGN module — closes the TPPA correction loop automatically.
         TPPA (manual mode, Polar Alignment System = None) measures and logs
         "Calculated Error" lines to the NINA log; this GUI tails that log,
         calibrates axis signs with one probe move per axis, then applies
         correction = -k * error per axis (capped), waits for the next clean
         solve, and repeats until TPPA's own tolerance gate finishes the
         alignment. Tolerance / cap / gain / log folder persisted in settings.
         (auto-selected at next launch when still present in available ports)
  - NEW: ±1° jog button added for large pre-alignment moves
  - REMOVED: ±5" and ±1" jog buttons (below TPPA deadband ~3")
  - CHANGED: All UI labels translated to English (WEST/EAST/UP/DOWN, etc.)
  - LAYOUT: Learning Monitor relocated to row 2 of the top status bar
            (System Commands now alone at the bottom, all 4 buttons visible)
  - COLOR : Per-axis color family — AZM = blue (dark West / light East),
            ALT = orange (dark Down / light Up). Mnemonic: dark = negative.
  - LAYOUT: Reduced paddings + smaller status font → compact fit on 1300x750
  - FIX:  System Commands buttons no longer cut off at default window height

Previous changelog (v15.03g-V3):
  - Jog buttons redesigned — 5 increments (0.001°/3.6" to 5°/300')
  - Arc labels (arcmin/arcsec) shown above each button column
  - Separate +/− rows for cleaner layout

Previous changelog (v15.03g-V2):
  - Profile selector dialog at startup (Proto V1 / V2 CNC)
  - ALT_LIMIT_NEG / ALT_LIMIT_POS / AXIS_REV_ALT / TILT_CRANK_RATIO per profile

Install:  pip3 install pyserial
Run:      python3 PolarAlignGUI_v16_00.py
"""

import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import threading
import re
import json
import time
import datetime
import os
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
# PERSISTED USER SETTINGS (last COM port)
# ─────────────────────────────────────────────────────────────
SETTINGS_FILE = os.path.join(os.path.expanduser("~"), ".polaralign_gui.json")


def load_settings():
    """Return a dict of user settings, or empty dict on any failure."""
    try:
        with open(SETTINGS_FILE, "r") as f:
            data = json.load(f)
            if isinstance(data, dict):
                return data
    except Exception:
        pass
    return {}


def save_settings(d):
    """Persist user settings (best-effort, silent failure)."""
    try:
        with open(SETTINGS_FILE, "w") as f:
            json.dump(d, f, indent=2)
    except Exception:
        pass


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

# Button palette — v15.03g-V4: per-axis color family
#   AZM = blue   (dark = West/negative, light = East/positive)
#   ALT = orange (dark = Down/negative, light = Up/positive)
BTN_WEST  = "#1e3a8a"   # AZM West (-) — dark navy
BTN_EAST  = "#3b82f6"   # AZM East (+) — medium-light blue
BTN_DOWN  = "#9a3412"   # ALT Down (-) — dark burnt orange
BTN_UP    = "#f97316"   # ALT Up   (+) — medium-light orange

# ─────────────────────────────────────────────────────────────
# JOG STEP DEFINITIONS — common for AZM and ALT
# Each tuple: (degrees, primary_label, deg_equiv_label)
# Ordered LARGE → SMALL (left to right when going positive)
# v15.03g-V4: ±1° added (pre-alignment), ±5" and ±1" removed (below TPPA deadband)
# ─────────────────────────────────────────────────────────────
JOG_DEG_STEPS = [
    (1.0,    "1°",   "60'"),
]
JOG_ARCMIN_STEPS = [
    (30/60,  "30'",  "0.500°"),
    (10/60,  "10'",  "0.167°"),
    (5/60,   "5'",   "0.083°"),
    (1/60,   "1'",   "0.017°"),
]
JOG_ARCSEC_STEPS = [
    (30/3600, '30"', "0.0083°"),
    (10/3600, '10"', "0.0028°"),
]


# ─────────────────────────────────────────────────────────────
# HARDWARE PROFILES
# Keys must match CONFIG_PARAMS key names exactly.
# Only the values that differ between profiles are listed here —
# everything else falls back to the CONFIG_PARAMS default.
# ─────────────────────────────────────────────────────────────
PROFILES = {
    "Proto V1": {
        "TILT_CRANK_RATIO":   4.96,
        "AXIS_REV_ALT":       True,
        "ALT_LIMIT_NEG":      0.0,
        "ALT_LIMIT_POS":     10.0,
        "BACKLASH_AZM_INIT":  0.033,
        "BACKLASH_ALT_INIT":  0.033,
    },
    "V2 CNC": {
        "TILT_CRANK_RATIO":   4.13,
        "AXIS_REV_ALT":       False,
        "ALT_LIMIT_NEG":     -2.0,
        "ALT_LIMIT_POS":     10.0,
        "BACKLASH_AZM_INIT":  0.033,
        "BACKLASH_ALT_INIT":  0.050,
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
        "Proto V1": "Commercial tilt plate\nUMOT 30:1 × 4.96 crank\nALT: 0° to +10°",
        "V2 CNC":   "V3 CNC bielle geometry\nUMOT 30:1 × 4.13 crank\nALT: −2° to +10°",
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
                line = raw.decode("utf-8", errors="replace").strip()
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
# NINA LOG WATCHER  (AUTO ALIGN measurement source)
# Parses TPPA lines like:
#   ...|INFO|PolarAlignment.cs|Execute|604|Calculated Error: Az: -00° 12' 35", Alt: 00° 55' 17", Tot: 00° 56' 42"
# Sign lives on the degree token ("-00" possible) → captured separately.
# ─────────────────────────────────────────────────────────────
GUI_VERSION = "17.0"

NINA_LOG_DIR_DEFAULT = os.path.join(
    os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "NINA", "Logs")

RE_NINA_ERR = re.compile(
    r"^(?P<ts>\S+?)\|.*Calculated Error: "
    r"Az: (?P<azs>-?)(?P<azd>\d+)\u00b0 (?P<azm>\d+)' (?P<azss>\d+)\", "
    r"Alt: (?P<als>-?)(?P<ald>\d+)\u00b0 (?P<alm>\d+)' (?P<alss>\d+)\", "
    r"Tot: (?P<tos>-?)(?P<tod>\d+)\u00b0 (?P<tom>\d+)' (?P<toss>\d+)\"")
RE_NINA_START  = re.compile(r"Starting polar alignment:")
RE_NINA_FINISH = re.compile(r"Automatically finishing polar alignment")


def _dms_to_arcmin(sign, d, m, s):
    v = int(d) * 60.0 + int(m) + int(s) / 60.0
    return -v if sign == "-" else v


class NinaLogWatcher:
    """Tails the newest NINA log file; emits parsed events to a callback.

    Events (via cb(kind, payload)):
      ("error",  {"t": datetime, "az": arcmin, "alt": arcmin, "tot": arcmin})
      ("start",  None)   new 3-point measurement begun (stale errors reset)
      ("finish", None)   TPPA auto-finished below tolerance
      ("file",   path)   watching a (new) file
    """

    RESCAN_S = 5.0

    def __init__(self, log_dir, cb):
        self.log_dir = log_dir
        self._cb = cb
        self._running = False
        self._thread = None

    def start(self):
        self.stop()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.5)
        self._thread = None

    def _newest(self):
        try:
            files = [os.path.join(self.log_dir, f)
                     for f in os.listdir(self.log_dir) if f.endswith(".log")]
            return max(files, key=os.path.getmtime) if files else None
        except Exception:
            return None

    def _loop(self):
        cur, fh, last_scan = None, None, 0.0
        while self._running:
            now = time.time()
            if now - last_scan > self.RESCAN_S:
                last_scan = now
                newest = self._newest()
                if newest and newest != cur:
                    if fh:
                        try: fh.close()
                        except Exception: pass
                    cur = newest
                    try:
                        fh = open(cur, "r", encoding="utf-8", errors="replace")
                        fh.seek(0, os.SEEK_END)   # tail from now — history is stale
                        self._cb("file", cur)
                    except Exception:
                        fh = None
            if not fh:
                time.sleep(1.0); continue
            line = fh.readline()
            if not line:
                time.sleep(0.3); continue
            line = line.strip()
            m = RE_NINA_ERR.search(line)
            if m:
                try:
                    ts = datetime.datetime.fromisoformat(m.group("ts"))
                except ValueError:
                    ts = datetime.datetime.now()
                self._cb("error", {
                    "t":   ts,
                    "az":  _dms_to_arcmin(m.group("azs"), m.group("azd"), m.group("azm"), m.group("azss")),
                    "alt": _dms_to_arcmin(m.group("als"), m.group("ald"), m.group("alm"), m.group("alss")),
                    "tot": _dms_to_arcmin(m.group("tos"), m.group("tod"), m.group("tom"), m.group("toss")),
                })
            elif RE_NINA_START.search(line):
                self._cb("start", None)
            elif RE_NINA_FINISH.search(line):
                self._cb("finish", None)


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
    ("TILT_CRANK_RATIO",   "ALT tilt crank ratio",                 4.96, float, ":1  (Proto=4.96 / V2=4.13)"),
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
    ("ALT_LIMIT_POS",      "ALT travel limit (positive)",         10.0,  float, "degrees  (Proto=10 / V2=10)"),
    ("FEEDBACK_MIN_SCALE", "Feedback report minimum scale",        0.50, float, "(0–1)"),
    ("BACKLASH_AZM_INIT",  "AZM backlash initial value",          0.033, float, "degrees  (~2' typical)"),
    ("BACKLASH_ALT_INIT",  "ALT backlash initial value",          0.033, float, "degrees  (Proto=0.033 / V2=0.050)"),
]

# Regex patterns for parsing firmware learning output
RE_ALT_RATIO = re.compile(r"ML Ratio:\s*([\d.]+)\s*\(was\s*([\d.]+)\)")
RE_ALT_MPU   = re.compile(r"MPU:\s*act=([\d.+-]+)\s+tgt=([\d.+-]+)\s+err=([\d.+-]+)")
# ALT backlash learning (kept in firmware v16)
RE_ALT_BLC   = re.compile(r"ALT BLC ML:.*?([\d.]+)→([\d.]+)\s*\(n=(\d+)\)")
# v16 — AZM backlash is MANUAL: parse the firmware's BLC replies instead
#   "BLC:AZM=0.0330 ALT=0.0500 (1.98'/3.00')  AZM=manual  ALT samples=4"  (BLC?)
#   "BLC:AZM=0.0330 (1.98')"                                              (BLC:AZM?)
#   "BLC:AZM set to 0.0330 (1.98') [saved]"                               (BLC:AZM:x)
RE_BLC_QUERY   = re.compile(r"BLC:AZM=([\d.]+)\s+ALT=([\d.]+)")
RE_BLC_AZM     = re.compile(r"BLC:AZM=([\d.]+)\s+\(")
RE_BLC_AZM_SET = re.compile(r"BLC:AZM set to ([\d.]+)")
RE_BLC_ALT_SET = re.compile(r"BLC:ALT set to ([\d.]+)")


# ─────────────────────────────────────────────────────────────
# APPLICATION
# ─────────────────────────────────────────────────────────────
class App:

    POLL_MS = 500

    def __init__(self, root, profile_name):
        self.root = root
        self.profile_name = profile_name
        self.profile = PROFILES[profile_name]
        self.settings = load_settings()   # persisted user settings (last port)
        # ── AUTO ALIGN state ──
        self.cur_status = "?"
        self.aa_watcher = None
        self.aa_events  = []            # queue filled from watcher thread
        self.aa_lock    = threading.Lock()
        self.aa_state   = "IDLE"        # IDLE/ARMED/CAL_AZM/CAL_ALT/CORRECT/WAIT/DONE
        self.aa_last    = None          # last accepted error dict
        self.aa_ref     = None          # error before current probe/correction
        self.aa_sign    = {"AZM": None, "ALT": None}
        self.aa_pending = None          # ("AZM"|"ALT", move_arcmin) awaiting completion
        self.aa_purge   = 0             # solves to discard after a move
        self.aa_ncorr   = 0
        self.aa_t_last  = 0.0           # wall time of last event (staleness watch)
        self.aa_watch   = None          # dict axis -> |err| before last correction
        self.aa_flipped = {}            # divergence guard: one sign flip allowed
        self.aa_seq     = []            # chained second-axis move of a cycle
        self.aa_t_idle  = None          # wall time when last move finished
        self.aa_dyncap  = {}            # per-axis dynamic cap from efficiency guard
        self.aa_lastdir = {}            # per-axis sign of last move (backlash watch)

        self.root.title(f"PolarAlign Controller v{GUI_VERSION}  —  {profile_name}  (FW v16.x)")
        # v15.03g-V4: compacted layout — smaller default + minsize so the
        # System Commands buttons remain visible at startup on 1080p screens.
        self.root.minsize(1000, 540)
        self.root.geometry("1080x560")
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
        # v16: AZM ratio is fixed in firmware — no AZM learning state

        self.serial = SerialManager(
            on_line=self._cb_line,
            on_status=self._cb_status,
            on_mpu=self._cb_mpu,
            on_disconnect=self._cb_disconnect)

        self._build()
        self._refresh_ports()

    # ── BUILD ────────────────────────────────────────────────

    def _build(self):
        # — Top row: Connection (left) + System Commands (right) —
        # v17 compact: system buttons promoted from the Control tab to here.
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill="x", padx=10, pady=(8, 4))

        cf = tk.LabelFrame(top, text="Connection", bg=BG, fg=TXT_DIM,
                            font=("Helvetica", 10), padx=10, pady=3)
        cf.pack(side="left", fill="both", expand=True)

        tk.Label(cf, text="Port:", font=("Helvetica", 12), bg=BG, fg=TXT).pack(side="left")
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(cf, textvariable=self.port_var,
                                     width=14, state="readonly",
                                     font=("Helvetica", 11))
        self.port_cb.pack(side="left", padx=6)
        ttk.Button(cf, text=" ⟳ ", command=self._refresh_ports).pack(side="left")
        self.conn_btn = ttk.Button(cf, text="  Connect  ",
                                    command=self._toggle_conn)
        self.conn_btn.pack(side="left", padx=8)
        self.conn_lbl = tk.Label(cf, text=" ● DISCONNECTED ",
                                  fg="gray", bg=BG, font=("Helvetica", 11, "bold"))
        self.conn_lbl.pack(side="left", padx=6)

        sysf = tk.LabelFrame(top, text="System Commands", bg=BG, fg=TXT_DIM,
                             font=("Helvetica", 10), padx=8, pady=3)
        sysf.pack(side="right", padx=(8, 0))
        for cmd, txt, bg in [
            ("HOME",     "HOME",     "#1d4ed8"),
            ("DIAG",     "DIAG",     "#b45309"),
            ("RST",      "RST",      "#b91c1c"),
            ("AZM:ZERO", "AZM:ZERO", "#15803d"),
        ]:
            make_button(sysf, f" {txt} ", bg=bg, fg="white",
                        font=("Helvetica", 10, "bold"), padx=8, pady=3,
                        command=lambda c=cmd: self._send(c)
                        ).pack(side="left", padx=3)

        # — Status bar (2 rows: positions on top, learning info below) —
        # v15.03g-V4: learning monitor labels promoted to top status bar
        sfrow = tk.Frame(self.root, bg=BG)
        sfrow.pack(fill="x", padx=10, pady=4)
        sf = tk.Frame(sfrow, bg=BORDER, padx=12, pady=3)
        sf.pack(side="left", fill="both", expand=True)
        badge_bg = "#1565c0" if self.profile_name == "V2 CNC" else "#4a148c"
        badge = tk.Frame(sfrow, bg=BORDER, padx=4, pady=3)
        badge.pack(side="right", fill="y", padx=(8, 0))
        tk.Label(badge, text=f"  {self.profile_name}  ",
                 font=("Helvetica", 12, "bold"),
                 bg=badge_bg, fg="white", relief="flat",
                 padx=10, pady=4).pack(expand=True)

        # Row 1: state | AZM pos | ALT pos                                MPU
        row1 = tk.Frame(sf, bg=BG)
        row1.pack(fill="x")
        self.st_lbl = tk.Label(row1, text="—", font=("Helvetica", 14, "bold"),
                                fg="#666", bg=BG, width=6, anchor="w")
        self.st_lbl.pack(side="left", padx=(0, 16))
        self.azm_lbl = tk.Label(row1, text="AZM    0.000°    (   0.0')",
                                 font=(MONO, 13), fg=CYAN, bg=BG)
        self.azm_lbl.pack(side="left", padx=(0, 24))
        self.alt_lbl = tk.Label(row1, text="ALT    0.000°    (   0.0')",
                                 font=(MONO, 13), fg=AMBER, bg=BG)
        self.alt_lbl.pack(side="left")
        self.mpu_lbl = tk.Label(row1, text="MPU  —",
                                 font=(MONO, 11), fg="#aaaaaa", bg=BG)
        self.mpu_lbl.pack(side="right", padx=(16, 0))

        # v17: row 2 (AZM blc manual panel + ALT learning monitor) removed from
        # the UI per user request — MPU readout lives on row 1. Ghost widgets
        # below keep the _process_line update callbacks harmless; AZM backlash
        # can still be set via the Raw field (BLC:AZM:x.xx).
        _ghost = tk.Frame(sf, bg=BG)   # never packed
        self._lbl_azm_blc    = tk.Label(_ghost)
        self._azm_blc_e      = tk.Entry(_ghost)
        self._lbl_alt_ratio  = tk.Label(_ghost)
        self._lbl_alt_err    = tk.Label(_ghost)
        self._lbl_alt_acttgt = tk.Label(_ghost)
        self._lbl_alt_blc    = tk.Label(_ghost)

        # — Main area: PanedWindow (left ~75% controls, right ~25% log) —
        self._paned = tk.PanedWindow(self.root, orient="horizontal",
                                      sashwidth=6, sashrelief="raised",
                                      bg=BORDER)
        self._paned.pack(fill="both", expand=True, padx=10, pady=(4, 8))

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

        # v17: bottom bars packed FIRST (side=bottom) so Clear/Raw/Send always
        # stay visible; the log text then absorbs whatever height remains —
        # the column height follows the left-side content, never the opposite.
        send_frame = tk.Frame(log_lf)
        send_frame.pack(side="bottom", fill="x", pady=(4, 0))
        log_btns = tk.Frame(log_lf)
        log_btns.pack(side="bottom", fill="x", pady=(4, 0))
        ttk.Button(log_btns, text="Clear", command=self._clear_log).pack(side="right")

        self.log = scrolledtext.ScrolledText(log_lf, font=(MONO, 10), height=8,
                                              state="disabled", wrap="word",
                                              bg="#1a1a1a", fg="#c9d1d9")
        self.log.pack(fill="both", expand=True)

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
        pos = int(w * 0.72)
        self._paned.sash_place(0, pos, 0)
        self.root.after(150, lambda: self._paned.sash_place(0, int(self.root.winfo_width() * 0.72), 0))

    def _build_ctrl(self, nb):
        # v17 compact layout — everything visible in a 1300x680 window so the
        # GUI and NINA can share the screen:
        #   * WEST/EAST and UP/DOWN labels merged into the jog button rows
        #   * Go-to fields moved onto the section header lines
        #   * "System Commands" title removed (buttons only)
        tab = tk.Frame(nb, bg=BG, padx=10, pady=5)
        tab.configure(bg=BG)
        nb.add(tab, text="  ★ Control  ")

        def jog(parent, label, sublabel, color, delta, axis):
            make_button(parent, f"{label}\n{sublabel}", bg=color, fg="white",
                        font=("Helvetica", 10, "bold"), padx=3, pady=2,
                        command=lambda: self._jog(axis, delta)
                        ).pack(side="left", padx=1, pady=1)

        def sep(parent):
            tk.Label(parent, text="|", bg=BG, fg=BORDER,
                     font=("Helvetica", 11)).pack(side="left", padx=1)

        def goto_inline(parent, axis):
            g = tk.Frame(parent, bg=BG)
            g.pack(side="right", padx=(0, 2))
            tk.Label(g, text="Go to (°) [ABS]:", bg=BG, fg=TXT_DIM,
                     font=("Helvetica", 10)).pack(side="left")
            e = tk.Entry(g, width=8, font=("Helvetica", 11),
                         bg=BORDER, relief="flat")
            e.pack(side="left", padx=4)
            make_button(g, " Go ", bg="#556677", fg="white",
                        font=("Helvetica", 10, "bold"), padx=8, pady=2,
                        command=lambda: self._goto(axis, e)).pack(side="left")
            return e

        # ── AZM ──────────────────────────────────────────────────────────
        azm_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        azm_outer.pack(fill="x", pady=(0, 4))
        azm_hdr = tk.Frame(azm_outer, bg=BG)
        azm_hdr.pack(fill="x", padx=8, pady=(3, 0))
        tk.Label(azm_hdr, text="Azimuth (AZM)  —  RELATIVE moves",
                 bg=BG, fg=CYAN, font=("Helvetica", 11, "bold")
                 ).pack(side="left")
        self.azm_e = goto_inline(azm_hdr, "AZM")

        # v17: staggered AZM rows to cut panel WIDTH — WEST row left-aligned,
        # EAST row one line below, right-aligned; they overlap in vertical
        # projection so the panel no longer needs the full 14-button width.
        west_row = tk.Frame(azm_outer, bg=BG)
        west_row.pack(fill="x", padx=6, pady=(1, 0))
        wst = tk.Frame(west_row, bg=BG)
        wst.pack(side="left")
        tk.Label(wst, text="←WEST", bg=BG, fg=BTN_WEST,
                 font=("Helvetica", 10, "bold")).pack(side="left", padx=(2, 2))
        for deg, lbl, eq in JOG_DEG_STEPS:
            jog(wst, f"−{lbl}", eq, BTN_WEST, -deg, "AZM")
        sep(wst)
        for deg, lbl, eq in JOG_ARCMIN_STEPS:
            jog(wst, f"−{lbl}", eq, BTN_WEST, -deg, "AZM")
        sep(wst)
        for deg, lbl, eq in JOG_ARCSEC_STEPS:
            jog(wst, f"−{lbl}", eq, BTN_WEST, -deg, "AZM")

        east_row = tk.Frame(azm_outer, bg=BG)
        east_row.pack(fill="x", padx=6, pady=(0, 4))
        est = tk.Frame(east_row, bg=BG)
        est.pack(side="right")
        for deg, lbl, eq in reversed(JOG_ARCSEC_STEPS):
            jog(est, f"+{lbl}", eq, BTN_EAST, deg, "AZM")
        sep(est)
        for deg, lbl, eq in reversed(JOG_ARCMIN_STEPS):
            jog(est, f"+{lbl}", eq, BTN_EAST, deg, "AZM")
        sep(est)
        for deg, lbl, eq in reversed(JOG_DEG_STEPS):
            jog(est, f"+{lbl}", eq, BTN_EAST, deg, "AZM")
        tk.Label(est, text="EAST→", bg=BG, fg=BTN_EAST,
                 font=("Helvetica", 10, "bold")).pack(side="left", padx=(2, 2))

        # ── ALT ──────────────────────────────────────────────────────────
        alt_limits = (
            self.profile.get("ALT_LIMIT_NEG",
                             next(d for k,_,d,_,_ in CONFIG_PARAMS if k=="ALT_LIMIT_NEG")),
            self.profile.get("ALT_LIMIT_POS",
                             next(d for k,_,d,_,_ in CONFIG_PARAMS if k=="ALT_LIMIT_POS")),
        )
        alt_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        alt_outer.pack(fill="x", pady=(0, 4))
        alt_hdr = tk.Frame(alt_outer, bg=BG)
        alt_hdr.pack(fill="x", padx=8, pady=(3, 0))
        tk.Label(alt_hdr,
                 text=f"Altitude (ALT)  [{alt_limits[0]:.0f}° to +{alt_limits[1]:.0f}°]  —  RELATIVE moves",
                 bg=BG, fg=AMBER, font=("Helvetica", 11, "bold")
                 ).pack(side="left")
        self.alt_e = goto_inline(alt_hdr, "ALT")

        alt_lf = tk.Frame(alt_outer, bg=BG, padx=10, pady=2)
        alt_lf.pack(fill="x")

        up_row = tk.Frame(alt_lf, bg=BG)
        up_row.pack(anchor="center", pady=(0, 1))
        tk.Label(up_row, text="▲ UP     ", bg=BG, fg=BTN_UP,
                 font=("Helvetica", 11, "bold")).pack(side="left", padx=(4, 3))
        for deg, lbl, eq in JOG_DEG_STEPS:
            jog(up_row, f"+{lbl}", eq, BTN_UP, deg, "ALT")
        sep(up_row)
        for deg, lbl, eq in JOG_ARCMIN_STEPS:
            jog(up_row, f"+{lbl}", eq, BTN_UP, deg, "ALT")
        sep(up_row)
        for deg, lbl, eq in JOG_ARCSEC_STEPS:
            jog(up_row, f"+{lbl}", eq, BTN_UP, deg, "ALT")

        dn_row = tk.Frame(alt_lf, bg=BG)
        dn_row.pack(anchor="center", pady=(1, 2))
        tk.Label(dn_row, text="▼ DOWN", bg=BG, fg=BTN_DOWN,
                 font=("Helvetica", 11, "bold")).pack(side="left", padx=(4, 3))
        for deg, lbl, eq in JOG_DEG_STEPS:
            jog(dn_row, f"−{lbl}", eq, BTN_DOWN, -deg, "ALT")
        sep(dn_row)
        for deg, lbl, eq in JOG_ARCMIN_STEPS:
            jog(dn_row, f"−{lbl}", eq, BTN_DOWN, -deg, "ALT")
        sep(dn_row)
        for deg, lbl, eq in JOG_ARCSEC_STEPS:
            jog(dn_row, f"−{lbl}", eq, BTN_DOWN, -deg, "ALT")

        # ── AUTO ALIGN (TPPA log loop) ───────────────────────
        aa_outer = tk.Frame(tab, bg=BG, highlightbackground=BORDER, highlightthickness=1)
        aa_outer.pack(fill="x", pady=(0, 0))
        tk.Label(aa_outer, text="  AUTO ALIGN  —  closes the TPPA correction loop "
                 "(TPPA manual mode, System=None)", bg=BG, fg="#7c3aed",
                 font=("Helvetica", 11, "bold"), anchor="w"
                 ).pack(fill="x", padx=8, pady=(3, 0))
        aa = tk.Frame(aa_outer, bg=BG, padx=10, pady=3)
        aa.pack(fill="x")

        r1 = tk.Frame(aa, bg=BG); r1.pack(fill="x", pady=(0, 2))
        tk.Label(r1, text="NINA log folder:", bg=BG, fg=TXT_DIM,
                 font=("Helvetica", 10)).pack(side="left")
        self.aa_dir_e = tk.Entry(r1, font=("Helvetica", 9), bg=BORDER, relief="flat")
        self.aa_dir_e.pack(side="left", fill="x", expand=True, padx=6)
        self.aa_dir_e.insert(0, self.settings.get("nina_log_dir", NINA_LOG_DIR_DEFAULT))
        self.aa_dir_e.bind("<FocusOut>", lambda _e: self._aa_save_fields())
        make_button(r1, "\u21ba", bg="#556677", fg="white",
                    font=("Helvetica", 10, "bold"), padx=6, pady=1,
                    command=self._aa_reset_dir).pack(side="left")

        r2 = tk.Frame(aa, bg=BG); r2.pack(fill="x", pady=(0, 2))
        def num_field(parent, label, key, default, width=6):
            tk.Label(parent, text=label, bg=BG, fg=TXT_DIM,
                     font=("Helvetica", 10)).pack(side="left", padx=(0, 2))
            e = tk.Entry(parent, width=width, font=("Helvetica", 10),
                         bg=BORDER, relief="flat")
            e.insert(0, str(self.settings.get(key, default)))
            e.pack(side="left", padx=(0, 10))
            e.bind("<FocusOut>", lambda _e: self._aa_save_fields())
            return e
        self.aa_tol_e = num_field(r2, "Tolerance ('):", "aa_tolerance", 0.5)
        self.aa_cap_e = num_field(r2, "Cap ('):",       "aa_cap",       10.0)
        self.aa_k_e   = num_field(r2, "Gain k:",        "aa_gain",      0.95)
        self.aa_start_b = make_button(r2, "  START  ", bg="#15803d", fg="white",
                                      font=("Helvetica", 11, "bold"), padx=10, pady=2,
                                      command=self._aa_start)
        self.aa_start_b.pack(side="left", padx=(10, 4))
        self.aa_stop_b = make_button(r2, "  STOP  ", bg="#b91c1c", fg="white",
                                     font=("Helvetica", 11, "bold"), padx=10, pady=2,
                                     command=self._aa_stop)
        self.aa_stop_b.pack(side="left")
        self.aa_state_lbl = tk.Label(r2, text="AUTO: idle", bg=BG, fg=TXT_DIM,
                                     font=(MONO, 10), anchor="w")
        self.aa_state_lbl.pack(side="left", padx=(14, 0))

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

        # ALT backlash learning: "ALT BLC ML: ... 0.0500→0.0510 (n=3)"
        m = RE_ALT_BLC.search(line)
        if m:
            new_blc = float(m.group(2))
            n = int(m.group(3))
            self._lbl_alt_blc.configure(
                text=f"{new_blc*60.0:.2f}' ({n})", fg="#4CAF50")

        # v16 — AZM (and ALT) backlash from firmware BLC replies
        m = RE_BLC_QUERY.search(line)          # "BLC:AZM=x ALT=y ..."
        if m:
            self._lbl_azm_blc.configure(
                text=f"{float(m.group(1))*60.0:.2f}'", fg="#4CAF50")
            self._lbl_alt_blc.configure(
                text=f"{float(m.group(2))*60.0:.2f}'", fg="#4CAF50")
        else:
            m = RE_BLC_AZM.search(line)        # "BLC:AZM=x (...')"
            if m:
                self._lbl_azm_blc.configure(
                    text=f"{float(m.group(1))*60.0:.2f}'", fg="#4CAF50")
        m = RE_BLC_AZM_SET.search(line)        # "BLC:AZM set to x ..."
        if m:
            self._lbl_azm_blc.configure(
                text=f"{float(m.group(1))*60.0:.2f}'", fg="#00bcd4")
        m = RE_BLC_ALT_SET.search(line)        # "BLC:ALT set to x ..."
        if m:
            self._lbl_alt_blc.configure(
                text=f"{float(m.group(1))*60.0:.2f}'", fg="#00bcd4")

        self._log(line)

    # ── CONNECTION ───────────────────────────────────────────

    def _refresh_ports(self):
        """Refresh the available COM ports list.
        v15.03g-V4: prefer the last successfully-used port (from settings)
        when it is still present; otherwise fall back to the last entry.
        """
        ports = SerialManager.list_ports()
        self.port_cb["values"] = ports
        if not ports:
            return
        # Only auto-select if nothing is currently selected
        if self.port_var.get():
            return
        last_port = self.settings.get("last_port", "")
        if last_port and last_port in ports:
            self.port_var.set(last_port)
        else:
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
                # v16: populate the backlash display shortly after connect
                self.root.after(1000, lambda: self.serial.connected
                                and self.serial.send("BLC?"))
                # v15.03g-V4: persist last successful port
                self.settings["last_port"] = p
                save_settings(self.settings)

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
        self.cur_status = st
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

    def _set_azm_blc(self):
        """v16: send the manual AZM backlash value (entry is in arcminutes)."""
        raw = self._azm_blc_e.get().strip().replace(",", ".")
        try:
            arcmin = float(raw)
        except ValueError:
            messagebox.showwarning("Error", "Enter the AZM backlash in arcminutes (e.g. 2.0).")
            return
        deg = arcmin / 60.0
        if not (0.0 <= deg <= 0.5):
            messagebox.showwarning("Error", "Out of range: 0' to 30' (firmware hardstop).")
            return
        self._send(f"BLC:AZM:{deg:.4f}")

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
            "",
            f"/* ───── BACKLASH INITIAL VALUES ({self.profile_name}) ─────",
            "   Paste inside loadOrSelectProfile(), matching profile branch:",
            f"     activeBacklashDegAZM = {v['BACKLASH_AZM_INIT']:.4f}f;   // {v['BACKLASH_AZM_INIT']*60.0:.1f}'",
            f"     activeBacklashDegALT = {v['BACKLASH_ALT_INIT']:.4f}f;   // {v['BACKLASH_ALT_INIT']*60.0:.1f}'",
            "   v16: AZM = manual (set once via BLC:AZM:, persisted to EEPROM).",
            "        ALT = seed value, auto-learned by the MPU & persisted. */",
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

    # ── AUTO ALIGN controller ────────────────────────────────

    AA_TICK_MS   = 250
    AA_STALE_S   = 120.0   # no fresh solve for this long → warn (stay armed)
    AA_PROBE_AZM = 5.0     # calibration probe (arcmin)
    AA_PROBE_ALT = 2.0     # smaller: ALT range is tight
    AA_MIN_MOVE  = 0.3     # below this (arcmin), don't bother moving an axis
    AA_COMBINE_MAX = 180.0 # tot error (') below which both axes move per cycle
    AA_CAP_ALT_MAX = 20.0  # hard per-cycle cap for ALT: long continuous climbs
                           # above ~20' were measured to skip steps under load
    AA_EFF_MIN     = 0.5   # measured transfer below this → shrink that axis' cap
    AA_FINE        = 5.0   # error (') below which an axis is in fine phase
    AA_BL_PASS     = 4.0   # fine-phase anti-backlash overshoot leg (')
    AA_SOLVE_SAFE_S = 12.0 # solve arriving this long after move-end needs no
                           # purge (her exposures are 8 s + solve margin)

    def _aa_cfg(self, entry, key, default):
        try:
            v = float(entry.get())
            self.settings[key] = v
            return v
        except ValueError:
            return default

    def _aa_save_fields(self):
        """Persist AUTO ALIGN fields whenever edited (not only on START)."""
        d = self.aa_dir_e.get().strip()
        if d:
            self.settings["nina_log_dir"] = d
        self._aa_cfg(self.aa_tol_e, "aa_tolerance", 0.5)
        self._aa_cfg(self.aa_cap_e, "aa_cap", 10.0)
        self._aa_cfg(self.aa_k_e,   "aa_gain", 0.95)
        save_settings(self.settings)

    def _aa_reset_dir(self):
        self.aa_dir_e.delete(0, "end")
        self.aa_dir_e.insert(0, NINA_LOG_DIR_DEFAULT)
        self._aa_save_fields()

    def _aa_start(self):
        if self.aa_state != "IDLE":
            return
        if not self.serial.connected:
            messagebox.showwarning("AUTO ALIGN", "Connect the controller first.")
            return
        log_dir = self.aa_dir_e.get().strip()
        if not os.path.isdir(log_dir):
            messagebox.showwarning("AUTO ALIGN", f"Log folder not found:\n{log_dir}")
            return
        self.settings["nina_log_dir"] = log_dir
        self._aa_cfg(self.aa_tol_e, "aa_tolerance", 0.5)
        self._aa_cfg(self.aa_cap_e, "aa_cap", 10.0)
        self._aa_cfg(self.aa_k_e,   "aa_gain", 0.95)
        save_settings(self.settings)
        with self.aa_lock:
            self.aa_events.clear()
        self.aa_last = self.aa_ref = self.aa_pending = None
        self.aa_sign = {"AZM": None, "ALT": None}
        self.aa_purge, self.aa_ncorr = 0, 0
        self.aa_watch, self.aa_flipped = None, {}
        self.aa_seq, self.aa_t_idle = [], None
        self.aa_dyncap, self.aa_lastdir = {}, {}
        self.aa_t_last = time.time()
        self.aa_watcher = NinaLogWatcher(log_dir, self._aa_event)
        self.aa_watcher.start()
        self.aa_state = "ARMED"
        self._aa_say("armed — waiting for TPPA error solves…")
        self._log("AUTO ALIGN: started (waiting for TPPA measurements)")
        self.root.after(self.AA_TICK_MS, self._aa_tick)

    def _aa_stop(self, reason="stopped by user"):
        if self.aa_watcher:
            self.aa_watcher.stop()
            self.aa_watcher = None
        if self.aa_state != "IDLE":
            self._log(f"AUTO ALIGN: {reason}")
        self.aa_state = "IDLE"
        try:
            self._aa_say(f"idle ({reason})")
        except Exception:
            pass   # widget may be gone at app close

    def _aa_event(self, kind, payload):
        """Called from watcher thread — queue only, no Tk calls here."""
        with self.aa_lock:
            self.aa_events.append((kind, payload))

    def _aa_say(self, txt):
        self.aa_state_lbl.configure(text=f"AUTO: {txt}")

    def _aa_err_txt(self, e):
        return f"Az {e['az']:+.2f}'  Alt {e['alt']:+.2f}'  Tot {abs(e['tot']):.2f}'"

    def _aa_move(self, axis, delta_arcmin):
        """Move one axis by delta (arcmin) via absolute target, then purge 1 solve."""
        cur_arcmin = self.azm if axis == "AZM" else self.alt
        target_arcmin = cur_arcmin + delta_arcmin
        self._send(f"{axis}:{target_arcmin / 60.0:.4f}")
        self.aa_pending = (axis, delta_arcmin, target_arcmin, time.time())
        self.aa_purge = 1

    def _aa_move_done(self):
        """True once the pending move has REALLY finished — judged by the
        polled position reaching the commanded target, never by the polled
        status string (which lags by up to 500 ms and lies right after a
        send: the firmware aborts an in-flight move when a new position
        command arrives, so chaining on stale status kills the first move)."""
        if not self.aa_pending:
            return True
        axis, _delta, target, t0 = self.aa_pending
        if time.time() - t0 < 0.8:          # let polling refresh at least once
            return False
        pos = self.azm if axis == "AZM" else self.alt
        if abs(pos - target) <= 0.7:
            return True
        if time.time() - t0 > 20.0:         # stuck/slipping axis: do not hang
            self._log(f"AUTO ALIGN: {axis} move timeout "
                      f"(pos {pos:.1f}' vs target {target:.1f}') — proceeding")
            return True
        return False

    def _aa_tick(self):
        if self.aa_state == "IDLE":
            return
        tol = self.settings.get("aa_tolerance", 0.5)
        cap = self.settings.get("aa_cap", 10.0)
        k   = self.settings.get("aa_gain", 0.95)

        # drain watcher queue
        with self.aa_lock:
            events, self.aa_events = self.aa_events, []
        fresh = None
        for kind, payload in events:
            if kind == "file":
                self._log(f"AUTO ALIGN: watching {os.path.basename(payload)}")
            elif kind == "start":
                self._log("AUTO ALIGN: new TPPA measurement detected — resetting")
                self.aa_last = self.aa_ref = None
                self.aa_purge = 0
            elif kind == "finish":
                self._log("AUTO ALIGN: TPPA finished below tolerance — DONE "
                          f"after {self.aa_ncorr} corrections")
                self._aa_stop("converged (TPPA auto-finish)")
                return
            elif kind == "error":
                self.aa_t_last = time.time()
                if self.aa_purge > 0:
                    # conditional purge: a solve arriving well after move-end
                    # started its exposure after the move — keep it, save 1 cycle
                    if (self.aa_t_idle and not self.aa_pending
                            and time.time() - self.aa_t_idle >= self.AA_SOLVE_SAFE_S):
                        self.aa_purge = 0
                        fresh = payload
                    else:
                        self.aa_purge -= 1
                        self._log(f"AUTO ALIGN: purged solve ({self._aa_err_txt(payload)})")
                else:
                    fresh = payload   # keep the latest clean solve

        # staleness watch
        if time.time() - self.aa_t_last > self.AA_STALE_S:
            self._aa_say("no fresh solve for 2 min — is TPPA running? (still armed)")

        # movement in progress? wait for REAL completion (position-based)
        if self.aa_pending and not self._aa_move_done():
            self.root.after(self.AA_TICK_MS, self._aa_tick)
            return

        if fresh:
            prev = self.aa_last
            # outlier guard: ignore a solve that explodes vs the previous one
            if prev and abs(fresh["tot"]) > max(30.0, 10.0 * abs(prev["tot"])):
                self._log(f"AUTO ALIGN: outlier ignored ({self._aa_err_txt(fresh)})")
                fresh = None
            else:
                self.aa_last = fresh

        if fresh:
            st = self.aa_state
            if st == "ARMED":
                if abs(fresh["tot"]) <= tol:
                    self._aa_say("already below tolerance — waiting for TPPA gate  "
                                 f"[{self._aa_err_txt(fresh)}]")
                else:
                    g_az = self.settings.get("aa_gain_azm")
                    g_al = self.settings.get("aa_gain_alt")
                    if g_az and g_al and 0.2 <= abs(g_az) <= 3.0 and 0.2 <= abs(g_al) <= 3.0:
                        # gains remembered from a previous session: skip the
                        # ~45 s calibration and start correcting immediately.
                        # Online learning + divergence guard will catch drift.
                        self.aa_sign = {"AZM": g_az, "ALT": g_al}
                        self.aa_state = "CORRECT"
                        self._log(f"AUTO ALIGN: using stored gains "
                                  f"(AZM {g_az:+.2f}, ALT {g_al:+.2f}) — calibration skipped")
                    else:
                        self.aa_ref = fresh
                        self._aa_say(f"calibrating AZM sign (+{self.AA_PROBE_AZM:.0f}')  "
                                     f"[{self._aa_err_txt(fresh)}]")
                        self._aa_move("AZM", +self.AA_PROBE_AZM)
                        self.aa_state = "CAL_AZM"
            elif st == "CAL_AZM":
                d = fresh["az"] - self.aa_ref["az"]
                g = d / self.AA_PROBE_AZM          # SIGNED gain derr/dmove
                self.aa_sign["AZM"] = g
                gain = abs(g)
                self.settings["aa_gain_azm"] = round(g, 3)
                self._log(f"AUTO ALIGN: AZM calib  derr={d:+.2f}' for "
                          f"+{self.AA_PROBE_AZM:.0f}' -> signed gain={g:+.2f}")
                if not (0.3 <= gain <= 3.0):
                    self._log("AUTO ALIGN: WARNING — AZM gain far from 1, check setup")
                self.aa_ref = fresh
                self._aa_say(f"calibrating ALT sign (+{self.AA_PROBE_ALT:.0f}')  "
                             f"[{self._aa_err_txt(fresh)}]")
                self._aa_move("ALT", +self.AA_PROBE_ALT)
                self.aa_state = "CAL_ALT"
            elif st == "CAL_ALT":
                d = fresh["alt"] - self.aa_ref["alt"]
                g = d / self.AA_PROBE_ALT
                self.aa_sign["ALT"] = g
                gain = abs(g)
                self.settings["aa_gain_alt"] = round(g, 3)
                save_settings(self.settings)
                self._log(f"AUTO ALIGN: ALT calib  derr={d:+.2f}' for "
                          f"+{self.AA_PROBE_ALT:.0f}' -> signed gain={g:+.2f}")
                if not (0.3 <= gain <= 3.0):
                    self._log("AUTO ALIGN: WARNING — ALT gain far from 1, check setup")
                self.aa_state = "CORRECT"
            if self.aa_state == "CORRECT" and fresh:
                # divergence guard, per corrected axis: if an axis' |error|
                # grew after its correction, flip that axis' sign once; if it
                # grows again right after the flip, abort — never walk away.
                if self.aa_watch:
                    for axis_w, (before, sent, judge) in self.aa_watch.items():
                        after_s = fresh["az"] if axis_w == "AZM" else fresh["alt"]
                        after = abs(after_s)
                        if not judge:
                            continue   # attribution unreliable this cycle
                        # efficiency guard: measured transfer vs expected —
                        # a long move that skipped steps shrinks this axis' cap
                        expected = self.aa_sign[axis_w] * sent
                        if abs(expected) > 2.0:
                            effic = (after_s - before) / expected
                            # online gain learning: refine the signed gain with
                            # the actually measured transfer (EWMA 50/50), so a
                            # noisy calibration self-corrects within one cycle
                            g_obs = (after_s - before) / sent
                            if 0.2 <= abs(g_obs) <= 3.0 and g_obs * self.aa_sign[axis_w] > 0:
                                self.aa_sign[axis_w] = 0.5 * self.aa_sign[axis_w] + 0.5 * g_obs
                                self.settings[f"aa_gain_{axis_w.lower()}"] = round(self.aa_sign[axis_w], 3)
                            if effic < self.AA_EFF_MIN:
                                newcap = max(5.0, abs(sent) * 0.5)
                                cur = self.aa_dyncap.get(axis_w)
                                if cur is None or newcap < cur:
                                    self.aa_dyncap[axis_w] = newcap
                                    self._log(f"AUTO ALIGN: {axis_w} transfer only "
                                              f"{effic*100:.0f}% of expected — "
                                              f"cap reduced to {newcap:.1f}'")
                            elif effic > 0.8 and self.aa_dyncap.get(axis_w):
                                self.aa_dyncap[axis_w] = min(self.aa_dyncap[axis_w] * 1.5, 60.0)
                        if after > abs(before) + max(1.0, 0.15 * abs(before)):
                            if self.aa_flipped.get(axis_w):
                                self._log(f"AUTO ALIGN: {axis_w} diverging even after "
                                          f"sign flip ({abs(before):.2f}' -> {after:.2f}') — ABORT")
                                self._aa_stop("diverging — aborted for safety")
                                return
                            self.aa_sign[axis_w] = -self.aa_sign[axis_w]
                            self.aa_flipped[axis_w] = True
                            self.settings.pop(f"aa_gain_{axis_w.lower()}", None)
                            self._log(f"AUTO ALIGN: {axis_w} error grew "
                                      f"({abs(before):.2f}' -> {after:.2f}') — sign flipped, retrying")
                        else:
                            self.aa_flipped[axis_w] = False
                    self.aa_watch = None
                if abs(fresh["tot"]) <= tol:
                    self._aa_say("below tolerance — holding for TPPA confirmation  "
                                 f"[{self._aa_err_txt(fresh)}]")
                else:
                    # Both axes per cycle below AA_COMBINE_MAX (coupling is
                    # negligible when roughly aligned — field-measured <2%);
                    # dominant-axis-only above it (large-offset regime).
                    errs = {"AZM": fresh["az"], "ALT": fresh["alt"]}
                    if abs(fresh["tot"]) <= self.AA_COMBINE_MAX:
                        order = sorted(errs, key=lambda a: -abs(errs[a]))
                    else:
                        order = [max(errs, key=lambda a: abs(errs[a]))]
                    moves, watch = [], {}
                    for axis in order:
                        err = errs[axis]
                        other = "ALT" if axis == "AZM" else "AZM"
                        # phase gating: a fine-phase axis waits while the other
                        # is still coarse — its tiny correction would be drowned
                        # by cross-coupling from the big move (field-measured
                        # ALT->Az coupling ~0.2 at high ALT angles)
                        if abs(err) < self.AA_FINE and abs(errs[other]) > 3 * self.AA_FINE:
                            continue
                        g = self.aa_sign[axis]
                        g_eff = g if 0.3 <= abs(g) <= 3.0 else (1.0 if g >= 0 else -1.0)
                        corr = -k * err / g_eff
                        # adaptive cap: user cap guaranteed; large errors may
                        # move up to 50% of the error (60' hard ceiling).
                        if axis == "AZM":
                            # AZM is field-proven reliable at any amplitude:
                            # allow a full-error correction (120' ceiling)
                            eff_cap = max(cap, min(1.00 * abs(err), 120.0))
                        else:
                            eff_cap = max(cap, min(0.50 * abs(err), 60.0))
                            eff_cap = min(eff_cap, self.AA_CAP_ALT_MAX)
                        if self.aa_dyncap.get(axis):
                            eff_cap = min(eff_cap, self.aa_dyncap[axis])
                        corr = max(-eff_cap, min(eff_cap, corr))
                        if abs(corr) >= self.AA_MIN_MOVE:
                            rev = (self.aa_lastdir.get(axis, 0)
                                   and self.aa_lastdir.get(axis) * corr < 0)
                            if rev and abs(err) < self.AA_FINE:
                                # fine-phase direction reversal: two-leg move.
                                # Overshoot past the target by AA_BL_PASS, then
                                # come back along the reference direction. The
                                # firmware's backlash compensation error of the
                                # two reversals cancels EXACTLY, whatever the
                                # true mechanical play is — net motion is exact.
                                sref = self.aa_lastdir[axis]
                                moves.append((axis, corr - sref * self.AA_BL_PASS))
                                moves.append((axis, sref * self.AA_BL_PASS))
                                watch[axis] = (err, corr, False)
                                # lastdir ends back on the reference direction
                                self.aa_lastdir[axis] = sref
                            else:
                                moves.append((axis, corr))
                                # judge only when attribution is reliable:
                                # big enough and not direction-reversed
                                watch[axis] = (err, corr, abs(corr) >= 2.0 and not rev)
                                self.aa_lastdir[axis] = 1 if corr > 0 else -1
                    if len(watch) == 2:
                        (a1, w1), (a2, w2) = watch.items()
                        if abs(w2[1]) > 3 * abs(w1[1]):
                            watch[a1] = (w1[0], w1[1], False)
                        elif abs(w1[1]) > 3 * abs(w2[1]):
                            watch[a2] = (w2[0], w2[1], False)
                    if moves:
                        axis, corr = moves[0]
                        self.aa_seq = moves[1:]
                        self.aa_ncorr += 1
                        self.aa_watch = watch
                        self._aa_say(f"correction #{self.aa_ncorr}: "
                                     + " ".join(f"{a} {c:+.2f}'" for a, c in moves)
                                     + f"  [{self._aa_err_txt(fresh)}]")
                        self._log(f"AUTO ALIGN: corr #{self.aa_ncorr}  "
                                  + "  ".join(f"{a} {c:+.2f}'" for a, c in moves)
                                  + f"  (err {self._aa_err_txt(fresh)})")
                        self._aa_move(axis, corr)
                    else:
                        self._aa_say("errors below min move — waiting  "
                                     f"[{self._aa_err_txt(fresh)}]")

        if self.aa_pending and self._aa_move_done():
            self.aa_pending = None
            if self.aa_seq:
                axis, corr = self.aa_seq.pop(0)
                self._log(f"AUTO ALIGN: corr #{self.aa_ncorr}  {axis} {corr:+.2f}' (chained)")
                self._aa_move(axis, corr)
            else:
                self.aa_t_idle = time.time()

        self.root.after(self.AA_TICK_MS, self._aa_tick)

    def close(self):
        self._aa_save_fields()
        self._aa_stop("app closing")
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