# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL / MPU-6500)

Welcome to what is likely **the world's first motorized Polar Alignment mount featuring an active Gyroscopic Machine Learning ratio adaptation.**

This is a minimal **GRBL‑style** firmware + hardware recipe designed to drive a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.**

It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the "Avalon" protocol using a **non‑blocking motion engine**.

> **TL;DR** – Flash `PolarAlign_auto.ino`, select your hardware profile via serial (`1` = Prototype, `2` = V2), wire the motors and MPU-6500, set N.I.N.A. to talk to an **"Avalon Polar Alignment"**, **leave the TPPA "Gear Ratio" field at `1.0`**, and let TPPA's plate-solve loop converge to sub-arcminute precision while the firmware silently learns your mechanics.

> 🏆 **Field-tested result: < 0.2 arcminute polar alignment error** achieved with TPPA in ~15 iterations, with a 20 kg payload.

---

## 🔀 Which Version Should I Build?

Two hardware configurations are supported. A **single firmware** handles both — you select the profile once at first boot via the serial monitor. No recompile needed.

| | **Prototype** | **V2** |
|---|---|---|
| ALT axis | Commercial tilt plate + T8 lead screw | Custom CNC ALT (bielle mechanism) |
| AZM bearing | igus PRT-02 LC J4 slewing ring | RU42 crossed roller bearing |
| Base | Monolithic 15180 aluminium profiles | Two-piece CNC aluminium plates |
| Firmware profile | `1` — PROTO | `2` — V2 |
| `ALT_MOTOR_GEARBOX` | 148.8 (30:1 × 4.96) | ~124 (initial estimate — ML converges to true value within 2–3 jogs) |
| Status | ✅ **Field-validated — recommended** | 🔧 **Delivered & under testing** |
| Hardware docs | [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md) | [`HARDWARE_V2.md`](./HARDWARE_V2.md) |

> 💡 If you're building for the first time, **start with the Prototype**. It uses off-the-shelf parts, has been validated to < 0.2 arcmin under 20 kg, and the firmware is identical.

---

## ⚖️ Payload Rating

| Rating | Prototype | V2 |
|--------|:---------:|:--:|
| Recommended | **20 kg** | **25 kg** |
| Advanced | 25 kg | 30+ kg |

The V2's RU42 crossed roller bearing (153 Nm tilting moment, 4.1× SF at 25 kg) eliminates the tilting-moment uncertainty of the Prototype's igus PRT-02 LC.

> 💡 **Counterweights lower the effective centre of gravity.** If your EQ mount uses a counterweight bar, the counterweights hanging below the RA axis reduce the effective torque arm seen by the ALT mechanism — a 32 kg setup (mount + scope + 8 kg counterweights) behaves mechanically closer to a 20 kg unbalanced load. This is a net positive for structural margins, but means your payload budget should always be calculated with the **full assembly** (mount + scope + counterweights + accessories), not scope alone.

---

## 🎬 See It In Action

> 📌 Videos 1–3 show the **Prototype hardware**. Video 4 is a V2 kinematic simulation. Video 5 is the assembled V2 under full load.

| # | Video | What you'll see |
|---|-------|-----------------|
| 1 | [First Test with Full Payload](https://youtu.be/girvoCZ_UCE) | 15 kg equatorial mount on the PA platform — first motorized movements under real load. |
| 2 | [Homing Sequence (Arduino Serial Monitor)](https://youtu.be/NkoLJ03FSSY) | Live serial output: homing, limit switch detection, pull-off, MPU-6500 gyroscope tare. |
| 3 | [**TPPA Session — Below 0.2 Arcminute!**](https://youtu.be/gfE6sZmrzuw) | Complete polar alignment run in N.I.N.A. — watch TPPA converge to < 0.2' in real-time. |
| 4 | [**V2 ALT Bielle — Fusion 360 Kinematic Simulation**](https://youtu.be/YnkVJ2hzqB0) | Kinematic simulation of the V2 CNC bielle: full −2° to +10° travel, pivot geometry, T8 drive. |
| 5 | [**V2 in the Real World — First Stress Test**](https://youtube.com/shorts/aVQfPjl87hA) | The V2 CNC assembly under full load — first real-world mechanical stress test. |
| 6 | *(coming soon)* **V2 Field Demo — TPPA Session** | First complete TPPA polar alignment session on the assembled V2 hardware. |

---

## ⚡ Quick Start

### 1 — Arduino IDE Setup

1. **Install ESP32 core** — Boards Manager → *esp32* (≥ v2.0.17)
2. **Install TMCStepper library** via Library Manager
3. **Board settings:**

   | Option | Value |
   |--------|-------|
   | Board | ESP32 Dev Module |
   | CPU Freq | 240 MHz |
   | Flash Size | 4 MB |
   | Partition | **Huge APP (3 MB / 1 MB SPIFFS)** |
   | Upload speed | 115200 bps |
   | **Erase All Flash** | **Disabled** ← critical for NVS profile persistence |

4. Flash `Arduino code/PolarAlign_auto.ino` — same binary for both hardware versions

### 2 — Hardware Profile Selection

The firmware stores the hardware profile in ESP32 NVS flash (survives reflashes):

```
+---------------------------------------------------+
|  HARDWARE PROFILE NOT SET                         |
|  Send '1'  -> PROTO  (commercial tilt plate)      |
|  Send '2'  -> V2     (CNC + RU42 — v15.03g-auto-p4) |
+---------------------------------------------------+
```

Open Serial Monitor (115200 baud), send `1` or `2`. Profile is saved and the ESP32 reboots. It persists across reflashes **provided** `Tools → Erase All Flash Before Upload` = **Disabled**.

- **To change profile later:** send `PROFILE:RESET`
- **To verify current profile:** send `PROFILEINFO`

### 3 — Wire the Hardware

See the hardware doc for your version:
- Prototype: [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md) — FYSETC E4 wiring, MPU-6500 SD card hack, GPIO map, UART jumpers
- V2: [`HARDWARE_V2.md`](./HARDWARE_V2.md)

### 4 — Home Before Every Session

**You MUST run `HOME` before launching a TPPA session.**

Without homing, TPPA jogs are silently blocked (firmware replies `ok` but doesn't move).

**What homing does:**
1. Moves ALT down until the physical limit switch triggers
2. Performs a safety pull-off (0.2°)
3. Defines this position as the mechanical zero
4. Tares the MPU-6500 gyroscope
5. Saves homing state to EEPROM (survives DTR reboots)
6. Unlocks TPPA jog commands

> 💡 **Auto-recovery:** If the limit switch is already pressed at power-on, the firmware runs homing automatically.

> 💡 **Bench testing without homing:** Direct serial commands (`ALT:`, `AZM:`) and GUI jog buttons work without homing.

---

## 🚨 Critical TPPA Plugin Settings (read this — it will save you hours)

The firmware can be working perfectly and the mount will still **never move** if the TPPA plugin is misconfigured. These settings live in the TPPA plugin **Options → Settings** tab in N.I.N.A.

| Setting | Required value | Why it matters |
|---------|:--------------:|----------------|
| **Do automated adjustments?** | **ON** | ⚠️ **The #1 gotcha.** If OFF, TPPA measures and displays the error but **sends no `$J=` commands at all** — motors never move, no error message. **Check this first.** |
| **Polar Alignment System** | `UPAS` | Selects the Avalon/GRBL dialect the firmware emulates. |
| **Gear Ratio** (AZM & ALT) | `1.0` | Firmware converts arcmin ↔ degrees internally. Any other value double-scales every correction. |
| **Reverse Azimuth Axis** / **Reverse Altitude Axis** | `OFF` (default) | Direction handled by firmware profile. **If a motor moves the wrong way, toggle the corresponding axis here** — one axis at a time. |
| **Default Move Rate** | `10` | Factory default `3` produces very small corrections — motor barely moves per step. **Set to 10.** |
| **Settle Time** | `3 s` | Factory default 5 s is unnecessary. 3 s is sufficient: the firmware's `GLOBAL_SETTLE_MS = 500` absorbs vibrations before TPPA plate-solves. |
| **Alignment Tolerance** | `0.2` arcmin | Factory default 1 arcmin stops too early. **0.2 arcmin is achievable** with this hardware. Tune between 0.2 and 1.0 depending on seeing conditions. |

> 💡 **Initial error and travel limits.** If the starting polar error is large, TPPA displays *"Initial Polar Alignment error is large. Correction phase will be unreliable."* — but corrections still proceed as long as the required moves stay within the firmware's travel limits (`AZM_LIMIT ±30°`, `ALT_LIMIT −2°/+10°`). Rough-align to your latitude before launching TPPA to avoid hitting these limits mid-session.

> 🔍 **Debugging "no movement":** Run a TPPA session, close it, reconnect the GUI, send `DIAG`. No `$J=` lines in the log → TPPA never sent commands → plugin setting issue (almost always *Do automated adjustments = OFF*).

---

## 🧪 Serial Command Reference

Use the **Serial Monitor** (115200 baud, Newline) or the **GUI Raw field**.

### Profile Commands

| Command | Action |
|---------|--------|
| `PROFILEINFO` | Show active profile and all cfg_ values |
| `PROFILE:RESET` | Clear profile from NVS → next boot asks to re-select |

### Direct Commands (degrees)

| Command | Action |
|---------|--------|
| `HOME` / `$H` | Homing + gyroscope tare. Required before TPPA. |
| `DIAG` | Full diagnostic — positions, ratios, command log. |
| `ALT:2.5` | Move ALT to 2.5° (absolute). |
| `AZM:5.0` | Move AZM to 5.0° (absolute). |
| `AZM:ZERO` | Redefine current AZM as 0° and reset AZM learning state. |
| `RST` | Soft reset — abort motion, clear log. |
| `MPU` | Lightweight gyroscope query → `MPU:tared,raw`. |

### GRBL Protocol (N.I.N.A./TPPA, arcminutes)

| Command | Meaning |
|---------|---------|
| `$J=G53X+300.00F400` | Absolute jog: AZM to +300' (= 5.0°) |
| `$J=G91G21Y-390.00F300` | Relative jog: ALT −390' (= −6.5°) |
| `?` | Status poll → `<Idle\|MPos:x,y,0\|>` |
| `!` / `~` | Feed-Hold / Resume |

> 💡 Set TPPA **Gear Ratio = `1.0`**. MPos values are in arcminutes and map directly.

> ⚠️ TPPA Free Field sends absolute G53 commands. Preset buttons and auto-alignment send relative G91.

---

## 🖥️ Desktop Controller GUI

A cross-platform desktop application (`GUI/PolarAlignGUI_v15_03g_V2.py`) controls the mount **without N.I.N.A.** — useful for bench testing, manual polar alignment, and firmware configuration.

**Features:**
- **Profile selector** at startup (Prototype / V2) — sets all hardware-specific defaults automatically
- **Jog buttons** in arcminutes (30', 10', 5', 1') and arcseconds (30", 10", 5", 1") with degree equivalents shown
- **Directional layout** — AZM: West ← → East / ALT: ▲ Up / ▼ Down, mirrored for intuitive use
- **Absolute positioning** — "Go to (°)" fields clearly labeled [ABSOLU]
- **Live position display** — AZM/ALT in degrees and arcminutes, real-time polling
- **Learning Monitor** — live MPU error and learned ratios (ALT/AZM) after each jog
- **System commands** — HOME, DIAG, RST, AZM:ZERO — one click
- **Raw serial console** — full log + send any command directly
- **Firmware Config tab** — edit hardware constants and generate ready-to-paste Arduino code

**Run from source (any OS — Windows, macOS, Linux):**
```bash
pip3 install pyserial
python3 GUI/PolarAlignGUI_v15_03g_V2.py
```

> ℹ️ No pre-built executable is distributed. Python 3.8+ and pyserial are the only dependencies.

---

## 🌟 Key Features

| Feature | Description |
|---------|-------------|
| 🔧 **Single unified firmware** | `PolarAlign_auto.ino` runs on both Prototype and V2. Hardware profile stored in ESP32 NVS flash — set once at first boot, survives all subsequent reflashes. |
| 🧠 **ALT Machine Learning** | The MPU-6500 measures real physical movement after every ALT jog, computes the true steps-per-degree ratio, and saves it to EEPROM. The mount silently improves its own accuracy over time. |
| 🧠 **AZM Ratio Learning** | The firmware infers the AZM gear ratio from consecutive TPPA correction residuals (no sensor needed). A triple guard prevents NaN/deadlock bugs. |
| 🔭 **TPPA-Driven Convergence** | The firmware trusts TPPA's plate-solve loop rather than running its own corrections. Result: **< 0.2 arcminute**. |
| 📐 **Arcminute Protocol** | Bidirectional unit conversion: TPPA jog commands (arcminutes) → internal degrees → MPos reports back in arcminutes. Direct serial commands remain in degrees for bench testing. |
| 🛡️ **Homing Guard + DTR Persistence** | TPPA jogs are blocked until homing is completed. Homing state survives DTR-triggered reboots — no re-homing needed when switching between GUI and TPPA. |
| ⏱️ **Optimised Timing** | RAMP_LENGTH = 500 steps, GLOBAL_SETTLE = 500 ms — comfortably within N.I.N.A.'s 7-second movement timeout. |
| ⚡ **Zero Lag Engine** | Non-blocking trapezoidal acceleration. No `delay()` in the main loop — N.I.N.A. polls 10×/second without interruption. |
| 🔘 **HOME button DIAG dump** | Short press during TPPA session → dumps full diagnostic to serial (TPPA ignores it). Long press → starts homing. |

---

## 🔬 How It Works: Architecture Deep Dive

### Layer 1 — Non-Blocking Motion Engine

The entire firmware is a state machine inside `tickMotion()`. Motor pulses, MPU sampling, settle timers, and serial communication are interleaved without ever calling `delay()`. N.I.N.A. polls status 10×/second — no blocking call ever causes a missed poll.

Trapezoidal acceleration (soft start → cruise → soft stop) prevents step loss on high-inertia loads.

### Layer 2a — MPU-6500 Observe & Learn (ALT)

After every ALT movement:
1. **Settle** (500 ms) — vibrations damp
2. **Sample** (50 readings × 5 ms) — true tilt measured using Earth's gravity vector
3. **Learn** — commanded vs. actual movement → new steps/degree ratio, EWMA 10%, saved to EEPROM if changed > 0.5 steps/deg

MPU operates in **observe-only mode** — it learns, never corrects. TPPA's plate-solve loop handles convergence.

### Layer 2b — Residual Learning (AZM)

No sensor on AZM. The firmware infers the gear ratio from TPPA's correction residuals:
```
effectiveMoved = prevDelta − currDelta
measuredRatio  = currentRatio × prevDelta / effectiveMoved
```

Three guards prevent the deadlock seen in earlier versions (effectiveMoved threshold, NaN/Inf check, direction-reversal wipe). EWMA 5%, band ±10%.

### Layer 3 — GRBL Protocol Synchronisation

- **Silent boot** — firmware is silent until the first `?` poll
- **Status scaling** — reported ALT position held slightly below target during MPU observation to prevent TPPA from reclaiming control prematurely
- **Diagnostic buffer** — 4 KB RAM log, invisible to N.I.N.A., retrieved via `DIAG`
- **AZM backlash** — 2' of dead steps on direction reversal compensate for harmonic drive elastic zone

---

## 📁 Repository Structure

```
├── Arduino code/
│   ├── PolarAlign_auto.ino          ← ✅ Current unified firmware (PROTO + V2)
│   └── archive/                     ← Legacy versions (reference only)
├── GUI/
│   ├── PolarAlignGUI_v15_03g_V2.py  ← ✅ Current GUI
│   └── archive/
├── 3D STEP Models/
│   ├── PolarALIGN_Proto_STEP.zip
│   └── PolarALIGN_V2_STEP.zip
├── IMAGES/
├── HARDWARE_Prototype.md
├── HARDWARE_V2.md
└── README.md
```

---

## 📄 License

**MIT License** — do whatever you want, just keep the header.

---

## 🙏 Acknowledgements

* **Stefan Berg** – author of the **Three-Point Polar Alignment** plug-in and core N.I.N.A. contributor; his protocol docs and DLL patches made this project possible.
* **Avalon Instruments** – for the idea of a lean, GRBL-style alignment controller.
* **Claude** (Anthropic) & **Gemini** (Google) – for the non-blocking engine architecture, the gyroscopic ML system, the GRBL protocol reverse-engineering, and months of hardcore debugging.
* Maintained by **Antonino Nicoletti** — *clear skies!*
