# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL / MPU-6500)

Welcome to what is likely **the world's first motorized Polar Alignment mount featuring an active Gyroscopic Machine Learning ratio adaptation.**

This is a minimal **GRBL‑style** firmware + hardware recipe designed to drive a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.**

It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the "Avalon" protocol using a **non‑blocking motion engine**.

> **TL;DR** – Flash `PolarAlign_auto.ino`, select your hardware profile via serial (`1` = Prototype, `2` = V2), wire the motors and MPU-6500, set N.I.N.A. to talk to an **"Avalon Polar Alignment"**, **leave the TPPA "Gear Ratio" field at `1.0`**, and let TPPA's plate-solve loop converge to sub-arcminute precision while the firmware silently learns your mechanics.

> 🏆 **Field-tested result: < 0.2 arcminute polar alignment error** achieved with TPPA in under 4 iterations, with a 20 kg payload.

---

## 🔀 Which Version Should I Build?

Two hardware configurations are supported. A **single firmware** handles both — you select the profile once at first boot via the serial monitor. No recompile needed.

| | **Prototype** | **V2** *(under fabrication)* |
|---|---|---|
| ALT axis | Commercial tilt plate + T8 lead screw | Custom CNC ALT (bielle mechanism) |
| AZM bearing | igus PRT-02 LC J4 slewing ring | RU42 crossed roller bearing |
| Base | Monolithic 15180 aluminium profiles | Two-piece CNC aluminium plates |
| Firmware profile | `1` — PROTO | `2` — V2 |
| `ALT_MOTOR_GEARBOX` | 148.8 (30:1 × 4.96) | 208.3 (30:1 × 6.94) |
| Status | ✅ **Field-validated — recommended** | 🔧 Under fabrication |
| Hardware docs | [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md) | [`HARDWARE_V2.md`](./HARDWARE_V2.md) |

> 💡 If you're building for the first time, **start with the Prototype**. It uses off-the-shelf parts, has been validated to < 0.2 arcmin under 20 kg, and the firmware is identical.

---

## 🎬 See It In Action

> 📌 Videos 1–3 show the **Prototype hardware**. Video 4 is a V2 kinematic simulation. Video 5 is the CNC-machine V2 in action with a full setup!!

| # | Video | What you'll see |
|---|-------|-----------------|
| 1 | [First Test with Full Payload](https://youtu.be/girvoCZ_UCE) | 15 kg equatorial mount on the PA platform — first motorized movements under real load. |
| 2 | [Homing Sequence (Arduino Serial Monitor)](https://youtu.be/NkoLJ03FSSY) | Live serial output: homing, limit switch detection, pull-off, MPU-6500 gyroscope tare. |
| 3 | [**TPPA Session — Below 0.2 Arcminute!**](https://youtu.be/gfE6sZmrzuw) | Complete polar alignment run in N.I.N.A. — watch TPPA converge to < 0.2' in real-time. |
| 4 | [**V2 ALT Bielle — Fusion 360 Kinematic Simulation**](https://youtu.be/YnkVJ2hzqB0) | Kinematic simulation of the V2 CNC bielle: full −2° to +10° travel, pivot geometry, T8 drive. |
| 5 | [**V2 in the Real World — First Stress Test**](https://youtube.com/shorts/aVQfPjl87hA) | The V2 CNC assembly under full load — first real-world mechanical stress test. |
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

**Quick start (Windows):**
1. Download `PolarAlignController.exe` from the [latest Release](../../releases/latest)
2. Double-click — no installation, no Python needed

**From source (any OS):**
```bash
pip3 install pyserial
python3 GUI/PolarAlignGUI_v15_03g_V2.py
```

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

## ⚙️ First Flash: Hardware Profile Selection

The firmware stores the hardware profile in the ESP32's NVS flash (survives reflashes):

**First boot after flashing:**
```
+---------------------------------------------------+
|  HARDWARE PROFILE NOT SET                         |
|  Send '1'  -> PROTO  (commercial tilt plate)      |
|  Send '2'  -> V2     (CNC + RU42)       |
+---------------------------------------------------+
```

Send `1` or `2` in the Serial Monitor. The profile is saved and the ESP32 reboots. It persists across reflashes **provided** `Tools → Erase All Flash Before Upload` is set to **Disabled** (the Arduino IDE default). If this option is **Enabled**, the entire flash including NVS is wiped and you will need to re-select the profile after each flash.

**To change profile later:** send `PROFILE:RESET` — the firmware clears NVS and asks again on next boot.

**To verify current profile:** send `PROFILEINFO`.

---

## ⚠️ Before You Start: Homing is Required

**You MUST run `HOME` (or `$H`, or press the physical Home button) before launching a TPPA session.**

Without homing, the firmware doesn't know the true physical ALT position. TPPA jogs are silently blocked (firmware replies `ok` but doesn't move).

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

## ⚡ Electronics & Wiring

### The Brain: FYSETC E4 V1.0

| Spec | Value |
|------|-------|
| MCU | ESP32-WROOM-32 @ 240 MHz |
| Drivers | 4× TMC2209, UART-addressed |
| Used channels | MOT-X (Azimuth) + MOT-Y (Altitude) |
| Power input | 12 V DC |

> ⚠️ **FYSETC E4 V1.0 only!** V2.0 has different pin mapping — not compatible.

### The Sensor: MPU-6500 via SD Card Hack

GPIO 18 (SD SCK) → I2C SCL / GPIO 19 (SD MISO) → I2C SDA

| Wire | Signal | GPIO | FYSETC E4 target |
|:----:|--------|:----:|------------------|
| 🔴 | VCC 3.3V | — | 3.3V header |
| 🟡 | GND | — | GND header |
| 🔵 | SCL | 18 | SD Card `SCK` |
| 🟢 | SDA | 19 | SD Card `MISO` |

### Motors

| Axis | Port | Mode | µstep | Current |
|------|------|------|:-----:|:-------:|
| AZM | MOT-X | SpreadCycle | 16 | 600 mA |
| ALT | MOT-Y | SpreadCycle | 4 | 300 mA |

### Safety Inputs

| Function | GPIO | Type |
|----------|:----:|------|
| ALT Limit Switch | 34 | Input only (X-MIN header) |
| Home Button | 35 | Input only (Y-MIN header) |

### Full GPIO Map

| Signal | GPIO | E4 silkscreen |
|--------|:----:|---------------|
| STEP AZM | 27 | MOT-X |
| DIR AZM | 26 | MOT-X |
| EN (both) | 25 | /ENABLE |
| UART | 21/22 | Shared bus (Addr 1=AZM, Addr 2=ALT) |
| STEP ALT | 33 | MOT-Y |
| DIR ALT | 32 | MOT-Y |
| SCL | 18 | SD Card `SCK` |
| SDA | 19 | SD Card `MISO` |
| Limit switch | 34 | Z-MIN |
| Home button | 35 | Y-MIN |

> ⚠️ **UART jumpers:** Place two jumper caps on the TXD/RXD header. Without them, motors won't respond.

---

## 🛠️ Configuration Knobs

Profile-dependent constants are set automatically from NVS. All other constants can be edited directly in the sketch or via the **Firmware Config tab** in the GUI.

```cpp
/* ── AUTO-SELECTED BY FIRMWARE PROFILE ── */
// PROTO profile:
constexpr float ALT_MOTOR_GEARBOX  = 148.8f;   // UMOT 30:1 × 4.96 crank
constexpr bool  AXIS_REV_ALT       = true;
constexpr float HOME_TRIGGER_ANGLE = 0.0f;
constexpr float ALT_LIMIT_NEG      = 0.0f;

// V2 profile:
constexpr float ALT_MOTOR_GEARBOX  = 208.3f;   // UMOT 30:1 × 6.94 bielle
constexpr bool  AXIS_REV_ALT       = false;
constexpr float HOME_TRIGGER_ANGLE = -2.0f;
constexpr float ALT_LIMIT_NEG      = -2.0f;

/* ── SHARED (both profiles) ── */
constexpr float GEAR_RATIO_AZM     = 100.0f;   // Harmonic drive
constexpr uint16_t RMS_CURRENT_AZM = 600;       // mA
constexpr uint16_t RMS_CURRENT_ALT = 300;       // mA — thermal-safe in UMOT
constexpr float AZM_BACKLASH_DEG   = 0.033f;   // 2' harmonic drive elastic zone
constexpr unsigned long GLOBAL_SETTLE_MS = 500; // ms — anti-vibration settle
constexpr long RAMP_LENGTH         = 500;       // steps to reach cruise speed

/* ── TRAVEL LIMITS ── */
constexpr float AZM_LIMIT_NEG = -30.0f;
constexpr float AZM_LIMIT_POS =  30.0f;
constexpr float ALT_LIMIT_POS =  10.0f;
```

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

## ⚙️ Arduino IDE Setup

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
   | **Erase All Flash** | **Disabled** ← important for NVS profile persistence |

4. **Flash** `Arduino code/PolarAlign_auto.ino` — same binary for both hardware versions
5. **Open Serial Monitor** (115200 baud) → send `1` (Prototype) or `2` (V2) when prompted
6. Profile stored in NVS — survives reflash **as long as** `Tools → Erase All Flash Before Upload` = **Disabled**. If Enabled, re-select after each flash

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

## 🌡️ Motor Thermal Management

| RMS Current | Power | Temperature | PLA-safe? |
|:-----------:|:-----:|:-----------:|:---------:|
| 800 mA | ~1.6 W | 55–65°C | ❌ |
| **300 mA (default)** | ~0.2 W | Barely warm | ✅ |
| 400 mA (cold weather) | ~0.4 W | ~35°C | ✅ |

> Use **PETG** or **ABS** for the ALT motor cradle above 300 mA.

---

## ⚖️ Payload Rating

| Rating | Prototype | V2 |
|--------|:---------:|:--:|
| Recommended | **20 kg** | **25 kg** |
| Advanced | 25 kg | 30+ kg |

The V2's RU42 crossed roller bearing (153 Nm tilting moment, 4.1× SF at 25 kg) eliminates the tilting-moment uncertainty of the Prototype's igus PRT-02 LC.

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
