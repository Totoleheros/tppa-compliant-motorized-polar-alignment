# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL / MPU-6500)

Welcome to what is likely **the world's first motorized Polar Alignment mount featuring an active Gyroscopic Machine Learning ratio adaptation.**

This is a minimal **GRBL‑style** firmware + hardware recipe designed to drive a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.**

It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the "Avalon" protocol using a **non‑blocking motion engine**.

> **TL;DR** – Flash the sketch, wire the motors and the MPU-6500 gyroscope, set N.I.N.A. to talk to an **"Avalon Polar Alignment"**, **leave the TPPA "Gear Ratio" field at `1.0`**, and let TPPA's plate-solve loop converge to sub-arcminute precision while the firmware silently learns your mechanics.

> 🏆 **Field-tested result: < 0.2 arcminute polar alignment error** achieved with TPPA in under 4 iterations, with a 20 kg payload.

---

## 🎬 See It In Action

| # | Video | What you'll see |
|---|-------|-----------------|
| 1 | [First Test with Full Payload](https://youtu.be/girvoCZ_UCE) | 15 kg equatorial mount on the PA platform — first motorized movements under real load. |
| 2 | [Homing Sequence (Arduino Serial Monitor)](https://youtu.be/NkoLJ03FSSY) | Live serial output: homing, limit switch detection, pull-off, MPU-6500 gyroscope tare (with the same 15 kg payload). |
| 3 | [**TPPA Session — Below 0.2 Arcminute!**](https://youtu.be/gfE6sZmrzuw) | Complete polar alignment run in N.I.N.A. — watch TPPA converge to < 0.2' in real-time (with the same 15 kg payload). |

---

## 🔭 Author's Testing Setup

The annotated photo below shows the full astrophotography rig used to develop and validate this project. The PA platform (item 2) carries everything above it — roughly **20 kg of total payload**, which sits right at the recommended limit.

![Setup Overview](IMAGES/ASSEMBLY/Real_World/setup_annotated.jpeg)

| # | Component | Est. Weight | Details |
|---|-----------|:-----------:|---------|
| 1 | **iOptron LiteRoc Tripod** | — | Stable field tripod (below the PA platform — not counted in payload) |
| 2 | **TPPA Motorized PA System** | — | ESP32 + harmonic drive (AZM), UMOT worm gear (ALT), MPU-6500 gyroscope (part of the base — not counted) |
| 3 | **iOptron GEM28 Mount** | ~11.5 kg | EQ mount head (4.5 kg), CW bar (0.5 kg), counterweights (4.5 + 2 kg) |
| 4 | **Optical Train** | ~5.2 kg | Sharpstar 94EDPH + 0.8× reducer, ZWO EAF autofocuser, 60 mm guide scope + ASI120MM + DIY guide focuser, DIY flat panel, dew heaters ×2, rings + dovetail bar |
| 5 | **Imaging Train** | ~1.5 kg | RisingCam IMX571 Mono, ToupTek 5×2" filter wheel, DIY rotator + spacers |
| 6 | **Power & Compute** | ~1.5 kg | Pegasus Ultimate Powerbox, Mele mini-PC, cables + 3D-printed parts |
| | **Total on PA platform** | **~19.5–20 kg** | Within the recommended 20 kg limit |

> 💡 This is a real-world astrophotography rig, not a test bench. Every firmware version is validated under these conditions.

---

## 🖼️ Build Gallery

The **[IMAGES](./IMAGES/)** directory contains everything you need to visualize the project:

- **`3D_Model/`** — Full CAD renders of the assembly (exploded views, cross-sections, detail shots).
- **`Real_World/`** — Photos from the actual build of the first prototype — wiring, mechanical assembly, field setup.

> 💡 If you're considering building one, start with the 3D renders to understand the architecture, then check the real-world photos to see what it actually looks like assembled.

---

## 🖥️ Desktop Controller GUI

A cross-platform desktop application is included to control the mount **without N.I.N.A.** — useful for bench testing, manual positioning, and firmware configuration.

![PolarAlign Controller GUI](IMAGES/GUI/PolarAlignController.jpg)

**Features:**
- **Jog buttons** (±0.1° ±1° ±5°) for both AZM and ALT axes, plus free-field absolute positioning
- **Live position display** with real-time polling (AZM/ALT in degrees and arcminutes)
- **System commands** — HOME, DIAG, RST, AZM:ZERO — one click
- **Raw serial console** — full log + send any command directly
- **Firmware Config tab** — edit all hardware constants (gear ratios, currents, limits…) and generate a ready-to-paste Arduino code block
- **Save/Load** configurations as JSON files

**Quick start (Windows):**
1. Download `PolarAlignController.exe` from the [latest Release](../../releases/latest)
2. Double-click — no installation, no Python needed

**From source (any OS):**
```bash
pip3 install pyserial
python3 PolarAlignGUI.py
```

> 💡 The GUI uses direct serial commands (`ALT:`, `AZM:`) in degrees — these work **without homing**, ideal for bench testing. TPPA uses its own `$J=` protocol in arcminutes during alignment sessions.

---

## 🌟 Key Features

| Feature | Description |
|---------|-------------|
| 🧠 **Machine Learning Ratio** | The MPU-6500 measures real physical movement after every ALT jog, computes the true steps-per-degree ratio, and saves it to EEPROM. The mount silently improves its own accuracy over time. |
| 🔭 **TPPA-Driven Convergence** | The firmware trusts TPPA's plate-solve loop rather than running its own corrections. Star-based measurements are far more accurate than accelerometer readings. Result: **< 0.2 arcminute**. |
| 📐 **Arcminute Protocol** | Bidirectional unit conversion: TPPA jog commands (arcminutes) → internal degrees → MPos reports back in arcminutes. Direct serial commands remain in degrees for bench testing. |
| 🛡️ **Homing Guard** | TPPA jog commands are blocked until homing is completed, preventing the ALT travel limits from clamping incorrectly due to an unknown physical position. |
| 🔇 **Global Settle** | 2-second anti-vibration delay after every movement before reporting `<Idle>`. Prevents TPPA from plate-solving on a still-vibrating mount. |
| ⚡ **Zero Lag Engine** | Non-blocking trapezoidal acceleration with real-time serial polling. No `delay()` calls anywhere — N.I.N.A. polls status 10× per second and the firmware never misses a beat. |

---

## 🔬 How It Works: Architecture Deep Dive

### The Challenge

The ALT axis (worm gearbox + lead screw + crank mechanism) is inherently subject to mechanical backlash and hysteresis, especially under a 20 kg payload. The AZM axis (harmonic drive) has its own issue: elastic deformation in the flex spline creates a "lost motion" dead zone on direction reversals. For TPPA to converge reliably, the firmware must handle both problems transparently.

### The Solution: A Three-Layer Architecture

**Layer 1 — Non-Blocking Motion Engine**

The entire firmware is built as a state machine inside `tickMotion()`. Motor pulses, MPU sampling, settle timers, and serial communication are all interleaved without ever calling `delay()`. This is critical because N.I.N.A. polls the status (`?`) ten times per second — a single blocking call would cause buffer overflows, display lag, or connection drops.

The motion engine generates trapezoidal acceleration profiles (soft start → cruise → soft stop) to prevent step loss on high-inertia loads like the 100:1 harmonic drive.

**Layer 2 — MPU-6500 Observe & Learn (Altitude)**

After every ALT movement, the firmware enters a non-blocking observation phase:

1. **Settle** (500 ms) — mechanical vibrations damp out.
2. **Sample** (50 readings over ~250 ms) — the MPU-6500's accelerometer measures the true tilt angle using Earth's gravity vector. Averaging 50 samples eliminates MEMS sensor noise.
3. **Learn** — the firmware compares commanded vs. actual movement, computes a new steps-per-degree ratio, and blends it into the running average (10% EWMA smoothing per observation). If the ratio drifts significantly, it's saved to EEPROM.

The MPU operates in **observe-only mode**: it learns, but never corrects. Earlier firmware versions (v14.67b and before) ran an active correction loop with up to 5 micro-corrections per jog — this added 1–4 seconds of overhead and created complex interactions with TPPA's own convergence loop. Removing the corrections and letting TPPA handle convergence via plate-solving proved both faster and more accurate.

Learning is skipped on direction reversals (backlash would corrupt the measurement). Over a typical TPPA session (6–8 ALT jogs), the ratio converges within 2–3 observations.

**Layer 3 — GRBL Protocol Synchronization**

Taming N.I.N.A.'s strict GRBL parser required several tricks:

- **Silent boot**: The ESP32 emits an unavoidable boot message on USB connect (`ets Jul 29 2019...`). Stefan Berg patched the TPPA plugin to discard pre-`?` lines — the firmware stays completely silent until asked.
- **Status report scaling**: During MPU observation phases, the reported ALT position is held slightly below target (scaled to ~90% of actual progress). This prevents TPPA from reclaiming control prematurely. When the observation completes, the position snaps to the exact commanded value with an `<Idle>` state.
- **Diagnostic buffer**: All MPU data, corrections, and learning events are written silently to a 4 KB RAM buffer (`diagLog`) instead of printing to serial. N.I.N.A. never sees debug text that could confuse its regex parser. The user retrieves the full log via the `DIAG` command.
- **AZM backlash compensation**: On direction reversals, extra "dead" steps are injected to eat through the harmonic drive's elastic deformation zone. These steps move the motor but don't update the reported position — TPPA sees smooth, accurate movements.

---

## ⚠️ Before You Start: Homing is Required

**You MUST run `HOME` (or `$H`, or press the physical Home button) before launching a TPPA session.**

Without homing, the firmware doesn't know the true physical position of the ALT axis. The position counter starts at 0.0° regardless of where the tilt plate actually is, causing the software travel limits (ALT 0–5°) to clamp movements incorrectly. TPPA will loop endlessly trying to correct an error it can never reach.

The firmware enforces this: **all TPPA jog commands (`$J=`) are silently ignored until homing is completed.** The controller still replies `ok` (so TPPA doesn't hang), but no movement occurs. Check the serial log for `!BLOCKED: ... (HOME not done)`.

**What homing does:**
1. Moves ALT down until the physical limit switch triggers
2. Performs a safety pull-off (0.2°)
3. Defines this position as 0.0° (mechanical zero)
4. Tares the MPU-6500 gyroscope (defines the gravity reference)
5. Unlocks TPPA jog commands

> 💡 **Auto-recovery:** If the limit switch is already pressed at power-on, the firmware runs homing automatically.

> 💡 **Bench testing without homing:** Direct serial commands (`ALT:`, `AZM:`) and the GUI work without homing. Only TPPA's `$J=` commands are blocked.

---

## ⚡ Electronics & Wiring

Instead of a complex Arduino + shield + external driver assembly, the project leverages a **3D printer control board** repurposed for telescope work.

### The Brain: FYSETC E4 V1.0

| Spec | Value |
|------|-------|
| MCU | ESP32-WROOM-32 @ 240 MHz (dual-core, WiFi/BT) |
| Drivers | 4× TMC2209, factory-soldered, UART-addressed |
| Used channels | MOT-X (Azimuth) + MOT-Y (Altitude) |
| Power input | 12 V DC (24 V supported but 12 V is quieter) |

> ⚠️ **FYSETC E4 V1.0 only!** The V2.0 has a different pin mapping and is **not compatible** without firmware changes.

### The Sensor: MPU-6500 (I2C) & The SD Card Hack

The gyroscope/accelerometer acts as a **digital plumb bob** for the Altitude axis. It measures the tilt plate's absolute angle using Earth's gravity vector — no external reference needed.

> **How we connected it (The Hack):** The FYSETC E4 V1.0 board is a 3D printer controller and does not have a dedicated, easily accessible I2C expansion header. To connect the MPU-6500 cleanly, we **hijacked the onboard microSD card reader**. By entirely bypassing the SD card feature (our firmware uses the ESP32's internal EEPROM and RAM for data storage instead), we repurposed its SPI pins to act as our I2C bus:
> - **GPIO 18** (originally the SD Card `SCK` clock pin) becomes the I2C `SCL` line.
> - **GPIO 19** (originally the SD Card `MISO` data pin) becomes the I2C `SDA` line.

| Wire Color | Signal | ESP32 GPIO | FYSETC E4 Target |
|:----------:|--------|:----------:|------------------|
| 🔴 Red     | VCC (3.3 V) | —        | 3.3V Header      |
| 🟡 Yellow  | GND    | —          | GND Header       |
| 🔵 Blue    | SCL (I2C) | GPIO 18 | SD Card `SCK` pin  |
| 🟢 Green   | SDA (I2C) | GPIO 19 | SD Card `MISO` pin |

> ⚠️ **EMI warning:** Keep the I2C wires (blue/green) as far as possible from stepper motor cables. Twist the GND wire around the I2C lines to act as a shield. The firmware detects I2C failures and reports them via `DIAG`.

### Motors: NEMA 17 + TMC2209

| Axis | E4 Port | STEP | DIR | Mode | Microstepping | Current | Rationale |
|------|---------|:----:|:---:|------|:-------------:|:-------:|-----------|
| **AZM** | MOT-X | GPIO 27 | GPIO 26 | StealthChop | 16 | 600 mA | Silent operation, harmonic drive is easy to turn |
| **ALT** | MOT-Y | GPIO 33 | GPIO 32 | SpreadCycle | 4 | 300 mA | Maximum torque for lifting payload through worm gear |

> The ALT current is deliberately low (300 mA) to prevent overheating inside the compact UMOT worm gearbox housing. Even at this level, the torque margin is ≥23× at 30:1 ratio. See [Motor Thermal Management](#-motor-thermal-management) for details.

### Safety Inputs

| Function | E4 Silk | GPIO | Type | Purpose |
|----------|---------|:----:|------|---------|
| **Limit Switch** | Z-MIN | 34 | Input only | Physical endstop at ALT bottom-of-travel. Triggers emergency stop + pull-off if hit outside homing. |
| **Home Button** | Y-MIN | 35 | Input only | Manual push-button to trigger full homing + gyroscope tare sequence. |

### UART Jumpers (Critical!)

The TMC2209 drivers communicate with the ESP32 via a shared UART bus. **You must place two jumper caps** on the TXD/RXD header to enable this communication. Without them, the motors won't respond. See the [FYSETC E4 Wiki](https://wiki.fysetc.com/docs/E4) for jumper placement details.

### Full GPIO Map

| Signal | Axis / Role | ESP32 GPIO | E4 silkscreen |
|--------|-------------|:----------:|---------------|
| STEP | AZM | 27 | MOT‑X |
| DIR | AZM | 26 | MOT‑X |
| EN | Both | 25 | /ENABLE (Active LOW) |
| UART | AZM & ALT | 21 | Shared Bus (Addr 1 = AZM, Addr 2 = ALT) |
| STEP | ALT | 33 | MOT‑Y |
| DIR | ALT | 32 | MOT‑Y |
| SCL | Gyroscope | 18 | SD Card `SCK` |
| SDA | Gyroscope | 19 | SD Card `MISO` |
| SENSOR | ALT Limit | 34 | Z-MIN |
| BUTTON | Manual Home | 35 | Y-MIN |

---

## ⚖️ Payload Rating

This mount is designed for **heavy-duty astrophotography setups**. The operating range for the ALT axis is intentionally small (0–5°): the equatorial mount should be set to roughly your site latitude minus 1–2°, so the PA mount only needs fine corrections.

| Rating | Max Payload | Notes |
|--------|-------------|-------|
| **Recommended** | **20 kg** | Safe for all builders, ~2× margin. The author's own [testing setup](#-authors-testing-setup) sits right at this limit. |
| **Advanced** | **25 kg** | Requires centered payload and careful assembly. |

**Weakest link analysis:**

The limiting factor is the **igus PRT-02 LC J4 orientation ring** (azimuth bearing). Its tilting moment capacity (eccentric off-axis load) is not published for the LC variant — this is why we cap the recommended payload at 20 kg with a safety margin.

| Component | Capacity | Actual Load | Margin |
|-----------|----------|-------------|--------|
| igus PRT-02 – Axial dynamic | 4,000 N (~408 kg) | ~245 N | 16× |
| igus PRT-02 – Radial dynamic | 500 N (~51 kg) | ~50 N | 10× |
| igus PRT-02 – Tilting moment | Unknown (LC variant) | ~25–37 Nm est. | ⚠️ **Assumed weakest** |
| T8×2mm lead screw | 500–1000 N | ~25–40 N | 15–25× |
| UMOT worm gearbox | ≥2 Nm output | ~0.25 Nm required | 8–16× |
| 15180 aluminum profiles | >5 kN bending | <250 N | 20×+ |

> **Builder's note:** All 3D-printed parts (motor cradles, sensor brackets, enclosures) are **non-structural** and carry only the weight of their respective components. The telescope payload is transmitted entirely through metal: tilt plate → lead screw → 15180 profiles → igus bearing → tripod.

---

## 🔧 ALT Motor: Speed vs Torque (UMOT Ratio)

The ALT axis uses a NEMA 17 + UMOT worm gearbox driving a T8×2mm lead screw through a crank-arm mechanism. Total gear ratio ≈ UMOT ratio × 5.

| UMOT Ratio | Time for 1° | Torque Margin | Self-Locking | Status |
|:----------:|:-----------:|:-------------:|:------------:|--------|
| **100:1** | 6.3 s | 80× | ✅ Worm + screw | Current prototype — safe but slow |
| **50:1** | 3.1 s | 40× | ✅ Worm + screw | Conservative, 2× faster |
| **30:1** | 1.9 s | 23× | ⚠️ Screw only | **Best balance** *(on order)* |
| **17:1** | 1.1 s | 13× | ❌ Screw only | Fast but risky in cold weather |

> ⚠️ The UMOT 30:1 has been ordered but **not yet tested**. The current prototype uses 100:1. This section will be updated with field results.

> **Safety note:** The T8×2mm lead screw is always self-locking (helix angle 4° < friction angle ~8.5°). The telescope cannot back-drive under any circumstance, even if the worm gear loses self-locking at lower ratios.

**Firmware change:** Only one constant: `constexpr float ALT_MOTOR_GEARBOX = 30.0f;` — then erase EEPROM (the old learned ratio is invalid). The firmware recalibrates automatically within 2–3 movements.

---

## 🌡️ Motor Thermal Management

The ALT stepper receives holding current even when stationary. Inside the compact UMOT housing:

| RMS Current | Power | Temperature | PLA-safe? |
|:-----------:|:-----:|:-----------:|:---------:|
| 800 mA (old) | ~1.6 W | 55–65°C | ❌ |
| **300 mA (default)** | ~0.2 W | Barely warm | ✅ |
| 400 mA (cold weather) | ~0.4 W | ~35°C | ✅ |

> Use **PETG** or **ABS** for the ALT motor cradle if running above 300 mA. At 300 mA, standard PLA is fine.

---

## ⚙️ Arduino IDE Setup

1. **Install ESP32 core** — Boards Manager → *esp32* (≥ v2.0.17)
   ```
   Preferences → Additional Board URLs:
   [https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json](https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json)
   ```

2. **Install TMCStepper library** via Library Manager

3. **Board settings:**

   | Option | Value |
   |--------|-------|
   | Board | ESP32 Dev Module |
   | CPU Freq | 240 MHz |
   | Flash Size | 4 MB |
   | Partition | Huge APP (3 MB / 1 MB SPIFFS) |
   | Upload speed | 115200 bps |

4. **Upload** — the Arduino code is in the **Arduino code** directory.

> On boot, the serial monitor is silent for ~1 second (Silent Boot). Send `?` to wake it up.

---

## 🧪 Serial Command Reference

Use the **Arduino IDE Serial Monitor** or the **PolarAlign Controller GUI** (115200 baud, Newline line ending).

### Direct Commands (bench testing, degrees)

| Command | Action |
|---------|--------|
| `HOME` / `$H` | **Homing + Tare** — required before TPPA. |
| `DIAG` / `MPU?` | **Full diagnostic** — sensor status, positions, ML ratio, command log. |
| `ALT:2.5` | Move ALT to 2.5° (absolute). |
| `AZM:5.0` | Move AZM to 5.0° (absolute, from power-on zero). |
| `AZM:ZERO` | Redefine current AZM position as 0.0°. |
| `RST` | Soft reset — abort motion, clear log. |

### GRBL Protocol (used by N.I.N.A / TPPA, arcminutes)

| Command | Meaning |
|---------|---------|
| `$J=G53X+300.00F400` | Absolute jog: AZM to +300' (= 5.0°) |
| `$J=G91G21Y-390.00F300` | Relative jog: ALT –390' (= –6.5°) |
| `?` | Status poll → `<Idle\|MPos:x,y,0\|>` |
| `!` / `~` | Feed-Hold / Resume |

> 💡 MPos values are in **arcminutes**. With TPPA Gear Ratio = `1.0`, they map directly.

> ⚠️ **TPPA Free Field** sends absolute commands (G53) — the value is a target position, not a delta. Preset buttons (±1, ±5) send relative commands (G91). During auto-alignment, everything is relative.

---

## 🛠️ Configuration Knobs

Edit in the Arduino code or use the **Firmware Config tab** in the GUI (generates copy-pasteable code):

```cpp
/* ───── HARDWARE SETTINGS ───── */
constexpr float MOTOR_FULL_STEPS = 200.0f;    // 1.8° motor
constexpr uint16_t MICROSTEPPING_AZM = 16;    // StealthChop
constexpr uint16_t MICROSTEPPING_ALT = 4;     // SpreadCycle
constexpr float GEAR_RATIO_AZM = 100.0f;      // Harmonic drive
constexpr float ALT_MOTOR_GEARBOX = 496.0f;   // UMOT ratio × crank
constexpr float ALT_SCREW_PITCH_MM = 2.0f;    // T8 lead screw
constexpr float ALT_RADIUS_MM = 60.0f;        // Pivot-to-screw distance

constexpr bool AXIS_REV_AZM = true;           // Reverse if needed
constexpr bool AXIS_REV_ALT = true;

constexpr uint16_t RMS_CURRENT_AZM = 600;     // mA
constexpr uint16_t RMS_CURRENT_ALT = 300;     // mA (≤400 for UMOT thermal)

/* ───── TRAVEL LIMITS (degrees) ───── */
constexpr float AZM_LIMIT_NEG = -30.0f;
constexpr float AZM_LIMIT_POS =  30.0f;
constexpr float ALT_LIMIT_NEG =   0.0f;
constexpr float ALT_LIMIT_POS =   5.0f;

/* ───── MPU LEARNING ───── */
constexpr float ALT_TOLERANCE_DEG = 0.05f;       // Min move for observation
constexpr float MIN_LEARNING_ANGLE = 0.5f;       // Min move for ML update
constexpr float LEARNING_SMOOTHING = 0.10f;      // EWMA weight (10%)
constexpr unsigned long GLOBAL_SETTLE_MS = 2000;  // Anti-vibration delay
```

---

## 📄 License

**MIT License** — do whatever you want, just keep the header.

---

## 🙏 Acknowledgements

* **Stefan Berg** – author of the **Three-Point Polar Alignment** plug-in and core N.I.N.A. contributor; his protocol docs and DLL patches made this project possible.
* **Avalon Instruments** – for the idea of a lean, GRBL-style alignment controller.
* **Claude** (Anthropic) & **Gemini** (Google) – for the non-blocking engine architecture, the gyroscopic ML system, the GRBL protocol reverse-engineering, and months of hardcore debugging.
* Maintained by **Antonino Nicoletti** ([antonino.antispam@free.fr]) – *clear skies!*
