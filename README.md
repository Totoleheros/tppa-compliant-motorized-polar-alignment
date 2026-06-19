# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL / MPU-6500)

Welcome to what is likely **the world's first motorized Polar Alignment mount featuring an active Gyroscopic Machine Learning ratio adaptation.**

A minimal **GRBL‑style** firmware + hardware platform designed to drive a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.** It sits **between the tripod and the equatorial mount**, adding motorized azimuth and altitude correction capability to any existing setup. It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the Avalon UPAS protocol using a **non‑blocking motion engine**.

> 🏆 **Field-tested: < 0.2 arcminute polar alignment error** in ~15 iterations, 20 kg payload. **In routine use, aim for < 1 arcmin** — TPPA's adaptive controller can destabilize convergence below that threshold under average seeing.

---

## 🔀 Two Hardware Versions, One Firmware

This project supports two hardware configurations. A **single firmware binary** handles both — you select the profile once at first boot via the serial monitor. No recompile needed.

| | **Prototype** | **V2** |
|---|---|---|
| ALT axis | Commercial tilt plate + T8 lead screw | Custom CNC ALT bielle mechanism |
| AZM bearing | igus PRT-02 LC J4 slewing ring | RU42 crossed roller bearing |
| Base | Monolithic 15180 aluminium profiles | Two-piece CNC aluminium plates |
| Firmware profile | `1` — PROTO | `2` — V2 |
| `ALT_MOTOR_GEARBOX` | 148.8 (UMOT 30:1 × 4.96 crank) | ~124 (initial estimate — ML converges within 2–3 jogs) |
| ALT travel | −2° to +10° (mechanical limit) | −2° to +10° (mechanical limit) |
| ALT home / limit switch | Physical limit switch at **0°** | Physical limit switch at **−2°** |
| AZM travel | ±30° (firmware limit) | ±30° (firmware limit) |
| Status | ✅ **Field-validated** | 🔧 **Delivered & under testing** |
| Estimated cost | ~**490€** | ~**586€** |
| Hardware docs | [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md) | [`HARDWARE_V2.md`](./HARDWARE_V2.md) |

> 💡 First build? **Start with the Prototype.** Off-the-shelf parts, validated to < 0.2 arcmin under 20 kg, identical firmware.

### ⚖️ Payload

| | Prototype | V2 |
|--|:---------:|:--:|
| Recommended | **20 kg** | **25 kg** |
| Advanced | 25 kg | 30+ kg |

> 💡 **Counterweights lower the effective CG.** A 32 kg setup (mount + scope + 8 kg counterweights) applies mechanically closer to a 20 kg unbalanced load on the ALT mechanism — a net positive for safety margins. Always budget for the **full assembly weight**, not scope alone.

---

## Why It Works

Most motorized polar alignment projects stop at "move a motor when TPPA says so." This one goes further on three fronts:

**🧠 It learns.** After every ALT move, the MPU-6500 gyroscope measures the real physical movement and silently refines the steps-per-degree ratio (EWMA, saved to EEPROM). The AZM axis infers its own ratio from TPPA's correction residuals — no sensor needed. The mount gets more accurate with every session.

**⚡ It never blocks.** The entire firmware is a non-blocking state machine. Motor pulses, gyroscope sampling, settle timers, and serial communication are all interleaved — N.I.N.A. polls status 10×/second and never gets a timeout. No `delay()` anywhere in the motion path.

**🔬 It understands TPPA.** The firmware knows that TPPA's `GearRatio` is not a physical gear ratio but a software scaling multiplier — a lever for balancing convergence speed against final precision. The `Speed` parameter is capped by the firmware's cruise step interval (`RAMP_CRUISE_ALT_US = 150 µs` → ~462 arcmin/min on ALT); values above this have no effect without a firmware change. TPPA's adaptive controller (`AutomatedAdjustmentController`) builds a 2×2 response matrix and resets it when any corrective move worsens total error by more than 5% — which means backlash on direction reversals can indefinitely stall convergence.

| Feature | Detail |
|---------|--------|
| 🛡️ **Homing guard + DTR persistence** | TPPA jogs blocked until homing completes; homing state survives GUI reconnect |
| 🔘 **Physical HOME button + ALT limit switch** | Button triggers full homing sequence; limit switch defines mechanical zero (0° on Prototype, −2° on V2) |
| ⏱️ **Optimised timing** | RAMP_LENGTH = 500 steps, GLOBAL_SETTLE = 500 ms — inside N.I.N.A.'s 7 s timeout |
| 📐 **Arcminute protocol** | TPPA jogs (arcmin) ↔ internal degrees ↔ MPos reports (arcmin) — transparent to TPPA |

---

## 🎬 See It In Action

| # | Video | What you'll see |
|---|-------|-----------------|
| 1 | [First Test with Full Payload](https://youtu.be/girvoCZ_UCE) | First motorized movements under real load. |
| 2 | [Homing Sequence](https://youtu.be/NkoLJ03FSSY) | Live serial: homing, limit switch, MPU-6500 tare. |
| 3 | [**TPPA Session — Below 0.2 Arcminute!**](https://youtu.be/gfE6sZmrzuw) | Complete TPPA run converging to < 0.2' in real-time. |
| 4 | [V2 ALT Bielle — Fusion 360 Simulation](https://youtu.be/YnkVJ2hzqB0) | Full −2° to +10° travel, pivot geometry, T8 drive. |
| 5 | [V2 in the Real World — First Stress Test](https://youtube.com/shorts/aVQfPjl87hA) | V2 CNC assembly under full load. |
| 6 | [**V2 Field Demo — TPPA Full Auto Session**](https://youtu.be/jZpT88h_pEg) | Complete TPPA automated polar alignment session on the V2 hardware in N.I.N.A. |
| 7 | [**V2 PA Session + Observation Session**](https://youtu.be/C0Mx80t3AEk) | PA session starting from a few arcmin error, followed by a full observation session — Askar APO 140 + reducer, 80 mm guide scope, SkyEye 62M, 7-position filter wheel, DIY rotator, ClearSky ST25 Pro. PHD2 guiding performance visible throughout. |

---

## I — Hardware

Full wiring, part references, mechanical assembly and CNC fabrication details are in the hardware docs:

- **Prototype:** [`HARDWARE_Prototype.md`](./HARDWARE_Prototype.md) — tilt plate, igus bearing, FYSETC E4 wiring, MPU-6500 SD card hack, UMOT 30:1
- **V2:** [`HARDWARE_V2.md`](./HARDWARE_V2.md) — CNC bielle mechanism, RU42 crossed roller bearing, Mean Well LRS-100-12 PSU

---

## II — Software

### Flash & Profile Selection

**1. Arduino IDE setup**

| Option | Value |
|--------|-------|
| Board | ESP32 Dev Module |
| CPU Freq | 240 MHz |
| Partition | **Huge APP (3 MB / 1 MB SPIFFS)** |
| **Erase All Flash** | **Disabled** ← critical for NVS persistence |

Install the *esp32* core (≥ v2.0.17) and the *TMCStepper* library, then flash `Arduino code/PolarAlign_auto.ino`.

**2. Profile selection at first boot**

Open Serial Monitor (115200 baud) — the firmware displays:

```
+---------------------------------------------------+
|  HARDWARE PROFILE NOT SET                         |
|  Send '1'  -> PROTO  (commercial tilt plate)      |
|  Send '2'  -> V2     (CNC + RU42 — v15.03g-auto-p4) |
+---------------------------------------------------+
```

Send `1` or `2`. The profile is saved to NVS and survives all subsequent reflashes, **provided** `Tools → Erase All Flash Before Upload` = **Disabled**.

To change later: `PROFILE:RESET` — To verify: `PROFILEINFO`

---

### The GUI

`GUI/PolarAlignGUI_v15_03g_V2.py` controls the mount without N.I.N.A. — essential for bench testing, pre-alignment, and diagnostics.

**Run from source (Windows / macOS / Linux):**
```bash
pip3 install pyserial
python3 GUI/PolarAlignGUI_v15_03g_V2.py
```
> No pre-built executable. Python 3.8+ and pyserial are the only dependencies.

**Key panels:**
- **Jog controls** — AZM (West/East) and ALT (Up/Down) in arcminutes and arcseconds, with degree readout
- **Absolute positioning** — Go to any angle directly
- **Live position + Learning Monitor** — real-time AZM/ALT position, MPU error, learned ratios
- **System commands** — HOME, DIAG, RST, AZM:ZERO in one click
- **Raw serial console** — send any command, see full log
- **Firmware Config tab** — edit hardware constants and generate ready-to-paste Arduino code

---

## III — In the Field

### Step 1 — Home

**Run `HOME` before every session.** Without homing, TPPA jogs are silently blocked (firmware replies `ok` but doesn't move).

What homing does: moves ALT down to the physical limit switch → safety pull-off (0.2°) → defines mechanical zero → tares the MPU-6500 → saves state to EEPROM → unlocks TPPA jogs.

> 💡 **Auto-recovery:** If the limit switch is already pressed at power-on, the firmware runs homing automatically.

---

### Step 2 — Pre-align with the GUI

Before launching TPPA in full auto mode, **use the GUI to get within ~1° of true polar alignment.** This pays dividends:

- TPPA's adaptive controller converges much faster from < 1° than from 3–5°
- Reduces the risk of hitting firmware travel limits (`AZM ±30°`, `ALT −2°/+10°`) mid-session
- Allows you to verify motor directions before handing control to TPPA

**Procedure:**
1. Connect the GUI, run `HOME`
2. **Set your EQ mount's latitude to your actual latitude minus ~1°.** This is important: the PA platform has only **−2° of downward ALT correction range** (V2) or 0° (Prototype, which homes at 0°). If your mount is set too high (ALT above your true latitude), TPPA will need to correct downward — and may hit the **mechanical travel limit** before converging. Setting the mount slightly low gives TPPA room to correct in both directions.
3. **Close the GUI**, then open TPPA in N.I.N.A. Go to **Options → Settings** (back-office) and run the connection test. It will fail the first time — this is a known bug. Run it a second time — it will succeed. Then close the back-office.
4. Launch TPPA in measurement-only mode (automated adjustments **OFF**): TPPA will plate-solve and display the current AZM and ALT polar error in real time.
5. **Reconnect the GUI** and use the jog buttons to apply corrections manually — exactly as you would turn the manual adjustment screws on a traditional mount. TPPA updates the error display after each plate-solve.
6. Iterate until you're within ~1° on both axes.
7. Close the GUI again, enable automated adjustments in TPPA and launch the full auto session.

> ⚠️ **The GUI and TPPA cannot share the serial port.** Reconnecting the GUI after TPPA triggers a DTR reboot that clears the RAM diagnostic log. Always close TPPA before reconnecting the GUI, and vice versa.

---

### Step 3 — TPPA Session

#### 🚨 Plugin Settings (read this first — it will save you hours)

| Setting | Value | Why |
|---------|:-----:|-----|
| **Do automated adjustments?** | **ON** | ⚠️ If OFF, TPPA measures but sends zero commands. Motors never move. **Check this first.** |
| **Polar Alignment System** | `UPAS` | Selects the Avalon/GRBL dialect. |
| **Reverse AZM / ALT Axis** | `OFF` | Direction set by firmware profile. **Toggle here if a motor moves the wrong way.** |
| **Default Move Rate** | `10` | Factory default `3` is too slow for this hardware. |
| **Settle Time** | `3 s` | 5 s is unnecessary — firmware settle absorbs vibration first. |
| **Alignment Tolerance** | `0.2–1.0 arcmin` | See convergence strategy below. |

> 💡 **Initial error alert.** If TPPA displays *"Initial Polar Alignment error is large. Correction phase will be unreliable."*, corrections still proceed as long as moves stay within firmware travel limits. Pre-aligning with the GUI avoids this.

#### Understanding Gear Ratio — the most misunderstood TPPA setting

> ⚠️ **`GearRatio` is not a physical gear ratio.** It is a pure software scaling multiplier applied to every command TPPA sends to the firmware:
> ```
> command_sent = position_plan_units × GearRatio
> ```
> The firmware receives this value in arcminutes and moves accordingly. TPPA's internal cap is ±5 plan units per iteration, so the effective physical cap per iteration is `5 × GearRatio` arcminutes.

This creates a fundamental speed/precision tradeoff:

| GearRatio | Max move/iteration | Minimum move (deadband) | Practical use |
|:---------:|:-----------------:|:-----------------------:|---------------|
| 1 | 5 arcmin | 0.05 arcmin | Precision phase only — very slow from large errors |
| **5** | **25 arcmin** | **0.25 arcmin** | **Good all-round starting point** |
| 10 | 50 arcmin | 0.5 arcmin | Fast initial convergence, good final precision |
| 20 | 100 arcmin | 1 arcmin | Rough-in only — too coarse for final alignment |

**Recommended strategy:**
1. **Start at GearRatio = 5–10** (or higher if initial error > 2°)
2. Watch TPPA converge — it will slow down naturally as it approaches the target
3. **Never go below GearRatio = 2** — the resulting tiny physical moves make convergence unreliable under average seeing
4. For final precision (< 1 arcmin), keep GearRatio at 5 — don't chase 0.2 arcmin unless seeing is excellent

> 💡 **On the `Speed` parameter:** this maps to the GRBL `F` feed rate. ALT is physically capped at ~462 arcmin/min by `RAMP_CRUISE_ALT_US = 150 µs` — setting Speed above this value has no effect on ALT without a firmware change.

#### Convergence behaviour

TPPA's `AutomatedAdjustmentController` is a learning adaptive controller. It builds a 2×2 response matrix from observed corrections and resets the model if any corrective move worsens total error by more than 5%. When this happens, TPPA drops back to 1 arcmin probe moves and rebuilds from scratch — you'll see this as a sudden slow-down mid-session.

**What triggers a model reset:**
- T8 mechanical backlash on direction reversals (ALT axis — main culprit)
- Seeing-induced plate-solve noise above ~1 arcmin
- Firmware travel limit reached mid-move

**What helps:**
- Pre-aligning with the GUI to minimize direction reversals during TPPA
- Targeting < 1 arcmin tolerance rather than < 0.2 arcmin in average conditions
- Keeping GearRatio ≥ 2 so each move is physically meaningful

> 🔍 **Debugging "no movement":** If motors don't move during a TPPA auto session, the cause is almost always a plugin setting. Check in order: (1) *Do automated adjustments* = **ON**, (2) back-office connection test passes on second attempt, (3) *Polar Alignment System* = `UPAS`. Note that reconnecting the GUI after a TPPA session triggers a DTR reboot that clears the diagnostic log — `DIAG` will always be empty in this scenario.

> ⚠️ **Known bug — connection test always fails on first attempt.** Before launching a TPPA auto session, go to the plugin back-office (Options → Settings) and run the connection test. It will fail the first time — this is a known, unresolved issue. **Run the test a second time** — it will succeed. Then disconnect from the back-office and launch TPPA from the front-end as normal.

---

## IV — Under the Hood

### Serial Command Reference

Open the Serial Monitor (115200 baud, Newline terminator) or the GUI Raw console.

#### Profile Commands

| Command | Action |
|---------|--------|
| `PROFILEINFO` | Show active profile and all runtime cfg_ values |
| `PROFILE:RESET` | Clear profile from NVS → prompts re-selection at next boot |

#### Motion & Diagnostics

| Command | Action |
|---------|--------|
| `HOME` / `$H` | Homing sequence + MPU tare. Required before TPPA. |
| `DIAG` | Full diagnostic: positions, learned ratios, last 4 KB of command log |
| `ALT:2.5` | Move ALT to 2.5° absolute |
| `AZM:5.0` | Move AZM to 5.0° absolute |
| `AZM:ZERO` | Redefine current AZM position as 0° and reset AZM learning |
| `RST` | Soft reset — abort motion, clear log |
| `MPU` | Lightweight gyroscope query → `MPU:tared,raw` |

#### GRBL Protocol (used by N.I.N.A./TPPA)

All TPPA commands arrive in arcminutes via the GRBL dialect. The firmware converts internally.

| Command | Meaning |
|---------|---------|
| `$J=G53X+300.00F400` | Absolute jog: AZM to +300' (= 5.0°) |
| `$J=G91G21Y-390.00F300` | Relative jog: ALT −390' (= −6.5°) |
| `?` | Status poll → `<Idle\|MPos:x,y,0\|>` (MPos in arcminutes) |
| `!` / `~` | Feed-Hold / Resume |

> ⚠️ TPPA Free Field sends absolute G53 commands. Preset buttons and auto-alignment send relative G91.

#### Diagnostics in depth

The firmware maintains a 4 KB RAM diagnostic buffer (`diagLog`) invisible to N.I.N.A. Every ALT jog logs: commanded delta, MPU-measured delta, computed ratio, EWMA update, and EEPROM write decision. Every AZM learning event logs: prevDelta, currDelta, effectiveMoved, measured ratio, guard outcomes.

Retrieve with `DIAG` from the GUI console. The buffer clears on `RST` or DTR reboot — never reconnect the GUI during a TPPA session.

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

* **Stefan Berg** — author of the Three-Point Polar Alignment plugin and core N.I.N.A. contributor; his protocol docs and DLL patches made this project possible.
* **Avalon Instruments** — for the idea of a lean, GRBL-style alignment controller.
* **Claude** (Anthropic) & **Gemini** (Google) — for the non-blocking engine architecture, the gyroscopic ML system, the GRBL protocol reverse-engineering, and months of hardcore debugging.
* Maintained by **Antonino Nicoletti** — *clear skies!*
