# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL / MPU-6500)

Welcome to what is likely **the world's first motorized Polar Alignment mount featuring an active Gyroscopic Feedback Loop with Machine Learning ratio adaptation.**

This is a minimal **GRBL‑style** firmware + hardware recipe designed to drive a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.**

It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the "Avalon" protocol using a **non‑blocking motion engine**.

> **TL;DR** – Flash the sketch, wire the motors and the MPU-6500 gyroscope, set N.I.N.A. to talk to an **"Avalon Polar Alignment"**, **leave the TPPA "Gear Ratio" field at `1.0`**, and let the mount physically correct its own mechanical flaws in real-time.

---

## 🌟 Groundbreaking Features

| Feature | How it changes the game |
|---------|-------------------------|
| 🧠 **Machine Learning Ratio** | The firmware measures the *real* physical movement of the tilt-plate using the gyroscope and compares it to the theoretical motor steps. It **automatically calculates and saves a new perfect gear ratio** in its EEPROM after every movement. |
| 🎯 **Active Feedback Loop** | Tired of mechanical backlash or friction ruining your polar alignment? The MPU-6500 acts as a digital plumb bob. If the mount didn't move enough due to backlash, the ESP32 knows it instantly and triggers a micro-correction until the exact requested angle is reached. |
| 🛡️ **Anti-Crash & Auto-Recovery** | **Software Endstops** prevent the mount from diving below 0.0°. If you power on the mount while it's already resting on its physical limit switch, the system detects it and **automatically runs a homing/pull-off sequence at boot** to free itself safely. |
| 📉 **Smart EMI / Friction Alarms** | The feedback loop is smart: if the I2C bus crashes due to EMI, or if the tilt-plate gets mechanically stuck (friction), the firmware will instantly abort the movement to prevent motor runaway or hardware damage, throwing a clear text alarm. |
| 🔇 **Silent Boot** | **Zero** serial output on boot to prevent connection timeouts (TPPA handshake fix for N.I.N.A). |
| ⚡ **Zero Lag Engine** | Strict polling architecture: non-blocking trapezoidal acceleration eliminates step-loss on high inertia loads (Harmonic Drives) while maintaining perfect buffer synchronization (fixes N.I.N.A display lag). |

---

## ⚖️ Payload Rating

This mount is designed for **heavy-duty astrophotography setups**. The operating range for the ALT axis is intentionally small (0–4°): the equatorial mount sitting on top should be set to roughly your site latitude minus 1–2°, so the PA mount only needs fine corrections.

| Rating | Max Payload | Notes |
|--------|-------------|-------|
| **Recommended** | **20 kg** | Safe for all builders, with a ~2× safety margin on the weakest link. |
| **Advanced** | **25 kg** | Tested by the author. Requires centered payload and careful assembly. |

**Weakest link analysis:**

The limiting factor is **not** the motor or the gearbox — it is the **igus PRT-02 LC J4 orientation ring** (azimuth bearing). Its rated specs are generous for centered axial loads (4,000 N dynamic = ~408 kg), but the **tilting moment capacity** (eccentric load) is the real constraint for telescope setups where the center of gravity sits above and offset from the bearing.

| Component | Capacity | Status |
|-----------|----------|--------|
| igus PRT-02 LC J4 – Axial dynamic | 4,000 N (~408 kg) | ✅ Not limiting |
| igus PRT-02 LC J4 – Radial dynamic | 500 N (~51 kg) | ✅ Not limiting |
| igus PRT-02 LC J4 – Tilting moment | Not published for LC variant | ⚠️ **Weakest link** |
| T8×2mm lead screw (axial load @2° tilt) | ~500–1000 N rated | ✅ ~25–40 N actual |
| UMOT worm gearbox (any ratio) | Torque margin ≥ 7× @25 kg | ✅ Comfortable |
| 15180 aluminum profiles (crossed) | Effectively unlimited | ✅ Overkill |
| 3D-printed parts (motor cradles, sensor brackets, enclosures) | Non-structural (no telescope load) | ✅ Not in load path |

> **Builder's note:** Several parts are 3D-printed (ALT and AZM motor cradles, MPU bracket, homing sensor bracket, FYSETC enclosure, PSU case), but **none are in the telescope load path**. They carry only the weight of their respective components (motors, sensors, electronics). The telescope payload is transmitted entirely through metal: tilt plate → lead screw → crossed 15180 profiles → igus bearing → tripod. The author uses PLA+CF; PETG is recommended for the ALT motor cradle due to proximity to the UMOT housing heat.

---

## 🖥️ Demo and Images

First functional prototype: **To come...**

Check the **IMAGES directory** for 3D model images ('3D_Model') and images taken during the true assembly of the first prototype ('Real_World').

---

## 🔩 Hardware Overview

See **[`HARDWARE.md`](./HARDWARE.md)** for full assembly photos, 3D files (including a drilling jig!), and detailed wiring diagrams.

> ⚠️ **IMPORTANT WARNING**
> **Please note that this is still an ongoing open-source project!**

| Part | Notes |
|------|-------|
| **FYSETC E4 V1.0** | ESP32‑WROOM‑32, 4 × on‑board TMC2209 – we use two of them (MOT‑X = Azimuth, MOT‑Y = Altitude) |
| **MPU-6500 Module** | I2C Gyroscope/Accelerometer used for active Altitude feedback. |
| **Stepper motors** | 1.8 ° NEMA‑17 recommended (e.g. 17HS19‑2004S1) |
| **Supply** | 12 V DC (quiet) — 24 V also works if your mechanics can take it |

### Default GPIO Map (Firmware v14.x)

| Signal   | Axis / Role | ESP32 GPIO | E4 silkscreen | Notes |
|----------|-------------|------------|---------------|-------|
| STEP     | AZM         | 27         | **MOT‑X** | |
| DIR      | AZM         | 26         | **MOT‑X** | |
| EN       | Both        | 25         | `/ENABLE`     | Active LOW |
| UART RX  | AZM & ALT   | 21         | Shared Bus    | **Set Addr 1 (AZM) & Addr 2 (ALT) via jumpers** |
| STEP     | ALT         | 33         | **MOT‑Y** | |
| DIR      | ALT         | 32         | **MOT‑Y** | |
| SCL      | Gyroscope   | 18         | SCL           | I2C Clock (MPU-6500) |
| SDA      | Gyroscope   | 19         | SDA           | I2C Data (MPU-6500) |
| SENSOR   | ALT Limit   | 34         | Z-MIN         | Physical Limit Switch (Input Only) |
| BUTTON   | Manual Home | 35         | Y-MIN         | Manual Home Button (Input Only) |

---

## ⚙️ Arduino IDE Setup

1. **Install ESP32 core**
   ```text
   Preferences → Additional Board URLs:
   [https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json](https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json)
   ```
   Boards Manager → *esp32* (≥ v2.0.17).
   
2. **Arduino code**
   * Download the last version of the Arduino code in the **Arduino directory**.

3. **Install library**
   * **TMCStepper** (latest version) via Library Manager.

4. **Board menu settings**

   | Option             | Value |
   |--------------------|-------|
   | Board              | **ESP32 Dev Module** |
   | CPU Freq           | 240 MHz (WiFi/BT) |
   | Flash Freq / Mode  | 80 MHz / DIO |
   | Flash Size         | 4 MB |
   | Partition Scheme   | Huge APP (3 MB / 1 MB SPIFFS) |
   | Upload speed       | 115 200 bps |
   | Port               | `COMx` / `/dev/tty.usbmodem…` |

5. **Upload**
   Compile ⇒ Upload.
   > **Note:** On boot, the serial monitor will be completely empty for about 1 second (Silent Boot). Send `?` to wake it up or use the diagnostic commands below.

---

## 🧪 Serial Command Reference

To test your mount, open the **Arduino IDE Serial Monitor**. 
⚠️ **CRITICAL:** Set the baud rate to **`115200`** and the line ending to **`Newline`** (or `Both NL & CR`).

### 1️⃣ Custom Diagnostic & Manual Control (For Makers)

These commands were specifically built to help you test the mechanics and the Active Feedback Loop without needing N.I.N.A.

| Command      | Action |
|--------------|--------|
| `HOME` (or `$H`) | **Trigger Homing & Tare:** The mount will move down until it hits the limit switch, perform a safety pull-off, define this point as `0.0°`, and perfectly tare the MPU-6500 Gyroscope. |
| `DIAG` (or `MPU?`) | **Print System Diagnostic:** The ultimate debugging tool. It instantly prints the Limit Switch status, the Raw Gyroscope reading, the Tared physical angle, any I2C EMI errors, and the current active Gear Ratio learned from the EEPROM. |
| `ALT:2.5`    | **Absolute Altitude Jog:** Commands the mount to go to an absolute altitude of `2.5°`. Watch the Serial Monitor to see the Active Feedback Loop in action as it performs micro-corrections to reach the exact target! |
| `AZM:5.0`    | **Absolute Azimuth Jog:** Commands the mount to go to an absolute azimuth angle of `5.0°`. Note: **0.0°** is defined as the position of the mount at power-on. Positive values move East (Clockwise), negative values move West (Counter-Clockwise). |
| `AZM:ZERO`   | **Tare Azimuth:** Instantly defines the current physical position of the mount as the new absolute `0.0°` for the Azimuth axis without needing to reboot the controller. |
| `RST`        | **Soft Reset:** Instantly aborts any motion, clears the learning queues, and resets the state machine. |

### 2️⃣ GRBL‑Style (Used by N.I.N.A / TPPA)

This is the hidden language your mount uses to talk to astrophotography software.

| Command                  | Meaning | Response |
|--------------------------|---------|----------|
| `$J=G53X+5.00F400`       | Absolute jog **+5.00 °** on **Azimuth** | `ok` |
| `$J=G91G21Y-6.50F300`    | Relative jog **–6.50 °** on **Altitude** | `ok` |
| `?`                      | Poll Status (used 10 times a second by N.I.N.A) | `<Idle\|MPos:…\|>` + `\n` |
| `!` / `~`                | Feed‑Hold / Cycle-Resume | `ok` |

> 💡 **TPPA Progress Note:** During Altitude corrections, the firmware uses **linear scaling** on the reported position to prevent TPPA from prematurely reclaiming control while the gyroscope feedback loop is active. The angle display advances smoothly during the entire motor movement, but at ~90% of actual position — so when the motor reaches its target, the display shows a value of ~90% of the target angle. There is then a brief pause while the MPU-6500 measures the physical angle and applies micro-corrections, after which the position snaps to the exact commanded value with an `Idle` status and TPPA resumes plate-solving.

---

## 🔧 ALT Motor: Speed vs Torque (Choosing Your UMOT Ratio)

The ALT axis uses a NEMA 17 stepper coupled to a **UMOT worm gearbox**, which drives a T8×2mm lead screw through a crank-arm mechanism. The total gear ratio is approximately `UMOT ratio × 5` (the crank adds ~5× reduction).

Since this mount operates in a narrow 0–4° range (fine polar alignment corrections), the torque requirements are very low. This means you can trade some of the massive torque margin for **speed** by choosing a lower UMOT ratio.

### Comparison Table (for 25 kg payload at 0–4° tilt)

| UMOT Ratio | Total Effective | Time for 1° | Torque Margin | Self-Locking (worm) | Recommendation |
|------------|----------------|-------------|---------------|---------------------|----------------|
| **100:1** | ~496:1 | 6.3 s | 80× | ✅ Yes | Safe but very slow |
| **50:1** | ~248:1 | 3.1 s | 40× | ✅ Yes | Conservative, 2× faster |
| **30:1** | ~149:1 | 1.9 s | 23× | ⚠️ Borderline | **Best balance** — 3.3× faster |
| **17:1** | ~84:1 | 1.1 s | 13× | ❌ Lost | Fast but risky in cold weather |

> **Safety note:** Even if the worm gearbox loses its self-locking property at lower ratios (30:1 and below), the **T8×2mm lead screw is always self-locking** (helix angle 4° < friction angle ~8.5°). The telescope cannot back-drive through this screw under any circumstance. The axial load on the screw at 2° tilt with 25 kg is only ~25–40 N, well within the bronze nut's rated capacity of 500–1000 N.

### Author's choice: 30:1

The 30:1 ratio delivers 1.9 seconds per degree (a typical 2° adjustment completes in under 4 seconds instead of 12.6 seconds with 100:1), while maintaining a comfortable 23× torque margin even in the worst case. For a TPPA session requiring 6–8 altitude corrections, this saves 1–2 minutes of waiting per alignment run.

### Firmware changes when switching ratio

Only one constant needs to change:
```cpp
constexpr float ALT_MOTOR_GEARBOX = 30.0f;   // was 496.0f for the 100:1 + crank
```

After changing the ratio, **erase the EEPROM** (the machine-learned value from the old ratio is invalid). The firmware will automatically recalibrate within 2–3 ALT movements using MPU feedback.

---

## 🌡️ Motor Thermal Management

The ALT stepper motor receives a constant holding current even when stationary. With the default 800 mA RMS setting, this dissipates ~1.6 W inside the compact UMOT housing, reaching surface temperatures of **55–65°C** — hot to the touch but within the motor's 130°C insulation rating.

Since the torque margin is enormous (23× at 30:1, 80× at 100:1), the firmware ships with a **reduced ALT current of 300 mA**, dropping dissipation to ~0.2 W. The motor will be barely warm.

```cpp
constexpr uint16_t RMS_CURRENT_ALT = 300;   // Thermal-optimized (was 800)
```

> **For cold-weather observers:** If you operate below -10°C with thick grease, you can raise this to 400 mA for extra starting torque. Even at 400 mA, the motor stays well below 40°C.
>
> ⚠️ **Material note:** The author uses PLA+CF (carbon fiber reinforced) for all printed parts. For the **ALT motor cradle** specifically (in contact with the UMOT housing), PETG (75°C) or ABS (95°C) is recommended if you plan to run `RMS_CURRENT_ALT` above 300 mA. At the default 300 mA, PLA+CF or standard PLA is fine.

---

## 🛠️ Configuration Knobs

Open **the last version of the Arduino code** to adjust the physical properties of your specific build. Since every DIY mount is different, you might need to tweak these values before compiling:

```cpp
/* ───── HARDWARE SETTINGS (Immutable physical properties) ───── */
constexpr float MOTOR_FULL_STEPS = 200.0f; // Standard 1.8° NEMA 17
constexpr uint16_t MICROSTEPPING_AZM = 16; // StealthChop for smooth Azimuth
constexpr uint16_t MICROSTEPPING_ALT = 4;  // SpreadCycle for high torque on Altitude

// Gear Ratios (Theoretical starting points)
// Note: The firmware will dynamically adjust the ALT ratio and save it to EEPROM.
constexpr float GEAR_RATIO_AZM = 100.0f;     // Harmonic drive ratio
constexpr float ALT_MOTOR_GEARBOX = 496.0f;  // UMOT 100:1 + crank ~5×. Change to 30.0 for UMOT 30:1.
constexpr float ALT_SCREW_PITCH_MM = 2.0f;   // Lead screw pitch (T8 = 2mm per revolution)
constexpr float ALT_RADIUS_MM = 60.0f;       // Distance from pivot axis to the lead screw

// Axis Directions
// -> Change to 'false' if your mount moves in the wrong direction!
constexpr bool AXIS_REV_AZM = true;        
constexpr bool AXIS_REV_ALT = true;        

// Motor Currents (in mA)
// -> ALT is set low (300mA) to prevent overheating inside the UMOT housing.
//    Increase to 400mA only if you experience stalling in extreme cold.
//    AZM can stay at 600mA (Harmonic Drive has good ventilation).
constexpr uint16_t RMS_CURRENT_AZM = 600;  
constexpr uint16_t RMS_CURRENT_ALT = 300;  

/* ───── FEEDBACK LOOP THRESHOLDS (For advanced users) ───── */
// You can tighten or loosen the Active Feedback Loop behavior here:
constexpr float ALT_TOLERANCE_DEG = 0.05f;  // Acceptable error margin to declare a move "successful"
constexpr uint8_t MAX_CORRECTIONS = 3;      // Max attempts to fix backlash/error before giving up
```
---

## 📄 License

**MIT License** — do whatever you want, just keep the header.

---

## 🙏 Acknowledgements

* **Stefan Berg** – author of the **Three-Point Polar Alignment** plug-in and core N.I.N.A. contributor; his support was key to cracking the handshake protocol.
* **Avalon Instruments** – for the idea of a lean, GRBL-style alignment controller.
* **Claude** & **Gemini** (AI) – for the non-blocking engine architecture, the I2C Gyroscopic Feedback Loop, and the hardcore debugging.
* Maintained by **Antonino Nicoletti** ([antonino.antispam@free.fr]) – *clear skies!*
