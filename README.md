# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL / MPU-6500)

Welcome to what is likely **the world's first motorized Polar Alignment mount featuring an active Gyroscopic Feedback Loop with Machine Learning ratio adaptation.**

This is a minimal **GRBL‑style** firmware + hardware recipe designed to drive a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.**

It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the "Avalon" protocol using a **non‑blocking motion engine**.

> **TL;DR** – Flash the sketch, wire the motors and the MPU-6500 gyroscope, set N.I.N.A. to talk to an **"Avalon Polar Alignment"**, **leave the TPPA “Gear Ratio” field at `1.0`**, and let the mount physically correct its own mechanical flaws in real-time.

---

## 🌟 Groundbreaking Features

| Feature | How it changes the game |
|---------|-------------------------|
| 🧠 **Machine Learning Ratio** | The firmware measures the *real* physical movement of the tilt-plate using the gyroscope and compares it to the theoretical motor steps. It **automatically calculates and saves a new perfect gear ratio** in its EEPROM after every movement. |
| 🎯 **Active Feedback Loop** | Tired of mechanical backlash or friction ruining your polar alignment? The MPU-6500 acts as a digital plumb bob. If the mount didn't move enough due to backlash, the ESP32 knows it instantly and triggers a micro-correction until the exact requested angle is reached. |
| 🛡️ **Anti-Crash & Auto-Recovery** | **Software Endstops** prevent the mount from diving below 0.0°. If you power on the mount while it's already resting on its physical limit switch, the system detects it and **automatically runs a homing/pull-off sequence at boot** to free itself safely. |
| 📉 **Smart EMI / Friction Alarms** | The feedback loop is smart: if the I2C bus crashes due to EMI, or if the tilt-plate gets mechanically stuck (friction), the firmware will instantly abort the movement to prevent motor runaway or hardware damage, throwing a clear text alarm. |
| 🔇 **Silent Boot** | **Zero** serial output on boot to prevent connection timeouts (TPPA handshake fix for N.I.N.A). |
| ⚡ **Zero Lag Engine** | Strict polling architecture: non-blocking pulse generation eliminates buffer desynchronization (fixes the famous "0.1° vs 1.1°" display lag in N.I.N.A). |

---

## 🖥️ Demo

First functional prototype: To come

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
| **Supply** | 12 V DC (quiet) — 24 V also works if your mechanics can take it |

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

2. **Install library**
   * **TMCStepper** (latest version) via Library Manager.

3. **Board menu settings**

   | Option             | Value |
   |--------------------|-------|
   | Board              | **ESP32 Dev Module** |
   | CPU Freq           | 240 MHz (WiFi/BT) |
   | Flash Freq / Mode  | 80 MHz / DIO |
   | Flash Size         | 4 MB |
   | Partition Scheme   | Huge APP (3 MB / 1 MB SPIFFS) |
   | Upload speed       | 115 200 bps |
   | Port               | `COMx` / `/dev/tty.usbmodem…` |

4. **Upload**
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
| `RST`        | **Soft Reset:** Instantly aborts any motion, clears the learning queues, and resets the state machine. |

### 2️⃣ GRBL‑Style (Used by N.I.N.A / TPPA)

This is the hidden language your mount uses to talk to astrophotography software.

| Command                  | Meaning | Response |
|--------------------------|---------|----------|
| `$J=G53X+5.00F400`       | Absolute jog **+5.00 °** on **Azimuth** | `ok` |
| `$J=G91G21Y-6.50F300`    | Relative jog **–6.50 °** on **Altitude** | `ok` |
| `?`                      | Poll Status (used 10 times a second by N.I.N.A) | `<Idle\|MPos:…\|>` + `\n` |
| `!` / `~`                | Feed‑Hold / Cycle-Resume | `ok` |

> **Pro-Tip:** Keep **“Gear Ratio” = 1.0** in the TPPA settings; the firmware already includes all mechanical reductions and self-corrects the ratio anyway!

---

---

## 🛠️ Configuration Knobs

Open **`PolarAlignVX.ino`** to adjust the physical properties of your specific build. Since every DIY mount is different, you might need to tweak these values before compiling:

```cpp
/* ───── HARDWARE SETTINGS (Immutable physical properties) ───── */
constexpr float MOTOR_FULL_STEPS = 200.0f; // Standard 1.8° NEMA 17
constexpr uint16_t MICROSTEPPING_AZM = 16; // StealthChop for smooth Azimuth
constexpr uint16_t MICROSTEPPING_ALT = 4;  // SpreadCycle for high torque on Altitude

// Gear Ratios (Theoretical starting points)
// Note: The firmware will dynamically adjust the ALT ratio and save it to EEPROM.
constexpr float GEAR_RATIO_AZM = 100.0f;   // Harmonic drive ratio
constexpr float ALT_MOTOR_GEARBOX = 496.0f;// Worm gearbox ratio
constexpr float ALT_SCREW_PITCH_MM = 2.0f; // Lead screw pitch (T8 usually 2mm or 8mm)
constexpr float ALT_RADIUS_MM = 60.0f;     // Distance from pivot axis to the lead screw

// Axis Directions
// -> Change to 'false' if your mount moves in the wrong direction!
constexpr bool AXIS_REV_AZM = true;        
constexpr bool AXIS_REV_ALT = true;        

// Motor Currents (in mA)
// -> Lower these values if your motors get too hot to touch.
constexpr uint16_t RMS_CURRENT_AZM = 600;  
constexpr uint16_t RMS_CURRENT_ALT = 800;  

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
