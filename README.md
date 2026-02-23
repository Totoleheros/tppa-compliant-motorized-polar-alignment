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
