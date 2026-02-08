# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL)

Minimal **GRBL‑style** firmware + hardware recipe for driving a two‑axis (Azimuth & Altitude) mount during polar‑alignment routines such as **TPPA** in **N.I.N.A.**

It runs on the *FYSETC E4 V1.0* (ESP32 + dual TMC2209) and emulates the "Avalon" protocol using a **non‑blocking motion engine** and a **strict request‑reply** serial architecture.

> **TL;DR** – Flash the sketch, wire the motors, set N.I.N.A. to talk to a **"Avalon Polar Alignment"**, **leave the TPPA “Gear Ratio” field at `1.0`**, and the routine will move your mount by up to **± 15 °**.

---

## 📦 Key Features

| Area            | What you get                                                                 |
|-----------------|------------------------------------------------------------------------------|
| **Silent Boot** | **Zero** serial output on boot to prevent connection timeouts (TPPA handshake fix) |
| **Motion Engine**| **Non-blocking** pulse generation allowing real-time `?` query responses |
| **Zero Lag** | Strict polling architecture: eliminates buffer desynchronization (fixes "0.1° vs 1.1°" display lag) |
| **Precision** | Target snapping ensures exact final coordinates (no floating-point drift) |
| **Driver layer**| UART control of two **TMC2209** drivers (StealthChop AZM / SpreadCycle ALT) |
| **Protocol** | ✔ Immediate `ok` on `$J=`<br>✔ GRBL status frames (`<Idle|…|` / `<Run|…|`)<br>✔ Feed‑Hold `!` / Cycle‑Start `~` |
| **Safety** | Software limits, Homing sensor support, Emergency Stop |
| **Hardware** | Single FYSETC E4 board – WiFi-capable, integrated drivers |

---

## 🖥️ Demo

First functional prototype: <https://d.pr/v/Lk6GNp>

---

## 🔩 Hardware Overview

See **[`HARDWARE.md`](./HARDWARE.md)** for full assembly photos and wiring diagrams.

> ⚠️ **IMPORTANT WARNING**
> **Please note that I have a major issue with the holding torque on the ALT axis in the current mechanical design.**
> **I will post a revised design soon. DO NOT FOLLOW the current mechanical design blindly!!!**

| Part | Notes |
|------|-------|
| **FYSETC E4 V1.0** | ESP32‑WROOM‑32, 4 × on‑board TMC2209 – we use two of them (MOT‑X = Azimuth, MOT‑Y = Altitude) |
| **Stepper motors** | 1.8 ° NEMA‑17 recommended (e.g. 17HS19‑2004S1) |
| **Supply** | 12 V DC (quiet) — 24 V also works if your mechanics can take it |
| **USB cable** | USB‑C → host PC |

### Default GPIO Map (Firmware v12.00)

| Signal   | Axis | ESP32 GPIO | E4 silkscreen | Notes |
|----------|------|-----------|---------------|-------|
| STEP     | AZM  | 27        | **MOT‑X** | |
| DIR      | AZM  | 26        | **MOT‑X** | |
| EN       | Both | 25        | `/ENABLE` | Active LOW |
| UART RX  | AZM  | 21        | Shared Bus | **Set Addr 1 via jumpers** |
| STEP     | ALT  | 33        | **MOT‑Y** | |
| DIR      | ALT  | 32        | **MOT‑Y** | |
| UART RX  | ALT  | 21        | Shared Bus | **Set Addr 2 via jumpers** |
| SENSOR   | ALT  | 34        | Z-MIN | Limit Switch (Input Only) |
| BUTTON   | HOME | 35        | Y-MIN | Manual Home (Input Only) |

> **Note:** The UART pins (RX=21, TX=22) are shared. You **must** set the MS1/MS2 jumpers under the drivers to assign unique addresses (AZM=1, ALT=2).

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
   > **Note:** On boot, the serial monitor will be empty (Silent Boot). Send `?` to wake it up.

---

## 🧪 Serial Command Reference

### 1️⃣ GRBL‑Style (**used by TPPA**)

| Command                  | Meaning | Response |
|--------------------------|---------|----------|
| `$J=G53X+5.00F400`       | Absolute jog **+5 °** on **Azimuth** | `ok` |
| `$J=G91G21Y-6.50F300`    | Relative jog **–6.5 °** on **Altitude** | `ok` |
| `?`                      | Poll Status | `<Idle\|MPos:…\|>` + `\n` |
| `$X`                     | Unlock | `ok` |
| `!` / `~`                | Feed‑Hold / Resume | `ok` |
| `RST`                    | Soft Reset | `Grbl 1.1h...` |

> **Tip:** Keep **“Gear Ratio” = 1.0** in the TPPA settings; the firmware already includes all mechanical reductions.

### 2️⃣ Legacy Console (for manual USB testing)

| Command      | Action |
|--------------|--------|
| `ALT:+2.5`   | Jog Altitude +2.5 ° |
| `HOME`       | Trigger Homing Sequence |

---

## 🛠️ Configuration Knobs

Open **`polar-align-controller.ino`** to adjust mechanical settings if your build differs:

```cpp
/* ───── HARDWARE SETTINGS ───── */
constexpr float MOTOR_FULL_STEPS = 200.0f;
constexpr uint16_t MICROSTEPPING_AZM = 16; // StealthChop
constexpr uint16_t MICROSTEPPING_ALT = 4;  // SpreadCycle (Torque)

// Gear Ratios (Calibrated)
constexpr float GEAR_RATIO_AZM = 100.0f;
constexpr float ALT_MOTOR_GEARBOX = 496.0f;
```

---

## 🛣 Roadmap

* [x] **Silent Boot** to fix TPPA connection timeout.
* [x] **Non-blocking motion engine** for zero-lag display.
* [ ] Trapezoidal acceleration (currently fixed speed).
* [ ] Low‑current sleep (`M18`) when idle.
* [ ] Web Interface (WiFi) for manual adjustment without USB.

Pull requests welcome!

---

## 📄 License

**MIT License** — do whatever you want, just keep the header.

---

## 🙏 Acknowledgements

* **Stefan Berg** – author of the **Three-Point Polar Alignment** plug-in and core N.I.N.A. contributor; his support was key to cracking the handshake protocol.
* **Avalon Instruments** – for the idea of a lean, GRBL-style alignment controller.
* **Claude** & **Gemini** (AI) – for the non-blocking engine architecture & debugging.
* Maintained by **Antonino Nicoletti** ([antonino.antispam@free.fr]) – *clear skies!*
