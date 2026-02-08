# Serial Alt‑Az Polar Alignment Controller (ESP32 / GRBL)

A robust, **GRBL‑style** firmware for the **FYSETC E4 V1.0** board (ESP32 + dual TMC2209) designed to drive a custom Alt‑Az mount for the **Three Point Polar Alignment (TPPA)** plugin in **N.I.N.A.**

It emulates the "Avalon" protocol using a **non‑blocking motion engine** and a **strict request‑reply** serial architecture to ensure real‑time responsiveness and perfect synchronization with the plugin.

> **TL;DR** – Flash the firmware, wire your motors as described below, select **"Avalon Polar Alignment"** in N.I.N.A., and **leave the Gear Ratio at `1.0`**. The firmware handles all mechanical math internally.

---

## 📦 Key Features

| Feature | Description |
| :--- | :--- |
| **Silent Boot** | Prevents connection timeouts by suppressing ESP32 system logs (`ets Jul...`); waits for TPPA polling before speaking. |
| **Non‑Blocking Engine** | Allows smooth motor motion while maintaining real‑time serial communication. |
| **Zero Lag Display** | Strict polling architecture eliminates buffer desynchronization (fixes the "0.1° vs 1.1°" display lag). |
| **Precision Math** | Target snapping ensures exact final coordinates, preventing floating‑point drift errors. |
| **Hybrid Driver Config** | **StealthChop** (1/16) for precise Azimuth control; **SpreadCycle** (1/4) for high‑torque Altitude lifting. |
| **Hardware** | Single FYSETC E4 board – WiFi‑capable, integrated drivers, compact form factor. |

---

## 🔧 Hardware & Mechanics

This project is built around the **Fysetc E4 V1.0** board.

### 1. Bill of Materials (BOM)
* **Controller:** Fysetc E4 V1.0 (ESP32).
* **Drivers:** 2x TMC2209 (UART Mode).
* **Motors:** 2x NEMA 17 Stepper Motors (1.8°).
* **Sensors:**
    * 1x Microswitch (Limit Switch) for Altitude Homing.
    * 1x Momentary Push Button for manual Homing.
* **Power:** 12V - 24V DC.

### 2. Wiring & Pinout

| Component | Function | ESP32 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Azimuth (X)** | Step | GPIO **27** | |
| | Dir | GPIO **26** | |
| **Altitude (Y)** | Step | GPIO **33** | |
| | Dir | GPIO **32** | |
| **Common** | Enable | GPIO **25** | Active LOW |
| **UART Bus** | RX | GPIO **21** | Shared Bus |
| | TX | GPIO **22** | Shared Bus |
| **Sensors** | Home Switch | GPIO **34** | *Input Only* |
| | Home Button | GPIO **35** | *Input Only* |

> **⚠️ CRITICAL: TMC2209 UART Addressing**
> Since the Fysetc E4 uses a shared UART bus, you **MUST** set the jumpers under the driver sticks to assign unique addresses:
> * **Azimuth Driver:** Address **1** (MS1: VCC, MS2: GND)
> * **Altitude Driver:** Address **2** (MS1: GND, MS2: VCC)
> * *(Check TMC2209 datasheet or E4 schematic for specific jumper positions
