/*
 * ---------------------------------------------------------------
 * Polar Alignment Motor Controller – FYSETC E4 V1.0 (ESP32 + TMC2209)
 * Author: Antonino Nicoletti + ChatGPT (June 2025)
 * ---------------------------------------------------------------
 *
 * 🧩 REQUIRED LIBRARY:
 *   - TMCStepper: https://github.com/teemuatlut/TMCStepper
 *     → Install via Arduino IDE > Tools > Manage Libraries
 *
 * 🛠️ ARDUINO IDE SETTINGS:
 *   - Board:              “ESP32 Dev Module”
 *   - Port:               "/dev/cu.wchusbserial10" (may vary)
 *   - CPU Frequency:      “240 MHz (WiFi/BT)”
 *   - Flash Frequency:    “40 MHz”
 *   - Flash Mode:         “DIO”
 *   - Flash Size:         “4MB (32Mb)”
 *   - Partition Scheme:   “Huge APP (3MB No OTA/1MB SPIFFS)”
 *   - PSRAM:              “Enabled”
 *   - Core Debug Level:   “None”
 *   - Upload Speed:       “115200”
 *
 * ⚠️ FLASHING TIP:
 *   - If upload fails or ESP32 resets too quickly:
 *     → Add a 1µF–10µF capacitor between the RST and GND pins
 *       to delay auto-reset and allow flashing.
 *
 * 🔌 MOTOR CONNECTIONS:
 *   - AZM motor → MOTX port
 *     • STEP = GPIO27, DIR = GPIO26, EN = GPIO25
 *     • UART = GPIO39 (Z-MIN → PDN)
 *
 *   - ALT motor → MOTY port
 *     • STEP = GPIO33, DIR = GPIO32, EN = GPIO25 (shared)
 *     • UART = GPIO36 (Y-MIN → PDN)
 *
 * 📡 SERIAL COMMANDS (via Serial Monitor @9600 baud):
 *   - AZM:+2.5     → Move azimuth motor +2.5 degrees
 *   - AZM:-1.0     → Move azimuth motor -1.0 degrees
 *   - ALT:+0.5     → Move altitude motor +0.5 degrees
 *   - ALT:-0.2     → Move altitude motor -0.2 degrees
 *   - POS?         → Query current AZM and ALT offsets
 *   - STA?         → Alias of POS?
 *   - RST          → Reset both positions to zero
 *
 * 🔁 DEVICE RESPONSES:
 *   - OK           → Command executed successfully
 *   - BUSY         → Motor is currently executing a move
 *   - ERR:<msg>    → Invalid or out-of-bounds command
 */

#include <TMCStepper.h>

// ------------------- SYSTEM PARAMETERS TO CONFIGURE -------------------
// === EDIT THESE VALUES TO MATCH YOUR SETUP ===

// Motor step angle (commonly 200 steps/rev for 1.8° motors)
const float MOTOR_STEPS_PER_REV = 200.0;

// Microstepping (e.g. 1, 2, 4, 8, 16, 32...)
const float MICROSTEPPING = 16.0;

// Gear ratios (mechanical reduction)
const float GEAR_RATIO_AZM = 100.0;
const float GEAR_RATIO_ALT = 90.0;

// Auto-calculated steps per degree
const float STEPS_PER_DEGREE_AZM = (MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO_AZM) / 360.0;
const float STEPS_PER_DEGREE_ALT = (MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO_ALT) / 360.0;

// Movement limits (software)
const float LIMIT_MIN_AZM = -10.0;
const float LIMIT_MAX_AZM =  10.0;
const float LIMIT_MIN_ALT = -15.0;
const float LIMIT_MAX_ALT =  15.0;

// ------------------------- PIN CONFIGURATION --------------------------
// AZM motor (MOTX)
#define EN_PIN_AZM     25
#define DIR_PIN_AZM    26
#define STEP_PIN_AZM   27
#define SERIAL_RX_AZM  39  // UART RX (Z-MIN)

// ALT motor (MOTY)
#define EN_PIN_ALT     25  // Shared with AZM
#define DIR_PIN_ALT    32
#define STEP_PIN_ALT   33
#define SERIAL_RX_ALT  36  // UART RX (Y-MIN)

// TMC2209 driver config
#define DRIVER_ADDRESS 0
#define R_SENSE        0.11f

HardwareSerial AZMSerial(2); // UART2: GPIO16/17 (only RX used)
HardwareSerial ALTSerial(1); // UART1: GPIO9/10 (only RX used)

TMC2209Stepper driverAZM(&AZMSerial, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverALT(&ALTSerial, R_SENSE, DRIVER_ADDRESS);

// ----------------------------- STATE -----------------------------
float currentPosAZM = 0.0;
float currentPosALT = 0.0;

// ----------------------------- SETUP -----------------------------
void setup() {
  Serial.begin(9600);
  delay(500);

  // AZM pins
  pinMode(EN_PIN_AZM, OUTPUT);
  pinMode(DIR_PIN_AZM, OUTPUT);
  pinMode(STEP_PIN_AZM, OUTPUT);
  digitalWrite(EN_PIN_AZM, LOW);

  // ALT pins
  pinMode(DIR_PIN_ALT, OUTPUT);
  pinMode(STEP_PIN_ALT, OUTPUT);
  digitalWrite(EN_PIN_ALT, LOW);

  // UART init
  AZMSerial.begin(115200, SERIAL_8N1, SERIAL_RX_AZM, -1);
  ALTSerial.begin(115200, SERIAL_8N1, SERIAL_RX_ALT, -1);

  // Init drivers
  driverAZM.begin();
  driverAZM.pdn_disable(true);
  driverAZM.I_scale_analog(false);
  driverAZM.rms_current(500);
  driverAZM.microsteps(MICROSTEPPING);
  driverAZM.en_spreadCycle(false);

  driverALT.begin();
  driverALT.pdn_disable(true);
  driverALT.I_scale_analog(false);
  driverALT.rms_current(500);
  driverALT.microsteps(MICROSTEPPING);
  driverALT.en_spreadCycle(false);

  Serial.println("READY");
}

// ------------------------ MOVE FUNCTION ------------------------
void moveMotor(int stepPin, int dirPin, float degrees, float stepsPerDegree, float &pos) {
  long steps = round(degrees * stepsPerDegree);
  if (steps == 0) return;

  digitalWrite(dirPin, steps > 0 ? HIGH : LOW);
  steps = abs(steps);
  for (long i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(600);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(600);
  }
  pos += degrees;
}

// ----------------------------- LOOP -----------------------------
void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("AZM:")) {
      float deg = input.substring(4).toFloat();
      float newPos = currentPosAZM + deg;
      if (newPos >= LIMIT_MIN_AZM && newPos <= LIMIT_MAX_AZM) {
        if (deg != 0.0) Serial.println("BUSY");
        moveMotor(STEP_PIN_AZM, DIR_PIN_AZM, deg, STEPS_PER_DEGREE_AZM, currentPosAZM);
        Serial.println("OK");
      } else {
        Serial.println("ERR:AZM out of bounds");
      }
    }

    else if (input.startsWith("ALT:")) {
      float deg = input.substring(4).toFloat();
      float newPos = currentPosALT + deg;
      if (newPos >= LIMIT_MIN_ALT && newPos <= LIMIT_MAX_ALT) {
        if (deg != 0.0) Serial.println("BUSY");
        moveMotor(STEP_PIN_ALT, DIR_PIN_ALT, deg, STEPS_PER_DEGREE_ALT, currentPosALT);
        Serial.println("OK");
      } else {
        Serial.println("ERR:ALT out of bounds");
      }
    }

    else if (input == "RST") {
      currentPosAZM = 0.0;
      currentPosALT = 0.0;
      Serial.println("OK");
    }

    else if (input == "POS?" || input == "STA?") {
      Serial.print("POS:AZM=");
      Serial.print(currentPosAZM, 3);
      Serial.print(",ALT=");
      Serial.println(currentPosALT, 3);
    }

    else {
      Serial.println("ERR:Unknown command");
    }
  }
}