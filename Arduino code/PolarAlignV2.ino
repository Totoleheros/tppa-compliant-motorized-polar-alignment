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
 * 📡 SUPPORTED SERIAL COMMANDS (via Serial Monitor @9600 baud):
 * 
 *   🔹 Native axis commands:
 *     - AZM:+2.5     → Move azimuth motor +2.5 degrees
 *     - AZM:-1.0     → Move azimuth motor -1.0 degrees
 *     - ALT:+0.5     → Move altitude motor +0.5 degrees
 *     - ALT:-0.2     → Move altitude motor -0.2 degrees
 *
 *   🔹 Query & reset commands:
 *     - POS? / STA? → Return current AZM and ALT positions (legacy format)
 *     - ?           → Return TPPA-compatible status: <Idle|MPos:+X.XXX,+Y.YYY,0|
 *     - RST         → Reset both AZM and ALT logical positions to zero
 *     - HOME        → Move both motors back to logical 0.0° (soft home)
 *
 *   🔹 G-code-style movement commands:
 *     - J=G53X1.5F300        → Move azimuth to absolute position X=1.5° at speed F=300 (X = AZM)
 *     - J=G53Y-0.8F200       → Move altitude to absolute position Y=-0.8° at speed F=200 (Y = ALT)
 *     - J=G91G21X+0.5F150    → Move azimuth relatively +0.5° at F=150
 *     - J=G91G21Y-1.0F250    → Move altitude relatively -1.0° at F=250
 *
 *     Notes:
 *       - G53 = absolute mode, G91G21 = relative mode (G91 + millimeter mode required by TPPA)
 *       - Only X (AZM) and Y (ALT) axes are supported
 *       - Z axis is ignored, included only for TPPA compatibility
 *       - Speed value (F) is parsed but not used dynamically (fixed step delay for now)
 */

#include <TMCStepper.h>

// ------------------- SYSTEM PARAMETERS TO CONFIGURE -------------------
const float MOTOR_STEPS_PER_REV = 200.0;
const float MICROSTEPPING = 16.0;
const float GEAR_RATIO_AZM = 100.0;
const float GEAR_RATIO_ALT = 90.0;

const float STEPS_PER_DEGREE_AZM = (MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO_AZM) / 360.0;
const float STEPS_PER_DEGREE_ALT = (MOTOR_STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO_ALT) / 360.0;

const float LIMIT_MIN_AZM = -10.0;
const float LIMIT_MAX_AZM =  10.0;
const float LIMIT_MIN_ALT = -15.0;
const float LIMIT_MAX_ALT =  15.0;

// ------------------------- PIN CONFIGURATION --------------------------
#define EN_PIN_AZM     25
#define DIR_PIN_AZM    26
#define STEP_PIN_AZM   27
#define SERIAL_RX_AZM  39

#define EN_PIN_ALT     25
#define DIR_PIN_ALT    32
#define STEP_PIN_ALT   33
#define SERIAL_RX_ALT  36

#define DRIVER_ADDRESS 0
#define R_SENSE        0.11f

HardwareSerial AZMSerial(2);
HardwareSerial ALTSerial(1);

TMC2209Stepper driverAZM(&AZMSerial, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driverALT(&ALTSerial, R_SENSE, DRIVER_ADDRESS);

// ----------------------------- STATE -----------------------------
float currentPosAZM = 0.0;
float currentPosALT = 0.0;

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

// ----------------------------- SETUP -----------------------------
void setup() {
  Serial.begin(115200);            // ✅ S'assurer que la vitesse correspond à celle attendue par TPPA
  Serial.setTimeout(10);
  delay(100);                      // Laisse le temps à l'USB de s'initialiser

  // 🔧 Vide le buffer série des messages de boot ESP32
  while (Serial.available()) Serial.read();

  pinMode(EN_PIN_AZM, OUTPUT);
  pinMode(DIR_PIN_AZM, OUTPUT);
  pinMode(STEP_PIN_AZM, OUTPUT);
  digitalWrite(EN_PIN_AZM, LOW);

  pinMode(DIR_PIN_ALT, OUTPUT);
  pinMode(STEP_PIN_ALT, OUTPUT);
  digitalWrite(EN_PIN_ALT, LOW);

  AZMSerial.begin(115200, SERIAL_8N1, SERIAL_RX_AZM, -1);
  ALTSerial.begin(115200, SERIAL_8N1, SERIAL_RX_ALT, -1);

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

  // ❌ Ne pas envoyer de message comme "READY" ici
}

// ----------------------------- LOOP -----------------------------
void loop() {
  if (Serial.available()) {
    // Gérer la commande '?' (TPPA status) immédiatement
    if (Serial.peek() == '?') {
      Serial.read();  // Consomme '?'
      Serial.print("<Idle|MPos:");
      Serial.print(currentPosAZM, 3);
      Serial.print(",");
      Serial.print(currentPosALT, 3);
      Serial.println(",0|");
      Serial.println();  // Ligne vide attendue

      // 🔁 Vider le buffer jusqu'à \n inclus
      while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') break;
      }
      return;
    }

    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;  // 🔥 Ignore les lignes vides

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

    else if (input == "HOME") {
      Serial.println("BUSY");
      moveMotor(STEP_PIN_AZM, DIR_PIN_AZM, -currentPosAZM, STEPS_PER_DEGREE_AZM, currentPosAZM);
      moveMotor(STEP_PIN_ALT, DIR_PIN_ALT, -currentPosALT, STEPS_PER_DEGREE_ALT, currentPosALT);
      Serial.println("OK");
    }

    else if (input == "POS?" || input == "STA?") {
      Serial.print("POS:AZM=");
      Serial.print(currentPosAZM, 3);
      Serial.print(",ALT=");
      Serial.println(currentPosALT, 3);
    }

    else if (input.startsWith("J=")) {
      input = input.substring(2);
      input.replace("G21", "");

      bool isRelative = false;
      char axis = 0;
      float value = 0.0;
      int fIndex = input.indexOf('F');

      if (input.startsWith("G91")) {
        isRelative = true;
        input = input.substring(3);
      } else if (input.startsWith("G53")) {
        isRelative = false;
        input = input.substring(3);
      } else {
        Serial.println("ERR:Malformed G-code");
        return;
      }

      input.trim();
      if (input.length() < 3 || fIndex == -1) {
        Serial.println("ERR:Malformed G-code");
        return;
      }

      axis = input.charAt(0);
      String valStr = input.substring(1, fIndex);
      value = valStr.toFloat();

      if (axis != 'X' && axis != 'Y') {
        Serial.println("ERR:Unknown axis");
        return;
      }

      if (axis == 'X') {
        float target = isRelative ? currentPosAZM + value : value;
        if (target >= LIMIT_MIN_AZM && target <= LIMIT_MAX_AZM) {
          if (value != 0.0) Serial.println("BUSY");
          moveMotor(STEP_PIN_AZM, DIR_PIN_AZM, target - currentPosAZM, STEPS_PER_DEGREE_AZM, currentPosAZM);
          Serial.println("OK");
        } else {
          Serial.println("ERR:AZM out of bounds");
        }
      }

      else if (axis == 'Y') {
        float target = isRelative ? currentPosALT + value : value;
        if (target >= LIMIT_MIN_ALT && target <= LIMIT_MAX_ALT) {
          if (value != 0.0) Serial.println("BUSY");
          moveMotor(STEP_PIN_ALT, DIR_PIN_ALT, target - currentPosALT, STEPS_PER_DEGREE_ALT, currentPosALT);
          Serial.println("OK");
        } else {
          Serial.println("ERR:ALT out of bounds");
        }
      }
    }

    else {
      Serial.println("ERR:Unknown command");
    }
  }
}
