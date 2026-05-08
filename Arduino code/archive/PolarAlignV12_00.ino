/*****************************************************************************************
 * PROJECT: ESP32 TILT-PLATE CONTROLLER FOR N.I.N.A TPPA
 * ---------------------------------------------------------------------------------------
 * AUTHOR  : Antonino Nicoletti (Hardware/Concept) & Gemini (Code Architecture)
 * VERSION : 12.00 (Gold Master - Non-Blocking Architecture)
 * DATE    : 2026-02-08
 *
 * DESCRIPTION:
 * This firmware emulates the communication protocol required by the "Avalon Polar Alignment"
 * driver in N.I.N.A. It allows a custom DIY ESP32-based Alt-Az base to be controlled
 * directly by the Three Point Polar Alignment plugin.
 *
 * KEY FEATURES:
 * - Silent Boot: Prevents TPPA connection timeouts caused by ESP32 boot logs.
 * - Non-Blocking Engine: Allows real-time position updates during motor movement.
 * - Strict Polling: Eliminates serial buffer lag/desync.
 * - Precision Fix: Corrects floating-point drift at the end of movements.
 *****************************************************************************************/

#include <TMCStepper.h>
#include <math.h>

/* ───────────────────────────────────────────────────────────────────────────────────────
   1. HARDWARE SETTINGS (SANCTIFIED)
   ---------------------------------------------------------------------------------------
   These values are calibrated for the specific mechanics of the tilt-plate.
   DO NOT CHANGE unless hardware changes.
   ─────────────────────────────────────────────────────────────────────────────────────── */
constexpr float     MOTOR_FULL_STEPS   = 200.0f;   // Standard 1.8° stepper
constexpr uint16_t  MICROSTEPPING_AZM  = 16;       // High resolution for Azimuth
constexpr uint16_t  MICROSTEPPING_ALT  = 4;        // High torque for Altitude (lifting weight)

// Mechanical Ratios
constexpr float     GEAR_RATIO_AZM     = 100.0f;
constexpr float     ALT_MOTOR_GEARBOX  = 496.0f;   // Calibrated ratio (Real 4.8° vs Requested 5.0°)
constexpr float     ALT_SCREW_PITCH_MM = 2.0f;
constexpr float     ALT_RADIUS_MM      = 60.0f;

// Directions (Invert if motors move the wrong way)
constexpr bool      AXIS_REV_AZM       = true;
constexpr bool      AXIS_REV_ALT       = true;

// Safety & Power
constexpr float     HOME_SAFETY_MARGIN = 0.2f;     // Degrees to back off after homing
constexpr uint16_t  RMS_CURRENT_AZM    = 600;      // mA
constexpr uint16_t  RMS_CURRENT_ALT    = 800;      // mA (Higher for lifting)

/* ───── PINOUT CONFIGURATION (FYSETC E4 Board) ───── */
constexpr uint8_t PIN_EN          = 25;
constexpr uint8_t PIN_DIR_AZM     = 26;
constexpr uint8_t PIN_STEP_AZM    = 27;
constexpr uint8_t PIN_DIR_ALT     = 32;
constexpr uint8_t PIN_STEP_ALT    = 33;
constexpr uint8_t PIN_HOME_SENSOR = 34;
constexpr uint8_t PIN_BUTTON_HOME = 35; // Input Only pin

// UART Configuration for TMC2209 (Shared Bus)
#define PIN_SERIAL_RX 21
#define PIN_SERIAL_TX 22
#define R_SENSE       0.11f
#define ADDR_AZM      1
#define ADDR_ALT      2

/* ───── AUTOMATIC CALCULATIONS ───── */
// Converts degrees to steps based on geometry and microstepping
constexpr float STEPS_PER_DEG_AZM = (MOTOR_FULL_STEPS * MICROSTEPPING_AZM * GEAR_RATIO_AZM) / 360.0f;
constexpr float STEPS_PER_MM_ALT  = (MOTOR_FULL_STEPS * MICROSTEPPING_ALT * ALT_MOTOR_GEARBOX) / ALT_SCREW_PITCH_MM;
constexpr float MM_PER_DEGREE_ALT = (2.0f * PI * ALT_RADIUS_MM) / 360.0f;
constexpr float STEPS_PER_DEG_ALT = STEPS_PER_MM_ALT * MM_PER_DEGREE_ALT;

/* ───── GLOBAL VARIABLES ───── */
// Volatile is required as these are accessed within the interrupt-like tick function
volatile float posDegAZM = 0.0f;
volatile float posDegALT = 0.0f;
volatile bool  feedHold  = false; // Pauses motion if true
volatile bool  isMoving  = false; // Status flag for TPPA
volatile bool  abortCmd  = false; // Triggers soft reset

/* ───── TMC2209 DRIVER OBJECTS ───── */
HardwareSerial & SerialDrivers = Serial2;
TMC2209Stepper drvAzm(&SerialDrivers, R_SENSE, ADDR_AZM);
TMC2209Stepper drvAlt(&SerialDrivers, R_SENSE, ADDR_ALT);

/* ───────────────────────────────────────────────────────────────────────────────────────
   2. NON-BLOCKING MOTION ENGINE
   ---------------------------------------------------------------------------------------
   This architecture replaces standard 'for' loops. It allows the main loop() to continue
   running (listening to Serial) while the motor steps are generated based on time.
   ─────────────────────────────────────────────────────────────────────────────────────── */

// Structure defining a requested movement
struct MotionJob {
  uint8_t stepPin;
  uint8_t dirPin;
  float deltaDeg;
  float stepsPerDeg;
  volatile float* globalPos; // Pointer to the global position variable (AZM or ALT)
};

// Structure holding the state of the currently active movement
struct {
  bool active;                // Is a move currently in progress?
  uint8_t stepPin;
  long stepsRemaining;        // How many steps left to do
  float stepDegInc;           // Degree increment per single step (for realtime update)
  float targetPos;            // Mathematical target to enforce at the end
  float stepsPerDeg;
  volatile float* globalPos;
  bool isAzm;                 // Flag to identify axis (for limits)
  float deltaDeg;
  unsigned long lastStepUs;   // Timestamp of the last pulse
} mot = {false};

// Simple Queue System (Buffer depth = 2)
MotionJob jobQueue[2];
uint8_t jobCount = 0;

// Adds a move command to the queue
void enqueueMotion(uint8_t stepPin, uint8_t dirPin, float deltaDeg, float stepsPerDeg, volatile float* globalPos) {
  if (jobCount >= 2) return; // Buffer full, ignore (should not happen with TPPA logic)
  jobQueue[jobCount++] = {stepPin, dirPin, deltaDeg, stepsPerDeg, globalPos};
}

// Loads the next job from queue into the active engine
bool startNextJob() {
  if (jobCount == 0) return false;

  // Pop from queue
  MotionJob job = jobQueue[0];
  jobQueue[0] = jobQueue[1]; // Shift queue
  jobCount--;

  // Calculate Steps
  long stepsTot = lroundf(job.deltaDeg * job.stepsPerDeg);
  if (!stepsTot) return startNextJob(); // Skip 0-step moves

  // Determine Direction
  bool isAzm = (job.dirPin == PIN_DIR_AZM);
  bool inv   = isAzm ? AXIS_REV_AZM : AXIS_REV_ALT;
  bool logicalDir = (stepsTot > 0) ? !inv : inv;

  // Set Hardware Direction
  digitalWrite(job.dirPin, logicalDir ? HIGH : LOW);
  // Sync driver internal counter (optional but good practice)
  if (isAzm) drvAzm.shaft(logicalDir); else drvAlt.shaft(logicalDir);
  delayMicroseconds(50); // Setup time

  // Initialize Active Job State
  mot.active = true;
  mot.stepPin = job.stepPin;
  mot.stepsRemaining = labs(stepsTot);
  mot.stepDegInc = job.deltaDeg / (float)labs(stepsTot); // Pre-calculate float increment
  mot.targetPos = *job.globalPos + job.deltaDeg;         // Pre-calculate exact target
  mot.stepsPerDeg = job.stepsPerDeg;
  mot.globalPos = job.globalPos;
  mot.isAzm = isAzm;
  mot.deltaDeg = job.deltaDeg;
  mot.lastStepUs = micros();

  isMoving = true; // Signal TPPA that we are running
  return true;
}

// The "Heartbeat" function. Must be called as fast as possible in loop()
void tickMotion() {
  if (!mot.active || feedHold) return;

  // Non-blocking delay check (Speed control)
  unsigned long now = micros();
  // 120us = ~8.3 kHz max step rate (Adjust if needed, safe for ESP32)
  if (now - mot.lastStepUs < 120) return;
  mot.lastStepUs = now;

  // 1. Real-time Position Update
  // We update the global variable continuously so TPPA sees progress when querying "?"
  *mot.globalPos += mot.stepDegInc;

  // 2. Hardware Pulse
  digitalWrite(mot.stepPin, HIGH);
  delayMicroseconds(60); // Minimum pulse width
  digitalWrite(mot.stepPin, LOW);

  mot.stepsRemaining--;

  // 3. Safety Limit Check (Altitude Axis Only)
  if (!mot.isAzm && mot.deltaDeg < 0 && digitalRead(PIN_HOME_SENSOR) == LOW) {
    Serial.println("ALARM: Limit Hit!");
    mot.active = false; isMoving = false;
    jobCount = 0; // Clear queue
    performPullOff(mot.stepsPerDeg);
    *mot.globalPos = 0.0f; // Reset Home
    sendStatus(); Serial.println();
    return;
  }

  // 4. Abort Check
  if (abortCmd) {
    mot.active = false; isMoving = false;
    jobCount = 0;
    softReset();
    return;
  }

  // 5. End of Move
  if (mot.stepsRemaining <= 0) {
    // CRITICAL FIX: Snapping to target
    // Overwrite the accumulated float with the exact target to prevent drift (0.999 -> 1.0)
    *mot.globalPos = mot.targetPos;

    mot.active = false;
    // Try to start next job, if none, stop.
    if (!startNextJob()) {
      isMoving = false;
    }
  }
}

/* ───────────────────────────────────────────────────────────────────────────────────────
   3. COMMUNICATION PROTOCOL
   ---------------------------------------------------------------------------------------
   Strict Request-Reply implementation to avoid buffer lags.
   ─────────────────────────────────────────────────────────────────────────────────────── */

// Formats the GRBL-style status string
void sendStatus() {
  Serial.print('<');
  if (feedHold) Serial.print("Hold");
  else if (isMoving) Serial.print("Run");
  else Serial.print("Idle");
  Serial.print("|MPos:");
  Serial.print(posDegAZM, 3); // 3 decimals precision
  Serial.print(',');
  Serial.print(posDegALT, 3);
  Serial.println(",0|");
}

// Handles incoming serial characters immediately
void scanSerialRealtime() {
  if (!Serial.available()) return;
  char c = Serial.peek(); // Peek to check for realtime commands without consuming buffer

  // TPPA Polling
  if (c == '?') {
    Serial.read(); // Consume '?'
    sendStatus();
    Serial.println(); // Mandatory empty line for TPPA parser
  }
  // GRBL Realtime commands
  else if (c == '!') { Serial.read(); feedHold = true; Serial.println("ok"); }
  else if (c == '~') { Serial.read(); feedHold = false; Serial.println("ok"); }
  else if (c == 0x18) { Serial.read(); abortCmd = true; } // Ctrl-X
}

// Resets internal state and sends Banner (GRBL Hello)
void softReset() {
  feedHold = false; isMoving = false; abortCmd = false;
  mot.active = false; jobCount = 0;
  Serial.println("\r\nGrbl 1.1h ['$' for help]");
}

/* ───────────────────────────────────────────────────────────────────────────────────────
   4. HOMING ROUTINE (BLOCKING)
   ---------------------------------------------------------------------------------------
   Homing is an exception where blocking code is acceptable as no complex TPPA
   interaction happens during this manual phase.
   ─────────────────────────────────────────────────────────────────────────────────────── */

void performPullOff(float stepsPerDeg) {
  // Move away from the sensor
  bool physicalDir = (AXIS_REV_ALT) ? LOW : HIGH;
  digitalWrite(PIN_DIR_ALT, physicalDir);
  drvAlt.shaft(physicalDir);
  delay(50);

  long timeoutSteps = 10.0 * stepsPerDeg;
  long count = 0;
  // Step until sensor released
  while (digitalRead(PIN_HOME_SENSOR) == LOW && count < timeoutSteps) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    count++;
  }
  // Extra safety margin
  long safetySteps = HOME_SAFETY_MARGIN * stepsPerDeg;
  for (long s = 0; s < safetySteps; s++) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
  }
}

void startHoming() {
  Serial.println("MSG: Homing ALT...");
  isMoving = true;

  // 1. If already triggered, back off
  if (digitalRead(PIN_HOME_SENSOR) == LOW) {
    performPullOff(STEPS_PER_DEG_ALT);
    // Move up a bit to re-approach cleanly
    bool inv = AXIS_REV_ALT;
    bool logicalDir = !inv;
    digitalWrite(PIN_DIR_ALT, logicalDir ? HIGH : LOW);
    drvAlt.shaft(logicalDir);
    delayMicroseconds(50);
    long steps = lroundf(2.0f * STEPS_PER_DEG_ALT);
    for (long s = 0; s < steps; s++) {
      scanSerialRealtime(); if (abortCmd) break;
      digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
      digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    }
    if (abortCmd) { softReset(); return; }
    posDegALT += 2.0f;
    delay(200);
  }

  // 2. Approach Sensor
  bool homingDirVal = (AXIS_REV_ALT) ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, homingDirVal);
  drvAlt.shaft(homingDirVal);
  delay(10);

  bool hit = false;
  long maxSteps = 50.0 * STEPS_PER_DEG_ALT; // Max search range
  for (long s = 0; s < maxSteps; s++) {
    if (digitalRead(PIN_HOME_SENSOR) == LOW) { hit = true; break; }
    scanSerialRealtime(); if (abortCmd) break;
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
  }

  if (abortCmd) { softReset(); return; }
  if (!hit) { Serial.println("ALARM: Homing fail"); isMoving = false; return; }

  delay(200);

  // 3. Final Pull-off and Zeroing
  performPullOff(STEPS_PER_DEG_ALT);
  posDegALT = 0.0f;
  isMoving = false;
  Serial.println("MSG: Homing OK. ALT set to 0.0 deg.");
  sendStatus(); Serial.println();
}

/* ───────────────────────────────────────────────────────────────────────────────────────
   5. SETUP
   ─────────────────────────────────────────────────────────────────────────────────────── */

void setup() {
  Serial.begin(115200);

  // NOTE: Silent Boot Strategy
  // We do NOT print anything here. We wait for TPPA to send '?' before replying.
  // This prevents the plugin from parsing ESP32 boot logs as invalid status.

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DIR_AZM, OUTPUT); pinMode(PIN_STEP_AZM, OUTPUT);
  pinMode(PIN_DIR_ALT, OUTPUT); pinMode(PIN_STEP_ALT, OUTPUT);
  pinMode(PIN_HOME_SENSOR, INPUT);
  pinMode(PIN_BUTTON_HOME, INPUT);
  digitalWrite(PIN_EN, LOW); // Enable drivers

  // TMC2209 UART Init
  SerialDrivers.begin(115200, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);
  delay(100);

  // Driver Configuration
  drvAzm.begin(); drvAzm.pdn_disable(true); drvAzm.mstep_reg_select(true);
  drvAzm.rms_current(RMS_CURRENT_AZM, 0.1f); drvAzm.microsteps(MICROSTEPPING_AZM);
  drvAzm.en_spreadCycle(false); // StealthChop for Azimuth (Smoothness)
  drvAzm.toff(4);

  drvAlt.begin(); drvAlt.pdn_disable(true); drvAlt.mstep_reg_select(true);
  drvAlt.rms_current(RMS_CURRENT_ALT, 0.1f); drvAlt.microsteps(MICROSTEPPING_ALT);
  drvAlt.en_spreadCycle(true);  // SpreadCycle for Altitude (Torque)
  drvAlt.toff(4);
}

/* ───────────────────────────────────────────────────────────────────────────────────────
   6. MAIN LOOP
   ─────────────────────────────────────────────────────────────────────────────────────── */

void loop() {
  // 1. Process Serial Queries (Highest Priority for responsiveness)
  scanSerialRealtime();

  // 2. Abort Handling
  if (abortCmd && !mot.active) { softReset(); return; }

  // 3. Advance Motion (One tick)
  tickMotion();

  // 4. Physical Home Button
  if (digitalRead(PIN_BUTTON_HOME) == LOW && !isMoving) {
    delay(50); // Debounce
    if (digitalRead(PIN_BUTTON_HOME) == LOW) {
      startHoming();
      while (digitalRead(PIN_BUTTON_HOME) == LOW) { delay(10); } // Wait release
      return;
    }
  }

  // 5. Parse Serial Commands
  if (!Serial.available()) return;
  char p = Serial.peek();
  // If it's a realtime character, skip readStringUntil to avoid blocking
  if (p == '?' || p == '!' || p == '~' || p == 0x18) return;

  String line = Serial.readStringUntil('\n'); line.trim();
  if (line.isEmpty()) return;

  if (line == "RST") { softReset(); return; }

  // --- JOG COMMAND ($J=...) Handling ---
  if (line.startsWith("$J=") || line.startsWith("J=")) {

    // Stop existing motion if any
    if (isMoving) {
      if (mot.active) {
        *mot.globalPos = mot.targetPos; // Snap to target before stopping
        mot.active = false;
      }
      jobCount = 0;
      isMoving = false;
    }

    // Parse G-Code parameters
    int eqPos = line.indexOf('=');
    if (eqPos < 0) return;
    String rest = line.substring(eqPos + 1);
    bool rel = true;
    if (rest.indexOf("G91") >= 0) rel = true;
    else if (rest.indexOf("G53") >= 0) rel = false;
    // Clean string
    rest.replace("G91", ""); rest.replace("G53", ""); rest.replace("G21", "");

    // Acknowledge command immediately
    Serial.println("ok");

    // Extract Targets
    int idxX = rest.indexOf('X');
    int idxY = rest.indexOf('Y');

    // Queue Azimuth (X) Move
    if (idxX >= 0) {
      int endNum = rest.length();
      for (int i = idxX + 1; i < rest.length(); i++) {
        if (!isDigit(rest.charAt(i)) && rest.charAt(i) != '.' && rest.charAt(i) != '-') { endNum = i; break; }
      }
      float val = rest.substring(idxX + 1, endNum).toFloat();
      float tgt = rel ? (posDegAZM + val) : val;
      float delta = tgt - posDegAZM;
      enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, delta, STEPS_PER_DEG_AZM, &posDegAZM);
    }

    // Queue Altitude (Y) Move
    if (idxY >= 0) {
      int endNum = rest.length();
      for (int i = idxY + 1; i < rest.length(); i++) {
        if (!isDigit(rest.charAt(i)) && rest.charAt(i) != '.' && rest.charAt(i) != '-') { endNum = i; break; }
      }
      float val = rest.substring(idxY + 1, endNum).toFloat();
      float tgt = rel ? (posDegALT + val) : val;
      float delta = tgt - posDegALT;
      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, delta, STEPS_PER_DEG_ALT, &posDegALT);
    }

    // Ignite Engine
    startNextJob();
    return;
  }

  // --- Manual Console Command (ALT:) ---
  if (line.startsWith("ALT:")) {
    float val = line.substring(4).toFloat();
    enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, val, STEPS_PER_DEG_ALT, &posDegALT);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println(); return;
  }
}
