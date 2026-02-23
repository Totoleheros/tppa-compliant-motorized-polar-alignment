/*****************************************************************************************
 * FYSETC-E4 (ESP32 + TMC2209) - POLAR ALIGNMENT CONTROLLER
 * Version : 14.50 - ACTIVE FEEDBACK LOOP & MACHINE LEARNING
 * * Hardware: FYSETC E4 V1.0, 2x NEMA 17, Harmonic Drive (AZM), Worm Gearbox (ALT)
 * Sensor: MPU-6500 (I2C) for active Altitude tracking and backlash compensation.
 * MPU Wiring: Red (3.3V), Yellow (GND), Blue (SCL/18), Green (SDA/19)
 *****************************************************************************************/
#include <TMCStepper.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

/* ───── HARDWARE SETTINGS (Immutable physical properties) ───── */
constexpr float MOTOR_FULL_STEPS = 200.0f;
constexpr uint16_t MICROSTEPPING_AZM = 16; // StealthChop for smooth Azimuth
constexpr uint16_t MICROSTEPPING_ALT = 4;  // SpreadCycle for high torque on Altitude
constexpr float GEAR_RATIO_AZM = 100.0f;   // Harmonic drive ratio
constexpr float ALT_MOTOR_GEARBOX = 496.0f;// Worm gearbox ratio
constexpr float ALT_SCREW_PITCH_MM = 2.0f; // Lead screw pitch (T8 usually 2mm or 8mm)
constexpr float ALT_RADIUS_MM = 60.0f;     // Distance from pivot axis to the lead screw
constexpr bool AXIS_REV_AZM = true;        // Reverse Azimuth direction
constexpr bool AXIS_REV_ALT = true;        // Reverse Altitude direction
constexpr float HOME_SAFETY_MARGIN = 0.2f; // Degrees to pull off the limit switch after homing
constexpr uint16_t RMS_CURRENT_AZM = 600;  // Motor current in mA
constexpr uint16_t RMS_CURRENT_ALT = 800;  // Motor current in mA

/* ───── PINOUT (FYSETC E4 V1.0 Specific) ───── */
constexpr uint8_t PIN_EN = 25;
constexpr uint8_t PIN_DIR_AZM = 26;
constexpr uint8_t PIN_STEP_AZM = 27;
constexpr uint8_t PIN_DIR_ALT = 32;
constexpr uint8_t PIN_STEP_ALT = 33;
constexpr uint8_t PIN_HOME_SENSOR = 34;    // Physical limit switch for Altitude (Z-MIN)
constexpr uint8_t PIN_BUTTON_HOME = 35;    // Optional manual physical button (Y-MIN)
#define PIN_SERIAL_RX 21                   // Shared UART bus for TMC2209
#define PIN_SERIAL_TX 22                   // Shared UART bus for TMC2209
#define R_SENSE 0.11f
#define ADDR_AZM 1                         // Requires MS1/MS2 jumpers set on E4 board
#define ADDR_ALT 2                         // Requires MS1/MS2 jumpers set on E4 board

/* ───── I2C & MPU-6500 SETTINGS ───── */
constexpr uint8_t SDA_PIN = 19;
constexpr uint8_t SCL_PIN = 18;
constexpr uint8_t MPU_ADDR = 0x68;

/* ───── FEEDBACK LOOP & MACHINE LEARNING THRESHOLDS ───── */
constexpr float HOME_TRIGGER_ANGLE = 0.0f;  // Theoretical angle when hitting the limit switch
constexpr float ALT_TOLERANCE_DEG = 0.05f;  // Acceptable error margin to declare a move "successful"
constexpr uint8_t MAX_CORRECTIONS = 3;      // Max attempts to fix backlash/error before giving up
constexpr float MIN_LEARNING_ANGLE = 0.5f;  // Minimum move size required to train a new ratio
constexpr float MAX_CORRECTION_STEP = 1.0f; // Hard limit for a single correction move (prevents runaway)
volatile float mpuOffset = 0.0f;            // Tare offset calculated during homing
bool mpuAvailable = false;

/* ───── THEORETICAL KINEMATICS ───── */
constexpr float STEPS_PER_DEG_AZM = (MOTOR_FULL_STEPS * MICROSTEPPING_AZM * GEAR_RATIO_AZM) / 360.0f;
constexpr float STEPS_PER_MM_ALT = (MOTOR_FULL_STEPS * MICROSTEPPING_ALT * ALT_MOTOR_GEARBOX) / ALT_SCREW_PITCH_MM;
constexpr float MM_PER_DEGREE_ALT = (2.0f * PI * ALT_RADIUS_MM) / 360.0f;
constexpr float STEPS_PER_DEG_ALT = STEPS_PER_MM_ALT * MM_PER_DEGREE_ALT; 

/* Active Ratio (Will be continuously updated by Machine Learning and stored in EEPROM) */
float activeStepsPerDegALT = STEPS_PER_DEG_ALT;

/* ───── GLOBAL STATE VARIABLES ───── */
volatile float posDegAZM = 0.0f;
volatile float posDegALT = 0.0f;
volatile bool feedHold = false;
volatile bool isMoving = false;
volatile bool abortCmd = false;

/* Feedback & Learning State */
volatile bool settlingForCorrection = false;
volatile unsigned long settleStartMs = 0;
volatile uint8_t correctionCount = 0;
volatile float targetAltAngle = 0.0f;
volatile float learningStartAngle = 0.0f;
volatile float learningRequestedDelta = 0.0f;
volatile bool lastMoveWasUp = true; 
volatile float lastCorrectionError = 0.0f; 
volatile float lastCorrectionApplied = 0.0f; // Tracks how much we tried to move to detect mechanical jams

/* Non-Blocking MPU Sampler State */
volatile int mpuSampleCount = 0;
volatile float mpuSumAngles = 0.0f;
volatile unsigned long lastMpuSampleMs = 0;

/* ───── TMC2209 DRIVERS INSTANTIATION ───── */
HardwareSerial & SerialDrivers = Serial2;
TMC2209Stepper drvAzm(&SerialDrivers, R_SENSE, ADDR_AZM);
TMC2209Stepper drvAlt(&SerialDrivers, R_SENSE, ADDR_ALT);

/* ───── I2C SENSOR FUNCTIONS ───── */
void initMPU_Silent() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00); // Wake up MPU
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00); // Set Accelerometer to +/- 2g for highest sensitivity
    Wire.endTransmission(true);
    mpuAvailable = true;
  } else {
    mpuAvailable = false;
  }
}

// Reads the raw Y-axis angle using gravity vector (Accelerometer).
// Returns -999.0f if the I2C bus crashes (e.g., due to motor EMI).
float readMPUAngleY() {
  if (!mpuAvailable) return -999.0f; 
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return -999.0f; // Intercept EMI crash
  
  Wire.requestFrom(MPU_ADDR, (size_t)6, true);
  if (Wire.available() == 6) {
    int16_t AcX = Wire.read() << 8 | Wire.read();  
    int16_t AcY = Wire.read() << 8 | Wire.read();  
    int16_t AcZ = Wire.read() << 8 | Wire.read();  
    return atan2(AcX, sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180.0f / PI;
  }
  return -999.0f; 
}

/* ───── NON-BLOCKING MOTION ENGINE ───── */
// This queue allows parsing N.I.N.A requests while the motor is physically stepping.
struct MotionJob {
  uint8_t stepPin; uint8_t dirPin; float deltaDeg; float stepsPerDeg; volatile float* globalPos;
};
struct {
  bool active; uint8_t stepPin; long stepsRemaining; float stepDegInc;
  float targetPos; float stepsPerDeg; volatile float* globalPos;
  bool isAzm; float deltaDeg; unsigned long lastStepUs;
} mot = {false};

MotionJob jobQueue[2];
uint8_t jobCount = 0;

void enqueueMotion(uint8_t stepPin, uint8_t dirPin, float deltaDeg, float stepsPerDeg, volatile float* globalPos) {
  if (jobCount >= 2) return;
  jobQueue[jobCount++] = {stepPin, dirPin, deltaDeg, stepsPerDeg, globalPos};
}

bool startNextJob() {
  if (jobCount == 0) return false;
  MotionJob job = jobQueue[0];
  jobQueue[0] = jobQueue[1]; jobCount--;

  long stepsTot = lroundf(job.deltaDeg * job.stepsPerDeg);
  if (!stepsTot) return startNextJob();

  bool isAzm = (job.dirPin == PIN_DIR_AZM);
  bool inv = isAzm ? AXIS_REV_AZM : AXIS_REV_ALT;
  bool logicalDir = (stepsTot > 0) ? !inv : inv; 
  bool isUp = (job.deltaDeg > 0);

  // Setup learning parameters if it's an altitude move (and not a micro-correction)
  if (!isAzm && mpuAvailable && correctionCount == 0) {
    // ANTI-BACKLASH FILTER: Only learn if moving in the same direction as the previous move
    if (isUp == lastMoveWasUp) {
      float startRaw = readMPUAngleY();
      if (startRaw > -900.0f) {
        learningStartAngle = startRaw - mpuOffset;
        learningRequestedDelta = job.deltaDeg;
      } else {
        learningRequestedDelta = 0.0f; 
      }
    } else {
      learningRequestedDelta = 0.0f; 
    }
    lastMoveWasUp = isUp;
  }

  digitalWrite(job.dirPin, logicalDir ? HIGH : LOW);
  delayMicroseconds(50);

  mot.active = true;
  mot.stepPin = job.stepPin;
  mot.stepsRemaining = labs(stepsTot);
  mot.stepDegInc = job.deltaDeg / (float)labs(stepsTot);
  mot.targetPos = *job.globalPos + job.deltaDeg;
  mot.stepsPerDeg = job.stepsPerDeg;
  mot.globalPos = job.globalPos;
  mot.isAzm = isAzm;
  mot.deltaDeg = job.deltaDeg;
  mot.lastStepUs = micros();
  isMoving = true;
  return true;
}

void performPullOff(float stepsPerDeg); 
void softReset(); 
void sendStatus(); 

void tickMotion() {
  // Handle Emergency Stop / Halt
  if (abortCmd) {
    if (mot.active || settlingForCorrection) {
      mot.active = false; isMoving = false; jobCount = 0;
      settlingForCorrection = false;
      softReset();
    }
    return;
  }

  // ACTIVE FEEDBACK LOOP (Runs after an Altitude move finishes)
  if (settlingForCorrection) {
    if (millis() - settleStartMs < 500) return; // Let mechanics stabilize

    // Non-blocking sampler: takes 50 readings over 250ms to filter out mechanical vibrations
    if (millis() - lastMpuSampleMs >= 5) {
      float r = readMPUAngleY();
      if (r > -900.0f) {
        mpuSumAngles += r;
        mpuSampleCount++;
      }
      lastMpuSampleMs = millis();
    }

    if (mpuSampleCount < 50) return; // Keep gathering data

    float rawAngle = mpuSumAngles / (float)mpuSampleCount; // Calculate clean average
    settlingForCorrection = false;
    mpuSampleCount = 0;
    mpuSumAngles = 0.0f;

    float actualAngle = rawAngle - mpuOffset;
    float error = targetAltAngle - actualAngle; // Calculate deviation from target

    // MACHINE LEARNING RATIO ADAPTATION
    if (correctionCount == 0 && abs(learningRequestedDelta) >= MIN_LEARNING_ANGLE) {
      float actualMoved = actualAngle - learningStartAngle;
      if (abs(actualMoved) > 0.1f) {
        float stepsSent = learningRequestedDelta * activeStepsPerDegALT;
        float measuredRatio = stepsSent / actualMoved;
        
        // Filter out absurd ratios (limits to +/- 20% of theoretical)
        if (measuredRatio > (STEPS_PER_DEG_ALT * 0.8f) && measuredRatio < (STEPS_PER_DEG_ALT * 1.2f)) {
          // Smoothing filter: 90% old value, 10% new value to prevent jumpiness
          activeStepsPerDegALT = (activeStepsPerDegALT * 0.90f) + (measuredRatio * 0.10f);
          EEPROM.put(0, activeStepsPerDegALT);
          EEPROM.commit();
          Serial.print("MSG: Ratio trained successfully. New Active Ratio: ");
          Serial.println(activeStepsPerDegALT, 2);
        }
      }
    }

    // SMART FRICTION / JAM ALARM
    // Triggers ONLY if we requested a large move (>0.5 deg) but the plate barely moved (<0.1 deg).
    // Prevents false positives during small backlash-clearing micro-corrections.
    if (correctionCount > 0 && abs(lastCorrectionApplied) >= 0.5f) {
      float prog = abs(lastCorrectionError - error);
      if (prog < 0.1f) { 
        Serial.println("ALARM: Mechanical jam or extreme friction detected. Aborting corrections to prevent damage.");
        correctionCount = 0;
        posDegALT = actualAngle; // Sync logical position with physical reality
        if (!startNextJob()) isMoving = false;
        return;
      }
    }

    // APPLY CORRECTION IF OUT OF TOLERANCE
    if (abs(error) > ALT_TOLERANCE_DEG && correctionCount < MAX_CORRECTIONS) {
      correctionCount++;
      lastCorrectionError = error;
      
      float safeCorrection = error;
      if (safeCorrection > MAX_CORRECTION_STEP) safeCorrection = MAX_CORRECTION_STEP;
      if (safeCorrection < -MAX_CORRECTION_STEP) safeCorrection = -MAX_CORRECTION_STEP;

      lastCorrectionApplied = safeCorrection; 

      Serial.print("MSG: ALT Correction #"); Serial.print(correctionCount);
      Serial.print(" (Read Error: "); Serial.print(error, 3); 
      Serial.print(" deg | Applying Move: "); Serial.print(safeCorrection, 3); Serial.println(" deg)");
      
      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, safeCorrection, activeStepsPerDegALT, &posDegALT);
      startNextJob();
    } else {
      correctionCount = 0; 
      posDegALT = actualAngle; // Hard sync the logical axis with the Gyroscope's absolute truth
      Serial.print("MSG: MPos synchronized with absolute Gyroscope reality: ");
      Serial.println(posDegALT, 3);
      if (!startNextJob()) isMoving = false;
    }
    return;
  }

  // MOTOR PULSE GENERATION
  if (!mot.active || feedHold) return;
  
  unsigned long now = micros();
  if (now - mot.lastStepUs < 120) return; // Basic speed capping
  mot.lastStepUs = now;

  *mot.globalPos += mot.stepDegInc; 

  digitalWrite(mot.stepPin, HIGH); delayMicroseconds(60);
  digitalWrite(mot.stepPin, LOW);
  mot.stepsRemaining--;

  // REAL-TIME HARDWARE SAFETY LIMIT
  if (!mot.isAzm && mot.deltaDeg < 0 && digitalRead(PIN_HOME_SENSOR) == LOW) {
    Serial.println("\n!!! CRITICAL ALARM: PHYSICAL LIMIT SWITCH HIT OUTSIDE HOMING !!!");
    mot.active = false; isMoving = false; jobCount = 0;
    performPullOff(mot.stepsPerDeg);
    *mot.globalPos = HOME_TRIGGER_ANGLE;
    sendStatus(); Serial.println();
    return;
  }

  // JOB COMPLETED
  if (mot.stepsRemaining <= 0) {
    *mot.globalPos = mot.targetPos; 
    mot.active = false;
    
    // If it was an Altitude move, trigger the feedback loop
    if (!mot.isAzm && mpuAvailable && !feedHold) {
      settlingForCorrection = true;
      settleStartMs = millis();
      
      // Lock in the target ONLY on the first pass (fixes moving target bug)
      if (correctionCount == 0) {
        targetAltAngle = mot.targetPos;
        lastCorrectionApplied = 0.0f; 
      }
      
      mpuSampleCount = 0;
      mpuSumAngles = 0.0f;
      lastMpuSampleMs = 0;
      return; 
    }

    if (!startNextJob()) isMoving = false;
  }
}

/* ───── SERIAL PROTOCOL & GRBL EMULATION ───── */
void sendStatus() {
  Serial.print('<');
  if (feedHold) Serial.print("Hold"); else if (isMoving) Serial.print("Run"); else Serial.print("Idle");
  Serial.print("|MPos:"); Serial.print(posDegAZM, 3); Serial.print(','); Serial.print(posDegALT, 3);
  Serial.println(",0|");
}

void scanSerialRealtime() {
  if (!Serial.available()) return;
  char c = Serial.peek();
  if (c == '?') { Serial.read(); sendStatus(); Serial.println(); } // TPPA Polling
  else if (c == '!') { Serial.read(); feedHold = true; Serial.println("ok"); }
  else if (c == '~') { Serial.read(); feedHold = false; Serial.println("ok"); }
  else if (c == 0x18) { Serial.read(); abortCmd = true; } // Ctrl+X (Abort)
}

void softReset() {
  feedHold = false; isMoving = false; abortCmd = false; mot.active = false; jobCount = 0;
  settlingForCorrection = false;
  Serial.println("\r\nGrbl 1.1h ['$' for help]");
}

/* ───── HOMING & MPU CALIBRATION ───── */
// Pulls the mount off the switch to prevent endless triggering loops
void performPullOff(float stepsPerDeg) {
  bool dirUp = !AXIS_REV_ALT ? HIGH : LOW; 
  digitalWrite(PIN_DIR_ALT, dirUp); 
  delay(50);
  
  long count = 0;
  int confirmHigh = 0;
  long maxSteps = 10.0 * stepsPerDeg; // Safety abort if stuck
  
  while (count < maxSteps) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60); 
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    count++;
    
    if (count % 2000 == 0) yield(); 

    if (digitalRead(PIN_HOME_SENSOR) == HIGH) {
      confirmHigh++;
      if (confirmHigh > 20) break; // Debounce
    } else {
      confirmHigh = 0; 
    }
  }

  // Extra margin to ensure switch is fully released
  long safetySteps = HOME_SAFETY_MARGIN * stepsPerDeg;
  for (long s=0; s<safetySteps; s++) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60); 
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    if (s % 2000 == 0) yield();
  }
}

void startHoming() {
  Serial.println("MSG: Homing ALT axis...");
  isMoving = true;
  
  // If already pressed at start, pull off first
  if (digitalRead(PIN_HOME_SENSOR) == LOW) { 
    performPullOff(activeStepsPerDegALT); 
    delay(200); 
  }
  
  // Move down to find switch
  bool dirDown = AXIS_REV_ALT ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, dirDown); 
  delay(10);
  
  int confirmLow = 0;
  bool hit = false;
  
  for (long s=0; s<(50.0*activeStepsPerDegALT); s++) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60); 
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    
    if (s % 2000 == 0) yield(); 

    if (digitalRead(PIN_HOME_SENSOR) == LOW) {
      confirmLow++;
      if (confirmLow > 20) { hit = true; break; }
    } else {
      confirmLow = 0;
    }
    scanSerialRealtime(); if (abortCmd) break;
  }
  
  if (abortCmd) { softReset(); return; }
  if (!hit) { Serial.println("ALARM: Homing failed to find sensor"); isMoving = false; return; }
  
  delay(200); 
  performPullOff(activeStepsPerDegALT); // Step off
  posDegALT = HOME_TRIGGER_ANGLE;       // Define absolute Zero
  
  // Tare the MPU to this physical Zero position
  if (mpuAvailable) {
    Serial.println("MSG: Mechanical stabilization... taring Gyroscope.");
    delay(500); 
    float sumAngles = 0;
    int validSamples = 0;
    for(int i=0; i<50; i++) {
      float r = readMPUAngleY();
      if (r > -900.0f) { 
        sumAngles += r;
        validSamples++;
      }
      delay(5);
    }
    if (validSamples > 0) {
      mpuOffset = (sumAngles / (float)validSamples) - HOME_TRIGGER_ANGLE;
      Serial.print("MSG: Gyroscope tared successfully. Hardware Offset = ");
      Serial.println(mpuOffset, 3);
    } else {
      Serial.println("ALARM: Gyroscope Tare Failed (I2C Bus unresponsive).");
    }
  }
  
  isMoving = false;
  Serial.println("MSG: Homing OK."); sendStatus(); Serial.println();
}

/* ───── INITIALIZATION ───── */
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n=======================================================");
  Serial.println("  BOOT: POLAR ALIGNMENT CONTROLLER V14.50 (ESP32)");
  Serial.println("=======================================================\n");
  
  initMPU_Silent(); 
  
  // Load Learned Ratio from EEPROM
  EEPROM.begin(sizeof(float));
  float storedRatio = 0.0f;
  EEPROM.get(0, storedRatio);
  
  if (!isnan(storedRatio) && storedRatio > (STEPS_PER_DEG_ALT * 0.8f) && storedRatio < (STEPS_PER_DEG_ALT * 1.2f)) {
    activeStepsPerDegALT = storedRatio;
    Serial.print("MSG: Loaded learned ALT Ratio from EEPROM: ");
  } else {
    activeStepsPerDegALT = STEPS_PER_DEG_ALT; 
    Serial.print("MSG: Using default theoretical ALT Ratio: ");
  }
  Serial.println(activeStepsPerDegALT);

  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_DIR_AZM, OUTPUT); pinMode(PIN_STEP_AZM, OUTPUT);
  pinMode(PIN_DIR_ALT, OUTPUT); pinMode(PIN_STEP_ALT, OUTPUT);
  pinMode(PIN_HOME_SENSOR, INPUT); pinMode(PIN_BUTTON_HOME, INPUT);
  digitalWrite(PIN_EN, LOW); // Enable drivers
  
  // TMC2209 UART Init (Requires UART Jumpers on E4 Board)
  SerialDrivers.begin(115200, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);
  delay(100);
  
  drvAzm.begin(); drvAzm.pdn_disable(true); drvAzm.mstep_reg_select(true);
  drvAzm.rms_current(RMS_CURRENT_AZM, 0.1f); drvAzm.microsteps(MICROSTEPPING_AZM); drvAzm.en_spreadCycle(false); drvAzm.toff(4);
  drvAzm.shaft(AXIS_REV_AZM); 

  drvAlt.begin(); drvAlt.pdn_disable(true); drvAlt.mstep_reg_select(true);
  drvAlt.rms_current(RMS_CURRENT_ALT, 0.1f); drvAlt.microsteps(MICROSTEPPING_ALT); drvAlt.en_spreadCycle(true); drvAlt.toff(4);
  drvAlt.shaft(AXIS_REV_ALT); 

  // SMART BOOT: Auto-Recovery if mount was packed/stored on the limit switch
  if (digitalRead(PIN_HOME_SENSOR) == LOW) {
    Serial.println("MSG: ---> ACTION: SENSOR DEPRESSED AT BOOT. INITIATING AUTO-RECOVERY <---");
    startHoming();
  } else {
    Serial.println("MSG: System READY. Waiting for N.I.N.A or Serial commands.");
  }
  Serial.println("-------------------------------------------------------\n");
}

/* ───── MAIN LOOP ───── */
void loop() {
  scanSerialRealtime();
  if (abortCmd && !mot.active && !settlingForCorrection) { softReset(); return; }
  tickMotion();

  // Physical Button Handling
  if (digitalRead(PIN_BUTTON_HOME) == LOW && !isMoving) {
    delay(50); // Debounce
    if (digitalRead(PIN_BUTTON_HOME) == LOW) {
      startHoming(); while (digitalRead(PIN_BUTTON_HOME) == LOW) delay(10); return;
    }
  }

  // Parse incoming commands
  if (!Serial.available()) return;
  char p = Serial.peek();
  if (p == '?' || p == '!' || p == '~' || p == 0x18) return; 

  String line = Serial.readStringUntil('\n'); line.trim();
  if (line.isEmpty()) return;
  
  if (line == "RST") { softReset(); return; }
  if (line == "HOME" || line == "$H") { startHoming(); return; }
  
  // DIAGNOSTIC TOOL
  if (line == "DIAG" || line == "MPU?") {
    Serial.println("\n--- SYSTEM DIAGNOSTIC ---");
    Serial.print("Limit Sensor (Pin 34) : ");
    Serial.println(digitalRead(PIN_HOME_SENSOR) == LOW ? "TRIGGERED (LOW)" : "OPEN (HIGH)");
    Serial.println("");
    if (mpuAvailable) {
      float rawAngle = readMPUAngleY();
      if (rawAngle > -900.0f) {
        Serial.print("MPU-6500 Raw (Y)      : "); Serial.print(rawAngle, 3); Serial.println(" deg");
        Serial.print("MPU-6500 Tare Offset  : "); Serial.print(mpuOffset, 3); Serial.println(" deg");
        Serial.print("MPU-6500 TRUE ANGLE   : "); Serial.print(rawAngle - mpuOffset, 3); Serial.println(" deg");
      } else {
        Serial.println("MPU-6500 STATUS       : I2C ERROR (Bus crashed)");
      }
    } else {
      Serial.println("MPU-6500 STATUS       : Not detected!");
    }
    Serial.println("");
    Serial.print("Active ALT Gear Ratio : "); Serial.print(activeStepsPerDegALT, 3); Serial.println(" steps/deg");
    Serial.println("---------------------------\n");
    return;
  }

  // N.I.N.A TPPA COMMAND PARSING ($J=...)
  if (line.startsWith("$J=") || line.startsWith("J=")) {
    if (isMoving) {
      if (mot.active) { *mot.globalPos = mot.targetPos; mot.active = false; }
      jobCount = 0; isMoving = false; settlingForCorrection = false;
    }
    int eqPos = line.indexOf('='); if (eqPos < 0) return;
    String rest = line.substring(eqPos + 1);
    bool rel = true;
    if (rest.indexOf("G91") >= 0) rel = true; else if (rest.indexOf("G53") >= 0) rel = false;
    rest.replace("G91", ""); rest.replace("G53", ""); rest.replace("G21", "");
    Serial.println("ok"); // Instant reply to prevent NINA timeout

    int idxX = rest.indexOf('X'); int idxY = rest.indexOf('Y');
    if (idxX >= 0) {
      int endNum = rest.length();
      for (int i = idxX + 1; i < rest.length(); i++) {
        if (!isDigit(rest.charAt(i)) && rest.charAt(i) != '.' && rest.charAt(i) != '-') { endNum = i; break; }
      }
      float val = rest.substring(idxX + 1, endNum).toFloat();
      float tgt = rel ? (posDegAZM + val) : val;
      enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, tgt - posDegAZM, STEPS_PER_DEG_AZM, &posDegAZM);
    }
    if (idxY >= 0) {
      int endNum = rest.length();
      for (int i = idxY + 1; i < rest.length(); i++) {
        if (!isDigit(rest.charAt(i)) && rest.charAt(i) != '.' && rest.charAt(i) != '-') { endNum = i; break; }
      }
      float val = rest.substring(idxY + 1, endNum).toFloat();
      float tgt = rel ? (posDegALT + val) : val;
      
      // SOFTWARE ENDSTOP: Prevents diving under 0.0 into the hard limits
      if (tgt < 0.0f) {
        tgt = 0.0f;
        Serial.println("MSG: Software limit reached. Forcing Target to 0.0");
      }
      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    }
    startNextJob(); return;
  }
  
  // MANUAL ABSOLUTE JOG (Console override)
  if (line.startsWith("ALT:")) {
    float tgt = line.substring(4).toFloat();
    if (tgt < 0.0f) {
        tgt = 0.0f;
        Serial.println("MSG: Software limit reached. Forcing Target to 0.0");
    }
    enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    startNextJob(); Serial.println("OK"); sendStatus(); Serial.println(); return;
  }
}
