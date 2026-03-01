/*****************************************************************************************
 * FYSETC-E4 (ESP32 + TMC2209) - POLAR ALIGNMENT CONTROLLER
 * Version : 14.64c - CMD ECHO FIX (diagClear moved to jog entry)
 *
 * Changelog vs 14.64b:
 *  - Moved diagClear() from JOB COMPLETE to jog handler entry, so CMD log
 *    survives through the entire feedback cycle and is visible in DIAG.
 *
 * Hardware: FYSETC E4 V1.0, 2x NEMA 17, Harmonic Drive (AZM), Worm Gearbox (ALT)
 * Sensor: MPU-6500 (I2C) for active Altitude tracking and backlash compensation.
 * MPU Wiring: Red (3.3V), Yellow (GND), Blue (SCL/18), Green (SDA/19)
 *****************************************************************************************/
#include <TMCStepper.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <stdarg.h>

/* ───── HARDWARE SETTINGS ───── */
constexpr float MOTOR_FULL_STEPS = 200.0f;
constexpr uint16_t MICROSTEPPING_AZM = 16;
constexpr uint16_t MICROSTEPPING_ALT = 4;
constexpr float GEAR_RATIO_AZM = 100.0f;
constexpr float ALT_MOTOR_GEARBOX = 496.0f;
constexpr float ALT_SCREW_PITCH_MM = 2.0f;
constexpr float ALT_RADIUS_MM = 60.0f;

constexpr bool AXIS_REV_AZM = true;
constexpr bool AXIS_REV_ALT = true;

constexpr float HOME_SAFETY_MARGIN = 0.2f;
constexpr uint16_t RMS_CURRENT_AZM = 600;
constexpr uint16_t RMS_CURRENT_ALT = 300;

/* ───── PINOUT ───── */
constexpr uint8_t PIN_EN = 25;
constexpr uint8_t PIN_DIR_AZM = 26;
constexpr uint8_t PIN_STEP_AZM = 27;
constexpr uint8_t PIN_DIR_ALT = 32;
constexpr uint8_t PIN_STEP_ALT = 33;
constexpr uint8_t PIN_HOME_SENSOR = 34;
constexpr uint8_t PIN_BUTTON_HOME = 35;
#define PIN_SERIAL_RX 21
#define PIN_SERIAL_TX 22
#define R_SENSE 0.11f
#define ADDR_AZM 1
#define ADDR_ALT 2

/* ───── I2C & MPU-6500 ───── */
constexpr uint8_t SDA_PIN = 19;
constexpr uint8_t SCL_PIN = 18;
constexpr uint8_t MPU_ADDR = 0x68;

/* ───── FEEDBACK LOOP & MACHINE LEARNING THRESHOLDS ───── */
constexpr float HOME_TRIGGER_ANGLE = 0.0f;
constexpr float ALT_TOLERANCE_DEG = 0.05f;
constexpr uint8_t MAX_CORRECTIONS = 3;
constexpr float MIN_LEARNING_ANGLE = 0.5f;
constexpr float MAX_CORRECTION_STEP = 1.0f;

constexpr float RATIO_BAND_LOW = 0.8f;
constexpr float RATIO_BAND_HIGH = 1.2f;
constexpr float LEARNING_SMOOTHING = 0.10f;
constexpr float LEARNING_MIN_ACTUAL = 0.1f;
constexpr float JAM_MIN_MOVE = 0.5f;
constexpr float JAM_MIN_PROGRESS = 0.1f;
constexpr float EEPROM_WRITE_THRESHOLD = 0.5f;
constexpr unsigned long SETTLE_DELAY_MS = 500;
constexpr uint8_t MPU_SAMPLE_TARGET = 50;
constexpr unsigned long MPU_SAMPLE_INTERVAL_MS = 5;

/* ───── RAMP PARAMETERS ───── */
constexpr unsigned long RAMP_START_US = 2000;
constexpr unsigned long RAMP_CRUISE_AZM_US = 240;
constexpr unsigned long RAMP_CRUISE_ALT_US = 120;
constexpr long RAMP_LENGTH = 3000;

/* ───── FEEDBACK REPORT SCALING ───── */
constexpr float FEEDBACK_REPORT_MARGIN = 0.10f;
constexpr float FEEDBACK_MIN_SCALE = 0.50f;

float mpuOffset = 0.0f;
bool mpuAvailable = false;

/* ───── KINEMATICS ───── */
constexpr float STEPS_PER_DEG_AZM = (MOTOR_FULL_STEPS * MICROSTEPPING_AZM * GEAR_RATIO_AZM) / 360.0f;
constexpr float STEPS_PER_MM_ALT = (MOTOR_FULL_STEPS * MICROSTEPPING_ALT * ALT_MOTOR_GEARBOX) / ALT_SCREW_PITCH_MM;
constexpr float MM_PER_DEGREE_ALT = (2.0f * (float)M_PI * ALT_RADIUS_MM) / 360.0f;
constexpr float STEPS_PER_DEG_ALT = STEPS_PER_MM_ALT * MM_PER_DEGREE_ALT;

float activeStepsPerDegALT = STEPS_PER_DEG_ALT;

/* ───── GLOBAL STATE ───── */
volatile float posDegAZM = 0.0f;
volatile float posDegALT = 0.0f;
volatile bool feedHold = false;
volatile bool isMoving = false;
volatile bool abortCmd = false;

volatile float shadowALT = 0.0f;

bool inFeedbackCycle = false;
float feedbackStartPos = 0.0f;

bool settlingForCorrection = false;
unsigned long settleStartMs = 0;
uint8_t correctionCount = 0;
float targetAltAngle = 0.0f;
float learningStartAngle = 0.0f;
float learningRequestedDelta = 0.0f;
bool lastMoveWasUp = true;
float lastCorrectionError = 0.0f;
float lastCorrectionApplied = 0.0f;

int mpuSampleCount = 0;
float mpuSumAngles = 0.0f;
unsigned long lastMpuSampleMs = 0;

/* ───── DIAGNOSTIC LOG BUFFER ───── */
static char diagLog[512];
static uint16_t diagLen = 0;

void diagClear() { diagLen = 0; diagLog[0] = '\0'; }

void diagPrintf(const char* fmt, ...) {
  if (diagLen >= sizeof(diagLog) - 1) return;
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(diagLog + diagLen, sizeof(diagLog) - diagLen, fmt, args);
  va_end(args);
  if (n > 0 && diagLen + n < sizeof(diagLog)) diagLen += n;
}

/* ───── TMC2209 DRIVERS ───── */
HardwareSerial & SerialDrivers = Serial2;
TMC2209Stepper drvAzm(&SerialDrivers, R_SENSE, ADDR_AZM);
TMC2209Stepper drvAlt(&SerialDrivers, R_SENSE, ADDR_ALT);

/* ───── I2C SENSOR ───── */
void initMPU_Silent() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00);
    Wire.endTransmission(true);
    mpuAvailable = true;
  } else {
    mpuAvailable = false;
  }
}

float readMPUAngleY() {
  if (!mpuAvailable) return -999.0f;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return -999.0f;

  Wire.requestFrom(MPU_ADDR, (size_t)6, true);
  if (Wire.available() == 6) {
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    float ay = (float)AcY;
    float az = (float)AcZ;
    return atan2f((float)AcX, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
  }
  return -999.0f;
}

/* ───── NON-BLOCKING MOTION ENGINE ───── */
struct MotionJob {
  uint8_t stepPin; uint8_t dirPin; float deltaDeg; float stepsPerDeg; volatile float* globalPos;
};
struct {
  bool active; uint8_t stepPin; long stepsRemaining; float stepDegInc;
  float targetPos; float stepsPerDeg; volatile float* globalPos;
  bool isAzm; float deltaDeg; unsigned long lastStepUs;
  long totalSteps;
  long stepsDone;
} mot = {false};

MotionJob jobQueue[2];
uint8_t jobCount = 0;

void enqueueMotion(uint8_t stepPin, uint8_t dirPin, float deltaDeg, float stepsPerDeg, volatile float* globalPos) {
  if (jobCount >= 2) return;
  jobQueue[jobCount++] = {stepPin, dirPin, deltaDeg, stepsPerDeg, globalPos};
}

void performPullOff(float stepsPerDeg);
void softReset();
void sendStatus();
void scanSerialRealtime();

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

  if (!isAzm && mpuAvailable && correctionCount == 0) {
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

    if (fabsf(job.deltaDeg) >= ALT_TOLERANCE_DEG) {
      inFeedbackCycle = true;
      feedbackStartPos = *job.globalPos;
      targetAltAngle = *job.globalPos + job.deltaDeg;
    }
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
  mot.totalSteps = mot.stepsRemaining;
  mot.stepsDone = 0;
  isMoving = true;
  return true;
}

void tickMotion() {
  if (abortCmd) {
    if (mot.active || settlingForCorrection) {
      mot.active = false; isMoving = false; jobCount = 0;
      settlingForCorrection = false;
      correctionCount = 0;
      inFeedbackCycle = false;
      learningRequestedDelta = 0;
      softReset();
    }
    return;
  }

  /* ── ACTIVE FEEDBACK LOOP ── */
  if (settlingForCorrection) {
    if (millis() - settleStartMs < SETTLE_DELAY_MS) return;

    if (millis() - lastMpuSampleMs >= MPU_SAMPLE_INTERVAL_MS) {
      float r = readMPUAngleY();
      if (r > -900.0f) {
        mpuSumAngles += r;
        mpuSampleCount++;
      }
      lastMpuSampleMs = millis();
    }

    if (mpuSampleCount < MPU_SAMPLE_TARGET) return;

    float rawAngle = mpuSumAngles / (float)mpuSampleCount;
    settlingForCorrection = false;
    mpuSampleCount = 0;
    mpuSumAngles = 0.0f;

    float actualAngle = rawAngle - mpuOffset;
    float error = targetAltAngle - actualAngle;

    diagPrintf("MPU: act=%.3f tgt=%.3f err=%.3f cc=%d\n",
               actualAngle, targetAltAngle, error, correctionCount);

    /* MACHINE LEARNING RATIO ADAPTATION */
    if (correctionCount == 0 && fabsf(learningRequestedDelta) >= MIN_LEARNING_ANGLE) {
      float actualMoved = actualAngle - learningStartAngle;
      if (fabsf(actualMoved) > LEARNING_MIN_ACTUAL) {
        float stepsSent = learningRequestedDelta * activeStepsPerDegALT;
        float measuredRatio = stepsSent / actualMoved;

        if (measuredRatio > (STEPS_PER_DEG_ALT * RATIO_BAND_LOW) &&
            measuredRatio < (STEPS_PER_DEG_ALT * RATIO_BAND_HIGH)) {

          float oldRatio = activeStepsPerDegALT;
          activeStepsPerDegALT = (activeStepsPerDegALT * (1.0f - LEARNING_SMOOTHING))
                               + (measuredRatio * LEARNING_SMOOTHING);

          if (fabsf(activeStepsPerDegALT - oldRatio) > EEPROM_WRITE_THRESHOLD) {
            EEPROM.put(0, activeStepsPerDegALT);
            EEPROM.commit();
          }
          diagPrintf("Ratio: %.2f\n", activeStepsPerDegALT);
        }
      }
    }

    /* SMART JAM ALARM */
    if (correctionCount > 0 && fabsf(lastCorrectionApplied) >= JAM_MIN_MOVE) {
      float prog = fabsf(lastCorrectionError - error);
      if (prog < JAM_MIN_PROGRESS) {
        diagPrintf("JAM!\n");
        correctionCount = 0;
        inFeedbackCycle = false;
        posDegALT = targetAltAngle;
        if (!startNextJob()) isMoving = false;
        return;
      }
    }

    /* APPLY CORRECTION OR FINALIZE */
    if (fabsf(error) > ALT_TOLERANCE_DEG && correctionCount < MAX_CORRECTIONS) {
      correctionCount++;
      lastCorrectionError = error;

      float safeCorrection = error;
      if (safeCorrection > MAX_CORRECTION_STEP) safeCorrection = MAX_CORRECTION_STEP;
      if (safeCorrection < -MAX_CORRECTION_STEP) safeCorrection = -MAX_CORRECTION_STEP;
      lastCorrectionApplied = safeCorrection;

      diagPrintf("Corr #%d move:%.3f\n", correctionCount, safeCorrection);

      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, safeCorrection, activeStepsPerDegALT, &shadowALT);
      startNextJob();
    } else {
      // ═══ FINAL SYNC ═══
      correctionCount = 0;
      inFeedbackCycle = false;
      posDegALT = targetAltAngle;
      diagPrintf("DONE: reported=%.3f mpuActual=%.3f\n", posDegALT, actualAngle);
      if (!startNextJob()) isMoving = false;
    }
    return;
  }

  /* ── MOTOR PULSE GENERATION ── */
  if (!mot.active || feedHold) return;

  long rampPos = mot.stepsDone;
  long stepsFromEnd = mot.stepsRemaining;
  if (stepsFromEnd < rampPos) rampPos = stepsFromEnd;
  if (rampPos > RAMP_LENGTH) rampPos = RAMP_LENGTH;

  unsigned long cruiseUs = mot.isAzm ? RAMP_CRUISE_AZM_US : RAMP_CRUISE_ALT_US;

  unsigned long interval;
  if (rampPos < RAMP_LENGTH) {
    interval = RAMP_START_US - ((RAMP_START_US - cruiseUs) * rampPos) / RAMP_LENGTH;
  } else {
    interval = cruiseUs;
  }

  unsigned long now = micros();
  if (now - mot.lastStepUs < interval) return;
  mot.lastStepUs = now;

  *mot.globalPos += mot.stepDegInc;

  digitalWrite(mot.stepPin, HIGH); delayMicroseconds(60);
  digitalWrite(mot.stepPin, LOW);
  mot.stepsRemaining--;
  mot.stepsDone++;

  scanSerialRealtime();

  /* HARDWARE SAFETY LIMIT */
  if (!mot.isAzm && mot.deltaDeg < 0 && digitalRead(PIN_HOME_SENSOR) == LOW) {
    Serial.println("\n!!! CRITICAL ALARM: PHYSICAL LIMIT SWITCH HIT OUTSIDE HOMING !!!");
    mot.active = false; isMoving = false; jobCount = 0;
    inFeedbackCycle = false;
    performPullOff(mot.stepsPerDeg);
    *mot.globalPos = HOME_TRIGGER_ANGLE;
    posDegALT = HOME_TRIGGER_ANGLE;
    sendStatus(); Serial.println();
    return;
  }

  /* ── JOB COMPLETE ── */
  if (mot.stepsRemaining <= 0) {
    mot.active = false;

    bool enterFeedback = false;

    if (!mot.isAzm && mpuAvailable && !feedHold) {
      if (correctionCount == 0) {
        if (fabsf(mot.deltaDeg) >= ALT_TOLERANCE_DEG) {
          lastCorrectionApplied = 0.0f;
          shadowALT = 0.0f;
          // v14.64c: diagClear() removed from here — now done at jog entry
          diagPrintf("Feedback: tgt=%.3f start=%.3f\n", targetAltAngle, feedbackStartPos);
          enterFeedback = true;
        }
      } else {
        enterFeedback = true;
      }
    }

    if (enterFeedback) {
      settlingForCorrection = true;
      settleStartMs = millis();
      mpuSampleCount = 0;
      mpuSumAngles = 0.0f;
      lastMpuSampleMs = 0;
      return;
    }

    *mot.globalPos = mot.targetPos;
    if (!startNextJob()) isMoving = false;
  }
}

/* ───── SERIAL PROTOCOL ───── */
void sendStatus() {
  float reportALT = posDegALT;

  if (inFeedbackCycle) {
    float totalDelta = targetAltAngle - feedbackStartPos;
    float absDelta = fabsf(totalDelta);

    if (absDelta > 0.001f) {
      float scaleFactor = (absDelta - FEEDBACK_REPORT_MARGIN) / absDelta;
      if (scaleFactor < FEEDBACK_MIN_SCALE) scaleFactor = FEEDBACK_MIN_SCALE;

      float realProgress = posDegALT - feedbackStartPos;
      reportALT = feedbackStartPos + (realProgress * scaleFactor);
    }
  }

  Serial.print('<');
  if (feedHold) Serial.print("Hold"); else if (isMoving) Serial.print("Run"); else Serial.print("Idle");
  Serial.print("|MPos:"); Serial.print(posDegAZM, 3); Serial.print(','); Serial.print(reportALT, 3);
  Serial.println(",0|");
}

void scanSerialRealtime() {
  if (!Serial.available()) return;
  char c = Serial.peek();
  if (c == '?') { Serial.read(); sendStatus(); Serial.println(); }
  else if (c == '!') { Serial.read(); feedHold = true; Serial.println("ok"); }
  else if (c == '~') { Serial.read(); feedHold = false; Serial.println("ok"); }
  else if (c == 0x18) { Serial.read(); abortCmd = true; }
}

void softReset() {
  feedHold = false; isMoving = false; abortCmd = false;
  mot.active = false; jobCount = 0;
  settlingForCorrection = false;
  correctionCount = 0;
  inFeedbackCycle = false;
  learningRequestedDelta = 0;
  Serial.println("\r\nGrbl 1.1h ['$' for help]");
}

/* ───── HOMING ───── */
void performPullOff(float stepsPerDeg) {
  bool dirUp = !AXIS_REV_ALT ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, dirUp);
  delay(50);

  long count = 0;
  int confirmHigh = 0;
  long maxSteps = (long)(10.0f * stepsPerDeg);

  while (count < maxSteps) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    count++;
    if (count % 2000 == 0) yield();
    if (digitalRead(PIN_HOME_SENSOR) == HIGH) {
      confirmHigh++;
      if (confirmHigh > 20) break;
    } else {
      confirmHigh = 0;
    }
  }

  long safetySteps = (long)(HOME_SAFETY_MARGIN * stepsPerDeg);
  for (long s = 0; s < safetySteps; s++) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW); delayMicroseconds(60);
    if (s % 2000 == 0) yield();
  }
}

void startHoming() {
  Serial.println("MSG: Homing ALT axis...");
  isMoving = true;
  inFeedbackCycle = false;

  if (digitalRead(PIN_HOME_SENSOR) == LOW) {
    performPullOff(activeStepsPerDegALT);
    delay(200);
  }

  bool dirDown = AXIS_REV_ALT ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, dirDown);
  delay(10);

  int confirmLow = 0;
  bool hit = false;

  for (long s = 0; s < (long)(50.0f * activeStepsPerDegALT); s++) {
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
  performPullOff(activeStepsPerDegALT);
  posDegALT = HOME_TRIGGER_ANGLE;

  if (mpuAvailable) {
    Serial.println("MSG: Mechanical stabilization... taring Gyroscope.");
    delay(500);
    float sumAngles = 0;
    int validSamples = 0;
    for (int i = 0; i < MPU_SAMPLE_TARGET; i++) {
      float r = readMPUAngleY();
      if (r > -900.0f) { sumAngles += r; validSamples++; }
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
  correctionCount = 0;
  learningRequestedDelta = 0;
  Serial.println("MSG: Homing OK."); sendStatus(); Serial.println();
}

/* ───── DIAGNOSTIC ───── */
void printDiagnostic() {
  Serial.println("\n--- SYSTEM DIAGNOSTIC (v14.64c) ---");
  Serial.print("Limit Sensor (Pin 34) : ");
  Serial.println(digitalRead(PIN_HOME_SENSOR) == LOW ? "TRIGGERED (LOW)" : "OPEN (HIGH)");
  Serial.println("");
  if (mpuAvailable) {
    float rawAngle = readMPUAngleY();
    if (rawAngle > -900.0f) {
      Serial.print("MPU-6500 Raw (Y)      : "); Serial.print(rawAngle, 3); Serial.println(" deg");
      Serial.print("MPU-6500 Tare Offset  : "); Serial.print(mpuOffset, 3); Serial.println(" deg");
      Serial.print("MPU-6500 Homing-Rel   : "); Serial.print(rawAngle - mpuOffset, 3); Serial.println(" deg");
    } else {
      Serial.println("MPU-6500 STATUS       : I2C ERROR");
    }
  } else {
    Serial.println("MPU-6500 STATUS       : Not detected!");
  }
  Serial.println("");
  Serial.print("posDegALT (internal)  : "); Serial.println(posDegALT, 3);
  Serial.print("shadowALT             : "); Serial.println(shadowALT, 3);
  Serial.print("targetAltAngle        : "); Serial.println(targetAltAngle, 3);
  Serial.print("inFeedbackCycle       : "); Serial.println(inFeedbackCycle ? "YES" : "NO");
  Serial.print("feedbackStartPos      : "); Serial.println(feedbackStartPos, 3);
  Serial.print("correctionCount       : "); Serial.println(correctionCount);
  Serial.print("settling              : "); Serial.println(settlingForCorrection ? "YES" : "NO");
  Serial.println("");
  Serial.print("Active ALT Ratio      : "); Serial.print(activeStepsPerDegALT, 3); Serial.println(" steps/deg");
  Serial.print("Theoretical ALT Ratio : "); Serial.print(STEPS_PER_DEG_ALT, 3); Serial.println(" steps/deg");
  Serial.print("ALT RMS Current       : "); Serial.print(RMS_CURRENT_ALT); Serial.println(" mA");
  Serial.print("AZM Cruise            : "); Serial.print(RAMP_CRUISE_AZM_US); Serial.println(" us");
  Serial.print("ALT Cruise            : "); Serial.print(RAMP_CRUISE_ALT_US); Serial.println(" us");
  Serial.print("Report margin         : "); Serial.print(FEEDBACK_REPORT_MARGIN, 3); Serial.println(" deg");
  Serial.print("Report min scale      : "); Serial.println(FEEDBACK_MIN_SCALE, 2);
  Serial.print("Ramp length           : "); Serial.print(RAMP_LENGTH); Serial.println(" steps");
  Serial.print("EEPROM Stored Ratio   : ");
  float stored = 0.0f; EEPROM.get(0, stored);
  Serial.println(stored, 3);

  if (diagLen > 0) {
    Serial.println("");
    Serial.println("--- LAST FEEDBACK LOG ---");
    Serial.print(diagLog);
    Serial.println("--- END LOG ---");
  }
  Serial.println("----------------------------------\n");
}

/* ───── COMMAND PROCESSOR ───── */
void processCommand(const char* line) {
  if (line[0] == '\0') return;

  if (strcmp(line, "RST") == 0) { softReset(); return; }
  if (strcmp(line, "HOME") == 0 || strcmp(line, "$H") == 0) { startHoming(); return; }
  if (strcmp(line, "DIAG") == 0 || strcmp(line, "MPU?") == 0) { printDiagnostic(); return; }

  if (strncmp(line, "$J=", 3) == 0 || strncmp(line, "J=", 2) == 0) {
    if (isMoving) {
      if (mot.active) { *mot.globalPos = mot.targetPos; mot.active = false; }

      if (inFeedbackCycle || settlingForCorrection || correctionCount > 0) {
        posDegALT = targetAltAngle;
        inFeedbackCycle = false;
      }

      jobCount = 0; isMoving = false; settlingForCorrection = false;
      correctionCount = 0;
      learningRequestedDelta = 0;
    }

    const char* rest = strchr(line, '=');
    if (!rest) return;
    rest++;

    bool rel = true;
    if (strstr(rest, "G53")) rel = false;

    Serial.println("ok");

    // v14.64c: Clear + log at jog entry so CMD survives through feedback
    diagClear();
    diagPrintf("CMD: %s pos=%.3f\n", line, posDegALT);

    const char* xp = strchr(rest, 'X');
    const char* yp = strchr(rest, 'Y');

    if (xp) {
      float val = atof(xp + 1);
      float tgt = rel ? (posDegAZM + val) : val;
      enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, tgt - posDegAZM, STEPS_PER_DEG_AZM, &posDegAZM);
    }

    if (yp) {
      float val = atof(yp + 1);
      float tgt = rel ? (posDegALT + val) : val;
      if (tgt < 0.0f) {
        tgt = 0.0f;
        Serial.println("MSG: Software limit reached. Forcing Target to 0.0");
      }
      diagPrintf("Parse: rel=%d val=%.3f tgt=%.3f delta=%.3f\n", rel, val, tgt, tgt - posDegALT);
      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    }

    startNextJob();
    return;
  }

  if (strcmp(line, "AZM:ZERO") == 0) {
    if (isMoving && mot.isAzm) return;
    posDegAZM = 0.0f;
    Serial.println("MSG: AZM absolute position forcefully reset to 0.0");
    sendStatus(); Serial.println();
    return;
  }

  if (strncmp(line, "AZM:", 4) == 0) {
    float tgt = atof(line + 4);
    enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, tgt - posDegAZM, STEPS_PER_DEG_AZM, &posDegAZM);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println();
    return;
  }

  if (strncmp(line, "ALT:", 4) == 0) {
    float tgt = atof(line + 4);
    if (tgt < 0.0f) {
      tgt = 0.0f;
      Serial.println("MSG: Software limit reached. Forcing Target to 0.0");
    }
    enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println();
    return;
  }
}

/* ───── INITIALIZATION ───── */
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=======================================================");
  Serial.println("  BOOT: POLAR ALIGNMENT CONTROLLER V14.64c (ESP32)");
  Serial.println("=======================================================\n");

  initMPU_Silent();
  diagClear();

  EEPROM.begin(sizeof(float));
  float storedRatio = 0.0f;
  EEPROM.get(0, storedRatio);

  if (!isnan(storedRatio) &&
      storedRatio > (STEPS_PER_DEG_ALT * RATIO_BAND_LOW) &&
      storedRatio < (STEPS_PER_DEG_ALT * RATIO_BAND_HIGH)) {
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
  digitalWrite(PIN_EN, LOW);

  SerialDrivers.begin(115200, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);
  delay(100);

  drvAzm.begin(); drvAzm.pdn_disable(true); drvAzm.mstep_reg_select(true);
  drvAzm.rms_current(RMS_CURRENT_AZM, 0.1f); drvAzm.microsteps(MICROSTEPPING_AZM);
  drvAzm.en_spreadCycle(false); drvAzm.toff(4);
  drvAzm.shaft(false);

  drvAlt.begin(); drvAlt.pdn_disable(true); drvAlt.mstep_reg_select(true);
  drvAlt.rms_current(RMS_CURRENT_ALT, 0.1f); drvAlt.microsteps(MICROSTEPPING_ALT);
  drvAlt.en_spreadCycle(true); drvAlt.toff(4);
  drvAlt.shaft(false);

  Serial.print("MSG: ALT motor current set to "); Serial.print(RMS_CURRENT_ALT);
  Serial.println("mA (thermal optimized)");

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

  if (digitalRead(PIN_BUTTON_HOME) == LOW && !isMoving) {
    delay(50);
    if (digitalRead(PIN_BUTTON_HOME) == LOW) {
      startHoming(); while (digitalRead(PIN_BUTTON_HOME) == LOW) delay(10); return;
    }
  }

  static char lineBuf[64];
  static uint8_t lineIdx = 0;

  while (Serial.available()) {
    char c = Serial.peek();
    if (c == '?' || c == '!' || c == '~' || c == 0x18) break;
    Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineIdx > 0) {
        lineBuf[lineIdx] = '\0';
        processCommand(lineBuf);
        lineIdx = 0;
      }
      break;
    }
    if (lineIdx < sizeof(lineBuf) - 1) lineBuf[lineIdx++] = c;
  }
}
