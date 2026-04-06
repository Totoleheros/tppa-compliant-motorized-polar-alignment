/*****************************************************************************************
 * FYSETC-E4 (ESP32 + TMC2209) — POLAR ALIGNMENT CONTROLLER
 * Version : 15.03g-p1  (Gemini-reviewed + AZM hold current fix)
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * WHAT THIS FIRMWARE DOES
 * ──────────────────────────────────────────────────────────────────────────────────────
 * Drives a two-axis motorized polar alignment platform (Azimuth + Altitude) and
 * speaks the "Avalon" dialect of the GRBL protocol so N.I.N.A.'s Three-Point Polar
 * Alignment (TPPA) plug-in treats it as a supported mount.
 *
 * The firmware receives jog commands in arcminutes from TPPA, converts them to
 * degrees, moves the motors, and reports position back in arcminutes. It silently
 * learns the true mechanical gear ratios over successive sessions.
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * HARDWARE
 * ──────────────────────────────────────────────────────────────────────────────────────
 *  MCU       : FYSETC E4 V1.0  (ESP32-WROOM-32 @ 240 MHz)
 *  Drivers   : 2× TMC2209 (UART, addresses 1 & 2)
 *  AZM motor : NEMA 17 → Harmonic Drive 100:1  (StealthChop, 16 µstep, 600 mA)
 *  ALT motor : NEMA 17 → UMOT worm 30:1 → T8 lead screw → crank (SpreadCycle, 4 µstep, 300 mA)
 *  Sensor    : MPU-6500 on I2C (SCL=GPIO18, SDA=GPIO19 — repurposed SD card pins)
 *  Limit sw  : Active-LOW, GPIO34 (Z-MIN header)
 *  Home btn  : Active-LOW, GPIO35 (Y-MIN header)
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * CHANGELOG
 * ──────────────────────────────────────────────────────────────────────────────────────
 * v15.03g-p1 vs v15.03g:
 *   FIX : AZM hold current 10% → 50% of run current (60 mA → 300 mA).
 *         Harmonic drive is not self-locking — 60 mA was insufficient to hold
 *         position against the flex spline return torque under load.
 *         New constant: AZM_HOLD_MULTIPLIER = 0.5f
 *
 * v15.03g vs v15.03 (Gemini review):
 *   FIX : AZM:ZERO now calls resetAzmLearning() — prevents stale learning state after
 *         a referential reset (azmLrnPrevDeltaDeg was left pointing at a phantom delta).
 *
 * v15.03 vs v15.01:
 *   NEW : activeStepsPerDegAZM — learnable AZM ratio, mirrors the ALT ML system.
 *         Stored in EEPROM at offset 12 (spare slot in existing 16-byte layout).
 *         EWMA 5% (conservative — harmonic drive ratio is stable).
 *         Band ±10% around theoretical (ALT uses ±20%).
 *   FIX : Triple guard prevents the 3rd-jog AZM deadlock seen in v15.02:
 *         (1) effectiveMoved ≥ 0.5' before division  → no NaN from tiny denominator
 *         (2) isnan + isinf + in-band check           → no deadlock from corrupt ratio
 *         (3) azmLrnValid = false on reversal + tiny  → no stale state reuse
 *   NEW : resetAzmLearning() — single atomic function, called from softReset(),
 *         startHoming(), direction reversal, and AZM:ZERO.
 *
 * v15.01 vs v14.70:
 *   CHG : ALT_MOTOR_GEARBOX 496 → 148.8  (UMOT 30:1 × 4.96 crank)
 *   CHG : ALT_LIMIT_POS 5° → 10°         (V2 tilt plate wider travel range)
 *   CHG : RAMP_CRUISE_ALT_US 120 → 200   (slower cruise for 30:1 torque profile)
 *
 * v14.70 vs v14.69:
 *   FIX : Homing state survives DTR-triggered reboots (GUI ↔ TPPA switch).
 *         mpuOffset + magic word saved to EEPROM on homing completion.
 *         On boot with valid magic: firmware restores mpuOffset, reads MPU to
 *         reconstruct posDegALT, sets homingDone=true — no re-homing needed.
 *         AZM resets to 0 on every reboot (no absolute sensor — use AZM:ZERO).
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * EEPROM LAYOUT (16 bytes total)
 * ──────────────────────────────────────────────────────────────────────────────────────
 *  Offset  0 : float  activeStepsPerDegALT   (learned ALT ratio)
 *  Offset  4 : float  mpuOffset              (gyroscope tare value, saved at homing)
 *  Offset  8 : uint32 HOMING_MAGIC           (0x484F4D45 = "HOME" — homing validity)
 *  Offset 12 : float  activeStepsPerDegAZM   (learned AZM ratio)
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * UNIT CONVENTION
 * ──────────────────────────────────────────────────────────────────────────────────────
 *  TPPA / GRBL protocol : arcminutes  (MPos X/Y, $J= values)
 *  Firmware internals   : degrees     (posDegAZM, posDegALT, all math)
 *  Direct serial cmds   : degrees     (ALT:, AZM:, HOME, DIAG — bench testing)
 *
 * Conversion applied at the $J= entry point only:
 *   incoming arcmin × ARCMIN_TO_DEG → internal degrees
 *   outgoing degrees × DEG_TO_ARCMIN → MPos arcmin
 *****************************************************************************************/

#include <TMCStepper.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <stdarg.h>

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 1 — HARDWARE SETTINGS
   Edit these constants to match your physical build.
   The GUI "Firmware Config" tab generates this block for copy-paste.
   ═══════════════════════════════════════════════════════════════════════════════════════ */

/* ── Stepper motor ── */
constexpr float    MOTOR_FULL_STEPS   = 200.0f;   // Standard 1.8° NEMA 17 = 200 steps/rev

/* ── Microstepping — affects resolution and torque mode ── */
constexpr uint16_t MICROSTEPPING_AZM  = 16;        // StealthChop mode: silent at cost of torque
constexpr uint16_t MICROSTEPPING_ALT  = 4;         // SpreadCycle mode: maximum torque for lift

/* ── Gear ratios ── */
constexpr float    GEAR_RATIO_AZM     = 100.0f;    // Harmonic drive: 100 motor turns = 1 output turn

// ALT total ratio = UMOT worm gearbox × T8 crank-arm geometry.
// UMOT 30:1 (tested) × 4.96 (measured crank factor) = 148.8
// For UMOT 100:1: ALT_MOTOR_GEARBOX = 496.0f
// The firmware learns the true ratio via MPU feedback — this is only the starting point.
constexpr float    ALT_MOTOR_GEARBOX  = 148.8f;

/* ── ALT kinematics ── */
constexpr float    ALT_SCREW_PITCH_MM = 2.0f;      // T8 lead screw: 2 mm linear travel per revolution
constexpr float    ALT_RADIUS_MM      = 60.0f;     // Pivot-to-screw horizontal distance (mm)
                                                    //   Converts linear screw travel → angular tilt

/* ── Axis direction — flip to 'false' if your mount moves the wrong way ── */
constexpr bool     AXIS_REV_AZM       = true;
constexpr bool     AXIS_REV_ALT       = true;

/* ── Homing ── */
constexpr float    HOME_SAFETY_MARGIN = 0.2f;      // Pull-off distance after limit switch triggers (°)

/* ── Motor currents (mA RMS) ──
   ALT is deliberately low: 300 mA keeps the UMOT housing cool (~0.2 W).
   At full 800 mA the housing reaches 60°C — hot enough to soften PLA mounts.
   Torque margin at 300 mA is still ≥23× for any payload within spec.
   Raise to 400 mA only in extreme cold (grease thickens below −10°C). ── */
constexpr uint16_t RMS_CURRENT_AZM    = 600;
constexpr uint16_t RMS_CURRENT_ALT    = 300;

// AZM hold current multiplier.
// Harmonic drive is NOT self-locking — the motor must actively hold position.
// 0.1 (default) = 60 mA → insufficient against flex spline return torque.
// 0.5 = 300 mA → solid hold, still well within thermal limits (harmonic
//   drive is open / ventilated, unlike the enclosed UMOT housing).
// ALT hold stays at 0.1 (10% × 300 mA = 30 mA) — T8 screw is self-locking.
constexpr float    AZM_HOLD_MULTIPLIER  = 0.5f;

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 2 — TRAVEL LIMITS
   Software endstops. The firmware clamps any target outside these bounds.
   ALT 0° = homed (limit switch) position.
   AZM 0° = position at power-on (no absolute sensor).
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr float AZM_LIMIT_NEG = -30.0f;
constexpr float AZM_LIMIT_POS =  30.0f;
constexpr float ALT_LIMIT_NEG =   0.0f;
constexpr float ALT_LIMIT_POS =  10.0f;   // V2 tilt plate (5° for V1)

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 3 — TPPA UNIT CONVERSION
   TPPA (and GRBL) communicate in arcminutes. The firmware works in degrees internally.
   Conversion happens once, at the $J= command entry point.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr float ARCMIN_TO_DEG = 1.0f / 60.0f;
constexpr float DEG_TO_ARCMIN = 60.0f;

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 4 — PIN ASSIGNMENTS
   FYSETC E4 V1.0 specific. Do not change unless you have a different board.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr uint8_t PIN_EN          = 25;   // Active LOW — enables all drivers simultaneously
constexpr uint8_t PIN_DIR_AZM     = 26;
constexpr uint8_t PIN_STEP_AZM    = 27;
constexpr uint8_t PIN_DIR_ALT     = 32;
constexpr uint8_t PIN_STEP_ALT    = 33;
constexpr uint8_t PIN_HOME_SENSOR = 34;   // Z-MIN header, active LOW
constexpr uint8_t PIN_BUTTON_HOME = 35;   // Y-MIN header, active LOW

#define PIN_SERIAL_RX 21    // TMC2209 UART shared bus RX
#define PIN_SERIAL_TX 22    // TMC2209 UART shared bus TX
#define R_SENSE 0.11f       // TMC2209 sense resistor on FYSETC E4
#define ADDR_AZM 1          // TMC2209 UART address for AZM driver
#define ADDR_ALT 2          // TMC2209 UART address for ALT driver

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 5 — MPU-6500 (I2C)
   Hijacked SD card pins. See README "SD Card Hack" section.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr uint8_t SDA_PIN  = 19;   // SD Card MISO repurposed as I2C SDA
constexpr uint8_t SCL_PIN  = 18;   // SD Card SCK  repurposed as I2C SCL
constexpr uint8_t MPU_ADDR = 0x68; // MPU-6500 default I2C address (AD0 = GND)

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 6 — ALT MACHINE LEARNING THRESHOLDS
   The MPU-6500 measures actual tilt after each ALT move and updates the gear ratio.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr float HOME_TRIGGER_ANGLE    = 0.0f;   // ALT angle assigned at homing (degrees)
constexpr float ALT_TOLERANCE_DEG     = 0.05f;  // Minimum ALT move to start MPU observation
constexpr float MIN_LEARNING_ANGLE    = 0.5f;   // Minimum ALT move to update the learned ratio

// Sanity band: reject ratio updates outside ±20% of the theoretical value.
// Protects against MPU noise or hardware anomalies corrupting the ratio.
constexpr float RATIO_BAND_LOW        = 0.8f;
constexpr float RATIO_BAND_HIGH       = 1.2f;

// EWMA smoothing: new_ratio = old × 0.90 + measured × 0.10
// At 10%, 7 observations to reach 50% blend — safe convergence over a TPPA session.
constexpr float LEARNING_SMOOTHING    = 0.10f;

constexpr float LEARNING_MIN_ACTUAL   = 0.1f;   // MPU must measure ≥0.1° actual movement
constexpr float EEPROM_WRITE_THRESHOLD = 0.5f;  // Only write EEPROM if ratio changed by ≥0.5

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 7 — EEPROM LAYOUT
   16 bytes total — fits in a single 32-byte EEPROM page for atomic updates.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr int      EEPROM_SIZE          = 16;
constexpr int      EEPROM_ADDR_RATIO    = 0;    // float (4 bytes): activeStepsPerDegALT
constexpr int      EEPROM_ADDR_MPU_OFF  = 4;    // float (4 bytes): mpuOffset (gyro tare)
constexpr int      EEPROM_ADDR_MAGIC    = 8;    // uint32 (4 bytes): homing validity flag
constexpr int      EEPROM_ADDR_AZM_RATIO = 12;  // float (4 bytes): activeStepsPerDegAZM
constexpr uint32_t HOMING_MAGIC         = 0x484F4D45; // "HOME" in ASCII

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 8 — AZM RATIO LEARNING PARAMETERS
   No sensor on AZM — the firmware infers the gear ratio from TPPA residuals.
   Formula: effectiveMoved = prevDelta − currDelta
            measuredRatio  = currentRatio × prevDelta / effectiveMoved
   Three guards prevent the 3rd-jog deadlock (see v15.02 post-mortem in header).
   ═══════════════════════════════════════════════════════════════════════════════════════ */
// Tighter band than ALT (±10% vs ±20%) — harmonic drive ratio is very stable.
constexpr float AZM_RATIO_BAND_LOW     = 0.90f;
constexpr float AZM_RATIO_BAND_HIGH    = 1.10f;

// More conservative smoothing than ALT (5% vs 10%) — AZM signal is noisier
// (plate-solve residuals conflate AZM error with flexure / seeing / ALT coupling).
constexpr float AZM_LEARNING_SMOOTHING = 0.05f;

// Guard 1: minimum jog size to record learning state (below this = noise)
constexpr float MIN_AZM_LEARNING_ANGLE = 1.0f / 60.0f;  // 1 arcmin in degrees

// Guard 1: minimum effectiveMoved before division (prevents NaN when prevDelta ≈ currDelta)
constexpr float AZM_EFFECTIVE_MIN_DEG  = 0.5f / 60.0f;  // 0.5 arcmin in degrees

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 9 — MPU SAMPLING & TIMING
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr unsigned long SETTLE_DELAY_MS      = 500;   // Post-move settle before MPU sampling
constexpr uint8_t       MPU_SAMPLE_TARGET    = 50;    // Number of samples to average
constexpr unsigned long MPU_SAMPLE_INTERVAL_MS = 5;   // 5 ms between samples = 250 ms total

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 10 — MOTION RAMP PARAMETERS
   Trapezoidal velocity profile: start slow → cruise → decelerate.
   All values in microseconds between steps (smaller = faster).
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr unsigned long RAMP_START_US     = 2000;  // Starting speed (slowest)
constexpr unsigned long RAMP_CRUISE_AZM_US = 240;  // AZM cruise speed
constexpr unsigned long RAMP_CRUISE_ALT_US = 200;  // ALT cruise speed (slower for 30:1 torque)
constexpr long          RAMP_LENGTH        = 3000; // Steps to reach cruise speed

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 11 — FEEDBACK REPORT SCALING
   During the MPU observation phase, the reported ALT position is compressed so it
   never quite reaches the target. This prevents TPPA from reclaiming control before
   the observation completes. When done, position snaps to exact target + <Idle>.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr float FEEDBACK_REPORT_MARGIN = 0.10f;  // Hold this many degrees below target
constexpr float FEEDBACK_MIN_SCALE     = 0.50f;  // Never compress below 50% of real progress

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 12 — GLOBAL SETTLE & BACKLASH
   ═══════════════════════════════════════════════════════════════════════════════════════ */
// Anti-vibration delay after every move before reporting <Idle>.
// Prevents TPPA from plate-solving on a still-vibrating mount.
constexpr unsigned long GLOBAL_SETTLE_MS = 2000;

// AZM backlash: harmonic drive elastic deformation (lost motion) on direction reversal.
// Extra steps injected at start of reversal — motor moves but position isn't updated.
// ~2 arcminutes is typical for a 100:1 harmonic drive under load.
constexpr float AZM_BACKLASH_DEG = 0.033f;  // 2' = 0.033°

/* ═══════════════════════════════════════════════════════════════════════════════════════
   COMPUTED KINEMATICS (do not edit — derived from Section 1)
   ═══════════════════════════════════════════════════════════════════════════════════════ */
// AZM: steps = motor_steps × microstepping × gear_ratio / 360°
constexpr float STEPS_PER_DEG_AZM =
    (MOTOR_FULL_STEPS * MICROSTEPPING_AZM * GEAR_RATIO_AZM) / 360.0f;

// ALT: steps = (motor_steps × microstepping × gearbox) / screw_pitch × arc_per_deg
//   arc_per_deg = (2π × radius_mm) / 360°   [converts linear mm → angular deg]
constexpr float STEPS_PER_MM_ALT =
    (MOTOR_FULL_STEPS * MICROSTEPPING_ALT * ALT_MOTOR_GEARBOX) / ALT_SCREW_PITCH_MM;
constexpr float MM_PER_DEGREE_ALT =
    (2.0f * (float)M_PI * ALT_RADIUS_MM) / 360.0f;
constexpr float STEPS_PER_DEG_ALT = STEPS_PER_MM_ALT * MM_PER_DEGREE_ALT;

/* ═══════════════════════════════════════════════════════════════════════════════════════
   RUNTIME STATE — learned ratios (start at theoretical, updated by ML)
   ═══════════════════════════════════════════════════════════════════════════════════════ */
float mpuOffset           = 0.0f;    // Gyro tare value set at homing
bool  mpuAvailable        = false;   // true if MPU-6500 responded on I2C

float activeStepsPerDegALT = STEPS_PER_DEG_ALT; // Updated by MPU learning
float activeStepsPerDegAZM = STEPS_PER_DEG_AZM; // Updated by residual learning

/* ═══════════════════════════════════════════════════════════════════════════════════════
   GLOBAL STATE
   ═══════════════════════════════════════════════════════════════════════════════════════ */
volatile float posDegAZM = 0.0f;    // Current AZM position (degrees, from power-on zero)
volatile float posDegALT = 0.0f;    // Current ALT position (degrees, from homing zero)
volatile bool  feedHold  = false;   // Feed-hold active (GRBL '!' command)
volatile bool  isMoving  = false;   // Any motion or settling in progress
volatile bool  abortCmd  = false;   // Soft-reset requested (GRBL 0x18 command)

bool  inFeedbackCycle    = false;   // true during MPU observation (ALT scaling active)
float feedbackStartPos   = 0.0f;    // ALT position at start of current feedback cycle

bool          settlingForObserve = false; // true during post-move MPU observation phase
unsigned long settleStartMs      = 0;
float         targetAltAngle     = 0.0f; // ALT target for current move
float         learningStartAngle = 0.0f; // MPU angle at start of move (for ML delta)
float         learningRequestedDelta = 0.0f; // Commanded ALT delta (for ML ratio)
bool          lastMoveWasUp      = true; // Direction of last ALT move (skip ML on reversal)

bool  homingDone = false;  // TPPA jogs blocked until true

// AZM backlash tracking — separate from AZM learning state (independent concerns)
int8_t lastAzmDir = 0;  // +1 = last move positive, -1 = negative, 0 = unknown

/* ── AZM Learning state ──
   resetAzmLearning() is the SINGLE point of truth for clearing this.
   Called from: softReset(), startHoming(), direction reversal, AZM:ZERO. ── */
float  azmLrnPrevDeltaDeg = 0.0f;  // Signed delta from the previous AZM jog (degrees)
int8_t azmLrnPrevDir      = 0;     // Direction of previous AZM jog (+1 / -1)
bool   azmLrnValid        = false; // true only when previous jog data is trustworthy

/* ── MPU sampling accumulators ── */
int           mpuSampleCount  = 0;
float         mpuSumAngles    = 0.0f;
unsigned long lastMpuSampleMs = 0;

/* ── Global settle ── */
bool          waitingForGlobalSettle = false;
unsigned long globalSettleStartMs   = 0;

/* ═══════════════════════════════════════════════════════════════════════════════════════
   DIAGNOSTIC LOG
   4 KB ring buffer accumulates all learning events, errors, and jog details.
   N.I.N.A. never sees this — retrieved on demand via the DIAG serial command.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
static char    diagLog[4096];
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

/* ═══════════════════════════════════════════════════════════════════════════════════════
   AZM LEARNING — ATOMIC RESET HELPER
   All code that needs to wipe AZM learning state calls this function.
   Having a single function prevents the "forgot one callsite" class of bugs
   (which caused the deadlock in v15.02 for the AZM:ZERO case).
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void resetAzmLearning() {
  azmLrnPrevDeltaDeg = 0.0f;
  azmLrnPrevDir      = 0;
  azmLrnValid        = false;
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   TMC2209 DRIVER OBJECTS
   Shared UART bus: both drivers on Serial2, differentiated by UART address.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
HardwareSerial& SerialDrivers = Serial2;
TMC2209Stepper drvAzm(&SerialDrivers, R_SENSE, ADDR_AZM);
TMC2209Stepper drvAlt(&SerialDrivers, R_SENSE, ADDR_ALT);

/* ═══════════════════════════════════════════════════════════════════════════════════════
   MPU-6500 — INIT & READ
   ═══════════════════════════════════════════════════════════════════════════════════════ */

// Silent init: tries I2C, sets mpuAvailable. No serial output (N.I.N.A. can't handle it).
void initMPU_Silent() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // 100 kHz — conservative, safe over long wires
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); Wire.write(0x00);   // PWR_MGMT_1: wake up (clear sleep bit)
    Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); Wire.write(0x00);   // ACCEL_CONFIG: ±2g range (maximum sensitivity)
    Wire.endTransmission(true);
    mpuAvailable = true;
  } else {
    mpuAvailable = false;
  }
}

// Returns the tilt angle of the sensor board around the Y axis (degrees).
// Uses atan2(AcX, sqrt(AcY² + AcZ²)) — measures pitch relative to gravity.
// Returns -999.0 on any I2C failure (caller checks for this sentinel value).
float readMPUAngleY() {
  if (!mpuAvailable) return -999.0f;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H — first register of 6-byte accelerometer block
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

/* ═══════════════════════════════════════════════════════════════════════════════════════
   NON-BLOCKING MOTION ENGINE
   ═══════════════════════════════════════════════════════════════════════════════════════
   Architecture: a 2-slot job queue feeds a single active motor state (mot).
   startNextJob() pops the queue; tickMotion() fires one step per loop iteration.
   No delay() calls — the loop runs at ~100 kHz so N.I.N.A.'s 10 Hz polls are
   serviced without interruption.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
struct MotionJob {
  uint8_t stepPin;
  uint8_t dirPin;
  float   deltaDeg;
  float   stepsPerDeg;
  volatile float* globalPos;
};

// Active motor state — one axis at a time
struct {
  bool     active;
  uint8_t  stepPin;
  long     stepsRemaining;
  float    stepDegInc;       // Degrees per step (signed, for position accumulation)
  float    targetPos;        // Final position after this job
  float    stepsPerDeg;
  volatile float* globalPos;
  bool     isAzm;
  float    deltaDeg;         // Requested delta for this job
  unsigned long lastStepUs;  // Timestamp of last step pulse (µs)
  long     totalSteps;       // Total steps for this job (used for ramp calculation)
  long     stepsDone;        // Steps executed so far
  long     backlashSteps;    // Dead steps at start (position not updated during these)
} mot = {false};

MotionJob jobQueue[2];
uint8_t   jobCount = 0;

void enqueueMotion(uint8_t stepPin, uint8_t dirPin, float deltaDeg,
                   float stepsPerDeg, volatile float* globalPos) {
  if (jobCount >= 2) return;  // Queue full — caller must flush first
  jobQueue[jobCount++] = {stepPin, dirPin, deltaDeg, stepsPerDeg, globalPos};
}

// Forward declarations (implementations below)
void performPullOff(float stepsPerDeg);
void softReset();
void sendStatus();
void scanSerialRealtime();

// Pops next job from queue, sets up AZM backlash + ALT MPU learning, starts motor.
// Returns false if queue is empty.
bool startNextJob() {
  if (jobCount == 0) return false;

  MotionJob job = jobQueue[0];
  jobQueue[0] = jobQueue[1];  // Shift queue
  jobCount--;

  long stepsTot = lroundf(job.deltaDeg * job.stepsPerDeg);
  if (!stepsTot) return startNextJob();  // Skip zero-step jobs (clamped travel)

  bool isAzm     = (job.dirPin == PIN_DIR_AZM);
  bool inv       = isAzm ? AXIS_REV_AZM : AXIS_REV_ALT;
  bool logicalDir = (stepsTot > 0) ? !inv : inv;
  bool isUp      = (job.deltaDeg > 0);

  /* ── AZM BACKLASH COMPENSATION ──
     On direction reversal, inject extra dead steps to eat through the harmonic
     drive's elastic deformation zone. Motor moves, but posDegAZM is not updated
     until the actual movement begins. TPPA sees a clean, uninterrupted position. ── */
  long backlashExtra = 0;
  if (isAzm && AZM_BACKLASH_DEG > 0.0f) {
    int8_t newDir = (stepsTot > 0) ? 1 : -1;
    if (lastAzmDir != 0 && newDir != lastAzmDir) {
      backlashExtra = lroundf(AZM_BACKLASH_DEG * job.stepsPerDeg);
      diagPrintf("AZM BACKLASH: %ld dead steps (%.1f')\n",
                 backlashExtra, AZM_BACKLASH_DEG * 60.0f);
    }
    lastAzmDir = newDir;
  }

  /* ── ALT MPU LEARNING SETUP ──
     Capture the start angle before motion begins. Only valid for same-direction
     moves (direction reversals would include backlash, corrupting the ratio). ── */
  if (!isAzm && mpuAvailable) {
    if (isUp == lastMoveWasUp) {
      float startRaw = readMPUAngleY();
      if (startRaw > -900.0f) {
        learningStartAngle    = startRaw - mpuOffset;
        learningRequestedDelta = job.deltaDeg;
      } else {
        learningRequestedDelta = 0.0f;  // I2C failure — skip learning this jog
      }
    } else {
      learningRequestedDelta = 0.0f;  // Direction reversal — backlash would corrupt
    }
    lastMoveWasUp = isUp;

    if (fabsf(job.deltaDeg) >= ALT_TOLERANCE_DEG) {
      inFeedbackCycle  = true;
      feedbackStartPos = *job.globalPos;
      targetAltAngle   = *job.globalPos + job.deltaDeg;
    }
  }

  // Set direction pin and give the driver 50µs to settle
  digitalWrite(job.dirPin, logicalDir ? HIGH : LOW);
  delayMicroseconds(50);

  // Load the active motor state
  mot.active         = true;
  mot.stepPin        = job.stepPin;
  mot.stepsRemaining = labs(stepsTot) + backlashExtra;
  mot.backlashSteps  = backlashExtra;
  mot.stepDegInc     = job.deltaDeg / (float)labs(stepsTot);
  mot.targetPos      = *job.globalPos + job.deltaDeg;
  mot.stepsPerDeg    = job.stepsPerDeg;
  mot.globalPos      = job.globalPos;
  mot.isAzm          = isAzm;
  mot.deltaDeg       = job.deltaDeg;
  mot.lastStepUs     = micros();
  mot.totalSteps     = mot.stepsRemaining;
  mot.stepsDone      = 0;
  isMoving           = true;
  return true;
}

void enterGlobalSettle() {
  waitingForGlobalSettle = true;
  globalSettleStartMs    = millis();
  diagPrintf("SETTLE: 2s\n");
}

/* ─────────────────────────────────────────────────────────────────────────────────────
   tickMotion() — called every loop iteration
   State machine: abort → global settle → MPU observe → motor pulse
   ───────────────────────────────────────────────────────────────────────────────────── */
void tickMotion() {

  /* ── ABORT: wipe everything on soft-reset request ── */
  if (abortCmd) {
    if (mot.active || settlingForObserve || waitingForGlobalSettle) {
      mot.active             = false;
      isMoving               = false;
      jobCount               = 0;
      settlingForObserve     = false;
      waitingForGlobalSettle = false;
      inFeedbackCycle        = false;
      learningRequestedDelta = 0;
      softReset();
    }
    return;
  }

  /* ── GLOBAL SETTLE: 2-second anti-vibration delay before <Idle> ── */
  if (waitingForGlobalSettle) {
    if (millis() - globalSettleStartMs < GLOBAL_SETTLE_MS) return;
    waitingForGlobalSettle = false;
    isMoving               = false;
    diagPrintf("IDLE\n");
    return;
  }

  /* ── MPU OBSERVE PHASE: sample gyro, update ALT ratio ──
     Entered after every ALT move above ALT_TOLERANCE_DEG.
     Sequence: 500ms settle → 50 samples → compute ratio → EEPROM if changed ── */
  if (settlingForObserve) {

    // Phase 1: mechanical settle (vibrations damp)
    if (millis() - settleStartMs < SETTLE_DELAY_MS) return;

    // Phase 2: accumulate MPU samples
    if (millis() - lastMpuSampleMs >= MPU_SAMPLE_INTERVAL_MS) {
      float r = readMPUAngleY();
      if (r > -900.0f) {
        mpuSumAngles  += r;
        mpuSampleCount++;
      }
      lastMpuSampleMs = millis();
    }
    if (mpuSampleCount < MPU_SAMPLE_TARGET) return;

    // Phase 3: compute average and update ratio
    float rawAngle         = mpuSumAngles / (float)mpuSampleCount;
    settlingForObserve     = false;
    mpuSampleCount         = 0;
    mpuSumAngles           = 0.0f;

    float actualAngle = rawAngle - mpuOffset;
    float error       = targetAltAngle - actualAngle;

    diagPrintf("MPU: act=%.3f tgt=%.3f err=%.3f (observe)\n",
               actualAngle, targetAltAngle, error);

    /* MACHINE LEARNING RATIO ADAPTATION (ALT axis)
       If the actual movement differs from commanded, adjust activeStepsPerDegALT.
       EWMA blend: 10% new measurement, 90% running average.
       Sanity band: only accept measurements within ±20% of theoretical. */
    if (fabsf(learningRequestedDelta) >= MIN_LEARNING_ANGLE) {
      float actualMoved = actualAngle - learningStartAngle;
      if (fabsf(actualMoved) > LEARNING_MIN_ACTUAL) {
        float stepsSent     = learningRequestedDelta * activeStepsPerDegALT;
        float measuredRatio = stepsSent / actualMoved;

        if (measuredRatio > (STEPS_PER_DEG_ALT * RATIO_BAND_LOW) &&
            measuredRatio < (STEPS_PER_DEG_ALT * RATIO_BAND_HIGH)) {
          float oldRatio      = activeStepsPerDegALT;
          activeStepsPerDegALT = (activeStepsPerDegALT * (1.0f - LEARNING_SMOOTHING))
                               + (measuredRatio * LEARNING_SMOOTHING);
          if (fabsf(activeStepsPerDegALT - oldRatio) > EEPROM_WRITE_THRESHOLD) {
            EEPROM.put(EEPROM_ADDR_RATIO, activeStepsPerDegALT);
            EEPROM.commit();
          }
          diagPrintf("ML Ratio: %.2f (was %.2f)\n", activeStepsPerDegALT, oldRatio);
        }
      }
    }

    /* No motor correction — TPPA's plate-solve loop handles convergence.
       Snap position to exact target and report done. */
    inFeedbackCycle = false;
    posDegALT       = targetAltAngle;
    diagPrintf("DONE: reported=%.3f mpuActual=%.3f\n", posDegALT, actualAngle);
    if (!startNextJob()) enterGlobalSettle();
    return;
  }

  /* ── MOTOR PULSE GENERATION (trapezoidal ramp) ── */
  if (!mot.active || feedHold) return;

  // Ramp position = min(steps done, steps remaining, RAMP_LENGTH)
  // This gives symmetric acceleration and deceleration.
  long rampPos    = mot.stepsDone;
  long stepsFromEnd = mot.stepsRemaining;
  if (stepsFromEnd < rampPos) rampPos = stepsFromEnd;
  if (rampPos > RAMP_LENGTH)  rampPos = RAMP_LENGTH;

  unsigned long cruiseUs = mot.isAzm ? RAMP_CRUISE_AZM_US : RAMP_CRUISE_ALT_US;
  unsigned long interval;
  if (rampPos < RAMP_LENGTH) {
    // Linear interpolation from RAMP_START_US down to cruiseUs
    interval = RAMP_START_US - ((RAMP_START_US - cruiseUs) * rampPos) / RAMP_LENGTH;
  } else {
    interval = cruiseUs;
  }

  unsigned long now = micros();
  if (now - mot.lastStepUs < interval) return;  // Not time yet — yield
  mot.lastStepUs = now;

  // Position update: skip during backlash dead steps (motor moves, position doesn't)
  if (mot.stepsDone >= mot.backlashSteps) {
    *mot.globalPos += mot.stepDegInc;
  }

  // Fire step pulse (60µs HIGH, then LOW — well within TMC2209 minimum pulse width)
  digitalWrite(mot.stepPin, HIGH); delayMicroseconds(60);
  digitalWrite(mot.stepPin, LOW);
  mot.stepsRemaining--;
  mot.stepsDone++;

  scanSerialRealtime();  // Service '?' polls mid-move for smooth N.I.N.A. status display

  /* ── HARDWARE SAFETY LIMIT ──
     If the physical limit switch triggers during a downward ALT move (outside homing),
     stop immediately, pull off, and redefine position as 0° (re-home in place). ── */
  if (!mot.isAzm && mot.deltaDeg < 0 && digitalRead(PIN_HOME_SENSOR) == LOW) {
    Serial.println("\n!!! CRITICAL ALARM: PHYSICAL LIMIT SWITCH HIT OUTSIDE HOMING !!!");
    mot.active             = false;
    isMoving               = false;
    jobCount               = 0;
    inFeedbackCycle        = false;
    waitingForGlobalSettle = false;
    performPullOff(mot.stepsPerDeg);
    *mot.globalPos = HOME_TRIGGER_ANGLE;
    posDegALT      = HOME_TRIGGER_ANGLE;
    sendStatus(); Serial.println();
    return;
  }

  /* ── JOB COMPLETE ── */
  if (mot.stepsRemaining <= 0) {
    mot.active = false;

    // For ALT moves above tolerance: enter MPU observation instead of settling directly.
    // This is where the machine learning measurement happens.
    if (!mot.isAzm && mpuAvailable && !feedHold && fabsf(mot.deltaDeg) >= ALT_TOLERANCE_DEG) {
      diagPrintf("Observe: tgt=%.3f start=%.3f\n", targetAltAngle, feedbackStartPos);
      settlingForObserve = true;
      settleStartMs      = millis();
      mpuSampleCount     = 0;
      mpuSumAngles       = 0.0f;
      lastMpuSampleMs    = 0;
      return;
    }

    // No observation needed: snap to exact target, start next job or enter settle
    *mot.globalPos = mot.targetPos;
    if (!startNextJob()) enterGlobalSettle();
  }
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SERIAL PROTOCOL
   ═══════════════════════════════════════════════════════════════════════════════════════ */

// Formats and sends the GRBL status response.
// During MPU observation, ALT position is scaled slightly below target to keep
// TPPA's progress display moving without triggering premature completion.
void sendStatus() {
  float reportALT = posDegALT;

  if (inFeedbackCycle) {
    float totalDelta = targetAltAngle - feedbackStartPos;
    float absDelta   = fabsf(totalDelta);
    if (absDelta > 0.001f) {
      float scaleFactor = (absDelta - FEEDBACK_REPORT_MARGIN) / absDelta;
      if (scaleFactor < FEEDBACK_MIN_SCALE) scaleFactor = FEEDBACK_MIN_SCALE;
      float realProgress = posDegALT - feedbackStartPos;
      reportALT = feedbackStartPos + (realProgress * scaleFactor);
    }
  }

  // All MPos values reported in arcminutes for TPPA compatibility
  float mposAZM = posDegAZM * DEG_TO_ARCMIN;
  float mposALT = reportALT * DEG_TO_ARCMIN;

  Serial.print('<');
  if (feedHold)    Serial.print("Hold");
  else if (isMoving) Serial.print("Run");
  else             Serial.print("Idle");
  Serial.print("|MPos:"); Serial.print(mposAZM, 3);
  Serial.print(','); Serial.print(mposALT, 3);
  Serial.println(",0|");
}

// Polled from inside the motor ISR to service GRBL realtime characters mid-move.
// This is what gives N.I.N.A. smooth status updates even during long moves.
void scanSerialRealtime() {
  if (!Serial.available()) return;
  char c = Serial.peek();
  if      (c == '?')    { Serial.read(); sendStatus(); Serial.println(); }
  else if (c == '!')    { Serial.read(); feedHold = true;  Serial.println("ok"); }
  else if (c == '~')    { Serial.read(); feedHold = false; Serial.println("ok"); }
  else if (c == 0x18)   { Serial.read(); abortCmd = true; }
}

void softReset() {
  feedHold               = false;
  isMoving               = false;
  abortCmd               = false;
  mot.active             = false;
  jobCount               = 0;
  settlingForObserve     = false;
  waitingForGlobalSettle = false;
  inFeedbackCycle        = false;
  learningRequestedDelta = 0;
  resetAzmLearning();   // Atomically clear all AZM learning state
  diagClear();
  Serial.println("\r\nGrbl 1.1h ['$' for help]");
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   HOMING SEQUENCE
   ═══════════════════════════════════════════════════════════════════════════════════════ */

// Pull-off: after the limit switch triggers, move UP until the switch releases,
// then advance by HOME_SAFETY_MARGIN to establish a clean mechanical zero.
void performPullOff(float stepsPerDeg) {
  bool dirUp = !AXIS_REV_ALT ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, dirUp);
  delay(50);

  long count       = 0;
  int  confirmHigh = 0;
  long maxSteps    = (long)(10.0f * stepsPerDeg);  // Safety cap: max 10° of pull-off

  while (count < maxSteps) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW);  delayMicroseconds(60);
    count++;
    if (count % 2000 == 0) yield();  // Feed watchdog
    if (digitalRead(PIN_HOME_SENSOR) == HIGH) {
      confirmHigh++;
      if (confirmHigh > 20) break;   // 20 consecutive HIGH readings = switch released
    } else {
      confirmHigh = 0;
    }
  }

  // Advance the additional safety margin
  long safetySteps = (long)(HOME_SAFETY_MARGIN * stepsPerDeg);
  for (long s = 0; s < safetySteps; s++) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW);  delayMicroseconds(60);
    if (s % 2000 == 0) yield();
  }
}

void startHoming() {
  Serial.println("MSG: Homing ALT axis...");
  isMoving               = true;
  inFeedbackCycle        = false;
  waitingForGlobalSettle = false;
  diagClear();

  // If already on the switch, pull off first
  if (digitalRead(PIN_HOME_SENSOR) == LOW) {
    performPullOff(activeStepsPerDegALT);
    delay(200);
  }

  // Drive DOWN toward the limit switch
  bool dirDown = AXIS_REV_ALT ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, dirDown);
  delay(10);

  int  confirmLow = 0;
  bool hit        = false;

  // Search up to 50° of travel (much more than needed, but safe)
  for (long s = 0; s < (long)(50.0f * activeStepsPerDegALT); s++) {
    digitalWrite(PIN_STEP_ALT, HIGH); delayMicroseconds(60);
    digitalWrite(PIN_STEP_ALT, LOW);  delayMicroseconds(60);
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
  if (!hit) {
    Serial.println("ALARM: Homing failed to find sensor");
    isMoving = false;
    return;
  }

  delay(200);
  performPullOff(activeStepsPerDegALT);

  // Define mechanical zero
  posDegALT = HOME_TRIGGER_ANGLE;
  posDegAZM = 0.0f;    // AZM has no absolute sensor — always resets to 0 at homing
  lastAzmDir = 0;

  // Tare the gyroscope at the homed position
  if (mpuAvailable) {
    Serial.println("MSG: Mechanical stabilization... taring Gyroscope.");
    delay(500);
    float sumAngles  = 0;
    int   validSamples = 0;
    for (int i = 0; i < MPU_SAMPLE_TARGET; i++) {
      float r = readMPUAngleY();
      if (r > -900.0f) { sumAngles += r; validSamples++; }
      delay(5);
    }
    if (validSamples > 0) {
      mpuOffset = (sumAngles / (float)validSamples) - HOME_TRIGGER_ANGLE;
      Serial.print("MSG: Gyroscope tared. Offset = "); Serial.println(mpuOffset, 3);
    } else {
      Serial.println("ALARM: Gyroscope Tare Failed (I2C Bus unresponsive).");
    }
  }

  isMoving               = false;
  homingDone             = true;
  learningRequestedDelta = 0;
  resetAzmLearning();  // Fresh start — previous session's AZM learning is invalid

  // Persist homing state to EEPROM so it survives a DTR-triggered reboot
  // (GUI and TPPA both toggle DTR when opening the serial port).
  // On next boot, if the magic is valid, the firmware reconstructs posDegALT from
  // the MPU and restores homingDone=true — no re-homing needed.
  EEPROM.put(EEPROM_ADDR_MPU_OFF, mpuOffset);
  uint32_t magic = HOMING_MAGIC;
  EEPROM.put(EEPROM_ADDR_MAGIC, magic);
  EEPROM.commit();

  Serial.println("MSG: Homing OK. TPPA jogs enabled (state saved to EEPROM).");
  sendStatus(); Serial.println();
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   DIAGNOSTIC OUTPUT
   Comprehensive system state dump — retrieved via the DIAG serial command.
   Everything that can help debug a field issue is included here.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void printDiagnostic() {
  Serial.println("\n--- SYSTEM DIAGNOSTIC (v15.03g-p1) ---");

  // Hardware inputs
  Serial.print("Limit Sensor (Pin 34) : ");
  Serial.println(digitalRead(PIN_HOME_SENSOR) == LOW ? "TRIGGERED (LOW)" : "OPEN (HIGH)");

  Serial.println("");

  // MPU status
  if (mpuAvailable) {
    float rawAngle = readMPUAngleY();
    if (rawAngle > -900.0f) {
      Serial.print("MPU-6500 Raw (Y)      : "); Serial.print(rawAngle, 3);   Serial.println(" deg");
      Serial.print("MPU-6500 Tare Offset  : "); Serial.print(mpuOffset, 3);  Serial.println(" deg");
      Serial.print("MPU-6500 Homing-Rel   : "); Serial.print(rawAngle - mpuOffset, 3); Serial.println(" deg");
    } else {
      Serial.println("MPU-6500 STATUS       : I2C ERROR");
    }
  } else {
    Serial.println("MPU-6500 STATUS       : Not detected!");
  }

  Serial.println("");

  // Position & motion state
  Serial.print("posDegAZM (deg)       : "); Serial.println(posDegAZM, 3);
  Serial.print("posDegALT (deg)       : "); Serial.println(posDegALT, 3);
  Serial.print("MPos AZM (arcmin)     : "); Serial.println(posDegAZM * DEG_TO_ARCMIN, 3);
  Serial.print("MPos ALT (arcmin)     : "); Serial.println(posDegALT * DEG_TO_ARCMIN, 3);
  Serial.print("targetAltAngle        : "); Serial.println(targetAltAngle, 3);
  Serial.print("inFeedbackCycle       : "); Serial.println(inFeedbackCycle ? "YES" : "NO");
  Serial.print("feedbackStartPos      : "); Serial.println(feedbackStartPos, 3);
  Serial.print("settlingForObserve    : "); Serial.println(settlingForObserve ? "YES" : "NO");
  Serial.print("globalSettle          : "); Serial.println(waitingForGlobalSettle ? "YES" : "NO");
  Serial.print("homingDone            : ");
  Serial.println(homingDone ? "YES — TPPA jogs enabled" : "NO — *** TPPA JOGS BLOCKED ***");

  Serial.println("");

  // AZM details
  Serial.print("AZM backlash comp     : "); Serial.print(AZM_BACKLASH_DEG * 60.0f, 1);
  Serial.print("' ("); Serial.print(AZM_BACKLASH_DEG, 4); Serial.println(" deg)");
  Serial.print("lastAzmDir            : ");
  Serial.println(lastAzmDir ==  1 ? "+1 (positive)" :
                 lastAzmDir == -1 ? "-1 (negative)" : "0 (unknown)");
  Serial.print("AZM Lrn valid         : "); Serial.println(azmLrnValid ? "YES" : "NO");
  if (azmLrnValid) {
    Serial.print("AZM Lrn prevDelta     : ");
    Serial.print(azmLrnPrevDeltaDeg * 60.0f, 2);
    Serial.print("'  dir="); Serial.println(azmLrnPrevDir == 1 ? "+1" : "-1");
  }

  Serial.println("");

  // Travel & config
  Serial.print("Travel limits AZM     : "); Serial.print(AZM_LIMIT_NEG);
  Serial.print(" to "); Serial.print(AZM_LIMIT_POS); Serial.println(" deg");
  Serial.print("Travel limits ALT     : "); Serial.print(ALT_LIMIT_NEG);
  Serial.print(" to "); Serial.print(ALT_LIMIT_POS); Serial.println(" deg");
  Serial.print("Global settle time    : "); Serial.print(GLOBAL_SETTLE_MS); Serial.println(" ms");
  Serial.print("Jog unit conversion   : arcmin → deg (×"); Serial.print(ARCMIN_TO_DEG, 5); Serial.println(")");

  Serial.println("");

  // Learned ratios
  Serial.print("Active ALT Ratio      : "); Serial.print(activeStepsPerDegALT, 3); Serial.println(" steps/deg");
  Serial.print("Theoretical ALT Ratio : "); Serial.print(STEPS_PER_DEG_ALT, 3);    Serial.println(" steps/deg");
  Serial.print("Active AZM Ratio      : "); Serial.print(activeStepsPerDegAZM, 3); Serial.println(" steps/deg");
  Serial.print("Theoretical AZM Ratio : "); Serial.print(STEPS_PER_DEG_AZM, 3);    Serial.println(" steps/deg");
  Serial.print("ALT RMS Current       : "); Serial.print(RMS_CURRENT_ALT); Serial.println(" mA");
  Serial.print("AZM Cruise            : "); Serial.print(RAMP_CRUISE_AZM_US); Serial.println(" µs");
  Serial.print("ALT Cruise            : "); Serial.print(RAMP_CRUISE_ALT_US); Serial.println(" µs");
  Serial.print("Ramp length           : "); Serial.print(RAMP_LENGTH); Serial.println(" steps");

  Serial.println("");

  // EEPROM
  Serial.print("EEPROM ALT Ratio      : ");
  float stored = 0.0f; EEPROM.get(EEPROM_ADDR_RATIO, stored); Serial.println(stored, 3);
  Serial.print("EEPROM AZM Ratio      : ");
  float storedAzm = 0.0f; EEPROM.get(EEPROM_ADDR_AZM_RATIO, storedAzm); Serial.println(storedAzm, 3);
  uint32_t storedMagic = 0; EEPROM.get(EEPROM_ADDR_MAGIC, storedMagic);
  Serial.print("EEPROM Homing State   : ");
  Serial.println(storedMagic == HOMING_MAGIC ?
                 "SAVED (persists across reboot)" : "NOT SAVED");
  Serial.print("Diag buffer used      : "); Serial.print(diagLen);
  Serial.print("/"); Serial.println(sizeof(diagLog));

  // Command & learning log
  if (diagLen > 0) {
    Serial.println("");
    Serial.println("--- COMMAND & FEEDBACK LOG ---");
    Serial.print(diagLog);
    Serial.println("--- END LOG ---");
  }
  Serial.println("------------------------------------\n");
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   COMMAND PROCESSOR
   Handles both GRBL protocol ($J=, ?, !, ~, 0x18) and direct serial commands.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void processCommand(const char* line) {
  if (line[0] == '\0') return;

  /* ── System commands ── */
  if (strcmp(line, "RST") == 0)                      { softReset(); return; }
  if (strcmp(line, "HOME") == 0 || strcmp(line, "$H") == 0) { startHoming(); return; }
  if (strcmp(line, "DIAG") == 0 || strcmp(line, "MPU?") == 0) { printDiagnostic(); return; }

  /* ── Lightweight MPU query (GUI status bar polling, avoids full DIAG overhead) ── */
  if (strcmp(line, "MPU") == 0) {
    if (mpuAvailable) {
      float raw = readMPUAngleY();
      if (raw > -900.0f) {
        Serial.print("MPU:"); Serial.print(raw - mpuOffset, 3);
        Serial.print(","); Serial.println(raw, 3);
      } else { Serial.println("MPU:ERR"); }
    } else { Serial.println("MPU:NA"); }
    return;
  }

  /* ── TPPA / GRBL jog commands ($J=G91G21X... / $J=G53Y...) ── */
  if (strncmp(line, "$J=", 3) == 0 || strncmp(line, "J=", 2) == 0) {

    // HOMING GUARD: refuse all TPPA jogs until HOME has been executed.
    // Reply 'ok' anyway so TPPA doesn't hang waiting for a response.
    if (!homingDone) {
      Serial.println("ok");
      diagPrintf("!BLOCKED: %s (HOME not done)\n", line);
      Serial.println("MSG: *** JOG REFUSED — run HOME first! ***");
      return;
    }

    // If a previous move is still settling, snap to its target and clear state.
    // This lets TPPA issue rapid-fire corrections without waiting for each settle.
    if (isMoving) {
      if (mot.active) { *mot.globalPos = mot.targetPos; mot.active = false; }
      if (inFeedbackCycle || settlingForObserve) {
        posDegALT       = targetAltAngle;
        inFeedbackCycle = false;
      }
      jobCount               = 0;
      isMoving               = false;
      settlingForObserve     = false;
      waitingForGlobalSettle = false;
      learningRequestedDelta = 0;
    }

    const char* rest = strchr(line, '=');
    if (!rest) return;
    rest++;

    // G53 = absolute coordinate system, G91/G21 = relative
    bool rel = (strstr(rest, "G53") == nullptr);

    Serial.println("ok");
    diagPrintf("CMD: %s pos=%.3f,%.3f\n", line, posDegAZM, posDegALT);

    const char* xp = strchr(rest, 'X');  // AZM axis
    const char* yp = strchr(rest, 'Y');  // ALT axis

    /* ── AZM jog ── */
    if (xp) {
      float rawVal = atof(xp + 1);
      float val    = rawVal * ARCMIN_TO_DEG;          // arcmin → degrees
      float tgt    = rel ? (posDegAZM + val) : val;

      diagPrintf("AZM: raw=%.3f' → %.4f° tgt=%.4f°\n", rawVal, val, tgt);

      // Software endstop clamp
      if (tgt < AZM_LIMIT_NEG) { diagPrintf("!LIMIT AZM: clamped to %.1f\n", AZM_LIMIT_NEG); tgt = AZM_LIMIT_NEG; }
      if (tgt > AZM_LIMIT_POS) { diagPrintf("!LIMIT AZM: clamped to %.1f\n", AZM_LIMIT_POS); tgt = AZM_LIMIT_POS; }

      float  azmDeltaDeg = tgt - posDegAZM;
      int8_t newAzmDir   = (azmDeltaDeg >= 0.0f) ? 1 : -1;

      /* ── AZM RATIO LEARNING (sensor-free, residual-based) ──
         The algorithm infers the true gear ratio by observing consecutive TPPA
         corrections. If TPPA sent prevDelta but still needs currDelta, the mount
         only moved (prevDelta − currDelta) = effectiveMoved.
         From that: measuredRatio = currentRatio × prevDelta / effectiveMoved

         Three guards prevent the deadlock seen in v15.02:
           Guard 1 — effectiveMoved threshold: prevents NaN from tiny denominator
           Guard 2 — NaN/Inf/band check:        prevents deadlock on corrupt ratio
           Guard 3 — direction reset:            prevents stale data reuse          ── */
      if (fabsf(azmDeltaDeg) >= MIN_AZM_LEARNING_ANGLE) {

        if (azmLrnValid && newAzmDir == azmLrnPrevDir &&
            fabsf(azmLrnPrevDeltaDeg) >= MIN_AZM_LEARNING_ANGLE) {

          float prevAbs       = fabsf(azmLrnPrevDeltaDeg);
          float currAbs       = fabsf(azmDeltaDeg);
          float effectiveMoved = prevAbs - currAbs;

          if (effectiveMoved >= AZM_EFFECTIVE_MIN_DEG) {           // Guard 1
            float measuredRatio = activeStepsPerDegAZM * prevAbs / effectiveMoved;

            if (!isnan(measuredRatio) && !isinf(measuredRatio) && // Guard 2
                measuredRatio > (STEPS_PER_DEG_AZM * AZM_RATIO_BAND_LOW) &&
                measuredRatio < (STEPS_PER_DEG_AZM * AZM_RATIO_BAND_HIGH)) {

              float oldRatio      = activeStepsPerDegAZM;
              activeStepsPerDegAZM = (activeStepsPerDegAZM * (1.0f - AZM_LEARNING_SMOOTHING))
                                   + (measuredRatio * AZM_LEARNING_SMOOTHING);
              diagPrintf("AZM ML: %.2f→%.2f (prev=%.2f' curr=%.2f' eff=%.2f')\n",
                         oldRatio, activeStepsPerDegAZM,
                         prevAbs * 60.0f, currAbs * 60.0f, effectiveMoved * 60.0f);
              if (fabsf(activeStepsPerDegAZM - oldRatio) > EEPROM_WRITE_THRESHOLD) {
                EEPROM.put(EEPROM_ADDR_AZM_RATIO, activeStepsPerDegAZM);
                EEPROM.commit();
              }
            } else {
              diagPrintf("AZM ML: skipped (ratio=%.2f out-of-band or NaN)\n", measuredRatio);
            }
          } else {
            diagPrintf("AZM ML: skipped (effectiveMoved=%.2f' < threshold)\n",
                       effectiveMoved * 60.0f);
          }
          azmLrnPrevDeltaDeg = azmDeltaDeg;  // Update for next jog
          // azmLrnPrevDir unchanged — still same direction
          // azmLrnValid stays true

        } else if (newAzmDir != azmLrnPrevDir && azmLrnValid) {   // Guard 3
          // Direction reversal: stale data — wipe and start fresh
          diagPrintf("AZM ML: dir reversal → reset\n");
          resetAzmLearning();
          azmLrnPrevDeltaDeg = azmDeltaDeg;
          azmLrnPrevDir      = newAzmDir;
          azmLrnValid        = true;

        } else {
          // First valid jog: record for next time, don't learn yet
          azmLrnPrevDeltaDeg = azmDeltaDeg;
          azmLrnPrevDir      = newAzmDir;
          azmLrnValid        = true;
        }

      } else {
        // Tiny move (< 1'): too small to learn from — mark state as invalid
        diagPrintf("AZM ML: tiny move (%.2f' < 1') → not recorded\n",
                   fabsf(azmDeltaDeg) * 60.0f);
        azmLrnValid = false;
      }

      enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, azmDeltaDeg, activeStepsPerDegAZM, &posDegAZM);
    }

    /* ── ALT jog ── */
    if (yp) {
      float rawVal = atof(yp + 1);
      float val    = rawVal * ARCMIN_TO_DEG;
      float tgt    = rel ? (posDegALT + val) : val;

      diagPrintf("ALT: raw=%.3f' → %.4f° tgt=%.4f°\n", rawVal, val, tgt);

      if (tgt < ALT_LIMIT_NEG) { diagPrintf("!LIMIT ALT: clamped to %.1f\n", ALT_LIMIT_NEG); tgt = ALT_LIMIT_NEG; }
      if (tgt > ALT_LIMIT_POS) { diagPrintf("!LIMIT ALT: clamped to %.1f\n", ALT_LIMIT_POS); tgt = ALT_LIMIT_POS; }

      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    }

    startNextJob();
    return;
  }

  /* ── Direct serial commands (bench testing, degrees) ── */

  // AZM:ZERO — redefine current AZM position as 0.0°.
  // Also resets AZM learning state to prevent a stale azmLrnPrevDeltaDeg from
  // being used after the referential jump (v15.03g fix).
  if (strcmp(line, "AZM:ZERO") == 0) {
    if (isMoving && mot.isAzm) return;  // Refuse during active AZM move
    posDegAZM  = 0.0f;
    lastAzmDir = 0;
    resetAzmLearning();  // v15.03g FIX: prevents stale learning state after referential reset
    Serial.println("MSG: AZM absolute position forcefully reset to 0.0");
    sendStatus(); Serial.println();
    return;
  }

  if (strncmp(line, "AZM:", 4) == 0) {
    float tgt = atof(line + 4);
    if (tgt < AZM_LIMIT_NEG) tgt = AZM_LIMIT_NEG;
    if (tgt > AZM_LIMIT_POS) tgt = AZM_LIMIT_POS;
    enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, tgt - posDegAZM, activeStepsPerDegAZM, &posDegAZM);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println();
    return;
  }

  if (strncmp(line, "ALT:", 4) == 0) {
    float tgt = atof(line + 4);
    if (tgt < ALT_LIMIT_NEG) tgt = ALT_LIMIT_NEG;
    if (tgt > ALT_LIMIT_POS) tgt = ALT_LIMIT_POS;
    enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println();
    return;
  }
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(1000);  // Flush ESP32 ROM boot garbage before any output
  Serial.println("\n=======================================================");
  Serial.println("  BOOT: POLAR ALIGNMENT CONTROLLER V15.03g-p1 (ESP32)");
  Serial.println("  ALT: MPU-6500 observe-only (ML learning, no corrections)");
  Serial.println("  AZM: Residual ratio learning (guarded, no sensor needed)");
  Serial.println("  TPPA jog units: ARCMINUTES (÷60 conversion active)");
  Serial.print  ("  AZM backlash:   "); Serial.print(AZM_BACKLASH_DEG * 60.0f, 1); Serial.println("'");
  Serial.println("  HOME required — persists across DTR reboots");
  Serial.println("=======================================================\n");

  initMPU_Silent();
  diagClear();

  EEPROM.begin(EEPROM_SIZE);

  /* ── Load learned ALT ratio ── */
  float storedRatio = 0.0f;
  EEPROM.get(EEPROM_ADDR_RATIO, storedRatio);
  if (!isnan(storedRatio) &&
      storedRatio > (STEPS_PER_DEG_ALT * RATIO_BAND_LOW) &&
      storedRatio < (STEPS_PER_DEG_ALT * RATIO_BAND_HIGH)) {
    activeStepsPerDegALT = storedRatio;
    Serial.print("MSG: Loaded learned ALT Ratio: ");
  } else {
    activeStepsPerDegALT = STEPS_PER_DEG_ALT;
    Serial.print("MSG: Using theoretical ALT Ratio: ");
  }
  Serial.println(activeStepsPerDegALT);

  /* ── Load learned AZM ratio ── */
  float storedAzmRatio = 0.0f;
  EEPROM.get(EEPROM_ADDR_AZM_RATIO, storedAzmRatio);
  if (!isnan(storedAzmRatio) &&
      storedAzmRatio > (STEPS_PER_DEG_AZM * AZM_RATIO_BAND_LOW) &&
      storedAzmRatio < (STEPS_PER_DEG_AZM * AZM_RATIO_BAND_HIGH)) {
    activeStepsPerDegAZM = storedAzmRatio;
    Serial.print("MSG: Loaded learned AZM Ratio: ");
  } else {
    activeStepsPerDegAZM = STEPS_PER_DEG_AZM;
    Serial.print("MSG: Using theoretical AZM Ratio: ");
  }
  Serial.println(activeStepsPerDegAZM);

  Serial.print("MSG: AZM limits "); Serial.print(AZM_LIMIT_NEG);
  Serial.print("° to "); Serial.print(AZM_LIMIT_POS); Serial.println("°");
  Serial.print("MSG: ALT limits "); Serial.print(ALT_LIMIT_NEG);
  Serial.print("° to "); Serial.print(ALT_LIMIT_POS); Serial.println("°");
  Serial.print("MSG: Global settle "); Serial.print(GLOBAL_SETTLE_MS); Serial.println(" ms");

  /* ── GPIO init ── */
  pinMode(PIN_EN,          OUTPUT);
  pinMode(PIN_DIR_AZM,     OUTPUT); pinMode(PIN_STEP_AZM, OUTPUT);
  pinMode(PIN_DIR_ALT,     OUTPUT); pinMode(PIN_STEP_ALT, OUTPUT);
  pinMode(PIN_HOME_SENSOR, INPUT);  pinMode(PIN_BUTTON_HOME, INPUT);
  digitalWrite(PIN_EN, LOW);   // Active LOW — enables drivers

  /* ── TMC2209 init ── */
  SerialDrivers.begin(115200, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);
  delay(100);

  drvAzm.begin(); drvAzm.pdn_disable(true); drvAzm.mstep_reg_select(true);
  drvAzm.rms_current(RMS_CURRENT_AZM, AZM_HOLD_MULTIPLIER);
  drvAzm.microsteps(MICROSTEPPING_AZM);
  drvAzm.en_spreadCycle(false);  // StealthChop for quiet AZM operation
  drvAzm.toff(4);
  drvAzm.shaft(false);

  drvAlt.begin(); drvAlt.pdn_disable(true); drvAlt.mstep_reg_select(true);
  drvAlt.rms_current(RMS_CURRENT_ALT, 0.1f);
  drvAlt.microsteps(MICROSTEPPING_ALT);
  drvAlt.en_spreadCycle(true);   // SpreadCycle for maximum ALT torque
  drvAlt.toff(4);
  drvAlt.shaft(false);

  Serial.print("MSG: ALT motor current "); Serial.print(RMS_CURRENT_ALT); Serial.println(" mA");

  /* ── Auto-home or restore homing state ── */
  if (digitalRead(PIN_HOME_SENSOR) == LOW) {
    // Limit switch already pressed at boot — auto-recover
    Serial.println("MSG: Sensor triggered at boot — AUTO-HOMING");
    startHoming();
  } else {
    // Try to restore from EEPROM (survives DTR reboot)
    uint32_t savedMagic = 0;
    EEPROM.get(EEPROM_ADDR_MAGIC, savedMagic);

    if (savedMagic == HOMING_MAGIC && mpuAvailable) {
      float savedOffset = 0.0f;
      EEPROM.get(EEPROM_ADDR_MPU_OFF, savedOffset);
      float rawAngle = readMPUAngleY();

      if (rawAngle > -900.0f) {
        float restoredALT = rawAngle - savedOffset;
        if (restoredALT >= (ALT_LIMIT_NEG - 0.5f) &&
            restoredALT <= (ALT_LIMIT_POS + 0.5f)) {
          mpuOffset      = savedOffset;
          posDegALT      = restoredALT;
          targetAltAngle = restoredALT;
          homingDone     = true;
          Serial.println("MSG: *** HOMING STATE RESTORED FROM EEPROM ***");
          Serial.print  ("MSG: mpuOffset="); Serial.print(mpuOffset, 3);
          Serial.print  ("  posDegALT=");   Serial.print(posDegALT, 3); Serial.println("°");
          Serial.println("MSG: TPPA jogs enabled (no re-homing needed).");
        } else {
          Serial.print("MSG: EEPROM ALT out of range ("); Serial.print(restoredALT, 3);
          Serial.println("°) — please HOME.");
        }
      } else {
        Serial.println("MSG: MPU read failed — please HOME.");
      }
    } else {
      Serial.println("MSG: No saved homing state. Send HOME or press the Home button.");
    }
  }
  Serial.println("-------------------------------------------------------\n");
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   MAIN LOOP
   Runs continuously at ~100 kHz. All work is cooperative, non-blocking.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void loop() {
  scanSerialRealtime();  // Service '?' '!' '~' 0x18 immediately (GRBL real-time chars)

  // Handle deferred soft-reset (abortCmd set by 0x18)
  if (abortCmd && !mot.active && !settlingForObserve && !waitingForGlobalSettle) {
    softReset(); return;
  }

  tickMotion();  // Advance the motion state machine by one step

  // Physical home button (debounced)
  if (digitalRead(PIN_BUTTON_HOME) == LOW && !isMoving) {
    delay(50);
    if (digitalRead(PIN_BUTTON_HOME) == LOW) {
      startHoming();
      while (digitalRead(PIN_BUTTON_HOME) == LOW) delay(10);
      return;
    }
  }

  // Line-buffered serial command reader
  // Real-time characters (? ! ~ 0x18) are intercepted above and never reach here.
  static char    lineBuf[64];
  static uint8_t lineIdx = 0;

  while (Serial.available()) {
    char c = Serial.peek();
    if (c == '?' || c == '!' || c == '~' || c == 0x18) break;  // Hand off to realtime
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
