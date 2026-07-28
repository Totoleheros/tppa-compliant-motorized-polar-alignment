/*****************************************************************************************
 * FYSETC-E4 (ESP32 + TMC2209) — POLAR ALIGNMENT CONTROLLER
 * Version : 15.04-p5  (backlash comp + auto-learning for both axes)
 *
 * ══════════════════════════════════════════════════════════════════════
 * PROFILE SELECTION — uncomment ONE profile before compiling
 * ══════════════════════════════════════════════════════════════════════
 * #define PROFILE_PROTO   // Prototype hardware
 * #define PROFILE_V2      // V2_CNC hardware
 * ══════════════════════════════════════════════════════════════════════
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
 * learns the true mechanical gear ratios AND backlash values over successive sessions.
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * HARDWARE
 * ──────────────────────────────────────────────────────────────────────────────────────
 *  MCU       : FYSETC E4 V1.0  (ESP32-WROOM-32 @ 240 MHz)
 *  Drivers   : 2× TMC2209 (UART, addresses 1 & 2)
 *  AZM motor : NEMA 17 → Harmonic Drive 100:1  (SpreadCycle, 16 µstep, 600 mA)
 *  ALT motor : NEMA 17 → UMOT worm 30:1 → T8 lead screw (SpreadCycle, 4 µstep, 300 mA)
 *  Sensor    : MPU-6500 on I2C (SCL=GPIO18, SDA=GPIO19 — repurposed SD card pins)
 *  Limit sw  : Active-LOW, GPIO34 (X-MIN header)
 *  Home btn  : Active-LOW, GPIO35 (Y-MIN header)
 *
 * ──────────────────────────────────────────────────────────────────────────────────────
 * CHANGELOG
 * ──────────────────────────────────────────────────────────────────────────────────────
 * v15.04-p5 vs v15.04-p4 (field testing round 2):
 *   FIX : startNextJob() would clobber active motion when called from a command
 *         handler (ALT:, AZM:, $J=) while the previous move was still running.
 *         Symptom: two consecutive "ALT BACKLASH" log entries with only one
 *         observe cycle — the first move was aborted mid-flight.
 *         Fix: return early if mot.active is already true; the tickMotion
 *         completion path picks up the queued job cleanly.
 *   FIX : EEPROM backlash slot for the un-updated axis was left with garbage
 *         (NaN) after the first single-axis learning write, because only that
 *         axis's slot + the magic word were written. DIAG showed "AZM=nan'".
 *         Fix: new saveBacklashSlots() helper always writes BOTH slots + magic
 *         atomically. All 5 write sites now use the helper.
 *
 * v15.04-p4 vs v15.04-p3 (field testing fixes):
 *   FIX : Any command containing '?' after the first byte was truncated at that
 *         '?' by the realtime GRBL char interceptor (BLC?, BLC:AZM? etc.
 *         silently failed, only sending a status query). The interceptor now
 *         fires only when '?'/'!' / '~' / 0x18 is the FIRST byte of a line.
 *         TPPA's use of these characters is always at position 0, so no impact.
 *   FIX : DIAG header string was hardcoded to old version; now shows v15.04-p4.
 *   FIX : Invalid AZM ratio stored in EEPROM (NaN from a prior corrupted write)
 *         is now overwritten with the theoretical value at boot, so DIAG stops
 *         reporting "EEPROM AZM Ratio : nan".
 *
 * v15.04-p3 vs v15.04-p2 (post-review round 3):
 *   FIX : AZM ping-pong deadlock. In severe over-compensation (C >> B, e.g.
 *         after a forced BLC:AZM:x too high), the mount overshoots on every
 *         reversal, causing TPPA to reverse immediately in the opposite
 *         direction. The steady-state 0.995 leak never fires because the
 *         probe requires a same-direction follow-up — which never arrives.
 *         Fix: detect the ping-pong signature (pending flag already true when
 *         entering Guard 3) and apply a 20% multiplicative penalty per hit.
 *         Convergence from a stuck 30' hardstop → true B in ~13 reversals.
 *
 * v15.04-p2 vs v15.04-p1 (post-review round 2):
 *   FIX : AZM backlash ratchet vulnerability. The signal (|azmDeltaDeg| after
 *         reversal) is positive-only and includes atmospheric noise + residual
 *         TPPA correction work — never fully zero. Pure additive update thus
 *         monotonically grows C over many sessions, eventually hitting the
 *         hardstop and causing wild over-shoot.
 *         Fix: apply 0.5% multiplicative leak per probe, but only in steady
 *         state (samples > BACKLASH_WARMUP_SAMPLES). Warmup phase remains
 *         pure additive so cold-start convergence to true B stays clean.
 *         Steady-state equilibrium: C ≈ 0.91×(B+ε) under-comp, C capped at
 *         ~10' in the noise-only over-comp regime.
 *
 * v15.04-p1 vs v15.04 (post-review fixes):
 *   FIX : AZM backlash EWMA changed from multiplicative blend to additive update.
 *         Old formula C_new = (1-α)C + αS converges to B/2 (only half the true
 *         backlash). Correct formula C_new = C + αS converges to B.
 *   FIX : azmBacklashLearnPending was leaking across:
 *           (1) interrupted jogs — the armed flag was consumed by an unrelated
 *               subsequent jog after TPPA replaced the in-flight command.
 *           (2) tiny (< MIN_AZM_LEARNING_ANGLE) moves — the flag stayed armed
 *               and was consumed with stale sequence data on the next big jog.
 *         Both paths now clear the pending flag explicitly.
 *
 * v15.04 vs v15.03g-p2:
 *   NEW : ALT backlash compensation — dead steps injected on direction reversal
 *         (mirror of the existing AZM mechanism from v15.03g).
 *   NEW : Auto-learning of backlash values on BOTH axes:
 *           ALT — MPU-based, residual = |commanded| − |actualMoved|
 *           AZM — TPPA-residual-based, only after ratio stability reached
 *         Adaptive α (15% for first 10 samples, 5% steady). Persisted to EEPROM.
 *   NEW : EEPROM layout extended 16 → 32 bytes with BACKLASH_MAGIC guard.
 *         Transparent migration from v15.03g (missing magic → profile defaults).
 *   NEW : AZM ratio stability tracking (5 consecutive sub-1.0 steps/deg updates)
 *         gates AZM backlash learning.
 *   NEW : Serial commands BLC?, BLC:AZM?, BLC:ALT?, BLC:AZM:<v>, BLC:ALT:<v>
 *   DEP : TPPA plugin's built-in AZM BacklashCompensation must be disabled
 *         (both axes now handled inside the firmware — see README).
 *
 * v15.03g-p2 vs v15.03g-p1:
 *   CHG : AZM driver switched from StealthChop to SpreadCycle (en_spreadCycle = true).
 *         SpreadCycle provides firmer, more predictable holding behaviour on the harmonic
 *         drive — the motor locks its position more rigidly than StealthChop, which is
 *         preferable for a non-self-locking axis under load.
 *         Note: the perceived AZM drift under field conditions is intrinsic flex-spline
 *         elasticity (soft zone + harder mechanical stop), not a current issue.
 *         SpreadCycle is retained regardless as it improves static holding stiffness.
 *
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
 *   CHG : ALT_MOTOR_GEARBOX 496 → 148.8  (PROTO mechanical ratio)
 *   CHG : ALT_LIMIT_POS 5° → 10°         (wider travel range)
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

// ══════════════════════════════════════════════════════════════════════
// PROFILE SELECTION — uncomment exactly ONE line
// ══════════════════════════════════════════════════════════════════════
#define PROFILE_PROTO   // ← Prototype hardware
//#define PROFILE_V2    // ← V2_CNC hardware

#if !defined(PROFILE_PROTO) && !defined(PROFILE_V2)
  #error "No profile selected — uncomment PROFILE_PROTO or PROFILE_V2 above"
#endif
#if defined(PROFILE_PROTO) && defined(PROFILE_V2)
  #error "Two profiles selected — keep only one"
#endif
// ══════════════════════════════════════════════════════════════════════

#include <TMCStepper.h>
#include <Preferences.h>   // ESP32 NVS — stores hardware profile across reboots/reflashes
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
constexpr uint16_t MICROSTEPPING_AZM  = 16;        // SpreadCycle mode: firm hold on harmonic drive
constexpr uint16_t MICROSTEPPING_ALT  = 4;         // SpreadCycle mode: maximum torque for lift

/* ── Gear ratios ── */
constexpr float    GEAR_RATIO_AZM     = 100.0f;    // Harmonic drive: 100 motor turns = 1 output turn

// ALT gearbox ratio — set at runtime from NVS profile (see loadOrSelectProfile())
// PROTO=148.8   V2_CNC=124.0  (empirically measured ALT mechanical ratios)
// Do NOT declare constexpr here — value comes from cfg_ALT_MOTOR_GEARBOX.

/* ── ALT kinematics ── */
constexpr float    ALT_SCREW_PITCH_MM = 2.0f;      // T8 lead screw: 2 mm linear travel per revolution
constexpr float    ALT_RADIUS_MM      = 60.0f;     // Pivot-to-screw horizontal distance (mm)
                                                    //   Converts linear screw travel → angular tilt

/* ── Axis direction ──
   AZM: same for both profiles.
   ALT: PROTO=true   V2_CNC=false (inverted kinematics)
   cfg_AXIS_REV_ALT is set at runtime from NVS profile. ── */
constexpr bool     AXIS_REV_AZM       = true;
// cfg_AXIS_REV_ALT — see runtime profile variables below

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
// cfg_ALT_LIMIT_NEG: PROTO=0.0°  V2=-2.0° — set at runtime via cfg_ALT_LIMIT_NEG
// ALT_LIMIT_POS: same for both
constexpr float ALT_LIMIT_POS =  10.0f;

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
constexpr uint8_t PIN_HOME_SENSOR = 34;   // X-MIN header, active LOW
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
// cfg_HOME_TRIGGER_ANGLE: PROTO=0.0°  V2=-2.0° — set at runtime via cfg_HOME_TRIGGER_ANGLE
// NOTE: if TPPA stops correcting after first jog on V2, this is the first suspect.
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

// Backlash learning (v15.04) — adaptive α, applies to both AZM and ALT
constexpr float   BACKLASH_LEARNING_RATE_INIT   = 0.15f;  // α for first N samples
constexpr float   BACKLASH_LEARNING_RATE_STEADY = 0.05f;  // α after warmup
constexpr uint8_t BACKLASH_WARMUP_SAMPLES       = 10;     // sample count for α transition
constexpr float   BACKLASH_MAX_SINGLE_UPDATE    = 0.30f;  // residual clamp = 30% of hardstop
constexpr float   BACKLASH_HARDSTOP_AZM_DEG     = 0.50f;  // 30' max
constexpr float   BACKLASH_HARDSTOP_ALT_DEG     = 1.00f;  // 60' max
constexpr float   BACKLASH_EEPROM_THRESHOLD     = 0.005f; // 0.3' → EEPROM write
constexpr float   MIN_BLC_LEARNING_ANGLE        = 0.05f;  // 3' — smaller than MIN_LEARNING_ANGLE
constexpr float   BLC_MIN_ACTUAL                = 0.03f;  // 1.8' — MPU noise floor guard

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 7 — EEPROM LAYOUT (v15.04: extended to 32 bytes)
   Backlash slots added at offsets 16-23, guarded by BACKLASH_MAGIC at 24-27.
   Migration from v15.03g layout (16 bytes) is transparent: if BACKLASH_MAGIC
   is missing/corrupt, the profile default values are kept (see setup()).
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr int      EEPROM_SIZE          = 32;
constexpr int      EEPROM_ADDR_RATIO    = 0;    // float (4): activeStepsPerDegALT
constexpr int      EEPROM_ADDR_MPU_OFF  = 4;    // float (4): mpuOffset (gyro tare)
constexpr int      EEPROM_ADDR_MAGIC    = 8;    // uint32 (4): HOMING_MAGIC
constexpr int      EEPROM_ADDR_AZM_RATIO = 12;  // float (4): activeStepsPerDegAZM
constexpr int      EEPROM_ADDR_AZM_BLC  = 16;   // float (4): activeBacklashDegAZM   (v15.04)
constexpr int      EEPROM_ADDR_ALT_BLC  = 20;   // float (4): activeBacklashDegALT   (v15.04)
constexpr int      EEPROM_ADDR_BLC_MAGIC = 24;  // uint32 (4): BACKLASH_MAGIC        (v15.04)
constexpr uint32_t HOMING_MAGIC         = 0x484F4D45; // "HOME"
constexpr uint32_t BACKLASH_MAGIC       = 0x424C4348; // "BLCH" — backlash slots valid

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

// v15.04 — AZM RATIO STABILITY tracking (gates backlash learning)
// After AZM_STABLE_COUNT consecutive ratio updates each < AZM_STABLE_DELTA_STEPS,
// the ratio is considered "locked" and AZM backlash learning is enabled.
constexpr float   AZM_STABLE_DELTA_STEPS = 1.0f;   // steps/deg change threshold
constexpr uint8_t AZM_STABLE_COUNT       = 5;      // consecutive stable samples

// v15.04 — AZM BACKLASH learning uses the *post-reversal probe* move as signal.
// Guard: reject signals > MAX_AZM_BLC_SIGNAL_DEG (dominated by alignment error, not backlash).
constexpr float   MAX_AZM_BLC_SIGNAL_DEG = 0.083f; // 5' cap

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
// RAMP_CRUISE_ALT_US: PROTO=120µs  V2_CNC=150µs — set at runtime via cfg_RAMP_CRUISE_ALT_US
constexpr long          RAMP_LENGTH        = 500;   // 3000 was too long — caused NINA 7s timeout // Steps to reach cruise speed

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
constexpr unsigned long GLOBAL_SETTLE_MS = 500;    // 1000 was too long — caused NINA 7s timeout

// AZM backlash: moved to runtime state (activeBacklashDegAZM) in v15.04
// so it can be initialized per profile and (later) auto-learned.
// See RUNTIME STATE section below.

/* ═══════════════════════════════════════════════════════════════════════════════════════
   COMPUTED KINEMATICS
   AZM is fully constexpr (all params identical for both profiles).
   ALT depends on cfg_ALT_MOTOR_GEARBOX → computed at runtime in loadOrSelectProfile().
   ═══════════════════════════════════════════════════════════════════════════════════════ */
// AZM: pure constexpr — harmonic drive ratio 100:1 is the same on both platforms
constexpr float STEPS_PER_DEG_AZM =
    (MOTOR_FULL_STEPS * MICROSTEPPING_AZM * GEAR_RATIO_AZM) / 360.0f;

// ALT: runtime — value set by loadOrSelectProfile(), then used to initialize activeStepsPerDegALT
float STEPS_PER_DEG_ALT = 0.0f;   // computed in loadOrSelectProfile()

/* ═══════════════════════════════════════════════════════════════════════════════════════
   RUNTIME HARDWARE PROFILE
   All parameters that differ between PROTO and V2.
   Set once at boot by loadOrSelectProfile(), never changed afterwards.
   Prefixed cfg_ to distinguish from compile-time constants.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
float         cfg_ALT_MOTOR_GEARBOX   = 148.8f;  // PROTO=148.8  V2_CNC=124.0
bool          cfg_AXIS_REV_ALT        = true;     // PROTO=true   V2=false
unsigned long cfg_RAMP_CRUISE_ALT_US  = 120;      // PROTO=120µs  V2_CNC=150µs
float         cfg_ALT_LIMIT_NEG       = 0.0f;     // PROTO=0.0°   V2=-2.0°
float         cfg_HOME_TRIGGER_ANGLE  = 0.0f;     // PROTO=0.0°   V2=-2.0°
const char*   cfg_profile_name        = "PROTO";  // human-readable name for DIAG/boot

/* ═══════════════════════════════════════════════════════════════════════════════════════
   RUNTIME STATE — learned ratios (start at theoretical, updated by ML)
   ═══════════════════════════════════════════════════════════════════════════════════════ */
float mpuOffset           = 0.0f;    // Gyro tare value set at homing
bool  mpuAvailable        = false;   // true if MPU-6500 responded on I2C

// activeStepsPerDegALT is initialized to STEPS_PER_DEG_ALT in loadOrSelectProfile()
// because STEPS_PER_DEG_ALT itself depends on cfg_ALT_MOTOR_GEARBOX (runtime value).
float activeStepsPerDegALT = 0.0f;  // set by loadOrSelectProfile()
float activeStepsPerDegAZM = STEPS_PER_DEG_AZM; // Updated by residual learning

/* ── AZM/ALT BACKLASH COMPENSATION (v15.04 — runtime variables) ──
   Extra steps injected at direction reversal — motor moves but position isn't
   updated (dead steps). TPPA sees only the "real" position change, so the
   response stays linear and the adaptive matrix doesn't get confused.
   AZM value ~ 2' typical for a 100:1 harmonic drive under load.
   Both values initialized per profile in loadOrSelectProfile() and will be
   auto-learned in later steps. ── */
float activeBacklashDegAZM = 0.033f; // seeded, overwritten by profile
float activeBacklashDegALT = 0.033f; // seeded, overwritten by profile

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
bool          learningIsReversal = false; // v15.04: true when current ALT move is a reversal
                                          // → routes MPU observation to backlash learning
                                          // instead of ratio learning
uint8_t       altBacklashSamples = 0;   // v15.04: EWMA sample count for adaptive α

/* ── ALT LEARNING CONVERGENCE (Optimization B) ──
   Once the learned ratio is stable across ALT_CONVERGE_COUNT consecutive
   observations, MPU observation is disabled for the rest of the session.
   This removes the ~750 ms observe overhead from every subsequent ALT jog.
   Reset on reboot (RAM-only) and on homing, so the ratio is re-validated
   over the first few jogs of each session. ── */
constexpr uint8_t ALT_CONVERGE_COUNT = 3;     // stable observations needed to lock
uint8_t altStableCount   = 0;                  // consecutive stable measurements
bool    altRatioConverged = false;             // true = skip observation (fast mode)

bool  homingDone = false;  // TPPA jogs blocked until true

// AZM backlash tracking — separate from AZM learning state (independent concerns)
int8_t lastAzmDir = 0;  // +1 = last move positive, -1 = negative, 0 = unknown
int8_t lastAltDir = 0;  // v15.04: ALT backlash tracking (mirror of lastAzmDir)

/* ── AZM Learning state ──
   resetAzmLearning() is the SINGLE point of truth for clearing this.
   Called from: softReset(), startHoming(), direction reversal, AZM:ZERO. ── */
float  azmLrnPrevDeltaDeg = 0.0f;  // Signed delta from the previous AZM jog (degrees)
int8_t azmLrnPrevDir      = 0;     // Direction of previous AZM jog (+1 / -1)
bool   azmLrnValid        = false; // true only when previous jog data is trustworthy

// v15.04 — AZM ratio stability + backlash learning state
uint8_t azmStableCount           = 0;     // consecutive stable ratio updates
bool    azmRatioStable           = false; // ratio locked → backlash learning enabled
bool    azmBacklashLearnPending  = false; // set on reversal (after stability),
                                          // consumed on next same-direction move
uint8_t azmBacklashSamples       = 0;     // EWMA sample count for adaptive α

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
   HARDWARE PROFILE LOADER
   ─────────────────────────────────────────────────────────────────────────────────────
   Reads the hardware profile from ESP32 NVS (Preferences library).
   NVS survives both power cycles AND firmware reflashes — the profile is written once
   and persists until explicitly reset via the PROFILE:RESET serial command.

   Flow:
     1. Read NVS key "profile" (namespace "polaralign").
     2. If 0 (never set): block on serial, ask user to send '1' or '2', save, reboot.
     3. If 1 (PROTO) or 2 (V2): apply all cfg_ variables and compute STEPS_PER_DEG_ALT.

   Called from setup() before any other hardware init.
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void loadOrSelectProfile() {
  Preferences prefs;
  prefs.begin("polaralign", false);  // namespace, read-write
  int profileId = prefs.getInt("profile", 0);  // 0 = never set

  if (profileId == 0) {
    // ── First boot after flash (or after PROFILE:RESET) ──────────────
    // The Serial port is already open (115200 baud, started in setup before this call).
    Serial.println("\n+---------------------------------------------------+");
    Serial.println("|  HARDWARE PROFILE NOT SET                         |");
    Serial.println("|  Required once after each firmware flash.         |");
    Serial.println("+---------------------------------------------------+");
    Serial.println("|  Send '1'  -> PROTO                               |");
    Serial.println("|  Send '2'  -> V2_CNC                              |");
    Serial.println("+---------------------------------------------------+");

    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '1' || c == '2') {
          profileId = c - '0';
          prefs.putInt("profile", profileId);
          prefs.end();
          Serial.print("Profile ");
          Serial.print(profileId == 1 ? "PROTO" : "V2");
          Serial.println(" saved to NVS. Rebooting in 1 second...");
          delay(1000);
          ESP.restart();   // Clean reboot — firmware re-reads NVS on next boot
        }
      }
      delay(50);
      yield();  // Keep watchdog happy while waiting
    }
    // Never reaches here — ESP.restart() above
  }

  prefs.end();

  // ── Apply profile ─────────────────────────────────────────────────
  if (profileId == 1) {
    cfg_profile_name        = "PROTO";
    cfg_ALT_MOTOR_GEARBOX   = 148.8f;   // PROTO empirical ALT mechanical ratio
    cfg_AXIS_REV_ALT        = true;     // Motor CW = platform tilts up
    cfg_RAMP_CRUISE_ALT_US  = 120;      // 200 was too slow — caused NINA 7s timeout
    cfg_ALT_LIMIT_NEG       = 0.0f;     // Home = bottom of travel
    cfg_HOME_TRIGGER_ANGLE  = 0.0f;     // 0° = homed position
    activeBacklashDegAZM    = 0.033f;   // 2' — harmonic drive elastic zone (default)
    activeBacklashDegALT    = 0.033f;   // 2' — commercial tilt plate typical
  } else {
    cfg_profile_name        = "V2_CNC";
    cfg_ALT_MOTOR_GEARBOX   = 124.0f;   // V2_CNC empirical ALT mechanical ratio
    cfg_AXIS_REV_ALT        = false;    // Kinematics inverted — pivot below T8 axis
    cfg_RAMP_CRUISE_ALT_US  = 150;      // 150µs — reduces UMOT microstep clicking on V2_CNC
    cfg_ALT_LIMIT_NEG       = -2.0f;    // Platform can go 2° below home for corrections
    cfg_HOME_TRIGGER_ANGLE  = -2.0f;    // Physical home = -2° tilt
    // NOTE on cfg_HOME_TRIGGER_ANGLE=-2: if TPPA stops correcting after first jog,
    // suspect this value. Test with 0.0f to verify (TPPA may assume home=0').
    activeBacklashDegAZM    = 0.033f;   // 2' — same harmonic drive as PROTO
    activeBacklashDegALT    = 0.050f;   // 3' — T8 anti-backlash spring removed
  }

  // ── Compute ALT kinematics from profile ───────────────────────────
  // These would have been constexpr in a single-profile build; here they
  // are computed once at boot and treated as read-only constants thereafter.
  float steps_per_mm = (MOTOR_FULL_STEPS * MICROSTEPPING_ALT * cfg_ALT_MOTOR_GEARBOX)
                       / ALT_SCREW_PITCH_MM;
  float mm_per_deg   = (2.0f * (float)M_PI * ALT_RADIUS_MM) / 360.0f;
  STEPS_PER_DEG_ALT  = steps_per_mm * mm_per_deg;

  // Seed the learned ratio with the theoretical value.
  // If a valid EEPROM value exists it will be loaded and override this in setup().
  activeStepsPerDegALT = STEPS_PER_DEG_ALT;
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

/* v15.04-p5 — persist backlash to EEPROM. Always writes BOTH slots so the
   idle slot is never left uninitialized (which used to produce NaN garbage
   in the AZM slot after the first ALT learning write). */
void saveBacklashSlots() {
  EEPROM.put(EEPROM_ADDR_AZM_BLC, activeBacklashDegAZM);
  EEPROM.put(EEPROM_ADDR_ALT_BLC, activeBacklashDegALT);
  EEPROM.put(EEPROM_ADDR_BLC_MAGIC, BACKLASH_MAGIC);
  EEPROM.commit();
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
  // v15.04-p5: refuse to clobber an active motion. When user rapid-clicks
  // GUI buttons, the command handler's startNextJob() call would overwrite
  // `mot` mid-motion, aborting the first move (two BACKLASH logs, one observe).
  // The tickMotion completion path (line ~960) picks up the queued job cleanly.
  if (mot.active) return true;

  MotionJob job = jobQueue[0];
  jobQueue[0] = jobQueue[1];  // Shift queue
  jobCount--;

  long stepsTot = lroundf(job.deltaDeg * job.stepsPerDeg);
  if (!stepsTot) return startNextJob();  // Skip zero-step jobs (clamped travel)

  bool isAzm     = (job.dirPin == PIN_DIR_AZM);
  bool inv       = isAzm ? AXIS_REV_AZM : cfg_AXIS_REV_ALT;
  bool logicalDir = (stepsTot > 0) ? !inv : inv;
  bool isUp      = (job.deltaDeg > 0);

  /* ── AZM/ALT BACKLASH COMPENSATION ──
     On direction reversal, inject extra dead steps to eat through mechanical
     lost motion (AZM: harmonic drive elastic zone; ALT: T8 nut backlash).
     Motor moves, but posDeg is not updated during backlashSteps — TPPA sees a
     clean linear response. ── */
  long backlashExtra = 0;
  if (isAzm && activeBacklashDegAZM > 0.0f) {
    int8_t newDir = (stepsTot > 0) ? 1 : -1;
    if (lastAzmDir != 0 && newDir != lastAzmDir) {
      backlashExtra = lroundf(activeBacklashDegAZM * job.stepsPerDeg);
      diagPrintf("AZM BACKLASH: %ld dead steps (%.1f')\n",
                 backlashExtra, activeBacklashDegAZM * 60.0f);
    }
    lastAzmDir = newDir;
  } else if (!isAzm && activeBacklashDegALT > 0.0f) {
    int8_t newDir = (stepsTot > 0) ? 1 : -1;
    if (lastAltDir != 0 && newDir != lastAltDir) {
      backlashExtra = lroundf(activeBacklashDegALT * job.stepsPerDeg);
      diagPrintf("ALT BACKLASH: %ld dead steps (%.1f')\n",
                 backlashExtra, activeBacklashDegALT * 60.0f);
      learningIsReversal = true;  // v15.04: signal backlash learning path
    }
    lastAltDir = newDir;
  }

  /* ── ALT MPU LEARNING SETUP ──
     Capture the start angle before motion begins.
     v15.04: on direction reversal, we still capture startAngle — but the observed
     movement will feed BACKLASH learning (via learningIsReversal) instead of
     ratio learning. This is the only way to learn the true backlash value. ── */
  if (!isAzm && mpuAvailable) {
    float startRaw = readMPUAngleY();
    if (startRaw > -900.0f) {
      learningStartAngle    = startRaw - mpuOffset;
      learningRequestedDelta = job.deltaDeg;
    } else {
      learningRequestedDelta = 0.0f;  // I2C failure — skip learning
      learningIsReversal     = false;
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
  diagPrintf("SETTLE\n");
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

    /* MACHINE LEARNING (ALT axis) — v15.04 routes reversal moves to backlash
       learning instead of ratio learning (backlash comp is a distinct signal). */
    if (learningIsReversal) {
      /* ── BACKLASH LEARNING (reversal move) ──
         residual = |commanded| - |actualMoved| (positive = under-compensated).
         Adaptive α: fast at startup, conservative after warmup. Guardrails on
         sign agreement, MPU noise floor, and outlier magnitude. ── */
      if (fabsf(learningRequestedDelta) >= MIN_BLC_LEARNING_ANGLE) {
        float actualMoved = actualAngle - learningStartAngle;
        // Sign check: actualMoved must go the same way as commanded
        bool signsAgree = (actualMoved * learningRequestedDelta > 0);
        if (signsAgree && fabsf(actualMoved) > BLC_MIN_ACTUAL) {
          float residual = fabsf(learningRequestedDelta) - fabsf(actualMoved);
          float maxSwing = BACKLASH_MAX_SINGLE_UPDATE * BACKLASH_HARDSTOP_ALT_DEG;

          if (fabsf(residual) <= maxSwing) {
            float alpha = (altBacklashSamples < BACKLASH_WARMUP_SAMPLES)
                          ? BACKLASH_LEARNING_RATE_INIT
                          : BACKLASH_LEARNING_RATE_STEADY;
            float oldBlc = activeBacklashDegALT;
            activeBacklashDegALT += alpha * residual;
            // Clamp to [0, HARDSTOP]
            if (activeBacklashDegALT < 0.0f) activeBacklashDegALT = 0.0f;
            if (activeBacklashDegALT > BACKLASH_HARDSTOP_ALT_DEG)
              activeBacklashDegALT = BACKLASH_HARDSTOP_ALT_DEG;
            altBacklashSamples++;

            diagPrintf("ALT BLC ML: residual=%.2f' α=%.2f  %.4f→%.4f (n=%u)\n",
                       residual * 60.0f, alpha, oldBlc, activeBacklashDegALT,
                       altBacklashSamples);

            if (fabsf(activeBacklashDegALT - oldBlc) > BACKLASH_EEPROM_THRESHOLD) {
              saveBacklashSlots();  // v15.04-p5: writes both slots + magic
            }
          } else {
            diagPrintf("ALT BLC ML: outlier residual %.2f' > max %.2f' — skip\n",
                       residual * 60.0f, maxSwing * 60.0f);
          }
        } else {
          diagPrintf("ALT BLC ML: sign disagreement or tiny move — skip\n");
        }
      }
      learningIsReversal = false;  // consume the flag

    } else if (fabsf(learningRequestedDelta) >= MIN_LEARNING_ANGLE) {
      /* ── RATIO LEARNING (same-direction move) — unchanged from v15.03g ── */
      float actualMoved = actualAngle - learningStartAngle;
      if (fabsf(actualMoved) > LEARNING_MIN_ACTUAL) {
        float stepsSent     = learningRequestedDelta * activeStepsPerDegALT;
        float measuredRatio = stepsSent / actualMoved;

        if (measuredRatio > (STEPS_PER_DEG_ALT * RATIO_BAND_LOW) &&
            measuredRatio < (STEPS_PER_DEG_ALT * RATIO_BAND_HIGH)) {
          float oldRatio      = activeStepsPerDegALT;
          activeStepsPerDegALT = (activeStepsPerDegALT * (1.0f - LEARNING_SMOOTHING))
                               + (measuredRatio * LEARNING_SMOOTHING);
          float change = fabsf(activeStepsPerDegALT - oldRatio);
          if (change > EEPROM_WRITE_THRESHOLD) {
            EEPROM.put(EEPROM_ADDR_RATIO, activeStepsPerDegALT);
            EEPROM.commit();
            altStableCount = 0;                 // significant change — not converged yet
          } else {
            // Measurement barely moved the ratio → count as stable
            if (++altStableCount >= ALT_CONVERGE_COUNT) {
              altRatioConverged = true;
              diagPrintf("ML: ratio converged (%.2f) — observation disabled\n",
                         activeStepsPerDegALT);
            }
          }
          diagPrintf("ML Ratio: %.2f (was %.2f) stable=%u\n",
                     activeStepsPerDegALT, oldRatio, altStableCount);
        }
      }
    }

    /* No motor correction — TPPA's plate-solve loop handles convergence.
       Snap position to exact target and report done. */
    inFeedbackCycle = false;
    posDegALT       = targetAltAngle;
    diagPrintf("DONE: reported=%.3f mpuActual=%.3f\n", posDegALT, actualAngle);

    /* Optimization A: the mount already sat still for SETTLE_DELAY_MS + sampling
       (~750 ms) during observation — vibrations are long gone. Skip the
       redundant global settle and report Idle immediately. */
    if (!startNextJob()) {
      isMoving = false;
      diagPrintf("IDLE (post-observe, global settle skipped)\n");
    }
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

  unsigned long cruiseUs = mot.isAzm ? RAMP_CRUISE_AZM_US : cfg_RAMP_CRUISE_ALT_US;
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
    *mot.globalPos = cfg_HOME_TRIGGER_ANGLE;
    posDegALT      = cfg_HOME_TRIGGER_ANGLE;
    sendStatus(); Serial.println();
    return;
  }

  /* ── JOB COMPLETE ── */
  if (mot.stepsRemaining <= 0) {
    mot.active = false;

    // For ALT moves above tolerance: enter MPU observation instead of settling directly.
    // This is where the machine learning measurement happens.
    // v15.04: also observe on reversal even if ratio converged (backlash learning path)
    if (!mot.isAzm && mpuAvailable && !feedHold &&
        (!altRatioConverged || learningIsReversal) &&
        fabsf(mot.deltaDeg) >= ALT_TOLERANCE_DEG) {
      diagPrintf("Observe: tgt=%.3f start=%.3f\n", targetAltAngle, feedbackStartPos);
      settlingForObserve = true;
      settleStartMs      = millis();
      mpuSampleCount     = 0;
      mpuSumAngles       = 0.0f;
      lastMpuSampleMs    = 0;
      return;
    }

    // No observation needed (AZM, sub-tolerance ALT, or converged ALT):
    // clear feedback scaling, snap to exact target, then next job or settle.
    inFeedbackCycle = false;   // required: converged ALT moves set this true in startNextJob
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
  altStableCount         = 0;       // re-validate ALT ratio after reset
  altRatioConverged      = false;
  learningIsReversal     = false;   // v15.04
  altBacklashSamples     = 0;       // v15.04: restart warmup α on next reversal
  azmStableCount         = 0;       // v15.04
  azmRatioStable         = false;   // v15.04
  azmBacklashLearnPending = false;  // v15.04
  azmBacklashSamples     = 0;       // v15.04
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
  bool dirUp = !cfg_AXIS_REV_ALT ? HIGH : LOW;
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
  bool dirDown = cfg_AXIS_REV_ALT ? HIGH : LOW;
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
  posDegALT      = cfg_HOME_TRIGGER_ANGLE;
  targetAltAngle = cfg_HOME_TRIGGER_ANGLE;   // FIX: prevent stale pre-homing value
  posDegAZM = 0.0f;    // AZM has no absolute sensor — always resets to 0 at homing
  lastAzmDir = 0;
  lastAltDir = 0;

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
      mpuOffset = (sumAngles / (float)validSamples) - cfg_HOME_TRIGGER_ANGLE;
      Serial.print("MSG: Gyroscope tared. Offset = "); Serial.println(mpuOffset, 3);
    } else {
      Serial.println("ALARM: Gyroscope Tare Failed (I2C Bus unresponsive).");
    }
  }

  isMoving               = false;
  homingDone             = true;
  learningRequestedDelta = 0;
  altStableCount         = 0;       // re-validate ALT ratio over first few jogs of session
  altRatioConverged      = false;
  learningIsReversal     = false;   // v15.04
  altBacklashSamples     = 0;       // v15.04: restart warmup α (mechanics may have shifted)
  azmStableCount         = 0;       // v15.04
  azmRatioStable         = false;   // v15.04
  azmBacklashLearnPending = false;  // v15.04
  azmBacklashSamples     = 0;       // v15.04
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
  Serial.print("\n--- SYSTEM DIAGNOSTIC (v15.04-p5) [");
  Serial.print(cfg_profile_name);
  Serial.println("] ---");

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
  Serial.print("AZM backlash comp     : "); Serial.print(activeBacklashDegAZM * 60.0f, 1);
  Serial.print("' ("); Serial.print(activeBacklashDegAZM, 4); Serial.println(" deg)");
  Serial.print("lastAzmDir            : ");
  Serial.println(lastAzmDir ==  1 ? "+1 (positive)" :
                 lastAzmDir == -1 ? "-1 (negative)" : "0 (unknown)");
  Serial.print("AZM Lrn valid         : "); Serial.println(azmLrnValid ? "YES" : "NO");
  if (azmLrnValid) {
    Serial.print("AZM Lrn prevDelta     : ");
    Serial.print(azmLrnPrevDeltaDeg * 60.0f, 2);
    Serial.print("'  dir="); Serial.println(azmLrnPrevDir == 1 ? "+1" : "-1");
  }
  // AZM ratio stability + backlash learning (v15.04)
  Serial.print("AZM ratio stable      : ");
  Serial.print(azmRatioStable ? "YES" : "NO");
  Serial.print("  (stableCount="); Serial.print(azmStableCount);
  Serial.print("/"); Serial.print(AZM_STABLE_COUNT); Serial.println(")");
  Serial.print("AZM backlash samples  : "); Serial.print(azmBacklashSamples);
  Serial.print(azmBacklashSamples < BACKLASH_WARMUP_SAMPLES ? " (fast α)" : " (steady α)");
  if (azmBacklashLearnPending) Serial.print("  [probe armed]");
  Serial.println("");

  // ALT details (v15.04)
  Serial.print("ALT backlash comp     : "); Serial.print(activeBacklashDegALT * 60.0f, 1);
  Serial.print("' ("); Serial.print(activeBacklashDegALT, 4); Serial.println(" deg)");
  Serial.print("ALT backlash samples  : "); Serial.print(altBacklashSamples);
  Serial.print(altBacklashSamples < BACKLASH_WARMUP_SAMPLES ? " (fast α)" : " (steady α)");
  Serial.println("");
  Serial.print("lastAltDir            : ");
  Serial.println(lastAltDir ==  1 ? "+1 (up)" :
                 lastAltDir == -1 ? "-1 (down)" : "0 (unknown)");

  Serial.println("");

  // Travel & config
  Serial.print("Travel limits AZM     : "); Serial.print(AZM_LIMIT_NEG);
  Serial.print(" to "); Serial.print(AZM_LIMIT_POS); Serial.println(" deg");
  Serial.print("Travel limits ALT     : "); Serial.print(cfg_ALT_LIMIT_NEG);
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
  Serial.print("ALT Cruise            : "); Serial.print(cfg_RAMP_CRUISE_ALT_US); Serial.println(" µs");
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
  uint32_t storedBlcMagic2 = 0; EEPROM.get(EEPROM_ADDR_BLC_MAGIC, storedBlcMagic2);
  Serial.print("EEPROM Backlash slots : ");
  if (storedBlcMagic2 == BACKLASH_MAGIC) {
    float bA = 0.0f, bL = 0.0f;
    EEPROM.get(EEPROM_ADDR_AZM_BLC, bA);
    EEPROM.get(EEPROM_ADDR_ALT_BLC, bL);
    Serial.print("SAVED  AZM="); Serial.print(bA * 60.0f, 2);
    Serial.print("'  ALT="); Serial.print(bL * 60.0f, 2); Serial.println("'");
  } else {
    Serial.println("NOT SAVED (using profile defaults)");
  }
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

  /* ── BLC: backlash compensation query & set (v15.04) ── */
  if (strcmp(line, "BLC?") == 0) {
    Serial.print("BLC:AZM="); Serial.print(activeBacklashDegAZM, 4);
    Serial.print(" ALT="); Serial.print(activeBacklashDegALT, 4);
    Serial.print(" ("); Serial.print(activeBacklashDegAZM * 60.0f, 2); Serial.print("'/");
    Serial.print(activeBacklashDegALT * 60.0f, 2); Serial.print("')  AZM stable=");
    Serial.print(azmRatioStable ? "YES" : "NO");
    Serial.print(" samples AZM/ALT="); Serial.print(azmBacklashSamples);
    Serial.print("/"); Serial.println(altBacklashSamples);
    return;
  }
  if (strcmp(line, "BLC:AZM?") == 0) {
    Serial.print("BLC:AZM="); Serial.print(activeBacklashDegAZM, 4);
    Serial.print(" ("); Serial.print(activeBacklashDegAZM * 60.0f, 2); Serial.println("')");
    return;
  }
  if (strcmp(line, "BLC:ALT?") == 0) {
    Serial.print("BLC:ALT="); Serial.print(activeBacklashDegALT, 4);
    Serial.print(" ("); Serial.print(activeBacklashDegALT * 60.0f, 2); Serial.println("')");
    return;
  }
  if (strncmp(line, "BLC:AZM:", 8) == 0) {
    float v = atof(line + 8);
    if (v < 0.0f || v > BACKLASH_HARDSTOP_AZM_DEG || isnan(v)) {
      Serial.print("!BLC:AZM out of range [0, ");
      Serial.print(BACKLASH_HARDSTOP_AZM_DEG, 3); Serial.println("]");
      return;
    }
    activeBacklashDegAZM = v;
    saveBacklashSlots();  // v15.04-p5
    Serial.print("BLC:AZM set to "); Serial.print(v, 4);
    Serial.print(" ("); Serial.print(v * 60.0f, 2); Serial.println("') [saved]");
    return;
  }
  if (strncmp(line, "BLC:ALT:", 8) == 0) {
    float v = atof(line + 8);
    if (v < 0.0f || v > BACKLASH_HARDSTOP_ALT_DEG || isnan(v)) {
      Serial.print("!BLC:ALT out of range [0, ");
      Serial.print(BACKLASH_HARDSTOP_ALT_DEG, 3); Serial.println("]");
      return;
    }
    activeBacklashDegALT = v;
    saveBacklashSlots();  // v15.04-p5
    Serial.print("BLC:ALT set to "); Serial.print(v, 4);
    Serial.print(" ("); Serial.print(v * 60.0f, 2); Serial.println("') [saved]");
    return;
  }

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
      learningIsReversal     = false; // v15.04
      azmBacklashLearnPending = false; // v15.04-p1: interrupted reversal must not leak
      lastAzmDir             = 0;   // FIX: prevent spurious backlash on interrupted jog
      lastAltDir             = 0;   // v15.04: same fix for ALT axis
    }

    const char* rest = strchr(line, '=');
    if (!rest) return;
    rest++;

    // G53 = absolute coordinate system, G91/G21 = relative
    bool rel = (strstr(rest, "G53") == nullptr);

    Serial.println("ok");
    diagPrintf("CMD: %s %s pos=%.1f',%.1f'\n", line, rel?"REL":"ABS", posDegAZM*60.0f, posDegALT*60.0f);

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
      long   azmStepsTot = lroundf(fabsf(azmDeltaDeg)*activeStepsPerDegAZM);
      diagPrintf("AZM delta=%.4f deg steps=%ld\n", azmDeltaDeg, azmStepsTot);
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

          /* v15.04 — BACKLASH LEARNING PROBE (fires on same-direction move
             immediately following a reversal, only if ratio has stabilized).
             The current jog magnitude is treated as a signal proportional to
             the residual backlash. Small biases are averaged out by EWMA. ── */
          if (azmBacklashLearnPending && azmRatioStable) {
            float signal = fabsf(azmDeltaDeg);
            if (signal <= MAX_AZM_BLC_SIGNAL_DEG) {
              float alpha = (azmBacklashSamples < BACKLASH_WARMUP_SAMPLES)
                            ? BACKLASH_LEARNING_RATE_INIT
                            : BACKLASH_LEARNING_RATE_STEADY;
              float oldBlc = activeBacklashDegAZM;
              activeBacklashDegAZM += alpha * signal;
              // v15.04-p2: leaky bucket, steady-state only.
              // Without leak, positive-only signal ratchets C monotonically upward
              // (noise + residual TPPA correction work never fully vanish),
              // slowly walking C to the hardstop over dozens of sessions.
              // Leak applied only after warmup so first 10 samples converge cleanly
              // to true B, then a gentle 0.5% decay per probe caps the equilibrium.
              if (azmBacklashSamples >= BACKLASH_WARMUP_SAMPLES) {
                activeBacklashDegAZM *= 0.995f;
              }
              // Clamp
              if (activeBacklashDegAZM < 0.0f) activeBacklashDegAZM = 0.0f;
              if (activeBacklashDegAZM > BACKLASH_HARDSTOP_AZM_DEG)
                activeBacklashDegAZM = BACKLASH_HARDSTOP_AZM_DEG;
              azmBacklashSamples++;

              diagPrintf("AZM BLC ML: signal=%.2f' α=%.2f  %.4f→%.4f (n=%u)\n",
                         signal * 60.0f, alpha, oldBlc, activeBacklashDegAZM,
                         azmBacklashSamples);

              if (fabsf(activeBacklashDegAZM - oldBlc) > BACKLASH_EEPROM_THRESHOLD) {
                saveBacklashSlots();  // v15.04-p5
              }
            } else {
              diagPrintf("AZM BLC ML: signal too large (%.2f' > %.2f') — skip\n",
                         signal * 60.0f, MAX_AZM_BLC_SIGNAL_DEG * 60.0f);
            }
            azmBacklashLearnPending = false;   // consume the flag regardless
          }

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

              /* v15.04 — STABILITY TRACKING for backlash-learning gate */
              if (fabsf(activeStepsPerDegAZM - oldRatio) < AZM_STABLE_DELTA_STEPS) {
                if (!azmRatioStable && ++azmStableCount >= AZM_STABLE_COUNT) {
                  azmRatioStable = true;
                  diagPrintf("AZM ratio STABLE (%.2f) — backlash learning enabled\n",
                             activeStepsPerDegAZM);
                }
              } else {
                azmStableCount = 0;
                // Big ratio change → un-stabilize (rare, but be safe)
                if (azmRatioStable) {
                  azmRatioStable = false;
                  diagPrintf("AZM ratio DESTABILIZED — backlash learning paused\n");
                }
              }

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
          // Direction reversal: stale ratio data — wipe and start fresh
          // v15.04: if ratio is stable, arm backlash learning for the NEXT jog
          if (azmRatioStable) {
            /* v15.04-p3 — PING-PONG DETECTION (over-compensation penalty)
               If pending is already true when we enter Guard 3, it means the
               previous reversal overshot so badly that TPPA had to immediately
               reverse again. The steady-state 0.995 leak can never fire (probe
               requires same-direction follow-up, never arrives during ping-pong),
               so C would stay stuck. Apply an aggressive 20% penalty per
               ping-pong to knock C back down toward true B. ── */
            if (azmBacklashLearnPending) {
              float oldBlc = activeBacklashDegAZM;
              activeBacklashDegAZM *= 0.80f;
              if (fabsf(activeBacklashDegAZM - oldBlc) > BACKLASH_EEPROM_THRESHOLD) {
                saveBacklashSlots();  // v15.04-p5
              }
              diagPrintf("AZM BLC ML: ping-pong detected → penalty  %.4f→%.4f\n",
                         oldBlc, activeBacklashDegAZM);
            }
            azmBacklashLearnPending = true;
            diagPrintf("AZM ML: dir reversal → backlash probe armed\n");
          } else {
            diagPrintf("AZM ML: dir reversal → reset\n");
          }
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
        azmBacklashLearnPending = false; // v15.04-p1: stale pending flag would
                                          // be consumed with unrelated sequence data
      }

      enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, azmDeltaDeg, activeStepsPerDegAZM, &posDegAZM);
    }

    /* ── ALT jog ── */
    if (yp) {
      float rawVal = atof(yp + 1);
      float val    = rawVal * ARCMIN_TO_DEG;
      float tgt    = rel ? (posDegALT + val) : val;

      diagPrintf("ALT: raw=%.3f' → %.4f° tgt=%.4f°\n", rawVal, val, tgt);

      if (tgt < cfg_ALT_LIMIT_NEG) { diagPrintf("!LIMIT ALT: clamped to %.1f\n", cfg_ALT_LIMIT_NEG); tgt = cfg_ALT_LIMIT_NEG; }
      if (tgt > ALT_LIMIT_POS) { diagPrintf("!LIMIT ALT: clamped to %.1f\n", ALT_LIMIT_POS); tgt = ALT_LIMIT_POS; }

      float altDelta = tgt - posDegALT;
      long  altSteps  = lroundf(fabsf(altDelta)*activeStepsPerDegALT);
      diagPrintf("ALT delta=%.4f deg steps=%ld\n", altDelta, altSteps);
      enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, altDelta, activeStepsPerDegALT, &posDegALT);
    }

    if (!startNextJob()) diagPrintf("WARN: startNextJob=false (zero delta?)\n");
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
    if (tgt < cfg_ALT_LIMIT_NEG) tgt = cfg_ALT_LIMIT_NEG;
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
  delay(300);  // short — ESP32 must answer within TPPA's connection timeout (~1-1.5s)

  // MUST be first: loads profile from NVS, sets cfg_ vars, computes STEPS_PER_DEG_ALT
  // On first boot after flash: blocks until user sends 1 or 2 on serial.
  loadOrSelectProfile();

  Serial.println("\n=======================================================");
  Serial.print("  BOOT: V15.04-p5 ["); Serial.print(cfg_profile_name); Serial.println("] (ESP32)");
  Serial.print("  Profile: "); Serial.print(cfg_profile_name);
  Serial.print("  ALT_GEARBOX="); Serial.print(cfg_ALT_MOTOR_GEARBOX,1);
  Serial.print("  AXIS_REV_ALT="); Serial.println(cfg_AXIS_REV_ALT ? "true":"false");
  Serial.println("  AZM: Residual learning  TPPA: ARCMINUTES  HOME required");
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
    activeStepsPerDegALT = STEPS_PER_DEG_ALT;  // explicit guarantee
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
    // v15.04-p4: overwrite invalid stored value so DIAG stops showing "nan"
    EEPROM.put(EEPROM_ADDR_AZM_RATIO, activeStepsPerDegAZM);
    EEPROM.commit();
  }
  Serial.println(activeStepsPerDegAZM);

  /* ── Load backlash values (v15.04) — migration-safe ──
     If BACKLASH_MAGIC is invalid (fresh flash, upgrade from v15.03g), keep the
     profile-default values that loadOrSelectProfile() already installed. ── */
  uint32_t storedBlcMagic = 0;
  EEPROM.get(EEPROM_ADDR_BLC_MAGIC, storedBlcMagic);
  if (storedBlcMagic == BACKLASH_MAGIC) {
    float bAZM = 0.0f, bALT = 0.0f;
    EEPROM.get(EEPROM_ADDR_AZM_BLC, bAZM);
    EEPROM.get(EEPROM_ADDR_ALT_BLC, bALT);
    // Guardrails: reject NaN/Inf/negative/out-of-band
    if (!isnan(bAZM) && !isinf(bAZM) && bAZM >= 0.0f && bAZM <= 0.50f) {
      activeBacklashDegAZM = bAZM;
    }
    if (!isnan(bALT) && !isinf(bALT) && bALT >= 0.0f && bALT <= 1.00f) {
      activeBacklashDegALT = bALT;
    }
    Serial.print("MSG: Loaded backlash comp — AZM=");
    Serial.print(activeBacklashDegAZM * 60.0f, 2);
    Serial.print("' ALT="); Serial.print(activeBacklashDegALT * 60.0f, 2);
    Serial.println("'");
  } else {
    Serial.print("MSG: Backlash defaults from profile — AZM=");
    Serial.print(activeBacklashDegAZM * 60.0f, 2);
    Serial.print("' ALT="); Serial.print(activeBacklashDegALT * 60.0f, 2);
    Serial.println("'");
  }

  Serial.print("MSG: AZM limits "); Serial.print(AZM_LIMIT_NEG);
  Serial.print("° to "); Serial.print(AZM_LIMIT_POS); Serial.println("°");
  Serial.print("MSG: ALT limits "); Serial.print(cfg_ALT_LIMIT_NEG);
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
  drvAzm.en_spreadCycle(true);   // SpreadCycle: firmer hold on non-self-locking harmonic drive (p2)
  drvAzm.toff(4);
  drvAzm.shaft(false);

  drvAlt.begin(); drvAlt.pdn_disable(true); drvAlt.mstep_reg_select(true);
  drvAlt.rms_current(RMS_CURRENT_ALT, 0.1f);
  drvAlt.microsteps(MICROSTEPPING_ALT);
  drvAlt.en_spreadCycle(true);   // SpreadCycle for maximum ALT torque
  drvAlt.toff(4);
  drvAlt.shaft(false);

  // FIX: UART health check — 0x21 = TMC2209 OK
  { uint8_t vAzm = drvAzm.version(); uint8_t vAlt = drvAlt.version();
    Serial.print("MSG: TMC2209 AZM UART: 0x"); Serial.print(vAzm, HEX);
    Serial.println(vAzm == 0x21 ? "  OK" : "  *** FAIL — check UART address/wiring ***");
    Serial.print("MSG: TMC2209 ALT UART: 0x"); Serial.print(vAlt, HEX);
    Serial.println(vAlt == 0x21 ? "  OK" : "  *** FAIL — check UART address/wiring ***"); }

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

      Serial.print("MSG: EEPROM restore: magic=VALID, offset=");
      Serial.print(savedOffset, 3);
      Serial.print(", rawMPU="); Serial.print(rawAngle, 3);

      if (isnan(savedOffset)) {
        Serial.println(" -> FAIL: offset is NaN. HOME required.");
      } else if (rawAngle <= -900.0f) {
        Serial.println(" -> FAIL: MPU read error. HOME required.");
      } else {
        float restoredALT = rawAngle - savedOffset;
        Serial.print(", restoredALT="); Serial.print(restoredALT, 3);
        Serial.print(" (limits: "); Serial.print(cfg_ALT_LIMIT_NEG - 0.5f, 1);
        Serial.print(" to "); Serial.print(ALT_LIMIT_POS + 0.5f, 1); Serial.println(")");

        if (restoredALT >= (cfg_ALT_LIMIT_NEG - 0.5f) &&
            restoredALT <= (ALT_LIMIT_POS + 0.5f)) {
          mpuOffset      = savedOffset;
          posDegALT      = restoredALT;
          targetAltAngle = restoredALT;
          homingDone     = true;
          Serial.println("MSG: *** HOMING STATE RESTORED *** — TPPA jogs enabled.");
        } else {
          Serial.println("MSG: *** EEPROM ALT OUT OF RANGE *** — HOME required.");
        }
      }
    } else {
      Serial.print("MSG: EEPROM restore: magic=");
      Serial.print(savedMagic == HOMING_MAGIC ? "VALID" : "INVALID");
      Serial.print(", MPU="); Serial.println(mpuAvailable ? "OK" : "MISSING");
      Serial.println("MSG: *** HOMING REQUIRED *** — send HOME or press button.");
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

  // Physical home button (debounced) → start homing
  if (digitalRead(PIN_BUTTON_HOME) == LOW) {
    delay(50);
    if (digitalRead(PIN_BUTTON_HOME) == LOW) {
      while (digitalRead(PIN_BUTTON_HOME) == LOW) delay(10);
      if (!isMoving) startHoming();
      return;
    }
  }

  // Line-buffered serial command reader
  // Real-time characters (? ! ~ 0x18) are intercepted above and never reach here.
  static char    lineBuf[64];
  static uint8_t lineIdx = 0;

  while (Serial.available()) {
    char c = Serial.peek();
    // v15.04-p4: only intercept realtime chars when they are the FIRST byte
    // of a new command. Otherwise 'BLC?' etc. would be truncated to 'BLC'
    // and the '?' would spuriously trigger a GRBL status query.
    if (lineIdx == 0 && (c == '?' || c == '!' || c == '~' || c == 0x18)) break;
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
