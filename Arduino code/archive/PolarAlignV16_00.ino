/*****************************************************************************************
 * FYSETC-E4 (ESP32 + TMC2209) — POLAR ALIGNMENT CONTROLLER
 * Version : 16.00  (post-audit overhaul — AZM learning frozen, robustness fixes)
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
 * v16.00 vs v15.04-p5 (full audit — 4 independent review passes, scope validated):
 *   REMOVED : AZM backlash auto-learning (probe / 0.995 leak / ping-pong penalty /
 *         stability gate). Root cause: leak equilibrium C* ≈ 10×signal (signal
 *         floor 1' → C* ≥ 10'; typical 2-3' residuals → 20-30'), and the
 *         ping-pong penalty fires on the normal sign alternation of TPPA
 *         corrections near convergence. The learner tracked noise statistics,
 *         not backlash. Compensation itself is KEPT — value set via BLC:AZM:
 *         (persisted to EEPROM). Calibrate once, manually.
 *   REMOVED : AZM ratio learning. The estimator was one-sided: every accepted
 *         sample had measuredRatio > activeRatio (overshoot evidence lands in
 *         the reversal branch which learns nothing) → monotonic drift to the
 *         +10% band edge. Harmonic drive 100:1 is machined and stable: ratio
 *         frozen at the theoretical 888.9 steps/deg. EEPROM slot 12 retired.
 *   KEPT  : ALT backlash learning (real MPU signal), now gated on ALT ratio
 *         convergence (no cross-contamination while the ratio is still moving)
 *         and decoupled from injection (learning no longer dies if comp
 *         reaches 0 — the >0 gate used to control learningIsReversal too).
 *   FIX : HOME/$H during motion now purges the active job + queue. Previously
 *         the stale job resumed after homing and snapped to a pre-homing
 *         target in the NEW coordinate frame.
 *   FIX : realtime-char handling unified. '?' intercepted only at line start
 *         (lineIdx==0 shared with the reader — the p4 fix was bypassable when
 *         the '?' of BLC? arrived in a separate UART chunk); '!' '~' 0x18
 *         intercepted anywhere (they never occur inside commands). No more
 *         "ok" reply to '!' / '~' (GRBL realtime chars are silent).
 *   FIX : ALT ratio stability/EEPROM threshold now RELATIVE (0.3% of
 *         theoretical) instead of 0.5 steps/deg absolute (8 ppm of 62k —
 *         unreachable → altRatioConverged never latched, Optimization B was
 *         dead code, every ALT jog paid ~750 ms observe, EEPROM churned).
 *   FIX : MPU observe phase has a timeout (3 s without a valid sample →
 *         clean abort; 2 consecutive timeouts → MPU disabled for the session).
 *         Previously an I2C failure mid-observe left <Run forever.
 *   FIX : failed gyro tare now FAILS the homing (no EEPROM magic written,
 *         stale magic invalidated). Boot-time restore averages ~10 MPU reads
 *         instead of trusting a single one.
 *   FIX : first-boot profile menu is line-based ("1"+Enter) — it used to
 *         latch on the first '1'/'2' byte of ANY traffic ($J=G91G21X…).
 *         PROFILE:RESET implemented (was documented but missing).
 *   NEW : trapezoidal ramp restarts after any pause > 20 ms (feedHold resume,
 *         serial stall) instead of resuming at full cruise speed.
 *   NEW : diagLog is a 4 KB ring buffer — DIAG now shows the END of a long
 *         session (it used to fill after ~25 jogs and silently stop).
 *   NEW : unknown serial lines get "error:20" (GRBL-ish strictness).
 *   CHG : BACKLASH_HARDSTOP_ALT 1.0° → 0.3° (1° of dead steps took 7.5 s with
 *         a frozen reported position — guaranteed NINA 7 s timeout).
 *   CHG : status report closed properly: <Idle|MPos:a,b,0|>
 *   CHG : drivers held disabled (EN high) until TMC2209 config is applied.
 *
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
 * EEPROM LAYOUT (32 bytes total)
 * ──────────────────────────────────────────────────────────────────────────────────────
 *  Offset  0 : float  activeStepsPerDegALT   (learned ALT ratio)
 *  Offset  4 : float  mpuOffset              (gyroscope tare value, saved at homing)
 *  Offset  8 : uint32 HOMING_MAGIC           (0x484F4D45 = "HOME" — homing validity)
 *  Offset 12 : (retired in v16 — was learned AZM ratio; AZM ratio is now fixed)
 *  Offset 16 : float  activeBacklashDegAZM   (manual/persisted AZM backlash comp)
 *  Offset 20 : float  activeBacklashDegALT   (learned ALT backlash comp)
 *  Offset 24 : uint32 BACKLASH_MAGIC         (0x424C4348 = "BLCH")
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

// v16: stability / EEPROM-write threshold is RELATIVE to the theoretical ratio.
// The old absolute 0.5 steps/deg was 8 ppm of ALT's ~62k steps/deg — unreachable,
// so altRatioConverged never latched and every observation committed EEPROM.
// 0.3% ≈ 190 steps/deg on ALT: reachable after the EWMA settles, still meaningful.
constexpr float RATIO_STABLE_FRACT    = 0.003f;

// Backlash learning (v15.04) — adaptive α, applies to both AZM and ALT
constexpr float   BACKLASH_LEARNING_RATE_INIT   = 0.15f;  // α for first N samples
constexpr float   BACKLASH_LEARNING_RATE_STEADY = 0.05f;  // α after warmup
constexpr uint8_t BACKLASH_WARMUP_SAMPLES       = 10;     // sample count for α transition
constexpr float   BACKLASH_MAX_SINGLE_UPDATE    = 0.30f;  // residual clamp = 30% of hardstop
constexpr float   BACKLASH_HARDSTOP_AZM_DEG     = 0.50f;  // 30' max
// v16: 1.0° of ALT dead steps = ~62k steps = 7.5 s with a frozen reported
// position — guaranteed NINA 7 s timeout. 0.3° (18') is still 6-9× any
// plausible T8 backlash and keeps worst-case dead time under ~2.3 s.
constexpr float   BACKLASH_HARDSTOP_ALT_DEG     = 0.30f;  // 18' max
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
constexpr int      EEPROM_ADDR_AZM_RATIO = 12;  // RETIRED in v16 (was learned AZM ratio) — kept so offsets stay documented
constexpr int      EEPROM_ADDR_AZM_BLC  = 16;   // float (4): activeBacklashDegAZM   (v15.04)
constexpr int      EEPROM_ADDR_ALT_BLC  = 20;   // float (4): activeBacklashDegALT   (v15.04)
constexpr int      EEPROM_ADDR_BLC_MAGIC = 24;  // uint32 (4): BACKLASH_MAGIC        (v15.04)
constexpr uint32_t HOMING_MAGIC         = 0x484F4D45; // "HOME"
constexpr uint32_t BACKLASH_MAGIC       = 0x424C4348; // "BLCH" — backlash slots valid

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 8 — AZM: NO LEARNING (v16)
   AZM ratio and AZM backlash auto-learning were REMOVED after the v15.04-p5 audit
   (one-sided ratio estimator; backlash learner tracked noise — see header).
   The ratio is the machined harmonic-drive theoretical value (STEPS_PER_DEG_AZM);
   backlash compensation uses the persisted manual value (BLC:AZM:<deg>).
   ═══════════════════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 9 — MPU SAMPLING & TIMING
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr unsigned long SETTLE_DELAY_MS      = 500;   // Post-move settle before MPU sampling
constexpr uint8_t       MPU_SAMPLE_TARGET    = 50;    // Number of samples to average
constexpr unsigned long MPU_SAMPLE_INTERVAL_MS = 5;   // 5 ms between samples = 250 ms total

// v16: if the MPU stops answering mid-observe, abort instead of spinning forever.
// Budget = settle + sampling (~750 ms) + margin. After OBSERVE_FAIL_LIMIT
// consecutive aborted observations the MPU is declared dead for the session.
constexpr unsigned long OBSERVE_TIMEOUT_MS   = 3000;
constexpr uint8_t       OBSERVE_FAIL_LIMIT   = 2;

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SECTION 10 — MOTION RAMP PARAMETERS
   Trapezoidal velocity profile: start slow → cruise → decelerate.
   All values in microseconds between steps (smaller = faster).
   ═══════════════════════════════════════════════════════════════════════════════════════ */
constexpr unsigned long RAMP_START_US     = 2000;  // Starting speed (slowest)
constexpr unsigned long RAMP_CRUISE_AZM_US = 240;  // AZM cruise speed
// RAMP_CRUISE_ALT_US: PROTO=120µs  V2_CNC=150µs — set at runtime via cfg_RAMP_CRUISE_ALT_US
constexpr long          RAMP_LENGTH        = 500;   // 3000 was too long — caused NINA 7s timeout // Steps to reach cruise speed

// v16: any pause longer than this (feedHold, serial stall, observe insertion)
// restarts the acceleration ramp instead of resuming at full cruise speed
// (instant-cruise restart from standstill risks lost steps on the ALT worm).
constexpr unsigned long RAMP_RESUME_GAP_US = 20000;  // 20 ms

// v16: homing search cap. Was 50° — with a failed-open switch that meant
// ~6 minutes driving into the mechanical hard stop. 12° covers the full
// 10° travel plus margin.
constexpr float HOMING_SEARCH_RANGE_DEG = 12.0f;

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
// v16: AZM ratio is FIXED at the theoretical value — use STEPS_PER_DEG_AZM directly.

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

// v16: AZM learning state removed (ratio frozen, backlash manual — see header).

// v16: MPU observe-phase failure tracking (see OBSERVE_TIMEOUT_MS)
uint8_t mpuObserveFails = 0;

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
/* v16: ring buffer. The old linear buffer filled after ~25 jogs and silently
   stopped logging — the END of a long session (the part being debugged) was
   exactly what DIAG couldn't show. Now the oldest entries are overwritten. */
static char     diagLog[4096];
static uint16_t diagHead    = 0;      // next write position
static bool     diagWrapped = false;  // true once the buffer has cycled

void diagClear() { diagHead = 0; diagWrapped = false; }

void diagPrintf(const char* fmt, ...) {
  char tmp[192];
  va_list args;
  va_start(args, fmt);
  int n = vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);
  if (n <= 0) return;
  if (n >= (int)sizeof(tmp)) n = sizeof(tmp) - 1;  // entry truncated to tmp size
  for (int i = 0; i < n; i++) {
    diagLog[diagHead++] = tmp[i];
    if (diagHead >= sizeof(diagLog)) { diagHead = 0; diagWrapped = true; }
  }
}

// Dump the ring in chronological order (oldest → newest).
void diagDump() {
  if (diagWrapped) {
    Serial.println("[...oldest entries overwritten...]");
    Serial.write((const uint8_t*)diagLog + diagHead, sizeof(diagLog) - diagHead);
  }
  Serial.write((const uint8_t*)diagLog, diagHead);
}

/* ── Line-buffered command reader state ──
   v16: file-scope so scanSerialRealtime() can honour "only intercept '?' at
   the start of a line" (the p4 fix lived only in the loop() reader and was
   bypassed when the '?' of BLC? arrived in a separate UART chunk). ── */
static char    lineBuf[64];
static uint8_t lineIdx = 0;

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

    /* v16: LINE-BASED selection. The old byte-by-byte reader latched on the
       FIRST '1' or '2' seen in ANY traffic — "$J=G91G21X..." contains both,
       so connecting N.I.N.A. or the GUI before selecting could silently pick
       a (possibly wrong) profile and reboot. Now only an exact "1" or "2"
       line (digit + Enter) is accepted; anything else is ignored. */
    char    selBuf[8];
    uint8_t selIdx = 0;
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
          if (selIdx == 1 && (selBuf[0] == '1' || selBuf[0] == '2')) {
            profileId = selBuf[0] - '0';
            prefs.putInt("profile", profileId);
            prefs.end();
            Serial.print("Profile ");
            Serial.print(profileId == 1 ? "PROTO" : "V2");
            Serial.println(" saved to NVS. Rebooting in 1 second...");
            delay(1000);
            ESP.restart();   // Clean reboot — firmware re-reads NVS on next boot
          }
          selIdx = 0;  // not a valid selection — discard the line
        } else if (selIdx < sizeof(selBuf) - 1) {
          selBuf[selIdx++] = c;
        } else {
          selIdx = sizeof(selBuf) - 1;  // overlong line — will never match
        }
      }
      delay(10);
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
  long     rampCursor;       // v16: ramp position — reset to 0 after any pause
                             // (feedHold, stall) so motion re-accelerates
} mot = {false};

MotionJob jobQueue[2];
uint8_t   jobCount = 0;

void enqueueMotion(uint8_t stepPin, uint8_t dirPin, float deltaDeg,
                   float stepsPerDeg, volatile float* globalPos) {
  if (jobCount >= 2) return;  // Queue full — caller must flush first
  jobQueue[jobCount++] = {stepPin, dirPin, deltaDeg, stepsPerDeg, globalPos};
}

// Forward declarations (implementations below)
bool performPullOff(float stepsPerDeg);   // v16: returns false if aborted (0x18)
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
  if (isAzm) {
    int8_t newDir = (stepsTot > 0) ? 1 : -1;
    if (lastAzmDir != 0 && newDir != lastAzmDir && activeBacklashDegAZM > 0.0f) {
      backlashExtra = lroundf(activeBacklashDegAZM * job.stepsPerDeg);
      diagPrintf("AZM BACKLASH: %ld dead steps (%.1f')\n",
                 backlashExtra, activeBacklashDegAZM * 60.0f);
    }
    lastAzmDir = newDir;
  } else {
    /* v16: direction tracking + reversal flag are DECOUPLED from the >0
       injection gate. Previously, once the learned comp hit 0, reversals no
       longer set learningIsReversal → backlash learning was dead forever
       (and reversal moves contaminated ratio learning instead). */
    int8_t newDir = (stepsTot > 0) ? 1 : -1;
    if (lastAltDir != 0 && newDir != lastAltDir) {
      learningIsReversal = true;  // route the MPU observation to backlash learning
      if (activeBacklashDegALT > 0.0f) {
        backlashExtra = lroundf(activeBacklashDegALT * job.stepsPerDeg);
        diagPrintf("ALT BACKLASH: %ld dead steps (%.1f')\n",
                   backlashExtra, activeBacklashDegALT * 60.0f);
      }
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
  mot.rampCursor     = 0;    // v16: fresh ramp for every job
  isMoving           = true;
  return true;
}

/* v16 — purge any in-flight motion/observation state. Called before homing
   and by command paths that must take over cleanly. Positions are snapped to
   the current job target first so no phantom offset survives. */
void purgeMotion() {
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
  learningIsReversal     = false;
  lastAzmDir             = 0;
  lastAltDir             = 0;
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

  /* ── GLOBAL SETTLE: anti-vibration delay (GLOBAL_SETTLE_MS) before <Idle> ── */
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
    if (mpuSampleCount < MPU_SAMPLE_TARGET) {
      /* v16: TIMEOUT GUARD. If the MPU stops answering (I2C glitch, wiring),
         samples never accumulate and this phase used to spin forever with
         isMoving=true → <Run forever → TPPA timeout, no escape. */
      if (millis() - settleStartMs > SETTLE_DELAY_MS + OBSERVE_TIMEOUT_MS) {
        mpuObserveFails++;
        diagPrintf("OBSERVE TIMEOUT: %d/%d samples, fail #%u\n",
                   mpuSampleCount, MPU_SAMPLE_TARGET, mpuObserveFails);
        if (mpuObserveFails >= OBSERVE_FAIL_LIMIT) {
          mpuAvailable = false;   // MPU dead for this session — skip all future observes
          diagPrintf("MPU DISABLED for session (I2C unresponsive)\n");
        }
        settlingForObserve = false;
        mpuSampleCount     = 0;
        mpuSumAngles       = 0.0f;
        learningIsReversal = false;
        inFeedbackCycle    = false;
        posDegALT          = targetAltAngle;   // snap — no learning this move
        if (!startNextJob()) {
          isMoving = false;
          diagPrintf("IDLE (observe aborted)\n");
        }
      }
      return;
    }

    // Phase 3: compute average and update ratio
    float rawAngle         = mpuSumAngles / (float)mpuSampleCount;
    settlingForObserve     = false;
    mpuSampleCount         = 0;
    mpuSumAngles           = 0.0f;
    mpuObserveFails        = 0;    // v16: healthy observation resets the fail streak

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
      if (!altRatioConverged) {
        /* v16: GATE on ratio convergence. While the ALT ratio is still being
           learned, a ratio error of a few % on a reversal move masquerades as
           a backlash residual (e.g. 5% × 0.3° ≈ 0.9' of false signal vs a
           true B of 2-3'). Learn backlash only once the ratio is locked. */
        diagPrintf("ALT BLC ML: ratio not converged yet — skip\n");
      } else if (fabsf(learningRequestedDelta) >= MIN_BLC_LEARNING_ANGLE) {
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
          // v16: threshold relative to the theoretical ratio (see RATIO_STABLE_FRACT)
          if (change > STEPS_PER_DEG_ALT * RATIO_STABLE_FRACT) {
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

  // Ramp position = min(ramp cursor, steps remaining, RAMP_LENGTH)
  // This gives symmetric acceleration and deceleration.
  // v16: the accel side uses rampCursor (not stepsDone) so a pause can
  // restart the ramp without affecting position bookkeeping.
  long rampPos    = mot.rampCursor;
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
  unsigned long sinceLast = now - mot.lastStepUs;
  if (sinceLast < interval) return;  // Not time yet — yield

  /* v16: RE-RAMP AFTER PAUSE. A gap ≫ interval means motion was suspended
     (feedHold released, serial stall, DIAG dump…). Restarting at cruise
     speed from a standstill risks lost steps — re-enter the ramp instead. */
  if (sinceLast > RAMP_RESUME_GAP_US && mot.rampCursor > 0) {
    mot.rampCursor = 0;
    mot.lastStepUs = now;   // next step fires after the (slow) ramp-start interval
    return;
  }
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
  mot.rampCursor++;

  scanSerialRealtime();  // Service '?' polls mid-move for smooth N.I.N.A. status display

  /* ── HARDWARE SAFETY LIMIT ──
     If the physical limit switch triggers during a downward ALT move (outside homing),
     stop immediately, pull off, and redefine position as 0° (re-home in place). ── */
  if (!mot.isAzm && mot.deltaDeg < 0 && digitalRead(PIN_HOME_SENSOR) == LOW) {
    Serial.println("\n!!! CRITICAL ALARM: PHYSICAL LIMIT SWITCH HIT OUTSIDE HOMING !!!");
    mot.active             = false;
    jobCount               = 0;
    inFeedbackCycle        = false;
    waitingForGlobalSettle = false;
    learningRequestedDelta = 0;      // v16: don't let the aborted move feed learning
    learningIsReversal     = false;  // v16
    lastAltDir             = 0;      // v16: direction history invalid after alarm
    (void)performPullOff(mot.stepsPerDeg);  // abort inside is handled by tickMotion next pass
    isMoving               = false;  // v16: only AFTER pull-off — the pull-off now answers
                                     // '?' polls, which must report Run, not a stale Idle
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
  Serial.println(",0|>");   // v16: properly closed GRBL report (TPPA regex keeps matching)
}

// v16: single point of truth for GRBL realtime characters.
// '!' '~' 0x18 never occur inside a legitimate command → intercepted anywhere.
// '?' DOES occur inside commands (BLC?, MPU?) → intercepted only at the start
// of a line (lineIdx==0), which is the semantic the p4 fix intended but only
// enforced in the loop() reader (bypassed when '?' arrived in its own chunk).
// Realtime chars get no "ok" reply (GRBL realtime commands are silent).
bool handleRealtimeChar(char c) {
  if      (c == '!')  { feedHold = true;  return true; }
  else if (c == '~')  { feedHold = false; return true; }
  else if (c == 0x18) { abortCmd = true;  return true; }
  return false;
}

// Called between motor step pulses and from blocking phases (homing, pull-off)
// so N.I.N.A.'s 10 Hz '?' polls stay serviced during long operations.
void scanSerialRealtime() {
  while (Serial.available()) {
    char c = Serial.peek();
    if (c == '!' || c == '~' || c == 0x18) { Serial.read(); handleRealtimeChar(c); continue; }
    if (c == '?') {
      if (lineIdx == 0) { Serial.read(); sendStatus(); Serial.println(); continue; }
      // Mid-line '?' belongs to the command (BLC?, MPU?) — absorb it into
      // lineBuf here so it can't sit at the queue head during a blocking
      // phase and starve the '?'/'!'/0x18 characters queued behind it.
      Serial.read();
      if (lineIdx < sizeof(lineBuf) - 1) lineBuf[lineIdx++] = c;
      continue;
    }
    break;  // ordinary command byte — leave it for the loop() reader
  }
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
  mpuObserveFails        = 0;       // v16
  diagClear();
  Serial.println("\r\nGrbl 1.1h ['$' for help]");
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   HOMING SEQUENCE
   ═══════════════════════════════════════════════════════════════════════════════════════ */

// Pull-off: after the limit switch triggers, move UP until the switch releases,
// then advance by HOME_SAFETY_MARGIN to establish a clean mechanical zero.
// v16: services serial ('?' polls answered, 0x18 honoured) and returns false
// if aborted. Previously this ran deaf and blind for up to ~80 s worst-case.
bool performPullOff(float stepsPerDeg) {
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
    if (count % 64 == 0) {
      scanSerialRealtime();          // v16: keep NINA polls alive
      if (abortCmd) return false;    // v16: honour soft-reset
      yield();
    }
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
    if (s % 64 == 0) {
      scanSerialRealtime();
      if (abortCmd) return false;
      yield();
    }
  }
  return true;
}

void startHoming() {
  /* v16: purge ANY in-flight motion first. Previously a HOME received during
     a move left mot.active=true — after homing, tickMotion resumed the stale
     job and snapped to a pre-homing target in the new coordinate frame. */
  purgeMotion();

  Serial.println("MSG: Homing ALT axis...");
  isMoving               = true;
  diagClear();

  // If already on the switch, pull off first
  if (digitalRead(PIN_HOME_SENSOR) == LOW) {
    if (!performPullOff(activeStepsPerDegALT)) { softReset(); return; }
    delay(200);
  }

  // Drive DOWN toward the limit switch
  bool dirDown = cfg_AXIS_REV_ALT ? HIGH : LOW;
  digitalWrite(PIN_DIR_ALT, dirDown);
  delay(10);

  int  confirmLow = 0;
  bool hit        = false;

  // v16: search capped at 12° (full 10° travel + margin). The old 50° cap
  // meant ~6 minutes driving into the hard stop if the switch had failed open.
  for (long s = 0; s < (long)(HOMING_SEARCH_RANGE_DEG * activeStepsPerDegALT); s++) {
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
  if (!performPullOff(activeStepsPerDegALT)) { softReset(); return; }

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
      /* v16: a failed tare now FAILS the homing. Previously the code carried
         on: homingDone=true and the stale/zero offset was committed to EEPROM
         with a valid magic — the next boot restored a wrong absolute ALT
         reference. Also invalidate any stale magic so no old state restores. */
      Serial.println("ALARM: Gyroscope Tare Failed (I2C Bus unresponsive).");
      Serial.println("ALARM: Homing NOT completed — fix MPU wiring and re-run HOME.");
      uint32_t noMagic = 0;
      EEPROM.put(EEPROM_ADDR_MAGIC, noMagic);
      EEPROM.commit();
      isMoving   = false;
      homingDone = false;
      return;
    }
  }

  isMoving               = false;
  homingDone             = true;
  learningRequestedDelta = 0;
  altStableCount         = 0;       // re-validate ALT ratio over first few jogs of session
  altRatioConverged      = false;
  learningIsReversal     = false;   // v15.04
  altBacklashSamples     = 0;       // v15.04: restart warmup α (mechanics may have shifted)
  mpuObserveFails        = 0;       // v16

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
  Serial.print("\n--- SYSTEM DIAGNOSTIC (v16.00) [");
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
  Serial.println("AZM learning          : DISABLED (v16 — ratio fixed, backlash manual)");

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
  Serial.print("AZM Ratio (fixed)     : "); Serial.print(STEPS_PER_DEG_AZM, 3);    Serial.println(" steps/deg");
  Serial.print("ALT RMS Current       : "); Serial.print(RMS_CURRENT_ALT); Serial.println(" mA");
  Serial.print("AZM Cruise            : "); Serial.print(RAMP_CRUISE_AZM_US); Serial.println(" µs");
  Serial.print("ALT Cruise            : "); Serial.print(cfg_RAMP_CRUISE_ALT_US); Serial.println(" µs");
  Serial.print("Ramp length           : "); Serial.print(RAMP_LENGTH); Serial.println(" steps");

  Serial.println("");

  // EEPROM
  Serial.print("EEPROM ALT Ratio      : ");
  float stored = 0.0f; EEPROM.get(EEPROM_ADDR_RATIO, stored); Serial.println(stored, 3);
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
  Serial.print("Diag buffer           : ");
  Serial.print(diagWrapped ? (unsigned)sizeof(diagLog) : diagHead);
  Serial.print("/"); Serial.print(sizeof(diagLog));
  Serial.println(diagWrapped ? " (ring wrapped)" : "");

  // Command & learning log (v16: chronological ring dump)
  if (diagHead > 0 || diagWrapped) {
    Serial.println("");
    Serial.println("--- COMMAND & FEEDBACK LOG ---");
    diagDump();
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

  /* ── PROFILE:RESET (v16 — was documented since v15 but never implemented) ──
     Clears the NVS profile and reboots into the first-boot selection menu. */
  if (strcmp(line, "PROFILE:RESET") == 0) {
    Preferences prefs;
    prefs.begin("polaralign", false);
    prefs.putInt("profile", 0);
    prefs.end();
    Serial.println("MSG: Profile cleared from NVS — rebooting into selection menu...");
    delay(200);
    ESP.restart();
  }

  /* ── BLC: backlash compensation query & set (v15.04) ── */
  if (strcmp(line, "BLC?") == 0) {
    Serial.print("BLC:AZM="); Serial.print(activeBacklashDegAZM, 4);
    Serial.print(" ALT="); Serial.print(activeBacklashDegALT, 4);
    Serial.print(" ("); Serial.print(activeBacklashDegAZM * 60.0f, 2); Serial.print("'/");
    Serial.print(activeBacklashDegALT * 60.0f, 2); Serial.print("')  AZM=manual  ALT samples=");
    Serial.println(altBacklashSamples);
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
    if (isMoving) purgeMotion();   // v16: single purge helper (shared with HOME)

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
      long   azmStepsTot = lroundf(fabsf(azmDeltaDeg) * STEPS_PER_DEG_AZM);
      diagPrintf("AZM delta=%.4f deg steps=%ld\n", azmDeltaDeg, azmStepsTot);

      /* v16: AZM ratio + backlash learning REMOVED (see header).
         The ratio is the machined theoretical value; backlash compensation
         uses the persisted manual value and is injected in startNextJob(). */

      enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, azmDeltaDeg, STEPS_PER_DEG_AZM, &posDegAZM);
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
  // v16: guard widened — refuse during ANY motion/settle/observe (the old
  // `isMoving && mot.isAzm` let it fire during ALT moves and during the AZM
  // settle window after mot.active had already cleared).
  if (strcmp(line, "AZM:ZERO") == 0) {
    if (isMoving || mot.active) { Serial.println("!BUSY — finish or abort motion first"); return; }
    posDegAZM  = 0.0f;
    lastAzmDir = 0;
    Serial.println("MSG: AZM absolute position forcefully reset to 0.0");
    sendStatus(); Serial.println();
    return;
  }

  if (strncmp(line, "AZM:", 4) == 0) {
    if (isMoving) purgeMotion();  // v16: no more clobbering of settle/observe state
    float tgt = atof(line + 4);
    if (tgt < AZM_LIMIT_NEG) tgt = AZM_LIMIT_NEG;
    if (tgt > AZM_LIMIT_POS) tgt = AZM_LIMIT_POS;
    enqueueMotion(PIN_STEP_AZM, PIN_DIR_AZM, tgt - posDegAZM, STEPS_PER_DEG_AZM, &posDegAZM);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println();
    return;
  }

  if (strncmp(line, "ALT:", 4) == 0) {
    if (isMoving) purgeMotion();  // v16: no more clobbering of settle/observe state
    float tgt = atof(line + 4);
    if (tgt < cfg_ALT_LIMIT_NEG) tgt = cfg_ALT_LIMIT_NEG;
    if (tgt > ALT_LIMIT_POS) tgt = ALT_LIMIT_POS;
    enqueueMotion(PIN_STEP_ALT, PIN_DIR_ALT, tgt - posDegALT, activeStepsPerDegALT, &posDegALT);
    startNextJob();
    Serial.println("OK"); sendStatus(); Serial.println();
    return;
  }

  /* ── v16: GRBL-ish strictness — anything unrecognized gets an error reply
     instead of silence (a send-and-wait host used to hang forever). ── */
  Serial.print("error:20 (unsupported: ");
  Serial.print(line);
  Serial.println(")");
}

/* ═══════════════════════════════════════════════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════════════════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(300);  // short — ESP32 must answer within TPPA's connection timeout (~1-1.5s)

  // v16: hold the stepper drivers DISABLED before anything that can block
  // (profile menu). Previously PIN_EN floated until GPIO init further down.
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);   // Active LOW → HIGH = drivers disabled

  // Loads profile from NVS, sets cfg_ vars, computes STEPS_PER_DEG_ALT.
  // On first boot after flash: blocks until user sends "1" or "2" + Enter.
  loadOrSelectProfile();

  Serial.println("\n=======================================================");
  Serial.print("  BOOT: V16.00 ["); Serial.print(cfg_profile_name); Serial.println("] (ESP32)");
  Serial.print("  Profile: "); Serial.print(cfg_profile_name);
  Serial.print("  ALT_GEARBOX="); Serial.print(cfg_ALT_MOTOR_GEARBOX,1);
  Serial.print("  AXIS_REV_ALT="); Serial.println(cfg_AXIS_REV_ALT ? "true":"false");
  Serial.println("  AZM: fixed ratio, manual BLC  TPPA: ARCMINUTES  HOME required");
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

  /* ── AZM ratio: fixed (v16) — EEPROM slot 12 retired, nothing to load ── */
  Serial.print("MSG: AZM Ratio fixed at theoretical: ");
  Serial.println(STEPS_PER_DEG_AZM);

  /* ── Load backlash values (v15.04) — migration-safe ──
     If BACKLASH_MAGIC is invalid (fresh flash, upgrade from v15.03g), keep the
     profile-default values that loadOrSelectProfile() already installed. ── */
  uint32_t storedBlcMagic = 0;
  EEPROM.get(EEPROM_ADDR_BLC_MAGIC, storedBlcMagic);
  if (storedBlcMagic == BACKLASH_MAGIC) {
    float bAZM = 0.0f, bALT = 0.0f;
    EEPROM.get(EEPROM_ADDR_AZM_BLC, bAZM);
    EEPROM.get(EEPROM_ADDR_ALT_BLC, bALT);
    // Guardrails: reject NaN/Inf/negative/out-of-band (v16: use the hardstop
    // constants — a stored value above the new ALT hardstop is clamped out)
    if (!isnan(bAZM) && !isinf(bAZM) && bAZM >= 0.0f && bAZM <= BACKLASH_HARDSTOP_AZM_DEG) {
      activeBacklashDegAZM = bAZM;
    }
    if (!isnan(bALT) && !isinf(bALT) && bALT >= 0.0f) {
      // Migration: a v15 value above the new 0.3° hardstop is CLAMPED, not discarded
      activeBacklashDegALT = (bALT <= BACKLASH_HARDSTOP_ALT_DEG)
                             ? bALT : BACKLASH_HARDSTOP_ALT_DEG;
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

  /* ── GPIO init (PIN_EN already OUTPUT+HIGH since the top of setup) ── */
  pinMode(PIN_DIR_AZM,     OUTPUT); pinMode(PIN_STEP_AZM, OUTPUT);
  pinMode(PIN_DIR_ALT,     OUTPUT); pinMode(PIN_STEP_ALT, OUTPUT);
  pinMode(PIN_HOME_SENSOR, INPUT);  pinMode(PIN_BUTTON_HOME, INPUT);
  // v16: drivers stay DISABLED until the TMC2209 config below is applied

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

  digitalWrite(PIN_EN, LOW);   // v16: enable drivers only now that they are configured

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

      /* v16: average ~10 reads instead of trusting a single one — the tare
         this is compared against is a 50-sample average; one noisy read used
         to become the absolute ALT reference for the whole session. */
      float rawAngle = -999.0f;
      { float sum = 0.0f; int n = 0;
        for (int i = 0; i < 10; i++) {
          float r = readMPUAngleY();
          if (r > -900.0f) { sum += r; n++; }
          delay(5);
        }
        if (n >= 5) rawAngle = sum / (float)n;   // ≥5 valid reads required
      }

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

  // Line-buffered serial command reader (lineBuf/lineIdx are file-scope —
  // shared with scanSerialRealtime so the '?' line-start rule is consistent).
  while (Serial.available()) {
    char c = Serial.peek();
    // v16: '!' '~' 0x18 are realtime ANYWHERE (they never occur inside a
    // command — a mid-line 0x18 used to be swallowed into the command text).
    if (c == '!' || c == '~' || c == 0x18) {
      Serial.read();
      handleRealtimeChar(c);
      continue;
    }
    // '?' at line start belongs to scanSerialRealtime (status query);
    // mid-line it is part of a command (BLC?, MPU?).
    if (lineIdx == 0 && c == '?') break;
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
