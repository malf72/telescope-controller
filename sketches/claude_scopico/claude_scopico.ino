/*
 * Telescope controller - claude_scopico (sketch #6): DRV8871 drivers, new pins
 *   Same firmware as sketch #5 (telescope_wifi) re-targeted at the rewired
 *   hardware: two DRV8871 driver boards instead of the TB6612, new GPIOs,
 *   and an 'auto dir' motor-direction check for after any wiring change.
 *
 * Sketch #5 notes: WiFi, star alignment, LX200 goto/sync
 * Board:  Raspberry Pi Pico 2 W  (arduino-pico core, RP2350)
 * Tools -> Flash Size: pick an option WITH a filesystem, e.g. "4MB (Sketch: 3584KB, FS: 512KB)"
 *
 * Builds on sketch #4 (telescope_calib: servo, auto-tune, saved calibration). Adds:
 *   - the 1 kHz servo runs on CORE 1 (its own alarm pool), so WiFi/network
 *     work on core 0 can never stall motor control; state shared between the
 *     cores is guarded by a spin-lock critical section (SERVO_LOCK)
 *   - WiFi (credentials saved in flash via the 'wifi' command), NTP time,
 *     mDNS name telescope.local
 *   - LX200 server on TCP port 4030 (lx200_core.h): Stellarium connects via
 *     INDI (indi_lx200basic, Network connection) for goto + sync; SkySafari etc.
 *     can connect directly
 *   - the serial console mirrored on telnet port 23 ('telnet telescope.local')
 *   - coordinate engine + Taki star-sync alignment model (coord_core.h), for an
 *     equatorial wedge or alt-az: goto, sidereal/alt-az tracking, manual moves
 *   - site / mount settings saved in /sky.txt, alignment saved by 'park'
 *
 * From sketches #3/#4: gain-scheduled feedforward+PI servo (aggressive while
 * MOVING, gentle while HOLDing -- see servo_core.h), 'auto full' calibration,
 * calibration file /calib.txt (calib_config.h).
 *
 * WIRING (2026-11-01, DRV8871 rebuild) -- encoders through 10k/20k dividers
 * from 5 V (3.33 V at the pin; GP26/27 are ADC-capable pins and are not 5 V
 * tolerant, so the dividers are required there):
 *   Axis 0 (Az/RA):  encoder ch1 -> GP6,  ch2 -> GP7    DRV8871 IN1 GP12, IN2 GP13
 *   Axis 1 (Alt/Dec): encoder ch1 -> GP27, ch2 -> GP26   DRV8871 IN1 GP20, IN2 GP21
 * The PIO decoder reads two consecutive pins starting at the lower one, so
 * the alt encoder (ch1 on the HIGHER pin) counts in the opposite sense to its
 * wiring -- harmless: invertMotor[] makes positive drive give positive counts,
 * and 'auto dir' sets it from a measurement.
 * DRV8871: IN1/IN2 = 0/0 coast, 1/0 forward, 0/1 reverse, 1/1 brake. No PWM
 * or standby pin: both IN pins are PWM outputs (GP12/13 = PWM slice 6 A/B,
 * GP20/21 = slice 2 A/B).
 */
#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <SimpleMDNS.h>
#include <sys/time.h>
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "pico/critical_section.h"
#include "quadrature_encoder.pio.h"
#include "servo_core.h"
#include "calib_config.h"
#include "coord_core.h"
#include "lx200_core.h"
#include "sky_config.h"

#if NUM_PIOS < 3
#error "Select the Raspberry Pi Pico 2 W board (RP2350 has 3 PIO blocks)."
#endif

// ---------------------------------------------------------------- console (USB serial + telnet)
// Everything the sketch prints goes through Con: always to USB serial, and
// also to the telnet console client when one is connected. Telnet output is
// queued here and sent from loop() (flushTelnet), so a slow network link can
// never block a print.
static WiFiClient telnetClient;
static char       telnetOut[2048];
static size_t     telnetOutLen = 0;

class ConsolePrint : public Print {
public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *b, size_t n) override {
    Serial.write(b, n);
    if (telnetClient) {
      for (size_t i = 0; i < n && telnetOutLen < sizeof(telnetOut) - 1; i++) {
        if (b[i] == '\n') telnetOut[telnetOutLen++] = '\r';   // telnet wants CRLF
        telnetOut[telnetOutLen++] = b[i];
      }
    }
    return n;
  }
};
static ConsolePrint Con;

// ---------------------------------------------------------------- pins
static const uint    ENC_PIN[2]  = {6, 26};   // LOWER pin of each encoder pair (decoder reads pin, pin+1)
static const uint8_t IN1_PIN[2]  = {12, 20};  // DRV8871 IN1
static const uint8_t IN2_PIN[2]  = {13, 21};  // DRV8871 IN2
static const uint16_t PWM_TOP    = 1023;      // PWM counts per period: level 0 = always low, PWM_TOP+1 = always high
static const uint32_t PWM_FREQ   = 20000;     // Hz, above hearing

// ---------------------------------------------------------------- settings
static const bool     USE_INTERNAL_PULLUPS = false;
static const uint32_t MAX_STEP_RATE  = 100000;
static const double   SERVO_DT       = 0.001;
static bool           invertMotor[2] = {false, false}; // set by 'auto dir' (old TB6612 values don't apply to the rewired hardware)

// ---------------------------------------------------------------- mechanics (constants in coord_core.h)
using coord::TOTAL_RATIO;
using coord::CPR;
using coord::SIDEREAL_CPS;
// bench seed for the feedforward default (only used until 'auto full' has been run):
// ~7 % duty gave 10.34 counts/s -- measured when CPR was taken as 108, so that
// speed is really 1/4 sidereal; kept as the measured (duty, speed) pair.
static const double MEASURED_SEED_DUTY = 7.0, MEASURED_SEED_CPS = 10.3435;

// ---------------------------------------------------------------- runtime state
static PIO      encPio = pio2;
static AxisCtrl axis[2];
static int32_t  zeroOffset[2] = {0, 0};
static uint     in1Slice[2], in1Chan[2], in2Slice[2], in2Chan[2];
static bool     motorsEnabled[2] = {false, false};
static volatile bool faulted[2]  = {false, false};
static float    faultThreshold[2] = {200.0f, 200.0f};   // following-error limit at standstill, counts
static const double FAULT_LAG_S = 0.1;                    // plus this many seconds of travel at the current speed
static repeating_timer_t servoTimer;
static alarm_pool_t     *servoPool;           // created on core 1, so the servo alarm fires there
static critical_section_t servoLock;          // guards axis[] etc. between core 0 and core 1
static volatile bool     core0Ready = false;  // core 1 starts the servo only after setup() finished
// noInterrupts() only masks the calling core; with the servo on core 1 every
// read-modify-write of shared servo state must take this spin lock instead.
#define SERVO_LOCK()   critical_section_enter_blocking(&servoLock)
#define SERVO_UNLOCK() critical_section_exit(&servoLock)
static volatile bool testMode[2] = {false, false};  // true = loop() owns this axis's hardware, ISR hands off
static bool     fsReady = false;       // LittleFS mounted
static bool     autoFullBusy = false;  // 'auto full' sequence in progress (owns both axes until done)

static volatile int32_t posSnap[2];
static volatile double  errSnap[2], targetSnap[2], dutySnap[2];
static volatile double  velSnap[2];          // measured axis speed, counts/s (over the last 10 ms)

// what the servo was doing at the moment a following-error fault tripped
struct FaultInfo { double err, limit, targetVel, measVel, duty; int mode; float supplyV; };

// Motor supply monitor: 12 V -> 10k -> GP28 (ADC2) -> 1k -> GND, i.e. 11:1.
// Read every servo tick on core 1. Below SUPPLY_PRESENT_V it's treated as
// "not fitted" (floating pin) rather than a 0 V supply.
static const uint   SUPPLY_ADC_PIN = 28, SUPPLY_ADC_INPUT = 2;
static const double SUPPLY_DIVIDER = 11.0;       // (10k + 1k) / 1k
static const double SUPPLY_PRESENT_V = 3.0;
static volatile float supplyV = 0, supplyMinV = 99, supplyMaxV = 0;
static FaultInfo faultInfo[2];
static volatile bool faultInfoNew[2] = {false, false};

// High-rate capture ('cap'): the servo ISR records one axis at CAP_RATE_HZ so
// periodic drag can be traced to the part turning at that rate (motor, a
// gearbox stage, the worm). Armed from core 0, filled on core 1.
enum { CAP_IDLE, CAP_ARMED, CAP_RUN, CAP_DONE };
static const int    CAP_RATE_HZ = 500;            // every 2nd servo tick
static const int    CAP_MAX     = 10 * CAP_RATE_HZ;
static const double CAP_START_CPS = 1000;         // armed capture starts once the axis moves this fast
static volatile int capState = CAP_IDLE;
static volatile int capAxis = 0, capN = 0, capWant = 0;
static int32_t      capPos[CAP_MAX];
static float        capDuty[CAP_MAX], capErr[CAP_MAX];
static uint32_t     capArmedMs = 0;

// Fault "black box": the ISR keeps the last BB_N ticks (1 kHz) of position and
// duty for each axis and freezes on a fault, so the lead-up can be inspected:
// how fast the speed fell tells encoder dropout / jam (a few ms) from loss of
// drive power (tens of ms of coasting).
static const int BB_N = 512;
static int32_t   bbPos[2][BB_N];
static float     bbDuty[2][BB_N];
static float     bbVolt[2][BB_N];
static volatile int  bbHead[2] = {0, 0};          // next slot to write
static volatile bool bbFrozen[2] = {false, false};
static bool haveFaultInfo[2] = {false, false};

// ---------------------------------------------------------------- encoders (PIO)
static int32_t readRaw(int a) {
  int n = pio_sm_get_rx_fifo_level(encPio, a) + 1;
  uint32_t v = 0;
  while (n-- > 0) v = pio_sm_get_blocking(encPio, a);
  return (int32_t)v;
}
static int32_t countOf(int a) { return readRaw(a) - zeroOffset[a]; }

static void encoderBegin(int a) {
  uint pin = ENC_PIN[a];
  pio_sm_claim(encPio, a);
  pio_gpio_init(encPio, pin);
  pio_gpio_init(encPio, pin + 1);
  pio_sm_set_consecutive_pindirs(encPio, a, pin, 2, false);
  if (USE_INTERNAL_PULLUPS) { gpio_pull_up(pin); gpio_pull_up(pin + 1); }
  else                      { gpio_disable_pulls(pin); gpio_disable_pulls(pin + 1); }

  pio_sm_config c = quadrature_encoder_program_get_default_config(0);
  sm_config_set_in_pins(&c, pin);
  sm_config_set_in_shift(&c, false, false, 32);
  sm_config_set_out_shift(&c, true, false, 32);
  sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (MAX_STEP_RATE * 14.0f));
  pio_sm_init(encPio, a, 0, &c);
  pio_sm_exec(encPio, a, pio_encode_set(pio_y, 0));
  pio_sm_set_enabled(encPio, a, true);
}

// ---------------------------------------------------------------- raw motor hardware (DRV8871)
// Direct register writes -- safe to call from the ISR or from loop(), never both
// for the same axis at the same time (testMode[] arbitrates which one is allowed to).
//
// Slow-decay PWM: one input held high, the other pulsed. During the "off" part
// of each period both inputs are high = brake, so the motor current
// recirculates through the driver instead of the supply. That is what the
// TB6612 did (PWM low = short brake) and it gives the near-linear duty->speed
// behaviour at low duty that tracking depends on. Forward at duty d:
// IN1 = high, IN2 = low for d of the period and high (brake) for the rest.
static void setInLevels(int a, uint32_t l1, uint32_t l2) {
  pwm_set_chan_level(in1Slice[a], in1Chan[a], l1);
  pwm_set_chan_level(in2Slice[a], in2Chan[a], l2);
}
static void writeMotorHardware(int a, double pct) {
  if (pct == 0.0) { setInLevels(a, 0, 0); return; }           // coast
  const uint32_t FULL = PWM_TOP + 1;                           // always high
  double d = fabs(pct) / 100.0;
  if (d > 1.0) d = 1.0;
  uint32_t pulsed = (uint32_t)((1.0 - d) * FULL + 0.5);        // high time of the pulsed input
  bool fwd = (pct > 0) != invertMotor[a];
  if (fwd) setInLevels(a, FULL, pulsed);
  else     setInLevels(a, pulsed, FULL);
}

// Brake (IN1 = IN2 = high): the motor's own back-EMF stops it quickly.
static void brakeMotorHardware(int a) {
  setInLevels(a, PWM_TOP + 1, PWM_TOP + 1);
}

// Both IN pins of an axis become PWM outputs at PWM_FREQ with PWM_TOP+1 steps.
// Configured directly (not analogWrite, whose top and scaling the core adjusts).
static void motorPwmBegin(int a) {
  const uint pins[2] = {IN1_PIN[a], IN2_PIN[a]};
  for (int k = 0; k < 2; k++) {
    uint pin = pins[k], slice = pwm_gpio_to_slice_num(pin);
    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / ((float)(PWM_TOP + 1) * PWM_FREQ));
    pwm_config_set_wrap(&c, PWM_TOP);
    pwm_init(slice, &c, false);
    pwm_set_gpio_level(pin, 0);                                // low before the pin is connected
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_enabled(slice, true);
  }
  in1Slice[a] = pwm_gpio_to_slice_num(IN1_PIN[a]); in1Chan[a] = pwm_gpio_to_channel(IN1_PIN[a]);
  in2Slice[a] = pwm_gpio_to_slice_num(IN2_PIN[a]); in2Chan[a] = pwm_gpio_to_channel(IN2_PIN[a]);
}

// Hand an axis back to the normal servo. Called from loop() at the end of any
// auto-tune procedure. The motor is braked and the servo only takes over once
// the axis has actually stopped (updateHandback): grabbing the position while
// the motor is still coasting -- e.g. right after the 'auto speed' step at md --
// leaves the gentle hold gains chasing a moving target and trips the
// following-error fault. testMode[] stays set until the handover completes.
static bool     handbackPending[2] = {false, false};
static uint32_t handbackStartMs[2], handbackStillSinceMs[2];
static int32_t  handbackAnchor[2];
static const uint32_t HANDBACK_STILL_MS   = 300;   // no movement for this long = stopped
static const uint32_t HANDBACK_TIMEOUT_MS = 3000;  // hand over anyway after this

static void returnToNormalServo(int a) {
  brakeMotorHardware(a);
  handbackPending[a]      = true;
  handbackStartMs[a]      = millis();
  handbackStillSinceMs[a] = handbackStartMs[a];
  handbackAnchor[a]       = countOf(a);
}

static void finishHandback(int a) {
  writeMotorHardware(a, 0);
  int32_t pos = countOf(a);
  SERVO_LOCK();
  axis[a].targetPos = (double)pos;
  axis[a].targetVel = 0;
  axis[a].integral  = 0;
  axis[a].mode = MODE_HOLD;
  SERVO_UNLOCK();
  testMode[a] = false;
}

static void updateHandback(int a) {
  if (!handbackPending[a]) return;
  if (!testMode[a]) { handbackPending[a] = false; return; }  // axis was disabled meanwhile
  uint32_t now = millis();
  int32_t c = countOf(a);
  if (abs(c - handbackAnchor[a]) > 1) { handbackAnchor[a] = c; handbackStillSinceMs[a] = now; }  // 1-count edge jitter ok
  if (now - handbackStillSinceMs[a] >= HANDBACK_STILL_MS || now - handbackStartMs[a] >= HANDBACK_TIMEOUT_MS) {
    handbackPending[a] = false;
    finishHandback(a);
  }
}

// ---------------------------------------------------------------- 1 kHz servo ISR (runs on core 1)
static bool servoISR(repeating_timer_t *) {
  {
    float v = adc_read() * (3.3f / 4095.0f) * (float)SUPPLY_DIVIDER;   // ~2 us
    supplyV = v;
    if (v >= SUPPLY_PRESENT_V) {
      if (v < supplyMinV) supplyMinV = v;
      if (v > supplyMaxV) supplyMaxV = v;
    }
  }
  for (int a = 0; a < 2; a++) {
    if (testMode[a]) continue;  // loop() owns this axis right now (an auto-tune test is running)
    SERVO_LOCK();
    int32_t pos = countOf(a);
    static int32_t posHist[2][10];
    static uint8_t histIdx[2];
    velSnap[a] = (pos - posHist[a][histIdx[a]]) / (10 * SERVO_DT);
    posHist[a][histIdx[a]] = pos;
    histIdx[a] = (histIdx[a] + 1) % 10;
    double duty = servoStep(axis[a], (double)pos, SERVO_DT);

    // following-error limit grows with speed: 0.1 s of lag is fine at slew speed
    // (200 counts is only 10 ms at ~19,000 counts/s) but still catches a stall
    double faultAt = faultThreshold[a] + fabs(axis[a].targetVel) * FAULT_LAG_S;
    if (motorsEnabled[a] && !faulted[a] && fabs(axis[a].lastErr) > faultAt) {
      faulted[a] = true;
      faultInfo[a] = {axis[a].lastErr, faultAt, axis[a].targetVel, velSnap[a], duty, (int)axis[a].mode, supplyV};
      faultInfoNew[a] = true;
      { int h = bbHead[a]; bbPos[a][h] = pos; bbDuty[a][h] = (float)duty; bbVolt[a][h] = supplyV; bbHead[a] = (h + 1) % BB_N; }
      bbFrozen[a] = true;
      haveFaultInfo[a] = true;
      // freeze the target where the motor is: a slew profile would otherwise
      // run on to its goal with the motor stopped, which makes 'p' misleading
      axis[a].mode = MODE_HOLD;
      axis[a].targetPos = pos;
      axis[a].targetVel = axis[a].velCmd = 0;
      axis[a].integral = 0;
      duty = 0;
    }
    if (!motorsEnabled[a] || faulted[a]) duty = 0;
    writeMotorHardware(a, duty);

    posSnap[a]    = pos;
    errSnap[a]    = axis[a].lastErr;
    targetSnap[a] = axis[a].targetPos;
    dutySnap[a]   = duty;
    if (!bbFrozen[a]) {
      int h = bbHead[a];
      bbPos[a][h] = pos; bbDuty[a][h] = (float)duty; bbVolt[a][h] = supplyV;
      bbHead[a] = (h + 1) % BB_N;
    }
    if (a == capAxis && capState != CAP_IDLE && capState != CAP_DONE) {
      static uint8_t capTick = 0;
      if (capState == CAP_ARMED && fabs(velSnap[a]) >= CAP_START_CPS) { capState = CAP_RUN; capN = 0; capTick = 0; }
      if (capState == CAP_RUN && (capTick++ & 1) == 0) {
        capPos[capN] = pos; capDuty[capN] = (float)duty; capErr[capN] = (float)axis[a].lastErr;
        if (++capN >= capWant) capState = CAP_DONE;
      }
    }
    SERVO_UNLOCK();
  }
  return true;
}

// ---------------------------------------------------------------- axis mode helpers (loop() only)
// An axis may only be enabled once 'auto dir' has confirmed, on THIS hardware,
// that positive drive gives positive encoder counts. With the direction wrong
// the servo pushes away from its target -- a full-speed runaway until the
// fault trips -- and after the DRV8871 rewiring the saved TB6612-era
// invertMotor values can't be trusted. Persisted in /hwdir.txt.
static bool dirChecked[2] = {false, false};

static void enableAxis(int a) {
  if (!dirChecked[a]) {
    Con.printf("axis %d: motor direction not verified on this hardware -- run 'auto dir %d' (or 'auto dir a') first\n", a, a);
    return;
  }
  SERVO_LOCK();
  axis[a].targetPos = (double)countOf(a);
  axis[a].targetVel = 0;
  axis[a].integral  = 0;
  axis[a].mode = MODE_HOLD;
  faulted[a] = false;
  bbFrozen[a] = false;
  SERVO_UNLOCK();
  motorsEnabled[a] = true;
}
static void disableAxis(int a) {
  motorsEnabled[a] = false;
  testMode[a] = false;
  SERVO_LOCK();
  axis[a].mode = MODE_IDLE;
  axis[a].targetVel = 0;
  SERVO_UNLOCK();
  writeMotorHardware(a, 0);                // DRV8871 coast (no standby pin)
}
static void startTrack(int a, double countsPerSec) {
  if (!motorsEnabled[a] || testMode[a] || autoFullBusy) { Con.println(F("axis disabled or busy, ignoring")); return; }
  SERVO_LOCK();
  axis[a].velCmd = countsPerSec;
  axis[a].mode = MODE_TRACK;
  SERVO_UNLOCK();
}
static void startSlew(int a, double goalCounts) {
  if (!motorsEnabled[a] || testMode[a] || autoFullBusy) { Con.println(F("axis disabled or busy, ignoring")); return; }
  SERVO_LOCK();
  axis[a].goalPos = goalCounts;
  axis[a].mode = MODE_SLEW;
  SERVO_UNLOCK();
}
static void holdHere(int a) {
  if (!motorsEnabled[a] || testMode[a]) return;
  SERVO_LOCK();
  axis[a].targetPos = (double)posSnap[a];
  axis[a].targetVel = 0;
  axis[a].integral  = 0;
  axis[a].mode = MODE_HOLD;
  SERVO_UNLOCK();
}

// ==================================================================
// AUTO-TUNE #1: feedforward (ff), both directions
// ==================================================================
struct FFTune {
  bool active = false;
  int phase = 0;              // 0 = positive direction, 1 = negative direction
  double duty = 0;
  uint32_t stepStartMs = 0;
  int32_t  stepStartCnt = 0;
  double resultPos = 0, resultNeg = 0;
  bool capped = false;        // a direction hit FF_MAX_DUTY without reaching the target rate
  double prevDuty = 0, prevRate = 0;   // last step that did NOT reach the target rate
};
static FFTune fftune[2];
static double lastTuneFF[2]; static bool haveFFResult[2] = {false, false};

static const double FF_TARGET_RATE = SIDEREAL_CPS; // calibrate at the same speed we'll actually track at
static const double FF_STEP_PCT    = 0.5;
static const uint32_t FF_STEP_MS   = 1500;
static const double FF_MAX_DUTY    = 20.0;         // safety cap while probing -- well above the ~7% baseline

static void startFFTune(int a) {
  if (!motorsEnabled[a]) { Con.println(F("enable axis first ('e <axis>')")); return; }
  if (testMode[a]) { Con.println(F("axis busy (a test is running or just finishing)")); return; }
  fftune[a] = FFTune();
  fftune[a].active = true;
  haveFFResult[a] = false;
  fftune[a].stepStartMs = millis();
  fftune[a].stepStartCnt = countOf(a);
  testMode[a] = true;
  writeMotorHardware(a, 0);
  Con.printf("axis %d: ff auto-tune started, positive direction (keep clear, motor will move)\n", a);
}

static void updateFFTune(int a) {
  FFTune &f = fftune[a];
  if (!f.active) return;
  if (!testMode[a]) { f.active = false; return; }  // axis was disabled ('d'/'x') mid-test
  uint32_t now = millis();
  if (now - f.stepStartMs < FF_STEP_MS) return;

  int32_t cnt = countOf(a);
  double dtS = (now - f.stepStartMs) / 1000.0;
  double rate = (cnt - f.stepStartCnt) / dtS;
  double dir = (f.phase == 0) ? 1.0 : -1.0;

  if (fabs(rate) >= FF_TARGET_RATE || f.duty >= FF_MAX_DUTY) {
    // Interpolate between the last step that didn't reach the target rate and
    // this one. (The old through-zero scaling, duty * target/rate, assumed no
    // friction: with the LX90's ~5-6 % breakaway step it gave 3.2 % one way
    // and 0.9 % the other for what is really ~5.8 % both ways.)
    double resultDuty = f.duty;
    if (fabs(rate) > f.prevRate + 1e-6 && fabs(rate) >= FF_TARGET_RATE)
      resultDuty = f.prevDuty + (FF_TARGET_RATE - f.prevRate) / (fabs(rate) - f.prevRate) * (f.duty - f.prevDuty);
    if (fabs(rate) < FF_TARGET_RATE) {
      f.capped = true;
      Con.printf("axis %d: WARNING hit the %.0f%% safety cap before reaching target rate -- result is an extrapolation\n",
                    a, FF_MAX_DUTY);
    }
    if (f.phase == 0) f.resultPos = resultDuty; else f.resultNeg = resultDuty;
    Con.printf("axis %d: %s direction reached %.2f counts/s at %.2f%% duty (%.2f%% at the target rate, interpolated)\n",
                  a, f.phase == 0 ? "positive" : "negative", rate, f.duty, resultDuty);

    if (f.phase == 0) {
      f.phase = 1;
      f.duty = 0;
      f.prevDuty = f.prevRate = 0;
      writeMotorHardware(a, 0);
      f.stepStartMs = millis();
      f.stepStartCnt = countOf(a);
    } else {
      f.active = false;
      returnToNormalServo(a);
      double avgFF = ((f.resultPos + f.resultNeg) / 2.0) / FF_TARGET_RATE;
      lastTuneFF[a] = avgFF;
      haveFFResult[a] = true;
      double denom = (fabs(f.resultPos) + fabs(f.resultNeg)) / 2.0;
      double asym = denom > 1e-6 ? fabs(f.resultPos - f.resultNeg) / denom * 100.0 : 0;
      Con.printf("axis %d: ff auto-tune done. +dir=%.2f%%, -dir=%.2f%%, asymmetry=%.1f%%\n",
                    a, f.resultPos, f.resultNeg, asym);
      Con.printf("  suggested ff = %.4f %%/(count/s).  Apply with 'auto apply %d ff'.\n", avgFF, a);
      if (asym > 15.0)
        Con.println(F("  NOTE: >15% asymmetry between directions -- worth a look (preload, wiring, binding)."));
    }
    return;
  }
  f.prevDuty = f.duty;
  f.prevRate = fabs(rate);
  f.duty += FF_STEP_PCT;
  writeMotorHardware(a, f.duty * dir);
  f.stepStartMs = now;
  f.stepStartCnt = cnt;
}

// ==================================================================
// AUTO-TUNE #2: relay (Astrom-Hagglund) feedback tuning for kp/ki
// ==================================================================
struct RelayTune {
  bool active = false;
  double d = 0, h = 0;
  int32_t startCnt = 0;
  double targetOffset = 20.0;   // small fixed setpoint offset, counts -- harmless movement
  bool relayHigh = true;
  static const int NSW = 16;
  double switchT[NSW]; double peakVal[NSW]; int nSwitch = 0;
  double runningExtreme = -1e9;
  uint32_t startMs = 0;
  double maxAbsErr = 0;
};
static RelayTune relay[2];
static const double RELAY_DEFAULT_DUTY = 15.0;  // %
static const double RELAY_DEFAULT_H    = 1.5;   // counts
static double lastKpTL[2], lastKiTL[2], lastKpZN[2], lastKiZN[2];
// Applied to relay-tuned moving gains before use. The relay test runs near
// standstill, where friction damps the motor; at slew speed the same gains made
// the az servo oscillate at ~28 Hz (+-13 % duty, measured with 'cap', 2026-11)
// and the current swings collapsed the drive into following-error faults.
// Halving kp and ki removed the oscillation completely (duty +-2.7 %).
static const double PI_DERATE = 0.5;
static bool havePIResult[2] = {false, false};

static void startRelayTune(int a, double duty, double hysteresis) {
  if (!motorsEnabled[a]) { Con.println(F("enable axis first ('e <axis>')")); return; }
  if (testMode[a]) { Con.println(F("axis busy (a test is running or just finishing)")); return; }
  relay[a] = RelayTune();
  relay[a].active = true;
  havePIResult[a] = false;
  relay[a].d = duty;
  relay[a].h = hysteresis;
  relay[a].startCnt = countOf(a);
  relay[a].relayHigh = true;
  relay[a].runningExtreme = -1e9;
  relay[a].startMs = millis();
  testMode[a] = true;
  writeMotorHardware(a, relay[a].d);
  Con.printf("axis %d: relay auto-tune started (d=%.1f%%, h=%.1f counts, keep clear)\n", a, duty, hysteresis);
}

static void updateRelayTune(int a) {
  RelayTune &r = relay[a];
  if (!r.active) return;
  if (!testMode[a]) { r.active = false; return; }  // axis was disabled ('d'/'x') mid-test
  int32_t pos = countOf(a);
  double err = (double)(r.startCnt) + r.targetOffset - pos;
  double t = (millis() - r.startMs) / 1000.0;
  if (fabs(err) > r.maxAbsErr) r.maxAbsErr = fabs(err);

  if (t > 15.0 || fabs(err) > 150.0) {
    r.active = false;
    returnToNormalServo(a);
    Con.printf("axis %d: relay auto-tune ABORTED (t=%.1fs, err=%.1f counts) -- check enable/wiring/direction\n",
                  a, t, err);
    return;
  }

  bool flip = (r.relayHigh && err < -r.h) || (!r.relayHigh && err > r.h);
  if (flip) {
    if (r.nSwitch < RelayTune::NSW) { r.switchT[r.nSwitch] = t; r.peakVal[r.nSwitch] = r.runningExtreme; r.nSwitch++; }
    r.relayHigh = !r.relayHigh;
    r.runningExtreme = r.relayHigh ? -1e9 : 1e9;
    writeMotorHardware(a, r.relayHigh ? r.d : -r.d);
  } else {
    if (r.relayHigh) { if (err > r.runningExtreme) r.runningExtreme = err; }
    else              { if (err < r.runningExtreme) r.runningExtreme = err; }
  }

  if (r.nSwitch >= 12) {
    double periodSum = 0; int periodCount = 0;
    double ampSum = 0; int ampCount = 0;
    for (int i = r.nSwitch - 6; i < r.nSwitch - 1; i++) {
      if (i - 2 >= 0) { periodSum += r.switchT[i] - r.switchT[i - 2]; periodCount++; }
      ampSum += fabs(r.peakVal[i]); ampCount++;
    }
    double Tu = periodSum / periodCount;
    double Aamp = ampSum / ampCount;
    double Ku = (4.0 * r.d) / (M_PI * Aamp);
    double kp_tl = Ku / 3.2,   ki_tl = Ku / (7.04 * Tu);   // Tyreus-Luyben: gentler, minimal overshoot
    double kp_zn = 0.45 * Ku,  ki_zn = 0.54 * Ku / Tu;     // Ziegler-Nichols: more aggressive

    r.active = false;
    returnToNormalServo(a);
    lastKpTL[a] = kp_tl; lastKiTL[a] = ki_tl;
    lastKpZN[a] = kp_zn; lastKiZN[a] = ki_zn;
    havePIResult[a] = true;

    Con.printf("axis %d: relay auto-tune done. Ku=%.3f Tu=%.3fs maxErr=%.1f counts\n", a, Ku, Tu, r.maxAbsErr);
    Con.printf("  Tyreus-Luyben (gentler):    kp=%.4f ki=%.4f   -- apply with 'auto apply %d tl'\n", kp_tl, ki_tl, a);
    Con.printf("  Ziegler-Nichols (sharper):  kp=%.4f ki=%.4f   -- apply with 'auto apply %d zn'\n", kp_zn, ki_zn, a);
    Con.printf("  These become the MOVING (track/slew) gains only, x %.2f when applied -- hold gains are untouched on purpose.\n",
               PI_DERATE);
  }
}

// ==================================================================
// AUTO-TUNE #3: open-loop speed step, to suggest mv/ma
// ==================================================================
struct SpeedTune {
  bool active = false;
  uint32_t startMs = 0;
  int32_t  startCnt = 0;
  static const int NSAMP = 24;
  double sampT[NSAMP]; int32_t sampC[NSAMP]; int nSamp = 0;
  uint32_t lastSampMs = 0;
};
static SpeedTune speedtune[2];
static double lastSuggestMaxVel[2], lastSuggestMaxAccel[2];
static double lastPlateau[2], lastPlateauDuty[2];   // speed test: counts/s reached at that duty
static bool haveSpeedResult[2] = {false, false};

static const uint32_t SPEED_SAMPLE_PERIOD_MS = 100;
static const uint32_t SPEED_TOTAL_MS = 2400; // NSAMP * SPEED_SAMPLE_PERIOD_MS

static void startSpeedTune(int a) {
  if (!motorsEnabled[a]) { Con.println(F("enable axis first ('e <axis>')")); return; }
  if (testMode[a]) { Con.println(F("axis busy (a test is running or just finishing)")); return; }
  speedtune[a] = SpeedTune();
  speedtune[a].active = true;
  haveSpeedResult[a] = false;
  speedtune[a].startMs = millis();
  speedtune[a].startCnt = countOf(a);
  speedtune[a].lastSampMs = speedtune[a].startMs;
  testMode[a] = true;
  writeMotorHardware(a, axis[a].maxDuty);
  Con.printf("axis %d: speed auto-tune started, stepping to md=%.1f%% for %.1fs (keep clear)\n",
                a, axis[a].maxDuty, SPEED_TOTAL_MS / 1000.0);
}

static void updateSpeedTune(int a) {
  SpeedTune &s = speedtune[a];
  if (!s.active) return;
  if (!testMode[a]) { s.active = false; return; }  // axis was disabled ('d'/'x') mid-test
  uint32_t now = millis();
  if (now - s.lastSampMs >= SPEED_SAMPLE_PERIOD_MS && s.nSamp < SpeedTune::NSAMP) {
    s.sampT[s.nSamp] = (now - s.startMs) / 1000.0;
    s.sampC[s.nSamp] = countOf(a) - s.startCnt;
    s.nSamp++;
    s.lastSampMs = now;
  }
  if (now - s.startMs >= SPEED_TOTAL_MS || s.nSamp >= SpeedTune::NSAMP) {
    s.active = false;
    returnToNormalServo(a);

    if (s.nSamp < 6) { Con.println(F("speed auto-tune: not enough samples, aborted")); return; }
    int n = s.nSamp;
    double plateauRate = (s.sampC[n - 1] - s.sampC[n - 4]) / (s.sampT[n - 1] - s.sampT[n - 4]);
    double riseTime = s.sampT[n - 1];
    for (int i = 1; i < n; i++) {
      double instRate = (s.sampC[i] - s.sampC[i - 1]) / (s.sampT[i] - s.sampT[i - 1]);
      if (instRate >= 0.9 * fabs(plateauRate)) { riseTime = s.sampT[i]; break; }
    }
    double suggestedMaxVel = 0.7 * fabs(plateauRate);
    double suggestedMaxAccel = (riseTime > 0.02) ? (0.9 * fabs(plateauRate) / riseTime) * 0.5 : 100.0;

    lastSuggestMaxVel[a] = suggestedMaxVel;
    lastSuggestMaxAccel[a] = suggestedMaxAccel;
    lastPlateau[a] = fabs(plateauRate);
    lastPlateauDuty[a] = axis[a].maxDuty;
    haveSpeedResult[a] = true;

    Con.printf("axis %d: speed auto-tune done. plateau=%.2f counts/s at md=%.1f%%, rise-to-90%%=%.2fs\n",
                  a, plateauRate, axis[a].maxDuty, riseTime);
    Con.printf("  suggested mv=%.2f counts/s, ma=%.2f counts/s^2  -- apply with 'auto apply %d speed'\n",
                  suggestedMaxVel, suggestedMaxAccel, a);
    Con.println(F("  This is one open-loop step, not a rigorous ID -- treat as a starting point."));
  }
}

// ==================================================================
// AUTO DIR: which way does positive drive turn the encoder? (sets invertMotor)
// ==================================================================
// Open loop and tiny: ramps the duty up from DIR_START_DUTY until the encoder
// has moved DIR_MOVE_COUNTS (~0.03 deg at the axis), then brakes. Works with the
// axis disabled -- it takes the axis over like the other tests (testMode), so
// it is the safe first thing to run after any wiring change.
struct DirTest {
  bool active = false;
  int32_t startCnt = 0;
  double duty = 0;
  uint32_t stepMs = 0;
};
static DirTest dirtest[2];
static const double   DIR_START_DUTY  = 3.0, DIR_STEP_DUTY = 1.0, DIR_MAX_DUTY = 20.0;
static const uint32_t DIR_STEP_MS     = 100;
static const int32_t  DIR_MOVE_COUNTS = 300;
static const char HWDIR_PATH[] = "/hwdir.txt";
static bool saveCalib();
static bool writeFileAtomic(const char *path, const char *data, size_t len);

static void saveDirChecked() {
  char buf[48];
  int len = snprintf(buf, sizeof buf, "drv8871 dir0=%d dir1=%d\n", dirChecked[0] ? 1 : 0, dirChecked[1] ? 1 : 0);
  writeFileAtomic(HWDIR_PATH, buf, len);
}
static void loadDirChecked() {
  dirChecked[0] = dirChecked[1] = false;
  if (!fsReady || !LittleFS.exists(HWDIR_PATH)) return;
  File f = LittleFS.open(HWDIR_PATH, "r");
  if (!f) return;
  char buf[48];
  size_t n = f.read((uint8_t *)buf, sizeof buf - 1);
  f.close();
  buf[n] = 0;
  dirChecked[0] = strstr(buf, "dir0=1") != nullptr;
  dirChecked[1] = strstr(buf, "dir1=1") != nullptr;
}

static void startDirTest(int a) {
  if (testMode[a]) { Con.printf("axis %d busy (a test is running or just finishing)\n", a); return; }
  dirtest[a] = DirTest();
  dirtest[a].active = true;
  dirtest[a].startCnt = countOf(a);
  dirtest[a].duty = DIR_START_DUTY;
  dirtest[a].stepMs = millis();
  testMode[a] = true;
  writeMotorHardware(a, dirtest[a].duty);
  Con.printf("axis %d: direction check -- a small open-loop nudge (keep clear)\n", a);
}

static void updateDirTest(int a) {
  DirTest &d = dirtest[a];
  if (!d.active) return;
  if (!testMode[a]) { d.active = false; return; }   // aborted ('d', 'x', 'auto abort')
  int32_t moved = countOf(a) - d.startCnt;
  if (labs(moved) >= DIR_MOVE_COUNTS) {
    d.active = false;
    returnToNormalServo(a);                          // brake, then hand back once stopped
    bool flip = moved < 0;
    if (flip) invertMotor[a] = !invertMotor[a];
    dirChecked[a] = true;
    Con.printf("axis %d: +%.0f%% drive moved the encoder %+ld counts -> %s (invertMotor now %d)\n",
               a, d.duty, (long)moved, flip ? "REVERSED, corrected" : "correct", invertMotor[a] ? 1 : 0);
    saveDirChecked();
    saveCalib();                                     // invertMotor lives in the calibration file
    if (dirChecked[0] && dirChecked[1])
      Con.println(F("  both axes verified. Next: 'auto full' to recalibrate for the DRV8871 drivers."));
    return;
  }
  if (millis() - d.stepMs < DIR_STEP_MS) return;
  d.stepMs = millis();
  d.duty += DIR_STEP_DUTY;
  if (d.duty > DIR_MAX_DUTY) {
    d.active = false;
    returnToNormalServo(a);
    Con.printf("axis %d: direction check FAILED -- no encoder movement up to %.0f%% duty (moved %ld counts).\n"
               "  Check the motor wiring/supply, the encoder wiring (GP%u/GP%u) and that the clutch is engaged.\n",
               a, DIR_MAX_DUTY, (long)moved, ENC_PIN[a], ENC_PIN[a] + 1);
    return;
  }
  writeMotorHardware(a, d.duty);
}

// ==================================================================
// FEEDFORWARD FIT: duty = sign(v) * offset + v * gain, through two measured points
// ==================================================================
// (sidereal rate, duty) from 'auto ff' and (plateau speed, duty) from 'auto speed'.
// A line through zero fitted only at sidereal (the old model) is fine for
// tracking but asks for ~200x too much duty at slew speed on the LX90, whose
// ~3.5 % at sidereal is nearly all friction.
static bool fitFF(double sidDuty, double plateau, double plateauDuty, double &gain, double &offset) {
  gain = sidDuty / SIDEREAL_CPS;
  offset = 0;
  if (plateau > 4 * SIDEREAL_CPS && plateauDuty > sidDuty) {
    double g = (plateauDuty - sidDuty) / (plateau - SIDEREAL_CPS);
    double o = sidDuty - g * SIDEREAL_CPS;
    if (g > 0 && o >= 0) { gain = g; offset = o; return true; }
  }
  return false;
}
static double sidDutyNow(int a) { return axis[a].ffOffset + axis[a].ffGain * SIDEREAL_CPS; }
// the high-speed point implied by the current calibration (only if it already has an offset)
static bool currentHighPoint(int a, double &plateau, double &duty) {
  if (axis[a].ffOffset <= 0 || axis[a].ffGain <= 0) return false;
  duty = axis[a].maxDuty;
  plateau = (duty - axis[a].ffOffset) / axis[a].ffGain;
  return true;
}
static void applyFF(int a, double sidDuty, double plateau, double plateauDuty) {
  double g, o;
  bool two = fitFF(sidDuty, plateau, plateauDuty, g, o);
  SERVO_LOCK(); axis[a].ffGain = g; axis[a].ffOffset = o; SERVO_UNLOCK();
  if (two) Con.printf("axis %d: feedforward = %.3f%% + %.6f %%/(count/s)  (%.2f%% at sidereal, %.1f%% at %.0f counts/s)\n",
                      a, o, g, sidDuty, plateauDuty, plateau);
  else     Con.printf("axis %d: feedforward = %.4f %%/(count/s) through zero (no usable speed-test point)\n", a, g);
}

// ==================================================================
// CALIBRATION STORAGE (LittleFS text file, format in calib_config.h)
// ==================================================================
static const char CALIB_PATH[]     = "/calib.txt";
static const char CALIB_TMP_PATH[] = "/calib.tmp";
static const size_t CALIB_MAX_BYTES = 1024;
static char calibBuf[CALIB_MAX_BYTES];  // static, not on the loop() stack

static AxisCalib calibFromAxis(int a) {
  AxisCalib c;
  SERVO_LOCK();
  c.ffGain = axis[a].ffGain;   c.ffOffset = axis[a].ffOffset;   c.kp = axis[a].kp;   c.ki = axis[a].ki;   c.maxIntegral = axis[a].maxIntegral;
  c.kpHold = axis[a].kpHold;   c.kiHold = axis[a].kiHold;   c.maxIntegralHold = axis[a].maxIntegralHold;
  c.maxDuty = axis[a].maxDuty; c.maxVel = axis[a].maxVel;   c.maxAccel = axis[a].maxAccel;
  SERVO_UNLOCK();
  c.faultThreshold = faultThreshold[a];
  c.invertMotor    = invertMotor[a];
  return c;
}

// Only call with the axis disabled if invertMotor might change (boot, 'cfg load').
static void calibToAxis(int a, const AxisCalib &c) {
  SERVO_LOCK();
  axis[a].ffGain = c.ffGain;   axis[a].ffOffset = c.ffOffset;   axis[a].kp = c.kp;   axis[a].ki = c.ki;   axis[a].maxIntegral = c.maxIntegral;
  axis[a].kpHold = c.kpHold;   axis[a].kiHold = c.kiHold;   axis[a].maxIntegralHold = c.maxIntegralHold;
  axis[a].maxDuty = c.maxDuty; axis[a].maxVel = c.maxVel;   axis[a].maxAccel = c.maxAccel;
  SERVO_UNLOCK();
  faultThreshold[a] = c.faultThreshold;
  invertMotor[a]    = c.invertMotor;
}

// Write to a temp file then rename over the real one, so a power cut mid-save
// leaves the previous calibration intact. Note: flash erase/program briefly
// pauses interrupts (the core does this), so the servo ISR stalls for a few ms --
// harmless while holding, which is why 'cfg save' refuses while moving.
static bool saveCalib() {
  if (!fsReady) { Con.println(F("cfg: no filesystem -- set Tools > Flash Size to an option with FS")); return false; }
  AxisCalib c[2] = {calibFromAxis(0), calibFromAxis(1)};
  int len = formatCalib(calibBuf, sizeof(calibBuf), c);
  if (len < 0) { Con.println(F("cfg: internal error, calibration text too long")); return false; }

  File f = LittleFS.open(CALIB_TMP_PATH, "w");
  if (!f) { Con.println(F("cfg: could not create file")); return false; }
  size_t wrote = f.write((const uint8_t *)calibBuf, (size_t)len);
  f.close();
  if (wrote != (size_t)len) {
    LittleFS.remove(CALIB_TMP_PATH);
    Con.println(F("cfg: write failed (filesystem full?) -- previous file kept"));
    return false;
  }
  if (!LittleFS.rename(CALIB_TMP_PATH, CALIB_PATH)) {
    LittleFS.remove(CALIB_PATH);
    if (!LittleFS.rename(CALIB_TMP_PATH, CALIB_PATH)) { Con.println(F("cfg: rename failed")); return false; }
  }
  Con.printf("cfg: calibration saved to flash (%s, %d bytes)\n", CALIB_PATH, len);
  return true;
}

// Reads the file into calibBuf, NUL-terminated. Returns false (with a message
// unless quietIfMissing and the file simply doesn't exist yet).
static bool readCalibFile(bool quietIfMissing) {
  if (!fsReady) { Con.println(F("cfg: no filesystem -- set Tools > Flash Size to an option with FS")); return false; }
  if (!LittleFS.exists(CALIB_PATH)) {
    if (!quietIfMissing) Con.println(F("cfg: no saved calibration yet ('auto full' or 'cfg save' creates one)"));
    return false;
  }
  File f = LittleFS.open(CALIB_PATH, "r");
  if (!f) { Con.println(F("cfg: could not open saved file")); return false; }
  if (f.size() >= sizeof(calibBuf)) { f.close(); Con.println(F("cfg: saved file too large, ignored")); return false; }
  size_t n = f.read((uint8_t *)calibBuf, sizeof(calibBuf) - 1);
  f.close();
  calibBuf[n] = 0;
  return true;
}

static bool loadCalib(bool quietIfMissing) {
  if (!readCalibFile(quietIfMissing)) return false;
  AxisCalib c[2] = {calibFromAxis(0), calibFromAxis(1)};
  char err[96];
  if (!parseCalib(calibBuf, c, err, sizeof(err))) {
    Con.printf("cfg: saved file rejected (%s) -- keeping current values\n", err);
    return false;
  }
  calibToAxis(0, c[0]);
  calibToAxis(1, c[1]);
  Con.printf("cfg: calibration loaded from flash (%s)\n", CALIB_PATH);
  return true;
}

// ==================================================================
// AUTO FULL: ff + pi + speed on axis 0, then axis 1, apply everything, save
// ==================================================================
// All three tests are open loop (they never use ff/kp/ki/mv/ma), so results are only
// collected along the way and applied together at the very end: an abort
// part-way through leaves the running calibration exactly as it was.
struct FullTune {
  bool useZN = false;
  double relayDuty = RELAY_DEFAULT_DUTY, relayH = RELAY_DEFAULT_H;
  int step = 0;             // axis = step / 3; test = step % 3 (0 = ff, 1 = pi, 2 = speed)
  bool running = false;     // false = settling in HOLD before starting 'step'
  uint32_t settleStartMs = 0;
};
static FullTune fulltune;
static const uint32_t FULL_SETTLE_MS = 1500;  // let the axis come to rest between tests
static const int FULL_TESTS_PER_AXIS = 3;
static const int FULL_NSTEPS = 2 * FULL_TESTS_PER_AXIS;
enum { FULL_FF, FULL_PI, FULL_SPEED };
static const char *const FULL_TEST_NAME[] = {"feedforward tune", "relay pi tune", "speed step test"};

static void cancelFullTune(const char *why) {
  if (!autoFullBusy) return;
  autoFullBusy = false;
  int st = fulltune.step;
  if (fulltune.running || st == 0)
    Con.printf("auto full: failed during step %d/%d (axis %d %s)\n", st + 1, FULL_NSTEPS,
                  st / FULL_TESTS_PER_AXIS, FULL_TEST_NAME[st % FULL_TESTS_PER_AXIS]);
  else
    Con.printf("auto full: failed while holding/settling after step %d/%d (axis %d %s)\n", st, FULL_NSTEPS,
                  (st - 1) / FULL_TESTS_PER_AXIS, FULL_TEST_NAME[(st - 1) % FULL_TESTS_PER_AXIS]);
  for (int a = 0; a < 2; a++) {
    fftune[a].active = false;
    relay[a].active = false;
    speedtune[a].active = false;
    if (testMode[a]) returnToNormalServo(a);
  }
  Con.printf("auto full: ABORTED (%s). Calibration unchanged, nothing saved.\n", why);
}

static void startFullTune(bool useZN, double relayDuty, double relayH) {
  if (autoFullBusy) { Con.println(F("auto full already running ('auto abort a' to stop)")); return; }
  if (testMode[0] || testMode[1]) { Con.println(F("another auto-tune is running -- 'auto abort a' first")); return; }
  fulltune = FullTune();
  fulltune.useZN = useZN;
  fulltune.relayDuty = relayDuty;
  fulltune.relayH = relayH;
  fulltune.settleStartMs = millis();
  enableAxis(0);   // (re)enable both: clears any fault, holds at the current position
  enableAxis(1);
  if (!motorsEnabled[0] || !motorsEnabled[1]) { Con.println(F("auto full: not started")); return; }
  autoFullBusy = true;
  Con.printf("auto full: started -- ff + pi (%s, d=%.1f%%, h=%.1f) + speed on axis 0 then axis 1, then save.\n",
                useZN ? "Ziegler-Nichols" : "Tyreus-Luyben", relayDuty, relayH);
  Con.println(F("  Both axes will move -- the speed test runs each axis at md for ~2.4s. Keep clear.\n"
                   "  Takes a couple of minutes. 'x' or 'auto abort a' stops it."));
}

static void updateFullTune() {
  if (!autoFullBusy) return;
  FullTune &F = fulltune;
  int a = F.step / FULL_TESTS_PER_AXIS;
  int test = F.step % FULL_TESTS_PER_AXIS;

  for (int b = 0; b < 2; b++) {
    if (!motorsEnabled[b]) { cancelFullTune("axis disabled"); return; }
    if (faulted[b])        { cancelFullTune("following-error fault"); return; }
  }

  if (!F.running) {
    if (millis() - F.settleStartMs < FULL_SETTLE_MS) return;
    Con.printf("auto full: step %d/%d -- axis %d %s\n", F.step + 1, FULL_NSTEPS, a, FULL_TEST_NAME[test]);
    if (test == FULL_FF)      startFFTune(a);
    else if (test == FULL_PI) startRelayTune(a, F.relayDuty, F.relayH);
    else                      startSpeedTune(a);
    F.running = true;
    return;
  }

  bool stillRunning = test == FULL_FF ? fftune[a].active : test == FULL_PI ? relay[a].active : speedtune[a].active;
  if (stillRunning || testMode[a]) return;  // test running, or axis still braking to a stop

  // that test just finished (or aborted) -- is its result usable?
  if (test == FULL_FF) {
    if (!haveFFResult[a] || fftune[a].capped || !isfinite(lastTuneFF[a]) || lastTuneFF[a] <= 0) {
      cancelFullTune("feedforward tune failed or hit the duty cap"); return;
    }
  } else if (test == FULL_PI) {
    double kp = F.useZN ? lastKpZN[a] : lastKpTL[a];
    double ki = F.useZN ? lastKiZN[a] : lastKiTL[a];
    if (!havePIResult[a] || !isfinite(kp) || !isfinite(ki) || kp <= 0 || ki <= 0) {
      cancelFullTune("relay pi tune failed"); return;
    }
  } else {
    // mv must at least allow sidereal tracking speed, or slews would be slower than the sky
    if (!haveSpeedResult[a] || !isfinite(lastSuggestMaxVel[a]) || !isfinite(lastSuggestMaxAccel[a]) ||
        lastSuggestMaxVel[a] <= SIDEREAL_CPS || lastSuggestMaxAccel[a] <= 0) {
      cancelFullTune("speed test failed (axis barely moved?)"); return;
    }
  }

  F.step++;
  F.running = false;
  F.settleStartMs = millis();
  if (F.step < FULL_NSTEPS) return;

  // all done: apply together, then save
  autoFullBusy = false;
  for (int b = 0; b < 2; b++) {
    applyFF(b, lastTuneFF[b] * SIDEREAL_CPS, lastPlateau[b], lastPlateauDuty[b]);
    SERVO_LOCK();
    axis[b].kp = PI_DERATE * (F.useZN ? lastKpZN[b] : lastKpTL[b]);
    axis[b].ki = PI_DERATE * (F.useZN ? lastKiZN[b] : lastKiTL[b]);
    axis[b].maxVel   = lastSuggestMaxVel[b];
    axis[b].maxAccel = lastSuggestMaxAccel[b];
    SERVO_UNLOCK();
  }
  Con.printf("auto full: DONE, applied to both axes (moving gains = relay result x %.2f safety factor):\n", PI_DERATE);
  for (int b = 0; b < 2; b++)
    Con.printf("  axis %d: ff=%.3f%%+%.6f/(c/s)  moving kp=%.4f ki=%.4f (%s)  mv=%.2f ma=%.2f\n",
                  b, axis[b].ffOffset, axis[b].ffGain, axis[b].kp, axis[b].ki, F.useZN ? "ZN" : "TL",
                  axis[b].maxVel, axis[b].maxAccel);
  if (!saveCalib()) Con.println(F("auto full: results are live but NOT saved -- fix the filesystem and 'cfg save'"));
}

// ==================================================================
// SMALL FILES (sky settings, park state, WiFi credentials)
// ==================================================================
static char fileBuf[2048];

// write to <path>.tmp then rename over <path>: a power cut mid-save keeps the old file
static bool writeFileAtomic(const char *path, const char *data, size_t len) {
  if (!fsReady) { Con.println(F("fs: no filesystem -- set Tools > Flash Size to an option with FS")); return false; }
  char tmp[32];
  snprintf(tmp, sizeof tmp, "%s.tmp", path);
  File f = LittleFS.open(tmp, "w");
  if (!f) { Con.printf("fs: could not create %s\n", tmp); return false; }
  size_t w = f.write((const uint8_t *)data, len);
  f.close();
  if (w != len) { LittleFS.remove(tmp); Con.printf("fs: write to %s failed\n", path); return false; }
  if (!LittleFS.rename(tmp, path)) {
    LittleFS.remove(path);
    if (!LittleFS.rename(tmp, path)) { Con.printf("fs: rename to %s failed\n", path); return false; }
  }
  return true;
}

// whole (small) file into fileBuf, NUL-terminated; false if missing or too big
static bool readFileToBuf(const char *path) {
  if (!fsReady || !LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  if (f.size() >= sizeof(fileBuf)) { f.close(); return false; }
  size_t n = f.read((uint8_t *)fileBuf, sizeof(fileBuf) - 1);
  f.close();
  fileBuf[n] = 0;
  return true;
}

// ==================================================================
// TIME (NTP when online; 'time' console command or LX200 :SC/:SL otherwise)
// ==================================================================
static const char *timeSource = "not set";
static bool timeWasValid = false;

static double nowUnix() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (double)tv.tv_sec + tv.tv_usec * 1e-6;
}
static bool timeValid() { return nowUnix() > 1.7e9; }   // anything before late 2023 = never set
static void setClockUtc(double u, const char *src) {
  struct timeval tv;
  tv.tv_sec  = (time_t)floor(u);
  tv.tv_usec = (suseconds_t)((u - floor(u)) * 1e6);
  settimeofday(&tv, nullptr);
  timeSource = src;
  timeWasValid = true;
}
static void printUtc(double u) {
  long y; unsigned mo, d; int hh, mi; double ss;
  coord::civilFromUnix(u, y, mo, d, hh, mi, ss);
  Con.printf("%04ld-%02u-%02u %02d:%02d:%04.1f UTC", y, mo, d, hh, mi, ss);
}

// ==================================================================
// SKY: site, mount settings, alignment model, goto / tracking / moves / park
// ==================================================================
static const char SKY_PATH[]  = "/sky.txt";
static const char PARK_PATH[] = "/park.txt";
static SkySettings       sky;
static coord::AlignModel model;

enum SkyState { SKY_IDLE, SKY_GOTO, SKY_TRACK, SKY_PARK };
static SkyState skyState = SKY_IDLE;
static bool   trackingOn = true;             // track the target after a goto or sync
static double trackRa = 0, trackDec = 0;     // JNow radians: what we're slewing to / tracking
static int    gotoPass = 0;
static bool   gotoAborting = false;
static bool   abortToIdle = false;           // console 'stop': don't resume tracking after the slew stops
static double moveCmd[2] = {0, 0}, moveVel[2] = {0, 0};   // manual move: commanded / ramped counts/s
static bool   wasMoving[2] = {false, false};
static int    moveSettle[2] = {0, 0};        // sky ticks to wait after a move for the servo ramp to finish
static char   moveRate = 'C';
static bool   jogTimed = false;
static uint32_t jogStopAtMs = 0;
static uint32_t lastSkyMs = 0, lastSpeedWarnMs = 0;

static const double RAW0_LIMIT          = 1.5 * coord::kPi; // primary axis stays within +-270 deg of home (cable wrap)
static const double SLOW_AXIS_CPS       = 0.2;   // slower than this, an axis holds and steps instead of TRACKing:
                                                 // TRACK gains hunt against stiction on a near-static target (servo_core.h)
static const double TRACK_TAU_S         = 2.0;   // how fast the servo target is pulled onto the model position
static const double TRACK_REGOTO_COUNTS = 50;    // further off than this while tracking: correct with a slew
static const int    GOTO_MAX_PASSES     = 4;

static double siteLat() { return sky.latDeg * coord::kDeg; }
static double siteLon() { return sky.lonDeg * coord::kDeg; }
static bool   skyReady() { return sky.siteSet && timeValid(); }
static double lstNow()   { return coord::lst(nowUnix(), siteLon()); }
static int    axisSign(int a) { return a == 0 ? sky.sign0 : sky.sign1; }
static double axisRev(int a)     { return a == 0 ? sky.rev0 : sky.rev1; }   // encoder counts per axis turn
static double radPerCount(int a) { return coord::k2Pi / axisRev(a); }
static double countsToRaw(int a, double c) { return axisSign(a) * c * radPerCount(a); }
static double rawToCounts(int a, double r) { return r / (axisSign(a) * radPerCount(a)); }
static bool   axesUsable() {
  return motorsEnabled[0] && motorsEnabled[1] && !faulted[0] && !faulted[1] &&
         !testMode[0] && !testMode[1] && !autoFullBusy;
}

static void saveSky() {
  int len = formatSky(fileBuf, sizeof fileBuf, sky);
  if (len > 0 && writeFileAtomic(SKY_PATH, fileBuf, len)) Con.println(F("sky: settings saved"));
}
static void loadSky() {
  if (!readFileToBuf(SKY_PATH)) return;
  char err[96];
  SkySettings s = sky;
  if (parseSky(fileBuf, s, err, sizeof err)) sky = s;
  else Con.printf("sky: %s rejected (%s) -- using defaults\n", SKY_PATH, err);
}
static void resetModel(const char *why) {
  coord::modelReset(model, sky.mode, siteLat());
  if (why) Con.printf("align: alignment cleared (%s)\n", why);
}

// pointing for given encoder counts / for where the mount is now
static bool pointingAt(double c0, double c1, double &ra, double &dec) {
  if (!skyReady()) return false;
  double ha;
  coord::rawToHaDec(model, countsToRaw(0, c0), countsToRaw(1, c1), ha, dec);
  ra = coord::wrap2Pi(lstNow() - ha);
  return true;
}
static bool pointingNow(double &ra, double &dec) { return pointingAt(posSnap[0], posSnap[1], ra, dec); }

// encoder counts that put (ra, dec) in the eyepiece at unix time t
static void targetCounts(double ra, double dec, double t, double &c0, double &c1) {
  double ha = coord::wrapPi(coord::lst(t, siteLon()) - ra);
  double r0, r1;
  coord::haDecToRaw(model, ha, dec, countsToRaw(0, posSnap[0]), RAW0_LIMIT, r0, r1);
  c0 = rawToCounts(0, r0);
  c1 = rawToCounts(1, r1);
}
static double altitudeOf(double ra, double dec, double t) {
  double alt, az;
  coord::haDecToAltAz(coord::wrapPi(coord::lst(t, siteLon()) - ra), dec, siteLat(), alt, az);
  return alt;
}

static void setSite(double latDeg, double lonDeg, const char *src) {
  bool changed = !sky.siteSet || fabs(latDeg - sky.latDeg) > 0.01 || fabs(lonDeg - sky.lonDeg) > 0.01;
  sky.latDeg = latDeg; sky.lonDeg = lonDeg; sky.siteSet = true;
  if (!changed) return;
  Con.printf("site: lat %.4f lon %.4f (from %s)\n", latDeg, lonDeg, src);
  resetModel("site changed");
  saveSky();
}

// enable both axes for a sky command if they're simply off (not faulted, not testing)
static bool ensureAxesForSky(char *msg, size_t n) {
  if (autoFullBusy || testMode[0] || testMode[1]) { snprintf(msg, n, "Auto-tune running"); return false; }
  if (faulted[0] || faulted[1]) {
    static uint32_t lastAutoClearMs = 0;
    if (lastAutoClearMs && millis() - lastAutoClearMs < 10000) {
      snprintf(msg, n, "Axis faulted again - check the mount");
      return false;
    }
    lastAutoClearMs = millis();
    Con.printf("sky: clearing a following-error fault on axis %s to carry on (it trips again if really stuck)\n",
               faulted[0] && faulted[1] ? "0 and 1" : faulted[0] ? "0" : "1");
    for (int a = 0; a < 2; a++) if (faulted[a]) enableAxis(a);
  }
  if (!motorsEnabled[0] || !motorsEnabled[1]) {
    if (!dirChecked[0] || !dirChecked[1]) { snprintf(msg, n, "Run auto dir a first"); return false; }
    enableAxis(0); enableAxis(1);
    Con.println(F("sky: motors enabled"));
  }
  return true;
}

// slew-time estimate for the trapezoid profile in servo_core.h
static double slewTime(int a, double dist) {
  double v = axis[a].maxVel, acc = axis[a].maxAccel;
  dist = fabs(dist);
  if (dist < v * v / acc) return 2.0 * sqrt(dist / acc);
  return dist / v + v / acc;
}

// slew both axes to where the target will be when we get there
static void startGotoPass() {
  double t = nowUnix(), c0 = posSnap[0], c1 = posSnap[1], T = 0;
  for (int it = 0; it < 3; it++) {
    targetCounts(trackRa, trackDec, t + T, c0, c1);
    T = fmax(slewTime(0, c0 - posSnap[0]), slewTime(1, c1 - posSnap[1])) + 0.5;
  }
  startSlew(0, c0);
  startSlew(1, c1);
  gotoPass++;
}

// decelerate a running slew along its profile instead of stopping dead
// (stopping dead leaves the gentle hold gains chasing a coasting motor)
static void smoothStop(int a) {
  SERVO_LOCK();
  if (axis[a].mode == MODE_SLEW) {
    double v = axis[a].targetVel;
    axis[a].goalPos = axis[a].targetPos + v * fabs(v) / (2.0 * axis[a].maxAccel);
  }
  SERVO_UNLOCK();
}

static int skyGoto(double ra, double dec, char *msg, size_t n) {
  if (!skyReady()) { snprintf(msg, n, "Set site and time first"); return 2; }
  if (skyState == SKY_GOTO || skyState == SKY_PARK) { snprintf(msg, n, "Busy slewing"); return 2; }
  double altDeg = altitudeOf(ra, dec, nowUnix()) / coord::kDeg;
  if (altDeg < sky.horizonDeg) { snprintf(msg, n, "Below horizon (%.0f deg)", altDeg); return 1; }
  if (!ensureAxesForSky(msg, n)) return 2;
  moveCmd[0] = moveCmd[1] = moveVel[0] = moveVel[1] = 0;
  wasMoving[0] = wasMoving[1] = false;
  trackRa = ra; trackDec = dec;
  gotoPass = 0;
  gotoAborting = false;
  skyState = SKY_GOTO;
  startGotoPass();
  Con.printf("goto: RA %.4fh Dec %+.3f, alt %.1f deg\n", ra / coord::kHour, dec / coord::kDeg, altDeg);
  return 0;
}

static void beginTrackingHere() {
  double ra, dec;
  if (trackingOn && axesUsable() && pointingNow(ra, dec)) {
    trackRa = ra; trackDec = dec;
    skyState = SKY_TRACK;
  } else {
    skyState = SKY_IDLE;
  }
}

static bool skySync(double ra, double dec, char *msg, size_t n) {
  if (!skyReady()) { snprintf(msg, n, "Set site and time first"); return false; }
  if (skyState == SKY_GOTO || skyState == SKY_PARK) { snprintf(msg, n, "Busy slewing"); return false; }
  double t = nowUnix();
  double r0 = countsToRaw(0, posSnap[0]), r1 = countsToRaw(1, posSnap[1]);
  double ha = coord::wrapPi(coord::lst(t, siteLon()) - ra);
  double oh, od;
  coord::rawToHaDec(model, r0, r1, oh, od);
  double before = coord::angleBetween(coord::haDecToVec(oh, od), coord::haDecToVec(ha, dec));
  coord::SyncReport rep = coord::addSyncStar(model, r0, r1, ha, dec);

  Con.printf("sync: star %d%s -- pointing was off by %.2f deg\n", rep.nStars, rep.replaced ? " (replaced a nearby one)" : "",
             before / coord::kDeg);
  if (rep.nStars >= 2)
    Con.printf("  model fit: rms %.1f arcmin, worst %.1f arcmin; mount axis %.2f deg from ideal %s\n",
               rep.rmsResidual / coord::kDeg * 60, rep.maxResidual / coord::kDeg * 60, rep.axisTilt / coord::kDeg,
               sky.mode == coord::MOUNT_EQ ? "(polar alignment error)" : "(levelling error)");
  else
    Con.println(F("  1 star: offsets only. Sync a 2nd star well away from this one for a full model."));
  if (rep.mirrored)
    Con.println(F("  WARNING: the stars look mirrored -- an axis count direction is reversed. Check 'msign'."));
  if (rep.z1Unsolved)
    Con.println(F("  WARNING: couldn't fit the star separations -- wrong star centred? 'align clear' to start over."));
  if (rep.nStars >= 2 && rep.axisTilt > 10 * coord::kDeg)
    Con.println(F("  WARNING: mount axis >10 deg from ideal -- check setup, 'mount' mode and 'msign' directions."));
  if (rep.nStars >= 2 && before > 5 * coord::kDeg)
    Con.println(F("  NOTE: big correction. If the scope was moved by hand (clutches), 'align clear' and sync again."));

  // keep tracking the synced object from exactly where we are
  if (trackingOn && axesUsable()) { trackRa = ra; trackDec = dec; skyState = SKY_TRACK; }
  return true;
}

static void skyStop() {
  moveCmd[0] = moveCmd[1] = 0;
  jogTimed = false;
  if (skyState == SKY_GOTO || skyState == SKY_PARK) {
    smoothStop(0); smoothStop(1);
    gotoAborting = true;
    Con.println(F("sky: stopping"));
  }
}

// manual moves: direction letter -> axis and count direction
static void moveAxisDir(char d, int &a, int &s) {
  bool southEq = sky.mode == coord::MOUNT_EQ && sky.latDeg < 0;
  if (d == 'n' || d == 's') { a = 1; s = d == 'n' ? 1 : -1; if (southEq) s = -s; }
  else { a = 0; s = d == 'e' ? 1 : -1; if (southEq || sky.mode == coord::MOUNT_ALTAZ) s = -s; }
  s *= axisSign(a);
}
static double moveRateCps(char r, int a) {
  double sid = axisRev(a) / coord::SIDEREAL_DAY_S;   // sidereal rate on this axis, counts/s
  switch (r) {
    case 'G': return 1 * sid;
    case 'C': return 8 * sid;
    case 'M': return 32 * sid;
    default:  return 1e9;                     // 'S' = slew: capped below
  }
}
// The fastest manual move ('S' rate, e.g. Stellarium's top arrow speed) is
// sky.jogMax x mv -- a console setting ('jogmax'), saved in /sky.txt, default 0.5.
static bool startMove(char d, char r, uint32_t ms) {
  char msg[48];
  if (skyState == SKY_GOTO || skyState == SKY_PARK) { Con.println(F("move: busy slewing")); return false; }
  if (!ensureAxesForSky(msg, sizeof msg)) { Con.printf("move: %s\n", msg); return false; }
  int a, s;
  moveAxisDir(d, a, s);
  moveCmd[a] = s * fmin(moveRateCps(r, a), sky.jogMax * axis[a].maxVel);
  jogTimed = ms > 0;
  jogStopAtMs = millis() + ms;
  return true;
}
static void stopMove(char d) { int a, s; moveAxisDir(d, a, s); moveCmd[a] = 0; }

static void savePark(long c0, long c1) {
  int len = formatPark(fileBuf, sizeof fileBuf, model, c0, c1);
  if (len > 0 && writeFileAtomic(PARK_PATH, fileBuf, len))
    Con.println(F("park: alignment saved -- safe to power off. It is restored at the next boot."));
}
static void startPark(bool here) {
  char msg[48];
  if (here) {
    skyState = SKY_IDLE;
    disableAxis(0); disableAxis(1);
    savePark(posSnap[0], posSnap[1]);
    return;
  }
  if (!ensureAxesForSky(msg, sizeof msg)) { Con.printf("park: %s\n", msg); return; }
  moveCmd[0] = moveCmd[1] = moveVel[0] = moveVel[1] = 0;
  startSlew(0, 0);
  startSlew(1, 0);
  gotoAborting = false;
  skyState = SKY_PARK;
  Con.println(F("park: slewing to the home position"));
}

// Boot: restore the alignment saved by 'park'. One-shot: the file is removed,
// so the next power-off needs another park to keep the alignment.
static void restorePark() {
  if (!readFileToBuf(PARK_PATH)) return;
  coord::AlignModel m;
  long c0, c1;
  char err[96];
  bool ok = parsePark(fileBuf, m, c0, c1, err, sizeof err);
  LittleFS.remove(PARK_PATH);
  if (!ok) { Con.printf("park: saved file rejected (%s)\n", err); return; }
  if (m.mode != sky.mode || fabs(m.latitude - siteLat()) > 1e-4) {
    Con.println(F("park: site or mount mode changed since parking -- alignment discarded"));
    return;
  }
  for (int a = 0; a < 2; a++) zeroOffset[a] = readRaw(a) - (int32_t)(a == 0 ? c0 : c1);
  model = m;
  Con.printf("park: alignment restored (%d sync star%s). If the scope was moved while off, 'align clear' and sync again.\n",
             m.nStars, m.nStars == 1 ? "" : "s");
}

// One tracking update: steer each axis's servo target onto the model position.
static void trackStep(double now) {
  double p[2], p1[2];
  targetCounts(trackRa, trackDec, now, p[0], p[1]);
  targetCounts(trackRa, trackDec, now + 1.0, p1[0], p1[1]);
  bool regoto = false;
  for (int a = 0; a < 2; a++) {
    double v = p1[a] - p[a];                       // counts/s needed right now
    if (fabs(v) > axis[a].maxVel) {
      v = v > 0 ? axis[a].maxVel : -axis[a].maxVel;
      if (millis() - lastSpeedWarnMs > 60000) {
        Con.printf("track: axis %d can't keep up (near the zenith in alt-az?)\n", a);
        lastSpeedWarnMs = millis();
      }
    }
    SERVO_LOCK();
    AxisCtrl &x = axis[a];
    double e = p[a] - x.targetPos;
    if (wasMoving[a]) {                            // manual move on top of tracking
      x.velCmd = v + moveVel[a];
      x.mode = MODE_TRACK;
    } else if (fabs(e) > TRACK_REGOTO_COUNTS) {
      regoto = true;
    } else if (fabs(v) < SLOW_AXIS_CPS) {
      if (x.mode != MODE_HOLD || fabs(e) >= 1.0) { x.targetPos = p[a]; x.targetVel = x.velCmd = 0; x.mode = MODE_HOLD; }
    } else {
      double corr = e / TRACK_TAU_S, lim = 0.5 * SIDEREAL_CPS;
      if (corr > lim) corr = lim;
      if (corr < -lim) corr = -lim;
      x.velCmd = v + corr;
      x.mode = MODE_TRACK;
    }
    SERVO_UNLOCK();
  }
  if (regoto) {
    Con.println(F("track: off target, correcting with a slew"));
    gotoPass = 0;
    skyState = SKY_GOTO;
    startGotoPass();
  }
}

// 10 Hz sky state machine (core 0)
static void skyUpdate() {
  uint32_t ms = millis();
  if (ms - lastSkyMs < 100) return;
  double dt = (ms - lastSkyMs) / 1000.0;
  if (dt > 0.5) dt = 0.5;
  lastSkyMs = ms;

  if (jogTimed && (int32_t)(ms - jogStopAtMs) >= 0) { jogTimed = false; moveCmd[0] = moveCmd[1] = 0; }

  if (!axesUsable()) {
    if (skyState != SKY_IDLE || wasMoving[0] || wasMoving[1]) {
      Con.println(F("sky: stopped (motors off, fault or auto-tune)"));
      for (int a = 0; a < 2; a++)
        if (!testMode[a]) { SERVO_LOCK(); axis[a].targetVel = axis[a].velCmd = 0; axis[a].mode = MODE_HOLD; SERVO_UNLOCK(); }
    }
    skyState = SKY_IDLE;
    moveCmd[0] = moveCmd[1] = moveVel[0] = moveVel[1] = 0;
    wasMoving[0] = wasMoving[1] = false;
    return;
  }

  // ramp manual move speeds with the axis acceleration limit
  bool moveEnded[2] = {false, false};
  for (int a = 0; a < 2; a++) {
    double step = axis[a].maxAccel * dt, d = moveCmd[a] - moveVel[a];
    if (d > step) d = step;
    if (d < -step) d = -step;
    moveVel[a] += d;
    if (moveCmd[a] != 0 || moveVel[a] != 0) { wasMoving[a] = true; moveSettle[a] = 20; }
    else if (wasMoving[a]) {
      // wait (max 2 s) until the motor itself has caught up with the target speed:
      // switching to the gentle HOLD gains while it still coasts overshoots and faults
      bool caughtUp = fabs(velSnap[a] - axis[a].targetVel) < 50 && fabs(errSnap[a]) < 50;
      if (caughtUp || --moveSettle[a] <= 0) { wasMoving[a] = false; moveEnded[a] = true; }
    }
  }

  switch (skyState) {
    case SKY_IDLE:
      for (int a = 0; a < 2; a++) {
        if (!wasMoving[a] && !moveEnded[a]) continue;
        SERVO_LOCK();
        if (moveEnded[a]) { axis[a].targetVel = axis[a].velCmd = 0; axis[a].mode = MODE_HOLD; }   // stop where the target is
        else              { axis[a].velCmd = moveVel[a]; axis[a].mode = MODE_TRACK; }
        SERVO_UNLOCK();
      }
      break;

    case SKY_TRACK:
      if (!skyReady()) { skyState = SKY_IDLE; break; }
      if (moveEnded[0] || moveEnded[1]) {           // re-centre tracking on where the move left us
        double c0, c1, ra, dec;
        SERVO_LOCK(); c0 = axis[0].targetPos; c1 = axis[1].targetPos; SERVO_UNLOCK();
        if (pointingAt(c0, c1, ra, dec)) { trackRa = ra; trackDec = dec; }
      }
      trackStep(nowUnix());
      break;

    case SKY_GOTO:
    case SKY_PARK:
      if (axis[0].mode == MODE_SLEW || axis[1].mode == MODE_SLEW) break;    // still slewing
      if (gotoAborting) {
        gotoAborting = false;
        if (abortToIdle) skyState = SKY_IDLE; else beginTrackingHere();
        abortToIdle = false;
        break;
      }
      if (skyState == SKY_PARK) {
        if (abs(posSnap[0]) > 5 || abs(posSnap[1]) > 5) { startSlew(0, 0); startSlew(1, 0); break; }
        skyState = SKY_IDLE;
        disableAxis(0); disableAxis(1);
        savePark(posSnap[0], posSnap[1]);
        break;
      }
      {
        double c0, c1;
        targetCounts(trackRa, trackDec, nowUnix(), c0, c1);
        if ((fabs(c0 - posSnap[0]) > 3 || fabs(c1 - posSnap[1]) > 3) && gotoPass < GOTO_MAX_PASSES) {
          startGotoPass();                         // the sky moved while we slewed: short correction
          break;
        }
      }
      Con.printf("goto: arrived (%d pass%s)\n", gotoPass, gotoPass == 1 ? "" : "es");
      if (trackingOn) skyState = SKY_TRACK;
      else skyState = SKY_IDLE;
      break;
  }
}

// ==================================================================
// WIFI, NTP, mDNS, LX200 server (port 4030), telnet console (port 23)
// ==================================================================
static const char WIFI_PATH[] = "/wifi.txt";
static const char HOSTNAME[]  = "telescope";
static const uint16_t LX200_PORT = 4030, TELNET_PORT = 23;
static char wifiSsid[33], wifiPass[65];
static bool haveWifiCreds = false, netUp = false, servicesStarted = false;
static uint32_t wifiLastAttemptMs = 0;
static WiFiServer lxServer(LX200_PORT), telnetServer(TELNET_PORT);
static WiFiClient lxClient;
static bool lxLog = false;

static void loadWifi() {
  haveWifiCreds = false;
  if (!readFileToBuf(WIFI_PATH)) return;
  // two lines: ssid=<ssid> and pass=<password> (taken verbatim, spaces allowed)
  wifiSsid[0] = wifiPass[0] = 0;
  char *ls;
  for (char *line = strtok_r(fileBuf, "\n", &ls); line; line = strtok_r(NULL, "\n", &ls)) {
    size_t l = strlen(line);
    if (l && line[l - 1] == '\r') line[--l] = 0;
    if (!strncmp(line, "ssid=", 5)) { strncpy(wifiSsid, line + 5, sizeof wifiSsid - 1); wifiSsid[sizeof wifiSsid - 1] = 0; }
    if (!strncmp(line, "pass=", 5)) { strncpy(wifiPass, line + 5, sizeof wifiPass - 1); wifiPass[sizeof wifiPass - 1] = 0; }
  }
  haveWifiCreds = wifiSsid[0] != 0;
}

static void wifiConnect() {
  if (!haveWifiCreds) return;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  if (wifiPass[0]) WiFi.beginNoBlock(wifiSsid, wifiPass);
  else             WiFi.beginNoBlock(wifiSsid);          // open network
  wifiLastAttemptMs = millis();
  Con.printf("wifi: connecting to \"%s\"...\n", wifiSsid);
}

static void wifiUpdate() {
  static uint32_t lastCheckMs = 0;
  if (millis() - lastCheckMs < 250) return;
  lastCheckMs = millis();
  bool up = WiFi.status() == WL_CONNECTED && WiFi.localIP().isSet();
  if (up && !netUp) {
    netUp = true;
    Con.printf("wifi: connected, IP %s  (LX200: %s.local:%u, console: telnet %s.local)\n",
               WiFi.localIP().toString().c_str(), HOSTNAME, LX200_PORT, HOSTNAME);
    if (!servicesStarted) {
      servicesStarted = true;
      NTP.begin("pool.ntp.org", "time.nist.gov");
      MDNS.begin(HOSTNAME);
      lxServer.begin();
      lxServer.setNoDelay(true);
      telnetServer.begin();
    }
  } else if (!up && netUp) {
    netUp = false;
    Con.println(F("wifi: connection lost, will retry"));
  }
  if (!up && haveWifiCreds && millis() - wifiLastAttemptMs > 20000) wifiConnect();

  if (!timeWasValid && timeValid()) {
    timeWasValid = true;
    timeSource = "NTP";
    Con.print(F("time: "));
    printUtc(nowUnix());
    Con.println(F(" (NTP)"));
  }
}

struct PicoMount : Lx200Mount {
  bool getRaDec(double &ra, double &dec) override { return pointingNow(ra, dec); }
  bool getAltAz(double &alt, double &az) override {
    double ra, dec;
    if (!pointingNow(ra, dec)) return false;
    coord::haDecToAltAz(coord::wrapPi(lstNow() - ra), dec, siteLat(), alt, az);
    return true;
  }
  int  gotoTarget(double ra, double dec, char *msg, size_t n) override { return skyGoto(ra, dec, msg, n); }
  bool sync(double ra, double dec, char *msg, size_t n) override { return skySync(ra, dec, msg, n); }
  void stop() override { skyStop(); }
  void move(char d, bool start) override { if (start) startMove(d, moveRate, 0); else stopMove(d); }
  void setRate(char r) override { moveRate = r; }
  bool isSlewing() override { return skyState == SKY_GOTO || skyState == SKY_PARK; }
  bool isEquatorial() override { return sky.mode == coord::MOUNT_EQ; }
  bool isTracking() override { return skyState == SKY_TRACK; }
  int  alignStars() override { return model.nStars; }
  bool getSite(double &lat, double &lon) override { lat = siteLat(); lon = siteLon(); return sky.siteSet; }
  void setLatitude(double lat) override { setSite(lat / coord::kDeg, sky.lonDeg, "LX200"); }
  void setLongitude(double lon) override { setSite(sky.latDeg, lon / coord::kDeg, "LX200"); }
  bool getUtc(double &u) override { u = nowUnix(); return timeValid(); }
  void setUtc(double u) override {
    if (!strcmp(timeSource, "NTP")) return;          // NTP is better than a phone's clock
    setClockUtc(u, "LX200");
    Con.print(F("time: "));
    printUtc(u);
    Con.println(F(" (from LX200 client)"));
  }
};
static PicoMount   picoMount;
static Lx200State  lxState;
static Lx200Framer lxFramer;

static void pollLx200() {
  if (!servicesStarted) return;
  WiFiClient c = lxServer.accept();
  if (c) {
    if (lxClient && lxClient.connected()) lxClient.stop();   // one client at a time: newest wins
    lxClient = c;
    lxClient.setNoDelay(true);
    lxFramer = Lx200Framer();
    lxState.highPrec = true;
    Con.printf("lx200: client connected from %s\n", lxClient.remoteIP().toString().c_str());
  }
  char cmd[48], out[96];
  while (lxClient && lxClient.available()) {
    int r = lx200Feed(lxFramer, (char)lxClient.read(), cmd, sizeof cmd);
    if (r == LX_NONE) continue;
    if (r == LX_CMD) lx200Handle(cmd, lxState, picoMount, out, sizeof out);
    else             lx200Ack(picoMount, out, sizeof out);
    if (out[0]) lxClient.write((const uint8_t *)out, strlen(out));
    if (lxLog) Con.printf("lx200: :%s# -> %s\n", r == LX_CMD ? cmd : "<ACK>", out);
  }
}

static void handleLineFull(char *line);   // console command handler, below

static void pollTelnet() {
  if (!servicesStarted) return;
  WiFiClient c = telnetServer.accept();
  if (c) {
    if (telnetClient && telnetClient.connected()) telnetClient.stop();
    telnetClient = c;
    telnetOutLen = 0;
    Con.println(F("\nTelescope console (same as USB serial). 'h' for help."));
  }
  static char buf[96];
  static uint8_t n = 0;
  static uint8_t iacSkip = 0;
  while (telnetClient && telnetClient.available()) {
    uint8_t ch = telnetClient.read();
    if (iacSkip) { iacSkip--; continue; }
    if (ch == 255) { iacSkip = 2; continue; }      // telnet option negotiation: IAC cmd opt
    if (ch == '\r' || ch == '\n') {
      if (n) { buf[n] = 0; handleLineFull(buf); n = 0; }
    } else if (ch >= 32 && ch < 127 && n < sizeof(buf) - 1) buf[n++] = ch;
  }
  // send queued console output
  if (telnetOutLen) {
    if (!telnetClient || !telnetClient.connected()) { telnetOutLen = 0; return; }
    int room = telnetClient.availableForWrite();
    if (room <= 0) return;
    size_t k = telnetOutLen < (size_t)room ? telnetOutLen : (size_t)room;
    size_t w = telnetClient.write((const uint8_t *)telnetOut, k);
    memmove(telnetOut, telnetOut + w, telnetOutLen - w);
    telnetOutLen -= w;
  }
}

// ---------------------------------------------------------------- 'cap' analysis
// Goertzel amplitude of the duty signal at frequency f (Hz): the size, in duty
// %, of the part of the drag that repeats at that rate.
static double capAmplitude(int n, double mean, double f) {
  double w = 2.0 * M_PI * f / CAP_RATE_HZ, c = 2.0 * cos(w), s1 = 0, s2 = 0;
  for (int i = 0; i < n; i++) { double s0 = (capDuty[i] - mean) + c * s1 - s2; s2 = s1; s1 = s0; }
  double re = s1 - s2 * cos(w), im = s2 * sin(w);
  return 2.0 * sqrt(re * re + im * im) / n;
}

static void capReport() {
  int a = capAxis, n = capN;
  if (n < CAP_RATE_HZ) { Con.println(F("cap: too few samples")); return; }
  double T = (double)n / CAP_RATE_HZ, v = (capPos[n - 1] - capPos[0]) / ((n - 1) / (double)CAP_RATE_HZ);
  double mean = 0, dmin = 1e9, dmax = -1e9, emax = 0;
  for (int i = 0; i < n; i++) {
    mean += capDuty[i];
    dmin = fmin(dmin, capDuty[i]); dmax = fmax(dmax, capDuty[i]); emax = fmax(emax, fabs(capErr[i]));
  }
  mean /= n;
  // drive-train rates for this speed. Gear train confirmed on the az gearbox
  // (2026-11): 12T motor pinion -> 38/16 -> 38/16 -> 38/16 -> 48T output
  // = 6859/128 = 53.59:1.
  double motorHz = fabs(v) / coord::CPR;
  double g1 = motorHz * 12.0 / 38.0, g2 = g1 * 16.0 / 38.0, g3 = g2 * 16.0 / 38.0;
  struct Part { const char *name; double hz; } parts[] = {
    {"motor shaft (12T pinion)", motorHz},
    {"1st 38/16 compound gear", g1},
    {"2nd 38/16 compound gear", g2},
    {"3rd 38/16 compound gear", g3},
    {"48T output gear / worm", motorHz / coord::GEARBOX_RATIO},
  };
  Con.printf("cap: axis %d, %d samples over %.1f s at %d Hz; speed %.0f c/s; duty mean %.1f%% (min %.1f, max %.1f); worst err %.0f\n",
             a, n, T, CAP_RATE_HZ, v, mean, dmin, dmax, emax);
  for (const Part &p : parts) Con.printf("  %-36s turns at %6.2f Hz (every %.3f s)\n", p.name, p.hz, 1.0 / p.hz);

  // scan 0.2 .. 100 Hz; keep the strongest local peaks
  const double fLo = 0.2, fHi = 100.0, df = 0.5 / T;    // half the record's resolution
  const int K = 6;
  double pf[K] = {0}, pa[K] = {0};
  double aPrev2 = 0, aPrev1 = 0;
  for (double f = fLo; f <= fHi + df; f += df) {
    double a0 = capAmplitude(n, mean, f);
    if (aPrev1 > aPrev2 && aPrev1 >= a0) {               // local maximum at f - df
      for (int k = 0; k < K; k++)
        if (aPrev1 > pa[k]) {
          for (int j = K - 1; j > k; j--) { pa[j] = pa[j - 1]; pf[j] = pf[j - 1]; }
          pa[k] = aPrev1; pf[k] = f - df; break;
        }
    }
    aPrev2 = aPrev1; aPrev1 = a0;
  }
  Con.println(F("  strongest repeating parts of the drag (duty amplitude):"));
  for (int k = 0; k < K && pa[k] > 0; k++) {
    // closest candidate (part x harmonic) within tolerance; fundamentals win ties
    const char *match = "";
    int harm = 0;
    double best = 1e9;
    for (int h = 1; h <= 3; h++)
      for (const Part &p : parts) {
        double d = fabs(pf[k] - h * p.hz);
        if (d < fmax(0.04 * h * p.hz, 1.5 * df) && d < best - 1e-9) { best = d; match = p.name; harm = h; }
      }
    Con.printf("    %6.2f Hz  +-%.2f%%  %s%s%s\n", pf[k], pa[k], match[0] ? "<- " : "", match,
               harm > 1 ? (harm == 2 ? " (2x per turn)" : " (3x per turn)") : "");
  }
  Con.println(F("  ('cap dump' prints the raw samples as CSV -- use USB serial, it is long)"));
}

// Lead-up to the last fault from the black box: speed and duty at fixed times
// before the trip, and how quickly the speed collapsed.
static void printBlackBox(int a) {
  if (!bbFrozen[a]) return;
  int end = (bbHead[a] + BB_N - 1) % BB_N;        // the trip sample
  auto posAt = [&](int msBefore) { return bbPos[a][(end - msBefore + BB_N) % BB_N]; };
  auto speedAt = [&](int msBefore) {               // counts/s over the 2 ms ending msBefore ms before the trip
    return (posAt(msBefore) - posAt(msBefore + 2)) * 500.0;
  };
  double vRef = 0;
  for (int t = 300; t < 400; t++) vRef += speedAt(t);
  vRef /= 100;                                     // speed 0.3-0.4 s before the trip
  Con.printf("  lead-up (speed counts/s, duty %%) -- reference speed %.0f c/s 0.3-0.4 s before:\n", vRef);
  const int marks[] = {400, 250, 150, 100, 60, 40, 25, 15, 10, 6, 4, 2, 0};
  for (int m : marks) {
    double v = m + 2 < BB_N ? speedAt(m) : 0;
    int k = (end - m + BB_N) % BB_N;
    if (bbVolt[a][k] >= SUPPLY_PRESENT_V)
      Con.printf("    t-%3d ms  speed %7.0f  duty %6.1f  supply %5.2f V\n", m, v, bbDuty[a][k], bbVolt[a][k]);
    else
      Con.printf("    t-%3d ms  speed %7.0f  duty %6.1f\n", m, v, bbDuty[a][k]);
  }
  // how long the fall from 80 % to 20 % of the reference speed took
  if (fabs(vRef) > 2000) {
    int t80 = -1, t20 = -1;
    for (int m = 0; m < 380; m++) {
      double r = speedAt(m) / vRef;
      if (t20 < 0 && r > 0.2) t20 = m;             // latest time it was still above 20 %
      if (r > 0.8) { t80 = m; break; }             // latest time it was still above 80 %
    }
    if (t80 >= 0 && t20 >= 0) {
      int fall = t80 - t20;
      Con.printf("  speed fell from 80%% to 20%% in about %d ms, ending %d ms before the trip\n", fall, t20);
      if (fall <= 6)
        Con.println(F("  -> too fast for the motor to physically slow down: the ENCODER SIGNAL most likely\n"
                      "     dropped out (or a hard jam). Check the az encoder cable, connector and 5 V supply."));
      else if (fabs(speedAt(0)) > 0.1 * fabs(vRef))
        Con.printf("  -> it slowed over tens of ms and kept turning at %.0f%% of speed with full drive applied:\n"
                   "     the motor was getting far less drive than commanded (driver current limit/protection,\n"
                   "     12 V supply sag) or met heavy resistance. If the servo was oscillating ('cap'), the\n"
                   "     current swings can trip the driver -- lower kp/ki first.\n", 100.0 * fabs(speedAt(0) / vRef));
      else
        Con.println(F("  -> the motor slowed over tens of ms to a stop with drive applied: it most likely LOST DRIVE\n"
                      "     POWER and coasted (DRV8871 fault, 12 V supply, motor wiring) or jammed gradually.\n"
                      "     Check the motor wires, driver supply and driver temperature."));
    } else Con.println(F("  (speed didn't fall cleanly in the recorded window)"));
  }
}

static void printFaultInfo(int a) {
  if (!haveFaultInfo[a]) { Con.printf("axis %d: no fault since power-up\n", a); return; }
  const FaultInfo &f = faultInfo[a];
  static const char *const modeName[] = {"IDLE", "TRACK", "SLEW", "HOLD"};
  Con.printf("axis %d FAULT details: mode %s, error %.0f counts (limit %.0f), target speed %.0f c/s, "
             "actual %.0f c/s, duty %.1f%% (max %.0f%%)\n",
             a, modeName[f.mode & 3], f.err, f.limit, f.targetVel, f.measVel, f.duty, axis[a].maxDuty);
  if (f.supplyV >= SUPPLY_PRESENT_V) Con.printf("  motor supply at the trip: %.2f V\n", f.supplyV);
  printBlackBox(a);
}

// ---------------------------------------------------------------- serial UI
static bool streaming = false;
static uint32_t lastReportMs = 0;

static void printHelp() {
  Con.println(F(
    "\ncommands:\n"
    "  h                       this help\n"
    "  s                       toggle 2 Hz status stream\n"
    "  e <axis|a>              enable axis (or 'a' for both)\n"
    "  d <axis|a>              disable axis (coasts, clears fault, aborts any auto-tune)\n"
    "  z <axis|a>              zero the encoder count (axis must be disabled first)\n"
    "  g <axis> <counts>       slew to an absolute count\n"
    "  gd <axis> <delta>       slew by a relative count\n"
    "  t <axis> sid|-sid|<c/s> track at sidereal or an arbitrary signed counts/s\n"
    "  o <axis>                stop and hold the current position\n"
    "  p                       print axis parameters and state\n"
    "  x                       emergency stop: disable both axes\n"
    "  fault                   show what each axis was doing when it last faulted\n"
    "  vsup [reset]            motor supply voltage now / lowest / highest (12V-10k-GP28-1k-GND)\n"
    "  cap <axis> [sec]        record duty at 500 Hz during a move, report repeating drag + which part\n"
    "                          (motor / gearbox stage / worm) turns at that rate; 'cap dump' = raw CSV\n"
    "  kp/ki <axis> <val>      tune the MOVING (track/slew) gains directly\n"
    "  kph/kih <axis> <val>    tune the HOLD gains directly\n"
    "  ff <axis> <val>         set feedforward gain directly (duty %% per count/s)\n"
    "  ffo <axis> <val>        set feedforward friction offset directly (duty %%)\n"
    "  md/mi <axis> <val>      max duty %% / max integral (moving)\n"
    "  mv/ma <axis> <val>      max slew velocity (counts/s) / accel (counts/s^2)\n"
    "  et <axis> <val>         following-error fault threshold, counts\n"
    "\n"
    "auto-tune (axis must be enabled first, keep hands/cables clear):\n"
    "  auto full [tl|zn] [duty] [h]  ONE-SHOT: ff+pi+speed on axis 0 then 1, apply all, save to flash\n"
    "                              (enables both axes itself; tl is the default)\n"
    "  auto ff <axis>              measure feedforward gain (open-loop ramp, both directions)\n"
    "  auto pi <axis> [duty] [h]   relay auto-tune for kp/ki (default duty=15%%, hysteresis=1.5 counts)\n"
    "  auto speed <axis>           open-loop step to md, suggest mv/ma\n"
    "  auto apply <axis> ff|tl|zn|speed   commit the last suggestion from that test\n"
    "  auto abort <axis|a>         stop any running auto-tune (and 'auto full')\n"
    "  auto dir <axis|a>           check motor direction (sets invertMotor, saved). Needed once after\n"
    "                              any wiring change -- axes can't be enabled until it has passed\n"
    "\n"
    "calibration storage (flash, loaded automatically at boot):\n"
    "  cfg save                save current calibration of both axes\n"
    "  cfg load                reload the saved calibration (both axes must be disabled)\n"
    "  cfg show                print the saved file\n"
    "  cfg erase               delete the saved file (built-in defaults from next boot)\n"
    "\n"
    "network (this console is also on 'telnet telescope.local'):\n"
    "  wifi \"<ssid>\" \"<pass>\"  save WiFi credentials and connect   | wifi = status | wifi forget\n"
    "  lxlog                   toggle logging of LX200 traffic (port 4030)\n"
    "\n"
    "sky (coordinates are JNow; Stellarium/INDI/SkySafari do this over LX200):\n"
    "  where                   time, site, mount, pointing, alignment summary\n"
    "  site <lat> <lon>        observing site, decimal degrees, east +   (saved)\n"
    "  mount eq|altaz          wedge (equatorial) or alt-az              (saved, clears alignment)\n"
    "  msign <axis> 1|-1       encoder count direction of an axis        (saved, clears alignment)\n"
    "  horizon <deg>           refuse gotos below this altitude          (saved)\n"
    "  jogmax <fraction>       fastest arrow-button speed as a fraction of mv, 0.05-1 (saved)\n"
    "  scale [<axis> mark | <axis> <deg> | <axis> rev <n>]   measure/set counts per axis turn (saved)\n"
    "  time YYYY-MM-DD HH:MM:SS   set UTC by hand (no internet)\n"
    "  sync <ra> <dec>         'the scope is centred on this' (hh:mm:ss +dd:mm:ss)\n"
    "  goto <ra> <dec>         slew there and track\n"
    "  align [clear]           show the alignment stars and residuals / start over\n"
    "  jog n|s|e|w [g|c|m|s] [sec]   nudge (guide/centre/move/slew rate), 'jog stop'\n"
    "  trk on|off              tracking after goto/sync\n"
    "  stop                    stop slewing, moving and tracking (motors hold)\n"
    "  park [here]             slew home (or stay) and save the alignment for the next boot\n"));
}

static void printStatus(int a) {
  AxisCtrl &x = axis[a];
  const char *modeStr = x.mode==MODE_IDLE?"IDLE":x.mode==MODE_TRACK?"TRACK":x.mode==MODE_SLEW?"SLEW":"HOLD";
  Con.printf("axis %d: %s%s%s en=%d mode=%-5s pos=%9ld target=%10.2f err=%7.3f duty=%6.2f%%\n"
                "  moving: kp=%.4f ki=%.4f maxI=%.1f   hold: kp=%.4f ki=%.4f maxI=%.1f\n"
                "  ff=%.4f ffo=%.3f maxDuty=%.1f maxVel=%.2f maxAccel=%.2f faultAt=%.1f (+%.1fs of travel)\n",
                a, invertMotor[a] ? "(inv) " : "      ", faulted[a] ? "FAULT " : "",
                testMode[a] ? "TESTING " : "",
                motorsEnabled[a], modeStr, (long)posSnap[a], targetSnap[a], errSnap[a], dutySnap[a],
                x.kp, x.ki, x.maxIntegral, x.kpHold, x.kiHold, x.maxIntegralHold,
                x.ffGain, x.ffOffset, x.maxDuty, x.maxVel, x.maxAccel, faultThreshold[a], FAULT_LAG_S);
}

static void report() {
  Con.printf("t=%8.2f ", millis() / 1000.0);
  for (int a = 0; a < 2; a++)
    Con.printf("| A%d %s%s%s pos=%9ld tgt=%10.2f err=%7.3f duty=%6.2f%% ",
                  a, motorsEnabled[a] ? "EN " : "off", faulted[a] ? "!FAULT" : "",
                  testMode[a] ? "*TEST*" : "",
                  (long)posSnap[a], targetSnap[a], errSnap[a], dutySnap[a]);
  Con.println();
}

static void forEachAxis(const char *sel, void (*fn)(int)) {
  if (sel && (sel[0] == 'a' || sel[0] == 'A')) { fn(0); fn(1); }
  else if (sel) { int a = atoi(sel); if (a == 0 || a == 1) fn(a); }
}
static void abortTune(int a) {
  cancelFullTune("user abort");
  fftune[a].active = false;
  relay[a].active = false;
  speedtune[a].active = false;
  dirtest[a].active = false;
  if (testMode[a]) returnToNormalServo(a);
  Con.printf("axis %d: any running auto-tune stopped\n", a);
}

static void handleAutoCmd(char *rest) {
  char *sub = strtok(rest, " \t");
  if (!sub) { Con.println(F("usage: auto full|ff|pi|speed|apply|abort <axis> ...")); return; }
  char *t1 = strtok(NULL, " \t");
  char *t2 = strtok(NULL, " \t");
  char *t3 = strtok(NULL, " \t");
  bool axValid = (t1 && (t1[0]=='0' || t1[0]=='1'));
  int ax = axValid ? atoi(t1) : -1;

  if (autoFullBusy && strcasecmp(sub, "abort")) {
    Con.println(F("auto full is running -- only 'auto abort a' (or 'x') accepted")); return;
  }
  if (!strcasecmp(sub, "dir")) {
    if (t1 && (t1[0] == 'a' || t1[0] == 'A')) { startDirTest(0); startDirTest(1); }
    else if (axValid) startDirTest(ax);
    else Con.println(F("usage: auto dir <axis|a>"));
    return;
  }
  if (!strcasecmp(sub, "full")) {
    if (t1 && strcasecmp(t1, "tl") && strcasecmp(t1, "zn")) {
      Con.println(F("usage: auto full [tl|zn] [duty%] [hysteresis_counts]")); return;
    }
    bool zn = t1 && !strcasecmp(t1, "zn");
    startFullTune(zn, t2 ? atof(t2) : RELAY_DEFAULT_DUTY, t3 ? atof(t3) : RELAY_DEFAULT_H);
  } else if (!strcasecmp(sub, "ff")) {
    if (axValid) startFFTune(ax); else Con.println(F("usage: auto ff <axis>"));
  } else if (!strcasecmp(sub, "pi")) {
    if (axValid) {
      double d = t2 ? atof(t2) : RELAY_DEFAULT_DUTY;
      double h = t3 ? atof(t3) : RELAY_DEFAULT_H;
      startRelayTune(ax, d, h);
    } else Con.println(F("usage: auto pi <axis> [duty%] [hysteresis_counts]"));
  } else if (!strcasecmp(sub, "speed")) {
    if (axValid) startSpeedTune(ax); else Con.println(F("usage: auto speed <axis>"));
  } else if (!strcasecmp(sub, "abort")) {
    forEachAxis(t1, abortTune);
  } else if (!strcasecmp(sub, "apply")) {
    if (!axValid || !t2) { Con.println(F("usage: auto apply <axis> ff|tl|zn|speed")); return; }
    if (!strcasecmp(t2, "ff")) {
      if (!haveFFResult[ax]) { Con.println(F("no ff result yet")); return; }
      double P = 0, Pd = 0;
      if (haveSpeedResult[ax]) { P = lastPlateau[ax]; Pd = lastPlateauDuty[ax]; }
      else currentHighPoint(ax, P, Pd);           // keep the existing high-speed slope if there is one
      applyFF(ax, lastTuneFF[ax] * SIDEREAL_CPS, P, Pd);
    } else if (!strcasecmp(t2, "tl")) {
      if (!havePIResult[ax]) { Con.println(F("no pi result yet")); return; }
      SERVO_LOCK(); axis[ax].kp = PI_DERATE * lastKpTL[ax]; axis[ax].ki = PI_DERATE * lastKiTL[ax]; SERVO_UNLOCK();
      Con.printf("axis %d: moving kp=%.4f ki=%.4f applied (Tyreus-Luyben x %.2f safety factor)\n",
                 ax, axis[ax].kp, axis[ax].ki, PI_DERATE);
    } else if (!strcasecmp(t2, "zn")) {
      if (!havePIResult[ax]) { Con.println(F("no pi result yet")); return; }
      SERVO_LOCK(); axis[ax].kp = PI_DERATE * lastKpZN[ax]; axis[ax].ki = PI_DERATE * lastKiZN[ax]; SERVO_UNLOCK();
      Con.printf("axis %d: moving kp=%.4f ki=%.4f applied (Ziegler-Nichols x %.2f safety factor)\n",
                 ax, axis[ax].kp, axis[ax].ki, PI_DERATE);
    } else if (!strcasecmp(t2, "speed")) {
      if (!haveSpeedResult[ax]) { Con.println(F("no speed result yet")); return; }
      SERVO_LOCK(); axis[ax].maxVel = lastSuggestMaxVel[ax]; axis[ax].maxAccel = lastSuggestMaxAccel[ax]; SERVO_UNLOCK();
      Con.printf("axis %d: mv=%.2f ma=%.2f applied\n", ax, lastSuggestMaxVel[ax], lastSuggestMaxAccel[ax]);
      applyFF(ax, sidDutyNow(ax), lastPlateau[ax], lastPlateauDuty[ax]);
    } else Con.println(F("usage: auto apply <axis> ff|tl|zn|speed"));
  } else {
    Con.println(F("usage: auto full|ff|pi|speed|apply|abort <axis> ..."));
  }
}

static void handleCfgCmd(const char *sub) {
  if (!sub) { Con.println(F("usage: cfg save|load|show|erase")); return; }
  if (autoFullBusy) { Con.println(F("auto full is running -- wait for it to finish")); return; }
  if (!strcasecmp(sub, "save")) {
    for (int a = 0; a < 2; a++)
      if (testMode[a] || axis[a].mode == MODE_TRACK || axis[a].mode == MODE_SLEW) {
        Con.printf("axis %d is moving/testing -- stop it ('o %d') before saving\n", a, a); return;
      }
    saveCalib();
  } else if (!strcasecmp(sub, "load")) {
    if (motorsEnabled[0] || motorsEnabled[1]) { Con.println(F("disable both axes first ('d a')")); return; }
    loadCalib(false);
  } else if (!strcasecmp(sub, "show")) {
    if (readCalibFile(false)) { Con.println(calibBuf); }
  } else if (!strcasecmp(sub, "erase")) {
    if (!fsReady) { Con.println(F("cfg: no filesystem")); return; }
    if (LittleFS.remove(CALIB_PATH)) Con.println(F("cfg: saved calibration erased (current values stay live until reboot)"));
    else Con.println(F("cfg: nothing to erase"));
  } else Con.println(F("usage: cfg save|load|show|erase"));
}

// wifi                         show status
// wifi "<ssid>" "<password>"   save credentials and connect (quotes optional without spaces)
// wifi forget                  delete saved credentials
static void handleWifiCmd(char *rest) {
  while (*rest == ' ') rest++;
  if (!*rest) {
    if (!haveWifiCreds) { Con.println(F("wifi: not configured -- wifi \"<ssid>\" \"<password>\"")); return; }
    if (netUp) Con.printf("wifi: \"%s\" connected, IP %s, RSSI %d dBm, LX200 port %u, telnet port %u\n", wifiSsid,
                          WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), LX200_PORT, TELNET_PORT);
    else Con.printf("wifi: \"%s\" not connected (retrying every 20 s)\n", wifiSsid);
    Con.printf("lx200 client: %s\n", lxClient && lxClient.connected() ? lxClient.remoteIP().toString().c_str() : "none");
    return;
  }
  if (!strcasecmp(rest, "forget")) {
    if (fsReady) LittleFS.remove(WIFI_PATH);
    haveWifiCreds = false;
    WiFi.disconnect();
    Con.println(F("wifi: credentials deleted, disconnected"));
    return;
  }
  char *tok[2] = {nullptr, nullptr};
  int nt = 0;
  char *p = rest;
  while (*p && nt < 2) {
    while (*p == ' ') p++;
    if (!*p) break;
    if (*p == '"') {
      char *q = strchr(++p, '"');
      if (!q) { Con.println(F("wifi: missing closing quote")); return; }
      *q = 0; tok[nt++] = p; p = q + 1;
    } else {
      tok[nt++] = p;
      char *q = strchr(p, ' ');
      if (q) { *q = 0; p = q + 1; } else p += strlen(p);
    }
  }
  if (!tok[0] || !tok[0][0] || strlen(tok[0]) > 32 || (tok[1] && strlen(tok[1]) > 63)) {
    Con.println(F("usage: wifi \"<ssid>\" \"<password>\"   (ssid <= 32, password <= 63 chars)")); return;
  }
  int len = snprintf(fileBuf, sizeof fileBuf, "ssid=%s\npass=%s\n", tok[0], tok[1] ? tok[1] : "");
  if (!writeFileAtomic(WIFI_PATH, fileBuf, len)) return;
  loadWifi();
  Con.println(F("wifi: credentials saved (plain text in the Pico's flash)"));
  WiFi.disconnect();
  netUp = false;
  wifiConnect();
}

static void printRaDec(double ra, double dec) {
  char a[16], b[16];
  lx200FormatRa(ra, true, a, sizeof a);
  lx200FormatDec(dec, true, b, sizeof b);
  a[strlen(a) - 1] = 0; b[strlen(b) - 1] = 0;          // drop the LX200 '#'
  Con.printf("RA %s  Dec %s", a, b);
}

static void printWhere() {
  static const char *const stateName[] = {"idle", "goto", "tracking", "parking"};
  Con.print(F("time:   "));
  if (timeValid()) {
    printUtc(nowUnix());
    double l = lstNow() / coord::kHour;
    Con.printf(" (%s)  LST %02d:%02d\n", timeSource, (int)l, (int)((l - (int)l) * 60));
  } else Con.println(F("not set -- needs WiFi/NTP, 'time', or an LX200 client that sends it"));
  if (sky.siteSet) Con.printf("site:   lat %.4f  lon %.4f (east +)\n", sky.latDeg, sky.lonDeg);
  else Con.println(F("site:   not set -- 'site <lat> <lon>'"));
  Con.printf("mount:  %s  axis directions %+d %+d  horizon limit %.1f deg\n",
             sky.mode == coord::MOUNT_EQ ? "equatorial (wedge)" : "alt-az", sky.sign0, sky.sign1, sky.horizonDeg);
  Con.printf("scale:  axis 0 %.0f, axis 1 %.0f counts per turn\n", sky.rev0, sky.rev1);
  Con.printf("jog:    fastest arrow speed %.2f x mv (axis 0 %.2f deg/s, axis 1 %.2f deg/s)\n", sky.jogMax,
             sky.jogMax * axis[0].maxVel * 360.0 / sky.rev0, sky.jogMax * axis[1].maxVel * 360.0 / sky.rev1);
  Con.printf("state:  %s, tracking %s\n", stateName[skyState], trackingOn ? "on" : "off");
  double ra, dec;
  if (pointingNow(ra, dec)) {
    double alt, az;
    coord::haDecToAltAz(coord::wrapPi(lstNow() - ra), dec, siteLat(), alt, az);
    Con.print(F("scope:  "));
    printRaDec(ra, dec);
    Con.printf("  Alt %.2f  Az %.2f\n", alt / coord::kDeg, az / coord::kDeg);
  } else Con.println(F("scope:  position unknown until site and time are set"));
  Con.printf("align:  %d sync star%s\n", model.nStars, model.nStars == 1 ? "" : "s");
}

static void printAlign() {
  Con.printf("align: %s mount, %d sync star%s, axis-1 zero %.3f deg\n",
             model.mode == coord::MOUNT_EQ ? "EQ" : "alt-az", model.nStars, model.nStars == 1 ? "" : "s", model.z1 / coord::kDeg);
  for (int i = 0; i < model.nStars; i++) {
    const coord::SyncStar &s = model.stars[i];
    double h, d;
    coord::rawToHaDec(model, s.raw0, s.raw1, h, d);
    double r = coord::angleBetween(coord::haDecToVec(h, d), coord::haDecToVec(s.ha, s.dec));
    Con.printf("  star %d: HA %+.3fh Dec %+.3f  residual %.1f arcmin\n", i + 1, s.ha / coord::kHour, s.dec / coord::kDeg,
               r / coord::kDeg * 60);
  }
  if (model.nStars >= 2) {
    double tilt = coord::angleBetween(coord::mul(model.T, coord::Vec3{0, 0, 1}),
                                      coord::mul(coord::nominalT(model.mode, model.latitude), coord::Vec3{0, 0, 1}));
    Con.printf("  mount axis is %.2f deg from ideal\n", tilt / coord::kDeg);
  }
}

// Sky / alignment / network commands. Returns false if cmd isn't one of them.
static bool handleSkyCmd(const char *cmd, char *t1, char *t2, char *t3) {
  char msg[48];
  if (!strcasecmp(cmd, "where")) { printWhere(); return true; }
  if (!strcasecmp(cmd, "site")) {
    if (t1 && t2) {
      double la = atof(t1), lo = atof(t2);
      if (fabs(la) > 90 || fabs(lo) > 180) { Con.println(F("site: lat -90..90, lon -180..180 (east +)")); return true; }
      setSite(la, lo, "console");
    } else if (sky.siteSet) Con.printf("site: lat %.4f lon %.4f (east +)\n", sky.latDeg, sky.lonDeg);
    else Con.println(F("site: not set. usage: site <lat> <lon>   (decimal degrees, east +)"));
    return true;
  }
  if (!strcasecmp(cmd, "mount")) {
    if (t1 && (!strcasecmp(t1, "eq") || !strcasecmp(t1, "altaz"))) {
      sky.mode = !strcasecmp(t1, "eq") ? coord::MOUNT_EQ : coord::MOUNT_ALTAZ;
      resetModel("mount mode changed");
      saveSky();
    } else Con.printf("mount: %s. usage: mount eq|altaz\n", sky.mode == coord::MOUNT_EQ ? "eq" : "altaz");
    return true;
  }
  if (!strcasecmp(cmd, "msign")) {
    int a = t1 ? atoi(t1) : -1, s = t2 ? atoi(t2) : 0;
    if ((a == 0 || a == 1) && (s == 1 || s == -1) && t1[0] != 'a') {
      (a == 0 ? sky.sign0 : sky.sign1) = s;
      resetModel("axis direction changed");
      saveSky();
    } else Con.printf("msign: axis 0 %+d, axis 1 %+d. usage: msign <axis> 1|-1\n", sky.sign0, sky.sign1);
    return true;
  }
  if (!strcasecmp(cmd, "jogmax")) {
    if (t1) {
      double f = atof(t1);
      if (f < SKY_JOGMAX_MIN || f > SKY_JOGMAX_MAX) { Con.println(F("jogmax: 0.05..1 (fraction of mv)")); return true; }
      sky.jogMax = f;
      saveSky();
    }
    for (int b = 0; b < 2; b++)
      Con.printf("jogmax %.2f: axis %d fastest arrow move %.0f counts/s = %.2f deg/s (mv %.0f)\n", sky.jogMax, b,
                 sky.jogMax * axis[b].maxVel, sky.jogMax * axis[b].maxVel * 360.0 / (b == 0 ? sky.rev0 : sky.rev1),
                 axis[b].maxVel);
    return true;
  }
  if (!strcasecmp(cmd, "horizon")) {
    if (t1) {
      double h = atof(t1);
      if (h < -10 || h > 60) { Con.println(F("horizon: -10..60 deg")); return true; }
      sky.horizonDeg = h;
      saveSky();
    }
    Con.printf("horizon limit: %.1f deg\n", sky.horizonDeg);
    return true;
  }
  if (!strcasecmp(cmd, "time")) {
    if (t1 && t2) {
      long y; unsigned mo, d; int hh, mi; double ss = 0;
      if (sscanf(t1, "%ld-%u-%u", &y, &mo, &d) == 3 && sscanf(t2, "%d:%d:%lf", &hh, &mi, &ss) >= 2 &&
          mo >= 1 && mo <= 12 && d >= 1 && d <= 31 && hh >= 0 && hh < 24 && mi >= 0 && mi < 60 && ss >= 0 && ss < 61) {
        setClockUtc(coord::unixFromCivil(y, mo, d, hh, mi, ss), "manual");
      } else { Con.println(F("usage: time YYYY-MM-DD HH:MM[:SS]   (UTC)")); return true; }
    }
    Con.print(F("time: "));
    if (timeValid()) { printUtc(nowUnix()); Con.printf(" (%s)\n", timeSource); }
    else Con.println(F("not set"));
    return true;
  }
  if (!strcasecmp(cmd, "align")) {
    if (t1 && !strcasecmp(t1, "clear")) resetModel("by request");
    printAlign();
    return true;
  }
  if (!strcasecmp(cmd, "goto") || !strcasecmp(cmd, "sync")) {
    double ra, dec;
    if (!t1 || !t2 || !lx200ParseRa(t1, ra) || !lx200ParseDec(t2, dec)) {
      Con.printf("usage: %s <RA hh:mm:ss> <Dec +dd:mm:ss>   (JNow)\n", cmd); return true;
    }
    msg[0] = 0;
    if (cmd[0] == 'g' || cmd[0] == 'G') { if (skyGoto(ra, dec, msg, sizeof msg) != 0) Con.printf("goto: %s\n", msg); }
    else if (!skySync(ra, dec, msg, sizeof msg)) Con.printf("sync: %s\n", msg);
    return true;
  }
  if (!strcasecmp(cmd, "jog")) {
    if (t1 && !strcasecmp(t1, "stop")) { moveCmd[0] = moveCmd[1] = 0; jogTimed = false; return true; }
    char d = t1 ? (char)tolower(t1[0]) : 0;
    if (!d || !strchr("nsew", d) || t1[1]) {
      Con.println(F("usage: jog n|s|e|w [g|c|m|s] [seconds]   (rate: guide 1x, centre 8x, move 32x, slew max; default c 1s)\n"
                    "       jog stop"));
      return true;
    }
    char r = t2 ? (char)toupper(t2[0]) : 'C';
    if (!strchr("GCMS", r)) r = 'C';
    double secs = t3 ? atof(t3) : 1.0;
    if (secs <= 0 || secs > 60) secs = 1.0;
    startMove(d, r, (uint32_t)(secs * 1000));
    return true;
  }
  if (!strcasecmp(cmd, "trk")) {
    if (t1 && !strcasecmp(t1, "off")) {
      trackingOn = false;
      if (skyState == SKY_TRACK) {
        skyState = SKY_IDLE;
        for (int a = 0; a < 2; a++) { SERVO_LOCK(); axis[a].targetVel = axis[a].velCmd = 0; axis[a].mode = MODE_HOLD; SERVO_UNLOCK(); }
      }
    } else if (t1 && !strcasecmp(t1, "on")) {
      trackingOn = true;
      if (skyState == SKY_IDLE) {
        if (!skyReady()) Con.println(F("trk: set site and time first"));
        else if (ensureAxesForSky(msg, sizeof msg)) beginTrackingHere();
        else Con.printf("trk: %s\n", msg);
      }
    }
    Con.printf("tracking %s (%s)\n", trackingOn ? "on" : "off", skyState == SKY_TRACK ? "active" : "not active");
    return true;
  }
  if (!strcasecmp(cmd, "stop")) {
    skyStop();
    abortToIdle = gotoAborting;
    if (skyState == SKY_TRACK) {
      skyState = SKY_IDLE;
      for (int a = 0; a < 2; a++) { SERVO_LOCK(); axis[a].targetVel = axis[a].velCmd = 0; axis[a].mode = MODE_HOLD; SERVO_UNLOCK(); }
    }
    return true;
  }
  if (!strcasecmp(cmd, "park")) { startPark(t1 && !strcasecmp(t1, "here")); return true; }
  if (!strcasecmp(cmd, "scale")) {
    static long markPos[2];
    static bool marked[2] = {false, false};
    int a = (t1 && (t1[0] == '0' || t1[0] == '1') && !t1[1]) ? t1[0] - '0' : -1;
    if (a >= 0 && t2 && !strcasecmp(t2, "mark")) {
      markPos[a] = posSnap[a];
      marked[a] = true;
      Con.printf("scale: axis %d marked at %ld counts. Move it a known angle (bigger = more accurate), then 'scale %d <degrees>'\n",
                 a, markPos[a], a);
      return true;
    }
    if (a >= 0 && t2 && !strcasecmp(t2, "rev") && t3) {
      double rev = atof(t3);
      if (rev < SKY_REV_MIN || rev > SKY_REV_MAX) { Con.println(F("scale: counts per rev out of range")); return true; }
      (a == 0 ? sky.rev0 : sky.rev1) = rev;
      resetModel("axis scale changed");
      saveSky();
    } else if (a >= 0 && t2) {
      double deg = atof(t2);
      long delta = (long)posSnap[a] - markPos[a];
      if (!marked[a]) { Con.printf("scale: 'scale %d mark' first\n", a); return true; }
      if (fabs(deg) < 5 || labs(delta) < 10000) {
        Con.println(F("scale: move at least 5 deg / 10000 counts between mark and measurement")); return true;
      }
      double rev = fabs(delta) * 360.0 / fabs(deg);
      if (rev < SKY_REV_MIN || rev > SKY_REV_MAX) { Con.printf("scale: %.0f counts/rev is implausible -- not saved\n", rev); return true; }
      Con.printf("scale: axis %d moved %ld counts for %.2f deg -> %.0f counts per turn (%.3f x the default)\n",
                 a, delta, deg, rev, rev / coord::COUNTS_PER_REV);
      (a == 0 ? sky.rev0 : sky.rev1) = rev;
      marked[a] = false;
      resetModel("axis scale changed");
      saveSky();
    } else if (t1) {
      Con.println(F("usage: scale                       show counts per axis turn\n"
                    "       scale <axis> mark           remember where the axis is now\n"
                    "       scale <axis> <degrees>      it has since turned this far (measured): set from that\n"
                    "       scale <axis> rev <counts>   set counts per turn directly"));
      return true;
    }
    for (int b = 0; b < 2; b++)
      Con.printf("scale: axis %d = %.0f counts per turn (%.1f counts/deg, %.3f arcsec/count)\n",
                 b, axisRev(b), axisRev(b) / 360.0, 1296000.0 / axisRev(b));
    return true;
  }
  if (!strcasecmp(cmd, "lxlog")) { lxLog = !lxLog; Con.printf("lx200 logging %s\n", lxLog ? "on" : "off"); return true; }
  return false;
}

// Line handler. "auto ..." and "wifi ..." are special-cased first (via a peeked
// copy of the line) so their arguments survive intact (strtok, quotes, spaces).
static void handleLineFull(char *line) {
  // peek first token without disturbing the rest of the line for the 'auto' case
  char lineCopy[48];
  strncpy(lineCopy, line, sizeof(lineCopy) - 1);
  lineCopy[sizeof(lineCopy) - 1] = 0;
  char *firstSpace = strchr(lineCopy, ' ');
  if (firstSpace) *firstSpace = 0;

  if (!strcasecmp(lineCopy, "auto")) {
    handleAutoCmd(firstSpace ? firstSpace + 1 : (char*)"");
    return;
  }
  if (!strcasecmp(lineCopy, "wifi")) {
    handleWifiCmd(firstSpace ? line + (firstSpace - lineCopy) + 1 : (char*)"");
    return;
  }

  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *t1 = strtok(NULL, " \t");
  char *t2 = strtok(NULL, " \t");
  char *t3 = strtok(NULL, " \t");
  int ax = t1 ? atoi(t1) : -1;
  bool axValid = (t1 && (t1[0]=='0' || t1[0]=='1'));

  if (!strcasecmp(cmd, "h")) { printHelp(); return; }
  if (!strcasecmp(cmd, "fault")) { printFaultInfo(0); printFaultInfo(1); return; }
  if (!strcasecmp(cmd, "vsup")) {
    if (t1 && !strcasecmp(t1, "reset")) { supplyMinV = 99; supplyMaxV = 0; }
    if (supplyV < SUPPLY_PRESENT_V) { Con.println(F("vsup: monitor not fitted (12V -> 10k -> GP28 -> 1k -> GND)")); return; }
    Con.printf("vsup: now %.2f V, min %.2f V, max %.2f V (since boot or 'vsup reset')\n", supplyV, supplyMinV, supplyMaxV);
    return;
  }
  if (!strcasecmp(cmd, "cap")) {
    if (t1 && !strcasecmp(t1, "dump")) {
      if (capState != CAP_DONE) { Con.println(F("cap: no finished capture")); return; }
      Con.println(F("i,t_s,pos,speed_cps,duty_pct,err_counts"));
      for (int i = 0; i < capN; i++) {
        double sp = i ? (capPos[i] - capPos[i - 1]) * (double)CAP_RATE_HZ : 0;
        Con.printf("%d,%.4f,%ld,%.0f,%.2f,%.2f\n", i, (double)i / CAP_RATE_HZ, (long)capPos[i], sp, capDuty[i], capErr[i]);
        if ((i & 63) == 63) Serial.flush();
      }
      return;
    }
    if (!axValid) { Con.println(F("usage: cap <axis> [seconds 1-10]   then start a steady move; 'cap dump' for raw CSV")); return; }
    double secs = t2 ? atof(t2) : 4.0;
    if (secs < 1 || secs > 10) secs = 4.0;
    capState = CAP_IDLE;
    capAxis = ax;
    capWant = (int)(secs * CAP_RATE_HZ);
    capArmedMs = millis();
    capState = CAP_ARMED;
    Con.printf("cap: armed on axis %d for %.1f s -- starts when it moves faster than %.0f counts/s.\n"
               "     Start a steady move now (e.g. hold an arrow in Stellarium).\n", ax, secs, CAP_START_CPS);
    return;
  }
  if (handleSkyCmd(cmd, t1, t2, t3)) return;
  // the raw axis commands below take an axis away from the sky engine
  if (!strcasecmp(cmd, "g") || !strcasecmp(cmd, "gd") || !strcasecmp(cmd, "t") || !strcasecmp(cmd, "o")) {
    skyState = SKY_IDLE;
    moveCmd[0] = moveCmd[1] = moveVel[0] = moveVel[1] = 0;
    wasMoving[0] = wasMoving[1] = false;
  }
  if (!strcasecmp(cmd, "cfg")) { handleCfgCmd(t1); return; }
  if (!strcasecmp(cmd, "s")) { streaming = !streaming; lastReportMs = millis(); return; }
  if (!strcasecmp(cmd, "e")) { forEachAxis(t1, enableAxis); return; }
  if (!strcasecmp(cmd, "d")) { forEachAxis(t1, disableAxis); return; }
  if (!strcasecmp(cmd, "z")) {
    for (int a = 0; a < 2; a++) {
      if (t1 && t1[0] != 'a' && ax != a) continue;
      if (motorsEnabled[a]) { Con.printf("axis %d must be disabled before zeroing\n", a); continue; }
      zeroOffset[a] += readRaw(a);
      SERVO_LOCK(); axis[a].targetPos = 0; SERVO_UNLOCK();
      Con.printf("axis %d zeroed\n", a);
    }
    return;
  }
  if (!strcasecmp(cmd, "g"))  { if (axValid && t2) startSlew(ax, atof(t2)); else Con.println(F("usage: g <axis> <counts>")); return; }
  if (!strcasecmp(cmd, "gd")) { if (axValid && t2) startSlew(ax, (double)posSnap[ax] + atof(t2)); else Con.println(F("usage: gd <axis> <delta>")); return; }
  if (!strcasecmp(cmd, "t")) {
    if (axValid && t2) {
      double rate;
      if (!strcasecmp(t2, "sid")) rate = SIDEREAL_CPS;
      else if (!strcasecmp(t2, "-sid")) rate = -SIDEREAL_CPS;
      else rate = atof(t2);
      startTrack(ax, rate);
      Con.printf("axis %d tracking at %.3f counts/s\n", ax, rate);
    } else Con.println(F("usage: t <axis> sid|-sid|<counts/s>"));
    return;
  }
  if (!strcasecmp(cmd, "o")) { if (axValid) holdHere(ax); return; }
  if (!strcasecmp(cmd, "p")) {
    for (int a = 0; a < 2; a++) printStatus(a);
    Con.printf("sidereal = %.4f counts/s, %.3f arcsec/count\n", SIDEREAL_CPS, 1296000.0 / (TOTAL_RATIO * CPR));
    if (supplyV >= SUPPLY_PRESENT_V)
      Con.printf("motor supply = %.2f V (min %.2f, max %.2f since 'vsup reset')\n", supplyV, supplyMinV, supplyMaxV);
    else Con.println(F("motor supply monitor: not fitted (12V -> 10k -> GP28 -> 1k -> GND)"));
    return;
  }
  if (!strcasecmp(cmd, "x")) { disableAxis(0); disableAxis(1); Con.println(F("EMERGENCY STOP")); return; }
  if (!strcasecmp(cmd, "kp"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].kp = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "ki"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].ki = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "kph")) { if (axValid && t2) { SERVO_LOCK(); axis[ax].kpHold = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "kih")) { if (axValid && t2) { SERVO_LOCK(); axis[ax].kiHold = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "ff"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].ffGain = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "ffo")) { if (axValid && t2) { SERVO_LOCK(); axis[ax].ffOffset = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "md"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].maxDuty = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "mi"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].maxIntegral = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "mv"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].maxVel = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "ma"))  { if (axValid && t2) { SERVO_LOCK(); axis[ax].maxAccel = atof(t2); SERVO_UNLOCK(); } return; }
  if (!strcasecmp(cmd, "et"))  { if (axValid && t2) faultThreshold[ax] = atof(t2); return; }

  Con.println(F("?  (h for help)"));
}

static void pollSerial() {
  static char buf[128];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (n) { buf[n] = 0; handleLineFull(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) buf[n++] = ch;
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  critical_section_init(&servoLock);   // before anything touches axis[] under SERVO_LOCK
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  for (int a = 0; a < 2; a++) motorPwmBegin(a);   // both DRV8871 inputs low = coast
  adc_init();                                      // motor supply monitor on GP28 / ADC2
  adc_gpio_init(SUPPLY_ADC_PIN);
  adc_select_input(SUPPLY_ADC_INPUT);

  if (!pio_can_add_program_at_offset(encPio, &quadrature_encoder_program, 0)) {
    Con.println(F("ERROR: PIO2 has no room at offset 0")); while (true) delay(1000);
  }
  pio_add_program_at_offset(encPio, &quadrature_encoder_program, 0);
  encoderBegin(0);
  encoderBegin(1);

  double ffGain = MEASURED_SEED_DUTY / MEASURED_SEED_CPS;
  for (int a = 0; a < 2; a++) {
    axis[a].ffGain         = ffGain;
    axis[a].kp             = 0.15;  // moving gains: safe default until 'auto pi' is run
    axis[a].ki             = 0.05;
    axis[a].maxIntegral    = 40.0;
    axis[a].kpHold         = 0.15;  // hold gains: deliberately left gentle always
    axis[a].kiHold         = 0.05;
    axis[a].maxIntegralHold= 40.0;
    axis[a].maxDuty        = 40.0;
    axis[a].maxVel         = 5.0 * SIDEREAL_CPS;
    axis[a].maxAccel       = 100.0;
    axis[a].mode           = MODE_IDLE;
  }

  // saved calibration (if any) overrides the defaults above; motors are still off here
  fsReady = LittleFS.begin();
  bool loaded = false;
  if (!fsReady) Con.println(F("WARNING: no LittleFS -- set Tools > Flash Size to an option with FS; calibration won't persist"));
  else loaded = loadCalib(true);

  loadDirChecked();
  loadSky();
  resetModel(nullptr);
  restorePark();       // may set zeroOffset -- must happen before the servo starts
  loadWifi();

  core0Ready = true;   // core 1 starts the 1 kHz servo now (setup1)

  Con.println(F("\nTelescope servo + auto-tune ready (motors DISABLED). Type 'h' for help."));
  Con.printf("sidereal=%.4f counts/s (%.3f arcsec/count)\n", SIDEREAL_CPS, 1296000.0 / (TOTAL_RATIO * CPR));
  for (int a = 0; a < 2; a++)
    Con.printf("axis %d: ff=%.4f  moving kp=%.4f ki=%.4f\n", a, axis[a].ffGain, axis[a].kp, axis[a].ki);
  Con.println(F("Hardware: DRV8871 drivers. Az enc GP6/7 motor GP12/13, Alt enc GP26/27 motor GP20/21."));
  if (!dirChecked[0] || !dirChecked[1])
    Con.printf("Motor direction NOT verified (axis 0 %s, axis 1 %s) -- run 'auto dir a' before anything else.\n",
               dirChecked[0] ? "ok" : "unverified", dirChecked[1] ? "ok" : "unverified");
  if (loaded) Con.println(F("Using SAVED calibration. Re-run 'auto full' any time to recalibrate."));
  else Con.println(F(
    "Using built-in DEFAULT calibration (nothing saved yet).\n"
    "Run 'auto full' to tune both axes, apply and save in one go.\n"
    "Hold gains are never auto-suggested -- see the chat writeup on why."));
  if (!sky.siteSet) Con.println(F("Site not set: 'site <lat> <lon>' (decimal degrees, east +)."));
  Con.printf("Mount: %s. 'where' shows time, site and pointing.\n", sky.mode == coord::MOUNT_EQ ? "equatorial" : "alt-az");
  if (haveWifiCreds) wifiConnect();
  else Con.println(F("WiFi not configured: wifi \"<ssid>\" \"<password>\""));
  lastReportMs = millis();
}

void loop() {
  pollSerial();
  wifiUpdate();
  pollLx200();
  pollTelnet();

  for (int a = 0; a < 2; a++) {
    updateHandback(a);
    updateFFTune(a);
    updateRelayTune(a);
    updateSpeedTune(a);
    updateDirTest(a);
  }
  updateFullTune();
  skyUpdate();

  if (capState == CAP_DONE) {
    static int reported = -1;
    if (reported != capN || capArmedMs) { capArmedMs = 0; reported = capN; capReport(); }
  } else if (capState == CAP_ARMED && millis() - capArmedMs > 60000) {
    capState = CAP_IDLE;
    Con.println(F("cap: no move within 60 s -- disarmed"));
  } else if (capState == CAP_RUN && (faulted[capAxis] || !motorsEnabled[capAxis])) {
    Con.printf("cap: axis stopped during the capture -- analysing the %d samples taken\n", capN);
    capState = CAP_DONE;
  }

  for (int a = 0; a < 2; a++) {
    if (faultInfoNew[a]) {
      faultInfoNew[a] = false;
      printFaultInfo(a);
    }
    if (faulted[a]) {
      static uint32_t lastWarn[2] = {0, 0};
      if (millis() - lastWarn[a] > 2000) {
        Con.printf("axis %d FAULT: following error exceeded %.1f counts ('d %d' then 'e %d' to clear)\n",
                      a, faultThreshold[a], a, a);
        lastWarn[a] = millis();
      }
    }
  }

  if (streaming && millis() - lastReportMs >= 500) { report(); lastReportMs = millis(); }
}

// ---------------------------------------------------------------- core 1: the servo
// The alarm pool is created here so its interrupt is registered on core 1; the
// 1 kHz servo then never competes with WiFi, NTP or the servers on core 0.
// (LittleFS writes still pause core 1 for a few ms -- the core does that for
// flash safety; saves only happen while the axes hold.)
void setup1() {
  while (!core0Ready) tight_loop_contents();
  servoPool = alarm_pool_create_with_unused_hardware_alarm(4);
  alarm_pool_add_repeating_timer_us(servoPool, -1000, servoISR, NULL, &servoTimer);
}

void loop1() {
  tight_loop_contents();
}
