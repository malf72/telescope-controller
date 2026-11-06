/*
 * Telescope controller - bring-up sketch #3: gain-scheduled servo + auto-tune
 * Board:  Raspberry Pi Pico 2 W  (arduino-pico core, RP2350)
 *
 * Builds on sketch #2 (1 kHz feedforward+PI servo, trapezoidal slew). Adds:
 *   - gain scheduling: aggressive PI while the target is MOVING (TRACK/SLEW),
 *     gentle PI while HOLDing a fixed position. Simulation showed aggressive
 *     gains hunt against motor stiction once the setpoint stops moving --
 *     see servo_core.h and the chat writeup for the numbers.
 *   - 'auto ff <axis>'    : automates the open-loop duty-ramp calibration
 *   - 'auto pi <axis> [d]': relay (Astrom-Hagglund) auto-tune for kp/ki
 *   - 'auto speed <axis>' : open-loop step test to suggest mv/ma
 *   - 'auto apply <axis> <ff|tl|zn|speed>' commits a suggested result
 * All three tests take the axis over completely (open loop) while running;
 * the 1 kHz ISR skips any axis with testMode[] set, so there's no fight
 * between the servo and the test for control of the motor.
 *
 * WIRING - unchanged from sketches #1/#2:
 *   Encoder axis 0 (RA):  A -> GP2,  B -> GP3   (through 1k/2k dividers from 5V)
 *   Encoder axis 1 (Dec): A -> GP4,  B -> GP5
 *   TB6612 axis 0 (RA):   PWMA GP10, AIN1 GP11, AIN2 GP12
 *   TB6612 axis 1 (Dec):  PWMB GP13, BIN1 GP14, BIN2 GP15
 *   TB6612 STBY GP16
 */
#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/time.h"
#include "quadrature_encoder.pio.h"
#include "servo_core.h"

#if NUM_PIOS < 3
#error "Select the Raspberry Pi Pico 2 W board (RP2350 has 3 PIO blocks)."
#endif

// ---------------------------------------------------------------- pins
static const uint    ENC_PIN[2]  = {2, 4};
static const uint8_t PWM_PIN[2]  = {10, 13};
static const uint8_t IN1_PIN[2]  = {11, 14};
static const uint8_t IN2_PIN[2]  = {12, 15};
static const uint8_t STBY_PIN    = 16;

// ---------------------------------------------------------------- settings
static const bool     USE_INTERNAL_PULLUPS = false;
static const uint32_t MAX_STEP_RATE  = 100000;
static const double   SERVO_DT       = 0.001;
static bool           invertMotor[2] = {false, true}; // RA: no invert, Dec: inverted (measured)

// ---------------------------------------------------------------- mechanics
static const double GEARBOX_RATIO  = 6859.0 / 128.0;
static const double WORM_RATIO     = 154.0;
static const double TOTAL_RATIO    = GEARBOX_RATIO * WORM_RATIO;  // 8252.234375
static const double CPR            = 108.0;
static const double SIDEREAL_DAY_S = 86164.0905;
static const double SIDEREAL_CPS   = CPR * TOTAL_RATIO / SIDEREAL_DAY_S; // ~10.345 counts/s
static const double MEASURED_SIDEREAL_DUTY = 7.0; // % duty at sidereal rate, measured on the bench

// ---------------------------------------------------------------- runtime state
static PIO      encPio = pio2;
static AxisCtrl axis[2];
static int32_t  zeroOffset[2] = {0, 0};
static uint8_t  pwmSlice[2], pwmChan[2];
static bool     motorsEnabled[2] = {false, false};
static volatile bool faulted[2]  = {false, false};
static float    faultThreshold[2] = {200.0f, 200.0f};
static repeating_timer_t servoTimer;
static volatile bool testMode[2] = {false, false};  // true = loop() owns this axis's hardware, ISR hands off

static volatile int32_t posSnap[2];
static volatile double  errSnap[2], targetSnap[2], dutySnap[2];

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

// ---------------------------------------------------------------- raw motor hardware
// Direct register writes -- safe to call from the ISR or from loop(), never both
// for the same axis at the same time (testMode[] arbitrates which one is allowed to).
static void writeMotorHardware(int a, double pct) {
  if (pct == 0.0) {
    gpio_put(IN1_PIN[a], 0); gpio_put(IN2_PIN[a], 0);
    pwm_set_chan_level(pwmSlice[a], pwmChan[a], 0);
    return;
  }
  bool fwd = (pct > 0) != invertMotor[a];
  gpio_put(IN1_PIN[a], fwd ? 1 : 0);
  gpio_put(IN2_PIN[a], fwd ? 0 : 1);
  uint16_t level = (uint16_t)(fabs(pct) * 1023.0 / 100.0 + 0.5);
  pwm_set_chan_level(pwmSlice[a], pwmChan[a], level);
}

// Hand an axis back to the normal servo, holding wherever it currently is.
// Called from loop() at the end of any auto-tune procedure.
static void returnToNormalServo(int a) {
  writeMotorHardware(a, 0);
  int32_t pos = countOf(a);
  noInterrupts();
  axis[a].targetPos = (double)pos;
  axis[a].targetVel = 0;
  axis[a].integral  = 0;
  axis[a].mode = MODE_HOLD;
  interrupts();
  testMode[a] = false;
}

// ---------------------------------------------------------------- 1 kHz servo ISR
static bool servoISR(repeating_timer_t *) {
  for (int a = 0; a < 2; a++) {
    if (testMode[a]) continue;  // loop() owns this axis right now (an auto-tune test is running)
    int32_t pos = countOf(a);
    double duty = servoStep(axis[a], (double)pos, SERVO_DT);

    if (motorsEnabled[a] && !faulted[a] && fabs(axis[a].lastErr) > faultThreshold[a]) {
      faulted[a] = true;
      duty = 0;
    }
    if (!motorsEnabled[a] || faulted[a]) duty = 0;
    writeMotorHardware(a, duty);

    posSnap[a]    = pos;
    errSnap[a]    = axis[a].lastErr;
    targetSnap[a] = axis[a].targetPos;
    dutySnap[a]   = duty;
  }
  return true;
}

// ---------------------------------------------------------------- axis mode helpers (loop() only)
static void enableAxis(int a) {
  noInterrupts();
  axis[a].targetPos = (double)countOf(a);
  axis[a].targetVel = 0;
  axis[a].integral  = 0;
  axis[a].mode = MODE_HOLD;
  faulted[a] = false;
  interrupts();
  motorsEnabled[a] = true;
  gpio_put(STBY_PIN, 1);
}
static void disableAxis(int a) {
  motorsEnabled[a] = false;
  testMode[a] = false;
  noInterrupts();
  axis[a].mode = MODE_IDLE;
  axis[a].targetVel = 0;
  interrupts();
  writeMotorHardware(a, 0);
  if (!motorsEnabled[0] && !motorsEnabled[1]) gpio_put(STBY_PIN, 0);
}
static void startTrack(int a, double countsPerSec) {
  if (!motorsEnabled[a] || testMode[a]) { Serial.println(F("axis disabled or busy, ignoring")); return; }
  noInterrupts();
  axis[a].targetVel = countsPerSec;
  axis[a].mode = MODE_TRACK;
  interrupts();
}
static void startSlew(int a, double goalCounts) {
  if (!motorsEnabled[a] || testMode[a]) { Serial.println(F("axis disabled or busy, ignoring")); return; }
  noInterrupts();
  axis[a].goalPos = goalCounts;
  axis[a].mode = MODE_SLEW;
  interrupts();
}
static void holdHere(int a) {
  if (!motorsEnabled[a] || testMode[a]) return;
  noInterrupts();
  axis[a].targetPos = (double)posSnap[a];
  axis[a].targetVel = 0;
  axis[a].integral  = 0;
  axis[a].mode = MODE_HOLD;
  interrupts();
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
};
static FFTune fftune[2];
static double lastTuneFF[2]; static bool haveFFResult[2] = {false, false};

static const double FF_TARGET_RATE = SIDEREAL_CPS; // calibrate at the same speed we'll actually track at
static const double FF_STEP_PCT    = 0.5;
static const uint32_t FF_STEP_MS   = 1500;
static const double FF_MAX_DUTY    = 20.0;         // safety cap while probing -- well above the ~7% baseline

static void startFFTune(int a) {
  if (!motorsEnabled[a]) { Serial.println(F("enable axis first ('e <axis>')")); return; }
  fftune[a] = FFTune();
  fftune[a].active = true;
  fftune[a].stepStartMs = millis();
  fftune[a].stepStartCnt = countOf(a);
  testMode[a] = true;
  writeMotorHardware(a, 0);
  Serial.printf("axis %d: ff auto-tune started, positive direction (keep clear, motor will move)\n", a);
}

static void updateFFTune(int a) {
  FFTune &f = fftune[a];
  if (!f.active) return;
  uint32_t now = millis();
  if (now - f.stepStartMs < FF_STEP_MS) return;

  int32_t cnt = countOf(a);
  double dtS = (now - f.stepStartMs) / 1000.0;
  double rate = (cnt - f.stepStartCnt) / dtS;
  double dir = (f.phase == 0) ? 1.0 : -1.0;

  if (fabs(rate) >= FF_TARGET_RATE || f.duty >= FF_MAX_DUTY) {
    // scale the duty that was actually used down/up to the exact target rate,
    // assuming a roughly linear duty-to-speed relationship near this point
    double resultDuty = (fabs(rate) > 1e-6) ? f.duty * (FF_TARGET_RATE / fabs(rate)) : f.duty;
    if (f.phase == 0) f.resultPos = resultDuty; else f.resultNeg = resultDuty;
    Serial.printf("axis %d: %s direction reached %.2f counts/s at %.2f%% duty (scaled to %.2f%% at target rate)\n",
                  a, f.phase == 0 ? "positive" : "negative", rate, f.duty, resultDuty);

    if (f.phase == 0) {
      f.phase = 1;
      f.duty = 0;
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
      Serial.printf("axis %d: ff auto-tune done. +dir=%.2f%%, -dir=%.2f%%, asymmetry=%.1f%%\n",
                    a, f.resultPos, f.resultNeg, asym);
      Serial.printf("  suggested ff = %.4f %%/(count/s).  Apply with 'auto apply %d ff'.\n", a, avgFF);
      if (asym > 15.0)
        Serial.println(F("  NOTE: >15% asymmetry between directions -- worth a look (preload, wiring, binding)."));
    }
    return;
  }
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
static double lastKpTL[2], lastKiTL[2], lastKpZN[2], lastKiZN[2];
static bool havePIResult[2] = {false, false};

static void startRelayTune(int a, double duty, double hysteresis) {
  if (!motorsEnabled[a]) { Serial.println(F("enable axis first ('e <axis>')")); return; }
  relay[a] = RelayTune();
  relay[a].active = true;
  relay[a].d = duty;
  relay[a].h = hysteresis;
  relay[a].startCnt = countOf(a);
  relay[a].relayHigh = true;
  relay[a].runningExtreme = -1e9;
  relay[a].startMs = millis();
  testMode[a] = true;
  writeMotorHardware(a, relay[a].d);
  Serial.printf("axis %d: relay auto-tune started (d=%.1f%%, h=%.1f counts, keep clear)\n", a, duty, hysteresis);
}

static void updateRelayTune(int a) {
  RelayTune &r = relay[a];
  if (!r.active) return;
  int32_t pos = countOf(a);
  double err = (double)(r.startCnt) + r.targetOffset - pos;
  double t = (millis() - r.startMs) / 1000.0;
  if (fabs(err) > r.maxAbsErr) r.maxAbsErr = fabs(err);

  if (t > 15.0 || fabs(err) > 150.0) {
    r.active = false;
    returnToNormalServo(a);
    Serial.printf("axis %d: relay auto-tune ABORTED (t=%.1fs, err=%.1f counts) -- check enable/wiring/direction\n",
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

    Serial.printf("axis %d: relay auto-tune done. Ku=%.3f Tu=%.3fs maxErr=%.1f counts\n", a, Ku, Tu, r.maxAbsErr);
    Serial.printf("  Tyreus-Luyben (gentler):    kp=%.4f ki=%.4f   -- apply with 'auto apply %d tl'\n", kp_tl, ki_tl, a);
    Serial.printf("  Ziegler-Nichols (sharper):  kp=%.4f ki=%.4f   -- apply with 'auto apply %d zn'\n", kp_zn, ki_zn, a);
    Serial.println(F("  These become the MOVING (track/slew) gains only -- hold gains are untouched on purpose."));
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
static bool haveSpeedResult[2] = {false, false};

static const uint32_t SPEED_SAMPLE_PERIOD_MS = 100;
static const uint32_t SPEED_TOTAL_MS = 2400; // NSAMP * SPEED_SAMPLE_PERIOD_MS

static void startSpeedTune(int a) {
  if (!motorsEnabled[a]) { Serial.println(F("enable axis first ('e <axis>')")); return; }
  speedtune[a] = SpeedTune();
  speedtune[a].active = true;
  speedtune[a].startMs = millis();
  speedtune[a].startCnt = countOf(a);
  speedtune[a].lastSampMs = speedtune[a].startMs;
  testMode[a] = true;
  writeMotorHardware(a, axis[a].maxDuty);
  Serial.printf("axis %d: speed auto-tune started, stepping to md=%.1f%% for %.1fs (keep clear)\n",
                a, axis[a].maxDuty, SPEED_TOTAL_MS / 1000.0);
}

static void updateSpeedTune(int a) {
  SpeedTune &s = speedtune[a];
  if (!s.active) return;
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

    if (s.nSamp < 6) { Serial.println(F("speed auto-tune: not enough samples, aborted")); return; }
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
    haveSpeedResult[a] = true;

    Serial.printf("axis %d: speed auto-tune done. plateau=%.2f counts/s at md=%.1f%%, rise-to-90%%=%.2fs\n",
                  a, plateauRate, axis[a].maxDuty, riseTime);
    Serial.printf("  suggested mv=%.2f counts/s, ma=%.2f counts/s^2  -- apply with 'auto apply %d speed'\n",
                  suggestedMaxVel, suggestedMaxAccel, a);
    Serial.println(F("  This is one open-loop step, not a rigorous ID -- treat as a starting point."));
  }
}

// ---------------------------------------------------------------- serial UI
static bool streaming = false;
static uint32_t lastReportMs = 0;

static void printHelp() {
  Serial.println(F(
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
    "  kp/ki <axis> <val>      tune the MOVING (track/slew) gains directly\n"
    "  kph/kih <axis> <val>    tune the HOLD gains directly\n"
    "  ff <axis> <val>         set feedforward gain directly\n"
    "  md/mi <axis> <val>      max duty %% / max integral (moving)\n"
    "  mv/ma <axis> <val>      max slew velocity (counts/s) / accel (counts/s^2)\n"
    "  et <axis> <val>         following-error fault threshold, counts\n"
    "\n"
    "auto-tune (axis must be enabled first, keep hands/cables clear):\n"
    "  auto ff <axis>              measure feedforward gain (open-loop ramp, both directions)\n"
    "  auto pi <axis> [duty] [h]   relay auto-tune for kp/ki (default duty=15%%, hysteresis=1.5 counts)\n"
    "  auto speed <axis>           open-loop step to md, suggest mv/ma\n"
    "  auto apply <axis> ff|tl|zn|speed   commit the last suggestion from that test\n"
    "  auto abort <axis>           stop any running auto-tune on that axis\n"));
}

static void printStatus(int a) {
  AxisCtrl &x = axis[a];
  const char *modeStr = x.mode==MODE_IDLE?"IDLE":x.mode==MODE_TRACK?"TRACK":x.mode==MODE_SLEW?"SLEW":"HOLD";
  Serial.printf("axis %d: %s%s%s en=%d mode=%-5s pos=%9ld target=%10.2f err=%7.3f duty=%6.2f%%\n"
                "  moving: kp=%.4f ki=%.4f maxI=%.1f   hold: kp=%.4f ki=%.4f maxI=%.1f\n"
                "  ff=%.4f maxDuty=%.1f maxVel=%.2f maxAccel=%.2f faultAt=%.1f\n",
                a, invertMotor[a] ? "(inv) " : "      ", faulted[a] ? "FAULT " : "",
                testMode[a] ? "TESTING " : "",
                motorsEnabled[a], modeStr, (long)posSnap[a], targetSnap[a], errSnap[a], dutySnap[a],
                x.kp, x.ki, x.maxIntegral, x.kpHold, x.kiHold, x.maxIntegralHold,
                x.ffGain, x.maxDuty, x.maxVel, x.maxAccel, faultThreshold[a]);
}

static void report() {
  Serial.printf("t=%8.2f ", millis() / 1000.0);
  for (int a = 0; a < 2; a++)
    Serial.printf("| A%d %s%s%s pos=%9ld tgt=%10.2f err=%7.3f duty=%6.2f%% ",
                  a, motorsEnabled[a] ? "EN " : "off", faulted[a] ? "!FAULT" : "",
                  testMode[a] ? "*TEST*" : "",
                  (long)posSnap[a], targetSnap[a], errSnap[a], dutySnap[a]);
  Serial.println();
}

static void forEachAxis(const char *sel, void (*fn)(int)) {
  if (sel && (sel[0] == 'a' || sel[0] == 'A')) { fn(0); fn(1); }
  else if (sel) { int a = atoi(sel); if (a == 0 || a == 1) fn(a); }
}
static void abortTune(int a) {
  fftune[a].active = false;
  relay[a].active = false;
  speedtune[a].active = false;
  if (testMode[a]) returnToNormalServo(a);
  Serial.printf("axis %d: any running auto-tune stopped\n", a);
}

static void handleAutoCmd(char *rest) {
  char *sub = strtok(rest, " \t");
  if (!sub) { Serial.println(F("usage: auto ff|pi|speed|apply|abort <axis> ...")); return; }
  char *t1 = strtok(NULL, " \t");
  char *t2 = strtok(NULL, " \t");
  char *t3 = strtok(NULL, " \t");
  bool axValid = (t1 && (t1[0]=='0' || t1[0]=='1'));
  int ax = axValid ? atoi(t1) : -1;

  if (!strcasecmp(sub, "ff")) {
    if (axValid) startFFTune(ax); else Serial.println(F("usage: auto ff <axis>"));
  } else if (!strcasecmp(sub, "pi")) {
    if (axValid) {
      double d = t2 ? atof(t2) : 15.0;
      double h = t3 ? atof(t3) : 1.5;
      startRelayTune(ax, d, h);
    } else Serial.println(F("usage: auto pi <axis> [duty%] [hysteresis_counts]"));
  } else if (!strcasecmp(sub, "speed")) {
    if (axValid) startSpeedTune(ax); else Serial.println(F("usage: auto speed <axis>"));
  } else if (!strcasecmp(sub, "abort")) {
    forEachAxis(t1, abortTune);
  } else if (!strcasecmp(sub, "apply")) {
    if (!axValid || !t2) { Serial.println(F("usage: auto apply <axis> ff|tl|zn|speed")); return; }
    if (!strcasecmp(t2, "ff")) {
      if (!haveFFResult[ax]) { Serial.println(F("no ff result yet")); return; }
      noInterrupts(); axis[ax].ffGain = lastTuneFF[ax]; interrupts();
      Serial.printf("axis %d: ff=%.4f applied\n", ax, lastTuneFF[ax]);
    } else if (!strcasecmp(t2, "tl")) {
      if (!havePIResult[ax]) { Serial.println(F("no pi result yet")); return; }
      noInterrupts(); axis[ax].kp = lastKpTL[ax]; axis[ax].ki = lastKiTL[ax]; interrupts();
      Serial.printf("axis %d: moving kp=%.4f ki=%.4f applied (Tyreus-Luyben)\n", ax, lastKpTL[ax], lastKiTL[ax]);
    } else if (!strcasecmp(t2, "zn")) {
      if (!havePIResult[ax]) { Serial.println(F("no pi result yet")); return; }
      noInterrupts(); axis[ax].kp = lastKpZN[ax]; axis[ax].ki = lastKiZN[ax]; interrupts();
      Serial.printf("axis %d: moving kp=%.4f ki=%.4f applied (Ziegler-Nichols)\n", ax, lastKpZN[ax], lastKiZN[ax]);
    } else if (!strcasecmp(t2, "speed")) {
      if (!haveSpeedResult[ax]) { Serial.println(F("no speed result yet")); return; }
      noInterrupts(); axis[ax].maxVel = lastSuggestMaxVel[ax]; axis[ax].maxAccel = lastSuggestMaxAccel[ax]; interrupts();
      Serial.printf("axis %d: mv=%.2f ma=%.2f applied\n", ax, lastSuggestMaxVel[ax], lastSuggestMaxAccel[ax]);
    } else Serial.println(F("usage: auto apply <axis> ff|tl|zn|speed"));
  } else {
    Serial.println(F("usage: auto ff|pi|speed|apply|abort <axis> ..."));
  }
}

// Line handler. "auto ..." is special-cased first (via a peeked copy of the
// line) so its sub-arguments survive intact through a separate strtok pass.
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

  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *t1 = strtok(NULL, " \t");
  char *t2 = strtok(NULL, " \t");
  int ax = t1 ? atoi(t1) : -1;
  bool axValid = (t1 && (t1[0]=='0' || t1[0]=='1'));

  if (!strcasecmp(cmd, "h")) { printHelp(); return; }
  if (!strcasecmp(cmd, "s")) { streaming = !streaming; lastReportMs = millis(); return; }
  if (!strcasecmp(cmd, "e")) { forEachAxis(t1, enableAxis); return; }
  if (!strcasecmp(cmd, "d")) { forEachAxis(t1, disableAxis); return; }
  if (!strcasecmp(cmd, "z")) {
    for (int a = 0; a < 2; a++) {
      if (t1 && t1[0] != 'a' && ax != a) continue;
      if (motorsEnabled[a]) { Serial.printf("axis %d must be disabled before zeroing\n", a); continue; }
      zeroOffset[a] += readRaw(a);
      noInterrupts(); axis[a].targetPos = 0; interrupts();
      Serial.printf("axis %d zeroed\n", a);
    }
    return;
  }
  if (!strcasecmp(cmd, "g"))  { if (axValid && t2) startSlew(ax, atof(t2)); else Serial.println(F("usage: g <axis> <counts>")); return; }
  if (!strcasecmp(cmd, "gd")) { if (axValid && t2) startSlew(ax, (double)posSnap[ax] + atof(t2)); else Serial.println(F("usage: gd <axis> <delta>")); return; }
  if (!strcasecmp(cmd, "t")) {
    if (axValid && t2) {
      double rate;
      if (!strcasecmp(t2, "sid")) rate = SIDEREAL_CPS;
      else if (!strcasecmp(t2, "-sid")) rate = -SIDEREAL_CPS;
      else rate = atof(t2);
      startTrack(ax, rate);
      Serial.printf("axis %d tracking at %.3f counts/s\n", ax, rate);
    } else Serial.println(F("usage: t <axis> sid|-sid|<counts/s>"));
    return;
  }
  if (!strcasecmp(cmd, "o")) { if (axValid) holdHere(ax); return; }
  if (!strcasecmp(cmd, "p")) {
    for (int a = 0; a < 2; a++) printStatus(a);
    Serial.printf("sidereal = %.4f counts/s, %.3f arcsec/count\n", SIDEREAL_CPS, 1296000.0 / (TOTAL_RATIO * CPR));
    return;
  }
  if (!strcasecmp(cmd, "x")) { disableAxis(0); disableAxis(1); Serial.println(F("EMERGENCY STOP")); return; }
  if (!strcasecmp(cmd, "kp"))  { if (axValid && t2) { noInterrupts(); axis[ax].kp = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "ki"))  { if (axValid && t2) { noInterrupts(); axis[ax].ki = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "kph")) { if (axValid && t2) { noInterrupts(); axis[ax].kpHold = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "kih")) { if (axValid && t2) { noInterrupts(); axis[ax].kiHold = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "ff"))  { if (axValid && t2) { noInterrupts(); axis[ax].ffGain = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "md"))  { if (axValid && t2) { noInterrupts(); axis[ax].maxDuty = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "mi"))  { if (axValid && t2) { noInterrupts(); axis[ax].maxIntegral = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "mv"))  { if (axValid && t2) { noInterrupts(); axis[ax].maxVel = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "ma"))  { if (axValid && t2) { noInterrupts(); axis[ax].maxAccel = atof(t2); interrupts(); } return; }
  if (!strcasecmp(cmd, "et"))  { if (axValid && t2) faultThreshold[ax] = atof(t2); return; }

  Serial.println(F("?  (h for help)"));
}

static void pollSerial() {
  static char buf[64];
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
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, LOW);
  for (int a = 0; a < 2; a++) {
    pinMode(IN1_PIN[a], OUTPUT); pinMode(IN2_PIN[a], OUTPUT); pinMode(PWM_PIN[a], OUTPUT);
    digitalWrite(IN1_PIN[a], LOW); digitalWrite(IN2_PIN[a], LOW);
  }
  analogWriteFreq(20000);
  analogWriteResolution(10);
  for (int a = 0; a < 2; a++) {
    analogWrite(PWM_PIN[a], 0);
    pwmSlice[a] = pwm_gpio_to_slice_num(PWM_PIN[a]);
    pwmChan[a]  = pwm_gpio_to_channel(PWM_PIN[a]);
  }

  if (!pio_can_add_program_at_offset(encPio, &quadrature_encoder_program, 0)) {
    Serial.println(F("ERROR: PIO2 has no room at offset 0")); while (true) delay(1000);
  }
  pio_add_program_at_offset(encPio, &quadrature_encoder_program, 0);
  encoderBegin(0);
  encoderBegin(1);

  double ffGain = MEASURED_SIDEREAL_DUTY / SIDEREAL_CPS;
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

  add_repeating_timer_us(-1000, servoISR, NULL, &servoTimer);

  Serial.println(F("\nTelescope servo + auto-tune ready (motors DISABLED). Type 'h' for help."));
  Serial.printf("ffGain=%.4f%%/(count/s), sidereal=%.4f counts/s (%.3f arcsec/count)\n",
                ffGain, SIDEREAL_CPS, 1296000.0 / (TOTAL_RATIO * CPR));
  Serial.println(F(
    "Suggested first run: 'e a', then 'auto ff 0', 'auto ff 1', then 'auto pi 0', 'auto pi 1',\n"
    "review the numbers printed, then 'auto apply <axis> ff' and 'auto apply <axis> tl' (or zn).\n"
    "Hold gains are never auto-suggested -- see the chat writeup on why."));
  lastReportMs = millis();
}

void loop() {
  pollSerial();

  for (int a = 0; a < 2; a++) {
    updateFFTune(a);
    updateRelayTune(a);
    updateSpeedTune(a);
  }

  for (int a = 0; a < 2; a++) {
    if (faulted[a]) {
      static uint32_t lastWarn[2] = {0, 0};
      if (millis() - lastWarn[a] > 2000) {
        Serial.printf("axis %d FAULT: following error exceeded %.1f counts ('d %d' then 'e %d' to clear)\n",
                      a, faultThreshold[a], a, a);
        lastWarn[a] = millis();
      }
    }
  }

  if (streaming && millis() - lastReportMs >= 500) { report(); lastReportMs = millis(); }
}
