/*
 * Telescope controller - bring-up sketch #2: closed-loop position servo
 * Board:  Raspberry Pi Pico 2 W  (arduino-pico core, RP2350)
 *
 * Builds on sketch #1 (encoder decode + open-loop drive). Adds:
 *   - a 1 kHz position servo per axis (feedforward + PI on position error)
 *   - a trapezoidal slew profile for "goto"
 *   - basic fault detection (following error too large -> auto-stop)
 * Still core 0 only, no WiFi. servo_core.h holds the pure math, already
 * bench-tested in a standalone C++ simulation against a motor model with
 * stiction and lag -- see the chat writeup for what that found.
 *
 * WIRING - unchanged from sketch #1:
 *   Encoder axis 0 (RA):  A -> GP2,  B -> GP3   (through 1k/2k dividers from 5V)
 *   Encoder axis 1 (Dec): A -> GP4,  B -> GP5
 *   TB6612 axis 0 (RA):   PWMA GP10, AIN1 GP11, AIN2 GP12
 *   TB6612 axis 1 (Dec):  PWMB GP13, BIN1 GP14, BIN2 GP15
 *   TB6612 STBY GP16
 *
 * MEASURED (from the bench tests in chat):
 *   CPR = 108 (confirmed quadrature counts per motor rev)
 *   Dec motor needed direction inversion so + duty gives + counts
 *   Both axes, both directions: 7% duty ~= sidereal rate
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
static const uint32_t MAX_STEP_RATE  = 100000;   // PIO clock budget, counts/s
static const double   SERVO_DT       = 0.001;    // 1 kHz
static bool           invertMotor[2] = {false, true}; // RA: no invert, Dec: inverted (measured)

// ---------------------------------------------------------------- mechanics
static const double GEARBOX_RATIO  = 6859.0 / 128.0;              // 53.5859375
static const double WORM_RATIO     = 154.0;
static const double TOTAL_RATIO    = GEARBOX_RATIO * WORM_RATIO;  // 8252.234375
static const double CPR            = 108.0;                      // measured
static const double SIDEREAL_DAY_S = 86164.0905;
static const double SIDEREAL_CPS   = CPR * TOTAL_RATIO / SIDEREAL_DAY_S; // ~10.345 counts/s
static const double MEASURED_SIDEREAL_DUTY = 7.0; // % duty that gave ~sidereal rate, both axes/directions

// ---------------------------------------------------------------- runtime state
static PIO      encPio = pio2;             // leave PIO0/1 free for the WiFi driver
static AxisCtrl axis[2];
static int32_t  zeroOffset[2] = {0, 0};
static uint8_t  pwmSlice[2], pwmChan[2];
static bool     motorsEnabled[2] = {false, false};
static volatile bool faulted[2]  = {false, false};
static float    faultThreshold[2] = {200.0f, 200.0f};   // counts of following error before auto-stop
static repeating_timer_t servoTimer;

// values written by the ISR, read (with interrupts briefly held off) by loop()
static volatile int32_t posSnap[2];
static volatile double  errSnap[2], targetSnap[2], dutySnap[2];
static volatile uint32_t isrTicks = 0;

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

// ---------------------------------------------------------------- motor output
// Called only from the 1 kHz ISR. pct is already clamped by servoStep().
static void applyDuty(int a, double pct) {
  if (!motorsEnabled[a] || faulted[a]) pct = 0;
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

// ---------------------------------------------------------------- 1 kHz servo ISR
static bool servoISR(repeating_timer_t *) {
  for (int a = 0; a < 2; a++) {
    int32_t pos = countOf(a);
    double duty = servoStep(axis[a], (double)pos, SERVO_DT);

    if (motorsEnabled[a] && !faulted[a] && fabs(axis[a].lastErr) > faultThreshold[a]) {
      faulted[a] = true;
      duty = 0;
    }
    applyDuty(a, duty);

    posSnap[a]    = pos;
    errSnap[a]    = axis[a].lastErr;
    targetSnap[a] = axis[a].targetPos;
    dutySnap[a]   = duty;
  }
  isrTicks++;
  return true; // keep repeating
}

// ---------------------------------------------------------------- axis mode helpers (call only from loop(), not the ISR)
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
  noInterrupts();
  axis[a].mode = MODE_IDLE;
  axis[a].targetVel = 0;
  interrupts();
  if (!motorsEnabled[0] && !motorsEnabled[1]) gpio_put(STBY_PIN, 0);
}
static void startTrack(int a, double countsPerSec) {
  if (!motorsEnabled[a]) { Serial.println(F("axis disabled, ignoring")); return; }
  noInterrupts();
  axis[a].targetVel = countsPerSec;
  axis[a].mode = MODE_TRACK;
  interrupts();
}
static void startSlew(int a, double goalCounts) {
  if (!motorsEnabled[a]) { Serial.println(F("axis disabled, ignoring")); return; }
  noInterrupts();
  axis[a].goalPos = goalCounts;
  axis[a].mode = MODE_SLEW;
  interrupts();
}
static void holdHere(int a) {
  if (!motorsEnabled[a]) return;
  noInterrupts();
  axis[a].targetPos = (double)posSnap[a];
  axis[a].targetVel = 0;
  axis[a].mode = MODE_HOLD;
  interrupts();
}

// ---------------------------------------------------------------- serial UI
static bool streaming = false;
static uint32_t lastReportMs = 0;

static void printHelp() {
  Serial.println(F(
    "\ncommands:\n"
    "  h                 this help\n"
    "  s                 toggle 2 Hz status stream\n"
    "  e <axis|a>        enable axis (0,1) or 'a' for both -- zeros the servo target to current pos\n"
    "  d <axis|a>        disable axis (coasts, clears fault)\n"
    "  z <axis|a>        zero the encoder count (axis must be disabled first)\n"
    "  g <axis> <counts> slew to an absolute encoder count via the trapezoidal profile\n"
    "  gd <axis> <delta> slew by a relative count\n"
    "  t <axis> sid      track at sidereal rate (use 't <axis> -sid' for the other direction)\n"
    "  t <axis> <c/s>    track at an arbitrary signed counts/s rate (0 = hold)\n"
    "  o <axis>          stop tracking/slewing and hold the current position\n"
    "  kp/ki/ff/md/mi <axis> <val>   tune gains (position gain, integral gain, feedforward, max duty%, max integral)\n"
    "  mv/ma <axis> <val>            tune slew max velocity (counts/s) / max accel (counts/s^2)\n"
    "  et <axis> <val>   set the following-error fault threshold, in counts\n"
    "  p                 print all axis parameters and state once\n"
    "  x                 emergency stop: disable both axes now\n"));
}

static void printStatus(int a) {
  AxisCtrl &x = axis[a];
  const char *modeStr = x.mode==MODE_IDLE?"IDLE":x.mode==MODE_TRACK?"TRACK":x.mode==MODE_SLEW?"SLEW":"HOLD";
  Serial.printf("axis %d: %s%s en=%d mode=%-5s pos=%9ld target=%10.2f err=%7.3f duty=%6.2f%%\n"
                "         kp=%.4f ki=%.4f ff=%.4f maxDuty=%.1f maxI=%.1f maxVel=%.2f maxAccel=%.2f faultAt=%.1f\n",
                a, invertMotor[a] ? "(inv) " : "      ", faulted[a] ? "FAULT " : "",
                motorsEnabled[a], modeStr, (long)posSnap[a], targetSnap[a], errSnap[a], dutySnap[a],
                x.kp, x.ki, x.ffGain, x.maxDuty, x.maxIntegral, x.maxVel, x.maxAccel, faultThreshold[a]);
}

static void report() {
  Serial.printf("t=%8.2f ", millis() / 1000.0);
  for (int a = 0; a < 2; a++)
    Serial.printf("| A%d %s%s pos=%9ld tgt=%10.2f err=%7.3f duty=%6.2f%% ",
                  a, motorsEnabled[a] ? "EN " : "off", faulted[a] ? "!FAULT" : "",
                  (long)posSnap[a], targetSnap[a], errSnap[a], dutySnap[a]);
  Serial.println();
}

static void forEachAxis(const char *sel, void (*fn)(int)) {
  if (sel && (sel[0] == 'a' || sel[0] == 'A')) { fn(0); fn(1); }
  else if (sel) { int a = atoi(sel); if (a == 0 || a == 1) fn(a); }
}

static void handleLine(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *t1 = strtok(NULL, " \t");
  char *t2 = strtok(NULL, " \t");
  int ax = t1 ? atoi(t1) : -1;
  bool axValid = (t1 && (t1[0]=='0' || t1[0]=='1'));

  switch (tolower(cmd[0])) {
    case 'h': printHelp(); break;
    case 's': streaming = !streaming; lastReportMs = millis(); break;
    case 'e': forEachAxis(t1, enableAxis); break;
    case 'd': forEachAxis(t1, disableAxis); break;
    case 'z':
      for (int a = 0; a < 2; a++) {
        if (t1 && !(t1[0]=='a') && ax != a) continue;
        if (motorsEnabled[a]) { Serial.printf("axis %d must be disabled before zeroing\n", a); continue; }
        zeroOffset[a] += readRaw(a);
        noInterrupts(); axis[a].targetPos = 0; interrupts();
        Serial.printf("axis %d zeroed\n", a);
      }
      break;
    case 'g':
      if (cmd[1] == 'd' && axValid && t2) startSlew(ax, (double)posSnap[ax] + atof(t2));
      else if (axValid && t2) startSlew(ax, atof(t2));
      else Serial.println(F("usage: g <axis> <counts>  |  gd <axis> <delta>"));
      break;
    case 't':
      if (axValid && t2) {
        double rate;
        if (!strcasecmp(t2, "sid"))       rate =  SIDEREAL_CPS;
        else if (!strcasecmp(t2, "-sid")) rate = -SIDEREAL_CPS;
        else                              rate = atof(t2);
        startTrack(ax, rate);
        Serial.printf("axis %d tracking at %.3f counts/s\n", ax, rate);
      } else Serial.println(F("usage: t <axis> sid|-sid|<counts/s>"));
      break;
    case 'o': if (axValid) holdHere(ax); break;
    case 'p':
      for (int a = 0; a < 2; a++) printStatus(a);
      Serial.printf("sidereal = %.4f counts/s, %.3f arcsec/count\n",
                    SIDEREAL_CPS, 1296000.0 / (TOTAL_RATIO * CPR));
      break;
    case 'x': disableAxis(0); disableAxis(1); Serial.println(F("EMERGENCY STOP")); break;
    case 'k': if (cmd[1]=='p' && axValid && t2) axis[ax].kp = atof(t2); break;
    case 'i': if (cmd[1]=='i' && axValid && t2) axis[ax].ki = atof(t2); break;
    case 'f': if (cmd[0]=='f' && axValid && t2) axis[ax].ffGain = atof(t2); break;
    case 'm':
      if (cmd[1]=='d' && axValid && t2) axis[ax].maxDuty = atof(t2);
      else if (cmd[1]=='i' && axValid && t2) axis[ax].maxIntegral = atof(t2);
      else if (cmd[1]=='v' && axValid && t2) axis[ax].maxVel = atof(t2);
      else if (cmd[1]=='a' && axValid && t2) axis[ax].maxAccel = atof(t2);
      break;
    default: Serial.println(F("?  (h for help)"));
  }
}

static void pollSerial() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (n) { buf[n] = 0; handleLine(buf); n = 0; }
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
    analogWrite(PWM_PIN[a], 0);              // locks in freq/resolution on this slice
    pwmSlice[a] = pwm_gpio_to_slice_num(PWM_PIN[a]);
    pwmChan[a]  = pwm_gpio_to_channel(PWM_PIN[a]);
  }

  if (!pio_can_add_program_at_offset(encPio, &quadrature_encoder_program, 0)) {
    Serial.println(F("ERROR: PIO2 has no room at offset 0")); while (true) delay(1000);
  }
  pio_add_program_at_offset(encPio, &quadrature_encoder_program, 0);
  encoderBegin(0);
  encoderBegin(1);

  // --- calibration from the bench tests ---
  double ffGain = MEASURED_SIDEREAL_DUTY / SIDEREAL_CPS;   // % duty per (count/s)
  for (int a = 0; a < 2; a++) {
    axis[a].ffGain      = ffGain;
    axis[a].kp          = 0.15;   // duty% per count of error
    axis[a].ki          = 0.05;   // duty% per (count*s) of accumulated error
    axis[a].maxIntegral = 40.0;   // keep modest: see the stiction/hunting note below
    axis[a].maxDuty     = 40.0;
    axis[a].maxVel      = 5.0 * SIDEREAL_CPS;  // conservative first goto speed, ~51.7 counts/s
    axis[a].maxAccel    = 100.0;               // counts/s^2
    axis[a].mode        = MODE_IDLE;
  }

  add_repeating_timer_us(-1000, servoISR, NULL, &servoTimer);

  Serial.println(F("\nTelescope servo ready (motors DISABLED). Type 'h' for help."));
  Serial.printf("ffGain=%.4f%%/(count/s), sidereal=%.4f counts/s (%.3f arcsec/count)\n",
                ffGain, SIDEREAL_CPS, 1296000.0 / (TOTAL_RATIO * CPR));
  Serial.println(F(
    "NOTE: gains above give ~0.2 count steady tracking error and a few counts\n"
    "of static droop after a slew, in simulation against a plausible motor model.\n"
    "Raising maxIntegral to remove that droop caused hunting against stiction in\n"
    "simulation -- don't just crank ki/maxIntegral up on the bench without watching\n"
    "for that. A deadband 'kick' strategy is the proper fix; ask if you want it."));
  lastReportMs = millis();
}

void loop() {
  pollSerial();

  for (int a = 0; a < 2; a++) {
    if (faulted[a]) {
      static uint32_t lastWarn[2] = {0, 0};
      if (millis() - lastWarn[a] > 2000) {
        Serial.printf("axis %d FAULT: following error exceeded %.1f counts, motor disabled ('d %d' then 'e %d' to clear)\n",
                      a, faultThreshold[a], a, a);
        lastWarn[a] = millis();
      }
    }
  }

  if (streaming && millis() - lastReportMs >= 500) { report(); lastReportMs = millis(); }
}
