/*
 * Telescope controller - bring-up sketch #1
 * Board:  Raspberry Pi Pico 2 W  (arduino-pico core by Earle Philhower, RP2350)
 *
 * Purpose: (1) decode both motor-shaft quadrature encoders in PIO,
 *          (2) drive both motors open-loop through a TB6612 board,
 *          (3) let you measure counts-per-revolution, direction, minimum
 *              starting duty and the duty that gives sidereal speed.
 * No closed-loop control, no WiFi yet. Everything runs on core 0.
 *
 * Open the Serial Monitor at 115200 baud, line ending "Newline", and type 'h'.
 *
 * WIRING (Pico GPIO numbers)
 *   Encoder axis 0 (RA):  A -> GP2,  B -> GP3     (through 1k/2k dividers from 5 V)
 *   Encoder axis 1 (Dec): A -> GP4,  B -> GP5     (A and B MUST be consecutive pins)
 *   TB6612 axis 0 (ch A): PWMA GP10, AIN1 GP11, AIN2 GP12
 *   TB6612 axis 1 (ch B): PWMB GP13, BIN1 GP14, BIN2 GP15
 *   TB6612 STBY GP16, VCC (logic) 3V3, VM = motor supply (<= 13 V), GNDs common.
 *   Encoder divider: 5 V signal -> 1k in series -> Pico pin -> 2k to GND.
 *   Encoder Vcc = 5 V, encoder GND = common ground.
 */
#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "quadrature_encoder.pio.h"

#if NUM_PIOS < 3
#error "Select the Raspberry Pi Pico 2 W board (RP2350 has 3 PIO blocks)."
#endif

// ---------------------------------------------------------------- pins
static const uint    ENC_PIN[2]  = {2, 4};     // A on this pin, B on pin+1
static const uint8_t PWM_PIN[2]  = {10, 13};
static const uint8_t IN1_PIN[2]  = {11, 14};
static const uint8_t IN2_PIN[2]  = {12, 15};
static const uint8_t STBY_PIN    = 16;

// ---------------------------------------------------------------- settings
static const bool   USE_INTERNAL_PULLUPS = false; // true only for open-collector encoders WITHOUT dividers
static const float  MAX_DUTY_PCT         = 40.0f; // hard cap while bench testing
static const uint32_t MOTOR_TIMEOUT_MS   = 10000; // motors auto-stop if not refreshed
static const uint32_t MAX_STEP_RATE      = 100000; // encoder counts/s the PIO clock is sized for

// ---------------------------------------------------------------- mechanics
static const double GEARBOX_RATIO  = 6859.0 / 128.0;              // 53.5859375
static const double WORM_RATIO     = 154.0;
static const double TOTAL_RATIO    = GEARBOX_RATIO * WORM_RATIO;  // motor revs per axis rev
static const double SIDEREAL_DAY_S = 86164.0905;

// ---------------------------------------------------------------- state
static PIO   encPio = pio2;              // keep PIO0/1 free for the WiFi driver
static float cpr = 108.0f;               // counts per MOTOR revolution (set with 'k')
static int32_t zeroOffset[2]   = {0, 0};
static float   dutyPct[2]      = {0, 0}; // signed, as commanded
static bool    invertMotor[2]  = {false, false};
static bool    streaming       = false;
static uint32_t lastCmdMs      = 0;
static uint32_t lastReportMs   = 0;
static int32_t  lastReportCnt[2] = {0, 0};

// duty-ramp helper ('f')
static bool     ramping = false;
static int      rampAxis = 0;
static float    rampDir = 1, rampDuty = 0;
static uint32_t rampStepMs = 0;
static int32_t  rampStartCnt = 0;

// ---------------------------------------------------------------- encoders
static int32_t readRaw(int ax) {
  // The PIO pushes the count continuously without blocking, so the FIFO can
  // hold stale values. Drain it, then wait for one guaranteed-fresh sample.
  int n = pio_sm_get_rx_fifo_level(encPio, ax) + 1;
  uint32_t v = 0;
  while (n-- > 0) v = pio_sm_get_blocking(encPio, ax);
  return (int32_t)v;
}
static int32_t countOf(int ax) { return readRaw(ax) - zeroOffset[ax]; }

static void encoderBegin(int ax) {
  uint pin = ENC_PIN[ax];
  pio_sm_claim(encPio, ax);
  pio_gpio_init(encPio, pin);
  pio_gpio_init(encPio, pin + 1);
  pio_sm_set_consecutive_pindirs(encPio, ax, pin, 2, false);
  if (USE_INTERNAL_PULLUPS) { gpio_pull_up(pin); gpio_pull_up(pin + 1); }
  else                      { gpio_disable_pulls(pin); gpio_disable_pulls(pin + 1); }

  pio_sm_config c = quadrature_encoder_program_get_default_config(0);
  sm_config_set_in_pins(&c, pin);
  sm_config_set_in_shift(&c, false, false, 32);   // shift left, no autopush
  sm_config_set_out_shift(&c, true, false, 32);   // shift right, no autopull
  // The program needs up to 14 cycles per sample; size the clock for MAX_STEP_RATE.
  sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / (MAX_STEP_RATE * 14.0f));
  pio_sm_init(encPio, ax, 0, &c);
  pio_sm_exec(encPio, ax, pio_encode_set(pio_y, 0));
  pio_sm_set_enabled(encPio, ax, true);
}

// ---------------------------------------------------------------- motors
static void driveMotor(int ax, float pct) {
  if (pct >  MAX_DUTY_PCT) pct =  MAX_DUTY_PCT;
  if (pct < -MAX_DUTY_PCT) pct = -MAX_DUTY_PCT;
  dutyPct[ax] = pct;
  if (pct == 0.0f) {                      // coast
    digitalWrite(IN1_PIN[ax], LOW);
    digitalWrite(IN2_PIN[ax], LOW);
    analogWrite(PWM_PIN[ax], 0);
    return;
  }
  bool fwd = (pct > 0) != invertMotor[ax];
  digitalWrite(IN1_PIN[ax], fwd ? HIGH : LOW);
  digitalWrite(IN2_PIN[ax], fwd ? LOW : HIGH);
  digitalWrite(STBY_PIN, HIGH);
  analogWrite(PWM_PIN[ax], (int)(fabsf(pct) * 1023.0f / 100.0f + 0.5f));
}
static void brakeAll() {
  ramping = false;
  for (int a = 0; a < 2; a++) {
    dutyPct[a] = 0;
    digitalWrite(IN1_PIN[a], HIGH);       // IN1 = IN2 = H : short brake
    digitalWrite(IN2_PIN[a], HIGH);
    analogWrite(PWM_PIN[a], 1023);
  }
}
static void coastAll() {                  // outputs off and driver in standby
  ramping = false;
  for (int a = 0; a < 2; a++) driveMotor(a, 0);
  digitalWrite(STBY_PIN, LOW);
}
static bool motorsActive() { return dutyPct[0] != 0 || dutyPct[1] != 0; }

// ---------------------------------------------------------------- helpers
static double siderealCountsPerSec() { return cpr * TOTAL_RATIO / SIDEREAL_DAY_S; }
static double arcsecPerCount()       { return 1296000.0 / (TOTAL_RATIO * cpr); }

static void printHelp() {
  Serial.println(F(
    "\nCommands (end with Enter):\n"
    "  h              this help\n"
    "  s              toggle 2 Hz status stream (counts, counts/s, duty)\n"
    "  z [axis]       zero the count (both axes if no axis given)\n"
    "  c              show how the current counts relate to CPR (turn shaft exactly 1 rev first)\n"
    "  k <cpr>        set counts per MOTOR rev used for the sidereal target (default 108)\n"
    "  m <axis> <pct> open-loop drive, signed percent, e.g.  m 0 12.5   (auto-stops after 10 s)\n"
    "  f <axis> <+1|-1>  ramp duty up slowly and print the response (stops at 1.5x sidereal or on x)\n"
    "  i <axis>       invert motor direction (so + duty gives + counts)\n"
    "  x              brake both motors\n"
    "  q              coast both motors and put the driver in standby\n"));
}

static void printCalibration() {
  Serial.println(F("--- counts vs CPR (rotate the motor shaft exactly ONE revolution after 'z') ---"));
  for (int a = 0; a < 2; a++) {
    long c = countOf(a);
    Serial.printf("axis %d: count=%ld  (expect +/-108 if 108 is quadrature counts, +/-432 if 108 is slots)\n", a, c);
  }
  Serial.printf("with cpr=%.1f: %.3f arcsec/count, sidereal = %.2f counts/s, axis rev = %.1f counts\n",
                cpr, arcsecPerCount(), siderealCountsPerSec(), cpr * TOTAL_RATIO);
}

static void report() {
  uint32_t now = millis();
  float dt = (now - lastReportMs) / 1000.0f;
  if (dt <= 0) return;
  Serial.printf("t=%7.1f ", now / 1000.0f);
  for (int a = 0; a < 2; a++) {
    int32_t c = countOf(a);
    Serial.printf("| A%d cnt=%9ld rate=%8.2f c/s duty=%6.1f%% ", a, (long)c,
                  (c - lastReportCnt[a]) / dt, dutyPct[a]);
    lastReportCnt[a] = c;
  }
  Serial.printf("| sid=%.2f c/s\n", siderealCountsPerSec());
  lastReportMs = now;
}

// ---------------------------------------------------------------- commands
static void handleLine(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *t1 = strtok(NULL, " \t");
  char *t2 = strtok(NULL, " \t");
  int ax = t1 ? atoi(t1) : -1;
  lastCmdMs = millis();

  switch (tolower(cmd[0])) {
    case 'h': printHelp(); break;
    case 's': streaming = !streaming; lastReportMs = millis();
              lastReportCnt[0] = countOf(0); lastReportCnt[1] = countOf(1); break;
    case 'z':
      for (int a = 0; a < 2; a++)
        if (!t1 || ax == a) { zeroOffset[a] = readRaw(a); lastReportCnt[a] = 0; }
      Serial.println(F("zeroed"));
      break;
    case 'c': printCalibration(); break;
    case 'k': if (t1) { cpr = atof(t1); Serial.printf("cpr=%.1f  sidereal=%.2f c/s  %.3f arcsec/count\n",
                                                     cpr, siderealCountsPerSec(), arcsecPerCount()); } break;
    case 'm':
      if (t1 && t2 && (ax == 0 || ax == 1)) {
        ramping = false;
        driveMotor(ax, atof(t2));
        Serial.printf("axis %d duty %.1f%% (cap %.0f%%)\n", ax, dutyPct[ax], MAX_DUTY_PCT);
        if (!streaming) { streaming = true; lastReportMs = millis();
                          lastReportCnt[0] = countOf(0); lastReportCnt[1] = countOf(1); }
      } else Serial.println(F("usage: m <0|1> <percent>"));
      break;
    case 'f':
      if (t1 && t2 && (ax == 0 || ax == 1)) {
        rampAxis = ax; rampDir = (atof(t2) >= 0) ? 1.0f : -1.0f; rampDuty = 0;
        ramping = true; rampStepMs = millis(); rampStartCnt = countOf(ax);
        Serial.println(F("ramping: +0.5% every 2 s. 'x' to stop."));
      } else Serial.println(F("usage: f <0|1> <+1|-1>"));
      break;
    case 'i': if (ax == 0 || ax == 1) { invertMotor[ax] = !invertMotor[ax];
                Serial.printf("axis %d invert=%d\n", ax, (int)invertMotor[ax]); } break;
    case 'x': brakeAll(); Serial.println(F("brake")); break;
    case 'q': coastAll(); Serial.println(F("coast / standby")); break;
    default:  Serial.println(F("?  (h for help)"));
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

// ---------------------------------------------------------------- setup/loop
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(STBY_PIN, OUTPUT); digitalWrite(STBY_PIN, LOW);      // driver off until first 'm'
  for (int a = 0; a < 2; a++) {
    pinMode(IN1_PIN[a], OUTPUT); pinMode(IN2_PIN[a], OUTPUT); pinMode(PWM_PIN[a], OUTPUT);
    digitalWrite(IN1_PIN[a], LOW); digitalWrite(IN2_PIN[a], LOW);
  }
  analogWriteFreq(20000);       // 20 kHz: inaudible, smooth current
  analogWriteResolution(10);    // 0..1023
  for (int a = 0; a < 2; a++) analogWrite(PWM_PIN[a], 0);

  if (!pio_can_add_program_at_offset(encPio, &quadrature_encoder_program, 0)) {
    Serial.println(F("ERROR: PIO2 has no room at offset 0 for the encoder program"));
    while (true) delay(1000);
  }
  pio_add_program_at_offset(encPio, &quadrature_encoder_program, 0);
  encoderBegin(0);
  encoderBegin(1);

  Serial.println(F("\nTelescope bring-up ready. Type 'h' for help."));
  Serial.printf("Total ratio %.6f motor revs/axis rev; sidereal needs %.4f motor rev/s (%.2f rpm)\n",
                TOTAL_RATIO, TOTAL_RATIO / SIDEREAL_DAY_S, 60.0 * TOTAL_RATIO / SIDEREAL_DAY_S);
  lastReportMs = millis();
}

void loop() {
  pollSerial();
  uint32_t now = millis();

  // Safety: motors stop unless a command refreshed them recently.
  if (motorsActive() && now - lastCmdMs > MOTOR_TIMEOUT_MS && !ramping) {
    coastAll();
    Serial.println(F("TIMEOUT: motors stopped (send 'm' again to continue)"));
  }

  // Duty ramp: raise duty in 0.5% steps every 2 s and print what happens.
  if (ramping && now - rampStepMs >= 2000) {
    int32_t c = countOf(rampAxis);
    float rate = (c - rampStartCnt) / ((now - rampStepMs) / 1000.0f);
    if (rampDuty > 0)
      Serial.printf("duty %5.1f%%  -> %7.2f counts/s (sidereal %.2f)\n", rampDuty * rampDir, rate, siderealCountsPerSec());
    rampStartCnt = c; rampStepMs = now; lastCmdMs = now;
    if (fabsf(rate) >= 1.5 * siderealCountsPerSec() && rampDuty > 0) {
      brakeAll(); Serial.println(F("ramp done: reached 1.5x sidereal"));
    } else if (rampDuty >= MAX_DUTY_PCT) {
      brakeAll(); Serial.println(F("ramp done: hit duty cap"));
    } else {
      rampDuty += 0.5f;
      driveMotor(rampAxis, rampDuty * rampDir);
    }
  }

  if (streaming && now - lastReportMs >= 500) report();
}
