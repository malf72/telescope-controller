// Host simulation test for sketches/claude_scopico/servo_core.h against a DC
// motor model fitted to the bench-measured axis-1 numbers (2026-10-09 'p'):
//   ~3.52 % duty -> sidereal 10.34 counts/s (auto ff), 40 % -> ~27,700 counts/s
//   (auto speed plateau = maxVel/0.7), 90 % rise ~0.2 s.
// i.e. a big friction deadband (~3.5 %) then ~760 counts/s per % duty.
// build: g++ -std=gnu++17 -Wall -Wextra -I../sketches/claude_scopico test_servo.cpp -o test_servo && ./test_servo
#include "servo_core.h"
#include <cstdio>
#include <cmath>
#include <initializer_list>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static const double SID = 10.3435;
static const double DEADBAND = 3.505, K = 759.5, TAU = 0.087;   // motor model

struct Motor {
  double w = 0, pos = 0;
  double load = 0;            // constant opposing torque in duty-% units (gravity on an unbalanced tube)
  void step(double duty, double dt) {
    duty -= load;
    double ad = std::fabs(duty);
    double wss = ad > DEADBAND ? (duty > 0 ? 1 : -1) * (ad - DEADBAND) * K : 0;
    if (ad <= DEADBAND && std::fabs(w) < 1) w = 0;               // stiction: stays stopped
    else w += (wss - w) * dt / TAU;
    pos += w * dt;
  }
  double count() const { return std::floor(pos); }               // quantized encoder
};

static AxisCtrl axis1(bool affineFF) {
  AxisCtrl a;
  a.kp = 0.8289; a.ki = 19.8309; a.maxIntegral = 40;
  a.kpHold = 0.15; a.kiHold = 0.05; a.maxIntegralHold = 40;
  a.maxDuty = 40; a.maxVel = 19397; a.maxAccel = 62347.5;
  if (affineFF) {   // two-point fit: (sidereal, 3.519 %) and (27,710 c/s, 40 %)
    a.ffGain = (40.0 - 3.519) / (27710.0 - SID);
    a.ffOffset = 3.519 - a.ffGain * SID;
  } else {
    a.ffGain = 0.3402;   // what 'auto ff' alone produces: duty/speed through zero
    a.ffOffset = 0;
  }
  return a;
}

// Manual move the way the firmware's sky loop drives it: 10 Hz updates of the
// commanded velocity, each step limited by maxAccel*0.1 s; hold for 'holdS'
// at full speed, then ramp back down. Returns the worst following error.
static double simulateMove(bool affineFF, double rate, double holdS, double &endErr) {
  AxisCtrl a = axis1(affineFF);
  Motor m;
  a.mode = MODE_TRACK;
  double cmd = 0, worst = 0, t = 0;
  const double dt = 0.001;
  int phase = 0;  double phaseT = 0;
  for (int i = 0; t < 30; i++, t += dt) {
    if (i % 100 == 0) {                            // 10 Hz sky loop
      double target = phase == 0 ? rate : 0;
      double step = a.maxAccel * 0.1, d = target - cmd;
      if (d > step) d = step;
      if (d < -step) d = -step;
      cmd += d;
      a.velCmd = cmd;
      if (phase == 0 && cmd == rate) { phaseT += 0.1; if (phaseT >= holdS) phase = 1; }
      if (phase == 1 && cmd == 0 && std::fabs(a.targetVel) < 1e-9) break;
    }
    double duty = servoStep(a, m.count(), dt);
    m.step(duty, dt);
    worst = std::fmax(worst, std::fabs(a.lastErr));
  }
  endErr = a.lastErr;
  return worst;
}

// Sidereal tracking for 60 s after settling: RMS error in counts
static double simulateTracking(bool affineFF) {
  AxisCtrl a = axis1(affineFF);
  Motor m;
  a.mode = MODE_TRACK;
  a.velCmd = SID;
  double sum2 = 0; int n = 0;
  for (int i = 0; i < 90000; i++) {
    double duty = servoStep(a, m.count(), 0.001);
    m.step(duty, 0.001);
    if (i >= 30000) { sum2 += a.lastErr * a.lastErr; n++; }
  }
  return std::sqrt(sum2 / n);
}

// Hold for 20 s with the target a few counts away, as on the real axes (hold
// error of ~4-13 counts seen in 'p'): stiction keeps the motor stuck, so the
// hold integral winds up to its limit. Then start a manual move at 'M' rate
// (32x sidereal) -- the Stellarium Mobile sequence that ran away on the
// hardware. Reports the integral's duty contribution just before and just
// after the gain switch (pre-fix code: raw integral x moving ki = ~800 %).
struct HoldMoveResult { double holdI, iDutyBefore, iDutyAfter, oldIDuty, worstErr, settledSpeed; };
static HoldMoveResult holdThenMove(double rate) {
  HoldMoveResult r{};
  AxisCtrl a = axis1(true);
  Motor m;
  a.mode = MODE_HOLD;
  a.targetPos = 6;                            // stuck 6 counts short: inside the friction deadband
  const double dt = 0.001;
  for (int i = 0; i < 20000; i++) { double d = servoStep(a, m.count(), dt); m.step(d, dt); }
  r.holdI = a.integral;
  r.iDutyBefore = a.kiHold * a.integral;
  r.oldIDuty = a.ki * a.integral;             // what the old code would have applied
  a.mode = MODE_TRACK;
  a.velCmd = rate;
  int nV = 0, p0 = 0;
  for (int i = 0; i < 3000; i++) {
    double d = servoStep(a, m.count(), dt);
    if (i == 0) r.iDutyAfter = a.ki * a.integral;
    m.step(d, dt);
    r.worstErr = std::fmax(r.worstErr, std::fabs(a.lastErr));
    if (i == 1000) p0 = (int)m.count();
    if (i >= 1000) nV++;
  }
  r.settledSpeed = (m.count() - p0) / (nV * dt);   // average speed over the last 2 s
  return r;
}

int main() {
  double endErr;
  {
    HoldMoveResult r = holdThenMove(-331);
    printf("hold (integral %.1f) then move at -331 c/s: integral duty %.2f%% before switch -> %.2f%% after "
           "(old code: %.0f%%); worst err %.0f counts; settled speed %.0f c/s\n",
           r.holdI, r.iDutyBefore, r.iDutyAfter, r.oldIDuty, r.worstErr, r.settledSpeed);
    CHECK(r.holdI > 30);                                  // the hold integral really did wind up
    CHECK(r.oldIDuty > 100);                              // ...so the old code would have saturated the drive
    CHECK(std::fabs(r.iDutyAfter - r.iDutyBefore) < 0.5); // bumpless (allowing one normal integration step)
    CHECK(r.worstErr < 200 + 331 * 0.1);                  // no fault
    CHECK(std::fabs(r.settledSpeed + 331) < 0.1 * 331);   // and it moves at the requested speed
  }
  double oldWorst = simulateMove(false, 19397, 2.0, endErr);
  double newWorst = simulateMove(true, 19397, 2.0, endErr);
  printf("full-speed move (19,397 c/s): worst following error  old ff %.0f counts | new ff %.0f counts (end %.1f)\n",
         oldWorst, newWorst, endErr);
  CHECK(oldWorst > 200);                      // reproduces the fault seen on the hardware
  double allowance = 200 + 19397 * 0.1;       // fault threshold at full speed (firmware: base + |v| * 0.1 s)
  CHECK(newWorst < allowance);
  CHECK(std::fabs(endErr) < 20);

  for (double r : {SID, 8 * SID, 32 * SID, 2000.0}) {
    double o = simulateMove(false, r, 2.0, endErr), w = simulateMove(true, r, 2.0, endErr);
    printf("move at %8.1f c/s: worst err old %7.1f | new %6.1f counts\n", r, o, w);
    CHECK(w < 200 + r * 0.1);
  }

  double tOld = simulateTracking(false), tNew = simulateTracking(true);
  printf("sidereal tracking RMS error: old ff %.3f counts | new ff %.3f counts\n", tOld, tNew);
  // The hard-stiction model here stick-slips at sidereal far worse than the real
  // motor does (which the person confirmed tracks very well), so only check the
  // new feedforward is no worse than the old one -- they are identical at
  // sidereal rate by construction of the two-point fit.
  CHECK(tNew <= tOld * 1.02 + 0.1);

  printf(fails ? "%d FAILURES\n" : "ALL PASSED\n", fails);
  return fails != 0;
}
