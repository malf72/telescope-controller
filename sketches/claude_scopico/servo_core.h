// Pure math core: no Arduino/pico calls, so it can be unit-tested with g++
// and then #included unchanged into the real sketch.
#pragma once
#include <cmath>

enum AxisMode { MODE_IDLE, MODE_TRACK, MODE_SLEW, MODE_HOLD };

struct AxisCtrl {
  // calibration (from bench measurement)
  // Feedforward duty = sign(v) * ffOffset + v * ffGain. The offset is the
  // friction/deadband duty the motor needs before it turns at all; measured
  // on the LX90 it is almost all of the ~3.5 % used at sidereal rate, so a
  // through-zero line fitted at sidereal (ffOffset = 0) overstates the duty at
  // slew speed ~200x and saturates the drive (fault at full-speed moves).
  double ffGain     = 0.0;   // duty %, per (count/s) of target velocity
  double ffOffset   = 0.0;   // duty %, applied in the direction of motion when moving

  // gains used while the target is MOVING (MODE_TRACK or MODE_SLEW).
  // These are the ones the relay auto-tune produces; they can be aggressive
  // because the moving setpoint means the motor is already turning, which
  // avoids the stiction/hunting problem a static setpoint runs into.
  double kp         = 0.0;
  double ki         = 0.0;
  double maxIntegral= 40.0;

  // gains used while HOLDING a fixed position (MODE_HOLD / MODE_IDLE).
  // Deliberately NOT auto-tuned: simulation showed aggressive gains hunt
  // against motor stiction once the setpoint stops moving. Keep these gentle.
  double kpHold          = 0.15;
  double kiHold          = 0.05;
  double maxIntegralHold = 40.0;

  double maxDuty     = 40.0; // hard duty cap, matches MAX_DUTY_PCT in bring-up sketch

  // slew profile limits
  double maxVel      = 50.0;   // counts/s
  double maxAccel    = 100.0;  // counts/s^2
  double arriveEps   = 2.0;    // counts
  double arriveVelEps= 1.0;    // counts/s

  // live state
  AxisMode mode      = MODE_IDLE;
  double targetPos   = 0.0;  // where the profile/tracker wants to be right now (fractional counts)
  double targetVel   = 0.0;  // signed counts/s commanded right now
  double velCmd      = 0.0;  // MODE_TRACK: requested velocity; targetVel ramps to it at maxAccel
  double goalPos     = 0.0;  // used only in MODE_SLEW
  double integral    = 0.0;
  double kiPrev      = 0.0;   // effective ki used on the previous step (bumpless rescaling)

  // Speed scheduling of the MOVING gains (kp/ki). Full gains at or below
  // gainVSlow (tracking -- where the relay-tuned gains give the best accuracy),
  // gainFast x kp/ki at or above gainVFast (slews/fast moves -- where the same
  // gains made the az servo oscillate at ~28 Hz, 2026-11), interpolated on a
  // log-speed scale in between. gainFast = 1 disables it. Uses |targetVel|.
  double gainFast    = 1.0;
  double gainVSlow   = 0.0;    // counts/s
  double gainVFast   = 0.0;    // counts/s

  double lastOut     = 0.0;
  double lastErr      = 0.0;
};

// Advance the trapezoidal profile by dt seconds. Only touches targetPos/targetVel/mode.
inline void stepProfile(AxisCtrl &a, double dt) {
  if (a.mode != MODE_SLEW) return;
  double remaining = a.goalPos - a.targetPos;
  double decelDist = (a.targetVel * a.targetVel) / (2.0 * a.maxAccel);
  double desiredVel = (remaining >= 0 ? 1.0 : -1.0) * a.maxVel;
  if (std::fabs(remaining) <= decelDist)
    desiredVel = (remaining >= 0 ? 1.0 : -1.0) * std::sqrt(2.0 * a.maxAccel * std::fabs(remaining));

  double dv = desiredVel - a.targetVel;
  double maxDv = a.maxAccel * dt;
  if (dv >  maxDv) dv =  maxDv;
  if (dv < -maxDv) dv = -maxDv;
  a.targetVel += dv;
  a.targetPos += a.targetVel * dt;

  if (std::fabs(remaining) < a.arriveEps && std::fabs(a.targetVel) < a.arriveVelEps) {
    a.targetVel = 0.0;
    a.targetPos = a.goalPos;
    a.mode = MODE_HOLD;
    a.integral = 0.0;   // fresh start under the gentle hold gains, no carried-over windup
  }
}

// Fraction of the moving gains to use at target speed v (counts/s).
inline double movingGainScale(const AxisCtrl &a, double v) {
  if (a.gainFast >= 1.0 || a.gainVSlow <= 0 || a.gainVFast <= a.gainVSlow) return 1.0;
  v = std::fabs(v);
  if (v <= a.gainVSlow) return 1.0;
  if (v >= a.gainVFast) return a.gainFast;
  double t = std::log(v / a.gainVSlow) / std::log(a.gainVFast / a.gainVSlow);
  return 1.0 + t * (a.gainFast - 1.0);
}

// One 1 kHz control step. posNow is the current encoder count (as a double).
// Returns signed duty percent, already clamped to +-maxDuty.
inline double servoStep(AxisCtrl &a, double posNow, double dt) {
  if (a.mode == MODE_SLEW) stepProfile(a, dt);
  else if (a.mode == MODE_TRACK) {
    // ramp toward the requested velocity at maxAccel (1 kHz), so a changed
    // request -- e.g. a manual move starting -- is never a velocity step
    double dv = a.velCmd - a.targetVel, maxDv = a.maxAccel * dt;
    if (dv >  maxDv) dv =  maxDv;
    if (dv < -maxDv) dv = -maxDv;
    a.targetVel += dv;
    a.targetPos += a.targetVel * dt;
  }
  // MODE_IDLE / MODE_HOLD: targetPos stays fixed, targetVel forced to 0 by caller when entering those modes.

  bool moving = (a.mode == MODE_TRACK || a.mode == MODE_SLEW);
  double sched = moving ? movingGainScale(a, a.targetVel) : 1.0;
  double kp    = moving ? a.kp * sched : a.kpHold;
  double ki    = moving ? a.ki * sched : a.kiHold;
  double maxI  = moving ? a.maxIntegral : a.maxIntegralHold;
  // the integral term alone may never ask for more than the drive limit
  if (ki > 0 && maxI > a.maxDuty / ki) maxI = a.maxDuty / ki;

  // Bumpless gain change. Whenever the effective ki changes -- hold <-> moving
  // gain sets, or the speed schedule -- rescale the integral so its duty
  // contribution ki*integral is unchanged. The hold gains (ki ~0.05)
  // legitimately wind the integral up to its limit to hold against
  // gravity/friction; carried raw into the moving gains (ki ~20, 400x larger)
  // that value meant ~800 % duty and the axis ran away the instant a move
  // started (seen from Stellarium Mobile, 2026-10-09).
  if (ki != a.kiPrev) {
    if (a.kiPrev > 0 && ki > 0) a.integral *= a.kiPrev / ki;
    else if (ki <= 0) a.integral = 0.0;
    a.kiPrev = ki;
  }

  double err = a.targetPos - posNow;
  double ff  = a.targetVel * a.ffGain;
  if (a.targetVel > 0) ff += a.ffOffset;
  else if (a.targetVel < 0) ff -= a.ffOffset;

  // anti-windup: don't integrate further into a saturated output
  double trial = ff + kp * err + ki * (a.integral + err * dt);
  bool saturatedSameWay = (trial > a.maxDuty && err > 0) || (trial < -a.maxDuty && err < 0);
  if (!saturatedSameWay) a.integral += err * dt;
  if (a.integral >  maxI) a.integral =  maxI;
  if (a.integral < -maxI) a.integral = -maxI;

  double out = ff + kp * err + ki * a.integral;
  if (out >  a.maxDuty) out =  a.maxDuty;
  if (out < -a.maxDuty) out = -a.maxDuty;

  a.lastOut = out;
  a.lastErr = err;
  return out;
}
