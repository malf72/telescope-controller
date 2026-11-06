// Pure math core: no Arduino/pico calls, so it can be unit-tested with g++
// and then #included unchanged into the real sketch.
#pragma once
#include <cmath>

enum AxisMode { MODE_IDLE, MODE_TRACK, MODE_SLEW, MODE_HOLD };

struct AxisCtrl {
  // calibration (from bench measurement)
  double ffGain     = 0.0;   // duty %, per (count/s) of target velocity
  double kp         = 0.0;   // duty % per count of position error
  double ki         = 0.0;   // duty % per (count*s) of accumulated error
  double maxIntegral= 40.0;  // clamp on ki*integral contribution's raw integral, in count*s
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
  double goalPos     = 0.0;  // used only in MODE_SLEW
  double integral    = 0.0;

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
  }
}

// One 1 kHz control step. posNow is the current encoder count (as a double).
// Returns signed duty percent, already clamped to +-maxDuty.
inline double servoStep(AxisCtrl &a, double posNow, double dt) {
  if (a.mode == MODE_SLEW)       stepProfile(a, dt);
  else if (a.mode == MODE_TRACK) a.targetPos += a.targetVel * dt;
  // MODE_IDLE / MODE_HOLD: targetPos stays fixed, targetVel forced to 0 by caller when entering those modes.

  double err = a.targetPos - posNow;
  double ff  = a.targetVel * a.ffGain;

  a.integral += err * dt;
  if (a.integral >  a.maxIntegral) a.integral =  a.maxIntegral;
  if (a.integral < -a.maxIntegral) a.integral = -a.maxIntegral;

  double out = ff + a.kp * err + a.ki * a.integral;
  if (out >  a.maxDuty) out =  a.maxDuty;
  if (out < -a.maxDuty) out = -a.maxDuty;

  a.lastOut = out;
  a.lastErr = err;
  return out;
}
