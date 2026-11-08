// Calibration file format: pure text formatting/parsing, no Arduino/pico calls,
// so the save/load round trip can be unit-tested with plain g++ (same
// convention as servo_core.h). The sketch does the actual flash file I/O.
//
// Format -- human readable, one line per axis, key=value pairs:
//   telescope-calib v1
//   # comment lines are allowed
//   axis=0 ffGain=0.6766 kp=... ki=... ... invertMotor=0
//   axis=1 ffGain=0.6812 kp=... ki=... ... invertMotor=1
// The first line is a magic/version header so the format can change later.
// Unknown keys are ignored and missing keys keep their current value, so a
// newer firmware can add fields without invalidating an older file.
#pragma once
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CALIB_MAGIC "telescope-calib v1"

struct AxisCalib {
  double ffGain, ffOffset, kp, ki, maxIntegral;
  double kpHold, kiHold, maxIntegralHold;
  double maxDuty, maxVel, maxAccel, faultThreshold;
  double gainFast, gainVSlow, gainVFast;     // speed schedule of the moving gains (servo_core.h)
  bool   invertMotor;
};

struct CalibField { const char *key; double AxisCalib::*member; };
static const CalibField CALIB_FIELDS[] = {
  {"ffGain",          &AxisCalib::ffGain},
  {"ffOffset",        &AxisCalib::ffOffset},
  {"kp",              &AxisCalib::kp},
  {"ki",              &AxisCalib::ki},
  {"maxIntegral",     &AxisCalib::maxIntegral},
  {"kpHold",          &AxisCalib::kpHold},
  {"kiHold",          &AxisCalib::kiHold},
  {"maxIntegralHold", &AxisCalib::maxIntegralHold},
  {"maxDuty",         &AxisCalib::maxDuty},
  {"maxVel",          &AxisCalib::maxVel},
  {"maxAccel",        &AxisCalib::maxAccel},
  {"faultThreshold",  &AxisCalib::faultThreshold},
  {"gainFast",        &AxisCalib::gainFast},
  {"gainVSlow",       &AxisCalib::gainVSlow},
  {"gainVFast",       &AxisCalib::gainVFast},
};
static const int CALIB_NFIELDS = sizeof(CALIB_FIELDS) / sizeof(CALIB_FIELDS[0]);

inline bool calibAppend(char *buf, size_t n, size_t &len, const char *fmt, ...) {
  if (len >= n) return false;
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(buf + len, n - len, fmt, ap);
  va_end(ap);
  if (w < 0 || (size_t)w >= n - len) return false;
  len += (size_t)w;
  return true;
}

// Writes the whole file for both axes into buf. Returns its length, or -1 if
// buf is too small. %.9g keeps enough digits that a save/load round trip is
// effectively lossless for these values.
inline int formatCalib(char *buf, size_t n, const AxisCalib c[2]) {
  size_t len = 0;
  if (!calibAppend(buf, n, len, "%s\n# written by telescope_calib firmware\n", CALIB_MAGIC)) return -1;
  for (int a = 0; a < 2; a++) {
    if (!calibAppend(buf, n, len, "axis=%d", a)) return -1;
    for (int i = 0; i < CALIB_NFIELDS; i++)
      if (!calibAppend(buf, n, len, " %s=%.9g", CALIB_FIELDS[i].key, c[a].*CALIB_FIELDS[i].member)) return -1;
    if (!calibAppend(buf, n, len, " invertMotor=%d\n", c[a].invertMotor ? 1 : 0)) return -1;
  }
  return (int)len;
}

// Rejects values that would be unsafe to hand to the servo, so a corrupt or
// hand-mangled file falls back to the defaults instead of driving the motors.
inline bool checkCalib(const AxisCalib &c, char *err, size_t errN) {
  for (int i = 0; i < CALIB_NFIELDS; i++)
    if (!std::isfinite(c.*CALIB_FIELDS[i].member)) {
      snprintf(err, errN, "%s is not a finite number", CALIB_FIELDS[i].key); return false;
    }
  if (c.ffGain <= 0)                              { snprintf(err, errN, "ffGain must be > 0"); return false; }
  if (c.ffOffset < 0 || c.ffOffset >= c.maxDuty)  { snprintf(err, errN, "ffOffset must be in [0, maxDuty)"); return false; }
  if (c.kp < 0 || c.ki < 0 || c.kpHold < 0 || c.kiHold < 0)
                                                  { snprintf(err, errN, "gains must be >= 0"); return false; }
  if (c.maxIntegral < 0 || c.maxIntegralHold < 0) { snprintf(err, errN, "integral limits must be >= 0"); return false; }
  if (c.maxDuty <= 0 || c.maxDuty > 100)          { snprintf(err, errN, "maxDuty must be in (0, 100]"); return false; }
  if (c.maxVel <= 0 || c.maxAccel <= 0)           { snprintf(err, errN, "maxVel/maxAccel must be > 0"); return false; }
  if (c.faultThreshold <= 0)                      { snprintf(err, errN, "faultThreshold must be > 0"); return false; }
  if (c.gainFast <= 0 || c.gainFast > 1)          { snprintf(err, errN, "gainFast must be in (0, 1]"); return false; }
  if (c.gainVSlow <= 0 || c.gainVFast <= c.gainVSlow) { snprintf(err, errN, "need 0 < gainVSlow < gainVFast"); return false; }
  return true;
}

inline bool calibParseDouble(const char *s, double &out) {
  char *end;
  double v = strtod(s, &end);
  if (end == s || *end != 0 || !std::isfinite(v)) return false;
  out = v;
  return true;
}

// Parses a whole file (text is modified in place). out[] must be pre-filled
// with the current values; keys missing from the file leave them unchanged.
// Returns true only if the header matched, both axis lines were present and
// every value passed checkCalib(). On false, out[] is untouched and err says why.
inline bool parseCalib(char *text, AxisCalib out[2], char *err, size_t errN) {
  AxisCalib tmp[2] = {out[0], out[1]};
  bool seen[2] = {false, false};
  bool haveMagic = false;
  char *lineSave;
  for (char *line = strtok_r(text, "\n", &lineSave); line; line = strtok_r(NULL, "\n", &lineSave)) {
    size_t l = strlen(line);
    while (l && (line[l - 1] == '\r' || line[l - 1] == ' ' || line[l - 1] == '\t')) line[--l] = 0;
    while (*line == ' ' || *line == '\t') line++;
    if (!*line) continue;

    if (!haveMagic) {
      if (strcmp(line, CALIB_MAGIC) != 0) { snprintf(err, errN, "missing '%s' header", CALIB_MAGIC); return false; }
      haveMagic = true;
      continue;
    }
    if (line[0] == '#') continue;

    char *tokSave;
    char *tok = strtok_r(line, " \t", &tokSave);
    if (strncmp(tok, "axis=", 5) != 0) continue;           // not an axis line: ignore (future use)
    if (strcmp(tok + 5, "0") != 0 && strcmp(tok + 5, "1") != 0) { snprintf(err, errN, "bad '%s'", tok); return false; }
    int a = tok[5] - '0';
    if (seen[a]) { snprintf(err, errN, "axis %d listed twice", a); return false; }
    seen[a] = true;

    while ((tok = strtok_r(NULL, " \t", &tokSave)) != NULL) {
      char *eq = strchr(tok, '=');
      if (!eq) { snprintf(err, errN, "axis %d: '%s' is not key=value", a, tok); return false; }
      *eq = 0;
      const char *key = tok, *val = eq + 1;
      if (!strcmp(key, "invertMotor")) {
        if (strcmp(val, "0") && strcmp(val, "1")) { snprintf(err, errN, "axis %d: invertMotor must be 0 or 1", a); return false; }
        tmp[a].invertMotor = (val[0] == '1');
        continue;
      }
      for (int i = 0; i < CALIB_NFIELDS; i++) {
        if (strcmp(key, CALIB_FIELDS[i].key)) continue;
        if (!calibParseDouble(val, tmp[a].*CALIB_FIELDS[i].member)) {
          snprintf(err, errN, "axis %d: bad value for %s", a, key); return false;
        }
        break;
      }
      // unknown keys fall through the loop and are ignored
    }
  }
  if (!haveMagic)          { snprintf(err, errN, "file is empty"); return false; }
  if (!seen[0] || !seen[1]) { snprintf(err, errN, "axis %d line missing", seen[0] ? 1 : 0); return false; }
  for (int a = 0; a < 2; a++) {
    char sub[64];
    if (!checkCalib(tmp[a], sub, sizeof(sub))) { snprintf(err, errN, "axis %d: %s", a, sub); return false; }
  }
  out[0] = tmp[0];
  out[1] = tmp[1];
  return true;
}
