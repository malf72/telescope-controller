// Text formats for the sky settings file (/sky.txt) and the park file
// (/park.txt). Pure C++, host-tested (tests/test_sky.cpp), same conventions
// as calib_config.h: magic header line, key=value tokens, unknown keys ignored,
// everything validated before anything is applied.
//
//   telescope-sky v1
//   mode=eq lat=51.4779 lon=-0.0015 site=1 sign0=1 sign1=1 horizon=0 rev0=3564965.25 rev1=3564965.25
//
//   telescope-park v1
//   mode=eq lat=0.898... o0=... z1=... T=t00,t01,...,t22 pos=counts0,counts1
//   star=raw0,raw1,ha,dec          (one line per sync star, radians)
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "coord_core.h"
#include "calib_config.h"   // calibAppend()

struct SkySettings {
  coord::MountMode mode = coord::MOUNT_EQ;
  bool   siteSet = false;
  double latDeg = 0, lonDeg = 0;     // east-positive longitude
  int    sign0 = 1, sign1 = 1;       // encoder count direction vs. the model's axis direction
  double horizonDeg = 0;             // gotos below this altitude are refused
  // encoder counts per full turn of each axis. Default = the bench-confirmed
  // gearing (432 counts/motor rev x 6859/128 x 154); adjustable per axis with
  // 'scale' in case an axis ever differs (both LX90 axes measured within 0.5 %).
  double rev0 = coord::COUNTS_PER_REV, rev1 = coord::COUNTS_PER_REV;
};
static const double SKY_REV_MIN = 1e5, SKY_REV_MAX = 1e8;

// ---- small shared helpers
inline bool skyParseNum(const char *s, double &v) {
  char *end;
  double x = strtod(s, &end);
  if (end == s || *end || !std::isfinite(x)) return false;
  v = x;
  return true;
}
// comma-separated list of exactly n numbers
inline bool skyParseList(const char *s, double *out, int n) {
  for (int i = 0; i < n; i++) {
    char *end;
    out[i] = strtod(s, &end);
    if (end == s || !std::isfinite(out[i])) return false;
    s = end;
    if (i < n - 1) { if (*s != ',') return false; s++; }
  }
  return *s == 0;
}

// Walk the key=value tokens of every line after the magic header.
// fn(key, value, err, errN) returns false to reject the file.
template <class F>
inline bool skyForEachPair(char *text, const char *magic, F fn, char *err, size_t errN) {
  bool haveMagic = false;
  char *ls;
  for (char *line = strtok_r(text, "\n", &ls); line; line = strtok_r(NULL, "\n", &ls)) {
    size_t l = strlen(line);
    while (l && (line[l - 1] == '\r' || line[l - 1] == ' ' || line[l - 1] == '\t')) line[--l] = 0;
    while (*line == ' ' || *line == '\t') line++;
    if (!*line) continue;
    if (!haveMagic) {
      if (strcmp(line, magic)) { snprintf(err, errN, "missing '%s' header", magic); return false; }
      haveMagic = true;
      continue;
    }
    if (line[0] == '#') continue;
    char *ts;
    for (char *tok = strtok_r(line, " \t", &ts); tok; tok = strtok_r(NULL, " \t", &ts)) {
      char *eq = strchr(tok, '=');
      if (!eq) { snprintf(err, errN, "'%s' is not key=value", tok); return false; }
      *eq = 0;
      if (!fn(tok, eq + 1, err, errN)) return false;
    }
  }
  if (!haveMagic) { snprintf(err, errN, "file is empty"); return false; }
  return true;
}

// ---------------------------------------------------------------- sky settings
#define SKY_MAGIC "telescope-sky v1"

inline int formatSky(char *buf, size_t n, const SkySettings &s) {
  size_t len = 0;
  if (!calibAppend(buf, n, len, "%s\nmode=%s lat=%.7f lon=%.7f site=%d sign0=%d sign1=%d horizon=%.2f rev0=%.2f rev1=%.2f\n",
                   SKY_MAGIC, s.mode == coord::MOUNT_ALTAZ ? "altaz" : "eq", s.latDeg, s.lonDeg,
                   s.siteSet ? 1 : 0, s.sign0, s.sign1, s.horizonDeg, s.rev0, s.rev1)) return -1;
  return (int)len;
}

inline bool parseSky(char *text, SkySettings &out, char *err, size_t errN) {
  SkySettings t = out;
  bool ok = skyForEachPair(text, SKY_MAGIC, [&](const char *k, const char *v, char *e, size_t en) {
    double x;
    if (!strcmp(k, "mode")) {
      if (!strcmp(v, "eq")) t.mode = coord::MOUNT_EQ;
      else if (!strcmp(v, "altaz")) t.mode = coord::MOUNT_ALTAZ;
      else { snprintf(e, en, "mode must be eq or altaz"); return false; }
      return true;
    }
    bool known = !strcmp(k, "lat") || !strcmp(k, "lon") || !strcmp(k, "site") || !strcmp(k, "sign0") ||
                 !strcmp(k, "sign1") || !strcmp(k, "horizon") || !strcmp(k, "rev0") || !strcmp(k, "rev1");
    if (!known) return true;                                   // unknown key: ignore
    if (!skyParseNum(v, x)) { snprintf(e, en, "bad value for %s", k); return false; }
    if (!strcmp(k, "lat")) {
      if (std::fabs(x) > 90) { snprintf(e, en, "lat out of range"); return false; }
      t.latDeg = x;
    } else if (!strcmp(k, "lon")) {
      if (std::fabs(x) > 180) { snprintf(e, en, "lon out of range"); return false; }
      t.lonDeg = x;
    } else if (!strcmp(k, "site")) {
      t.siteSet = x != 0;
    } else if (!strcmp(k, "sign0") || !strcmp(k, "sign1")) {
      if (x != 1 && x != -1) { snprintf(e, en, "%s must be 1 or -1", k); return false; }
      (k[4] == '0' ? t.sign0 : t.sign1) = (int)x;
    } else if (!strcmp(k, "rev0") || !strcmp(k, "rev1")) {
      if (x < SKY_REV_MIN || x > SKY_REV_MAX) { snprintf(e, en, "%s out of range", k); return false; }
      (k[3] == '0' ? t.rev0 : t.rev1) = x;
    } else {
      if (x < -10 || x > 60) { snprintf(e, en, "horizon out of range"); return false; }
      t.horizonDeg = x;
    }
    return true;
  }, err, errN);
  if (!ok) return false;
  out = t;
  return true;
}

// ---------------------------------------------------------------- park state
#define PARK_MAGIC "telescope-park v1"

// pos = encoder counts at the moment of parking; restored at the next boot
// because the encoders start from zero at power-up.
inline int formatPark(char *buf, size_t n, const coord::AlignModel &m, long pos0, long pos1) {
  size_t len = 0;
  if (!calibAppend(buf, n, len, "%s\nmode=%s lat=%.12g o0=%.12g z1=%.12g T=", PARK_MAGIC,
                   m.mode == coord::MOUNT_ALTAZ ? "altaz" : "eq", m.latitude, m.o0, m.z1)) return -1;
  for (int i = 0; i < 9; i++)
    if (!calibAppend(buf, n, len, i ? ",%.15g" : "%.15g", m.T.m[i / 3][i % 3])) return -1;
  if (!calibAppend(buf, n, len, " pos=%ld,%ld\n", pos0, pos1)) return -1;
  for (int i = 0; i < m.nStars; i++)
    if (!calibAppend(buf, n, len, "star=%.12g,%.12g,%.12g,%.12g\n",
                     m.stars[i].raw0, m.stars[i].raw1, m.stars[i].ha, m.stars[i].dec)) return -1;
  return (int)len;
}

// T must come back as a proper rotation (orthonormal, det +1) or the file is rejected.
inline bool parsePark(char *text, coord::AlignModel &out, long &pos0, long &pos1, char *err, size_t errN) {
  coord::AlignModel t;
  double pos[2] = {0, 0};
  bool haveT = false, haveO = false, haveZ = false, haveLat = false, haveMode = false, havePos = false;
  t.nStars = 0;
  bool ok = skyForEachPair(text, PARK_MAGIC, [&](const char *k, const char *v, char *e, size_t en) {
    double x;
    if (!strcmp(k, "mode")) {
      if (!strcmp(v, "eq")) t.mode = coord::MOUNT_EQ;
      else if (!strcmp(v, "altaz")) t.mode = coord::MOUNT_ALTAZ;
      else { snprintf(e, en, "bad mode"); return false; }
      haveMode = true;
    } else if (!strcmp(k, "T")) {
      double a[9];
      if (!skyParseList(v, a, 9)) { snprintf(e, en, "bad T"); return false; }
      for (int i = 0; i < 9; i++) t.T.m[i / 3][i % 3] = a[i];
      haveT = true;
    } else if (!strcmp(k, "pos")) {
      if (!skyParseList(v, pos, 2) || std::fabs(pos[0]) > 1e9 || std::fabs(pos[1]) > 1e9) {
        snprintf(e, en, "bad pos"); return false;
      }
      havePos = true;
    } else if (!strcmp(k, "star")) {
      double a[4];
      if (!skyParseList(v, a, 4)) { snprintf(e, en, "bad star"); return false; }
      if (t.nStars < coord::AlignModel::MAX_STARS) t.stars[t.nStars++] = {a[0], a[1], a[2], a[3]};
    } else if (!strcmp(k, "lat") || !strcmp(k, "o0") || !strcmp(k, "z1")) {
      if (!skyParseNum(v, x)) { snprintf(e, en, "bad value for %s", k); return false; }
      if (k[0] == 'l') { t.latitude = x; haveLat = true; }
      else if (k[0] == 'o') { t.o0 = x; haveO = true; }
      else { t.z1 = x; haveZ = true; }
    }
    return true;
  }, err, errN);
  if (!ok) return false;
  if (!haveT || !haveO || !haveZ || !haveLat || !haveMode || !havePos) { snprintf(err, errN, "incomplete park file"); return false; }
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      double d = 0;
      for (int k = 0; k < 3; k++) d += t.T.m[k][i] * t.T.m[k][j];
      if (std::fabs(d - (i == j ? 1 : 0)) > 1e-6) { snprintf(err, errN, "T is not a rotation"); return false; }
    }
  coord::Vec3 c0 = {t.T.m[0][0], t.T.m[1][0], t.T.m[2][0]}, c1 = {t.T.m[0][1], t.T.m[1][1], t.T.m[2][1]},
              c2 = {t.T.m[0][2], t.T.m[1][2], t.T.m[2][2]};
  if (coord::dot(coord::cross(c0, c1), c2) < 0) { snprintf(err, errN, "T is mirrored"); return false; }
  out = t;
  pos0 = std::lround(pos[0]);
  pos1 = std::lround(pos[1]);
  return true;
}
