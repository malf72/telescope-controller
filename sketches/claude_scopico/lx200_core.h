// LX200 protocol: byte framing, coordinate parsing/formatting and command
// dispatch. Pure C++ (no Arduino/pico calls) so it is unit-tested with g++
// against a fake mount (tests/test_lx200.cpp). The sketch implements
// Lx200Mount and feeds bytes from the TCP socket through lx200Feed().
//
// Command set: what INDI's lx200basic driver uses (Stellarium via INDI) plus
// the common extras other clients (SkySafari, INDI LX200 drivers) send:
//   :GR# :GD# :GA# :GZ#            get RA / Dec / Alt / Az
//   :Sr..# :Sd..#                  set target RA / Dec           -> '1' | '0'
//   :MS#                           goto target  -> '0' | '1<msg>#' (horizon) | '2<msg>#'
//   :CM# :CMR#                     sync on target                -> '<msg>#'
//   :Q# :Qn# :Qs# :Qe# :Qw#        stop all / stop one direction
//   :Mn# :Ms# :Me# :Mw#            start moving;  :RG# :RC# :RM# :RS# pick the rate
//   :U#                            toggle high/low precision
//   :D#                            '|#' while slewing, else '#'
//   :Gt# :Gg# :St..# :Sg..#        site latitude / longitude (Meade: west-positive)
//   :SG..# :SL..# :SC..# :GG# :GL# :GC#   UTC offset, local time, date
//   :GVP# :GVN# :GVD# :GVT# :Gc#   product info, 24h clock
//   :GW#                           status <P|A><T|N><0-3># : mount, tracking, alignment stars
//   <ACK 0x06>                     alignment mode: 'P' (polar) or 'A' (alt-az)
// Unknown commands get no reply, like a real LX200.
// All coordinates are JNow, radians at the interface.
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctype.h>

struct Lx200Mount {
  virtual bool getRaDec(double &ra, double &dec) = 0;      // false if position unknown (no time/site)
  virtual bool getAltAz(double &alt, double &az) = 0;
  virtual int  gotoTarget(double ra, double dec, char *msg, size_t n) = 0;  // 0 ok, 1 below horizon, 2 other
  virtual bool sync(double ra, double dec, char *msg, size_t n) = 0;
  virtual void stop() = 0;                                  // :Q#
  virtual void move(char dir, bool start) = 0;              // dir = n s e w
  virtual void setRate(char r) = 0;                         // G C M S
  virtual bool isSlewing() = 0;
  virtual bool isEquatorial() = 0;
  virtual bool isTracking() = 0;
  virtual int  alignStars() = 0;                            // sync stars in the model
  virtual bool getSite(double &lat, double &lonEast) = 0;
  virtual void setLatitude(double lat) = 0;
  virtual void setLongitude(double lonEast) = 0;
  virtual bool getUtc(double &unixSec) = 0;
  virtual void setUtc(double unixSec) = 0;
  virtual ~Lx200Mount() {}
};

struct Lx200State {
  bool   highPrec = true;
  bool   haveRa = false, haveDec = false;
  double ra = 0, dec = 0;            // target set by :Sr / :Sd
  double utcOffsetH = 0;             // :SG -- hours ADDED to local time to give UTC
  int    localH = 0, localM = 0, localS = 0;   // :SL
};

// ---------------------------------------------------------------- framing
struct Lx200Framer {
  char buf[48];
  int  n = -1;                       // -1 = waiting for ':'
};
enum { LX_NONE = 0, LX_CMD = 1, LX_ACK = 2 };

// Feed one received byte. Returns LX_CMD with the command text (between ':'
// and '#') in cmd, LX_ACK for a bare ACK byte, else LX_NONE.
inline int lx200Feed(Lx200Framer &f, char c, char *cmd, size_t n) {
  if (c == 0x06 && f.n < 0) return LX_ACK;
  if (f.n < 0) {                                   // between commands: wait for ':', ignore noise / lone '#'
    if (c == ':') f.n = 0;
    return LX_NONE;
  }
  // inside a command ':' is data (":Sr 10:20:30#"); only '#' ends it
  if (c == '#') {
    f.buf[f.n] = 0;
    strncpy(cmd, f.buf, n - 1);
    cmd[n - 1] = 0;
    f.n = -1;
    return LX_CMD;
  }
  if (c == '\r' || c == '\n') return LX_NONE;
  if (f.n < (int)sizeof(f.buf) - 1) f.buf[f.n++] = c;
  else f.n = -1;                                   // overlong: drop it
  return LX_NONE;
}

// ---------------------------------------------------------------- numbers
static const double LX_PI = 3.14159265358979323846;

// Parse up to 3 sexagesimal fields ("DD*MM:SS", "HH:MM.T", "sDD\xDFMM'SS"...):
// any non-digit, non-'.' character separates fields. Returns the number of fields.
inline int lx200ParseSexa(const char *s, bool &neg, double v[3]) {
  while (*s == ' ') s++;
  neg = false;
  if (*s == '-') { neg = true; s++; } else if (*s == '+') s++;
  int nf = 0;
  v[0] = v[1] = v[2] = 0;
  while (*s && nf < 3) {
    if (!isdigit((unsigned char)*s) && *s != '.') { s++; continue; }
    char *end;
    v[nf++] = strtod(s, &end);
    if (end == s) break;
    s = end;
  }
  return nf;
}

inline bool lx200ParseRa(const char *s, double &ra) {
  bool neg; double v[3];
  int nf = lx200ParseSexa(s, neg, v);
  if (nf < 2 || neg || v[0] >= 24 || v[1] >= 60 || v[2] >= 60) return false;
  ra = (v[0] + v[1] / 60.0 + v[2] / 3600.0) * LX_PI / 12.0;
  return true;
}
inline bool lx200ParseDec(const char *s, double &dec) {   // also used for latitude
  bool neg; double v[3];
  int nf = lx200ParseSexa(s, neg, v);
  if (nf < 2 || v[0] > 90 || v[1] >= 60 || v[2] >= 60) return false;
  double d = v[0] + v[1] / 60.0 + v[2] / 3600.0;
  if (d > 90) return false;
  dec = (neg ? -d : d) * LX_PI / 180.0;
  return true;
}
inline bool lx200ParseAngle360(const char *s, double &a, bool &neg) {   // longitude "sDDD*MM[:SS]"
  double v[3];
  int nf = lx200ParseSexa(s, neg, v);
  if (nf < 2 || v[0] > 360 || v[1] >= 60 || v[2] >= 60) return false;
  a = (v[0] + v[1] / 60.0 + v[2] / 3600.0) * LX_PI / 180.0;
  return true;
}

// Split a non-negative value into whole units / minutes / seconds (or tenths
// of a minute), rounding once at the end so 59.99 s never prints as "60".
inline void lx200Split(double units, bool tenths, long &a, long &b, long &c) {
  if (tenths) {
    long t = lround(units * 600.0);          // tenths of a minute
    a = t / 600; b = (t / 10) % 60; c = t % 10;
  } else {
    long t = lround(units * 3600.0);         // seconds
    a = t / 3600; b = (t / 60) % 60; c = t % 60;
  }
}

inline void lx200FormatRa(double ra, bool hp, char *out, size_t n) {
  double h = std::fmod(ra * 12.0 / LX_PI, 24.0);
  if (h < 0) h += 24.0;
  long a, b, c;
  lx200Split(h, !hp, a, b, c);
  if (a >= 24) a -= 24;
  if (hp) snprintf(out, n, "%02ld:%02ld:%02ld#", a, b, c);
  else    snprintf(out, n, "%02ld:%02ld.%01ld#", a, b, c);
}
// Dec / Alt / latitude: "sDD*MM:SS#" or "sDD*MM#"
inline void lx200FormatDec(double dec, bool hp, char *out, size_t n) {
  double d = dec * 180.0 / LX_PI;
  char sg = d < 0 ? '-' : '+';
  long a, b, c;
  lx200Split(std::fabs(d), false, a, b, c);
  if (hp) snprintf(out, n, "%c%02ld*%02ld:%02ld#", sg, a, b, c);
  else {
    long m = lround(std::fabs(d) * 60.0);
    snprintf(out, n, "%c%02ld*%02ld#", sg, m / 60, m % 60);
  }
}
// Azimuth "DDD*MM:SS#" / "DDD*MM#"; longitude uses the signed variant
inline void lx200FormatAz(double az, bool hp, char *out, size_t n) {
  double d = std::fmod(az * 180.0 / LX_PI, 360.0);
  if (d < 0) d += 360.0;
  long a, b, c;
  lx200Split(d, false, a, b, c);
  if (hp) snprintf(out, n, "%03ld*%02ld:%02ld#", a % 360, b, c);
  else { long m = lround(d * 60.0) % (360 * 60); snprintf(out, n, "%03ld*%02ld#", m / 60, m % 60); }
}

// ---------------------------------------------------------------- dispatch
inline const char *lx200SkipSpaces(const char *s) { while (*s == ' ') s++; return s; }

// Handle one command (text between ':' and '#'). The reply, possibly empty,
// goes to out.
inline void lx200Handle(const char *cmd, Lx200State &st, Lx200Mount &m, char *out, size_t n) {
  out[0] = 0;
  double a, b;
  char msg[48];

  if (!strcmp(cmd, "GR")) {
    if (!m.getRaDec(a, b)) a = 0;
    lx200FormatRa(a, st.highPrec, out, n);
  } else if (!strcmp(cmd, "GD")) {
    if (!m.getRaDec(a, b)) b = 0;
    lx200FormatDec(b, st.highPrec, out, n);
  } else if (!strcmp(cmd, "GA")) {
    if (!m.getAltAz(a, b)) a = 0;
    lx200FormatDec(a, st.highPrec, out, n);
  } else if (!strcmp(cmd, "GZ")) {
    if (!m.getAltAz(a, b)) b = 0;
    lx200FormatAz(b, st.highPrec, out, n);
  } else if (!strncmp(cmd, "Sr", 2)) {
    bool ok = lx200ParseRa(cmd + 2, a);
    if (ok) { st.ra = a; st.haveRa = true; }
    snprintf(out, n, "%c", ok ? '1' : '0');
  } else if (!strncmp(cmd, "Sd", 2)) {
    bool ok = lx200ParseDec(cmd + 2, a);
    if (ok) { st.dec = a; st.haveDec = true; }
    snprintf(out, n, "%c", ok ? '1' : '0');
  } else if (!strcmp(cmd, "MS")) {
    if (!st.haveRa || !st.haveDec) { snprintf(out, n, "2No target set#"); return; }
    msg[0] = 0;
    int r = m.gotoTarget(st.ra, st.dec, msg, sizeof(msg));
    if (r == 0) snprintf(out, n, "0");
    else snprintf(out, n, "%d%s#", r == 1 ? 1 : 2, msg[0] ? msg : "Goto refused");
  } else if (!strcmp(cmd, "CM") || !strcmp(cmd, "CMR")) {
    if (!st.haveRa || !st.haveDec) { snprintf(out, n, "No target set#"); return; }
    msg[0] = 0;
    bool ok = m.sync(st.ra, st.dec, msg, sizeof(msg));
    snprintf(out, n, "%s#", msg[0] ? msg : ok ? "Coordinates     matched.        " : "Sync failed");
  } else if (!strcmp(cmd, "Q")) {
    m.stop();
  } else if (cmd[0] == 'Q' && strchr("nsew", cmd[1]) && cmd[1] && !cmd[2]) {
    m.move(cmd[1], false);
  } else if (cmd[0] == 'M' && cmd[1] && strchr("nsew", cmd[1]) && !cmd[2]) {
    m.move(cmd[1], true);
  } else if (cmd[0] == 'R' && cmd[1] && strchr("GCMS", cmd[1]) && !cmd[2]) {
    m.setRate(cmd[1]);
  } else if (!strcmp(cmd, "U")) {
    st.highPrec = !st.highPrec;
  } else if (!strcmp(cmd, "D")) {
    snprintf(out, n, "%s", m.isSlewing() ? "|#" : "#");
  } else if (!strcmp(cmd, "Gt")) {
    if (!m.getSite(a, b)) a = 0;
    double d = a * 180.0 / LX_PI;
    long mm = lround(std::fabs(d) * 60.0);
    snprintf(out, n, "%c%02ld*%02ld#", d < 0 ? '-' : '+', mm / 60, mm % 60);
  } else if (!strcmp(cmd, "Gg")) {
    if (!m.getSite(a, b)) b = 0;
    double w = -b * 180.0 / LX_PI;                       // Meade longitude is west-positive
    long mm = lround(std::fabs(w) * 60.0);
    snprintf(out, n, "%c%03ld*%02ld#", w < 0 ? '-' : '+', mm / 60, mm % 60);
  } else if (!strncmp(cmd, "St", 2)) {
    bool ok = lx200ParseDec(cmd + 2, a);
    if (ok) m.setLatitude(a);
    snprintf(out, n, "%c", ok ? '1' : '0');
  } else if (!strncmp(cmd, "Sg", 2)) {
    bool neg;
    bool ok = lx200ParseAngle360(cmd + 2, a, neg);
    if (ok) {
      double west = neg ? -a : a;                         // west-positive, 0..360 or signed
      double east = -west;
      while (east <= -LX_PI) east += 2 * LX_PI;
      while (east > LX_PI) east -= 2 * LX_PI;
      m.setLongitude(east);
    }
    snprintf(out, n, "%c", ok ? '1' : '0');
  } else if (!strncmp(cmd, "SG", 2)) {
    char *end;
    const char *s = lx200SkipSpaces(cmd + 2);
    double h = strtod(s, &end);
    bool ok = end != s && std::fabs(h) <= 14;
    if (ok) st.utcOffsetH = h;
    snprintf(out, n, "%c", ok ? '1' : '0');
  } else if (!strncmp(cmd, "SL", 2)) {
    bool neg; double v[3];
    int nf = lx200ParseSexa(cmd + 2, neg, v);
    bool ok = nf == 3 && !neg && v[0] < 24 && v[1] < 60 && v[2] < 60;
    if (ok) { st.localH = (int)v[0]; st.localM = (int)v[1]; st.localS = (int)v[2]; }
    snprintf(out, n, "%c", ok ? '1' : '0');
  } else if (!strncmp(cmd, "SC", 2)) {
    int mo, d, y;
    if (sscanf(lx200SkipSpaces(cmd + 2), "%d/%d/%d", &mo, &d, &y) == 3 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
      if (y < 100) y += 2000;
      // days from civil (same algorithm as coord_core.h, duplicated to keep this header standalone)
      long yy = y - (mo <= 2);
      long era = (yy >= 0 ? yy : yy - 399) / 400;
      unsigned yoe = (unsigned)(yy - era * 400);
      unsigned doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + d - 1;
      unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
      long days = era * 146097 + (long)doe - 719468;
      double local = days * 86400.0 + st.localH * 3600.0 + st.localM * 60.0 + st.localS;
      m.setUtc(local + st.utcOffsetH * 3600.0);
      snprintf(out, n, "1Updating Planetary Data#                              #");
    } else snprintf(out, n, "0");
  } else if (!strcmp(cmd, "GG")) {
    snprintf(out, n, "%+05.1f#", st.utcOffsetH);
  } else if (!strcmp(cmd, "GL") || !strcmp(cmd, "GC")) {
    double u;
    if (!m.getUtc(u)) u = 0;
    double local = u - st.utcOffsetH * 3600.0;
    double days = std::floor(local / 86400.0);
    long sod = (long)(local - days * 86400.0);
    if (cmd[1] == 'L') {
      snprintf(out, n, "%02ld:%02ld:%02ld#", sod / 3600, (sod / 60) % 60, sod % 60);
    } else {
      long z = (long)days + 719468;
      long era = (z >= 0 ? z : z - 146096) / 146097;
      unsigned doe = (unsigned)(z - era * 146097);
      unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
      long y = (long)yoe + era * 400;
      unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
      unsigned mp = (5 * doy + 2) / 153;
      unsigned d = doy - (153 * mp + 2) / 5 + 1;
      unsigned mo = mp < 10 ? mp + 3 : mp - 9;
      y += (mo <= 2);
      snprintf(out, n, "%02u/%02u/%02ld#", mo, d, y % 100);
    }
  } else if (!strcmp(cmd, "GW")) {
    int stars = m.alignStars();
    if (stars > 3) stars = 3;
    if (stars < 0) stars = 0;
    snprintf(out, n, "%c%c%d#", m.isEquatorial() ? 'P' : 'A', m.isTracking() ? 'T' : 'N', stars);
  } else if (!strcmp(cmd, "Gc")) {
    snprintf(out, n, "24#");
  } else if (!strcmp(cmd, "GVP")) {
    snprintf(out, n, "LX90 Pico#");
  } else if (!strcmp(cmd, "GVN")) {
    snprintf(out, n, "1.0#");
  } else if (!strcmp(cmd, "GVD")) {
    snprintf(out, n, "Oct 09 2026#");
  } else if (!strcmp(cmd, "GVT")) {
    snprintf(out, n, "00:00:00#");
  }
  // anything else: no reply
}

inline void lx200Ack(Lx200Mount &m, char *out, size_t n) {
  snprintf(out, n, "%c", m.isEquatorial() ? 'P' : 'A');
}
