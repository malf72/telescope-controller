// Coordinate engine: UTC -> sidereal time, sky (HA/Dec) <-> mount axis angles,
// and the star-sync alignment model. Pure C++, no Arduino/pico calls, so it is
// unit-tested with plain g++ (tests/test_coord.cpp), same as servo_core.h.
//
// All angles are radians internally. Coordinates are JNow (INDI convention);
// no precession/nutation/refraction is applied -- fine for visual pointing.
//
// Frames
//   sky frame   : fixed to the Earth. x = equator on the meridian, y = east
//                 horizon point (HA = -6h), z = north celestial pole.
//                 A star at hour angle H, declination d has longitude -H, latitude d.
//   instrument  : fixed to the mount's two axes. Axis angles (t0, t1) give the
//                 vector (cos t1 cos t0, cos t1 sin t0, sin t1): t0 turns about
//                 the mount's primary axis (RA / azimuth), t1 is the secondary
//                 (Dec / altitude). Instrument angle = raw axis angle + offset.
//   raw angle   : sign * encoderCounts * RAD_PER_COUNT, zero where the axis was
//                 when it was zeroed (normally the home/park position).
//
// Alignment model (Taki's matrix method, extended with the axis-1 zero offset):
//   sky = T * instrument, T a rotation. Rotations absorb the primary axis zero
//   (o0) and polar-alignment / levelling error, so one model covers both an
//   equatorial wedge and alt-az. The secondary axis zero (z1) is not a
//   rotation, so with 2+ sync stars it is solved separately by matching the
//   stars' angular separations (which no rotation can change).
#pragma once
#include <cmath>

namespace coord {

constexpr double kPi   = 3.14159265358979323846;
constexpr double k2Pi  = 2.0 * kPi;
constexpr double kDeg  = kPi / 180.0;
constexpr double kHour = kPi / 12.0;           // one hour of RA
constexpr double kArcsec = kDeg / 3600.0;

// ---- mechanics (bench-confirmed, see CLAUDE.md)
constexpr double GEARBOX_RATIO   = 6859.0 / 128.0;
constexpr double WORM_RATIO      = 154.0;
constexpr double TOTAL_RATIO     = GEARBOX_RATIO * WORM_RATIO;   // 8252.234375 motor revs / axis rev
// 108 encoder lines x 4 quadrature edges. (Was 108 until 2026-10-09: that made
// every axis angle 4x off; confirmed by measuring both LX90 axes on the mount.)
constexpr double CPR             = 432.0;                        // quadrature counts / motor rev
constexpr double COUNTS_PER_REV  = CPR * TOTAL_RATIO;            // 3,564,965.25 counts / axis rev
constexpr double RAD_PER_COUNT   = k2Pi / COUNTS_PER_REV;        // ~0.364 arcsec
constexpr double SIDEREAL_DAY_S  = 86164.0905;
constexpr double SIDEREAL_CPS    = COUNTS_PER_REV / SIDEREAL_DAY_S; // ~41.37 counts/s

// ---------------------------------------------------------------- small math
inline double wrap2Pi(double a) { a = std::fmod(a, k2Pi); return a < 0 ? a + k2Pi : a; }
inline double wrapPi(double a)  { a = wrap2Pi(a); return a > kPi ? a - k2Pi : a; }

struct Vec3 { double x, y, z; };
inline Vec3   operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3   operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3   operator*(double s, Vec3 a) { return {s * a.x, s * a.y, s * a.z}; }
inline double dot(Vec3 a, Vec3 b)   { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3   cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double norm(Vec3 a)          { return std::sqrt(dot(a, a)); }
inline Vec3   normalize(Vec3 a)     { double n = norm(a); return n > 0 ? (1.0 / n) * a : a; }
// angle between two vectors; atan2 form stays accurate for tiny and near-180 angles
inline double angleBetween(Vec3 a, Vec3 b) { return std::atan2(norm(cross(a, b)), dot(a, b)); }

struct Mat3 { double m[3][3]; };
inline Mat3 identity() { return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }
inline Vec3 mul(const Mat3 &M, Vec3 v) {
  return {M.m[0][0] * v.x + M.m[0][1] * v.y + M.m[0][2] * v.z,
          M.m[1][0] * v.x + M.m[1][1] * v.y + M.m[1][2] * v.z,
          M.m[2][0] * v.x + M.m[2][1] * v.y + M.m[2][2] * v.z};
}
inline Vec3 mulT(const Mat3 &M, Vec3 v) {   // transpose(M) * v  (= inverse for a rotation)
  return {M.m[0][0] * v.x + M.m[1][0] * v.y + M.m[2][0] * v.z,
          M.m[0][1] * v.x + M.m[1][1] * v.y + M.m[2][1] * v.z,
          M.m[0][2] * v.x + M.m[1][2] * v.y + M.m[2][2] * v.z};
}
inline Mat3 mul(const Mat3 &A, const Mat3 &B) {
  Mat3 R;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      R.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
  return R;
}
inline Mat3 fromColumns(Vec3 a, Vec3 b, Vec3 c) {
  return {{{a.x, b.x, c.x}, {a.y, b.y, c.y}, {a.z, b.z, c.z}}};
}
inline Mat3 rotX(double a) { double c = std::cos(a), s = std::sin(a); return {{{1, 0, 0}, {0, c, -s}, {0, s, c}}}; }
inline Mat3 rotY(double a) { double c = std::cos(a), s = std::sin(a); return {{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}}; }
inline Mat3 rotZ(double a) { double c = std::cos(a), s = std::sin(a); return {{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}; }

// spherical (longitude, latitude) <-> unit vector
inline Vec3 sphToVec(double lon, double lat) {
  return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}
inline void vecToSph(Vec3 v, double &lon, double &lat) {
  v = normalize(v);
  lon = std::atan2(v.y, v.x);
  lat = std::asin(v.z > 1 ? 1 : v.z < -1 ? -1 : v.z);
}
inline Vec3 haDecToVec(double ha, double dec) { return sphToVec(-ha, dec); }
inline void vecToHaDec(Vec3 v, double &ha, double &dec) { double lon; vecToSph(v, lon, dec); ha = wrapPi(-lon); }

// ---------------------------------------------------------------- time
// Days from civil date (proleptic Gregorian), Howard Hinnant's algorithm.
inline long daysFromCivil(long y, unsigned m, unsigned d) {
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (long)doe - 719468;
}
inline double unixFromCivil(long y, unsigned mo, unsigned d, int hh, int mi, double ss) {
  return (double)daysFromCivil(y, mo, d) * 86400.0 + hh * 3600.0 + mi * 60.0 + ss;
}
// civil date/time from unix seconds (for printing)
inline void civilFromUnix(double unixSec, long &y, unsigned &mo, unsigned &d, int &hh, int &mi, double &ss) {
  double days = std::floor(unixSec / 86400.0);
  double sod = unixSec - days * 86400.0;
  long z = (long)days + 719468;
  long era = (z >= 0 ? z : z - 146096) / 146097;
  unsigned doe = (unsigned)(z - era * 146097);
  unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  y = (long)yoe + era * 400;
  unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned mp = (5 * doy + 2) / 153;
  d = doy - (153 * mp + 2) / 5 + 1;
  mo = mp < 10 ? mp + 3 : mp - 9;
  y += (mo <= 2);
  hh = (int)(sod / 3600.0);
  mi = (int)((sod - hh * 3600.0) / 60.0);
  ss = sod - hh * 3600.0 - mi * 60.0;
}

constexpr double UNIX_J2000 = 946728000.0;  // 2000-01-01 12:00:00 UTC

inline double julianDate(double unixSec) { return unixSec / 86400.0 + 2440587.5; }

// Greenwich mean sidereal time, radians in [0, 2pi). Meeus eq. 12.4; UT1 taken
// as UTC (< 0.9 s difference = < 14 arcsec, absorbed by any sync).
inline double gmst(double unixSec) {
  double d = (unixSec - UNIX_J2000) / 86400.0;
  double T = d / 36525.0;
  // split 360.98564736629*d into whole turns + remainder to keep precision
  double deg = std::fmod(280.46061837 + 0.98564736629 * d, 360.0)
             + 360.0 * (d - std::floor(d))
             + T * T * (0.000387933 - T / 38710000.0);
  return wrap2Pi(deg * kDeg);
}
// local sidereal time; longitude east-positive, radians
inline double lst(double unixSec, double longitude) { return wrap2Pi(gmst(unixSec) + longitude); }

// ---------------------------------------------------------------- horizon coordinates
// Azimuth from north through east, altitude above the horizon. latitude north-positive.
inline void haDecToAltAz(double ha, double dec, double latitude, double &alt, double &az) {
  double sa = std::sin(latitude) * std::sin(dec) + std::cos(latitude) * std::cos(dec) * std::cos(ha);
  alt = std::asin(sa > 1 ? 1 : sa < -1 ? -1 : sa);
  az = wrap2Pi(std::atan2(-std::cos(dec) * std::sin(ha),
                          std::sin(dec) * std::cos(latitude) - std::cos(dec) * std::cos(ha) * std::sin(latitude)));
}

// Horizon frame (x = south, y = east, z = zenith) -> sky frame, for a level mount.
// A point at azimuth Az has horizon-frame longitude pi - Az.
inline Mat3 horizonToSky(double latitude) {
  double s = std::sin(latitude), c = std::cos(latitude);
  return fromColumns({s, 0, -c}, {0, 1, 0}, {c, 0, s});
}

// ---------------------------------------------------------------- alignment model
enum MountMode { MOUNT_EQ = 0, MOUNT_ALTAZ = 1 };

struct SyncStar { double raw0, raw1, ha, dec; };   // raw axis angles + where the star was (sky frame)

struct AlignModel {
  MountMode mode   = MOUNT_EQ;
  double latitude  = 0;       // radians, north +
  double o0 = 0, z1 = 0;      // instrument angle = raw angle + offset
  Mat3   T = identity();      // instrument -> sky
  static const int MAX_STARS = 8;
  SyncStar stars[MAX_STARS];
  int nStars = 0;
};

// Nominal instrument->sky rotation for a perfectly set-up mount.
inline Mat3 nominalT(MountMode mode, double latitude) {
  if (mode == MOUNT_ALTAZ) return horizonToSky(latitude);
  // EQ: polar axis on the visible pole. Southern hemisphere = turned over about x.
  return latitude >= 0 ? identity() : rotX(kPi);
}

// Back to the nominal model, assuming the mount sits at its home position with
// both encoders zeroed there:
//   EQ    : tube parallel to the polar axis (pointing at the pole), HA zero = 0
//   ALTAZ : tube level, pointing north
inline void modelReset(AlignModel &m, MountMode mode, double latitude) {
  m.mode = mode;
  m.latitude = latitude;
  m.T = nominalT(mode, latitude);
  m.nStars = 0;
  if (mode == MOUNT_ALTAZ) { m.o0 = kPi; m.z1 = 0; }   // north = horizon longitude pi
  else                     { m.o0 = 0;   m.z1 = kPi / 2; }
}

inline Vec3 instVec(const AlignModel &m, double raw0, double raw1) {
  return sphToVec(raw0 + m.o0, raw1 + m.z1);
}

// Where is the mount pointing? (raw axis angles -> HA/Dec)
inline void rawToHaDec(const AlignModel &m, double raw0, double raw1, double &ha, double &dec) {
  vecToHaDec(mul(m.T, instVec(m, raw0, raw1)), ha, dec);
}

// Raw axis angles to point at (ha, dec). The secondary axis takes the
// principal solution (|t1| <= 90 deg). For the primary axis, the full-turn
// equivalent nearest nearRaw0 is chosen, unless that would put it more than
// raw0Limit from home (cable wrap), in which case the other way round is used.
inline void haDecToRaw(const AlignModel &m, double ha, double dec, double nearRaw0, double raw0Limit,
                       double &raw0, double &raw1) {
  double t0, t1;
  vecToSph(mulT(m.T, haDecToVec(ha, dec)), t0, t1);
  raw1 = t1 - m.z1;
  double r = t0 - m.o0;
  r = nearRaw0 + wrapPi(r - nearRaw0);
  if (std::fabs(r) > raw0Limit) r += (r > 0 ? -k2Pi : k2Pi);
  raw0 = r;
}

struct SyncReport {
  int    nStars = 0;
  double rmsResidual = 0, maxResidual = 0;   // radians, over all sync stars
  double z1Change = 0;                       // how far the axis-1 zero moved
  double axisTilt = 0;                       // primary axis vs. where the nominal setup puts it
  bool   replaced = false;                   // new star was close to an old one and replaced it
  bool   mirrored = false;                   // 3+ stars: handedness mismatch = an axis direction is reversed
  bool   z1Unsolved = false;                 // separations couldn't be matched (bad star / ill-conditioned)
};

// sum over star pairs of (instrument separation - sky separation)^2, as a function of z1
inline double separationCost(const AlignModel &m, double z1) {
  double cost = 0;
  for (int i = 0; i < m.nStars; i++)
    for (int j = i + 1; j < m.nStars; j++) {
      const SyncStar &a = m.stars[i], &b = m.stars[j];
      double si = angleBetween(sphToVec(a.raw0, a.raw1 + z1), sphToVec(b.raw0, b.raw1 + z1));
      double ss = angleBetween(haDecToVec(a.ha, a.dec), haDecToVec(b.ha, b.dec));
      cost += (si - ss) * (si - ss);
    }
  return cost;
}

// Solve the axis-1 zero offset: global scan +-30 deg around the current value,
// golden-section refine of every local minimum, keep the lowest -- and among
// (near-)equal minima the one nearest the current value (mirror solutions).
inline bool solveZ1(AlignModel &m) {
  const double span = 30 * kDeg, step = 0.25 * kDeg;
  const int N = (int)(2 * span / step);
  double z0 = m.z1, bestZ = z0, bestCost = 1e300;
  double prev2 = 0, prev1 = 0;
  for (int k = 0; k <= N; k++) {
    double c = separationCost(m, z0 - span + k * step);
    if (k >= 2 && prev1 <= prev2 && prev1 <= c) {        // local minimum at k-1
      double lo = z0 - span + (k - 2) * step, hi = z0 - span + k * step;
      const double g = 0.6180339887498949;
      double x1 = hi - g * (hi - lo), x2 = lo + g * (hi - lo);
      double f1 = separationCost(m, x1), f2 = separationCost(m, x2);
      for (int it = 0; it < 60; it++) {
        if (f1 < f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - g * (hi - lo); f1 = separationCost(m, x1); }
        else         { lo = x1; x1 = x2; f1 = f2; x2 = lo + g * (hi - lo); f2 = separationCost(m, x2); }
      }
      double z = 0.5 * (lo + hi), f = separationCost(m, z);
      const double tie = (0.02 * kDeg) * (0.02 * kDeg);
      bool better = f < bestCost - tie || (f <= bestCost + tie && std::fabs(z - z0) < std::fabs(bestZ - z0));
      if (better) { bestCost = f; bestZ = z; }
    }
    prev2 = prev1; prev1 = c;
  }
  int pairs = m.nStars * (m.nStars - 1) / 2;
  double rms = pairs ? std::sqrt(bestCost / pairs) : 0;
  if (bestCost > 1e299 || rms > 0.5 * kDeg) return false;   // keep z1 as it was
  m.z1 = bestZ;
  return true;
}

// Orthonormal triad from two directions, centred on their bisector so the fit
// error is shared equally between them.
inline Mat3 triad(Vec3 a, Vec3 b) {
  Vec3 u = normalize(a + b), w = normalize(cross(a, b)), v = cross(w, u);
  return fromColumns(u, v, w);
}
inline Mat3 transpose(const Mat3 &M) {
  Mat3 R;
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) R.m[i][j] = M.m[j][i];
  return R;
}

// Add a sync star: "the mount, at these raw axis angles, is centred on (ha, dec)".
//   1 star : keep the nominal rotation, set both axis offsets so the star fits exactly.
//   2+     : solve z1 from separations, then fit T to the best-separated pair.
// A star within 3 deg of an earlier one replaces it (re-centring refines the
// model instead of piling up near-duplicates); beyond MAX_STARS the oldest drops.
inline SyncReport addSyncStar(AlignModel &m, double raw0, double raw1, double ha, double dec) {
  SyncReport rep;
  Vec3 sky = haDecToVec(ha, dec);
  int slot = -1;
  for (int i = 0; i < m.nStars; i++)
    if (angleBetween(sky, haDecToVec(m.stars[i].ha, m.stars[i].dec)) < 3 * kDeg) { slot = i; rep.replaced = true; break; }
  if (slot < 0) {
    if (m.nStars == AlignModel::MAX_STARS) {
      for (int i = 1; i < m.nStars; i++) m.stars[i - 1] = m.stars[i];
      m.nStars--;
    }
    slot = m.nStars++;
  }
  m.stars[slot] = {raw0, raw1, ha, dec};

  double oldZ1 = m.z1;
  if (m.nStars == 1) {
    m.T = nominalT(m.mode, m.latitude);
    double t0, t1;
    vecToSph(mulT(m.T, sky), t0, t1);
    m.o0 = wrapPi(t0 - raw0);
    m.z1 = t1 - raw1;
  } else {
    if (!solveZ1(m)) rep.z1Unsolved = true;
    // best-conditioned pair: largest sin(separation) on the sky
    int bi = 0, bj = 1; double best = -1;
    for (int i = 0; i < m.nStars; i++)
      for (int j = i + 1; j < m.nStars; j++) {
        double s = norm(cross(haDecToVec(m.stars[i].ha, m.stars[i].dec), haDecToVec(m.stars[j].ha, m.stars[j].dec)));
        if (s > best) { best = s; bi = i; bj = j; }
      }
    const SyncStar &a = m.stars[bi], &b = m.stars[bj];
    Mat3 S = triad(haDecToVec(a.ha, a.dec), haDecToVec(b.ha, b.dec));
    Mat3 I = triad(instVec(m, a.raw0, a.raw1), instVec(m, b.raw0, b.raw1));
    m.T = mul(S, transpose(I));
  }
  rep.z1Change = m.z1 - oldZ1;

  // residuals
  double sum2 = 0;
  for (int i = 0; i < m.nStars; i++) {
    double r = angleBetween(mul(m.T, instVec(m, m.stars[i].raw0, m.stars[i].raw1)),
                            haDecToVec(m.stars[i].ha, m.stars[i].dec));
    sum2 += r * r;
    if (r > rep.maxResidual) rep.maxResidual = r;
  }
  rep.nStars = m.nStars;
  rep.rmsResidual = std::sqrt(sum2 / m.nStars);

  // primary axis direction vs. nominal (polar-alignment / levelling error, or a sign problem)
  rep.axisTilt = angleBetween(mul(m.T, Vec3{0, 0, 1}), mul(nominalT(m.mode, m.latitude), Vec3{0, 0, 1}));

  // handedness: with 3+ stars, a reversed axis direction mirrors the instrument frame
  if (m.nStars >= 3) {
    for (int i = 0; i + 2 < m.nStars && !rep.mirrored; i++) {
      const SyncStar &a = m.stars[i], &b = m.stars[i + 1], &c = m.stars[i + 2];
      double ts = dot(haDecToVec(a.ha, a.dec), cross(haDecToVec(b.ha, b.dec), haDecToVec(c.ha, c.dec)));
      double ti = dot(instVec(m, a.raw0, a.raw1), cross(instVec(m, b.raw0, b.raw1), instVec(m, c.raw0, c.raw1)));
      if (std::fabs(ts) > 0.05 && std::fabs(ti) > 0.05 && (ts > 0) != (ti > 0)) rep.mirrored = true;
    }
  }
  return rep;
}

} // namespace coord
