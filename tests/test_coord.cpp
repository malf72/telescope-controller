// Host unit test for sketches/claude_scopico/coord_core.h
// build: g++ -std=gnu++17 -Wall -Wextra -I../sketches/claude_scopico test_coord.cpp -o test_coord && ./test_coord
#include "coord_core.h"
#include <cstdio>
#include <cstdlib>

using namespace coord;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static double hms(double h, double m, double s) { return (h + m / 60 + s / 3600) * kHour; }
static double dms(double d, double m, double s) { double sg = d < 0 ? -1 : 1; return sg * (std::fabs(d) + m / 60 + s / 3600) * kDeg; }

// A "real" mount with known setup errors, used to generate encoder readings.
struct TrueMount { AlignModel m; double sign0 = 1, sign1 = 1; };
static void trueRaw(const TrueMount &t, double ha, double dec, double &raw0, double &raw1) {
  haDecToRaw(t.m, ha, dec, 0, 10 * kPi, raw0, raw1);
  raw0 *= t.sign0; raw1 *= t.sign1;            // what the encoders would report
}
static double pointingError(const AlignModel &m, const TrueMount &t, double ha, double dec) {
  double r0, r1, h2, d2;
  trueRaw(t, ha, dec, r0, r1);
  rawToHaDec(m, r0, r1, h2, d2);
  return angleBetween(haDecToVec(ha, dec), haDecToVec(h2, d2));
}

static const double TEST_STARS[][2] = {   // ha (hours), dec (deg)
  {-2, 20}, {3, 60}, {-4.5, -10}, {1, 45}, {5, 5}, {-1, 75}, {0.2, -25}, {2.5, 30}, {-3, 50},
};

static void syntheticMount(MountMode mode, double lat, const char *name) {
  TrueMount t;
  modelReset(t.m, mode, lat);
  // setup errors: 1.5 / 2 deg tilt of the primary axis, arbitrary primary zero,
  // secondary zero 2.3 deg off from the assumed home position
  t.m.T = mul(nominalT(mode, lat), mul(rotX(1.5 * kDeg), rotY(-2.0 * kDeg)));
  t.m.o0 += 37 * kDeg;
  t.m.z1 += 2.3 * kDeg;

  AlignModel m;
  modelReset(m, mode, lat);

  double r0, r1, worst = 0;
  trueRaw(t, TEST_STARS[0][0] * kHour, TEST_STARS[0][1] * kDeg, r0, r1);
  addSyncStar(m, r0, r1, TEST_STARS[0][0] * kHour, TEST_STARS[0][1] * kDeg);
  CHECK(pointingError(m, t, TEST_STARS[0][0] * kHour, TEST_STARS[0][1] * kDeg) < 1e-9);
  double oneStarWorst = 0;
  for (auto &s : TEST_STARS) oneStarWorst = fmax(oneStarWorst, pointingError(m, t, s[0] * kHour, s[1] * kDeg));

  trueRaw(t, TEST_STARS[1][0] * kHour, TEST_STARS[1][1] * kDeg, r0, r1);
  SyncReport rep = addSyncStar(m, r0, r1, TEST_STARS[1][0] * kHour, TEST_STARS[1][1] * kDeg);
  CHECK(!rep.z1Unsolved);
  CHECK(rep.nStars == 2);
  for (auto &s : TEST_STARS) worst = fmax(worst, pointingError(m, t, s[0] * kHour, s[1] * kDeg));
  printf("%-22s 1-star worst %7.3f deg | 2-star worst %.4f arcsec, rms resid %.4f arcsec, tilt %.2f deg\n",
         name, oneStarWorst / kDeg, worst / kArcsec, rep.rmsResidual / kArcsec, rep.axisTilt / kDeg);
  CHECK(worst < 1 * kArcsec);
  CHECK(fabs(rep.axisTilt / kDeg - 2.5) < 0.01);   // sqrt(1.5^2 + 2^2)

  trueRaw(t, TEST_STARS[2][0] * kHour, TEST_STARS[2][1] * kDeg, r0, r1);
  rep = addSyncStar(m, r0, r1, TEST_STARS[2][0] * kHour, TEST_STARS[2][1] * kDeg);
  CHECK(rep.nStars == 3 && !rep.mirrored && rep.maxResidual < 1 * kArcsec);
}

int main() {
  // --- sidereal time: Meeus examples 12.a and 12.b
  double t1 = unixFromCivil(1987, 4, 10, 0, 0, 0);
  double g1 = gmst(t1) / kHour * 3600;                   // seconds of sidereal time
  CHECK(fabs(g1 - hms(13, 10, 46.3668) / kHour * 3600) < 0.01);
  double t2 = unixFromCivil(1987, 4, 10, 19, 21, 0);
  double g2 = gmst(t2) / kHour * 3600;
  CHECK(fabs(g2 - hms(8, 34, 57.0896) / kHour * 3600) < 0.01);
  printf("GMST 1987-04-10 0h      = %.4f s (Meeus 47446.3668)\n", g1);
  printf("GMST 1987-04-10 19:21   = %.4f s (Meeus 30897.0896)\n", g2);
  CHECK(fabs(julianDate(t1) - 2446895.5) < 1e-9);

  // civil round trip
  long y; unsigned mo, d; int hh, mi; double ss;
  civilFromUnix(unixFromCivil(2026, 10, 9, 21, 7, 33.25), y, mo, d, hh, mi, ss);
  CHECK(y == 2026 && mo == 10 && d == 9 && hh == 21 && mi == 7 && fabs(ss - 33.25) < 1e-6);
  civilFromUnix(unixFromCivil(2024, 2, 29, 0, 0, 0), y, mo, d, hh, mi, ss);
  CHECK(y == 2024 && mo == 2 && d == 29);

  // --- alt/az: Meeus example 13.b (Venus from Washington, using his hour angle)
  double alt, az;
  haDecToAltAz(64.352133 * kDeg, dms(-6, 43, 11.61), dms(38, 55, 17), alt, az);
  printf("Meeus 13.b: alt %.4f (15.1249)  az %.4f (68.0337 + 180)\n", alt / kDeg, az / kDeg);
  CHECK(fabs(alt / kDeg - 15.1249) < 1e-4);
  CHECK(fabs(az / kDeg - 248.0337) < 1e-4);

  // horizon frame matrix agrees with the alt/az formulas everywhere
  double worstH = 0;
  for (double lat = -60; lat <= 60; lat += 30)
    for (double h = -11; h <= 11; h += 2.5)
      for (double dc = -80; dc <= 80; dc += 20) {
        haDecToAltAz(h * kHour, dc * kDeg, lat * kDeg, alt, az);
        Vec3 v = mul(horizonToSky(lat * kDeg), sphToVec(kPi - az, alt));
        worstH = fmax(worstH, angleBetween(v, haDecToVec(h * kHour, dc * kDeg)));
      }
  CHECK(worstH < 1e-9);

  // --- nominal model round trips
  for (int mode = 0; mode < 2; mode++) {
    AlignModel m; modelReset(m, (MountMode)mode, 51.5 * kDeg);
    double worst = 0;
    for (auto &s : TEST_STARS) {
      double r0, r1, h, dd;
      haDecToRaw(m, s[0] * kHour, s[1] * kDeg, 0, 1.5 * kPi, r0, r1);
      rawToHaDec(m, r0, r1, h, dd);
      worst = fmax(worst, angleBetween(haDecToVec(h, dd), haDecToVec(s[0] * kHour, s[1] * kDeg)));
    }
    CHECK(worst < 1e-9);
  }
  // home positions: EQ home points at the pole, alt-az home is level and due north
  {
    AlignModel m; double h, dd;
    modelReset(m, MOUNT_EQ, 51.5 * kDeg);   rawToHaDec(m, 0, 0, h, dd); CHECK(fabs(dd - kPi / 2) < 1e-9);
    modelReset(m, MOUNT_EQ, -33 * kDeg);    rawToHaDec(m, 0, 0, h, dd); CHECK(fabs(dd + kPi / 2) < 1e-9);
    modelReset(m, MOUNT_ALTAZ, 51.5 * kDeg); rawToHaDec(m, 0, 0, h, dd);
    haDecToAltAz(h, dd, 51.5 * kDeg, alt, az);
    CHECK(fabs(alt) < 1e-9 && fabs(wrapPi(az)) < 1e-9);
  }

  // --- synthetic misaligned mounts: 2 sync stars must recover them exactly
  syntheticMount(MOUNT_EQ, 51.5 * kDeg, "EQ north 51.5");
  syntheticMount(MOUNT_EQ, -33.9 * kDeg, "EQ south -33.9");
  syntheticMount(MOUNT_ALTAZ, 51.5 * kDeg, "alt-az 51.5");
  syntheticMount(MOUNT_ALTAZ, 10 * kDeg, "alt-az 10");

  // --- reversed axis direction is flagged once 3 stars are in
  {
    TrueMount t; modelReset(t.m, MOUNT_EQ, 51.5 * kDeg); t.sign0 = -1;
    AlignModel m; modelReset(m, MOUNT_EQ, 51.5 * kDeg);
    SyncReport rep;
    for (int i = 0; i < 3; i++) {
      double r0, r1;
      trueRaw(t, TEST_STARS[i][0] * kHour, TEST_STARS[i][1] * kDeg, r0, r1);
      rep = addSyncStar(m, r0, r1, TEST_STARS[i][0] * kHour, TEST_STARS[i][1] * kDeg);
    }
    printf("reversed axis 0: mirrored=%d tilt=%.1f deg maxResid=%.2f deg\n", rep.mirrored, rep.axisTilt / kDeg, rep.maxResidual / kDeg);
    CHECK(rep.mirrored);
  }

  // --- re-syncing near an existing star replaces it; MAX_STARS caps the list
  {
    AlignModel m; modelReset(m, MOUNT_EQ, 51.5 * kDeg);
    addSyncStar(m, 0.1, 0.2, 0, 30 * kDeg);
    SyncReport rep = addSyncStar(m, 0.11, 0.21, 0.01, 31 * kDeg);
    CHECK(rep.replaced && rep.nStars == 1);
    for (int i = 0; i < 12; i++) rep = addSyncStar(m, 0.3 * i, 0.1, 0.3 * i, (10 + 5 * i) * kDeg);
    CHECK(rep.nStars == AlignModel::MAX_STARS);
  }

  // --- cable-wrap aware primary-axis choice
  {
    AlignModel m; modelReset(m, MOUNT_EQ, 51.5 * kDeg);
    double r0, r1;
    haDecToRaw(m, -3 * kHour, 20 * kDeg, 0, 1.5 * kPi, r0, r1);          // 3h east of meridian
    CHECK(fabs(r0 - 3 * kHour) < 1e-9);
    haDecToRaw(m, -3 * kHour, 20 * kDeg, 3 * kHour + k2Pi, 1.5 * kPi, r0, r1);  // near a full turn: unwinds
    CHECK(fabs(r0 - 3 * kHour) < 1e-9);
  }

  printf(fails ? "%d FAILURES\n" : "ALL PASSED\n", fails);
  return fails != 0;
}
