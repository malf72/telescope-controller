// Host unit test for sketches/claude_scopico/sky_config.h
// build: g++ -std=gnu++17 -Wall -Wextra -I../sketches/claude_scopico test_sky.cpp -o test_sky && ./test_sky
#include "sky_config.h"
#include <string>

using namespace coord;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static bool parseSkyStr(std::string s, SkySettings &o, char *err) { return parseSky(&s[0], o, err, 96); }
static bool parseParkStr(std::string s, AlignModel &o, char *err) { long a, b; return parsePark(&s[0], o, a, b, err, 96); }

int main() {
  char buf[2048], err[96];

  // --- sky settings round trip
  SkySettings s; s.mode = MOUNT_ALTAZ; s.siteSet = true; s.latDeg = 51.4779; s.lonDeg = -0.0015;
  s.sign0 = -1; s.sign1 = 1; s.horizonDeg = 12.5; s.rev1 = 3342154.75; s.jogMax = 0.85;
  int len = formatSky(buf, sizeof buf, s);
  CHECK(len > 0);
  printf("%s", buf);
  SkySettings r;
  CHECK(parseSky(buf, r, err, sizeof err));
  CHECK(r.mode == MOUNT_ALTAZ && r.siteSet && fabs(r.latDeg - 51.4779) < 1e-9 && fabs(r.lonDeg + 0.0015) < 1e-9 &&
        r.sign0 == -1 && r.sign1 == 1 && r.horizonDeg == 12.5 &&
        fabs(r.rev0 - COUNTS_PER_REV) < 0.01 && fabs(r.rev1 - 3342154.75) < 0.01 && fabs(r.jogMax - 0.85) < 1e-9);

  // unknown keys ignored, missing keys keep prior values, CRLF ok
  SkySettings q; q.horizonDeg = 7;
  CHECK(parseSkyStr("telescope-sky v1\r\nmode=eq lat=10 future=3\r\n", q, err));
  CHECK(q.mode == MOUNT_EQ && q.latDeg == 10 && q.horizonDeg == 7);

  const char *badSky[] = {"", "wrong\nmode=eq\n", "telescope-sky v1\nmode=dob\n", "telescope-sky v1\nlat=95\n",
                          "telescope-sky v1\nsign0=2\n", "telescope-sky v1\nlon=abc\n", "telescope-sky v1\nhorizon=80\n",
                          "telescope-sky v1\nlat\n", "telescope-sky v1\nrev1=5\n", "telescope-sky v1\njogmax=1.5\n"};
  for (const char *b : badSky) {
    SkySettings t; t.latDeg = 42;
    CHECK(!parseSkyStr(b, t, err));
    CHECK(t.latDeg == 42);                       // untouched on failure
    printf("sky rejected ok: %s\n", err);
  }

  // --- park round trip with a non-trivial model
  AlignModel m; modelReset(m, MOUNT_EQ, 51.5 * kDeg);
  addSyncStar(m, 0.31, 0.62, -0.5, 0.4);
  addSyncStar(m, 1.12, 0.21, 0.9, 0.2);
  addSyncStar(m, -0.4, 0.95, 0.2, 1.0);
  len = formatPark(buf, sizeof buf, m, -123456, 789012);
  CHECK(len > 0);
  printf("%s", buf);
  AlignModel p; long pos0 = 0, pos1 = 0;
  CHECK(parsePark(buf, p, pos0, pos1, err, sizeof err));
  CHECK(pos0 == -123456 && pos1 == 789012);
  CHECK(p.mode == m.mode && p.nStars == 3 && fabs(p.o0 - m.o0) < 1e-11 && fabs(p.z1 - m.z1) < 1e-11 &&
        fabs(p.latitude - m.latitude) < 1e-11);
  double worst = 0;
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) worst = fmax(worst, fabs(p.T.m[i][j] - m.T.m[i][j]));
  CHECK(worst < 1e-13);
  // the restored model points identically
  double h1, d1, h2, d2;
  rawToHaDec(m, 0.7, 0.3, h1, d1); rawToHaDec(p, 0.7, 0.3, h2, d2);
  CHECK(fabs(h1 - h2) < 1e-11 && fabs(d1 - d2) < 1e-11);

  // corrupt park files are rejected
  std::string good = buf;
  const char *badPark[] = {
    "telescope-park v1\nmode=eq lat=0.9 o0=0 z1=1.5 pos=0,0\n",                          // no T
    "telescope-park v1\nmode=eq lat=0.9 o0=0 z1=1.5 T=1,0,0,0,1,0,0,0,1\n",              // no pos
    "telescope-park v1\nmode=eq lat=0.9 o0=0 z1=1.5 pos=0,0 T=1,0,0,0,1,0,0,0\n",                // 8 values
    "telescope-park v1\nmode=eq lat=0.9 o0=0 z1=1.5 pos=0,0 T=2,0,0,0,1,0,0,0,1\n",              // not a rotation
    "telescope-park v1\nmode=eq lat=0.9 o0=0 z1=1.5 pos=0,0 T=1,0,0,0,1,0,0,0,-1\n",             // mirrored
    "telescope-park v1\nmode=eq lat=0.9 o0=0 z1=1.5 pos=0,0 T=1,0,0,0,1,0,0,0,1\nstar=1,2,3\n",  // short star
  };
  for (const char *b : badPark) {
    AlignModel t; t.o0 = 9;
    CHECK(!parseParkStr(b, t, err));
    CHECK(t.o0 == 9);
    printf("park rejected ok: %s\n", err);
  }

  printf(fails ? "%d FAILURES\n" : "ALL PASSED\n", fails);
  return fails != 0;
}
