// Host unit test for sketches/claude_scopico/calib_config.h (latest sketch)
// build: g++ -std=gnu++17 -Wall -I../sketches/claude_scopico test_calib.cpp -o test_calib && ./test_calib
#include "calib_config.h"
#include <cassert>
#include <string>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)

static AxisCalib sample(int a) {
  AxisCalib c;
  c.ffGain = 0.676631 + a * 0.01; c.ffOffset = 3.5 - a * 1.9; c.kp = 1.23456789; c.ki = 0.3141592653; c.maxIntegral = 40;
  c.kpHold = 0.15; c.kiHold = 0.05; c.maxIntegralHold = 40; c.maxDuty = 40;
  c.maxVel = 51.72; c.maxAccel = 100; c.faultThreshold = 200; c.invertMotor = (a == 1);
  c.gainFast = 0.25; c.gainVSlow = 413.7; c.gainVFast = 4137.4;
  return c;
}
static bool same(const AxisCalib &x, const AxisCalib &y) {
  for (int i = 0; i < CALIB_NFIELDS; i++) {
    double p = x.*CALIB_FIELDS[i].member, q = y.*CALIB_FIELDS[i].member;
    if (fabs(p - q) > 1e-8 * fmax(1.0, fabs(p))) return false;
  }
  return x.invertMotor == y.invertMotor;
}
static bool parse(std::string text, AxisCalib out[2], char *err) {
  std::string copy = text;
  return parseCalib(&copy[0], out, err, 96);
}

int main() {
  char buf[1024], err[96];
  AxisCalib c[2] = {sample(0), sample(1)};
  int len = formatCalib(buf, sizeof(buf), c);
  CHECK(len > 0);
  printf("--- formatted (%d bytes) ---\n%s---\n", len, buf);

  // round trip
  AxisCalib d[2]; memset(d, 0, sizeof(d)); d[0].maxDuty = d[1].maxDuty = 1;
  CHECK(parse(buf, d, err));
  CHECK(same(c[0], d[0]) && same(c[1], d[1]));

  // too-small buffer
  CHECK(formatCalib(buf, 50, c) == -1);

  // CRLF line endings, comments, blank lines, unknown key, missing key keeps prior
  AxisCalib e[2] = {sample(0), sample(1)};
  CHECK(parse("telescope-calib v1\r\n\r\n# hi\r\naxis=1 kp=2.5 futureKey=7\r\naxis=0 ki=0.9\r\n", e, err));
  CHECK(e[1].kp == 2.5 && e[1].ki == sample(1).ki && e[0].ki == 0.9 && e[0].kp == sample(0).kp);

  // rejections leave out[] untouched
  struct { const char *text; } bad[] = {
    {""}, {"wrong header\naxis=0\naxis=1\n"}, {"telescope-calib v1\naxis=0 kp=1\n"},
    {"telescope-calib v1\naxis=0\naxis=0\naxis=1\n"}, {"telescope-calib v1\naxis=2\naxis=0\naxis=1\n"},
    {"telescope-calib v1\naxis=0 kp=abc\naxis=1\n"}, {"telescope-calib v1\naxis=0 kp=1.5x\naxis=1\n"},
    {"telescope-calib v1\naxis=0 kp=nan\naxis=1\n"}, {"telescope-calib v1\naxis=0 maxDuty=150\naxis=1\n"},
    {"telescope-calib v1\naxis=0 ffGain=-1\naxis=1\n"},
    {"telescope-calib v1\naxis=0 ffOffset=-0.5\naxis=1\n"},
    {"telescope-calib v1\naxis=0 gainFast=1.5\naxis=1\n"},
    {"telescope-calib v1\naxis=0 gainVSlow=5000 gainVFast=4000\naxis=1\n"}, {"telescope-calib v1\naxis=0 invertMotor=2\naxis=1\n"},
    {"telescope-calib v1\naxis=0 kp\naxis=1\n"}, {"telescope-calib v1\naxis=0 kp=\naxis=1\n"},
  };
  for (auto &b : bad) {
    AxisCalib f[2] = {sample(0), sample(1)};
    err[0] = 0;
    bool ok = parse(b.text, f, err);
    CHECK(!ok);
    CHECK(same(f[0], sample(0)) && same(f[1], sample(1)));
    printf("rejected ok: %-8s\n", err);
  }
  printf(fails ? "%d FAILURES\n" : "ALL PASSED\n", fails);
  return fails != 0;
}
