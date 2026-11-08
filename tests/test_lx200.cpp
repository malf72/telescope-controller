// Host unit test for claude_scopico/lx200_core.h
// build: g++ -std=gnu++17 -Wall -Wextra -I../claude_scopico test_lx200.cpp -o test_lx200 && ./test_lx200
#include "lx200_core.h"
#include <string>

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL line %d: %s\n", __LINE__, #c); fails++; } } while (0)
#define CHECK_STR(a, b) do { std::string x_ = (a), y_ = (b); if (x_ != y_) { \
  printf("FAIL line %d: got \"%s\" want \"%s\"\n", __LINE__, x_.c_str(), y_.c_str()); fails++; } } while (0)

static const double D2R = LX_PI / 180.0, H2R = LX_PI / 12.0;

struct FakeMount : Lx200Mount {
  double ra = 5.5 * H2R, dec = -12.25 * D2R, alt = 33.5 * D2R, az = 271.0 * D2R;
  double lat = 51.5 * D2R, lon = -0.125 * D2R, utc = 0;
  bool known = true, slewing = false, eq = true, tracking = false;
  int stars = 0;
  int gotoResult = 0;
  double lastGotoRa = -1, lastGotoDec = -1, lastSyncRa = -1, lastSyncDec = -1;
  std::string moves; char rate = 0; int stops = 0;
  bool getRaDec(double &r, double &d) override { r = ra; d = dec; return known; }
  bool getAltAz(double &a, double &z) override { a = alt; z = az; return known; }
  int gotoTarget(double r, double d, char *msg, size_t n) override {
    lastGotoRa = r; lastGotoDec = d;
    if (gotoResult == 1) snprintf(msg, n, "Below horizon");
    return gotoResult;
  }
  bool sync(double r, double d, char *, size_t) override { lastSyncRa = r; lastSyncDec = d; return true; }
  void stop() override { stops++; }
  void move(char dir, bool start) override { moves += start ? (char)toupper(dir) : dir; }
  void setRate(char r) override { rate = r; }
  bool isSlewing() override { return slewing; }
  bool isEquatorial() override { return eq; }
  bool isTracking() override { return tracking; }
  int  alignStars() override { return stars; }
  bool getSite(double &la, double &lo) override { la = lat; lo = lon; return true; }
  void setLatitude(double la) override { lat = la; }
  void setLongitude(double lo) override { lon = lo; }
  bool getUtc(double &u) override { u = utc; return true; }
  void setUtc(double u) override { utc = u; }
};

// run a byte stream through framer + dispatcher, return all replies concatenated
static std::string run(Lx200State &st, FakeMount &m, const char *bytes) {
  Lx200Framer f;
  std::string all;
  char cmd[64], out[96];
  for (const char *p = bytes; *p; p++) {
    int r = lx200Feed(f, *p, cmd, sizeof(cmd));
    if (r == LX_CMD) { lx200Handle(cmd, st, m, out, sizeof(out)); all += out; }
    else if (r == LX_ACK) { lx200Ack(m, out, sizeof(out)); all += out; }
  }
  return all;
}

int main() {
  FakeMount m; Lx200State st;
  char out[64];

  // --- formatting, incl. rounding carries
  lx200FormatRa(5.5 * H2R, true, out, sizeof out);                         CHECK_STR(out, "05:30:00#");
  lx200FormatRa((23 + 59.0 / 60 + 59.7 / 3600) * H2R, true, out, sizeof out); CHECK_STR(out, "00:00:00#");
  lx200FormatRa((12 + 34.56 / 60) * H2R, false, out, sizeof out);          CHECK_STR(out, "12:34.6#");
  lx200FormatDec(-12.25 * D2R, true, out, sizeof out);                     CHECK_STR(out, "-12*15:00#");
  lx200FormatDec((45 + 59.0 / 60 + 59.8 / 3600) * D2R, true, out, sizeof out); CHECK_STR(out, "+46*00:00#");
  lx200FormatDec(-0.2 / 60 * D2R, false, out, sizeof out);                 CHECK_STR(out, "-00*00#");
  lx200FormatAz(271 * D2R, true, out, sizeof out);                         CHECK_STR(out, "271*00:00#");
  lx200FormatAz(359.9999 * D2R, false, out, sizeof out);                   CHECK_STR(out, "000*00#");

  // --- parsing, all the formats clients send
  double v;
  CHECK(lx200ParseRa("12:34:56", v) && fabs(v - (12 + 34 / 60.0 + 56 / 3600.0) * H2R) < 1e-12);
  CHECK(lx200ParseRa(" 12:34.5", v) && fabs(v - (12 + 34.5 / 60) * H2R) < 1e-12);
  CHECK(lx200ParseRa("12:34:56.78", v) && fabs(v - (12 + 34 / 60.0 + 56.78 / 3600.0) * H2R) < 1e-12);
  CHECK(!lx200ParseRa("24:00:00", v) && !lx200ParseRa("12:60:00", v) && !lx200ParseRa("12", v));
  CHECK(lx200ParseDec("-05*30:15", v) && fabs(v + (5 + 30 / 60.0 + 15 / 3600.0) * D2R) < 1e-12);
  CHECK(lx200ParseDec("+45\xDF" "30", v) && fabs(v - 45.5 * D2R) < 1e-12);
  CHECK(lx200ParseDec(" -00*30'00", v) && fabs(v + 0.5 * D2R) < 1e-12);
  CHECK(lx200ParseDec("+89:59:59.9", v));
  CHECK(!lx200ParseDec("+91*00", v) && !lx200ParseDec("+90*01", v));

  // --- what INDI lx200basic does: handshake read, set target, goto, poll, sync, abort
  CHECK_STR(run(st, m, ":GR#"), "05:30:00#");
  CHECK_STR(run(st, m, ":GD#"), "-12*15:00#");
  CHECK_STR(run(st, m, ":Sr 10:20:30#:Sd +20*30:40#"), "11");
  CHECK_STR(run(st, m, ":MS#"), "0");
  CHECK(fabs(m.lastGotoRa - (10 + 20 / 60.0 + 30 / 3600.0) * H2R) < 1e-12);
  CHECK(fabs(m.lastGotoDec - (20 + 30 / 60.0 + 40 / 3600.0) * D2R) < 1e-12);
  m.gotoResult = 1;
  CHECK_STR(run(st, m, ":MS#"), "1Below horizon#");
  CHECK_STR(run(st, m, ":CM#"), "Coordinates     matched.        #");
  CHECK(fabs(m.lastSyncRa - (10 + 20 / 60.0 + 30 / 3600.0) * H2R) < 1e-12);
  CHECK_STR(run(st, m, ":Q#"), ""); CHECK(m.stops == 1);
  CHECK_STR(run(st, m, ":Sr 99:00:00#"), "0");

  // --- no target yet
  { Lx200State s2; FakeMount m2;
    CHECK_STR(run(s2, m2, ":MS#"), "2No target set#");
    CHECK(m2.lastGotoRa < 0); }

  // --- framing: noise, CR/LF, lone '#', restart on ':', ACK
  CHECK_STR(run(st, m, "#\r\n  :GR#"), "05:30:00#");
  CHECK_STR(run(st, m, ":G\r\nR#"), "05:30:00#");                   // CR/LF inside are dropped
  CHECK_STR(run(st, m, "\x06"), "P");
  m.eq = false; CHECK_STR(run(st, m, "\x06"), "A"); m.eq = true;
  CHECK_STR(run(st, m, ":XYZ#:GVP#"), "LX90 Pico#");     // unknown command: silent

  // --- :GW# status (Stellarium Mobile polls it)
  CHECK_STR(run(st, m, ":GW#"), "PN0#");
  m.tracking = true; m.stars = 5; m.eq = false;
  CHECK_STR(run(st, m, ":GW#"), "AT3#");
  m.tracking = false; m.stars = 0; m.eq = true;

  // --- precision toggle
  CHECK_STR(run(st, m, ":U#:GR#:GD#:U#"), "05:30.0#-12*15#");

  // --- motion and rates
  CHECK_STR(run(st, m, ":RC#:Mn#:Me#:Qn#:Qe#:Mx#"), "");
  CHECK(m.rate == 'C'); CHECK_STR(m.moves, "NEne");
  m.slewing = true;  CHECK_STR(run(st, m, ":D#"), "|#");
  m.slewing = false; CHECK_STR(run(st, m, ":D#"), "#");

  // --- site (Meade longitude is west-positive)
  CHECK_STR(run(st, m, ":Gt#:Gg#"), "+51*30#+000*08#");
  CHECK_STR(run(st, m, ":St+40*30#:Sg074*00#"), "11");
  CHECK(fabs(m.lat - 40.5 * D2R) < 1e-12 && fabs(m.lon + 74 * D2R) < 1e-12);
  CHECK_STR(run(st, m, ":Sg-010*30#"), "1");
  CHECK(fabs(m.lon - 10.5 * D2R) < 1e-12);
  CHECK_STR(run(st, m, ":Sg350*00#"), "1");             // 350 W = 10 E
  CHECK(fabs(m.lon - 10 * D2R) < 1e-9);

  // --- SkySafari-style time setting: offset, local time, then date
  // local 2026-10-09 21:30:15 at UTC-5 (SG +05.0 = add 5 h to get UTC) -> 2026-10-10 02:30:15 UTC
  CHECK_STR(run(st, m, ":SG+05.0#:SL21:30:15#"), "11");
  CHECK_STR(run(st, m, ":SC10/09/26#"), "1Updating Planetary Data#                              #");
  double expect = 1791599415.0;   // 2026-10-10 02:30:15 UTC (checked with Python datetime)
  CHECK(fabs(m.utc - expect) < 1e-6);
  CHECK_STR(run(st, m, ":GL#:GC#:GG#"), "21:30:15#10/09/26#+05.0#");
  CHECK_STR(run(st, m, ":SC13/40/26#"), "0");

  printf(fails ? "%d FAILURES\n" : "ALL PASSED\n", fails);
  return fails != 0;
}
