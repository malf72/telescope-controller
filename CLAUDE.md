# Meade LX90 Telescope Controller — Project Context

This file is read automatically by Claude Code at the start of every session in
this directory. It summarizes a long planning/design conversation done in
claude.ai chat before this project folder existed, so Claude Code has the
background without the person needing to re-explain it.

## Goal

Replace the failed original control electronics on a Meade LX90 telescope with
a custom controller built on a Raspberry Pi Pico 2 W, programmed in Arduino
C++. Requirements:
- Connect to home WiFi
- Act as a telescope server for the Stellarium desktop app (Telescope Control
  plugin, LX200-style protocol over TCP)
- Serve a small setup web page, to be written in **Elm**, for site/time entry,
  alignment, and manual control
- Support both equatorial (native) and alt-az tracking modes eventually

## Hardware (confirmed by bench measurement, not assumption)

- **MCU:** Raspberry Pi Pico 2 W (RP2350), arduino-pico core (Earle Philhower)
- **Motors:** 12V DC gear motors (not steppers), one per axis (RA/Dec), with
  a gearbox + worm drive to the axis
- **Motor driver (since 2026-11-01):** two DRV8871 boards, one per axis
  (IN1/IN2 control only: 00 coast, 10 fwd, 01 rev, 11 brake; 3.6 A peak,
  internal current limit). Previously one Adafruit TB6612 (1.2 A cont.).
- **Encoders:** optical quadrature, mounted on the **motor shaft** (not the
  axis), 5V signal, A/B outputs, **432 quadrature counts per motor rev**
  (108 lines x 4 edges). CORRECTED 2026-10-09: this line used to say "108
  counts/rev confirmed (quadrature counts)" -- that was wrong and made every
  axis angle 4x off; confirmed by measuring both axes on the mount.
- **Gearing:** gearbox ratio 6859/128 = 53.5859375 : 1, worm ratio 154:1,
  total = 8252.234375 motor revs per axis rev
  - Gear train CONFIRMED on the az gearbox (2026-11): 12T motor pinion ->
    38/16 compound -> 38/16 compound -> 38/16 compound -> 48T output
    = (38/12)(38/16)(38/16)(48/16) = 6859/128. Matches the mount-measured
    scale (~3,565,000 counts/turn = 432 x 53.59 x 154) to 0.04 %. The `cap`
    command labels drag peaks with these shafts' rotation rates.
- **Wiring (2026-11-01 rebuild, used by sketch #6 `claude_scopico`):**
  encoders through 10k/20k dividers from 5 V (GP26/27 are ADC-capable pins,
  not 5 V tolerant -- dividers required)
  - Az (axis 0): encoder ch1 GP6, ch2 GP7; DRV8871 IN1 GP12, IN2 GP13 (PWM slice 6)
  - Alt (axis 1): encoder ch1 GP27, ch2 GP26; DRV8871 IN1 GP20, IN2 GP21 (PWM slice 2)
  - The PIO decoder reads (pin, pin+1) from the lower pin, so the alt encoder
    counts opposite to its ch1/ch2 labelling; invertMotor[] absorbs it.
- **Old wiring (sketches #1-#5, TB6612 -- no longer matches the hardware):**
  RA enc GP2/3, Dec enc GP4/5, PWMA GP10 AIN1 GP11 AIN2 GP12, PWMB GP13
  BIN1 GP14 BIN2 GP15, STBY GP16.
- **Axis scale (CORRECTED 2026-10-09, supersedes the CPR/gearing above for
  pointing):** both axes = **3,564,965 counts per axis turn**, set on the
  Pico with `scale <axis> rev 3564965` (stored in `/sky.txt` as rev0/rev1).
  The person confirmed an earlier assumption was wrong; the value equals
  432 x 6859/128 x 154 = 3,564,965.25, i.e. 432 counts per motor rev
  (108 lines x 4 quadrature edges), not 108. Measured values on the mount
  (az 3,566,471 full-turn, alt 3,582,072 inclinometer) agreed within 0.5 %.
  Earlier az results (2.59M, 4.78M) were wrong -- most likely azimuth clutch
  slip: with the encoder on the motor shaft, a slipping clutch inflates
  counts and nothing in the firmware can detect it. Keep the clutches tight.
  Fixed in sketch #5 (`telescope_wifi`): coord::CPR = 432, so SIDEREAL_CPS
  = 41.37 counts/s (true sidereal). Sketches #1-#4 still say 108 -- their
  "sidereal" (10.34 c/s) is really 1/4 sidereal; the ~7 % duty bench figure
  was measured at that 10.34 c/s.
- Az "stall" faults (2026-10 TB6612, 2026-11 DRV8871): at slew speed the
  motor suddenly drops to a fraction of its speed with 40 % duty applied.
  Traced (2026-11) with the 500 Hz `cap` recorder and the fault black box:
  the relay-tuned moving gains (kp ~0.6, ki ~14) make the az servo
  OSCILLATE at ~28 Hz (+-13 % duty) at speed -- the relay test runs near
  standstill where friction damps the motor. The current swings then
  collapse the drive (likely driver current protection; not proven) into a
  following-error fault. Halving kp/ki removed the oscillation (duty
  +-2.7 %, drive train otherwise smooth, <1 % ripple at gear rates) and the
  faults stopped; re-running `auto full` restored the old gains and the
  faults came back. NOT a mechanical fault. First fix was a blanket 0.5
  on the relay gains, but that cost sidereal tracking accuracy; replaced
  (2026-11-06) by a SPEED SCHEDULE of the moving gains in `servo_core.h`:
  full relay-tuned kp/ki at or below 10x sidereal, 0.25x at or above 100x
  sidereal, log-speed blend between (`gs` command, saved per axis as
  gainFast/gainVSlow/gainVFast). The integral is rescaled on every change of
  effective ki (bumpless), generalising the hold<->moving fix below.
- (2026-11-06) The "12 V" motor supply read ~7.5 V idle and sagged to 4.4 V
  (GP28 monitor, `vsup`) -- below the DRV8871's 6.5 V minimum. That caused
  the reduced speed / drive cut-outs / failed auto full runs of that day;
  fixed on the power side. `auto full` now refuses to save if the supply
  dipped below 10.5 V or a speed test reached < 50x sidereal.
- **Measured facts:**
  - Dec motor needed direction inversion (`invertMotor[1] = true`) so
    positive duty gives positive encoder counts; RA did not.
  - Both axes, both directions: ~7% PWM duty gives sidereal tracking rate
    (~10.345 counts/s). This measurement is the seed value for feedforward
    gain in the firmware, later refined by the `auto ff` auto-tune command.
  - (2026-10-09, from `auto full` results) The motors are FAST: 40 % duty
    gives ~27,700 counts/s (~11°/s at the axis); calibrated mv ≈ 19–21k
    counts/s. The ~1.6–3.5 % duty at sidereal is almost all friction, so a
    feedforward line through zero fitted at sidereal asks ~200x too much
    duty at slew speed → saturates → following-error fault on full-speed
    moves (seen from Stellarium Mobile). Fix in sketch #5: two-point
    feedforward `ffOffset + ffGain*v` (sidereal point + speed-test plateau),
    TRACK-mode velocity ramp in the servo, speed-scaled fault threshold
    (+0.1 s of travel), anti-windup. `tests/test_servo.cpp` reproduces the
    fault (7,062 counts) and the fix (58 counts) on a fitted motor model.
  - (2026-10-09) Second runaway, found from a fault snapshot (asked 331 c/s,
    got 17,500): the HOLD integral routinely winds up to its limit (hold
    error of a few counts stuck in the friction deadband; kiHold*40 = 2 %),
    and the raw value carried into the moving gains (ki ~10-20) meant ~800 %
    duty. Fixed in `servo_core.h`: bumpless rescale of the integral on every
    hold<->moving gain switch, and the I-term capped at maxDuty. Keep this if
    the gain scheduling is ever touched.

## Software status: six sketches, in order, each building on the last

All in `sketches/`, each a complete Arduino sketch folder (`.ino` + a shared
`servo_core.h` math header + a pre-assembled PIO header for the quadrature
decoder).

1. **`telescope_bringup/`** — encoder-only + open-loop motor test sketch.
   Used to confirm CPR, wiring, and direction by hand. Superseded for normal
   use but kept as a diagnostic fallback.
2. **`telescope_servo/`** — first closed-loop version: 1 kHz feedforward + PI
   position servo per axis (PIO quadrature decode, TB6612 direct PWM/GPIO
   writes from a hardware repeating timer ISR), trapezoidal slew profile,
   following-error fault detection. Manually-tuned starting gains.
3. **`telescope_autotune/`** — current/most advanced sketch. Adds:
   - **Gain scheduling**: separate, more aggressive PI gains for
     TRACK/SLEW (`kp`/`ki`) vs. a deliberately gentle, fixed set for a
     static HOLD (`kpHold`/`kiHold`). This split exists because a bench
     simulation (plain C++ code, run against a motor model with stiction —
     not on real hardware) showed that aggressive gains that give excellent
     tracking accuracy **hunt/oscillate against motor stiction the moment
     the target position stops moving**. Do not remove this scheduling or
     auto-tune the hold gains without re-deriving that result.
   - **`auto ff <axis>`**: automated open-loop duty ramp, both directions,
     to measure the feedforward gain (replaces manual bench testing).
   - **`auto pi <axis> [duty] [hysteresis]`**: relay (Åström–Hägglund)
     feedback auto-tune for the moving-mode `kp`/`ki`. Forces a small
     bang-bang oscillation, measures ultimate gain/period, and offers both
     Tyreus-Luyben (gentler) and Ziegler-Nichols (sharper) results.
   - **`auto speed <axis>`**: open-loop step test suggesting `mv`/`ma`
     (max slew velocity/acceleration). Treated as a rough starting point,
     not a rigorous identification.
   - **`auto apply <axis> ff|tl|zn|speed`**: commits a suggested result.
   - The person has confirmed on real hardware that after running
     `auto ff` + `auto pi` and applying the results, **tracking is very
     good**.

None of this has been compiled with the real arduino-pico toolchain in the
environment that produced it (no network access to fetch board packages) —
it was validated by (a) assembling the actual PIO program with the real
`pioasm` tool, (b) porting the servo math verbatim into a standalone C++
program and testing it with g++ against a hand-built motor simulation
(feedforward + PI + stiction + lag), and (c) brace/paren balance checks on
the final files. It has since been flashed and run successfully by the
person through sketch #2 and #3.

4. **`telescope_calib/`** — sketch #3 plus `auto full [tl|zn]` (ff+pi+speed
   on axis 0 then axis 1, results applied together only if all six succeed,
   then saved) and LittleFS persistence (`/calib.txt`, auto-loaded at boot,
   `cfg save|load|show|erase`). File format/parser lives in the pure header
   `calib_config.h`, unit-tested by `tests/test_calib.cpp`. Compiled cleanly
   with the real arduino-pico 6.1.1 core (arduino-cli bundled in the
   flatpak Arduino IDE, FQBN `rp2040:rp2040:rpipico2w:flash=4194304_524288`);
   **not yet run on hardware**. Also fixes a sketch #3 bug: `d`/`x` during an
   auto-tune didn't stop the test from driving the motor. End-of-test
   handover now brakes (TB6612 short brake) and waits for the motor to stop
   before the servo takes over (the `auto speed` coast tripped the
   following-error fault in `auto full`).
5. **`telescope_wifi/`** — current sketch. Sketch #4 plus roadmap items 2–5:
   - 1 kHz servo on **core 1** (own alarm pool created in `setup1()`);
     shared servo state guarded by `SERVO_LOCK()` (spin-lock critical
     section — `noInterrupts()` only masks one core).
   - WiFi (credentials in `/wifi.txt` via `wifi "<ssid>" "<pass>"`), NTP,
     mDNS `telescope.local`, serial console mirrored on telnet port 23
     (all output goes through `Con`, not `Serial`).
   - **LX200 server on TCP 4030** (`lx200_core.h`, pure + host-tested).
     Stellarium's built-in LX200 client is serial-only, so Stellarium
     connects via INDI: `indiserver indi_lx200basic`, Network connection to
     telescope.local:4030 (lx200basic = goto/sync/abort only, no jog).
     Coordinates are JNow (INDI convention).
   - **Coordinate engine + Taki alignment model** (`coord_core.h`, pure +
     host-tested): one rotation-matrix model covers EQ wedge and alt-az; the
     secondary-axis zero offset is solved from star separations (not a
     rotation). 1 star = offsets, 2+ = full model, mirrored-axis detection.
   - Goto (multi-pass to catch sky motion), tracking (axes slower than 0.2
     counts/s use stepped HOLD instead of TRACK, because of the stiction
     hunting result), manual moves, `park` (saves model + counts to
     `/park.txt`, one-shot restore at boot). Site/mount/axis-sign/horizon in
     `/sky.txt` (`sky_config.h`).
   - Compiled cleanly with arduino-pico **6.2.0**; all host tests pass
     (`tests/test_{coord,lx200,sky,calib}.cpp`, run with g++ in a throwaway
     `podman` alpine container — this PC has no host compiler).
     **Not yet run on hardware.** Axis count directions (`msign`) relative
     to the sky are unknown until bench-checked.

6. **`claude_scopico/`** — CURRENT sketch (2026-11-01). Sketch #5 re-targeted
   at the rewired hardware: new pins, DRV8871 drive (slow-decay PWM on both
   IN pins, configured directly with the SDK at 20 kHz, top 1023; coast =
   00, brake = 11; no STBY), and `auto dir <axis|a>` -- an open-loop nudge
   that sets invertMotor from the measured encoder direction. Axes refuse to
   enable until `auto dir` has passed on this hardware (persisted in
   `/hwdir.txt`), because the TB6612-era invertMotor values in the saved
   `/calib.txt` can't be trusted and a wrong direction is a runaway.
   Calibration must be redone (`auto full`) for the new drivers. Compiled
   cleanly with arduino-pico 6.3.0; not yet run on the rewired hardware.

## Previously the immediate next task (now implemented as sketch #4)

The person asked for two things:

1. **A single command that runs the full auto-tune sequence and applies the
   results for both axes** — i.e., sequence `auto ff 0` → apply →
   `auto pi 0` → apply → `auto ff 1` → apply → `auto pi 1` → apply,
   entirely hands-off after one command (something like `auto full`).
2. **Persist calibration to the Pico's onboard flash filesystem** (the
   person has a 512 KB LittleFS partition configured in the Arduino IDE's
   board flash-size menu) so that calibration survives a power cycle, and
   **auto-load it at boot if present**. Needs save/load commands too, and
   sensible behavior when no saved config exists yet (fall back to the
   hardcoded defaults already in `setup()`).

Design intent discussed but not yet written to a file: a simple human-readable
text config (not binary, not requiring ArduinoJson) with one line per axis,
`key=value` pairs, magic header line for future format versioning, covering
per-axis: `ffGain`, `kp`, `ki`, `maxIntegral`, `kpHold`, `kiHold`,
`maxIntegralHold`, `maxDuty`, `maxVel`, `maxAccel`, `faultThreshold`,
`invertMotor`. A non-blocking state machine (matching the existing
`FFTune`/`RelayTune`/`SpeedTune` pattern in `telescope_autotune.ino`) should
drive the "run everything" sequence so it doesn't block `loop()`.

## Longer-term roadmap (items 2–5 implemented in sketch #5, untested on hardware)

1. Bench-validate the auto-tune/persistence sketch thoroughly (fault
   injection, long tracking runs, both axes simultaneously).
2. Move the 1 kHz servo from core 0 to core 1 so WiFi/networking can never
   stall motor control.
3. WiFi connect, NTP time sync, mDNS (`telescope.local`), OTA updates.
4. Coordinate engine (UTC+longitude → LST, RA/Dec ↔ Alt/Az, star-sync
   alignment model) — written and unit-tested as portable C++ on a PC
   first, independent of the Pico.
5. Stellarium LX200-style TCP server on core 0.
6. Full tracking/alignment integration test against Stellarium.
7. Elm web UI: status, site/time entry (with browser geolocation),
   alignment/sync, manual jog, tracking controls, settings — built against
   a mock server first.
8. Enclosure, wiring, power supply finalization, and writing down final
   calibration numbers somewhere durable.

## Conventions worth preserving

- **Version control (since 2026-11-06):** the project is a git repo (branch
  `main`, repo-local identity malf72). Commit before every firmware upload,
  and tag the commit that is actually flashed (e.g. `git tag flashed-2026-11-06a`)
  so "it worked two versions ago" can be checked by rebuilding that exact
  version (`git checkout <tag>`, build, upload).

- Keep the shared math (`servo_core.h`) free of any Arduino/pico-SDK calls,
  so it stays unit-testable with plain g++ — this caught the stiction/hunting
  bug before it ever reached hardware and is worth doing again for any new
  control logic (e.g., the coordinate engine in step 4 above should follow
  the same pattern).
- Serial command style: short 1–2 letter commands, axis as first numeric
  argument, `a` meaning "both axes" where relevant. See the `printHelp()`
  function in `telescope_autotune.ino` for the full current command list.
- All hardware writes (PWM level, direction GPIOs) go through a single
  `writeMotorHardware()` function; the 1 kHz ISR and any open-loop test
  routine coordinate access via a `testMode[]` flag per axis so they never
  fight over the same motor at the same time.
