# Next steps — where the project stopped (2026-11-06)

## State when work stopped

- **Firmware on the Pico:** `sketches/claude_scopico`, git tag `flashed-2026-11-06b`.
  It has the speed-scheduled gains (`gs`), the stricter `auto full` checks, the
  supply monitor (`vsup`), the fault black box (`fault`), the 500 Hz recorder
  (`cap`) and the open-loop `spin` test.
- **Power:** the motors now run from the **15 V 6 A supply directly**. The
  LM2596 buck has been taken out of the motor path; it was collapsing under
  load, to 4–5 V. `md` is **32 %** on both axes, which gives the same top motor
  voltage as 40 % of 12 V.
- **Supply monitor:** wired as 15 V → 10k → GP28 → 1k → GND. It reads about 15.3 V.
- **Calibration:** `auto full` completed after the 15 V change.
- **Saved files on the Pico:** `/calib.txt`, `/sky.txt` (site, alt-az mount,
  scale 3,564,965 counts per turn, `jogmax`), `/wifi.txt`, `/hwdir.txt`.

## The open problem: the az drive jams

The az motor runs normally, then **stops and stays stopped, humming**: it's
powered but not turning. The supply holds at 15.3 V when this happens, and it
also happens with the az clutch released. That means the obstruction is in the
**az motor, gearbox or worm**. It isn't the supply, the encoder, the driver,
the tuning or the fork.

Earlier causes that are now fixed (details in CLAUDE.md):
- a gain oscillation at about 28 Hz
- the sagging LM2596 supply

## Steps to resume

1. **Power off** before any hands-on work.
2. **Reverse test:** with the az clutch released, run `spin 0 -22 60`.
   - Jams one way only: suspect a chipped or skewed tooth, or a gear walking along its shaft.
   - Jams both ways: suspect debris or a seizing bearing.
   - When it jams, type `x` straight away. A stalled motor overheats.
3. **Isolate:** unbolt the az motor/gearbox block from the worm if possible.
   - Turn the worm by hand. It should feel even all the way round.
   - Run the motor and gearbox alone with `spin 0 22 60`. If it still jams,
     the fault is in the motor or gearbox; otherwise it's the worm or its bearings.
4. **Inspect the gearbox:** 12T pinion → 38/16 → 38/16 → 38/16 → 48T. Look for:
   - debris
   - chipped teeth, especially on the 16T pinions
   - dried or contaminated grease
   - gears sliding or wobbling on their shafts

   Clean it and re-grease with a light, plastic-safe grease.
5. **Optional cross-check:** swap the az and alt motor/gearbox assemblies and
   see whether the jam follows the assembly.
6. **Verify the fix:** run `spin 0 22 60` and `spin 0 -22 60`. Both should
   report no speed drops and a steady supply.
7. **Recalibrate:**
   1. Run `vsup reset`.
   2. Run `auto full` with the clutches engaged.
   3. Run `vsup`. The minimum should stay close to 15 V.
8. **Check the servo:**
   - **Tracking:** with `s` streaming, run `t 0 sid` for a minute. The az
     `err` should stay within a few counts. Then run `o 0`.
   - **Fast moves:** use the Stellarium arrows at the fastest speed. There
     should be no faults.
   - **Oscillation:** run `cap 0 10` during a fast move. There should be no
     big peak near 28 Hz. If one appears in the blend region, try
     `gs a 0.25 200 3000`.
9. **Night alignment:** with Stellarium connected:
   1. Centre a star at 20–60° altitude and press Sync.
   2. Sync a second star about 90° away in azimuth, and optionally a third.
      Avoid the zenith.
   3. Run `align`. The residuals should be a few arcminutes.
   4. Try some gotos.
   5. Run `park` before powering off.

## Offered but not yet done (ask Claude if wanted)

- **Supply threshold:** raise `auto full`'s minimum from 10.5 V to about
  13 V, which suits the 15 V supply better.
- **Low-supply safeguard:** if the supply drops below a threshold, stop the
  motors smoothly and report "motor supply low" rather than a following-error
  fault.
- **Faster relay test:** run the relay auto-tune at a moderate speed rather
  than near standstill. The speed schedule makes this less necessary.

## Handy commands

```
where / p / vsup / fault / gs / jogmax / align / scale
spin <axis> <duty> [s]   cap <axis> [s]   auto dir a   auto full   cfg save
```

## Git

The project is a git repo. Commit before uploading, then tag the build that
is flashed, for example `git tag flashed-YYYY-MM-DDa`. That way any earlier
version can be rebuilt exactly.
