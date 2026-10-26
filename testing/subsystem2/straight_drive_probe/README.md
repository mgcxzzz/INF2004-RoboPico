# Straight drive probe V1

This is a four-second diagnostic, not a line-following replacement. Flash
firmware/buddy23_straight_drive_probe_v1.uf2 on the same Pico W. Preserve the
current mounting/calibration for the first trial. Normal V15 remains available
in circuit_with_barcode/firmware and its binary is unchanged.

## Purpose

The supplied V15 straight/A trial stopped manually at20.715s,592.3mm,28.59mm/s
average. The retained window1.933..21.698s contains950 active rows. Decision24
straight hold occurred139 times; ordinary left/right decisions occurred732
times. Both wheels show low-speed/restart episodes; front readings sometimes
fall to white even though the user reported a straight approach. The A frame
was not accepted. A white loss at8.103s/reacquisition8.489s occurred during the
barcode region. These observations do not distinguish optical misalignment,
physical overshoot/vibration/drag, controller stiction or tape wear by themselves.

Probe V1 holds BOTH wheel targets at45mm/s, irrespective of front/side readings.
It retains the V14/V15 wheel PI,30% normal PWM cap, encoder feedback, breakaway,
pins, sample scheduling, button/USB/control/direction/invalid-encoder guards
and saved motion logging. It bypasses optical steering/search, route dispatch,
profile2 ramp drive and V15 straight-only PWM smoothing. Its independent1ms
guard uses a hard4s deadline. It does not enforce an externally measured
heading or guarantee a straight path on unequal/slipping wheels.

## Test

1. Place the robot on the same flat mat with about40cm of clear path. It can
   leave the stripe because steering is disabled. Leave the sensor mounting
   and battery/power method unchanged for this first comparison.
2. Use the usual offline start: wait5s after boot,hold both buttons3s,
   release,wait1s,tap GP21,then2s countdown. Or USB g for2s countdown,
   f for hold-GP21. s/either button stops early. Profile keys do not change
   the probe: it forces profile1 and45/45 targets.
3. Watch whether it still jerks and whether it veers. It stops after4s;
   STOP:4-second fixed-speed diagnostic complete is expected.
4. Wait5s before power changes, reconnect to PuTTY and send p. This short run
   fits entirely in retained logging. The underlying CSV container says
   CIRCUIT WITH BARCODE v15 but key=D and a probe heading identify this test;
   decision25 means fixed targets. No new saved-record format/bank is used.

Existing valid front calibration is needed to launch, but its readings do not
choose the wheel commands. Do not recalibrate between V15 and the first probe.
The latest probe log replaces the latest saved motion log, as normal runs do.
Flash normal V15 again before line following/barcode/junction operation.

## Interpretation

- Jerky with fixed45/45: pursue motor PI/breakaway, supply, friction/scraping
  and mechanical motion before adjusting optical steering again.
- Smooth with fixed45/45: pursue steering gain, optical geometry/height,
  transient confidence and the straight-to-bend response.
- Veers with constant targets: compare actual left/right speeds and observe
  physical contact/traction. Equal encoder speeds do not prove no tyre slip.

This is an isolation test, not a conclusion or a performance fix. Avoid
changing several physical/software settings before collecting this baseline.

## Build

Uses the existing common adapter under B_STRAIGHT_DRIVE_PROBE. The default
normal build excludes these hooks; rebuilding normal V15 reproduced its
published SHA256 exactly. The pure probe helper selects fixed targets and
inhibits ramp/search/straight smoothing; startup sets the4s deadline and
saved key=D. Profile selection is ignored in this diagnostic.

Pico SDK2.2.0, ARM GCC14.2.1, C11 -Wall -Wextra -Werror, Release. CMake uses
the existing packaged picotool via picotool_DIR; no new runtime dependency.
test_probe.c checks optical/profile independence, all bypasses, invalid-cal
rejection, deadline timing, manual stop and retained normal breakaway.
See BUILD_V1.md for artifact verification.
