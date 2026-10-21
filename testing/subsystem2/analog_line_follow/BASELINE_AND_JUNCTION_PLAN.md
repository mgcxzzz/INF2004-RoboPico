# V11 baseline and next junction experiment — 21 October 2026

## Preserve this working baseline

- Firmware: buddy23_analog_line_follow_v11.uf2, profile 6, commit c6fc501.
- Symmetric front modules side by side with no gap, centred across the line.
  Both see black when aligned. Retain final mounting height and load.
- Left A0 -> GP26/ADC0, right A0 -> GP28/ADC2; 3.3 V supply, common ground.
- Last reported calibration: white=184,184; black=3464,3692. Recapture after
  changing mounting/height/lighting; these are recorded endpoints, not universal constants.
- Adaptive targets: absolute normalized L-R difference 0..80 gives 45/45 mm/s;
  250 gives inner/outer 40/50; 500 gives 25/50; 1000 gives 0/45.
  More left black slows the left wheel; more right black slows the right wheel.
- PWM cap 250 permille (25%); encoder PI and bounded breakaway retained.
- White recovery requires recent directional evidence and is bounded by 650 ms
  or 25 mm average absolute wheel travel. Unknown direction stops.
- Test limits remain 10 seconds / 500 mm. This is not yet endless course firmware.

## Successful physical evidence

The user reported successful recovery in both directions. Both fresh V11 logs
use the calibration above, profile 6, zero dropped rows and zero invalid encoder
transitions. Both stop for the normal 10000 ms run-time limit, not line loss.

| Run | Wheel distances at cutoff L/R | Relative encoder heading | Settled mean distance |
|---|---|---|---|
| Right recovery | 422.81 / 291.82 mm | +68.23 degrees | 357.87 mm |
| Left recovery | 289.22 / 399.95 mm | -57.68 degrees | 345.14 mm |

Raw logs: logs/v11_right_recovery_2026-10-21.txt and
logs/v11_left_recovery_2026-10-21.txt. Heading is change since start, positive
right, using provisional 110 mm wheel spacing. It is not an absolute angle to
the line. Initial-angle estimates require final alignment and no-slip geometry.
These tests establish straight/angle recovery; continuous curved-course and
junction operation still require physical tests.

## Current junction requirements and geometry

- Navigation action applies at the next cross-junction after a barcode;
  barcode-to-junction distance varies. Course ultimately runs endlessly.
- Specification image: driving line18 mm wide, barcode approximately141 mm long
  along the scan direction, lateral gap20 mm. The20 mm is NOT a forward distance.
- Barcode approach is straight. A white gap between its final bar and the cross
  is NOT guaranteed. Simple next-black detection after decoding is not reliable.
- No additional IR module can be mounted. Keep the three existing modules.
- Offset sensor is robot-left, D0 on GP19; black/white polarity not yet physically
  checked. Use stationary capture on both surfaces; no need for A0 on this sensor.
- Front sensing points approximately5 inches (127 mm) ahead of the imaginary line
  joining the two powered rear-wheel centres. Offset point35 mm behind the front
  pair, therefore approximately92 mm AHEAD of the powered-wheel line.
- These are approximate longitudinal measurements to refine with a stop test.
  Cross leading edge to axle-centre travel is approximately92+9=101 mm for an18 mm
  cross-arm. A measured stripe midpoint instead needs92 mm additional travel.
- Both-front-black is normal and cannot independently identify a junction.

## Isolated junction test now built, physical validation pending

See ../junction_left_test/README.md and firmware/buddy23_junction_left_v1.uf2 in
that test folder. V11 controller/artifact remains unchanged. Barcode work deferred.

- Mode1(default): V11 approach -> existing offset sensor detects cross-arm ->
  measure leading/trailing encoder edges -> stripe midpoint+92 mm axle goal ->
  stop and settle. Straight approach, NO barcode/other side marks.
- Mode2: same centring, then settled opposite-wheel left turn, fresh outgoing-line
  confirmation near90 degrees, settle, V11 follow20 mm, final stop.
- Mode2 must leave the original line before accepting both-front-black as the
  outgoing branch. Bounded encoder angle/time and feedback guards remain active.
- First physical step is stationary GP19 white/black validation; then mode1
  measures axle centring before mode2 rotation is evaluated.
- No barcode decoder/mapping, right/U-turn, final RTOS nav integration or endless
  course run is claimed. This experiment does NOT resolve the no-white-gap
  barcode/cross-arm ambiguity. A dedicated extra sensor was discussed but is
  unavailable under the user's mounting constraint.
