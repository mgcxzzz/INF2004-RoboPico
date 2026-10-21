# User-reported physical evidence — 21 October 2026

## Stationary GP19 validation

Front calibration imported from V11: white184/184, black3464/3692, valid1.
Side D0 GP19: white0, black1, mask3, side_valid1. User supplied live samples
with motors off and confirmed corresponding surfaces. No analog wiring change.

## V1 mode1 centring: passed visually

User reported powered rear-wheel midpoint over + centre from multiple starting
distances. Supplied full log:24 rows, dropped0, CRC restored, normal completion.

- Leading139.66mm, trailing154.90mm, measured stripe15.24mm.
- Axle target239.28mm; settled L238.98/R237.95mm, mean238.465mm.
- Approx0.82mm short of encoder target; NOT independent physical accuracy proof.
- Approach->centre3983ms->settle7607ms->done7913ms. Invalid0/0.
- Final relative encoder heading+0.54deg. Front normalized levels3/1000 at end:
  front pair already ahead of the cross, not evidence both were centred on line.

## V1 mode2: centred, loaded turn stalled

User saw wheel attempt/jerk and stop. No hardware supply diagnosis is established.

- Leading161.87mm, trailing176.99mm, stripe15.12mm, axle goal261.43mm.
- Centre drive-off7906ms at mean260.55mm; settled8212ms at260.67mm.
- Turn origin L261.56/R259.78mm. Cutoff L258.16/R261.92mm at9115ms.
- Left reversed3.40mm, right advanced2.14mm: relative LEFT turn2.89deg.
- 8612ms: velocities-3.0/+9.3mm/s and PWM-250/+250 (25% kick).
- 9013ms: both speeds0, PWM-162/+179. No meaningful movement thereafter.
- STOP no encoder movement under drive, invalid0/0. This was not a branch/angle
  timeout, not reversed turn direction, and not a no-line stop.
- User hand-turn clarification: RIGHT wheel forward + LEFT backward rotates LEFT.
  That matches the firmware's signed commands.

V2 addresses the observed loss of sustained turning effort: turn-only loaded
feedforward220permille with signed PI, retaining250permille cap and500ms stall
cutoff. It does not claim a verified fix before another physical log. No change
to geometry, optical branch acquisition, forward controller or V11 baseline.

## V2 mode2: still stalled at sustained25%

Fresh offline V2 test2,25 rows/dropped0. User video (11.54seconds, about60fps)
video_2026-10-21_23-51-53.mp4 inspected at0,7,9,11seconds. It shows the straight
approach, reaching the crossing, and only a small rotation before remaining
stationary. The overhead view cannot establish sensor-bracket/underside drag.
Do not claim a visible free-spinning wheel, proven obstruction or supply failure.

- Leading139.34mm, trailing154.31mm, stripe14.96mm; axle target238.82mm.
- Centre drive-off7289ms at237.91mm; turn starts7595ms after settling,
  L238.51/R237.47mm, mean237.99mm (about0.83mm short of encoder goal).
- 7996ms: both speeds0, PWM-250/+250: unlike V1, effort reached the full cap.
- Cutoff8296ms, L234.94/R239.92mm: left reversed3.57mm, right advanced2.45mm.
  Relative left rotation3.13degrees, invalid0/0. STOP no encoder movement under drive.
- Centring remains visually successful; the ongoing turn failure occurs before
  optical branch acquisition. Do not extend the stalled-drive timeout as a fix.

V3 raises only turn-phase output cap to300permille and loaded feedforward to270.
Forward phases retain250permille. All movement/angle/time/button guards remain.
This is not evidence that30% is sufficient or that stall current is safe/measured.
Manufacturer board information was checked at https://www.cytron.io/p-robo-pico
(3.6..6V motors,1A continuous/channel); actual motor stall current and loaded
bank voltage have not been measured. Board ratings do not determine current from PWM.
