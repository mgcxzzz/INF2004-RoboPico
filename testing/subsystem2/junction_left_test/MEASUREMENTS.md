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

## V3 mode2 — 22 October 2026: left reverse stalls, right keeps moving

- Fresh offline V3 test2,25 rows/dropped0, invalid0/0.
- Leading137.17mm/trailing152.76mm, stripe15.60mm, axle target236.96mm.
  Centring drive-off7242ms mean236.05mm; turn origin7547ms L237.79/R234.47mm.
- 7947ms L232.64/R243.64: left estimated speed0 despite PWM-300 (30%);
  right19.3mm/s at PWM279, targets-38/+31. Left position then stays fixed.
- Cutoff8246ms L232.64/R251.08: left reversed5.15mm, right advanced16.61mm.
  Relative left rotation11.34degrees. Right speed28.4mm/s at cutoff. STOP is
  no encoder movement under drive, specifically left feedback, not optical loss.
- Improvement over V2's3.13degree twitch, but not a completed turn. Right moved
  about3.2 times the left arc; cannot describe this as balanced in-place motion.
- User subsequently confirmed front IR housings ARE scraping the mat, and front
  support ball rolls freely. Earlier contact height was chosen for stable scans.
  Additional drag is a plausible contributor, not proof of sole cause; loaded
  motor current/voltage and contact force remain unmeasured.

Next recommendation: preserve close, fixed optical height while relieving housing
drag/weight-bearing contact (mount adjustment or low-friction support). If optical
height changes, recheck stationary white/black response and recalibrate at the
final height before repeating V3. No new PWM increase or firmware was made for
this recommendation. A one-wheel pivot/rolling corner remains a possible later
test, but needs earlier turn start and more branch space; it is not equivalent
to swapping left reverse for zero in the current axle-centred turn.

## V4 request and bounded implementation — 22 October 2026

After the drag discussion, user reported substantial hand torque needed and
explicitly requested much higher turning effort. User prefers keeping contact
height for the present comparison. Firmware controls PWM, not measured torque.

V4 chooses one40% turn-start pulse per wheel for nominal150ms, ending early at
25mm/s measured along speed, followed by35% maximum with31% turn feedforward.
It never repeats that peak within a turn; independent1ms timer clamps expired
peaks, plus pre-commit expiry check. Same500ms no-motion-under-established-demand,
6s/110degree turn limits and button/late-loop kills. Forward25% unchanged.
No supply voltage change or current/torque guarantee. Physical V4 result pending.
The V3 binary remains available. Mount contact is still a plausible drag source.

## V4 mode2: pulse starts wheels, left stalls later

Fresh offline V4 test2,28 rows/dropped0, invalid0/0. Centring goal242.92mm,
settled turn origin7840ms L243.10/R241.35mm, mean242.22mm.

- 7850ms: both turn outputs-400/+400, speeds0/0, targets-35/+35.
- 7882ms: left-37.5mm/s, right17.8; left peak ends early, output-275/+400.
- 7893ms: left-56.1/right26.3; both peaks ended, output-256/+288.
- 8294ms: left-0.4/right22.6 at PWM-349/+316; left then stationary.
- Cutoff8668ms L234.07/R259.38mm: left reversed9.03mm, right advanced18.03mm,
  relative left angle14.10degrees. STOP no encoder movement under drive (left),
  not optical loss/branch timeout. Startup boost helped but did not complete turn.

Board published continuous current rating1A/channel was checked again at
https://www.cytron.io/p-robo-pico . PWM can theoretically reach100% but there is
no established safe maximum for this motor/load from duty alone; actual motor
stall current and supply drop remain unmeasured. No100% test was introduced.

## V5 rolling corner accepted by user — physical test pending

User confirmed a rolling turn is acceptable instead of exact-centre in-place
rotation. New mode3 removes left reverse demand: left motor OFF/coast, right
forward; begin55mm before cross centre using provisional110mm wheel spacing.
Ideal turn ends55mm along outgoing branch, outer wheel172.8mm arc at90degrees.
Needs outgoing black line>=220mm due127mm front overhang plus20mm exit travel.

Keep right rolling across the position-to-turn transition (no phase3 stop).
Preserve its speed ramp, clear old straight I when activating loaded turn FF.
No PWM increase beyond V4: only right may use35% sustained/one40% start pulse;
if already>=25mm/s it skips that peak. Left request0 resets all PI/boost/pulse.
Inner encoder displacement bound12mm; fresh-line gates/110deg remain, rolling
outer bound240mm/9s accounts for longer arc. Mode1/2 behavior preserved.


## V5 mode3 � first rolling test: substantial turn, front spots beyond mat

22 October2026. Fresh offline test3,33rows/dropped0, invalid0/0. Cross leading
131.94/trailing146.90mm, stripe14.96, turn-start goal176.42. Turn starts5275ms
at L177.41/R173.87, ends10747ms at L177.89/R370.21: left coast0.48mm,
right forward196.34mm, relative left rotation102.02deg. STOP no encoder movement
under drive, phase4. Both front levels stayed nearwhite; no phase5/6/normal exit.
Video video_2026-10-22_00-36-44.mp4 frames5/8/10/12seconds show the rotation and
front sensing points reaching beyond the lower mat edge. Rolling torque/motion
improved, but this was not a completed junction-and-follow sequence. Extending
outgoing track was recommended before changing acquisition.

## V5 mode3 � repeated with outgoing track available: first sensor is skipped

Fresh offline test3,25rows/dropped0, invalid0/0. Video
video_2026-10-22_01-05-10.mp4 duration10.402s inspected at3/7/8/9seconds.
Robot approaches from below the image, turns left toward the image-left branch,
and continues rotating across the branch. Unlike the first video, the outgoing
track remains on the mat with room ahead; insufficient mat is not the explanation
for this second optical failure.

- Leading36.50/trailing48.57, stripe12.07mm, goal79.54mm.
- Rolling starts3267ms L80.19/R76.89mm, heading+1.72deg.
- 7274ms L80.82/R254.16, normalized left721/right3. Relative left rotation is
  about92.01deg (run-relative heading-90.29deg plus initial1.72deg).
- 7675ms left1000/right8. V5 still commands left0/right30, because both>=650
  never qualify together. It passes the line rather than transferring control.
- Cutoff8142ms L80.82/R288.73, relative left110.01deg, phase8fault,
  STOP left branch not confirmed. Right still41.2mm/s. This is NOT a stall
  or weak-torque cutoff; adding PWM is not supported by this log.
- Raw ADC and normalized level columns differ because normalized control uses
  filtered ADC; e.g. right raw1466 with level8 is possible during a fast crossing.

V6 mode3 starts slow forward alignment from either>=400 for30ms in75..<110deg,
after leaving the old line. Both wheels forward20..35mm/s,25% cap, no peak;
require both>=400/difference<=350 for80ms, then unchanged V11 follow20mm.
Acquisition3s/60mm/60..115deg, bothwhite150ms with recent direction only.
No changes to centre/turn-start geometry, turn power, pins, calibration or V11.
Physical V6 outcome pending; simulated transitions are not hardware proof.
