# Junction detection, centring and left turn - V9

Separate SDK test on the Buddy2 branch. Uses the successful V11 symmetric
front controller and an existing offset digital sensor. No new sensor, barcode
decoder, RTOS coordinator, IMU or radio. V1 centring passed by user observation
from different starting distances. Both V1 and V2 loaded turns stalled; V3
turn-only30% experiment is software-verified. Its22 October physical test made
11.34degrees, then the LEFT reversing wheel stalled at30% while the right kept
moving. User confirmed front IR housings scrape the mat; support ball is free.
See MEASUREMENTS.md. User then explicitly requested a stronger drive test before
mounting changes. V4 also stalled after14.10degrees. The user accepted a rolling
turn as an alternative; V5 implements it as MODE3. V5 rolling motion passed
visually, but outgoing-line handover failed. V6 changes that handover;
user reported successful V6 handover. V7 extended the outgoing follow; its new log exposed a controller transition problem addressed by V8.

## V9 quick start: six-second alignment, then ten-second tracing

**Physical V9 result:** junction detection, rolling left turn, stable alignment
and return to V11 line following observed. Alignment took4.053s, then phase6
followed about156.4mm before the user-reported board edge caused optical loss.
Final stop was `line not recovered`, not the ten-second normal completion.
Keep V9 as the working junction/exit baseline; full ten-second completion still
needs sufficient outgoing track. See MEASUREMENTS.md and the archived log at
`logs/v9_left_turn_exit_edge_2026-10-22.txt`.

User explicitly requests **6s alignment**, followed by **10s normal line tracing**
after alignment succeeds. The old3s acquisition window was a provisional test
guard introduced in V6, not a supplied NFR. The timers are separate: phase9
allows up to6s; entering phase6 starts a new10s clock. Earlier alignment starts
tracing earlier, rather than waiting for all6s to elapse.

The V8 log confirmed right PWM74 at acquisition entry (the prior zero-output
handover was fixed). It crossed from left~1000/right~0 to left75/right743 at
10008ms, only~0.6s before the3s deadline. Phase6 still never began. More settling
time is a useful next comparison; this does not prove it will establish tracking.

1. Flash **firmware/buddy23_junction_left_v9.uf2**; press **h**: V9, front
   valid1, side_valid1. Keep current mounting/height/calibration.
2. Press **3**, **k**, **h** and confirm rolling mode3.
3. Same start: bank boot wait5s, hold both3s, release both, wait1s, tap/release
   GP21,2s countdown; or USB ready1/buttons released, then **g**.
4. Expected **1 -> 2 -> 4 -> 9 -> 6 -> 7**; normal stop is **test complete**
   about10s after phase6 entry. Either button or serial **s** still stops.
5. Wait3s after stop/save, switch sources separately and press **p**. Send the
   complete fresh V9 log and video; old restored records retain their revision.

Alignment absolute-path allowance increases60->180mm, matching up to6s of its
nominal<=30mm/s mean target. The V8 acquisition already used57.71mm; changing
only the timer would otherwise hit60mm almost immediately. Actual overspeed can
still hit the path bound earlier. Angle60..115deg and optical loss guards stay.
Tracking confirmation remains both>=400/difference<=350 for80ms. V8 PI reset,
V11 follower/targets, PWM, wiring and calibration are unchanged.

The normal trace interval changes400->500ms so the100-row flash record fits the
new35s whole-run deadline and settling/transition snapshots. Motor/sensor control
continues at10ms; logging rate is separate.900mm whole-path,500ms feedback stall,
button/USB/late-loop/encoder checks and bothwhite loss handling remain active.
A guard failure can interrupt either window. The normal phase6 completion uses
elapsed time, not distance.

At45mm/s ten seconds covers about450mm after alignment. Allow at least another
450..500mm of continuous black stripe and surrounding flat white surface beyond
the reacquisition/alignment point; extend the mat if needed. If the stripe ends,
the normal line-loss stop can interrupt the test. Keep optical height fixed.

Strict host tests cover active just before6s and timeout at6s, no premature old
60mm cutoff,180mm abort, successful alignment at5.58s followed by a fresh10s
clock, continued follow past5s, and completion exactly at10s. Existing optical,
controller and PWM checks pass. Arm/flash-layout checks pass. The physical
junction/exit result is recorded above; ten-second completion remains unverified
because available track ended. See firmware/BUILD_V9.md.

## Historical V8 quick start: correct the pivot-to-forward PI handover

The V7 log reached phase9 at7145ms but never phase6. Left stayed near1000,
right near0, and alignment timed out3002ms later. Right PWM dropped from228
in the pivot to0 at handover, then56. A negative integral accumulated under
turn feedforward can cancel the lower ordinary-forward feedforward. The old
adapter cleared integral when ENTERING a pivot, but retained it when LEAVING.

V8 clears integral at both turn/forward boundaries, once per transition, while
preserving the speed ramp. It then allows the normal forward integral to build.
The V11 forward controller, alignment20..35mm/s targets,3s/60mm bounds, optical
thresholds, pivot geometry and PWM limits remain unchanged. New phase9 timeouts
say **outgoing-line alignment timeout**; old V7 logs retain their saved reason.
The five-second phase6 timer starts after stable alignment, as in V7.

1. Flash **firmware/buddy23_junction_left_v8.uf2**; press **h** and verify V8,
   front valid1 and side_valid1. Same mounting/height retains saved endpoints.
2. Press **3**, **k**, **h** and confirm mode3.
3. Repeat the same incoming placement and bank sequence: wait5s after boot,
   both buttons3s, release both, wait1s, tap/release GP21,2s countdown.
   USB alternative: release buttons, ready1, then **g**.
4. Expected **1 -> 2 -> 4 -> 9 -> 6 -> 7**, then **test complete** after5s
   in phase6. Leave enough outgoing flat track (about250mm after reacquisition,
   roughly500mm of branch from the crossing centre allows alignment room).
5. Wait3s after stop/save, switch power sources separately and press **p**.
   Send the fresh V8 log and video. It may still stop early on a guard condition.

Host regression fixture: right ramp30, prior I=-82, speed31.8, next target33.
This representative state produced0 PWM in the V7 calculation; V8 gives75
permille on the first tick, retaining ramp31 and resetting I to near0. The
integral value is inferred/modelled, not a logged measurement. Further tests
check one-time reset, either sign of inherited I, no40% peak leakage, exact
forward-controller equivalence, full acquisition and five-second boundaries.
Arm/UF2 checks pass; physical V8 result pending. See firmware/BUILD_V8.md.

## Historical V7 quick start: five seconds of following after the turn

User reports the V6 gradual handover worked. A fresh V6 log has not yet been
supplied, so this is user-observed success, not a measured timing/angle claim.
V7 keeps that turn/acquisition controller and replaces the20mm exit completion
with **five seconds of normal V11 line following**. The timer starts on phase6
entry, after stable outgoing-line alignment. Turning/acquisition time does not
count toward the five seconds. Both turn modes2/3 use the timed exit; mode1
continues to centre and stop.

1. Flash **firmware/buddy23_junction_left_v7.uf2**. Press **h**: verify V7,
   front valid1 and side_valid1. Keep the existing calibration/mounting/height.
2. Press **3**, **k**, then **h** to verify rolling mode3.
3. Repeat the same bank start sequence (wait5s, both3s, release both, wait1s,
   tap/release GP21,2s countdown), or ready1 then USB hands-free **g**.
4. Expected phases **1 -> 2 -> 4 -> 9 -> 6 -> 7**. It turns, aligns, follows
   for5s, then stops with **test complete**. Either button or serial **s** stops.
5. Wait3s for settling/save before switching sources separately; **p** prints
   the fresh V7 log. Send log/video from the longer follow.

At a nominal45mm/s the follow covers about225mm after alignment; leave about
250mm of continuous outgoing line beyond the point where the sensors reacquire
it. Because the sensor spots are already roughly182mm down the outgoing branch
at the nominal quarter turn (plus any alignment travel), about500mm of outgoing
black line and surrounding flat white surface from the+centre gives room for
this test. If the available track ends sooner, the normal line-loss stop can
interrupt the five seconds. Keep the optical height unchanged across tile joins.

The whole-run deadline is30s (was25s) to leave more time for the longer exit;
400ms logging and the100-row saved record still fit the static trace budget.
The900mm overall absolute-path cutoff, line-loss logic,500ms feedback stall,
button/USB/late-loop/encoder checks and all PWM limits stay active. The900mm
limit is an overall abort, not the new phase6 completion condition.

Host tests check continue past20mm, still active at4,999,999us after phase6,
normal completion at5,000,000us, early optical loss/path cutoff, and the existing
V6 full handover sequences and PWM guards. See firmware/BUILD_V7.md. Physical
V7 five-second result pending. V1..V6 calibration/records retain their layout;
new traces identify V7, older restored traces retain their original revision.

## Historical V6 quick start: gradual outgoing-line acquisition (MODE 3)

V5 physically completed the rolling rotation, but kept pivoting past a visible
outgoing line because it waited for BOTH levels>=650 at the same time. The latest
log first shows left721/right3 near92degrees of turn-only rotation, then never
qualifies before the110degree cutoff. This is evidence to change optical handover,
not increase PWM. See MEASUREMENTS.md.

1. Flash **firmware/buddy23_junction_left_v6.uf2**; open PuTTY115200, flow None.
2. Press **h**: verify V6, front valid1, side_valid1. V5 calibration is imported
   unchanged; no recapture if mounting/height/lighting response is unchanged.
3. Press **3**, **k**, **h**: verify mode3. Keep the same straight approach,
   front sensors on the black stripe, offset sensor on white, no barcode.
4. Use the existing bank sequence: wait5s after boot, hold both buttons3s,
   release both, wait1s, tap/release GP21, then leave it for the2s countdown.
   USB hands-free alternative: release both buttons, ready1, then **g**.
5. Expect **1 -> 2 -> 4 -> 9 -> 6 -> 7**: follow, position, rolling left,
   slow forward alignment, V11 follow20mm, test complete. Mode3 no longer stops
   to settle between optical acquisition and following. Either button stops.
6. Wait3s after the final stop/save; switch power sources separately, **p**.
   Send the complete fresh V6/test3 log and video. Old restored logs retain V5.

**Phase4:** leaving the original line is still required (both<200 for30ms after
10degrees). In the75..<110degree window, EITHER level>=400 continuously for30ms
starts phase9. No blind handover based only on reaching90degrees. Mode2 retains
its old both>=650/settle gate. Rolling geometry and phase4 PWM are unchanged.

**Phase9:** both wheels now drive FORWARD, so the inner wheel is no longer held
OFF. Use the V11 sensor error/sign and direction memory, with gentler targets:
error magnitude<=80 gives30/30mm/s; at1000 the inner/outer targets are20/35.
Interpolate between those endpoints. More black on the left means left slower;
more black on the right means right slower. Motor output is capped25% in this
phase (including breakaway), with no40% turn pulse or loaded turn feedforward.
Targets are wheel speeds, not fixed PWM or measured torque.

Both levels>=400 AND their difference<=350 continuously for80ms establishes
tracking. Then flow directly into the unchanged V11 follower for20mm and stop.
No assumption that the first sensor must be left: either sensor may see the
branch first depending on placement/slip. The stored side cross-arm signal is
ignored during the turn/alignment/exit.

Acquisition is bounded by3s,60mm average absolute wheel path, and60..115degrees
of turn-relative heading. If both levels fall below150, allow up to150ms only
when a recent clear sensor difference supplies a direction; otherwise stop
immediately. Loss of alignment or missing-line bounds stop, not endless search.
The original500ms wheel stall/button/late-loop/encoder kills remain active.

Keep a continuous flat white surface around the outgoing black line. At least
220mm from cross centre is needed for the nominal turn+exit; **prefer300mm** to
leave room for the additional bounded alignment travel. Keep sensor height fixed.
The video for the latest V5 test shows the branch on the mat, not the prior
short-mat/table-edge situation. This software change is unverified on hardware.

Host tests cover the721/3 first-sensor observation, either-side handover, brief
flicker/old-line/early-angle rejection, stable tracking, lost-line/time/path/angle
bounds, and successful exit. Arm build retains the3792-byte saved record/row192
layout, imports V1..V5 calibration/logs, and labels new tracesV6. Existing modes1/2,
forward V11 baseline, and35% sustained/40% bounded turning effort are preserved.
See firmware/BUILD_V6.md for the artifact and verification.

## Historical V5 quick start: select MODE 3 for the rolling left corner

The left motor stays OFF/coast, while the right keeps driving forward. This
avoids demanding the left reverse movement that stalled in V1..V4. It does NOT
rotate about the axle midpoint at the centre of the +.

With provisional110mm wheel spacing, the ideal stationary-inner-wheel arc has
radius55mm. Mode3 starts turning55mm BEFORE the cross centre, ending about55mm
ALONG the outgoing left branch at90degrees. Outer-wheel travel is about172.8mm.
The unpowered left wheel is not locked: encoder drift is bounded12mm, and the
fresh outgoing line must still be found.

**Outgoing black line must extend at least220mm from the crossing centre.**
After turning, the front spots are about55+127=182mm down that branch, plus20mm
exit travel. Extend the straight with matching black tape on a flat white surface
if the home mat is too short. No barcode or other side marks in this test.

1. Flash firmware/buddy23_junction_left_v5.uf2; PuTTY115200, flow None; press h.
2. Confirm V5, front valid1 and side_valid1. Same unchanged height retains calibration.
3. Press **3**, **k**, then **h** and verify **mode=3**. Existing saved mode2 can
   restore, so selecting3 explicitly is essential. Mode2 remains the in-place turn.
4. Use the same incoming straight placement and bank button sequence below.
5. Expected: follow -> detect cross -> advance to point55mm before centre ->
   flowing left corner (no stop before corner) -> confirm outgoing line -> settle
   -> follow20mm -> stop. Turning before rear axle reaches + is intentional.
6. Wait3s, switch power sources separately, p. Send fresh V5/test3 log and video.

Right target45mm/s, reducing to30 near65degrees. It retains V4's35% sustained/one
40% start-pulse bounds; an already-moving right wheel>=25mm/s skips the peak.
Left target/output remain0 throughout rolling phase4. The right ramp is kept,
but its old straight integral is cleared when loaded turn feedforward activates.
Forward approach/positioning/exit still use V11 PI and25% cap. No higher PWM.

Rolling turn gets9s/240mm outer-wheel bounds (longer arc); angle still<=110deg,
no-motion-under-demand still500ms, whole-run still25s/900mm. Branch confirmation
requires leaving the old line and BOTH levels>=650 for30ms at75..110deg.
Modes1/2 retain their earlier geometry and6s/120mm turn bounds. V1..V4 records
import; old padding is normalized before reading the new rolling flag.
See firmware/BUILD_V5.md and MEASUREMENTS.md.

## Historical V4: stronger one-shot turn start

Only the phase4 in-place turn gets one40% start pulse per wheel, nominal150 ms
maximum, released early when that wheel's along-direction speed reaches25 mm/s.
It does not repeat during a stalled turn. Afterward signed PI uses31% loaded
feedforward and35% maximum; it can reduce actual output for overspeed. All forward
phases still use25%. Same turn directions, optical gates, geometry and limits.

The absolute pulse expiry is checked before motor output commit AND by the
independent1ms safety timer; after expiry output is clamped to35% at the next
timer tick even if the main control is delayed. Control-late/button/stall kills
remain active. Normal trace also captures peak-mask transitions, so the brief
pulse is visible in the next log instead of falling between400ms samples.

V1/V2/V3 calibration/mode records load. Flash V4, h verify v4/valid1/side_valid1,
2,k,h confirm mode2, then use the same physical start sequence. No height change
or recalibration is required for this software-only comparison. Use the current
power source/voltage. Retrieve the fresh log after one attempt. An older restored
trace keeps its original version; new runs print V4. Older V2/V3 firmware may
reject V4 records on downgrade, requiring calibration recapture.

## Historical V3 change: increase the turn-only limit

V2 sustained25% but the user log/video still showed a small3.13degree twitch
then stationary wheels. V3 raises ONLY the phase4 opposite-wheel turn ceiling
to30% and its loaded feedforward estimate to27%. Signed PI can reduce effort
for overspeed. Forward approach/centring/exit retain25%, same V11 controller.
The same500ms stall,6s turn and110degree angle bounds remain active.

Both the PI output and phase-gated motor bridge mapping allow300permille during
the turn; the shared V11 helpers still clamp250 for every other phase. New tests
verify actual bridge300 during stationary turn demand, and exact forward-output
equality with V11. This is a controlled next test, not a measured current/torque
guarantee. Keep the existing power source and voltage.

V1/V2 calibration and mode records still load. h must show v3 and valid1,
side_valid1. Choose2,k,h then repeat the same junction approach. A restored old
run prints its original V1/V2 version; only a fresh run prints V3. See BUILD_V3.md
in firmware/ and MEASUREMENTS.md for evidence and binary verification.

## Historical V2 change: sustain turning effort

The V1 turn log showed a brief25% kick then approximately16..18% PWM while
nearly stationary. V2 uses a provisional22% LOADED feedforward estimate during
phase4 only, replacing the lower unloaded motor estimate. Signed PI still
reduces output for overspeed and can raise it to the same25% maximum for
underspeed. This is not a forced minimum output, higher target speed, raised
supply voltage or increased PWM cap. Centring geometry, turn direction,
timeouts/500ms stall cutoff and forward V11 controller remain unchanged.

V1 saved front/side calibration, mode and run are compatible. No recapture needed
if mounting is unchanged and h shows valid1/side_valid1. Restored V1 traces are
labelled V1 even under V2 firmware; only a fresh run is V2 evidence. Choose2,k,h
to verify mode2, then repeat the same junction turn test. Full build details in
firmware/BUILD_V2.md; user-reported evidence is in MEASUREMENTS.md.

## Hardware and layout

- Front LEFT A0 -> GP26; RIGHT A0 -> GP28. Keep the successful side-by-side
  mounting, height, 3.3 V supply and common ground. D0 unused on front modules.
- Offset sensor D0 -> GP19, A0 unused. Keep 3.3 V/common ground. D0 is sufficient;
  GP19 is not an ADC pin. No black/white polarity is assumed until captured.
- Encoder/motor/button pins and normalized forward signs remain those of V11.
- Measured front sensing points approximately 127 mm ahead of the imaginary line
  joining the two powered rear-wheel centres. Offset sensing point approximately
  35 mm behind those, hence 92 mm AHEAD of the powered-wheel line.
- Straight approach to a + crossing, with no barcode, side marks or bends.
  Offset sensor should see white along the approach and black over the cross-arm.
- A turn LEFT is from the robot's perspective. When travelling from image right
  toward image left in the user's mat photo, its left branch runs toward image bottom.
- For mode2, the outgoing black line must extend at least 170 mm from the crossing
  centre: the front sensors already sit 127 mm ahead, and the exit travel is 20 mm.
  Extend the straight with matching black tape on a flat white surface if needed.

## 1. Flash and check sensors while stopped

Download firmware/buddy23_junction_left_v9.uf2 using GitHub's **Download raw file**,
then copy to RPI-RP2 in BOOTSEL mode. Open PuTTY at 115200, 8N1, flow control None.
Use only one power source at a time. Keep the robot on the mat at its final height.
Do not request a movement during this check.

1. Press `h`. Confirm `BUDDY23 JUNCTION LEFT v9`.
2. Front calibration is imported from a CRC-valid V11 record if present. `valid=1`
   means front endpoints are valid; `imported_v11=1` identifies the initial import.
   No V11 movement trace is imported. If invalid, capture BOTH front spots over
   white with `w`, left spot over black with `l`, right spot over black with `r`;
   wait 1 second at final height before each key. These keys never drive motors.
3. Put the OFFSET sensing point over white; wait 1 second and press lowercase `o`.
4. Move it over black at the same height; wait 1 second and press lowercase `b`.
   Here `b` means capture BLACK, not reverse movement.
5. `h` must now show `side_mask=3 side_valid=1`, with DIFFERENT `side_white` and
   `side_black` values. Either polarity is accepted. If both captures match,
   adjust only the offset module's blue threshold screw and repeat `o`, `b`.
6. Return the offset point to white and check live `side_raw` and `side_black=0`;
   moving it onto black should give `side_black=1`, then back to 0 on white.
7. Press `1`, then `k`. Expect `CALIBRATION/MODE SAVED`.

First physical step: send the `h` CAL line and white/black live readings for review.

## 2. Centre-and-stop test (mode 1)

This checks the approximate mounting measurements before an automatic rotation.

1. Mode `1` then `k` selected/saved on computer USB. Put BOTH front spots on the
   straight black line, roughly parallel. Start with front sensing points about
   100 mm before the crossing; offset sensor on white. Place no barcode.
2. Disconnect computer USB, connect the power bank to the board power port.
   Wait 5 seconds after boot.
3. Hold BOTH GP20 and GP21 for 3 seconds; release BOTH; wait 1 second;
   tap and release GP21. A 2-second countdown follows.
4. It follows, detects the side cross-arm, advances, settles and stops. Either
   button stops at any time. It does not turn in mode1.
5. Check whether the midpoint between the powered rear wheels is over the centre
   of the +. Measure any forward/backward error and photograph from above.
6. Wait 3 seconds after stopping. Disconnect bank, connect computer USB, open
   PuTTY and press `p`. Send the COMPLETE `JUNCTION LEFT v1` log.

USB alternative: with both buttons released and `ready=1`, press `g` for a
2-second hands-free countdown. Either button, `s`, or USB disconnection stops it.
`f` retains USB GP21-hold mode; releasing GP21 stops, so `g` avoids pressing on
the chassis. The USB cable must not restrict the motion.

## 3. Full left-turn test (mode 2)

After centring is checked: select `2`, then `k`; use the same placement and bank
start sequence. Intended sequence:

**Follow -> detect cross -> centre axle -> settle -> turn left -> confirm branch
-> settle -> follow 20 mm -> stop.**

Left wheel runs backward, right wheel forward. Both stop before and after the
rotation; no reversal while deliberately driving forward. Branch confirmation
requires leaving the original line, then BOTH front normalized levels >=650 for
30 ms during 75..110 degrees of encoder-estimated left rotation. It does not stop
merely because both sensors were black at turn start. This is an approximate
90-degree turn with optical confirmation, not a calibrated angle guarantee.
If the branch is not found, it stops rather than resuming blindly.

Wait 3 seconds after the final stop, switch power separately, then `p`. Send log
and ideally a top-down video. A mode1 result should precede interpreting mode2.

Runs can repeat without reboot: save/settling finished, both buttons released,
estimated standstill for 1 second (`ready=1`), then a fresh explicit start.
The newest run replaces the previous run; retrieve it before another test.

## Original controller details (see V9 changes above)

- Approach: unchanged V11 symmetric targets, loss logic, ADC filtering; at most
  12 seconds or 450 mm forward approach before declaring no cross.
- Side sensor: 100 ms white qualification, 20 ms black/white edge debounce.
  Record encoder position at each edge's FIRST observed sample. Accept stripe
  width 6..40 mm. Missing trailing edge by 50 mm or centring timeout stops.
- Modes1/2 axle target = average of leading/trailing edge positions +92 mm. Nominal
  18 mm strip gives approximately 101 mm after the leading edge. This does not
  assume the 20 mm lateral barcode gap is a forward distance.
- Mode3 subtracts55mm from that axle-centre goal and flows directly into the
  turn without the phase3 settle used in modes1/2.
- During centring, steer by encoder heading hold to avoid following the cross-arm.
  Speed30 mm/s, last15 mm uses25 mm/s; drive-off at remaining<=1 mm, then coast.
  The first physical stop test measures optical/encoder/coasting error.
- Settle: output0, minimum300 ms and both estimated speeds<5 mm/s for200 ms;
  fail after2 seconds. Zero clears PI/kick. No active electrical braking.
- Left turn: 35 mm/s opposite wheel directions, down to25 near65 degrees;
  bounded arc synchronization. Abort after6 seconds, 110 degrees,120 mm on
  either wheel, reverse feedback or arc disagreement>20 mm.
- Mode3: left unpowered/right45 then30mm/s, no reverse;9s/240mm outer arc, inner
  displacement limited12mm. Phase4 retains110deg/fresh-line gates; V6 mode3 then uses the bounded phase9 alignment above. Optical confirmation
  remains essential because the unpowered inner wheel can coast slightly.
- V11 PI gains Kp1.0/Ki3.0, integral limit140, ramp100 mm/s^2, direction-specific
  feedforward plus the V4 turn-only loaded estimate31%; sustained turn ceiling35%,
  one40%/150ms start pulse per wheel, forward25%, and
  bounded120 ms breakaway. Reverse adaptation
  uses signed feedback. Adequate loaded reverse torque is not yet established.
- Timer ISR cuts motor power for button/deadline/100 ms late control. Main checks
  stalled feedback500 ms under demand, wrong direction50 ms and invalid edges>=8.
- Whole run35 seconds or900 mm average absolute wheel path; exit10 seconds.
  These are output limits, not guarantees of physical stopping distance.

## Logs and persistence

Trace500 ms plus phase transitions and1 second settling, capacity100 rows. `p`
prints phase, raw GP19, front ADC/normalized levels, signed wheel targets/PWM,
encoder distance/heading, measured cross width and axle goal. Phase numbers:

| Phase | Meaning |
|---|---|
| 1 | Approach |
| 2 | Position axle (centre modes1/2;55mm early mode3) |
| 3 | Settle before turn / finish mode1; skipped in mode3 |
| 4 | Left turn |
| 5 | Settle on outgoing line |
| 6 | V9 follows10s; V7/V8 used5s, V1..V6 used20mm |
| 7 | Completed |
| 8 | Fault |
| 9 | V6 mode3: slow forward alignment after first branch detection |

Heading is relative encoder rotation (negative left), not an absolute compass.
The final turn summary gives positive left rotation relative to turn start.

CRC-verified last4 KiB flash record, written only with drive off on one core.
The revision byte uses V1 zero-filled padding, keeping record size/row offsets
and CRC range unchanged. V1's zero revision is interpreted as V1; revisions2..9
identify later runs. The V5 rolling flag fits former Junction padding, with
old versions normalized to false. Fresh runs use the running firmware's version.
New record magic is separate from V11. First `k` or completed run replaces the
old flash log; back up logs/endpoints first. New firmware can import valid V11
front calibration, but V11 cannot import this experiment's record. Returning to
V11 requires recapturing front calibration (or restoring recorded endpoints by
an appropriate separately reviewed tool). Do not reflash to retrieve a log.

## Scope

This experiment does NOT solve barcode/cross-arm separation when there is no
white gap. It intentionally contains no barcode. Success establishes junction
centering/left turn with the existing sensor placement, not the final combined
barcode navigation or endless-course system. Keep V11 as the line-follow baseline.

See firmware/BUILD_V9.md for the current verified binary and software checks;
earlier BUILD_V1..V8 records and UF2s remain available.
