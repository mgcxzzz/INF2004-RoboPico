# Junction detection, centring and left turn — V4

Separate SDK test on the Buddy2 branch. Uses the successful V11 symmetric
front controller and an existing offset digital sensor. No new sensor, barcode
decoder, RTOS coordinator, IMU or radio. V1 centring passed by user observation
from different starting distances. Both V1 and V2 loaded turns stalled; V3
turn-only30% experiment is software-verified. Its22 October physical test made
11.34degrees, then the LEFT reversing wheel stalled at30% while the right kept
moving. User confirmed front IR housings scrape the mat; support ball is free.
See MEASUREMENTS.md. User then explicitly requested a stronger drive test before
mounting changes. V4 is the bounded stronger-turn experiment below, physical test
pending. The scraping/contact issue remains a potential contributor.

## V4: stronger one-shot turn start

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

Download firmware/buddy23_junction_left_v4.uf2 using GitHub's **Download raw file**,
then copy to RPI-RP2 in BOOTSEL mode. Open PuTTY at 115200, 8N1, flow control None.
Use only one power source at a time. Keep the robot on the mat at its final height.
Do not request a movement during this check.

1. Press `h`. Confirm `BUDDY23 JUNCTION LEFT v4`.
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

## Controller and limits

- Approach: unchanged V11 symmetric targets, loss logic, ADC filtering; at most
  12 seconds or 450 mm forward approach before declaring no cross.
- Side sensor: 100 ms white qualification, 20 ms black/white edge debounce.
  Record encoder position at each edge's FIRST observed sample. Accept stripe
  width 6..40 mm. Missing trailing edge by 50 mm or centring timeout stops.
- Axle target = average of leading/trailing edge positions +92 mm. Nominal
  18 mm strip gives approximately 101 mm after the leading edge. This does not
  assume the 20 mm lateral barcode gap is a forward distance.
- During centring, steer by encoder heading hold to avoid following the cross-arm.
  Speed30 mm/s, last15 mm uses25 mm/s; drive-off at remaining<=1 mm, then coast.
  The first physical stop test measures optical/encoder/coasting error.
- Settle: output0, minimum300 ms and both estimated speeds<5 mm/s for200 ms;
  fail after2 seconds. Zero clears PI/kick. No active electrical braking.
- Left turn: 35 mm/s opposite wheel directions, down to25 near65 degrees;
  bounded arc synchronization. Abort after6 seconds, 110 degrees,120 mm on
  either wheel, reverse feedback or arc disagreement>20 mm.
- V11 PI gains Kp1.0/Ki3.0, integral limit140, ramp100 mm/s^2, direction-specific
  feedforward plus the V4 turn-only loaded estimate31%; sustained turn ceiling35%,
  one40%/150ms start pulse per wheel, forward25%, and
  bounded120 ms breakaway. Reverse adaptation
  uses signed feedback. Adequate loaded reverse torque is not yet established.
- Timer ISR cuts motor power for button/deadline/100 ms late control. Main checks
  stalled feedback500 ms under demand, wrong direction50 ms and invalid edges>=8.
- Whole run25 seconds or900 mm average absolute wheel path; exit20 mm or3 seconds.
  These are output limits, not guarantees of physical stopping distance.

## Logs and persistence

Trace400 ms plus phase transitions and1 second settling, capacity100 rows. `p`
prints phase, raw GP19, front ADC/normalized levels, signed wheel targets/PWM,
encoder distance/heading, measured cross width and axle goal. Phase numbers:

| Phase | Meaning |
|---|---|
| 1 | Approach |
| 2 | Centre axle |
| 3 | Settle before turn / finish mode1 |
| 4 | Left turn |
| 5 | Settle on outgoing line |
| 6 | Follow outgoing line20 mm |
| 7 | Completed |
| 8 | Fault |

Heading is relative encoder rotation (negative left), not an absolute compass.
The final turn summary gives positive left rotation relative to turn start.

CRC-verified last4 KiB flash record, written only with drive off on one core.
The revision byte uses V1 zero-filled padding, keeping record size/row offsets
and CRC range unchanged. V1's zero revision is interpreted as V1; revisions2/3/4
identify V2/V3/V4 runs. Fresh runs use the running firmware's version.
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

See firmware/BUILD_V4.md for the current verified binary and software checks;
firmware/BUILD_V1.md, BUILD_V2.md, BUILD_V3.md and their UF2s remain available.
