# Continuous left-turn circuit test V2

Standalone Pico W / Robo Pico SDK experiment for Buddy 2 and 3. Reuses the
working symmetric two-sensor follower and Junction Left V10 rolling turn.
V2 separates faster straight cruising from correction speed, and hands a
confirmed outgoing branch to the normal curve follower immediately.
Select **profile 1 first**. Profiles 2 and 3 are experimental speed targets.
This circuit firmware has not yet been tested on the physical full course.

## Download

`firmware/buddy23_circuit_left_v2.uf2` (V1 is also retained for comparison).

Flash using BOOTSEL and the RPI-RP2 drive as usual. Open USB serial at
115200, 8 data bits, no parity, 1 stop bit, no flow control.
Motors stay off at boot. This is a separate test, not a change to production
navigation, Hump V5, or Junction Left V10.

## Intended route

Begin at the marked right-hand vertical section, facing down the photo:

1. Follow the bottom-right curve into the lower cross from its east arm.
   Turn left onto its south arm.
2. Follow the bottom loop, return to the lower cross from its west arm.
   Turn left onto its north arm.
3. Follow the left bends to the upper cross from its west arm.
   Turn left onto its north arm.
4. Follow the top loop, return to the upper cross from its east arm.
   Turn left onto its south arm.
5. Follow the right bends through the start location and repeat.

There is no lap or whole-run time/distance stop. Each qualified cross receives
the same left-turn action. Four completed turns are reported as a route cycle;
there is no sensor marker identifying the exact start location.

**Use a bare line with no barcodes or other marks under the offset sensor.**
The offset detector cannot distinguish a barcode bar from a cross-arm here.
Cross approaches must be straight. IMU, hump crossing, barcode decoding,
right turns and U-turns are not part of this test.

## Three profiles

All entries below are requested wheel speeds in mm/s, not guaranteed measured speeds.

| Profile | Stable centred cruise | Cross approach / final approach | Rolling left: early / late | Corrections and outgoing branch following | Fast-cruise PWM cap |
|---|---:|---:|---:|---|---:|
| 1, usual | 45 | 30 / 25 | 45 / 30 | Profile 1 targets | 30% |
| 2, faster | 90 | 35 / 30 | 55 / 35 | Same profile 1 targets | 36% |
| 3, fastest | 135 | 40 / 35 | 65 / 40 | Same profile 1 targets | 42% |

**V2 no longer multiplies steering targets by the profile number.** At every
profile the original normalized-error thresholds and correcting speeds apply:

| Absolute left/right level difference | Inner / outer wheel targets, mm/s |
|---|---:|
| At most 80 | 45 / 45 before qualifying for fast cruise |
| 250 | 40 / 50 |
| 500 | 25 / 50 |
| 1000 | 0 / 45 |

Intermediate corrections use the existing interpolation. Only when **both
levels are at least 400 and their difference stays at most 80 continuously
for 300 ms** do profiles 2 and 3 request 90/90 or 135/135. If that condition
breaks, the normal follower supplies profile 1 targets immediately. Fast cruise
must qualify for another 300 ms before returning. Optical blind recovery also
keeps profile 1's 0/45 targets and bounded timeout/distance policy.

Correcting and branch-following phases use the profile 1 wheel controller and
30% forward PWM cap. On leaving fast cruise, accumulated cruise compensation
is cleared and the requested-speed ramp is capped at 50 mm/s. Higher cruise
speeds still ramp up; actual speed cannot change instantly because of inertia.
The higher 36/42% forward limits apply only while fast cruise is qualified.

Rolling left leaves the left wheel unpowered and moves the right wheel
forward; late turning begins at the existing estimated 65 degrees. Those
explicit junction turn targets remain progressively faster by profile, as
requested earlier. Turn output stays capped at 42%, with one start pulse up
to 48% for at most 150 ms. The encoder PI can reduce output for overspeed.

Higher profiles require more room to slow before a cross. Deceleration is
requested immediately at the first qualified side edge, before its 20 ms
confirmation. If either measured wheel speed still exceeds 65 mm/s when the
turn should begin, the run stops with `junction approach too fast`. Use a
slower profile if that occurs; the higher requested cruise may not be suitable
for the available geometry or grip.

## Wiring and calibration

Wiring is unchanged: front LEFT A0 → GP26/ADC0, front RIGHT A0 → GP28/ADC2,
offset D0 → GP19, 3.3 V and common ground. Keep the adjacent front sensing
points in the working placement. GP20 is STOP, GP21 is the start/hold button.

On first boot, this firmware tries to import **both** front and side calibration
from a valid Junction Left V1–V10 record. It defaults to speed profile 1;
the junction's old mode 3 does **not** become speed profile 3. A valid saved
circuit V1 or V2 record takes precedence on later boots. V1 calibration and logs
remain readable; a restored V1 log is labelled V1 even when the boot banner
identifies V2 firmware. Each new V2 run is labelled V2. Check the printed `CAL` line.

If calibration is valid and placement/height has not changed, select `1`, then
press `k` to save profile 1. If calibration is missing or unsuitable:

1. Stop, release both buttons. Wait for any previous run to finish saving.
2. Put both front sensing points on white; wait one second; press `w`.
3. Put the left sensing point fully over black at normal driving height;
   wait one second; press `l`.
4. Put the right sensing point fully over black; wait one second; press `r`.
5. Put the offset sensing point on white; press `o`.
6. Put the offset sensing point on black; press `b`.
7. Press `1`, then `k`. Check `valid=1`, `side_valid=1`, `profile=1`.

The old calibration was white 184/184, black 3464/3692, side white 0 / black 1.
Those are previous measurements, not values forced into this firmware.
Changing calibration clears the current circuit log; retrieve a wanted log first.

## First lab test

1. Select `1`, `k` while stopped and connected to USB. Check calibration.
2. Put the robot at the marked start, facing along the line, with the offset
   detector on white. At least one front level must be 650 or higher.
3. Use the usual power bank sequence below. Let it complete the first left turn,
   the loop, and the second left turn through the same cross.
4. If correct, allow a full four-turn circuit and a second circuit to verify rearming.
5. Press either button to stop. Wait **five seconds** before disconnecting power.
6. Reconnect to computer USB and press `p`; send the complete output.
7. Only after profile 1 succeeds, select `2`, `k`, repeat. Then try profile 3.

Record visible overshoot, missed/false junctions, alignment oscillation or slowdowns.
The gate is a heuristic to reject side hits while bending; it needs verification
on this actual course. Straightness qualification or a cross appearing less than
150 mm after a completed alignment can prevent detection.

### Offline start and stop

- Power from the bank and wait five seconds.
- Hold GP20 and GP21 together for three seconds.
- Release both, wait one second, tap and release GP21.
- After a two-second countdown, the circuit begins.
- Either button stops the run. It saves automatically after a one-second settling
  period. **Wait five seconds after stopping before changing power.**
- Repeat the complete explicit button sequence for another run. It never restarts
  automatically. One saved run is retained; the next completed run replaces it.

### USB start and stop

- `g`: hands-free two-second countdown; release robot and buttons.
- `f`: run while GP21 is held; release GP21 to stop.
- `s` or either button stops; changing the USB connection also stops.
- `h`: help / calibration. `p`: print saved run when stopped and saved.
- `1`, `2`, `3`: select speed profile while stopped; `k` saves selection.

## Junction sequence and guards

Phases: `10` follow, `2` cross approach, `4` rolling left,
`9` align outgoing stripe, back to `10` follow. `8` indicates a fault.

- Both front sensors seeing black is normal line following, not a junction cue.
- A side black rising edge qualifies only after side white for 100 ms,
  both wheels moving at least 10 mm/s, a visible front line, and at least
  300 ms with no more than 3 degrees accumulated encoder yaw.
- A qualified rising edge slows immediately; 20 ms stable black confirms it.
- Measure the cross-arm's two edges (accepted width 6–40 mm). Their midpoint
  plus side-to-axle offset 92 mm locates the wheel axle at the crossing.
- Start the rolling left 55 mm before that centre, as in the successful test.
- Ignore the side sensor during turn/alignment. On successful alignment, resume
  following immediately and require another 150 mm before a new junction.
- The rolling turn must still leave the old line, then see a front level at
  least 400 for 30 ms within its estimated 75–110 degree branch window.
- V2 immediately uses the **normal profile 1 curve follower** at that point,
  including its full 0–50 mm/s adaptive correction targets. It no longer waits
  for both sensors to balance on an outgoing straight section.
- Phase 9 confirms either front level at least 400 continuously for 150 ms,
  then counts the turn and returns to phase 10. This confirms tracked branch
  visibility, not exact physical centring. Normal corrections continue in phase 10.
- The handover retains a six-second / 180 mm bound and the normal line-loss
  policy. It does not use V1's separate 150 ms both-white alignment stop, nor
  V1's requirement for a balanced two-sensor reading for 80 ms. This lets the
  normal follower handle a curve beginning immediately after the cross.
- Retain bounded turning, branch confirmation, line loss, encoder checks and
  the independent 1 ms button / late-control cutoff. A commanded moving wheel
  with inadequate encoder movement for 500 ms stops the run.
- Normal both-white recovery is at most 650 ms / 25 mm with a recent known
  correction direction; without that direction it stops immediately.
- Local manoeuvre distance limits reset for every junction. The old single-test
  12-second approach / 450 mm and 35-second total limit do not terminate a circuit.

## Saved logs and summary

The offline CSV contains elapsed milliseconds, active state, phase, raw side
signal, total distance, each wheel's distance and measured speed, actual PWM,
raw ADC and normalized levels, requested wheel speeds, decision, and turn count.
Decision 16 identifies qualified fast straight cruise; decisions 11–15 keep
the existing normal following/correction/recovery meanings.
It samples every 250 ms and retains the **latest approximately five minutes**
(1,200 rows, including a few start/stop/settling samples). Older rows are
overwritten. The latest 64 completed turns also have separate timing summaries.
Whole-run counters continue even when either buffer wraps.

The `SUMMARY:` line reports:

- `total_time_s`: start of drive until the stop cutoff, excluding start countdown.
- `total_distance_mm`: mean of the two encoder-derived wheel travel magnitudes.
- `average_speed_mm_s`: that distance divided by total drive time, including
  slow turning, alignment and any pauses while the run remained active.
- Completed lefts, four-turn cycles, ignored side edges and invalid encoder counts.

Distance is an **encoder estimate**, not externally measured mat distance.
Commands are forward-only; small encoder backtracking reduces net wheel travel.
Wheel slip can make distance and heading inaccurate. The circumference remains
200 mm, CPR 2524.05 left / 2528.30 right, provisional track width 110 mm.
64-bit accumulated encoder counts avoid the old per-run 32-bit position ceiling.
Very long detailed rows clip distance beyond roughly 214 km per wheel / 429 km
total; whole-run summary counters have a much larger range.

Flash writes happen only while stopped. Stop with a button and allow saving;
cutting power while running loses the current unsaved run. A separate CRC-checked
64 KiB record at flash `[0x101e0000,0x101f0000)` leaves old junction and hump
records at the end of flash intact. `p` reports a failed save if verification fails.
Connected USB status prints every 200 ms; PuTTY session logging can retain that
entire stream for runs longer than the offline detail window.

## Software checks

`test_circuit.c`: twelve successive turns (three route cycles) at each profile;
long following beyond old limits; curve-side hit rejection; clearance lockout;
measured-speed entry guard; missing branch timeout; controller caps/zero output;
profile 1 wheel-controller agreement with V10; calibration import and V1/V2
record CRC validation; ring wrap; every normalized steering error from -1000
to +1000 at every profile; fast-cruise dwell and immediate exit; one-sensor
branch handover directly into a curve; bounded both-white recovery.

Build C11 host tests with warnings as errors, then the standalone Pico W CMake
project. `verify_firmware.py` checks delivered V1 and V2 UF2 identity, RP2040
blocks and reserved flash boundaries. See `BUILD_V2.md` for this artifact's
build evidence and `MEASUREMENTS.md` for the user-reported V1 failures.
