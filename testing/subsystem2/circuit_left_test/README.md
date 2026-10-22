# Continuous left-turn circuit test V1

Standalone Pico W / Robo Pico SDK experiment for Buddy 2 and 3. Reuses the
working symmetric two-sensor follower and Junction Left V10 rolling turn.
Select **profile 1 first**. Profiles 2 and 3 are experimental speed targets.
This circuit firmware has not yet been tested on the physical full course.

## Download

`firmware/buddy23_circuit_left_v1.uf2`

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

| Profile | Centred cruise | Cross approach / final approach | Rolling left: early / late | Alignment base / range | Forward PWM cap |
|---|---:|---:|---:|---:|---:|
| 1, usual | 45 | 30 / 25 | 45 / 30 | 30 / 20–35 | 30% |
| 2, faster | 90 | 35 / 30 | 55 / 35 | 35 / 25–40 | 36% |
| 3, fastest | 135 | 40 / 35 | 65 / 40 | 40 / 30–45 | 42% |

Normal line-steering wheel targets scale by 1, 2 or 3 while preserving the
existing steering ratio. Strong corrections already slow the inner wheel,
potentially to zero. Optical blind recovery stays at the tested 0/45 mm/s.
The rolling left turn leaves the left wheel unpowered and moves the right
wheel forward; late turning begins at the existing estimated 65 degrees.
Turn output remains capped at 42%, with one start pulse up to 48% for at
most 150 ms. The encoder wheel-speed PI can reduce output when over speed.

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
circuit record takes precedence on later boots. Check the printed `CAL` line.

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
- Alignment retains the tested six-second / 180 mm window. Both front levels
  at least 400 and their difference at most 350 for 80 ms complete alignment.
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
profile 1 agreement with V10; calibration import, CRC corruption and ring wrap.

Build C11 host tests with warnings as errors, then the standalone Pico W CMake
project. `verify_firmware.py` checks delivered UF2 identity, RP2040 blocks and
reserved flash boundaries. See `BUILD_V1.md` for this artifact's build evidence.
