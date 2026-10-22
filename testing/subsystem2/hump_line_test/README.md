# Hump line test V3

Isolated Buddy2/Buddy3 Pico W SDK experiment, 2026-10-22. Uses the working
V11 symmetric front-sensor steering with a selectable wheel-load response.
No junction execution, barcode decoding, IMU, RTOS, or production integration.

V2 fixes V1 assistance dropping out when steering requests 20..29 mm/s.
The failed near-crest run commanded the uphill right wheel to 20 mm/s, below
V1's 30 mm/s assistance floor. V2 uses the existing 20 mm/s stall-guard floor.
Requests 1..19 retain ordinary output; zero immediately resets and switches off.
V2 retained targets, sensor thresholds, 35% ceiling and 500 ms stall cutoff.
V2 physically cleared the small black hump; the user reported failure on the steeper orange hump.

V3 scales every motor output by1.20, including steering, breakaway, recovery
and assistance. This is a20% relative increase, not20 percentage points.
Example: PWM150 ->180; normal ceiling250 ->300; assisted350 ->420.
The speed PI can reduce duty as speed recovers. This does not promise20%
higher speed or a fixed20% rise in steady measured PWM. Targets and all
stop/optical guards are unchanged. Zero commands remain off.

## What changes

| Setting | Profile 1 | Profile 2 |
|---|---|---|
| Nominal centred speed | 45/45 mm/s | 45/45 mm/s |
| Line steering | V11 symmetric | Same V11 symmetric |
| Ordinary PWM ceiling | 300 permille (30%) | 300 permille (30%) |
| Extra load effort | None | Up to 180 permille added; total capped at 420 (42%) |

PWM is effort, not a torque or current measurement. Raising effort aims to
restore the existing requested speed, without raising the cruise target.
Each wheel uses its own request and encoder speed. An intentional slower
inner-wheel request does not itself qualify as a load deficit.

Assistance requires a visible line (at least one normalized level >=400),
no line-loss recovery, target >=20 mm/s, and the speed ramp at its requested
target (within 0.5 mm/s). A deficit >=15 mm/s sustained for 100 ms qualifies.
Physical extra output rises at900 permille/s to a maximum180. With deficit
<=5 mm/s it decays at600 permille/s. Overspeed of >=3 mm/s clears it
immediately. An intermediate deficit holds the existing extra effort.
Zero targets or lost eligibility clear it. PI anti-windup remains in original controller units before scaling and includes the extra
effort and actual output limit. These are provisional experimental settings.

This can respond to friction and other loads as well as a hump. It cannot
identify an incline, distinguish an obstruction, or detect body motion if a
wheel spins without traction. IMU data could later help select a terrain
policy; speed feedback would still control the wheels. No ramp dimensions
or assumed position are used by the controller. One wheel climbing changes
roll and sensor height; the line readings must also be inspected.

## V3 test: flat, small black, then orange

1. Flash `firmware/buddy23_hump_line_v3.uf2` using BOOTSEL.
2. Open PuTTY (115200, 8N1, flow control None), press `h`, and confirm
   `BUDDY23 HUMP LINE v3`, `valid=1`. On first use it imports the existing
   V9 front calibration (`source=1`) or V11 calibration (`source=2`).
3. Keep the same sensor height and adjacent mounting. Put both front sensing
   points over the straight black strip. Select `2`, then `k` while stopped
   with both buttons released. This saves the assisted profile for bank use.
4. Use a power bank through the existing Robo Pico power input for a free
   rolling test: wait 5 seconds after boot, hold both buttons for 3 seconds,
   release both, wait 1 second, tap/release GP21, then let the 2-second
   countdown finish. Either button stops the run.
5. Start centred on flat with both normalized levels high and similar, then
   approach the small black hump under the RIGHT wheel. Press GP20 after
   clearing the hump, before a junction or mat edge. Wait 3 seconds after
   stopping before changing to computer power, open PuTTY, and press `p`
   for the log.

Use one power source at a time. The test ends after 10 seconds or a 500 mm
encoder path, whichever comes first. Optical line loss and existing stall
protection may end it earlier. It allows repeat runs after saving/settling
and 1 second stopped with both buttons released; a fresh sequence is needed.

Check a short flat run first, then film wheel/sensor behaviour during the small-hump retry. Test the orange hump only
after the small one passes. Success on either fixture does not establish
that every hump height, slope, or surface is traversable.

### USB and calibration controls

- `g`: hands-free start after 2 seconds, buttons released; `s`/either button
  cancels or stops. Avoid cable pull during movement.
- `f`: hold GP21 while driving; release stops immediately.
- `1` / `2`: baseline / assistance; `k` saves calibration and selected profile.
- `h`: help, endpoints, selected profile, calibration source and restored flag.
- `p`: saved hump run log, including `extra_L` and `extra_R` in permille.
- If mounting/height changed or `valid=0`: stopped/released, capture both
  white with `w`, left black with `l`, right black with `r`, then `k`.
  Wait a second at each stationary position before capturing. Do not lift
  the chassis while taking endpoints.

## Limits and retained behaviour

- Same front A0 pins: left GP26, right GP28; 3.3 V and common ground.
- Same motor/encoder wiring and forward signs. GP19 is unused in this test.
- Forward only; no reverse or active downhill braking. Overspeed reduces
  drive, so this is not a guarantee against acceleration on a descent.
- Retains 500 ms blocked-wheel detection, encoder direction/invalid-edge
  checks, 1 ms independent button/deadline/late-control cutoff, and USB-change
  stop. Assistance must act within the retained stall window.
- Ordinary loss recovery retains V11's output policy and bounded search.
- Software tests verify caps and logic. V3 physical hump capability remains
  unverified until user tests.
- Prior V9 junction and V11 UF2 artifacts are preserved. V11 source is unchanged. Before eventual
  integration, repeat flat, both bend directions, junction approach/turn/
  alignment, and line-loss checks with the new policy.

## Saved data and rollback

Hump calibration/log uses the penultimate 4 KiB flash sector at `0x101fe000`.
The final sector at `0x101ff000`, containing the V9 junction record and side
calibration, is read-only here. Flash saves occur only with drive stopped.
The image is checked to end before both reserved sectors. No flash-nuke.
Flashing this image preserves the junction sector. Running/saving junction V10
replaces its own sector; V9 cannot read revision10 and needs recalibration
if rolling back after that save.

If a valid hump record already exists, it takes precedence over the legacy
record on future boots. V1/V2 hump calibration/log remains readable; an old run
retains its recorded revision until a new V3 run replaces it. Saving selection alone retains
the recorded run revision. Recapture `w/l/r` if calibration must change.

## Build and checks

Pico SDK 2.2.0, Pico W/RP2040, Arm GCC 14.2.1, C11, warnings as errors,
USB stdio enabled and UART stdio disabled. Configure with the existing local
Pico SDK/toolchain; `cmake --build build`. Host checks:

```text
gcc -std=c11 -Wall -Wextra -Werror -O2 test_hump.c -lm -o test_hump
./test_hump
python verify_firmware.py
```

See `BUILD_V3.md` for the artifact hash and `MEASUREMENTS.md` for the failed
V9 hump run that motivated this separate test.
