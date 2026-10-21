# Analog left-edge line following bench v3

Standalone hardware test: GP26/GP28 analog sensors -> line error -> Buddy2
encoder PI speed control -> both motors. It is NOT the real RTOS Navigation
Coordinator/S3 task integration. That build's bank-only startup remains unresolved.
No barcode, GP19 use, junction branch turn, obstacle module, radio or IMU.
Boots DRIVE OFF; calibrating, printing, saving and switching power never start it.

## V3: repeat runs, looser start and optional USB hands-free mode

User requested repeat tests without power cycling and less restrictive starting
alignment. Left-on-black level1000 previously failed the250..750 gate. The new
start gate permits LEFT150..1000 and RIGHT0..450, with valid calibration. Thus
left full black starts, but all-white, only-right-black and both-dark reject.
This changes only the gate, not normalization, edge target500 or calibration.
The controller may make a stronger initial correction when starting on full black.

New runs require settling/save complete, both buttons released, and estimated
wheel speeds below5mm/s continuously for1second. ready=1 indicates this gate.
Holding GP21 after readiness supports f; a stop/active run/wheel movement clears
readiness. Readiness alone NEVER starts drive. Each repeat needs f, g, or a fresh
bank-only both-button/tap sequence. Record p before the next run replaces its log.
No power cycle is needed between accepted runs. UART and GPIO wiring unchanged.

USB hands-free: select1 for first short test, release both, wait ready=1, send g.
A2second countdown precedes movement; release the robot before it starts. Either
button or s cancels countdown/stops motion; USB/DTR disconnect also cancels/stops.
Start conditions are rechecked at countdown expiry; rejection prints its exact
reason and current levels. Calibration/profile changes cancel a pending countdown.
The same3s/150mm or12s/600mm profile limits and independent timer still apply.
This avoids pressing on the chassis during a floor test. Give the USB cable slack.
The original f requires GP21 held; releasing it stops. Bank mode stays bank-only.

Live ANALOG adds ready, pending and runs (runtime accepted-run counter). Log key=g
means USB hands-free; key=f is USB hold or bank as labelled in mode. Flash record
layout/magic/version is unchanged: saved V1/V2 calibration and logs remain readable.
Software checks passed; new repeat/hands-free behaviour still needs hardware tests.

## V2 floor-start adjustment

V1 did start offline and save its log, but the wheels travelled only about2.2mm
before its no-movement guard stopped the run. V2 raises only breakaway assistance
from180permille/80ms to250permille/120ms. It ends early once feedback reaches20mm/s,
then waits100ms before another eligible pulse. Main-loop sampling checks the
pulse deadline: duration can include up to the next control update; the separate
100ms control-late cutoff still removes drive if updates cease. The no-movement
counter is NOT reset by a boost or cooldown. PWM cap remains250permille throughout.
Steady motion still uses PI speed feedback and the same analog target rule.
V2 hardware performance is not confirmed by compilation or host tests.

Calibration/profile/log flash format is unchanged from V1. Existing saved analog
calibration should load without recapture; verify h shows valid1/profile1. After
any change in sensor height/brackets, recapture the affected endpoints and k.
Do not use old raw edge targets after recalibration: align normalized levelL500.

## Wiring

Left A0 -> GP26/ADC0; right A0 -> GP28/ADC2. Sensors powered3.3V with common GND;
D0 unused. Encoder L GP0/1, R GP4/5. Motor outputs GP8..11, buttons20/21 unchanged.
Use only one power source, bank OR computer USB. Never overlap them.

## First stage: CALIBRATE ONLY, no f/buttons yet

Computer USB only, flash firmware/buddy23_analog_line_follow_v3.uf2 with BOOTSEL.
PuTTY Serial115200,8N1,flow None. All keys are lowercase, no Enter needed.
Keep the sensors at final mat height, fixed spacing/lighting, no pot adjustment.

1. Both sensing points over white, wait1 second, press w. This resets the old
   black calibration and captures BOTH white values.
2. LEFT spot well inside black, RIGHT white; wait1 second; press l.
3. RIGHT spot well inside black, LEFT white; wait1 second; press r.
4. Press1 (short profile), then k to persist calibration/profile to flash.
5. Press h; send the CAL line. Expect mask7, valid1, profile1 and a save-success
   message after k. restored indicates loaded at boot, NOT whether k succeeded.
   If SAVE FAILED or valid0, stop here. No motion before calibration/edge check.

Do not press l/r expecting turns: in THIS firmware they capture black values.
Calibration uses latest32-sample averages; wait on the surface before each key.
Minimum required black-white span100 counts EACH; measured polarity is black high.
White values are not assumed identical. The supplied user evidence is recorded
in MEASUREMENTS.md but is not hardcoded as final calibration.

## Place the LEFT sensor on the RIGHT-HAND edge of the strip

Directions are from the robot looking forwards. Its physical LEFT sensing spot
should sit across the strip's right black/white boundary, with physical RIGHT
sensor on white beside the strip. Move laterally until live levelL is near500
and levelR below200. Normalized levels are0white..1000black, separately calibrated.
Left fully black is NOT the target: we follow the edge with intermediate reflectance.

Above the midpoint, the left wheel is commanded faster (turn right). Below it,
the right wheel is faster (turn left). At midpoint, both request65mm/s. A small
deadband60/1000 reduces noise corrections. Differential capped30mm/s, so wheel
targets stay35..95mm/s. A dark right sensor requests a bounded right correction.
Both-dark>=750 each for120ms stops as an AMBIGUOUS crossing, not a proven +.
Both-white<150 each seeks left for at most650ms OR40mm estimated wheel path,
then stops. Analog readings still cannot locate a line outside both sensor spots.
This fixed recovery is only appropriate near the calibrated right-hand edge;
if positioned on the opposite edge its correction will be wrong.

Starting requires valid calibration, levelL150..1000, levelR<=450, GP20 released,
settling complete and ready=1 after the stationary/released interval. Treat persistent start rejection
as a diagnostic, not a reason to increase sensitivity or bypass the checks.

## Raised steering/stop stage, only after CAL review

Support both wheels off the floor. Present mat/tape at the normal sensing height.
Do not change calibration height just to raise wheels.
HoldGP21 and send f. Start on the calibrated edge, then vary the surface under
LEFT slightly each way to check correction direction. ReleaseGP21 to stop;
GP20 or serial s also stop. All motion is forwards, no on-the-spot pivot.
Wait3 seconds, release both buttons and press p for the complete run. When
ready=1, request another f/g without unplugging. Saved calibration remains in use.

## Offline floor stage, later after raised checks

Profile1: estimated average absolute wheel path150mm OR3seconds.
Profile2:600mm OR12seconds, for a longer curved segment; calibration unchanged.
Choose1 or2 while stopped/released, then k to save before changing power sources.

1. Computer USB fully disconnected; power bank only, allow5seconds startup.
2. Put left spot on calibrated right-hand edge (right spot white), clear run space.
3. Hold BOTH buttons3seconds, release BOTH, wait1second.
4. GP21 press/release requests a2second countdown; either button cancels it.
5. Either button stops an offline run. It also stops at loss/crossing/feedback
   fault or profile distance/time bound. USB host enumeration cancels offline
   arming and stops offline motion, independently of PuTTY/DTR.
6. Wait3seconds AFTER stopping. Disconnect bank, connect computer normally,
   open PuTTY115200 and p. Do not reflash between run and retrieval.

Limits are output cutoffs, not physical stopping guarantees. Encoder distance
and heading are wheel estimates, not external position/compass measurements.
Record actual distance, direction of drift and a video when possible. Neither
profile is a full-course autonomous run; curved tracking requires physical tuning.

## Motion guard and records

PI/encoder polarity/calibration from Buddy2 V7 at deac4940395b6cb9da23cfa29003e95f75a7f0fa:
L2524.05 CPR, R2528.30 CPR,200mm circumference,110mm wheel spacing (provisional).
Positive wheel speed means forwards on both motors; right PWM polarity inverted.
Hardware1ms timer cuts on buttons, run deadline or100ms late control loop.
10ms target control (actual elapsed time),25% PWM cap, encoder invalid>=8,
reverse feedback<-20mm/s50ms and no movement under demand500ms stop the run.
Bounded25%/120ms forward breakaway pulses, end at20mm/s,100ms cooldown; floor tuning
still needed. Main loop floating point cannot override a timer cutoff: generation
and active flags are rechecked with interrupts masked before committing PWM.

Raw ADC:32samples/channel, first after mux discarded. Control adds an IIR factor0.35.
Raw ADC columns and filtered normalized control columns may differ during motion.
Console logs every200ms, buffered trace50ms short/150ms long plus1s settling.
Decision:0edge,1right assist,2both-dark,3seek-left,4lost. Output stops have targets0.

CRC verified single4KiB flash record holds calibration, selected profile, latest
run. k and completed runs replace it. Calibration commands clear RAM trace to
avoid interpreting old readings using a new calibration. Previous probe/RTOS/
dual-bench formats are rejected, not converted. A calibration-only record has
no run: p says no recorded run, h still shows calibration. Flash saves occur
only drive-off on one core; interrupted save can lose the record. Don't save
continuously. User calibration/power-bank/steering results remain unvalidated.

## Software verification / build

Pico SDK2.2.0, Arm GCC14.2.1, Pico W/RP2040, USB stdio only (no UART).
Application -Wall -Wextra -Werror. test_line.c covers independent calibration,
normalized saturation, edge correction signs, right assist, transient/sustained
both-dark, bounded white recovery/reset, speed bounds across52k ADC combinations,
release/GP20/late-control guards and deliberate offline arming. It is not a
physical robot simulation or proof of stable floor following.
