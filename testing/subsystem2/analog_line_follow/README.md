# Analog left-edge line following bench v6

Standalone hardware test: GP26/GP28 analog sensors -> line error -> Buddy2
encoder PI speed control -> both motors. It is NOT the real RTOS Navigation
Coordinator/S3 task integration. That build's bank-only startup remains unresolved.
No barcode, GP19 use, junction branch turn, obstacle module, radio or IMU.
Boots DRIVE OFF; calibrating, printing, saving and switching power never start it.

## V6: gentler optical steering for the closer sensor mounting

The user reported jerking after lowering the sensors; calibration is now
white177/183,black3639/3629. The original user log is saved under
logs/v5_close_height_line_2026-10-21.txt. It stopped at the NORMAL3s time limit,
not the line-loss guard. It repeatedly crossed from near-white to dark and back;
wheel feedback lagged the target reversals. Right PWM often hit25% while speed
remained near65mm/s despite a95mm/s request. Sensor contact may add drag, but
this cause is not established by the log. Existing bounded startup kicks also
remain a possible contributor to the visible jerking.

V6 changes only the optical target mapping in profiles1/2:

| Setting | V5 | V6 |
|---|---|---|
| Equal-speed cruise |65mm/s|45mm/s|
| Proportional differential |+/-30mm/s|+/-15mm/s|
| Full-black L/R target |95/35|60/30mm/s|
| Full-white recovery L/R target |35/95|30/60mm/s|
| Right-sensor assist L/R target |95/35|60/30mm/s|

The full sensor-range reversal changes a wheel request by30 rather than60mm/s.
The normal max right-left difference is30 rather than60mm/s. Keep target500,
strict deadband|error|<60, same ADC sampling/IIR, white/both-dark thresholds,
650ms/40mm loss guard,120ms both-dark stop, profile distances/deadlines, V5 wheel
PI/ramp,25% cap and kick timing. No derivative or extra target smoothing added.
Both-dark pending targets stay35/35, then stop; profile3 stays EXACTLY65/65,
35/95,95/35 for2s each. New steering gains may be too weak for tight curves;
actual improvement is pending the next physical test, not proven by compilation.
Mechanical contact/drag is not corrected by a firmware change.

### Next physical test: profile1 on a straight, same close mounting/load

1. Flash firmware/buddy23_analog_line_follow_v6.uf2. PuTTY115200,8N1,flow None.
   h confirms V6 and valid1. Existing close-height calibration should restore:
   approximately white177/183,black3639/3629. Do not recalibrate solely for V6.
2. Select1 then k. Aim normalized levelL450..550, right near0, on the straight.
   Keep normal robot load, contact pressure and mounting consistent.
3. Computer USB OFF before bank ON. Wait5s; hold BOTH3s; release BOTH; wait1s;
   tap/release GP21;2s countdown. Either button stops. Keep the route on the mat.
4. Expect slower movement. Profile1 still stops at3s OR150mm: it may travel less
   than150mm at this lower speed, and a normal time-limit stop is not failure.
5. Wait3s after stop. Bank OFF, computer USB ON; open PuTTY and p. Send the whole
   run and whether corrections are smoother, whether startup still stutters,
   and whether the sensors remain in light consistent contact.
6. Validate this short straight before profile2 curves. Profile2 still permits
   600mm/12s but slower motion can reach the time limit before600mm. Neither
   profile changes its stop reason merely because it is now slower.

Flash Trace/SavedLog structure and version unchanged; V1-V5 analog calibration
and saved logs remain readable. A restored old run is still old evidence, even
though h now shows V6. Start a fresh run to evaluate the new steering.

## V5: provisional wheel-speed PI adjustment

V4 profile3 completed offline and the user observed forward/left/right motion.
The saved user log is in logs/v4_wheel_response_2026-10-21.txt. Averages of final
samples in each stage (1.4..2.0s,3.4..4.0s,5.4..6.0s) were:

| Stage | Requested L/R mm/s | Measured L/R mm/s | Mean PWM L/R permille |
|---|---|---|---|
| Equal |65/65|51.7/40.3|137.5/164.2|
| Left arc |35/95|40.6/79.7|91.2/228.7|
| Right arc |95/35|86.6/43.6|185.3/137.7|

These finite-stage samples show tracking error, not proven steady-state values.
No invalid encoder transitions/dropped rows; normal6s limit and CRC restore.
Final encoder average310.31mm is wheel-derived, not external travel distance.

V5 changes ONLY the wheel speed PI in this standalone test, via wheel_control.h:

| Setting | V4 | V5 |
|---|---|---|
| Kp (permille per mm/s error) |0.6|1.0|
| Ki (permille per mm/s error per second) |0.8|3.0|
| Integral bounds (permille) |+/-80|+/-140|

Keep feedforward,100mm/s^2 ramp, filtered speed,25% PWM cap, bounded breakaway,
all guards, calibration/flash layout, test phases and optical steering identical.
Conditional integration blocks further windup into upper/lower saturation but
allows opposite-sign error to unwind it. A zero request resets PI and outputs0.
No derivative term, motor rating assumption, automatic gain tuning or braking
polarity reversal. Shared dual_wheel_bench/dual_logic.h is unchanged.
The new PI is used in profiles1/2/3; physically validate profile3 BEFORE resuming
line tests. More correction could cause oscillation, so no improvement is claimed
until measured. Host checks validate bounds and correction behaviour, not the robot.

### Next test: repeat the V4 offline profile3 run

Use the SAME mat, load/power bank, wheel condition and placement as V4.
Flash firmware/buddy23_analog_line_follow_v5.uf2; open PuTTY115200,8N1,flow None.
Press h: confirm V5 and valid1 calibration; select3, press k. No recalibration
needed if sensors/mounting haven't changed. Do NOT recapture endpoints solely
because the firmware changed. Remove computer USB before applying bank power.
On roughly1m square clear floor, wait5s, hold BOTH3s, release BOTH, wait1s,
tap/release GP21;2s countdown then forward/left/right sequence. Either button
stops. Wait3s after stopping, remove bank, reconnect computer, open PuTTY, p.
Send the full log and whether startup/curves were smooth or jerky. Video optional.
The line does not steer profile3; identical6s/450mm limits. Do not run line tests
next until these wheel-response results have been compared.

## V4: isolate wheel-speed response before adding steering PID

The latest V3 floor run recovered optical contact several times but ended in
line loss. Near its end, targets were35/95mm/s while measured speeds were about
47/36mm/s and right PWM reached250permille. This does not identify the cause.
V4 keeps the V3 steering, wheel PI gains, feedforward, calibration, ramp and25%
PWM cap unchanged. It adds diagnostic profile3 to measure the existing response.
No derivative term or automatic tuning has been added.

### Next physical test: profile3 on the mat, normal driving load

1. Flash `firmware/buddy23_analog_line_follow_v4.uf2`; open PuTTY115200,8N1,
   flow None. Press h and verify V4 and valid calibration (mask7/valid1).
   Existing analog calibration should restore; keep sensor height unchanged.
2. Select3 then k. Profile3 is a WHEEL RESPONSE test, not a line follower.
   The sensing points can be anywhere; line loss/both-dark stops are disabled
   only in this profile. Calibration is still required. Provide roughly a
   1m square of clear floor: it will go forward, arc left, then arc right.
3. Computer USB only: release both, wait ready=1, send g; it starts after2s.
   Give the cable slack. Either button, s, or USB/DTR disconnect stops it.
   Alternatively, bank only: remove computer USB before applying bank power,
   wait5s, hold BOTH3s, release BOTH, wait1s, tap/release GP21, then2s countdown.
   Do not use both power sources together. Either button stops the offline run.
4. Stages from drive start:0..2s targets65/65;2..4s targets35/95;
   4..6s targets95/35mm/s. Existing100mm/s^2 ramp and PI continue across stages.
   The first stage includes startup, so compare later samples in each stage.
   Stop at6s OR450mm average absolute encoder path, or any motor safety guard.
5. Wait3s after it stops. USB: press p. Bank: disconnect bank, connect computer,
   open PuTTY, press p. Send the WHOLE run plus whether it moved continuously,
   whether the left/right arcs were visible, and any cable tug/changed load.
   Video is optional; keep the whole robot visible if recording.
6. To repeat, release both and wait ready=1, then request a new run. Save p first:
   the next accepted run replaces the buffered/persistent run.

Profile3 decision5=equal speed,6=right faster/left arc,7=left faster/right arc.
CSV targets are final requests (the internal ramp is not shown); PWM remains
permille,250=25%. Sensor columns are recorded for context but never steer this
profile. Log header TEST explicitly identifies wheel response. No movement or
arc is a hardware result, not a reason to defeat the guards or raise the cap.
Use1 or2, then k, to return to line following. Profile3 persists across reboot;
verify the selected profile before every offline button sequence.

Flash structure/version is unchanged. V4 uses the former reserved byte to keep
the recorded profile distinct from a newly selected profile; old zero values
fall back to the old selected-profile field. V4 accepts profile3 records; older firmware
rejects a V4 record with profile3, including its calibration. If downgrading,
select1 then k in V4 first. V4 does not otherwise require recalibration.

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

Computer USB only, flash firmware/buddy23_analog_line_follow_v6.uf2 with BOOTSEL.
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
