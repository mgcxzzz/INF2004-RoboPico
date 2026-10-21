# Analog line following bench v11

Working V11 setup, successful recovery results and the proposed next experiment
are recorded in [BASELINE_AND_JUNCTION_PLAN.md](BASELINE_AND_JUNCTION_PLAN.md).

Standalone hardware test: GP26/GP28 analog sensors -> line error -> Buddy2
encoder PI speed control -> both motors. It is NOT the real RTOS Navigation
Coordinator/S3 task integration. That build's bank-only startup remains unresolved.
No barcode, GP19 use, junction branch turn, obstacle module, radio or IMU.
Boots DRIVE OFF; calibrating, printing, saving and switching power never start it.

## V11: adaptive speed and tighter symmetric turns (profile6)

User approved stronger/weaker correction by imbalance, including permission to
increase PWM if necessary. This version keeps the tested25% cap: it gets tighter
turns by reducing the inner wheel, not assuming additional PWM is needed. A
loaded outer-wheel speed shortfall can be evaluated separately from the next log.
No motor voltage/rating claim or untested higher PWM is introduced.

Historical V9 offset evidence is saved in logs/v9_offset_response_2026-10-21.txt:
normal5s timer stop, initial right correction then extended left correction,
about-31deg encoder heading at cutoff and right PWM reaching250permille. This
shows an offset response, not proven stable centring. The later user-supplied
steep-angle log requested left and ended at the650ms loss bound; arbitrary steep
stationary acquisition is explicitly out of scope. Test nearly aligned starts.

The latest V10 log is saved as logs/v10_successful_run_2026-10-21.txt. The user
subsequently clarified that this was a SUCCESSFUL physical run, not the intended
failure example. Its normal10s timer stop must not be reported as a line-loss
failure. It is archived as successful user-observed evidence; do not infer a
failed bend or a need for more PWM from it. V11 is the previously requested
adaptive-steering experiment. Subsequent successful V11 recovery runs in both
directions are recorded in BASELINE_AND_JUNCTION_PLAN.md; continuous-course
bend and junction validation remain separate physical tests.

Profile6 now maps the magnitude of calibrated difference (L-R) progressively:

| Absolute difference | Inner/outer target mm/s | Average forward target |
|---|---|---|
|0..80|45/45|45|
|250|40/50|45|
|500|25/50|37.5|
|1000|0/45|22.5|

Integer interpolation joins the anchors with at most1mm/s adjacent changes.
In the strongest segment the outer target is quantized alongside the inner so
rounding cannot weaken turn curvature as error increases. More left black makes
LEFT the inner wheel and turns LEFT; more right black mirrors it. A zero target
resets its speed PI and bounded kick and sets both motor bridge PWM channels0:
this is coasting, not active braking, a locked wheel or reverse drive. Tiny
nonzero targets may still be limited by physical stiction; observe the next log.

BOTH BLACK remains normal. Both-white recovery retains the same reliable-side
memory (difference>=200 and one level>=400 within500ms),650ms/25mm bounds and
unknown-direction stop, but now uses0/45left or45/0right. This pairs recovery
with the severe-error turn instead of reverting to a shallow30/60 arc.

Keep10s/500mm profile limit,125ms logging, calibration format, sampling/filtering,
PI gains,100mm/s^2 ramp for nonzero targets,25% PWM cap, startup assistance and all
button/USB/encoder/feedback/control-late cutoffs. No derivative term or automatic
motion/calibration added. Old profiles1..5 remain unchanged and still use the old
placement. Profile6 behaviour needs physical verification; sharper steering can
also overshoot, so this is not a guarantee of smooth tracking or curve success.

### Next test: centred straight, then modest offsets after review

1. Flash firmware/buddy23_analog_line_follow_v11.uf2; PuTTY1152008N1 flowNone.
   h confirms V11 and valid1. Existing calibration should load (white184/184,
   black3464/3692 in latest stationary test). Recalibrate only for mounting/height/
   surface changes, not the firmware version. Press6 then k.
2. Keep the symmetric spots, normal load, both on black, robot nearly parallel.
   Provide about600mm clear continuous line/stopping room. Run centred first.
3. ComputerUSB OFF before bank ON. Wait5s,BOTH3s,release BOTH,wait1s,GP21tap/
   release,2s countdown. Either button stops. It can still stop before10s due to
   distance/loss or another guard. Do not extend recovery to force completion.
4. Wait3s after stop, bank OFF then computerUSB ON, p. Save under a new filename.
   Send the full run and whether steering is smooth, whether an inner wheel
   pauses during large errors, and whether it returns to both-black balance.
5. After reviewing centred behaviour, use a small sideways offset with robot
   still parallel: L1000/R700 for LEFT correction, orL700/R1000 for RIGHT. Do not
   start at a steep crossing angle. One fresh run/log per test. These do not prove
   continuous bends; later mat tests must cover both bend directions.

## V10: user-requested longer symmetric run

Use profile6 with the new symmetric placement. Only the profile6 run envelope
and logging cadence changed:5s/200mm ->10s/500mm;75ms ->125ms trace interval so
all10s plus1s settling fit the existing100-row flash record. A compile-time
capacity assertion checks this budget. Steering, calibration, PI, PWM cap,
startup boost,650ms/25mm loss recovery and all other motor stops are unchanged.
This allows observing settling for longer; it does not prove that the earlier
overshoot was caused solely by the shorter run. Earlier records remain readable.

Flash firmware/buddy23_analog_line_follow_v10.uf2; h confirm V10/valid1;6 then k.
Keep current calibration if surface/mount/height unchanged. Give roughly600mm
clear continuous line and stopping space. Start with the same modest offset or
centred position being compared; keep normal load. ComputerUSB OFF before bank
ON; wait5s,BOTH3s,release BOTH,wait1s,GP21tap/release,2s countdown. Either button
stops; USB-only g/s also supported. Run ends at10s OR500mm, or earlier for loss/
other guards. Wait3s after stop, switch power sources separately, p. Save full
log under a distinct name before another run. Observe whether it settles after
5s or continues weaving; a longer run cannot substitute for later bend testing.

## V9: NEW symmetric two-sensor placement, profile6

Both front modules now sit side by side near the robot centre. The user observed
both-black when centred, left fades with robot pointing left, right fades with
robot pointing right. Stationary evidence/calibration is saved under
logs/symmetric_stationary_2026-10-21.txt. Reported calibration: white184/184,
black3464/3692. The snapshots988/982,6/1000,1000/24 produce straight/right/left
requests respectively. These are stationary tests, not successful driving trials.
Assumes lateral movements/yaw at constant height; lifting contaminates readings.

Use ONLY profile6 with the new layout. Historical profiles remain available for
reproducibility but their asymmetric/left-edge geometry is incompatible.
The new controller is in symmetric_logic.h; original controllers/PI/kick are
unchanged. Main dispatch selects this code only for profile6.

### Symmetric rules

- Normalize each sensor with its OWN white/black endpoints. Error=levelL-levelR.
- Difference strictly between-80 and80: both targets45mm/s (noise deadband).
- More left black -> left slower/right faster -> turn LEFT.
- More right black -> left faster/right slower -> turn RIGHT.
- Correction=15*error/1000mm/s. Targets45-correction,45+correction, bounded30..60.
- BOTH BLACK is normal straight, not a crossing stop. Both saturated spots leave
  a blind band: equality cannot prove exact geometric centring or detect a junction.
- Both levels<150: remember only a clear asymmetry (absolute difference>=200 and
  at least one level>=400) seen within500ms of loss. That side is latched at loss.
  Recover with30/60left or60/30right, at most650ms OR25mm average absolute
  encoder path. If no recent reliable side, STOP immediately. No invented left
  recovery, reverse drive, or unbounded search. Return to proportional control
  when either level>=150. This heuristic does not resolve every ambiguous loss.
- Start requires valid calibration, ready/released buttons/mode gates and at least
  one normalized sensor>=650. Both-black and either offset side are accepted.
- Profile6 is SHORT5s OR200mm, trace75ms (fits100-row buffer plus settling).
- Stop reasons, wheel PI gains,100mm/s^2 ramp,25% PWM cap, bounded startup boost,
  encoder guards, USB disconnect and independent1ms late-control cutoff retained.
- Decision11=balanced straight,12=left,13=right,14=left recovery,15=right recovery,
  4=line lost. Neither these states nor heading estimates identify a + junction.

### First home test: centred straight ONLY

1. Flash firmware/buddy23_analog_line_follow_v9.uf2. Open PuTTY1152008N1 flowNone.
   h must show V9/valid1 and your new calibration. No recapture for firmware alone.
   If home line/surface, mounting or height differs, redo stopped w/l/r then k
   using the actual home test surface. Keep sensors at normal loaded driving height.
2. Press6 then k; h verify profile6. BOTH spots on black, normalized levels near
   900..1000 and difference preferably below80. Do NOT aim one sensor at500 and
   the other on white. At least250..300mm clear line gives room for stopping.
3. Offline: computerUSB OFF before bank ON; wait5s, hold BOTH3s, release BOTH,
   wait1s, tap/release GP21;2s countdown. Either button stops. Normal travel may
   be below200mm if5s expires. Support no part of the chassis by hand during drive.
   Alternative USB only: both released, ready1, g ->2s countdown; s/either button/
   USB disconnect stops. Use cable slack. Never bank and computerUSB simultaneously.
4. Wait3s after stop, bank OFF then computerUSB ON, open PuTTY and p. Save the
   complete run under a distinct filename before requesting another movement.
   Send the log and whether it stays near the strip and whether startup/corrections
   jerk. Video optional. Stop here for analysis before curve or longer testing.

### Follow-up tests on the SAME straight (after centred result)

These are correction tests, not replacements for later continuous left/right
curve trials. Keep both wheel contacts on the surface, same heading, slide only.

- Robot slightly RIGHT: left remains dark, right becomes less dark; aim a moderate
  level difference around+200..350. Expected LEFT correction toward the line.
- Robot slightly LEFT: right remains dark, left less dark; difference-200..-350.
  Expected RIGHT correction. Do not start with both white or far away.
- Use one run/log at a time. Return to centred placement between runs. Failure
  to settle, excessive weaving, stale/missing logs or no movement are results to
  inspect, not reasons to extend timers or raise PWM.

SavedLog structure/version unchanged; profile6 extends selected value to5 and
recorded-profile byte to6. V9 reads old calibration/log records. Older firmware
can reject a V9 record with selected OR recorded profile6. Changing selection
alone may not restore downgrade compatibility while the old trace remains;
record calibration endpoints and expect to recapture after a downgrade.

## V8: separate left-curve recovery experiment (profile5)

User's V6 profile2 run tracked a straight then lost the line on a LEFT bend.
The saved source log is logs/v6_left_bend_loss_2026-10-21.txt. Final continuous
white interval ended at9.538s after the650ms bound (about8.888s onset). At8.931s
heading estimate was+0.28deg; cutoff-3.20deg. Thus about3.5deg of estimated left
rotation in the last logged part, while requesting about30/60mm/s. Both wheels
kept moving forward, so recovery was a shallow arc. No encoder-invalid or
row-loss errors; this was neither overall12s timeout nor600mm distance limit.
An earlier brief contact at8.778s reset the loss interval. We cannot infer the
physical curve radius or global heading from those encoder values alone.

V8 adds profile5; profiles1/2 (edge),3 (wheel response),4 (V7 black=straight)
remain unchanged. Select5 then k to save. Normal tracking is the V6 edge rule:
left target500, cruise45, wheel requests30..60mm/s. On BOTH levels<150:

-Left target0mm/s, PWM0 (coasts, no active braking or reverse polarity).
-Right target45mm/s under unchanged speed PI and bounded startup assistance.
-Decision10 distinguishes this recovery from decision3 in old profiles.
-Stop after1200ms OR30mm average absolute encoder path, whichever comes first.
-Reacquiring contact (either level>=150) returns to normal edge/right-assist
 logic and resets the white timer. Both-dark>=750/120ms still stops.
-Overall600mm/12s, trace150ms interval, independent timer/button/USB/encoder/
 feedback/direction guards unchanged. Stop sets both targets/PWM0.

The longer TIME allowance is coupled to a SMALLER path limit (30 vs40mm) and a
stationary-left request, not simply a longer period driving blindly forward.
Ideal no-slip stationary-left pivot would rotate about31deg over30mm average
wheelpath; this is geometry, NOT a measured bound on chassis heading. Real coast,
slip or drag changes it. No dynamic loss timeout in the1ms ISR: recovery is
checked in the control loop with its existing100ms independent late cutoff.

This profile assumes the missing line lies LEFT, appropriate to the reported
left bend but ambiguous on arbitrary departures. It is not general search,
right-bend recovery, edge-of-mat detection, or full-course/junction navigation.
It may still require repeated recovery on a tight curve and is not claimed smooth.

### Use after the currently running V7 experiment

Keep/send V7 profile4's result before switching firmware. V8 still includes4,
so selecting4 reproduces the same black-straight algorithm. For the new recovery:

1. Flash firmware/buddy23_analog_line_follow_v8.uf2, open PuTTY1152008N1 flowNone.
   h verifies V8/valid1 and the current close-height calibration; no recapture
   unless mounting/lighting changed. Press5 then k; verify profile5.
2. Place on a short straight immediately BEFORE a gentle LEFT bend. Left edge
   level450..550/rightwhite. Keep mat beneath the entire planned route and allow
   stopping space; do not start all-white or beyond the curve.
3. ComputerUSB OFF before bank ON. Wait5s, BOTH3s, release BOTH, wait1s,
   GP21tap/release;2s countdown. Either button stops. Observe whether the left
   wheel stops briefly and right wheel turns the chassis back onto the line.
4. After stop wait3s, bank OFF then computerUSB ON, p. Send the complete log,
   whether recovery reacquires the line, and any video of the left bend.
   If it repeatedly searches or fails, save that result rather than increasing
   the recovery time further. A new run replaces the previous log.

SavedLog structure/version unchanged. Selected profile5 extends range to4;
reserved recorded-profile byte5. Old records/calibration load in V8. Older
firmware rejects a saved selected profile5; select1 then k before downgrading.

## V7: experimental left-black means straight (profile4)

User explicitly requested trying level1000 as the straight condition. Profile4
implements a different discrete strategy; it does NOT infer the physical centre
of the strip from a saturated reading. Profiles1/2 keep V6 analog edge following,
profile3 keeps the original wheel-response sequence. No PI/PWM/kick changes.

Profile4 targets and normalized thresholds:

| Condition | L/R request mm/s | Decision |
|---|---|---|
| Left>=850, right<650 |45/45 straight|8|
| Left650..850 |retain previous straight/reacquire state|8 or9|
| Left<=650, right<650 |30/60 steer left to reacquire|9 (3 if both white)|
| Right>=650, unless both>=750 |60/30 steer right|1|
| Both>=750 |35/35 then stop after120ms|2|
| Both<150 |left recovery, stop after650ms or40mm|3 then4|

The850-enter/650-exit hysteresis avoids rapid switching around one threshold.
Left1000 requests equal wheel speeds when right is white; exact1000 is NOT
required. Right assist clears straight state until left black is reacquired.
Initial profile4 alignment requires left>=850/right<=450, valid calibration,
released/rearmed buttons and all original mode start gates. Profile4 uses a
SHORT3s/150mm bound independent of profile2. All motion/encoder/USB/timer guards
remain. A new run resets the state. Calibration is shared with the other profiles.

Limitation: with35mm sensor spacing and20mm line, both-white can mean departure
on either side. This experiment assumes the line is LEFT during both-white
recovery; it stops when recovery fails. It is not general line search, automatic
centre finding, a junction navigator, or validated curve following. If the user
starts on a straight, drives until a gentle curve and observes loss, save the log
rather than extend the recovery timeout. Existing startup kicks/contact drag may
still cause jerking even though left-black no longer commands a right correction.

### First test: short straight with LEFT fully black, RIGHT white

1. Flash firmware/buddy23_analog_line_follow_v7.uf2. PuTTY115200,8N1,flow None.
   h confirms V7/valid1 and the current close-height calibration. No recapture
   solely for this firmware; keep mounting and lighting unchanged.
2. Press4 then k. Verify h says profile4. Left sensing point fully inside black,
   right white; aim levelL850..1000 and levelR<150. Do NOT aim left500 here.
3. Computer USB OFF before bank ON. With normal load, on a clear straight, wait5s;
   hold BOTH3s, release BOTH, wait1s, tap/release GP21;2s countdown. Either button
   stops. Maximum3s or150mm; lower travel at the time limit is normal.
4. Wait3s after stop. Bank OFF, computer USB ON, PuTTY p. Send complete log and
   whether it stays straight/smooth, and whether it loses contact or jerks.
   Header must say experimental black means straight and profile4; old restored
   records are not evidence for V7 until a fresh run has completed.
5. Repeated runs still supported. Use1/2 then k to return to edge following.
   Test the short straight before curves or any longer black-straight profile.

SavedLog structure/version is unchanged; selected profile4 uses value3 and the
reserved recorded-profile byte uses4. V7 reads earlier records/calibration.
Older V1-V6 firmware rejects a saved selected profile4; choose1 then k in V7
before downgrading to keep the saved calibration readable.

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

Computer USB only, flash firmware/buddy23_analog_line_follow_v11.uf2 with BOOTSEL.
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
