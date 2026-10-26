# Circuit with barcode V15: straight-only smoothing on the V14 baseline

Use firmware/buddy23_circuit_with_barcode_v15.uf2 on the same Pico W.
Select profile1 and retain the current calibration for the first comparison.
Profile2 is still the separate paused ramp experiment.

## V15 small-correction changes

V14 completed the supplied school run through straights and bends, with manual
GP21 stop at59.458s. Small straight-line jerks remained. The retained motion
window is40.615..60.445s, not the entire route, so straight/bend labels cannot
be inferred for every sample. Calibration is unchanged and valid.

V15 adds two narrow adapters to the working V14 base:

- Qualify both front levels>=700 and absolute difference<=220 for100ms. In
  this strong-black plateau, request equal45mm/s instead of chasing small
  brightness differences. Existing equal fast-cruise targets remain intact.
  Exit immediately if either level<600 or imbalance>300. Raw loss/junction
  thresholds, front calibration, recovery confidence and barcode ADCs do not
  change. Decision24 marks the settled paired-black hold in motion logs.
- Slew positive PWM at800 units/s upward and1600 downward ONLY while the
  hold is active, both actual wheel speeds>=25mm/s and both targets35..55.
  Start/slow wheel, zero output, loaded correction/capture, ramps, junction
  turns and stationary search bypass it. No new PI or V13 visible-wheel
  controller is used. This keeps breakaway/urgent bend authority available.

Both-black sensing does not prove geometric centring or a perfectly straight
track. This is deliberate small-error tolerance; it can change when correction
begins on a gentle curve. Verify the straight-to-bend transition physically.
Large-error and near-pivot bypass remain V14. No scan success is claimed from
input replay or host tests; improve movement first, then try one barcode.

## V14 baseline retained

V13 performed worse on the school circuit. Its gradual visible-wheel controller
and smoothing of strong corrections are removed from production. V14 restored smooth_follow.h
and motor dependencies from V12(commit f0e3b35); V15 adds only the adapters above.
The historical visible_wheel.h/test_visible_wheel.c remain as V13 references;
main.c does not include or call that controller.

The school V13 run repeatedly saw strong front levels while a wheel slowed to
zero, then entered white recovery. At9.585s it entered a loss; the first search
reacquired at11.189s. The final loss began15.495s, stationary searching began
16.998s, and its last endpoint ended the run at26.291s without a front return.
The encoders show counter-rotation, not an encoder-invalid stop. Much of the
871.2mm summary is accumulated wheel motion during that search, not forward
route progress. This evidence cannot prove physical wheel slip or tape wear.

Calibration remains white176/175,black3483/3498,span3307/3323. Fully covered worn
black measured left923..927/right775..786, well above detection/loss thresholds.
No threshold reduction, new hardcoded black reference or automatic calibration
change is justified by these samples. Faint moving readings can also result
from an edge, sensor height or displacement. Keep calibration constant for the
first V14/V12-controller comparison; reassess normal driving before more tuning.

V14 retains V13's1.5s white wait and stationary encoder-angle search described
below. A/F/Z decoding, fast Z-turn geometry/power, saved logs, wiring and all
normal speed/PWM caps remain. Strong visible corrections again bypass target
smoothing as in V12, and normal breakaway assistance is restored. This restores
the previous physical behaviour; it does not claim that the earlier V12 jerks
or every circuit geometry issue are solved.

| Complete Code39 frame | Queued action at the next qualified cross |
|---|---|
| *A* | Left |
| *F* | Right |
| *Z* | U-turn |

The reader still decodes generic Code39 symbols. Only these three queue route
commands; other valid symbols are reported unsupported. No character is guessed
from a partial pattern. All29 elements, valid start/data/stop and agreement of
two neighbouring ADC thresholds are required. The next command replaces an
older unexecuted supported command. Unknown/ambiguous reads never force a turn.

## V12 fast U-turn retained; V13 replaces rolling recovery below

| Demand | V11 | V12 |
|---|---|---|
| U-turn forward quarter |45 then30mm/s|90/55/35mm/s, taper at60/82 degrees|
| U-turn reverse quarter |35 then25mm/s|80/45/30mm/s, taper at150/170 degrees|
| Measured-still pause |200ms|120ms, still requires both speeds below5mm/s|
| Rolling search |45mm/s,30 while confirming|90mm/s,45 while confirming|
| U-turn/search PWM |42% sustained;48% turn-only start|60% sustained;75% bounded start|

The dedicated physical-PWM controller applies ONLY to Z turning and active
search wheels. It uses300mm/s^2 acceleration,700mm/s^2 reference deceleration,
Kp2.2/Ki6 and anti-windup. A start pulse ends at180ms or when measured along
speed reaches min(50mm/s,80% of target), whichever occurs first. No extra20%
multiplier is applied. The independent1ms ISR enforces the same750/600 limits.

V13 changes ordinary visible-line drive as described above. A/F decoding and
left/right turn power remain unchanged. Z geometry/completion remain as V11.
These higher demands still require physical tuning under scraping/slip.

### Both-white recovery

During ordinary following or outgoing-line alignment:

- The first150ms keeps the existing short-gap bridge when direction is known.
- After that, both-white readings command coast while a continuous1.5s white
  interval is confirmed. The robot remains active and can reacquire earlier.
- Both levels below150/1000 start the timer;150..199 is a hysteresis band;
  either level reaching200 resets the pre-search timer. Confirmed line return
  uses the existing40/80/120ms confidence bands.
- Once1.5s is reached, counter-rotate left/right wheels. Encoder-relative
  endpoints are+35,-60,+90,-120,+150deg, mirrored toward the last reliable
  line side; no history starts left.100mm/s per wheel tapers to25 within12deg
  of each endpoint. Wait for both measured speeds below5mm/s for80ms before
  reversing. This minimizes commanded translation around the rear axle.
- On a possible line, command both wheels OFF for the40/80/120ms confirmation
  dwell. A flash resumes the same search, never a renewed budget.
- Search ends at the last endpoint,12s active,1000mm accumulated wheel travel,
  or30mm absolute encoder mean-wheel displacement. A small common-mode trim
  opposes measured centre drift. Encoder estimates cannot detect tyre slip or
  guarantee a stationary chassis. Confirmed return resumes the restored V12 tracing.
- Outgoing alignment allows15s/1100mm to fit the wait and stationary search.
  Expected white during a planned Z turn does not start ordinary recovery;
  its own angle/time/line-confirmation limits still apply.

j adds RECOVERY_WAIT(width=ms), RECOVERY_SWEEP(width=mm/s,flags=stage1..5),
RECOVERY_FOUND and RECOVERY_FAILED(width=reason). This makes the wait and
search visible without requiring the long p dump.

## How the U-turn works

The first half reuses rolling approach geometry: the front-to-axle/offset
assumptions remain as previously measured, including side-ahead92mm and a
55mm early rolling turn start. A Z waits for a separate11..30mm effective
junction stripe and4mm white after its trailing edge. It never turns directly
at the barcode or after an assumed barcode-to-junction distance.

1. Right wheel forward, left OFF, to approximately90 degrees of encoder yaw.
   The crossing branch near90 degrees is deliberately ignored.
2. Both outputs OFF; wait until measured speeds are below5mm/s for120ms.
3. Left wheel reverse, right OFF, continuing the same leftward yaw toward180.
   Slow the reverse wheel near the expected return line.
4. Require prior departure from the old line,165..195 degrees of encoder yaw,
   and a front line signal of at least400/1000 for30ms. Stop and settle again.
5. Use the existing outgoing-line follower to confirm the visible branch for
   150ms, then count the U-turn and continue ordinary following indefinitely.

A forward-only180-degree arc around one stopped wheel would finish one track
width beside the original line. This two-quarter manoeuvre instead returns
to the original longitudinal line under ideal no-slip geometry. It involves
forward and reverse rolling arcs, so it needs clear floor around the junction.
It does not counter-rotate both powered wheels while stationary.

V12 U-turn target tiers and power are listed above, independent of faster
profile settings. Signed output is enabled for Z turning and active stationary
search, including its settling/sighting stops. The ramp
controller remains disabled during every junction manoeuvre.

Stationary-wheel drift is limited to12mm, active-wheel travel to220mm per
quarter, each quarter to8s and the turning/settling sequence to18s. Settle
phases allow2s. Missing return line by195 degrees faults. The V13 15s/1100mm
outgoing alignment window and manual/USB/control/direction/invalid checks
remain. Physical slip/drag can still require tuning; no IMU is assumed.

## Test with profile1

1. Keep the mounting, wiring and saved front calibration. Flash V15, press h
   and verify the V15 banner and valid=1. Select1, then k to retain that profile.
2. Put a complete Z barcode beside a straight leading to the cross, in the
   same position used by A/F. Start the offset sensor roughly20mm before it.
3. Start with g over USB for a2s countdown, or use the usual power-bank
   gesture: wait5s after boot,both buttons3s,release,wait1s,tap GP21,2s countdown.
4. Observe decode, junction approach, forward quarter, pause, reverse quarter,
   pause and line-following back along the approach. Stop manually after enough
   return travel to judge acquisition; wait5s before changing power.
5. Send j. Expected evidence is DECODE Z/U, CROSS, DISPATCH Z/U, UTURN_STAGE,
   COMPLETE Z/U, decoded_Z=1 and uturns=1. If a stage fails, send j first; p is
   optional detailed wheel/optical evidence. a is only needed for scan problems.

The unchanged A/F reader and left/right manoeuvres remain available. Use the
actual Z print; reusing A/F does not exercise Z decoding. Measured geometry
is still provisional if the offset mount moved since the92mm measurement.

## Wiring and calibration

Front-left A0 GP26/ADC0; front-right A0 GP28/ADC2; offset A0 GP27/ADC1,
physical Pico W pin32. D0/GP19 is unused.3.3V/common GND are unchanged.

Front-only w=both white,l=left black,r=right black,k=save. Keep a working saved
calibration rather than recapturing just for this firmware. Minimum span is50.
Barcode A0 has no manual calibration. Its63-bank search retains coarse
128..3968/128 with +/-32 hysteresis and fine192..254/2 with +/-3 hysteresis,
2ms confirmation and black-high polarity. Ordinary visible-line smoothing and cruising retain V9 for profile1. Profile2 retains V10's180mm/s experimental ramp controller;
profile3 remains135mm/s stable cruising. Scanning/pending approach uses45.

## Logs, distance and storage

h=help/calibration; j=compact events/summary; p=motion+events; a=raw barcode;
v=live toggle(defaultOFF); s/either button=stop. f still requires GP21 held
on USB. Manual buttons, USB changes and existing control guards remain.

UTURN_STAGE width column is rotation in degrees. Flags1=first quarter reached,
2=reverse quarter started,3=return line seen,4=forward alignment started.
COMPLETE action U denotes a finished Z manoeuvre, separately from lefts/rights.
SUMMARY/live include decoded_Z and uturns; they survive offline save/restore.

From V11, motion/SUMMARY distance accumulates absolute encoder deltas so the
reverse quarter is included in travel and average-speed estimates. Signed
wheel positions still determine yaw and bar widths. Event distance remains
signed mean-wheel position and can decrease during reverse; it is not the
whole-run travelled-distance metric. Subsequent junctions use cumulative path
and relative signed wheel geometry consistently.

The main record appends8 bytes for Z/U counters: old64840-byte V1..V10 prefixes
and trace56/turn-summary40 layouts remain byte-compatible; V11-V15 use64848 bytes.
CRC length depends on version. V1..V14 calibration/logs still load; unknown16
rejects. Main bank remains0x101d0000. Raw record/address is unchanged at
0x101b0000, separate from probe calibration0x101c0000/0x101c1000.

Raw capture retains the first4096 nominal2ms A0 samples with actual timestamps,
signed mean-wheel progress and ADC. a prints a paced5ms/row CSV with
RAW_ADC_BEGIN/END counts and CRC. Wait for the footer and send a full PuTTY
logging file if raw analysis is needed. full=1 means later run samples were
omitted. matches_motion checks capture identity. No raw rows are overwritten.

See BUILD_V15.md for build/test evidence. Physical school-circuit behaviour
and stationary recovery still require testing. Start with5..10s of straight
tracing, stop and collect p before a long loop overwrites it; then try one A
and collect j immediately after passing it. Older manifests remain available.
