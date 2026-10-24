# Circuit with barcode V7: alternating rolling line search

Driving test: existing Circuit Left V6 follower and mirrored junction turns,
now reading offset A0 on GP27. **No manual barcode white/black calibration.**
Front line sensors still require their existing calibration, imported from the
saved barcode/circuit record when available. Motors stay off until started.

## V7 line-loss search

Normal steering, faint-line corrections, front calibration, speed profiles and
barcode decoding retain V6 behavior. A target-private CIRCUIT_SWEEP_SEARCH
option changes only lost-line recovery and gives its moving wheel loaded
motor effort. Other test targets keep the original recovery implementation.

After both fronts fall below150/1000, preserve the old brief bridge for150ms.
With no direction history, wait with outputs off during this confirmation.
If the loss persists, search toward the remembered side first; without usable
history, start left. Alternate directions over five durations:
350,700,1050,1400,1750ms (5.25s total search). Each sweep coasts one wheel and
requests45mm/s on the other; reduce that request to30 during line confirmation.
These are rolling arcs, not stationary rotations or guaranteed measured angles.
They can move the chassis forward as well as turn it. Search also stops at
150mm of additional encoder path; time expires even with no encoder ticks.

Use the existing confirmation rules: a return>=400 held40ms, >=300 held80ms,
or consistent faint asymmetric return held120ms restores ordinary following.
An isolated flash does not restart the search clock. Both sensors need not
be black; this preserves following a bend immediately after a junction.
If all sweeps fail, stop with the specific search timeout/distance reason.

Only the moving search wheel uses the loaded42% PWM allowance. Its controller
still regulates speed; this is a cap, not forced constant PWM. Search does
not receive a48% start pulse. Coast-wheel output remains zero. Manual/button,
USB, control-loop, invalid-encoder/direction, calibration and turn checks remain.
Encoder progress remains warning-only. While lost, junction dispatch is blocked
and a queued A/F instruction is retained. Barcode decoding itself is unchanged.

Logs keep their layout and now save version7; versions1-6 still load. Motion
decision19..23 identifies sweep1..5, with targetL/targetR showing direction.
Assist5 means loaded left search wheel;6 means loaded right search wheel.
Normal motion rows, summary, offline save and p/j commands remain unchanged.

Use firmware/buddy23_circuit_with_barcode_v7.uf2. After lowering the front
sensors, recapture w/l/r at the normal resting height, then k. Select profile1
and test a straight first. To exercise recovery, test a short gap in the line
on an open surface; it should alternate if the line stays out of view, and
resume once the line remains visible. Poor calibration is not fixed by search.

## V6 startup change (retained)

V6 removes only the front normalized-black>=650 requirement that produced
START reason11. It does not change the line follower or copy a different
robot's steering, sensor mapping, junction detector or turn geometry.

Starting still requires valid front calibration (both spans>=50 and all three
captures), inactive/settled state, ready=1 after one second stationary with
buttons released, GP20 released, and the correct GP21 state. USB g starts a
2s countdown with both buttons released; f requires GP21 held. Power-bank
arming remains unchanged. h identifies V6; rejection diagnostics now print
front_valid, front levels and optical_start_gate=OFF. Barcode calibration
is independent and is not required for startup.

Place the front sensing spots on the line before requesting a run. Removing
this gate does not give an all-white start a direction to search: existing
running line-loss checks still apply, and all-white without direction memory
can stop immediately. Calibration must match the normal resting height;
loosening a start gate cannot correct unrepresentative black references.

V6 firmware remains archived; use the V7 artifact above for the new search.
Saved records1-7 are supported with the same layout. Speed profiles, PWM,
analogue barcode decoding and the old92mm side-ahead geometry are unchanged.
The replacement offset-to-axle measurement is still needed before judging
junction positioning with the repositioned module.

## Raised-height detection and the supplied line-following document

The groupmate normalizes (raw-white)/(black-white) to0..1. This firmware
already uses the equivalent0..1000 scale:0/0.5/1 maps to0/500/1000. Changing
the display scale cannot restore contrast lost at a new mounting height.
Use measured references from this robot in its normal resting posture; do not
copy the other robot's default200/2400 references. V6 does not adopt the
groupmate's P steering, speed schedule, edge placement, automatic spin
calibration, junction classifier or recovery sweeps. Existing bend-following
logic is retained at the user's explicit request.

## V5 raised-front-sensor trial

The user's new front mounting produced references white138/152,black199/229
(spans61/77). Held black samples were left197-200/right230-232; held white
samples left136-139/right153-156. V4 rejected spans below100 even though these
reported ranges do not overlap. V5 permits spans of at least50 counts through
**LINE_CAL_MIN_SPAN=50 on this executable only**. Shared default remains100;
other existing tests keep their requirement. No raw reference values are
hardcoded. All gains, normalized thresholds, speeds/PWM, ADC averaging, A0
barcode search and junction geometry remain unchanged.

After flashing, **recapture w/l/r at the final mounted height, then k**. The
rejected RAM calibration in V4 was never saved, so an older restored front
record may still describe the previous height. h now prints min_span and both
measured spans. Check min_span=50,valid=1 and CALIBRATION/PROFILE SAVED.
Test a plain straight first in profile1, then stop and inspect p. Static
separation is encouraging but does not establish stability while moving.

**Junction side-ahead geometry is still92mm.** The offset mounting was moved;
the new rear-axle-to-offset sensing-point forward distance is still needed.
Do not treat this change as a junction geometry update. V5 saved records use
version5 with the same V4 motion layout; V1-V4 records remain readable.

## Firmware and wiring

Use `firmware/buddy23_circuit_with_barcode_v7.uf2` (driving firmware), not the
separate motors-off analogue probe. V1-V6 UF2 remain unchanged. Historical
digital V3 instructions are archived in README_V3.md.

| Sensor connection | Pin |
|---|---|
| Offset barcode A0 | GP27 / ADC1, Pico W physical pin32 |
| Offset VCC/GND | Existing3.3V/common GND |
| Offset D0 / GP19 | Unused |
| Front-left A0 | Existing GP26 / ADC0, unchanged Grove wiring |
| Front-right A0 | Existing GP28 / ADC2, unchanged Grove wiring |

The confirmed module has higher A0 on broad black than broad white. This
version uses that polarity; it does not assume narrow/wide have four voltage
levels. No w/b/k operation or saved analogue-probe calibration is needed for
the barcode. A0 is sampled by the main-loop ADC owner nominally every2ms;
front-channel multiplexing discards the first conversion and is not accessed
from the timer ISR. Actual sampling gaps over10ms are diagnostic events.

## First physical test: small A, profile1

1. Flash V7 and open USB serial115200/8N1/no flow control. h must show
   **BUDDY23 CIRCUIT WITH BARCODE v7 | AUTO ANALOG GP27**.
2. Check `CAL FRONT ... valid=1`. Existing front calibration should import.
   **Do not press w/b to calibrate the barcode.** If the FRONT references are
   invalid, w/l/r capture front white/left-black/right-black, then k saves them.
3. Select **1**, then k to save profile. Fronts should be aligned on the line;
   offset sensing point should start on clear paper at least20mm before the
   first barcode bar, with a straight approach across the complete *A*.
4. USB: press g for the two-second hands-free countdown (f instead requires
   holding GP21). s or either board button stops. v toggles optional live status,
   default OFF to avoid flooding. Provide cable slack for the short test.
5. Alternatively, power-bank mode uses the same gesture: wait5s after boot,
   hold BOTH buttons3s, release both, wait1s, tap/release GP21, wait2s. Either
   button stops. A computer USB host disables this offline gesture.
6. It follows the line, searches the barcode signal automatically, queues
   **A -> LEFT / F -> RIGHT**, and executes at the next qualified separate cross.
   If decoding fails or is ambiguous it continues normal following without
   inventing a route command. Bounded line-loss/turn checks still apply.
7. Stop after the first barcode/junction attempt. WAIT5s before changing power.
   Offline logs save as usual. Reconnect USB, enable PuTTY session file logging,
   then j for events/summary, or p for full motion+events. Send the whole file
   through END BARCODE CIRCUIT LOG if using p.

## Automatic threshold search

31 independent Code39 readers use raw thresholds128,256,...3968 with32-count
hysteresis and2ms edge debounce. Edge timestamps and encoder positions retain
the first stable-candidate sample. All readers apply the original full *X*
framing, all29 run widths, independent black/white narrow/wide learning,
pattern lookup, start/stop and expiry checks. No narrow black bars are omitted,
no nearest-letter substitution is performed, and A is not inferred from total
print length. Forward/reverse and both supplied scales remain supported in
software, using distance rather than a fixed travel time.

A route is accepted only when two adjacent raw thresholds agree on code,
orientation and frame start/end (within6mm and500ms). Among agreeing pairs,
the candidate nearest the observed signal midpoint is selected. Repeated
threshold votes for the same physical frame do not queue duplicate commands;
contradicting frame votes clear pending. Unsupported valid characters are
logged without replacing a pending A/F. This is an experimental analogue
reader: threshold search cannot restore optical detail absent from A0, nor
prove the known narrow-gap inversion has been resolved on hardware.

## Junction and motion behaviour

Junction search begins only after a verified A/F command. The selecting stop
bar cannot be reused as the cross. The selected signal measures an11-30mm
separate stripe and confirms4mm white after its trailing edge. Existing
straight gate300ms/3degrees and post-turn150mm clearance remain. No minimum
fixed barcode-to-cross travel distance is assumed. Physically merged bars
and junctions remain ambiguous to one offset sensor.

Same geometry: side sensor92mm ahead of drive axle; rolling turn begins55mm
before cross centre. LEFT leaves left wheel unpowered/right forward; RIGHT
mirrors it. Existing branch acquisition and continuous follower handover remain.
Cruising profiles45/90/135mm/s; scanning and pending-command approach use
profile1. Motor PWM caps and correction thresholds are unchanged. Encoder
progress remains warning-only; manual/USB/control/direction/invalid-encoder
and bounded line/turn checks are retained.

## What to look for in j/p

- ANALOG_CANDIDATE: one threshold decoded a full frame, not accepted alone.
- ANALOG_SELECT then DECODE A/F: adjacent-threshold agreement queued a route.
- CROSS_CANDIDATE, CROSS, DISPATCH, COMPLETE: cross and turn stages.
- ANALOG_CONFLICT: contradictory full-frame votes; pending cleared.
- BAD_FRAME: selected reader's rejection; other thresholds may still succeed.
- ANALOG_SAMPLE_GAP: foreground ADC sampling gap >10ms.

For candidate/select events, `width_mm` carries threshold ADC counts;
`flags` is bank1..31. For SAMPLE_GAP, width is milliseconds. DECODE retains
frame span in millimetres; flags bit7=analogue, bit0=reverse, bits1..5=bank0..30.
V4/V5 motion rows replace the old four tail fields with side_ADC,threshold_ADC,
analog_black,bank. Front and encoder/motion columns remain. Log version1-3
retains its original field meaning when restored. The raw ADC snapshot is at
20ms plus state/event changes, not a500Hz waveform export.

p/j dumps are paced5ms/row and have a5s USB-output timeout, addressing the
missing-output issue seen in raw probe V4 logs. MOTION_DUMP_END reports count;
wait for final markers before closing the session. Motion/event rings retain
latest1000 rows/192 events; long runs can overwrite older detail while totals
remain. A first short A test provides the clearest evidence.

## Verification and storage

Existing record ABI remains64840 bytes in the same64KiB bank at0x101d0000.
Versions4/5 mark the new four-byte motion tail; V1-V4 load and print as legacy
records. The unused legacy side-calibration field is initialized to0/1 for
record compatibility, not used as an A0 threshold. Probe calibration slots
at0x101c0000/0x101c1000 and older circuit/tail banks are not written.

Executed tests include the original barcode/record/navigation suite plus raw
ADC fixtures for A/F, scales/directions/ratios/speed variation, diminished
narrow-bar amplitude, malformed/missing-bar rejection, noise, unsupported code,
conflicting votes, cross qualification and alternating turns in all profiles.
Build/UF2/ABI evidence is in BUILD_V5.md. Physical small-barcode success is not
claimed until the next run.
