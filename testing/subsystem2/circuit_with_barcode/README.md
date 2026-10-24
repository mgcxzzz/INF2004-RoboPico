# Circuit with barcode V11: A left, F right, Z U-turn

Use firmware/buddy23_circuit_with_barcode_v11.uf2 on the same Pico W.
For the Z trial select profile1, retaining the V9 smoothing and motor settings
that successfully read A and F. Existing profile2 ramp experiment and
profile3 remain available; ramp work is paused and not needed for this test.

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

## How the U-turn works

The first half reuses rolling approach geometry: the front-to-axle/offset
assumptions remain as previously measured, including side-ahead92mm and a
55mm early rolling turn start. A Z waits for a separate11..30mm effective
junction stripe and4mm white after its trailing edge. It never turns directly
at the barcode or after an assumed barcode-to-junction distance.

1. Right wheel forward, left OFF, to approximately90 degrees of encoder yaw.
   The crossing branch near90 degrees is deliberately ignored.
2. Both outputs OFF; wait until measured speeds are below5mm/s for200ms.
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

U-turn targets are right45 then30mm/s on the first quarter and left-35 then
-25mm/s on the reverse quarter, independent of faster profile settings.
Existing turn power remains42% sustained/48% bounded start; no ramp boost is
used during the junction. Signed output is enabled only for Z turning.

Stationary-wheel drift is limited to12mm, active-wheel travel to220mm per
quarter, each quarter to8s and the turning/settling sequence to18s. Settle
phases allow2s. Missing return line by195 degrees faults. The existing6s
outgoing alignment window and manual/USB/control/direction/invalid checks
remain. Physical slip/drag can still require tuning; no IMU is assumed.

## Test with profile1

1. Keep the mounting, wiring and saved front calibration. Flash V11, press h
   and verify the V11 banner and valid=1. Select1, then k to retain that profile.
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
2ms confirmation and black-high polarity. Smoothing/speeds are unchanged from
V9 for profile1. Profile2 retains V10's180mm/s experimental ramp controller;
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
and trace56/turn-summary40 layouts remain byte-compatible; V11 is64848 bytes.
CRC length depends on version. V1..V10 calibration/logs still load; unknown12
rejects. Main bank remains0x101d0000. Raw record/address is unchanged at
0x101b0000, separate from probe calibration0x101c0000/0x101c1000.

Raw capture retains the first4096 nominal2ms A0 samples with actual timestamps,
signed mean-wheel progress and ADC. a prints a paced5ms/row CSV with
RAW_ADC_BEGIN/END counts and CRC. Wait for the footer and send a full PuTTY
logging file if raw analysis is needed. full=1 means later run samples were
omitted. matches_motion checks capture identity. No raw rows are overwritten.

See BUILD_V11.md for build/test evidence. V11 has software and kinematic test
coverage; physical Z U-turn success is not yet claimed. Historical V9/V10
behaviour and build manifests remain in BUILD_V9.md and BUILD_V10.md.
