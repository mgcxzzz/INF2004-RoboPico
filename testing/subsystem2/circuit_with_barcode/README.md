# Circuit with barcode V9: smoother ordinary line following

Use firmware/buddy23_circuit_with_barcode_v9.uf2 on the same Pico W.
V9 adds a visible-line steering adapter in smooth_follow.h. It applies on
ordinary C_FOLLOW travel, with or without a barcode, in all three profiles.

- Small corrections with both levels at least300 blend from65% to100% of
  the previous differential by350/1000 error. The mean target is preserved.
- A60ms target filter and200mm/s^2 slew soften ordinary corrections. They
  progressively shorten to20ms/increase to600mm/s^2 as error approaches700.
- The weak/normal400-level target discontinuity is also smoothed. Raw levels
  still control the original loss thresholds and recovery memory.
- Error at least700, a near-pivot demand with a strong visible line, stops,
  blind recovery/search, and every busy junction phase bypass the adapter.
  Old targets pass through immediately and smoothing history resets.
- On leaving faster90/135 cruising, correction targets are immediately
  restricted to the original50mm/s ceiling, then continue smoothing.
- Wheel PI, PWM limits, breakaway pulses, A0 sampling, decoder, pending command,
  junction geometry/turns, search and calibration requirements are unchanged.
  The adapter does not add heading lock or adaptive barcode thresholds.

## First V9 test

1. Keep the current mounting, wiring and front calibration. Flash V9, press h
   to verify its banner and valid=1, then select profile1.
2. Run on a plain straight first, starting with both front spots on the line.
   Start with g over USB or the usual power-bank gesture. Stop manually after
   enough travel to compare visible wobble; wait5s before changing power.
3. Send j and describe whether the small corrections look smoother. p remains
   optional diagnostic detail; actual target columns include V9 smoothing.
4. If the straight improves, repeat the same small A scan several times, then
   test a bend in each direction. Physical smoothness/barcode success still
   needs these tests; host fixtures cannot establish wheel traction or drag.

Versions1-9 of the existing64840-byte motion record load; unknown10 rejects.
V8 raw capture and commands remain compatible. Older UF2s are retained.
See BUILD_V9.md for executed validation. The sections below document the
unchanged V8 barcode reader and capture used by V9.

## Why this trial

The user's stationary samples on25Oct2026 were:

| Offset sensing position | Raw ADC samples | Mean |
|---|---|---|
| Broad white |182,179,182,181,180|180.8|
| Narrow black |221,220,215,218,219|218.6|
| Neighbouring narrow white gap |204,206,203,205,206|204.8|

The nearest observed black/gap readings differ by9 counts. V7's original
threshold spacing128 and hysteresis+/-32 cannot distinguish that pair. Five
stationary readings do not establish moving performance over the full frame.

V8 keeps all31 coarse thresholds128,256,...3968 at their original+/-32
hysteresis. It adds32 fine thresholds192,194,...254 with+/-3 hysteresis,
63 sorted thresholds in total. The2ms edge confirmation remains. The initial
212-ADC preview should distinguish the supplied stationary bar/gap pair;
it is a diagnostic preview, not permission to queue a turn.

Every reader still requires the complete29-element Code39 *X* frame, all
narrow bars/gaps, correct sentinels/character and width checks. Two adjacent
thresholds must agree before accepting a frame. A queues left,F queues right;
other valid characters are reported as unsupported. No missing bars are
inferred or forced into A. Fine sensitivity can also expose more noise.

## Wiring and calibration

- Front-left A0: GP26/ADC0. Front-right A0: GP28/ADC2.
- Offset barcode A0: GP27/ADC1, physical Pico W pin32. D0/GP19 unused.
- Existing3.3V supply and common GND remain.
- Keep the now-working front mounting and saved calibration. h should report
  valid=1. No manual barcode calibration or potentiometer adjustment is needed.
- Front-only commands remain w=both white,l=left black,r=right black,k=save.
  Do not recapture those just to change barcode sensitivity.

## One short self-driving test

1. Flash V9. Open USB serial115200/8N1/no flow control. Press h and verify
   BUDDY23 CIRCUIT WITH BARCODE v9 and front valid=1. Saved V7 calibration
   imports. Select1; k saves the profile.
2. Put the front sensing points on the straight. Offset sensing point starts
   on blank paper roughly20mm before the first barcode bar, with its path
   passing through the middle of every bar. Start close: capture retains
   only the FIRST4096 samples, not an unlimited whole-course waveform.
3. Press g for a2s hands-free USB countdown. The robot drives itself using
   the V9 smoothed line follower. For power-bank mode, retain the usual boot
   wait5s,both buttons3s,release,wait1s,tap GP21,2s countdown gesture.
4. Stop after the offset sensor clears the final bar onto blank paper. Use s
   or either board button. Wait5s for settling and both flash saves.
5. With PuTTY session logging enabled, press j for decoded events, then a for
   the full-rate raw capture. Wait through RAW_ADC_END; up to4096 rows take
   about21 seconds to print. p still prints ordinary motion+events if needed.
6. Send the whole saved file. Identify A/F, print length and scan direction.

The robot still executes a queued A/F at a qualified later junction if allowed
forward. For this first sensitivity test, stop just beyond the barcode before
continuing into the junction. Side-ahead geometry is still the old92mm; a moved
sensor needs a new axle-to-sensing-point measurement before turn positioning
can be assessed. Search and manual-stop behavior remain as in V7.

## Raw capture and dump integrity

A0 is sampled nominally every2ms by the existing single main-loop ADC owner.
The first4096 samples are saved with actual microsecond times and encoder
mean-wheel progress at0.01mm resolution. This is roughly8s at500Hz; real gaps
can extend it. full=1 means the run outlasted the capture capacity. No captured
rows are overwritten. The new run replaces the previous raw capture.

The a command prints RAW_ADC_BEGIN metadata, sample/us/path_mm/adc CSV rows,
and RAW_ADC_END with count and data_crc. CRC covers used little-endian12-byte
records(us,uint32;path100,int32;adc,uint16;reserved0,uint16). a is blocked
while driving/settling, and cancels a pending countdown before dumping.

Raw records are CRC/readback verified and stored at0x101b0000..0x101bffff.
This is separate from probe calibration at0x101c0000/0x101c1000 and the usual
motion record at0x101d0000. a after reboot restores the capture. matches_motion
checks duration,ticks,profile and stop reason against the motion log. A raw
save failure still leaves a RAM copy that a can print before power removal.

Run analyze_raw_capture.py against a PuTTY file to validate row order/count
and CRC before drawing conclusions from a partial upload. The user does not
need Python to test; sending the complete file is sufficient.

## Existing logs and behavior

j/p motion and events remain. Main record layout stays64840 bytes; versions1-9
load and unknown10 rejects. V4-V7 saved bank indices retain their original
threshold mapping; V8/V9 use the new63-bank map. Event candidate/select flags
are bank1..63; DECODE bit7 marks analogue,bit0 reverse,bits1..6 bank0..62.

The original complete Code39 reader, scan/pending base45mm/s, A/F mapping,
junction stripe11..30mm plus4mm white, rolling turns and V7 line-loss search
are retained. Normal profiles request45/90/135 on stable unmarked straights.
The removed650 startup placement gate stays removed; valid front calibration
and explicit start/buttons/readiness still apply. Old V1-V7 UF2 are retained.

See BUILD_V8.md for validation and memory evidence. The small A has decoded in physical V8 runs, but repeatability is not yet
established. V9 tests whether smoother movement improves that repeatability.
