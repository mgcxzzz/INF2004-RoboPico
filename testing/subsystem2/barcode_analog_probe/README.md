# Barcode analogue probe V5: saved calibration and raw distance sweep

Motors-off USB diagnostic. Barcode A0 -> GP27 / ADC1 (Pico W physical pin32),
3.3 V and common GND; D0/GP19 unused. Leave both front Grove connections and
existing encoder wiring unchanged. V5 measures the full analogue profile before
choosing a threshold. It does not decode, line-follow or execute junction turns.

## Why capture a full A?

The broad calibration passed (white469, black3943.5), but the labelled narrow
white gaps read3077/3081 while narrow black bars read2250/1966. This reverses the
broad ordering. A motion trace can reveal whether sensing footprint, alignment,
bar/gap widths or transient response accounts for this. Four separate ADC levels
are not assumed: narrow/wide describe spatial widths, not fixed voltage levels.
No reading is forced to A. Both missing gaps and missing bars remain measurable.

## Save calibration once

1. Flash `firmware/buddy23_barcode_analog_probe_v5.uf2`; open computer USB serial
   115200 / 8N1 / no flow control. Press h: verify V5 and GP27/ADC1, motors OFF.
2. At normal mounted height, broad white on the actual barcode paper: w, hold
   still for one second. On a wide black bar on that paper: b, hold still.
3. Check CAL `mask=3 valid=1`, then keep still and press **k**. Expect
   `CAL SAVED: CRC/readback verified` and `saved=1`.
4. After a power cycle, c should show `valid=1 saved=1 restored=1` and matching
   references. Recalibrate if height, lighting, paper or mounting changes.

V3 calibration was RAM-only and is lost when resetting to flash V4: recapture
once. V4/V5 do not hardcode earlier values or import the front sensor calibration.
Broad reference checks: >=500 samples/capture, >=100-count mean separation,
>=6x largest reference SD. Normalized white0 / black1000 / invalid-1, either
voltage polarity. g/n still capture narrow gap/bar without changing references.
A new w resets the black reference; press k only after a valid new w/b pair.

## Record the whole small A by hand, with wheels rolling

1. Keep USB connected and enable your terminal's file logging before the dump.
2. Put the offset sensing point on blank paper **20 mm before the first bar**.
   Align the robot so that a straight forward push takes that point through
   all bars. Keep all wheels on the surface at normal height; do not lift,
   rotate or slide the robot sideways during the scan.
3. Press **f**. Wait two seconds for **SWEEP_BEGIN**. Then roll the whole robot
   steadily forward by hand across A and onto blank paper about20 mm beyond it.
   Aim for roughly5-10 seconds for the pass; exact constant speed is not needed
   because encoder distance is logged. Avoid backwards motion and tyre slip.
4. Press **s** after the passage, or recording ends automatically after20 seconds.
5. Press **p** to print ANALOG_SWEEP and all samples. Send the entire output
   through END ANALOG_SWEEP, and identify the barcode as A plus its print length.
6. Repeat f/roll/s/p for a second pass if possible. Each new started sweep
   replaces the previous RAM sweep; save the output first.

**Motors never run in this firmware.** f starts recording, not driving. Encoder
inputs GP0/GP1 (left) and GP4/GP5 (right) measure the hand-rolled motion. Raw
quadrature signs are left negative/right positive for forward, matching the
working circuit firmware. Conversion uses the same provisional calibration:
200 mm circumference, left2524.05/right2528.30 counts per revolution. Wheel
slip and angle make these estimates inaccurate: roll straight with wheels turning.

## V5 dump transport fix

The complete user PuTTY file contains three V4 dump headers but only1069/9690,
1369/8241 and1347/7321 complete rows. One missing stretch jumps2.73->16.91s,
even though its recorder reports no missed sampling slots. V4 had a10ms USB
stdout timeout; the SDK can discard text on backpressure, consistent with the
joined/truncated rows. V5 raises that timeout to5s, flushes and paces each row
by5ms, numbers rows0..N-1 and appends SWEEP_DUMP_END with row count and CRC32
of numeric CSV rows (LF line endings). Sampling is unchanged; pacing happens
only during the later dump. This mitigates data loss; the checksum detects any
remaining incomplete/corrupted transfer. Wait up to about55s at the maximum
row count, and until both end markers. Use Session Logging to a fresh file.

Run `python analyze_sweep_log.py putty.txt` to audit mixed logs. V4 counts can
be audited but have no transport checksum. V5 is checksum_verified only with
the expected row count, contiguous indexes, increasing times, end marker and
matching CRC. Run `python test_sweep_log.py` for6 transport-audit tests.

## Data and controls

500 Hz nominal acquisition (2 ms interval), up to10,000 rows/20 seconds. Raw
12-bit ADC values and signed encoder ticks are buffered in RAM. No live stream,
float formatting, flash save or barcode decode occurs during the sweep.
Actual timestamps, largest sampling interval, missed slots and invalid encoder
transitions are reported. The data can show collapsed modulation rather than
pretending a missing bar/gap was detected. Motion thresholds will be evaluated
against distance and full known *A* framing after the trace is collected.

| Key | Action |
|---|---|
| w / b | One-second white / black calibration captures |
| k | Save valid calibration; stationary and no active capture/sweep |
| c | Print calibration and saved/restored status |
| g / n | One-second narrow white gap / adjacent narrow black bar capture |
| f | Arm2seconds, then raw hand-rolled sweep; motors remain OFF |
| s | Stop sweep / cancel arming |
| p | Dump last completed sweep (not a stationary capture) |
| q | Reprint last stationary capture plus calibration |
| v | Toggle100ms LIVE windows; default OFF to reduce serial noise |
| r | Clear current RAM references only; saved flash copy remains |
| h | Help |

Calibration survives power-off after k. **Sweep samples remain RAM-only:** dump
with p before power-off. Reopening serial without power-off preserves the sweep.
USB disconnect stops recording; if power stays on, recorded RAM data remains.
No offline powered run exists. Current circuit V3 still expects D0 on GP19:
power off and restore both signal ends before using that firmware.

## Storage and isolation

GP8..11 held LOW before USB and continuously. No PWM setup or motor-start path.
ADC1 belongs solely to this loop. Encoder ISRs only update quadrature counters.
The persistent record is specific to GP27/ADC1, versioned and CRC32 checked.
Two alternating4KiB sectors at flash offsets0x1c0000 and0x1c1000 (Pico W2MiB)
retain the previous valid slot during an update. k erases the alternate sector,
programs one256-byte page and verifies CRC and byte-for-byte readback. Latest
valid sequence loads at boot; invalid records are ignored. Existing barcode
bank0x1d0000, circuit bank0x1e0000 and legacy tail records are not written.
Binary-end check rejects writes if the firmware overlaps these probe slots.
A future full-chip erase or other firmware using these slots can remove records.

V1-V4 UF2 remain unchanged. See BUILD_V5.md for build/UF2 evidence. V5 has
not been physically tested; verify calibration restore and hand-scan readings.
