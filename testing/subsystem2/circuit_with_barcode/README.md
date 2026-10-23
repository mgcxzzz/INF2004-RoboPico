# Circuit with barcode V1

Standalone Buddy 2/3 Pico W / Robo Pico test, based on **Circuit Left V6**.
It adds actual single-character Code 39 decoding and queued junction commands
in this separate folder. The existing circuit, junction and hump tests are
unchanged. Motors remain off at boot.

## Firmware and behaviour

Download `firmware/buddy23_circuit_with_barcode_v1.uf2` and flash using BOOTSEL
and the RPI-RP2 drive. Open USB serial at 115200, 8N1, no flow control.
Press `h` and check **BUDDY23 CIRCUIT WITH BARCODE v1**.

| Decoded complete frame | Stored command | At the next qualified cross |
|---|---|---|
| `*A*` | A | Rolling left turn |
| `*F*` | F | Rolling right turn |
| Other valid character | Logged as unsupported | No new command |
| Invalid/incomplete frame | Logged as rejected | No new command |

Both supplied print sizes are supported using **relative narrow/wide element
widths measured with the wheel encoders**. The decoder does not identify a
letter from overall barcode length or assume constant travel speed. Each of
the three nine-element characters must have a valid narrow/wide pattern;
start/stop `*` characters and the two narrow inter-character gaps are checked.
Both scan directions work. This version supports one payload character with
no optional checksum, matching the supplied sample frames.

The pending command survives any travel distance between the barcode and
junction, and is consumed once when a cross is accepted. A newer valid A/F
replaces the previous pending command; unsupported characters preserve it.
Pending commands reset on boot or a new run. There is no automatic left-turn
fallback: **a cross with no pending command is traversed straight**.

The supplied sample PDF contains A and Z sheets at both sizes. Its measured
A/Z patterns were used as decoder fixtures. Z is deliberately unsupported for
routing: the actual test circuit is A/F as confirmed by the user. The standard
F pattern is decoded as F, not relabelled from Z. Encoding values were checked
against [ZXing's Code39Reader table](https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java).

## Junction detection

The offset D0 sensor measures the completed black run at its trailing edge.
A separate cross-arm has a nominal ink width of 18–20 mm. The accepted
**effective encoder-measured** width is 11–30 mm, allowing for the earlier
18 mm cross tests measuring approximately 12–16 mm. It must also be wider
than the learned barcode wide element. Barcode frame context prevents its
individual bars from dispatching turns.

There must be a detectable white separation between the final barcode bar
and the cross-arm. No fixed separation distance is assumed. A physically
merged bar and cross cannot reliably be separated by this single D0 sensor;
an ambiguous/incomplete frame is rejected rather than assigned a direction.

Crosses must be approached on a straight. Qualification retains the V6
300 ms / 3 degree encoder-yaw gate, visible front line and both wheels moving
at least 10 mm/s. After a completed turn/alignment, another cross is inhibited
for 150 mm. A rejected cross is logged and retains the pending command.

Geometry is unchanged: front sensors about 127 mm ahead of the rear drive
axle, offset sensor 35 mm behind them, therefore **92 mm ahead of the axle**.
The cross midpoint is estimated from its two edges. The rolling turn starts
55 mm before the axle would reach that midpoint.

- Left: left wheel unpowered, right wheel forward.
- Right: right wheel unpowered, left wheel forward. It mirrors the same
  turn and branch-acquisition logic, then restores normal follower state.
- The outgoing branch must be detected within the existing angle window;
  acquisition uses the V6 follower and resumes continuous following.

There is no whole-run/lap stop. Manual stop, USB change, invalid/direction
feedback, control lateness, bounded line recovery and junction checks remain.
Encoder no-progress feedback remains **warning only**, as in V6. Barcode edge
queue overflow stops the run because its captured pattern is incomplete.

## Wiring, calibration and speed

No wiring changes:

- Front LEFT A0: GP26/ADC0; front RIGHT A0: GP28/ADC2.
- Offset barcode/junction **D0: GP19**, 3.3 V and common GND.
- GP20 STOP; GP21 start/hold. Existing motor/encoder pins are unchanged.

Keep the offset sensor over the bars throughout the straight approach. The
specification's 20 mm white gap is **sideways** between barcode and line; it
is not a longitudinal junction offset. Line width is specified as 18 mm.

The first boot imports front/side calibration from a valid Circuit Left
V1–V6 record, otherwise from the previous junction record. A saved barcode
record takes precedence afterward. Select `1`, then `k`, for the first test.
Check the `CAL` line for `valid=1`, `side_valid=1`, `profile=1`.

If calibration is needed, stop and release the buttons, then:

1. Both front sensors on white: wait one second, press `w`.
2. Left sensor on black at normal height: wait one second, press `l`.
3. Right sensor on black: wait one second, press `r`.
4. Offset sensor on white: press `o`; on black: press `b`.
5. Press `1`, then `k`. Do not calibrate using the barcode gaps themselves.

| Profile | Unmarked stable centred cruise | Scanning / pending-command approach | Rolling turn early / late |
|---|---:|---:|---:|
| 1 | 45 mm/s | Usual profile 1 follower | 45 / 30 mm/s |
| 2 | 90 mm/s | Usual profile 1 follower | 55 / 35 mm/s |
| 3 | 135 mm/s | Usual profile 1 follower | 65 / 40 mm/s |

Normal corrections, faint tape handling and recovery use the V6 settings.
Slower scanning improves the first hardware test; barcode widths still use
encoder distance when speed varies. Correction PWM cap is 30%, with the
existing bounded load assistance/turn cap of 42% and turn start up to 48%
for at most 150 ms. This version adds no higher effort limit.

## First physical tests

Use **profile 1** and test the two commands separately before a full circuit:

1. Start centred on a straight **before** the entire barcode, with offset D0
   on white. At least one front normalized level must be 650 or higher.
2. Drive over A. Confirm it continues past the barcode and turns left at the
   separate cross. Stop, wait five seconds, reconnect USB and press `j`.
3. Repeat with F; confirm the right turn and outgoing branch following.
4. Repeat with the other print size and opposite scan direction as available.
5. Only then allow repeated circuits. Unmarked crosses go straight, so the
   placed A/F instructions determine the route rather than the old all-left
   route. Try profiles 2/3 after profile 1 works.

Expected event order is `START`, `DECODE` (A/L or F/R), `CROSS`, `DISPATCH`,
`COMPLETE`. This is the expected sequence, not a recorded hardware result.
Send `j` output first, including `STOP` and `SUMMARY`. If decoding fails,
the measured `RUN` widths and `BAD_FRAME` patterns help diagnose it; `p`
provides the full wheel/front-sensor log as well.

### Starting and stopping

- USB: `g` gives a hands-free two-second countdown. Alternatively `f` runs
  while GP21 is held; releasing it stops. `s` or either button stops.
- Power bank: wait five seconds; hold both buttons for three seconds; release,
  wait one second; tap/release GP21; the run begins after two seconds.
- Stop with either button. **Wait five seconds before changing power** so the
  stopped run can settle and save. A USB connection change also stops.
- `h`: help/calibration; `j`: compact barcode/events + summary; `p`: full log.
- `1`/`2`/`3`: select profile while stopped; `k`: save calibration/profile.

## Saved diagnostics

D0 is sampled every 1 ms and must be stable for 2 ms. The first candidate
timestamp and encoder counts are retained for each confirmed edge; decoding
and floating-point work run outside the interrupt. A 256-entry edge queue
has 255 usable entries and reports overflow explicitly.

The stopped run saves CRC-verified latest 1,000 motion rows, 192 barcode
events and 64 turn summaries. Whole-run totals include elapsed time, encoder
distance, average speed, left/right completions, A/F decodes, unsupported and
rejected frames, ignored crosses and no-command straight crosses. Circular
logs may overwrite earlier details on long runs; totals remain. Motion rows
add pending, last decoded character, action and scan state. Turn summary
rotation is positive left and negative right.

This test uses its own 64 KiB flash bank at `0x101d0000..0x101e0000`.
Circuit Left's `0x101e0000..0x101f0000` bank and the older tail records are
preserved. Flashing with a separate full-chip erase would erase those records.

## Verification and limitations

See `BUILD_V1.md` for host tests, Pico build and UF2 hash. The software tests
cover measured PDF fixtures, A/F at both sizes/directions, malformed frames,
variable travel speed, pending commands and alternating left/right turns.
**Physical barcode scanning and the mirrored right turn still require lab
validation.** Encoder distance can be distorted by wheel slip; D0 contrast,
sensor alignment and print quality must be checked on the actual sheets.

This folder reuses shared headers from adjacent standalone tests, so build it
inside this repository. It does not integrate barcode handling into the
production application or enable IMU/hump navigation.
