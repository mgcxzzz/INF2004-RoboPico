# Step 2 v0.1.2 — angle-only junction experiment

**Experimental micro T-Kernel firmware, not a completed migration.** This image
is for one straight and separate left/right/U-turn tests. It has not yet been
physically validated. Keep v0.1.1 available as the comparison firmware.

## What changed

The reference turn requires both normalized front levels below200, after10deg,
continuously for30ms before an outgoing branch can be accepted. Your stationary
capture measured actual white around143/177, but the imported calibration is
white122/138, black246/267. The right sensor therefore reports roughly294/1000
on white and cannot qualify. Acquisition remained healthy.

This experiment removes **only the prior-white prerequisite**:

- Greater than10deg continuously for30ms arms branch acceptance; white is not
  required. This is not evidence that the robot has physically cleared a line.
- Rolling left/right still need at least one normalized front level >=400,
  continuously for30ms, at75deg or more and below110deg.
- U-turn still uses its two-stage manoeuvre and second-stage branch confirmation
  from165deg;195deg remains a failure limit. The two settles remain.
- The outgoing-line acquisition stage still has to succeed. Branch acceptance
  is not the same as completing the junction and resuming ordinary following.
- Encoder geometry/drift bounds, turn/acquisition timeouts, normal and emergency
  stops, USB-loss stop, motor power and steering are unchanged.

The build sets `JUNCTION_ANGLE_GATE_ONLY=1` only for `APP=rtos_line`. The two
shared turn headers default to0, preserving the white prerequisite in reference
and SDK builds. No buddy bypasses the motion API/navigation coordinator.

## 1. Download, extract and flash

1. Download `robopico_rtos_line_following_v0_1_2_bundle.zip` and **extract it**.
   Use this bundle's updated exporter; it understands the new junction fields.
2. Stop the robot and disconnect motor/battery power. Close the serial console.
3. Hold BOOTSEL while reconnecting Pico USB, release it, then copy
   `firmware/robopico_rtos_line_following_v0_1_2.uf2` to RPI-RP2.
4. Let the Pico reboot. Open the serial console at115200, flow controlNone.
   Check that the banner says **v0.1.2 / ANGLE-ONLY JUNCTION EXPERIMENT**.
5. Keep the current sensor height/tilt and mat unchanged. Press **p** and save
   the output. Check `CLOCK ... status=PASS`, `LINE flags=0x3`, released buttons,
   and healthy `ADC` with no genuine timeout/error. Ready CONTROL flags are0xc,
   USB state3. Readiness needs three good clock windows after boot.
6. Confirm `CAL_FRONT white=122,138 black=246,267 valid=1` on this tested car.
   Its origin can be1(SDK) or2(RTOS saved). **Do not press w/l/r/k/c for this
   comparison.** If the values differ, send the p output before driving.
7. If a previous emergency latch is present, release both buttons, press **x**,
   wait at least one second, and press **p** again. Clearing a latch does not
   bypass an active ADC/clock fault. If g is rejected, send the exact rejection
   plus p output rather than repeatedly retrying.
8. Reconnect motor/battery power with the robot stopped. Keep USB and the serial
   console connected during each run; disconnecting/closing the console stops
   this checkpoint. Leave enough loose cable to avoid pulling the car.

The calibration loading order is unchanged: CRC-valid own RTOS saved page,
CRC-valid SDK record28, original fallback preset. The UF2 does not force a new
calibration or erase saved settings. A calibration held only in RAM would not
survive reboot; this particular captured calibration was imported from SDK.

No broker, dashboard, Wi-Fi setup or barcode mapping is used in this Step2 image.
The previous ADC fix is retained; repeating the70-second idle test is unnecessary
when the above readiness checks pass and the mounting has not changed.

## 2. Short straight comparison

1. Centre the car on a plain straight with enough room ahead.
2. Press lowercase **g**. A2-second countdown precedes motion. Profile1 is fixed;
   do not use Step1's1/2/3 wheel commands.
3. Let it follow for around3–5seconds, then press lowercase **s**.
4. Confirm it stops. Export this run using section4 before another g.
5. If its straight behaviour has substantially changed, send that log/video
   before proceeding with junctions.

## 3. Separate junction runs

Remove barcode sheets for these tests: the GP27 channel currently identifies a
bare cross, not a Code39 letter. Use an actual cross with the intended outgoing
branch and at least20–30cm of unobstructed outgoing track. The previous left run
was stopped early for limited space; that cannot establish turn completion.

For **each** of left, right and U-turn:

1. Stop and place the car centred on the approach straight. Put the side GP27
   sensor approximately15–20cm before the cross, so there is time to queue the
   route and qualify a straight approach. Do not begin with it already on black.
2. Start the video, then press lowercase **g**.
3. As soon as the car starts moving, queue **one capital letter**:

   | Desired next turn | Key | Route shown in logs |
   |---|---|---|
   | Left | **L** (Shift+l) | A / ASCII65 |
   | Right | **R** (Shift+r) | F / ASCII70 |
   | U-turn | **U** (Shift+u) | Z / ASCII90 |

   Check `USB route ... -> Buddy3 -> NAV result=0; await separate junction.`
   Lowercase l/r are calibration commands. Do not send another route mid-turn.
4. Let the car reach the cross and turn. For U-turn, the middle stop/settle and
   direction change are intentional; they are not automatically a failure.
5. After it finds the outgoing line and follows for roughly1–2seconds, press
   **s**. If it leaves the track or reaches an unexpected stop, stop immediately
   and export that failed attempt too. Do not reposition it while powered/moving.
6. **Export before another g or a power cycle.** Label the files/video LEFT,
   RIGHT or UTURN. Then repeat from a fresh approach for the next direction.

First send one attempt per direction, whether successful or not. If all three
succeed, repeat each twice to check repeatability. Sharp bends remain a separate
test; this image does not retune their controller.

## 4. Export after every attempt

The stopped RAM ring keeps the final1536samples, about15.36seconds. A new g
replaces it; reboot or power loss clears it. An `overwritten` count means earlier
samples fell out of the ring; retain the files anyway. Stop shortly after the
outcome so the turn stays in the recording.

**Windows**

1. Press s, keep Pico USB powered, then close PuTTY.
2. In the extracted **new** bundle, double-click **Export line log.cmd**.
3. Enter the COM port, e.g.COM4. If needed, install the dependency once:
   `py -3 -m pip install pyserial`.
4. Wait for `LINE_LOG_END` and the summary. The helper sends **d only**; it does
   not start, reset or recalibrate the robot.
5. Find the three timestamped files in `rtos_line_session`.

**Mac**

Close the serial console, keep USB powered, and run these from the extracted
bundle directory. Install pyserial once if missing. List ports if its name has
changed; substitute the actual Pico port in the export command.

```bash
python3 -m pip install pyserial
python3 -m serial.tools.list_ports
python3 tools/rtos_migration/export_line_log.py --port /dev/cu.usbmodemMTK3PICO1
```

For either OS, send all three files for **each attempt**:

- `<timestamp>_line.csv`
- `<timestamp>_summary.json`
- `<timestamp>_usb.txt`

Also send the initial p output, test direction and whether you stopped it
yourself or it stopped automatically. Zip one folder per test if convenient.
`COMPLETE` describes export integrity, not physical turn success. An empty or
incomplete export is still useful alongside the console error; do not discard it.

## 5. Video and the new diagnostics

Video is **strongly recommended** for the junction runs. Logs show what the
controller accepted; video shows whether that was the correct physical branch.
Film from above or diagonally above, showing the whole junction, approach,
sensor/front end and outgoing path. Start before g and continue until the stop.
If there is only time for one video, prioritise a failed turn and identify the
matching exported log. A straight video is optional unless its behaviour changes.

Version2 CSV adds:

| Field | Meaning |
|---|---|
| `turn_degrees_x10` | Encoder-estimated turn angle, divide by10 for degrees. |
| `junction_both_white` | Both normalized front levels currently below200; observation only. |
| `junction_prior_white` | Original >10deg + both-white30ms condition has qualified during this turn. |
| `junction_departure_ready` | Departure gate has armed (angle/dwell in this experiment); does not prove white was crossed. |
| `junction_branch_accepted` | A branch has been accepted in this manoeuvre; latched until next turn/run. |
| `junction_accepted_without_white` | That acceptance occurred without original white qualification; latched. |
| `junction_accept_event` | One control sample marking acceptance; for U-turn this is stage2 ->3, before final settle. |

The existing `white` column is ordinary follower recovery state, **not** the
junction's prior-white qualification. `JUNCTION policy=angle_only ...` is also
copied in p/automatic status. Summary JSON lists retained acceptance events.
We will compare acceptance angle, front readings, phase4 -> acquisition9 ->
ordinary-follow10 (with U-turn settles), speed/PWM and stop reasons against video.

## Software verification and remaining uncertainty

Production-code tests cover both policies, departure and branch dwell, reset of
interrupted qualification, L/R mirroring, U-turn stages, angle/time failure stops,
diagnostic flags/packing and unchanged ordinary tracker/calibration/power paths.
The exporter reads both old version1 and new version2 logs. The row remains
56bytes, using previously spare bits for six flags and signed13-bit angle x10.

Removing prior white may accept black from the same intersection instead of the
intended outgoing branch. The retained angle/optical/acquisition checks reduce
that possibility but cannot establish correctness without the physical tests.
