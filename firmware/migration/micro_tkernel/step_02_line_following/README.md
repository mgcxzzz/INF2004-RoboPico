# Step 2 — RTOS line following v0.1.1 (experimental)

**Current update:** [ADC correction and exact first test](UPDATE_V0_1_1.md).
Start with the stopped 70-second ADC check before another driving attempt.
The old v0.1 image became ADC-unavailable during the user’s idle test.

This is the second **micro T-Kernel SMP** migration checkpoint, on
`codex/micro-tkernel-migration`. It is not the final Buddy1–5 build.
`main` and its SDK integration remain the behaviour reference.

The preceding Step 1 v0.1.1 hardware test passed: both clocks matched laptop
time, motor readiness was restored, both wheels ran, and the user confirmed
the serial/button emergency and normal stops. See
[Step 1 acceptance](../step_01_motor_bringup/HARDWARE_ACCEPTANCE.md).

## What runs here

- Front IR acquisition on GP26/GP28, the reference 32-conversion average and
  0.35 front filter, existing calibrated optical tracker and Continuous V2
  steering. **Profile 1 only**, nominal 45 mm/s; correction targets can be lower.
- Existing per-wheel ContinuousDrive for ordinary following; original loaded
  capture, rolling recovery and junction/U-turn power paths for special motion.
  The 1 ms safety task also expires committed peak-power deadlines.
- Existing line-loss confirmation, widening line search, junction geometry,
  mirrored right turn, two-stage U-turn and outgoing-line acquisition.
- Unchanged shared navigation API/coordinator and generation-checked motor
  commits. Buddy 3 provides targets through the API; no buddy writes PWM directly.
- Saved calibration import, optional new calibration and a stopped RAM run log.

Wi-Fi/MQTT/dashboard are **not running** until Step 3. Physical barcode decoding
and runtime A–Z mapping return in Step 4. IMU/ramp assistance returns in Step 5.
Buddy 5 remains unintegrated. Do not use this image as a ramp/obstacle test.

The USB cable and an open serial connection are required during motion in
this checkpoint. Closing PuTTY or losing USB DTR stops the car. The battery
must still supply the motors, as in the successful Step 1 test.

## Download and flash

Use `firmware/robopico_rtos_line_following_v0_1_1_bundle.zip` for the complete
source, UF2, export helper and these instructions. Extract the bundle first.
Its top-level `firmware` folder contains the UF2.

1. Stop the robot. Enter BOOTSEL and copy
   `robopico_rtos_line_following_v0_1_1.uf2` to RPI-RP2.
2. Open the Pico COM port in PuTTY at **115200**, with flow control **None**.
   Motors stay off at boot. Do not start a dashboard/broker for this test.
3. Wait for `CLOCK ... status=PASS` and a released-button, fault-free status.
   Press **p** for a fresh status. The normal ready flags are `0xc`, `usb=3`.
4. If an emergency latch is present, release GP20/GP21 and press **x**.
   Allow at least one second of rest. A current clock/ADC fault must be resolved;
   clearing a latch does not bypass those checks.
5. Read `CAL_FRONT` before changing calibration. Keep the current low/tilted
   sensor mounting and first try the imported calibration.

### Which calibration starts the firmware?

In priority order:

1. A CRC-valid calibration saved by this RTOS checkpoint (`origin=2`).
2. The car's CRC-valid **SDK record 28** (`origin=1`), including your teammate's
   recalibration if that is what was saved on this particular car.
3. Original successful Continuous V2 preset (`origin=0`):
   white **163,160**, black **3457,3700**.

It does not invent a new calibration or force the old preset over a valid saved
SDK calibration. The exact imported values are read from the car at boot;
we cannot know which values this car currently holds without its `p` output.
A normal UF2 flash does not erase those saved-data sectors.

Optional commands, while stopped with buttons released:

| Key | Action |
|---|---|
| `w` | Both sensors over white: capture both white references. |
| `l` | Left sensor over black: capture left black reference. |
| `r` | Right sensor over black: capture right black reference. |
| `k` | Save a valid calibration to the separate RTOS sector; wait for `VERIFIED`. |
| `c` | Restore the original preset **in RAM**. Use `k` too if it should survive reboot. |

Captures report `origin=3` until saved. After `k`, allow the clocks to return
to PASS before starting; flash programming pauses both cores while stopped.
SDK history/calibration and Wi-Fi settings are not overwritten. RTOS calibration
is at flash offset `0x1a9000`; code is constrained below `0x100000`.

## Hardware tests, in order

Keep enough loose USB cable for the short test area. Use the same mat/mounting
for comparisons. Every **g** starts a fresh run/log after a two-second countdown.
This is continuous following, not the Step 1 three-second wheel pulse.

1. **Straight only:** centre the car on a straight, press **g**, let it run a
   short distance, then press **s**. It should follow at the familiar profile 1
   pace without an unexpected stop. Export the log before the next **g**.
2. **One bend:** repeat for a gentle bend, then the sharp bend that previously
   lost the line. Stop/export immediately if it loses the line. Existing
   recovery may search; a missed bend is evidence to review, not a migration pass.
3. **Junction, when 1–2 are acceptable:** start on the approach straight with
   **g**, then press capital **L**, **R** or **U** to queue the next junction
   action through Buddy 3's API. Lowercase `l/r` are calibration commands and
   are rejected while moving. The queued turn still waits for a **separate
   physical cross**. Check turn completion **and following the outgoing line**.
4. Recheck **s**, **GP21**, and **GP20/e** on this image. GP20/e latch; **x**
   clears a latch while stopped, buttons released and clock valid.

The side GP27 channel is junction-only for now: the original preview threshold
212, hysteresis 3 ADC, 2 ms debounce, black width 11–30 mm measured by encoders,
and 4 mm white confirmation. This is not the adaptive barcode/peak decoder.
**Remove barcode sheets for Step 2 junction tests.** Optical contrast still
affects cross detection, so raw GP27 and its state are logged. A route acceptance
message confirms a queued request, not that the cross or completed turn was seen.

The existing straight-approach gate and 150 mm post-junction clearance remain.
Do not inject another route during an unfinished turn. There is no dashboard
mapping in this checkpoint; USB L/R/U use the reference A/F/Z route identifiers.

## Export the run to send back

The car keeps the **last 1,536 control samples**, about **15.36 seconds at 100 Hz**,
plus one terminal stopped/fault sample. Older rows are overwritten; the count
is reported. RAM history survives closing PuTTY, but not power loss or reboot.
A new accepted **g** replaces the previous run history.

1. Stop with **s**. Keep USB power connected and do not start another run.
2. Close PuTTY, then double-click **Export line log.cmd** in the extracted
   bundle. Select the current COM port. The helper sends **d only**, never Start.
3. If prompted about a missing dependency, run
   `py -3 -m pip install pyserial`, then retry.
4. Send the generated **`_line.csv`**, **`_summary.json`**, **`_usb.txt`** in
   `rtos_line_session`, plus the video and which test you performed.

Alternatively, enable PuTTY session logging, stop, press **d** and save through
`LINE_LOG_END`. That dump can be converted later:

```powershell
py -3 tools/rtos_migration/export_line_log.py --input putty.txt
```

The CSV contains raw/filtered front ADC, side ADC, calibrated levels, raw
optical targets, emitted targets, measured speed, physical signed PWM, steering
gain/damping, phase/decision/recovery flags, stop reason and control timing.
The run's calibration is preserved with the export even if you change it later.
`COMPLETE` means the export passed integrity checks; it does not certify driving.

Use **p/n** for clock/calibration/motion status and **z** for recent coordinator
requests/results. `Run timing check.cmd` remains available for a stopped idle
timing comparison; it opens the COM port, so PuTTY must be closed first.

## Build and implementation

From Git Bash with the same installed toolchain as Step 1:

```bash
bash tools/rtos_migration/build_line_following.sh
```

The new boundary code is in `app_program/line_navigation`; the common verified
motor, encoder, stop and clock foundation is in `app_program/rtos_bringup`.
`APP=rtos_line` selects this image; `APP=rtos_bringup` still builds Step 1.
See [architecture and data representation](../../../../app_program/line_navigation/README.md)
and [software checks](TEST_RESULTS.md).

**Hardware acceptance is pending.** These tests deliberately leave the known
SDK sharp-bend and post-junction issues visible before changing their tuning.
