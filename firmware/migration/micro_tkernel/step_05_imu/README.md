# Step5 v0.1.2 — short logging critical sections and recent IMU timing

**EXPERIMENTAL micro T-Kernel Buddy1–4 test build.** Buddy5 is not integrated.
Prepared in parallel while Step4 v0.1.2 hardware results are pending; this is
not acceptance of Step4, completion of migration or a main-branch release.
Step4 barcode/turn dispatch has been reported working; full left/right line
reacquisition still needs sufficient mat. Step5 v0.1 stationary testing found
intermittent IMU read errors. v0.1.1 passed reads/Reset in two stationary
windows but retained9/10 late gaps. **This update needs a stationary retest
first: logging now formats outside interrupt-disabled locks.**
Any further Step4 correction must also
be carried into this checkpoint before final acceptance/merge.

## What is included

- All Step4 v0.1.2 line/barcode/junction/MQTT logic,110mm sensor offset and
  confirmation-time approach gate.
- Existing front calibration and45mm/s Continuous V2 profile1. No forced
  calibration replacement and no front sensor tuning.
- Buddy4's GY-511/LSM303 on **SDA GP2, SCL GP3**,400kHz I2C1,400Hz FIFO,
 50Hz processing and2second stationary IMU calibration.
- The reference powered acceptance and ramp detector6: bounded feedforward
  addition, stall push and minimum-speed hold through the shared nav API.
- Kernel-owned networking, dashboard IMU/quality/ramp cards and copied
  telemetry. USB diagnostics add **IMU**, **IMU_TILT** and **RAMP** lines.

The IMU is sampled in its own core0 priority13 task. The core1 control task
remains100Hz and owns ADC/encoders/PWM; it does not perform I2C transfers.
The motor cap420 and slew1200/s remain. Buddy4 requests temporary modifiers
through the nav coordinator; it does not take line ownership or write PWM.

**USB supervision is still mandatory.** Powerbank/untethered policy changes
are deferred to the final integration as requested. This image also retains
the SDK baseline's IMU fault policy: faults are reported and assistance becomes
unavailable; they are not a new unconditional global driving stop.

The current baseline detects **one wheel higher than the other** using side
tilt. Use the same one-sided ramp and mounting as the working Buddy4 SDK
tests. A ramp lifting both wheels equally is not the same acceptance test.

## Setup

1. Finish/export the current Step4 run before changing firmware. Keep its
   files and the Step4 v0.1.2 bundle for comparison/rollback.
2. Extract this complete Step5 bundle into a NEW folder. Close the previous
   dashboard and USB serial programs. With motor power off, BOOTSEL and flash
   `firmware/robopico_rtos_connected_imu_v0_1_2.uf2`.
3. Saved Wi-Fi and front calibration remain. Do not use w/l/r/k/c. If the
   network or laptop broker IP changed, run **Setup Wi-Fi.cmd**; it sends
   settings only. Otherwise skip Wi-Fi setup.
4. Run **Start dashboard.cmd**, leave it open, and hard-refresh
   http://localhost:8765/. Connect ws://localhost:9001. The fresh telemetry
   badge should identify **Step5 experimental**.
5. Run **Capture connected test.cmd** and choose the actual USB port. Keep
   this capture reading while driving; its default duration is300seconds.
   Close other serial programs. Apply/read back the desired barcode mapping
   again after flashing. Start is still through the dashboard with2s delay.

Mac equivalents, from the extracted folder after installing Python3.10+,
Mosquitto and pyserial in a local virtual environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install pyserial
.venv/bin/python tools/dashboard/run_dashboard.py --mosquitto "$(brew --prefix mosquitto)/sbin/mosquitto"
```

Leave that first terminal open. In a second terminal, enter the same folder:

```bash
.venv/bin/python tools/rtos_migration/capture_connected_line.py
```

For changed Wi-Fi settings only:

```bash
.venv/bin/python tools/rtos_migration/provision_network.py
```

Use the Mac's current Wi-Fi IPv4 as the broker and follow the BOOTSEL prompt.
No BOOTSEL is required for dashboard/capture/log exports.

## Test A — IMU readiness while stationary

Keep motor power off, car flat and still for at least3seconds after boot.
Confirm the banner **CONNECTED IMU v0.1.2 / Step5**, CLOCK PASS, network online,
and the unchanged front calibration. Confirm:

- `IMU state=2` (Idle), `valid=1`, `fault=0` after stationary calibration.
- `IMU ticks` increases by approximately50 per wall-clock second.
- `IMU_TILT tracking=0 valid=0` while idle is normal: its valid flag is for
  active-run ramp measurements, not the stationary calibration result.
- Use the new **IMU_ACQ** and **IMU_I2C** lines for full counters. The old
  `IMU i2c_errors` field is only6bits and wraps; `missed` combines late gaps
  and FIFO overruns. Those old fields alone do not identify the cause.

### Exact v0.1.2 stationary retest (no Start and no ramp yet)

1. USB power only, car flat and still. Start the dashboard and the connected
   capture as above; leave telemetry ON and other serial apps closed.
2. Confirm `LOGGING atomic=1` is present. Wait10seconds for boot/connection/calibration to settle. Check `state=2`,
   `valid=1`, `fault=0` and `IMU_ACQ epoch=ready_epoch=1`.
3. Keep capturing for60seconds. Read counters as **changes during this
   window**: `failed`, `late_gaps`, `fifo_overruns`, `timeout_tx/stop/rx`
   and `aborts` should not increase. `LOGGING dropped/oversize` should stay
   constant in settled windows. Keep the initial counter values as baseline.
   Initial magnetometer probing can produce
   NACKs during init, so do not require lifetime abort counters to start at0.
   `reads` should increase about3000 in60seconds. `rescued` may increase:
   it counts completed transfers saved from the former false-timeout race.
4. While still stopped, click dashboard **Reset once**. This version starts
   a fresh IMU hardware init/calibration even from healthy Idle. Keep it still;
   `epoch` advances to2 and `ready_epoch` catches up to2 after calibration.
   A brief Calibrating state is expected. Do not press Start.
5. Wait10seconds, then observe another60second window. Acquisition/transport
   counters remain lifetime counters; compare changes after settling, not
   absolute totals across the deliberate reset/init.
6. Stop the capture with Ctrl+C. Zip the new `rtos_connected_session`
   folder and send it (USB text, MQTT CSV and summary JSON), with your
   observation of heartbeat and both calibration epochs. **No video, line
   export or barcode export is needed for this stationary retest.**

`IMU_TIMING` records last gap and last late gap (with its read number), plus
current/previous10second bucket maxima and late counts. `win_seq` changes
every10seconds; these recent buckets are separate from lifetime
`max_gap_us`. `LOGGING max_copy_us` measures only the short masked ring copy,
not formatting or lock-wait time. Include these lines if late gaps persist.

If any errors grow, send the capture rather than repeatedly resetting or
changing calibration. `IMU_I2C` now identifies the failing hardware phase
and abort source; `IMU_ACQ` identifies late scheduling separately from FIFO
overrun. These counters are included in the new capture summary.

The v0.1.1 correction uses status rechecks and bounded per-phase waits with
interrupts enabled; it does not hide errors by removing timeouts. The existing
binary MQTT packet formats, steering, front calibration, barcode logic and
ramp settings are retained. Physical success remains to be established.

If IMU state=6/fault is nonzero, retain the capture, check GP2/GP3 power/ground
and mounting, then use **dashboard Reset while stopped**, release buttons and
keep the car still for a fresh2second IMU calibration. Reset does not change
front IR calibration. Do not attempt the ramp until IMU readiness is confirmed.

## Test B — short flat run

Turn motor/battery power on. Keep the cable loose. Run a short supported flat
straight, Stop, then test Pause/Resume and Stop as space permits. Compare
smoothness with Step4. On flat ground, RAMP should remain idle with no applied
boost/hold/push. IMU state3 (Tracking) and fresh tilt/odometry are expected
during a run. Confirm USB/button Stop/E-stop still work before ramp driving.
Export before another Start, following the instructions below.

## Test C — the same one-sided ramp as the SDK baseline

Start on level mat before the ramp, using the original low/tilted mounting
and unchanged front calibration. Keep the full car and IR path on supported
mat; avoid cable loading. Keep GP20/GP21 accessible and video the full climb,
crest, descent and return to level. First use a ramp-only straight, with no
barcode/junction nearby, so ramp behavior can be assessed independently.

Watch the dashboard IMU/side-height quality and ramp assistance cards:
assistance can start after sustained valid side height; it is bounded and
must clear after the episode or Stop/Pause/search/junction. A physical ramp
crossing and an accepted hump estimate are separate checks: report both.
Record accepted/rejected hump count, peak height and any quality exclusions.

If the robot stalls, loses the line, jerks severely or the heartbeat freezes,
Stop promptly and retain that evidence. Do not change IR calibration or
increase speed to mask a migration problem.

After these checks, repeat a mapped barcode/junction with IMU running and
check that flat steering, barcode decoding, turn completion and heartbeat
remain comparable to Step4. The first Step5 test does not need a whole circuit.

## Export and send after EACH run

Click Stop, keep Pico power connected and do not new Start/reboot/reflash.
Let capture reach30seconds total, Ctrl+C while stopped and wait for serial
close. Run **Export barcode log.cmd**, then **Export line log.cmd**, one at a
time. Mac commands:

```bash
.venv/bin/python tools/rtos_migration/export_barcode_log.py
.venv/bin/python tools/rtos_migration/export_line_log.py
```

Send the files from all three folders:

- `rtos_connected_session`: USB text, MQTT CSV, summary JSON. **Essential for
  Step5:** MQTT capture includes source2/4 IMU/quality and source15 ramp data;
  USB includes IMU timing/quality and ramp readback.
- `rtos_barcode_session`: USB text, events CSV, raw CSV, summary JSON.
- `rtos_line_session`: USB text, line CSV, summary JSON.
- Video, plus `dashboard_session/broker.log` if delivery/timing misbehaves.
- Optionally export the dashboard's Buddy4 CSV for convenient viewing.

Stored line history is the last6.4seconds. Raw GP27 freezes on first decode,
or rolls until Stop when no barcode decodes. Neither stored export contains
a full raw IMU FIFO history: use the live MQTT/USB capture for IMU evidence.
The dashboard does not download stored RTOS line/barcode histories.

Label each set A/B/C with the physical ramp, intended action and observation.
Export before another Start; Start replaces the RAM run history. Resume an
ordinary line pause only; an interrupted junction still requires Stop/new Start.
