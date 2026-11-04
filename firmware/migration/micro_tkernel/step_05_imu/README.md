# Step5 v0.1.3 — isolated IMU task and calibration startup diagnostics

**EXPERIMENTAL micro T-Kernel Buddy1–4 test build.** Buddy5 is not integrated.
Step4 barcode/turn dispatch has been reported working; full left/right line
reacquisition still needs sufficient mat. This is not a main-branch release
or acceptance of the complete migration. v0.1.2 stationary testing retained
17/20 late IMU gaps and a cold calibration timeout, despite clean reads.
**v0.1.3 isolates IMU work from core0 USB/Wi-Fi and records exact calibration
restart evidence. Complete the stationary test first.** Any further Step4
correction must also be carried into this checkpoint before final merge.

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

The IMU is sampled in its own core1 priority16 task, below motor/safety/
navigation. The core1 control task remains100Hz and owns ADC/encoders/PWM;
it does not perform I2C transfers.
The motor cap420 and slew1200/s remain. Buddy4 requests temporary modifiers
through the nav coordinator; it does not take line ownership or write PWM.

**USB supervision is still mandatory.** Powerbank/untethered policy changes
are deferred to the final integration as requested. This image also retains
the SDK baseline's IMU fault policy: faults are reported and assistance becomes
unavailable; they are not a new unconditional global driving stop.

The current baseline detects **one wheel higher than the other** using side
tilt. Use the same one-sided ramp and mounting as the working Buddy4 SDK
tests. A ramp lifting both wheels equally is not the same acceptance test.

The main-branch Buddy1–4 UF2 uses the Pico SDK. This migration image uses
micro T-Kernel tasks; the shared algorithms do not by themselves guarantee
the same timing after task/core placement changes.

## Setup

1. Finish/export the current Step4 run before changing firmware. Keep its
   files and the Step4 v0.1.2 bundle for comparison/rollback.
2. Extract this complete Step5 bundle into a NEW folder. Close the previous
   dashboard and USB serial programs. With motor power off, BOOTSEL and flash
   `firmware/robopico_rtos_connected_imu_v0_1_3.uf2`.
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
Confirm the banner **CONNECTED IMU v0.1.3 / Step5**, CLOCK PASS, network online,
and the unchanged front calibration. Confirm:

- `IMU state=2` (Idle), `valid=1`, `fault=0` after stationary calibration.
- `IMU ticks` increases by approximately50 per wall-clock second.
- `IMU_TILT tracking=0 valid=0` while idle is normal: its valid flag is for
  active-run ramp measurements, not the stationary calibration result.
- Use the new **IMU_ACQ** and **IMU_I2C** lines for full counters. The old
  `IMU i2c_errors` field is only6bits and wraps; `missed` combines late gaps
  and FIFO overruns. Those old fields alone do not identify the cause.

### Exact v0.1.3 stationary cold-boot and two-window retest

1. Motor/battery power OFF. Place the car flat before the new UF2 boots and
   keep it still. Start the dashboard and connected capture, telemetry ON,
   other serial apps closed. Do not press Start or use w/l/r/k/c.
2. **Observe cold-boot status before pressing Reset.** Record at least20s
   of capture. Confirm the v0.1.3 banner, `IMU_CAL core=1 priority=16`,
   `NET_SERVICE core=0`, `LOGGING atomic=1`, CLOCK PASS and the expected front
   calibration. Initial success is IMU state2/valid1/fault0, progress100/best100
   and ready_epoch matching the actual epoch. Do not relabel epochs.
3. If cold boot faults, leave the capture running for20seconds before using
   one stopped Reset. The frozen IMU_CAL counters/progress/reasons/spans are
   evidence; do not discard the run or repeatedly Reset. Mark cold boot as
   failed even if this recovery succeeds. If recovery also fails, stop here
   and send the capture. Do not force calibration or change thresholds.
4. Once calibrated Idle is reached, wait10seconds, then record a60second
   settled window. `reads` should increase about3000; failed/late_gaps/FIFO
   overruns/timeouts/aborts and LOGGING dropped/oversize should not increase.
   Compare deltas, since init can probe an absent optional magnetometer.
   `rescued` can increase. Main CONTROL should stay100Hz with CLOCK PASS and
   no new late intervals; keep checking this after the IMU core change.
5. Click stopped dashboard Reset once, keep the car still and confirm the
   new ready_epoch catches up after calibration. Wait10seconds, then record
   another60second window. Keep all epochs and any initial failure evidence.
6. Ctrl+C while stopped. Zip the entire new `rtos_connected_session` folder
   (USB text, MQTT CSV and summary JSON) and send it, with cold-boot outcome
   and the two actual window epochs. No video or stored-log exports needed.

New diagnostic meanings:

- IMU_CAL progress/best are stable means out of100. age_ms is total elapsed
  time from starting that calibration, not time since the last stability
  restart. Counters reset only on physical IMU reinitialisation and remain
  visible after a fault, even when capture opens after startup.
- last_reason bits1/2/4 identify forward/lateral/up axis range>40mg. Bit8
  identifies the existing estimated-speed stationary guard. axis_restarts
  count each affected axis; last_span_mg retains the rejected ranges.
  Failed/empty reads, late gaps and FIFO overruns during calibration are
  recorded separately. State6/fault3 means the unchanged10s deadline expired.
- NET_SERVICE shows elapsed radio/link, lwIP timer and MQTT bridge service
  durations on core0. They include interruptions/waits and are not exclusive
  CPU time. The application only reads a copied snapshot.
- IMU_TIMING retains recent last/late gaps and current/previous10second bucket
  maxima; LOGGING max_copy_us measures the short masked ring copy. Lifetime
  maxima may predate the settled windows, so retain recent bucket records.

The code still uses the same2second calibration requirement,10second deadline,
30ms timing guard, IR calibration, decoder, motor limits and ramp logic. This
retest establishes scheduling/readiness; ramp success is not assumed.

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
