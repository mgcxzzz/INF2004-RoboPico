# Step5 v0.1.5 — bounded hump-qualification recovery

**EXPERIMENTAL micro T-Kernel Buddy1–4 migration build. Buddy5 is not integrated.**
Main is unchanged. Full left/right line reacquisition still needs enough mat;
this is not final migration acceptance.

The v0.1.4 flat/ramp runs kept IMU near50Hz and motor control near100Hz, with
no new motion-window acquisition/timing faults. Ramp assistance ran1.959s and
ended on descent. The accepted hump/max stayed0 because measurement qualification
was lost. Brief dirty odometry is the leading explanation; the decimated logs
could not identify its exact20ms sample.

v0.1.5 retains the v0.1.4 scheduling/read budgets and adds bounded qualification
recovery. Once valid level ground has been proved, an isolated odometry-quality
gap may pause readiness/candidate progress for at most100ms. Uncertain samples
cannot increase peak, rise/return confirmation or trusted travel. Candidate
excluded time remains limited to25%; prolonged gaps or genuine turn/reverse/
shock/Stop still cancel. First recovery rebases the movement history so outage
travel is not charged to the hump. See [change details](UPDATE_V0_1_5.md).

Existing200ms level/return qualification,5mm/60ms rise and minimum10mm trusted
travel remain. This changes complete-hump qualification, not the meaning of
maximum accepted height: it updates after a complete accepted hump. A separate
maximum-observed-height feature is not added in this checkpoint.

Continuous V2 steering, profile1 at45mm/s, barcode/peak decoder, Step4 gate and
110mm sensor offset, detector6 assistance, motor limits and shared nav APIs remain.
IMU stays core0 priority2 above USB3/Wi-Fi4; safety6/nav10/motor14 stay core1.
Periodic USB reports yield between groups rather than filling the ring in one
burst. Manual status reports remain immediate; do not repeatedly query them
while the automatic capture is running. Binary MQTT payload sizes are unchanged.

## Setup

1. Export any old run first. Extract the entire v0.1.5 bundle into a new
   folder. Close the old dashboard and all USB serial programs.
2. Motor power OFF: use BOOTSEL and flash
   `firmware/robopico_rtos_connected_imu_v0_1_5.uf2`.
3. Saved Wi-Fi and front calibration remain. Do not use w/l/r/k/c.
   Calibration priority is CRC-valid RTOS save, then valid SDK record28,
   then fallback. The expected car values are white122/138, black246/267;
   verify the actual readback instead of assuming it.
4. If SSID/password or laptop broker IPv4 changed, run **Setup Wi-Fi.cmd**
   and follow its settings-only BOOTSEL prompt. Otherwise skip this step.
5. Run **Start dashboard.cmd**, keep that terminal open, hard-refresh
   http://localhost:8765/ and connect ws://localhost:9001.
6. Run **Capture connected test.cmd**, select the actual USB port and keep
   it open. It records300seconds by default; Ctrl+C finishes early. Close
   PuTTY and any competing serial program. Apply/read back the desired
   mapping again after boot. The capture never starts motion for you.

**USB supervision remains mandatory.** Keep battery/motor power on in
addition to USB for driving. Closing capture removes supervision and stops
motion. Powerbank-only driving is still deferred to final integration.

## Mac commands

From the extracted bundle folder, using Python3.10+ and installed Mosquitto:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install pyserial
.venv/bin/python tools/dashboard/run_dashboard.py --mosquitto "$(brew --prefix mosquitto)/sbin/mosquitto"
```

Leave that terminal open. In a second terminal in the same folder:

```bash
.venv/bin/python tools/rtos_migration/capture_connected_line.py
```

For changed network settings only, close the serial capture first and run:

```bash
.venv/bin/python tools/rtos_migration/provision_network.py
```

Enter the Mac's current Wi-Fi IPv4 as broker and follow the BOOTSEL prompt.
Dashboard, capture and exports do not require BOOTSEL. Provisioning may close
after it sends the settings; dashboard/broker and capture stay open during tests.

## Test A — flat negative control

Use the unchanged low/tilted sensor mounting and saved calibration.

1. Keep the car flat/still through calibration. Open dashboard and capture,
   confirm the v0.1.5 banner, CLOCK PASS, calibrated Idle state2/valid1/fault0,
   IMU core0/priority2 and expected front calibration. No long stationary
   campaign is needed; about10–20seconds of initial evidence is enough.
2. Switch motor power on in addition to USB. On a supported flat straight,
   Start, allow2seconds countdown, drive about5seconds, Pause, Resume briefly
   and Stop. Keep the cable loose and stop controls accessible.
3. Wait10seconds while stopped, then Ctrl+C. Retain this connected-session
   capture separately. Expected accepted hump count and max are both0.

## Test B — one complete one-sided ramp

1. Start capture again before Start. Place the car on level mat at least2seconds
   of driving before the same10mm one-sided ramp. Support the complete IR/wheel
   path. Keep the same ramp orientation as the successful v0.1.4 crossing.
2. Apply/read back mapping if needed. Start and cross the ramp without touching
   the robot. Allow it to drive onto level ground for about2seconds after descent,
   then Stop. Do not stop at the crest: acceptance needs a qualified return.
3. If motion becomes unsafe, Stop immediately. Do not prioritize completing the
   measurement over keeping the robot on supported mat.
4. Capture another10seconds while stopped, then Ctrl+C. Zip the entire new
   `rtos_connected_session` folder (USB text, MQTT CSV and summary JSON).
   Also export the line log before another Start/reboot using the section below.
5. Send both flat and ramp captures and the ramp line export. A video of this
   one ramp crossing helps align the climb/descent with the qualification trace;
   if unavailable, report whether the full crossing and return occurred before Stop.

Expected ramp result is one accepted hump with a nonzero accepted maximum,
not a forced10mm value. Live height, complete-hump count and assist activation
are distinct measurements. Short-gap holds/recoveries are allowed; disarms/
early clears must have explainable reasons. Capture retains `IMU_HUMP` flags,
counts and latched reasons so even a brief interruption is visible later.

Compare settled-window deltas: IMU about50Hz, control about100Hz, no new read/
late/FIFO/budget faults, responsive heartbeat/Stop, and no additional USB record
drops. Lifetime startup counters are not silently reset or treated as a pass.
The capture still reports REVIEW_PHYSICAL_TEST, not an automatic driving pass.

`IMU_HUMP flags`:1=level-ready,2=candidate,4=confirmed rise,8=returning.
Its counters restart at physical IMU initialization, not every Start. Use count
deltas for each run. Full field/reason definitions are in UPDATE_V0_1_5.md.
All previous IMU_PHASE/IMU_BUDGET/IMU_CAL/NET_SERVICE/LOGGING diagnostics remain.

## Stored exports when requested

Stop, keep power connected and do not Start/reboot/reflash. Finish capture so
its serial port closes. Run **Export line log.cmd** and, for barcode evidence,
**Export barcode log.cmd**, one at a time. Mac equivalents:

```bash
.venv/bin/python tools/rtos_migration/export_line_log.py
.venv/bin/python tools/rtos_migration/export_barcode_log.py
```

Send `rtos_line_session` and `rtos_barcode_session` alongside the connected
capture. Retain `dashboard_session/broker.log` if network delivery stalls.
