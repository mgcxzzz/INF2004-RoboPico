# Step5 v0.1.4 — IMU timing during driving

**EXPERIMENTAL micro T-Kernel Buddy1–4 migration build. Buddy5 is not integrated.**
Main is unchanged. Step4 barcode/turn dispatch works in the reported tests;
full left/right line reacquisition still needs enough mat. This is not final
migration acceptance.

v0.1.3 passed stationary cold boot/Reset and functional flat tracking,
Pause/Resume and Stop. During driving its IMU fell to roughly15–20Hz and
accumulated late gaps/FIFO overruns. v0.1.4 moves the sole IMU task to
**core0 priority2**, ahead of USB3 and kernel Wi-Fi4. Safety6, navigation10
and motor14 remain on core1. It adds an8ms aggregate FIFO read deadline,
a20ms init deadline and a2ms reset-ready bound. The whole task tick is not
hard-capped at8ms: processing wall time is measured separately.

Continuous V2 steering, profile1 at45mm/s, barcode/peak decoder, Step4 gate
and110mm sensor offset, detector6 ramp thresholds, motor limits and shared
APIs/nav coordinator remain. Buddy4 requests modifiers through navigation;
it never writes PWM. Network calls remain with the kernel Wi-Fi service.
Packed binary message layouts are unchanged. See [the change review](UPDATE_V0_1_4.md).

## Setup

1. Export any old run first. Extract the entire v0.1.4 bundle into a new
   folder. Close the old dashboard and all USB serial programs.
2. Motor power OFF: use BOOTSEL and flash
   `firmware/robopico_rtos_connected_imu_v0_1_4.uf2`.
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

## First test — short stationary baseline, then flat motion

Do this before another ramp test. No repeated long stationary campaign is needed.

1. Keep the car flat and still with motor power off. Record about20seconds
   after opening capture. Confirm the v0.1.4 banner, CLOCK PASS,
   `IMU_CAL core=0 priority=2`, `NET_SERVICE core=0`, and the expected IR values.
   Calibrated Idle is `IMU state=2 valid=1 fault=0`; ready_epoch must match
   epoch. `IMU_TILT valid=0 tracking=0` while idle is normal.
2. If calibration fails, retain20seconds of failure evidence, Stop, and send
   the capture. Do not repeatedly Reset or change calibration to hide it.
3. Once ready, switch motor power on. Use a supported flat straight with a
   loose USB cable and both front sensors remaining above mat. Keep GP20/GP21
   accessible. Click dashboard **Start**, allow its2second countdown, run
   about5seconds, **Pause** for3seconds, **Resume** for about5seconds, **Stop**.
   Shorten the moving segments if mat space requires it and report that.
4. Check Stop and GP20/GP21 safety response on this build before ramp work.
   Stop promptly if tracking, USB or heartbeat becomes unresponsive.
5. Leave capture running another10seconds while stopped, then Ctrl+C. Zip
   the entire new `rtos_connected_session` folder: USB text, MQTT CSV and
   summary JSON. State which parts were moving/paused. Send it for review.

Expected outcome: reads/ticks advance near50/s during motion as well as Idle;
no new settled-window failed reads, late_gaps, FIFO overruns or budget
exhaustions; motor control stays100Hz/CLOCK PASS and heartbeat/USB stay live.
Compare counter deltas, not just lifetime maxima. Optional magnetometer
probing can produce an init abort, so preserve the baseline and all details.
The script reports REVIEW_PHYSICAL_TEST; it does not certify driving quality.

The new USB lines are saved automatically:

- **IMU_PHASE:** FIFO read, conversion/processor and whole-tick wall times,
  maxima and FIFO batch size. These are wall times, not exclusive CPU use.
- **IMU_BUDGET:** FIFO8ms/init20ms exhaustion and excess beyond each limit,
  reset-ready2ms timeouts, and full ticks over8ms (diagnostic only).
- **SIDE_SERVICE:** every due side ADC attempt and decoder/planner wall time,
  including polls between motor ticks. Totals wrap modulo32bits.
- Existing IMU_ACQ/IMU_TIMING/IMU_CAL, NET_SERVICE, LOGGING and CONTROL lines
  remain. Use full acquisition counters rather than the legacy6-bit error field.

**No video or stored exports are required for this first latency check.**
If a physical line-tracking problem occurs, a video and stored line log help.
After reviewing clean flat timing, repeat the same10mm one-sided ramp, then
barcode/junction compatibility. A physical ramp crossing and accepted Buddy4
height/assistance are separate results; neither is assumed from this build.

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
