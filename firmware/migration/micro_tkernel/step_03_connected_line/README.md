# Step 3B — connected line following v0.1.2

See [v0.1.2 network comparison](UPDATE_V0_1_2.md).
Latest artifacts: [firmware/v0_1_2](firmware/v0_1_2). Older v0.1 files remain for rollback.

**Experimental, hardware validation pending. Keep USB connected and its serial
capture actively reading while driving.** This joins Step2 v0.1.2 to the professor-style
kernel-owned Wi-Fi/MQTT bridge. It is not the complete Buddy1–4 migration.

Continuous V2 profile1 (nominal45mm/s), motor controller, front filtering,
calibration priority and angle-only junction experiment are retained. No line-follower gain,
threshold or turn-window changes. Physical barcode decoding is Step4; IMU/ramp
assist is Step5. The dashboard mapping can be set/read back but does not yet
control physical barcode actions. USB L/R/U still queue the canonical actions
at a separate bare cross. L/R and sharp bends still need the extended-mat test.

## Download and flash

1. Extract `robopico_rtos_connected_line_v0_1_2_bundle.zip` into a new folder.
   Keep the accepted Step2 and Step3A bundles for rollback.
2. Stop the robot, disconnect battery/motor power and close serial programs.
   Hold BOOTSEL while connecting USB; copy
   `firmware/robopico_rtos_connected_line_v0_1_2.uf2` onto RPI-RP2.
3. Close the previous bundle's dashboard/server terminal first so port8765
   serves this updated dashboard. Run **Start dashboard.cmd** from the new
   bundle and keep its broker/server terminal open.
   Mac: `python3 tools/dashboard/run_dashboard.py`.
4. Reuse the saved Wi-Fi settings if SSID/password and the laptop broker IPv4
   are unchanged. Otherwise run **Setup Wi-Fi.cmd**, or on Mac
   `python3 tools/rtos_migration/provision_network.py`. Enter the current
   laptop Wi-Fi LAN IPv4, not localhost/gateway/virtual-adapter address.
   Enter BOOTSEL only when prompted. The helper sends a settings-only UF2;
   **do not flash the application again afterward.** It may close when done.
5. Open the USB console at115200/no flow control. Check the connected-line
   v0.1.2 banner, `CLOCK ... status=PASS`, `LINE ... flags=0x3` while idle,
   and `NET ... connected=1 subscribed=1` with advancing polls/last_ms.
   Kernel/USB tasks must show successful creation. Motors must remain off.
   The kernel reports `wifi: drive PM set=0 get=0 mode=0` when no-power-save
   setup/readback succeeds. Send that line if it differs.
6. Press **n** and check `CAL_FRONT`. This firmware keeps Step2's priority:
   saved RTOS page, then valid SDK record28, then the original fallback.
   Your recent car imported **white122,138 / black246,267 (origin1)**.
   Flashing does not overwrite either calibration sector. **Do not recalibrate
   or press c for this comparison.** If the values differ, send the console
   output before judging steering; c restores the old fallback, not122/138.
7. In the browser at http://localhost:8765/ connect to **ws://localhost:9001**.
   Check Car online, advancing heartbeat and live motion/sensor values.
   The profile badge should say **RTOS connected line … Step3B**. Refresh
   with Ctrl+F5 if the old dashboard is cached. Use only one robot and one
   dashboard client on this broker during testing.

## If Start is rejected as Busy or the car returns to Safe Stop

The original dashboard mislabeled Step3B response4 as **queue full**. This
response means **Busy/start conditions unmet**, which includes a closed USB
serial session. The refreshed dashboard uses the correct label. v0.1.1 also includes the documented encoder/network changes. A visible command window showing the completed summary or
"Press any key" is **not** an open serial session. Browser MQTT alone does not
satisfy Step3B's USB supervision requirement.

For an immediate check, close the finished capture, open PuTTY at115200,
leave it connected, press n, then use dashboard Start. Check usb=3,
CLOCK PASS, released GP20/GP21 and valid ADC/calibration. Leave PuTTY open
through the run. Do not open the capture and PuTTY on the same COM port.
If it still refuses, send the n output and NET_COMMAND line from that attempt.

Robo Pico can power its controller and motors from USB with the board power
switch on; USB-only power is not proof of the cause. For a loaded floor run,
use the robot's usual tested power arrangement and keep USB serial supervision
open. See [Cytron's power guide](https://www.cytron.io/tutorial/get-started-robo-pico-circuitpython-power-up-robo-pico)
and [board power specifications](https://www.cytron.io/p-robo-pico).

## Record each test

Install the USB capture dependency once: `py -3 -m pip install pyserial`
(Mac: `python3 -m pip install pyserial`, or use your existing environment).

Close PuTTY and run **Capture connected test.cmd**. Choose COM4, or the full
Mac `/dev/cu.usbmodem...` path. The refreshed helper records300seconds and **holds USB
supervision open**; it sends no robot commands. Mac equivalent:

```bash
python3 tools/rtos_migration/capture_connected_line.py
```

Keep that capture window, the USB cable and the broker open. **Do not open
PuTTY at the same time.** Closing the capture/console or unplugging USB stops
the robot. Wireless driving without this supervision is not enabled yet.
Click Start in the first10–20seconds. When capture ends, its serial connection
closes and motion stops even if the command window remains visible. Reopen
capture or PuTTY before another Start. The old helper defaults to120seconds;
use `--seconds 300` to extend it without changing firmware.
The automatic summary deliberately says `REVIEW_PHYSICAL_TEST`; it does not
declare tracking or physical stopping successful from packet timing alone.

## Test A — stationary networking first

With battery/motor power off, capture at least75seconds. Use dashboard Ping,
apply/read mapping, and Telemetry off/on. Heartbeat must continue when motion
telemetry is off. With telemetry on, confirm sensor values change when a line
is moved beneath the front sensors. No Start is needed. Check clocks track
wall time, about100control updates/s, no new late intervals/USB drops, advancing
network polls, and received heartbeat gaps normally at most2seconds.

## Test B — short straight, Pause/Resume and stops

1. Start a fresh capture, put the sensors fully over the mat and place the car
   on a straight. Connect battery power; retain a loose USB cable. Wait for
   CLOCK PASS, both buttons released and the dashboard online for at least1s.
2. Click **Start**. Accepted means a **2s countdown**, not immediate wheel
   movement. Motion-active/Running must appear only when the controller starts.
   If rejected, keep the console capture; do not change calibration blindly.
3. Run a short straight, then click **Pause**. Motors must turn off, state
   becomes Paused, and coordinator owner becomes Buddy1. Sensor and encoder
   readings/heartbeat continue. Wait at least1s before clicking **Resume**.
   Resume requires both wheels still for120ms, a visible line for150ms and
   no fault. Ordinary tracking is reinitialized at the current position;
   pending canonical route and the run log are preserved.
4. Click **Stop**. It ends the session; Resume is no longer valid. After1s
   rest, reposition if needed and use a new Start for another run.
5. Separately verify GP21 normal Stop, GP20 E-stop and dashboard E-stop.
   After E-stop, release buttons, click **Reset**, wait1s, then use a new Start.
   Reset clears an eligible stopped latch; it never starts, erases calibration
   or reboots the Pico. Local s/e/x remain available through a serial console.
6. On a separate short run, close the serial capture while the dashboard
   stays connected: USB-supervision loss must stop motion. Reopening a
   console/capture must **not** restart it.

**Pause during a junction turn/search can interrupt navigation.** If the
coordinator requires recovery, Resume is rejected. Use Stop, place the car
back on a line, then a fresh Start. This checkpoint does not resume a half
completed turn from old deadlines. Do not retune junctions based on sensors
leaving the edge of the home mat.

## Test C — loss and reconnect

First verify this with wheels lifted and adequate sensor support. Keep USB
capture open. Start, then stop/restart the laptop broker; separately turn the
AP/hotspot off/on if practical. Preserve the capture through the interruption.
The robot must stop when network failure is detected, remain stopped after
reconnect, resubscribe, restore heartbeat and return the same RAM mapping.
Only an explicit new Start after reconnection may move it.

The safety task stops an active session if the kernel network poll or app
bridge is stale by250ms, or the reported link/MQTT/subscription is lost.
That is **not a250ms guarantee from an invisible RF/TCP failure**: transport
detection can take longer (MQTT keepalive5s). Measure the actual outage trial;
GP20/GP21 and USB supervision remain the local stop paths. Closing only the
browser is not a broker disconnect or a dead-man stop.

## Send me these files and a video

1. The three files in **rtos_connected_session** from each capture:
   `_usb.txt`, `_mqtt.csv`, `_summary.json`. MQTT capture is read-only,
   subscribes to is28/#, records ACKs/telemetry/heartbeats and reconnects its
   monitor after broker loss. It contains no Wi-Fi password.
2. **After a short driving run**, stop, let the capture finish (or close it),
   close PuTTY, keep USB power connected and run **Export line log.cmd**.
   Mac: `python3 tools/rtos_migration/export_line_log.py`. This sends **d only**.
   Send all three files in **rtos_line_session**. The RAM history holds the
   latest1536samples (about15.36s), including one terminal stopped row. Export
   before another Start, reboot or reflash replaces/clears it.
3. In the dashboard Exports panel, click **Export telemetry CSV**, **Export
   telemetry JSON**, **Export event CSV** and **Export all frames CSV** if
   available; Messages → **Save log** saves readable messages. Keep that tab
   open until exported. Live motion graphs include both speeds, targets, PWM,
   raw front/side ADC, normalized line levels, distance and heading.
4. A video of the short straight + Pause/Resume/Stop, with the full car and
   front sensors visible. Say which stop/control was pressed. For later bends
   or junctions, keep the entire sensor sweep on the mat.

The dashboard's **Download stopped-only log** is disabled for this RTOS
checkpoint: use **Export line log.cmd** for the full-rate stored RAM history.
Empty barcode/IMU panels are expected until Steps4/5. Live telemetry is20Hz;
USB stored motion is100Hz. Do not substitute one for the other when checking
short control events.

Only proceed to Step4 after timing/heartbeat remain healthy during motion and
the Start/Pause/Resume/Stop/E-stop/reconnect trials pass. Extended-mat L/R and
sharp bends remain explicit Step2 acceptance items.
