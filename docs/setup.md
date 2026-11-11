# Set up Buddy 1–4 RTOS v0.2.3

**PROVISIONAL main release. Buddy 5 is not integrated.** Use the complete new
bundle, not the old SDK/Step 5 helpers. Profile 1 is the tested 45 mm/s controller.

[Exact v0.2.3 IMU safety checks and downloads](imu_end_run_v0_2_3.md). Use the new
bundle/dashboard and hard-refresh; the old dashboard cannot explain junction/bend/recovery snapshots.

## Flash and network setup

1. Extract the full bundle into a new folder. Install Python3.10+ and Mosquitto
   on the laptop. For optional USB capture/export install pyserial once:
   `py -3 -m pip install pyserial`(Windows) or use the Mac commands below.
2. With motion stopped, enter BOOTSEL and copy
   `firmware/robopico_buddy1_4_rtos_v0_2_3.uf2` to RPI-RP2. It reboots.
   Saved Wi-Fi and front calibration are preserved. Keep the robot flat/still
   for IMU calibration. No w/l/r/k/c recalibration is needed for this update.
3. Run **Start dashboard.cmd** and keep it open. Open http://localhost:8765/,
   hard-refresh, then Connect to ws://localhost:9001. Browser-to-broker and
   car-to-broker are separate connections; wait for car-online.
4. If SSID/password or laptop broker IPv4 changed, close serial programs and
   run **Setup Wi-Fi.cmd**. Enter the2.4GHz network and laptop's active LAN
   IPv4, not localhost/Pico/virtual-Ethernet IP. Follow its BOOTSEL prompt.
   This writes a settings-only UF2 and preserves the application. Do not
   reflash the main UF2 afterward; the settings helper may close on completion.
   If existing settings work, skip this step.
5. Confirm the dashboard identifies **RTOS Buddy 1–4 v0.2.3**, Profile 1 and
   calibration. Expected saved values on the tested car are white122/138,
   black246/267; actual readback is authoritative. Wait for IMU calibrated
   Idle(valid1/fault0), car-online and ready clocks before Start.
6. Set four distinct A–Z letters for straight/U-turn/left/right, Apply mapping
   and verify readback. Mappings are in RAM: send them again after power loss.

## Run from a powerbank

7. Stop before switching power arrangements. Connect the Pico/controller to
   the powerbank; use the same working motor/battery supply as the successful
   tests. A powered Pico alone does not confirm the motors have power.
   Leave the laptop broker/dashboard running on the same Wi-Fi/hotspot.
8. The new firmware needs no laptop USB cable, PuTTY or serial capture for
   permission to drive. Motors remain OFF at boot. Once online/calibrated,
   click Start; the existing2 second countdown runs. Pause/Resume/Stop work
   over Wi-Fi. GP21 stops; GP20 is emergency stop. Reset while stopped clears
   eligible latches and recalibrates IMU; it does not start motion.
9. A lost Pico/broker link or stale network delivery stops motion; reconnect
   does not resume it. Closing just the dashboard or capture does not stop
   the robot while its broker connection remains healthy. Use Stop/buttons.
   Do not remove the controller's only power source expecting RAM logs to survive.

First checks: normal tracking, qualified lift/tilt motor stop, blocked restart,
upright Reset/recalibration, and one normal ramp for false-positive checking. Follow the
[v0.2.3 test guide](imu_end_run_v0_2_3.md). No full network reconnect sequence is required for this safety addition.

## Wireless logs — no COM port

10. Optionally, before Start run **Capture wireless test.cmd**. It subscribes passively
    to MQTT and sends no motion commands. Default duration300s; Ctrl+C ends
    early. Stop the robot first; ending this observer does not stop motion.
11. After Stop, keep the Pico/powerbank and broker on. Wait 5 seconds for fresh
    coordinator Off, keep Telemetry on, and use the dashboard stored-data
    selector: **Motion rows**, **Diagnostics**, **Raw sensors**, **Navigation**.
    Request one dataset at a time. Wait for Complete; partial/retried transfers
    are never labelled complete. Start is blocked during an active transfer.
12. Each stored download produces .csv/.bin/.json on the laptop. Diagnostics
    now decode the full RTOS controller/steering/junction row; raw CSV has
    time/distance/GP27 ADC. Navigation uses the existing compact record bytes.
    Also export dashboard messages/events. Send the entire
    `rtos_wireless_session` folder plus the requested stored files and video
    for bend/turn failures. Record barcode letter/mapping and where it failed.

Histories are RAM-only:640 recent100 Hz controller rows(~6.4s),2048 raw GP27
rows for the latest reportable barcode attempt, and the bounded nav-coordinator log.
They are not an unlimited full-course recorder. The passive MQTT capture
records the full period while open. Export before another Start or reboot;
main does not silently restore an old SDK run as the new RTOS run.

## Optional detailed USB logs

Keep power connected after Stop. Attach the laptop USB data connection without
rebooting the Pico, close competing serial programs, then use **Export line
log.cmd** and **Export barcode log.cmd** one at a time. These retain the detailed
Step 5 text/counter formats. **Capture connected test.cmd** is also available;
in this release closing it does not stop motion. Physical power continuity
matters: disconnecting the sole powerbank feed and then attaching USB reboots
and loses RAM histories. Wireless downloads avoid that problem.

## Mac equivalents

From the extracted bundle root, with Python3 and Homebrew Mosquitto installed:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install pyserial
.venv/bin/python tools/dashboard/run_dashboard.py --mosquitto "$(brew --prefix mosquitto)/sbin/mosquitto"
```

Leave that terminal open. In another terminal in the same folder:

```bash
.venv/bin/python tools/rtos_migration/capture_wireless.py
```

Changed network settings only(close serial programs first, follow BOOTSEL):

```bash
.venv/bin/python tools/rtos_migration/provision_network.py
```

Optional stopped USB exports:

```bash
.venv/bin/python tools/rtos_migration/export_line_log.py
.venv/bin/python tools/rtos_migration/export_barcode_log.py
```

Windows Python equivalents use `py -3` with the same script paths.
The broker address provisioned to the Pico is the laptop's current Wi-Fi LAN
IPv4. The dashboard/observer on that laptop use localhost. If no car-online,
check the broker log for a Pico connection and confirm TCP1883 firewall/LAN
access; localhost WebSockets alone does not establish the Pico's connection.
