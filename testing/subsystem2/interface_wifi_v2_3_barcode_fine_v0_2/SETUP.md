# V2.3 setup and run logs

## Updating the existing working laptop setup

1. Keep your current Mosquitto installation and dashboard package. They are compatible; replacing the laptop package is optional. Stop the robot before updating.
2. Download `firmware/buddy123_interface_wifi_v2_3.uf2`. Enter BOOTSEL, copy this UF2 to RPI-RP2, then let it reboot. Do not erase the entire flash or copy an old firmware afterward. Normal reflashing preserves the saved Wi-Fi/broker sector.
3. Open PuTTY at 115200. Press `h`: the banner must say `BUDDY123 INTERFACE WIFI v2.3`. Check `CAL FRONT white=163,160 black=3457,3700 valid=1` and profile 1. No calibration is needed for the first test. A V2.3 calibration you saved later will replace these defaults; `c` then `k` restores them while stopped.
4. Start your existing dashboard/broker as before. If using this package, double-click **Start dashboard.cmd**, or run `py run_dashboard.py` on Windows / `python3 run_dashboard.py` on macOS. Keep its terminal open. Mosquitto must be installed; its MQTT listener uses 1883 and browser WebSockets use 9001. Open the printed dashboard URL, normally `http://localhost:8765/`, and Connect to `ws://localhost:9001` on that laptop.
5. Press `n` twice, about 10 seconds apart. Look for `mqtt=1 subscribed=1`, MQTT stage `online`, and advancing network poll/time counters. Confirm fresh heartbeats and Car online in the dashboard. Heartbeat uptime should advance by about 10 seconds over 10 wall seconds. If it still advances tens of times slower, stop here and save both status samples.
6. Reapply your barcode mapping after boot; there are no default A/F/Z assignments. For the familiar test, assign A=Left, F=Right and Z=U-turn, and choose a different letter for Straight. Apply mapping and check the acknowledgement. Leave unused letters unassigned.
7. Put the robot on the mat, release both buttons and select profile 1 (USB key `1`; `k` saves). Switch Telemetry on in the dashboard. Press Start: its countdown should take about two real seconds. Check a short straight first, then a bend and barcode/junction circuit. Use the physical stop button to end the run; verify heartbeats remain fresh during and after motion.

Wi-Fi and MQTT work without USB once configured. Run from the normal battery supply for an untethered test. Stop and wait five seconds before changing power; changing USB connectivity during a run invokes the existing stop behavior. The dashboard/broker terminal stays open; the one-time Wi-Fi configuration window may close.

## Only if Wi-Fi, password or laptop broker IP changed

Use the laptop's current IPv4 on the same 2.4 GHz Wi-Fi/hotspot, not the Pico's IP or the gateway. Existing WCF1 device settings from the UTK setup are compatible. No reprovision is required if those settings are still correct.

With **V2.3 running normally over USB**, close PuTTY and double-click **Configure WiFi.cmd**. This SDK helper uses `provision_wifi.py` over the USB serial port: it **does not need BOOTSEL**. Enter the full port, e.g. `COM4`, SSID, password locally and laptop broker IPv4. It stops the robot, waits for saving, writes settings and reboots. Success says settings were verified. Reopen PuTTY and press `n`.

Python equivalent (from this folder):

```powershell
py -m pip install pyserial
py provision_wifi.py --port COM4 --broker YOUR_LAPTOP_IPV4
```

On macOS use `python3`, install pyserial in your Python environment, and pass the real `/dev/cu.usbmodem...` device. Keep passwords out of chat and Git. The older UTK BOOTSEL provisioning helper is a different workflow; do not mix its steps with this serial helper.

## Logs to send after a run

Keep the dashboard connected before starting; telemetry exports capture data received in that browser session.

1. Stop the car. Wait at least five seconds and check the coordinator reports **Off**.
2. Click **Export telemetry CSV** and **Export event CSV**. Also use **Save log** for the dashboard's message log. Keep these files with the test video.
3. In **Stored data**, choose **Diagnostics**, then click **Download stopped-only log** and wait for completion. Repeat for **Motion rows**, **Navigation rows**, and **Raw sensor rows**. Download all four if diagnosing a new failure. These read the robot's bounded saved captures; a successful download does not imply an unlimited whole-run recording.
4. If networking drops, stop physically. Save `dashboard_session/broker.log` from the laptop and obtain USB dumps in PuTTY: `h`, `n`, `p` (motion/events), `d` (drive diagnostics), `j` (barcode), `a` (raw ADC), `z` (nav coordinator). Use PuTTY Session logging -> All session output to save a text file, then press each key once and let it finish. Avoid recalibration, a new run or reset before exporting.
5. Report profile, straight/bend/circuit, USB or battery, duration, visible behavior and whether heartbeat uptime matched wall time. Include the video if motion remains wavy.

Live telemetry, the saved diagnostic capture and raw sensor capture have different finite limits; export promptly. The physical V2.3 test is pending. Retain the working V2.2 UF2 for comparison.
