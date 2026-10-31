# Set up the provisional Buddy1–4 package

**Buddy5 obstacle avoidance is not integrated. This newly restructured image still needs physical acceptance.** Keep the known-working combinedv0.5 image available during comparison. A published build is not a claim of new physical testing.

## Install and flash

1. Download/extract the complete bundle, keeping its folder structure. On Windows install Python3, Mosquitto, and the USB helper once: `py -m pip install pyserial`. Mosquitto is an external installation, not bundled software. Node and the ARM toolchain are needed only for developer tests/builds.
2. Use `firmware/integration/buddy1_4/artifacts/robopico_buddy1_4_integration_v0_1.uf2`. With the robot stopped, enter BOOTSEL and copy the UF2 onto RPI-RP2. It reboots automatically.
3. Open PuTTY on the reappearing COM port at115200. Press `h`. Verify the PROVISIONAL Buddy1–4 banner and calibration. Existing valid record28 calibration is preserved. **Do not press c on the tilted-front mounting unless you intend to restore the old low-height preset.** See [calibration](calibration.md). A blank Pico may require front calibration appropriate to its mounting.

## Start broker/dashboard and provision Wi-Fi

4. Double-click **Start dashboard.cmd** in the extracted root. Keep this window running. It serves http://localhost:8765/ and checks/starts Mosquitto TCP1883 plus WebSockets9001. If one listener is already occupied without the other, inspect the existing broker configuration; do not start competing brokers. The broker log is in dashboard_session/broker.log.
5. Put laptop and Pico on the same2.4GHz Wi-Fi/hotspot. Find the laptop's active Wi-Fi IPv4 using ipconfig (Mac: network settings). Use that LAN address, not127.0.0.1, a virtual Ethernet adapter or the Pico's address. If the laptop IP changes later, reprovision the broker address.
6. Close PuTTY, then double-click **Configure WiFi.cmd**. Enter COM4 (or4), SSID, hidden password and laptop LAN IPv4. This helper uses the normal firmware's USB serial port; **do not enter BOOTSEL again**. The robot is stopped and allowed to settle before settings are submitted. Wait for the verified settings response and reboot. You can close this helper afterward.
7. Reopen PuTTY, press `n`, and verify configured=1, mqtt=1 and subscribed=1, with advancing poll counters. The dashboard can connect to the laptop broker even when the car is offline; those are separate links. Existing working saved Wi-Fi settings can be reused without provisioning again. Passwords are not included in the public UF2.
8. In the dashboard connect to ws://localhost:9001. Wait for a fresh car heartbeat. Set the four mappings (straight/U-turn/left/right) using distinct A–Z letters and verify the readback. No boot mapping is assumed. Local firewall/network isolation can block Pico-to-laptop TCP1883 even while localhost WebSockets works; inspect broker.log for the Pico address.

Python equivalents: `py tools/dashboard/run_dashboard.py` and `py tools/provisioning/provision_wifi.py`. On Mac/Linux use python3. Runtime provisioning needs pyserial; dashboard hosting uses the Python standard library and vendored MQTT.js.

## Run and stop

9. Keep the robot still for the IMU's stationary calibration; use dashboard Reset while stopped if necessary and wait for readiness. Start with profile1 and a short flat run. Confirm Stop before a ramp test. Compare heartbeat, requested/effective ramp targets and physical behavior with the old baseline.
10. For USB hands-free testing, `g` retains the2s countdown; `s` or either robot button stops. For wireless testing, stop and wait at least5s for saving before disconnecting USB; switch to the normal power source, wait for the robot/network/IMU to become ready and start from the dashboard. The laptop broker must remain running. Do not remove the controller's only power source while expecting it to continue running.
11. After flat/bend checks, test both barcode directions with a mapped action followed by a separate junction, then the ramp. The code retains the working baseline's tuning; new interface/scheduling changes still require these physical checks. Buddy5 will not detect an obstacle in this image.

## Download logs after a run

12. Stop and wait5s. Request stored data in the dashboard. Download **Motion**, **Drive diagnostics**, **Navigation** and **Raw sensors** as CSV; also export messages/events text. Let each request finish. Raw history is the first2048 samples, so an event later in a long run may require a shorter targeted capture. The download timestamp does not identify when the failure happened.
13. Include firmware filename/build manifest, `h` calibration readback, profile, barcode letter/direction, and a video when useful. Source15 peak/ramp/target details are in live messages; the regular source9 target remains the requested target, while the new card shows the hold-constrained target separately.
14. USB alternative: enable PuTTY **All session output**, then while stopped issue `h`, `p`, `d`, `j`, `a`, `z` individually, letting each dump finish. These provide identity/calibration, motion summary, detailed drive, barcode events, raw ADC and coordinator audit. `n` gives current network state. Avoid long live dumps while moving.

Imported earlier logs live under [testing/evidence](../testing/evidence/README.md); they are kept separate from this image's new acceptance runs.
